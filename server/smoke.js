/**
 * End-to-end protocol smoke test (no browser needed).
 *
 *   npm run smoke
 *
 * Verifies: room creation, controller join/slot assignment, room_full,
 * binary MOTION/TOUCH relay with slot insertion, SYNC round-trip,
 * STATUS/HAPTIC downlink, disconnect handling and room teardown.
 * Exits 0 on success, 1 on failure.
 */

import WebSocket from 'ws';
import * as P from '../shared/protocol.js';
import { startServer } from './index.js';

let failures = 0;

function assert(cond, label) {
  if (cond) {
    console.log(`  PASS  ${label}`);
  } else {
    failures++;
    console.error(`  FAIL  ${label}`);
  }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/** Wait for the next JSON message matching a type (with timeout). */
function expectJson(ws, type, timeoutMs = 3000) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      ws.removeListener('message', onMsg);
      reject(new Error(`timeout waiting for ${type}`));
    }, timeoutMs);
    const onMsg = (data, isBinary) => {
      if (isBinary) return;
      const msg = JSON.parse(data.toString('utf8'));
      if (msg.t !== type) return; // skip others
      clearTimeout(timer);
      ws.removeListener('message', onMsg);
      resolve(msg);
    };
    ws.on('message', onMsg);
  });
}

/** Collect binary messages until `count` arrive (or timeout). */
function collectBinary(ws, count, timeoutMs = 3000) {
  return new Promise((resolve, reject) => {
    const out = [];
    const timer = setTimeout(() => {
      ws.removeListener('message', onMsg);
      if (out.length >= count) resolve(out);
      else reject(new Error(`timeout: got ${out.length}/${count} binary frames`));
    }, timeoutMs);
    const onMsg = (data, isBinary) => {
      if (!isBinary) return;
      out.push(Buffer.from(data));
      if (out.length >= count) {
        clearTimeout(timer);
        ws.removeListener('message', onMsg);
        resolve(out);
      }
    };
    ws.on('message', onMsg);
  });
}

function connect(url) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(url);
    ws.on('open', () => resolve(ws));
    ws.on('error', reject);
  });
}

const main = async () => {
  console.log('REALPLAY smoke test');
  const { httpServer, ready } = startServer({ port: 0 });
  await ready;
  const port = httpServer.address().port;
  const url = `ws://127.0.0.1:${port}/ws`;
  console.log(`server on :${port}\n`);

  /* 1. Host creates a room. */
  const host = await connect(url);
  host.send(JSON.stringify({ t: P.T.HOST_HELLO }));
  const roomMsg = await expectJson(host, P.T.ROOM);
  assert(typeof roomMsg.code === 'string' && roomMsg.code.length === 4, `host received room code (${roomMsg.code})`);
  assert(Array.isArray(roomMsg.ips) && roomMsg.ips.length >= 0, 'host received LAN ip list');
  const code = roomMsg.code;

  /* 2. Two controllers join, third is rejected. */
  const c1 = await connect(url);
  c1.send(JSON.stringify({ t: P.T.CTRL_HELLO, room: code, name: 'Phone A', device: 'test-a' }));
  const w1 = await expectJson(c1, P.T.WELCOME);
  assert(w1.slot === 1, 'controller 1 got slot 1');

  const c2 = await connect(url);
  c2.send(JSON.stringify({ t: P.T.CTRL_HELLO, room: code, name: 'Phone B', device: 'test-b' }));
  const w2 = await expectJson(c2, P.T.WELCOME);
  assert(w2.slot === 2, 'controller 2 got slot 2');

  const c3 = await connect(url);
  c3.send(JSON.stringify({ t: P.T.CTRL_HELLO, room: code, name: 'Phone C', device: 'test-c' }));
  const err = await expectJson(c3, P.T.ERROR);
  assert(err.code === 'room_full', 'third controller rejected with room_full');
  c3.close();

  const lobby1 = await expectJson(host, P.T.LOBBY);
  assert(lobby1.slots.length === 2 && lobby1.slots.every((s) => s.connected), 'lobby shows both slots connected');

  /* 3. Binary MOTION/TOUCH relay with slot insertion.
   * NOTE: TCP guarantees ordering per socket, not across the two phone
   * sockets, so we match frames by (type, slot) instead of arrival index. */
  const hostBin = collectBinary(host, 4);
  c1.send(P.encodeMotion({ flags: P.FLAGS.SWING_ACTIVE, battery: 88, time: Date.now(), qx: 0, qy: 0, qz: 0, qw: 1, ax: 0, ay: 9.8, az: 0, gx: 0, gy: 0, gz: 0, swing: 0 }));
  c2.send(P.encodeMotion({ flags: 0, battery: 51, time: Date.now(), qx: 0, qy: 0, qz: 0, qw: 1, ax: 1, ay: 2, az: 3, gx: 0, gy: 0, gz: 0, swing: 0 }));
  c1.send(P.encodeTouch({ active: true, dx: 0.5, dy: -0.25 }));
  c1.send(P.encodeMotion({ flags: P.FLAGS.SWING_EVENT | P.FLAGS.CALIBRATED, battery: 88, time: Date.now(), qx: 0, qy: 0, qz: 0, qw: 1, ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, swing: 9.5 }));
  const bin = await hostBin;

  const frames = bin.map((b) => ({ type: b[0], slot: b[1], buf: b }));
  const mFrames = frames
    .filter((f) => f.type === P.MOTION && f.buf.length === P.MOTION_RELAY_SIZE)
    .map((f) => {
      const dv = new DataView(f.buf.buffer, f.buf.byteOffset, f.buf.byteLength);
      return { slot: f.slot, body: P.readMotionBody(dv, 2, {}) };
    });
  const tFrames = frames.filter((f) => f.type === P.TOUCH);
  assert(mFrames.length === 3, 'host received 3 motion relays');
  assert(tFrames.length === 1 && tFrames[0].slot === 1 && tFrames[0].buf.length === P.TOUCH_RELAY_SIZE, 'touch relay carries slot 1');
  if (tFrames.length) {
    const dv = new DataView(tFrames[0].buf.buffer, tFrames[0].buf.byteOffset, tFrames[0].buf.byteLength);
    const tb = P.readTouchBody(dv, 2, {});
    assert(tb.active === true && Math.abs(tb.dx - 0.5) < 1e-6 && Math.abs(tb.dy - -0.25) < 1e-6, 'touch payload intact');
  }

  const m1 = mFrames.find((f) => f.slot === 1 && (f.body.flags & P.FLAGS.SWING_ACTIVE));
  assert(!!m1 && m1.body.battery === 88, 'motion relay slot 1 (battery intact)');
  const m2 = mFrames.find((f) => f.slot === 2);
  assert(!!m2 && m2.body.battery === 51 && m2.body.ax === 1, 'slot-2 payload intact');
  const m3 = mFrames.find((f) => f.slot === 1 && (f.body.flags & P.FLAGS.SWING_EVENT));
  assert(!!m3 && m3.body.flags === (P.FLAGS.SWING_EVENT | P.FLAGS.CALIBRATED) && Math.abs(m3.body.swing - 9.5) < 1e-6, 'swing event payload intact');

  /* 4. SYNC round trip. */
  const t0 = 12345.6;
  c1.send(JSON.stringify({ t: P.T.SYNC, t0 }));
  const ack = await expectJson(c1, P.T.SYNC_ACK);
  assert(ack.t0 === t0 && Math.abs(ack.t1 - Date.now()) < 2000, 'sync_ack echoes t0 and fresh t1');

  /* 5. Downlink STATUS + HAPTIC. */
  const s1 = expectJson(c1, P.T.STATUS);
  host.send(JSON.stringify({ t: P.T.STATUS, slot: 1, text: 'Hello phone', kind: 'good' }));
  const s1r = await s1;
  assert(s1r.text === 'Hello phone' && s1r.kind === 'good', 'status delivered to phone 1');

  const h2 = expectJson(c2, P.T.HAPTIC);
  host.send(JSON.stringify({ t: P.T.HAPTIC, slot: 2, pattern: 'smash' }));
  const h2r = await h2;
  assert(h2r.pattern === 'smash', 'haptic delivered to phone 2');

  /* 6. EV event uplink. */
  const evp = expectJson(host, P.T.EV);
  c1.send(JSON.stringify({ t: P.T.EV, name: 'calibrated', data: null }));
  const evr = await evp;
  assert(evr.slot === 1 && evr.name === 'calibrated', 'EV relayed to host with slot');

  /* 7. Controller disconnect frees the slot. */
  const gone = expectJson(host, P.T.CTRL_GONE);
  c2.close();
  const goneR = await gone;
  assert(goneR.slot === 2, 'ctrl_gone reports slot 2');
  const lobby2 = await expectJson(host, P.T.LOBBY);
  assert(lobby2.slots[1].connected === false, 'lobby shows slot 2 free again');

  /* 8. Host disconnect tears the room down. */
  const closed = expectJson(c1, P.T.ERROR);
  host.close();
  const closedR = await closed;
  assert(closedR.code === 'room_closed', 'controllers notified the room closed');

  c1.close();
  await sleep(100);

  console.log(failures === 0 ? '\nALL TESTS PASSED' : `\n${failures} TEST(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
};

main().catch((e) => {
  console.error('SMOKE TEST CRASHED:', e);
  process.exit(1);
});
