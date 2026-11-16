/**
 * Fake phone controller for Stage C browser E2E (`node tests/fakePhone.mjs ROOM`).
 *
 * Streams a still phone (identity quat, zero accel → hand filter ZUPT) over
 * the same WS protocol a real controller uses, with the CALIBRATED flag held
 * for the first ~400 ms so the PC captures a baseline. Runtime knobs via
 * stdin lines (optional): "swing" fires one SWING_EVENT packet.
 */
import WebSocket from 'ws';
import { encodeMotion } from '../shared/protocol.js';

const room = process.argv[2] || '';
if (!room) {
  console.error('usage: node tests/fakePhone.mjs ROOM');
  process.exit(1);
}

const ws = new WebSocket('ws://127.0.0.1:8080/ws');
let n = 0;
let slot = 0;

ws.on('open', () => {
  ws.send(JSON.stringify({ t: 'ctrl_hello', room, name: 'FakeP1', device: 'e2e-fake' }));
  console.log(`[fakePhone] hello sent (room ${room})`);
});

ws.on('message', (data) => {
  let msg;
  try { msg = JSON.parse(data.toString()); } catch { return; }
  if (msg.t === 'welcome') {
    slot = msg.slot;
    console.log(`[fakePhone] welcome, slot ${slot}`);
  } else if (msg.t === 'ev') {
    console.log(`[fakePhone] ev slot=${msg.slot} ${msg.name}`);
  }
});

ws.on('close', () => {
  console.log('[fakePhone] closed');
  process.exit(0);
});

setInterval(() => {
  if (ws.readyState !== 1) return;
  const flags = n < 25 ? 4 : 0; // CALIBRATED for the first ~400 ms
  ws.send(encodeMotion({
    flags,
    battery: 90,
    time: Date.now(),
    qx: 0, qy: 0, qz: 0, qw: 1,   // still, identity orientation
    ax: 0, ay: 0, az: 0,          // gravity-removed: at rest
    gx: 0, gy: 0, gz: 0,
    swing: 0,
  }));
  n++;
}, 16);
