/**
 * PC-side WebSocket client.
 *
 * Connects to the game server (same origin, /ws), registers as host,
 * creates the room and routes every incoming frame:
 *   - JSON control: room, lobby updates, controller gone, EV events, errors
 *   - Binary relay: MOTION / TOUCH with the slot byte at [1] — forwarded
 *     into the matching ControllerInputSource's SyncBuffer.
 *
 * Timestamps: phones clock-sync to the server and stamp frames in server
 * epoch ms; the PC compares them against Date.now() (same machine or LAN
 * NTP — sub-10 ms agreement locally, far inside the interpolation buffer).
 */

import {
  WS_PATH, T, MOTION, TOUCH, readMotionBody, readTouchBody,
  MOTION_SIZE, TOUCH_SIZE,
} from '../../shared/protocol.js';
import { ControllerInputSource } from '../core/input.js';

export class NetClient {
  constructor({ onRoom, onLobby, onCtrlGone, onEv, onError } = {}) {
    this.ws = null;
    this.wantHost = false;
    this.roomCode = '';
    this.lanIps = [];
    this.slots = []; // [{slot, connected, name, device, battery, rtt}]
    this.sources = new Map(); // slot -> ControllerInputSource
    this.onRoom = onRoom || (() => {});
    this.onLobby = onLobby || (() => {});
    this.onCtrlGone = onCtrlGone || (() => {});
    this.onEv = onEv || (() => {});
    this.onError = onError || (() => {});
    this._motionBuf = {};
    this._touchBuf = {};
    this._dv = null;
  }

  connect() {
    if (this.ws && this.ws.readyState <= 1) return;
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const ws = new WebSocket(`${proto}//${location.host}${WS_PATH}`);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onopen = () => {
      if (this.wantHost) {
        ws.send(JSON.stringify({ t: T.HOST_HELLO }));
      }
    };

    ws.onmessage = (e) => {
      if (e.data instanceof ArrayBuffer) return this.onBinary(e.data);
      let msg;
      try { msg = JSON.parse(e.data); } catch { return; }
      switch (msg.t) {
        case T.ROOM:
          this.roomCode = msg.code;
          this.lanIps = msg.ips || [];
          this.onRoom(msg);
          break;
        case T.LOBBY:
          this.slots = msg.slots;
          this.onLobby(msg);
          break;
        case T.CTRL_GONE:
          this.onCtrlGone(msg);
          break;
        case T.EV:
          this.onEv(msg);
          // 'calibrated' is also delivered via the CALIBRATED flag on the
          // binary stream; the EV message gives us an instant trigger.
          if (msg.name === 'calibrated') {
            const src = this.sources.get(msg.slot);
            if (src) src.markCalibrated();
          }
          break;
        case T.ERROR:
          this.onError(msg);
          break;
      }
    };

    ws.onclose = () => { this.ws = null; };
    ws.onerror = () => { /* onclose follows */ };
  }

  /** Ask the server to create a room for this PC. */
  createRoom() {
    this.wantHost = true;
    this.connect();
    if (this.ws && this.ws.readyState === 1) {
      this.ws.send(JSON.stringify({ t: T.HOST_HELLO }));
    }
  }

  registerSource(slot, source) {
    if (source) this.sources.set(slot, source);
    else this.sources.delete(slot);
  }

  sendStatus(slot, text, kind = 'info') {
    this._send({ t: T.STATUS, slot, text, kind });
  }

  sendHaptic(slot, pattern = 'tap') {
    this._send({ t: T.HAPTIC, slot, pattern });
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === 1) {
      try { this.ws.send(JSON.stringify(obj)); } catch { /* ignore */ }
    }
  }

  onBinary(data) {
    const dv = this._dv && this._dv.buffer === data && this._dv.byteLength === data.byteLength
      ? this._dv
      : new DataView(data);
    this._dv = dv;
    if (data.byteLength < 2) return;
    const type = dv.getUint8(0);
    const slot = dv.getUint8(1);
    const source = this.sources.get(slot);
    if (!source) return;

    if (type === MOTION && data.byteLength >= MOTION_SIZE + 1) {
      const m = readMotionBody(dv, 2, this._motionBuf);
      source.pushMotion(m);
    } else if (type === TOUCH && data.byteLength >= TOUCH_SIZE + 1) {
      const t = readTouchBody(dv, 2, this._touchBuf);
      source.pushTouch(t);
    }
  }

  close() {
    this.wantHost = false;
    if (this.ws) {
      try { this.ws.close(1000, 'bye'); } catch { /* ignore */ }
      this.ws = null;
    }
  }
}
