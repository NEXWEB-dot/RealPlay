/**
 * REALPLAY shared protocol — the single source of truth used by
 *   - the Node server  (server/index.js)
 *   - the PC client    (client/net/*)
 *   - the phone controller (controller/controller.js)
 *
 * Two transport layers:
 *   1. JSON text frames for control messages (field `t`).
 *   2. Binary frames for high-rate sensor data (motion, touchpad).
 *
 * All binary layouts are LITTLE-ENDIAN.
 */

export const WS_PATH = '/ws';

/** Player slots in Phase 1 (table tennis singles). Doubles sports will use 4. */
export const MAX_SLOTS = 2;

/** JSON message types (field `t`). */
export const T = {
  HOST_HELLO: 'host_hello',  // PC   -> server: register as host, create a room
  ROOM: 'room',              // server -> PC:    { code, ips: [...] }
  CTRL_HELLO: 'ctrl_hello',  // phone -> server: { room, name, device }
  WELCOME: 'welcome',        // server -> phone: { slot, code, serverTime }
  LOBBY: 'lobby',            // server -> PC:    { code, slots: [...] }
  CTRL_GONE: 'ctrl_gone',    // server -> PC:    { slot, reason }
  SYNC: 'sync',              // phone -> server: { t0 }
  SYNC_ACK: 'sync_ack',      // server -> phone: { t0, t1 }
  STATUS: 'status',          // PC   -> phone (relayed): { slot, text, kind }
  HAPTIC: 'haptic',          // PC   -> phone (relayed): { slot, pattern }
  EV: 'ev',                  // phone -> PC (relayed):    { slot, name, data }
  ERROR: 'error',            // server -> anyone: { code, message }
};

/* ------------------------------------------------------------------------
 * Binary frames
 * ------------------------------------------------------------------------
 * MOTION (phone -> server), MOTION_SIZE bytes:
 *   [0]      u8     type = 0x10
 *   [1]      u8     flags                (see FLAGS)
 *   [2]      u16    battery %            (0..100, 65535 = unknown)
 *   [4]      f64    time, epoch ms, the phone's estimate of SERVER time
 *   [12]     f32 x4 quaternion (x,y,z,w)   device -> world, Y up
 *   [28]     f32 x3 linear acceleration  device frame, m/s^2 (gravity removed)
 *   [40]     f32 x3 rotation rate        device frame, rad/s
 *   [52]     f32    swing peak speed within the current window, m/s
 *
 * The server relays MOTION to the PC with the player slot inserted at [1]
 * (MOTION_RELAY_SIZE). readMotionBody(dv, 2, out) decodes the relayed frame.
 *
 * TOUCH (phone -> server), TOUCH_SIZE bytes:
 *   [0] u8 type = 0x11, [1] u8 flags (bit0 = active), [2..3] pad,
 *   [4] f32 dx, [8] f32 dy   (normalized -1..1, +y = pad up)
 * Relay: slot inserted at [1] (TOUCH_RELAY_SIZE).
 * ---------------------------------------------------------------------- */

export const MOTION = 0x10;
export const TOUCH = 0x11;

export const MOTION_SIZE = 56;
export const MOTION_RELAY_SIZE = 57;
export const TOUCH_SIZE = 12;
export const TOUCH_RELAY_SIZE = 13;

export const FLAGS = {
  SWING_ACTIVE: 1,  // inside a swing window right now
  SWING_EVENT: 2,   // a completed swing is reported with this frame (rising edge)
  CALIBRATED: 4,    // a calibration happened within the last second
  NO_GYRO: 8,       // no gyroscope on this phone — tilt-only fallback (Stage B)
};

export const BATTERY_UNKNOWN = 65535;

export const ROOM_ALPHABET = 'ABCDEFGHJKLMNPQRSTUVWXYZ';

export function makeRoomCode(len = 4, rng = Math.random) {
  let s = '';
  for (let i = 0; i < len; i++) {
    s += ROOM_ALPHABET[(rng() * ROOM_ALPHABET.length) | 0];
  }
  return s;
}

/**
 * Write the 55-byte motion body (everything after the 1-byte type).
 * `offset` points at the flags byte. Used by the phone (offset 1) and,
 * after the server inserts the slot byte, by the PC (offset 2).
 */
export function writeMotionBody(dv, offset, m) {
  dv.setUint8(offset, m.flags & 0xff);
  dv.setUint16(offset + 1, m.battery & 0xffff, true);
  dv.setFloat64(offset + 3, m.time, true);
  dv.setFloat32(offset + 11, m.qx, true);
  dv.setFloat32(offset + 15, m.qy, true);
  dv.setFloat32(offset + 19, m.qz, true);
  dv.setFloat32(offset + 23, m.qw, true);
  dv.setFloat32(offset + 27, m.ax, true);
  dv.setFloat32(offset + 31, m.ay, true);
  dv.setFloat32(offset + 35, m.az, true);
  dv.setFloat32(offset + 39, m.gx, true);
  dv.setFloat32(offset + 43, m.gy, true);
  dv.setFloat32(offset + 47, m.gz, true);
  dv.setFloat32(offset + 51, m.swing, true);
}

/** Read the motion body at `offset` into a flat object (or a reusable `out`). */
export function readMotionBody(dv, offset, out = {}) {
  out.flags = dv.getUint8(offset);
  out.battery = dv.getUint16(offset + 1, true);
  out.time = dv.getFloat64(offset + 3, true);
  out.qx = dv.getFloat32(offset + 11, true);
  out.qy = dv.getFloat32(offset + 15, true);
  out.qz = dv.getFloat32(offset + 19, true);
  out.qw = dv.getFloat32(offset + 23, true);
  out.ax = dv.getFloat32(offset + 27, true);
  out.ay = dv.getFloat32(offset + 31, true);
  out.az = dv.getFloat32(offset + 35, true);
  out.gx = dv.getFloat32(offset + 39, true);
  out.gy = dv.getFloat32(offset + 43, true);
  out.gz = dv.getFloat32(offset + 47, true);
  out.swing = dv.getFloat32(offset + 51, true);
  return out;
}

/** Encode a MOTION frame (phone -> server). Returns a Uint8Array ready for ws.send. */
export function encodeMotion(m) {
  const buf = new ArrayBuffer(MOTION_SIZE);
  const dv = new DataView(buf);
  dv.setUint8(0, MOTION);
  writeMotionBody(dv, 1, m);
  return new Uint8Array(buf);
}

/** Encode a TOUCH frame (phone -> server). */
export function encodeTouch(t) {
  const buf = new ArrayBuffer(TOUCH_SIZE);
  const dv = new DataView(buf);
  dv.setUint8(0, TOUCH);
  dv.setUint8(1, t.active ? 1 : 0);
  dv.setFloat32(4, t.dx, true);
  dv.setFloat32(8, t.dy, true);
  return new Uint8Array(buf);
}

/** Read a TOUCH body at `offset` (1 on the phone side layout, 2 when relayed). */
export function readTouchBody(dv, offset, out = {}) {
  out.active = dv.getUint8(offset) === 1;
  out.dx = dv.getFloat32(offset + 3, true);
  out.dy = dv.getFloat32(offset + 7, true);
  return out;
}

/** Haptic pattern names (PC -> phone). */
export const HAPTIC_PATTERNS = {
  tap: 15,             // ms
  smash: [30, 40, 60],
  buzz: [80, 40, 80],
};
