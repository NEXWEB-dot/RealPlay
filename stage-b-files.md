# Stage B — every changed file in full

Complete current content of the ten files changed for Stage B
("Hand Motion Positioning"), as of commit `2cccd48`.

What changed, per file:

1. **`client/core/handMotion.js`** — NEW file: the `HandMotionFilter`
   estimator (low-pass → bias learning → dead zone → leaky double
   integration → spring return → ZUPT).
2. **`shared/protocol.js`** — `FLAGS.NO_GYRO = 8` (line 69).
3. **`client/core/input.js`** — `RacketInput.hand` struct;
   `ControllerInputSource` owns a filter, rotates the device-frame accel
   by the calibration baseline, resets on recalibration, flips to
   tilt-only tuning on the NO_GYRO rising edge.
4. **`client/core/settings.js`** — per-player defaults:
   `posMode ['assist','assist']`, `handSens [1,1]`, `handReturn [0.8,0.8]`,
   `handDead [0.2,0.2]`.
5. **`client/sports/tableTennis/TableTennisSport.js`** — `HAND_REACH`
   clamp, `posMode` handling in `_applyPosition` (hand path + mirrors +
   sens + clamp + touchpad + assist blend), `_clampHandOffset`,
   `debugHandInfo(slot)`, filter cfg (omega/dead) synced from settings
   every frame.
6. **`client/main.js`** — `slots` map handed to the sport ctx;
   `getHandInfo()` for the F3 overlay; tilt-only toast (once per phone)
   and `tilt` chip flag.
7. **`client/ui/hud.js`** — the live chip appends "tilt-only".
8. **`client/ui/debugOverlay.js`** — `getHandInfo` param; per-slot HAND
   line (mode, p/v/F/raw, ZUPT/TILT flags, assist target); graph rows
   shifted to make room.
9. **`client/ui/screens.js`** — `addSelectAt`/`addRangeAt` per-player
   helpers; "Positioning (per player)" settings section; updated
   settings note.
10. **`controller/controller.js`** (the phone) — no-gyro detection
    (2 s window), tilt-only secondary gravity for the accel subtraction,
    compass-alpha freeze at calibration, heavier smoothing, NO_GYRO flag
    in the MOTION stream.

E2E results and the dead-zone bug found & fixed are in `summary.md`
(Stage B section). Test instructions are in the chat delivery message.

---

# File contents (exact bytes as of commit 2cccd48)

## 1. `client/core/handMotion.js`

```js
/**
 * Hand Motion Positioning estimator (Stage B).
 *
 * Turns timestamped gravity-removed linear acceleration into a hand
 * POSITION + VELOCITY estimate, so the phone's physical motion can drive
 * the racket position (not just its rotation).
 *
 * The caller owns the frame: the input source rotates the device-frame
 * acceleration into the CALIBRATED frame (the phone's pose at calibration —
 * screen facing the opponent), so the estimate comes out as
 *   +x = player's left,  +y = up,  +z = toward the net
 * relative to that pose. Stage C's camera fusion can feed absolute
 * measurements into the same structure or replace this estimator outright.
 *
 * Pipeline per sample (all allocation-free):
 *   1. accel low-pass  (tau ~45 ms; 80 ms on tilt-only devices)
 *   2. bias learning   — while ZUPT-still, the residual (gravity leakage +
 *      sensor bias) is low-passed out, so "at rest" really means zero accel
 *   3. dead zone       — soft magnitude gate (direction kept, small
 *      magnitudes scaled toward zero) so tremor/noise can't move the racket
 *   4. leaky double integration — velocity decays with tau 0.7 s (0.5 s on
 *      tilt-only devices), the drift bleed
 *   5. spring-damper return — critically damped pull toward the neutral
 *      (zero) pose, rate ω from the "Return-to-center" setting
 *   6. ZUPT — ~100 ms of near-zero corrected accel latches Zero-velocity
 *      UPdaTe: velocity is hard-zeroed and the position relaxes home at rate
 *      ω. Holding the phone still therefore accumulates NO drift; a still
 *      displaced phone glides back to ready at the return rate (set it to 0
 *      to park the estimate anywhere).
 *
 * Interface (kept stable for Stage C):
 *   const f = new HandMotionFilter();
 *   f.reset();
 *   f.update({ t /* seconds *\/, ax, ay, az }, out) -> out
 *   out: { pos, vel, accel, raw, zupt, stillMs }  (reusable, no allocation)
 */

export class HandMotionFilter {
  constructor() {
    this.cfg = {
      tauAcc: 0.045,   // s — accel low-pass time constant
      tauVel: 0.7,     // s — velocity leak (drift bleed)
      tauBias: 0.4,    // s — bias learning rate while still
      zuptMs: 100,     // ms of stillness before the ZUPT latch
      zuptAcc: 0.55,   // m/s^2 — "near-zero" corrected accel threshold
      omega: 0.8,      // rad/s — spring/return rate (Return-to-center setting)
      dead: 0.2,       // m/s^2 — accel dead zone (Dead zone setting)
    };
    this.pos = new THREE(0, 0, 0);
    this.vel = new THREE(0, 0, 0);
    this.accel = new THREE(0, 0, 0); // filtered + bias-corrected (dead zone applied)
    this.raw = new THREE(0, 0, 0);   // last raw sample in the caller's frame
    this.zupt = false;
    this.stillMs = 0;
    this.started = false;
    this._af = new THREE(0, 0, 0);   // low-passed raw accel
    this._bias = new THREE(0, 0, 0); // learned residual (gravity leak + bias)
    this._use = new THREE(0, 0, 0);
    this._lastT = 0;
    this._stillT = 0;
  }

  /** Tilt-only (no gyroscope) devices: heavier smoothing, faster leak. */
  setLimited(v) {
    this.cfg.tauAcc = v ? 0.08 : 0.045;
    this.cfg.tauVel = v ? 0.5 : 0.7;
  }

  /** Clear all state — called when the phone recalibrates. */
  reset() {
    this.pos.set(0, 0, 0);
    this.vel.set(0, 0, 0);
    this.accel.set(0, 0, 0);
    this.raw.set(0, 0, 0);
    this._af.set(0, 0, 0);
    this._bias.set(0, 0, 0);
    this._use.set(0, 0, 0);
    this.zupt = false;
    this.stillMs = 0;
    this.started = false;
    this._stillT = 0;
    this._lastT = 0;
  }

  /**
   * @param {object} s { t: seconds, ax, ay, az } — gravity-removed linear
   *   acceleration in the caller's reference frame.
   * @param {object} out reusable { pos, vel, accel, raw, zupt, stillMs }
   * @returns {object} out
   */
  update(s, out) {
    const c = this.cfg;

    // First sample / stream gap / reordered stamp: re-seed (no integration
    // across a hole — that would fling the estimate).
    if (!this.started || s.t < this._lastT || s.t - this._lastT > 0.25) {
      this.started = true;
      this._lastT = s.t;
      this.pos.set(0, 0, 0);
      this.vel.set(0, 0, 0);
      this.accel.set(0, 0, 0);
      this._stillT = 0;
      this.stillMs = 0;
      this.zupt = false;
      this.raw.set(s.ax, s.ay, s.az);
      out.pos.set(0, 0, 0);
      out.vel.set(0, 0, 0);
      out.accel.set(0, 0, 0);
      out.raw.copy(this.raw);
      out.zupt = false;
      out.stillMs = 0;
      return out;
    }

    let dt = s.t - this._lastT;
    this._lastT = s.t;
    if (dt > 0.05) dt = 0.05;
    else if (dt < 0.001) dt = 0.001;

    // 1) accel low-pass (raw copy kept for the debug overlay)
    const kA = 1 - Math.exp(-dt / c.tauAcc);
    this._af.x += (s.ax - this._af.x) * kA;
    this._af.y += (s.ay - this._af.y) * kA;
    this._af.z += (s.az - this._af.z) * kA;
    this.raw.set(s.ax, s.ay, s.az);

    // 2) bias-corrected accel + stillness test
    this._use.set(
      this._af.x - this._bias.x,
      this._af.y - this._bias.y,
      this._af.z - this._bias.z,
    );
    const aErr = this._use.length();

    if (aErr < c.zuptAcc) {
      this._stillT += dt;
    } else {
      this._stillT = 0;
    }
    this.stillMs = this._stillT * 1000;
    const wasZupt = this.zupt;
    this.zupt = this._stillT * 1000 >= c.zuptMs;

    if (this.zupt) {
      // 6) ZUPT latch: the phone is at rest. Learn the residual bias (so the
      // next motion starts clean), hard-zero the velocity and relax the
      // position home at the return rate — no drift can build while still.
      const kB = 1 - Math.exp(-dt / c.tauBias);
      this._bias.x += this._use.x * kB;
      this._bias.y += this._use.y * kB;
      this._bias.z += this._use.z * kB;
      this.vel.set(0, 0, 0);
      this.accel.set(0, 0, 0);
      const kP = 1 - Math.exp(-c.omega * dt);
      this.pos.multiplyScalar(1 - kP);
    } else {
      if (wasZupt) this._stillT = 0;
      // 3) soft dead zone: keep direction, scale magnitudes below `dead` to 0
      let a = aErr;
      if (a > 1e-6) {
        const gate = a > c.dead ? 1 - c.dead / a : 0;
        this._use.multiplyScalar(gate);
        a *= gate;
      } else {
        this._use.set(0, 0, 0);
        a = 0;
      }
      this.accel.copy(this._use);

      // 4) leaky integration + 5) critically damped spring return
      this.vel.x += this._use.x * dt;
      this.vel.y += this._use.y * dt;
      this.vel.z += this._use.z * dt;
      const leak = Math.exp(-dt / c.tauVel);
      this.vel.multiplyScalar(leak);
      const w = c.omega;
      this.vel.x += (-w * w * this.pos.x - 2 * w * this.vel.x) * dt;
      this.vel.y += (-w * w * this.pos.y - 2 * w * this.vel.y) * dt;
      this.vel.z += (-w * w * this.pos.z - 2 * w * this.vel.z) * dt;
      this.pos.x += this.vel.x * dt;
      this.pos.y += this.vel.y * dt;
      this.pos.z += this.vel.z * dt;
    }

    out.pos.copy(this.pos);
    out.vel.copy(this.vel);
    out.accel.copy(this.accel);
    out.raw.copy(this.raw);
    out.zupt = this.zupt;
    out.stillMs = this.stillMs;
    return out;
  }
}

/* Minimal 3-vector used so the module has no import cycle risk and stays
 * drop-in for workers later (Stage C). Same API subset as THREE.Vector3. */
function THREE(x, y, z) {
  this.x = x; this.y = y; this.z = z;
}
THREE.prototype.set = function (x, y, z) { this.x = x; this.y = y; this.z = z; return this; };
THREE.prototype.copy = function (v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; };
THREE.prototype.multiplyScalar = function (k) { this.x *= k; this.y *= k; this.z *= k; return this; };
THREE.prototype.length = function () { return Math.hypot(this.x, this.y, this.z); };

```

## 2. `shared/protocol.js`

```js
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

```

## 3. `client/core/input.js`

```js
/**
 * Input abstraction — the game NEVER touches raw phone data.
 *
 * Everything the gameplay (Phase 2+) and the debug view consume is a
 * RacketInput:
 *   { orientation, angularVelocity, linearAcceleration,
 *     swingEvent, touch, assistTarget, battery, latency, fresh }
 *
 * Sources implement the same interface:
 *   sample(nowMs, out) -> bool          (fill `out`, return data-ok)
 *   isLive / label                       (status for the HUD)
 *
 *   ControllerInputSource — a phone controller, via NetClient + SyncBuffer,
 *                           with calibration baseline handling.
 *   DemoInputSource       — scripted "attract mode" racket (empty slots).
 *   BotInputSource        — fills any empty slot with a bot; the neutral
 *                           core exists now, the brain lands in Phase 5.
 */

import * as THREE from 'three';
import { SyncBuffer, FLAGS } from '../net/sync.js';
import { HandMotionFilter } from './handMotion.js';

/** Reusable input struct. One instance per racket is allocated once. */
export class RacketInput {
  constructor() {
    this.orientation = new THREE.Quaternion();
    this.angularVelocity = new THREE.Vector3();
    this.linearAcceleration = new THREE.Vector3();
    this.swingEvent = null;      // { power: m/s, dir: THREE.Vector3(device frame) }
    this.touch = { x: 0, y: 0, active: false };
    this.assistTarget = null;    // THREE.Vector3 | null (set by gameplay, Phase 3)
    /* Stage B: hand-motion estimate (filled by the source, consumed by the
     * sport). pos/vel live in the CALIBRATED frame: +x = player's left,
     * +y = up, +z = toward the net, all relative to the calibration pose. */
    this.hand = {
      active: false,
      pos: new THREE.Vector3(),
      vel: new THREE.Vector3(),
      accel: new THREE.Vector3(),  // filtered + bias-corrected accel
      raw: new THREE.Vector3(),    // last raw sample (calibrated frame)
      zupt: false,
      stillMs: 0,
      noGyro: false,
    };
    this.battery = 65535;
    this.latency = 0;
    this.fresh = false;
  }
}

/* ------------------------------------------------------------------ */
/* Phone controller source                                             */
/* ------------------------------------------------------------------ */

const CALIB_CAPTURE_MS = 300;

export class ControllerInputSource {
  constructor(slot, net) {
    this.slot = slot;
    this.net = net;
    this.buffer = new SyncBuffer({ delayMs: 45 });
    this.baseline = new THREE.Quaternion();     // calibration pose (world)
    this.hasBaseline = false;
    this.calibCapture = null;                    // { start, samples[] } | null
    this.touch = { x: 0, y: 0, active: false };
    this.swingEvent = null;
    this.battery = 65535;
    this.latency = 0;
    this.lastFlags = 0;
    this.lastCalibFlag = 0;
    this.everReceived = false; // any MOTION packet ever? (wait vs stale chip)
    this.limited = false;      // phone reported NO_GYRO (tilt-only fallback)

    // Stage B: hand-motion estimator (timestamped IMU in -> pos/vel out).
    this.handFilter = new HandMotionFilter();
    this._handV = new THREE.Vector3(); // scratch: accel rotated to the cal frame

    // Raw newest sample for the debug overlay graphs (shared, no alloc).
    this.latestRaw = {
      t: 0, quat: new THREE.Quaternion(), acc: new THREE.Vector3(),
      gyro: new THREE.Vector3(), swing: 0, flags: 0, battery: 65535,
    };
    this._captureQ = new THREE.Quaternion();
  }

  /** Incoming decoded MOTION relay (from NetClient). */
  pushMotion(m) {
    this.buffer.push(m);
    this.everReceived = true;
    this.battery = m.battery;

    // Rising edge of NO_GYRO -> switch the estimator to tilt-only tuning.
    if ((m.flags & FLAGS.NO_GYRO) && !(this.lastFlags & FLAGS.NO_GYRO)) {
      this.limited = true;
      this.handFilter.setLimited(true);
    }

    // Keep the raw-latest copy for graphs (no allocation).
    this.latestRaw.t = m.time;
    this.latestRaw.quat.set(m.qx, m.qy, m.qz, m.qw);
    this.latestRaw.acc.set(m.ax, m.ay, m.az);
    this.latestRaw.gyro.set(m.gx, m.gy, m.gz);
    this.latestRaw.swing = m.swing;
    this.latestRaw.flags = m.flags;
    this.latestRaw.battery = m.battery;

    // Rising edge of SWING_EVENT -> queue a swing event with the world dir
    // computed later by the sport (it owns the racket world transform).
    // A degenerate (zero) accel direction falls back to the device's
    // forward (+Z, screen normal) so a swing always has a direction.
    if ((m.flags & FLAGS.SWING_EVENT) && !(this.lastFlags & FLAGS.SWING_EVENT)) {
      const dir = new THREE.Vector3(m.ax, m.ay, m.az);
      if (dir.lengthSq() > 1e-6) dir.normalize();
      else dir.set(0, 0, 1);
      this.swingEvent = { power: Math.max(1, m.swing), dir };
    }
    this.lastFlags = m.flags;

    // Rising edge of the CALIBRATED flag also starts a capture window
    // (covers EV messages that got lost on a reconnect).
    if ((m.flags & FLAGS.CALIBRATED) && !(this.lastCalibFlag & FLAGS.CALIBRATED)) {
      this.markCalibrated();
    }
    this.lastCalibFlag = m.flags;

    // Calibration: provisional baseline = first sample; the 'calibrated'
    // event/flag starts a 300 ms averaging window that replaces it.
    if (this.calibCapture) {
      this._captureQ.set(m.qx, m.qy, m.qz, m.qw);
      if (this.calibCapture.samples.length && this._captureQ.dot(this.calibCapture.samples[0]) < 0) {
        this._captureQ.set(-m.qx, -m.qy, -m.qz, -m.qw);
      }
      this.calibCapture.samples.push(this._captureQ.clone());
      if (Date.now() - this.calibCapture.start >= CALIB_CAPTURE_MS) {
        this._finishCalibration();
      }
    } else if (!this.hasBaseline) {
      this.baseline.set(m.qx, m.qy, m.qz, m.qw).normalize();
      this.hasBaseline = true;
    }
  }

  pushTouch(t) {
    this.touch.x = t.dx;
    this.touch.y = t.dy;
    this.touch.active = !!t.active;
  }

  /** Triggered by the 'calibrated' EV message (or the CALIBRATED flag edge). */
  markCalibrated() {
    if (!this.hasBaseline) return; // no data yet; provisional baseline will apply
    this.calibCapture = { start: Date.now(), samples: [] };
  }

  _finishCalibration() {
    const samples = this.calibCapture.samples;
    this.calibCapture = null;
    if (!samples.length) return;
    // Average with sign alignment to the first sample.
    const avg = new THREE.Quaternion(0, 0, 0, 0);
    const ref = samples[0];
    for (const q of samples) {
      avg.x += q.x; avg.y += q.y; avg.z += q.z; avg.w += q.w;
    }
    const n = 1 / samples.length;
    avg.set(avg.x * n, avg.y * n, avg.z * n, avg.w * n);
    if (avg.lengthSq() > 1e-8) {
      avg.normalize();
      this.baseline.copy(avg);
      this.hasBaseline = true;
      // A new calibration pose redefines the reference frame: the hand-motion
      // estimate restarts from the neutral (zero-offset) pose.
      this.handFilter.reset();
    }
  }

  /** Fill `out` (RacketInput). Returns true when fresh data exists. */
  sample(nowMs, out) {
    const ok = this.buffer.isFresh(nowMs) && this.buffer.sample(nowMs, out);
    out.fresh = ok;
    const hand = out.hand;
    hand.noGyro = this.limited;
    if (!ok) {
      // No live data — report stale, keep last known touch.
      out.touch = this.touch;
      hand.active = false;
      return false;
    }
    out.swingEvent = this.swingEvent;
    this.swingEvent = null;
    out.touch = this.touch;
    out.battery = this.battery;
    out.latency = this.net ? this.slotRtt() : 0;

    /* Stage B: hand-motion estimation. The gravity-removed DEVICE-frame
     * accel is rotated by the calibrated orientation (the baseline pose) so
     * the estimate lives in the player's reference frame: +x = player's
     * left, +y = up, +z = toward the net at calibration. The filter then
     * integrates it (leaky double integration + ZUPT + spring return). */
    hand.active = false;
    if (this.hasBaseline) {
      this._handV.copy(out.linearAcceleration).applyQuaternion(this.baseline);
      this.handFilter.update(
        { t: nowMs / 1000, ax: this._handV.x, ay: this._handV.y, az: this._handV.z },
        hand,
      );
      hand.active = true;
    } else {
      hand.pos.set(0, 0, 0);
      hand.vel.set(0, 0, 0);
      hand.accel.set(0, 0, 0);
      hand.zupt = false;
    }
    return true;
  }

  slotRtt() {
    const s = this.net.slots.find((x) => x.slot === this.slot);
    return s ? s.rtt : 0;
  }

  get label() { return 'phone'; }
  get isLive() { return this.buffer.isFresh(Date.now()); }
}

/* ------------------------------------------------------------------ */
/* Demo (attract mode) source — keeps empty slots alive on screen       */
/* ------------------------------------------------------------------ */

export class DemoInputSource {
  constructor(baseYaw = 0) {
    this.phase = Math.random() * Math.PI * 2;
    this.baseYaw = baseYaw;
    this.swingTimer = 2.2;
    this._euler = new THREE.Euler();
    this._q = new THREE.Quaternion();
    this._swingQ = new THREE.Quaternion();
    this._acc = new THREE.Vector3();
  }

  sample(nowMs, out) {
    const t = nowMs / 1000;
    // Gentle idle wobble + a scripted swing every few seconds.
    this.swingTimer -= 1 / 60;
    let swing = 0;
    if (this.swingTimer <= 0) {
      this.swingTimer = 2.0 + Math.random() * 1.6;
      swing = 6 + Math.random() * 6;
      this._acc.set((Math.random() - 0.5) * 0.4, 0.8 + Math.random() * 0.4, -1).normalize();
    }
    const w = 0.9 + 0.35 * Math.sin(t * 1.1 + this.phase);
    this._euler.set(
      0.12 * Math.sin(t * 0.9 + this.phase),
      0.35 * Math.sin(t * 0.6 + this.phase * 2),
      0.18 * Math.sin(t * 1.4 + this.phase * 3),
      'YXZ',
    );
    this._q.setFromEuler(this._euler);
    out.orientation.copy(this._q);
    out.angularVelocity.set(0.3, 0.2, 0.1);
    if (swing > 0) {
      out.swingEvent = { power: swing, dir: this._acc };
    } else {
      out.swingEvent = null;
    }
    out.linearAcceleration.set(0, 0, 0);
    out.touch.x = 0; out.touch.y = 0; out.touch.active = false;
    out.assistTarget = null;
    out.hand.active = false; // demo rackets are not hand-positioned
    out.battery = 100;
    out.latency = 0;
    out.fresh = true;
    return true;
  }

  get label() { return 'demo'; }
  get isLive() { return true; }
}

/* ------------------------------------------------------------------ */
/* Bot source — same interface, brain arrives in Phase 5                */
/* ------------------------------------------------------------------ */

export class BotInputSource {
  /**
   * @param {number} slot player slot this bot fills
   * @param {string} difficulty 'easy' | 'medium' | 'hard' | 'pro'
   */
  constructor(slot, difficulty = 'medium') {
    this.slot = slot;
    this.difficulty = difficulty;
  }

  sample(_nowMs, out) {
    out.orientation.identity();
    out.angularVelocity.set(0, 0, 0);
    out.linearAcceleration.set(0, 0, 0);
    out.swingEvent = null;
    out.touch.x = 0; out.touch.y = 0; out.touch.active = false;
    out.assistTarget = null;
    out.hand.active = false;
    out.battery = 100;
    out.latency = 0;
    out.fresh = true;
    return true;
  }

  get label() { return `bot:${this.difficulty}`; }
  get isLive() { return true; }
}

```

## 4. `client/core/settings.js`

```js
/**
 * Persistent settings + quality presets.
 * Stored in localStorage under one key; every consumer can subscribe to
 * changes so live-tuning in the settings screen propagates immediately.
 */

export const QUALITY_PRESETS = {
  low:    { pixelRatioCap: 0.75, shadowMapSize: 1024, label: 'Low' },
  medium: { pixelRatioCap: 1.0,  shadowMapSize: 1024, label: 'Medium' },
  high:   { pixelRatioCap: 1.5,  shadowMapSize: 2048, label: 'High' },
};

const DEFAULTS = {
  quality: 'high',        // low | medium | high
  dynamicRes: true,       // auto-adjust internal resolution to hold frame time
  resScale: 1.0,          // manual scale when dynamicRes is off (0.5..1)
  shadows: true,
  postFx: true,           // takes effect in Phase 6 (bloom/AA/vignette)
  master: 0.8,
  sfx: 0.9,
  music: 0.5,
  sensitivity: 1.0,       // swing sensitivity multiplier (Phase 3 hit model)
  swingThreshold: 24,     // m/s^2 swing start threshold
  handed: ['right', 'right'],
  invertAxes: [false, false],
  assist: 'medium',       // off | low | medium | high (used from Phase 3)
  /* Stage B: hand-motion positioning (per player).
   *   posMode    'assist' (default glide) | 'motion' (hand + assist blend)
   *              | 'motionOnly' (hand motion only)
   *   handSens   displacement scale for the hand-motion estimate
   *   handReturn return-to-center rate (rad/s; 0 parks the estimate)
   *   handDead   accel dead zone (m/s^2) — tremor/noise gate */
  posMode: ['assist', 'assist'],
  handSens: [1.0, 1.0],
  handReturn: [0.8, 0.8],
  handDead: [0.2, 0.2],
};

const KEY = 'realplay.settings.v1';

export class Settings {
  constructor() {
    let stored = {};
    try {
      stored = JSON.parse(localStorage.getItem(KEY) || '{}');
    } catch { /* corrupt storage -> defaults */ }
    this.data = { ...DEFAULTS, ...stored };
    this.listeners = new Set();
  }

  get(key) {
    return this.data[key];
  }

  /** Returns an array value (hand/invert) by index. */
  at(key, index) {
    const v = this.data[key];
    return Array.isArray(v) ? v[index] : undefined;
  }

  set(key, value) {
    this.data[key] = value;
    this.save();
    for (const cb of this.listeners) cb(key, value);
  }

  setAt(key, index, value) {
    const arr = Array.isArray(this.data[key]) ? this.data[key].slice() : [];
    arr[index] = value;
    this.set(key, arr);
  }

  onChange(cb) {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  save() {
    try {
      localStorage.setItem(KEY, JSON.stringify(this.data));
    } catch { /* private mode etc. */ }
  }

  reset() {
    this.data = { ...DEFAULTS };
    this.save();
    for (const cb of this.listeners) cb('*', null);
  }
}

```

## 5. `client/sports/tableTennis/TableTennisSport.js`

```js
/**
 * Table tennis sport — Phase 2.
 *
 * Phase 1 (kept): full PBR court/hall, live phone-driven rackets in split
 * screen (calibrated orientation IS the racket rotation, touchpad nudges
 * position, swing events fire a lunge + arrow + whoosh).
 *
 * Phase 2:
 *   - real ball physics: an 8-ball pool stepped at the fixed 240 Hz rate
 *     with gravity, quadratic drag, Magnus, spin-transfer bounces, net +
 *     net-cord, floor, swept-sphere CCD (see ./physics.js),
 *   - a ball machine behind player 2's end feeding the rally,
 *   - ball visuals: spin-visible seam/logo texture, blob shadows, trails,
 *   - pooled impact particles on bounces/net hits,
 *   - a cyan trajectory-prediction line (shown while the F3 overlay is open),
 *   - modes: 'debug2p' (split screen, machine off by default) and
 *     'practice' (single camera, machine feeding).
 *
 * Phase 3 (kept):
 *   - racket-ball hit model: swept ball-vs-blade contact at the 240 Hz
 *     step (blade interpolated between the render frame's transforms),
 *     impulse response against the MOVING blade (rubber restitution +
 *     friction spin transfer), sweet-spot power falloff, swing-boosted
 *     blade velocity straight from the phone's swing detector,
 *   - assisted positioning toward the predicted interception point
 *     (Settings ▸ Assist: off/low/medium/high) with an assisted return
 *     arc so passive blocks clear the net,
 *   - hit FX: particles, audio "pock", HUD popups (HIT/SMASH/SWEET/EDGE),
 *   - the machine rally loop: machine feeds -> racket returns -> rally.
 *
 * Phase 4 (new):
 *   - phone-driven singles MATCH with real rules: 11 points, win by 2,
 *     serve rotates every 2 points (every point at deuce),
 *   - a serve flow (held ball above the server's blade; a swing serves;
 *     demo rackets serve themselves) solved to be LEGAL BY CONSTRUCTION:
 *     own-side bounce first, clean net clearance, deep receiver landing,
 *   - point adjudication: out / no return (double bounce) / own-side
 *     mishit / serve faults / net-serve LET replay / volley rule (with a
 *     "would have landed" check via the real-integrator prediction) —
 *     edge balls count as legal table bounces,
 *   - match events (start/serve/point/let/end) for the HUD scoreboard,
 *     banners and the match-end screen.
 *
 * World frame: player 1 owns the -Z end, camera looks toward +Z; player 2
 * owns the +Z end (base yaw of 180°), camera looks toward -Z.
 */

import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Sport } from '../_template/Sport.js';
import { TABLE, buildCourt } from './court.js';
import { buildRacket } from './racket.js';
import { TrailRibbon } from './trail.js';
import {
  stepBall, predictBall, magnusAccel, surfaceBounce, solveFlight, solveServe,
  createBallState, PHYS, TABLE_TOP,
} from './physics.js';
import { Ball } from './ball.js';
import { ImpactParticles } from './effects.js';
import { BallMachine } from './machine.js';
import { MatchState } from '../../core/match.js';

const PLAYER_ACCENTS = [0xff8a1e, 0x2fb4ff];
const HOME = [
  new THREE.Vector3(0, 1.02, -(TABLE.length / 2 + 0.24)),
  new THREE.Vector3(0, 1.02, TABLE.length / 2 + 0.24),
];
const BASE_YAW = [0, Math.PI];

// Camera per player: behind and slightly above their own end of the table.
const CAM_POS = [new THREE.Vector3(0, 1.55, -(TABLE.length / 2 + 1.45)), new THREE.Vector3(0, 1.55, TABLE.length / 2 + 1.45)];
const CAM_TARGET = new THREE.Vector3(0, 0.86, 0);

const BALL_POOL = 8;
const PRED_MAX = 128;
const UP = new THREE.Vector3(0, 1, 0);

/* Phase 3: racket-ball hit model ------------------------------------ */
const BALL_R = PHYS.ballRadius;
const HIT_E = 0.78;        // rubber normal restitution
const HIT_MU = 0.55;       // rubber friction -> spin transfer
const BLADE_R = 0.08;      // blade contact circle (7.5 cm blade + tolerance)
const BOOST_T = 0.16;      // s of extra blade velocity after a swing event
const BOOST_SCALE = 0.55;  // phone swing speed -> blade speed at contact
const HIT_CD = 0.09;       // s a ball ignores blades after being hit
const ASSIST_VALUES = { off: 0, low: 0.45, medium: 0.75, high: 0.95 };
const REACH_X = 0.75;      // assisted glide clamp around home
const REACH_Y = [0.84, 1.42]; // blade centre clamp (min keeps it off the table)

/* Phase 4: match flow ------------------------------------------------ */
const SERVE_HOLD = 0.1;    // m the serve ball floats in front of the blade
const SERVE_TIMEOUT = 6;   // s of serve-wait before the auto-serve kicks in
const POINT_BAN = 1.4;     // s between a decided point and the next serve

/* Stage B: hand-motion positioning reach clamp. Offsets are relative to the
 * slot's HOME, in the WORLD frame after the per-slot map (toward-net positive
 * z). The Y minimum keeps the blade off the table top (0.76 m) when reaching
 * over it; the assist's REACH_Y[0] (0.84) is the same idea. */
const HAND_REACH = {
  x: 0.85,               // ± sideways
  yLow: -0.18, yHigh: 0.40, // down / up from HOME.y = 1.02
  zBack: -0.35, zNet: 0.55, // away-from-net / toward-net
};

export class TableTennisSport extends Sport {
  static id = 'tableTennis';
  static label = 'Table Tennis';
  static maxPlayers = 2;

  constructor(ctx) {
    super(ctx);
    this.ctx = ctx; // { settings, hud, audio, net, renderer }
    this.simTime = 0;
    this.mode = 'practice';
    this.showPrediction = false; // driven by the F3 overlay
    this.stats = { bounces: 0, maxSpeed: 0, launches: 0, netHits: 0, hits: 0 };
    this.lastHit = null; // details of the most recent racket hit (debug/E2E)

    /* Phase 4: match flow. `match` is null outside the match mode; the
     * rally object describes THE match ball (there is exactly one). */
    this.match = null;
    this.matchPhase = 'idle'; // idle | serve-wait | rally | point | over
    this.onMatchEvent = null; // main.js -> HUD scoreboard/banners/end screen
    this._rally = {
      toucher: 0,       // slot that last legally struck the ball
      serveFlight: true, // the current flight is a serve (two-bounce rule)
      ownBounced: false, // serve has bounced on the server's half yet
      oppBounced: false, // bounced legally on the receiver's/toucher's side
      letServe: false,  // serve touched the net (replay if it lands in)
      done: true,       // the point is decided — no more adjudication
    };
    this._matchBallIdx = -1; // pool index of the live match ball
    this._serveHold = -1;    // pool index of the held (pre-serve) ball
    this._serveWaitT = 0;
    this._pointBanT = 0;
    this._rallyStuckT = 0; // rally watchdog (match ball vanished without events)
    this._curBallIdx = -1;   // set around stepBall so events know their ball

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x070a12);
    this.scene.fog = new THREE.Fog(0x070a12, 10, 26);

    // PBR reflections from a procedural room (no external HDRI files yet —
    // a real HDRI can replace this in Phase 6 with the same API).
    const pmrem = new THREE.PMREMGenerator(ctx.renderer);
    const envScene = new RoomEnvironment();
    this.scene.environment = pmrem.fromScene(envScene, 0.04).texture;
    this.scene.environmentIntensity = 0.45;
    pmrem.dispose();

    const { lights } = buildCourt(this.scene);
    this.lightRig = lights;

    this.rackets = new Map(); // slot -> rig
    for (const slot of [1, 2]) this.rackets.set(slot, this._createRacketRig(slot));

    /* ---------------- Phase 2: balls / machine / FX ---------------- */

    this.balls = [];
    for (let i = 0; i < BALL_POOL; i++) this.balls.push(new Ball(this.scene, i + 1));

    this.particles = new ImpactParticles();
    this.scene.add(this.particles.points);

    this.machine = new BallMachine(this.scene);
    this.machine.onLaunch = (launch) => this._spawnLaunch(launch);

    // prediction line (cyan) — pooled geometry, drawn while F3 is open
    this._predPositions = new Float32Array(PRED_MAX * 3);
    const predGeo = new THREE.BufferGeometry();
    predGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(this._predPositions, 3).setUsage(THREE.DynamicDrawUsage),
    );
    this.predLine = new THREE.Line(
      predGeo,
      new THREE.LineBasicMaterial({ color: 0x35e0ff, transparent: true, opacity: 0.55, depthWrite: false }),
    );
    this.predLine.frustumCulled = false;
    this.predLine.visible = false;
    this.scene.add(this.predLine);

    // physics events (one shared handler object, zero allocation per step)
    this._ballEv = {
      onBounce: (side, pos, impact, surface) => this._onBounce(side, pos, impact, surface),
      onNet: (kind, pos) => this._onNet(kind, pos),
    };

    // Preallocated scratch (hot path must not allocate).
    this._qTmp = new THREE.Quaternion();
    this._qRel = new THREE.Quaternion();
    this._vTmp = new THREE.Vector3();
    this._vTmp2 = new THREE.Vector3();
    this._prevPos = new Map();

    // Phase 3 hit model: per-ball previous position (for the swept blade
    // crossing), hit cooldowns, and preallocated blade/contact scratch.
    this._ballPrev = [];
    for (let i = 0; i < BALL_POOL; i++) this._ballPrev.push(new THREE.Vector3());
    this._hitCd = new Float32Array(BALL_POOL);
    this._bladePos = new THREE.Vector3();
    this._bladeQ = new THREE.Quaternion();
    this._nHit = new THREE.Vector3();
    this._vBlade = new THREE.Vector3();
    this._contact = new THREE.Vector3();
    this._local = new THREE.Vector3();
    this._vArc = new THREE.Vector3();
    this._to = new THREE.Vector3();
    this._qInv = new THREE.Quaternion();
    this._assistPt = new THREE.Vector3();
    this._spinSolve = new THREE.Vector3();
    // Phase 4: serve/volley scratch
    this._volleyState = createBallState();
    this._holdN = new THREE.Vector3();
    // Per-rig blade transform at physics rate (index = slot - 1)
    this._phPos = [new THREE.Vector3(), new THREE.Vector3()];
    this._phQ = [new THREE.Quaternion(), new THREE.Quaternion()];
    this._phVel = [new THREE.Vector3(), new THREE.Vector3()];

    this._inputs = new Map(); // slot -> RacketInput (filled per frame)
  }

  _createRacketRig(slot) {
    const accent = PLAYER_ACCENTS[slot - 1];
    const group = buildRacket({ accent });
    group.position.copy(HOME[slot - 1]);
    group.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), BASE_YAW[slot - 1]);
    this.scene.add(group);

    const trail = new TrailRibbon(accent);
    this.scene.add(trail.line);

    const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), group.position, 0.35, accent, 0.08, 0.045);
    arrow.line.material.transparent = true;
    arrow.cone.material.transparent = true;
    this._setArrowOpacity(arrow, 0);
    arrow.visible = false;
    this.scene.add(arrow);

    const marker = group.getObjectByName('bladeCenter');
    return {
      slot, group, trail, arrow, marker,
      impulse: new THREE.Vector3(),
      posSmooth: group.position.clone(),
      started: false,
      active: false,
      // Phase 3 blade tracking (world frame, physics rate)
      bladePos: group.position.clone(),
      prevBladePos: group.position.clone(),
      bladeQ: group.quaternion.clone(),
      prevBladeQ: group.quaternion.clone(),
      bladeVel: new THREE.Vector3(),
      boostVel: new THREE.Vector3(), // extra blade velocity from a swing event
      boostT: 0,
      assistTarget: HOME[slot - 1].clone(),
      assistValid: false,
    };
  }

  /* ---------------- Sport interface ---------------- */

  createCamera(slot) {
    const cam = new THREE.PerspectiveCamera(56, 1, 0.05, 60);
    cam.position.copy(CAM_POS[slot - 1]);
    cam.lookAt(CAM_TARGET);
    return cam;
  }

  getCameras() {
    return [this.createCamera(1), this.createCamera(2)];
  }

  getHudConfig() {
    return { perSlot: ['name', 'state'], popups: true };
  }

  setShadows(enabled, size) {
    this.lightRig.setShadows(enabled, size);
  }

  /* ---------------- Phase 2/4 public API ---------------- */

  /**
   * 'match' (split screen, phone-driven scoring) | 'practice' (single
   * camera + machine). The ball machine only belongs to practice.
   */
  setMode(mode) {
    this.mode = mode;
    this.machine.setVisible(mode === 'practice');
    if (mode !== 'match') this.exitMatch();
  }

  /** Start (or restart) a match. `firstServer` is 1 or 2. */
  startMatch(firstServer = 1) {
    this.match = new MatchState({ firstServer });
    this.matchPhase = 'idle';
    this._serveHold = -1;
    this._matchBallIdx = -1;
    this._rally.done = true;
    this._emitMatchEvent({ type: 'start', serve: firstServer });
    this._startServe();
  }

  /** Rematch: the first serve swaps sides (standard rotation). */
  rematch() {
    this.startMatch(this.match ? 3 - this.match.firstServer : 1);
  }

  /** Leave match flow (back to lobby/menu) — stops scoring and serves. */
  exitMatch() {
    this.match = null;
    this.matchPhase = 'idle';
    this._serveHold = -1;
    this._matchBallIdx = -1;
    this._rally.done = true;
  }

  _emitMatchEvent(ev) {
    if (this.onMatchEvent) this.onMatchEvent(ev);
  }

  setMachineEnabled(running) {
    this.machine.config.running = !!running;
    if (running) this.machine._timer = Math.min(this.machine._timer, 0.6);
  }

  /** Manual single ball (Space / panel button) — works while off too. */
  machineToss() {
    this.machine.tossOne();
  }

  resetBalls() {
    for (const b of this.balls) {
      b.state.active = false;
      b.state.restTimer = 0;
      b.setVisible(false);
    }
    this.predLine.visible = false;
  }

  activeBallCount() {
    let n = 0;
    for (const b of this.balls) if (b.state.active) n++;
    return n;
  }

  /** Debug overlay data for the most recently launched active ball. */
  getBallDebugInfo() {
    let best = null;
    for (const b of this.balls) {
      if (b.state.active && (!best || b.state.age < best.state.age)) best = b;
    }
    if (!best) return null;
    const s = best.state;
    const sp = s.spin;
    const ax = Math.abs(sp.x) >= Math.abs(sp.y) && Math.abs(sp.x) >= Math.abs(sp.z)
      ? (sp.x >= 0 ? '+x' : '-x')
      : Math.abs(sp.y) >= Math.abs(sp.z)
        ? (sp.y >= 0 ? '+y' : '-y')
        : (sp.z >= 0 ? '+z' : '-z');
    return {
      id: best.id,
      speedKmh: s.vel.length() * 3.6,
      spinRpm: sp.length() * 60 / (2 * Math.PI),
      axis: ax,
      magnus: magnusAccel(s.vel, s.spin),
      bounces: this.stats.bounces,
    };
  }

  /* ---------------- Fixed 240 Hz step ---------------- */

  /**
   * @param {number} dt fixed step (1/240)
   * @param {number} step index of this substep within the render frame's batch
   * @param {number} batch number of substeps in the batch (blade interpolation)
   */
  physicsStep(dt, step = 0, batch = 1) {
    this.simTime += dt;
    if (this.match) this._matchStep(dt);

    // machine feed + model animation
    this.machine.update(dt);

    // Blade transforms at physics rate: interpolate between the render
    // frames' start/end transforms using this substep's position in the
    // batch. Effective blade velocity = frame finite difference + the
    // decaying swing boost (from the phone's swing detector).
    const f = batch > 0 ? Math.min(1, (step + 1) / batch) : 1;
    for (let sl = 0; sl < 2; sl++) {
      const rig = this.rackets.get(sl + 1);
      if (rig.boostT > 0) rig.boostT = Math.max(0, rig.boostT - dt);
      this._phPos[sl].lerpVectors(rig.prevBladePos, rig.bladePos, f);
      this._phQ[sl].slerpQuaternions(rig.prevBladeQ, rig.bladeQ, f);
      this._phVel[sl].copy(rig.bladeVel);
      if (rig.boostT > 0) this._phVel[sl].addScaledVector(rig.boostVel, rig.boostT / BOOST_T);
    }

    // ball physics (deterministic, fixed dt) + racket contacts
    for (let i = 0; i < this.balls.length; i++) {
      const b = this.balls[i];
      if (!b.state.active) continue;
      if (this._hitCd[i] > 0) this._hitCd[i] -= dt;
      this._curBallIdx = i;
      this._ballPrev[i].copy(b.state.pos);
      stepBall(b.state, dt, this._ballEv);
      if (!b.state.active && i === this._matchBallIdx) this._onMatchBallGone();
      if (b.state.active && this._hitCd[i] <= 0) this._testBladeHits(i, b);
    }
    this._curBallIdx = -1;
  }

  /* ---------------- Phase 4: match flow ---------------- */

  /**
   * Per-substep match timers: pin the held serve ball above the server's
   * blade (and auto-serve after the timeout), countdown between points.
   */
  _matchStep(dt) {
    if (this.matchPhase === 'serve-wait') {
      this._serveWaitT += dt;
      const rig = this.rackets.get(this.match.serveSlot());
      const s = this.balls[this._serveHold].state;
      // float just in front of the blade face (blade local +Z)
      this._holdN.set(0, 0, 1).applyQuaternion(rig.bladeQ);
      s.pos.copy(rig.bladePos).addScaledVector(this._holdN, SERVE_HOLD);
      s.pos.y += 0.03;
      s.vel.set(0, 0, 0);
      s.spin.set(0, 0, 0);
      s.active = true;
      s.restTimer = 0;
      s.age = 0;
      if (this._serveWaitT >= SERVE_TIMEOUT) this._executeServe(9);
    } else if (this.matchPhase === 'point') {
      this._pointBanT -= dt;
      if (this._pointBanT <= 0) this._startServe();
    } else if (this.matchPhase === 'rally') {
      // Safety net: if the match ball ever vanishes without a decisive
      // bounce/net event (an exotic despawn path), the match would strand
      // in 'rally' forever. Adjudicate it with the same rules as a ball
      // that despawned without events (see _onMatchBallGone).
      const ball = this._matchBallIdx >= 0 ? this.balls[this._matchBallIdx] : null;
      if (!ball || !ball.state.active) {
        this._rallyStuckT += dt;
        if (this._rallyStuckT >= 0.05) {
          this._rallyStuckT = 0;
          this._onMatchBallGone();
        }
      } else {
        this._rallyStuckT = 0;
      }
    }
  }

  /** Begin the next serve: park a fresh ball above the current server. */
  _startServe() {
    if (!this.match || this.match.winner) return;
    this.resetBalls(); // clear the previous point's dead ball
    const ball = this._freeBall();
    const idx = this.balls.indexOf(ball);
    ball.setVisible(true);
    ball.resetVisual();
    this._hitCd[idx] = 0;
    this._matchBallIdx = idx;
    this._serveHold = idx;
    this._serveWaitT = 0;
    this._rallyStuckT = 0;
    // Park the held ball at the server's home BEFORE the first blade pin:
    // the pin in _matchStep refines it to the blade face every substep, but
    // a swing can serve in the very frame _startServe ran (update() runs
    // after the fixed batch) — solveServe must never read a stale dead-ball
    // position (floor / out of bounds) as its launch point.
    const server = this.match.serveSlot();
    const dirZ = server === 1 ? 1 : -1;
    const s = ball.state;
    s.pos.set(HOME[server - 1].x, HOME[server - 1].y + 0.03, HOME[server - 1].z + dirZ * SERVE_HOLD);
    s.vel.set(0, 0, 0);
    s.spin.set(0, 0, 0);
    s.active = true;
    s.restTimer = 0;
    s.age = 0;
    this.matchPhase = 'serve-wait';
    this._rally.done = true;
    this._rally.toucher = 0;
    this._emitMatchEvent({ type: 'serve-wait', slot: server });
  }

  /**
   * Strike the serve: the held ball launches with a LEGAL serve arc solved
   * against the real integrator (own-side bounce -> over the net -> deep
   * landing on the receiver's half — see solveServe). `power` is the
   * triggering swing's speed and shapes serve pace + spin.
   */
  _executeServe(power) {
    if (!this.match || this.matchPhase !== 'serve-wait' || this._serveHold < 0) return;
    const server = this.match.serveSlot();
    const dirZ = server === 1 ? 1 : -1;
    const ball = this.balls[this._serveHold];
    const s = ball.state;
    const serve = solveServe(s.pos, dirZ, power, 40 + power * 4);
    s.vel.set(serve.vel.x, serve.vel.y, serve.vel.z);
    s.spin.set(serve.spin.x, serve.spin.y, serve.spin.z);
    this._serveHold = -1;
    this.matchPhase = 'rally';
    const r = this._rally;
    r.toucher = server;
    r.serveFlight = true;
    r.ownBounced = false;
    r.oppBounced = false;
    r.letServe = false;
    r.done = false;
    this.stats.launches++;
    const v = s.vel.length();
    if (v > this.stats.maxSpeed) this.stats.maxSpeed = v;
    this.ctx.hud.popup(server, 'SERVE', 'accent');
    this._emitMatchEvent({ type: 'serve', slot: server });
  }

  /**
   * Decide a point for `slot` with a human-readable `reason`
   * ('out', 'no return', 'own side', 'serve fault', 'volley', 'volley out').
   */
  _awardPoint(slot, reason) {
    if (!this.match || this._rally.done) return;
    this._rally.done = true;
    const res = this.match.pointFor(slot);
    this.matchPhase = res.over ? 'over' : 'point';
    this._pointBanT = POINT_BAN;
    this._emitMatchEvent({
      type: 'point',
      slot,
      reason,
      score: [this.match.points[0], this.match.points[1]],
      serveSlot: this.match.serveSlot(),
      deuce: res.deuce,
      gamePointFor: res.gamePointFor,
    });
    if (res.over) {
      this._emitMatchEvent({
        type: 'end',
        winner: res.winner,
        score: [this.match.points[0], this.match.points[1]],
      });
    }
  }

  /**
   * Serve touched the net AND landed in — a let: replay the serve, no
   * point (the same server keeps serving because the score is unchanged).
   */
  _replayServe() {
    if (!this.match || this._rally.done) return;
    this._rally.done = true;
    this.matchPhase = 'point';
    this._pointBanT = 1.1;
    this._emitMatchEvent({ type: 'let', slot: this.match.serveSlot() });
  }

  /**
   * The match ball despawned (rolled to rest, or flew out of world bounds)
   * without a decisive bounce event: with a legal bounce already on the
   * receiver's side the receiver failed to return; otherwise the last
   * hitter's shot went out (or the serve faulted).
   */
  _onMatchBallGone() {
    const r = this._rally;
    if (!this.match || r.done || !r.toucher) return;
    if (r.oppBounced) this._awardPoint(r.toucher, 'no return');
    else this._awardPoint(3 - r.toucher, r.serveFlight ? 'serve fault' : 'out');
  }

  _spawnLaunch(launch) {
    const ball = this._freeBall();
    if (!ball) return;
    const s = ball.state;
    s.pos.copy(launch.pos);
    s.vel.copy(launch.vel);
    s.spin.copy(launch.spin);
    s.active = true;
    s.age = 0;
    s.restTimer = 0;
    ball.setVisible(true);
    ball.resetVisual();
    this.stats.launches++;
    const v = s.vel.length();
    if (v > this.stats.maxSpeed) this.stats.maxSpeed = v;
  }

  _freeBall() {
    for (const b of this.balls) if (!b.state.active) return b;
    // pool exhausted: recycle the oldest active ball
    let oldest = null;
    for (const b of this.balls) if (!oldest || b.state.age > oldest.state.age) oldest = b;
    oldest.state.active = false;
    return oldest;
  }

  /* ---------------- Phase 3: racket-ball hit model ---------------- */

  /**
   * Swept ball-vs-blade contact for one ball after its physics substep.
   * The blade is a disc (radius BLADE_R) moving along the segment
   * prev->current ball position; each face (front/back) is a plane at
   * ±ballRadius along the blade normal. Lateral offset must be inside the
   * blade circle — outside it the ball simply passes (that's the skill).
   */
  _testBladeHits(ballIdx, ball) {
    const s = ball.state;
    const prev = this._ballPrev[ballIdx];
    for (let sl = 0; sl < 2; sl++) {
      const rig = this.rackets.get(sl + 1);
      if (!rig.active) continue;
      const center = this._phPos[sl];

      // blade face normal (local +Z -> world)
      this._nHit.set(0, 0, 1).applyQuaternion(this._phQ[sl]);
      const nx = this._nHit.x, ny = this._nHit.y, nz = this._nHit.z;
      const d0 = (prev.x - center.x) * nx + (prev.y - center.y) * ny + (prev.z - center.z) * nz;
      const d1 = (s.pos.x - center.x) * nx + (s.pos.y - center.y) * ny + (s.pos.z - center.z) * nz;

      for (const side of [1, -1]) {
        const dc = side * BALL_R; // contact plane offset (surface touch)
        const a0 = d0 - dc, a1 = d1 - dc;
        if (a0 * side <= 0 || a1 * side > 0) continue; // not approaching this face
        const denom = d0 - d1;
        const fr = Math.abs(denom) < 1e-9 ? 0 : (d0 - dc) / denom;
        this._contact.lerpVectors(prev, s.pos, fr);

        // lateral offset inside the blade circle? (blade-local frame)
        this._local.copy(this._contact).sub(center);
        this._qInv.copy(this._phQ[sl]).invert();
        this._local.applyQuaternion(this._qInv);
        const off2 = this._local.x * this._local.x + this._local.y * this._local.y;
        if (off2 > BLADE_R * BLADE_R) continue;

        this._hitRespond(sl, rig, ball, ballIdx, side, Math.sqrt(off2) / BLADE_R);
        sl = 2; // one contact per substep
        break;
      }
    }
  }

  /**
   * Impulse response of a blade hit: rubber restitution + friction against
   * the MOVING blade (surfaceBounce with surface velocity), sweet-spot
   * power falloff, optional assisted return arc, FX + bookkeeping.
   */
  _hitRespond(slotIdx, rig, ball, ballIdx, side, offNorm) {
    const s = ball.state;
    const n = this._nHit;
    if (side < 0) n.multiplyScalar(-1); // back-face hit -> normal flips

    /* ---- Phase 4: volley rule (match ball only) ----
     * A receiver striking the ball before it has bounced on their own
     * half is a volley. Legal only when the ball was going OUT: predict
     * the untouched trajectory with the real integrator and check whether
     * it would have bounced on the volleyer's half. */
    if (
      this.match && ballIdx === this._matchBallIdx
      && !this._rally.done && this._rally.toucher
      && rig.slot !== this._rally.toucher && !this._rally.oppBounced
    ) {
      this._volleyState.pos.copy(s.pos);
      this._volleyState.vel.copy(s.vel);
      this._volleyState.spin.copy(s.spin);
      const pred = predictBall(this._volleyState, { maxT: 1.2, dt: 1 / 240 });
      // predictBall returns a SHARED singleton — read immediately.
      if (pred.hasBounce && pred.bounceSide === rig.slot) {
        // it was on course to land on their half: hitting it early is a fault
        this.ctx.hud.popup(rig.slot, 'VOLLEY!', 'bad');
        this._awardPoint(3 - rig.slot, 'volley');
      } else {
        // the ball was going out: striking it early wins the point
        this.ctx.hud.popup(rig.slot, 'VOLLEY WIN', 'good');
        this._awardPoint(rig.slot, 'volley out');
      }
      return; // no hit response — the point is decided
    }

    // effective blade velocity at contact
    const vB = this._vBlade.copy(this._phVel[slotIdx]);

    // impulse response against the moving blade
    let impact = surfaceBounce(s, n.x, n.y, n.z, HIT_E, HIT_MU, vB.x, vB.y, vB.z);
    if (impact <= 0) impact = 0.4; // grazing touch — still separate + report

    // sweet spot: power falls off toward the rim (scales the rebound only)
    const powerScale = 1 - 0.35 * offNorm * offNorm;
    this._vArc.copy(s.vel).sub(vB).multiplyScalar(powerScale);
    s.vel.copy(vB).add(this._vArc);

    // assisted return arc (Settings ▸ Assist). A live swing overrides it:
    // the player's own stroke character wins over the help.
    const boostFactor = rig.boostT > 0
      ? Math.min(1, (rig.boostVel.length() * (rig.boostT / BOOST_T)) / 5)
      : 0;
    const assist = this._assistValue(rig.slot) * (1 - boostFactor);
    if (assist > 0.02) this._applyAssistArc(rig.slot, s, assist * 0.8);

    // separate the ball from the blade plane
    s.pos.copy(this._contact).addScaledVector(n, BALL_R + 0.002);
    this._hitCd[ballIdx] = HIT_CD;

    // FX + feedback
    const strength = Math.min(12, impact + Math.max(0, vB.dot(n)) * 0.5 + 1);
    const count = Math.min(14, 4 + ((impact * 1.1) | 0));
    this.particles.emit(this._contact, n, count, 0xffe9b0, Math.min(1.6, strength / 7));
    this.ctx.audio.bounce(strength * 1.05);
    rig.impulse.addScaledVector(n, -0.025); // small kick-back

    const kmh = s.vel.length() * 3.6;
    const boosted = rig.boostT > 0 && rig.boostVel.length() > 3;
    // SMASH keys off the impact (blade drives the approach speed) — an
    // assisted return arc caps the outgoing speed below 40 km/h, so the
    // old outgoing-speed test could never fire under assist.
    let label = `HIT ${kmh.toFixed(0)} km/h`;
    if (boosted && impact > 5.5) label = `SMASH ${kmh.toFixed(0)} km/h`;
    else if (offNorm > 0.9) label = `EDGE ${kmh.toFixed(0)} km/h`;
    else if (offNorm < 0.35) label = `SWEET ${kmh.toFixed(0)} km/h`;
    this.ctx.hud.popup(rig.slot, label, 'accent');
    this.stats.hits++;
    const sp = s.vel.length();
    if (sp > this.stats.maxSpeed) this.stats.maxSpeed = sp;
    this.lastHit = {
      slot: rig.slot,
      kmh: +kmh.toFixed(1),
      offNorm: +offNorm.toFixed(2),
      boosted,
      impact: +impact.toFixed(2),
      spinRpm: Math.round(s.spin.length() * 60 / (2 * Math.PI)),
      t: +this.simTime.toFixed(3),
    };

    /* Phase 4: rally handover — the opponent's legal strike makes them the
     * toucher and clears the serve/bounce tracking for this flight. (Volley
     * faults and volley wins returned above, so reaching here means the
     * contact was legal and happened after the bounce on their half.) */
    if (
      this.match && ballIdx === this._matchBallIdx
      && !this._rally.done && rig.slot !== this._rally.toucher
    ) {
      const r = this._rally;
      r.toucher = rig.slot;
      r.serveFlight = false;
      r.ownBounced = false;
      r.oppBounced = false;
      r.letServe = false;
    }
  }

  /**
   * Assisted return arc: ALWAYS a legal shot — the drag-aware flight
   * solver produces a velocity that clears the net and lands on the
   * opponent's half, so assisted rallies don't die in the tape. The hit's
   * own character survives through speed (harder hits fly faster) and
   * aim (the outgoing direction bends the target), scaled by the assist
   * level `w`. Deterministic target variety from a hash of the contact.
   */
  _applyAssistArc(slot, s, w) {
    const dirZ = slot === 1 ? 1 : -1;
    const j = Math.sin(s.pos.x * 73.7 + s.pos.y * 151.1 + this.simTime * 11.7) * 0.5 + 0.5;
    const raw = s.vel.length();
    // speed: the hit's energy drives the arc; assist level normalizes it
    const spd = Math.min(10.5, Math.max(4.2, 4.2 + (raw - 4.2) * (0.35 + 0.5 * w)));
    // aim: lateral hit direction bends the target width, assist-scaled
    const aimX = (s.vel.x / Math.max(1, raw)) * 2.2 * (0.4 + 0.6 * w);
    const tx = Math.max(-0.42, Math.min(0.42, aimX - s.pos.x * 0.25));
    const tz = dirZ * (0.55 + 0.35 * j);
    // the assist damps spin — solve against the spin that will remain
    const keep = 1 - 0.45 * w;
    this._spinSolve.copy(s.spin).multiplyScalar(keep);
    const flight = solveFlight(s.pos, this._to.set(tx, TABLE_TOP + BALL_R, tz), spd, 0.05, this._spinSolve);
    s.vel.set(flight.vel.x, flight.vel.y, flight.vel.z);
    s.spin.multiplyScalar(keep);
  }

  /** Assist strength (0..1) from Settings for one slot. */
  _assistValue(slot) {
    const v = ASSIST_VALUES[this.ctx.settings.get('assist')];
    return v == null ? 0.75 : v;
  }

  /**
   * Debug/E2E: predict one ball's path and annotate every point with the
   * assist-scan conditions, so tuning can see exactly why a point was or
   * wasn't chosen (`pick` marks the selected hang point).
   */
  debugAssistScan(slot, ballIdx) {
    const b = this.balls[ballIdx];
    if (!b || !b.state.active) return null;
    const s = b.state;
    const sideSign = slot === 1 ? -1 : 1;
    const stepDt = 1 / 90;
    const out = predictBall(s, { maxT: 1.4, dt: stepDt });
    const chosen = this._scanIntercept(slot, s, this._assistPt);
    const rows = [];
    let bounced = s.vel.y > 0.2;
    for (let i = 1; i < out.count; i++) {
      const p = out.points[i];
      const vy = (p.y - out.points[i - 1].y) / stepDt;
      if (!bounced && vy > 0.5 && p.y > 0.74 && p.y < 0.95 && p.z * sideSign >= 0.45) bounced = true;
      rows.push({
        i,
        t: +(i * stepDt).toFixed(2),
        x: +p.x.toFixed(2), y: +p.y.toFixed(2), z: +p.z.toFixed(2),
        vy: +vy.toFixed(1),
        post: bounced,
        deep: p.z * sideSign >= 0.45,
        band: p.y >= REACH_Y[0] && p.y <= REACH_Y[1],
        pick: i === chosen,
      });
    }
    return { count: out.count, chosen, rows };
  }

  /* ---------------- Phase 4 debug/E2E API ---------------- */

  /** Match snapshot for the F3 overlay and E2E asserts. */
  debugMatchInfo() {
    if (!this.match) return null;
    return {
      points: [this.match.points[0], this.match.points[1]],
      firstServer: this.match.firstServer,
      serveSlot: this.match.serveSlot(),
      winner: this.match.winner,
      phase: this.matchPhase,
      rules: { ...this.match.rules },
    };
  }

  /** Live rule tuning for short E2E games: debugMatchRules({ pointsToWin: 3 }). */
  debugMatchRules(patch = {}) {
    if (!this.match) return null;
    if (patch.pointsToWin) this.match.rules.pointsToWin = patch.pointsToWin;
    if (patch.winBy) this.match.rules.winBy = patch.winBy;
    return { ...this.match.rules };
  }

  /** Skip the serve-wait (timeout or swing): serve right now (E2E). */
  debugForceServe() {
    if (this.match && this.matchPhase === 'serve-wait') this._executeServe(9);
  }

  /**
   * Fast-forward scoring (E2E only): award a point to `slot` through the
   * real MatchState so rotation/deuce/end logic can be asserted without
   * waiting for demo rallies. Skips the point-ban wait.
   */
  debugAwardPoint(slot) {
    if (!this.match || this.match.winner) return null;
    const res = this.match.pointFor(slot);
    if (res.over) {
      this.matchPhase = 'over';
      this._emitMatchEvent({ type: 'end', winner: res.winner, score: [this.match.points[0], this.match.points[1]] });
    }
    return {
      points: [this.match.points[0], this.match.points[1]],
      serveSlot: this.match.serveSlot(),
      ...res,
    };
  }

  /**
   * Stage B debug/E2E: the hand-motion estimator state for one slot
   * (positions in the calibrated frame, plus the mode/blend inputs).
   */
  debugHandInfo(slot) {
    const entry = this.ctx.slots && this.ctx.slots.get(slot);
    const src = entry && entry.source;
    const hf = src && src.handFilter;
    const rig = this.rackets.get(slot);
    if (!hf || !hf.started) return null;
    const posMode = this.ctx.settings.at('posMode', slot - 1) || 'assist';
    return {
      active: !!(entry && entry.input && entry.input.hand && entry.input.hand.active),
      mode: posMode,
      limited: !!src.limited,
      pos: { x: +hf.pos.x.toFixed(3), y: +hf.pos.y.toFixed(3), z: +hf.pos.z.toFixed(3) },
      vel: { x: +hf.vel.x.toFixed(3), y: +hf.vel.y.toFixed(3), z: +hf.vel.z.toFixed(3) },
      accel: { x: +hf.accel.x.toFixed(2), y: +hf.accel.y.toFixed(2), z: +hf.accel.z.toFixed(2) },
      raw: { x: +hf.raw.x.toFixed(2), y: +hf.raw.y.toFixed(2), z: +hf.raw.z.toFixed(2) },
      zupt: hf.zupt,
      stillMs: Math.round(hf.stillMs),
      omega: +hf.cfg.omega.toFixed(2),
      assistValid: rig ? rig.assistValid : false,
      assistTarget: rig ? { x: +rig.assistTarget.x.toFixed(3), y: +rig.assistTarget.y.toFixed(3), z: +rig.assistTarget.z.toFixed(3) } : null,
      racketPos: rig ? { x: +rig.posSmooth.x.toFixed(3), y: +rig.posSmooth.y.toFixed(3), z: +rig.posSmooth.z.toFixed(3) } : null,
    };
  }

  _onBounce(side, pos, impact, surface) {
    this.stats.bounces++;
    if (impact > this.stats.maxSpeed) this.stats.maxSpeed = impact;

    const color = surface === 'floor' ? 0x8899aa : 0xffc46b;
    const count = Math.min(16, 3 + ((impact * 1.3) | 0));
    this.particles.emit(pos, UP, count, color, Math.min(1.7, impact / 6));
    this.ctx.audio.bounce(impact);

    /* ---- Phase 4: point adjudication (match ball only) ---- */
    if (!this.match || this._curBallIdx !== this._matchBallIdx) return;
    const r = this._rally;
    if (r.done || !r.toucher) return;

    if (surface === 'floor') {
      // Ball hit the floor: after a legal bounce on the other side this is
      // a missed return; otherwise the shot (or serve) missed the table.
      if (r.oppBounced) this._awardPoint(r.toucher, 'no return');
      else this._awardPoint(3 - r.toucher, r.serveFlight ? 'serve fault' : 'out');
      return;
    }

    // Table OR edge bounce — an edge ball is a legal bounce (ITTF rules),
    // adjudicated identically but flagged for flavour.
    if (surface === 'edge') this.ctx.hud.popup(r.toucher, 'EDGE BALL', 'good');
    const own = side === r.toucher;

    if (r.serveFlight) {
      if (own) {
        // serve must bounce on the server's half first (exactly once)
        if (r.ownBounced) this._awardPoint(3 - r.toucher, 'serve fault');
        else r.ownBounced = true;
      } else if (r.ownBounced) {
        // legal serve: it reached the receiver's half
        r.oppBounced = true;
        r.serveFlight = false;
        if (r.letServe) this._replayServe(); // clipped the net en route
      } else {
        // crossed without the own-side bounce: fault
        this._awardPoint(3 - r.toucher, 'serve fault');
      }
    } else if (r.oppBounced) {
      // second bounce before a return — the receiver failed
      this._awardPoint(r.toucher, 'no return');
    } else if (own) {
      // the hitter's ball came down on their OWN side (net dribble/mishit)
      this._awardPoint(3 - r.toucher, 'own side');
    } else {
      r.oppBounced = true; // legal bounce — the receiver must return now
    }
  }

  _onNet(kind, pos) {
    this.stats.netHits++;
    this.particles.emit(pos, UP, 8, 0xdfe8ff, 0.8);
    this.ctx.audio.thud();
    const slot = pos.z < 0 ? 1 : 2;
    this.ctx.hud.popup(slot, kind === 'cord' ? 'NET CORD!' : 'NET', 'bad');

    /* Phase 4: a serve that touches the net becomes a LET if it still
     * lands legally (replay); rally net hits adjudicate via the bounce. */
    if (!this.match || this._curBallIdx !== this._matchBallIdx) return;
    const r = this._rally;
    if (r.done || !r.toucher) return;
    if (r.serveFlight && !r.oppBounced) r.letServe = true;
  }

  /* ---------------- Per-rendered-frame update ---------------- */

  /**
   * @param {number} dt render frame dt (s)
   * @param {number} _alpha fixed-step interpolation remainder
   * @param {number} nowMs server-epoch ms
   * @param {Map<number, {source: object, input: RacketInput}>} slots
   */
  update(dt, _alpha, nowMs, slots) {
    for (const [slot, entry] of slots) {
      const rig = this.rackets.get(slot);
      const { source, input } = entry;

      // Stage B: the hand-motion filter's spring rate + dead zone are live
      // settings (Return-to-center speed / Dead zone). Two writes per frame
      // is cheaper than a settings event fan-out and always current.
      const hf = source && source.handFilter;
      if (hf) {
        const w = this.ctx.settings.at('handReturn', slot - 1);
        const d = this.ctx.settings.at('handDead', slot - 1);
        if (w != null) hf.cfg.omega = w;
        if (d != null) hf.cfg.dead = d;
      }

      const ok = source.sample(nowMs, input);

      if (!ok || !input.fresh) {
        // Stale / disconnected: hide the racket and wait.
        rig.group.visible = false;
        rig.trail.setVisible(false);
        rig.arrow.visible = false;
        rig.started = false;
        rig.active = false;
        continue;
      }
      const first = !rig.started;
      if (first) {
        rig.started = true;
        rig.posSmooth.copy(HOME[slot - 1]);
      }
      rig.group.visible = true;
      rig.trail.setVisible(true);
      rig.active = true;

      // Blade tracking: this frame's start transform = last frame's end.
      rig.prevBladePos.copy(rig.group.position);
      rig.prevBladeQ.copy(rig.group.quaternion);

      this._applyOrientation(slot, rig, input, source);
      this._updateAssistTarget(slot, rig);
      this._applyPosition(slot, rig, input, dt);
      this._handleSwing(slot, rig, input, dt);

      // Phase 4: the SERVER's swing launches the held ball. The swing FX
      // and blade boost above still apply — a serve is just a hit with a
      // solved-legal arc on top. Power shapes the serve's pace + spin.
      if (
        this.match && this.matchPhase === 'serve-wait'
        && slot === this.match.serveSlot() && input.swingEvent
      ) {
        this._executeServe(input.swingEvent.power);
      }
      this._updateTrail(rig, dt);

      // Blade end-of-frame transform + velocity (finite difference).
      rig.bladePos.copy(rig.group.position);
      rig.bladeQ.copy(rig.group.quaternion);
      if (first) {
        rig.bladeVel.set(0, 0, 0); // no velocity spike on (re)spawn
      } else {
        rig.bladeVel.copy(rig.bladePos).sub(rig.prevBladePos).divideScalar(Math.max(dt, 1 / 240));
        const bvl = rig.bladeVel.length();
        if (bvl > 30) rig.bladeVel.multiplyScalar(30 / bvl);
      }
    }

    // balls + FX + prediction
    for (let i = 0; i < this.balls.length; i++) {
      const b = this.balls[i];
      if (!b.state.active) {
        b.setVisible(false);
      } else {
        b.syncFromState(dt);
      }
    }
    this.particles.update(dt);
    this._updatePrediction();
  }

  _updatePrediction() {
    if (!this.showPrediction) {
      if (this.predLine.visible) this.predLine.visible = false;
      return;
    }
    // first active ball travelling toward the P1 end (-z)
    let src = null;
    for (const b of this.balls) {
      if (b.state.active && b.state.vel.z < -0.5) { src = b.state; break; }
    }
    if (!src) {
      this.predLine.visible = false;
      return;
    }
    const out = predictBall(src, { maxT: 1.2, dt: 1 / 90 });
    const n = Math.min(out.count, PRED_MAX);
    const arr = this._predPositions;
    for (let i = 0; i < n; i++) {
      const p = out.points[i];
      arr[i * 3] = p.x;
      arr[i * 3 + 1] = p.y;
      arr[i * 3 + 2] = p.z;
    }
    this.predLine.geometry.setDrawRange(0, n);
    this.predLine.geometry.attributes.position.needsUpdate = true;
    this.predLine.visible = n > 1;
  }

  /* ---------------- Racket logic (Phase 1) ---------------- */

  /** Racket rotation = calibrated relative phone orientation (1:1 mapping). */
  _applyOrientation(slot, rig, input, source) {
    const settings = this.ctx.settings;
    // qRel = baseline⁻¹ * current  (device-relative rotation)
    const baseline = source && source.hasBaseline ? source.baseline : IDENTITY;
    this._qTmp.copy(baseline).invert();
    this._qRel.copy(this._qTmp).multiply(input.orientation);

    // Left-handed mirror (180° about the screen normal: swaps left/right and
    // up/down of the blade, keeps the face direction).
    if (settings.at('handed', slot - 1) === 'left') {
      this._qRel.multiply(this._mirrorZ());
    }
    // Invert-axes mirror (180° about world up).
    if (settings.at('invertAxes', slot - 1)) {
      this._qRel.premultiply(this._mirrorY());
    }

    this._qTmp.setFromAxisAngle(this._up(), BASE_YAW[slot - 1]);
    rig.group.quaternion.copy(this._qTmp).multiply(this._qRel);
  }

  _mirrorZ() {
    if (!this._qMz) this._qMz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
    return this._qMz;
  }
  _mirrorY() {
    if (!this._qMy) this._qMy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);
    return this._qMy;
  }
  _up() {
    if (!this._vUp) this._vUp = new THREE.Vector3(0, 1, 0);
    return this._vUp;
  }

  /**
   * Scan one approaching ball's predicted path for this slot's best
   * interception point: among the points AFTER its bounce on this player's
   * half that sit inside the blade's reach band, pick the one with the
   * LEAST vertical speed — the hang at the top of the bounce — because a
   * near-zero |vy| is the most forgiving moment to meet the ball (a real
   * player takes the ball at the top of the bounce too). Fills `out` with
   * the clamped interception point; returns the prediction index or -1.
   */
  _scanIntercept(slot, s, out) {
    const sideSign = slot === 1 ? -1 : 1; // own-half sign (P1 half is -z)
    const stepDt = 1 / 90; // prediction sample interval
    const pred = predictBall(s, { maxT: 1.4, dt: stepDt });
    // predictBall returns a SHARED singleton — scan + copy immediately.
    let bounced = s.vel.y > 0.2; // already rising: the bounce happened
    let bestI = -1;
    let bestVy = Infinity;
    for (let i = 1; i < pred.count; i++) {
      const p = pred.points[i];
      const vy = (p.y - pred.points[i - 1].y) / stepDt;
      if (!bounced) {
        // table bounce: vy flips strongly positive just above table height
        // AND on this player's own half (a SERVE's first bounce happens on
        // the SERVER's half — counting it would park the receiver at the
        // net to volley)
        if (vy > 0.5 && p.y > 0.74 && p.y < 0.95 && p.z * sideSign >= 0.45) bounced = true;
        else continue;
      }
      if (p.z * sideSign < 0.45) continue;       // own half, past the net
      if (p.y < REACH_Y[0] || p.y > REACH_Y[1]) continue;
      if (Math.abs(p.x) > REACH_X + BLADE_R) continue;
      const av = Math.abs(vy);
      if (av < bestVy) { bestVy = av; bestI = i; }
    }
    if (bestI < 0) return -1;
    const p = pred.points[bestI];
    out.set(
      Math.max(-REACH_X, Math.min(REACH_X, p.x)),
      Math.max(REACH_Y[0], Math.min(REACH_Y[1], p.y)),
      Math.max(-1.34, Math.min(1.34, p.z)),
    );
    return bestI;
  }

  /**
   * Assisted positioning: among all incoming balls take the SOONEST
   * hittable one and glide the racket to its interception point. With no
   * hittable ball the racket returns home.
   */
  _updateAssistTarget(slot, rig) {
    rig.assistTarget.copy(HOME[slot - 1]);
    rig.assistValid = false;
    if (this._assistValue(slot) <= 0.01) return;
    let bestT = Infinity;
    for (const b of this.balls) {
      const s = b.state;
      if (!s.active) continue;
      const approaching = slot === 1 ? s.vel.z < -0.5 : s.vel.z > 0.5;
      if (!approaching) continue;
      const idx = this._scanIntercept(slot, s, this._assistPt);
      if (idx >= 0 && idx / 90 < bestT) {
        bestT = idx / 90;
        rig.assistTarget.copy(this._assistPt);
        rig.assistValid = true;
      }
    }
  }

  /**
   * Positioning (Phase 1 touchpad glide + Phase 3 assist + Stage B hand
   * motion). Per-player Positioning mode from Settings:
   *   'assist'     — the Phase 3 behaviour: assist owns the target while a
   *                  hittable ball is incoming, else home + touchpad nudge.
   *   'motion'     — the hand-motion estimate drives the racket; while a
   *                  hittable ball is incoming the target blends toward the
   *                  assist target by the Assist strength
   *                  (lerp(handMotionPos, assistTarget, assistStrength)).
   *   'motionOnly' — hand motion + touchpad only; the assist never owns the
   *                  position. (The assisted RETURN ARC is a separate Phase 3
   *                  feature and still follows the Assist setting.)
   * The touchpad stays a fine adjustment on top of hand motion.
   */
  _applyPosition(slot, rig, input, dt) {
    const towardNet = slot === 1 ? 1 : -1;
    const settings = this.ctx.settings;
    const target = this._vTmp.copy(HOME[slot - 1]);
    const mode = settings.at('posMode', slot - 1) || 'assist';
    const hand = input.hand;
    const handOn = !!(hand && hand.active) && mode !== 'assist';

    if (handOn) {
      // calibrated-frame offset -> this slot's world frame
      const off = this._vTmp2.copy(hand.pos);
      // Left-handed grip mirror (same mirror the rotation uses, applied in
      // the calibration frame).
      if (settings.at('handed', slot - 1) === 'left') { off.x = -off.x; off.y = -off.y; }
      // Slot yaw: P2 stands at the other end — flip the lateral/depth axes.
      if (slot === 2) { off.x = -off.x; off.z = -off.z; }
      // Invert-axes is a WORLD-frame mirror (as in _applyOrientation).
      if (settings.at('invertAxes', slot - 1)) { off.x = -off.x; off.z = -off.z; }
      const sens = settings.at('handSens', slot - 1);
      off.multiplyScalar(sens == null ? 1 : sens);
      this._clampHandOffset(slot, off);
      target.add(off);
      // touchpad fine adjustment still applies
      target.x += input.touch.x * 0.85;
      target.z += towardNet * input.touch.y * 0.5;
      if (mode === 'motion' && rig.assistValid) {
        target.lerp(rig.assistTarget, this._assistValue(slot));
      }
    } else {
      target.x += input.touch.x * 0.85;
      target.z += towardNet * input.touch.y * 0.5;
      if (rig.assistValid) target.copy(rig.assistTarget);
    }

    rig.posSmooth.lerp(target, 1 - Math.exp(-dt / 0.12));

    // Idle bob for life (few mm).
    rig.group.position.copy(rig.posSmooth);
    rig.group.position.y += Math.sin((this.simTime + slot * 2.1) * 1.3) * 0.004;
    rig.group.position.add(rig.impulse);
  }

  /** Clamp a hand-motion WORLD offset to the reachable zone around HOME. */
  _clampHandOffset(slot, off) {
    off.x = Math.max(-HAND_REACH.x, Math.min(HAND_REACH.x, off.x));
    off.y = Math.max(HAND_REACH.yLow, Math.min(HAND_REACH.yHigh, off.y));
    // z is clamped along the toward-net axis so the range is per slot.
    const zNet = off.z * (slot === 1 ? 1 : -1);
    off.z = Math.max(HAND_REACH.zBack, Math.min(HAND_REACH.zNet, zNet)) * (slot === 1 ? 1 : -1);
    return off;
  }

  /** Swing event: lunge along the world-space swing direction + FX. */
  _handleSwing(slot, rig, input, dt) {
    // decay any active impulse
    if (rig.impulse.lengthSq() > 1e-8) {
      rig.impulse.multiplyScalar(Math.exp(-dt / 0.13));
      if (rig.impulse.length() < 0.004) rig.impulse.set(0, 0, 0);
    }
    // fade swing arrow
    if (rig.arrow.visible) {
      const o = this._arrowOpacity(rig.arrow) - dt * 4;
      this._setArrowOpacity(rig.arrow, o);
      if (o <= 0) rig.arrow.visible = false;
    }

    if (!input.swingEvent) return;

    const { power, dir } = input.swingEvent;
    // Device-frame swing direction -> world frame via the racket quaternion.
    const world = this._vTmp.copy(dir).applyQuaternion(rig.group.quaternion);
    const strength = 0.22 * Math.min(1.4, Math.max(0.35, power / 10));
    rig.impulse.copy(world).multiplyScalar(strength);

    // Phase 3: the same swing drives the hit model — for a short window
    // the blade's effective contact velocity gets this swing velocity.
    // `dir` is a UNIT vector; the blade speed comes from the swing power
    // (a 12 m/s phone swing -> ~6.6 m/s blade at contact).
    rig.boostVel.copy(world).multiplyScalar(BOOST_SCALE * Math.min(16, Math.max(1, power)));
    rig.boostT = BOOST_T;

    // Arrow flash from the blade centre.
    rig.marker.getWorldPosition(rig.arrow.position);
    rig.arrow.setDirection(world);
    rig.arrow.setLength(0.25 + Math.min(0.4, power / 18), 0.08, 0.045);
    this._setArrowOpacity(rig.arrow, 1);
    rig.arrow.visible = true;

    this.ctx.audio.swing(power);
    // No popup here: the hit popup (HIT/SMASH/SWEET/EDGE) is the payoff and
    // a fast stream of swing events would overwrite it. The swing already
    // shows via the arrow flash, audio and the HUD chip's swing readout.
    this.ctx.hud.setSlot(slot, { lastSwing: power });
  }

  _setArrowOpacity(arrow, o) {
    const clamped = Math.max(0, Math.min(1, o));
    arrow.line.material.opacity = clamped;
    arrow.cone.material.opacity = clamped;
  }

  _arrowOpacity(arrow) {
    return arrow.line.material.opacity;
  }

  _updateTrail(rig, dt) {
    rig.marker.getWorldPosition(this._vTmp);
    const prev = this._prevPos.get(rig.slot) || this._vTmp.clone();
    const speed = prev.distanceTo(this._vTmp) / Math.max(1e-4, dt);
    this._prevPos.set(rig.slot, this._vTmp.clone());
    rig.trail.update(this._vTmp, speed, dt);
  }

  dispose() {
    this.scene.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) {
        for (const m of Array.isArray(o.material) ? o.material : [o.material]) {
          if (m.map) m.map.dispose();
          m.dispose();
        }
      }
    });
    this.scene.clear();
  }
}

const IDENTITY = new THREE.Quaternion();

```

## 6. `client/main.js`

```js
/**
 * REALPLAY PC client — entry point.
 *
 * Flow: main menu -> "2 Players" -> lobby (QR + room code) -> match
 * (split-screen, live phone-driven rackets, real scoring). ESC pauses, F3
 * toggles the debug overlay.
 *
 * Everything sport-specific lives behind the Sport interface; this file
 * wires core + net + UI and delegates gameplay to TableTennisSport.
 */

import { Settings, QUALITY_PRESETS } from './core/settings.js';
import { RendererManager } from './core/renderer.js';
import { GameLoop } from './core/loop.js';
import { ViewportManager } from './core/splitScreen.js';
import { AudioMgr } from './core/audio.js';
import { RacketInput, ControllerInputSource, DemoInputSource } from './core/input.js';
import { NetClient } from './net/client.js';
import { Hud } from './ui/hud.js';
import { Screens } from './ui/screens.js';
import { DebugOverlay } from './ui/debugOverlay.js';
import { TableTennisSport } from './sports/tableTennis/TableTennisSport.js';
import './ui/ui.css';

const settings = new Settings();
const audio = new AudioMgr(settings);
const rendererMgr = new RendererManager(settings);
const hud = new Hud(document.body);
hud.hide();

const net = new NetClient({
  onRoom: () => screens.openLobby(),
  onLobby: () => {
    screens.onLobbyUpdate();
    refreshSlots();
  },
  onCtrlGone: ({ slot }) => {
    hud.toast(`Player ${slot} disconnected — waiting for reconnect…`);
    hud.setSlot(slot, { state: 'wait' });
  },
  onEv: ({ slot, name }) => {
    if (name === 'calibrated') {
      audio.click('good');
      hud.popup(slot, 'CALIBRATED ✓', 'good');
      net.sendStatus(slot, 'Calibrated ✓ — rally ready!', 'good');
      net.sendHaptic(slot, 'tap');
    }
  },
  onError: (msg) => {
    hud.toast(msg.message || 'Server error', 4000);
  },
});

/* ------------------------------------------------------------------ */
/* Player slots: source (phone / demo / bot) + RacketInput per slot     */
/* ------------------------------------------------------------------ */

/** slot -> { source, input } */
const slots = new Map();

const sport = new TableTennisSport({ settings, hud, audio, renderer: rendererMgr.renderer, slots });
sport.setShadows(settings.get('shadows'), QUALITY_PRESETS[settings.get('quality')].shadowMapSize);

const viewports = new ViewportManager(rendererMgr.renderer);
const debugOverlay = new DebugOverlay({
  getRendererInfo: () => {
    const info = rendererMgr.info();
    info.pixelRatio = rendererMgr.pixelRatio;
    return info;
  },
  getSources: () => slots.size ? Array.from(slots.entries()).map(([slot, e]) => ({ slot, source: e.source })) : [],
  getResScale: () => rendererMgr.scale,
  getBallInfo: () => (state === 'match' || state === 'practice') ? sport.getBallDebugInfo() : null,
  // Stage B: per-slot hand-motion estimator state for the overlay
  getHandInfo: (slot) => {
    const entry = slots.get(slot);
    const hf = entry && entry.source && entry.source.handFilter;
    if (!hf || !hf.started) return null;
    const rig = sport.rackets.get(slot);
    return {
      pos: hf.pos, vel: hf.vel, accel: hf.accel, raw: hf.raw,
      zupt: hf.zupt, stillMs: hf.stillMs,
      limited: entry.source.limited,
      mode: settings.at('posMode', slot - 1) || 'assist',
      assistValid: rig ? rig.assistValid : false,
      assistTarget: rig ? rig.assistTarget : null,
      racket: rig ? rig.posSmooth : null,
    };
  },
});

/* ------------------------------------------------------------------ */
/* Player slots: source (phone / demo / bot) + RacketInput per slot     */
/* ------------------------------------------------------------------ */

function getSlotEntry(slot) {
  let entry = slots.get(slot);
  if (!entry) {
    entry = { source: null, input: new RacketInput() };
    slots.set(slot, entry);
  }
  return entry;
}

function refreshSlots() {
  const maxYaw = sport.constructor.maxPlayers || 2;
  for (let slot = 1; slot <= 2; slot++) {
    const connected = net.slots.some((s) => s.slot === slot && s.connected);
    const entry = getSlotEntry(slot);

    if (connected) {
      if (!(entry.source instanceof ControllerInputSource)) {
        if (entry.source instanceof DemoInputSource) net.registerSource(slot, null);
        entry.source = new ControllerInputSource(slot, net);
        net.registerSource(slot, entry.source);
        // 'wait' only on the join transition — the heartbeat lobby refresh
        // (every 2 s) must not stomp the frame loop's live/stale chip.
        const s = net.slots.find((x) => x.slot === slot);
        hud.setSlot(slot, { name: s?.name || `P${slot}`, state: 'wait' });
      }
    } else if (!(entry.source instanceof DemoInputSource)) {
      net.registerSource(slot, null);
      entry.source = new DemoInputSource(slot === 1 ? 0 : Math.PI);
      hud.setSlot(slot, { name: `Player ${slot}`, state: 'demo' });
    }
    if (slot > maxYaw) break;
  }
}

/* ------------------------------------------------------------------ */
/* App state machine                                                   */
/* ------------------------------------------------------------------ */

let state = 'menu'; // menu | lobby | match | practice
let paused = false;

const screens = new Screens({
  settings,
  hud,
  audio,
  net,
  hooks: {
    onPlay2P: () => {
      state = 'lobby';
      net.createRoom();
      // openLobby happens in onRoom when the server answers
    },
    onPlayPractice: () => {
      state = 'practice';
      paused = false;
      screens.hideAll();
      hud.show();
      hud.hideAllSlots();
      refreshSlots();
      sport.resetBalls();
      sport.setMode('practice');
      sport.setMachineEnabled(true);
      viewports.setLayout(1, [sport.createCamera(1)]);
      screens.showMachinePanel(true);
      screens.syncMachinePanel(sport.machine.config);
      hud.toast('Practice — SPACE tosses a ball · B toggles the machine');
    },
    onMachineConfig: (patch) => {
      Object.assign(sport.machine.config, patch);
    },
    onMachineToss: () => {
      sport.machineToss();
    },
    onStartMatch: () => {
      state = 'match';
      paused = false;
      screens.hideAll();
      screens.showMachinePanel(false);
      hud.show();
      refreshSlots();
      sport.resetBalls();
      sport.setMode('match');
      sport.setMachineEnabled(false);
      viewports.setLayout(2, [sport.createCamera(1), sport.createCamera(2)]);
      sport.startMatch(1);
      hud.toast('Match — first to 11 · swing to serve · ESC pauses');
    },
    onRematch: () => {
      if (state !== 'match') return;
      screens.hideAll();
      sport.rematch(); // 'start' event refreshes the scoreboard + banner
    },
    onQuitToLobby: () => {
      paused = false;
      hud.hideAllSlots();
      if (state === 'practice') {
        // practice pauses into "quit to menu"
        state = 'menu';
        sport.setMachineEnabled(false);
        sport.resetBalls();
        screens.showMachinePanel(false);
        hud.hide();
        screens.openMenu();
        return;
      }
      state = 'lobby';
      sport.exitMatch();
      hud.showScore(false);
      sport.setMachineEnabled(false);
      sport.resetBalls();
      screens.showMachinePanel(false);
      screens.openLobby();
    },
    onQuitToMenu: () => {
      state = 'menu';
      paused = false;
      net.close();
      sport.exitMatch();
      hud.showScore(false);
      sport.setMachineEnabled(false);
      sport.resetBalls();
      screens.showMachinePanel(false);
      hud.hideAllSlots();
      hud.hide();
      screens.openMenu();
    },
    onResume: () => {
      paused = false;
      screens.closePause();
    },
    applyGraphics: () => {
      rendererMgr.apply();
      sport.setShadows(settings.get('shadows'), QUALITY_PRESETS[settings.get('quality')].shadowMapSize);
    },
  },
});

/* ------------------------------------------------------------------ */
/* Phase 4: match events -> scoreboard / banners / end screen           */
/* ------------------------------------------------------------------ */

const POINT_LABEL = {
  out: 'OUT',
  'no return': 'NO RETURN',
  'own side': 'OWN SIDE',
  'serve fault': 'SERVE FAULT',
  volley: 'VOLLEY',
  'volley out': 'VOLLEY OUT',
};

/** Display name for a slot: the phone's name when connected, else Player N. */
function slotName(slot) {
  const s = net.slots.find((x) => x.slot === slot);
  return (s && s.connected && s.name) ? s.name : `Player ${slot}`;
}

function onMatchEvent(ev) {
  switch (ev.type) {
    case 'start':
      hud.showScore(true);
      hud.setScoreNames(slotName(1), slotName(2));
      hud.updateScore(ev.score || [0, 0], ev.serve);
      hud.banner('MATCH START', 1200);
      break;
    case 'serve-wait':
      hud.setScoreNames(slotName(1), slotName(2));
      if (sport.match) hud.updateScore(sport.match.points, ev.slot);
      break;
    case 'point': {
      hud.updateScore(ev.score, ev.serveSlot, ev.gamePointFor);
      const label = POINT_LABEL[ev.reason] || ev.reason.toUpperCase();
      hud.popup(ev.slot, `POINT — ${label}`, 'accent');
      if (ev.deuce) hud.banner('DEUCE', 1400);
      else if (ev.gamePointFor) hud.banner(`GAME POINT — ${slotName(ev.gamePointFor)}`, 1400);
      break;
    }
    case 'let':
      hud.toast('LET — serve clipped the net, replaying…');
      break;
    case 'end': {
      const winner = slotName(ev.winner);
      hud.updateScore(ev.score);
      hud.banner(`${winner.toUpperCase()} WINS ${ev.score[0]}–${ev.score[1]}`, 1800, 'win');
      setTimeout(() => {
        // still in this match (not quit/paused into something else)?
        if (state === 'match' && sport.match && sport.match.winner) {
          screens.showMatchEnd(winner, ev.score[0], ev.score[1]);
        }
      }, 1800);
      break;
    }
  }
}

sport.onMatchEvent = onMatchEvent;

screens.openMenu();

window.addEventListener('keydown', (e) => {
  if (e.key === 'F3') {
    e.preventDefault();
    debugOverlay.toggle();
    sport.showPrediction = debugOverlay.visible;
  } else if (e.key === 'Escape') {
    if (state === 'match' || state === 'practice') {
      if (paused) {
        paused = false;
        screens.closePause();
      } else if (screens.settingsEl.classList.contains('hidden')) {
        paused = true;
        screens.setPauseQuitLabel(state === 'practice' ? 'Quit to menu' : 'Quit to lobby');
        screens.openPause();
      }
    }
  } else if (e.code === 'Space') {
    // Practice only — in a match the SPACE ball machine doesn't belong.
    if (state === 'practice' && !paused) {
      e.preventDefault();
      sport.machineToss();
    }
  } else if (e.key === 'b' || e.key === 'B') {
    if (state === 'practice' && !paused) {
      const on = !sport.machine.config.running;
      sport.setMachineEnabled(on);
      screens.syncMachinePanel(sport.machine.config);
      hud.toast(on ? 'Ball machine ON' : 'Ball machine OFF');
    }
  }
});

window.addEventListener('blur', () => {
  if ((state === 'match' || state === 'practice') && !paused) {
    paused = true;
    screens.openPause();
  }
});

// Returning to the window auto-resumes (the pause was caused by losing focus).
window.addEventListener('focus', () => {
  if ((state === 'match' || state === 'practice') && paused && screens.settingsEl.classList.contains('hidden')) {
    paused = false;
    screens.closePause();
  }
});

// Debug hook for the overlay/e2e tooling (never used by gameplay).
window.__RP = { net, slots, sport, state: () => state, settings, hud };

/* ------------------------------------------------------------------ */
/* Game loop                                                           */
/* ------------------------------------------------------------------ */

const loop = new GameLoop({
  fixedDt: 1 / 240,
  fixed: (dt, step, batch) => {
    if (!paused && (state === 'match' || state === 'practice')) sport.physicsStep(dt, step, batch);
  },
  frame: (dt, alpha, nowMs) => {
    rendererMgr.update(dt);
    if ((state === 'match' || state === 'practice') && !paused) {
      sport.update(dt, alpha, nowMs, slots);

      // HUD chips + machine status (cheap DOM updates at ~5 Hz)
      hudTimer -= dt;
      if (hudTimer <= 0) {
        hudTimer = 0.2;
        if (state === 'practice') {
          const st = sport.stats;
          screens.updateMachineStatus(
            `${sport.activeBallCount()} in play · ${st.bounces} bounces · ${st.hits} hits · peak ${st.maxSpeed.toFixed(1)} m/s`,
          );
        } else {
          for (const [slot, entry] of slots) {
            const s = net.slots.find((x) => x.slot === slot);
            const live = entry.source && entry.source.isLive;
            if (entry.source instanceof DemoInputSource) {
              hud.setSlot(slot, { name: `Player ${slot}`, state: 'demo' });
            } else if (live) {
              const rtt = entry.source.slotRtt ? entry.source.slotRtt() : 0;
              const batt = entry.source.battery;
              const pps = entry.source.buffer ? Math.round(packetsPerSec(entry.source.buffer, nowMs)) : 0;
              hud.setSlot(slot, {
                name: s?.name || `P${slot}`,
                state: 'live',
                rtt,
                battery: batt,
                pps,
                tilt: !!entry.source.limited,
              });
              // Stage B: one-time notice when a phone reports no gyroscope.
              if (entry.source.limited && !tiltNotified.has(entry.source)) {
                tiltNotified.add(entry.source);
                hud.toast(`${s?.name || `P${slot}`}: limited sensors — tilt-only motion tracking`, 4000);
              }
            } else if (entry.source && entry.source.everReceived) {
              // packets flowed before but stopped -> real stutter
              hud.setSlot(slot, { state: 'stale' });
            }
            // else: joined but never streamed — keep the 'wait' chip
          }
        }
      }
    }
    viewports.render(sport.scene);
    debugOverlay.frame(dt, nowMs);
  },
});

let hudTimer = 0;
const pktRates = new Map();
const tiltNotified = new WeakSet(); // sources already toasted about NO_GYRO
function packetsPerSec(buffer, nowMs) {
  let p = pktRates.get(buffer);
  if (!p) {
    p = { last: buffer.pushes, pps: 0, at: nowMs };
    pktRates.set(buffer, p);
  }
  if (nowMs - p.at >= 1000) {
    p.pps = buffer.pushes - p.last;
    p.last = buffer.pushes;
    p.at = nowMs;
  }
  return p.pps;
}

loop.start();

```

## 7. `client/ui/hud.js`

```js
/**
 * HUD: per-viewport status chips, shot pop-ups, toasts.
 * Generic (sport-agnostic): the sport feeds labels via setSlot()/popup().
 */

export class Hud {
  constructor(root) {
    this.root = document.createElement('div');
    this.root.id = 'hud';
    this.root.classList.add('pass');
    root.appendChild(this.root);

    this.halves = new Map();
    for (const slot of [1, 2, 3, 4]) {
      const half = document.createElement('div');
      half.className = `hud-half h${slot} hidden`;
      const name = document.createElement('div');
      name.className = 'hud-chip';
      name.innerHTML = `<span class="p">P${slot}</span><b class="nm">—</b>`;
      const stat = document.createElement('div');
      stat.className = 'hud-chip stat';
      half.append(name, stat);
      const popup = document.createElement('div');
      popup.className = 'hud-popup';
      popup.style.left = slot % 2 === 1 ? '25%' : '75%';
      popup.style.transform = 'translateX(-50%)';
      popup.style.top = '26%';
      half.after(popup);
      this.root.appendChild(half);
      this.root.appendChild(popup);
      this.halves.set(slot, { half, nameEl: name.querySelector('.nm'), statEl: stat, popup });
    }

    this.toastEl = document.createElement('div');
    this.toastEl.id = 'toast';
    this.root.appendChild(this.toastEl);
    this._toastTimer = 0;
    this._slotData = new Map();

    /* Phase 4: scoreboard (top centre) + big banner. Serving side shows a
     * lit dot; game-point side glows. Set from main.js via onMatchEvent. */
    this.scoreEl = document.createElement('div');
    this.scoreEl.id = 'score';
    this.scoreEl.classList.add('hidden');
    this.scoreEl.innerHTML =
      '<div class="sc-side" id="sc-side1"><span class="sc-serve" id="sc-serve1"></span>' +
      '<span class="sc-name" id="sc-name1">PLAYER 1</span><span class="sc-pts" id="sc-pts1">0</span></div>' +
      '<div class="sc-colon">:</div>' +
      '<div class="sc-side" id="sc-side2"><span class="sc-pts" id="sc-pts2">0</span>' +
      '<span class="sc-name" id="sc-name2">PLAYER 2</span><span class="sc-serve" id="sc-serve2"></span></div>';
    this.root.appendChild(this.scoreEl);
    this.scoreRefs = {
      side1: this.scoreEl.querySelector('#sc-side1'),
      side2: this.scoreEl.querySelector('#sc-side2'),
      name1: this.scoreEl.querySelector('#sc-name1'),
      name2: this.scoreEl.querySelector('#sc-name2'),
      pts1: this.scoreEl.querySelector('#sc-pts1'),
      pts2: this.scoreEl.querySelector('#sc-pts2'),
      serve1: this.scoreEl.querySelector('#sc-serve1'),
      serve2: this.scoreEl.querySelector('#sc-serve2'),
    };

    this.bannerEl = document.createElement('div');
    this.bannerEl.id = 'banner';
    this.root.appendChild(this.bannerEl);
    this._bannerT = 0;
  }

  show() { this.root.classList.remove('hidden'); }
  hide() { this.root.classList.add('hidden'); }

  /**
   * @param {object} data { name, state: 'demo'|'live'|'wait'|'stale',
   *                        battery, rtt, pps, lastSwing }
   */
  setSlot(slot, data) {
    const h = this.halves.get(slot);
    if (!h) return;
    const merged = { ...this._slotData.get(slot), ...data };
    this._slotData.set(slot, merged);

    h.half.classList.remove('hidden');
    if (merged.name != null) h.nameEl.textContent = merged.name;

    let stat = '';
    let wait = false;
    switch (merged.state) {
      case 'demo': stat = 'DEMO · wait for phone'; wait = false; break;
      case 'live':
        stat = [
          merged.rtt != null ? `${merged.rtt} ms` : null,
          merged.battery != null && merged.battery !== 65535 ? `🔋${merged.battery}%` : null,
          merged.pps != null ? `${merged.pps}/s` : null,
          merged.lastSwing != null ? `swing ${merged.lastSwing.toFixed(1)} m/s` : null,
          merged.tilt ? 'tilt-only' : null,
        ].filter(Boolean).join(' · ');
        break;
      case 'wait': stat = 'waiting for phone…'; wait = true; break;
      case 'stale': stat = 'CONNECTION STUTTER'; wait = true; break;
      default: stat = merged.state || '';
    }
    h.statEl.textContent = stat;
    h.statEl.classList.toggle('wait', wait);
  }

  popup(slot, text, kind = '') {
    const h = this.halves.get(slot);
    if (!h) return;
    h.popup.textContent = text;
    h.popup.className = `hud-popup show${kind ? ' kind-' + kind : ''}`;
    clearTimeout(h._t);
    h._t = setTimeout(() => h.popup.classList.remove('show'), 900);
  }

  toast(text, ms = 2200) {
    this.toastEl.textContent = text;
    this.toastEl.style.opacity = '1';
    clearTimeout(this._toastTimer);
    this._toastTimer = setTimeout(() => { this.toastEl.style.opacity = '0'; }, ms);
  }

  hideAllSlots() {
    for (const h of this.halves.values()) h.half.classList.add('hidden');
  }

  /* ---------------- Phase 4: match scoreboard ---------------- */

  showScore(v) {
    this.scoreEl.classList.toggle('hidden', !v);
  }

  setScoreNames(n1, n2) {
    this.scoreRefs.name1.textContent = n1;
    this.scoreRefs.name2.textContent = n2;
  }

  /**
   * Live score: `points` [P1, P2], `serveSlot` lights that side's serve dot,
   * `gamePointFor` glows the side that wins the match with the next point.
   */
  updateScore(points, serveSlot = 0, gamePointFor = 0) {
    this.scoreRefs.pts1.textContent = points[0];
    this.scoreRefs.pts2.textContent = points[1];
    this.scoreRefs.serve1.classList.toggle('on', serveSlot === 1);
    this.scoreRefs.serve2.classList.toggle('on', serveSlot === 2);
    this.scoreRefs.side1.classList.toggle('gp', gamePointFor === 1);
    this.scoreRefs.side2.classList.toggle('gp', gamePointFor === 2);
  }

  /** Big centre banner — auto-hides after `ms`, optional `kind` ('win'). */
  banner(text, ms = 1400, kind = '') {
    const el = this.bannerEl;
    el.textContent = text;
    el.className = kind ? `kind-${kind}` : '';
    void el.offsetWidth; // restart the transition on consecutive banners
    el.classList.add('show');
    clearTimeout(this._bannerT);
    this._bannerT = setTimeout(() => el.classList.remove('show'), ms);
  }
}

```

## 8. `client/ui/debugOverlay.js`

```js
/**
 * Debug overlay (F3).
 *
 * Row 1: FPS / frame ms / draw calls / triangles / pixel ratio / res scale.
 * Rows per connected source: rtt, packets/s, battery + live sensor graphs
 * (gyro xyz, accel magnitude xyz, quaternion components) pushed from the
 * raw (non-interpolated) newest sample of each phone.
 */

import * as THREE from 'three';

class Ring {
  constructor(n = 240) {
    this.n = n;
    this.data = new Float32Array(n);
    this.idx = 0;
    this.len = 0;
  }
  push(v) {
    this.data[this.idx] = v;
    this.idx = (this.idx + 1) % this.n;
    this.len = Math.min(this.len + 1, this.n);
  }
}

const SERIES = [
  { key: 'gx', label: 'gyro X', min: -10, max: 10, color: '#ff5d5d' },
  { key: 'gy', label: 'gyro Y', min: -10, max: 10, color: '#35d07f' },
  { key: 'gz', label: 'gyro Z', min: -10, max: 10, color: '#2f7cff' },
  { key: 'ax', label: 'acc X', min: -40, max: 40, color: '#ff5d5d' },
  { key: 'ay', label: 'acc Y', min: -40, max: 40, color: '#35d07f' },
  { key: 'az', label: 'acc Z', min: -40, max: 40, color: '#2f7cff' },
  { key: 'qw', label: 'quat W', min: -1.2, max: 1.2, color: '#ffb020' },
  { key: 'qx', label: 'quat X', min: -1.2, max: 1.2, color: '#ff8a1e' },
  { key: 'qy', label: 'quat Y', min: -1.2, max: 1.2, color: '#35d07f' },
  { key: 'qz', label: 'quat Z', min: -1.2, max: 1.2, color: '#2f7cff' },
];

export class DebugOverlay {
  constructor({ getRendererInfo, getSources, getResScale, getBallInfo, getHandInfo }) {
    this.getRendererInfo = getRendererInfo;
    this.getSources = getSources;
    this.getResScale = getResScale;
    this.getBallInfo = getBallInfo || null;
    this.getHandInfo = getHandInfo || null;

    this.el = document.createElement('canvas');
    this.el.id = 'debug-overlay';
    this.el.height = 200;
    this.el.style.height = '200px';
    document.body.appendChild(this.el);
    this.ctx = this.el.getContext('2d');
    this.visible = false;

    this.fps = new Ring(240);
    this.series = new Map(); // slot -> Map(key -> Ring)
    this.pkts = new Map();   // slot -> { lastPushes, pps, at }

    this._lastPktPoll = 0;
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.el.width = Math.floor(window.innerWidth * dpr);
    this.el.height = Math.floor(200 * dpr);
    this.el.style.width = '100%';
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  toggle() {
    this.visible = !this.visible;
    this.el.style.display = this.visible ? 'block' : 'none';
    return this.visible;
  }

  _rawSeries(slot, key) {
    let m = this.series.get(slot);
    if (!m) {
      m = new Map();
      for (const s of SERIES) m.set(s.key, new Ring());
      this.series.set(slot, m);
    }
    return m.get(key);
  }

  _packetRate(slot, pushes, now) {
    let p = this.pkts.get(slot);
    if (!p) {
      p = { lastPushes: pushes, pps: 0, at: now };
      this.pkts.set(slot, p);
    }
    if (now - p.at >= 1000) {
      p.pps = pushes - p.lastPushes;
      p.lastPushes = pushes;
      p.at = now;
    }
    return p.pps;
  }

  frame(dt, nowMs) {
    if (!this.visible) return;

    // push raw data
    this.fps.push(1 / Math.max(1e-4, dt));
    for (const { slot, source } of this.getSources()) {
      if (!source || !source.latestRaw || !source.latestRaw.t) continue;
      const r = source.latestRaw;
      const push = (key, v) => this._rawSeries(slot, key).push(v);
      push('gx', r.gyro.x); push('gy', r.gyro.y); push('gz', r.gyro.z);
      push('ax', r.acc.x); push('ay', r.acc.y); push('az', r.acc.z);
      push('qw', r.quat.w); push('qx', r.quat.x); push('qy', r.quat.y); push('qz', r.quat.z);
      this._packetRate(slot, source.buffer ? source.buffer.pushes : 0, nowMs);
    }

    const ctx = this.ctx;
    const W = window.innerWidth;
    const H = 200;
    ctx.clearRect(0, 0, W, H);

    const info = this.getRendererInfo() || {};
    const scale = this.getResScale();
    ctx.font = '12px Consolas, monospace';
    ctx.fillStyle = '#8b96a8';
    ctx.fillText(
      `FPS ${this.fps.data[(this.fps.idx - 1 + this.fps.n) % this.fps.n].toFixed(0)}  ` +
      `frame ${(dt * 1000).toFixed(1)}ms  ` +
      `calls ${info.render?.calls ?? '-'}  tris ${info.render?.triangles ?? '-'}  ` +
      `pr ${info.pixelRatio ?? '-'}  dynScale ${scale.toFixed(2)}  ` +
      `(F3 hides this overlay)`,
      10, 16,
    );

    // ball telemetry line (Phase 2) — also shifts the graph rows down
    let y0 = 34;
    if (this.getBallInfo) {
      const ball = this.getBallInfo();
      if (ball) {
        ctx.fillStyle = '#7fe3ff';
        ctx.fillText(
          `BALL ${ball.id}  ${ball.speedKmh.toFixed(1)} km/h  ` +
          `spin ${Math.round(ball.spinRpm)} rpm (${ball.axis})  ` +
          `magnus ${ball.magnus.toFixed(1)} m/s²  bounces ${ball.bounces}`,
          10, 32,
        );
      } else {
        ctx.fillStyle = '#35405a';
        ctx.fillText('no ball in play — SPACE tosses one', 10, 32);
      }
      y0 = 50;
    }

    // graph slots: up to 2 columns
    const cols = Math.min(2, this.series.size);
    if (!cols) {
      ctx.fillStyle = '#35405a';
      ctx.fillText('no sensor sources yet — open the controller page on a phone', 10, y0 + 10);
      return;
    }
    const colW = (W - 20) / cols;
    let col = 0;
    for (const [slot, _m] of this.series) {
      const x0 = 10 + col * colW;
      const src = this.getSources().find((s) => s.slot === slot)?.source;
      const pkt = this.pkts.get(slot);
      ctx.fillStyle = '#c8d2e4';
      ctx.fillText(
        `P${slot}  rtt ${src?.slotRtt?.() ?? 0}ms  pkts ${pkt ? pkt.pps : 0}/s  batt ${src?.battery ?? '-'}  (${src?.label || '?'})`,
        x0, y0 - 4,
      );

      // Stage B: hand-motion estimator line — position/velocity in the
      // calibrated frame, filtered + raw accel, ZUPT/tilt flags, and the
      // assist target the position blends toward.
      let handH = 0;
      const hand = this.getHandInfo ? this.getHandInfo(slot) : null;
      if (hand) {
        handH = 14;
        const p = hand.pos, v = hand.vel, a = hand.accel, r = hand.raw;
        ctx.fillStyle = '#9fe8b7';
        ctx.fillText(
          `HAND[${hand.mode}] p(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) ` +
          `v(${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}) ` +
          `F(${a.x.toFixed(1)},${a.y.toFixed(1)},${a.z.toFixed(1)}) ` +
          `raw(${r.x.toFixed(1)},${r.y.toFixed(1)},${r.z.toFixed(1)}) ` +
          `${hand.zupt ? 'ZUPT ' : ''}${hand.limited ? 'TILT ' : ''}` +
          (hand.assistValid && hand.assistTarget
            ? `T(${hand.assistTarget.x.toFixed(2)},${hand.assistTarget.y.toFixed(2)},${hand.assistTarget.z.toFixed(2)})`
            : ''),
          x0, y0 + 10,
        );
      }

      const gh = (H - y0 - 8 - handH) / SERIES.length;
      const gy0 = y0 + handH;
      for (const s of SERIES) {
        const yTop = gy0 + SERIES.indexOf(s) * gh;
        this._drawRing(ctx, this._rawSeries(slot, s.key), x0, yTop, colW - 120, gh - 4, s);
        ctx.fillStyle = s.color;
        ctx.font = '10px Consolas, monospace';
        ctx.fillText(s.label, x0 + colW - 112, yTop + gh / 2);
        ctx.font = '12px Consolas, monospace';
      }
      col++;
      if (col >= cols) break;
    }
  }

  _drawRing(ctx, ring, x, y, w, h, s) {
    ctx.strokeStyle = 'rgba(35,43,58,0.8)';
    ctx.beginPath();
    ctx.moveTo(x, y + h / 2);
    ctx.lineTo(x + w, y + h / 2);
    ctx.stroke();

    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1;
    ctx.beginPath();
    const n = ring.n;
    for (let i = 0; i < ring.len; i++) {
      const v = ring.data[(ring.idx - ring.len + i + n * 2) % n];
      const px = x + (i / (n - 1)) * w;
      const norm = (v - s.min) / (s.max - s.min);
      const py = y + h - Math.max(0, Math.min(1, norm)) * h;
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.lineWidth = 1;
  }
}

```

## 9. `client/ui/screens.js`

```js
/**
 * UI screens: main menu, lobby (QR + room code + slots), settings, pause.
 * Plain DOM, no framework. Every screen is created once and shown/hidden.
 */

import QRCode from 'qrcode';
import { QUALITY_PRESETS } from '../core/settings.js';

const $ = (tag, cls, html) => {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (html != null) el.innerHTML = html;
  return el;
};

export class Screens {
  /**
   * @param {object} ctx { settings, hud, audio, net, hooks }
   * hooks: { onPlay2P, onPlayPractice, onMachineConfig, onMachineToss,
   *          onStartMatch, onQuitToLobby, onQuitToMenu, onRematch,
   *          onResume, applyGraphics }
   */
  constructor(ctx) {
    this.ctx = ctx;
    this.root = document.createElement('div');
    this.root.id = 'ui';
    document.body.appendChild(this.root);

    this.menuEl = this._buildMenu();
    this.lobbyEl = this._buildLobby();
    this.settingsEl = this._buildSettings();
    this.pauseEl = this._buildPause();
    this.machineEl = this._buildMachinePanel();
    this.matchEndEl = this._buildMatchEnd();
    for (const el of [this.menuEl, this.lobbyEl, this.settingsEl, this.pauseEl, this.machineEl, this.matchEndEl]) {
      this.root.appendChild(el);
    }

    this._ipChoice = '';
    this._urlOverride = '';
  }

  _show(el) {
    for (const e of [this.menuEl, this.lobbyEl, this.settingsEl, this.pauseEl, this.matchEndEl]) {
      e.classList.toggle('hidden', e !== el);
    }
  }
  hideAll() { this._show(null); }

  /* ---------------------------------------------------------------- */
  /* Main menu                                                        */
  /* ---------------------------------------------------------------- */

  _buildMenu() {
    const el = $('div', 'screen');
    const card = $('div', 'menu-card');
    card.appendChild($('div', 'logo', 'REAL<span>PLAY</span>'));
    card.appendChild($('div', 'tagline', 'Your phone is the racket. Scan, calibrate, swing.'));

    const list = $('div', 'menu-list');

    const b2p = $('button', 'menu-btn', '<span>2 Players</span><span class="soon">MATCH</span>');
    b2p.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onPlay2P(); });

    const bBot = $('button', 'menu-btn', '<span>Player vs Bot</span><span class="soon">PHASE 5</span>');
    bBot.disabled = true;

    const bPrac = $('button', 'menu-btn', '<span>Practice / Rally</span><span class="soon">BALL MACHINE</span>');
    bPrac.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onPlayPractice(); });

    const bSet = $('button', 'menu-btn', '<span>Settings</span><span></span>');
    bSet.addEventListener('click', () => { this.ctx.audio.click(); this.openSettings(); });

    list.append(b2p, bBot, bPrac, bSet);
    card.appendChild(list);
    card.appendChild($('div', 'menu-foot', '<span>F3 — debug overlay · ESC pauses</span><span>Phase 4 build</span>'));
    el.appendChild(card);
    return el;
  }

  openMenu() { this._show(this.menuEl); }

  /* ---------------------------------------------------------------- */
  /* Lobby                                                            */
  /* ---------------------------------------------------------------- */

  _buildLobby() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'panel');
    card.style.maxWidth = 'min(94vw, 780px)';
    const wrap = $('div', 'lobby');

    const left = $('div', 'lobby-left');
    left.appendChild($('div', 'room-code-label', 'ROOM CODE'));
    this.codeEl = $('div', 'room-code', '----');
    left.appendChild(this.codeEl);
    this.qrCanvas = $('canvas');
    this.qrCanvas.id = 'qr-canvas';
    left.appendChild(this.qrCanvas);
    this.linkInput = $('input');
    this.linkInput.readOnly = true;
    this.linkInput.style.flex = '1';
    const ipSelect = $('select');
    ipSelect.style.maxWidth = '150px';
    this.ipSelect = ipSelect;
    const linkRow = $('div', 'linkrow');
    linkRow.append(ipSelect, this.linkInput);
    left.appendChild(linkRow);
    const copyBtn = $('button', 'btn small', 'Copy link');
    copyBtn.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(this.linkInput.value);
        this.ctx.hud.toast('Link copied');
      } catch { this.ctx.hud.toast('Copy failed — select the text'); }
    });
    left.appendChild(copyBtn);

    const right = $('div', 'lobby-right');
    this.slotCards = [1, 2].map((slot) => {
      const c = $('div', 'slot-card');
      c.innerHTML = `
        <div class="slot-head"><div class="slot-dot" id="dot-${slot}"></div><div class="slot-name">Player ${slot}</div></div>
        <div class="slot-meta" id="meta-${slot}">waiting for a phone…</div>
        <div class="slot-actions"><button class="btn small" id="buzz-${slot}" disabled>Test buzz</button></div>`;
      right.appendChild(c);
      c.querySelector(`#buzz-${slot}`).addEventListener('click', () => {
        this.ctx.net.sendHaptic(slot, 'tap');
        this.ctx.audio.click();
      });
      return c;
    });

    const actions = $('div', 'lobby-actions');
    const back = $('button', 'btn danger', 'Close room');
    back.addEventListener('click', () => { this.ctx.audio.click(); this.ctx.hooks.onQuitToMenu(); });
    const enter = $('button', 'btn primary', 'Start match');
    enter.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onStartMatch(); });
    actions.append(back, enter);
    right.appendChild(actions);

    wrap.append(left, right);
    card.appendChild(wrap);
    el.appendChild(card);
    return el;
  }

  async openLobby() {
    this._show(this.lobbyEl);
    const net = this.ctx.net;
    this.codeEl.textContent = net.roomCode || '----';
    await this._refreshQr();
    this._refreshSlots();
  }

  async _refreshQr() {
    const net = this.ctx.net;
    const ipSelect = this.ipSelect;
    const ips = net.lanIps.length ? net.lanIps : [location.hostname];
    ipSelect.innerHTML = ips.map((ip) => `<option value="${ip}" ${ip === this._ipChoice ? 'selected' : ''}>${ip}</option>`).join('');
    if (!this._ipChoice || !ips.includes(this._ipChoice)) {
      this._ipChoice = ips[0];
    }
    ipSelect.onchange = () => {
      this._ipChoice = ipSelect.value;
      this._refreshQr();
    };

    const port = location.port ? ':' + location.port : '';
    const url = `${location.protocol}//${this._ipChoice}${port}/controller/index.html?room=${net.roomCode}`;
    this.linkInput.value = url;
    try {
      await QRCode.toCanvas(this.qrCanvas, url, {
        width: 208,
        margin: 1,
        color: { dark: '#0b1020', light: '#ffffff' },
      });
    } catch {
      this.ctx.hud.toast('QR generation failed');
    }
  }

  _refreshSlots() {
    const net = this.ctx.net;
    for (const slot of [1, 2]) {
      const s = net.slots.find((x) => x.slot === slot);
      const dot = document.getElementById(`dot-${slot}`);
      const meta = document.getElementById(`meta-${slot}`);
      const buzz = document.getElementById(`buzz-${slot}`);
      if (!dot) continue;
      if (s && s.connected) {
        dot.classList.add('on');
        meta.innerHTML = `<span>${s.name}</span><span>${s.device}</span><span>${s.battery !== 65535 ? s.battery + '%' : ''}</span><span>${s.rtt} ms</span>`;
        buzz.disabled = false;
      } else {
        dot.classList.remove('on');
        meta.textContent = 'waiting for a phone…';
        buzz.disabled = true;
      }
    }
  }

  /** Called by main when a lobby update arrives while the lobby is open. */
  onLobbyUpdate() {
    if (!this.lobbyEl.classList.contains('hidden')) this._refreshSlots();
  }

  /* ---------------------------------------------------------------- */
  /* Settings                                                         */
  /* ---------------------------------------------------------------- */

  _buildSettings() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'panel');
    card.style.minWidth = '520px';
    card.appendChild($('div', 'logo', 'Set<span>tings</span>'));
    const grid = $('div', 'settings-grid');
    const s = this.ctx.settings;

    const addSelect = (label, key, options, applyNow) => {
      const lab = $('label', null, label);
      const sel = $('select');
      for (const [value, text] of options) {
        const opt = $('option', null, text);
        opt.value = value;
        if (String(s.get(key)) === String(value)) opt.selected = true;
        sel.appendChild(opt);
      }
      sel.addEventListener('change', () => {
        s.set(key, sel.value);
        applyNow?.();
      });
      grid.append(lab, sel);
      return sel;
    };

    const addRange = (label, key, min, max, step, fmt, extraEl) => {
      const lab = $('label', null, label);
      const row = $('div', 'row2');
      const input = $('input');
      input.type = 'range';
      input.min = min; input.max = max; input.step = step;
      input.value = s.get(key);
      const val = $('span', 'val', fmt(s.get(key)));
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        s.set(key, v);
        val.textContent = fmt(v);
      });
      row.append(input, val);
      if (extraEl) row.appendChild(extraEl);
      grid.append(lab, row);
    };

    const addCheck = (label, key, applyNow) => {
      const lab = $('label', null, label);
      const box = $('div', 'check');
      const input = $('input');
      input.type = 'checkbox';
      input.checked = !!s.get(key);
      input.addEventListener('change', () => {
        s.set(key, input.checked);
        applyNow?.();
      });
      box.appendChild(input);
      box.appendChild($('span', null, 'on'));
      grid.append(lab, box);
    };

    /* Per-player variants (Stage B positioning settings are per player). */
    const addSelectAt = (label, key, index, options) => {
      const lab = $('label', null, label);
      const sel = $('select');
      for (const [value, text] of options) {
        const opt = $('option', null, text);
        opt.value = value;
        if (String(s.at(key, index)) === String(value)) opt.selected = true;
        sel.appendChild(opt);
      }
      sel.addEventListener('change', () => s.setAt(key, index, sel.value));
      grid.append(lab, sel);
      return sel;
    };

    const addRangeAt = (label, key, index, min, max, step, fmt) => {
      const lab = $('label', null, label);
      const row = $('div', 'row2');
      const input = $('input');
      input.type = 'range';
      input.min = min; input.max = max; input.step = step;
      input.value = s.at(key, index);
      const val = $('span', 'val', fmt(s.at(key, index)));
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        s.setAt(key, index, v);
        val.textContent = fmt(v);
      });
      row.append(input, val);
      grid.append(lab, row);
    };

    this._addSection(grid, 'Graphics');
    this.qSel = addSelect('Quality preset', 'quality',
      [['low', 'Low'], ['medium', 'Medium'], ['high', 'High']],
      () => this.ctx.hooks.applyGraphics());
    addCheck('Dynamic resolution', 'dynamicRes', () => this.ctx.hooks.applyGraphics());
    addCheck('Shadows', 'shadows', () => this.ctx.hooks.applyGraphics());
    addCheck('Post-processing', 'postFx', () => {}); // active from Phase 6

    this._addSection(grid, 'Audio');
    addRange('Master volume', 'master', 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`);
    addRange('SFX volume', 'sfx', 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`);
    addRange('Music volume', 'music', 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`);

    this._addSection(grid, 'Controls');
    addRange('Swing sensitivity', 'sensitivity', 0.5, 2, 0.05, (v) => `${v.toFixed(2)}x`);
    addRange('Swing threshold', 'swingThreshold', 8, 60, 1, (v) => `${v} m/s²`);
    addSelect('Assist strength', 'assist', [['off', 'Off'], ['low', 'Low'], ['medium', 'Medium'], ['high', 'High']]);

    this._addSection(grid, 'Positioning (per player)');
    for (const slot of [1, 2]) {
      const i = slot - 1;
      addSelectAt(`P${slot} positioning`, 'posMode', i, [
        ['assist', 'Assist only'],
        ['motion', 'Hand motion + assist'],
        ['motionOnly', 'Hand motion only'],
      ]);
      addRangeAt(`P${slot} hand sensitivity`, 'handSens', i, 0.4, 2, 0.05, (v) => `${v.toFixed(2)}x`);
      addRangeAt(`P${slot} return-to-center`, 'handReturn', i, 0, 2, 0.05, (v) => (v === 0 ? 'off (park anywhere)' : `${v.toFixed(2)}`));
      addRangeAt(`P${slot} motion dead zone`, 'handDead', i, 0, 1, 0.05, (v) => `${v.toFixed(2)} m/s²`);
    }

    const note = $('div', 'settings-note',
      'Assist and swing parameters drive the hit model from Phase 3. Hand motion (Stage B) moves the racket with your physical hand: hold the phone still and the position holds (no drift); recalibrating re-centres it. A phone without a gyroscope falls back to tilt-only tracking automatically.');

    const actions = $('div', 'lobby-actions');
    const reset = $('button', 'btn danger', 'Reset defaults');
    reset.addEventListener('click', () => {
      s.reset();
      this.openSettings(); // rebuild values
      this.ctx.hooks.applyGraphics();
    });
    const close = $('button', 'btn primary', 'Done');
    close.addEventListener('click', () => { this.ctx.audio.click(); this.closeSettings(); });
    actions.append(reset, close);

    card.append(grid, note, actions);
    el.appendChild(card);
    return el;
  }

  _addSection(grid, title) {
    // Section headers span both grid columns.
    const h = $('h3', 'section', title);
    h.style.gridColumn = '1 / -1';
    grid.appendChild(h);
  }

  openSettings() {
    this._prevScreen = this.settingsEl.classList.contains('hidden')
      ? (this.menuEl.classList.contains('hidden') ? this.lobbyEl : this.menuEl)
      : this._prevScreen;
    this._show(this.settingsEl);
  }

  closeSettings() {
    this._show(this._prevScreen || this.menuEl);
  }

  /* ---------------------------------------------------------------- */
  /* Pause                                                            */
  /* ---------------------------------------------------------------- */

  _buildPause() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'menu-card');
    card.appendChild($('div', 'logo', 'Pau<span>sed</span>'));
    const list = $('div', 'menu-list');
    const resume = $('button', 'menu-btn', '<span>Resume</span><span>ESC</span>');
    resume.addEventListener('click', () => this.ctx.hooks.onResume());
    const settings = $('button', 'menu-btn', '<span>Settings</span><span></span>');
    settings.addEventListener('click', () => this.openSettings());
    this.pauseQuitEl = $('button', 'menu-btn', '<span>Quit to lobby</span><span></span>');
    this.pauseQuitEl.addEventListener('click', () => this.ctx.hooks.onQuitToLobby());
    list.append(resume, settings, this.pauseQuitEl);
    card.appendChild(list);
    el.appendChild(card);
    return el;
  }

  /** Context label for the pause quit button ('Quit to menu' in practice). */
  setPauseQuitLabel(text) {
    if (this.pauseQuitEl) this.pauseQuitEl.firstChild.textContent = text;
  }

  openPause() { this._show(this.pauseEl); }
  closePause() { this.hideAll(); }

  /* ---------------------------------------------------------------- */
  /* Match end (Phase 4)                                              */
  /* ---------------------------------------------------------------- */

  _buildMatchEnd() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'menu-card');
    card.appendChild($('div', 'logo', 'Match <span>over</span>'));
    this.endScoreEl = $('div', 'end-score', '0 — 0');
    card.appendChild(this.endScoreEl);
    this.endWinnerEl = $('div', 'end-winner', '');
    card.appendChild(this.endWinnerEl);
    const list = $('div', 'menu-list');
    const again = $('button', 'menu-btn', '<span>Rematch</span><span class="soon">SERVER SWAPS</span>');
    again.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onRematch(); });
    const quit = $('button', 'menu-btn', '<span>Back to lobby</span><span></span>');
    quit.addEventListener('click', () => { this.ctx.audio.click(); this.ctx.hooks.onQuitToLobby(); });
    list.append(again, quit);
    card.appendChild(list);
    el.appendChild(card);
    return el;
  }

  /** `winnerName` won the game `a`—`b`. */
  showMatchEnd(winnerName, a, b) {
    this.endWinnerEl.textContent = `${winnerName} wins`;
    this.endScoreEl.textContent = `${a} — ${b}`;
    this._show(this.matchEndEl);
  }

  /* ---------------------------------------------------------------- */
  /* Ball machine panel (practice mode)                                */
  /* ---------------------------------------------------------------- */

  _buildMachinePanel() {
    const el = $('div', 'machine-panel hidden');
    el.appendChild($('h4', null, 'BALL MACHINE'));

    const rows = {};
    const addSlider = (label, key, min, max, step, fmt) => {
      const row = $('div', 'machine-row');
      row.appendChild($('span', null, label));
      const input = $('input');
      input.type = 'range';
      input.min = min; input.max = max; input.step = step;
      const val = $('span', 'val');
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        val.textContent = fmt(v);
        this.ctx.hooks.onMachineConfig({ [key]: v });
      });
      row.append(input, val);
      el.appendChild(row);
      rows[key] = { input, val, fmt };
    };

    addSlider('speed', 'speed', 2, 16, 0.5, (v) => `${v.toFixed(1)} m/s`);
    addSlider('spin', 'spinRpm', 0, 3000, 100, (v) => `${v} rpm`);
    addSlider('rate', 'interval', 0.4, 3, 0.05, (v) => `${v.toFixed(2)} s`);

    const rowS = $('div', 'machine-row');
    rowS.appendChild($('span', null, 'spin type'));
    const sel = $('select');
    for (const [v, t] of [
      ['mix', 'mix'], ['topspin', 'topspin'], ['backspin', 'backspin'],
      ['sidespinL', 'side L'], ['sidespinR', 'side R'], ['none', 'none'],
    ]) {
      const o = $('option', null, t);
      o.value = v;
      sel.appendChild(o);
    }
    sel.addEventListener('change', () => this.ctx.hooks.onMachineConfig({ spinType: sel.value }));
    rowS.appendChild(sel);
    rowS.appendChild($('span', 'val', ''));
    el.appendChild(rowS);
    rows.spinType = { el: sel };

    const rowR = $('div', 'machine-row');
    rowR.appendChild($('span', null, 'randomize'));
    const chk = $('input');
    chk.type = 'checkbox';
    chk.addEventListener('change', () => this.ctx.hooks.onMachineConfig({ randomize: chk.checked }));
    rowR.append(chk, $('span', 'val', ''));
    el.appendChild(rowR);
    rows.randomize = { el: chk };

    const btns = $('div', 'machine-btns');
    this.machineToggleBtn = $('button', 'btn small primary', 'STOP');
    this.machineToggleBtn.addEventListener('click', () => {
      const on = !(this._machineCfg && this._machineCfg.running);
      this.ctx.hooks.onMachineConfig({ running: on });
      this._setMachineToggle(on);
    });
    const toss = $('button', 'btn small', 'Toss one (SPACE)');
    toss.addEventListener('click', () => this.ctx.hooks.onMachineToss());
    btns.append(this.machineToggleBtn, toss);
    el.appendChild(btns);

    this.machineStatusEl = $('div', 'machine-status', 'idle');
    el.appendChild(this.machineStatusEl);

    this._machineRows = rows;
    return el;
  }

  _setMachineToggle(on) {
    this.machineToggleBtn.textContent = on ? 'STOP' : 'START';
    this.machineToggleBtn.classList.toggle('primary', on);
  }

  showMachinePanel(v) {
    this.machineEl.classList.toggle('hidden', !v);
  }

  /** Push the sport's machine config into the panel controls. */
  syncMachinePanel(cfg) {
    this._machineCfg = cfg;
    const r = this._machineRows;
    if (!r) return;
    r.speed.input.value = cfg.speed;
    r.speed.val.textContent = r.speed.fmt(cfg.speed);
    r.spinRpm.input.value = cfg.spinRpm;
    r.spinRpm.val.textContent = r.spinRpm.fmt(cfg.spinRpm);
    r.interval.input.value = cfg.interval;
    r.interval.val.textContent = r.interval.fmt(cfg.interval);
    r.spinType.el.value = cfg.spinType;
    r.randomize.el.checked = !!cfg.randomize;
    this._setMachineToggle(!!cfg.running);
  }

  updateMachineStatus(text) {
    this.machineStatusEl.textContent = text;
  }
}

```

## 10. `controller/controller.js`

```js
/**
 * REALPLAY phone controller.
 *
 * - Connects to the game server (same origin, /ws) with a room code.
 * - Streams a compact binary MOTION frame at up to 100 Hz:
 *     orientation (quaternion from alpha/beta/gamma, smoothed),
 *     linear acceleration (gravity removed), rotation rate (gyro),
 *     swing power, battery, clock-synced timestamp.
 * - Swing detection: linear acceleration magnitude crosses a threshold,
 *   opening a ~160 ms window; acceleration is leaky-integrated into a
 *   velocity estimate whose peak gives the swing power.
 * - Optional thumb touchpad sends normalized nudge vectors (TOUCH frames).
 * - Calibration: captures the "hold like a racket" reference pose. The PC
 *   averages the next 300 ms of samples as its baseline, so both ends agree.
 * - Latency: measured with a SYNC/SYNC_ACK round trip every 2 s; the clock
 *   offset lets the phone stamp frames in server time.
 *
 * No frameworks, no build-time dependencies beyond the shared protocol.
 */

import {
  WS_PATH, T, FLAGS, BATTERY_UNKNOWN,
  encodeMotion, encodeTouch, HAPTIC_PATTERNS,
} from '../shared/protocol.js';

/* ========================================================================= *
 * Tiny dependency-free quaternion math (right-handed, matches three.js).
 * ========================================================================= */

function eulerYXZToQuat(out, x, y, z) {
  // Exact port of three.js Quaternion.setFromEuler({ order: 'YXZ' })
  const c1 = Math.cos(x / 2), c2 = Math.cos(y / 2), c3 = Math.cos(z / 2);
  const s1 = Math.sin(x / 2), s2 = Math.sin(y / 2), s3 = Math.sin(z / 2);
  out[0] = s1 * c2 * c3 + c1 * s2 * s3;
  out[1] = c1 * s2 * c3 + s1 * c2 * s3;
  out[2] = c1 * c2 * s3 - s1 * s2 * c3;
  out[3] = c1 * c2 * c3 - s1 * s2 * s3;
}

function qMultiply(out, a, b) {
  const ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const bx = b[0], by = b[1], bz = b[2], bw = b[3];
  out[0] = ax * bw + aw * bx + ay * bz - az * by;
  out[1] = ay * bw + aw * by + az * bx - ax * bz;
  out[2] = az * bw + aw * bz + ax * by - ay * bx;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
}

function qSlerp(out, a, b, t) {
  let ax = a[0], ay = a[1], az = a[2], aw = a[3];
  let bx = b[0], by = b[1], bz = b[2], bw = b[3];
  let cos = ax * bx + ay * by + az * bz + aw * bw;
  if (cos < 0) { cos = -cos; bx = -bx; by = -by; bz = -bz; bw = -bw; }
  let s0, s1;
  if (cos > 0.9995) { s0 = 1 - t; s1 = t; }
  else {
    const theta = Math.acos(Math.min(1, cos));
    const sin = Math.sin(theta);
    s0 = Math.sin((1 - t) * theta) / sin;
    s1 = Math.sin(t * theta) / sin;
  }
  out[0] = ax * s0 + bx * s1;
  out[1] = ay * s0 + by * s1;
  out[2] = az * s0 + bz * s1;
  out[3] = aw * s0 + bw * s1;
  const n = 1 / Math.hypot(out[0], out[1], out[2], out[3]) || 1;
  out[0] *= n; out[1] *= n; out[2] *= n; out[3] *= n;
}

/** Rotate vector v (array [x,y,z]) by the INVERSE of quaternion q. */
function rotateByInverse(out, q, x, y, z) {
  const cx = -q[0], cy = -q[1], cz = -q[2], cw = q[3];
  // v' = q * v * q*
  const vx = cw * x + cy * z - cz * y;
  const vy = cw * y + cz * x - cx * z;
  const vz = cw * z + cx * y - cy * x;
  const tx = -cx * x - cy * y - cz * z;
  out[0] = vx * cw + tx * -cx + (vy * -cz - vz * -cy);
  out[1] = vy * cw + tx * -cy + (vz * -cx - vx * -cz);
  out[2] = vz * cw + tx * -cz + (vx * -cy - vy * -cx);
}

const deg2rad = Math.PI / 180;

/* ========================================================================= *
 * State
 * ========================================================================= */

const $ = (id) => document.getElementById(id);

const params = new URLSearchParams(location.search);
const S = {
  ws: null,
  wsOpen: false,
  room: (params.get('room') || '').toUpperCase(),
  wantedName: (params.get('name') || '').slice(0, 16),
  slot: 0,
  offsetMs: 0,       // serverTime ≈ clientTime + offsetMs
  rttMs: 0,
  // sensors
  sensorsOn: false,
  hasOrient: false,
  hasMotion: false,
  firstSensorAt: 0,
  qTarget: [0, 0, 0, 1],
  qSmooth: [0, 0, 0, 1],
  lastSmoothTs: 0,
  lastMotionTs: 0,
  screenAngle: 0,
  // no-gyro tilt fallback (Stage B): some phones lack a gyroscope — their
  // fused orientation drifts and swings are unusable, but tilt still works.
  noGyro: false,
  gyroDetected: false,
  gyroNullFrames: 0,   // devicemotion events without a rotationRate field
  gyroZeroFrames: 0,   // events with an exactly-zero rotation rate
  alphaFrozen: null,   // yaw locked at calibration (tilt-only mode)
  lastAlpha: 0,
  lastBeta: 0,
  lastGamma: 0,
  gFilt: [0, 0, 0],    // low-passed tilt-only gravity (device frame)
  gFiltInit: false,
  // linear acceleration (device frame)
  ax: 0, ay: 0, az: 0,
  gx: 0, gy: 0, gz: 0,
  // swing detector
  vx: 0, vy: 0, vz: 0,
  swingActive: false,
  swingStart: 0,
  swingPeak: 0,
  lastSwingPower: 0,
  lastSwingAt: 0,
  pendingSwingEvent: false,
  calibFlagUntil: 0,
  calSamplingUntil: 0,
  calSamples: [],
  // touchpad
  touch: { active: false, dx: 0, dy: 0, dirty: false },
  battery: BATTERY_UNKNOWN,
  charging: false,
  // loops
  sendTimer: null,
  syncTimer: null,
  syncPending: false,
  wakeLock: null,
  reconnectTimer: null,
};

const SWING_WINDOW_MS = 160;   // swing detection window
const SWING_MIN_POWER = 3.0;   // m/s, below this the "swing" is treated as noise
const SWING_DECAY_TAU = 0.25;  // s, leaky integrator time constant
const ORIENT_TAU = 0.035;      // s, orientation smoothing time constant
const ORIENT_TAU_TILT = 0.12;  // s, heavier smoothing without a gyroscope
const SEND_HZ = 100;           // max motion frames per second
const MAX_BUFFERED = 16384;    // bytes; skip sends while the socket is congested

// Scratch for the tilt-only gravity path (no per-event allocation).
const _gq = [0, 0, 0, 1];
const _gTilt = [0, 0, 0];
const _qScratch = [0, 0, 0, 1];

const epochNow = () => performance.timeOrigin + performance.now();

/* ========================================================================= *
 * UI helpers
 * ========================================================================= */

function show(screen) {
  $('screen-connect').classList.toggle('hidden', screen !== 'connect');
  $('screen-play').classList.toggle('hidden', screen !== 'play');
}

let toastTimer = 0;
function toast(text, ms = 1800) {
  const el = $('toast');
  el.textContent = text;
  el.classList.remove('hidden');
  el.style.opacity = '1';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.style.opacity = '0'; }, ms);
}

function setMainStatus(text, kind = '') {
  const el = $('status-main');
  el.textContent = text;
  el.className = 'status-main ' + kind;
}

function setSubStatus(text) {
  $('status-sub').textContent = text || '';
}

function deviceName() {
  const ua = navigator.userAgent;
  if (/iPhone/i.test(ua)) return 'iPhone';
  if (/iPad/i.test(ua)) return 'iPad';
  const m = ua.match(/Android[^;)]*;\s*([^;)]+build)/i);
  if (m) return m[1].replace(/build/i, '').trim().slice(0, 16);
  if (/Android/i.test(ua)) return 'Android';
  return 'Phone';
}

/* ========================================================================= *
 * Networking
 * ========================================================================= */

function wsUrl() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${proto}//${location.host}${WS_PATH}`;
}

function connect() {
  if (S.ws && (S.ws.readyState === 0 || S.ws.readyState === 1)) return;
  let ws;
  try {
    ws = new WebSocket(wsUrl());
    ws.binaryType = 'arraybuffer';
  } catch {
    scheduleReconnect();
    return;
  }
  S.ws = ws;

  ws.onopen = () => {
    ws.send(JSON.stringify({
      t: T.CTRL_HELLO,
      room: S.room,
      name: S.wantedName || deviceName(),
      device: `${deviceName()} ${screen.width}x${screen.height}`,
    }));
  };

  ws.onmessage = (e) => {
    if (e.data instanceof ArrayBuffer) return;
    let msg;
    try { msg = JSON.parse(e.data); } catch { return; }
    switch (msg.t) {
      case T.WELCOME:
        S.slot = msg.slot;
        $('slot-badge').textContent = `P${msg.slot}`;
        $('stat-room').textContent = `ROOM ${msg.code}`;
        setMainStatus('Connected!', 'good');
        setSubStatus('Hold the phone like a racket, then tap CALIBRATE.');
        startLoops();
        startSensorsIfNeeded();
        requestWakeLock();
        break;

      case T.STATUS:
        setMainStatus(msg.text || '', msg.kind === 'good' ? 'good' : msg.kind === 'bad' ? 'bad' : '');
        break;

      case T.HAPTIC: {
        const pattern = HAPTIC_PATTERNS[msg.pattern];
        if (pattern != null && navigator.vibrate) {
          try { navigator.vibrate(pattern); } catch { /* ignore */ }
        }
        break;
      }

      case T.SYNC_ACK: {
        const t2 = epochNow();
        const rtt = t2 - S.syncT0;
        if (rtt >= 0 && rtt < 5000) {
          S.rttMs = S.rttMs ? S.rttMs * 0.6 + rtt * 0.4 : rtt;
          const offset = msg.t1 - (S.syncT0 + rtt / 2);
          S.offsetMs = S.offsetMs ? S.offsetMs * 0.6 + offset * 0.4 : offset;
        }
        S.syncPending = false;
        break;
      }

      case T.ERROR:
        if (msg.code === 'unknown_room') {
          setMainStatus('Room not found', 'bad');
          setSubStatus(msg.message || '');
          stopReconnect();
        } else if (msg.code === 'room_closed') {
          setMainStatus('Room closed', 'bad');
          setSubStatus('The PC ended the session.');
          S.slot = 0;
        } else if (msg.code === 'room_full') {
          setMainStatus('Room is full', 'bad');
          setSubStatus(msg.message || '');
          stopReconnect();
        }
        break;
    }
  };

  ws.onclose = () => {
    S.wsOpen = false;
    S.ws = null;
    if (S.slot) {
      setMainStatus('Reconnecting…', 'bad');
      setSubStatus('Check your Wi-Fi connection.');
      S.slot = 0;
    }
    scheduleReconnect();
  };

  ws.onerror = () => { /* onclose follows */ };
}

function scheduleReconnect() {
  if (S.reconnectTimer) return;
  S.reconnectTimer = setInterval(() => {
    if (!S.ws || S.ws.readyState === 3) connect();
  }, 1200);
}

function stopReconnect() {
  clearInterval(S.reconnectTimer);
  S.reconnectTimer = null;
}

function startLoops() {
  if (!S.sendTimer) S.sendTimer = setInterval(sendTick, 1000 / SEND_HZ);
  if (!S.syncTimer) {
    S.syncTimer = setInterval(() => {
      if (!S.wsOpen || S.syncPending) return;
      S.syncPending = true;
      S.syncT0 = epochNow();
      safeSendJson({ t: T.SYNC, t0: S.syncT0 });
    }, 2000);
  }
}

function safeSendJson(obj) {
  if (S.ws && S.ws.readyState === 1) {
    try { S.ws.send(JSON.stringify(obj)); } catch { /* ignore */ }
  }
}

/* ========================================================================= *
 * Sensors
 * ========================================================================= */

function screenAngle() {
  if (screen.orientation && typeof screen.orientation.angle === 'number') {
    return screen.orientation.angle;
  }
  return window.orientation || 0;
}

const needsOrientationPermission =
  typeof DeviceOrientationEvent !== 'undefined' &&
  typeof DeviceOrientationEvent.requestPermission === 'function';
const needsMotionPermission =
  typeof DeviceMotionEvent !== 'undefined' &&
  typeof DeviceMotionEvent.requestPermission === 'function';

function startSensorsIfNeeded() {
  if (S.sensorsOn) return;
  if (needsOrientationPermission || needsMotionPermission) {
    $('motion-gate').classList.remove('hidden');
  } else {
    attachSensorListeners();
  }
}

async function enableMotion() {
  try {
    if (needsOrientationPermission) await DeviceOrientationEvent.requestPermission();
    if (needsMotionPermission) await DeviceMotionEvent.requestPermission();
  } catch {
    toast('Permission denied — motion will not work');
  }
  $('motion-gate').classList.add('hidden');
  attachSensorListeners();
}

let listenersAttached = false;
function attachSensorListeners() {
  if (listenersAttached) return;
  listenersAttached = true;
  S.sensorsOn = true;
  S.firstSensorAt = performance.now();
  window.addEventListener('deviceorientation', onDeviceOrientation);
  window.addEventListener('devicemotion', onDeviceMotion);
  screen.addEventListener?.('orientationchange', () => { S.screenAngle = -1; });
  $('sensor-chip').classList.add('on');
  $('sensor-chip').textContent = 'MOTION: WAIT…';
}

function onDeviceOrientation(e) {
  if (e.alpha == null || e.beta == null || e.gamma == null) return;
  const alphaRaw = e.alpha * deg2rad;
  const beta = e.beta * deg2rad;
  const gamma = e.gamma * deg2rad;
  S.lastAlpha = alphaRaw;
  S.lastBeta = beta;
  S.lastGamma = gamma;
  const orient = (S.screenAngle === -1 ? screenAngle() : S.screenAngle) * deg2rad;

  // Tilt-only fallback: without a gyro the compass-derived alpha drifts and
  // wobbles badly, so once calibration locks the grip we freeze the yaw and
  // keep only the accelerometer-derived tilt angles live.
  const alpha = (S.noGyro && S.alphaFrozen != null) ? S.alphaFrozen : alphaRaw;

  // Device -> world quaternion (Y-up world), the classic fusion formula:
  //   q = qEuler(beta, alpha, -gamma | 'YXZ') * qX(-90°) * qZ(-screenAngle)
  eulerYXZToQuat(S.qTarget, beta, alpha, -gamma);
  const qX90 = [-Math.SQRT1_2, 0, 0, Math.SQRT1_2]; // -90° around X
  qMultiply(S.qTarget, S.qTarget, qX90);
  qMultiply(S.qTarget, S.qTarget, [0, 0, Math.sin(-orient / 2), Math.cos(-orient / 2)]);

  if (!S.hasOrient) {
    S.qSmooth[0] = S.qTarget[0]; S.qSmooth[1] = S.qTarget[1];
    S.qSmooth[2] = S.qTarget[2]; S.qSmooth[3] = S.qTarget[3];
    S.hasOrient = true;
    markSensorAlive();
  }
}

function onDeviceMotion(e) {
  const now = performance.now();
  const dt = Math.min(0.05, Math.max(0.002, (now - S.lastMotionTs) / 1000));
  S.lastMotionTs = now;

  const aWG = e.accelerationIncludingGravity;
  if (!aWG || aWG.x == null) return;

  /* ---- gyroscope presence detection (Stage B tilt-only fallback) ------- */
  const rot = e.rotationRate;
  if (rot && rot.alpha != null) {
    S.gx = (rot.alpha || 0) * deg2rad;
    S.gy = (rot.beta || 0) * deg2rad;
    S.gz = (rot.gamma || 0) * deg2rad;
    if (!S.gyroDetected) {
      if (Math.abs(S.gx) + Math.abs(S.gy) + Math.abs(S.gz) > 1e-6) {
        S.gyroDetected = true; // real gyro: never exactly zero (sensor noise)
      } else {
        S.gyroZeroFrames++;
      }
    }
  } else {
    S.gyroNullFrames++;
  }
  if (!S.gyroDetected && !S.noGyro && now - S.firstSensorAt > 2000) {
    if (S.gyroNullFrames > 5 || S.gyroZeroFrames > 100) {
      enableTiltOnly();
    } else {
      S.gyroDetected = true; // rotation data present but quiet — it has a gyro
    }
  }

  // Gravity direction in device frame.
  const g = [0, 0, 0];
  if (S.noGyro) {
    // Secondary tilt-only gravity: derive it from the tilt angles ALONE
    // (gravity is yaw-invariant, so the noisy/frozen alpha is excluded) and
    // low-pass heavily — the tilt estimate is noisy without a gyro. Same
    // composition as the orientation formula minus the alpha rotation.
    if (S.hasOrient) {
      const orient = (S.screenAngle === -1 ? screenAngle() : S.screenAngle) * deg2rad;
      eulerYXZToQuat(_gq, S.lastBeta, 0, -S.lastGamma);
      qMultiply(_gq, _gq, [-Math.SQRT1_2, 0, 0, Math.SQRT1_2]); // -90° around X
      _qScratch[0] = 0; _qScratch[1] = 0;
      _qScratch[2] = Math.sin(-orient / 2); _qScratch[3] = Math.cos(-orient / 2);
      qMultiply(_gq, _gq, _qScratch);
      rotateByInverse(_gTilt, _gq, 0, -9.81, 0);
      if (!S.gFiltInit) {
        S.gFilt[0] = _gTilt[0]; S.gFilt[1] = _gTilt[1]; S.gFilt[2] = _gTilt[2];
        S.gFiltInit = true;
      } else {
        const k = 1 - Math.exp(-dt / 0.25);
        S.gFilt[0] += (_gTilt[0] - S.gFilt[0]) * k;
        S.gFilt[1] += (_gTilt[1] - S.gFilt[1]) * k;
        S.gFilt[2] += (_gTilt[2] - S.gFilt[2]) * k;
      }
    }
    g[0] = S.gFilt[0]; g[1] = S.gFilt[1]; g[2] = S.gFilt[2];
  } else {
    // g_dev = q⁻¹ · (0,-1,0) · 9.81 from the fused orientation
    rotateByInverse(g, S.qSmooth, 0, -9.81, 0);
  }

  const accel = e.acceleration;
  if (accel && accel.x != null && accel.y != null && accel.z != null) {
    // OS-provided gravity-removed acceleration (iOS) — prefer it.
    S.ax = accel.x; S.ay = accel.y; S.az = accel.z;
  } else {
    S.ax = aWG.x - g[0]; S.ay = aWG.y - g[1]; S.az = aWG.z - g[2];
  }

  if (!S.hasMotion) {
    S.hasMotion = true;
    markSensorAlive();
  }

  /* ---- swing detection ------------------------------------------------ */
  // Leaky-integrate linear acceleration into a velocity estimate; when the
  // acceleration magnitude spikes, open a window and track the peak speed.
  const decay = Math.exp(-dt / SWING_DECAY_TAU);
  S.vx = (S.vx + S.ax * dt) * decay;
  S.vy = (S.vy + S.ay * dt) * decay;
  S.vz = (S.vz + S.az * dt) * decay;
  const vMag = Math.hypot(S.vx, S.vy, S.vz);
  if (vMag > 30) { const k = 30 / vMag; S.vx *= k; S.vy *= k; S.vz *= k; }

  const aMag = Math.hypot(S.ax, S.ay, S.az);
  if (!S.swingActive && aMag > SWING_THRESHOLD()) {
    S.swingActive = true;
    S.swingStart = now;
    S.swingPeak = 0;
  }
  if (S.swingActive) {
    if (vMag > S.swingPeak) S.swingPeak = vMag;
    if (now - S.swingStart > SWING_WINDOW_MS) {
      S.swingActive = false;
      if (S.swingPeak > SWING_MIN_POWER) {
        S.pendingSwingEvent = true;
        S.lastSwingPower = Math.min(S.swingPeak, 25);
        S.lastSwingAt = now;
      }
    }
  }
}

let sensitivity = 1.0;
function SWING_THRESHOLD() {
  return 24 / sensitivity; // m/s^2
}

/** Switch to tilt-only tracking (no gyroscope detected). */
function enableTiltOnly() {
  if (S.noGyro) return;
  S.noGyro = true;
  const chip = $('sensor-chip');
  chip.textContent = 'MOTION: LIVE · TILT';
  toast('Limited sensors — tilt-only tracking', 4000);
  setSubStatus('No gyroscope detected: orientation uses tilt only. Recalibrate after changing your grip.');
  navigator.vibrate?.([40, 60, 40]);
}

function markSensorAlive() {
  const el = $('sensor-chip');
  el.classList.add('on');
  el.textContent = S.noGyro ? 'MOTION: LIVE · TILT' : 'MOTION: LIVE';
}

/* ========================================================================= *
 * Sending
 * ========================================================================= */

function sendTick() {
  if (!S.ws || S.ws.readyState !== 1 || !S.slot) return;
  const now = performance.now();

  // Orientation smoothing (time-constant ORIENT_TAU; heavier without a gyro —
  // the tilt-only fused orientation is noisier).
  if (S.hasOrient) {
    const dt = Math.min(0.05, Math.max(0.001, (now - S.lastSmoothTs) / 1000));
    S.lastSmoothTs = now;
    const tau = S.noGyro ? ORIENT_TAU_TILT : ORIENT_TAU;
    qSlerp(S.qSmooth, S.qSmooth, S.qTarget, 1 - Math.exp(-dt / tau));
  }

  // Calibration capture (300 ms average happens on the PC; flag tells it to start).
  if (S.calSamplingUntil) {
    if (now < S.calSamplingUntil) {
      S.calSamples.push([S.qSmooth[0], S.qSmooth[1], S.qSmooth[2], S.qSmooth[3]]);
      return;
    }
    S.calSamplingUntil = 0;
    S.calSamples.length = 0;
    S.calibFlagUntil = now + 1000;
    safeSendJson({ t: T.EV, name: 'calibrated', data: null });
    setSubStatus('Calibrated ✓ — keep that grip.');
    navigator.vibrate?.(20);
  }

  if (S.ws.bufferedAmount > MAX_BUFFERED) return; // congestion: skip this tick

  let flags = 0;
  if (S.swingActive) flags |= FLAGS.SWING_ACTIVE;
  if (S.pendingSwingEvent) flags |= FLAGS.SWING_EVENT;
  if (now < S.calibFlagUntil) flags |= FLAGS.CALIBRATED;
  if (S.noGyro) flags |= FLAGS.NO_GYRO; // Stage B: PC switches to tilt-only tuning
  S.pendingSwingEvent = false;

  const swing = S.swingActive ? S.swingPeak : (now - S.lastSwingAt < 500 ? S.lastSwingPower : 0);

  const frame = encodeMotion({
    flags,
    battery: S.battery,
    time: epochNow() + S.offsetMs,
    qx: S.qSmooth[0], qy: S.qSmooth[1], qz: S.qSmooth[2], qw: S.qSmooth[3],
    ax: S.ax, ay: S.ay, az: S.az,
    gx: S.gx, gy: S.gy, gz: S.gz,
    swing,
  });

  try {
    S.ws.send(frame);
  } catch { /* ignore */ }

  // Touchpad: 30 Hz while active, or immediately after a change.
  if (S.touch.active || S.touch.dirty) {
    if (S.touch.active || now - (S.lastTouchSend || 0) > 33) {
      S.lastTouchSend = now;
      S.touch.dirty = false;
      try { S.ws.send(encodeTouch(S.touch)); } catch { /* ignore */ }
    }
  }
}

/* ========================================================================= *
 * Calibration
 * ========================================================================= */

function calibrate() {
  if (!S.hasOrient) {
    toast('No sensor data yet — see the motion chip');
    return;
  }
  if (S.noGyro) {
    // Tilt-only fallback: lock the (drifting) compass yaw at the calibration
    // pose — the PC averages its baseline over the next 300 ms of samples,
    // so the orientation must hold still during the capture.
    S.alphaFrozen = S.lastAlpha;
  }
  S.calSamples.length = 0;
  S.calSamplingUntil = performance.now() + 300;
  setSubStatus('Hold still…');
  const btn = $('calibrate-btn');
  btn.disabled = true;
  setTimeout(() => { btn.disabled = false; }, 400);
}

/* ========================================================================= *
 * Touchpad
 * ========================================================================= */

function bindTouchpad() {
  const pad = $('pad');
  const knob = $('pad-knob');
  let radius = 1;

  const setKnob = (dx, dy) => {
    knob.style.transform = `translate(${dx * radius * 0.7}px, ${-dy * radius * 0.7}px)`;
  };

  pad.addEventListener('pointerdown', (e) => {
    pad.setPointerCapture(e.pointerId);
    const r = pad.getBoundingClientRect();
    radius = Math.min(r.width, r.height) * 0.38;
    S.touch.active = true;
    S.touch.dirty = true;
    pad.classList.add('active');
    move(e, r);
  });
  pad.addEventListener('pointermove', (e) => {
    if (!S.touch.active) return;
    move(e, pad.getBoundingClientRect());
  });
  const release = () => {
    if (!S.touch.active) return;
    S.touch.active = false;
    S.touch.dx = 0; S.touch.dy = 0;
    S.touch.dirty = true;
    pad.classList.remove('active');
    knob.style.transform = '';
  };
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
  pad.addEventListener('lostpointercapture', release);

  function move(e, r) {
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    let dx = (e.clientX - cx) / radius;
    let dy = -(e.clientY - cy) / radius; // +y = up
    const m = Math.hypot(dx, dy);
    if (m > 1) { dx /= m; dy /= m; }
    S.touch.dx = Math.round(dx * 1000) / 1000;
    S.touch.dy = Math.round(dy * 1000) / 1000;
    S.touch.dirty = true;
    setKnob(S.touch.dx, S.touch.dy);
  }
}

/* ========================================================================= *
 * Battery / wake lock / fullscreen
 * ========================================================================= */

async function initBattery() {
  if (!navigator.getBattery) return;
  try {
    const b = await navigator.getBattery();
    const update = () => {
      S.battery = Math.round(b.level * 100);
      S.charging = b.charging;
      $('stat-battery').textContent = `${S.battery}%${S.charging ? '⚡' : ''}`;
    };
    update();
    b.addEventListener('levelchange', update);
    b.addEventListener('chargingchange', update);
  } catch { /* ignore */ }
}

async function requestWakeLock() {
  if (!navigator.wakeLock) return;
  try {
    S.wakeLock = await navigator.wakeLock.request('screen');
    S.wakeLock.addEventListener?.('release', () => { S.wakeLock = null; });
  } catch { /* ignore */ }
}

async function goFullscreen() {
  try {
    await document.documentElement.requestFullscreen({ navigationUI: 'hide' });
    try { await screen.orientation.lock('portrait'); } catch { /* iOS has no lock — the protocol handles rotation anyway */ }
    toast('Fullscreen on');
  } catch {
    toast('Fullscreen blocked — add to Home Screen for best results');
  }
}

/* ========================================================================= *
 * UI watch dog (2 Hz): latency chip + sensor watchdog
 * ========================================================================= */

function startUiWatchdog() {
  setInterval(() => {
    $('stat-latency').textContent = S.rttMs ? `${Math.round(S.rttMs)} ms` : '-- ms';
    if (S.sensorsOn && !S.hasOrient && performance.now() - S.firstSensorAt > 2500 && !isSecureContext) {
      $('https-warn').classList.remove('hidden');
    }
  }, 500);
}

/* ========================================================================= *
 * Boot
 * ========================================================================= */

function join() {
  const code = ($('room-input').value || '').trim().toUpperCase();
  if (code.length < 3) {
    $('connect-error').textContent = 'Enter the 4-letter room code.';
    $('connect-error').classList.remove('hidden');
    return;
  }
  S.room = code;
  S.wantedName = $('name-input').value.trim().slice(0, 16);
  history.replaceState(null, '', `?room=${code}`);
  show('play');
  setMainStatus('Connecting…');
  connect();
}

function exit() {
  stopReconnect();
  clearInterval(S.sendTimer); S.sendTimer = null;
  clearInterval(S.syncTimer); S.syncTimer = null;
  try { S.wakeLock?.release(); } catch { /* ignore */ }
  try { S.ws?.close(1000, 'exit'); } catch { /* ignore */ }
  S.ws = null;
  S.slot = 0;
  show('connect');
}

function boot() {
  $('join-btn').addEventListener('click', join);
  $('room-input').addEventListener('keydown', (e) => { if (e.key === 'Enter') join(); });
  $('room-input').addEventListener('input', () => {
    $('room-input').value = $('room-input').value.toUpperCase().replace(/[^A-Z]/g, '');
  });
  if (S.room) $('room-input').value = S.room;
  $('name-input').value = S.wantedName;

  $('calibrate-btn').addEventListener('click', calibrate);
  $('fullscreen-btn').addEventListener('click', goFullscreen);
  $('exit-btn').addEventListener('click', exit);
  $('enable-motion-btn').addEventListener('click', enableMotion);
  $('https-ok-btn').addEventListener('click', () => $('https-warn').classList.add('hidden'));

  bindTouchpad();
  initBattery();
  startUiWatchdog();

  document.addEventListener('visibilitychange', () => {
    if (!document.hidden && S.slot) requestWakeLock();
  });

  // Sensitivity slider is stored per-device; expose via query param ?sens=1.4
  const sens = parseFloat(params.get('sens'));
  if (Number.isFinite(sens)) sensitivity = Math.min(2, Math.max(0.5, sens));

  if (S.room) {
    show('play');
    setMainStatus('Connecting…');
    connect();
  } else {
    show('connect');
  }
}

boot();

```
