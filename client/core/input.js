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
