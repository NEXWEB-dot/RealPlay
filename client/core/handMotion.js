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

  /**
   * Stage C: adopt an external position/velocity estimate (the fusion KF
   * handover) — integration continues from there instead of gliding home
   * from zero. Only meaningful while the filter is live (fresh samples);
   * after a stream gap the next update() re-seeds to neutral, as designed.
   */
  adopt(px, py, pz, vx, vy, vz) {
    this.pos.set(px, py, pz);
    this.vel.set(vx, vy, vz);
    this.started = true;
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
