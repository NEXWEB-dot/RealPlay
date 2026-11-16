/**
 * Fusion KF (Stage C) — fuses the webcam ArUco marker position (absolute,
 * ~30 Hz, σ ≈ 5–15 mm lateral / 3–6 cm depth) with the phone's IMU hand
 * estimate (relative, 60–100 Hz) into one POSITION + VELOCITY estimate.
 *
 * Per axis (x/y/z independent): state = [pos, vel], control input =
 * the IMU corrected accel (bias-learned + dead-zoned by the Stage B hand
 * filter, so "still" really is zero).
 *
 *   predict: p += v·dt + ½u·dt²      v += u·dt
 *            P = F·P·Fᵀ + G·σa²·Gᵀ   (F = [[1,dt],[0,1]], G = [½dt², dt]ᵀ)
 *   update:  H = [1,0], R = σ_cam²:
 *            K = [P00, P10]/(P00+R)  p += K0·inn  v += K1·inn  P = (I−KH)·P
 *
 * IMU REPLAY: camera frames carry 1–3 frames of pipeline latency while the
 * IMU keeps integrating forward. Every predict stores a checkpoint
 * {t, pending accel, state, P} in a ring buffer; a measurement dated t_m
 * rewinds to the newest checkpoint with t ≤ t_m, applies the update there,
 * then replays all stored inputs after it. Deterministic (same ops, same
 * order ⇒ the replayed state is bit-identical to the forward state), and
 * allocation-free: the ring is preallocated once.
 *
 * Outlier gate: a 3-D innovation beyond GATE·√(P00+R) per axis (combined) is
 * rejected as a whole — protects against marker swaps and mis-detections.
 *
 * Frame convention (CAMERA frame, NOT the player frame):
 *   X = image right, Y = image up, Z = away from the webcam (depth).
 * The caller (cameraTracker) subtracts the per-slot ready-pose home and
 * converts to the calibrated player frame (+x player's left, +y up,
 * +z toward net = toward the camera, so z_cal = −ΔZ).
 *
 * Node-safe + worker-safe: no THREE, no DOM. `npm run test:fusion`.
 */

const MAX_HISTORY = 96; // ~1.2 s at 80 Hz — replays never need more

export class HandFusionFilter {
  constructor() {
    this.cfg = {
      sigmaA: 0.35,   // m/s² — IMU accel uncertainty (post dead-zone)
      gate: 4.0,      // innovation gate (combined, in σ units)
    };

    this.p = [0, 0, 0];        // position (camera frame, m)
    this.v = [0, 0, 0];        // velocity (m/s)
    // Covariance per axis, symmetric 2×2 as [P00, P01(=P10), P11].
    // P00 0.15² m — the home-capture puts truth within ~15 cm of the seed;
    // P11 0.2² m/s — hands start near rest relative to the ready pose.
    this.P = [
      [0.0225, 0, 0.04],
      [0.0225, 0, 0.04],
      [0.0225, 0, 0.04],
    ];

    this.started = false;
    this.nMeas = 0;
    this.nRejected = 0;
    this.nClamped = 0;         // measurements older than the buffer
    this._rejStreak = 0;       // consecutive gate rejections (force re-lock)
    this.lastMeasT = 0;
    this.lastMeas = [0, 0, 0];
    this.lastMeasSigma = [0, 0, 0];

    /* Checkpoint ring. Entry k = state as of t_k BEFORE integrating its
     * pending accel a_k (which governs [t_k, t_{k+1}]). */
    this._hist = new Array(MAX_HISTORY);
    for (let i = 0; i < MAX_HISTORY; i++) {
      this._hist[i] = {
        t: 0, ax: 0, ay: 0, az: 0,
        p: [0, 0, 0], v: [0, 0, 0],
        P: [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
      };
    }
    this._head = 0;   // index of the OLDEST entry
    this._count = 0;  // entries in use
    this._lastT = 0;
  }

  reset() {
    this.started = false;
    this.nMeas = 0;
    this.nRejected = 0;
    this.nClamped = 0;
    this.lastMeasT = 0;
    for (let i = 0; i < 3; i++) {
      this.p[i] = 0;
      this.v[i] = 0;
      this.P[i][0] = 0.0225;
      this.P[i][1] = 0;
      this.P[i][2] = 0.04;
      this.lastMeas[i] = 0;
      this.lastMeasSigma[i] = 0;
    }
    this._rejStreak = 0;
    this._head = 0;
    this._count = 0;
    this._lastT = 0;
  }

  /**
   * Advance the state to `tMs` (epoch ms) under the IMU corrected accel
   * (camera frame!). Called once per IMU sample, same cadence as the
   * Stage B hand filter.
   */
  predict(tMs, ax, ay, az) {
    if (!this.started) {
      this.started = true;
      this._lastT = tMs;
      this._push(tMs, ax, ay, az);
      return;
    }
    let dt = (tMs - this._lastT) / 1000;
    if (dt <= 0) return;                 // reordered stamp — nothing to do
    if (dt > 0.05) dt = 0.05;            // same clamp as the hand filter
    this._step(dt, ax, ay, az);
    this._lastT = tMs;
    this._push(tMs, ax, ay, az);
  }

  /**
   * Apply a camera measurement dated `tMs` (epoch ms), possibly in the past
   * relative to the newest IMU sample (pipeline latency) — handled by the
   * rewind + replay described above. `sx/sy/sz` are the per-axis 1σ
   * measurement noises (m). Returns true when the measurement passed the
   * outlier gate and was applied.
   */
  fuse(tMs, mx, my, mz, sx, sy, sz) {
    const P = this.P;

    if (!this.started) {
      // Seed directly from the first measurement (no IMU yet). Position is
      // taken as-is; the covariance keeps honest room (≥ 15 cm) so the very
      // next gated measurements can still pull the state around.
      this.started = true;
      this.p[0] = mx; this.p[1] = my; this.p[2] = mz;
      this.v[0] = 0; this.v[1] = 0; this.v[2] = 0;
      for (let i = 0; i < 3; i++) {
        P[i][0] = Math.max([sx, sy, sz][i] * [sx, sy, sz][i], 0.0225);
        P[i][1] = 0;
        P[i][2] = 0.04;
      }
      this._lastT = tMs;
      this._push(tMs, 0, 0, 0);
      this._measStats(tMs, mx, my, mz, sx, sy, sz);
      return true;
    }

    // Clamp the measurement time into the buffered past.
    const newestIdx = (this._head + this._count - 1) % MAX_HISTORY;
    const oldest = this._hist[this._head];
    if (tMs > this._lastT) tMs = this._lastT;   // no phantom pre-predict
    let j = -1;                                  // newest entry with t <= tMs
    if (tMs >= this._hist[newestIdx].t) {
      j = newestIdx;
    } else {
      for (let s = this._count - 1; s >= 0; s--) {
        const idx = (this._head + s) % MAX_HISTORY;
        if (this._hist[idx].t <= tMs) { j = idx; break; }
      }
      if (j < 0) { j = this._head; this.nClamped++; } // predates the buffer
    }
    const entry = this._hist[j];

    // Rewind: restore the checkpointed state.
    for (let i = 0; i < 3; i++) {
      this.p[i] = entry.p[i];
      this.v[i] = entry.v[i];
      P[i][0] = entry.P[i][0];
      P[i][1] = entry.P[i][1];
      P[i][2] = entry.P[i][2];
    }

    // Partial predict over [entry.t, tMs] with the entry's pending accel.
    const dtP = Math.max(0, (tMs - entry.t) / 1000);
    if (dtP > 0) this._step(dtP, entry.ax, entry.ay, entry.az);

    // 3-D outlier gate on the innovation. Two bypasses keep the filter from
    // dead-locking:
    //  • warm-up (first few accepted measurements): the seed sits at the home
    //    pose while the hand may already be elsewhere — force-accept and
    //    widen P00 so the state is pulled to the camera fast.
    //  • re-lock (long rejection streak, e.g. marker swapped/re-seated or a
    //    bad detection got locked in): force-accept the next measurement the
    //    same way and re-acquire.
    // In both modes the velocity gain is ZEROED — position snaps to the
    // camera, velocity stays with the IMU (a position correction must not
    // invent velocity; the cross-covariance is uninformative at lock-on).
    const warmup = this.nMeas < 5;
    const relock = this._rejStreak >= 12;
    const m = [mx, my, mz];
    const s2 = [sx * sx, sy * sy, sz * sz];
    let d2 = 0, thr = 0;
    if (warmup || relock) {
      for (let i = 0; i < 3; i++) {
        const widened = 25 * s2[i];
        if (widened > P[i][0]) P[i][0] = widened;
      }
    } else {
      for (let i = 0; i < 3; i++) {
        const inn = m[i] - this.p[i];
        d2 += inn * inn;
        thr += (this.cfg.gate * this.cfg.gate) * (P[i][0] + s2[i]);
      }
      if (d2 > thr) {
        this.nRejected++;
        this._rejStreak++;
        this._replayFrom(j + 1, tMs);   // undo the rewind, keep the timeline
        return false;
      }
    }

    // KF update per axis (H = [1,0]).
    const posOnly = warmup || relock;   // no velocity injection
    this._rejStreak = 0;
    for (let i = 0; i < 3; i++) {
      const R = s2[i];
      const P00 = P[i][0], P01 = P[i][1], P11 = P[i][2];
      const denom = P00 + R;
      const K0 = P00 / denom;
      const K1 = posOnly ? 0 : P01 / denom;
      const inn = m[i] - this.p[i];
      this.p[i] += K0 * inn;
      this.v[i] += K1 * inn;
      // P = (I − K H) P  →  [[1−K0, 0],[−K1, 1]]·[[P00,P01],[P10,P11]]
      const n00 = (1 - K0) * P00;
      const n01 = (1 - K0) * P01;
      const n10 = P01 - K1 * P00;
      const n11 = P11 - K1 * P01;
      P[i][0] = n00;
      P[i][1] = 0.5 * (n01 + n10);    // keep symmetric
      P[i][2] = n11;
    }

    this._measStats(tMs, mx, my, mz, sx, sy, sz);
    // Replay the stored IMU inputs after the measurement point.
    this._replayFrom(j + 1, tMs);
    return true;
  }

  /**
   * Re-integrate every stored input after entry index `start` (from tMs).
   * The recomputed states are written back into the ring, so a SECOND late
   * measurement arriving in the same batch rewinds to the already-corrected
   * timeline instead of losing the first correction.
   */
  _replayFrom(start, tMs) {
    if (start >= this._count) return;
    for (let s = start; s < this._count; s++) {
      const k = (this._head + s) % MAX_HISTORY;
      const e = this._hist[k];
      const dt = Math.max(0, (e.t - tMs) / 1000);
      if (dt > 0) this._step(dt, e.ax, e.ay, e.az);
      tMs = e.t;
      for (let i = 0; i < 3; i++) {
        e.p[i] = this.p[i];
        e.v[i] = this.v[i];
        e.P[i][0] = this.P[i][0];
        e.P[i][1] = this.P[i][1];
        e.P[i][2] = this.P[i][2];
      }
    }
    // State now equals the (corrected) forward timeline state again — the
    // same ops in the same order make the replay deterministic. _lastT and
    // the pending accel of the newest entry were never touched.
  }

  /** One predict step (mutates p/v/P). */
  _step(dt, ax, ay, az) {
    const P = this.P;
    const a = [ax, ay, az];
    const sA2 = this.cfg.sigmaA * this.cfg.sigmaA;
    const dt2 = dt * dt;
    for (let i = 0; i < 3; i++) {
      const u = a[i];
      this.p[i] += this.v[i] * dt + 0.5 * u * dt2;
      this.v[i] += u * dt;
      const P00 = P[i][0], P01 = P[i][1], P11 = P[i][2];
      P[i][0] = P00 + 2 * dt * P01 + dt2 * P11 + sA2 * dt2 * dt2 * 0.25;
      P[i][1] = P01 + dt * P11 + sA2 * dt2 * dt * 0.5;
      P[i][2] = P11 + sA2 * dt2;
    }
  }

  _push(tMs, ax, ay, az) {
    let idx;
    if (this._count < MAX_HISTORY) {
      idx = (this._head + this._count) % MAX_HISTORY;
      this._count++;
    } else {
      idx = this._head;             // overwrite the oldest
      this._head = (this._head + 1) % MAX_HISTORY;
    }
    const e = this._hist[idx];
    e.t = tMs;
    e.ax = ax; e.ay = ay; e.az = az;
    for (let i = 0; i < 3; i++) {
      e.p[i] = this.p[i];
      e.v[i] = this.v[i];
      e.P[i][0] = this.P[i][0];
      e.P[i][1] = this.P[i][1];
      e.P[i][2] = this.P[i][2];
    }
  }

  _measStats(tMs, mx, my, mz, sx, sy, sz) {
    this.nMeas++;
    this.lastMeasT = tMs;
    this.lastMeas[0] = mx; this.lastMeas[1] = my; this.lastMeas[2] = mz;
    this.lastMeasSigma[0] = sx; this.lastMeasSigma[1] = sy; this.lastMeasSigma[2] = sz;
  }

  /** Copy the estimate into a reusable {p, v} pair of 3-arrays. */
  read(out) {
    out.p[0] = this.p[0]; out.p[1] = this.p[1]; out.p[2] = this.p[2];
    out.v[0] = this.v[0]; out.v[1] = this.v[1]; out.v[2] = this.v[2];
    return out;
  }

  /** Debug/E2E snapshot (small allocation — never called per frame). */
  debug() {
    return {
      started: this.started,
      p: this.p.slice(),
      v: this.v.slice(),
      P00: [this.P[0][0], this.P[1][0], this.P[2][0]],
      hist: this._count,
      nMeas: this.nMeas,
      nRejected: this.nRejected,
      nClamped: this.nClamped,
      lastMeasT: this.lastMeasT,
      lastMeas: this.lastMeas.slice(),
    };
  }
}
