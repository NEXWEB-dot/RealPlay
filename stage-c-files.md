# Stage C — every changed file in full

Complete current content of every file changed for Stage C
("Fusion Tracking": webcam ArUco marker tracking fused with the phone IMU),
as of commit `207653f`.

What changed, per file:

1. **`client/core/tracker/fusionKF.js`** — NEW: the `HandFusionFilter`
   Kalman filter. Per axis [pos, vel], control input = Stage B corrected
   accel; 96-entry checkpoint ring with rewind + replay for late camera
   measurements (bit-exact, states written back into the ring); 3-D outlier
   gate; warm-up / re-lock bypass with widened P00 and zeroed velocity gain;
   seed path from the first measurement.
2. **`client/core/tracker/cameraTracker.js`** — NEW: the CameraTracker.
   rVFC capture (setInterval fallback), one detection in flight,
   OffscreenCanvas → transferToImageBitmap → worker; fov-free pinhole ratio
   math; size EMA; ready-pose home capture (normalized {cxn, cyn, sideN},
   persisted); home subtraction + depth flip → calibrated-frame measurements
   {t, x, y, z, sx, sy, sz}; per-slot subscriptions; status machine
   off|wait|nocal|track|stale|lost; mirrored preview mount; debug/E2E hooks.
3. **`client/core/tracker/detectWorker.js`** — NEW: module worker running
   the vendored detector on a reused OffscreenCanvas; posts per-marker
   {id, cx, cy, side, ham}; never throws on a bad frame.
4. **`client/core/tracker/markerGen.js`** — NEW: SVG + raster generation
   from the same vendored dictionary (print == detect); black-square
   convention documented (markSize 6 of 8 units).
5. **`markers.html`** — NEW: printable marker sheet (IDs 0/1), 100 mm
   verification ruler, print checklist; mm input shared with settings.
6. **`tests/fusion.test.mjs`** — NEW: 16 fusion unit tests (convergence,
   mover tracking, bit-exact replay, outlier rejection, stale clamping,
   seeding) — `npm run test:fusion`.
7. **`tests/detector.test.mjs`** — NEW: synthetic 640×480 frame with both
   markers → exact centers/sides through the vendored detector.
8. **`tests/fakePhone.mjs`** — NEW: fake phone controller for browser E2E
   (WS ctrl_hello + real 56-byte MOTION frames, CALIBRATED first 25 packets).
9. **`client/core/tracker/vendor/cv.js`** — NEW, vendored js-aruco2@2.0.0
   (MIT) as ESM: OpenCV-style primitives used by the detector.
10. **`client/core/tracker/vendor/aruco.js`** — NEW, vendored: the AR
    detector + Dictionary (incl. generateSVG).
11. **`client/core/tracker/vendor/aruco_4x4_1000.js`** — NEW, vendored: the
    ARUCO_4X4_1000 dictionary (1000 words, markSize 6, tau 2).
12. **`client/core/tracker/vendor/LICENSE-js-aruco2.txt`** — NEW: vendored
    library license (MIT).
13. **`client/core/handMotion.js`** — `adopt(px..vz)`: the Stage B filter
    can adopt an external (fusion) position/velocity estimate and continue
    integrating from there instead of gliding home.
14. **`client/core/input.js`** — `RacketInput.hand.fused`;
    `ControllerInputSource(slot, net, tracker, settings)`: subscribes to the
    tracker's per-slot measurements (`fuseCameraMeas → fusion.fuse`), runs
    `fusion.predict(nowMs, hand.accel)` per sample, replaces hand.pos/vel
    with the fused state when fresh (< 400 ms) and trackMode is 'fusion',
    hands the last fused state to the hand filter on dropout, resets both
    filters on recalibration, `dispose()` unsubscribes.
15. **`client/core/settings.js`** — defaults `trackMode ['imu','imu']`,
    `camDevice ''`, `camMarkerMm 100`, `camHome [null, null]`.
16. **`client/main.js`** — tracker singleton; refreshSlots passes
    (slot, net, tracker, settings) and disposes replaced sources; getHandInfo
    shows the fused state; getTrackInfo (F3 TRACK line); screens ctx gets the
    tracker; `__RP.tracker` debug hooks.
17. **`client/sports/tableTennis/TableTennisSport.js`** — debugHandInfo
    exposes `fused`, `trackMode` and the fusion KF snapshot.
18. **`client/ui/debugOverlay.js`** — amber HAND line when fused
    (HAND[mode+fusion]) + per-slot TRACK line (status/age/cam fps/home).
19. **`client/ui/screens.js`** — per-player tracking selects; "Tracking &
    camera…" button; full Tracking setup screen (mirrored preview + overlay,
    device select, marker mm, per-slot capture/clear, status chips, camera
    keeps running after close).
20. **`package.json`** — js-aruco2 dependency (sources vendored in-tree) +
    `test:fusion` script.
21. **`package-lock.json`** — js-aruco2 entry (machine-generated).
22. **`vite.config.js`** — markers.html as a third MPA build input.
23. **`.gitignore`** — `.tmp/` (scratch logs).

Stage C summary + verification log: `summary.md` (Stage C section).
Test instructions are in the chat delivery message.

---

# File contents (exact bytes as of commit 207653f)

## 1. `client/core/tracker/fusionKF.js`

```js
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
```

## 2. `client/core/tracker/cameraTracker.js`

```js
/**
 * CameraTracker (Stage C) — webcam ArUco marker tracking.
 *
 * One webcam watches the player(s). Each player carries a printed marker
 * (ID 0 = P1, ID 1 = P2) on the phone/hand so it always faces the webcam.
 * The tracker:
 *
 *   1. captures video frames (requestVideoFrameCallback; setInterval
 *      fallback) into a reusable OffscreenCanvas and ships zero-copy
 *      ImageBitmaps to detectWorker.js (one detection in flight at a time —
 *      the worker's runtime sets the detection rate, the camera never queues);
 *   2. converts detections into CAMERA-frame meters with fov-free pinhole
 *      ratio math (see below), smoothed by a size EMA for stable depth;
 *   3. subtracts the per-slot READY-POSE home and flips the depth sign,
 *      producing measurements in the CALIBRATED player frame (+x = player's
 *      left, +y = up, +z = toward the net — the same frame the Stage B hand
 *      filter lives in);
 *   4. dispatches {t, x, y, z, sx, sy, sz} to per-slot subscribers (the
 *      controller input source feeds them to its fusion KF).
 *
 * FOV-FREE PINHOLE MATH — the reason there is no FOV setting:
 *   a marker of physical side S at depth Z projects to `side_px = f·S/Z`
 *   pixels, and its center sits at `(cx − w/2) = f·X/Z` from the optical
 *   axis. Dividing the two kills f entirely:
 *
 *     X = S·(cx/w − ½)/(side_px/w)      Z = S/(side_px/w)
 *
 *   so position follows from normalized pixel ratios and the printed size
 *   alone — no focal length, no resolution, no FOV calibration. (What FOV
 *   cannot buy us anyway is lens-distortion correction; keep the marker
 *   near the image center.)
 *
 * Known v1 limitation: a marker tilted θ out of the image plane shrinks by
 * ~cos θ, biasing depth (Z) high by ~θ²/2 (≈ 7% at 30°) and leaving lateral
 * x/y nearly unbiased (the quad centroid stays centered to 2nd order). The
 * fusion KF's depth sigma and the IMU absorb the swing-time transients.
 *
 * Home capture (`captureHome`) stores the marker's NORMALIZED image pose
 * {cxn, cyn, sideN} at the ready pose — resolution- and fov-independent, so
 * it survives settings changes and page reloads (persisted in settings).
 *
 * Debug/E2E hooks (never used by gameplay):
 *   debugInjectFrame(canvas, tMs)   — push a synthetic frame through the
 *                                     real worker detection path;
 *   debugInjectMeasure(slot, d, t)  — push a normalized detection straight
 *                                     into the processing/dispatch path.
 */

const MARKER_MIN_PX = 10;      // smaller quads are noise
const SIZE_EMA_TAU = 120;      // ms — marker-size smoothing for stable depth
const TRACK_FRESH_MS = 150;    // < this since the last fix -> 'track'
const TRACK_STALE_MS = 400;    // < this -> 'stale', else 'lost'
const HOME_TIMEOUT_MS = 4000;  // captureHome give-up

export class CameraTracker {
  /**
   * @param {object} settings the Settings singleton (camDevice, camMarkerMm,
   *                          camHome are read/persisted here)
   */
  constructor(settings) {
    this.settings = settings;

    // The video element lives permanently in the DOM (hidden) so decoding
    // and rVFC keep running during matches; the setup screen temporarily
    // re-parents it into its preview box.
    this.video = document.createElement('video');
    this.video.playsInline = true;
    this.video.muted = true;
    this.video.autoplay = true;
    this.video.style.cssText =
      'width:100%;height:100%;object-fit:cover;display:block;background:#000;' +
      'transform:scaleX(-1);'; // mirrored PREVIEW only — the captured frame is not
    this._hideHome = document.createElement('div');
    this._hideHome.style.cssText =
      'position:fixed;left:0;bottom:0;width:2px;height:2px;opacity:0.01;pointer-events:none;overflow:hidden;';
    this._hideHome.appendChild(this.video);
    document.body.appendChild(this._hideHome);

    this.canvas = new OffscreenCanvas(640, 480);
    this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });

    this.worker = null;
    this.stream = null;
    this.running = false;
    this.err = null;
    this.fps = 0;      // detection rounds per second
    this.capW = 0;
    this.capH = 0;

    // Per slot-1 state (preallocated — the detect callback is a hot path).
    this.slots = [this._newSlotState(), this._newSlotState()];
    // Restore persisted homes.
    const homes = settings.get('camHome');
    if (Array.isArray(homes)) {
      for (let i = 0; i < 2; i++) {
        const h = homes[i];
        if (h && isFinite(h.cxn) && isFinite(h.cyn) && isFinite(h.sideN)) {
          this.slots[i].home = h;
        }
      }
    }

    this.pendingHome = [false, false];
    this.homeWaiters = [null, null];
    this.subs = [null, null]; // { meas(m), home(), reset() }

    this._inflight = false;
    this._timer = 0;          // setInterval fallback handle
    this._raf = 0;
    this._rounds = 0;
    this._fpsT = 0;
    // Latest raw detections in normalized coords (for the preview overlay).
    this.lastMarkers = [];    // [{ id, cxn, cyn, sideN, t }] — replaced, not grown
    this.lastMarkersT = 0;
  }

  _newSlotState() {
    return {
      home: null,            // { cxn, cyn, sideN } ready pose in the image
      lastT: 0,              // epoch ms of the last detection of this marker
      nMeas: 0,
      sEma: 0,               // smoothed normalized side
      meas: { t: 0, x: 0, y: 0, z: 0, sx: 0, sy: 0, sz: 0 }, // reused
      homeM: { x: 0, y: 0, z: 0 },                            // reused
    };
  }

  /* ---------------------------------------------------------------- */
  /* Camera lifecycle                                                  */
  /* ---------------------------------------------------------------- */

  /** Open the camera (settings.camDevice or default) and start detecting. */
  async start() {
    if (this.running) return;
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      this.err = 'camera API unavailable (needs HTTPS or localhost)';
      throw new Error(this.err);
    }
    const dev = this.settings.get('camDevice') || undefined;
    this.err = null;
    let stream;
    try {
      stream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: dev
          ? { deviceId: { exact: dev }, width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 } }
          : { facingMode: 'user', width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 } },
      });
    } catch (e) {
      this.err = e && e.name === 'NotAllowedError' ? 'camera permission denied'
        : e && e.name === 'NotFoundError' ? 'no camera found'
          : (e && e.message) || String(e);
      throw new Error(this.err);
    }
    this.stream = stream;

    if (!this.worker) {
      this.worker = new Worker(new URL('./detectWorker.js', import.meta.url), { type: 'module' });
      this.worker.onmessage = (ev) => this._onDetect(ev.data);
    }

    this.video.srcObject = stream;
    try { await this.video.play(); } catch { /* autoplay policies — muted+inline is fine */ }

    this.capW = this.video.videoWidth || 640;
    this.capH = this.video.videoHeight || 480;
    this.canvas.width = this.capW;
    this.canvas.height = this.capH;

    this.running = true;
    this._inflight = false;
    this._rounds = 0;
    this._fpsT = Date.now();
    this._schedule();
  }

  stop() {
    this.running = false;
    if (this._timer) { clearInterval(this._timer); this._timer = 0; }
    this._raf = 0;
    if (this.stream) {
      for (const t of this.stream.getTracks()) t.stop();
      this.stream = null;
    }
    this.video.srcObject = null;
    this._inflight = false;
  }

  /** List video input devices (labels available once permission was granted). */
  async listDevices() {
    if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) return [];
    const devs = await navigator.mediaDevices.enumerateDevices();
    const out = [];
    for (const d of devs) {
      if (d.kind !== 'videoinput') continue;
      out.push({ id: d.deviceId, label: d.label || `Camera ${out.length + 1}` });
    }
    return out;
  }

  /* ---------------------------------------------------------------- */
  /* Capture loop                                                      */
  /* ---------------------------------------------------------------- */

  _schedule() {
    if (!this.running) return;
    const v = this.video;
    if (v.requestVideoFrameCallback) {
      this._raf = v.requestVideoFrameCallback(() => this._capture());
    } else {
      // Fallback: fixed 30 Hz pump.
      if (!this._timer) this._timer = setInterval(() => this._capture(), 33);
    }
  }

  _capture() {
    if (!this.running) return;
    this._raf = 0;
    if (this.video.videoWidth) {
      this.capW = this.video.videoWidth;
      this.capH = this.video.videoHeight;
      if (this.canvas.width !== this.capW) this.canvas.width = this.capW;
      if (this.canvas.height !== this.capH) this.canvas.height = this.capH;
      // One detection in flight at a time — skip frames while the worker is
      // busy so latency can never build a queue.
      if (!this._inflight) {
        this._inflight = true;
        this.ctx.drawImage(this.video, 0, 0, this.capW, this.capH);
        const bmp = this.canvas.transferToImageBitmap();
        // Wall-clock stamp at callback time; the frame content is a little
        // older (pipeline latency) and the fusion KF rewinds for it.
        this.worker.postMessage({ bmp, t: Date.now(), w: this.capW, h: this.capH }, [bmp]);
      }
    }
    this._schedule();
  }

  _onDetect(r) {
    this._inflight = false;
    const { t, w, h, markers } = r;
    if (this.running) {
      this._rounds++;
      const now = performance.now();
      if (now - this._fpsT >= 500) {
        this.fps = this._rounds * 1000 / (now - this._fpsT);
        this._rounds = 0;
        this._fpsT = now;
      }
    }
    this.lastMarkersT = t;

    // Rebuild the (small, bounded) latest-marker list for the overlay.
    // Processing runs even when the camera is off so the E2E frame
    // injection path exercises the same code as live capture.
    const list = [];
    for (let i = 0; i < markers.length; i++) {
      const m = markers[i];
      const slot = m.id + 1;                 // ID 0 -> P1, ID 1 -> P2
      if (slot < 1 || slot > 2) continue;
      list.push({ id: m.id, cxn: m.cx / w, cyn: m.cy / h, sideN: m.side / w, t });
      this._process(slot, m, t, w, h);
    }
    this.lastMarkers = list;
  }

  /* ---------------------------------------------------------------- */
  /* Detections -> calibrated-frame measurements                       */
  /* ---------------------------------------------------------------- */

  _process(slot, m, t, w, h) {
    if (m.side < MARKER_MIN_PX || !isFinite(m.cx) || !isFinite(m.cy)) return;

    const st = this.slots[slot - 1];
    const S = this.markerSizeM();

    // Size EMA — depth stability against edge jitter.
    const sN = m.side / w;
    const dt = st.lastT ? Math.min(200, Math.max(1, t - st.lastT)) : SIZE_EMA_TAU;
    st.sEma = st.sEma ? st.sEma + (sN - st.sEma) * (1 - Math.exp(-dt / SIZE_EMA_TAU)) : sN;

    // Camera-frame meters (fov-free ratios; y normalized by the frame height).
    const inv = S / st.sEma;
    const x = (m.cx / w - 0.5) * inv;   // image right
    const y = (0.5 - m.cy / h) * inv;   // image up
    const z = inv;                       // depth away from webcam

    // Pending ready-pose capture?
    if (this.pendingHome[slot - 1]) {
      this.pendingHome[slot - 1] = false;
      st.home = { cxn: m.cx / w, cyn: m.cy / h, sideN: sN };
      this.settings.setAt('camHome', slot - 1, st.home);
      const sub = this.subs[slot - 1];
      if (sub && sub.home) sub.home();
      if (sub && sub.reset) sub.reset(); // origin moved: fusion re-locks
      const waiter = this.homeWaiters[slot - 1];
      if (waiter) {
        this.homeWaiters[slot - 1] = null;
        waiter(true);
      }
    }

    st.lastT = t;
    st.nMeas++;
    if (!st.home) return;                 // tracking, but not calibrated yet

    // Home subtraction -> calibrated player frame (+z flipped: toward the
    // webcam = toward the net). Measurement noises scale with depth.
    this._homeMeters(st, S);
    const meas = st.meas;
    meas.t = t;
    meas.x = x - st.homeM.x;
    meas.y = y - st.homeM.y;
    meas.z = -(z - st.homeM.z);
    const latNoise = Math.max(0.005, (2.5 / w) * S / st.sEma);  // ~2.5 px of jitter
    meas.sx = latNoise;
    meas.sy = latNoise;
    meas.sz = Math.max(0.025, 0.035 * z);                      // ~3.5 % depth
    const sub = this.subs[slot - 1];
    if (sub && sub.meas) sub.meas(meas);
  }

  markerSizeM() {
    return Math.max(0.02, (this.settings.get('camMarkerMm') || 100) / 1000);
  }

  /** Camera-frame meters of the stored (normalized) home — fov-free. */
  _homeMeters(st, S) {
    const h = st.home;
    st.homeM.x = S * (h.cxn - 0.5) / h.sideN;
    st.homeM.y = -S * (h.cyn - 0.5) / h.sideN;
    st.homeM.z = S / h.sideN;
  }

  /* ---------------------------------------------------------------- */
  /* Homes                                                             */
  /* ---------------------------------------------------------------- */

  /**
   * Capture the ready-pose home for `slot` from the next valid detection of
   * its marker. Resolves true on capture, false after HOME_TIMEOUT_MS.
   */
  captureHome(slot) {
    return new Promise((resolve) => {
      this.pendingHome[slot - 1] = true;
      this.homeWaiters[slot - 1] = resolve;
      setTimeout(() => {
        if (this.homeWaiters[slot - 1] === resolve) {
          this.homeWaiters[slot - 1] = null;
          this.pendingHome[slot - 1] = false;
          resolve(false);
        }
      }, HOME_TIMEOUT_MS);
    });
  }

  clearHome(slot) {
    const st = this.slots[slot - 1];
    st.home = null;
    this.settings.setAt('camHome', slot - 1, null);
    const sub = this.subs[slot - 1];
    if (sub && sub.reset) sub.reset();
  }

  hasHome(slot) {
    return !!this.slots[slot - 1].home;
  }

  /* ---------------------------------------------------------------- */
  /* Subscriptions + status                                            */
  /* ---------------------------------------------------------------- */

  /**
   * @param {number} slot 1|2
   * @param {object} sub { meas(m), home(), reset() } — `meas` receives the
   *   reused per-slot measurement object in the CALIBRATED frame; consume it
   *   synchronously.
   */
  subscribe(slot, sub) {
    this.subs[slot - 1] = sub;
  }

  unsubscribe(slot, sub) {
    if (this.subs[slot - 1] === sub) this.subs[slot - 1] = null;
  }

  /** 'off' | 'wait' | 'nocal' | 'track' | 'stale' | 'lost' */
  status(slot) {
    if (!this.running) return 'off';
    const st = this.slots[slot - 1];
    if (!st.lastT) return 'wait';
    const age = Date.now() - st.lastT;
    if (age > TRACK_STALE_MS) return 'lost';
    if (!st.home) return 'nocal';
    return age > TRACK_FRESH_MS ? 'stale' : 'track';
  }

  /** Epoch-ms age of the slot's last detection (Infinity when never seen). */
  lastMeasAgeMs(slot) {
    const st = this.slots[slot - 1];
    return st.lastT ? Date.now() - st.lastT : Infinity;
  }

  /** Mount the video into a preview container (or back into the hidden home). */
  mountPreview(container) {
    (container || this._hideHome).appendChild(this.video);
  }

  /* ---------------------------------------------------------------- */
  /* Debug / E2E hooks                                                 */
  /* ---------------------------------------------------------------- */

  /**
   * Push a synthetic frame (canvas/image source with width/height) through
   * the REAL worker detection path, stamped `tMs` (epoch ms).
   */
  debugInjectFrame(source, tMs) {
    if (!this.worker) {
      this.worker = new Worker(new URL('./detectWorker.js', import.meta.url), { type: 'module' });
      this.worker.onmessage = (ev) => this._onDetect(ev.data);
    }
    const w = source.width, h = source.height;
    if (this.canvas.width !== w) this.canvas.width = w;
    if (this.canvas.height !== h) this.canvas.height = h;
    this._inflight = true;
    this.ctx.drawImage(source, 0, 0, w, h);
    const bmp = this.canvas.transferToImageBitmap();
    this.worker.postMessage({ bmp, t: tMs, w, h }, [bmp]);
  }

  /**
   * Push a normalized detection straight into the processing path (no
   * worker). `d` = { cxn, cyn, sideN }; marker id = slot-1; works with the
   * camera running or not (the math is fov/resolution-free).
   */
  debugInjectMeasure(slot, d, tMs) {
    const w = 640, h = 480;
    this._process(slot, { id: slot - 1, cx: d.cxn * w, cy: d.cyn * h, side: d.sideN * w, ham: 0 },
      tMs == null ? Date.now() : tMs, w, h);
  }

  /** Snapshot for overlays/screens (small allocations — diagnostics only). */
  debugState() {
    return {
      running: this.running,
      err: this.err,
      fps: Math.round(this.fps * 10) / 10,
      w: this.capW,
      h: this.capH,
      lastMarkers: this.lastMarkers,
      lastMarkersAgeMs: this.lastMarkersT ? Date.now() - this.lastMarkersT : Infinity,
      statuses: [this.status(1), this.status(2)],
      hasHome: [this.hasHome(1), this.hasHome(2)],
    };
  }
}
```

## 3. `client/core/tracker/detectWorker.js`

```js
/**
 * Marker detection worker (Stage C).
 *
 * Module worker: receives an ImageBitmap per video frame, draws it to a
 * reused OffscreenCanvas, runs the vendored js-aruco2 detector (ARUCO_4X4_1000
 * dictionary, marker IDs 0/1) and posts back a compact per-marker summary:
 *
 *   { t, w, h, markers: [{ id, cx, cy, side, ham }] }
 *
 * cx/cy = marker center in pixels, side = mean edge length in pixels
 * (the OUTER black square), ham = dictionary hamming distance. All pose
 * math happens on the main thread (cameraTracker) — the worker stays dumb.
 */

import { AR } from './vendor/aruco.js';
import './vendor/aruco_4x4_1000.js'; // registers ARUCO_4X4_1000 in AR.DICTIONARIES

const DICTIONARY = 'ARUCO_4X4_1000';

let detector = null;
let canvas = null;
let ctx = null;

self.onmessage = (ev) => {
  const { bmp, t, w, h } = ev.data;
  if (!bmp) return;

  try {
    if (!detector) detector = new AR.Detector({ dictionaryName: DICTIONARY });
    if (!canvas) {
      canvas = new OffscreenCanvas(w, h);
      ctx = canvas.getContext('2d', { willReadFrequently: true });
    }
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }

    ctx.drawImage(bmp, 0, 0, w, h);
  } finally {
    if (bmp.close) bmp.close(); // release the transferred bitmap's memory
  }

  let found;
  try {
    found = detector.detect(ctx.getImageData(0, 0, w, h));
  } catch {
    found = []; // never let a bad frame kill the worker
  }

  const out = [];
  for (let i = 0; i < found.length; i++) {
    const m = found[i];
    let cx = 0, cy = 0, side = 0;
    for (let k = 0; k < 4; k++) {
      cx += m.corners[k].x;
      cy += m.corners[k].y;
      const a = m.corners[k], b = m.corners[(k + 1) & 3];
      const dx = b.x - a.x, dy = b.y - a.y;
      side += Math.sqrt(dx * dx + dy * dy);
    }
    out.push({ id: m.id, cx: cx / 4, cy: cy / 4, side: side / 4, ham: m.hammingDistance });
  }

  self.postMessage({ t, w, h, markers: out });
};
```

## 4. `client/core/tracker/markerGen.js`

```js
/**
 * ArUco marker generation (Stage C) — the DRAW side of the tracker.
 *
 * The detector (detectWorker) and this generator share the same vendored
 * dictionary (ARUCO_4X4_1000, 4×4 bits), so what we print/rasterize is
 * bit-identical to what the detector reads back.
 *
 * Geometry convention (same as js-aruco2's Dictionary.generateSVG):
 *   the marker is markSize(=6) units of BLACK square (1-unit black border +
 *   4×4 data bits), surrounded by a 1-unit WHITE quiet zone on each side,
 *   i.e. 8×8 units total. The DETECTED quad is the OUTER BLACK SQUARE —
 *   that is the "marker side" every consumer talks about:
 *
 *     black side   = markSize / (markSize + 4)   of the full tile  (6/8)
 *     quiet zone   = 1 unit on all four sides
 *
 *   All sizing below therefore treats `side` as the black-square side.
 */

import { AR } from './vendor/aruco.js';
import './vendor/aruco_4x4_1000.js';

const dict = new AR.Dictionary('ARUCO_4X4_1000');
export const MARK_SIZE = dict.markSize;        // 6 — black square, in units
export const UNITS_TOTAL = dict.markSize + 2;  // 8 — incl. 1-unit quiet zone

/**
 * SVG string for marker `id` with the BLACK square side = sideMm
 * (print-ready; unit suffix included). IDs 0 and 1 are the game markers
 * (P1/P2); the dictionary holds 1000.
 */
export function markerSVG(id, sideMm) {
  const total = (sideMm * UNITS_TOTAL) / MARK_SIZE;
  return dict.generateSVG(id).replace(
    '<svg ',
    `<svg width="${total.toFixed(2)}mm" height="${total.toFixed(2)}mm" `,
  );
}

/**
 * Rasterize marker `id` onto a 2D canvas context. (x, y) = TOP-LEFT of the
 * BLACK square; its side = sidePx. The caller must ensure enough canvas
 * margin for the quiet zone (1 unit = sidePx/6). Returns the full tile rect
 * (including the quiet zone) in pixels.
 *
 * For clean detection pass an integer sidePx divisible by MARK_SIZE (no
 * anti-aliased unit edges) — e.g. 48, 60, 96.
 */
export function drawMarker(ctx, id, x, y, sidePx) {
  const u = sidePx / MARK_SIZE;
  const code = dict.codeList[id];
  if (code == null) throw new Error(`marker id ${id} is not in the dictionary`);

  // quiet zone (white) around the black square
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(x - u, y - u, sidePx + 2 * u, sidePx + 2 * u);

  // black square (border + data region)
  ctx.fillStyle = '#000000';
  ctx.fillRect(x, y, sidePx, sidePx);

  // data bits: '1' cells are WHITE, drawn inside the 1-unit black border
  ctx.fillStyle = '#ffffff';
  const size = MARK_SIZE - 2; // 4×4 data bits
  for (let r = 0; r < size; r++) {
    for (let c = 0; c < size; c++) {
      if (code[r * size + c] === '1') {
        ctx.fillRect(x + (c + 1) * u, y + (r + 1) * u, u, u);
      }
    }
  }
  return { x: x - u, y: y - u, w: sidePx + 2 * u, h: sidePx + 2 * u };
}
```

## 5. `markers.html`

```html
<!doctype html>
<!--
  REALPLAY Stage C — printable tracking markers.
  Prints marker ID 0 (P1) and ID 1 (P2) at the exact size configured in the
  game (Settings → Tracking & camera → marker print size, shared via
  localStorage), plus a 100 mm verification ruler. Print at 100 % scale
  ("actual size", no fit-to-page) — the tracker's depth math depends on the
  printed size matching camMarkerMm.
-->
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>REALPLAY — tracking markers</title>
<style>
  :root { color-scheme: light; }
  body { font-family: system-ui, sans-serif; background: #fff; color: #111; margin: 24px; }
  h1 { font-size: 20px; margin: 0 0 4px; }
  .hint { max-width: 78ch; font: 14px/1.55 system-ui; color: #333; margin: 8px 0 16px; }
  .sheet { display: flex; gap: 48px; flex-wrap: wrap; align-items: flex-start; }
  .marker { text-align: center; }
  .marker svg { display: block; margin: 0 auto; print-color-adjust: exact; -webkit-print-color-adjust: exact; }
  .label { font: 600 15px system-ui; margin-top: 10px; }
  .sub { font: 12px system-ui; color: #555; }
  .rulerbox { font: 12px system-ui; color: #333; }
  .ruler { width: 100mm; height: 16px; position: relative; margin: 14px 0 4px; }
  .ruler .bar { position: absolute; left: 0; right: 0; bottom: 0; height: 4mm;
    border-left: 1mm solid #000; border-right: 1mm solid #000; border-bottom: 1mm solid #000;
    print-color-adjust: exact; -webkit-print-color-adjust: exact; }
  .ruler .tick { position: absolute; bottom: 0; width: 1px; height: 3mm; background: #000;
    print-color-adjust: exact; -webkit-print-color-adjust: exact; }
  .ruler .tick.major { height: 4mm; width: 1mm; }
  .noprint button { font-size: 15px; padding: 8px 18px; margin-right: 10px; cursor: pointer; }
  .noprint label { font: 14px system-ui; }
  .noprint input { font-size: 15px; width: 70px; padding: 4px 6px; }
  @media print {
    .noprint { display: none !important; }
    body { margin: 10mm; }
    .sheet { gap: 20mm; }
  }
</style>
</head>
<body>
  <div class="noprint">
    <h1>REALPLAY tracking markers</h1>
    <div class="hint">
      Two players = two markers: <b>ID 0 → P1</b>, <b>ID 1 → P2</b> (ARUCO 4×4, 1000-word dictionary).
      Tape each marker flat on the <b>back of the phone</b> (screen side away from the webcam) or on the
      hand, so the webcam behind/next to your screen can see it during play.
    </div>
    <div style="margin: 10px 0 18px;">
      <button id="printBtn">Print</button>
      <label>Marker black-square size: <input id="mm" type="number" min="30" max="400" step="5"> mm</label>
      <span style="font:12px system-ui;color:#666;"> — same value as the game's Tracking &amp; camera screen</span>
    </div>
  </div>

  <div class="sheet">
    <div class="marker"><div id="m0"></div><div class="label">MARKER 0 — PLAYER 1</div></div>
    <div class="marker"><div id="m1"></div><div class="label">MARKER 1 — PLAYER 2</div></div>
    <div class="rulerbox">
      <div class="ruler" id="ruler"><div class="bar"></div></div>
      100 mm — verify with a real ruler after printing
    </div>
  </div>

  <div class="hint" style="margin-top:18px">
    <b>Print checklist</b>
    <ul style="margin:6px 0 0 18px; padding:0">
      <li>Print at <b>100 % / actual size</b> — never "fit to page" (that changes the size).</li>
      <li>Check the ruler below against a real ruler; if it's off, measure the printed black square
          and enter <i>that</i> size in the game's Tracking &amp; camera screen.</li>
      <li>Thick, matte, flat surface (no glare, no folds). The white 1-unit quiet zone around each
          black square is part of the marker — keep it clear.</li>
      <li>Lamp the play area evenly; avoid pointing a light straight at the marker.</li>
    </ul>
  </div>

  <script type="module">
    import { markerSVG } from '/client/core/tracker/markerGen.js';
    import { Settings } from '/client/core/settings.js';

    const settings = new Settings();
    const input = document.getElementById('mm');
    const draw = () => {
      const mm = Math.max(30, Math.min(400, parseFloat(input.value) || 100));
      document.getElementById('m0').innerHTML = markerSVG(0, mm);
      document.getElementById('m1').innerHTML = markerSVG(1, mm);
    };
    input.value = settings.get('camMarkerMm') || 100;
    input.addEventListener('change', () => {
      const v = Math.max(30, Math.min(400, parseFloat(input.value) || 100));
      input.value = v;
      settings.set('camMarkerMm', v);   // shared with the game's settings
      draw();
    });
    document.getElementById('printBtn').addEventListener('click', () => window.print());

    // verification ruler: ticks every 10 mm
    const ruler = document.getElementById('ruler');
    for (let i = 1; i < 10; i++) {
      const t = document.createElement('div');
      t.className = 'tick' + (i === 5 ? ' major' : '');
      t.style.left = i * 10 + 'mm';
      ruler.appendChild(t);
    }
    draw();
  </script>
</body>
</html>
```

## 6. `tests/fusion.test.mjs`

```js
/**
 * Fusion KF unit tests (Stage C) — `npm run test:fusion`.
 *
 * Runs in plain Node (the filter is DOM/THREE-free by design). Timestamps
 * use real epoch-ms magnitudes (~1.7e12) to mirror production conditions.
 */

import { HandFusionFilter } from '../client/core/tracker/fusionKF.js';

let pass = 0, fail = 0;
function ok(cond, name, detail = '') {
  if (cond) { pass++; console.log('  PASS ' + name); }
  else { fail++; console.log('  FAIL ' + name + (detail ? ' — ' + detail : '')); }
}
const near = (a, b, eps) => Math.abs(a - b) <= eps;

const T0 = 1794900000000; // epoch ms

/* 1. Convergence: silent IMU, static target. */
{
  const f = new HandFusionFilter();
  const tx = 10, ty = 5, tz = 20, sx = 0.01, sy = 0.01, sz = 0.05;
  for (let i = 0; i <= 60; i++) {          // 2 s of 30 Hz camera
    const t = T0 + i * 33.33;
    f.predict(t, 0, 0, 0);
    f.fuse(t, tx + sx * ((i % 3) - 1), ty, tz, sx, sy, sz);
  }
  const d = f.debug();
  ok(near(d.p[0], tx, 3 * sx) && near(d.p[1], ty, 3 * sy) && near(d.p[2], tz, 3 * sz),
    'static target converged', JSON.stringify(d.p));
  ok(Math.abs(d.v[0]) + Math.abs(d.v[1]) + Math.abs(d.v[2]) < 0.03,
    'velocity settled to ~0', JSON.stringify(d.v));
  ok(d.nMeas === 61 && d.nRejected === 0, 'all measurements accepted', `${d.nMeas}/${d.nRejected}`);
}

/* 2. Velocity tracking: silent IMU, target moving at 2 m/s. */
{
  const f = new HandFusionFilter();
  for (let i = 0; i <= 45; i++) {          // 1.5 s
    const t = T0 + i * 33.33;
    f.predict(t, 0, 0, 0);
    f.fuse(t, 2 * (i * 33.33) / 1000, 0, 0, 0.008, 0.008, 0.05);
  }
  const d = f.debug();
  ok(near(d.v[0], 2, 0.08), 'velocity tracks 2 m/s', `v=${d.v[0].toFixed(4)}`);
  ok(near(d.p[0], 3, 0.05), 'position tracks the mover', `p=${d.p[0].toFixed(4)}`);
}

/* 3. Delayed measurement + replay: exact determinism vs inline fusion. */
{
  const u = 2;                             // m/s² constant accel
  const truth = (dt) => 0.5 * u * dt * dt; // x(t) from rest
  // Run A: measurement fused inline exactly at t = 500 ms.
  const a = new HandFusionFilter();
  for (let i = 1; i <= 100; i++) {
    const t = T0 + i * 10;
    a.predict(t, u, 0, 0);
    if (i === 50) a.fuse(t, truth(0.5), 0, 0, 0.005, 0.005, 0.05);
  }
  // Run B: identical IMU stream, but the measurement arrives at t = 1000 ms
  // dated 500 ms (real webcam pipeline latency) — handled by replay.
  const b = new HandFusionFilter();
  for (let i = 1; i <= 100; i++) {
    const t = T0 + i * 10;
    b.predict(t, u, 0, 0);
    if (i === 100) b.fuse(t - 500, truth(0.5), 0, 0, 0.005, 0.005, 0.05);
  }
  const da = a.debug(), db = b.debug();
  ok(near(da.p[0], truth(1.0), 0.02) && near(db.p[0], truth(1.0), 0.02),
    'both runs end near ground truth', `inline=${da.p[0].toFixed(4)} replay=${db.p[0].toFixed(4)} truth=${truth(1.0)}`);
  ok(near(da.p[0], db.p[0], 1e-6) && near(da.v[0], db.v[0], 1e-6),
    'replay is bit-exact vs inline fusion', `dp=${Math.abs(da.p[0] - db.p[0]).toExponential(2)}`);
}

/* 4. Two back-to-back late measurements: the second must build on the
 *    first correction (write-back), not on the stale timeline. */
{
  const f = new HandFusionFilter();
  for (let i = 1; i <= 100; i++) f.predict(T0 + i * 10, 0, 0, 0); // silent IMU, drift-free
  // Two corrections for the same static position, both dated in the past.
  const r1 = f.fuse(T0 + 300, 1, 0, 0, 0.005, 0.005, 0.05);
  const r2 = f.fuse(T0 + 400, 1, 0, 0, 0.005, 0.005, 0.05);
  const d = f.debug();
  ok(r1 && r2, 'both late measurements accepted');
  ok(near(d.p[0], 1, 0.01) && near(d.p[1], 0, 0.01) && near(d.p[2], 0, 0.05),
    'state carries both corrections', JSON.stringify(d.p));
}

/* 5. Outlier gate: a wildly wrong measurement is rejected whole. */
{
  const f = new HandFusionFilter();
  for (let i = 0; i <= 60; i++) {
    const t = T0 + i * 33.33;
    f.predict(t, 0, 0, 0);
    f.fuse(t, 1, 1, 1, 0.01, 0.01, 0.05);
  }
  const before = f.debug().p.slice();
  const accepted = f.fuse(T0 + 2400, 5, 1, 1, 0.01, 0.01, 0.05); // 4 m off
  const d = f.debug();
  ok(!accepted && d.nRejected === 1, 'outlier rejected', `accepted=${accepted}`);
  ok(near(d.p[0], before[0], 1e-9), 'state unchanged by the outlier',
    `dp=${Math.abs(d.p[0] - before[0]).toExponential(2)}`);
}

/* 6. Measurement predating the buffer: clamped, no crash, applied. */
{
  const f = new HandFusionFilter();
  for (let i = 1; i <= 120; i++) f.predict(T0 + i * 10, 0, 0, 0); // 1.2 s @100 Hz > 96-slot ring
  const applied = f.fuse(T0, 0.5, 0, 0, 0.01, 0.01, 0.05);
  const d = f.debug();
  ok(applied && d.nClamped === 1, 'stale measurement clamped to buffer start', `applied=${applied}`);
  ok(near(d.p[0], 0.5, 0.05), 'clamped measurement still corrects', `p=${d.p[0].toFixed(3)}`);
}

/* 7. dt clamp: a 500 ms stream gap integrates at most 50 ms of motion. */
{
  const f = new HandFusionFilter();
  f.predict(T0, 2, 0, 0);
  f.predict(T0 + 500, 2, 0, 0);            // gap → dt clamped to 0.05
  f.predict(T0 + 510, 0, 0, 0);
  const d = f.debug();
  ok(d.v[0] < 0.11, 'gap clamp bounds integration', `v=${d.v[0].toFixed(4)}`);
}

/* 8. Camera measurement before any IMU: seeds the filter. */
{
  const f = new HandFusionFilter();
  const applied = f.fuse(T0, 0.2, 0.1, 1.5, 0.01, 0.01, 0.05);
  const d = f.debug();
  ok(applied && d.started && near(d.p[2], 1.5, 1e-9), 'first measurement seeds', JSON.stringify(d.p));
  f.predict(T0 + 16, 0, 0, 0);
  ok(f.debug().started, 'predict after seed stays consistent');
}

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
```

## 7. `tests/detector.test.mjs`

```js
/**
 * Node round-trip: synthesize the same 640×480 frame the browser E2E draws
 * (two ARUCO_4X4_1000 markers on a mid-grey background) and run the vendored
 * detector on it directly — isolates "bad synthetic image" vs "worker path".
 */
import { AR } from '../client/core/tracker/vendor/aruco.js';
import '../client/core/tracker/vendor/aruco_4x4_1000.js';

const W = 640, H = 480;
const data = new Uint8ClampedArray(W * H * 4);
for (let i = 0; i < W * H; i++) {
  data[i * 4] = 128; data[i * 4 + 1] = 128; data[i * 4 + 2] = 128; data[i * 4 + 3] = 255;
}

function drawMarker(code, x, y, side) {
  const u = side / 6;
  const px = (xx, yy, c) => {
    if (xx < 0 || yy < 0 || xx >= W || yy >= H) return;
    const i = (yy * W + xx) * 4;
    data[i] = c; data[i + 1] = c; data[i + 2] = c; data[i + 3] = 255;
  };
  for (let yy = y - u; yy < y + side + u; yy++)
    for (let xx = x - u; xx < x + side + u; xx++) px(xx, yy, 255); // quiet zone
  for (let yy = y; yy < y + side; yy++)
    for (let xx = x; xx < x + side; xx++) px(xx, yy, 0);            // black square
  for (let r = 0; r < 4; r++)
    for (let c = 0; c < 4; c++)
      if (code[r * 4 + c] === '1')
        for (let yy = y + (r + 1) * u; yy < y + (r + 2) * u; yy++)
          for (let xx = x + (c + 1) * u; xx < x + (c + 2) * u; xx++) px(xx, yy, 255);
}

drawMarker('1011010100110010', 200, 180, 60); // id 0
drawMarker('0000111110011010', 420, 240, 48); // id 1

const detector = new AR.Detector({ dictionaryName: 'ARUCO_4X4_1000' });
const markers = detector.detect({ width: W, height: H, data });
console.log('found', markers.length);
for (const m of markers) {
  const cx = m.corners.reduce((s, c) => s + c.x, 0) / 4;
  const cy = m.corners.reduce((s, c) => s + c.y, 0) / 4;
  console.log(`id=${m.id} center=(${cx.toFixed(1)},${cy.toFixed(1)}) ham=${m.hammingDistance}`);
}
```

## 8. `tests/fakePhone.mjs`

```js
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
```

## 9. `client/core/tracker/vendor/cv.js`

```js
/*
Copyright (c) 2011 Juan Mellado

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
References:
- "OpenCV: Open Computer Vision Library"
  http://sourceforge.net/projects/opencvlibrary/
- "Stack Blur: Fast But Goodlooking"
  http://incubator.quasimondo.com/processing/fast_blur_deluxe.php
*/

var CV = CV || {};

CV.Image = function(width, height, data){
  this.width = width || 0;
  this.height = height || 0;
  this.data = data || [];
};

CV.grayscale = function(imageSrc, imageDst){
  var src = imageSrc.data, dst = imageDst.data, len = src.length,
      i = 0, j = 0;

  for (; i < len; i += 4){
    dst[j ++] =
      (src[i] * 0.299 + src[i + 1] * 0.587 + src[i + 2] * 0.114 + 0.5) & 0xff;
  }

  imageDst.width = imageSrc.width;
  imageDst.height = imageSrc.height;

  return imageDst;
};

CV.threshold = function(imageSrc, imageDst, threshold){
  var src = imageSrc.data, dst = imageDst.data,
      len = src.length, tab = [], i;

  for (i = 0; i < 256; ++ i){
    tab[i] = i <= threshold? 0: 255;
  }

  for (i = 0; i < len; ++ i){
    dst[i] = tab[ src[i] ];
  }

  imageDst.width = imageSrc.width;
  imageDst.height = imageSrc.height;

  return imageDst;
};

CV.adaptiveThreshold = function(imageSrc, imageDst, kernelSize, threshold){
  var src = imageSrc.data, dst = imageDst.data, len = src.length, tab = [], i;

  CV.stackBoxBlur(imageSrc, imageDst, kernelSize);

  for (i = 0; i < 768; ++ i){
    tab[i] = (i - 255 <= -threshold)? 255: 0;
  }

  for (i = 0; i < len; ++ i){
    dst[i] = tab[ src[i] - dst[i] + 255 ];
  }

  imageDst.width = imageSrc.width;
  imageDst.height = imageSrc.height;

  return imageDst;
};

CV.otsu = function(imageSrc){
  var src = imageSrc.data, len = src.length, hist = [],
      threshold = 0, sum = 0, sumB = 0, wB = 0, wF = 0, max = 0,
      mu, between, i;

  for (i = 0; i < 256; ++ i){
    hist[i] = 0;
  }

  for (i = 0; i < len; ++ i){
    hist[ src[i] ] ++;
  }

  for (i = 0; i < 256; ++ i){
    sum += hist[i] * i;
  }

  for (i = 0; i < 256; ++ i){
    wB += hist[i];
    if (0 !== wB){

      wF = len - wB;
      if (0 === wF){
        break;
      }

      sumB += hist[i] * i;

      mu = (sumB / wB) - ( (sum - sumB) / wF );

      between = wB * wF * mu * mu;

      if (between > max){
        max = between;
        threshold = i;
      }
    }
  }

  return threshold;
};

CV.stackBoxBlurMult =
  [1, 171, 205, 293, 57, 373, 79, 137, 241, 27, 391, 357, 41, 19, 283, 265];

CV.stackBoxBlurShift =
  [0, 9, 10, 11, 9, 12, 10, 11, 12, 9, 13, 13, 10, 9, 13, 13];

CV.BlurStack = function(){
  this.color = 0;
  this.next = null;
};

CV.stackBoxBlur = function(imageSrc, imageDst, kernelSize){
  var src = imageSrc.data, dst = imageDst.data,
      height = imageSrc.height, width = imageSrc.width,
      heightMinus1 = height - 1, widthMinus1 = width - 1,
      size = kernelSize + kernelSize + 1, radius = kernelSize + 1,
      mult = CV.stackBoxBlurMult[kernelSize],
      shift = CV.stackBoxBlurShift[kernelSize],
      stack, stackStart, color, sum, pos, start, p, x, y, i;

  stack = stackStart = new CV.BlurStack();
  for (i = 1; i < size; ++ i){
    stack = stack.next = new CV.BlurStack();
  }
  stack.next = stackStart;

  pos = 0;

  for (y = 0; y < height; ++ y){
    start = pos;

    color = src[pos];
    sum = radius * color;

    stack = stackStart;
    for (i = 0; i < radius; ++ i){
      stack.color = color;
      stack = stack.next;
    }
    for (i = 1; i < radius; ++ i){
      stack.color = src[pos + i];
      sum += stack.color;
      stack = stack.next;
    }

    stack = stackStart;
    for (x = 0; x < width; ++ x){
      dst[pos ++] = (sum * mult) >>> shift;

      p = x + radius;
      p = start + (p < widthMinus1? p: widthMinus1);
      sum -= stack.color - src[p];

      stack.color = src[p];
      stack = stack.next;
    }
  }

  for (x = 0; x < width; ++ x){
    pos = x;
    start = pos + width;

    color = dst[pos];
    sum = radius * color;

    stack = stackStart;
    for (i = 0; i < radius; ++ i){
      stack.color = color;
      stack = stack.next;
    }
    for (i = 1; i < radius; ++ i){
      stack.color = dst[start];
      sum += stack.color;
      stack = stack.next;

      start += width;
    }

    stack = stackStart;
    for (y = 0; y < height; ++ y){
      dst[pos] = (sum * mult) >>> shift;

      p = y + radius;
      p = x + ( (p < heightMinus1? p: heightMinus1) * width );
      sum -= stack.color - dst[p];

      stack.color = dst[p];
      stack = stack.next;

      pos += width;
    }
  }

  return imageDst;
};

CV.gaussianBlur = function(imageSrc, imageDst, imageMean, kernelSize){
  var kernel = CV.gaussianKernel(kernelSize);

  imageDst.width = imageSrc.width;
  imageDst.height = imageSrc.height;

  imageMean.width = imageSrc.width;
  imageMean.height = imageSrc.height;

  CV.gaussianBlurFilter(imageSrc, imageMean, kernel, true);
  CV.gaussianBlurFilter(imageMean, imageDst, kernel, false);

  return imageDst;
};

CV.gaussianBlurFilter = function(imageSrc, imageDst, kernel, horizontal){
  var src = imageSrc.data, dst = imageDst.data,
      height = imageSrc.height, width = imageSrc.width,
      pos = 0, limit = kernel.length >> 1,
      cur, value, i, j, k;

  for (i = 0; i < height; ++ i){

    for (j = 0; j < width; ++ j){
      value = 0.0;

      for (k = -limit; k <= limit; ++ k){

        if (horizontal){
          cur = pos + k;
          if (j + k < 0){
            cur = pos;
          }
          else if (j + k >= width){
            cur = pos;
          }
        }else{
          cur = pos + (k * width);
          if (i + k < 0){
            cur = pos;
          }
          else if (i + k >= height){
            cur = pos;
          }
        }

        value += kernel[limit + k] * src[cur];
      }

      dst[pos ++] = horizontal? value: (value + 0.5) & 0xff;
    }
  }

  return imageDst;
};

CV.gaussianKernel = function(kernelSize){
  var tab =
    [ [1],
      [0.25, 0.5, 0.25],
      [0.0625, 0.25, 0.375, 0.25, 0.0625],
      [0.03125, 0.109375, 0.21875, 0.28125, 0.21875, 0.109375, 0.03125] ],
    kernel = [], center, sigma, scale2X, sum, x, i;

  if ( (kernelSize <= 7) && (kernelSize % 2 === 1) ){
    kernel = tab[kernelSize >> 1];
  }else{
    center = (kernelSize - 1.0) * 0.5;
    sigma = 0.8 + (0.3 * (center - 1.0) );
    scale2X = -0.5 / (sigma * sigma);
    sum = 0.0;
    for (i = 0; i < kernelSize; ++ i){
      x = i - center;
      sum += kernel[i] = Math.exp(scale2X * x * x);
    }
    sum = 1 / sum;
    for (i = 0; i < kernelSize; ++ i){
      kernel[i] *= sum;
    }
  }

  return kernel;
};

CV.findContours = function(imageSrc, binary){
  var width = imageSrc.width, height = imageSrc.height, contours = [],
      src, deltas, pos, pix, nbd, outer, hole, i, j;

  src = CV.binaryBorder(imageSrc, binary);

  deltas = CV.neighborhoodDeltas(width + 2);

  pos = width + 3;
  nbd = 1;

  for (i = 0; i < height; ++ i, pos += 2){

    for (j = 0; j < width; ++ j, ++ pos){
      pix = src[pos];

      if (0 !== pix){
        outer = hole = false;

        if (1 === pix && 0 === src[pos - 1]){
          outer = true;
        }
        else if (pix >= 1 && 0 === src[pos + 1]){
          hole = true;
        }

        if (outer || hole){
          ++ nbd;

          contours.push( CV.borderFollowing(src, pos, nbd, {x: j, y: i}, hole, deltas) );
        }
      }
    }
  }

  return contours;
};

CV.borderFollowing = function(src, pos, nbd, point, hole, deltas){
  var contour = [], pos1, pos3, pos4, s, s_end, s_prev;

  contour.hole = hole;

  s = s_end = hole? 0: 4;
  do{
    s = (s - 1) & 7;
    pos1 = pos + deltas[s];
    if (src[pos1] !== 0){
      break;
    }
  }while(s !== s_end);

  if (s === s_end){
    src[pos] = -nbd;
    contour.push( {x: point.x, y: point.y} );

  }else{
    pos3 = pos;
    s_prev = s ^ 4;

    while(true){
      s_end = s;

      do{
        pos4 = pos3 + deltas[++ s];
      }while(src[pos4] === 0);

      s &= 7;

      if ( ( (s - 1) >>> 0) < (s_end >>> 0) ){
        src[pos3] = -nbd;
      }
      else if (src[pos3] === 1){
        src[pos3] = nbd;
      }

      contour.push( {x: point.x, y: point.y} );

      s_prev = s;

      point.x += CV.neighborhood[s][0];
      point.y += CV.neighborhood[s][1];

      if ( (pos4 === pos) && (pos3 === pos1) ){
        break;
      }

      pos3 = pos4;
      s = (s + 4) & 7;
    }
  }

  return contour;
};

CV.neighborhood =
  [ [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1] ];

CV.neighborhoodDeltas = function(width){
  var deltas = [], len = CV.neighborhood.length, i = 0;

  for (; i < len; ++ i){
    deltas[i] = CV.neighborhood[i][0] + (CV.neighborhood[i][1] * width);
  }

  return deltas.concat(deltas);
};

CV.approxPolyDP = function(contour, epsilon){
  var slice = {start_index: 0, end_index: 0},
      right_slice = {start_index: 0, end_index: 0},
      poly = [], stack = [], len = contour.length,
      pt, start_pt, end_pt, dist, max_dist, le_eps,
      dx, dy, i, j, k;

  epsilon *= epsilon;

  k = 0;

  for (i = 0; i < 3; ++ i){
    max_dist = 0;

    k = (k + right_slice.start_index) % len;
    start_pt = contour[k];
    if (++ k === len) {k = 0;}

    for (j = 1; j < len; ++ j){
      pt = contour[k];
      if (++ k === len) {k = 0;}

      dx = pt.x - start_pt.x;
      dy = pt.y - start_pt.y;
      dist = dx * dx + dy * dy;

      if (dist > max_dist){
        max_dist = dist;
        right_slice.start_index = j;
      }
    }
  }

  if (max_dist <= epsilon){
    poly.push( {x: start_pt.x, y: start_pt.y} );

  }else{
    slice.start_index = k;
    slice.end_index = (right_slice.start_index += slice.start_index);

    right_slice.start_index -= right_slice.start_index >= len? len: 0;
    right_slice.end_index = slice.start_index;
    if (right_slice.end_index < right_slice.start_index){
      right_slice.end_index += len;
    }

    stack.push( {start_index: right_slice.start_index, end_index: right_slice.end_index} );
    stack.push( {start_index: slice.start_index, end_index: slice.end_index} );
  }

  while(stack.length !== 0){
    slice = stack.pop();

    end_pt = contour[slice.end_index % len];
    start_pt = contour[k = slice.start_index % len];
    if (++ k === len) {k = 0;}

    if (slice.end_index <= slice.start_index + 1){
      le_eps = true;

    }else{
      max_dist = 0;

      dx = end_pt.x - start_pt.x;
      dy = end_pt.y - start_pt.y;

      for (i = slice.start_index + 1; i < slice.end_index; ++ i){
        pt = contour[k];
        if (++ k === len) {k = 0;}

        dist = Math.abs( (pt.y - start_pt.y) * dx - (pt.x - start_pt.x) * dy);

        if (dist > max_dist){
          max_dist = dist;
          right_slice.start_index = i;
        }
      }

      le_eps = max_dist * max_dist <= epsilon * (dx * dx + dy * dy);
    }

    if (le_eps){
      poly.push( {x: start_pt.x, y: start_pt.y} );

    }else{
      right_slice.end_index = slice.end_index;
      slice.end_index = right_slice.start_index;

      stack.push( {start_index: right_slice.start_index, end_index: right_slice.end_index} );
      stack.push( {start_index: slice.start_index, end_index: slice.end_index} );
    }
  }

  return poly;
};

CV.warp = function(imageSrc, imageDst, contour, warpSize){
  var src = imageSrc.data, dst = imageDst.data,
      width = imageSrc.width, height = imageSrc.height,
      pos = 0,
      sx1, sx2, dx1, dx2, sy1, sy2, dy1, dy2, p1, p2, p3, p4,
      m, r, s, t, u, v, w, x, y, i, j;

  m = CV.getPerspectiveTransform(contour, warpSize - 1);

  r = m[8];
  s = m[2];
  t = m[5];

  for (i = 0; i < warpSize; ++ i){
    r += m[7];
    s += m[1];
    t += m[4];

    u = r;
    v = s;
    w = t;

    for (j = 0; j < warpSize; ++ j){
      u += m[6];
      v += m[0];
      w += m[3];

      x = v / u;
      y = w / u;

      sx1 = x >>> 0;
      sx2 = (sx1 === width - 1)? sx1: sx1 + 1;
      dx1 = x - sx1;
      dx2 = 1.0 - dx1;

      sy1 = y >>> 0;
      sy2 = (sy1 === height - 1)? sy1: sy1 + 1;
      dy1 = y - sy1;
      dy2 = 1.0 - dy1;

      p1 = p2 = sy1 * width;
      p3 = p4 = sy2 * width;

      dst[pos ++] =
        (dy2 * (dx2 * src[p1 + sx1] + dx1 * src[p2 + sx2]) +
         dy1 * (dx2 * src[p3 + sx1] + dx1 * src[p4 + sx2]) ) & 0xff;

    }
  }

  imageDst.width = warpSize;
  imageDst.height = warpSize;

  return imageDst;
};

CV.getPerspectiveTransform = function(src, size){
  var rq = CV.square2quad(src);

  rq[0] /= size;
  rq[1] /= size;
  rq[3] /= size;
  rq[4] /= size;
  rq[6] /= size;
  rq[7] /= size;

  return rq;
};

CV.square2quad = function(src){
  var sq = [], px, py, dx1, dx2, dy1, dy2, den;

  px = src[0].x - src[1].x + src[2].x - src[3].x;
  py = src[0].y - src[1].y + src[2].y - src[3].y;

  if (0 === px && 0 === py){
    sq[0] = src[1].x - src[0].x;
    sq[1] = src[2].x - src[1].x;
    sq[2] = src[0].x;
    sq[3] = src[1].y - src[0].y;
    sq[4] = src[2].y - src[1].y;
    sq[5] = src[0].y;
    sq[6] = 0;
    sq[7] = 0;
    sq[8] = 1;

  }else{
    dx1 = src[1].x - src[2].x;
    dx2 = src[3].x - src[2].x;
    dy1 = src[1].y - src[2].y;
    dy2 = src[3].y - src[2].y;
    den = dx1 * dy2 - dx2 * dy1;

    sq[6] = (px * dy2 - dx2 * py) / den;
    sq[7] = (dx1 * py - px * dy1) / den;
    sq[8] = 1;
    sq[0] = src[1].x - src[0].x + sq[6] * src[1].x;
    sq[1] = src[3].x - src[0].x + sq[7] * src[3].x;
    sq[2] = src[0].x;
    sq[3] = src[1].y - src[0].y + sq[6] * src[1].y;
    sq[4] = src[3].y - src[0].y + sq[7] * src[3].y;
    sq[5] = src[0].y;
  }

  return sq;
};

CV.isContourConvex = function(contour){
  var orientation = 0, convex = true,
      len = contour.length, i = 0, j = 0,
      cur_pt, prev_pt, dxdy0, dydx0, dx0, dy0, dx, dy;

  prev_pt = contour[len - 1];
  cur_pt = contour[0];

  dx0 = cur_pt.x - prev_pt.x;
  dy0 = cur_pt.y - prev_pt.y;

  for (; i < len; ++ i){
    if (++ j === len) {j = 0;}

    prev_pt = cur_pt;
    cur_pt = contour[j];

    dx = cur_pt.x - prev_pt.x;
    dy = cur_pt.y - prev_pt.y;
    dxdy0 = dx * dy0;
    dydx0 = dy * dx0;

    orientation |= dydx0 > dxdy0? 1: (dydx0 < dxdy0? 2: 3);

    if (3 === orientation){
        convex = false;
        break;
    }

    dx0 = dx;
    dy0 = dy;
  }

  return convex;
};

CV.perimeter = function(poly){
  var len = poly.length, i = 0, j = len - 1,
      p = 0.0, dx, dy;

  for (; i < len; j = i ++){
    dx = poly[i].x - poly[j].x;
    dy = poly[i].y - poly[j].y;

    p += Math.sqrt(dx * dx + dy * dy) ;
  }

  return p;
};

CV.minEdgeLength = function(poly){
  var len = poly.length, i = 0, j = len - 1,
      min = Infinity, d, dx, dy;

  for (; i < len; j = i ++){
    dx = poly[i].x - poly[j].x;
    dy = poly[i].y - poly[j].y;

    d = dx * dx + dy * dy;

    if (d < min){
      min = d;
    }
  }

  return Math.sqrt(min);
};

CV.countNonZero = function(imageSrc, square){
  var src = imageSrc.data, height = square.height, width = square.width,
      pos = square.x + (square.y * imageSrc.width),
      span = imageSrc.width - width,
      nz = 0, i, j;

  for (i = 0; i < height; ++ i){

    for (j = 0; j < width; ++ j){

      if ( 0 !== src[pos ++] ){
        ++ nz;
      }
    }

    pos += span;
  }

  return nz;
};

CV.binaryBorder = function(imageSrc, dst){
  var src = imageSrc.data, height = imageSrc.height, width = imageSrc.width,
      posSrc = 0, posDst = 0, i, j;

  for (j = -2; j < width; ++ j){
    dst[posDst ++] = 0;
  }

  for (i = 0; i < height; ++ i){
    dst[posDst ++] = 0;

    for (j = 0; j < width; ++ j){
      dst[posDst ++] = (0 === src[posSrc ++]? 0: 1);
    }

    dst[posDst ++] = 0;
  }

  for (j = -2; j < width; ++ j){
    dst[posDst ++] = 0;
  }

  return dst;
};

export { CV };
```

## 10. `client/core/tracker/vendor/aruco.js`

```js
/*
Copyright (c) 2020 Damiano Falcioni
Copyright (c) 2011 Juan Mellado

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
References:
- "ArUco: a minimal library for Augmented Reality applications based on OpenCv"
  http://www.uco.es/investiga/grupos/ava/node/26
- "js-aruco: a port to JavaScript of the ArUco library"
  https://github.com/jcmellado/js-aruco
*/

var AR = {};
import { CV } from './cv.js';
// ESM vendored (was: var CV = this.CV || require('./cv').CV; this.AR = AR;)

AR.DICTIONARIES = {
  ARUCO: {
    nBits: 25,
    tau: 3,
    codeList: [0x1084210,0x1084217,0x1084209,0x108420e,0x10842f0,0x10842f7,0x10842e9,0x10842ee,0x1084130,0x1084137,0x1084129,0x108412e,0x10841d0,0x10841d7,0x10841c9,0x10841ce,0x1085e10,0x1085e17,0x1085e09,0x1085e0e,0x1085ef0,0x1085ef7,0x1085ee9,0x1085eee,0x1085d30,0x1085d37,0x1085d29,0x1085d2e,0x1085dd0,0x1085dd7,0x1085dc9,0x1085dce,0x1082610,0x1082617,0x1082609,0x108260e,0x10826f0,0x10826f7,0x10826e9,0x10826ee,0x1082530,0x1082537,0x1082529,0x108252e,0x10825d0,0x10825d7,0x10825c9,0x10825ce,0x1083a10,0x1083a17,0x1083a09,0x1083a0e,0x1083af0,0x1083af7,0x1083ae9,0x1083aee,0x1083930,0x1083937,0x1083929,0x108392e,0x10839d0,0x10839d7,0x10839c9,0x10839ce,0x10bc210,0x10bc217,0x10bc209,0x10bc20e,0x10bc2f0,0x10bc2f7,0x10bc2e9,0x10bc2ee,0x10bc130,0x10bc137,0x10bc129,0x10bc12e,0x10bc1d0,0x10bc1d7,0x10bc1c9,0x10bc1ce,0x10bde10,0x10bde17,0x10bde09,0x10bde0e,0x10bdef0,0x10bdef7,0x10bdee9,0x10bdeee,0x10bdd30,0x10bdd37,0x10bdd29,0x10bdd2e,0x10bddd0,0x10bddd7,0x10bddc9,0x10bddce,0x10ba610,0x10ba617,0x10ba609,0x10ba60e,0x10ba6f0,0x10ba6f7,0x10ba6e9,0x10ba6ee,0x10ba530,0x10ba537,0x10ba529,0x10ba52e,0x10ba5d0,0x10ba5d7,0x10ba5c9,0x10ba5ce,0x10bba10,0x10bba17,0x10bba09,0x10bba0e,0x10bbaf0,0x10bbaf7,0x10bbae9,0x10bbaee,0x10bb930,0x10bb937,0x10bb929,0x10bb92e,0x10bb9d0,0x10bb9d7,0x10bb9c9,0x10bb9ce,0x104c210,0x104c217,0x104c209,0x104c20e,0x104c2f0,0x104c2f7,0x104c2e9,0x104c2ee,0x104c130,0x104c137,0x104c129,0x104c12e,0x104c1d0,0x104c1d7,0x104c1c9,0x104c1ce,0x104de10,0x104de17,0x104de09,0x104de0e,0x104def0,0x104def7,0x104dee9,0x104deee,0x104dd30,0x104dd37,0x104dd29,0x104dd2e,0x104ddd0,0x104ddd7,0x104ddc9,0x104ddce,0x104a610,0x104a617,0x104a609,0x104a60e,0x104a6f0,0x104a6f7,0x104a6e9,0x104a6ee,0x104a530,0x104a537,0x104a529,0x104a52e,0x104a5d0,0x104a5d7,0x104a5c9,0x104a5ce,0x104ba10,0x104ba17,0x104ba09,0x104ba0e,0x104baf0,0x104baf7,0x104bae9,0x104baee,0x104b930,0x104b937,0x104b929,0x104b92e,0x104b9d0,0x104b9d7,0x104b9c9,0x104b9ce,0x1074210,0x1074217,0x1074209,0x107420e,0x10742f0,0x10742f7,0x10742e9,0x10742ee,0x1074130,0x1074137,0x1074129,0x107412e,0x10741d0,0x10741d7,0x10741c9,0x10741ce,0x1075e10,0x1075e17,0x1075e09,0x1075e0e,0x1075ef0,0x1075ef7,0x1075ee9,0x1075eee,0x1075d30,0x1075d37,0x1075d29,0x1075d2e,0x1075dd0,0x1075dd7,0x1075dc9,0x1075dce,0x1072610,0x1072617,0x1072609,0x107260e,0x10726f0,0x10726f7,0x10726e9,0x10726ee,0x1072530,0x1072537,0x1072529,0x107252e,0x10725d0,0x10725d7,0x10725c9,0x10725ce,0x1073a10,0x1073a17,0x1073a09,0x1073a0e,0x1073af0,0x1073af7,0x1073ae9,0x1073aee,0x1073930,0x1073937,0x1073929,0x107392e,0x10739d0,0x10739d7,0x10739c9,0x10739ce,0x1784210,0x1784217,0x1784209,0x178420e,0x17842f0,0x17842f7,0x17842e9,0x17842ee,0x1784130,0x1784137,0x1784129,0x178412e,0x17841d0,0x17841d7,0x17841c9,0x17841ce,0x1785e10,0x1785e17,0x1785e09,0x1785e0e,0x1785ef0,0x1785ef7,0x1785ee9,0x1785eee,0x1785d30,0x1785d37,0x1785d29,0x1785d2e,0x1785dd0,0x1785dd7,0x1785dc9,0x1785dce,0x1782610,0x1782617,0x1782609,0x178260e,0x17826f0,0x17826f7,0x17826e9,0x17826ee,0x1782530,0x1782537,0x1782529,0x178252e,0x17825d0,0x17825d7,0x17825c9,0x17825ce,0x1783a10,0x1783a17,0x1783a09,0x1783a0e,0x1783af0,0x1783af7,0x1783ae9,0x1783aee,0x1783930,0x1783937,0x1783929,0x178392e,0x17839d0,0x17839d7,0x17839c9,0x17839ce,0x17bc210,0x17bc217,0x17bc209,0x17bc20e,0x17bc2f0,0x17bc2f7,0x17bc2e9,0x17bc2ee,0x17bc130,0x17bc137,0x17bc129,0x17bc12e,0x17bc1d0,0x17bc1d7,0x17bc1c9,0x17bc1ce,0x17bde10,0x17bde17,0x17bde09,0x17bde0e,0x17bdef0,0x17bdef7,0x17bdee9,0x17bdeee,0x17bdd30,0x17bdd37,0x17bdd29,0x17bdd2e,0x17bddd0,0x17bddd7,0x17bddc9,0x17bddce,0x17ba610,0x17ba617,0x17ba609,0x17ba60e,0x17ba6f0,0x17ba6f7,0x17ba6e9,0x17ba6ee,0x17ba530,0x17ba537,0x17ba529,0x17ba52e,0x17ba5d0,0x17ba5d7,0x17ba5c9,0x17ba5ce,0x17bba10,0x17bba17,0x17bba09,0x17bba0e,0x17bbaf0,0x17bbaf7,0x17bbae9,0x17bbaee,0x17bb930,0x17bb937,0x17bb929,0x17bb92e,0x17bb9d0,0x17bb9d7,0x17bb9c9,0x17bb9ce,0x174c210,0x174c217,0x174c209,0x174c20e,0x174c2f0,0x174c2f7,0x174c2e9,0x174c2ee,0x174c130,0x174c137,0x174c129,0x174c12e,0x174c1d0,0x174c1d7,0x174c1c9,0x174c1ce,0x174de10,0x174de17,0x174de09,0x174de0e,0x174def0,0x174def7,0x174dee9,0x174deee,0x174dd30,0x174dd37,0x174dd29,0x174dd2e,0x174ddd0,0x174ddd7,0x174ddc9,0x174ddce,0x174a610,0x174a617,0x174a609,0x174a60e,0x174a6f0,0x174a6f7,0x174a6e9,0x174a6ee,0x174a530,0x174a537,0x174a529,0x174a52e,0x174a5d0,0x174a5d7,0x174a5c9,0x174a5ce,0x174ba10,0x174ba17,0x174ba09,0x174ba0e,0x174baf0,0x174baf7,0x174bae9,0x174baee,0x174b930,0x174b937,0x174b929,0x174b92e,0x174b9d0,0x174b9d7,0x174b9c9,0x174b9ce,0x1774210,0x1774217,0x1774209,0x177420e,0x17742f0,0x17742f7,0x17742e9,0x17742ee,0x1774130,0x1774137,0x1774129,0x177412e,0x17741d0,0x17741d7,0x17741c9,0x17741ce,0x1775e10,0x1775e17,0x1775e09,0x1775e0e,0x1775ef0,0x1775ef7,0x1775ee9,0x1775eee,0x1775d30,0x1775d37,0x1775d29,0x1775d2e,0x1775dd0,0x1775dd7,0x1775dc9,0x1775dce,0x1772610,0x1772617,0x1772609,0x177260e,0x17726f0,0x17726f7,0x17726e9,0x17726ee,0x1772530,0x1772537,0x1772529,0x177252e,0x17725d0,0x17725d7,0x17725c9,0x17725ce,0x1773a10,0x1773a17,0x1773a09,0x1773a0e,0x1773af0,0x1773af7,0x1773ae9,0x1773aee,0x1773930,0x1773937,0x1773929,0x177392e,0x17739d0,0x17739d7,0x17739c9,0x17739ce,0x984210,0x984217,0x984209,0x98420e,0x9842f0,0x9842f7,0x9842e9,0x9842ee,0x984130,0x984137,0x984129,0x98412e,0x9841d0,0x9841d7,0x9841c9,0x9841ce,0x985e10,0x985e17,0x985e09,0x985e0e,0x985ef0,0x985ef7,0x985ee9,0x985eee,0x985d30,0x985d37,0x985d29,0x985d2e,0x985dd0,0x985dd7,0x985dc9,0x985dce,0x982610,0x982617,0x982609,0x98260e,0x9826f0,0x9826f7,0x9826e9,0x9826ee,0x982530,0x982537,0x982529,0x98252e,0x9825d0,0x9825d7,0x9825c9,0x9825ce,0x983a10,0x983a17,0x983a09,0x983a0e,0x983af0,0x983af7,0x983ae9,0x983aee,0x983930,0x983937,0x983929,0x98392e,0x9839d0,0x9839d7,0x9839c9,0x9839ce,0x9bc210,0x9bc217,0x9bc209,0x9bc20e,0x9bc2f0,0x9bc2f7,0x9bc2e9,0x9bc2ee,0x9bc130,0x9bc137,0x9bc129,0x9bc12e,0x9bc1d0,0x9bc1d7,0x9bc1c9,0x9bc1ce,0x9bde10,0x9bde17,0x9bde09,0x9bde0e,0x9bdef0,0x9bdef7,0x9bdee9,0x9bdeee,0x9bdd30,0x9bdd37,0x9bdd29,0x9bdd2e,0x9bddd0,0x9bddd7,0x9bddc9,0x9bddce,0x9ba610,0x9ba617,0x9ba609,0x9ba60e,0x9ba6f0,0x9ba6f7,0x9ba6e9,0x9ba6ee,0x9ba530,0x9ba537,0x9ba529,0x9ba52e,0x9ba5d0,0x9ba5d7,0x9ba5c9,0x9ba5ce,0x9bba10,0x9bba17,0x9bba09,0x9bba0e,0x9bbaf0,0x9bbaf7,0x9bbae9,0x9bbaee,0x9bb930,0x9bb937,0x9bb929,0x9bb92e,0x9bb9d0,0x9bb9d7,0x9bb9c9,0x9bb9ce,0x94c210,0x94c217,0x94c209,0x94c20e,0x94c2f0,0x94c2f7,0x94c2e9,0x94c2ee,0x94c130,0x94c137,0x94c129,0x94c12e,0x94c1d0,0x94c1d7,0x94c1c9,0x94c1ce,0x94de10,0x94de17,0x94de09,0x94de0e,0x94def0,0x94def7,0x94dee9,0x94deee,0x94dd30,0x94dd37,0x94dd29,0x94dd2e,0x94ddd0,0x94ddd7,0x94ddc9,0x94ddce,0x94a610,0x94a617,0x94a609,0x94a60e,0x94a6f0,0x94a6f7,0x94a6e9,0x94a6ee,0x94a530,0x94a537,0x94a529,0x94a52e,0x94a5d0,0x94a5d7,0x94a5c9,0x94a5ce,0x94ba10,0x94ba17,0x94ba09,0x94ba0e,0x94baf0,0x94baf7,0x94bae9,0x94baee,0x94b930,0x94b937,0x94b929,0x94b92e,0x94b9d0,0x94b9d7,0x94b9c9,0x94b9ce,0x974210,0x974217,0x974209,0x97420e,0x9742f0,0x9742f7,0x9742e9,0x9742ee,0x974130,0x974137,0x974129,0x97412e,0x9741d0,0x9741d7,0x9741c9,0x9741ce,0x975e10,0x975e17,0x975e09,0x975e0e,0x975ef0,0x975ef7,0x975ee9,0x975eee,0x975d30,0x975d37,0x975d29,0x975d2e,0x975dd0,0x975dd7,0x975dc9,0x975dce,0x972610,0x972617,0x972609,0x97260e,0x9726f0,0x9726f7,0x9726e9,0x9726ee,0x972530,0x972537,0x972529,0x97252e,0x9725d0,0x9725d7,0x9725c9,0x9725ce,0x973a10,0x973a17,0x973a09,0x973a0e,0x973af0,0x973af7,0x973ae9,0x973aee,0x973930,0x973937,0x973929,0x97392e,0x9739d0,0x9739d7,0x9739c9,0x9739ce,0xe84210,0xe84217,0xe84209,0xe8420e,0xe842f0,0xe842f7,0xe842e9,0xe842ee,0xe84130,0xe84137,0xe84129,0xe8412e,0xe841d0,0xe841d7,0xe841c9,0xe841ce,0xe85e10,0xe85e17,0xe85e09,0xe85e0e,0xe85ef0,0xe85ef7,0xe85ee9,0xe85eee,0xe85d30,0xe85d37,0xe85d29,0xe85d2e,0xe85dd0,0xe85dd7,0xe85dc9,0xe85dce,0xe82610,0xe82617,0xe82609,0xe8260e,0xe826f0,0xe826f7,0xe826e9,0xe826ee,0xe82530,0xe82537,0xe82529,0xe8252e,0xe825d0,0xe825d7,0xe825c9,0xe825ce,0xe83a10,0xe83a17,0xe83a09,0xe83a0e,0xe83af0,0xe83af7,0xe83ae9,0xe83aee,0xe83930,0xe83937,0xe83929,0xe8392e,0xe839d0,0xe839d7,0xe839c9,0xe839ce,0xebc210,0xebc217,0xebc209,0xebc20e,0xebc2f0,0xebc2f7,0xebc2e9,0xebc2ee,0xebc130,0xebc137,0xebc129,0xebc12e,0xebc1d0,0xebc1d7,0xebc1c9,0xebc1ce,0xebde10,0xebde17,0xebde09,0xebde0e,0xebdef0,0xebdef7,0xebdee9,0xebdeee,0xebdd30,0xebdd37,0xebdd29,0xebdd2e,0xebddd0,0xebddd7,0xebddc9,0xebddce,0xeba610,0xeba617,0xeba609,0xeba60e,0xeba6f0,0xeba6f7,0xeba6e9,0xeba6ee,0xeba530,0xeba537,0xeba529,0xeba52e,0xeba5d0,0xeba5d7,0xeba5c9,0xeba5ce,0xebba10,0xebba17,0xebba09,0xebba0e,0xebbaf0,0xebbaf7,0xebbae9,0xebbaee,0xebb930,0xebb937,0xebb929,0xebb92e,0xebb9d0,0xebb9d7,0xebb9c9,0xebb9ce,0xe4c210,0xe4c217,0xe4c209,0xe4c20e,0xe4c2f0,0xe4c2f7,0xe4c2e9,0xe4c2ee,0xe4c130,0xe4c137,0xe4c129,0xe4c12e,0xe4c1d0,0xe4c1d7,0xe4c1c9,0xe4c1ce,0xe4de10,0xe4de17,0xe4de09,0xe4de0e,0xe4def0,0xe4def7,0xe4dee9,0xe4deee,0xe4dd30,0xe4dd37,0xe4dd29,0xe4dd2e,0xe4ddd0,0xe4ddd7,0xe4ddc9,0xe4ddce,0xe4a610,0xe4a617,0xe4a609,0xe4a60e,0xe4a6f0,0xe4a6f7,0xe4a6e9,0xe4a6ee,0xe4a530,0xe4a537,0xe4a529,0xe4a52e,0xe4a5d0,0xe4a5d7,0xe4a5c9,0xe4a5ce,0xe4ba10,0xe4ba17,0xe4ba09,0xe4ba0e,0xe4baf0,0xe4baf7,0xe4bae9,0xe4baee,0xe4b930,0xe4b937,0xe4b929,0xe4b92e,0xe4b9d0,0xe4b9d7,0xe4b9c9,0xe4b9ce,0xe74210,0xe74217,0xe74209,0xe7420e,0xe742f0,0xe742f7,0xe742e9,0xe742ee,0xe74130,0xe74137,0xe74129,0xe7412e,0xe741d0,0xe741d7,0xe741c9,0xe741ce,0xe75e10,0xe75e17,0xe75e09,0xe75e0e,0xe75ef0,0xe75ef7,0xe75ee9,0xe75eee,0xe75d30,0xe75d37,0xe75d29,0xe75d2e,0xe75dd0,0xe75dd7,0xe75dc9,0xe75dce,0xe72610,0xe72617,0xe72609,0xe7260e,0xe726f0,0xe726f7,0xe726e9,0xe726ee,0xe72530,0xe72537,0xe72529,0xe7252e,0xe725d0,0xe725d7,0xe725c9,0xe725ce,0xe73a10,0xe73a17,0xe73a09,0xe73a0e,0xe73af0,0xe73af7,0xe73ae9,0xe73aee,0xe73930,0xe73937,0xe73929,0xe7392e,0xe739d0,0xe739d7,0xe739c9]
  },
  ARUCO_MIP_36h12: {
    nBits: 36,
    tau: 12,
    codeList: [0xd2b63a09d,0x6001134e5,0x1206fbe72,0xff8ad6cb4,0x85da9bc49,0xb461afe9c,0x6db51fe13,0x5248c541f,0x8f34503,0x8ea462ece,0xeac2be76d,0x1af615c44,0xb48a49f27,0x2e4e1283b,0x78b1f2fa8,0x27d34f57e,0x89222fff1,0x4c1669406,0xbf49b3511,0xdc191cd5d,0x11d7c3f85,0x16a130e35,0xe29f27eff,0x428d8ae0c,0x90d548477,0x2319cbc93,0xc3b0c3dfc,0x424bccc9,0x2a081d630,0x762743d96,0xd0645bf19,0xf38d7fd60,0xc6cbf9a10,0x3c1be7c65,0x276f75e63,0x4490a3f63,0xda60acd52,0x3cc68df59,0xab46f9dae,0x88d533d78,0xb6d62ec21,0xb3c02b646,0x22e56d408,0xac5f5770a,0xaaa993f66,0x4caa07c8d,0x5c9b4f7b0,0xaa9ef0e05,0x705c5750,0xac81f545e,0x735b91e74,0x8cc35cee4,0xe44694d04,0xb5e121de0,0x261017d0f,0xf1d439eb5,0xa1a33ac96,0x174c62c02,0x1ee27f716,0x8b1c5ece9,0x6a05b0c6a,0xd0568dfc,0x192d25e5f,0x1adbeccc8,0xcfec87f00,0xd0b9dde7a,0x88dcef81e,0x445681cb9,0xdbb2ffc83,0xa48d96df1,0xb72cc2e7d,0xc295b53f,0xf49832704,0x9968edc29,0x9e4e1af85,0x8683e2d1b,0x810b45c04,0x6ac44bfe2,0x645346615,0x3990bd598,0x1c9ed0f6a,0xc26729d65,0x83993f795,0x3ac05ac5d,0x357adff3b,0xd5c05565,0x2f547ef44,0x86c115041,0x640fd9e5f,0xce08bbcf7,0x109bb343e,0xc21435c92,0x35b4dfce4,0x459752cf2,0xec915b82c,0x51881eed0,0x2dda7dc97,0x2e0142144,0x42e890f99,0x9a8856527,0x8e80d9d80,0x891cbcf34,0x25dd82410,0x239551d34,0x8fe8f0c70,0x94106a970,0x82609b40c,0xfc9caf36,0x688181d11,0x718613c08,0xf1ab7629,0xa357bfc18,0x4c03b7a46,0x204dedce6,0xad6300d37,0x84cc4cd09,0x42160e5c4,0x87d2adfa8,0x7850e7749,0x4e750fc7c,0xbf2e5dfda,0xd88324da5,0x234b52f80,0x378204514,0xabdf2ad53,0x365e78ef9,0x49caa6ca2,0x3c39ddf3,0xc68c5385d,0x5bfcbbf67,0x623241e21,0xabc90d5cc,0x388c6fe85,0xda0e2d62d,0x10855dfe9,0x4d46efd6b,0x76ea12d61,0x9db377d3d,0xeed0efa71,0xe6ec3ae2f,0x441faee83,0xba19c8ff5,0x313035eab,0x6ce8f7625,0x880dab58d,0x8d3409e0d,0x2be92ee21,0xd60302c6c,0x469ffc724,0x87eebeed3,0x42587ef7a,0x7a8cc4e52,0x76a437650,0x999e41ef4,0x7d0969e42,0xc02baf46b,0x9259f3e47,0x2116a1dc0,0x9f2de4d84,0xeffac29,0x7b371ff8c,0x668339da9,0xd010aee3f,0x1cd00b4c0,0x95070fc3b,0xf84c9a770,0x38f863d76,0x3646ff045,0xce1b96412,0x7a5d45da8,0x14e00ef6c,0x5e95abfd8,0xb2e9cb729,0x36c47dd7,0xb8ee97c6b,0xe9e8f657,0xd4ad2ef1a,0x8811c7f32,0x47bde7c31,0x3adadfb64,0x6e5b28574,0x33e67cd91,0x2ab9fdd2d,0x8afa67f2b,0xe6a28fc5e,0x72049cdbd,0xae65dac12,0x1251a4526,0x1089ab841,0xe2f096ee0,0xb0caee573,0xfd6677e86,0x444b3f518,0xbe8b3a56a,0x680a75cfc,0xac02baea8,0x97d815e1c,0x1d4386e08,0x1a14f5b0e,0xe658a8d81,0xa3868efa7,0x3668a9673,0xe8fc53d85,0x2e2b7edd5,0x8b2470f13,0xf69795f32,0x4589ffc8e,0x2e2080c9c,0x64265f7d,0x3d714dd10,0x1692c6ef1,0x3e67f2f49,0x5041dad63,0x1a1503415,0x64c18c742,0xa72eec35,0x1f0f9dc60,0xa9559bc67,0xf32911d0d,0x21c0d4ffc,0xe01cef5b0,0x4e23a3520,0xaa4f04e49,0xe1c4fcc43,0x208e8f6e8,0x8486774a5,0x9e98c7558,0x2c59fb7dc,0x9446a4613,0x8292dcc2e,0x4d61631,0xd05527809,0xa0163852d,0x8f657f639,0xcca6c3e37,0xcb136bc7a,0xfc5a83e53,0x9aa44fc30,0xbdec1bd3c,0xe020b9f7c,0x4b8f35fb0,0xb8165f637,0x33dc88d69,0x10a2f7e4d,0xc8cb5ff53,0xde259ff6b,0x46d070dd4,0x32d3b9741,0x7075f1c04,0x4d58dbea0]
  }
};

AR.Dictionary = function (dicName) {
  this.codes = {};
  this.codeList = [];
  this.tau = 0;
  this._initialize(dicName);
};

AR.Dictionary.prototype._initialize = function (dicName) {
  this.codes = {};
  this.codeList = [];
  this.tau = 0;
  this.nBits = 0;
  this.markSize = 0;
  this.dicName = dicName;
  var dictionary = AR.DICTIONARIES[dicName];
  if (!dictionary)
    throw 'The dictionary "' + dicName + '" is not recognized.';
  
  this.nBits = dictionary.nBits;
  this.markSize = Math.sqrt(dictionary.nBits) + 2;
  for (var i = 0; i < dictionary.codeList.length; i++) {
    var code = null;
    if (typeof dictionary.codeList[i] === 'number')
      code = this._hex2bin(dictionary.codeList[i], dictionary.nBits);
    if (typeof dictionary.codeList[i] === 'string')
      code = this._hex2bin(parseInt(dictionary.codeList[i], 16), dictionary.nBits);
    if (Array.isArray(dictionary.codeList[i])) 
      code = this._bytes2bin(dictionary.codeList[i], dictionary.nBits);
    if (code === null) 
      throw 'Invalid code ' + i + ' in dictionary ' + dicName + ': ' + JSON.stringify(dictionary.codeList[i]);
    if (code.length != dictionary.nBits)
      throw 'The code ' + i + ' in dictionary ' + dicName + ' is not ' +  dictionary.nBits + ' bits long but ' + code.length + ': ' + code;
    this.codeList.push(code);
    this.codes[code] = {
      id: i
    };
  }
  this.tau = dictionary.tau || this._calculateTau();
};

AR.Dictionary.prototype.find = function (bits) {
  var val = '',
    i, j;
  for (i = 0; i < bits.length; i++) {
    var bitRow = bits[i];
    for (j = 0; j < bitRow.length; j++) {
      val += bitRow[j];
    }
  }
  var minFound = this.codes[val];
  if (minFound)
    return {
      id: minFound.id,
      distance: 0
    };

  for (i = 0; i < this.codeList.length; i++) {
    var code = this.codeList[i];
    var distance = this._hammingDistance(val, code);
    if (this._hammingDistance(val, code) < this.tau) {
      if (!minFound || minFound.distance > distance) {
        minFound = {
          id: this.codes[code].id,
          distance: distance
        };
      }
    }
  }
  return minFound;
};

AR.Dictionary.prototype._hex2bin = function (hex, nBits) {
  return hex.toString(2).padStart(nBits, '0');
};

AR.Dictionary.prototype._bytes2bin = function (byteList, nBits) {
  var bits = '', byte;
  for (byte of byteList) {
    bits += byte.toString(2).padStart(bits.length + 8 > nBits?nBits - bits.length:8, '0');
  }
  return bits;
};

AR.Dictionary.prototype._hammingDistance = function (str1, str2) {
  if (str1.length != str2.length)
    throw 'Hamming distance calculation require inputs of the same length';
  var distance = 0,
    i;
  for (i = 0; i < str1.length; i++)
    if (str1[i] !== str2[i])
      distance += 1;
  return distance;
};

AR.Dictionary.prototype._calculateTau = function () {
  var tau = Number.MAX_VALUE;
  for(var i=0;i<this.codeList.length;i++)
    for(var j=i+1;j<this.codeList.length;j++) {
      var distance = this._hammingDistance(this.codeList[i], this.codeList[j]);
      tau = distance < tau ? distance : tau;
    }
  return tau;
};

AR.Dictionary.prototype.generateSVG = function (id) {
  var code = this.codeList[id];
  if (code == null)
    throw 'The id "' + id + '" is not valid for the dictionary "' + this.dicName + '". ID must be between 0 and ' + (this.codeList.length-1) + ' included.';
  var size = this.markSize - 2;
  var svg = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 '+ (size+4) + ' ' + (size+4) + '">';
  svg += '<rect x="0" y="0" width="' + (size+4) + '" height="' + (size+4) + '" fill="white"/>';
  svg += '<rect x="1" y="1" width="' + (size+2) + '" height="' + (size+2) + '" fill="black"/>';
  for(var y=0;y<size;y++) {
    for(var x=0;x<size;x++) {
      if (code[y*size+x]=='1') 
        svg += '<rect x="' + (x+2) + '" y="' + (y+2) + '" width="1" height="1" fill="white"/>';
    }
  }
  svg += '</svg>';
  return svg;
};

AR.Marker = function (id, corners, hammingDistance) {
  this.id = id;
  this.corners = corners;
  this.hammingDistance = hammingDistance;
};

AR.Detector = function (config) {
  config = config || {};
  this.grey = new CV.Image();
  this.thres = new CV.Image();
  this.homography = new CV.Image();
  this.binary = [];
  this.contours = [];
  this.polys = [];
  this.candidates = [];
  config.dictionaryName = config.dictionaryName || 'ARUCO_MIP_36h12';
  this.dictionary = new AR.Dictionary(config.dictionaryName);
  this.dictionary.tau = config.maxHammingDistance != null ? config.maxHammingDistance : this.dictionary.tau;
};

AR.Detector.prototype.detectImage = function (width, height, data) {
  return this.detect({
    width: width,
    height: height,
    data: data
  });
};

AR.Detector.prototype.detectStreamInit = function (width, height, callback) {
  this.streamConfig = {};
  this.streamConfig.width = width;
  this.streamConfig.height = height;
  this.streamConfig.imageSize = width * height * 4; //provided image must be a sequence of rgba bytes (4 bytes represent a pixel)
  this.streamConfig.index = 0;
  this.streamConfig.imageData = new Uint8ClampedArray(this.streamConfig.imageSize);
  this.streamConfig.callback = callback || function (image, markerList) {};
};

//accept data chunks of different sizes
AR.Detector.prototype.detectStream = function (data) {
  for (var i = 0; i < data.length; i++) {
    this.streamConfig.imageData[this.streamConfig.index] = data[i];
    this.streamConfig.index = (this.streamConfig.index + 1) % this.streamConfig.imageSize;
    if (this.streamConfig.index == 0) {
      var image = {
        width: this.streamConfig.width,
        height: this.streamConfig.height,
        data: this.streamConfig.imageData
      };
      var markerList = this.detect(image);
      this.streamConfig.callback(image, markerList);
    }
  }
};

AR.Detector.prototype.detectMJPEGStreamInit = function (width, height, callback, decoderFn) {
  this.mjpeg = {
    decoderFn: decoderFn,
    chunks: [],
    SOI: [0xff, 0xd8],
    EOI: [0xff, 0xd9]
  };
  this.detectStreamInit(width, height, callback);
};

AR.Detector.prototype.detectMJPEGStream = function (chunk) {
  var eoiPos = chunk.findIndex(function (element, index, array) {
    return this.mjpeg.EOI[0] == element && array.length > index + 1 && this.mjpeg.EOI[1] == array[index + 1];
  });
  var soiPos = chunk.findIndex(function (element, index, array) {
    return this.mjpeg.SOI[0] == element && array.length > index + 1 && this.mjpeg.SOI[1] == array[index + 1];
  });

  if (eoiPos === -1) {
    this.mjpeg.chunks.push(chunk);
  } else {
    var part1 = chunk.slice(0, eoiPos + 2);
    if (part1.length) {
      this.mjpeg.chunks.push(part1);
    }
    if (this.mjpeg.chunks.length) {
      var jpegImage = this.mjpeg.chunks.flat();
      var rgba = this.mjpeg.decoderFn(jpegImage);
      this.detectStream(rgba);
    }
    this.mjpeg.chunks = [];
  }
  if (soiPos > -1) {
    this.mjpeg.chunks = [];
    this.mjpeg.chunks.push(chunk.slice(soiPos));
  }
};

AR.Detector.prototype.detect = function (image) {
  CV.grayscale(image, this.grey);
  CV.adaptiveThreshold(this.grey, this.thres, 2, 7);

  this.contours = CV.findContours(this.thres, this.binary);
  //Scale Fix: https://stackoverflow.com/questions/35936397/marker-detection-on-paper-sheet-using-javascript
  //this.candidates = this.findCandidates(this.contours, image.width * 0.20, 0.05, 10);
  this.candidates = this.findCandidates(this.contours, image.width * 0.01, 0.05, 10);
  this.candidates = this.clockwiseCorners(this.candidates);
  this.candidates = this.notTooNear(this.candidates, 10);

  return this.findMarkers(this.grey, this.candidates, 49);
};

AR.Detector.prototype.findCandidates = function (contours, minSize, epsilon, minLength) {
  var candidates = [],
    len = contours.length,
    contour, poly, i;

  this.polys = [];

  for (i = 0; i < len; ++i) {
    contour = contours[i];

    if (contour.length >= minSize) {
      poly = CV.approxPolyDP(contour, contour.length * epsilon);

      this.polys.push(poly);

      if ((4 === poly.length) && (CV.isContourConvex(poly))) {

        if (CV.minEdgeLength(poly) >= minLength) {
          candidates.push(poly);
        }
      }
    }
  }

  return candidates;
};

AR.Detector.prototype.clockwiseCorners = function (candidates) {
  var len = candidates.length,
    dx1, dx2, dy1, dy2, swap, i;

  for (i = 0; i < len; ++i) {
    dx1 = candidates[i][1].x - candidates[i][0].x;
    dy1 = candidates[i][1].y - candidates[i][0].y;
    dx2 = candidates[i][2].x - candidates[i][0].x;
    dy2 = candidates[i][2].y - candidates[i][0].y;

    if ((dx1 * dy2 - dy1 * dx2) < 0) {
      swap = candidates[i][1];
      candidates[i][1] = candidates[i][3];
      candidates[i][3] = swap;
    }
  }

  return candidates;
};

AR.Detector.prototype.notTooNear = function (candidates, minDist) {
  var notTooNear = [],
    len = candidates.length,
    dist, dx, dy, i, j, k;

  for (i = 0; i < len; ++i) {

    for (j = i + 1; j < len; ++j) {
      dist = 0;

      for (k = 0; k < 4; ++k) {
        dx = candidates[i][k].x - candidates[j][k].x;
        dy = candidates[i][k].y - candidates[j][k].y;

        dist += dx * dx + dy * dy;
      }

      if ((dist / 4) < (minDist * minDist)) {

        if (CV.perimeter(candidates[i]) < CV.perimeter(candidates[j])) {
          candidates[i].tooNear = true;
        } else {
          candidates[j].tooNear = true;
        }
      }
    }
  }

  for (i = 0; i < len; ++i) {
    if (!candidates[i].tooNear) {
      notTooNear.push(candidates[i]);
    }
  }

  return notTooNear;
};

AR.Detector.prototype.findMarkers = function (imageSrc, candidates, warpSize) {
  var markers = [],
    len = candidates.length,
    candidate, marker, i;

  for (i = 0; i < len; ++i) {
    candidate = candidates[i];

    CV.warp(imageSrc, this.homography, candidate, warpSize);

    CV.threshold(this.homography, this.homography, CV.otsu(this.homography));

    marker = this.getMarker(this.homography, candidate);
    if (marker) {
      markers.push(marker);
    }
  }

  return markers;
};

AR.Detector.prototype.getMarker = function (imageSrc, candidate) {
  var markSize = this.dictionary.markSize;
  var width = (imageSrc.width / markSize) >>> 0,
    minZero = (width * width) >> 1,
    bits = [],
    rotations = [],
    square, inc, i, j;

  for (i = 0; i < markSize; ++i) {
    inc = (0 === i || (markSize - 1) === i) ? 1 : (markSize - 1);

    for (j = 0; j < markSize; j += inc) {
      square = {
        x: j * width,
        y: i * width,
        width: width,
        height: width
      };
      if (CV.countNonZero(imageSrc, square) > minZero) {
        return null;
      }
    }
  }

  for (i = 0; i < markSize - 2; ++i) {
    bits[i] = [];

    for (j = 0; j < markSize - 2; ++j) {
      square = {
        x: (j + 1) * width,
        y: (i + 1) * width,
        width: width,
        height: width
      };

      bits[i][j] = CV.countNonZero(imageSrc, square) > minZero ? 1 : 0;
    }
  }

  rotations[0] = bits;

  var foundMin = null;
  var rot = 0;
  for (i = 0; i < 4; i++) {
    var found = this.dictionary.find(rotations[i]);
    if (found && (foundMin === null || found.distance < foundMin.distance)) {
      foundMin = found;
      rot = i;
      if (foundMin.distance === 0)
        break;
    }
    rotations[i + 1] = this.rotate(rotations[i]);
  }

  if (foundMin)
    return new AR.Marker(foundMin.id, this.rotate2(candidate, 4 - rot), foundMin.distance);

  return null;
};

AR.Detector.prototype.rotate = function (src) {
  var dst = [],
    len = src.length,
    i, j;

  for (i = 0; i < len; ++i) {
    dst[i] = [];
    for (j = 0; j < src[i].length; ++j) {
      dst[i][j] = src[src[i].length - j - 1][i];
    }
  }

  return dst;
};

AR.Detector.prototype.rotate2 = function (src, rotation) {
  var dst = [],
    len = src.length,
    i;

  for (i = 0; i < len; ++i) {
    dst[i] = src[(rotation + i) % len];
  }

  return dst;
};

export { AR };
```

## 11. `client/core/tracker/vendor/aruco_4x4_1000.js`

```js
/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.
                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)
Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

//Dictionary extracted from https://github.com/opencv/opencv_contrib/blob/4.x/modules/aruco/src/predefined_dictionaries.hpp

import { AR } from './aruco.js';
AR.DICTIONARIES['ARUCO_4X4_1000'] = {
  nBits: 16,
  tau: null,
  codeList: [[181,50],[15,154],[51,45],[153,70],[84,158],[121,205],[158,46],[196,242],[254,218],[207,86],[249,145],[17,167],[14,183],[42,15],[36,177],[38,62],[70,101],[102,0],[108,94],[118,175],[134,139],[176,43],[204,213],[221,130],[254,71],[148,113],[172,228],[165,84],[33,35],[52,111],[68,21],[87,178],[158,207],[240,203],[8,174],[9,41],[24,117],[4,255],[13,246],[28,90],[23,24],[42,40],[50,140],[56,178],[36,232],[46,235],[45,63],[75,100],[80,46],[80,19],[81,148],[85,104],[93,65],[95,151],[104,1],[104,103],[97,36],[97,233],[107,18],[111,229],[103,223],[126,27],[128,160],[131,68],[139,162],[147,122],[132,108],[133,42],[133,156],[156,137],[159,161],[187,124],[188,4],[182,91],[191,200],[183,171],[202,31],[201,98],[217,88],[211,213],[204,152],[199,160],[197,55],[233,93],[249,37],[251,187],[238,42],[247,77],[53,117],[138,173],[118,23],[10,207],[6,75],[45,193],[73,216],[67,244],[79,54],[79,211],[105,228],[112,199],[122,110],[180,234],[237,79],[252,231],[254,166],[0,37],[0,67],[10,136],[10,134],[2,111],[0,28],[0,151],[8,55],[10,49],[9,198],[11,1],[9,251],[11,88],[16,130],[24,45],[16,120],[16,115],[18,116],[18,177],[26,249],[19,6],[12,14],[12,241],[4,51],[12,159],[14,242],[14,253],[7,76],[15,164],[7,47],[5,181],[15,145],[7,219],[30,228],[20,57],[29,128],[21,200],[31,139],[21,186],[29,177],[32,128],[40,233],[34,162],[40,83],[42,240],[34,247],[41,64],[33,70],[41,185],[43,156],[43,178],[56,202],[56,46],[48,7],[56,231],[58,73],[58,101],[50,93],[59,136],[57,29],[59,211],[38,71],[39,128],[47,170],[45,20],[37,222],[37,83],[47,119],[52,72],[60,168],[60,65],[52,13],[52,251],[54,154],[61,224],[53,106],[61,9],[61,237],[63,196],[63,108],[55,206],[61,92],[61,118],[55,176],[63,23],[63,255],[72,229],[66,104],[74,45],[65,96],[73,81],[65,221],[75,223],[88,79],[90,72],[88,22],[80,93],[90,250],[90,181],[81,35],[91,138],[89,25],[81,53],[76,105],[70,193],[78,11],[68,95],[78,89],[77,131],[77,125],[71,216],[71,115],[92,133],[94,68],[86,43],[92,187],[85,195],[95,110],[95,235],[93,18],[85,94],[98,112],[98,21],[97,194],[107,32],[99,69],[107,92],[107,91],[120,12],[122,207],[120,127],[121,128],[113,229],[113,116],[121,182],[113,211],[123,51],[100,106],[102,168],[110,167],[110,145],[101,34],[109,203],[103,141],[109,49],[126,128],[126,226],[126,141],[116,210],[124,50],[126,53],[117,171],[119,5],[127,43],[125,218],[127,146],[128,117],[128,243],[129,166],[137,237],[129,252],[152,166],[154,32],[145,67],[153,249],[145,147],[155,212],[132,9],[132,107],[134,196],[142,100],[134,26],[133,78],[141,203],[133,103],[133,175],[133,215],[135,179],[156,225],[156,242],[148,23],[149,0],[149,162],[157,35],[159,98],[157,82],[149,218],[160,197],[170,205],[162,216],[162,87],[169,61],[169,87],[171,82],[163,54],[163,89],[176,244],[184,18],[176,191],[178,157],[187,237],[185,114],[185,150],[164,195],[172,210],[174,177],[165,130],[175,101],[165,123],[175,250],[180,100],[188,98],[180,129],[182,160],[190,238],[190,13],[188,217],[190,248],[181,40],[183,9],[183,210],[192,234],[192,25],[192,253],[200,211],[202,90],[193,77],[201,180],[193,87],[195,152],[195,29],[216,128],[216,239],[218,43],[208,30],[209,5],[211,173],[219,167],[196,201],[204,120],[205,69],[197,11],[207,207],[220,172],[212,2],[220,99],[212,39],[212,245],[214,120],[222,184],[221,230],[213,93],[221,189],[223,29],[226,202],[234,107],[224,180],[226,56],[226,212],[227,34],[225,216],[240,3],[242,204],[248,246],[241,73],[243,234],[241,156],[249,245],[241,59],[236,141],[238,201],[230,15],[228,247],[231,96],[239,232],[237,178],[229,21],[239,209],[244,134],[252,1],[246,195],[244,124],[252,147],[245,66],[253,152],[245,61],[2,189],[0,225],[2,226],[2,174],[8,120],[0,116],[8,158],[8,209],[8,125],[10,50],[10,222],[2,81],[1,162],[3,128],[11,131],[11,75],[11,39],[11,239],[9,182],[9,89],[9,147],[11,248],[3,217],[3,241],[16,196],[24,171],[26,160],[26,4],[26,108],[26,174],[18,137],[16,23],[26,243],[25,64],[17,2],[17,43],[17,207],[27,34],[19,46],[17,21],[19,187],[12,32],[12,201],[12,220],[12,54],[6,20],[6,114],[13,97],[5,13],[13,143],[15,224],[15,73],[7,133],[5,144],[13,51],[15,150],[15,118],[20,96],[28,141],[20,218],[28,115],[30,148],[30,186],[22,217],[30,61],[22,251],[29,233],[29,254],[31,159],[40,139],[32,175],[34,14],[34,169],[42,141],[42,163],[42,239],[40,144],[40,59],[42,88],[34,51],[33,160],[33,2],[33,165],[33,199],[43,3],[35,103],[41,48],[41,210],[43,25],[43,155],[43,151],[56,40],[56,165],[58,134],[50,1],[56,159],[50,210],[58,153],[58,213],[57,232],[59,193],[51,67],[59,231],[49,154],[51,144],[59,158],[36,196],[44,74],[44,173],[44,207],[44,103],[38,234],[46,229],[44,112],[46,18],[46,209],[46,57],[37,100],[37,231],[47,204],[45,188],[45,113],[37,213],[37,155],[39,16],[47,124],[39,242],[39,58],[47,182],[39,211],[47,179],[39,31],[60,75],[54,192],[54,238],[62,233],[52,184],[60,20],[60,82],[52,114],[52,126],[52,191],[62,113],[62,83],[61,140],[53,162],[53,46],[53,45],[55,172],[53,112],[55,250],[63,241],[63,219],[72,196],[72,233],[74,194],[74,65],[66,235],[72,19],[74,216],[66,253],[74,23],[73,99],[67,110],[65,58],[73,177],[65,61],[75,146],[75,155],[67,63],[88,34],[80,170],[88,39],[82,200],[82,132],[82,10],[90,15],[88,152],[88,92],[80,219],[80,247],[90,244],[81,236],[81,66],[81,13],[91,3],[83,235],[81,118],[89,113],[81,147],[83,249],[91,179],[83,151],[76,76],[68,75],[76,35],[70,140],[78,39],[70,144],[78,212],[69,206],[69,229],[69,39],[79,193],[71,5],[69,52],[69,114],[92,200],[92,14],[84,235],[86,137],[86,67],[94,231],[92,112],[84,178],[94,121],[86,243],[93,163],[93,242],[85,29],[93,157],[87,252],[87,210],[95,115],[104,45],[104,195],[104,135],[106,74],[98,105],[96,185],[104,255],[106,220],[106,218],[106,62],[106,81],[106,49],[98,215],[97,204],[107,130],[107,227],[105,58],[97,158],[97,149],[97,117],[105,95],[105,55],[99,218],[112,2],[120,99],[112,79],[114,202],[122,173],[112,123],[122,20],[122,249],[122,211],[122,187],[121,226],[113,41],[123,103],[113,208],[121,57],[115,48],[115,185],[115,83],[115,255],[108,136],[100,9],[108,67],[102,6],[102,131],[100,176],[100,218],[110,159],[103,200],[111,238],[109,59],[111,210],[116,128],[124,171],[126,104],[126,2],[124,156],[116,54],[124,17],[126,222],[126,182],[118,219],[125,196],[125,138],[117,109],[119,136],[119,32],[119,65],[117,56],[117,190],[125,155],[119,87],[136,40],[128,172],[136,13],[136,103],[130,78],[138,161],[130,43],[128,24],[136,249],[128,157],[138,156],[130,49],[138,117],[130,151],[129,9],[129,235],[129,7],[139,40],[139,172],[131,46],[131,229],[129,80],[137,50],[139,122],[139,150],[131,125],[144,135],[154,252],[146,245],[145,170],[147,65],[147,37],[155,235],[153,52],[145,247],[155,218],[147,86],[132,66],[140,129],[140,79],[134,72],[134,166],[142,3],[134,227],[134,111],[142,175],[132,94],[132,119],[134,250],[142,30],[142,55],[135,10],[143,138],[143,38],[135,33],[135,13],[133,114],[135,62],[156,67],[158,97],[148,88],[148,248],[156,50],[148,118],[148,177],[148,221],[148,155],[156,219],[158,156],[158,210],[150,25],[158,177],[149,105],[159,109],[151,43],[149,182],[149,185],[157,61],[157,87],[168,236],[168,37],[162,172],[162,2],[170,102],[170,143],[170,231],[168,48],[168,122],[168,246],[168,147],[162,20],[170,52],[162,114],[170,242],[162,241],[161,64],[169,10],[161,38],[169,197],[169,207],[161,52],[169,18],[161,250],[171,152],[163,247],[176,6],[176,69],[184,141],[178,132],[184,240],[184,85],[178,118],[186,145],[178,113],[185,192],[185,66],[185,42],[179,140],[179,202],[187,102],[179,15],[177,218],[187,20],[187,246],[179,19],[164,104],[172,44],[172,161],[172,235],[172,199],[164,103],[166,192],[174,224],[166,35],[173,232],[165,204],[167,236],[173,124],[165,26],[165,145],[173,25],[165,151],[180,109],[190,203],[188,58],[188,245],[190,189],[190,243],[181,37],[181,143],[183,104],[191,228],[189,254],[189,157],[181,245],[181,243],[191,176],[183,90],[191,62],[183,57],[191,213],[183,29],[191,53],[183,127],[200,1],[192,165],[194,130],[200,189],[194,252],[202,145],[194,91],[201,68],[193,42],[195,192],[201,122],[193,185],[201,117],[193,247],[203,177],[208,108],[216,135],[208,175],[218,196],[210,12],[218,9],[208,48],[216,148],[208,58],[208,182],[208,117],[210,118],[218,93],[218,53],[210,23],[217,2],[211,232],[211,229],[209,154],[209,246],[209,81],[219,20],[211,62],[211,211],[196,96],[204,167],[198,66],[198,71],[206,231],[196,92],[204,29],[204,53],[198,188],[205,168],[197,12],[197,228],[197,194],[205,45],[205,89],[205,149],[197,147],[199,95],[212,197],[222,136],[214,36],[222,236],[214,226],[222,198],[222,35],[220,220],[220,26],[212,17],[222,84],[214,148],[222,157],[221,129],[213,165],[215,172],[215,102],[223,169],[213,220],[221,31],[223,240],[226,72],[226,232],[226,7],[224,93],[234,245],[235,38],[235,237],[225,82],[225,126],[233,219],[248,6],[240,238],[248,161],[250,0],[250,194],[240,155],[250,244],[250,60],[242,252],[242,189],[242,147],[241,96],[249,236],[241,70],[249,225],[243,72],[243,174],[243,193],[243,139],[243,167],[241,115],[241,151],[243,244],[251,50],[228,7],[230,77],[236,85],[237,192],[237,133],[239,162],[231,78],[229,213],[239,80],[244,34],[244,137],[244,41],[246,106],[254,11],[254,111],[244,149],[244,53],[244,31],[246,176],[245,232],[245,197],[253,35],[255,192],[247,204],[247,233],[245,188],[253,246],[245,217],[253,151],[253,63],[255,156],[255,90],[247,254],[255,17],[247,191]]
};
```

## 12. `client/core/tracker/vendor/LICENSE-js-aruco2.txt`

```
Copyright (c) 2020 Damiano Falcioni
Copyright (c) 2011 Juan Mellado

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


ArUco
====
BSD License


OpenCV
======
 IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

 By downloading, copying, installing or using the software you agree to this license.
 If you do not agree to this license, do not download, install,
 copy or use the software.


                          License Agreement
                For Open Source Computer Vision Library

Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
Copyright (C) 2009, Willow Garage Inc., all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistribution's of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistribution's in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * The name of the copyright holders may not be used to endorse or promote products
    derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall the Intel Corporation or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.


AForge.NET
========
GNU LESSER GENERAL PUBLIC LICENSE

Version 3, 29 June 2007

Copyright � 2007 Free Software Foundation, Inc. <http://fsf.org/>

Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.

This version of the GNU Lesser General Public License incorporates the terms and conditions of version 3 of the GNU General Public License, supplemented by the additional permissions listed below.

0. Additional Definitions.
As used herein, �this License� refers to version 3 of the GNU Lesser General Public License, and the �GNU GPL� refers to version 3 of the GNU General Public License.

�The Library� refers to a covered work governed by this License, other than an Application or a Combined Work as defined below.

An �Application� is any work that makes use of an interface provided by the Library, but which is not otherwise based on the Library. Defining a subclass of a class defined by the Library is deemed a mode of using an interface provided by the Library.

A �Combined Work� is a work produced by combining or linking an Application with the Library. The particular version of the Library with which the Combined Work was made is also called the �Linked Version�.

The �Minimal Corresponding Source� for a Combined Work means the Corresponding Source for the Combined Work, excluding any source code for portions of the Combined Work that, considered in isolation, are based on the Application, and not on the Linked Version.

The �Corresponding Application Code� for a Combined Work means the object code and/or source code for the Application, including any data and utility programs needed for reproducing the Combined Work from the Application, but excluding the System Libraries of the Combined Work.

1. Exception to Section 3 of the GNU GPL.
You may convey a covered work under sections 3 and 4 of this License without being bound by section 3 of the GNU GPL.

2. Conveying Modified Versions.
If you modify a copy of the Library, and, in your modifications, a facility refers to a function or data to be supplied by an Application that uses the facility (other than as an argument passed when the facility is invoked), then you may convey a copy of the modified version:

a) under this License, provided that you make a good faith effort to ensure that, in the event an Application does not supply the function or data, the facility still operates, and performs whatever part of its purpose remains meaningful, or
b) under the GNU GPL, with none of the additional permissions of this License applicable to that copy.
3. Object Code Incorporating Material from Library Header Files.
The object code form of an Application may incorporate material from a header file that is part of the Library. You may convey such object code under terms of your choice, provided that, if the incorporated material is not limited to numerical parameters, data structure layouts and accessors, or small macros, inline functions and templates (ten or fewer lines in length), you do both of the following:

a) Give prominent notice with each copy of the object code that the Library is used in it and that the Library and its use are covered by this License.
b) Accompany the object code with a copy of the GNU GPL and this license document.
4. Combined Works.
You may convey a Combined Work under terms of your choice that, taken together, effectively do not restrict modification of the portions of the Library contained in the Combined Work and reverse engineering for debugging such modifications, if you also do each of the following:

a) Give prominent notice with each copy of the Combined Work that the Library is used in it and that the Library and its use are covered by this License.
b) Accompany the Combined Work with a copy of the GNU GPL and this license document.
c) For a Combined Work that displays copyright notices during execution, include the copyright notice for the Library among these notices, as well as a reference directing the user to the copies of the GNU GPL and this license document.
d) Do one of the following:
0) Convey the Minimal Corresponding Source under the terms of this License, and the Corresponding Application Code in a form suitable for, and under terms that permit, the user to recombine or relink the Application with a modified version of the Linked Version to produce a modified Combined Work, in the manner specified by section 6 of the GNU GPL for conveying Corresponding Source.
1) Use a suitable shared library mechanism for linking with the Library. A suitable mechanism is one that (a) uses at run time a copy of the Library already present on the user's computer system, and (b) will operate properly with a modified version of the Library that is interface-compatible with the Linked Version.
e) Provide Installation Information, but only if you would otherwise be required to provide such information under section 6 of the GNU GPL, and only to the extent that such information is necessary to install and execute a modified version of the Combined Work produced by recombining or relinking the Application with a modified version of the Linked Version. (If you use option 4d0, the Installation Information must accompany the Minimal Corresponding Source and Corresponding Application Code. If you use option 4d1, you must provide the Installation Information in the manner specified by section 6 of the GNU GPL for conveying Corresponding Source.)
5. Combined Libraries.
You may place library facilities that are a work based on the Library side by side in a single library together with other library facilities that are not Applications and are not covered by this License, and convey such a combined library under terms of your choice, if you do both of the following:

a) Accompany the combined library with a copy of the same work based on the Library, uncombined with any other library facilities, conveyed under the terms of this License.
b) Give prominent notice with the combined library that part of it is a work based on the Library, and explaining where to find the accompanying uncombined form of the same work.
6. Revised Versions of the GNU Lesser General Public License.
The Free Software Foundation may publish revised and/or new versions of the GNU Lesser General Public License from time to time. Such new versions will be similar in spirit to the present version, but may differ in detail to address new problems or concerns.

Each version is given a distinguishing version number. If the Library as you received it specifies that a certain numbered version of the GNU Lesser General Public License �or any later version� applies to it, you have the option of following the terms and conditions either of that published version or of any later version published by the Free Software Foundation. If the Library as you received it does not specify a version number of the GNU Lesser General Public License, you may choose any version of the GNU Lesser General Public License ever published by the Free Software Foundation.

If the Library as you received it specifies that a proxy can decide whether future versions of the GNU Lesser General Public License shall apply, that proxy's public statement of acceptance of any version is permanent authorization for you to choose that version for the Library.


StackBoxBlur
==========
Copyright (c) 2010 Mario Klingemann

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
```

## 13. `client/core/handMotion.js`

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
```

## 14. `client/core/input.js`

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
import { HandFusionFilter } from './tracker/fusionKF.js';

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
      fused: false,                // Stage C: pos/vel come from the fusion KF
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
  /**
   * @param {number} slot 1|2
   * @param {object} net the NetClient
   * @param {object} [tracker] the CameraTracker (Stage C fusion source);
   *   measurements for this slot's marker are routed into the fusion KF.
   * @param {object} [settings] the Settings singleton (trackMode read here)
   */
  constructor(slot, net, tracker, settings) {
    this.slot = slot;
    this.net = net;
    this.tracker = tracker || null;
    this.settings = settings || null;
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

    // Stage C: fusion KF (camera measurements + IMU control input). The
    // camera tracker pushes measurements for this slot's marker (id slot-1)
    // into `fuseCameraMeas`; `sample()` predicts at IMU cadence.
    this.fusion = new HandFusionFilter();
    this._fusionSub = tracker
      ? {
          meas: (m) => this.fuseCameraMeas(m),
          home: () => {},                    // capture already calls reset()
          reset: () => this.fusion.reset(),
        }
      : null;
    if (this._fusionSub) tracker.subscribe(slot, this._fusionSub);
    this._fusionRead = { p: [0, 0, 0], v: [0, 0, 0] }; // KF read scratch
    this._usingFusion = false;
    this._fusionLastAge = Infinity;

    // Raw newest sample for the debug overlay graphs (shared, no alloc).
    this.latestRaw = {
      t: 0, quat: new THREE.Quaternion(), acc: new THREE.Vector3(),
      gyro: new THREE.Vector3(), swing: 0, flags: 0, battery: 65535,
    };
    this._captureQ = new THREE.Quaternion();
  }

  /** Detach from the tracker (called when the slot's source is replaced). */
  dispose() {
    if (this.tracker && this._fusionSub) {
      this.tracker.unsubscribe(this.slot, this._fusionSub);
      this._fusionSub = null;
    }
  }

  /**
   * Stage C: one camera measurement (CALIBRATED frame, epoch-ms t, possibly
   * late vs the IMU timeline — the KF rewinds and replays).
   */
  fuseCameraMeas(m) {
    this.fusion.fuse(m.t, m.x, m.y, m.z, m.sx, m.sy, m.sz);
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
      // estimate restarts from the neutral (zero-offset) pose — and so does
      // the Stage C fusion KF (its measurements live in the same frame).
      this.handFilter.reset();
      this.fusion.reset();
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
      this._applyFusion(nowMs, hand);
    } else {
      hand.pos.set(0, 0, 0);
      hand.vel.set(0, 0, 0);
      hand.accel.set(0, 0, 0);
      hand.zupt = false;
      hand.fused = false;
      this._usingFusion = false;
    }
    return true;
  }

  /**
   * Stage C: run the fusion KF at IMU cadence — predict with the Stage B
   * corrected accel as the control input — and, when the marker is fresh
   * (< 400 ms) and trackMode is 'fusion', replace the hand estimate with the
   * fused state. On dropout the Stage B filter adopts the last fused state
   * so the racket never teleports between the two estimators.
   */
  _applyFusion(nowMs, hand) {
    if (!this.tracker || !this.settings) {
      hand.fused = false;
      this._usingFusion = false;
      return;
    }
    // Control input: bias-corrected + dead-zoned accel (zero while still —
    // exactly the "IMU says nothing moved" signal the KF wants).
    this.fusion.predict(nowMs, hand.accel.x, hand.accel.y, hand.accel.z);

    const fusionMode = this.settings.at('trackMode', this.slot - 1) === 'fusion';
    const age = this.tracker.lastMeasAgeMs(this.slot);
    const ok = fusionMode && this.fusion.started && age < 400;

    if (ok) {
      const f = this.fusion.read(this._fusionRead);
      hand.pos.set(f.p[0], f.p[1], f.p[2]);
      hand.vel.set(f.v[0], f.v[1], f.v[2]);
      hand.fused = true;
      this._usingFusion = true;
    } else {
      hand.fused = false;
      if (this._usingFusion && this.fusion.started) {
        // Fusion just dropped out: hand the last fused state to the Stage B
        // filter — the hand estimate continues where the camera left it.
        const f = this.fusion.read(this._fusionRead);
        this.handFilter.adopt(f.p[0], f.p[1], f.p[2], f.v[0], f.v[1], f.v[2]);
        hand.pos.set(f.p[0], f.p[1], f.p[2]);
        hand.vel.set(f.v[0], f.v[1], f.v[2]);
      }
      this._usingFusion = false;
    }
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

## 15. `client/core/settings.js`

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
  /* Stage C: webcam marker tracking (per player).
   *   trackMode    'imu' (Stage B hand motion) | 'fusion' (webcam + IMU KF)
   *   camDevice    preferred webcam deviceId ('' = default camera)
   *   camMarkerMm  printed marker BLACK-square side in mm (must match print)
   *   camHome      per-slot ready pose {cxn, cyn, sideN} | null (normalized,
   *                resolution/fov independent) */
  trackMode: ['imu', 'imu'],
  camDevice: '',
  camMarkerMm: 100,
  camHome: [null, null],
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

## 16. `client/main.js`

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
import { CameraTracker } from './core/tracker/cameraTracker.js';
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
/* Stage C: webcam marker tracker (fusion positioning source)           */
/* ------------------------------------------------------------------ */

const tracker = new CameraTracker(settings);

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
    const src = entry && entry.source;
    const hf = src && src.handFilter;
    const rig = sport.rackets.get(slot);
    if (!hf || !hf.started) return null;
    const input = entry.input;
    const hand = input && input.hand;
    const fused = !!(hand && hand.fused);
    return {
      // When the fusion KF is the active output, show its state (the Stage B
      // filter keeps integrating underneath and is shown via adopt/fallback).
      pos: fused ? hand.pos : hf.pos,
      vel: fused ? hand.vel : hf.vel,
      accel: hf.accel, raw: hf.raw,
      zupt: hf.zupt, stillMs: hf.stillMs,
      fused,
      limited: src.limited,
      mode: settings.at('posMode', slot - 1) || 'assist',
      assistValid: rig ? rig.assistValid : false,
      assistTarget: rig ? rig.assistTarget : null,
      racket: rig ? rig.posSmooth : null,
    };
  },
  // Stage C: per-slot webcam tracking status for the overlay
  getTrackInfo: (slot) => ({
    status: tracker.status(slot),
    ageMs: tracker.lastMeasAgeMs(slot),
    fps: tracker.fps,
    hasHome: tracker.hasHome(slot),
  }),
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
        if (entry.source && entry.source.dispose) entry.source.dispose();
        entry.source = new ControllerInputSource(slot, net, tracker, settings);
        net.registerSource(slot, entry.source);
        // 'wait' only on the join transition — the heartbeat lobby refresh
        // (every 2 s) must not stomp the frame loop's live/stale chip.
        const s = net.slots.find((x) => x.slot === slot);
        hud.setSlot(slot, { name: s?.name || `P${slot}`, state: 'wait' });
      }
    } else if (!(entry.source instanceof DemoInputSource)) {
      net.registerSource(slot, null);
      if (entry.source && entry.source.dispose) entry.source.dispose();
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
  tracker,
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
window.__RP = { net, slots, sport, state: () => state, settings, hud, tracker };

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

## 17. `client/sports/tableTennis/TableTennisSport.js`

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
    const trackMode = this.ctx.settings.at('trackMode', slot - 1) || 'imu';
    const hand = entry && entry.input && entry.input.hand;
    const fusion = src && src.fusion;
    return {
      active: !!(hand && hand.active),
      fused: !!(hand && hand.fused),
      trackMode,
      mode: posMode,
      limited: !!src.limited,
      pos: { x: +hf.pos.x.toFixed(3), y: +hf.pos.y.toFixed(3), z: +hf.pos.z.toFixed(3) },
      vel: { x: +hf.vel.x.toFixed(3), y: +hf.vel.y.toFixed(3), z: +hf.vel.z.toFixed(3) },
      accel: { x: +hf.accel.x.toFixed(2), y: +hf.accel.y.toFixed(2), z: +hf.accel.z.toFixed(2) },
      raw: { x: +hf.raw.x.toFixed(2), y: +hf.raw.y.toFixed(2), z: +hf.raw.z.toFixed(2) },
      zupt: hf.zupt,
      stillMs: Math.round(hf.stillMs),
      omega: +hf.cfg.omega.toFixed(2),
      // Stage C: fusion KF snapshot (positions in the calibrated frame)
      fusion: fusion ? {
        started: fusion.started,
        p: { x: +fusion.p[0].toFixed(3), y: +fusion.p[1].toFixed(3), z: +fusion.p[2].toFixed(3) },
        v: { x: +fusion.v[0].toFixed(3), y: +fusion.v[1].toFixed(3), z: +fusion.v[2].toFixed(3) },
        nMeas: fusion.nMeas,
        nRejected: fusion.nRejected,
        nClamped: fusion.nClamped,
      } : null,
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

## 18. `client/ui/debugOverlay.js`

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
  constructor({ getRendererInfo, getSources, getResScale, getBallInfo, getHandInfo, getTrackInfo }) {
    this.getRendererInfo = getRendererInfo;
    this.getSources = getSources;
    this.getResScale = getResScale;
    this.getBallInfo = getBallInfo || null;
    this.getHandInfo = getHandInfo || null;
    this.getTrackInfo = getTrackInfo || null;

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
      // assist target the position blends toward. Stage C adds the webcam
      // TRACK line (fusion status + measurement age) below it.
      let handH = 0;
      const hand = this.getHandInfo ? this.getHandInfo(slot) : null;
      if (hand) {
        handH = 14;
        const p = hand.pos, v = hand.vel, a = hand.accel, r = hand.raw;
        ctx.fillStyle = hand.fused ? '#ffd98a' : '#9fe8b7';
        ctx.fillText(
          `HAND[${hand.mode}${hand.fused ? '+fusion' : ''}] ` +
          `p(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)}) ` +
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
      const track = this.getTrackInfo ? this.getTrackInfo(slot) : null;
      if (track) {
        handH += 14;
        const age = isFinite(track.ageMs) ? `${Math.round(track.ageMs)}ms` : '—';
        ctx.fillStyle = track.status === 'track' ? '#7fe3ff' : '#8b96a8';
        ctx.fillText(
          `TRACK ${track.status.toUpperCase().padEnd(6)} age ${age}  ` +
          `cam ${track.fps ? track.fps.toFixed(0) + 'fps' : 'off'}  ` +
          `${track.hasHome ? 'home ✓' : 'no home'}`,
          x0, y0 + (hand ? 24 : 10),
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

## 19. `client/ui/screens.js`

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
   * @param {object} ctx { settings, hud, audio, net, tracker, hooks }
   * tracker: the Stage C CameraTracker (for the Tracking setup screen).
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
    this.trackingEl = this._buildTracking();
    for (const el of [this.menuEl, this.lobbyEl, this.settingsEl, this.pauseEl, this.machineEl, this.matchEndEl, this.trackingEl]) {
      this.root.appendChild(el);
    }

    this._ipChoice = '';
    this._urlOverride = '';
  }

  _show(el) {
    for (const e of [this.menuEl, this.lobbyEl, this.settingsEl, this.pauseEl, this.matchEndEl, this.trackingEl]) {
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
      addSelectAt(`P${slot} tracking`, 'trackMode', i, [
        ['imu', 'Phone motion only'],
        ['fusion', 'Webcam + phone (fusion)'],
      ]);
      addRangeAt(`P${slot} hand sensitivity`, 'handSens', i, 0.4, 2, 0.05, (v) => `${v.toFixed(2)}x`);
      addRangeAt(`P${slot} return-to-center`, 'handReturn', i, 0, 2, 0.05, (v) => (v === 0 ? 'off (park anywhere)' : `${v.toFixed(2)}`));
      addRangeAt(`P${slot} motion dead zone`, 'handDead', i, 0, 1, 0.05, (v) => `${v.toFixed(2)} m/s²`);
    }

    const note = $('div', 'settings-note',
      'Assist and swing parameters drive the hit model from Phase 3. Hand motion (Stage B) moves the racket with your physical hand: hold the phone still and the position holds (no drift); recalibrating re-centres it. A phone without a gyroscope falls back to tilt-only tracking automatically.');

    const actions = $('div', 'lobby-actions');
    const tracking = $('button', 'btn', 'Tracking & camera…');
    tracking.addEventListener('click', () => { this.ctx.audio.click(); this.openTracking(); });
    const reset = $('button', 'btn danger', 'Reset defaults');
    reset.addEventListener('click', () => {
      s.reset();
      this.openSettings(); // rebuild values
      this.ctx.hooks.applyGraphics();
    });
    const close = $('button', 'btn primary', 'Done');
    close.addEventListener('click', () => { this.ctx.audio.click(); this.closeSettings(); });
    actions.append(tracking, reset, close);

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
  /* Tracking setup (Stage C: webcam + marker fusion)                  */
  /* ---------------------------------------------------------------- */

  _buildTracking() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'panel');
    card.style.maxWidth = 'min(96vw, 980px)';
    card.appendChild($('h3', 'section', 'TRACKING SETUP'));

    const note = $('div', 'settings-note',
      'Print the two markers from <a href="/markers.html" target="_blank" style="color:#8cf">/markers.html</a> ' +
      '(exact size matters — set the mm below to your print), tape one on the BACK of each phone ' +
      '(facing the webcam), put the webcam behind/next to the screen facing the players, then per player: ' +
      'hold the phone in your ready pose (same pose as calibration) and press Capture. ' +
      'Fusion tracking (Settings → P1/P2 tracking) then fuses the webcam marker with the phone IMU.');

    const layout = $('div', 'track-layout');
    layout.style.cssText = 'display:flex;gap:16px;align-items:stretch;flex-wrap:wrap;margin:10px 0;';

    // ---- preview (video + overlay canvas) ----
    const previewBox = $('div', 'track-preview');
    previewBox.style.cssText =
      'position:relative;flex:1 1 420px;min-width:320px;min-height:280px;background:#000;border-radius:10px;overflow:hidden;';
    this.trackVideoBox = previewBox;
    const overlay = document.createElement('canvas');
    overlay.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;';
    previewBox.appendChild(overlay);
    this.trackOverlay = overlay;
    this.trackOverlayCtx = overlay.getContext('2d');
    const statusChip = $('div', 'track-status');
    statusChip.style.cssText =
      'position:absolute;left:8px;top:8px;padding:4px 10px;border-radius:8px;background:rgba(0,0,0,.55);' +
      'font:12px/1.4 monospace;color:#cde;pointer-events:none;white-space:pre;';
    previewBox.appendChild(statusChip);
    this.trackStatusEl = statusChip;

    // ---- controls column ----
    const col = $('div', 'track-controls');
    col.style.cssText = 'flex:1 1 300px;min-width:280px;display:flex;flex-direction:column;gap:10px;';

    // camera device + start/stop
    const camRow = $('div', 'row2');
    this.trackDevSel = $('select');
    this.trackDevSel.style.flex = '1';
    this.trackDevSel.addEventListener('change', () => {
      this.ctx.settings.set('camDevice', this.trackDevSel.value);
      this._restartCamera();
    });
    this.trackStartBtn = $('button', 'btn primary', 'Start camera');
    this.trackStartBtn.addEventListener('click', () => { this.ctx.audio.click(); this._restartCamera(); });
    camRow.append(this.trackDevSel, this.trackStartBtn);
    col.appendChild(camRow);

    // marker size
    const sizeRow = $('div', 'row2');
    sizeRow.appendChild($('span', null, 'Marker print size (mm)'));
    const sizeIn = $('input');
    sizeIn.type = 'number';
    sizeIn.min = 30; sizeIn.max = 400; sizeIn.step = 5;
    sizeIn.value = this.ctx.settings.get('camMarkerMm');
    sizeIn.style.width = '90px';
    sizeIn.addEventListener('change', () => {
      const v = Math.max(30, Math.min(400, parseFloat(sizeIn.value) || 100));
      sizeIn.value = v;
      this.ctx.settings.set('camMarkerMm', v);
    });
    sizeRow.appendChild(sizeIn);
    col.appendChild(sizeRow);

    // per-slot rows
    this.trackSlotRows = [];
    for (const slot of [1, 2]) {
      const row = $('div', 'track-slot');
      row.style.cssText = 'border:1px solid #345;border-radius:10px;padding:8px 10px;display:flex;flex-direction:column;gap:6px;';
      const head = $('div', 'row2');
      head.style.justifyContent = 'space-between';
      head.appendChild($('span', null, `P${slot} — marker ID ${slot - 1}`));
      const chip = $('span', 'val', 'off');
      head.appendChild(chip);
      const btns = $('div', 'row2');
      btns.style.gap = '8px';
      const cap = $('button', 'btn small primary', 'Capture ready pose');
      cap.addEventListener('click', async () => {
        cap.disabled = true;
        chip.textContent = 'capturing…';
        const ok = await this.ctx.tracker.captureHome(slot);
        cap.disabled = false;
        chip.textContent = ok ? 'captured ✓' : 'no marker seen';
        this.ctx.audio.click(ok ? 'good' : 'bad');
      });
      const clr = $('button', 'btn small', 'Clear');
      clr.addEventListener('click', () => {
        this.ctx.tracker.clearHome(slot);
        chip.textContent = 'off';
      });
      btns.append(cap, clr);
      row.append(head, btns);
      col.appendChild(row);
      this.trackSlotRows.push(chip);
    }

    col.appendChild($('div', 'settings-note',
      'Status: track = marker locked + home set · stale/lost = marker not seen · nocal = seen, capture the ready pose. ' +
      'The camera keeps running after you leave this screen.'));
    layout.append(previewBox, col);
    card.append(note, layout);

    const actions = $('div', 'lobby-actions');
    const back = $('button', 'btn primary', 'Back');
    back.addEventListener('click', () => { this.ctx.audio.click(); this.closeTracking(); });
    actions.appendChild(back);
    card.appendChild(actions);

    el.appendChild(card);
    return el;
  }

  openTracking() {
    this._prevScreen = this.trackingEl.classList.contains('hidden')
      ? (this.settingsEl.classList.contains('hidden') ? this.menuEl : this.settingsEl)
      : this._prevScreen;
    this._show(this.trackingEl);
    this.ctx.tracker.mountPreview(this.trackVideoBox);
    this._refreshTrackDevices();
    if (!this._trackRaf) this._trackLoop();
  }

  closeTracking() {
    this.ctx.tracker.mountPreview(null); // back to the hidden home (keeps running)
    if (this._trackRaf) { cancelAnimationFrame(this._trackRaf); this._trackRaf = 0; }
    this._show(this._prevScreen || this.menuEl);
  }

  async _refreshTrackDevices() {
    const s = this.ctx.settings;
    const sel = this.trackDevSel;
    sel.innerHTML = '';
    const def = $('option', null, 'Default camera');
    def.value = '';
    sel.appendChild(def);
    try {
      for (const d of await this.ctx.tracker.listDevices()) {
        if (!d.id) continue;
        const o = $('option', null, d.label);
        o.value = d.id;
        sel.appendChild(o);
      }
    } catch { /* labels need permission; the default still works */ }
    sel.value = s.get('camDevice') || '';
    if (sel.value !== (s.get('camDevice') || '')) sel.value = '';
  }

  async _restartCamera() {
    const t = this.ctx.tracker;
    t.stop();
    try {
      await t.start();
      this.trackStartBtn.textContent = 'Restart camera';
      this._refreshTrackDevices();
    } catch (e) {
      this.trackStartBtn.textContent = 'Start camera';
      this.trackStatusEl.textContent = `ERR: ${e.message || e}`;
    }
  }

  /** Overlay loop while the tracking screen is visible (~10 Hz is plenty). */
  _trackLoop() {
    if (this.trackingEl.classList.contains('hidden')) { this._trackRaf = 0; return; }
    this._trackRaf = requestAnimationFrame(() => this._trackLoop());
    const t = this.ctx.tracker;
    const st = t.debugState();

    // status chip
    const lines = [
      `${st.running ? (st.fps.toFixed(1) + ' fps · ' + st.w + '×' + st.h) : 'camera off'}`,
      st.err ? `ERR: ${st.err}` : '',
      `P1 ${st.statuses[0]}   P2 ${st.statuses[1]}`,
    ];
    this.trackStatusEl.textContent = lines.filter(Boolean).join('\n');
    for (let i = 0; i < 2; i++) {
      this.trackSlotRows[i].textContent = `${st.statuses[i]}${st.hasHome[i] ? ' · home ✓' : ''}`;
    }

    // overlay boxes (marker quads + home crosshairs), mirrored like the video
    const cv = this.trackOverlay;
    if (cv.width !== cv.clientWidth || cv.height !== cv.clientHeight) {
      cv.width = cv.clientWidth || 640;
      cv.height = cv.clientHeight || 400;
    }
    const ctx = this.trackOverlayCtx;
    ctx.clearRect(0, 0, cv.width, cv.height);
    const W = cv.width, H = cv.height;
    const colors = ['#41e0d0', '#ffb454'];
    for (let i = 0; i < 2; i++) {
      const home = this.ctx.settings.get('camHome')[i];
      if (home) {
        // home crosshair — remember the preview is mirrored (x -> 1-x)
        const hx = (1 - home.cxn) * W, hy = home.cyn * H;
        const r = (home.sideN * W) * 0.7;
        ctx.strokeStyle = colors[i];
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(hx - r, hy); ctx.lineTo(hx + r, hy);
        ctx.moveTo(hx, hy - r); ctx.lineTo(hx, hy + r);
        ctx.stroke();
      }
    }
    for (const m of st.lastMarkers) {
      const age = Date.now() - m.t;
      if (age > 500) continue;
      const side = m.sideN * W;
      const x = (1 - m.cxn) * W - side / 2;   // mirrored preview
      const y = m.cyn * H - side / 2;
      ctx.strokeStyle = colors[m.id] || '#fff';
      ctx.globalAlpha = age > 150 ? 0.45 : 1;
      ctx.lineWidth = 2;
      ctx.strokeRect(x, y, side, side);
      ctx.font = '12px monospace';
      ctx.fillStyle = ctx.strokeStyle;
      ctx.fillText(`P${m.id + 1}`, x, y - 4);
      ctx.globalAlpha = 1;
    }
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

## 20. `package.json`

```json
{
  "name": "realplay",
  "version": "0.1.0",
  "private": true,
  "type": "module",
  "description": "REALPLAY — your phone is the racket. 3D table tennis with phone-motion controllers (Phase 1: setup, lobby, controller streaming, split-screen debug view).",
  "scripts": {
    "dev": "concurrently -k -n server,web -c blue,green \"npm:dev:server\" \"npm:dev:client\"",
    "dev:server": "node server/index.js --port 8081",
    "dev:client": "vite --port 5173",
    "dev:https": "cross-env RP_HTTPS=1 concurrently -k -n server,web -c blue,green \"npm:dev:server\" \"npm:dev:client:https\"",
    "dev:client:https": "cross-env RP_HTTPS=1 vite --port 5173",
    "build": "vite build",
    "start": "node server/index.js --port 8080 --static dist",
    "smoke": "node server/smoke.js",
    "test:fusion": "node tests/fusion.test.mjs"
  },
  "dependencies": {
    "js-aruco2": "^2.0.0",
    "qrcode": "^1.5.4",
    "three": "^0.180.0",
    "ws": "^8.18.0"
  },
  "devDependencies": {
    "@vitejs/plugin-basic-ssl": "^1.2.0",
    "concurrently": "^9.1.0",
    "cross-env": "^7.0.3",
    "vite": "^5.4.11"
  },
  "engines": {
    "node": ">=18.17"
  }
}
```

## 21. `package-lock.json`

```json
{
  "name": "realplay",
  "version": "0.1.0",
  "lockfileVersion": 3,
  "requires": true,
  "packages": {
    "": {
      "name": "realplay",
      "version": "0.1.0",
      "dependencies": {
        "js-aruco2": "^2.0.0",
        "qrcode": "^1.5.4",
        "three": "^0.180.0",
        "ws": "^8.18.0"
      },
      "devDependencies": {
        "@vitejs/plugin-basic-ssl": "^1.2.0",
        "concurrently": "^9.1.0",
        "cross-env": "^7.0.3",
        "vite": "^5.4.11"
      },
      "engines": {
        "node": ">=18.17"
      }
    },
    "node_modules/@esbuild/aix-ppc64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/aix-ppc64/-/aix-ppc64-0.21.5.tgz",
      "integrity": "sha512-1SDgH6ZSPTlggy1yI6+Dbkiz8xzpHJEVAlF/AM1tHPLsf5STom9rwtjE4hKAF20FfXXNTFqEYXyJNWh1GiZedQ==",
      "cpu": [
        "ppc64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "aix"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/android-arm": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/android-arm/-/android-arm-0.21.5.tgz",
      "integrity": "sha512-vCPvzSjpPHEi1siZdlvAlsPxXl7WbOVUBBAowWug4rJHb68Ox8KualB+1ocNvT5fjv6wpkX6o/iEpbDrf68zcg==",
      "cpu": [
        "arm"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "android"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/android-arm64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/android-arm64/-/android-arm64-0.21.5.tgz",
      "integrity": "sha512-c0uX9VAUBQ7dTDCjq+wdyGLowMdtR/GoC2U5IYk/7D1H1JYC0qseD7+11iMP2mRLN9RcCMRcjC4YMclCzGwS/A==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "android"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/android-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/android-x64/-/android-x64-0.21.5.tgz",
      "integrity": "sha512-D7aPRUUNHRBwHxzxRvp856rjUHRFW1SdQATKXH2hqA0kAZb1hKmi02OpYRacl0TxIGz/ZmXWlbZgjwWYaCakTA==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "android"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/darwin-arm64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/darwin-arm64/-/darwin-arm64-0.21.5.tgz",
      "integrity": "sha512-DwqXqZyuk5AiWWf3UfLiRDJ5EDd49zg6O9wclZ7kUMv2WRFr4HKjXp/5t8JZ11QbQfUS6/cRCKGwYhtNAY88kQ==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "darwin"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/darwin-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/darwin-x64/-/darwin-x64-0.21.5.tgz",
      "integrity": "sha512-se/JjF8NlmKVG4kNIuyWMV/22ZaerB+qaSi5MdrXtd6R08kvs2qCN4C09miupktDitvh8jRFflwGFBQcxZRjbw==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "darwin"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/freebsd-arm64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/freebsd-arm64/-/freebsd-arm64-0.21.5.tgz",
      "integrity": "sha512-5JcRxxRDUJLX8JXp/wcBCy3pENnCgBR9bN6JsY4OmhfUtIHe3ZW0mawA7+RDAcMLrMIZaf03NlQiX9DGyB8h4g==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "freebsd"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/freebsd-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/freebsd-x64/-/freebsd-x64-0.21.5.tgz",
      "integrity": "sha512-J95kNBj1zkbMXtHVH29bBriQygMXqoVQOQYA+ISs0/2l3T9/kj42ow2mpqerRBxDJnmkUDCaQT/dfNXWX/ZZCQ==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "freebsd"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-arm": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-arm/-/linux-arm-0.21.5.tgz",
      "integrity": "sha512-bPb5AHZtbeNGjCKVZ9UGqGwo8EUu4cLq68E95A53KlxAPRmUyYv2D6F0uUI65XisGOL1hBP5mTronbgo+0bFcA==",
      "cpu": [
        "arm"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-arm64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-arm64/-/linux-arm64-0.21.5.tgz",
      "integrity": "sha512-ibKvmyYzKsBeX8d8I7MH/TMfWDXBF3db4qM6sy+7re0YXya+K1cem3on9XgdT2EQGMu4hQyZhan7TeQ8XkGp4Q==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-ia32": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-ia32/-/linux-ia32-0.21.5.tgz",
      "integrity": "sha512-YvjXDqLRqPDl2dvRODYmmhz4rPeVKYvppfGYKSNGdyZkA01046pLWyRKKI3ax8fbJoK5QbxblURkwK/MWY18Tg==",
      "cpu": [
        "ia32"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-loong64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-loong64/-/linux-loong64-0.21.5.tgz",
      "integrity": "sha512-uHf1BmMG8qEvzdrzAqg2SIG/02+4/DHB6a9Kbya0XDvwDEKCoC8ZRWI5JJvNdUjtciBGFQ5PuBlpEOXQj+JQSg==",
      "cpu": [
        "loong64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-mips64el": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-mips64el/-/linux-mips64el-0.21.5.tgz",
      "integrity": "sha512-IajOmO+KJK23bj52dFSNCMsz1QP1DqM6cwLUv3W1QwyxkyIWecfafnI555fvSGqEKwjMXVLokcV5ygHW5b3Jbg==",
      "cpu": [
        "mips64el"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-ppc64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-ppc64/-/linux-ppc64-0.21.5.tgz",
      "integrity": "sha512-1hHV/Z4OEfMwpLO8rp7CvlhBDnjsC3CttJXIhBi+5Aj5r+MBvy4egg7wCbe//hSsT+RvDAG7s81tAvpL2XAE4w==",
      "cpu": [
        "ppc64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-riscv64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-riscv64/-/linux-riscv64-0.21.5.tgz",
      "integrity": "sha512-2HdXDMd9GMgTGrPWnJzP2ALSokE/0O5HhTUvWIbD3YdjME8JwvSCnNGBnTThKGEB91OZhzrJ4qIIxk/SBmyDDA==",
      "cpu": [
        "riscv64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-s390x": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-s390x/-/linux-s390x-0.21.5.tgz",
      "integrity": "sha512-zus5sxzqBJD3eXxwvjN1yQkRepANgxE9lgOW2qLnmr8ikMTphkjgXu1HR01K4FJg8h1kEEDAqDcZQtbrRnB41A==",
      "cpu": [
        "s390x"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/linux-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/linux-x64/-/linux-x64-0.21.5.tgz",
      "integrity": "sha512-1rYdTpyv03iycF1+BhzrzQJCdOuAOtaqHTWJZCWvijKD2N5Xu0TtVC8/+1faWqcP9iBCWOmjmhoH94dH82BxPQ==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/netbsd-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/netbsd-x64/-/netbsd-x64-0.21.5.tgz",
      "integrity": "sha512-Woi2MXzXjMULccIwMnLciyZH4nCIMpWQAs049KEeMvOcNADVxo0UBIQPfSmxB3CWKedngg7sWZdLvLczpe0tLg==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "netbsd"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/openbsd-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/openbsd-x64/-/openbsd-x64-0.21.5.tgz",
      "integrity": "sha512-HLNNw99xsvx12lFBUwoT8EVCsSvRNDVxNpjZ7bPn947b8gJPzeHWyNVhFsaerc0n3TsbOINvRP2byTZ5LKezow==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "openbsd"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/sunos-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/sunos-x64/-/sunos-x64-0.21.5.tgz",
      "integrity": "sha512-6+gjmFpfy0BHU5Tpptkuh8+uw3mnrvgs+dSPQXQOv3ekbordwnzTVEb4qnIvQcYXq6gzkyTnoZ9dZG+D4garKg==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "sunos"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/win32-arm64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/win32-arm64/-/win32-arm64-0.21.5.tgz",
      "integrity": "sha512-Z0gOTd75VvXqyq7nsl93zwahcTROgqvuAcYDUr+vOv8uHhNSKROyU961kgtCD1e95IqPKSQKH7tBTslnS3tA8A==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/win32-ia32": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/win32-ia32/-/win32-ia32-0.21.5.tgz",
      "integrity": "sha512-SWXFF1CL2RVNMaVs+BBClwtfZSvDgtL//G/smwAc5oVK/UPu2Gu9tIaRgFmYFFKrmg3SyAjSrElf0TiJ1v8fYA==",
      "cpu": [
        "ia32"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@esbuild/win32-x64": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/@esbuild/win32-x64/-/win32-x64-0.21.5.tgz",
      "integrity": "sha512-tQd/1efJuzPC6rCFwEvLtci/xNFcTZknmXs98FYDfGE4wP9ClFV98nyKrzJKVPMhdDnjzLhdUyMX4PsQAPjwIw==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ],
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/@rollup/rollup-android-arm-eabi": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-android-arm-eabi/-/rollup-android-arm-eabi-4.68.3.tgz",
      "integrity": "sha512-mqfypgH/liYjaSCoPI4erckUcROgZtmFUUmyOJ05oyayi/tVqOEYAzWP7ufqCoCT3xB31Fw4XIVO/jX42hYvKA==",
      "cpu": [
        "arm"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "android"
      ]
    },
    "node_modules/@rollup/rollup-android-arm64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-android-arm64/-/rollup-android-arm64-4.68.3.tgz",
      "integrity": "sha512-5tY+GC/1gpt79cKS2DQzu6iD824j6BmI4q5Zb6qftZ4kWqIQmvR3DD4eJ2Gclu6/nVMYDk2IVpqxypWOiAuWJg==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "android"
      ]
    },
    "node_modules/@rollup/rollup-android-x64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-android-x64/-/rollup-android-x64-4.68.3.tgz",
      "integrity": "sha512-nDBcQ/o3PMmGxmAzGBJFn+n1rb83Va9WSdFIq1AHmlEF3tuibVsyJ3kG5x13r7VIy6WI8SvzCdHVNSlSDIdj8w==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "android"
      ]
    },
    "node_modules/@rollup/rollup-darwin-arm64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-darwin-arm64/-/rollup-darwin-arm64-4.68.3.tgz",
      "integrity": "sha512-dra5zl0QWKkIkaMgNDNPC19gkMlPAfAf41yWlkGztXMIvUQrX9XWZFQRw7abhH3nks+XZGQpybINAxT2pKdFew==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "darwin"
      ]
    },
    "node_modules/@rollup/rollup-darwin-x64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-darwin-x64/-/rollup-darwin-x64-4.68.3.tgz",
      "integrity": "sha512-xo5xeo/AjEB/ArzQjGn0JRDZuybZicQ5O3VPfko6ocH1li/qNrN4p5+R/WgjavzRz6UwAOaqsau7VvRQA6i6AA==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "darwin"
      ]
    },
    "node_modules/@rollup/rollup-freebsd-arm64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-freebsd-arm64/-/rollup-freebsd-arm64-4.68.3.tgz",
      "integrity": "sha512-zdckkcxHDEEKyDbC1t21DeaaiKGymxbkBU9c3zyfaaz+8N96ItJw4nmzHhZF+Mukgt906fnf2tsc90pVyStuHA==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "freebsd"
      ]
    },
    "node_modules/@rollup/rollup-freebsd-x64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-freebsd-x64/-/rollup-freebsd-x64-4.68.3.tgz",
      "integrity": "sha512-9oXqI7g/ycEz7TyicdGBC4sL6PfWMXAx8u15f/nd7lYkCLJd8U8VdBfbpc4SSnRBU/e7aPALve3mhzasGDeCYg==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "freebsd"
      ]
    },
    "node_modules/@rollup/rollup-linux-arm-gnueabihf": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-arm-gnueabihf/-/rollup-linux-arm-gnueabihf-4.68.3.tgz",
      "integrity": "sha512-hbgT4NlaOELE3D5UUzSqF+OFgwz4Scvo2SQD9nZJUFjdIsg9Kg3FabQauUxkHKY9N5yjNXfXRRmiKNN4JpLmFA==",
      "cpu": [
        "arm"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-arm-musleabihf": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-arm-musleabihf/-/rollup-linux-arm-musleabihf-4.68.3.tgz",
      "integrity": "sha512-yECcXZ/Bl+JVRhLFgld42rYrXgPdE0LtySGAmmmH6OtkaJM17A4cWG/eCzDymMVb52/jZjW0R54YrTHj4xXlSA==",
      "cpu": [
        "arm"
      ],
      "dev": true,
      "libc": [
        "musl"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-arm64-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-arm64-gnu/-/rollup-linux-arm64-gnu-4.68.3.tgz",
      "integrity": "sha512-q4Q4/B7wyqM+TFqY5tJYQlaz5+ksGopcRmMeMdu4q6eaPK7N1PymTjIaH5LL32mMJBqIzpRof7BhlYLim5JgNQ==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-arm64-musl": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-arm64-musl/-/rollup-linux-arm64-musl-4.68.3.tgz",
      "integrity": "sha512-Ich3TGl/7GqhQ6G6jVPzgGQGJjd3E5hECxSVanPYchy6Vwcq+WI8wG/gmEMilHNy+ZYCR6U8EN50EejG3TaIgw==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "libc": [
        "musl"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-loong64-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-loong64-gnu/-/rollup-linux-loong64-gnu-4.68.3.tgz",
      "integrity": "sha512-URBmOhwv4Y0Hi5Hhxc3cmqZLtQlx55DOs5PUhhITDC3zu0qwAfhcNAFAIwAhOUdA6NFVEck07CNZX6cBEs1vTg==",
      "cpu": [
        "loong64"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-loong64-musl": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-loong64-musl/-/rollup-linux-loong64-musl-4.68.3.tgz",
      "integrity": "sha512-PTdJXuZplRyaE3ECArlbZubI2LlUHWm0oCipCexnE/8LM+L7qIqUiNbU0piBxt46EB5zxXX+cpmDvHX4Z3WwcQ==",
      "cpu": [
        "loong64"
      ],
      "dev": true,
      "libc": [
        "musl"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-ppc64-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-ppc64-gnu/-/rollup-linux-ppc64-gnu-4.68.3.tgz",
      "integrity": "sha512-IlSQPDExSm9YdtrWeZIHj87GPXI9lYygRvHVwQ1o9t91JHIiB2CihrvNIASpc/zI28atqm04jS3proaQuaJMww==",
      "cpu": [
        "ppc64"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-ppc64-musl": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-ppc64-musl/-/rollup-linux-ppc64-musl-4.68.3.tgz",
      "integrity": "sha512-O2ssyDcBTZkH5v2P/9ZaXJreL0hCKvUA5UwuHn0jCKeMLKi4GhspZJ1I5FT5QsHtJ/dPx4pup7pg/Lw0emeY0Q==",
      "cpu": [
        "ppc64"
      ],
      "dev": true,
      "libc": [
        "musl"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-riscv64-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-riscv64-gnu/-/rollup-linux-riscv64-gnu-4.68.3.tgz",
      "integrity": "sha512-fesqagxcnc87lH2+aBWne/tFCmQVIGH+YezjY1wQwbUODuTBBlhHoyxju6umwn8LnjHVanjpoeaQdLsXP8JQSQ==",
      "cpu": [
        "riscv64"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-riscv64-musl": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-riscv64-musl/-/rollup-linux-riscv64-musl-4.68.3.tgz",
      "integrity": "sha512-WAjFDtLW3HjJ2RAytKoWdFwlK/hW8rJHrjhRZFw14nR+c9JN//WIlSBkyXUjEYCNOFs2sGLNutOEPQG4iVYHgA==",
      "cpu": [
        "riscv64"
      ],
      "dev": true,
      "libc": [
        "musl"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-s390x-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-s390x-gnu/-/rollup-linux-s390x-gnu-4.68.3.tgz",
      "integrity": "sha512-+fZiTeHV/1fy/14dRqE4thjYwUOABL6qQjxElOEQg2oHLkCamO8dxhHphVgD4+pFxtVEUIWxsGVduPtwnAlvbg==",
      "cpu": [
        "s390x"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-x64-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-x64-gnu/-/rollup-linux-x64-gnu-4.68.3.tgz",
      "integrity": "sha512-pfn4Zd9Ic3lEaSwW3GsQhTIDd7929C+eiEkOutsSCT2IHD6rDLblwmMQkgeGOso0oDo9Jk/ERtFdtFr9ElLs+g==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "libc": [
        "glibc"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-linux-x64-musl": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-linux-x64-musl/-/rollup-linux-x64-musl-4.68.3.tgz",
      "integrity": "sha512-eUXzaeNQbKXKNqseL3g/usUqOPmRu+DbQsvwr9MXMdV+VQFUNb9bcWSNAAs/TRV95PCbFK46kLzHEOhFSzujsg==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "libc": [
        "musl"
      ],
      "license": "MIT",
      "optional": true,
      "os": [
        "linux"
      ]
    },
    "node_modules/@rollup/rollup-openbsd-x64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-openbsd-x64/-/rollup-openbsd-x64-4.68.3.tgz",
      "integrity": "sha512-OiUclE7QDU4H/hr3ACRHhVGKWl+gNx+6Emignfq0hmNmzzTkZnT79wzB9NXKd0QH16S4t/wVqlAFBfC17nkZ5A==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "openbsd"
      ]
    },
    "node_modules/@rollup/rollup-openharmony-arm64": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-openharmony-arm64/-/rollup-openharmony-arm64-4.68.3.tgz",
      "integrity": "sha512-s6EclDWmk6zvIfk1yixgYthk7xHNx0xu5cEF5RDpS4IHNTpfswRX6xOXVG5cHF5xWn0B6EpDGARhPQ520KTlsA==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "openharmony"
      ]
    },
    "node_modules/@rollup/rollup-win32-arm64-msvc": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-win32-arm64-msvc/-/rollup-win32-arm64-msvc-4.68.3.tgz",
      "integrity": "sha512-G2xnmxGnU8XHLwtjnnXNRcLrub+Af3nm9VQ1QzL2aEiAL7woBdRgTCZBslJl8umA+YMqdEFpUh9kJB/Vyzk2wQ==",
      "cpu": [
        "arm64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ]
    },
    "node_modules/@rollup/rollup-win32-ia32-msvc": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-win32-ia32-msvc/-/rollup-win32-ia32-msvc-4.68.3.tgz",
      "integrity": "sha512-Y/4tApNVrMUYWbAHsyRWJgwuuJOfhTNhA2B04Xpa6+G9btkPZ9kU3XagUKzPTwF3hg36NpQk748JsnoTVho67Q==",
      "cpu": [
        "ia32"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ]
    },
    "node_modules/@rollup/rollup-win32-x64-gnu": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-win32-x64-gnu/-/rollup-win32-x64-gnu-4.68.3.tgz",
      "integrity": "sha512-i2vH/OA5BcW+TdNg12ZTRC4rn+vILaPMP9CvHqnv+QEpMjBHGGNLo4k4EnEqzjYvTD6x1Xark9WhsQbH4bTzUQ==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ]
    },
    "node_modules/@rollup/rollup-win32-x64-msvc": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/@rollup/rollup-win32-x64-msvc/-/rollup-win32-x64-msvc-4.68.3.tgz",
      "integrity": "sha512-ADb1Uz2MNM0sxVpAlMNdQG16Hw4sgW1aqVL7NeqBMpcZeiHb192d7W6shZdMQs33D+w9ihDYBBPsYqqc4MlD8w==",
      "cpu": [
        "x64"
      ],
      "dev": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "win32"
      ]
    },
    "node_modules/@types/estree": {
      "version": "1.0.10",
      "resolved": "https://registry.npmjs.org/@types/estree/-/estree-1.0.10.tgz",
      "integrity": "sha512-lTyOMnOcmRBzdCv6/4zUZCrEhV0xrqQiUZBZtVcbmLHBXKLnN8q89v/wKRF/Q0HbwVsqbIG3o5HDoFl9sduM+g==",
      "dev": true,
      "license": "MIT"
    },
    "node_modules/@vitejs/plugin-basic-ssl": {
      "version": "1.2.0",
      "resolved": "https://registry.npmjs.org/@vitejs/plugin-basic-ssl/-/plugin-basic-ssl-1.2.0.tgz",
      "integrity": "sha512-mkQnxTkcldAzIsomk1UuLfAu9n+kpQ3JbHcpCp7d2Oo6ITtji8pHS3QToOWjhPFvNQSnhlkAjmGbhv2QvwO/7Q==",
      "dev": true,
      "license": "MIT",
      "engines": {
        "node": ">=14.21.3"
      },
      "peerDependencies": {
        "vite": "^3.0.0 || ^4.0.0 || ^5.0.0 || ^6.0.0"
      }
    },
    "node_modules/ansi-regex": {
      "version": "5.0.1",
      "resolved": "https://registry.npmjs.org/ansi-regex/-/ansi-regex-5.0.1.tgz",
      "integrity": "sha512-quJQXlTSUGL2LH9SUXo8VwsY4soanhgo6LNSm84E1LBcE8s3O0wpdiRzyR9z/ZZJMlMWv37qOOb9pdJlMUEKFQ==",
      "license": "MIT",
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/ansi-styles": {
      "version": "4.3.0",
      "resolved": "https://registry.npmjs.org/ansi-styles/-/ansi-styles-4.3.0.tgz",
      "integrity": "sha512-zbB9rCJAT1rbjiVDb2hqKFHNYLxgtk8NURxZ3IZwD3F6NtxbXZQCnnSi1Lkx+IDohdPlFp222wVALIheZJQSEg==",
      "license": "MIT",
      "dependencies": {
        "color-convert": "^2.0.1"
      },
      "engines": {
        "node": ">=8"
      },
      "funding": {
        "url": "https://github.com/chalk/ansi-styles?sponsor=1"
      }
    },
    "node_modules/camelcase": {
      "version": "5.3.1",
      "resolved": "https://registry.npmjs.org/camelcase/-/camelcase-5.3.1.tgz",
      "integrity": "sha512-L28STB170nwWS63UjtlEOE3dldQApaJXZkOI1uMFfzf3rRuPegHaHesyee+YxQ+W6SvRDQV6UrdOdRiR153wJg==",
      "license": "MIT",
      "engines": {
        "node": ">=6"
      }
    },
    "node_modules/chalk": {
      "version": "4.1.2",
      "resolved": "https://registry.npmjs.org/chalk/-/chalk-4.1.2.tgz",
      "integrity": "sha512-oKnbhFyRIXpUuez8iBMmyEa4nbj4IOQyuhc/wy9kY7/WVPcwIO9VA668Pu8RkO7+0G76SLROeyw9CpQ061i4mA==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "ansi-styles": "^4.1.0",
        "supports-color": "^7.1.0"
      },
      "engines": {
        "node": ">=10"
      },
      "funding": {
        "url": "https://github.com/chalk/chalk?sponsor=1"
      }
    },
    "node_modules/chalk/node_modules/supports-color": {
      "version": "7.2.0",
      "resolved": "https://registry.npmjs.org/supports-color/-/supports-color-7.2.0.tgz",
      "integrity": "sha512-qpCAvRl9stuOHveKsn7HncJRvv501qIacKzQlO/+Lwxc9+0q2wLyv4Dfvt80/DPn2pqOBsJdDiogXGR9+OvwRw==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "has-flag": "^4.0.0"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/cliui": {
      "version": "8.0.1",
      "resolved": "https://registry.npmjs.org/cliui/-/cliui-8.0.1.tgz",
      "integrity": "sha512-BSeNnyus75C4//NQ9gQt1/csTXyo/8Sb+afLAkzAptFuMsod9HFokGNudZpi/oQV73hnVK+sR+5PVRMd+Dr7YQ==",
      "dev": true,
      "license": "ISC",
      "dependencies": {
        "string-width": "^4.2.0",
        "strip-ansi": "^6.0.1",
        "wrap-ansi": "^7.0.0"
      },
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/color-convert": {
      "version": "2.0.1",
      "resolved": "https://registry.npmjs.org/color-convert/-/color-convert-2.0.1.tgz",
      "integrity": "sha512-RRECPsj7iu/xb5oKYcsFHSppFNnsj/52OVTRKb4zP5onXwVF3zVmmToNcOfGC+CRDpfK/U584fMg38ZHCaElKQ==",
      "license": "MIT",
      "dependencies": {
        "color-name": "~1.1.4"
      },
      "engines": {
        "node": ">=7.0.0"
      }
    },
    "node_modules/color-name": {
      "version": "1.1.4",
      "resolved": "https://registry.npmjs.org/color-name/-/color-name-1.1.4.tgz",
      "integrity": "sha512-dOy+3AuW3a2wNbZHIuMZpTcgjGuLU/uBL/ubcZF9OXbDo8ff4O8yVp5Bf0efS8uEoYo5q4Fx7dY9OgQGXgAsQA==",
      "license": "MIT"
    },
    "node_modules/concurrently": {
      "version": "9.2.5",
      "resolved": "https://registry.npmjs.org/concurrently/-/concurrently-9.2.5.tgz",
      "integrity": "sha512-YMZlvWrTPQEhjKp60XSmMGt60+pffbv6DyRnOMvVO98+J7aF8w8Ui9FhdcOwvSixc/kyR6SoVvmejcn/2lZBqg==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "chalk": "4.1.2",
        "rxjs": "7.8.2",
        "shell-quote": "1.12.0",
        "supports-color": "8.1.1",
        "tree-kill": "1.2.2",
        "yargs": "17.7.2"
      },
      "bin": {
        "conc": "dist/bin/concurrently.js",
        "concurrently": "dist/bin/concurrently.js"
      },
      "engines": {
        "node": ">=18"
      },
      "funding": {
        "url": "https://github.com/open-cli-tools/concurrently?sponsor=1"
      }
    },
    "node_modules/cross-env": {
      "version": "7.0.3",
      "resolved": "https://registry.npmjs.org/cross-env/-/cross-env-7.0.3.tgz",
      "integrity": "sha512-+/HKd6EgcQCJGh2PSjZuUitQBQynKor4wrFbRg4DtAgS1aWO+gU52xpH7M9ScGgXSYmAVS9bIJ8EzuaGw0oNAw==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "cross-spawn": "^7.0.1"
      },
      "bin": {
        "cross-env": "src/bin/cross-env.js",
        "cross-env-shell": "src/bin/cross-env-shell.js"
      },
      "engines": {
        "node": ">=10.14",
        "npm": ">=6",
        "yarn": ">=1"
      }
    },
    "node_modules/cross-spawn": {
      "version": "7.0.6",
      "resolved": "https://registry.npmjs.org/cross-spawn/-/cross-spawn-7.0.6.tgz",
      "integrity": "sha512-uV2QOWP2nWzsy2aMp8aRibhi9dlzF5Hgh5SHaB9OiTGEyDTiJJyx0uy51QXdyWbtAHNua4XJzUKca3OzKUd3vA==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "path-key": "^3.1.0",
        "shebang-command": "^2.0.0",
        "which": "^2.0.1"
      },
      "engines": {
        "node": ">= 8"
      }
    },
    "node_modules/decamelize": {
      "version": "1.2.0",
      "resolved": "https://registry.npmjs.org/decamelize/-/decamelize-1.2.0.tgz",
      "integrity": "sha512-z2S+W9X73hAUUki+N+9Za2lBlun89zigOyGrsax+KUQ6wKW4ZoWpEYBkGhQjwAjjDCkWxhY0VKEhk8wzY7F5cA==",
      "license": "MIT",
      "engines": {
        "node": ">=0.10.0"
      }
    },
    "node_modules/dijkstrajs": {
      "version": "1.0.3",
      "resolved": "https://registry.npmjs.org/dijkstrajs/-/dijkstrajs-1.0.3.tgz",
      "integrity": "sha512-qiSlmBq9+BCdCA/L46dw8Uy93mloxsPSbwnm5yrKn2vMPiy8KyAskTF6zuV/j5BMsmOGZDPs7KjU+mjb670kfA==",
      "license": "MIT"
    },
    "node_modules/emoji-regex": {
      "version": "8.0.0",
      "resolved": "https://registry.npmjs.org/emoji-regex/-/emoji-regex-8.0.0.tgz",
      "integrity": "sha512-MSjYzcWNOA0ewAHpz0MxpYFvwg6yjy1NG3xteoqz644VCo/RPgnr1/GGt+ic3iJTzQ8Eu3TdM14SawnVUmGE6A==",
      "license": "MIT"
    },
    "node_modules/esbuild": {
      "version": "0.21.5",
      "resolved": "https://registry.npmjs.org/esbuild/-/esbuild-0.21.5.tgz",
      "integrity": "sha512-mg3OPMV4hXywwpoDxu3Qda5xCKQi+vCTZq8S9J/EpkhB2HzKXq4SNFZE3+NK93JYxc8VMSep+lOUSC/RVKaBqw==",
      "dev": true,
      "hasInstallScript": true,
      "license": "MIT",
      "bin": {
        "esbuild": "bin/esbuild"
      },
      "engines": {
        "node": ">=12"
      },
      "optionalDependencies": {
        "@esbuild/aix-ppc64": "0.21.5",
        "@esbuild/android-arm": "0.21.5",
        "@esbuild/android-arm64": "0.21.5",
        "@esbuild/android-x64": "0.21.5",
        "@esbuild/darwin-arm64": "0.21.5",
        "@esbuild/darwin-x64": "0.21.5",
        "@esbuild/freebsd-arm64": "0.21.5",
        "@esbuild/freebsd-x64": "0.21.5",
        "@esbuild/linux-arm": "0.21.5",
        "@esbuild/linux-arm64": "0.21.5",
        "@esbuild/linux-ia32": "0.21.5",
        "@esbuild/linux-loong64": "0.21.5",
        "@esbuild/linux-mips64el": "0.21.5",
        "@esbuild/linux-ppc64": "0.21.5",
        "@esbuild/linux-riscv64": "0.21.5",
        "@esbuild/linux-s390x": "0.21.5",
        "@esbuild/linux-x64": "0.21.5",
        "@esbuild/netbsd-x64": "0.21.5",
        "@esbuild/openbsd-x64": "0.21.5",
        "@esbuild/sunos-x64": "0.21.5",
        "@esbuild/win32-arm64": "0.21.5",
        "@esbuild/win32-ia32": "0.21.5",
        "@esbuild/win32-x64": "0.21.5"
      }
    },
    "node_modules/escalade": {
      "version": "3.2.0",
      "resolved": "https://registry.npmjs.org/escalade/-/escalade-3.2.0.tgz",
      "integrity": "sha512-WUj2qlxaQtO4g6Pq5c29GTcWGDyd8itL8zTlipgECz3JesAiiOKotd8JU6otB3PACgG6xkJUyVhboMS+bje/jA==",
      "dev": true,
      "license": "MIT",
      "engines": {
        "node": ">=6"
      }
    },
    "node_modules/find-up": {
      "version": "4.1.0",
      "resolved": "https://registry.npmjs.org/find-up/-/find-up-4.1.0.tgz",
      "integrity": "sha512-PpOwAdQ/YlXQ2vj8a3h8IipDuYRi3wceVQQGYWxNINccq40Anw7BlsEXCMbt1Zt+OLA6Fq9suIpIWD0OsnISlw==",
      "license": "MIT",
      "dependencies": {
        "locate-path": "^5.0.0",
        "path-exists": "^4.0.0"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/fsevents": {
      "version": "2.3.3",
      "resolved": "https://registry.npmjs.org/fsevents/-/fsevents-2.3.3.tgz",
      "integrity": "sha512-5xoDfX+fL7faATnagmWPpbFtwh/R77WmMMqqHGS65C3vvB0YHrgF+B1YmZ3441tMj5n63k0212XNoJwzlhffQw==",
      "dev": true,
      "hasInstallScript": true,
      "license": "MIT",
      "optional": true,
      "os": [
        "darwin"
      ],
      "engines": {
        "node": "^8.16.0 || ^10.6.0 || >=11.0.0"
      }
    },
    "node_modules/get-caller-file": {
      "version": "2.0.5",
      "resolved": "https://registry.npmjs.org/get-caller-file/-/get-caller-file-2.0.5.tgz",
      "integrity": "sha512-DyFP3BM/3YHTQOCUL/w0OZHR0lpKeGrxotcHWcqNEdnltqFwXVfhEBQ94eIo34AfQpo0rGki4cyIiftY06h2Fg==",
      "license": "ISC",
      "engines": {
        "node": "6.* || 8.* || >= 10.*"
      }
    },
    "node_modules/has-flag": {
      "version": "4.0.0",
      "resolved": "https://registry.npmjs.org/has-flag/-/has-flag-4.0.0.tgz",
      "integrity": "sha512-EykJT/Q1KjTWctppgIAgfSO0tKVuZUjhgMr17kqTumMl6Afv3EISleU7qZUzoXDFTAHTDC4NOoG/ZxU3EvlMPQ==",
      "dev": true,
      "license": "MIT",
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/is-fullwidth-code-point": {
      "version": "3.0.0",
      "resolved": "https://registry.npmjs.org/is-fullwidth-code-point/-/is-fullwidth-code-point-3.0.0.tgz",
      "integrity": "sha512-zymm5+u+sCsSWyD9qNaejV3DFvhCKclKdizYaJUuHA83RLjb7nSuGnddCHGv0hk+KY7BMAlsWeK4Ueg6EV6XQg==",
      "license": "MIT",
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/isexe": {
      "version": "2.0.0",
      "resolved": "https://registry.npmjs.org/isexe/-/isexe-2.0.0.tgz",
      "integrity": "sha512-RHxMLp9lnKHGHRng9QFhRCMbYAcVpn69smSGcq3f36xjgVVWThj4qqLbTLlq7Ssj8B+fIQ1EuCEGI2lKsyQeIw==",
      "dev": true,
      "license": "ISC"
    },
    "node_modules/js-aruco2": {
      "version": "2.0.0",
      "resolved": "https://registry.npmjs.org/js-aruco2/-/js-aruco2-2.0.0.tgz",
      "integrity": "sha512-2R9P7INqIkg1fsGBazSDLChmfwJpbaYN26gO6wFr1TLIOrvwQf0YzuhwVZAPF8frQ4cjdppAN0dRmExEd/6wqw==",
      "license": "MIT",
      "engines": {
        "node": ">=12.0.0"
      }
    },
    "node_modules/locate-path": {
      "version": "5.0.0",
      "resolved": "https://registry.npmjs.org/locate-path/-/locate-path-5.0.0.tgz",
      "integrity": "sha512-t7hw9pI+WvuwNJXwk5zVHpyhIqzg2qTlklJOf0mVxGSbe3Fp2VieZcduNYjaLDoy6p9uGpQEGWG87WpMKlNq8g==",
      "license": "MIT",
      "dependencies": {
        "p-locate": "^4.1.0"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/nanoid": {
      "version": "3.3.20",
      "resolved": "https://registry.npmjs.org/nanoid/-/nanoid-3.3.20.tgz",
      "integrity": "sha512-uKdg2G3GNCKQn9byYOpxbGqrT2fGO5KRt5J/8b3pok8rT6qxGWF6hxMyJiEYtAf+FVyYuD9hRaDqX5uPFYJ4ZQ==",
      "dev": true,
      "funding": [
        {
          "type": "github",
          "url": "https://github.com/sponsors/ai"
        }
      ],
      "license": "MIT",
      "bin": {
        "nanoid": "bin/nanoid.cjs"
      },
      "engines": {
        "node": "^10 || ^12 || ^13.7 || ^14 || >=15.0.1"
      }
    },
    "node_modules/p-limit": {
      "version": "2.3.0",
      "resolved": "https://registry.npmjs.org/p-limit/-/p-limit-2.3.0.tgz",
      "integrity": "sha512-//88mFWSJx8lxCzwdAABTJL2MyWB12+eIY7MDL2SqLmAkeKU9qxRvWuSyTjm3FUmpBEMuFfckAIqEaVGUDxb6w==",
      "license": "MIT",
      "dependencies": {
        "p-try": "^2.0.0"
      },
      "engines": {
        "node": ">=6"
      },
      "funding": {
        "url": "https://github.com/sponsors/sindresorhus"
      }
    },
    "node_modules/p-locate": {
      "version": "4.1.0",
      "resolved": "https://registry.npmjs.org/p-locate/-/p-locate-4.1.0.tgz",
      "integrity": "sha512-R79ZZ/0wAxKGu3oYMlz8jy/kbhsNrS7SKZ7PxEHBgJ5+F2mtFW2fK2cOtBh1cHYkQsbzFV7I+EoRKe6Yt0oK7A==",
      "license": "MIT",
      "dependencies": {
        "p-limit": "^2.2.0"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/p-try": {
      "version": "2.2.0",
      "resolved": "https://registry.npmjs.org/p-try/-/p-try-2.2.0.tgz",
      "integrity": "sha512-R4nPAVTAU0B9D35/Gk3uJf/7XYbQcyohSKdvAxIRSNghFl4e71hVoGnBNQz9cWaXxO2I10KTC+3jMdvvoKw6dQ==",
      "license": "MIT",
      "engines": {
        "node": ">=6"
      }
    },
    "node_modules/path-exists": {
      "version": "4.0.0",
      "resolved": "https://registry.npmjs.org/path-exists/-/path-exists-4.0.0.tgz",
      "integrity": "sha512-ak9Qy5Q7jYb2Wwcey5Fpvg2KoAc/ZIhLSLOSBmRmygPsGwkVVt0fZa0qrtMz+m6tJTAHfZQ8FnmB4MG4LWy7/w==",
      "license": "MIT",
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/path-key": {
      "version": "3.1.1",
      "resolved": "https://registry.npmjs.org/path-key/-/path-key-3.1.1.tgz",
      "integrity": "sha512-ojmeN0qd+y0jszEtoY48r0Peq5dwMEkIlCOu6Q5f41lfkswXuKtYrhgoTpLnyIcHm24Uhqx+5Tqm2InSwLhE6Q==",
      "dev": true,
      "license": "MIT",
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/picocolors": {
      "version": "1.1.1",
      "resolved": "https://registry.npmjs.org/picocolors/-/picocolors-1.1.1.tgz",
      "integrity": "sha512-xceH2snhtb5M9liqDsmEw56le376mTZkEX/jEb/RxNFyegNul7eNslCXP9FDj/Lcu0X8KEyMceP2ntpaHrDEVA==",
      "dev": true,
      "license": "ISC"
    },
    "node_modules/pngjs": {
      "version": "5.0.0",
      "resolved": "https://registry.npmjs.org/pngjs/-/pngjs-5.0.0.tgz",
      "integrity": "sha512-40QW5YalBNfQo5yRYmiw7Yz6TKKVr3h6970B2YE+3fQpsWcrbj1PzJgxeJ19DRQjhMbKPIuMY8rFaXc8moolVw==",
      "license": "MIT",
      "engines": {
        "node": ">=10.13.0"
      }
    },
    "node_modules/postcss": {
      "version": "8.5.33",
      "resolved": "https://registry.npmjs.org/postcss/-/postcss-8.5.33.tgz",
      "integrity": "sha512-RVHEU9/Pp1Fnjr/MpAJsZoADukCKhfIi15OunEYiGktIkYLkPVvC1sioWolGDS7XByMTx9m/IGbro0Js13zhnQ==",
      "dev": true,
      "funding": [
        {
          "type": "opencollective",
          "url": "https://opencollective.com/postcss/"
        },
        {
          "type": "tidelift",
          "url": "https://tidelift.com/funding/github/npm/postcss"
        },
        {
          "type": "github",
          "url": "https://github.com/sponsors/ai"
        }
      ],
      "license": "MIT",
      "dependencies": {
        "nanoid": "^3.3.20",
        "picocolors": "^1.1.1",
        "source-map-js": "^1.2.2"
      },
      "engines": {
        "node": "^10 || ^12 || >=14"
      }
    },
    "node_modules/qrcode": {
      "version": "1.5.4",
      "resolved": "https://registry.npmjs.org/qrcode/-/qrcode-1.5.4.tgz",
      "integrity": "sha512-1ca71Zgiu6ORjHqFBDpnSMTR2ReToX4l1Au1VFLyVeBTFavzQnv5JxMFr3ukHVKpSrSA2MCk0lNJSykjUfz7Zg==",
      "license": "MIT",
      "dependencies": {
        "dijkstrajs": "^1.0.1",
        "pngjs": "^5.0.0",
        "yargs": "^15.3.1"
      },
      "bin": {
        "qrcode": "bin/qrcode"
      },
      "engines": {
        "node": ">=10.13.0"
      }
    },
    "node_modules/qrcode/node_modules/cliui": {
      "version": "6.0.0",
      "resolved": "https://registry.npmjs.org/cliui/-/cliui-6.0.0.tgz",
      "integrity": "sha512-t6wbgtoCXvAzst7QgXxJYqPt0usEfbgQdftEPbLL/cvv6HPE5VgvqCuAIDR0NgU52ds6rFwqrgakNLrHEjCbrQ==",
      "license": "ISC",
      "dependencies": {
        "string-width": "^4.2.0",
        "strip-ansi": "^6.0.0",
        "wrap-ansi": "^6.2.0"
      }
    },
    "node_modules/qrcode/node_modules/wrap-ansi": {
      "version": "6.2.0",
      "resolved": "https://registry.npmjs.org/wrap-ansi/-/wrap-ansi-6.2.0.tgz",
      "integrity": "sha512-r6lPcBGxZXlIcymEu7InxDMhdW0KDxpLgoFLcguasxCaJ/SOIZwINatK9KY/tf+ZrlywOKU0UDj3ATXUBfxJXA==",
      "license": "MIT",
      "dependencies": {
        "ansi-styles": "^4.0.0",
        "string-width": "^4.1.0",
        "strip-ansi": "^6.0.0"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/qrcode/node_modules/y18n": {
      "version": "4.0.3",
      "resolved": "https://registry.npmjs.org/y18n/-/y18n-4.0.3.tgz",
      "integrity": "sha512-JKhqTOwSrqNA1NY5lSztJ1GrBiUodLMmIZuLiDaMRJ+itFd+ABVE8XBjOvIWL+rSqNDC74LCSFmlb/U4UZ4hJQ==",
      "license": "ISC"
    },
    "node_modules/qrcode/node_modules/yargs": {
      "version": "15.4.1",
      "resolved": "https://registry.npmjs.org/yargs/-/yargs-15.4.1.tgz",
      "integrity": "sha512-aePbxDmcYW++PaqBsJ+HYUFwCdv4LVvdnhBy78E57PIor8/OVvhMrADFFEDh8DHDFRv/O9i3lPhsENjO7QX0+A==",
      "license": "MIT",
      "dependencies": {
        "cliui": "^6.0.0",
        "decamelize": "^1.2.0",
        "find-up": "^4.1.0",
        "get-caller-file": "^2.0.1",
        "require-directory": "^2.1.1",
        "require-main-filename": "^2.0.0",
        "set-blocking": "^2.0.0",
        "string-width": "^4.2.0",
        "which-module": "^2.0.0",
        "y18n": "^4.0.0",
        "yargs-parser": "^18.1.2"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/qrcode/node_modules/yargs-parser": {
      "version": "18.1.3",
      "resolved": "https://registry.npmjs.org/yargs-parser/-/yargs-parser-18.1.3.tgz",
      "integrity": "sha512-o50j0JeToy/4K6OZcaQmW6lyXXKhq7csREXcDwk2omFPJEwUNOVtJKvmDr9EI1fAJZUyZcRF7kxGBWmRXudrCQ==",
      "license": "ISC",
      "dependencies": {
        "camelcase": "^5.0.0",
        "decamelize": "^1.2.0"
      },
      "engines": {
        "node": ">=6"
      }
    },
    "node_modules/require-directory": {
      "version": "2.1.1",
      "resolved": "https://registry.npmjs.org/require-directory/-/require-directory-2.1.1.tgz",
      "integrity": "sha512-fGxEI7+wsG9xrvdjsrlmL22OMTTiHRwAMroiEeMgq8gzoLC/PQr7RsRDSTLUg/bZAZtF+TVIkHc6/4RIKrui+Q==",
      "license": "MIT",
      "engines": {
        "node": ">=0.10.0"
      }
    },
    "node_modules/require-main-filename": {
      "version": "2.0.0",
      "resolved": "https://registry.npmjs.org/require-main-filename/-/require-main-filename-2.0.0.tgz",
      "integrity": "sha512-NKN5kMDylKuldxYLSUfrbo5Tuzh4hd+2E8NPPX02mZtn1VuREQToYe/ZdlJy+J3uCpfaiGF05e7B8W0iXbQHmg==",
      "license": "ISC"
    },
    "node_modules/rollup": {
      "version": "4.68.3",
      "resolved": "https://registry.npmjs.org/rollup/-/rollup-4.68.3.tgz",
      "integrity": "sha512-wd2gavj7CEQv52rdcVqfUupO/KPIKcoJKxyFWl4cOqYZGMQEOSijJRkliz0dRa7gpkj/h4TRqyWa+dJJi6T0OA==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "@types/estree": "^1.0.10"
      },
      "bin": {
        "rollup": "dist/bin/rollup"
      },
      "engines": {
        "node": ">=18.0.0",
        "npm": ">=8.0.0"
      },
      "optionalDependencies": {
        "@rollup/rollup-android-arm-eabi": "4.68.3",
        "@rollup/rollup-android-arm64": "4.68.3",
        "@rollup/rollup-android-x64": "4.68.3",
        "@rollup/rollup-darwin-arm64": "4.68.3",
        "@rollup/rollup-darwin-x64": "4.68.3",
        "@rollup/rollup-freebsd-arm64": "4.68.3",
        "@rollup/rollup-freebsd-x64": "4.68.3",
        "@rollup/rollup-linux-arm-gnueabihf": "4.68.3",
        "@rollup/rollup-linux-arm-musleabihf": "4.68.3",
        "@rollup/rollup-linux-arm64-gnu": "4.68.3",
        "@rollup/rollup-linux-arm64-musl": "4.68.3",
        "@rollup/rollup-linux-loong64-gnu": "4.68.3",
        "@rollup/rollup-linux-loong64-musl": "4.68.3",
        "@rollup/rollup-linux-ppc64-gnu": "4.68.3",
        "@rollup/rollup-linux-ppc64-musl": "4.68.3",
        "@rollup/rollup-linux-riscv64-gnu": "4.68.3",
        "@rollup/rollup-linux-riscv64-musl": "4.68.3",
        "@rollup/rollup-linux-s390x-gnu": "4.68.3",
        "@rollup/rollup-linux-x64-gnu": "4.68.3",
        "@rollup/rollup-linux-x64-musl": "4.68.3",
        "@rollup/rollup-openbsd-x64": "4.68.3",
        "@rollup/rollup-openharmony-arm64": "4.68.3",
        "@rollup/rollup-win32-arm64-msvc": "4.68.3",
        "@rollup/rollup-win32-ia32-msvc": "4.68.3",
        "@rollup/rollup-win32-x64-gnu": "4.68.3",
        "@rollup/rollup-win32-x64-msvc": "4.68.3",
        "fsevents": "~2.3.2"
      }
    },
    "node_modules/rxjs": {
      "version": "7.8.2",
      "resolved": "https://registry.npmjs.org/rxjs/-/rxjs-7.8.2.tgz",
      "integrity": "sha512-dhKf903U/PQZY6boNNtAGdWbG85WAbjT/1xYoZIC7FAY0yWapOBQVsVrDl58W86//e1VpMNBtRV4MaXfdMySFA==",
      "dev": true,
      "license": "Apache-2.0",
      "dependencies": {
        "tslib": "^2.1.0"
      }
    },
    "node_modules/set-blocking": {
      "version": "2.0.0",
      "resolved": "https://registry.npmjs.org/set-blocking/-/set-blocking-2.0.0.tgz",
      "integrity": "sha512-KiKBS8AnWGEyLzofFfmvKwpdPzqiy16LvQfK3yv/fVH7Bj13/wl3JSR1J+rfgRE9q7xUJK4qvgS8raSOeLUehw==",
      "license": "ISC"
    },
    "node_modules/shebang-command": {
      "version": "2.0.0",
      "resolved": "https://registry.npmjs.org/shebang-command/-/shebang-command-2.0.0.tgz",
      "integrity": "sha512-kHxr2zZpYtdmrN1qDjrrX/Z1rR1kG8Dx+gkpK1G4eXmvXswmcE1hTWBWYUzlraYw1/yZp6YuDY77YtvbN0dmDA==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "shebang-regex": "^3.0.0"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/shebang-regex": {
      "version": "3.0.0",
      "resolved": "https://registry.npmjs.org/shebang-regex/-/shebang-regex-3.0.0.tgz",
      "integrity": "sha512-7++dFhtcx3353uBaq8DDR4NuxBetBzC7ZQOhmTQInHEd6bSrXdiEyzCvG07Z44UYdLShWUyXt5M/yhz8ekcb1A==",
      "dev": true,
      "license": "MIT",
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/shell-quote": {
      "version": "1.12.0",
      "resolved": "https://registry.npmjs.org/shell-quote/-/shell-quote-1.12.0.tgz",
      "integrity": "sha512-PcByqNyT/38F2kDNi006HAMRJaULuBzq/FOsw3qdZvX/GA9W/jamDaRskgHjubHiftXK5sIFxLNkvrXUwcof6Q==",
      "dev": true,
      "license": "MIT",
      "engines": {
        "node": ">= 0.4"
      },
      "funding": {
        "url": "https://github.com/sponsors/ljharb"
      }
    },
    "node_modules/source-map-js": {
      "version": "1.2.2",
      "resolved": "https://registry.npmjs.org/source-map-js/-/source-map-js-1.2.2.tgz",
      "integrity": "sha512-KGj/8Y43x35aZVDtt+J4mK1hoLGHULMYfSkODJNQjNDC3oW1PqPoxMwo0pLUsWM/UEGzON/NxeHywEfNXNP3Vw==",
      "dev": true,
      "license": "BSD-3-Clause",
      "engines": {
        "node": ">=0.10.0"
      }
    },
    "node_modules/string-width": {
      "version": "4.2.3",
      "resolved": "https://registry.npmjs.org/string-width/-/string-width-4.2.3.tgz",
      "integrity": "sha512-wKyQRQpjJ0sIp62ErSZdGsjMJWsap5oRNihHhu6G7JVO/9jIB6UyevL+tXuOqrng8j/cxKTWyWUwvSTriiZz/g==",
      "license": "MIT",
      "dependencies": {
        "emoji-regex": "^8.0.0",
        "is-fullwidth-code-point": "^3.0.0",
        "strip-ansi": "^6.0.1"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/strip-ansi": {
      "version": "6.0.1",
      "resolved": "https://registry.npmjs.org/strip-ansi/-/strip-ansi-6.0.1.tgz",
      "integrity": "sha512-Y38VPSHcqkFrCpFnQ9vuSXmquuv5oXOKpGeT6aGrr3o3Gc9AlVa6JBfUSOCnbxGGZF+/0ooI7KrPuUSztUdU5A==",
      "license": "MIT",
      "dependencies": {
        "ansi-regex": "^5.0.1"
      },
      "engines": {
        "node": ">=8"
      }
    },
    "node_modules/supports-color": {
      "version": "8.1.1",
      "resolved": "https://registry.npmjs.org/supports-color/-/supports-color-8.1.1.tgz",
      "integrity": "sha512-MpUEN2OodtUzxvKQl72cUF7RQ5EiHsGvSsVG0ia9c5RbWGL2CI4C7EpPS8UTBIplnlzZiNuV56w+FuNxy3ty2Q==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "has-flag": "^4.0.0"
      },
      "engines": {
        "node": ">=10"
      },
      "funding": {
        "url": "https://github.com/chalk/supports-color?sponsor=1"
      }
    },
    "node_modules/three": {
      "version": "0.180.0",
      "resolved": "https://registry.npmjs.org/three/-/three-0.180.0.tgz",
      "integrity": "sha512-o+qycAMZrh+TsE01GqWUxUIKR1AL0S8pq7zDkYOQw8GqfX8b8VoCKYUoHbhiX5j+7hr8XsuHDVU6+gkQJQKg9w==",
      "license": "MIT"
    },
    "node_modules/tree-kill": {
      "version": "1.2.2",
      "resolved": "https://registry.npmjs.org/tree-kill/-/tree-kill-1.2.2.tgz",
      "integrity": "sha512-L0Orpi8qGpRG//Nd+H90vFB+3iHnue1zSSGmNOOCh1GLJ7rUKVwV2HvijphGQS2UmhUZewS9VgvxYIdgr+fG1A==",
      "dev": true,
      "license": "MIT",
      "bin": {
        "tree-kill": "cli.js"
      }
    },
    "node_modules/tslib": {
      "version": "2.9.1",
      "resolved": "https://registry.npmjs.org/tslib/-/tslib-2.9.1.tgz",
      "integrity": "sha512-a4Ux9VKuIMHsOS7ENbEq6hK2ZGYIT27Rtl0E4eD9KqwxhKiQeMZE/9xb3IdYwWaSOyIBQLExhQgXDTYoiCxnWA==",
      "dev": true,
      "license": "0BSD"
    },
    "node_modules/vite": {
      "version": "5.4.21",
      "resolved": "https://registry.npmjs.org/vite/-/vite-5.4.21.tgz",
      "integrity": "sha512-o5a9xKjbtuhY6Bi5S3+HvbRERmouabWbyUcpXXUA1u+GNUKoROi9byOJ8M0nHbHYHkYICiMlqxkg1KkYmm25Sw==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "esbuild": "^0.21.3",
        "postcss": "^8.4.43",
        "rollup": "^4.20.0"
      },
      "bin": {
        "vite": "bin/vite.js"
      },
      "engines": {
        "node": "^18.0.0 || >=20.0.0"
      },
      "funding": {
        "url": "https://github.com/vitejs/vite?sponsor=1"
      },
      "optionalDependencies": {
        "fsevents": "~2.3.3"
      },
      "peerDependencies": {
        "@types/node": "^18.0.0 || >=20.0.0",
        "less": "*",
        "lightningcss": "^1.21.0",
        "sass": "*",
        "sass-embedded": "*",
        "stylus": "*",
        "sugarss": "*",
        "terser": "^5.4.0"
      },
      "peerDependenciesMeta": {
        "@types/node": {
          "optional": true
        },
        "less": {
          "optional": true
        },
        "lightningcss": {
          "optional": true
        },
        "sass": {
          "optional": true
        },
        "sass-embedded": {
          "optional": true
        },
        "stylus": {
          "optional": true
        },
        "sugarss": {
          "optional": true
        },
        "terser": {
          "optional": true
        }
      }
    },
    "node_modules/which": {
      "version": "2.0.2",
      "resolved": "https://registry.npmjs.org/which/-/which-2.0.2.tgz",
      "integrity": "sha512-BLI3Tl1TW3Pvl70l3yq3Y64i+awpwXqsGBYWkkqMtnbXgrMD+yj7rhW0kuEDxzJaYXGjEW5ogapKNMEKNMjibA==",
      "dev": true,
      "license": "ISC",
      "dependencies": {
        "isexe": "^2.0.0"
      },
      "bin": {
        "node-which": "bin/node-which"
      },
      "engines": {
        "node": ">= 8"
      }
    },
    "node_modules/which-module": {
      "version": "2.0.1",
      "resolved": "https://registry.npmjs.org/which-module/-/which-module-2.0.1.tgz",
      "integrity": "sha512-iBdZ57RDvnOR9AGBhML2vFZf7h8vmBjhoaZqODJBFWHVtKkDmKuHai3cx5PgVMrX5YDNp27AofYbAwctSS+vhQ==",
      "license": "ISC"
    },
    "node_modules/wrap-ansi": {
      "version": "7.0.0",
      "resolved": "https://registry.npmjs.org/wrap-ansi/-/wrap-ansi-7.0.0.tgz",
      "integrity": "sha512-YVGIj2kamLSTxw6NsZjoBxfSwsn0ycdesmc4p+Q21c5zPuZ1pl+NfxVdxPtdHvmNVOQ6XSYG4AUtyt/Fi7D16Q==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "ansi-styles": "^4.0.0",
        "string-width": "^4.1.0",
        "strip-ansi": "^6.0.0"
      },
      "engines": {
        "node": ">=10"
      },
      "funding": {
        "url": "https://github.com/chalk/wrap-ansi?sponsor=1"
      }
    },
    "node_modules/ws": {
      "version": "8.23.0",
      "resolved": "https://registry.npmjs.org/ws/-/ws-8.23.0.tgz",
      "integrity": "sha512-wCkmX8QMOrKjgO8HW2WyDbTSGeqkocxgDzS9iR79rEAexty6/2qzilx3Ag8Tqf9ykop3irLKle0MtVn5qjVnoQ==",
      "license": "MIT",
      "engines": {
        "node": ">=10.0.0"
      },
      "peerDependencies": {
        "bufferutil": "^4.0.1",
        "utf-8-validate": ">=5.0.2"
      },
      "peerDependenciesMeta": {
        "bufferutil": {
          "optional": true
        },
        "utf-8-validate": {
          "optional": true
        }
      }
    },
    "node_modules/y18n": {
      "version": "5.0.8",
      "resolved": "https://registry.npmjs.org/y18n/-/y18n-5.0.8.tgz",
      "integrity": "sha512-0pfFzegeDWJHJIAmTLRP2DwHjdF5s7jo9tuztdQxAhINCdvS+3nGINqPd00AphqJR/0LhANUS6/+7SCb98YOfA==",
      "dev": true,
      "license": "ISC",
      "engines": {
        "node": ">=10"
      }
    },
    "node_modules/yargs": {
      "version": "17.7.2",
      "resolved": "https://registry.npmjs.org/yargs/-/yargs-17.7.2.tgz",
      "integrity": "sha512-7dSzzRQ++CKnNI/krKnYRV7JKKPUXMEh61soaHKg9mrWEhzFWhFnxPxGl+69cD1Ou63C13NUPCnmIcrvqCuM6w==",
      "dev": true,
      "license": "MIT",
      "dependencies": {
        "cliui": "^8.0.1",
        "escalade": "^3.1.1",
        "get-caller-file": "^2.0.5",
        "require-directory": "^2.1.1",
        "string-width": "^4.2.3",
        "y18n": "^5.0.5",
        "yargs-parser": "^21.1.1"
      },
      "engines": {
        "node": ">=12"
      }
    },
    "node_modules/yargs-parser": {
      "version": "21.1.1",
      "resolved": "https://registry.npmjs.org/yargs-parser/-/yargs-parser-21.1.1.tgz",
      "integrity": "sha512-tVpsJW7DdjecAiFpbIB1e3qxIQsE6NoPc5/eTdrbbIC4h0LVsWhnoa3g+m2HclBIujHzsxZ4VJVA+GUuc2/LBw==",
      "dev": true,
      "license": "ISC",
      "engines": {
        "node": ">=12"
      }
    }
  }
}
```

## 22. `vite.config.js`

```js
import { defineConfig } from 'vite';
import basicSsl from '@vitejs/plugin-basic-ssl';

/**
 * REALPLAY Vite config.
 *
 * - MPA: two pages are built, the PC game (index.html) and the phone
 *   controller (controller/index.html). appType 'mpa' disables the SPA
 *   fallback so /controller/* can never accidentally serve the PC app.
 * - /ws is proxied to the Node game server so the PC app and the phones
 *   talk to a single origin (important for the QR code).
 * - Set RP_HTTPS=1 (npm run dev:https) to enable a self-signed HTTPS dev
 *   server. Phone browsers only expose motion sensors in secure contexts,
 *   so LAN testing over plain HTTP will show no sensor data on iOS/Android.
 */
export default defineConfig(({ mode }) => ({
  appType: 'mpa',
  plugins: process.env.RP_HTTPS ? [basicSsl()] : [],
  server: {
    host: true, // listen on all interfaces so phones on the LAN can connect
    port: 5173,
    proxy: {
      '/ws': {
        target: 'http://127.0.0.1:8081',
        ws: true,
      },
    },
  },
  build: {
    rollupOptions: {
      input: {
        main: 'index.html',
        controller: 'controller/index.html',
        markers: 'markers.html',
      },
    },
    chunkSizeWarningLimit: 1500,
  },
}));
```

## 23. `.gitignore`

```
node_modules/
dist/
.vite/
*.log
.DS_Store
.tmp/
```
