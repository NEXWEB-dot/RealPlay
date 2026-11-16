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
