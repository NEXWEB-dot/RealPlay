/**
 * Fixed-timestep game loop with an accumulator.
 *
 *   fixed(fixedDt)    runs deterministically at 240 Hz (ball physics in
 *                     Phase 2 plugs in here)
 *   frame(dt, alpha)  runs once per rendered frame; `alpha` is the
 *                     interpolation remainder in [0,1)
 *
 * Frame time is clamped to 250 ms so a background tab never produces a
 * multi-second physics catch-up spiral.
 */

export class GameLoop {
  constructor({ fixedDt = 1 / 240, fixed, frame } = {}) {
    this.fixedDt = fixedDt;
    this.fixed = fixed || (() => {});
    this.frame = frame || (() => {});
    this.running = false;
    this._acc = 0;
    this._last = 0;
    this._raf = 0;
    this._watchdog = 0;
    this._bound = this._tick.bind(this);
  }

  start() {
    if (this.running) return;
    this.running = true;
    this._last = performance.now();
    this._acc = 0;
    this._raf = requestAnimationFrame(this._bound);
    // Watchdog: some embedded/occluded-but-visible webviews starve rAF.
    // If no frame ran for 120 ms while the document is visible, drive the
    // loop from the watchdog so the game keeps rendering.
    this._watchdog = setInterval(() => {
      if (!this.running) return;
      const now = performance.now();
      if (now - this._last < 120) return;
      if (document.visibilityState !== 'visible') return;
      this._raf = requestAnimationFrame(this._bound); // keep the rAF chain alive
      this._tick(now);
    }, 16);
  }

  stop() {
    this.running = false;
    cancelAnimationFrame(this._raf);
    clearInterval(this._watchdog);
  }

  _tick(t) {
    if (!this.running) return;
    this._raf = requestAnimationFrame(this._bound);
    let dt = (t - this._last) / 1000;
    this._last = t;
    if (dt <= 0) return;
    if (dt > 0.25) dt = 0.25; // tab was hidden — don't spiral

    this._acc += dt;
    // Batch size for this frame — substeps are told their index so fast
    // movers (racket blades) can be interpolated between frame transforms.
    const batch = Math.min(240, Math.floor(this._acc / this.fixedDt));
    let steps = 0;
    while (this._acc >= this.fixedDt && steps < 240) {
      this.fixed(this.fixedDt, steps, batch);
      this._acc -= this.fixedDt;
      steps++;
    }
    if (steps >= 240) this._acc = 0; // drop backlog rather than stall

    const alpha = this._acc / this.fixedDt;
    this.frame(dt, alpha, Date.now());
  }
}
