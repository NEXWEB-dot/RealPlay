/**
 * Audio manager (Phase 1 scope).
 *
 * - Lazily creates the AudioContext on the first user gesture.
 * - Bus layout: master -> sfx / music. Volumes come from Settings.
 * - Phase 1 sounds: UI blips and a synthesized swing "whoosh" scaled by
 *   swing power (placeholder-free stand-in until the sampled hit sounds
 *   of Phase 6; the API is already positional-ready via `at`).
 */

export class AudioMgr {
  constructor(settings) {
    this.settings = settings;
    this.ctx = null;
    this.masterGain = null;
    this.sfxGain = null;
    this.musicGain = null;
    this._noiseBuf = null;

    settings.onChange((key) => {
      if (key === 'master' || key === 'sfx' || key === 'music' || key === '*') {
        this.applyVolumes();
      }
    });

    // First gesture unlocks the context (autoplay policy).
    const unlock = () => {
      this.ensure();
      window.removeEventListener('pointerdown', unlock);
      window.removeEventListener('keydown', unlock);
    };
    window.addEventListener('pointerdown', unlock);
    window.addEventListener('keydown', unlock);
  }

  ensure() {
    if (this.ctx) return this.ctx;
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return null;
    this.ctx = new AC();
    this.masterGain = this.ctx.createGain();
    this.sfxGain = this.ctx.createGain();
    this.musicGain = this.ctx.createGain();
    this.sfxGain.connect(this.masterGain);
    this.musicGain.connect(this.masterGain);
    this.masterGain.connect(this.ctx.destination);
    this.applyVolumes();
    return this.ctx;
  }

  applyVolumes() {
    if (!this.ctx) return;
    this.masterGain.gain.value = this.settings.get('master');
    this.sfxGain.gain.value = this.settings.get('sfx');
    this.musicGain.gain.value = this.settings.get('music');
  }

  resume() {
    if (this.ctx && this.ctx.state === 'suspended') this.ctx.resume();
  }

  /** Short UI blip. kind: 'tick' | 'good' | 'bad' */
  click(kind = 'tick') {
    const ctx = this.ensure();
    if (!ctx) return;
    const t = ctx.currentTime;
    const osc = ctx.createOscillator();
    const g = ctx.createGain();
    osc.type = 'sine';
    const f = kind === 'good' ? 660 : kind === 'bad' ? 220 : 440;
    osc.frequency.setValueAtTime(f, t);
    osc.frequency.exponentialRampToValueAtTime(f * (kind === 'bad' ? 0.6 : 1.4), t + 0.08);
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(kind === 'tick' ? 0.06 : 0.12, t + 0.01);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.12);
    osc.connect(g);
    g.connect(this.sfxGain);
    osc.start(t);
    osc.stop(t + 0.14);
  }

  /**
   * Shared 0.25 s white-noise buffer (swing whoosh, bounce tick, net thud).
   */
  _noise() {
    const ctx = this.ensure();
    if (!ctx) return null;
    if (!this._noiseBuf) {
      const len = Math.floor(ctx.sampleRate * 0.25);
      this._noiseBuf = ctx.createBuffer(1, len, ctx.sampleRate);
      const d = this._noiseBuf.getChannelData(0);
      for (let i = 0; i < len; i++) d[i] = Math.random() * 2 - 1;
    }
    return this._noiseBuf;
  }

  /**
   * Synthesized swing whoosh; `power` in m/s from the phone swing detector.
   * Real racket-ball impact samples land in Phase 6.
   */
  swing(power = 6) {
    const ctx = this.ensure();
    if (!ctx) return;
    const buf = this._noise();
    if (!buf) return;
    const t = ctx.currentTime;
    const src = ctx.createBufferSource();
    src.buffer = buf;
    const bp = ctx.createBiquadFilter();
    bp.type = 'bandpass';
    bp.Q.value = 1.2;
    const k = Math.min(1.6, 0.55 + power / 14);
    bp.frequency.setValueAtTime(500 * k, t);
    bp.frequency.exponentialRampToValueAtTime(2400 * k, t + 0.12);
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(Math.min(0.22, 0.05 + power / 90), t + 0.03);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.2);
    src.connect(bp); bp.connect(g); g.connect(this.sfxGain);
    src.start(t);
    src.stop(t + 0.26);
  }

  /**
   * Ball impact: plastic "tick" (bandpassed noise) + low thump.
   * `strength` = normal impact speed in m/s (0.5..12 realistic range).
   */
  bounce(strength = 1) {
    const ctx = this.ensure();
    if (!ctx) return;
    const buf = this._noise();
    if (!buf) return;
    const t = ctx.currentTime;
    const k = Math.min(1, Math.max(0.06, strength / 12));

    const src = ctx.createBufferSource();
    src.buffer = buf;
    const bp = ctx.createBiquadFilter();
    bp.type = 'bandpass';
    bp.frequency.value = 1400 + 900 * k;
    bp.Q.value = 1.4;
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(0.05 + 0.25 * k, t + 0.004);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.05);
    src.connect(bp); bp.connect(g); g.connect(this.sfxGain);
    src.start(t);
    src.stop(t + 0.08);

    const osc = ctx.createOscillator();
    const g2 = ctx.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(190 + 60 * k, t);
    osc.frequency.exponentialRampToValueAtTime(75, t + 0.07);
    g2.gain.setValueAtTime(0.0001, t);
    g2.gain.exponentialRampToValueAtTime(0.1 + 0.2 * k, t + 0.005);
    g2.gain.exponentialRampToValueAtTime(0.0001, t + 0.09);
    osc.connect(g2); g2.connect(this.sfxGain);
    osc.start(t);
    osc.stop(t + 0.1);
  }

  /** Dead net / net cord: muffled low thud. */
  thud() {
    const ctx = this.ensure();
    if (!ctx) return;
    const buf = this._noise();
    if (!buf) return;
    const t = ctx.currentTime;

    const src = ctx.createBufferSource();
    src.buffer = buf;
    const lp = ctx.createBiquadFilter();
    lp.type = 'lowpass';
    lp.frequency.value = 480;
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(0.22, t + 0.006);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.14);
    src.connect(lp); lp.connect(g); g.connect(this.sfxGain);
    src.start(t);
    src.stop(t + 0.16);

    const osc = ctx.createOscillator();
    const g2 = ctx.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(120, t);
    osc.frequency.exponentialRampToValueAtTime(55, t + 0.12);
    g2.gain.setValueAtTime(0.0001, t);
    g2.gain.exponentialRampToValueAtTime(0.18, t + 0.008);
    g2.gain.exponentialRampToValueAtTime(0.0001, t + 0.15);
    osc.connect(g2); g2.connect(this.sfxGain);
    osc.start(t);
    osc.stop(t + 0.16);
  }
}
