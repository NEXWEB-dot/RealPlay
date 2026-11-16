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
