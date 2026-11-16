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
