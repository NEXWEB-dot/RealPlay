/**
 * Interpolation buffers for streamed phone sensor data.
 *
 * Phones stream at 60–100 Hz with jitter; rendering runs at 60 fps. Each
 * SyncBuffer holds a short history of motion samples and answers
 * `sample(queryTime)` with an interpolated orientation (slerp) and vectors
 * (lerp) at `queryTime` — typically `now - delay`. A small fixed delay
 * (~45 ms) absorbs network jitter without adding perceptible lag.
 *
 * Sample objects are pooled: the hot path (100 Hz per player) allocates
 * nothing after warm-up.
 */

import * as THREE from 'three';
import { FLAGS } from '../../shared/protocol.js';

const MAX_AGE_MS = 600;       // drop samples older than this when pushing
const EXTRAPOLATION_LIMIT = 250; // clamp-to-newest beyond this staleness

export class SyncBuffer {
  constructor({ delayMs = 45, maxSamples = 180 } = {}) {
    this.delayMs = delayMs;
    this.maxSamples = maxSamples;
    this.samples = [];     // pooled sample objects, ascending t
    this.pool = [];
    this.pushes = 0;       // total pushed (packet counter for the debug UI)
    this.drops = 0;

    this._qa = new THREE.Quaternion();
    this._qb = new THREE.Quaternion();
  }

  _alloc() {
    return (
      this.pool.pop() || {
        t: 0, quat: new THREE.Quaternion(),
        acc: new THREE.Vector3(), gyro: new THREE.Vector3(),
        swing: 0, flags: 0, battery: 65535,
      }
    );
  }

  /**
   * Push one decoded motion sample. `s` = { time, qx..qw, ax..az, gx..gz,
   * swing, flags, battery }. Times must be in the same time base the PC
   * will query with (server epoch ms — the phone clock-syncs to the server).
   */
  push(s) {
    this.pushes++;
    if (s.time <= 0) { this.drops++; return; }

    const last = this.samples[this.samples.length - 1];
    if (last) {
      if (s.time < last.t - 150) { this.drops++; return; } // stale reorder
      if (s.time <= last.t) s.time = last.t + 0.5;          // nudge to keep monotonic
      if (s.time - last.t > 1000) { this.samples.length = 0; } // big gap: reset
    }
    for (let i = 0; i < this.samples.length; i++) {
      if (this.samples[i].t < Date.now() - MAX_AGE_MS) {
        this.pool.push(this.samples[i]);
        this.samples.splice(i, 1);
        i--;
      }
    }

    const node = this._alloc();
    node.t = s.time;
    node.quat.set(s.qx, s.qy, s.qz, s.qw).normalize();
    node.acc.set(s.ax, s.ay, s.az);
    node.gyro.set(s.gx, s.gy, s.gz);
    node.swing = s.swing;
    node.flags = s.flags;
    node.battery = s.battery;
    this.samples.push(node);

    while (this.samples.length > this.maxSamples) {
      this.pool.push(this.samples.shift());
    }
  }

  /** True if the newest sample is fresh enough to drive a racket. */
  isFresh(nowMs) {
    const last = this.samples[this.samples.length - 1];
    return !!last && nowMs - last.t < EXTRAPOLATION_LIMIT;
  }

  /**
   * Interpolate into `out` (a RacketInput: orientation, linearAcceleration,
   * angularVelocity, swing, flags, battery). Returns false when no usable
   * data exists yet.
   */
  sample(nowMs, out) {
    const arr = this.samples;
    if (!arr.length) return false;
    const query = nowMs - this.delayMs;
    const last = arr[arr.length - 1];

    if (query >= last.t) {
      // Starved (jitter spike): hold the newest sample up to the limit.
      if (query - last.t > EXTRAPOLATION_LIMIT) return false;
      out.orientation.copy(last.quat);
      out.linearAcceleration.copy(last.acc);
      out.angularVelocity.copy(last.gyro);
      out.swing = last.swing;
      out.flags = last.flags;
      out.battery = last.battery;
      return true;
    }
    if (query <= arr[0].t) {
      out.orientation.copy(arr[0].quat);
      out.linearAcceleration.copy(arr[0].acc);
      out.angularVelocity.copy(arr[0].gyro);
      out.swing = arr[0].swing;
      out.flags = arr[0].flags;
      out.battery = arr[0].battery;
      return true;
    }

    // Find the bracketing pair (scan from the end — samples arrive mostly in order).
    for (let i = arr.length - 1; i > 0; i--) {
      const b = arr[i];
      const a = arr[i - 1];
      if (a.t <= query) {
        const u = (query - a.t) / Math.max(1e-6, b.t - a.t);
        out.orientation.slerpQuaternions(a.quat, b.quat, u);
        out.linearAcceleration.lerpVectors(a.acc, b.acc, u);
        out.angularVelocity.lerpVectors(a.gyro, b.gyro, u);
        out.swing = b.swing;
        out.flags = b.flags;
        out.battery = b.battery;
        return true;
      }
    }
    return false;
  }

  dispose() {
    for (const s of this.samples) this.pool.push(s);
    this.samples.length = 0;
  }
}

export { FLAGS };
