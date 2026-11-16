/**
 * Impact particles — one pooled THREE.Points for every bounce/net hit.
 *
 * A fixed ring of particles is reused (cursor); dead particles are parked
 * at y = -100 with zero color (additive blending -> invisible). Per
 * particle: position, velocity, base color, life. The update loop is a
 * flat Float32Array sweep with zero allocations.
 *
 * `emit(pos, normal, count, color, speedScale)` — normal is the surface
 * normal the burst sprays around (cone), speedScale scales velocities.
 */

import * as THREE from 'three';

const _c = new THREE.Color();

export class ImpactParticles {
  constructor({ max = 320, size = 0.028 } = {}) {
    this.max = max;
    this.cursor = 0;

    this.pos = new Float32Array(max * 3);
    this.vel = new Float32Array(max * 3);
    this.base = new Float32Array(max * 3);
    this.life = new Float32Array(max);
    this.maxLife = new Float32Array(max);
    for (let i = 0; i < max; i++) this.pos[i * 3 + 1] = -100; // parked

    const geo = new THREE.BufferGeometry();
    this.posAttr = new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage);
    this.colAttr = new THREE.BufferAttribute(new Float32Array(max * 3), 3).setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('position', this.posAttr);
    geo.setAttribute('color', this.colAttr);

    this.points = new THREE.Points(geo, new THREE.PointsMaterial({
      size,
      vertexColors: true,
      transparent: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      sizeAttenuation: true,
    }));
    this.points.frustumCulled = false;
  }

  /** Spawn a burst. Safe to call at 240 Hz rate events (pooled). */
  emit(pos, normal, count, color, speedScale = 1) {
    _c.set(color);
    for (let k = 0; k < count; k++) {
      const i = this.cursor;
      this.cursor = (this.cursor + 1) % this.max;
      const i3 = i * 3;

      // random cone around the surface normal
      const dx = normal.x + (Math.random() - 0.5) * 1.1;
      const dy = normal.y + (Math.random() - 0.5) * 1.1;
      const dz = normal.z + (Math.random() - 0.5) * 1.1;
      const l = Math.hypot(dx, dy, dz) || 1;
      const sp = (0.4 + Math.random() * 1.4) * speedScale;

      this.vel[i3] = (dx / l) * sp;
      this.vel[i3 + 1] = (dy / l) * sp + 0.35 * speedScale;
      this.vel[i3 + 2] = (dz / l) * sp;
      this.pos[i3] = pos.x;
      this.pos[i3 + 1] = pos.y + 0.004;
      this.pos[i3 + 2] = pos.z;
      this.base[i3] = _c.r;
      this.base[i3 + 1] = _c.g;
      this.base[i3 + 2] = _c.b;
      this.maxLife[i] = this.life[i] = 0.22 + Math.random() * 0.28;
    }
  }

  /** Integrate + fade to black (dt = render frame time). */
  update(dt) {
    const colors = this.colAttr.array;
    const drag = Math.exp(-3.5 * dt);
    for (let i = 0; i < this.max; i++) {
      if (this.life[i] <= 0) continue;
      this.life[i] -= dt;
      const i3 = i * 3;
      if (this.life[i] <= 0) {
        this.pos[i3 + 1] = -100;
        colors[i3] = colors[i3 + 1] = colors[i3 + 2] = 0;
        continue;
      }
      this.vel[i3] *= drag;
      this.vel[i3 + 1] = this.vel[i3 + 1] * drag - 2.2 * dt;
      this.vel[i3 + 2] *= drag;
      this.pos[i3] += this.vel[i3] * dt;
      this.pos[i3 + 1] += this.vel[i3 + 1] * dt;
      this.pos[i3 + 2] += this.vel[i3 + 2] * dt;
      const k = this.life[i] / this.maxLife[i];
      colors[i3] = this.base[i3] * k;
      colors[i3 + 1] = this.base[i3 + 1] * k;
      colors[i3 + 2] = this.base[i3 + 2] * k;
    }
    this.posAttr.needsUpdate = true;
    this.colAttr.needsUpdate = true;
  }
}
