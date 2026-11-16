/**
 * Swing trail — pooled ribbon line behind fast rackets.
 *
 * A fixed-size ring of points is copied backwards each frame (28 points x
 * 2 rackets = trivial), colors fade to black along the tail and the line
 * is drawn additive, so "black" = invisible. Nothing is allocated per
 * frame; the trail collapses onto the racket when speed is low.
 */

import * as THREE from 'three';

const POINTS = 28;

export class TrailRibbon {
  constructor(color = 0xff8a1e) {
    this.color = new THREE.Color(color);
    this.points = [];
    for (let i = 0; i < POINTS; i++) this.points.push(new THREE.Vector3());

    this.positions = new Float32Array(POINTS * 3);
    this.colors = new Float32Array(POINTS * 3);

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.positions, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('color', new THREE.BufferAttribute(this.colors, 3).setUsage(THREE.DynamicDrawUsage));

    this.line = new THREE.Line(
      geo,
      new THREE.LineBasicMaterial({
        vertexColors: true,
        transparent: true,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
      }),
    );
    this.line.frustumCulled = false;
  }

  /** Add a sample (world position) and fade colors by speed. */
  update(worldPos, speed, dt) {
    const pts = this.points;
    for (let i = pts.length - 1; i > 0; i--) pts[i].copy(pts[i - 1]);
    pts[0].copy(worldPos);

    const intensity = Math.max(0, Math.min(1, (speed - 1.5) / 7));
    const r = this.color.r, g = this.color.g, b = this.color.b;
    for (let i = 0; i < POINTS; i++) {
      const fade = intensity * (1 - i / POINTS) * (1 - i / POINTS);
      this.positions[i * 3] = pts[i].x;
      this.positions[i * 3 + 1] = pts[i].y;
      this.positions[i * 3 + 2] = pts[i].z;
      this.colors[i * 3] = r * fade;
      this.colors[i * 3 + 1] = g * fade;
      this.colors[i * 3 + 2] = b * fade;
    }
    this.line.geometry.attributes.position.needsUpdate = true;
    this.line.geometry.attributes.color.needsUpdate = true;
  }

  setVisible(v) {
    this.line.visible = v;
  }
}
