/**
 * Split-screen viewport manager.
 *
 * Supports 1 (fullscreen, e.g. vs Bot), 2 (side-by-side, table tennis
 * singles) and 4 (2x2 grid, doubles sports) viewports over ONE scene with
 * one camera per player slot. Layouts already support team assignment via
 * the cameras array order: [P1, P2] or [P1, P2, P3, P4].
 */

import * as THREE from 'three';

const GAP = 4; // px gap between viewports

export class ViewportManager {
  constructor(renderer) {
    this.renderer = renderer;
    this.cameras = [];
    this.count = 1;
    this._size = new THREE.Vector2();
  }

  /**
   * Configure the layout. `cameras` is an array of THREE.PerspectiveCamera,
   * one per slot; unused trailing slots are ignored.
   */
  setLayout(count, cameras) {
    this.count = Math.max(1, Math.min(4, count | 0));
    this.cameras = (cameras || []).slice(0, this.count);
  }

  _rects(w, h) {
    const gap = GAP;
    if (this.count === 1) {
      return [{ x: 0, y: 0, w, h }];
    }
    if (this.count === 2) {
      const half = Math.floor((w - gap) / 2);
      return [
        { x: 0, y: 0, w: half, h },
        { x: half + gap, y: 0, w: w - half - gap, h },
      ];
    }
    // 4: 2x2 grid
    const hw = Math.floor((w - gap) / 2);
    const hh = Math.floor((h - gap) / 2);
    return [
      { x: 0, y: hh + gap, w: hw, h: hh },          // top-left
      { x: hw + gap, y: hh + gap, w: hw, h: hh },   // top-right
      { x: 0, y: 0, w: hw, h: hh },                 // bottom-left
      { x: hw + gap, y: 0, w: hw, h: hh },          // bottom-right
    ];
  }

  render(scene) {
    const r = this.renderer;
    r.getSize(this._size);
    const w = Math.floor(this._size.x);
    const h = Math.floor(this._size.y);

    // Clear the whole canvas once so viewport gaps don't hold stale pixels.
    r.setScissorTest(false);
    r.setViewport(0, 0, w, h);
    r.clear(true, true, false);

    const rects = this._rects(w, h);
    for (let i = 0; i < this.count; i++) {
      const cam = this.cameras[i];
      if (!cam) continue;
      const rect = rects[i];
      cam.aspect = rect.w / rect.h;
      cam.updateProjectionMatrix();

      // Convert top-origin rect to GL bottom-origin coordinates.
      const yGl = h - rect.y - rect.h;
      r.setViewport(rect.x, yGl, rect.w, rect.h);
      r.setScissor(rect.x, yGl, rect.w, rect.h);
      r.setScissorTest(true);
      r.render(scene, cam);
    }
    r.setScissorTest(false);
  }
}
