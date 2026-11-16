/**
 * Renderer manager.
 *
 * - Single WebGL2 renderer, ACES tone mapping, PCF soft shadows.
 * - Quality presets cap the pixel ratio; dynamic resolution scales the
 *   effective pixel ratio between 0.5x and 1.0x of the cap based on a
 *   rolling average of frame times (hysteresis, ~0.5 s decision window).
 * - Exposes renderer.info to the debug overlay.
 */

import * as THREE from 'three';
import { QUALITY_PRESETS } from './settings.js';

export class RendererManager {
  constructor(settings) {
    this.settings = settings;
    this.scale = 1; // dynamic resolution factor (0.5..1)

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
      stencil: false,
    });
    this.renderer.domElement.id = 'gl';
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    document.body.appendChild(this.renderer.domElement);

    this.baseDpr = Math.max(1, window.devicePixelRatio || 1);
    this._frameAccum = 0;
    this._frameCount = 0;

    window.addEventListener('resize', () => this.apply());
    this.apply();
  }

  /** Current effective pixel ratio actually set on the renderer. */
  get pixelRatio() {
    return this.renderer.getPixelRatio();
  }

  /** Re-apply pixel ratio + size from settings and current scale. */
  apply() {
    const preset = QUALITY_PRESETS[this.settings.get('quality')] || QUALITY_PRESETS.high;
    const cap = preset.pixelRatioCap;
    const manual = this.settings.get('dynamicRes') ? this.scale : this.settings.get('resScale');
    const pr = Math.min(this.baseDpr, cap) * manual;
    this.renderer.setPixelRatio(pr);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  /**
   * Feed one frame time. When dynamic resolution is enabled, evaluates the
   * rolling average every 0.5 s and nudges the scale.
   */
  update(dt) {
    if (!this.settings.get('dynamicRes')) return;
    this._frameAccum += dt;
    this._frameCount++;
    if (this._frameAccum < 0.5) return;
    const avgMs = (this._frameAccum / this._frameCount) * 1000;
    this._frameAccum = 0;
    this._frameCount = 0;
    let changed = false;
    if (avgMs > 19.5 && this.scale > 0.5) {
      this.scale = Math.max(0.5, this.scale * 0.9);
      changed = true;
    } else if (avgMs < 14 && this.scale < 1) {
      this.scale = Math.min(1, this.scale * 1.05);
      changed = true;
    }
    if (changed) this.apply();
  }

  info() {
    return this.renderer.info;
  }

  dispose() {
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
