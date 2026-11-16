/**
 * Ball visuals — spin-visible mesh, blob shadow, trail.
 *
 * - The mesh texture (seam arcs + logo) makes spin readable by eye;
 *   `integrateSpinVisual` premultiplies an axis-angle quaternion derived
 *   from the physics angular velocity ω (rad/s, world frame).
 * - The blob shadow is a radial-gradient sprite that rides the nearest
 *   supporting surface (table top inside the table footprint, floor
 *   otherwise); it grows and fades with the ball's height.
 * - The trail reuses the pooled TrailRibbon from the rackets.
 *
 * Visual only: never allocates per frame, never touches physics.
 */

import * as THREE from 'three';
import { createBallState, TABLE_TOP, TABLE_BOX, FLOOR_Y, PHYS } from './physics.js';
import { TrailRibbon } from './trail.js';

const BALL_R = PHYS.ballRadius;

/* ------------------------------------------------------------------ */
/* Procedural textures                                                 */
/* ------------------------------------------------------------------ */

function makeBallTexture() {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 128;
  const ctx = c.getContext('2d');
  ctx.fillStyle = '#f7f3e8';
  ctx.fillRect(0, 0, 256, 128);
  // two seam circles (equirect placement reads as seams while spinning)
  ctx.strokeStyle = 'rgba(118, 106, 88, 0.95)';
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(64, 64, 26, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.arc(192, 64, 26, 0, Math.PI * 2); ctx.stroke();
  // faint meridian for extra spin readability
  ctx.strokeStyle = 'rgba(118, 106, 88, 0.35)';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(0, 96); ctx.bezierCurveTo(85, 120, 170, 120, 256, 96); ctx.stroke();
  // logo
  ctx.fillStyle = '#e8641e';
  ctx.font = 'bold 30px Arial, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('RP', 128, 64);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

function makeShadowTexture() {
  const c = document.createElement('canvas');
  c.width = 64; c.height = 64;
  const ctx = c.getContext('2d');
  const g = ctx.createRadialGradient(32, 32, 2, 32, 32, 32);
  g.addColorStop(0, 'rgba(0, 0, 0, 0.75)');
  g.addColorStop(0.55, 'rgba(0, 0, 0, 0.35)');
  g.addColorStop(1, 'rgba(0, 0, 0, 0)');
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

let _sharedTex = null;
function sharedTexture() {
  if (!_sharedTex) _sharedTex = makeBallTexture();
  return _sharedTex;
}

let _sharedShadowTex = null;
function sharedShadowTexture() {
  if (!_sharedShadowTex) _sharedShadowTex = makeShadowTexture();
  return _sharedShadowTex;
}

/* ------------------------------------------------------------------ */
/* Spin visual                                                         */
/* ------------------------------------------------------------------ */

const _axis = new THREE.Vector3();
const _dq = new THREE.Quaternion();

/**
 * Advance a mesh's rotation by the physics spin: premultiplied axis-angle
 * quaternion (ω axis × |ω| dt). |ω|·dt can exceed π at high rpm — the cap
 * keeps the visual sane while the net motion still reads as fast spin.
 */
export function integrateSpinVisual(mesh, spin, dt) {
  const w = spin.length();
  if (w < 0.5) return;
  _axis.copy(spin).multiplyScalar(1 / w);
  _dq.setFromAxisAngle(_axis, Math.min(Math.PI, w * dt));
  mesh.quaternion.premultiply(_dq);
}

/* ------------------------------------------------------------------ */
/* Ball                                                                */
/* ------------------------------------------------------------------ */

export class Ball {
  /**
   * @param {THREE.Scene} scene
   * @param {number} id 1-based id for the debug overlay
   */
  constructor(scene, id) {
    this.id = id;
    this.state = createBallState();
    this.state.pos.set(0, -10, 0); // parked

    this.mesh = new THREE.Mesh(
      new THREE.SphereGeometry(BALL_R, 20, 14),
      new THREE.MeshStandardMaterial({
        map: sharedTexture(),
        roughness: 0.55,
        metalness: 0,
      }),
    );
    this.mesh.castShadow = true;
    this.mesh.visible = false;
    scene.add(this.mesh);

    this.shadowMat = new THREE.MeshBasicMaterial({
      map: sharedShadowTexture(),
      transparent: true,
      depthWrite: false,
      opacity: 0.4,
    });
    this.shadow = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), this.shadowMat);
    this.shadow.rotation.x = -Math.PI / 2;
    this.shadow.renderOrder = 2;
    this.shadow.visible = false;
    scene.add(this.shadow);

    this.trail = new TrailRibbon(0xffd28a);
    this.trail.setVisible(false);
    scene.add(this.trail.line);
  }

  setVisible(v) {
    this.mesh.visible = v;
    this.shadow.visible = v;
    this.trail.setVisible(v);
  }

  /** Called when the ball is (re)spawned or deactivated. */
  resetVisual() {
    this.mesh.quaternion.set(0, 0, 0, 1);
  }

  /** Per-frame visual sync from the physics state (render frame dt). */
  syncFromState(dt) {
    const s = this.state;
    this.mesh.position.copy(s.pos);
    integrateSpinVisual(this.mesh, s.spin, dt);
    this._updateShadow();
    this.trail.update(s.pos, s.vel.length(), dt);
  }

  _updateShadow() {
    const p = this.state.pos;
    // Nearest supporting surface below the ball: the table top when the
    // ball is over the table footprint (and above it), the floor otherwise.
    let surfaceY = FLOOR_Y;
    if (
      p.y - BALL_R > TABLE_TOP &&
      Math.abs(p.x) <= TABLE_BOX.hx &&
      Math.abs(p.z) <= TABLE_BOX.hz
    ) {
      surfaceY = TABLE_TOP;
    }
    const h = Math.max(0, p.y - BALL_R - surfaceY);
    const d = 0.055 + h * 0.16; // blob diameter grows with height
    this.shadow.position.set(p.x, surfaceY + 0.002, p.z);
    this.shadow.scale.setScalar(d);
    this.shadowMat.opacity = Math.max(0.06, 0.5 - h * 0.32);
  }
}
