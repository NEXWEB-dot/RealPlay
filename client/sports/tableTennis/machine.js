/**
 * Ball machine — procedural model + launcher (Phase 2 test mode).
 *
 * Stands behind player 2's end (head pivot at z ≈ 1.95, y = 1.0) and feeds
 * balls toward random targets on player 1's half of the table.
 *
 * Launch solver: pick a landing target on the P1 half, aim the head at it,
 * then shooting-solve the launch velocity: candidates are verified against
 * the real integrator (drag + Magnus + spin + CCD) until the arc clears
 * the net tape with margin and lands on the target (see solveFlight).
 *
 * Spin axis (for travel direction û):
 *   topspin   ω = ŷ × û  (top of the ball rolls forward -> dives)
 *   backspin  −ω         (floats/checks)
 *   sidespin  ±ŷ         (curves sideways)
 * 'mix' picks one at random. Randomizing targets is spawning policy —
 * the physics itself stays deterministic.
 */

import * as THREE from 'three';
import { PHYS, TABLE_TOP, solveFlight } from './physics.js';

const NEG_Z = new THREE.Vector3(0, 0, -1);
const MIX_TYPES = ['topspin', 'backspin', 'sidespinL', 'sidespinR', 'none'];

const HEAD_Y = 1.08;  // head pivot height (world, group sits at origin)
const HEAD_Z = 1.62;  // head pivot depth (close behind the table end, robot-style)
const MUZZLE_LEN = 0.16;

const _v1 = new THREE.Vector3();
const _v2 = new THREE.Vector3();
const _v3 = new THREE.Vector3();
const _v4 = new THREE.Vector3();
const _v5 = new THREE.Vector3();

export class BallMachine {
  /**
   * @param {THREE.Scene} scene
   * @param {{ accent?: number }} opts
   */
  constructor(scene, { accent = 0xff8a1e } = {}) {
    this.config = {
      running: false,     // auto-feed on/off
      interval: 1.4,      // s between balls
      speed: 9,           // m/s (horizontal launch speed)
      spinType: 'mix',    // mix | topspin | backspin | sidespinL | sidespinR | none
      spinRpm: 900,       // |ω| magnitude
      randomize: true,    // random targets on the P1 half
    };
    this.onLaunch = null; // ({ pos, vel, spin }) set by the sport
    this._timer = 0.8;
    this._wheelSpeed = 0;

    this._buildModel(scene, accent);
  }

  _buildModel(scene, accent) {
    const g = new THREE.Group();
    g.name = 'ballMachine';
    this.group = g;

    const dark = new THREE.MeshStandardMaterial({ color: 0x1d232e, roughness: 0.6, metalness: 0.3 });
    const darker = new THREE.MeshStandardMaterial({ color: 0x141920, roughness: 0.7 });
    const accentMat = new THREE.MeshStandardMaterial({
      color: accent, emissive: accent, emissiveIntensity: 0.7, roughness: 0.4,
    });

    // base plate + wheels
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.05, 0.42), dark);
    base.position.set(0, 0.045, HEAD_Z + 0.16);
    base.castShadow = true;
    g.add(base);
    const wheelGeo = new THREE.CylinderGeometry(0.035, 0.035, 0.03, 16);
    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        const w = new THREE.Mesh(wheelGeo, darker);
        w.rotation.z = Math.PI / 2;
        w.position.set(sx * 0.15, 0.035, HEAD_Z + 0.16 + sz * 0.18);
        g.add(w);
      }
    }

    // column
    const column = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.92, 0.09), dark);
    column.position.set(0, 0.5, HEAD_Z + 0.12);
    column.castShadow = true;
    g.add(column);

    // head assembly (pitches/yaws toward the target)
    const head = new THREE.Group();
    head.name = 'head';
    head.position.set(0, HEAD_Y, HEAD_Z);
    g.add(head);
    this.head = head;

    const body = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.22, 0.3), dark);
    body.castShadow = true;
    head.add(body);

    // hopper basket above the body
    const hopper = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.11, 0.2), darker);
    hopper.position.set(0, 0.165, -0.02);
    head.add(hopper);
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.245, 0.02, 0.205), accentMat);
    stripe.position.set(0, 0.125, -0.02);
    head.add(stripe);

    // barrel pointing -z
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.028, MUZZLE_LEN, 20), darker);
    barrel.rotation.x = Math.PI / 2;
    barrel.position.set(0, 0, -MUZZLE_LEN / 2 - 0.02);
    barrel.castShadow = true;
    head.add(barrel);

    // accent ring at the muzzle
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.026, 0.004, 8, 24), accentMat);
    ring.position.set(0, 0, -MUZZLE_LEN - 0.02);
    head.add(ring);

    // drive wheels visible at the muzzle (counter-rotate while running)
    const driveGeo = new THREE.CylinderGeometry(0.048, 0.048, 0.018, 20);
    this.wheelL = new THREE.Mesh(driveGeo, accentMat);
    this.wheelR = new THREE.Mesh(driveGeo, accentMat);
    this.wheelL.rotation.z = Math.PI / 2;
    this.wheelR.rotation.z = Math.PI / 2;
    this.wheelL.position.set(-0.036, 0, -MUZZLE_LEN + 0.02);
    this.wheelR.position.set(0.036, 0, -MUZZLE_LEN + 0.02);
    head.add(this.wheelL, this.wheelR);

    scene.add(g);
  }

  /** Head base z is the recoil anchor; the kick offsets +z then eases back. */
  _recoil() {
    this.head.position.z = HEAD_Z + 0.02;
  }

  /**
   * Solve one launch. Returns { pos, vel, spin } (fresh vectors — once per
   * ball, not a hot path).
   */
  solveLaunch() {
    const cfg = this.config;

    // 1) target on the P1 half of the table — deep enough that the
    //    post-bounce rise happens near the player's end, inside blade reach
    const tx = cfg.randomize ? Math.random() * 0.9 - 0.45 : 0;
    const tz = cfg.randomize ? -(0.6 + Math.random() * 0.55) : -0.85;
    const targetY = TABLE_TOP;

    // 2) resolve the spin type first — the net-clearance margin depends on
    //    it (topspin dives into the net, backspin floats over it)
    const w = (cfg.spinRpm * 2 * Math.PI) / 60;
    const type = cfg.spinType === 'mix'
      ? MIX_TYPES[(Math.random() * MIX_TYPES.length) | 0]
      : cfg.spinType;
    const margin = type === 'topspin' && w > 1 ? 0.07
      : type === 'backspin' && w > 1 ? 0.015
      : 0.035;

    // 3) aim the head (pivot -> target), derive the muzzle world point
    const pivot = _v1.set(0, HEAD_Y, HEAD_Z);
    const dir = _v2.set(tx - pivot.x, targetY - pivot.y, tz - pivot.z).normalize();
    this.head.quaternion.setFromUnitVectors(NEG_Z, dir);
    const muzzle = _v3.copy(pivot).addScaledVector(dir, MUZZLE_LEN);

    // 4) horizontal travel direction + spin — the spin must be known
    //    BEFORE the flight solve (Magnus bends the arc)
    const dx = tx - muzzle.x;
    const dz = tz - muzzle.z;
    const distH = Math.hypot(dx, dz) || 1e-6;

    const spin = new THREE.Vector3();
    if (w > 1 && type !== 'none') {
      const ux = dx / distH;
      const uz = dz / distH;
      // topspin axis = ŷ × û = (uz, 0, -ux)
      switch (type) {
        case 'topspin': spin.set(uz, 0, -ux); break;
        case 'backspin': spin.set(-uz, 0, ux); break;
        case 'sidespinL': spin.set(0, 1, 0); break;
        case 'sidespinR': spin.set(0, -1, 0); break;
      }
      spin.multiplyScalar(w);
    }

    // 5) flight: drag-aware shooting solver — each candidate is verified
    //    against the real integrator, so the feed clears the net with
    //    margin and lands on the target despite drag/Magnus
    const flight = solveFlight(
      muzzle,
      _v4.set(tx, targetY + PHYS.ballRadius, tz),
      Math.max(2, cfg.speed),
      margin,
      w > 1 && type !== 'none' ? spin : null,
    );
    const vel = new THREE.Vector3(flight.vel.x, flight.vel.y, flight.vel.z);

    return {
      pos: muzzle.clone(),
      vel,
      spin,
      target: new THREE.Vector3(tx, targetY, tz),
      flightTime: flight.T,
    };
  }

  /** One manual ball (Space key / panel button) — ignores the timer. */
  tossOne() {
    const launch = this.solveLaunch();
    this._recoil();
    this.onLaunch?.(launch);
    return launch;
  }

  /** Show/hide the whole machine — it only exists in practice mode. */
  setVisible(v) {
    this.group.visible = !!v;
    if (!v) this.config.running = false;
  }

  /** Auto-feed tick (runs inside the fixed 240 Hz step). */
  update(dt) {
    // drive wheels spin up/down toward the running state
    const target = this.config.running ? 30 : 0;
    this._wheelSpeed += (target - this._wheelSpeed) * Math.min(1, dt * 4);
    if (this._wheelSpeed > 0.01) {
      this.wheelL.rotation.x += this._wheelSpeed * dt;
      this.wheelR.rotation.x -= this._wheelSpeed * dt;
    }

    // recoil recovery
    if (this.head.position.z > HEAD_Z) {
      this.head.position.z += (HEAD_Z - this.head.position.z) * Math.min(1, dt * 10);
      if (this.head.position.z - HEAD_Z < 0.0005) this.head.position.z = HEAD_Z;
    }

    if (!this.config.running) return;
    this._timer -= dt;
    if (this._timer <= 0) {
      const launch = this.solveLaunch();
      this._recoil();
      this.onLaunch?.(launch);
      this._timer = this.config.interval * (0.85 + Math.random() * 0.3);
    }
  }
}
