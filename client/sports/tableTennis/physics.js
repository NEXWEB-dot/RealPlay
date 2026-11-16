/**
 * Custom table-tennis ball physics — deterministic, fixed-timestep.
 *
 * Runs inside the game loop's 240 Hz accumulator (no variable dt ever
 * reaches this module). Real SI units: radius 20 mm, mass 2.7 g, table
 * 2.74 x 1.525 m with top at 0.76 m, net 0.1525 m high (top edge 0.9125).
 *
 * Model per substep (semi-implicit Euler + continuous collision detection):
 *   1. forces: gravity, quadratic air drag, Magnus (spin lift), spin decay
 *   2. swept-sphere CCD against the table slab (AABB), the net (AABB) and
 *      the floor plane — the earliest hit wins, so a 20+ m/s smash cannot
 *      tunnel through the 45 mm table or the 8 mm net
 *   3. impulse-based bounce with tangential friction that converts spin:
 *      topspin kicks forward, backspin checks/skids, sidespin deflects;
 *      angular velocity is updated from the friction impulse
 *   4. resting/rolling contact handling (no jitter, no per-frame allocs)
 *
 * Determinism: no Math.random inside the physics — the net-cord scatter
 * derives from a hash of the ball state. (The ball machine may randomize
 * its targets; that is spawning policy, not physics.)
 */

import * as THREE from 'three';

export const PHYS = {
  gravity: 9.81,
  ballRadius: 0.02,     // 40 mm ball
  ballMass: 0.0027,     // 2.7 g
  airDensity: 1.184,
  dragCoef: 0.4,        // drag crisis region for a smooth 40 mm sphere
  spinDecayAir: 0.05,   // 1/s — spin decays slowly in flight
  restitutionTable: 0.9,
  frictionTable: 0.3,
  restitutionEdge: 0.7, // hitting the table's side/edge
  frictionEdge: 0.4,
  restitutionFloor: 0.55,
  frictionFloor: 0.6,
  restitutionNet: 0.22, // dead net
  restitutionCord: 0.45,// net cord (tape) — luckier bounce
  cordBand: 0.014,      // m around the tape that counts as "cord"
  rollingDecel: 2.2,    // m/s^2 while rolling on table/floor
  restSpeed: 0.12,      // below this horizontal speed the ball is "resting"
  restTimeout: 3.0,     // s at rest before despawn
  maxSpeed: 40,
  maxSpin: 360,         // rad/s (~3440 rpm) hard cap
};

/** Table slab AABB (matches the visual court: top surface at 0.76). */
export const TABLE_BOX = {
  cx: 0, cy: 0.7375, cz: 0,
  hx: 0.7625, hy: 0.0225, hz: 1.37,
};
export const TABLE_TOP = TABLE_BOX.cy + TABLE_BOX.hy; // 0.76

/** Net AABB: 1.83 m wide, top edge at 0.9125, 8 mm thick. */
export const NET_BOX = {
  cx: 0, cy: 0.83625, cz: 0,
  hx: 0.915, hy: 0.07625, hz: 0.004,
};
export const NET_TOP = NET_BOX.cy + NET_BOX.hy; // 0.9125

export const FLOOR_Y = 0;

/* ------------------------------------------------------------------ */
/* Ball state                                                          */
/* ------------------------------------------------------------------ */

export function createBallState() {
  return {
    pos: new THREE.Vector3(),
    vel: new THREE.Vector3(),
    spin: new THREE.Vector3(), // ω, rad/s, world frame
    active: false,
    age: 0,
    restTimer: 0,
  };
}

export function copyBallState(dst, src) {
  dst.pos.copy(src.pos);
  dst.vel.copy(src.vel);
  dst.spin.copy(src.spin);
  dst.active = src.active;
  dst.age = src.age;
  dst.restTimer = src.restTimer;
  return dst;
}

export function deactivateBall(s) {
  s.active = false;
  s.restTimer = 0;
}

/* ------------------------------------------------------------------ */
/* Swept sphere vs AABB (Minkowski-expanded slab method)               */
/* ------------------------------------------------------------------ */

const _sweep = { t: 0, axis: 0, sign: 1, inside: false };

/**
 * Sweep a sphere (radius R) from p to p+d against an AABB.
 * Returns { t in [0,1], axis: 0|1|2, sign: ±1 face, inside } or false.
 * `sign` is the outward normal component on `axis` for the entry face.
 */
function sweepSphereBox(px, py, pz, dx, dy, dz, box, R) {
  const hx = box.hx + R, hy = box.hy + R, hz = box.hz + R;
  let tmin = 0, tmax = 1, axis = -1, near = 0;

  for (let i = 0; i < 3; i++) {
    // Slab test in the box's LOCAL frame: the AABB is centered at
    // (box.cx, box.cy, box.cz), so the sphere start must be offset by it
    // (world y=0.78 -> table-local 0.0425). Without this offset every
    // non-origin-centered box (table, net) silently never collides.
    let p, d, h;
    if (i === 0) { p = px - box.cx; d = dx; h = hx; }
    else if (i === 1) { p = py - box.cy; d = dy; h = hy; }
    else { p = pz - box.cz; d = dz; h = hz; }
    if (Math.abs(d) < 1e-9) {
      if (Math.abs(p) > h) return false; // parallel to this axis and outside
      continue;
    }
    let t1 = (-h - p) / d;
    let t2 = (h - p) / d;
    let side = -1; // entering through the -h face
    if (t1 > t2) { const tt = t1; t1 = t2; t2 = tt; side = 1; }
    if (t1 > tmin) { tmin = t1; axis = i; near = side; }
    if (t2 < tmax) tmax = t2;
    if (tmin > tmax) return false;
  }

  if (axis === -1) {
    // Started inside the expanded box: pop out along least penetration
    // (also measured in the box's local frame).
    let bestAxis = 0, bestPen = Infinity, bestSign = 1;
    for (let i = 0; i < 3; i++) {
      let p, h;
      if (i === 0) { p = px - box.cx; h = hx; }
      else if (i === 1) { p = py - box.cy; h = hy; }
      else { p = pz - box.cz; h = hz; }
      const pen = h - Math.abs(p);
      if (pen < bestPen) { bestPen = pen; bestAxis = i; bestSign = p >= 0 ? 1 : -1; }
    }
    _sweep.t = 0; _sweep.axis = bestAxis; _sweep.sign = bestSign; _sweep.inside = true;
    return _sweep;
  }

  _sweep.t = tmin < 0 ? 0 : tmin;
  _sweep.axis = axis;
  _sweep.sign = near;
  _sweep.inside = false;
  return _sweep;
}

/* ------------------------------------------------------------------ */
/* Bounce model (sphere vs half-plane with tangential friction)        */
/* ------------------------------------------------------------------ */

const _n = new THREE.Vector3();
const _rC = new THREE.Vector3();
const _u = new THREE.Vector3();
const _jt = new THREE.Vector3();
const _cross = new THREE.Vector3();

/**
 * Impulse bounce against a surface with outward normal `n`.
 * e = normal restitution, mu = Coulomb friction. Returns impact speed (>0)
 * or 0 if the ball was already separating.
 *
 * `sv*` = surface velocity (world, m/s) — zero for static surfaces, the
 * racket blade's velocity at contact for hits. All relative quantities
 * (approach, tangential slide) are taken against that surface, so the
 * friction impulse naturally imparts spin from a moving/brushing racket.
 */
export function surfaceBounce(s, nx, ny, nz, e, mu, svx = 0, svy = 0, svz = 0) {
  _n.set(nx, ny, nz);
  const vn = (s.vel.x - svx) * _n.x + (s.vel.y - svy) * _n.y + (s.vel.z - svz) * _n.z;
  if (vn >= -1e-6) return 0; // separating

  const m = PHYS.ballMass;
  const R = PHYS.ballRadius;
  const I = 0.4 * m * R * R; // solid sphere inertia

  // Normal impulse
  const jn = -(1 + e) * vn * m;

  // Contact-point tangential velocity: u = (v - v_surf)_t + ω × r_c, r_c = -R n
  _rC.copy(_n).multiplyScalar(-R);
  _u.set(s.vel.x - svx, s.vel.y - svy, s.vel.z - svz).addScaledVector(_n, -vn);
  _cross.crossVectors(s.spin, _rC);
  _u.add(_cross);

  // Impulse that would stop sliding: jt = -(2/7) m u  (sphere: 1/m + r²/I = 7/2m)
  _jt.copy(_u).multiplyScalar(-(2 / 7) * m);
  const slide = _jt.length();
  const jtMax = mu * jn;
  if (slide > jtMax) _jt.multiplyScalar(jtMax / slide);

  s.vel.addScaledVector(_n, jn / m).addScaledVector(_jt, 1 / m);
  _cross.crossVectors(_rC, _jt).multiplyScalar(1 / I);
  s.spin.add(_cross);

  return -vn;
}

/* ------------------------------------------------------------------ */
/* Force terms                                                          */
/* ------------------------------------------------------------------ */

const _acc = new THREE.Vector3();
const _dir = new THREE.Vector3();

function applyAerodynamics(vel, spin, out) {
  const sp = vel.length();
  if (sp < 1e-4) return;

  // Quadratic drag: F = ½ ρ A Cd |v|²  (opposite v̂)
  const area = Math.PI * PHYS.ballRadius * PHYS.ballRadius;
  const q = 0.5 * PHYS.airDensity * PHYS.dragCoef * area * sp * sp;
  out.addScaledVector(vel, -q / (PHYS.ballMass * sp));

  // Magnus: F = ½ ρ A C_L |v|² (ω̂ × v̂), C_L from spin ratio S = R|ω|/|v|
  const sw = spin.length();
  if (sw > 1 && sp > 0.5) {
    const S = (PHYS.ballRadius * sw) / sp;
    let cL = S / (0.5 + S);
    if (cL > 0.45) cL = 0.45;
    _dir.crossVectors(spin, vel);
    const dl = _dir.length();
    if (dl > 1e-6) {
      const F = 0.5 * PHYS.airDensity * area * cL * sp * sp;
      out.addScaledVector(_dir, F / (dl * PHYS.ballMass));
    }
  }
}

/** Deterministic hash for the net-cord scatter (no Math.random). */
function stateHash(x, y, z) {
  const v = Math.sin(x * 12.9898 + y * 78.233 + z * 37.719) * 43758.5453;
  return v - Math.floor(v);
}

/* ------------------------------------------------------------------ */
/* Fixed-step integration with CCD                                     */
/* ------------------------------------------------------------------ */

const EVENTS_NONE = { onBounce: null, onNet: null };
/** Bounce events below this impact speed are suppressed (resting micro-bounces). */
const BOUNCE_EVENT_MIN = 0.5;

/**
 * Advance one ball state by dt (call at the fixed 240 Hz rate).
 * @param {object} ev optional { onBounce(side, pos, impact, surface), onNet(kind, pos) }
 *   surface: 'table' | 'edge' | 'floor'; kind: 'cord' | 'dead'
 */
export function stepBall(s, dt, ev = EVENTS_NONE) {
  if (!s.active) return;
  const R = PHYS.ballRadius;
  s.age += dt;

  const p0x = s.pos.x, p0y = s.pos.y, p0z = s.pos.z;

  // 1) forces (semi-implicit Euler)
  _acc.set(0, -PHYS.gravity, 0);
  applyAerodynamics(s.vel, s.spin, _acc);
  s.vel.addScaledVector(_acc, dt);
  capVec(s.vel, PHYS.maxSpeed);
  s.spin.multiplyScalar(Math.max(0, 1 - PHYS.spinDecayAir * dt));
  capVec(s.spin, PHYS.maxSpin);

  // candidate end position for this substep
  const qx = p0x + s.vel.x * dt;
  const qy = p0y + s.vel.y * dt;
  const qz = p0z + s.vel.z * dt;

  // 2) CCD — earliest of table / net / floor
  let hitKind = null, hitT = 0, hitAxis = 1, hitSign = 1;
  let r = sweepSphereBox(p0x, p0y, p0z, qx - p0x, qy - p0y, qz - p0z, TABLE_BOX, R);
  if (r) { hitKind = 'table'; hitT = r.t; hitAxis = r.axis; hitSign = r.sign; }

  r = sweepSphereBox(p0x, p0y, p0z, qx - p0x, qy - p0y, qz - p0z, NET_BOX, R);
  if (r && (hitKind == null || r.t < hitT)) {
    hitKind = 'net'; hitT = r.t; hitAxis = r.axis; hitSign = r.sign;
  }

  if (p0y - R > FLOOR_Y && qy - R <= FLOOR_Y) {
    const tf = (p0y - R) / Math.max(1e-9, p0y - qy);
    if (hitKind == null || tf < hitT) { hitKind = 'floor'; hitT = tf; hitAxis = 1; hitSign = 1; }
  }

  if (hitKind) {
    const t = Math.min(1, Math.max(0, hitT));
    s.pos.set(p0x + (qx - p0x) * t, p0y + (qy - p0y) * t, p0z + (qz - p0z) * t);
    _n.set(
      hitAxis === 0 ? hitSign : 0,
      hitAxis === 1 ? hitSign : 0,
      hitAxis === 2 ? hitSign : 0,
    );

    if (hitKind === 'table') {
      if (hitAxis === 1 && hitSign > 0) {
        const impact = surfaceBounce(s, 0, 1, 0, PHYS.restitutionTable, PHYS.frictionTable);
        if (impact > BOUNCE_EVENT_MIN) ev.onBounce?.(s.pos.z < 0 ? 1 : 2, s.pos, impact, 'table');
      } else {
        const impact = surfaceBounce(s, _n.x, _n.y, _n.z, PHYS.restitutionEdge, PHYS.frictionEdge);
        if (impact > BOUNCE_EVENT_MIN) ev.onBounce?.(s.pos.z < 0 ? 1 : 2, s.pos, impact, 'edge');
      }
    } else if (hitKind === 'net') {
      resolveNet(s, _n, ev);
    } else {
      const impact = surfaceBounce(s, 0, 1, 0, PHYS.restitutionFloor, PHYS.frictionFloor);
      if (impact > BOUNCE_EVENT_MIN) ev.onBounce?.(0, s.pos, impact, 'floor');
    }
    // tiny separation so we don't re-collide next substep
    s.pos.addScaledVector(_n, 0.0005);
  } else {
    s.pos.set(qx, qy, qz);
  }

  // 3) resting / rolling contact
  handleRest(s, dt, ev);

  // world bounds — despawn runaway balls
  if (Math.abs(s.pos.x) > 12 || Math.abs(s.pos.z) > 12 || s.pos.y < -3) {
    deactivateBall(s);
  }
}

const _netN = new THREE.Vector3();

function resolveNet(s, n, ev) {
  const cord = s.pos.y + PHYS.ballRadius > NET_TOP - PHYS.cordBand;
  const e = cord ? PHYS.restitutionCord : PHYS.restitutionNet;
  const impact = surfaceBounce(s, n.x, n.y, n.z, e, 0.25);

  if (cord) {
    // "Net cord luck": deterministic scatter of the tangential response.
    const h1 = stateHash(s.pos.x, s.pos.y, s.pos.z);
    const h2 = stateHash(s.pos.z, s.pos.x, s.pos.y);
    s.vel.x += (h1 - 0.5) * 1.6;
    s.vel.y *= 0.72 + h2 * 0.25;
    s.spin.multiplyScalar(0.7);
    if (impact > 0.25) ev.onNet?.('cord', s.pos);
  } else {
    // Dead net: kill most of the cross-net speed, ball drops.
    s.vel.z *= 0.25;
    s.vel.x *= 0.4;
    s.vel.y = Math.min(s.vel.y, 0.2);
    s.spin.multiplyScalar(0.4);
    if (impact > 0.25) ev.onNet?.('dead', s.pos);
  }
  return impact;
}

function handleRest(s, dt, ev) {
  const R = PHYS.ballRadius;
  const onTable =
    Math.abs(s.pos.x) <= TABLE_BOX.hx + 0.01 &&
    Math.abs(s.pos.z) <= TABLE_BOX.hz + 0.01 &&
    Math.abs(s.pos.y - (TABLE_TOP + R)) < 0.004;
  const onFloor = Math.abs(s.pos.y - (FLOOR_Y + R)) < 0.004;
  if (!onTable && !onFloor) { s.restTimer = 0; return; }

  if (Math.abs(s.vel.y) < 0.08) s.vel.y = 0;

  // rolling friction on the horizontal velocity
  const hv = Math.hypot(s.vel.x, s.vel.z);
  if (hv > 1e-4) {
    const dec = Math.min(hv, PHYS.rollingDecel * dt);
    const k = (hv - dec) / hv;
    s.vel.x *= k; s.vel.z *= k;
  }
  // spin relaxes to the rolling condition: ωx = vz/R, ωz = -vx/R
  const k = Math.min(1, 6 * dt);
  s.spin.x += (s.vel.z / R - s.spin.x) * k;
  s.spin.z += (-s.vel.x / R - s.spin.z) * k;
  s.spin.y *= Math.max(0, 1 - 2 * dt);

  const hv2 = Math.hypot(s.vel.x, s.vel.z);
  if (hv2 < PHYS.restSpeed && Math.abs(s.vel.y) < 0.01) {
    s.restTimer += dt;
    if (s.restTimer > PHYS.restTimeout) deactivateBall(s);
  } else {
    s.restTimer = 0;
  }
}

function capVec(v, max) {
  const l = v.length();
  if (l > max) v.multiplyScalar(max / l);
}

/**
 * Magnitude of the Magnus acceleration for a ball state (debug UI / bots).
 */
export function magnusAccel(vel, spin) {
  const sp = vel.length();
  const sw = spin.length();
  if (sp < 0.5 || sw < 1) return 0;
  const S = (PHYS.ballRadius * sw) / sp;
  let cL = S / (0.5 + S);
  if (cL > 0.45) cL = 0.45;
  const area = Math.PI * PHYS.ballRadius * PHYS.ballRadius;
  const F = 0.5 * PHYS.airDensity * area * cL * sp * sp;
  return F / PHYS.ballMass;
}

/* ------------------------------------------------------------------ */
/* Trajectory prediction (bot / assist / debug)                        */
/* ------------------------------------------------------------------ */

const PRED_POINTS = 96;
const _predState = createBallState();
const _predEv = {
  onBounce: (side, pos, impact, surface) => {
    if (_predOut && !_predOut.hasBounce && surface === 'table') {
      _predOut.hasBounce = true;
      _predOut.bounceSide = side;
      _predOut.bouncePoint.copy(pos);
    }
  },
  onNet: null,
};

let _predOut = null;

function ensurePredictOut() {
  if (_predOutSingleton) return _predOutSingleton;
  const points = [];
  for (let i = 0; i < PRED_POINTS; i++) points.push(new THREE.Vector3());
  _predOutSingleton = {
    points, count: 0, t: 0,
    hasBounce: false, bounceSide: 0, bouncePoint: new THREE.Vector3(),
    hasIntercept: false, interceptPoint: new THREE.Vector3(), interceptVel: new THREE.Vector3(),
    endPos: new THREE.Vector3(), endVel: new THREE.Vector3(), endSpin: new THREE.Vector3(),
  };
  return _predOutSingleton;
}
let _predOutSingleton = null;

/**
 * Ballistic flight solver WITH air drag and Magnus: find a launch velocity
 * that flies `from` -> landing at `to` (table-top plane) while clearing the
 * net tape by `margin` (m). Each candidate is verified by simulating the
 * REAL integrator (drag, Magnus, spin, CCD), so machine feeds and assist
 * arcs stay accurate under air resistance. Bounded iterations, no per-call
 * allocation, deterministic. Returns { T, vel: {x, y, z} }.
 */
export function solveFlight(from, to, speed, margin = 0.035, spin = null) {
  const dx = to.x - from.x;
  const dz = to.z - from.z;
  const dy0 = to.y - from.y;
  const distH = Math.hypot(dx, dz) || 1e-6;

  let T = Math.max(0.25, Math.min(1.4, distH / Math.max(2, speed)));
  let bestT = T, bestScore = Infinity, haveBest = false;

  for (let iter = 0; iter < 12; iter++) {
    const vx = dx / T, vz = dz / T;
    const vy = (dy0 + 0.5 * PHYS.gravity * T * T) / T;
    const out = simFlight(from, vx, vy, vz, spin, T + 0.55);

    const landDist = Math.hypot(out.landX - from.x, out.landZ - from.z);
    const err = landDist - distH;
    const lowClear = out.seenNet && out.yNet < NET_TOP + PHYS.ballRadius + margin - 0.012;

    if (!out.netHit && !lowClear && out.landed && Math.abs(err) < 0.06) {
      return { T, vel: { x: vx, y: vy, z: vz } };
    }

    // remember the best candidate in case no candidate fully converges
    const score = (out.netHit || lowClear ? 8 : 0) + Math.abs(err);
    if (out.landed && score < bestScore) {
      bestScore = score; bestT = T; haveBest = true;
    }

    // corrections: net/too low -> raise the arc; flew past everything ->
    // flatten; short of the target -> more carry; long -> flatter
    if (out.netHit || lowClear) T += 0.055;
    else if (!out.landed) T -= 0.04;
    else T += err < 0 ? 0.05 : -0.04;
    T = Math.max(0.25, Math.min(1.4, T));
  }

  if (!haveBest) return { T, vel: { x: dx / T, y: (dy0 + 0.5 * PHYS.gravity * T * T) / T, z: dz / T } };
  const vx = dx / bestT, vz = dz / bestT;
  const vy = (dy0 + 0.5 * PHYS.gravity * bestT * bestT) / bestT;
  return { T: bestT, vel: { x: vx, y: vy, z: vz } };
}

/* Flight-solver scratch (module-level: zero allocation per solve). */
const _sfState = createBallState();
const _sfZero = new THREE.Vector3();
const _sfOut = {
  netHit: false, landed: false, floor: false,
  landX: 0, landZ: 0, seenNet: false, yNet: 0,
};
const _sfEv = {
  onBounce: (_side, pos, _impact, surface) => {
    if (_sfOut.landed || _sfOut.netHit) return;
    if (surface === 'table' || surface === 'edge') _sfOut.landed = true;
    else _sfOut.floor = true;
    _sfOut.landX = pos.x;
    _sfOut.landZ = pos.z;
  },
  onNet: () => { if (!_sfOut.landed) _sfOut.netHit = true; },
};

/** Simulate one candidate flight with the real integrator (see solveFlight). */
function simFlight(from, vx, vy, vz, spin, maxT) {
  _sfOut.netHit = false;
  _sfOut.landed = false;
  _sfOut.floor = false;
  _sfOut.seenNet = false;
  _sfState.pos.set(from.x, from.y, from.z);
  _sfState.vel.set(vx, vy, vz);
  _sfState.spin.copy(spin ?? _sfZero);
  _sfState.active = true;
  _sfState.age = 0;
  _sfState.restTimer = 0;
  const dt = 1 / 240;
  let minAz = 1;
  for (let t = 0; t < maxT; t += dt) {
    stepBall(_sfState, dt, _sfEv);
    if (!_sfState.active) break;
    if (_sfOut.landed || _sfOut.netHit) break;
    // height at the closest approach to the net plane (clearance check)
    const az = Math.abs(_sfState.pos.z);
    if (az < 0.3 && az < minAz) { minAz = az; _sfOut.yNet = _sfState.pos.y; _sfOut.seenNet = true; }
  }
  if (!_sfOut.landed && !_sfOut.netHit) {
    // never touched down within the window — treat the current spot as miss
    _sfOut.floor = true;
    _sfOut.landX = _sfState.pos.x;
    _sfOut.landZ = _sfState.pos.z;
  }
  return _sfOut;
}

/**
 * Predict a ball's future path with the SAME integrator (deterministic).
 *
 * @param {object} src ball state
 * @param {object} opts { maxT = 1.0, dt = 1/90, stopZ = null }
 *   stopZ: also stop when the ball crosses the plane z = stopZ while
 *   travelling toward it (interception query for bots/assist).
 * @param {object} [out] reusable output (defaults to a shared singleton —
 *   consume before the next call).
 * @returns {object} { points[], count, t, hasBounce, bounceSide, bouncePoint,
 *                     hasIntercept, interceptPoint, interceptVel, endPos, endVel, endSpin }
 */
export function predictBall(src, opts = {}, out = ensurePredictOut()) {
  const { maxT = 1.0, dt = 1 / 90, stopZ = null } = opts;
  copyBallState(_predState, src);
  _predState.active = true;
  _predOut = out;

  out.count = 0; out.t = 0;
  out.hasBounce = false; out.hasIntercept = false;
  if (out.count < PRED_POINTS) out.points[out.count++].copy(_predState.pos);

  const steps = Math.min(PRED_POINTS - 1, Math.ceil(maxT / dt));
  for (let i = 0; i < steps; i++) {
    // capture the pre-step position for an exact crossing interpolation
    const x0 = _predState.pos.x, y0 = _predState.pos.y, z0 = _predState.pos.z;
    stepBall(_predState, dt, _predEv);
    out.t += dt;
    if (!_predState.active) break;

    if (stopZ != null) {
      const z1 = _predState.pos.z;
      const toward = stopZ > z0 ? z1 > z0 : z1 < z0;
      const crossed = toward && (z0 - stopZ) * (z1 - stopZ) <= 0;
      if (crossed) {
        const dz = z1 - z0;
        const f = Math.abs(dz) < 1e-9 ? 0 : (stopZ - z0) / dz;
        const p = out.points[Math.min(out.count, PRED_POINTS - 1)];
        p.set(x0 + (_predState.pos.x - x0) * f, y0 + (_predState.pos.y - y0) * f, stopZ);
        out.interceptPoint.copy(p);
        out.interceptVel.copy(_predState.vel);
        out.hasIntercept = true;
        break;
      }
    }

    if (out.count < PRED_POINTS) out.points[out.count++].copy(_predState.pos);
  }

  out.endPos.copy(_predState.pos);
  out.endVel.copy(_predState.vel);
  out.endSpin.copy(_predState.spin);
  _predOut = null;
  return out;
}

/** Shared scratch for one-off predictions (bot, assist, debug line). */
export function getPredictOut() {
  return ensurePredictOut();
}

/* ------------------------------------------------------------------ */
/* Serve solver                                                        */
/* ------------------------------------------------------------------ */

/* Serve-trace scratch — SEPARATE from the _sf* flight-solver scratch so a
 * trace never clobbers a solveFlight in progress (they run back to back). */
const _svState = createBallState();
const _svZero = new THREE.Vector3();
const _svSpin = new THREE.Vector3();
const _svTo = new THREE.Vector3();
const _svVel = new THREE.Vector3();
const _svBest = { score: 0, vx: 0, vy: 0, vz: 0, sx: 0, sy: 0, sz: 0 };
const _svOut = {
  netHit: false, floor: false, nBounces: 0,
  z1: 0, z2: 0, x2: 0,
};
const _svEv = {
  onBounce: (_side, pos, _impact, surface) => {
    if (surface === 'table' || surface === 'edge') {
      if (_svOut.nBounces === 0) _svOut.z1 = pos.z;
      else if (_svOut.nBounces === 1) _svOut.z2 = pos.z;
      _svOut.nBounces++;
    } else {
      _svOut.floor = true;
    }
  },
  // ANY net contact during the serve flight disqualifies a candidate (the
  // game replays net serves as lets; the solver prefers clean arcs).
  onNet: () => { if (_svOut.nBounces < 2) _svOut.netHit = true; },
};

/**
 * Simulate a candidate serve with the REAL integrator through both of its
 * bounces (table serve rule: first bounce on the server's half, then over
 * the net onto the receiver's half). Fills _svOut and returns it:
 * { netHit, floor, nBounces, z1 (first bounce z), z2 (second bounce z) }.
 */
function traceServe(from, vx, vy, vz, spin) {
  _svOut.netHit = false;
  _svOut.floor = false;
  _svOut.nBounces = 0;
  _svOut.z1 = 0;
  _svOut.z2 = 0;
  _svState.pos.set(from.x, from.y, from.z);
  _svState.vel.set(vx, vy, vz);
  _svState.spin.copy(spin ?? _svZero);
  _svState.active = true;
  _svState.age = 0;
  _svState.restTimer = 0;
  const dt = 1 / 240;
  for (let t = 0; t < 1.8; t += dt) {
    stepBall(_svState, dt, _svEv);
    if (!_svState.active) break;
    if (_svOut.nBounces >= 2 || _svOut.floor || _svOut.netHit) break;
  }
  return _svOut;
}

/**
 * Solve a LEGAL table-tennis serve from `from` (the held ball's position,
 * just in front of the server's blade):
 *   1. bounce on the server's OWN half of the table,
 *   2. clear the net without touching it,
 *   3. land on the receiver's half, deep enough (>= 0.5 m past the net)
 *      that the receiver's assisted intercept scan sees the bounce.
 *
 * Candidates vary the own-side bounce depth and the launch speed; each is
 * solved with the drag-aware flight solver and then VERIFIED by simulating
 * the real integrator through both bounces (traceServe). Fully
 * deterministic (no randomness). `dirZ` is the travel direction sign
 * (+1 serves toward +z), `power` is the swing power (m/s) and `spinW` the
 * topspin magnitude (rad/s) the serve carries.
 * Returns { vel: {x,y,z}, spin: {x,y,z} }.
 */
export function solveServe(from, dirZ, power = 9, spinW = 60) {
  const ownSign = -dirZ;            // server's half sign (P1 half is -z)
  const spd = 4.0 + power * 0.32;   // swing power -> serve launch speed
  // deterministic slight lateral variety from the launch position
  const tx = Math.sin(from.x * 91.3 + from.y * 57.7) * 0.16;

  let haveBest = false;
  _svBest.score = Infinity;

  for (let di = 0; di < 4 && !haveBest; di++) {
    const d = 0.44 + di * 0.13;               // own-side bounce depth 0.44..0.83
    for (let si = 0; si < 3 && !haveBest; si++) {
      const v = spd * [1, 0.86, 1.14][si];

      // spin axis from the travel direction: topspin = ŷ × û drives the
      // serve forward off its own-side bounce
      const bx = tx - from.x;
      const bz = -dirZ * d - from.z;
      const distH = Math.hypot(bx, bz) || 1e-6;
      const ux = bx / distH, uz = bz / distH;
      const spin = _svSpin.set(uz, 0, -ux).multiplyScalar(spinW);

      // arc to the own-side bounce point, then verify the whole serve
      const flight = solveFlight(
        from,
        _svTo.set(tx, TABLE_TOP + PHYS.ballRadius, -dirZ * d),
        v,
        0.03,
        spin,
      );
      const out = traceServe(from, flight.vel.x, flight.vel.y, flight.vel.z, spin);

      // legality score — 0 = a clean, deep, legal serve
      let score = 0;
      if (out.netHit) score += 10;
      if (out.floor) score += 6;
      if (out.nBounces < 1 || Math.sign(out.z1) !== ownSign) score += 8;
      else if (out.nBounces < 2 || Math.sign(out.z2) !== dirZ) score += 5;
      else {
        score += Math.abs(Math.abs(out.z2) - 0.85) * 2; // prefer deep landings
        if (Math.abs(out.z2) < 0.5) score += 3;         // too shallow for the scan
      }
      if (score < _svBest.score) {
        _svBest.score = score;
        _svBest.vx = flight.vel.x; _svBest.vy = flight.vel.y; _svBest.vz = flight.vel.z;
        _svBest.sx = spin.x; _svBest.sy = spin.y; _svBest.sz = spin.z;
        if (score <= 1) haveBest = true; // early-exit on a clean candidate
      }
    }
  }

  return {
    vel: { x: _svBest.vx, y: _svBest.vy, z: _svBest.vz },
    spin: { x: _svBest.sx, y: _svBest.sy, z: _svBest.sz },
  };
}
