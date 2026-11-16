/**
 * Table tennis sport — Phase 2.
 *
 * Phase 1 (kept): full PBR court/hall, live phone-driven rackets in split
 * screen (calibrated orientation IS the racket rotation, touchpad nudges
 * position, swing events fire a lunge + arrow + whoosh).
 *
 * Phase 2:
 *   - real ball physics: an 8-ball pool stepped at the fixed 240 Hz rate
 *     with gravity, quadratic drag, Magnus, spin-transfer bounces, net +
 *     net-cord, floor, swept-sphere CCD (see ./physics.js),
 *   - a ball machine behind player 2's end feeding the rally,
 *   - ball visuals: spin-visible seam/logo texture, blob shadows, trails,
 *   - pooled impact particles on bounces/net hits,
 *   - a cyan trajectory-prediction line (shown while the F3 overlay is open),
 *   - modes: 'debug2p' (split screen, machine off by default) and
 *     'practice' (single camera, machine feeding).
 *
 * Phase 3 (kept):
 *   - racket-ball hit model: swept ball-vs-blade contact at the 240 Hz
 *     step (blade interpolated between the render frame's transforms),
 *     impulse response against the MOVING blade (rubber restitution +
 *     friction spin transfer), sweet-spot power falloff, swing-boosted
 *     blade velocity straight from the phone's swing detector,
 *   - assisted positioning toward the predicted interception point
 *     (Settings ▸ Assist: off/low/medium/high) with an assisted return
 *     arc so passive blocks clear the net,
 *   - hit FX: particles, audio "pock", HUD popups (HIT/SMASH/SWEET/EDGE),
 *   - the machine rally loop: machine feeds -> racket returns -> rally.
 *
 * Phase 4 (new):
 *   - phone-driven singles MATCH with real rules: 11 points, win by 2,
 *     serve rotates every 2 points (every point at deuce),
 *   - a serve flow (held ball above the server's blade; a swing serves;
 *     demo rackets serve themselves) solved to be LEGAL BY CONSTRUCTION:
 *     own-side bounce first, clean net clearance, deep receiver landing,
 *   - point adjudication: out / no return (double bounce) / own-side
 *     mishit / serve faults / net-serve LET replay / volley rule (with a
 *     "would have landed" check via the real-integrator prediction) —
 *     edge balls count as legal table bounces,
 *   - match events (start/serve/point/let/end) for the HUD scoreboard,
 *     banners and the match-end screen.
 *
 * World frame: player 1 owns the -Z end, camera looks toward +Z; player 2
 * owns the +Z end (base yaw of 180°), camera looks toward -Z.
 */

import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Sport } from '../_template/Sport.js';
import { TABLE, buildCourt } from './court.js';
import { buildRacket } from './racket.js';
import { TrailRibbon } from './trail.js';
import {
  stepBall, predictBall, magnusAccel, surfaceBounce, solveFlight, solveServe,
  createBallState, PHYS, TABLE_TOP,
} from './physics.js';
import { Ball } from './ball.js';
import { ImpactParticles } from './effects.js';
import { BallMachine } from './machine.js';
import { MatchState } from '../../core/match.js';

const PLAYER_ACCENTS = [0xff8a1e, 0x2fb4ff];
const HOME = [
  new THREE.Vector3(0, 1.02, -(TABLE.length / 2 + 0.24)),
  new THREE.Vector3(0, 1.02, TABLE.length / 2 + 0.24),
];
const BASE_YAW = [0, Math.PI];

// Camera per player: behind and slightly above their own end of the table.
const CAM_POS = [new THREE.Vector3(0, 1.55, -(TABLE.length / 2 + 1.45)), new THREE.Vector3(0, 1.55, TABLE.length / 2 + 1.45)];
const CAM_TARGET = new THREE.Vector3(0, 0.86, 0);

const BALL_POOL = 8;
const PRED_MAX = 128;
const UP = new THREE.Vector3(0, 1, 0);

/* Phase 3: racket-ball hit model ------------------------------------ */
const BALL_R = PHYS.ballRadius;
const HIT_E = 0.78;        // rubber normal restitution
const HIT_MU = 0.55;       // rubber friction -> spin transfer
const BLADE_R = 0.08;      // blade contact circle (7.5 cm blade + tolerance)
const BOOST_T = 0.16;      // s of extra blade velocity after a swing event
const BOOST_SCALE = 0.55;  // phone swing speed -> blade speed at contact
const HIT_CD = 0.09;       // s a ball ignores blades after being hit
const ASSIST_VALUES = { off: 0, low: 0.45, medium: 0.75, high: 0.95 };
const REACH_X = 0.75;      // assisted glide clamp around home
const REACH_Y = [0.84, 1.42]; // blade centre clamp (min keeps it off the table)

/* Phase 4: match flow ------------------------------------------------ */
const SERVE_HOLD = 0.1;    // m the serve ball floats in front of the blade
const SERVE_TIMEOUT = 6;   // s of serve-wait before the auto-serve kicks in
const POINT_BAN = 1.4;     // s between a decided point and the next serve

/* Stage B: hand-motion positioning reach clamp. Offsets are relative to the
 * slot's HOME, in the WORLD frame after the per-slot map (toward-net positive
 * z). The Y minimum keeps the blade off the table top (0.76 m) when reaching
 * over it; the assist's REACH_Y[0] (0.84) is the same idea. */
const HAND_REACH = {
  x: 0.85,               // ± sideways
  yLow: -0.18, yHigh: 0.40, // down / up from HOME.y = 1.02
  zBack: -0.35, zNet: 0.55, // away-from-net / toward-net
};

export class TableTennisSport extends Sport {
  static id = 'tableTennis';
  static label = 'Table Tennis';
  static maxPlayers = 2;

  constructor(ctx) {
    super(ctx);
    this.ctx = ctx; // { settings, hud, audio, net, renderer }
    this.simTime = 0;
    this.mode = 'practice';
    this.showPrediction = false; // driven by the F3 overlay
    this.stats = { bounces: 0, maxSpeed: 0, launches: 0, netHits: 0, hits: 0 };
    this.lastHit = null; // details of the most recent racket hit (debug/E2E)

    /* Phase 4: match flow. `match` is null outside the match mode; the
     * rally object describes THE match ball (there is exactly one). */
    this.match = null;
    this.matchPhase = 'idle'; // idle | serve-wait | rally | point | over
    this.onMatchEvent = null; // main.js -> HUD scoreboard/banners/end screen
    this._rally = {
      toucher: 0,       // slot that last legally struck the ball
      serveFlight: true, // the current flight is a serve (two-bounce rule)
      ownBounced: false, // serve has bounced on the server's half yet
      oppBounced: false, // bounced legally on the receiver's/toucher's side
      letServe: false,  // serve touched the net (replay if it lands in)
      done: true,       // the point is decided — no more adjudication
    };
    this._matchBallIdx = -1; // pool index of the live match ball
    this._serveHold = -1;    // pool index of the held (pre-serve) ball
    this._serveWaitT = 0;
    this._pointBanT = 0;
    this._rallyStuckT = 0; // rally watchdog (match ball vanished without events)
    this._curBallIdx = -1;   // set around stepBall so events know their ball

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x070a12);
    this.scene.fog = new THREE.Fog(0x070a12, 10, 26);

    // PBR reflections from a procedural room (no external HDRI files yet —
    // a real HDRI can replace this in Phase 6 with the same API).
    const pmrem = new THREE.PMREMGenerator(ctx.renderer);
    const envScene = new RoomEnvironment();
    this.scene.environment = pmrem.fromScene(envScene, 0.04).texture;
    this.scene.environmentIntensity = 0.45;
    pmrem.dispose();

    const { lights } = buildCourt(this.scene);
    this.lightRig = lights;

    this.rackets = new Map(); // slot -> rig
    for (const slot of [1, 2]) this.rackets.set(slot, this._createRacketRig(slot));

    /* ---------------- Phase 2: balls / machine / FX ---------------- */

    this.balls = [];
    for (let i = 0; i < BALL_POOL; i++) this.balls.push(new Ball(this.scene, i + 1));

    this.particles = new ImpactParticles();
    this.scene.add(this.particles.points);

    this.machine = new BallMachine(this.scene);
    this.machine.onLaunch = (launch) => this._spawnLaunch(launch);

    // prediction line (cyan) — pooled geometry, drawn while F3 is open
    this._predPositions = new Float32Array(PRED_MAX * 3);
    const predGeo = new THREE.BufferGeometry();
    predGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(this._predPositions, 3).setUsage(THREE.DynamicDrawUsage),
    );
    this.predLine = new THREE.Line(
      predGeo,
      new THREE.LineBasicMaterial({ color: 0x35e0ff, transparent: true, opacity: 0.55, depthWrite: false }),
    );
    this.predLine.frustumCulled = false;
    this.predLine.visible = false;
    this.scene.add(this.predLine);

    // physics events (one shared handler object, zero allocation per step)
    this._ballEv = {
      onBounce: (side, pos, impact, surface) => this._onBounce(side, pos, impact, surface),
      onNet: (kind, pos) => this._onNet(kind, pos),
    };

    // Preallocated scratch (hot path must not allocate).
    this._qTmp = new THREE.Quaternion();
    this._qRel = new THREE.Quaternion();
    this._vTmp = new THREE.Vector3();
    this._vTmp2 = new THREE.Vector3();
    this._prevPos = new Map();

    // Phase 3 hit model: per-ball previous position (for the swept blade
    // crossing), hit cooldowns, and preallocated blade/contact scratch.
    this._ballPrev = [];
    for (let i = 0; i < BALL_POOL; i++) this._ballPrev.push(new THREE.Vector3());
    this._hitCd = new Float32Array(BALL_POOL);
    this._bladePos = new THREE.Vector3();
    this._bladeQ = new THREE.Quaternion();
    this._nHit = new THREE.Vector3();
    this._vBlade = new THREE.Vector3();
    this._contact = new THREE.Vector3();
    this._local = new THREE.Vector3();
    this._vArc = new THREE.Vector3();
    this._to = new THREE.Vector3();
    this._qInv = new THREE.Quaternion();
    this._assistPt = new THREE.Vector3();
    this._spinSolve = new THREE.Vector3();
    // Phase 4: serve/volley scratch
    this._volleyState = createBallState();
    this._holdN = new THREE.Vector3();
    // Per-rig blade transform at physics rate (index = slot - 1)
    this._phPos = [new THREE.Vector3(), new THREE.Vector3()];
    this._phQ = [new THREE.Quaternion(), new THREE.Quaternion()];
    this._phVel = [new THREE.Vector3(), new THREE.Vector3()];

    this._inputs = new Map(); // slot -> RacketInput (filled per frame)
  }

  _createRacketRig(slot) {
    const accent = PLAYER_ACCENTS[slot - 1];
    const group = buildRacket({ accent });
    group.position.copy(HOME[slot - 1]);
    group.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), BASE_YAW[slot - 1]);
    this.scene.add(group);

    const trail = new TrailRibbon(accent);
    this.scene.add(trail.line);

    const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), group.position, 0.35, accent, 0.08, 0.045);
    arrow.line.material.transparent = true;
    arrow.cone.material.transparent = true;
    this._setArrowOpacity(arrow, 0);
    arrow.visible = false;
    this.scene.add(arrow);

    const marker = group.getObjectByName('bladeCenter');
    return {
      slot, group, trail, arrow, marker,
      impulse: new THREE.Vector3(),
      posSmooth: group.position.clone(),
      started: false,
      active: false,
      // Phase 3 blade tracking (world frame, physics rate)
      bladePos: group.position.clone(),
      prevBladePos: group.position.clone(),
      bladeQ: group.quaternion.clone(),
      prevBladeQ: group.quaternion.clone(),
      bladeVel: new THREE.Vector3(),
      boostVel: new THREE.Vector3(), // extra blade velocity from a swing event
      boostT: 0,
      assistTarget: HOME[slot - 1].clone(),
      assistValid: false,
    };
  }

  /* ---------------- Sport interface ---------------- */

  createCamera(slot) {
    const cam = new THREE.PerspectiveCamera(56, 1, 0.05, 60);
    cam.position.copy(CAM_POS[slot - 1]);
    cam.lookAt(CAM_TARGET);
    return cam;
  }

  getCameras() {
    return [this.createCamera(1), this.createCamera(2)];
  }

  getHudConfig() {
    return { perSlot: ['name', 'state'], popups: true };
  }

  setShadows(enabled, size) {
    this.lightRig.setShadows(enabled, size);
  }

  /* ---------------- Phase 2/4 public API ---------------- */

  /**
   * 'match' (split screen, phone-driven scoring) | 'practice' (single
   * camera + machine). The ball machine only belongs to practice.
   */
  setMode(mode) {
    this.mode = mode;
    this.machine.setVisible(mode === 'practice');
    if (mode !== 'match') this.exitMatch();
  }

  /** Start (or restart) a match. `firstServer` is 1 or 2. */
  startMatch(firstServer = 1) {
    this.match = new MatchState({ firstServer });
    this.matchPhase = 'idle';
    this._serveHold = -1;
    this._matchBallIdx = -1;
    this._rally.done = true;
    this._emitMatchEvent({ type: 'start', serve: firstServer });
    this._startServe();
  }

  /** Rematch: the first serve swaps sides (standard rotation). */
  rematch() {
    this.startMatch(this.match ? 3 - this.match.firstServer : 1);
  }

  /** Leave match flow (back to lobby/menu) — stops scoring and serves. */
  exitMatch() {
    this.match = null;
    this.matchPhase = 'idle';
    this._serveHold = -1;
    this._matchBallIdx = -1;
    this._rally.done = true;
  }

  _emitMatchEvent(ev) {
    if (this.onMatchEvent) this.onMatchEvent(ev);
  }

  setMachineEnabled(running) {
    this.machine.config.running = !!running;
    if (running) this.machine._timer = Math.min(this.machine._timer, 0.6);
  }

  /** Manual single ball (Space / panel button) — works while off too. */
  machineToss() {
    this.machine.tossOne();
  }

  resetBalls() {
    for (const b of this.balls) {
      b.state.active = false;
      b.state.restTimer = 0;
      b.setVisible(false);
    }
    this.predLine.visible = false;
  }

  activeBallCount() {
    let n = 0;
    for (const b of this.balls) if (b.state.active) n++;
    return n;
  }

  /** Debug overlay data for the most recently launched active ball. */
  getBallDebugInfo() {
    let best = null;
    for (const b of this.balls) {
      if (b.state.active && (!best || b.state.age < best.state.age)) best = b;
    }
    if (!best) return null;
    const s = best.state;
    const sp = s.spin;
    const ax = Math.abs(sp.x) >= Math.abs(sp.y) && Math.abs(sp.x) >= Math.abs(sp.z)
      ? (sp.x >= 0 ? '+x' : '-x')
      : Math.abs(sp.y) >= Math.abs(sp.z)
        ? (sp.y >= 0 ? '+y' : '-y')
        : (sp.z >= 0 ? '+z' : '-z');
    return {
      id: best.id,
      speedKmh: s.vel.length() * 3.6,
      spinRpm: sp.length() * 60 / (2 * Math.PI),
      axis: ax,
      magnus: magnusAccel(s.vel, s.spin),
      bounces: this.stats.bounces,
    };
  }

  /* ---------------- Fixed 240 Hz step ---------------- */

  /**
   * @param {number} dt fixed step (1/240)
   * @param {number} step index of this substep within the render frame's batch
   * @param {number} batch number of substeps in the batch (blade interpolation)
   */
  physicsStep(dt, step = 0, batch = 1) {
    this.simTime += dt;
    if (this.match) this._matchStep(dt);

    // machine feed + model animation
    this.machine.update(dt);

    // Blade transforms at physics rate: interpolate between the render
    // frames' start/end transforms using this substep's position in the
    // batch. Effective blade velocity = frame finite difference + the
    // decaying swing boost (from the phone's swing detector).
    const f = batch > 0 ? Math.min(1, (step + 1) / batch) : 1;
    for (let sl = 0; sl < 2; sl++) {
      const rig = this.rackets.get(sl + 1);
      if (rig.boostT > 0) rig.boostT = Math.max(0, rig.boostT - dt);
      this._phPos[sl].lerpVectors(rig.prevBladePos, rig.bladePos, f);
      this._phQ[sl].slerpQuaternions(rig.prevBladeQ, rig.bladeQ, f);
      this._phVel[sl].copy(rig.bladeVel);
      if (rig.boostT > 0) this._phVel[sl].addScaledVector(rig.boostVel, rig.boostT / BOOST_T);
    }

    // ball physics (deterministic, fixed dt) + racket contacts
    for (let i = 0; i < this.balls.length; i++) {
      const b = this.balls[i];
      if (!b.state.active) continue;
      if (this._hitCd[i] > 0) this._hitCd[i] -= dt;
      this._curBallIdx = i;
      this._ballPrev[i].copy(b.state.pos);
      stepBall(b.state, dt, this._ballEv);
      if (!b.state.active && i === this._matchBallIdx) this._onMatchBallGone();
      if (b.state.active && this._hitCd[i] <= 0) this._testBladeHits(i, b);
    }
    this._curBallIdx = -1;
  }

  /* ---------------- Phase 4: match flow ---------------- */

  /**
   * Per-substep match timers: pin the held serve ball above the server's
   * blade (and auto-serve after the timeout), countdown between points.
   */
  _matchStep(dt) {
    if (this.matchPhase === 'serve-wait') {
      this._serveWaitT += dt;
      const rig = this.rackets.get(this.match.serveSlot());
      const s = this.balls[this._serveHold].state;
      // float just in front of the blade face (blade local +Z)
      this._holdN.set(0, 0, 1).applyQuaternion(rig.bladeQ);
      s.pos.copy(rig.bladePos).addScaledVector(this._holdN, SERVE_HOLD);
      s.pos.y += 0.03;
      s.vel.set(0, 0, 0);
      s.spin.set(0, 0, 0);
      s.active = true;
      s.restTimer = 0;
      s.age = 0;
      if (this._serveWaitT >= SERVE_TIMEOUT) this._executeServe(9);
    } else if (this.matchPhase === 'point') {
      this._pointBanT -= dt;
      if (this._pointBanT <= 0) this._startServe();
    } else if (this.matchPhase === 'rally') {
      // Safety net: if the match ball ever vanishes without a decisive
      // bounce/net event (an exotic despawn path), the match would strand
      // in 'rally' forever. Adjudicate it with the same rules as a ball
      // that despawned without events (see _onMatchBallGone).
      const ball = this._matchBallIdx >= 0 ? this.balls[this._matchBallIdx] : null;
      if (!ball || !ball.state.active) {
        this._rallyStuckT += dt;
        if (this._rallyStuckT >= 0.05) {
          this._rallyStuckT = 0;
          this._onMatchBallGone();
        }
      } else {
        this._rallyStuckT = 0;
      }
    }
  }

  /** Begin the next serve: park a fresh ball above the current server. */
  _startServe() {
    if (!this.match || this.match.winner) return;
    this.resetBalls(); // clear the previous point's dead ball
    const ball = this._freeBall();
    const idx = this.balls.indexOf(ball);
    ball.setVisible(true);
    ball.resetVisual();
    this._hitCd[idx] = 0;
    this._matchBallIdx = idx;
    this._serveHold = idx;
    this._serveWaitT = 0;
    this._rallyStuckT = 0;
    // Park the held ball at the server's home BEFORE the first blade pin:
    // the pin in _matchStep refines it to the blade face every substep, but
    // a swing can serve in the very frame _startServe ran (update() runs
    // after the fixed batch) — solveServe must never read a stale dead-ball
    // position (floor / out of bounds) as its launch point.
    const server = this.match.serveSlot();
    const dirZ = server === 1 ? 1 : -1;
    const s = ball.state;
    s.pos.set(HOME[server - 1].x, HOME[server - 1].y + 0.03, HOME[server - 1].z + dirZ * SERVE_HOLD);
    s.vel.set(0, 0, 0);
    s.spin.set(0, 0, 0);
    s.active = true;
    s.restTimer = 0;
    s.age = 0;
    this.matchPhase = 'serve-wait';
    this._rally.done = true;
    this._rally.toucher = 0;
    this._emitMatchEvent({ type: 'serve-wait', slot: server });
  }

  /**
   * Strike the serve: the held ball launches with a LEGAL serve arc solved
   * against the real integrator (own-side bounce -> over the net -> deep
   * landing on the receiver's half — see solveServe). `power` is the
   * triggering swing's speed and shapes serve pace + spin.
   */
  _executeServe(power) {
    if (!this.match || this.matchPhase !== 'serve-wait' || this._serveHold < 0) return;
    const server = this.match.serveSlot();
    const dirZ = server === 1 ? 1 : -1;
    const ball = this.balls[this._serveHold];
    const s = ball.state;
    const serve = solveServe(s.pos, dirZ, power, 40 + power * 4);
    s.vel.set(serve.vel.x, serve.vel.y, serve.vel.z);
    s.spin.set(serve.spin.x, serve.spin.y, serve.spin.z);
    this._serveHold = -1;
    this.matchPhase = 'rally';
    const r = this._rally;
    r.toucher = server;
    r.serveFlight = true;
    r.ownBounced = false;
    r.oppBounced = false;
    r.letServe = false;
    r.done = false;
    this.stats.launches++;
    const v = s.vel.length();
    if (v > this.stats.maxSpeed) this.stats.maxSpeed = v;
    this.ctx.hud.popup(server, 'SERVE', 'accent');
    this._emitMatchEvent({ type: 'serve', slot: server });
  }

  /**
   * Decide a point for `slot` with a human-readable `reason`
   * ('out', 'no return', 'own side', 'serve fault', 'volley', 'volley out').
   */
  _awardPoint(slot, reason) {
    if (!this.match || this._rally.done) return;
    this._rally.done = true;
    const res = this.match.pointFor(slot);
    this.matchPhase = res.over ? 'over' : 'point';
    this._pointBanT = POINT_BAN;
    this._emitMatchEvent({
      type: 'point',
      slot,
      reason,
      score: [this.match.points[0], this.match.points[1]],
      serveSlot: this.match.serveSlot(),
      deuce: res.deuce,
      gamePointFor: res.gamePointFor,
    });
    if (res.over) {
      this._emitMatchEvent({
        type: 'end',
        winner: res.winner,
        score: [this.match.points[0], this.match.points[1]],
      });
    }
  }

  /**
   * Serve touched the net AND landed in — a let: replay the serve, no
   * point (the same server keeps serving because the score is unchanged).
   */
  _replayServe() {
    if (!this.match || this._rally.done) return;
    this._rally.done = true;
    this.matchPhase = 'point';
    this._pointBanT = 1.1;
    this._emitMatchEvent({ type: 'let', slot: this.match.serveSlot() });
  }

  /**
   * The match ball despawned (rolled to rest, or flew out of world bounds)
   * without a decisive bounce event: with a legal bounce already on the
   * receiver's side the receiver failed to return; otherwise the last
   * hitter's shot went out (or the serve faulted).
   */
  _onMatchBallGone() {
    const r = this._rally;
    if (!this.match || r.done || !r.toucher) return;
    if (r.oppBounced) this._awardPoint(r.toucher, 'no return');
    else this._awardPoint(3 - r.toucher, r.serveFlight ? 'serve fault' : 'out');
  }

  _spawnLaunch(launch) {
    const ball = this._freeBall();
    if (!ball) return;
    const s = ball.state;
    s.pos.copy(launch.pos);
    s.vel.copy(launch.vel);
    s.spin.copy(launch.spin);
    s.active = true;
    s.age = 0;
    s.restTimer = 0;
    ball.setVisible(true);
    ball.resetVisual();
    this.stats.launches++;
    const v = s.vel.length();
    if (v > this.stats.maxSpeed) this.stats.maxSpeed = v;
  }

  _freeBall() {
    for (const b of this.balls) if (!b.state.active) return b;
    // pool exhausted: recycle the oldest active ball
    let oldest = null;
    for (const b of this.balls) if (!oldest || b.state.age > oldest.state.age) oldest = b;
    oldest.state.active = false;
    return oldest;
  }

  /* ---------------- Phase 3: racket-ball hit model ---------------- */

  /**
   * Swept ball-vs-blade contact for one ball after its physics substep.
   * The blade is a disc (radius BLADE_R) moving along the segment
   * prev->current ball position; each face (front/back) is a plane at
   * ±ballRadius along the blade normal. Lateral offset must be inside the
   * blade circle — outside it the ball simply passes (that's the skill).
   */
  _testBladeHits(ballIdx, ball) {
    const s = ball.state;
    const prev = this._ballPrev[ballIdx];
    for (let sl = 0; sl < 2; sl++) {
      const rig = this.rackets.get(sl + 1);
      if (!rig.active) continue;
      const center = this._phPos[sl];

      // blade face normal (local +Z -> world)
      this._nHit.set(0, 0, 1).applyQuaternion(this._phQ[sl]);
      const nx = this._nHit.x, ny = this._nHit.y, nz = this._nHit.z;
      const d0 = (prev.x - center.x) * nx + (prev.y - center.y) * ny + (prev.z - center.z) * nz;
      const d1 = (s.pos.x - center.x) * nx + (s.pos.y - center.y) * ny + (s.pos.z - center.z) * nz;

      for (const side of [1, -1]) {
        const dc = side * BALL_R; // contact plane offset (surface touch)
        const a0 = d0 - dc, a1 = d1 - dc;
        if (a0 * side <= 0 || a1 * side > 0) continue; // not approaching this face
        const denom = d0 - d1;
        const fr = Math.abs(denom) < 1e-9 ? 0 : (d0 - dc) / denom;
        this._contact.lerpVectors(prev, s.pos, fr);

        // lateral offset inside the blade circle? (blade-local frame)
        this._local.copy(this._contact).sub(center);
        this._qInv.copy(this._phQ[sl]).invert();
        this._local.applyQuaternion(this._qInv);
        const off2 = this._local.x * this._local.x + this._local.y * this._local.y;
        if (off2 > BLADE_R * BLADE_R) continue;

        this._hitRespond(sl, rig, ball, ballIdx, side, Math.sqrt(off2) / BLADE_R);
        sl = 2; // one contact per substep
        break;
      }
    }
  }

  /**
   * Impulse response of a blade hit: rubber restitution + friction against
   * the MOVING blade (surfaceBounce with surface velocity), sweet-spot
   * power falloff, optional assisted return arc, FX + bookkeeping.
   */
  _hitRespond(slotIdx, rig, ball, ballIdx, side, offNorm) {
    const s = ball.state;
    const n = this._nHit;
    if (side < 0) n.multiplyScalar(-1); // back-face hit -> normal flips

    /* ---- Phase 4: volley rule (match ball only) ----
     * A receiver striking the ball before it has bounced on their own
     * half is a volley. Legal only when the ball was going OUT: predict
     * the untouched trajectory with the real integrator and check whether
     * it would have bounced on the volleyer's half. */
    if (
      this.match && ballIdx === this._matchBallIdx
      && !this._rally.done && this._rally.toucher
      && rig.slot !== this._rally.toucher && !this._rally.oppBounced
    ) {
      this._volleyState.pos.copy(s.pos);
      this._volleyState.vel.copy(s.vel);
      this._volleyState.spin.copy(s.spin);
      const pred = predictBall(this._volleyState, { maxT: 1.2, dt: 1 / 240 });
      // predictBall returns a SHARED singleton — read immediately.
      if (pred.hasBounce && pred.bounceSide === rig.slot) {
        // it was on course to land on their half: hitting it early is a fault
        this.ctx.hud.popup(rig.slot, 'VOLLEY!', 'bad');
        this._awardPoint(3 - rig.slot, 'volley');
      } else {
        // the ball was going out: striking it early wins the point
        this.ctx.hud.popup(rig.slot, 'VOLLEY WIN', 'good');
        this._awardPoint(rig.slot, 'volley out');
      }
      return; // no hit response — the point is decided
    }

    // effective blade velocity at contact
    const vB = this._vBlade.copy(this._phVel[slotIdx]);

    // impulse response against the moving blade
    let impact = surfaceBounce(s, n.x, n.y, n.z, HIT_E, HIT_MU, vB.x, vB.y, vB.z);
    if (impact <= 0) impact = 0.4; // grazing touch — still separate + report

    // sweet spot: power falls off toward the rim (scales the rebound only)
    const powerScale = 1 - 0.35 * offNorm * offNorm;
    this._vArc.copy(s.vel).sub(vB).multiplyScalar(powerScale);
    s.vel.copy(vB).add(this._vArc);

    // assisted return arc (Settings ▸ Assist). A live swing overrides it:
    // the player's own stroke character wins over the help.
    const boostFactor = rig.boostT > 0
      ? Math.min(1, (rig.boostVel.length() * (rig.boostT / BOOST_T)) / 5)
      : 0;
    const assist = this._assistValue(rig.slot) * (1 - boostFactor);
    if (assist > 0.02) this._applyAssistArc(rig.slot, s, assist * 0.8);

    // separate the ball from the blade plane
    s.pos.copy(this._contact).addScaledVector(n, BALL_R + 0.002);
    this._hitCd[ballIdx] = HIT_CD;

    // FX + feedback
    const strength = Math.min(12, impact + Math.max(0, vB.dot(n)) * 0.5 + 1);
    const count = Math.min(14, 4 + ((impact * 1.1) | 0));
    this.particles.emit(this._contact, n, count, 0xffe9b0, Math.min(1.6, strength / 7));
    this.ctx.audio.bounce(strength * 1.05);
    rig.impulse.addScaledVector(n, -0.025); // small kick-back

    const kmh = s.vel.length() * 3.6;
    const boosted = rig.boostT > 0 && rig.boostVel.length() > 3;
    // SMASH keys off the impact (blade drives the approach speed) — an
    // assisted return arc caps the outgoing speed below 40 km/h, so the
    // old outgoing-speed test could never fire under assist.
    let label = `HIT ${kmh.toFixed(0)} km/h`;
    if (boosted && impact > 5.5) label = `SMASH ${kmh.toFixed(0)} km/h`;
    else if (offNorm > 0.9) label = `EDGE ${kmh.toFixed(0)} km/h`;
    else if (offNorm < 0.35) label = `SWEET ${kmh.toFixed(0)} km/h`;
    this.ctx.hud.popup(rig.slot, label, 'accent');
    this.stats.hits++;
    const sp = s.vel.length();
    if (sp > this.stats.maxSpeed) this.stats.maxSpeed = sp;
    this.lastHit = {
      slot: rig.slot,
      kmh: +kmh.toFixed(1),
      offNorm: +offNorm.toFixed(2),
      boosted,
      impact: +impact.toFixed(2),
      spinRpm: Math.round(s.spin.length() * 60 / (2 * Math.PI)),
      t: +this.simTime.toFixed(3),
    };

    /* Phase 4: rally handover — the opponent's legal strike makes them the
     * toucher and clears the serve/bounce tracking for this flight. (Volley
     * faults and volley wins returned above, so reaching here means the
     * contact was legal and happened after the bounce on their half.) */
    if (
      this.match && ballIdx === this._matchBallIdx
      && !this._rally.done && rig.slot !== this._rally.toucher
    ) {
      const r = this._rally;
      r.toucher = rig.slot;
      r.serveFlight = false;
      r.ownBounced = false;
      r.oppBounced = false;
      r.letServe = false;
    }
  }

  /**
   * Assisted return arc: ALWAYS a legal shot — the drag-aware flight
   * solver produces a velocity that clears the net and lands on the
   * opponent's half, so assisted rallies don't die in the tape. The hit's
   * own character survives through speed (harder hits fly faster) and
   * aim (the outgoing direction bends the target), scaled by the assist
   * level `w`. Deterministic target variety from a hash of the contact.
   */
  _applyAssistArc(slot, s, w) {
    const dirZ = slot === 1 ? 1 : -1;
    const j = Math.sin(s.pos.x * 73.7 + s.pos.y * 151.1 + this.simTime * 11.7) * 0.5 + 0.5;
    const raw = s.vel.length();
    // speed: the hit's energy drives the arc; assist level normalizes it
    const spd = Math.min(10.5, Math.max(4.2, 4.2 + (raw - 4.2) * (0.35 + 0.5 * w)));
    // aim: lateral hit direction bends the target width, assist-scaled
    const aimX = (s.vel.x / Math.max(1, raw)) * 2.2 * (0.4 + 0.6 * w);
    const tx = Math.max(-0.42, Math.min(0.42, aimX - s.pos.x * 0.25));
    const tz = dirZ * (0.55 + 0.35 * j);
    // the assist damps spin — solve against the spin that will remain
    const keep = 1 - 0.45 * w;
    this._spinSolve.copy(s.spin).multiplyScalar(keep);
    const flight = solveFlight(s.pos, this._to.set(tx, TABLE_TOP + BALL_R, tz), spd, 0.05, this._spinSolve);
    s.vel.set(flight.vel.x, flight.vel.y, flight.vel.z);
    s.spin.multiplyScalar(keep);
  }

  /** Assist strength (0..1) from Settings for one slot. */
  _assistValue(slot) {
    const v = ASSIST_VALUES[this.ctx.settings.get('assist')];
    return v == null ? 0.75 : v;
  }

  /**
   * Debug/E2E: predict one ball's path and annotate every point with the
   * assist-scan conditions, so tuning can see exactly why a point was or
   * wasn't chosen (`pick` marks the selected hang point).
   */
  debugAssistScan(slot, ballIdx) {
    const b = this.balls[ballIdx];
    if (!b || !b.state.active) return null;
    const s = b.state;
    const sideSign = slot === 1 ? -1 : 1;
    const stepDt = 1 / 90;
    const out = predictBall(s, { maxT: 1.4, dt: stepDt });
    const chosen = this._scanIntercept(slot, s, this._assistPt);
    const rows = [];
    let bounced = s.vel.y > 0.2;
    for (let i = 1; i < out.count; i++) {
      const p = out.points[i];
      const vy = (p.y - out.points[i - 1].y) / stepDt;
      if (!bounced && vy > 0.5 && p.y > 0.74 && p.y < 0.95 && p.z * sideSign >= 0.45) bounced = true;
      rows.push({
        i,
        t: +(i * stepDt).toFixed(2),
        x: +p.x.toFixed(2), y: +p.y.toFixed(2), z: +p.z.toFixed(2),
        vy: +vy.toFixed(1),
        post: bounced,
        deep: p.z * sideSign >= 0.45,
        band: p.y >= REACH_Y[0] && p.y <= REACH_Y[1],
        pick: i === chosen,
      });
    }
    return { count: out.count, chosen, rows };
  }

  /* ---------------- Phase 4 debug/E2E API ---------------- */

  /** Match snapshot for the F3 overlay and E2E asserts. */
  debugMatchInfo() {
    if (!this.match) return null;
    return {
      points: [this.match.points[0], this.match.points[1]],
      firstServer: this.match.firstServer,
      serveSlot: this.match.serveSlot(),
      winner: this.match.winner,
      phase: this.matchPhase,
      rules: { ...this.match.rules },
    };
  }

  /** Live rule tuning for short E2E games: debugMatchRules({ pointsToWin: 3 }). */
  debugMatchRules(patch = {}) {
    if (!this.match) return null;
    if (patch.pointsToWin) this.match.rules.pointsToWin = patch.pointsToWin;
    if (patch.winBy) this.match.rules.winBy = patch.winBy;
    return { ...this.match.rules };
  }

  /** Skip the serve-wait (timeout or swing): serve right now (E2E). */
  debugForceServe() {
    if (this.match && this.matchPhase === 'serve-wait') this._executeServe(9);
  }

  /**
   * Fast-forward scoring (E2E only): award a point to `slot` through the
   * real MatchState so rotation/deuce/end logic can be asserted without
   * waiting for demo rallies. Skips the point-ban wait.
   */
  debugAwardPoint(slot) {
    if (!this.match || this.match.winner) return null;
    const res = this.match.pointFor(slot);
    if (res.over) {
      this.matchPhase = 'over';
      this._emitMatchEvent({ type: 'end', winner: res.winner, score: [this.match.points[0], this.match.points[1]] });
    }
    return {
      points: [this.match.points[0], this.match.points[1]],
      serveSlot: this.match.serveSlot(),
      ...res,
    };
  }

  /**
   * Stage B debug/E2E: the hand-motion estimator state for one slot
   * (positions in the calibrated frame, plus the mode/blend inputs).
   */
  debugHandInfo(slot) {
    const entry = this.ctx.slots && this.ctx.slots.get(slot);
    const src = entry && entry.source;
    const hf = src && src.handFilter;
    const rig = this.rackets.get(slot);
    if (!hf || !hf.started) return null;
    const posMode = this.ctx.settings.at('posMode', slot - 1) || 'assist';
    return {
      active: !!(entry && entry.input && entry.input.hand && entry.input.hand.active),
      mode: posMode,
      limited: !!src.limited,
      pos: { x: +hf.pos.x.toFixed(3), y: +hf.pos.y.toFixed(3), z: +hf.pos.z.toFixed(3) },
      vel: { x: +hf.vel.x.toFixed(3), y: +hf.vel.y.toFixed(3), z: +hf.vel.z.toFixed(3) },
      accel: { x: +hf.accel.x.toFixed(2), y: +hf.accel.y.toFixed(2), z: +hf.accel.z.toFixed(2) },
      raw: { x: +hf.raw.x.toFixed(2), y: +hf.raw.y.toFixed(2), z: +hf.raw.z.toFixed(2) },
      zupt: hf.zupt,
      stillMs: Math.round(hf.stillMs),
      omega: +hf.cfg.omega.toFixed(2),
      assistValid: rig ? rig.assistValid : false,
      assistTarget: rig ? { x: +rig.assistTarget.x.toFixed(3), y: +rig.assistTarget.y.toFixed(3), z: +rig.assistTarget.z.toFixed(3) } : null,
      racketPos: rig ? { x: +rig.posSmooth.x.toFixed(3), y: +rig.posSmooth.y.toFixed(3), z: +rig.posSmooth.z.toFixed(3) } : null,
    };
  }

  _onBounce(side, pos, impact, surface) {
    this.stats.bounces++;
    if (impact > this.stats.maxSpeed) this.stats.maxSpeed = impact;

    const color = surface === 'floor' ? 0x8899aa : 0xffc46b;
    const count = Math.min(16, 3 + ((impact * 1.3) | 0));
    this.particles.emit(pos, UP, count, color, Math.min(1.7, impact / 6));
    this.ctx.audio.bounce(impact);

    /* ---- Phase 4: point adjudication (match ball only) ---- */
    if (!this.match || this._curBallIdx !== this._matchBallIdx) return;
    const r = this._rally;
    if (r.done || !r.toucher) return;

    if (surface === 'floor') {
      // Ball hit the floor: after a legal bounce on the other side this is
      // a missed return; otherwise the shot (or serve) missed the table.
      if (r.oppBounced) this._awardPoint(r.toucher, 'no return');
      else this._awardPoint(3 - r.toucher, r.serveFlight ? 'serve fault' : 'out');
      return;
    }

    // Table OR edge bounce — an edge ball is a legal bounce (ITTF rules),
    // adjudicated identically but flagged for flavour.
    if (surface === 'edge') this.ctx.hud.popup(r.toucher, 'EDGE BALL', 'good');
    const own = side === r.toucher;

    if (r.serveFlight) {
      if (own) {
        // serve must bounce on the server's half first (exactly once)
        if (r.ownBounced) this._awardPoint(3 - r.toucher, 'serve fault');
        else r.ownBounced = true;
      } else if (r.ownBounced) {
        // legal serve: it reached the receiver's half
        r.oppBounced = true;
        r.serveFlight = false;
        if (r.letServe) this._replayServe(); // clipped the net en route
      } else {
        // crossed without the own-side bounce: fault
        this._awardPoint(3 - r.toucher, 'serve fault');
      }
    } else if (r.oppBounced) {
      // second bounce before a return — the receiver failed
      this._awardPoint(r.toucher, 'no return');
    } else if (own) {
      // the hitter's ball came down on their OWN side (net dribble/mishit)
      this._awardPoint(3 - r.toucher, 'own side');
    } else {
      r.oppBounced = true; // legal bounce — the receiver must return now
    }
  }

  _onNet(kind, pos) {
    this.stats.netHits++;
    this.particles.emit(pos, UP, 8, 0xdfe8ff, 0.8);
    this.ctx.audio.thud();
    const slot = pos.z < 0 ? 1 : 2;
    this.ctx.hud.popup(slot, kind === 'cord' ? 'NET CORD!' : 'NET', 'bad');

    /* Phase 4: a serve that touches the net becomes a LET if it still
     * lands legally (replay); rally net hits adjudicate via the bounce. */
    if (!this.match || this._curBallIdx !== this._matchBallIdx) return;
    const r = this._rally;
    if (r.done || !r.toucher) return;
    if (r.serveFlight && !r.oppBounced) r.letServe = true;
  }

  /* ---------------- Per-rendered-frame update ---------------- */

  /**
   * @param {number} dt render frame dt (s)
   * @param {number} _alpha fixed-step interpolation remainder
   * @param {number} nowMs server-epoch ms
   * @param {Map<number, {source: object, input: RacketInput}>} slots
   */
  update(dt, _alpha, nowMs, slots) {
    for (const [slot, entry] of slots) {
      const rig = this.rackets.get(slot);
      const { source, input } = entry;

      // Stage B: the hand-motion filter's spring rate + dead zone are live
      // settings (Return-to-center speed / Dead zone). Two writes per frame
      // is cheaper than a settings event fan-out and always current.
      const hf = source && source.handFilter;
      if (hf) {
        const w = this.ctx.settings.at('handReturn', slot - 1);
        const d = this.ctx.settings.at('handDead', slot - 1);
        if (w != null) hf.cfg.omega = w;
        if (d != null) hf.cfg.dead = d;
      }

      const ok = source.sample(nowMs, input);

      if (!ok || !input.fresh) {
        // Stale / disconnected: hide the racket and wait.
        rig.group.visible = false;
        rig.trail.setVisible(false);
        rig.arrow.visible = false;
        rig.started = false;
        rig.active = false;
        continue;
      }
      const first = !rig.started;
      if (first) {
        rig.started = true;
        rig.posSmooth.copy(HOME[slot - 1]);
      }
      rig.group.visible = true;
      rig.trail.setVisible(true);
      rig.active = true;

      // Blade tracking: this frame's start transform = last frame's end.
      rig.prevBladePos.copy(rig.group.position);
      rig.prevBladeQ.copy(rig.group.quaternion);

      this._applyOrientation(slot, rig, input, source);
      this._updateAssistTarget(slot, rig);
      this._applyPosition(slot, rig, input, dt);
      this._handleSwing(slot, rig, input, dt);

      // Phase 4: the SERVER's swing launches the held ball. The swing FX
      // and blade boost above still apply — a serve is just a hit with a
      // solved-legal arc on top. Power shapes the serve's pace + spin.
      if (
        this.match && this.matchPhase === 'serve-wait'
        && slot === this.match.serveSlot() && input.swingEvent
      ) {
        this._executeServe(input.swingEvent.power);
      }
      this._updateTrail(rig, dt);

      // Blade end-of-frame transform + velocity (finite difference).
      rig.bladePos.copy(rig.group.position);
      rig.bladeQ.copy(rig.group.quaternion);
      if (first) {
        rig.bladeVel.set(0, 0, 0); // no velocity spike on (re)spawn
      } else {
        rig.bladeVel.copy(rig.bladePos).sub(rig.prevBladePos).divideScalar(Math.max(dt, 1 / 240));
        const bvl = rig.bladeVel.length();
        if (bvl > 30) rig.bladeVel.multiplyScalar(30 / bvl);
      }
    }

    // balls + FX + prediction
    for (let i = 0; i < this.balls.length; i++) {
      const b = this.balls[i];
      if (!b.state.active) {
        b.setVisible(false);
      } else {
        b.syncFromState(dt);
      }
    }
    this.particles.update(dt);
    this._updatePrediction();
  }

  _updatePrediction() {
    if (!this.showPrediction) {
      if (this.predLine.visible) this.predLine.visible = false;
      return;
    }
    // first active ball travelling toward the P1 end (-z)
    let src = null;
    for (const b of this.balls) {
      if (b.state.active && b.state.vel.z < -0.5) { src = b.state; break; }
    }
    if (!src) {
      this.predLine.visible = false;
      return;
    }
    const out = predictBall(src, { maxT: 1.2, dt: 1 / 90 });
    const n = Math.min(out.count, PRED_MAX);
    const arr = this._predPositions;
    for (let i = 0; i < n; i++) {
      const p = out.points[i];
      arr[i * 3] = p.x;
      arr[i * 3 + 1] = p.y;
      arr[i * 3 + 2] = p.z;
    }
    this.predLine.geometry.setDrawRange(0, n);
    this.predLine.geometry.attributes.position.needsUpdate = true;
    this.predLine.visible = n > 1;
  }

  /* ---------------- Racket logic (Phase 1) ---------------- */

  /** Racket rotation = calibrated relative phone orientation (1:1 mapping). */
  _applyOrientation(slot, rig, input, source) {
    const settings = this.ctx.settings;
    // qRel = baseline⁻¹ * current  (device-relative rotation)
    const baseline = source && source.hasBaseline ? source.baseline : IDENTITY;
    this._qTmp.copy(baseline).invert();
    this._qRel.copy(this._qTmp).multiply(input.orientation);

    // Left-handed mirror (180° about the screen normal: swaps left/right and
    // up/down of the blade, keeps the face direction).
    if (settings.at('handed', slot - 1) === 'left') {
      this._qRel.multiply(this._mirrorZ());
    }
    // Invert-axes mirror (180° about world up).
    if (settings.at('invertAxes', slot - 1)) {
      this._qRel.premultiply(this._mirrorY());
    }

    this._qTmp.setFromAxisAngle(this._up(), BASE_YAW[slot - 1]);
    rig.group.quaternion.copy(this._qTmp).multiply(this._qRel);
  }

  _mirrorZ() {
    if (!this._qMz) this._qMz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
    return this._qMz;
  }
  _mirrorY() {
    if (!this._qMy) this._qMy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);
    return this._qMy;
  }
  _up() {
    if (!this._vUp) this._vUp = new THREE.Vector3(0, 1, 0);
    return this._vUp;
  }

  /**
   * Scan one approaching ball's predicted path for this slot's best
   * interception point: among the points AFTER its bounce on this player's
   * half that sit inside the blade's reach band, pick the one with the
   * LEAST vertical speed — the hang at the top of the bounce — because a
   * near-zero |vy| is the most forgiving moment to meet the ball (a real
   * player takes the ball at the top of the bounce too). Fills `out` with
   * the clamped interception point; returns the prediction index or -1.
   */
  _scanIntercept(slot, s, out) {
    const sideSign = slot === 1 ? -1 : 1; // own-half sign (P1 half is -z)
    const stepDt = 1 / 90; // prediction sample interval
    const pred = predictBall(s, { maxT: 1.4, dt: stepDt });
    // predictBall returns a SHARED singleton — scan + copy immediately.
    let bounced = s.vel.y > 0.2; // already rising: the bounce happened
    let bestI = -1;
    let bestVy = Infinity;
    for (let i = 1; i < pred.count; i++) {
      const p = pred.points[i];
      const vy = (p.y - pred.points[i - 1].y) / stepDt;
      if (!bounced) {
        // table bounce: vy flips strongly positive just above table height
        // AND on this player's own half (a SERVE's first bounce happens on
        // the SERVER's half — counting it would park the receiver at the
        // net to volley)
        if (vy > 0.5 && p.y > 0.74 && p.y < 0.95 && p.z * sideSign >= 0.45) bounced = true;
        else continue;
      }
      if (p.z * sideSign < 0.45) continue;       // own half, past the net
      if (p.y < REACH_Y[0] || p.y > REACH_Y[1]) continue;
      if (Math.abs(p.x) > REACH_X + BLADE_R) continue;
      const av = Math.abs(vy);
      if (av < bestVy) { bestVy = av; bestI = i; }
    }
    if (bestI < 0) return -1;
    const p = pred.points[bestI];
    out.set(
      Math.max(-REACH_X, Math.min(REACH_X, p.x)),
      Math.max(REACH_Y[0], Math.min(REACH_Y[1], p.y)),
      Math.max(-1.34, Math.min(1.34, p.z)),
    );
    return bestI;
  }

  /**
   * Assisted positioning: among all incoming balls take the SOONEST
   * hittable one and glide the racket to its interception point. With no
   * hittable ball the racket returns home.
   */
  _updateAssistTarget(slot, rig) {
    rig.assistTarget.copy(HOME[slot - 1]);
    rig.assistValid = false;
    if (this._assistValue(slot) <= 0.01) return;
    let bestT = Infinity;
    for (const b of this.balls) {
      const s = b.state;
      if (!s.active) continue;
      const approaching = slot === 1 ? s.vel.z < -0.5 : s.vel.z > 0.5;
      if (!approaching) continue;
      const idx = this._scanIntercept(slot, s, this._assistPt);
      if (idx >= 0 && idx / 90 < bestT) {
        bestT = idx / 90;
        rig.assistTarget.copy(this._assistPt);
        rig.assistValid = true;
      }
    }
  }

  /**
   * Positioning (Phase 1 touchpad glide + Phase 3 assist + Stage B hand
   * motion). Per-player Positioning mode from Settings:
   *   'assist'     — the Phase 3 behaviour: assist owns the target while a
   *                  hittable ball is incoming, else home + touchpad nudge.
   *   'motion'     — the hand-motion estimate drives the racket; while a
   *                  hittable ball is incoming the target blends toward the
   *                  assist target by the Assist strength
   *                  (lerp(handMotionPos, assistTarget, assistStrength)).
   *   'motionOnly' — hand motion + touchpad only; the assist never owns the
   *                  position. (The assisted RETURN ARC is a separate Phase 3
   *                  feature and still follows the Assist setting.)
   * The touchpad stays a fine adjustment on top of hand motion.
   */
  _applyPosition(slot, rig, input, dt) {
    const towardNet = slot === 1 ? 1 : -1;
    const settings = this.ctx.settings;
    const target = this._vTmp.copy(HOME[slot - 1]);
    const mode = settings.at('posMode', slot - 1) || 'assist';
    const hand = input.hand;
    const handOn = !!(hand && hand.active) && mode !== 'assist';

    if (handOn) {
      // calibrated-frame offset -> this slot's world frame
      const off = this._vTmp2.copy(hand.pos);
      // Left-handed grip mirror (same mirror the rotation uses, applied in
      // the calibration frame).
      if (settings.at('handed', slot - 1) === 'left') { off.x = -off.x; off.y = -off.y; }
      // Slot yaw: P2 stands at the other end — flip the lateral/depth axes.
      if (slot === 2) { off.x = -off.x; off.z = -off.z; }
      // Invert-axes is a WORLD-frame mirror (as in _applyOrientation).
      if (settings.at('invertAxes', slot - 1)) { off.x = -off.x; off.z = -off.z; }
      const sens = settings.at('handSens', slot - 1);
      off.multiplyScalar(sens == null ? 1 : sens);
      this._clampHandOffset(slot, off);
      target.add(off);
      // touchpad fine adjustment still applies
      target.x += input.touch.x * 0.85;
      target.z += towardNet * input.touch.y * 0.5;
      if (mode === 'motion' && rig.assistValid) {
        target.lerp(rig.assistTarget, this._assistValue(slot));
      }
    } else {
      target.x += input.touch.x * 0.85;
      target.z += towardNet * input.touch.y * 0.5;
      if (rig.assistValid) target.copy(rig.assistTarget);
    }

    rig.posSmooth.lerp(target, 1 - Math.exp(-dt / 0.12));

    // Idle bob for life (few mm).
    rig.group.position.copy(rig.posSmooth);
    rig.group.position.y += Math.sin((this.simTime + slot * 2.1) * 1.3) * 0.004;
    rig.group.position.add(rig.impulse);
  }

  /** Clamp a hand-motion WORLD offset to the reachable zone around HOME. */
  _clampHandOffset(slot, off) {
    off.x = Math.max(-HAND_REACH.x, Math.min(HAND_REACH.x, off.x));
    off.y = Math.max(HAND_REACH.yLow, Math.min(HAND_REACH.yHigh, off.y));
    // z is clamped along the toward-net axis so the range is per slot.
    const zNet = off.z * (slot === 1 ? 1 : -1);
    off.z = Math.max(HAND_REACH.zBack, Math.min(HAND_REACH.zNet, zNet)) * (slot === 1 ? 1 : -1);
    return off;
  }

  /** Swing event: lunge along the world-space swing direction + FX. */
  _handleSwing(slot, rig, input, dt) {
    // decay any active impulse
    if (rig.impulse.lengthSq() > 1e-8) {
      rig.impulse.multiplyScalar(Math.exp(-dt / 0.13));
      if (rig.impulse.length() < 0.004) rig.impulse.set(0, 0, 0);
    }
    // fade swing arrow
    if (rig.arrow.visible) {
      const o = this._arrowOpacity(rig.arrow) - dt * 4;
      this._setArrowOpacity(rig.arrow, o);
      if (o <= 0) rig.arrow.visible = false;
    }

    if (!input.swingEvent) return;

    const { power, dir } = input.swingEvent;
    // Device-frame swing direction -> world frame via the racket quaternion.
    const world = this._vTmp.copy(dir).applyQuaternion(rig.group.quaternion);
    const strength = 0.22 * Math.min(1.4, Math.max(0.35, power / 10));
    rig.impulse.copy(world).multiplyScalar(strength);

    // Phase 3: the same swing drives the hit model — for a short window
    // the blade's effective contact velocity gets this swing velocity.
    // `dir` is a UNIT vector; the blade speed comes from the swing power
    // (a 12 m/s phone swing -> ~6.6 m/s blade at contact).
    rig.boostVel.copy(world).multiplyScalar(BOOST_SCALE * Math.min(16, Math.max(1, power)));
    rig.boostT = BOOST_T;

    // Arrow flash from the blade centre.
    rig.marker.getWorldPosition(rig.arrow.position);
    rig.arrow.setDirection(world);
    rig.arrow.setLength(0.25 + Math.min(0.4, power / 18), 0.08, 0.045);
    this._setArrowOpacity(rig.arrow, 1);
    rig.arrow.visible = true;

    this.ctx.audio.swing(power);
    // No popup here: the hit popup (HIT/SMASH/SWEET/EDGE) is the payoff and
    // a fast stream of swing events would overwrite it. The swing already
    // shows via the arrow flash, audio and the HUD chip's swing readout.
    this.ctx.hud.setSlot(slot, { lastSwing: power });
  }

  _setArrowOpacity(arrow, o) {
    const clamped = Math.max(0, Math.min(1, o));
    arrow.line.material.opacity = clamped;
    arrow.cone.material.opacity = clamped;
  }

  _arrowOpacity(arrow) {
    return arrow.line.material.opacity;
  }

  _updateTrail(rig, dt) {
    rig.marker.getWorldPosition(this._vTmp);
    const prev = this._prevPos.get(rig.slot) || this._vTmp.clone();
    const speed = prev.distanceTo(this._vTmp) / Math.max(1e-4, dt);
    this._prevPos.set(rig.slot, this._vTmp.clone());
    rig.trail.update(this._vTmp, speed, dt);
  }

  dispose() {
    this.scene.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) {
        for (const m of Array.isArray(o.material) ? o.material : [o.material]) {
          if (m.map) m.map.dispose();
          m.dispose();
        }
      }
    });
    this.scene.clear();
  }
}

const IDENTITY = new THREE.Quaternion();
