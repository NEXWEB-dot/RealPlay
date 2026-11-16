/**
 * Sport interface contract.
 *
 * Every sport (table tennis now; padel and badminton later) implements this
 * interface. The core (renderer, split-screen, loop, net, input, UI) is
 * sport-agnostic: it drives a Sport instance and reads its config.
 *
 * THE RULES OF THE ARCHITECTURE
 * -----------------------------
 * 1. The core never imports sport internals — it calls only this interface.
 * 2. All gameplay state lives inside the sport module.
 * 3. Player input arrives exclusively as RacketInput objects (see
 *    client/core/input.js). The sport never reads raw phone data.
 * 4. Scoring/rules are expressed as plain data (see `rules` below) so the
 *    HUD and match-end screens stay generic.
 *
 * To add a sport (e.g. badminton):
 *   client/sports/badminton/BadmintonSport.js extends Sport, then register
 *   it in client/main.js (SPORTS registry) with its menu entry + config.
 */

export class Sport {
  /** Unique id used for settings keys and net routing. */
  static id = 'template';

  /** Human name + max players (2 for singles sports, 4 for doubles). */
  static label = 'Template';
  static maxPlayers = 2;

  /**
   * @param {object} ctx { settings, hud, audio, net }
   */
  constructor(_ctx) {
    if (this.constructor === Sport) {
      throw new Error('Sport is an abstract interface.');
    }
  }

  /** Build court + environment into a fresh THREE.Scene. */
  createCourt(_scene) {}

  /** Create the game object (ball / shuttlecock) and its pooled trail/FX. */
  createBall(_scene) {}

  /**
   * Create (or reuse) the racket model for a player slot. Slots may be
   * human, bot or empty; the sport owns the visual.
   * @returns {THREE.Object3D}
   */
  createRacket(_slot, _scene) { return null; }

  /**
   * Fixed-timestep simulation at 240 Hz (deterministic, accumulator-driven).
   * Table tennis: ball physics with gravity/drag/Magnus + swept collisions.
   * (_step, _batch) locate this substep inside the current render frame's
   * batch — used to interpolate fast-moving objects (racket blades) between
   * the frame's start and end transforms at physics rate.
   */
  physicsStep(_dt, _step, _batch) {}

  /**
   * Per-rendered-frame gameplay update. `inputs` maps slot -> RacketInput
   * (fresh=false means the source is stale/disconnected). `alpha` is the
   * fixed-step interpolation remainder.
   */
  update(_dt, _alpha, _nowMs, _inputs) {}

  /**
   * Cameras for the split-screen manager, index-aligned with slots.
   * 2 players -> two cameras; 4 -> four.
   * @returns {THREE.PerspectiveCamera[]}
   */
  getCameras() { return []; }

  /**
   * HUD configuration consumed by the generic HUD:
   *   { perSlot: ['name','score','serve','speed','spin'], popups: true }
   */
  getHudConfig() { return { perSlot: [], popups: false }; }

  /** Rules/scoring descriptor (Phase 4 fills this for table tennis):
   *  {
   *    serveRotation: 'every2Points', deuceAt: 10, winScore: 11, winBy: 2,
   *    faults: ['out','net','doubleBounce','notReturned','serveFault'],
   *    onServe, onBounce(side), onHit(player, speed, spin), onPoint(winner, reason)
   *  }
   */
  getRules() { return null; }

  /** Bot brain factory for a slot (Phase 5). */
  createBot(_slot, _difficulty) { return null; }

  /** Shadows config hook (quality preset changes). */
  setShadows(_enabled, _size) {}

  dispose() {}
}
