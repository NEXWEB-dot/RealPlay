/**
 * Generic match state — sport-agnostic scoring used by the match modes.
 *
 * Rules follow standard table tennis singles and are shaped to generalize:
 *   - a game is won at `pointsToWin` points with a lead of at least `winBy`,
 *   - at deuce (both players one point from winning) the lead requirement
 *     still applies, so play continues until it is met,
 *   - the serve rotates every 2 points, and every point once at deuce
 *     (the ITTF rule), starting from `firstServer`.
 *
 * Doubles sports (Padel/Badminton, 2-4 players) can subclass or wrap this:
 * `points` is a plain per-slot array and `serveSlot()` encodes the rotation
 * policy in one place.
 */
export class MatchState {
  /**
   * @param {object} opts { pointsToWin, winBy, firstServer }
   */
  constructor({ pointsToWin = 11, winBy = 2, firstServer = 1 } = {}) {
    this.rules = { pointsToWin, winBy };
    this.firstServer = firstServer;
    this.points = [0, 0]; // index 0 = slot 1
    this.winner = 0;      // 0 while the match is live
  }

  get over() {
    return this.winner !== 0;
  }

  /**
   * Award one point to `slot` (1-based). Returns
   * { over, winner, deuce, gamePointFor } — `gamePointFor` is the slot that
   * wins the match with the next point (0 when none).
   */
  pointFor(slot) {
    if (this.winner) return { over: true, winner: this.winner, deuce: false, gamePointFor: 0 };
    const i = slot - 1;
    this.points[i]++;
    const a = this.points[0], b = this.points[1];
    const { pointsToWin, winBy } = this.rules;

    if (Math.max(a, b) >= pointsToWin && Math.abs(a - b) >= winBy) {
      this.winner = a > b ? 1 : 2;
      return { over: true, winner: this.winner, deuce: false, gamePointFor: 0 };
    }

    const deuce = a >= pointsToWin - 1 && b >= pointsToWin - 1;
    let gamePointFor = 0;
    for (const p of [1, 2]) {
      const mine = this.points[p - 1];
      const other = this.points[p === 1 ? 1 : 0];
      if (mine + 1 >= pointsToWin && mine + 1 - other >= winBy) { gamePointFor = p; break; }
    }
    return { over: false, winner: 0, deuce, gamePointFor };
  }

  /**
   * Who serves the next point: every 2 points, every point at deuce.
   * Deterministic from the score — no stored server index to desync.
   */
  serveSlot() {
    const [a, b] = this.points;
    const deuce = a >= this.rules.pointsToWin - 1 && b >= this.rules.pointsToWin - 1;
    const every = deuce ? 1 : 2;
    const idx = Math.floor((a + b) / every) % 2;
    return idx === 0 ? this.firstServer : 3 - this.firstServer;
  }

  /** Fresh game. `firstServer` defaults to keeping the current one. */
  reset(firstServer = this.firstServer) {
    this.firstServer = firstServer;
    this.points = [0, 0];
    this.winner = 0;
  }
}
