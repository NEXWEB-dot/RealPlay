/**
 * HUD: per-viewport status chips, shot pop-ups, toasts.
 * Generic (sport-agnostic): the sport feeds labels via setSlot()/popup().
 */

export class Hud {
  constructor(root) {
    this.root = document.createElement('div');
    this.root.id = 'hud';
    this.root.classList.add('pass');
    root.appendChild(this.root);

    this.halves = new Map();
    for (const slot of [1, 2, 3, 4]) {
      const half = document.createElement('div');
      half.className = `hud-half h${slot} hidden`;
      const name = document.createElement('div');
      name.className = 'hud-chip';
      name.innerHTML = `<span class="p">P${slot}</span><b class="nm">—</b>`;
      const stat = document.createElement('div');
      stat.className = 'hud-chip stat';
      half.append(name, stat);
      const popup = document.createElement('div');
      popup.className = 'hud-popup';
      popup.style.left = slot % 2 === 1 ? '25%' : '75%';
      popup.style.transform = 'translateX(-50%)';
      popup.style.top = '26%';
      half.after(popup);
      this.root.appendChild(half);
      this.root.appendChild(popup);
      this.halves.set(slot, { half, nameEl: name.querySelector('.nm'), statEl: stat, popup });
    }

    this.toastEl = document.createElement('div');
    this.toastEl.id = 'toast';
    this.root.appendChild(this.toastEl);
    this._toastTimer = 0;
    this._slotData = new Map();

    /* Phase 4: scoreboard (top centre) + big banner. Serving side shows a
     * lit dot; game-point side glows. Set from main.js via onMatchEvent. */
    this.scoreEl = document.createElement('div');
    this.scoreEl.id = 'score';
    this.scoreEl.classList.add('hidden');
    this.scoreEl.innerHTML =
      '<div class="sc-side" id="sc-side1"><span class="sc-serve" id="sc-serve1"></span>' +
      '<span class="sc-name" id="sc-name1">PLAYER 1</span><span class="sc-pts" id="sc-pts1">0</span></div>' +
      '<div class="sc-colon">:</div>' +
      '<div class="sc-side" id="sc-side2"><span class="sc-pts" id="sc-pts2">0</span>' +
      '<span class="sc-name" id="sc-name2">PLAYER 2</span><span class="sc-serve" id="sc-serve2"></span></div>';
    this.root.appendChild(this.scoreEl);
    this.scoreRefs = {
      side1: this.scoreEl.querySelector('#sc-side1'),
      side2: this.scoreEl.querySelector('#sc-side2'),
      name1: this.scoreEl.querySelector('#sc-name1'),
      name2: this.scoreEl.querySelector('#sc-name2'),
      pts1: this.scoreEl.querySelector('#sc-pts1'),
      pts2: this.scoreEl.querySelector('#sc-pts2'),
      serve1: this.scoreEl.querySelector('#sc-serve1'),
      serve2: this.scoreEl.querySelector('#sc-serve2'),
    };

    this.bannerEl = document.createElement('div');
    this.bannerEl.id = 'banner';
    this.root.appendChild(this.bannerEl);
    this._bannerT = 0;
  }

  show() { this.root.classList.remove('hidden'); }
  hide() { this.root.classList.add('hidden'); }

  /**
   * @param {object} data { name, state: 'demo'|'live'|'wait'|'stale',
   *                        battery, rtt, pps, lastSwing }
   */
  setSlot(slot, data) {
    const h = this.halves.get(slot);
    if (!h) return;
    const merged = { ...this._slotData.get(slot), ...data };
    this._slotData.set(slot, merged);

    h.half.classList.remove('hidden');
    if (merged.name != null) h.nameEl.textContent = merged.name;

    let stat = '';
    let wait = false;
    switch (merged.state) {
      case 'demo': stat = 'DEMO · wait for phone'; wait = false; break;
      case 'live':
        stat = [
          merged.rtt != null ? `${merged.rtt} ms` : null,
          merged.battery != null && merged.battery !== 65535 ? `🔋${merged.battery}%` : null,
          merged.pps != null ? `${merged.pps}/s` : null,
          merged.lastSwing != null ? `swing ${merged.lastSwing.toFixed(1)} m/s` : null,
          merged.tilt ? 'tilt-only' : null,
        ].filter(Boolean).join(' · ');
        break;
      case 'wait': stat = 'waiting for phone…'; wait = true; break;
      case 'stale': stat = 'CONNECTION STUTTER'; wait = true; break;
      default: stat = merged.state || '';
    }
    h.statEl.textContent = stat;
    h.statEl.classList.toggle('wait', wait);
  }

  popup(slot, text, kind = '') {
    const h = this.halves.get(slot);
    if (!h) return;
    h.popup.textContent = text;
    h.popup.className = `hud-popup show${kind ? ' kind-' + kind : ''}`;
    clearTimeout(h._t);
    h._t = setTimeout(() => h.popup.classList.remove('show'), 900);
  }

  toast(text, ms = 2200) {
    this.toastEl.textContent = text;
    this.toastEl.style.opacity = '1';
    clearTimeout(this._toastTimer);
    this._toastTimer = setTimeout(() => { this.toastEl.style.opacity = '0'; }, ms);
  }

  hideAllSlots() {
    for (const h of this.halves.values()) h.half.classList.add('hidden');
  }

  /* ---------------- Phase 4: match scoreboard ---------------- */

  showScore(v) {
    this.scoreEl.classList.toggle('hidden', !v);
  }

  setScoreNames(n1, n2) {
    this.scoreRefs.name1.textContent = n1;
    this.scoreRefs.name2.textContent = n2;
  }

  /**
   * Live score: `points` [P1, P2], `serveSlot` lights that side's serve dot,
   * `gamePointFor` glows the side that wins the match with the next point.
   */
  updateScore(points, serveSlot = 0, gamePointFor = 0) {
    this.scoreRefs.pts1.textContent = points[0];
    this.scoreRefs.pts2.textContent = points[1];
    this.scoreRefs.serve1.classList.toggle('on', serveSlot === 1);
    this.scoreRefs.serve2.classList.toggle('on', serveSlot === 2);
    this.scoreRefs.side1.classList.toggle('gp', gamePointFor === 1);
    this.scoreRefs.side2.classList.toggle('gp', gamePointFor === 2);
  }

  /** Big centre banner — auto-hides after `ms`, optional `kind` ('win'). */
  banner(text, ms = 1400, kind = '') {
    const el = this.bannerEl;
    el.textContent = text;
    el.className = kind ? `kind-${kind}` : '';
    void el.offsetWidth; // restart the transition on consecutive banners
    el.classList.add('show');
    clearTimeout(this._bannerT);
    this._bannerT = setTimeout(() => el.classList.remove('show'), ms);
  }
}
