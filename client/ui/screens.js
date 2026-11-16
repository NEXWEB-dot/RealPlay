/**
 * UI screens: main menu, lobby (QR + room code + slots), settings, pause.
 * Plain DOM, no framework. Every screen is created once and shown/hidden.
 */

import QRCode from 'qrcode';
import { QUALITY_PRESETS } from '../core/settings.js';

const $ = (tag, cls, html) => {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (html != null) el.innerHTML = html;
  return el;
};

export class Screens {
  /**
   * @param {object} ctx { settings, hud, audio, net, hooks }
   * hooks: { onPlay2P, onPlayPractice, onMachineConfig, onMachineToss,
   *          onStartMatch, onQuitToLobby, onQuitToMenu, onRematch,
   *          onResume, applyGraphics }
   */
  constructor(ctx) {
    this.ctx = ctx;
    this.root = document.createElement('div');
    this.root.id = 'ui';
    document.body.appendChild(this.root);

    this.menuEl = this._buildMenu();
    this.lobbyEl = this._buildLobby();
    this.settingsEl = this._buildSettings();
    this.pauseEl = this._buildPause();
    this.machineEl = this._buildMachinePanel();
    this.matchEndEl = this._buildMatchEnd();
    for (const el of [this.menuEl, this.lobbyEl, this.settingsEl, this.pauseEl, this.machineEl, this.matchEndEl]) {
      this.root.appendChild(el);
    }

    this._ipChoice = '';
    this._urlOverride = '';
  }

  _show(el) {
    for (const e of [this.menuEl, this.lobbyEl, this.settingsEl, this.pauseEl, this.matchEndEl]) {
      e.classList.toggle('hidden', e !== el);
    }
  }
  hideAll() { this._show(null); }

  /* ---------------------------------------------------------------- */
  /* Main menu                                                        */
  /* ---------------------------------------------------------------- */

  _buildMenu() {
    const el = $('div', 'screen');
    const card = $('div', 'menu-card');
    card.appendChild($('div', 'logo', 'REAL<span>PLAY</span>'));
    card.appendChild($('div', 'tagline', 'Your phone is the racket. Scan, calibrate, swing.'));

    const list = $('div', 'menu-list');

    const b2p = $('button', 'menu-btn', '<span>2 Players</span><span class="soon">MATCH</span>');
    b2p.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onPlay2P(); });

    const bBot = $('button', 'menu-btn', '<span>Player vs Bot</span><span class="soon">PHASE 5</span>');
    bBot.disabled = true;

    const bPrac = $('button', 'menu-btn', '<span>Practice / Rally</span><span class="soon">BALL MACHINE</span>');
    bPrac.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onPlayPractice(); });

    const bSet = $('button', 'menu-btn', '<span>Settings</span><span></span>');
    bSet.addEventListener('click', () => { this.ctx.audio.click(); this.openSettings(); });

    list.append(b2p, bBot, bPrac, bSet);
    card.appendChild(list);
    card.appendChild($('div', 'menu-foot', '<span>F3 — debug overlay · ESC pauses</span><span>Phase 4 build</span>'));
    el.appendChild(card);
    return el;
  }

  openMenu() { this._show(this.menuEl); }

  /* ---------------------------------------------------------------- */
  /* Lobby                                                            */
  /* ---------------------------------------------------------------- */

  _buildLobby() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'panel');
    card.style.maxWidth = 'min(94vw, 780px)';
    const wrap = $('div', 'lobby');

    const left = $('div', 'lobby-left');
    left.appendChild($('div', 'room-code-label', 'ROOM CODE'));
    this.codeEl = $('div', 'room-code', '----');
    left.appendChild(this.codeEl);
    this.qrCanvas = $('canvas');
    this.qrCanvas.id = 'qr-canvas';
    left.appendChild(this.qrCanvas);
    this.linkInput = $('input');
    this.linkInput.readOnly = true;
    this.linkInput.style.flex = '1';
    const ipSelect = $('select');
    ipSelect.style.maxWidth = '150px';
    this.ipSelect = ipSelect;
    const linkRow = $('div', 'linkrow');
    linkRow.append(ipSelect, this.linkInput);
    left.appendChild(linkRow);
    const copyBtn = $('button', 'btn small', 'Copy link');
    copyBtn.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(this.linkInput.value);
        this.ctx.hud.toast('Link copied');
      } catch { this.ctx.hud.toast('Copy failed — select the text'); }
    });
    left.appendChild(copyBtn);

    const right = $('div', 'lobby-right');
    this.slotCards = [1, 2].map((slot) => {
      const c = $('div', 'slot-card');
      c.innerHTML = `
        <div class="slot-head"><div class="slot-dot" id="dot-${slot}"></div><div class="slot-name">Player ${slot}</div></div>
        <div class="slot-meta" id="meta-${slot}">waiting for a phone…</div>
        <div class="slot-actions"><button class="btn small" id="buzz-${slot}" disabled>Test buzz</button></div>`;
      right.appendChild(c);
      c.querySelector(`#buzz-${slot}`).addEventListener('click', () => {
        this.ctx.net.sendHaptic(slot, 'tap');
        this.ctx.audio.click();
      });
      return c;
    });

    const actions = $('div', 'lobby-actions');
    const back = $('button', 'btn danger', 'Close room');
    back.addEventListener('click', () => { this.ctx.audio.click(); this.ctx.hooks.onQuitToMenu(); });
    const enter = $('button', 'btn primary', 'Start match');
    enter.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onStartMatch(); });
    actions.append(back, enter);
    right.appendChild(actions);

    wrap.append(left, right);
    card.appendChild(wrap);
    el.appendChild(card);
    return el;
  }

  async openLobby() {
    this._show(this.lobbyEl);
    const net = this.ctx.net;
    this.codeEl.textContent = net.roomCode || '----';
    await this._refreshQr();
    this._refreshSlots();
  }

  async _refreshQr() {
    const net = this.ctx.net;
    const ipSelect = this.ipSelect;
    const ips = net.lanIps.length ? net.lanIps : [location.hostname];
    ipSelect.innerHTML = ips.map((ip) => `<option value="${ip}" ${ip === this._ipChoice ? 'selected' : ''}>${ip}</option>`).join('');
    if (!this._ipChoice || !ips.includes(this._ipChoice)) {
      this._ipChoice = ips[0];
    }
    ipSelect.onchange = () => {
      this._ipChoice = ipSelect.value;
      this._refreshQr();
    };

    const port = location.port ? ':' + location.port : '';
    const url = `${location.protocol}//${this._ipChoice}${port}/controller/index.html?room=${net.roomCode}`;
    this.linkInput.value = url;
    try {
      await QRCode.toCanvas(this.qrCanvas, url, {
        width: 208,
        margin: 1,
        color: { dark: '#0b1020', light: '#ffffff' },
      });
    } catch {
      this.ctx.hud.toast('QR generation failed');
    }
  }

  _refreshSlots() {
    const net = this.ctx.net;
    for (const slot of [1, 2]) {
      const s = net.slots.find((x) => x.slot === slot);
      const dot = document.getElementById(`dot-${slot}`);
      const meta = document.getElementById(`meta-${slot}`);
      const buzz = document.getElementById(`buzz-${slot}`);
      if (!dot) continue;
      if (s && s.connected) {
        dot.classList.add('on');
        meta.innerHTML = `<span>${s.name}</span><span>${s.device}</span><span>${s.battery !== 65535 ? s.battery + '%' : ''}</span><span>${s.rtt} ms</span>`;
        buzz.disabled = false;
      } else {
        dot.classList.remove('on');
        meta.textContent = 'waiting for a phone…';
        buzz.disabled = true;
      }
    }
  }

  /** Called by main when a lobby update arrives while the lobby is open. */
  onLobbyUpdate() {
    if (!this.lobbyEl.classList.contains('hidden')) this._refreshSlots();
  }

  /* ---------------------------------------------------------------- */
  /* Settings                                                         */
  /* ---------------------------------------------------------------- */

  _buildSettings() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'panel');
    card.style.minWidth = '520px';
    card.appendChild($('div', 'logo', 'Set<span>tings</span>'));
    const grid = $('div', 'settings-grid');
    const s = this.ctx.settings;

    const addSelect = (label, key, options, applyNow) => {
      const lab = $('label', null, label);
      const sel = $('select');
      for (const [value, text] of options) {
        const opt = $('option', null, text);
        opt.value = value;
        if (String(s.get(key)) === String(value)) opt.selected = true;
        sel.appendChild(opt);
      }
      sel.addEventListener('change', () => {
        s.set(key, sel.value);
        applyNow?.();
      });
      grid.append(lab, sel);
      return sel;
    };

    const addRange = (label, key, min, max, step, fmt, extraEl) => {
      const lab = $('label', null, label);
      const row = $('div', 'row2');
      const input = $('input');
      input.type = 'range';
      input.min = min; input.max = max; input.step = step;
      input.value = s.get(key);
      const val = $('span', 'val', fmt(s.get(key)));
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        s.set(key, v);
        val.textContent = fmt(v);
      });
      row.append(input, val);
      if (extraEl) row.appendChild(extraEl);
      grid.append(lab, row);
    };

    const addCheck = (label, key, applyNow) => {
      const lab = $('label', null, label);
      const box = $('div', 'check');
      const input = $('input');
      input.type = 'checkbox';
      input.checked = !!s.get(key);
      input.addEventListener('change', () => {
        s.set(key, input.checked);
        applyNow?.();
      });
      box.appendChild(input);
      box.appendChild($('span', null, 'on'));
      grid.append(lab, box);
    };

    /* Per-player variants (Stage B positioning settings are per player). */
    const addSelectAt = (label, key, index, options) => {
      const lab = $('label', null, label);
      const sel = $('select');
      for (const [value, text] of options) {
        const opt = $('option', null, text);
        opt.value = value;
        if (String(s.at(key, index)) === String(value)) opt.selected = true;
        sel.appendChild(opt);
      }
      sel.addEventListener('change', () => s.setAt(key, index, sel.value));
      grid.append(lab, sel);
      return sel;
    };

    const addRangeAt = (label, key, index, min, max, step, fmt) => {
      const lab = $('label', null, label);
      const row = $('div', 'row2');
      const input = $('input');
      input.type = 'range';
      input.min = min; input.max = max; input.step = step;
      input.value = s.at(key, index);
      const val = $('span', 'val', fmt(s.at(key, index)));
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        s.setAt(key, index, v);
        val.textContent = fmt(v);
      });
      row.append(input, val);
      grid.append(lab, row);
    };

    this._addSection(grid, 'Graphics');
    this.qSel = addSelect('Quality preset', 'quality',
      [['low', 'Low'], ['medium', 'Medium'], ['high', 'High']],
      () => this.ctx.hooks.applyGraphics());
    addCheck('Dynamic resolution', 'dynamicRes', () => this.ctx.hooks.applyGraphics());
    addCheck('Shadows', 'shadows', () => this.ctx.hooks.applyGraphics());
    addCheck('Post-processing', 'postFx', () => {}); // active from Phase 6

    this._addSection(grid, 'Audio');
    addRange('Master volume', 'master', 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`);
    addRange('SFX volume', 'sfx', 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`);
    addRange('Music volume', 'music', 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`);

    this._addSection(grid, 'Controls');
    addRange('Swing sensitivity', 'sensitivity', 0.5, 2, 0.05, (v) => `${v.toFixed(2)}x`);
    addRange('Swing threshold', 'swingThreshold', 8, 60, 1, (v) => `${v} m/s²`);
    addSelect('Assist strength', 'assist', [['off', 'Off'], ['low', 'Low'], ['medium', 'Medium'], ['high', 'High']]);

    this._addSection(grid, 'Positioning (per player)');
    for (const slot of [1, 2]) {
      const i = slot - 1;
      addSelectAt(`P${slot} positioning`, 'posMode', i, [
        ['assist', 'Assist only'],
        ['motion', 'Hand motion + assist'],
        ['motionOnly', 'Hand motion only'],
      ]);
      addRangeAt(`P${slot} hand sensitivity`, 'handSens', i, 0.4, 2, 0.05, (v) => `${v.toFixed(2)}x`);
      addRangeAt(`P${slot} return-to-center`, 'handReturn', i, 0, 2, 0.05, (v) => (v === 0 ? 'off (park anywhere)' : `${v.toFixed(2)}`));
      addRangeAt(`P${slot} motion dead zone`, 'handDead', i, 0, 1, 0.05, (v) => `${v.toFixed(2)} m/s²`);
    }

    const note = $('div', 'settings-note',
      'Assist and swing parameters drive the hit model from Phase 3. Hand motion (Stage B) moves the racket with your physical hand: hold the phone still and the position holds (no drift); recalibrating re-centres it. A phone without a gyroscope falls back to tilt-only tracking automatically.');

    const actions = $('div', 'lobby-actions');
    const reset = $('button', 'btn danger', 'Reset defaults');
    reset.addEventListener('click', () => {
      s.reset();
      this.openSettings(); // rebuild values
      this.ctx.hooks.applyGraphics();
    });
    const close = $('button', 'btn primary', 'Done');
    close.addEventListener('click', () => { this.ctx.audio.click(); this.closeSettings(); });
    actions.append(reset, close);

    card.append(grid, note, actions);
    el.appendChild(card);
    return el;
  }

  _addSection(grid, title) {
    // Section headers span both grid columns.
    const h = $('h3', 'section', title);
    h.style.gridColumn = '1 / -1';
    grid.appendChild(h);
  }

  openSettings() {
    this._prevScreen = this.settingsEl.classList.contains('hidden')
      ? (this.menuEl.classList.contains('hidden') ? this.lobbyEl : this.menuEl)
      : this._prevScreen;
    this._show(this.settingsEl);
  }

  closeSettings() {
    this._show(this._prevScreen || this.menuEl);
  }

  /* ---------------------------------------------------------------- */
  /* Pause                                                            */
  /* ---------------------------------------------------------------- */

  _buildPause() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'menu-card');
    card.appendChild($('div', 'logo', 'Pau<span>sed</span>'));
    const list = $('div', 'menu-list');
    const resume = $('button', 'menu-btn', '<span>Resume</span><span>ESC</span>');
    resume.addEventListener('click', () => this.ctx.hooks.onResume());
    const settings = $('button', 'menu-btn', '<span>Settings</span><span></span>');
    settings.addEventListener('click', () => this.openSettings());
    this.pauseQuitEl = $('button', 'menu-btn', '<span>Quit to lobby</span><span></span>');
    this.pauseQuitEl.addEventListener('click', () => this.ctx.hooks.onQuitToLobby());
    list.append(resume, settings, this.pauseQuitEl);
    card.appendChild(list);
    el.appendChild(card);
    return el;
  }

  /** Context label for the pause quit button ('Quit to menu' in practice). */
  setPauseQuitLabel(text) {
    if (this.pauseQuitEl) this.pauseQuitEl.firstChild.textContent = text;
  }

  openPause() { this._show(this.pauseEl); }
  closePause() { this.hideAll(); }

  /* ---------------------------------------------------------------- */
  /* Match end (Phase 4)                                              */
  /* ---------------------------------------------------------------- */

  _buildMatchEnd() {
    const el = $('div', 'screen hidden');
    const card = $('div', 'menu-card');
    card.appendChild($('div', 'logo', 'Match <span>over</span>'));
    this.endScoreEl = $('div', 'end-score', '0 — 0');
    card.appendChild(this.endScoreEl);
    this.endWinnerEl = $('div', 'end-winner', '');
    card.appendChild(this.endWinnerEl);
    const list = $('div', 'menu-list');
    const again = $('button', 'menu-btn', '<span>Rematch</span><span class="soon">SERVER SWAPS</span>');
    again.addEventListener('click', () => { this.ctx.audio.click('good'); this.ctx.hooks.onRematch(); });
    const quit = $('button', 'menu-btn', '<span>Back to lobby</span><span></span>');
    quit.addEventListener('click', () => { this.ctx.audio.click(); this.ctx.hooks.onQuitToLobby(); });
    list.append(again, quit);
    card.appendChild(list);
    el.appendChild(card);
    return el;
  }

  /** `winnerName` won the game `a`—`b`. */
  showMatchEnd(winnerName, a, b) {
    this.endWinnerEl.textContent = `${winnerName} wins`;
    this.endScoreEl.textContent = `${a} — ${b}`;
    this._show(this.matchEndEl);
  }

  /* ---------------------------------------------------------------- */
  /* Ball machine panel (practice mode)                                */
  /* ---------------------------------------------------------------- */

  _buildMachinePanel() {
    const el = $('div', 'machine-panel hidden');
    el.appendChild($('h4', null, 'BALL MACHINE'));

    const rows = {};
    const addSlider = (label, key, min, max, step, fmt) => {
      const row = $('div', 'machine-row');
      row.appendChild($('span', null, label));
      const input = $('input');
      input.type = 'range';
      input.min = min; input.max = max; input.step = step;
      const val = $('span', 'val');
      input.addEventListener('input', () => {
        const v = parseFloat(input.value);
        val.textContent = fmt(v);
        this.ctx.hooks.onMachineConfig({ [key]: v });
      });
      row.append(input, val);
      el.appendChild(row);
      rows[key] = { input, val, fmt };
    };

    addSlider('speed', 'speed', 2, 16, 0.5, (v) => `${v.toFixed(1)} m/s`);
    addSlider('spin', 'spinRpm', 0, 3000, 100, (v) => `${v} rpm`);
    addSlider('rate', 'interval', 0.4, 3, 0.05, (v) => `${v.toFixed(2)} s`);

    const rowS = $('div', 'machine-row');
    rowS.appendChild($('span', null, 'spin type'));
    const sel = $('select');
    for (const [v, t] of [
      ['mix', 'mix'], ['topspin', 'topspin'], ['backspin', 'backspin'],
      ['sidespinL', 'side L'], ['sidespinR', 'side R'], ['none', 'none'],
    ]) {
      const o = $('option', null, t);
      o.value = v;
      sel.appendChild(o);
    }
    sel.addEventListener('change', () => this.ctx.hooks.onMachineConfig({ spinType: sel.value }));
    rowS.appendChild(sel);
    rowS.appendChild($('span', 'val', ''));
    el.appendChild(rowS);
    rows.spinType = { el: sel };

    const rowR = $('div', 'machine-row');
    rowR.appendChild($('span', null, 'randomize'));
    const chk = $('input');
    chk.type = 'checkbox';
    chk.addEventListener('change', () => this.ctx.hooks.onMachineConfig({ randomize: chk.checked }));
    rowR.append(chk, $('span', 'val', ''));
    el.appendChild(rowR);
    rows.randomize = { el: chk };

    const btns = $('div', 'machine-btns');
    this.machineToggleBtn = $('button', 'btn small primary', 'STOP');
    this.machineToggleBtn.addEventListener('click', () => {
      const on = !(this._machineCfg && this._machineCfg.running);
      this.ctx.hooks.onMachineConfig({ running: on });
      this._setMachineToggle(on);
    });
    const toss = $('button', 'btn small', 'Toss one (SPACE)');
    toss.addEventListener('click', () => this.ctx.hooks.onMachineToss());
    btns.append(this.machineToggleBtn, toss);
    el.appendChild(btns);

    this.machineStatusEl = $('div', 'machine-status', 'idle');
    el.appendChild(this.machineStatusEl);

    this._machineRows = rows;
    return el;
  }

  _setMachineToggle(on) {
    this.machineToggleBtn.textContent = on ? 'STOP' : 'START';
    this.machineToggleBtn.classList.toggle('primary', on);
  }

  showMachinePanel(v) {
    this.machineEl.classList.toggle('hidden', !v);
  }

  /** Push the sport's machine config into the panel controls. */
  syncMachinePanel(cfg) {
    this._machineCfg = cfg;
    const r = this._machineRows;
    if (!r) return;
    r.speed.input.value = cfg.speed;
    r.speed.val.textContent = r.speed.fmt(cfg.speed);
    r.spinRpm.input.value = cfg.spinRpm;
    r.spinRpm.val.textContent = r.spinRpm.fmt(cfg.spinRpm);
    r.interval.input.value = cfg.interval;
    r.interval.val.textContent = r.interval.fmt(cfg.interval);
    r.spinType.el.value = cfg.spinType;
    r.randomize.el.checked = !!cfg.randomize;
    this._setMachineToggle(!!cfg.running);
  }

  updateMachineStatus(text) {
    this.machineStatusEl.textContent = text;
  }
}
