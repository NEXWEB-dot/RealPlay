/**
 * REALPLAY PC client — entry point.
 *
 * Flow: main menu -> "2 Players" -> lobby (QR + room code) -> match
 * (split-screen, live phone-driven rackets, real scoring). ESC pauses, F3
 * toggles the debug overlay.
 *
 * Everything sport-specific lives behind the Sport interface; this file
 * wires core + net + UI and delegates gameplay to TableTennisSport.
 */

import { Settings, QUALITY_PRESETS } from './core/settings.js';
import { RendererManager } from './core/renderer.js';
import { GameLoop } from './core/loop.js';
import { ViewportManager } from './core/splitScreen.js';
import { AudioMgr } from './core/audio.js';
import { RacketInput, ControllerInputSource, DemoInputSource } from './core/input.js';
import { CameraTracker } from './core/tracker/cameraTracker.js';
import { NetClient } from './net/client.js';
import { Hud } from './ui/hud.js';
import { Screens } from './ui/screens.js';
import { DebugOverlay } from './ui/debugOverlay.js';
import { TableTennisSport } from './sports/tableTennis/TableTennisSport.js';
import './ui/ui.css';

const settings = new Settings();
const audio = new AudioMgr(settings);
const rendererMgr = new RendererManager(settings);
const hud = new Hud(document.body);
hud.hide();

const net = new NetClient({
  onRoom: () => screens.openLobby(),
  onLobby: () => {
    screens.onLobbyUpdate();
    refreshSlots();
  },
  onCtrlGone: ({ slot }) => {
    hud.toast(`Player ${slot} disconnected — waiting for reconnect…`);
    hud.setSlot(slot, { state: 'wait' });
  },
  onEv: ({ slot, name }) => {
    if (name === 'calibrated') {
      audio.click('good');
      hud.popup(slot, 'CALIBRATED ✓', 'good');
      net.sendStatus(slot, 'Calibrated ✓ — rally ready!', 'good');
      net.sendHaptic(slot, 'tap');
    }
  },
  onError: (msg) => {
    hud.toast(msg.message || 'Server error', 4000);
  },
});

/* ------------------------------------------------------------------ */
/* Stage C: webcam marker tracker (fusion positioning source)           */
/* ------------------------------------------------------------------ */

const tracker = new CameraTracker(settings);

/* ------------------------------------------------------------------ */
/* Player slots: source (phone / demo / bot) + RacketInput per slot     */
/* ------------------------------------------------------------------ */

/** slot -> { source, input } */
const slots = new Map();

const sport = new TableTennisSport({ settings, hud, audio, renderer: rendererMgr.renderer, slots });
sport.setShadows(settings.get('shadows'), QUALITY_PRESETS[settings.get('quality')].shadowMapSize);

const viewports = new ViewportManager(rendererMgr.renderer);
const debugOverlay = new DebugOverlay({
  getRendererInfo: () => {
    const info = rendererMgr.info();
    info.pixelRatio = rendererMgr.pixelRatio;
    return info;
  },
  getSources: () => slots.size ? Array.from(slots.entries()).map(([slot, e]) => ({ slot, source: e.source })) : [],
  getResScale: () => rendererMgr.scale,
  getBallInfo: () => (state === 'match' || state === 'practice') ? sport.getBallDebugInfo() : null,
  // Stage B: per-slot hand-motion estimator state for the overlay
  getHandInfo: (slot) => {
    const entry = slots.get(slot);
    const src = entry && entry.source;
    const hf = src && src.handFilter;
    const rig = sport.rackets.get(slot);
    if (!hf || !hf.started) return null;
    const input = entry.input;
    const hand = input && input.hand;
    const fused = !!(hand && hand.fused);
    return {
      // When the fusion KF is the active output, show its state (the Stage B
      // filter keeps integrating underneath and is shown via adopt/fallback).
      pos: fused ? hand.pos : hf.pos,
      vel: fused ? hand.vel : hf.vel,
      accel: hf.accel, raw: hf.raw,
      zupt: hf.zupt, stillMs: hf.stillMs,
      fused,
      limited: src.limited,
      mode: settings.at('posMode', slot - 1) || 'assist',
      assistValid: rig ? rig.assistValid : false,
      assistTarget: rig ? rig.assistTarget : null,
      racket: rig ? rig.posSmooth : null,
    };
  },
  // Stage C: per-slot webcam tracking status for the overlay
  getTrackInfo: (slot) => ({
    status: tracker.status(slot),
    ageMs: tracker.lastMeasAgeMs(slot),
    fps: tracker.fps,
    hasHome: tracker.hasHome(slot),
  }),
});

/* ------------------------------------------------------------------ */
/* Player slots: source (phone / demo / bot) + RacketInput per slot     */
/* ------------------------------------------------------------------ */

function getSlotEntry(slot) {
  let entry = slots.get(slot);
  if (!entry) {
    entry = { source: null, input: new RacketInput() };
    slots.set(slot, entry);
  }
  return entry;
}

function refreshSlots() {
  const maxYaw = sport.constructor.maxPlayers || 2;
  for (let slot = 1; slot <= 2; slot++) {
    const connected = net.slots.some((s) => s.slot === slot && s.connected);
    const entry = getSlotEntry(slot);

    if (connected) {
      if (!(entry.source instanceof ControllerInputSource)) {
        if (entry.source instanceof DemoInputSource) net.registerSource(slot, null);
        if (entry.source && entry.source.dispose) entry.source.dispose();
        entry.source = new ControllerInputSource(slot, net, tracker, settings);
        net.registerSource(slot, entry.source);
        // 'wait' only on the join transition — the heartbeat lobby refresh
        // (every 2 s) must not stomp the frame loop's live/stale chip.
        const s = net.slots.find((x) => x.slot === slot);
        hud.setSlot(slot, { name: s?.name || `P${slot}`, state: 'wait' });
      }
    } else if (!(entry.source instanceof DemoInputSource)) {
      net.registerSource(slot, null);
      if (entry.source && entry.source.dispose) entry.source.dispose();
      entry.source = new DemoInputSource(slot === 1 ? 0 : Math.PI);
      hud.setSlot(slot, { name: `Player ${slot}`, state: 'demo' });
    }
    if (slot > maxYaw) break;
  }
}

/* ------------------------------------------------------------------ */
/* App state machine                                                   */
/* ------------------------------------------------------------------ */

let state = 'menu'; // menu | lobby | match | practice
let paused = false;

const screens = new Screens({
  settings,
  hud,
  audio,
  net,
  tracker,
  hooks: {
    onPlay2P: () => {
      state = 'lobby';
      net.createRoom();
      // openLobby happens in onRoom when the server answers
    },
    onPlayPractice: () => {
      state = 'practice';
      paused = false;
      screens.hideAll();
      hud.show();
      hud.hideAllSlots();
      refreshSlots();
      sport.resetBalls();
      sport.setMode('practice');
      sport.setMachineEnabled(true);
      viewports.setLayout(1, [sport.createCamera(1)]);
      screens.showMachinePanel(true);
      screens.syncMachinePanel(sport.machine.config);
      hud.toast('Practice — SPACE tosses a ball · B toggles the machine');
    },
    onMachineConfig: (patch) => {
      Object.assign(sport.machine.config, patch);
    },
    onMachineToss: () => {
      sport.machineToss();
    },
    onStartMatch: () => {
      state = 'match';
      paused = false;
      screens.hideAll();
      screens.showMachinePanel(false);
      hud.show();
      refreshSlots();
      sport.resetBalls();
      sport.setMode('match');
      sport.setMachineEnabled(false);
      viewports.setLayout(2, [sport.createCamera(1), sport.createCamera(2)]);
      sport.startMatch(1);
      hud.toast('Match — first to 11 · swing to serve · ESC pauses');
    },
    onRematch: () => {
      if (state !== 'match') return;
      screens.hideAll();
      sport.rematch(); // 'start' event refreshes the scoreboard + banner
    },
    onQuitToLobby: () => {
      paused = false;
      hud.hideAllSlots();
      if (state === 'practice') {
        // practice pauses into "quit to menu"
        state = 'menu';
        sport.setMachineEnabled(false);
        sport.resetBalls();
        screens.showMachinePanel(false);
        hud.hide();
        screens.openMenu();
        return;
      }
      state = 'lobby';
      sport.exitMatch();
      hud.showScore(false);
      sport.setMachineEnabled(false);
      sport.resetBalls();
      screens.showMachinePanel(false);
      screens.openLobby();
    },
    onQuitToMenu: () => {
      state = 'menu';
      paused = false;
      net.close();
      sport.exitMatch();
      hud.showScore(false);
      sport.setMachineEnabled(false);
      sport.resetBalls();
      screens.showMachinePanel(false);
      hud.hideAllSlots();
      hud.hide();
      screens.openMenu();
    },
    onResume: () => {
      paused = false;
      screens.closePause();
    },
    applyGraphics: () => {
      rendererMgr.apply();
      sport.setShadows(settings.get('shadows'), QUALITY_PRESETS[settings.get('quality')].shadowMapSize);
    },
  },
});

/* ------------------------------------------------------------------ */
/* Phase 4: match events -> scoreboard / banners / end screen           */
/* ------------------------------------------------------------------ */

const POINT_LABEL = {
  out: 'OUT',
  'no return': 'NO RETURN',
  'own side': 'OWN SIDE',
  'serve fault': 'SERVE FAULT',
  volley: 'VOLLEY',
  'volley out': 'VOLLEY OUT',
};

/** Display name for a slot: the phone's name when connected, else Player N. */
function slotName(slot) {
  const s = net.slots.find((x) => x.slot === slot);
  return (s && s.connected && s.name) ? s.name : `Player ${slot}`;
}

function onMatchEvent(ev) {
  switch (ev.type) {
    case 'start':
      hud.showScore(true);
      hud.setScoreNames(slotName(1), slotName(2));
      hud.updateScore(ev.score || [0, 0], ev.serve);
      hud.banner('MATCH START', 1200);
      break;
    case 'serve-wait':
      hud.setScoreNames(slotName(1), slotName(2));
      if (sport.match) hud.updateScore(sport.match.points, ev.slot);
      break;
    case 'point': {
      hud.updateScore(ev.score, ev.serveSlot, ev.gamePointFor);
      const label = POINT_LABEL[ev.reason] || ev.reason.toUpperCase();
      hud.popup(ev.slot, `POINT — ${label}`, 'accent');
      if (ev.deuce) hud.banner('DEUCE', 1400);
      else if (ev.gamePointFor) hud.banner(`GAME POINT — ${slotName(ev.gamePointFor)}`, 1400);
      break;
    }
    case 'let':
      hud.toast('LET — serve clipped the net, replaying…');
      break;
    case 'end': {
      const winner = slotName(ev.winner);
      hud.updateScore(ev.score);
      hud.banner(`${winner.toUpperCase()} WINS ${ev.score[0]}–${ev.score[1]}`, 1800, 'win');
      setTimeout(() => {
        // still in this match (not quit/paused into something else)?
        if (state === 'match' && sport.match && sport.match.winner) {
          screens.showMatchEnd(winner, ev.score[0], ev.score[1]);
        }
      }, 1800);
      break;
    }
  }
}

sport.onMatchEvent = onMatchEvent;

screens.openMenu();

window.addEventListener('keydown', (e) => {
  if (e.key === 'F3') {
    e.preventDefault();
    debugOverlay.toggle();
    sport.showPrediction = debugOverlay.visible;
  } else if (e.key === 'Escape') {
    if (state === 'match' || state === 'practice') {
      if (paused) {
        paused = false;
        screens.closePause();
      } else if (screens.settingsEl.classList.contains('hidden')) {
        paused = true;
        screens.setPauseQuitLabel(state === 'practice' ? 'Quit to menu' : 'Quit to lobby');
        screens.openPause();
      }
    }
  } else if (e.code === 'Space') {
    // Practice only — in a match the SPACE ball machine doesn't belong.
    if (state === 'practice' && !paused) {
      e.preventDefault();
      sport.machineToss();
    }
  } else if (e.key === 'b' || e.key === 'B') {
    if (state === 'practice' && !paused) {
      const on = !sport.machine.config.running;
      sport.setMachineEnabled(on);
      screens.syncMachinePanel(sport.machine.config);
      hud.toast(on ? 'Ball machine ON' : 'Ball machine OFF');
    }
  }
});

window.addEventListener('blur', () => {
  if ((state === 'match' || state === 'practice') && !paused) {
    paused = true;
    screens.openPause();
  }
});

// Returning to the window auto-resumes (the pause was caused by losing focus).
window.addEventListener('focus', () => {
  if ((state === 'match' || state === 'practice') && paused && screens.settingsEl.classList.contains('hidden')) {
    paused = false;
    screens.closePause();
  }
});

// Debug hook for the overlay/e2e tooling (never used by gameplay).
window.__RP = { net, slots, sport, state: () => state, settings, hud, tracker };

/* ------------------------------------------------------------------ */
/* Game loop                                                           */
/* ------------------------------------------------------------------ */

const loop = new GameLoop({
  fixedDt: 1 / 240,
  fixed: (dt, step, batch) => {
    if (!paused && (state === 'match' || state === 'practice')) sport.physicsStep(dt, step, batch);
  },
  frame: (dt, alpha, nowMs) => {
    rendererMgr.update(dt);
    if ((state === 'match' || state === 'practice') && !paused) {
      sport.update(dt, alpha, nowMs, slots);

      // HUD chips + machine status (cheap DOM updates at ~5 Hz)
      hudTimer -= dt;
      if (hudTimer <= 0) {
        hudTimer = 0.2;
        if (state === 'practice') {
          const st = sport.stats;
          screens.updateMachineStatus(
            `${sport.activeBallCount()} in play · ${st.bounces} bounces · ${st.hits} hits · peak ${st.maxSpeed.toFixed(1)} m/s`,
          );
        } else {
          for (const [slot, entry] of slots) {
            const s = net.slots.find((x) => x.slot === slot);
            const live = entry.source && entry.source.isLive;
            if (entry.source instanceof DemoInputSource) {
              hud.setSlot(slot, { name: `Player ${slot}`, state: 'demo' });
            } else if (live) {
              const rtt = entry.source.slotRtt ? entry.source.slotRtt() : 0;
              const batt = entry.source.battery;
              const pps = entry.source.buffer ? Math.round(packetsPerSec(entry.source.buffer, nowMs)) : 0;
              hud.setSlot(slot, {
                name: s?.name || `P${slot}`,
                state: 'live',
                rtt,
                battery: batt,
                pps,
                tilt: !!entry.source.limited,
              });
              // Stage B: one-time notice when a phone reports no gyroscope.
              if (entry.source.limited && !tiltNotified.has(entry.source)) {
                tiltNotified.add(entry.source);
                hud.toast(`${s?.name || `P${slot}`}: limited sensors — tilt-only motion tracking`, 4000);
              }
            } else if (entry.source && entry.source.everReceived) {
              // packets flowed before but stopped -> real stutter
              hud.setSlot(slot, { state: 'stale' });
            }
            // else: joined but never streamed — keep the 'wait' chip
          }
        }
      }
    }
    viewports.render(sport.scene);
    debugOverlay.frame(dt, nowMs);
  },
});

let hudTimer = 0;
const pktRates = new Map();
const tiltNotified = new WeakSet(); // sources already toasted about NO_GYRO
function packetsPerSec(buffer, nowMs) {
  let p = pktRates.get(buffer);
  if (!p) {
    p = { last: buffer.pushes, pps: 0, at: nowMs };
    pktRates.set(buffer, p);
  }
  if (nowMs - p.at >= 1000) {
    p.pps = buffer.pushes - p.last;
    p.last = buffer.pushes;
    p.at = nowMs;
  }
  return p.pps;
}

loop.start();
