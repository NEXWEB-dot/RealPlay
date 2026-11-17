/**
 * REALPLAY phone controller.
 *
 * - Connects to the game server (same origin, /ws) with a room code.
 * - Streams a compact binary MOTION frame at up to 100 Hz:
 *     orientation (quaternion from alpha/beta/gamma, smoothed),
 *     linear acceleration (gravity removed), rotation rate (gyro),
 *     swing power, battery, clock-synced timestamp.
 * - Swing detection: linear acceleration magnitude crosses a threshold,
 *   opening a ~160 ms window; acceleration is leaky-integrated into a
 *   velocity estimate whose peak gives the swing power.
 * - Optional thumb touchpad sends normalized nudge vectors (TOUCH frames).
 * - Calibration: captures the "hold like a racket" reference pose. The PC
 *   averages the next 300 ms of samples as its baseline, so both ends agree.
 * - Latency: measured with a SYNC/SYNC_ACK round trip every 2 s; the clock
 *   offset lets the phone stamp frames in server time.
 *
 * No frameworks, no build-time dependencies beyond the shared protocol.
 */

import {
  WS_PATH, T, FLAGS, BATTERY_UNKNOWN,
  encodeMotion, encodeTouch, HAPTIC_PATTERNS,
} from '../shared/protocol.js';

/* ========================================================================= *
 * Tiny dependency-free quaternion math (right-handed, matches three.js).
 * ========================================================================= */

function eulerYXZToQuat(out, x, y, z) {
  // Exact port of three.js Quaternion.setFromEuler({ order: 'YXZ' })
  const c1 = Math.cos(x / 2), c2 = Math.cos(y / 2), c3 = Math.cos(z / 2);
  const s1 = Math.sin(x / 2), s2 = Math.sin(y / 2), s3 = Math.sin(z / 2);
  out[0] = s1 * c2 * c3 + c1 * s2 * s3;
  out[1] = c1 * s2 * c3 + s1 * c2 * s3;
  out[2] = c1 * c2 * s3 - s1 * s2 * c3;
  out[3] = c1 * c2 * c3 - s1 * s2 * s3;
}

function qMultiply(out, a, b) {
  const ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const bx = b[0], by = b[1], bz = b[2], bw = b[3];
  out[0] = ax * bw + aw * bx + ay * bz - az * by;
  out[1] = ay * bw + aw * by + az * bx - ax * bz;
  out[2] = az * bw + aw * bz + ax * by - ay * bx;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
}

function qSlerp(out, a, b, t) {
  let ax = a[0], ay = a[1], az = a[2], aw = a[3];
  let bx = b[0], by = b[1], bz = b[2], bw = b[3];
  let cos = ax * bx + ay * by + az * bz + aw * bw;
  if (cos < 0) { cos = -cos; bx = -bx; by = -by; bz = -bz; bw = -bw; }
  let s0, s1;
  if (cos > 0.9995) { s0 = 1 - t; s1 = t; }
  else {
    const theta = Math.acos(Math.min(1, cos));
    const sin = Math.sin(theta);
    s0 = Math.sin((1 - t) * theta) / sin;
    s1 = Math.sin(t * theta) / sin;
  }
  out[0] = ax * s0 + bx * s1;
  out[1] = ay * s0 + by * s1;
  out[2] = az * s0 + bz * s1;
  out[3] = aw * s0 + bw * s1;
  const n = 1 / Math.hypot(out[0], out[1], out[2], out[3]) || 1;
  out[0] *= n; out[1] *= n; out[2] *= n; out[3] *= n;
}

/** Rotate vector v (array [x,y,z]) by the INVERSE of quaternion q. */
function rotateByInverse(out, q, x, y, z) {
  const cx = -q[0], cy = -q[1], cz = -q[2], cw = q[3];
  // v' = q * v * q*
  const vx = cw * x + cy * z - cz * y;
  const vy = cw * y + cz * x - cx * z;
  const vz = cw * z + cx * y - cy * x;
  const tx = -cx * x - cy * y - cz * z;
  out[0] = vx * cw + tx * -cx + (vy * -cz - vz * -cy);
  out[1] = vy * cw + tx * -cy + (vz * -cx - vx * -cz);
  out[2] = vz * cw + tx * -cz + (vx * -cy - vy * -cx);
}

const deg2rad = Math.PI / 180;

/* ========================================================================= *
 * State
 * ========================================================================= */

const $ = (id) => document.getElementById(id);

const params = new URLSearchParams(location.search);
const S = {
  ws: null,
  wsOpen: false,
  room: (params.get('room') || '').toUpperCase(),
  wantedName: (params.get('name') || '').slice(0, 16),
  slot: 0,
  offsetMs: 0,       // serverTime ≈ clientTime + offsetMs
  rttMs: 0,
  // sensors
  sensorsOn: false,
  hasOrient: false,
  hasMotion: false,
  firstSensorAt: 0,
  qTarget: [0, 0, 0, 1],
  qSmooth: [0, 0, 0, 1],
  lastSmoothTs: 0,
  lastMotionTs: 0,
  screenAngle: 0,
  // no-gyro tilt fallback (Stage B): some phones lack a gyroscope — their
  // fused orientation drifts and swings are unusable, but tilt still works.
  noGyro: false,
  gyroDetected: false,
  gyroNullFrames: 0,   // devicemotion events without a rotationRate field
  gyroZeroFrames: 0,   // events with an exactly-zero rotation rate
  alphaFrozen: null,   // yaw locked at calibration (tilt-only mode)
  lastAlpha: 0,
  lastBeta: 0,
  lastGamma: 0,
  gFilt: [0, 0, 0],    // low-passed tilt-only gravity (device frame)
  gFiltInit: false,
  // linear acceleration (device frame)
  ax: 0, ay: 0, az: 0,
  gx: 0, gy: 0, gz: 0,
  // swing detector
  vx: 0, vy: 0, vz: 0,
  swingActive: false,
  swingStart: 0,
  swingPeak: 0,
  lastSwingPower: 0,
  lastSwingAt: 0,
  pendingSwingEvent: false,
  calibFlagUntil: 0,
  calSamplingUntil: 0,
  calSamples: [],
  // touchpad
  touch: { active: false, dx: 0, dy: 0, dirty: false },
  battery: BATTERY_UNKNOWN,
  charging: false,
  // loops
  sendTimer: null,
  syncTimer: null,
  syncPending: false,
  wakeLock: null,
  reconnectTimer: null,
};

const SWING_WINDOW_MS = 160;   // swing detection window
const SWING_MIN_POWER = 3.0;   // m/s, below this the "swing" is treated as noise
const SWING_DECAY_TAU = 0.25;  // s, leaky integrator time constant
const ORIENT_TAU = 0.035;      // s, orientation smoothing time constant
const ORIENT_TAU_TILT = 0.12;  // s, heavier smoothing without a gyroscope
const SEND_HZ = 100;           // max motion frames per second
const MAX_BUFFERED = 16384;    // bytes; skip sends while the socket is congested

// Scratch for the tilt-only gravity path (no per-event allocation).
const _gq = [0, 0, 0, 1];
const _gTilt = [0, 0, 0];
const _qScratch = [0, 0, 0, 1];

const epochNow = () => performance.timeOrigin + performance.now();

/* ========================================================================= *
 * UI helpers
 * ========================================================================= */

function show(screen) {
  $('screen-connect').classList.toggle('hidden', screen !== 'connect');
  $('screen-play').classList.toggle('hidden', screen !== 'play');
}

let toastTimer = 0;
function toast(text, ms = 1800) {
  const el = $('toast');
  el.textContent = text;
  el.classList.remove('hidden');
  el.style.opacity = '1';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.style.opacity = '0'; }, ms);
}

function setMainStatus(text, kind = '') {
  const el = $('status-main');
  el.textContent = text;
  el.className = 'status-main ' + kind;
}

function setSubStatus(text) {
  $('status-sub').textContent = text || '';
}

function deviceName() {
  const ua = navigator.userAgent;
  if (/iPhone/i.test(ua)) return 'iPhone';
  if (/iPad/i.test(ua)) return 'iPad';
  const m = ua.match(/Android[^;)]*;\s*([^;)]+build)/i);
  if (m) return m[1].replace(/build/i, '').trim().slice(0, 16);
  if (/Android/i.test(ua)) return 'Android';
  return 'Phone';
}

/* ========================================================================= *
 * Networking
 * ========================================================================= */

function wsUrl() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${proto}//${location.host}${WS_PATH}`;
}

function connect() {
  if (S.ws && (S.ws.readyState === 0 || S.ws.readyState === 1)) return;
  let ws;
  try {
    ws = new WebSocket(wsUrl());
    ws.binaryType = 'arraybuffer';
  } catch {
    scheduleReconnect();
    return;
  }
  S.ws = ws;

  ws.onopen = () => {
    ws.send(JSON.stringify({
      t: T.CTRL_HELLO,
      room: S.room,
      name: S.wantedName || deviceName(),
      device: `${deviceName()} ${screen.width}x${screen.height}`,
    }));
  };

  ws.onmessage = (e) => {
    if (e.data instanceof ArrayBuffer) return;
    let msg;
    try { msg = JSON.parse(e.data); } catch { return; }
    switch (msg.t) {
      case T.WELCOME:
        S.slot = msg.slot;
        $('slot-badge').textContent = `P${msg.slot}`;
        $('stat-room').textContent = `ROOM ${msg.code}`;
        setMainStatus('Connected!', 'good');
        setSubStatus('Hold the phone like a racket, then tap CALIBRATE.');
        startLoops();
        startSensorsIfNeeded();
        requestWakeLock();
        break;

      case T.STATUS:
        setMainStatus(msg.text || '', msg.kind === 'good' ? 'good' : msg.kind === 'bad' ? 'bad' : '');
        break;

      case T.HAPTIC: {
        const pattern = HAPTIC_PATTERNS[msg.pattern];
        if (pattern != null && navigator.vibrate) {
          try { navigator.vibrate(pattern); } catch { /* ignore */ }
        }
        break;
      }

      case T.SYNC_ACK: {
        const t2 = epochNow();
        const rtt = t2 - S.syncT0;
        if (rtt >= 0 && rtt < 5000) {
          S.rttMs = S.rttMs ? S.rttMs * 0.6 + rtt * 0.4 : rtt;
          const offset = msg.t1 - (S.syncT0 + rtt / 2);
          S.offsetMs = S.offsetMs ? S.offsetMs * 0.6 + offset * 0.4 : offset;
        }
        S.syncPending = false;
        break;
      }

      case T.ERROR:
        if (msg.code === 'unknown_room') {
          setMainStatus('Room not found', 'bad');
          setSubStatus(msg.message || '');
          stopReconnect();
        } else if (msg.code === 'room_closed') {
          setMainStatus('Room closed', 'bad');
          setSubStatus('The PC ended the session.');
          S.slot = 0;
        } else if (msg.code === 'room_full') {
          setMainStatus('Room is full', 'bad');
          setSubStatus(msg.message || '');
          stopReconnect();
        }
        break;
    }
  };

  ws.onclose = () => {
    S.wsOpen = false;
    S.ws = null;
    if (S.slot) {
      setMainStatus('Reconnecting…', 'bad');
      setSubStatus('Check your Wi-Fi connection.');
      S.slot = 0;
    }
    scheduleReconnect();
  };

  ws.onerror = () => { /* onclose follows */ };
}

function scheduleReconnect() {
  if (S.reconnectTimer) return;
  S.reconnectTimer = setInterval(() => {
    if (!S.ws || S.ws.readyState === 3) connect();
  }, 1200);
}

function stopReconnect() {
  clearInterval(S.reconnectTimer);
  S.reconnectTimer = null;
}

function startLoops() {
  if (!S.sendTimer) S.sendTimer = setInterval(sendTick, 1000 / SEND_HZ);
  if (!S.syncTimer) {
    S.syncTimer = setInterval(() => {
      if (!S.wsOpen || S.syncPending) return;
      S.syncPending = true;
      S.syncT0 = epochNow();
      safeSendJson({ t: T.SYNC, t0: S.syncT0 });
    }, 2000);
  }
}

function safeSendJson(obj) {
  if (S.ws && S.ws.readyState === 1) {
    try { S.ws.send(JSON.stringify(obj)); } catch { /* ignore */ }
  }
}

/* ========================================================================= *
 * Sensors
 * ========================================================================= */

function screenAngle() {
  if (screen.orientation && typeof screen.orientation.angle === 'number') {
    return screen.orientation.angle;
  }
  return window.orientation || 0;
}

const needsOrientationPermission =
  typeof DeviceOrientationEvent !== 'undefined' &&
  typeof DeviceOrientationEvent.requestPermission === 'function';
const needsMotionPermission =
  typeof DeviceMotionEvent !== 'undefined' &&
  typeof DeviceMotionEvent.requestPermission === 'function';

function startSensorsIfNeeded() {
  if (S.sensorsOn) return;
  if (needsOrientationPermission || needsMotionPermission) {
    $('motion-gate').classList.remove('hidden');
  } else {
    attachSensorListeners();
  }
}

async function enableMotion() {
  let denied = false;
  try {
    if (needsOrientationPermission) await DeviceOrientationEvent.requestPermission();
    if (needsMotionPermission) await DeviceMotionEvent.requestPermission();
  } catch {
    denied = true;
  }
  if (denied) {
    // Keep the gate open so the user can retry after fixing the setting.
    toast('Permission denied — enable Settings ▸ Safari ▸ Advanced ▸ Motion & Orientation Access, then tap again', 6000);
    return;
  }
  $('motion-gate').classList.add('hidden');
  attachSensorListeners();
}

let listenersAttached = false;
function attachSensorListeners() {
  if (listenersAttached) return;
  listenersAttached = true;
  S.sensorsOn = true;
  S.firstSensorAt = performance.now();
  window.addEventListener('deviceorientation', onDeviceOrientation);
  window.addEventListener('devicemotion', onDeviceMotion);
  screen.addEventListener?.('orientationchange', () => { S.screenAngle = -1; });
  $('sensor-chip').classList.add('on');
  $('sensor-chip').textContent = 'MOTION: WAIT…';
  // No-data watchdog: HTTPS pages inside in-app browsers (or desktop
  // "mobile" emulation) silently deliver no motion events. Tell the user
  // WHY instead of leaving a forever-"WAIT…" chip.
  setTimeout(() => {
    if (!S.hasOrient && S.sensorsOn) {
      $('sensor-chip').textContent = 'MOTION: NO DATA';
      toast('No sensor data — open this page in Safari (iOS) or Chrome (Android), not inside another app\'s browser', 6000);
    }
  }, 3500);
}

function onDeviceOrientation(e) {
  if (e.alpha == null || e.beta == null || e.gamma == null) return;
  const alphaRaw = e.alpha * deg2rad;
  const beta = e.beta * deg2rad;
  const gamma = e.gamma * deg2rad;
  S.lastAlpha = alphaRaw;
  S.lastBeta = beta;
  S.lastGamma = gamma;
  const orient = (S.screenAngle === -1 ? screenAngle() : S.screenAngle) * deg2rad;

  // Tilt-only fallback: without a gyro the compass-derived alpha drifts and
  // wobbles badly, so once calibration locks the grip we freeze the yaw and
  // keep only the accelerometer-derived tilt angles live.
  const alpha = (S.noGyro && S.alphaFrozen != null) ? S.alphaFrozen : alphaRaw;

  // Device -> world quaternion (Y-up world), the classic fusion formula:
  //   q = qEuler(beta, alpha, -gamma | 'YXZ') * qX(-90°) * qZ(-screenAngle)
  eulerYXZToQuat(S.qTarget, beta, alpha, -gamma);
  const qX90 = [-Math.SQRT1_2, 0, 0, Math.SQRT1_2]; // -90° around X
  qMultiply(S.qTarget, S.qTarget, qX90);
  qMultiply(S.qTarget, S.qTarget, [0, 0, Math.sin(-orient / 2), Math.cos(-orient / 2)]);

  if (!S.hasOrient) {
    S.qSmooth[0] = S.qTarget[0]; S.qSmooth[1] = S.qTarget[1];
    S.qSmooth[2] = S.qTarget[2]; S.qSmooth[3] = S.qTarget[3];
    S.hasOrient = true;
    markSensorAlive();
  }
}

function onDeviceMotion(e) {
  const now = performance.now();
  const dt = Math.min(0.05, Math.max(0.002, (now - S.lastMotionTs) / 1000));
  S.lastMotionTs = now;

  const aWG = e.accelerationIncludingGravity;
  if (!aWG || aWG.x == null) return;

  /* ---- gyroscope presence detection (Stage B tilt-only fallback) ------- */
  const rot = e.rotationRate;
  if (rot && rot.alpha != null) {
    S.gx = (rot.alpha || 0) * deg2rad;
    S.gy = (rot.beta || 0) * deg2rad;
    S.gz = (rot.gamma || 0) * deg2rad;
    if (!S.gyroDetected) {
      if (Math.abs(S.gx) + Math.abs(S.gy) + Math.abs(S.gz) > 1e-6) {
        S.gyroDetected = true; // real gyro: never exactly zero (sensor noise)
      } else {
        S.gyroZeroFrames++;
      }
    }
  } else {
    S.gyroNullFrames++;
  }
  if (!S.gyroDetected && !S.noGyro && now - S.firstSensorAt > 2000) {
    if (S.gyroNullFrames > 5 || S.gyroZeroFrames > 100) {
      enableTiltOnly();
    } else {
      S.gyroDetected = true; // rotation data present but quiet — it has a gyro
    }
  }

  // Gravity direction in device frame.
  const g = [0, 0, 0];
  if (S.noGyro) {
    // Secondary tilt-only gravity: derive it from the tilt angles ALONE
    // (gravity is yaw-invariant, so the noisy/frozen alpha is excluded) and
    // low-pass heavily — the tilt estimate is noisy without a gyro. Same
    // composition as the orientation formula minus the alpha rotation.
    if (S.hasOrient) {
      const orient = (S.screenAngle === -1 ? screenAngle() : S.screenAngle) * deg2rad;
      eulerYXZToQuat(_gq, S.lastBeta, 0, -S.lastGamma);
      qMultiply(_gq, _gq, [-Math.SQRT1_2, 0, 0, Math.SQRT1_2]); // -90° around X
      _qScratch[0] = 0; _qScratch[1] = 0;
      _qScratch[2] = Math.sin(-orient / 2); _qScratch[3] = Math.cos(-orient / 2);
      qMultiply(_gq, _gq, _qScratch);
      rotateByInverse(_gTilt, _gq, 0, -9.81, 0);
      if (!S.gFiltInit) {
        S.gFilt[0] = _gTilt[0]; S.gFilt[1] = _gTilt[1]; S.gFilt[2] = _gTilt[2];
        S.gFiltInit = true;
      } else {
        const k = 1 - Math.exp(-dt / 0.25);
        S.gFilt[0] += (_gTilt[0] - S.gFilt[0]) * k;
        S.gFilt[1] += (_gTilt[1] - S.gFilt[1]) * k;
        S.gFilt[2] += (_gTilt[2] - S.gFilt[2]) * k;
      }
    }
    g[0] = S.gFilt[0]; g[1] = S.gFilt[1]; g[2] = S.gFilt[2];
  } else {
    // g_dev = q⁻¹ · (0,-1,0) · 9.81 from the fused orientation
    rotateByInverse(g, S.qSmooth, 0, -9.81, 0);
  }

  const accel = e.acceleration;
  if (accel && accel.x != null && accel.y != null && accel.z != null) {
    // OS-provided gravity-removed acceleration (iOS) — prefer it.
    S.ax = accel.x; S.ay = accel.y; S.az = accel.z;
  } else {
    S.ax = aWG.x - g[0]; S.ay = aWG.y - g[1]; S.az = aWG.z - g[2];
  }

  if (!S.hasMotion) {
    S.hasMotion = true;
    markSensorAlive();
  }

  /* ---- swing detection ------------------------------------------------ */
  // Leaky-integrate linear acceleration into a velocity estimate; when the
  // acceleration magnitude spikes, open a window and track the peak speed.
  const decay = Math.exp(-dt / SWING_DECAY_TAU);
  S.vx = (S.vx + S.ax * dt) * decay;
  S.vy = (S.vy + S.ay * dt) * decay;
  S.vz = (S.vz + S.az * dt) * decay;
  const vMag = Math.hypot(S.vx, S.vy, S.vz);
  if (vMag > 30) { const k = 30 / vMag; S.vx *= k; S.vy *= k; S.vz *= k; }

  const aMag = Math.hypot(S.ax, S.ay, S.az);
  if (!S.swingActive && aMag > SWING_THRESHOLD()) {
    S.swingActive = true;
    S.swingStart = now;
    S.swingPeak = 0;
  }
  if (S.swingActive) {
    if (vMag > S.swingPeak) S.swingPeak = vMag;
    if (now - S.swingStart > SWING_WINDOW_MS) {
      S.swingActive = false;
      if (S.swingPeak > SWING_MIN_POWER) {
        S.pendingSwingEvent = true;
        S.lastSwingPower = Math.min(S.swingPeak, 25);
        S.lastSwingAt = now;
      }
    }
  }
}

let sensitivity = 1.0;
function SWING_THRESHOLD() {
  return 24 / sensitivity; // m/s^2
}

/** Switch to tilt-only tracking (no gyroscope detected). */
function enableTiltOnly() {
  if (S.noGyro) return;
  S.noGyro = true;
  const chip = $('sensor-chip');
  chip.textContent = 'MOTION: LIVE · TILT';
  toast('Limited sensors — tilt-only tracking', 4000);
  setSubStatus('No gyroscope detected: orientation uses tilt only. Recalibrate after changing your grip.');
  navigator.vibrate?.([40, 60, 40]);
}

function markSensorAlive() {
  const el = $('sensor-chip');
  el.classList.add('on');
  el.textContent = S.noGyro ? 'MOTION: LIVE · TILT' : 'MOTION: LIVE';
}

/* ========================================================================= *
 * Sending
 * ========================================================================= */

function sendTick() {
  if (!S.ws || S.ws.readyState !== 1 || !S.slot) return;
  const now = performance.now();

  // Orientation smoothing (time-constant ORIENT_TAU; heavier without a gyro —
  // the tilt-only fused orientation is noisier).
  if (S.hasOrient) {
    const dt = Math.min(0.05, Math.max(0.001, (now - S.lastSmoothTs) / 1000));
    S.lastSmoothTs = now;
    const tau = S.noGyro ? ORIENT_TAU_TILT : ORIENT_TAU;
    qSlerp(S.qSmooth, S.qSmooth, S.qTarget, 1 - Math.exp(-dt / tau));
  }

  // Calibration capture (300 ms average happens on the PC; flag tells it to start).
  if (S.calSamplingUntil) {
    if (now < S.calSamplingUntil) {
      S.calSamples.push([S.qSmooth[0], S.qSmooth[1], S.qSmooth[2], S.qSmooth[3]]);
      return;
    }
    S.calSamplingUntil = 0;
    S.calSamples.length = 0;
    S.calibFlagUntil = now + 1000;
    safeSendJson({ t: T.EV, name: 'calibrated', data: null });
    setSubStatus('Calibrated ✓ — keep that grip.');
    navigator.vibrate?.(20);
  }

  if (S.ws.bufferedAmount > MAX_BUFFERED) return; // congestion: skip this tick

  let flags = 0;
  if (S.swingActive) flags |= FLAGS.SWING_ACTIVE;
  if (S.pendingSwingEvent) flags |= FLAGS.SWING_EVENT;
  if (now < S.calibFlagUntil) flags |= FLAGS.CALIBRATED;
  if (S.noGyro) flags |= FLAGS.NO_GYRO; // Stage B: PC switches to tilt-only tuning
  S.pendingSwingEvent = false;

  const swing = S.swingActive ? S.swingPeak : (now - S.lastSwingAt < 500 ? S.lastSwingPower : 0);

  const frame = encodeMotion({
    flags,
    battery: S.battery,
    time: epochNow() + S.offsetMs,
    qx: S.qSmooth[0], qy: S.qSmooth[1], qz: S.qSmooth[2], qw: S.qSmooth[3],
    ax: S.ax, ay: S.ay, az: S.az,
    gx: S.gx, gy: S.gy, gz: S.gz,
    swing,
  });

  try {
    S.ws.send(frame);
  } catch { /* ignore */ }

  // Touchpad: 30 Hz while active, or immediately after a change.
  if (S.touch.active || S.touch.dirty) {
    if (S.touch.active || now - (S.lastTouchSend || 0) > 33) {
      S.lastTouchSend = now;
      S.touch.dirty = false;
      try { S.ws.send(encodeTouch(S.touch)); } catch { /* ignore */ }
    }
  }
}

/* ========================================================================= *
 * Calibration
 * ========================================================================= */

function calibrate() {
  if (!S.hasOrient) {
    toast('No sensor data yet — see the motion chip');
    return;
  }
  if (S.noGyro) {
    // Tilt-only fallback: lock the (drifting) compass yaw at the calibration
    // pose — the PC averages its baseline over the next 300 ms of samples,
    // so the orientation must hold still during the capture.
    S.alphaFrozen = S.lastAlpha;
  }
  S.calSamples.length = 0;
  S.calSamplingUntil = performance.now() + 300;
  setSubStatus('Hold still…');
  const btn = $('calibrate-btn');
  btn.disabled = true;
  setTimeout(() => { btn.disabled = false; }, 400);
}

/* ========================================================================= *
 * Touchpad
 * ========================================================================= */

function bindTouchpad() {
  const pad = $('pad');
  const knob = $('pad-knob');
  let radius = 1;

  const setKnob = (dx, dy) => {
    knob.style.transform = `translate(${dx * radius * 0.7}px, ${-dy * radius * 0.7}px)`;
  };

  pad.addEventListener('pointerdown', (e) => {
    pad.setPointerCapture(e.pointerId);
    const r = pad.getBoundingClientRect();
    radius = Math.min(r.width, r.height) * 0.38;
    S.touch.active = true;
    S.touch.dirty = true;
    pad.classList.add('active');
    move(e, r);
  });
  pad.addEventListener('pointermove', (e) => {
    if (!S.touch.active) return;
    move(e, pad.getBoundingClientRect());
  });
  const release = () => {
    if (!S.touch.active) return;
    S.touch.active = false;
    S.touch.dx = 0; S.touch.dy = 0;
    S.touch.dirty = true;
    pad.classList.remove('active');
    knob.style.transform = '';
  };
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
  pad.addEventListener('lostpointercapture', release);

  function move(e, r) {
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    let dx = (e.clientX - cx) / radius;
    let dy = -(e.clientY - cy) / radius; // +y = up
    const m = Math.hypot(dx, dy);
    if (m > 1) { dx /= m; dy /= m; }
    S.touch.dx = Math.round(dx * 1000) / 1000;
    S.touch.dy = Math.round(dy * 1000) / 1000;
    S.touch.dirty = true;
    setKnob(S.touch.dx, S.touch.dy);
  }
}

/* ========================================================================= *
 * Battery / wake lock / fullscreen
 * ========================================================================= */

async function initBattery() {
  if (!navigator.getBattery) return;
  try {
    const b = await navigator.getBattery();
    const update = () => {
      S.battery = Math.round(b.level * 100);
      S.charging = b.charging;
      $('stat-battery').textContent = `${S.battery}%${S.charging ? '⚡' : ''}`;
    };
    update();
    b.addEventListener('levelchange', update);
    b.addEventListener('chargingchange', update);
  } catch { /* ignore */ }
}

async function requestWakeLock() {
  if (!navigator.wakeLock) return;
  try {
    S.wakeLock = await navigator.wakeLock.request('screen');
    S.wakeLock.addEventListener?.('release', () => { S.wakeLock = null; });
  } catch { /* ignore */ }
}

async function goFullscreen() {
  try {
    await document.documentElement.requestFullscreen({ navigationUI: 'hide' });
    try { await screen.orientation.lock('portrait'); } catch { /* iOS has no lock — the protocol handles rotation anyway */ }
    toast('Fullscreen on');
  } catch {
    toast('Fullscreen blocked — add to Home Screen for best results');
  }
}

/* ========================================================================= *
 * UI watch dog (2 Hz): latency chip + sensor watchdog
 * ========================================================================= */

function startUiWatchdog() {
  setInterval(() => {
    $('stat-latency').textContent = S.rttMs ? `${Math.round(S.rttMs)} ms` : '-- ms';
    if (S.sensorsOn && !S.hasOrient && performance.now() - S.firstSensorAt > 2500 && !isSecureContext) {
      $('https-warn').classList.remove('hidden');
    }
  }, 500);
}

/* ========================================================================= *
 * Boot
 * ========================================================================= */

function join() {
  const code = ($('room-input').value || '').trim().toUpperCase();
  if (code.length < 3) {
    $('connect-error').textContent = 'Enter the 4-letter room code.';
    $('connect-error').classList.remove('hidden');
    return;
  }
  S.room = code;
  S.wantedName = $('name-input').value.trim().slice(0, 16);
  history.replaceState(null, '', `?room=${code}`);
  show('play');
  setMainStatus('Connecting…');
  connect();
}

function exit() {
  stopReconnect();
  clearInterval(S.sendTimer); S.sendTimer = null;
  clearInterval(S.syncTimer); S.syncTimer = null;
  try { S.wakeLock?.release(); } catch { /* ignore */ }
  try { S.ws?.close(1000, 'exit'); } catch { /* ignore */ }
  S.ws = null;
  S.slot = 0;
  show('connect');
}

function boot() {
  $('join-btn').addEventListener('click', join);
  $('room-input').addEventListener('keydown', (e) => { if (e.key === 'Enter') join(); });
  $('room-input').addEventListener('input', () => {
    $('room-input').value = $('room-input').value.toUpperCase().replace(/[^A-Z]/g, '');
  });
  if (S.room) $('room-input').value = S.room;
  $('name-input').value = S.wantedName;

  $('calibrate-btn').addEventListener('click', calibrate);
  $('fullscreen-btn').addEventListener('click', goFullscreen);
  $('exit-btn').addEventListener('click', exit);
  $('enable-motion-btn').addEventListener('click', enableMotion);
  $('https-ok-btn').addEventListener('click', () => $('https-warn').classList.add('hidden'));

  bindTouchpad();
  initBattery();
  startUiWatchdog();

  document.addEventListener('visibilitychange', () => {
    if (!document.hidden && S.slot) requestWakeLock();
  });

  // Sensitivity slider is stored per-device; expose via query param ?sens=1.4
  const sens = parseFloat(params.get('sens'));
  if (Number.isFinite(sens)) sensitivity = Math.min(2, Math.max(0.5, sens));

  if (S.room) {
    show('play');
    setMainStatus('Connecting…');
    connect();
  } else {
    show('connect');
  }
}

boot();
