/**
 * REALPLAY game server.
 *
 * Responsibilities (Phase 1):
 *   - Hosts rooms created by the PC. The PC is the "host" websocket.
 *   - Phones join a room by 4-letter code and are assigned a player slot.
 *   - Relays binary sensor frames (MOTION / TOUCH) phone -> PC with the
 *     slot inserted, perMessageDeflate disabled and Nagle disabled for
 *     minimum latency.
 *   - Measures socket RTT (ws ping/pong) and reports it to the PC lobby.
 *   - Relays small JSON messages: STATUS / HAPTIC (PC -> phone) and
 *     EV events (phone -> PC).
 *   - Optionally serves the built client (dist/) so the same origin can be
 *     used in "production" mode:  npm run build && npm start
 */

import http from 'node:http';
import https from 'node:https';
import os from 'node:os';
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { WebSocketServer } from 'ws';
import * as P from '../shared/protocol.js';
import { Rooms } from './rooms.js';

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
  '.wasm': 'application/wasm',
  '.ktx2': 'image/ktx2',
  '.glb': 'model/gltf-binary',
  '.mp3': 'audio/mpeg',
  '.wav': 'audio/wav',
  '.ogg': 'audio/ogg',
  '.woff2': 'font/woff2',
  '.txt': 'text/plain; charset=utf-8',
  '.map': 'application/json',
};

function log(...args) {
  console.log(`[${new Date().toISOString().slice(11, 19)}]`, ...args);
}

/** IPv4 addresses of this machine on the LAN, best-guess order (192.168 / 10. / 172.). */
export function lanIps() {
  const out = [];
  for (const list of Object.values(os.networkInterfaces())) {
    for (const ni of list || []) {
      if (ni.family === 'IPv4' && !ni.internal) out.push(ni.address);
    }
  }
  const score = (ip) => {
    if (ip.startsWith('192.168.')) return 3;
    if (ip.startsWith('10.')) return 2;
    if (/^172\.(1[6-9]|2\d|3[01])\./.test(ip)) return 1;
    return 0;
  };
  return out.sort((a, b) => score(b) - score(a));
}

/* ------------------------------------------------------------------ */
/* Static file serving (used when --static dist is passed).            */
/* ------------------------------------------------------------------ */

function sendJson(res, code, obj) {
  const body = JSON.stringify(obj);
  res.writeHead(code, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
  res.end(body);
}

function makeStaticHandler(rootDir) {
  const root = path.resolve(rootDir);
  return function handle(req, res) {
    if (req.method !== 'GET' && req.method !== 'HEAD') {
      return sendJson(res, 405, { error: 'method not allowed' });
    }
    if (req.url === '/healthz') return sendJson(res, 200, { ok: true, rooms: undefined });

    let pathname;
    try {
      pathname = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
    } catch {
      return sendJson(res, 400, { error: 'bad url' });
    }
    if (pathname.endsWith('/')) pathname += 'index.html';

    const safe = path.normalize(path.join(root, '.' + pathname));
    if (!safe.startsWith(root)) return sendJson(res, 403, { error: 'forbidden' });

    fs.stat(safe, (err, st) => {
      if (err || !st.isFile()) return sendJson(res, 404, { error: 'not found', path: pathname });
      const ext = path.extname(safe).toLowerCase();
      res.writeHead(200, {
        'Content-Type': MIME[ext] || 'application/octet-stream',
        'Content-Length': st.size,
        'Cache-Control': ext === '.html' ? 'no-cache' : 'public, max-age=3600',
      });
      if (req.method === 'HEAD') return res.end();
      fs.createReadStream(safe).pipe(res);
    });
  };
}

/* ------------------------------------------------------------------ */
/* Server                                                              */
/* ------------------------------------------------------------------ */

export function startServer({ port = 8081, staticDir = null } = {}) {
  const rooms = new Rooms();
  /** ws -> session */
  const sessions = new Map();

  const httpServer = http.createServer(
    staticDir ? makeStaticHandler(staticDir) : (req, res) => {
      if (req.url === '/healthz') return sendJson(res, 200, { ok: true });
      sendJson(res, 404, { error: 'not found (dev: use the Vite server on :5173)' });
    },
  );

  const wss = new WebSocketServer({
    server: httpServer,
    path: P.WS_PATH,
    perMessageDeflate: false, // latency first
    maxPayload: 8 * 1024,
  });

  wss.on('connection', (ws) => {
    const session = {
      ws,
      role: null, // 'host' | 'controller'
      room: null,
      slot: 0,
      name: '',
      device: '',
      battery: P.BATTERY_UNKNOWN,
      rtt: 0,
      pendingPings: 0,
      lastPingAt: 0,
    };
    sessions.set(ws, session);
    ws.on('pong', () => {
      session.pendingPings = 0;
      if (session.lastPingAt) session.rtt = Math.max(0, Date.now() - session.lastPingAt);
    });
    ws.on('message', (data, isBinary) => {
      if (isBinary) onBinary(session, data);
      else onJson(session, data);
    });
    ws.on('close', () => onClose(session));
    ws.on('error', () => { /* handled by close */ });
  });

  function onJson(session, data) {
    let msg;
    try {
      msg = JSON.parse(Buffer.from(data).toString('utf8'));
    } catch {
      return;
    }
    if (!msg || typeof msg.t !== 'string') return;

    switch (msg.t) {
      case P.T.HOST_HELLO: {
        if (session.role === 'host') return;
        session.role = 'host';
        session.room = rooms.create();
        session.room.host = session;
        log(`host connected, room ${session.room.code}`);
        rooms.safeSend(session.ws, { t: P.T.ROOM, code: session.room.code, ips: lanIps() });
        break;
      }

      case P.T.CTRL_HELLO: {
        if (session.role === 'controller') return;
        const room = rooms.get(msg.room);
        if (!room) {
          return rooms.safeSend(session.ws, { t: P.T.ERROR, code: 'unknown_room', message: `Room "${msg.room}" not found.` });
        }
        const slot = rooms.freeSlot(room);
        if (slot == null) {
          return rooms.safeSend(session.ws, { t: P.T.ERROR, code: 'room_full', message: 'This room already has 2 players.' });
        }
        session.role = 'controller';
        session.slot = slot;
        session.room = room;
        session.name = String(msg.name || `Phone ${slot}`).slice(0, 24);
        session.device = String(msg.device || '').slice(0, 48);
        room.controllers.set(slot, session);
        log(`controller joined room ${room.code} as Player ${slot} (${session.name})`);
        rooms.safeSend(session.ws, { t: P.T.WELCOME, slot, code: room.code, serverTime: Date.now() });
        rooms.sendLobby(room);
        break;
      }

      case P.T.SYNC: {
        rooms.safeSend(session.ws, { t: P.T.SYNC_ACK, t0: msg.t0 ?? 0, t1: Date.now() });
        break;
      }

      case P.T.STATUS: {
        // host -> one phone
        if (session.role !== 'host' || !session.room) return;
        const c = session.room.controllers.get(msg.slot | 0);
        rooms.safeSend(c?.ws, { t: P.T.STATUS, text: String(msg.text || '').slice(0, 120), kind: msg.kind || 'info' });
        break;
      }

      case P.T.HAPTIC: {
        // host -> one phone
        if (session.role !== 'host' || !session.room) return;
        const c = session.room.controllers.get(msg.slot | 0);
        rooms.safeSend(c?.ws, { t: P.T.HAPTIC, pattern: String(msg.pattern || 'tap').slice(0, 12) });
        break;
      }

      case P.T.EV: {
        // phone -> host
        if (session.role !== 'controller' || !session.room) return;
        rooms.safeSend(session.room.host?.ws, {
          t: P.T.EV,
          slot: session.slot,
          name: String(msg.name || '').slice(0, 24),
          data: msg.data ?? null,
        });
        break;
      }

      default:
        break;
    }
  }

  function onBinary(session, data) {
    if (session.role !== 'controller' || !session.room) return;
    const buf = Buffer.from(data);
    if (buf.length < 2) return;
    const type = buf[0];

    if (type === P.MOTION && buf.length >= P.MOTION_SIZE) {
      session.battery = buf.readUInt16LE(2); // battery sits at body offset +1
      relayToHost(session, buf);
    } else if (type === P.TOUCH && buf.length >= P.TOUCH_SIZE) {
      relayToHost(session, buf);
    }
  }

  /** Insert the slot byte at [1] and forward to the room's host. */
  function relayToHost(session, buf) {
    const host = session.room.host;
    if (!host || host.ws.readyState !== 1) return;
    const out = Buffer.allocUnsafe(buf.length + 1);
    out[0] = buf[0];
    out[1] = session.slot;
    buf.copy(out, 2, 1);
    try { host.ws.send(out); } catch { /* ignore */ }
  }

  function onClose(session) {
    sessions.delete(session.ws);
    if (session.role === 'controller' && session.room) {
      const room = session.room;
      room.controllers.delete(session.slot);
      log(`controller left room ${room.code} (slot ${session.slot})`);
      rooms.safeSend(room.host?.ws, { t: P.T.CTRL_GONE, slot: session.slot, reason: 'disconnect' });
      rooms.sendLobby(room);
    } else if (session.role === 'host' && session.room) {
      log(`host disconnected, closing room ${session.room.code}`);
      rooms.destroy(session.room);
    }
  }

  /* Heartbeat + latency measurement + lobby refresh (every 2 s). */
  const heartbeat = setInterval(() => {
    const now = Date.now();
    for (const session of sessions.values()) {
      if (session.pendingPings > 2) {
        try { session.ws.terminate(); } catch { /* ignore */ }
        continue;
      }
      session.pendingPings++;
      session.lastPingAt = now;
      try { session.ws.ping(); } catch { /* ignore */ }
    }
    for (const room of rooms.map.values()) {
      if (room.host) rooms.sendLobby(room);
    }
  }, 2000);

  const ready = new Promise((resolve) => httpServer.once('listening', resolve));
  httpServer.listen(port, () => {
    log(`REALPLAY server listening on port ${port}${staticDir ? ` (serving ${staticDir})` : ' (API/WS only — dev mode uses Vite)'}`);
  });

  return { httpServer, wss, rooms, heartbeat, ready };
}

/* ------------------------------------------------------------------ */
/* CLI                                                                 */
/* ------------------------------------------------------------------ */

function parseArgs(argv) {
  const opts = { port: 8081, staticDir: null };
  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === '--port') opts.port = parseInt(argv[++i], 10) || opts.port;
    else if (argv[i] === '--static') opts.staticDir = argv[++i] || null;
  }
  return opts;
}

const invokedUrl = process.argv[1] ? pathToFileURL(path.resolve(process.argv[1])).href : '';
if (import.meta.url === invokedUrl) {
  const opts = parseArgs(process.argv.slice(2));
  if (opts.staticDir) {
    // Support http:// scheme upgrade behind tunnels is out of scope here.
    opts.staticDir = path.resolve(opts.staticDir);
    if (!fs.existsSync(opts.staticDir)) {
      console.error(`Static dir not found: ${opts.staticDir} — run "npm run build" first.`);
      process.exit(1);
    }
  }
  startServer(opts);
}
