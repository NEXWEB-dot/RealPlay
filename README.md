# REALPLAY — your phone is the racket

3D table tennis in the browser (Three.js + Vite) with **phones as motion controllers** over a Node.js WebSocket server. Modular core so padel/badminton and bots slot in later.

**Current status: Phase 4 complete** — Phase 1 (setup, lobby, phone controller with sensor streaming + calibration, split-screen debug view), Phase 2 (real ball physics at a fixed 240 Hz step: gravity, quadratic drag, Magnus lift, spin-transfer bounces, net + net cord, floor, swept-sphere CCD; ball-machine **practice mode**), Phase 3 (**racket-ball hit model**: swept racket contact at 240 Hz with rubber restitution/friction, sweet-spot falloff, phone-swing smash boost, HIT/SMASH/SWEET/EDGE popups, plus **assisted rallies** — Settings ▸ Assist: off/low/medium/high) and Phase 4 (**singles match**: swing to serve, ITTF serve rotation — every 2 points, every point at deuce — bounce-order + volley legality, lets, real scoring with win-by-2, live scoreboard with serve dot and game-point glow, banners, end screen, rematch with server swap).

---

## Requirements

- Node.js ≥ 18.17 (tested on v24)
- Chrome/Edge on the PC, Chrome (Android) or Safari (iOS) on the phone
- PC and phone(s) on the **same Wi-Fi / LAN**

## Install & run

```bash
npm install

# Development (recommended): Vite dev server + game server with hot reload
npm run dev

# Development with HTTPS (needed for phone sensors — see below)
npm run dev:https

# Production build, single origin on :8080
npm run build
npm start

# Protocol self-test (no browser needed)
npm run smoke
```

Dev mode: PC app at `http://localhost:5173`, game server on `:8081` (WS proxied through Vite at `/ws`).
Production mode: everything on `http://<pc-ip>:8080`.

## Testing with your phone

1. Start the server (`npm run dev` or `npm run dev:https`).
2. Open the PC app, click **2 Players**. A lobby appears with the 4-letter room code and a QR code. Pick the PC's LAN IP in the dropdown if the QR link is wrong (the link under the QR is always authoritative).
3. Scan the QR with the phone (or open the link). The phone shows the controller page: status, latency, battery, touchpad, **CALIBRATE**.
4. Tap **CALIBRATE** while holding the phone like a racket, screen facing the "opponent". The PC half for that player shows **CALIBRATED ✓** and the racket follows your hand in real time.
5. Use **FULLSCREEN** on the phone (locks portrait on Android; Wake Lock keeps the screen on).
6. Player 2 joins the same way (second phone or the same page in another tab).
7. In the PC debug view: left half = P1, right half = P2. Unconnected halves show a scripted **DEMO** racket. Press **Test buzz** in the lobby to verify haptics, **F3** on the PC for the debug overlay (FPS, RTT per phone, packets/s, live gyro/accel/quat graphs — plus a ball-telemetry line whenever a ball is in play), **ESC** to pause.

## Match mode (2 Players)

From the main menu click **2 Players**, fill the lobby with phone(s), then **Start match**. Split screen, both halves live.

- **Serve**: whoever serves holds the ball at their racket — swing the phone to serve. Ignore it for 6 s and the game auto-serves. Points are adjudicated with real table tennis legality: bounce order, own-side double bounce, volleys (VOLLEY / VOLLEY OUT), lets when a serve clips the net, and OUT/NO RETURN/SERVE FAULT.
- **Rotation**: serve switches every 2 points, and every point once deuce (10-10) is reached. Rematch swaps the first server.
- **Score**: first to 11, win by 2. The scoreboard shows a glowing serve dot on the serving side and glows GAME POINT; banners announce DEUCE / GAME POINT / winner; the end screen offers Rematch or Back to lobby.
- `ESC` pauses (physics halts); the pause quit returns to the lobby. `F3` shows the debug overlay plus match info.

## Practice mode (ball machine)

From the main menu click **Practice / Rally**: single camera behind player 1, ball machine feeding your end.

| Key | Action |
|---|---|
| `Space` | toss one ball (also works while the machine is off) |
| `B` | toggle the machine on/off |
| `ESC` | pause — the quit button returns to the menu |
| `F3` | debug overlay + cyan trajectory-prediction line |

The machine panel (bottom-left) adjusts launch **speed** (2–16 m/s), **spin** magnitude (0–3000 rpm), feed **rate** (0.4–3 s), **spin type** (mix / topspin / backspin / side L / side R / none), **randomize** targets, a STOP/START toggle and **Toss**. The status line shows balls in play, total bounces and peak ball speed. It also works inside the 2-player debug view: `Space` / `B` there, machine off by default.

### ⚠️ Sensors need HTTPS on phones

Both Android Chrome and iOS Safari expose motion/orientation sensors **only in secure contexts**. Over plain `http://<lan-ip>` you will see "MOTION: WAIT…" and an HTTPS hint on the phone.

Easiest options:

- **`npm run dev:https`** — Vite serves a self-signed cert. Open the QR link on the phone, accept the certificate warning, proceed. Sensors then work on Android and iOS (iOS also needs Settings → Safari → Advanced → *Motion & Orientation Access*, which the controller page tells you).
- **Cloudflare Tunnel** (Phase 7 workflow, works today): `cloudflared tunnel --url http://localhost:5173` and use the generated `https://…trycloudflare.com` URL in the lobby's link field.

## Controls mapping (Phase 1 semantics)

| Phone | PC |
|---|---|
| Rotation (calibrated quaternion) | racket rotation, 1:1 |
| Swing (acceleration spike + leaky-integrated velocity peak) | swing event → power (m/s), whoosh + "SWING x.x m/s" popup; hit model lands in Phase 3 |
| Thumb touchpad | nudges racket position inside the reach zone |
| Calibrate button | re-baselines the racket (any time) |

Data path: phone streams a 56-byte binary MOTION frame at up to 100 Hz (quaternion, linear acceleration, gyro, swing power, battery, clock-synced timestamp) → server relays with slot byte → PC interpolates in a ~45 ms buffer and samples at render time.

## Settings

Quality preset (Low/Medium/High), dynamic resolution (auto pixel-ratio scaling), shadows, post-FX (active Phase 6), volumes, swing sensitivity/threshold, handedness per player, invert axes, **assist strength** (off/low/medium/high — how strongly the game glides your racket to the interception point and shapes your returns into legal rally shots; a live swing always overrides the assist). Persisted in localStorage.

## Project structure

See `summary.md` for the phase-by-phase log, `client/sports/_template/Sport.js` for the sport interface contract (padel/badminton later), and `server/smoke.js` for the protocol test.

## Troubleshooting

- **Phone can't open the page** — the Vite dev server binds all interfaces (`host: true`); allow Node through Windows Firewall on first run. In production mode use port 8080.
- **Room code not found** — the phone must be on the same network; check that the QR link's IP matches the PC's LAN IP (use the dropdown + Copy link).
- **"MOTION: WAIT…"** — you're on plain HTTP (see HTTPS section) or denied the permission prompt.
- **No sensor data after HTTPS works** — iOS: enable *Motion & Orientation Access*; the page detects this and shows a hint.
- **Racket mirrored / inverted feel** — Settings → per-player handedness (left/right) and Invert axes.
