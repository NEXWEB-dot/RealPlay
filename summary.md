# REALPLAY — Build Summary

Per-phase log of everything that was built, exactly as requested. Latest phase at the bottom.

---

## Phase 1 — Project setup, lobby, phone controller, live debug view ✅

**Goal:** runnable foundation — Vite + Node ws server, lobby with QR/room code, phone controller page with sensor streaming + calibration, and a debug view proving a phone's orientation drives a 3D racket in real time, in split screen.

### What was built

**1. Project & tooling**
- `package.json` — scripts: `dev` (server+Vite via concurrently), `dev:https` (self-signed HTTPS for phone sensors), `build`, `start` (prod: Node serves `dist/` on :8080), `smoke` (protocol self-test). Deps: `three@0.180`, `ws`, `qrcode`; dev: `vite@5`, `@vitejs/plugin-basic-ssl`, `concurrently`, `cross-env`.
- `vite.config.js` — MPA build (PC app **and** controller page), `/ws` proxied to the Node server so PC + phones share one origin, `host: true` so phones on the LAN can reach the dev server, HTTPS toggle via `RP_HTTPS=1`.

**2. Server (`/server`)**
- `index.js` — Node http + `ws` server (`perMessageDeflate: false`, maxPayload capped): room lifecycle (host creates 4-letter rooms from an unambiguous alphabet), controller join with stable player slots (2 in Phase 1), binary relay phone→PC with the slot byte inserted, JSON relays both ways (`STATUS`, `HAPTIC`, `EV`), clock-sync `SYNC/SYNC_ACK`, transport RTT measurement via ws ping/pong (reported in the lobby), heartbeat with stuck-connection termination, room teardown + notifications on host disconnect, static file serving of `dist/` for production mode, LAN IP discovery for the QR link.
- `rooms.js` — room registry, slot allocation, lobby payload builder.
- `smoke.js` — 21-assertion end-to-end protocol test (`npm run smoke`): room create, join/slots, room_full rejection, binary MOTION/TOUCH relay with slot insertion + payload integrity, sync round-trip, status/haptic/EV relays, disconnect frees the slot, host disconnect closes the room. **All green.**

**3. Shared protocol (`/shared/protocol.js`)**
- Single source of truth for server, PC and phone. JSON control messages + two binary frames:
  - **MOTION (56 B)**: flags, battery, f64 server-synced timestamp, quaternion (device→world), linear acceleration (gravity removed), rotation rate, swing peak — relaid at 57 B with slot inserted.
  - **TOUCH (12 B)**: active flag + normalized nudge vector — relayed at 13 B.

**4. Phone controller (`/controller`)** — framework-free dark UI, served at `/controller/index.html?room=CODE`
- iOS permission gate ("Tap to enable motion"), Wake Lock, fullscreen + orientation lock (graceful on iOS), auto-reconnect, exit.
- Sensors: `deviceorientation` fused into a quaternion with the classic `alpha/beta/gamma` formula incl. screen-orientation compensation (correct in portrait and landscape); `devicemotion` for gravity-removed linear acceleration (uses OS-provided `acceleration` when available, otherwise subtracts gravity derived from the orientation) and gyro.
- Orientation smoothing (time-constant 35 ms), send loop capped at 100 Hz with congestion backoff (`bufferedAmount` guard).
- **Swing detection**: acceleration spike opens a 160 ms window; leaky-integrated velocity (τ = 0.25 s) peaks give swing power; event flag + power travel in the MOTION frame.
- **Calibration**: 300 ms capture + averaged quaternion; PC side re-averages its own 300 ms window for a robust baseline; recalibrate anytime; `CALIBRATED` flag + EV message (both paths wired).
- Clock sync every 2 s (NTP-lite) → frames stamped in server time; RTT shown on the phone; battery (charging indicator) via Battery API.
- Thumb **touchpad** (pointer-captured knob) streaming normalized nudge vectors at ~30 Hz.
- Haptics receiver (`tap`/`smash`/`buzz`), status text from the PC, HTTPS/motion-permission troubleshooting hints built into the page.

**5. PC core (`/client/core`, `/client/net`)**
- `loop.js` — fixed-timestep accumulator (240 Hz physics hook) + render frame with interpolation alpha, clamped catch-up, and a watchdog that keeps rendering when a webview starves `requestAnimationFrame` while visible (found via headless testing).
- `renderer.js` — WebGL2 renderer (ACES tone mapping, PCF soft shadows), quality presets as pixel-ratio caps, **dynamic resolution** (0.5–1.0× scale on rolling frame-time averages with hysteresis).
- `splitScreen.js` — 1/2/4 viewport manager over one scene with `setViewport/setScissor`, per-viewport aspect handling, gap clearing (already supports the future 2×2 doubles layout).
- `input.js` — the input contract: gameplay never sees raw phone data, only `RacketInput { orientation, angularVelocity, linearAcceleration, swingEvent, touch, assistTarget, battery, latency, fresh }`. Sources: `ControllerInputSource` (phone + calibration baseline), `DemoInputSource` (attract mode for empty slots), `BotInputSource` (neutral core now, brain in Phase 5).
- `net/sync.js` — pooled (allocation-free) interpolation buffers: slerp orientation + lerped vectors at `now − 45 ms`, reorder protection, staleness limits.
- `net/client.js` — PC WS client: host registration, room/lobby events, binary routing into per-slot buffers, status/haptic senders.
- `settings.js` — persisted settings (quality, dynamic res, shadows, post-FX, volumes, sensitivity, swing threshold, handedness, invert axes, assist) + presets; `audio.js` — WebAudio bus (master/sfx/music) with UI blips and a synthesized power-scaled swing whoosh.

**6. Sport layer (`/client/sports`)**
- `_template/Sport.js` — the interface contract every future sport implements (`createCourt/createBall/createRacket/physicsStep/update/getCameras/getHudConfig/getRules/createBot/setShadows`) with the architecture rules written down (padel/badminton will plug in without touching the core).
- `tableTennis/court.js` — full PBR sports hall: glossy clearcoat blue table with real dimensions (2.74 × 1.525 m, 0.76 m top, 0.1525 m net + overhang), white lines, metal legs, canvas-generated net grid + tape, plank floor, walls, glowing banners/ceiling panels, hemisphere + key/fill directional lights with soft shadows.
- `tableTennis/racket.js` — procedural racket (wood core, red/black rubber faces, edge tape, wooden handle with glowing player-color ring) in the **exact device-axis convention** (+Z face normal, +Y handle-up) so the calibrated phone quaternion maps 1:1.
- `tableTennis/trail.js` — pooled 28-point additive swing trail, speed-faded.
- `tableTennis/TableTennisSport.js` — Phase-1 gameplay: racket rotation = calibrated relative phone orientation (plus handedness/invert mirrors), assisted glide positioning with touchpad nudge (lag tuned so timing still matters), swing events → lunge + world-space swing arrow + whoosh + popup, per-player cameras behind their own table end, RoomEnvironment PMREM reflections, 240 Hz hook kept warm for Phase 2.

**7. PC UI (`/client/ui`)**
- Main menu (2 Players active; vs Bot/Practice labeled for later phases), lobby (big room code, QR canvas, LAN-IP picker + copyable controller link, live slot cards with battery/RTT, per-slot "Test buzz"), settings modal (all live-applied), pause overlay, HUD (per-half chips: name, RTT, battery, packets/s, swing power; popups, toasts), F3 **debug overlay** (FPS, frame ms, draw calls/triangles/pixel-ratio/dyn-scale, per-phone RTT + packets/s + battery + live gyro/accel/quat graphs).

### Verification performed
- `npm run smoke` — 21/21 protocol assertions pass.
- `vite build` — clean MPA production build (main + controller pages).
- Production server (`npm start`): `/`, `/controller/`, `/controller/index.html`, `/healthz` all serve; QR link correctly uses the LAN IP (`http://192.168.x.x:8080/controller/index.html?room=CODE`).
- Full in-browser E2E: created a room, entered the debug view, streamed a simulated phone (MOTION frames with quaternion wobble + swing flags + EV calibrated + TOUCH) through the real server — verified live: interpolation buffer pushing (~64 frames/s), calibration baseline captured, **racket quaternion animating from the phone stream**, HUD chip showing `Fake Phone · 0 ms · 🔋76% · 64/s · swing 9.4 m/s`, `SWING 9.4 m/s` popup, F3 overlay toggling, renderer drawing 40 calls / 4.2 k tris per frame — **zero console errors**.
- Bugs found & fixed during E2E: isMain CLI detection (path vs URL), missing `renderer` in sport ctx, `ArrowHelper` material handling, HUD visible over menus, missing `CALIBRATED`-flag edge path, `RacketInput` field-name mismatch in the interpolator, rAF starvation watchdog.

### How to run (quick start)
```bash
npm install
npm run dev:https     # HTTPS is required for phone sensors
# open http://localhost:5173 → 2 Players → scan the QR with your phone → CALIBRATE
```

### Next up — Phase 2
Table, net, ball physics: custom fixed-240 Hz ball simulation (gravity, quadratic drag, Magnus spin, spin-transfer table bounce, net cord, swept-sphere CCD) + ball machine test mode feeding the debug view.

---

## Phase 2 — Ball physics + ball machine practice mode ✅

**Goal:** real ball flight — a custom deterministic physics step at the fixed 240 Hz rate (gravity, quadratic drag, Magnus lift, spin-transfer bounces, net + net cord, floor, swept-sphere CCD) plus a **ball-machine practice mode** to feed the rally, with spin-visible ball visuals, impact particles and trajectory prediction.

### What was built

**1. Physics core (`client/sports/tableTennis/physics.js`)**
- Real SI units: gravity 9.81, ball r = 20 mm (⌀ 40 mm), m = 2.7 g, air ρ = 1.184, table 2.74 × 1.525 m with 0.76 m top, net tape 0.1525 m above the top (+ cord band).
- `stepBall(state, dt, events)` — the fixed-step integrator: gravity, **quadratic drag** (cD 0.4, drag-crisis region for a smooth 40 mm sphere), **Magnus lift** (spin parameter S = rω/|v| → cL = S/(0.5 + S), capped at 0.45 — physically consistent; spin magnitude decays slowly in flight), and spin-decay.
- **Impulse bounces with tangential friction that converts spin** (`surfaceBounce`): table e = 0.9 / μ = 0.3 (ITTF-consistent), table edge e = 0.7 / μ = 0.4 (full 3-D side normal), floor e = 0.55 / μ = 0.6, plus rolling friction while the ball skids/rolls to rest (rest timer 3 s → deactivate). A bouncing ball gains spin from surface friction and its flight curves afterwards — topspin kicks forward, backspin checks.
- **Net + net cord**: crossing the net plane while below the tape → dead net (kills cross-net speed, e = 0.22) or tape clip within the cord band (luckier e = 0.45 bounce + lateral/vertical scatter derived from a **deterministic state hash** — no `Math.random` inside the step, so replays stay identical).
- **Swept-sphere CCD**: continuous sphere-vs-surface collision between the previous and new position each 1/240 s substep — no tunneling at any realistic speed.
- **Event gating**: bounce events below 0.5 m/s normal impact and net events below 0.25 m/s are suppressed so a resting ball's micro-bounces can't spam particles/audio.
- `predictBall` — the **same integrator** for bots/assist/debug: pooled reusable output (points, bounce flags, exact `stopZ` plane-crossing interpolation with velocity, end state); `magnusAccel(vel, spin)` export for the debug overlay. Pool helpers: `createBallState`/`copyBallState`/`deactivateBall`.

**2. Ball visuals (`ball.js`)**
- Shared procedural canvas texture (seam arcs + faint meridian + RP logo) so spin is readable by eye; per-ball mesh. `integrateSpinVisual` premultiplies an axis-angle quaternion from the physics ω (capped at π per frame so fast spin still reads correctly).
- **Blob shadow**: radial-gradient plane riding the nearest support (table top when inside the table footprint, floor otherwise); diameter grows and opacity fades with ball height.
- Pooled 28-point trail ribbon shared with the rackets. `syncFromState` runs per render frame — zero allocation.

**3. Impact particles (`effects.js`)**
- Pooled `ImpactParticles`: one additive `THREE.Points` (320 particles max), positions/velocities/colors in flat `Float32Array`s, cursor ring reuse (dead particles parked at y = −100 with zeroed color), cone burst around the surface normal, drag + gravity integration and fade-to-black — zero per-frame allocation even with several balls bouncing per second.

**4. Ball machine (`machine.js`)**
- Procedural **robot-style model**: base plate + wheels, column, head assembly (hopper, accent stripe, barrel, muzzle ring, two counter-rotating drive wheels that spin up/down with the running state), head aims at each target with a recoil kick that eases back.
- **Launch solver**: pick a landing target on player 1's half (x ∈ [−0.45, 0.45], z ∈ [−1.1, −0.3]) → aim the head (`setFromUnitVectors(−Z, dir)`) → derive the muzzle point → iterate the flight time T outward until the ballistic arc clears the net tape at the z = 0 plane → `v = (dx/T, (dy + ½gT²)/T, dz/T)`. The net-clearance margin is **spin-type aware** (topspin dives → 0.07 margin, backspin floats → 0.015, else 0.035), resolved *before* aiming.
- Spin axis for travel direction û: topspin ω = ŷ × û = (uz, 0, −ux), backspin inverse, sidespin ±ŷ; `mix` resolves per ball. Config: running, interval (jittered ±15%), speed 2–16 m/s, spinRpm 0–3000, spinType, randomize. `tossOne()` for manual feeds. Randomizing targets is **spawning policy** — the physics itself stays deterministic.
- Head pivot at **y 1.08 / z 1.62** (close behind the table end, like real robots): the original position (z 1.95, y 1.0) geometrically capped launch speed at ~6 m/s so the speed slider saturated; the new position supports ~13 m/s and lands the slider honestly.

**5. Sport layer (`TableTennisSport.js`)**
- Phase 1 racket logic kept verbatim (orientation mapping, mirrors, assisted glide, swing lunge/arrow/whoosh, per-player cameras).
- **8-ball pool** stepped in `physicsStep`; machine wiring with `_spawnLaunch` (steals the oldest active ball when the pool is exhausted) and `_freeBall`.
- Bounce/net events → pooled particles (orange for table, grey-blue for floor, white for net), `audio.bounce(impact)` / `audio.thud()`, HUD popups (`NET` / `NET CORD!` on the offending side).
- **Prediction line**: cyan pooled `Line` fed by `predictBall` for the first active ball travelling toward P1 (driven by the F3 overlay flag).
- **Modes**: `'debug2p'` (split screen, machine off by default) and `'practice'` (single camera, machine feeding). API: `setMode`, `setMachineEnabled`, `machineToss`, `resetBalls`, `activeBallCount`, `getBallDebugInfo` (id, speed km/h, spin rpm + dominant axis, Magnus m/s², bounce count) for the debug overlay.
- Stats: launches, total bounces, net hits, peak speed.

**6. Audio (`core/audio.js`)**
- Shared 0.25 s white-noise buffer (`_noise()`) reused by swing/bounce/thud.
- `bounce(strength)` — bandpassed plastic "tick" (1.4–2.3 kHz by impact) + pitch-scaled sine thump, volume ∝ impact speed. `thud()` — lowpass noise + 120→55 Hz sine for dead nets/cord. `swing()` refactored onto the shared buffer.

**7. App wiring (`main.js`, `ui/screens.js`, `ui/debugOverlay.js`, `ui/ui.css`)**
- **Practice state**: single viewport + camera 1, machine ON, resetBalls, panel shown + synced; frame loop updates the panel status at ~5 Hz (`n in play · m bounces · peak x.x m/s`).
- **Keys**: `Space` toss, `B` machine toggle (+ toast + panel STOP/START label sync), `F3` toggles overlay AND sets `sport.showPrediction`, `ESC` pause with a **context-aware quit label** ("Quit to menu" in practice vs "Quit to lobby" in debug). Blur/focus auto-pause covers practice.
- **Quit routing**: practice pause quit → menu; any quit stops the machine, clears balls, hides the machine panel.
- **Machine panel UI**: speed/spin/rate sliders, spin-type select, randomize checkbox, STOP/START toggle, Toss button, live status line + `.machine-panel` styles.
- **F3 overlay**: new ball-telemetry line (`BALL n · x km/h · spin y rpm (axis) · magnus z m/s² · bounces`) or "no ball in play — SPACE tosses one"; graph rows shifted to make room.

### Verification performed
- `npm run build` clean; `npm run smoke` — 21/21 protocol assertions pass (both after the physics cleanup and again after the HUD fix).
- **Practice E2E** (prod server on :8080 + in-app browser): 3–4 launches in ~5 s, 9–12 bounces, **zero net hits**, speeds honest (slider 9 → 9.0 m/s ≈ 32.4 km/h peak), live panel status ("3 in play · 9 bounces · peak 9.0 m/s"); Space toss increments launches; B toggles the machine and the panel button label syncs (STOP↔START) with a toast; F3 opens the overlay, sets `showPrediction`, and the cyan line draws with ~96 points; rolling-ball **spin/velocity coupling** verified (vz = −1.88 m/s → 916 rpm ≈ ω = vz/r).
- **2P debug E2E with a fake phone** streaming hand-packed 56-byte MOTION frames through the real server: machine off by default, `B` turns it on (26 balls launched), racket visibly driven by the stream, HUD chip lifecycle verified end-to-end.
- **Pause/quit routing E2E**: debug pause label "Quit to lobby", practice "Quit to menu"; `B` correctly ignored while paused; quit-from-practice lands on the menu with machine stopped, panel hidden, HUD hidden and balls cleared.
- **Zero console errors** accumulated across the entire session (practice + 2P + pause/quit + reconnect).

### Bugs found & fixed during E2E
- **HUD chip flapping** — the server's 2 s lobby heartbeat re-ran `refreshSlots`, which reset live chips ("14 ms · 🔋76% · 63/s") to "waiting for phone…" every 2 s. Fixed: `'wait'` is set only on the join transition; the frame loop owns `'live'`/`'stale'` afterwards. Also distinguishes "joined but never streamed" (`'wait'`) from "packets stopped" (`'stale'` → CONNECTION STUTTER) via a new `everReceived` flag on `ControllerInputSource`. Re-verified across two heartbeat cycles.
- **Machine speed cap** — original head position capped launches at ~6 m/s (slider saturated); repositioned (see §4) + spin-type-aware net-clearance margin so balls stop hitting the net at speed.
- **predictBall intercept** — rewrote the messy clone/dead-code version into an exact crossing interpolation (pre-step position captured, fraction f along the substep); restored `out.endPos` accidentally swallowed by the edit.
- **Event spam gating** — added `BOUNCE_EVENT_MIN` (0.5 m/s) and net-event gates (0.25 m/s).
- Physics observations, **not bugs**: balls legitimately skid/roll off the table after bounces (topspin kick, e = 0.9 is ITTF-correct) and rest up to 3 s before despawn; at very high sidespin (~900+ rpm) balls drift up to ~2.6 m sideways off the table — physically consistent (cL cap 0.45), lower the spin in the panel if undesired.

### Next up — Phase 3
Racket-ball hit model: swing velocity + racket face normal → outgoing ball velocity and spin (sweet spot offset, power curve), assisted positioning toward the predicted interception point — first real rallies against the machine.

---

## Phase 3 — Racket-ball hit model + assisted rallies

### 1. Table & net CCD fix (`client/sports/tableTennis/physics.js`)
- **Critical bug**: `sweepSphereBox` tested the sphere against an ORIGIN-CENTERED box — `TABLE_BOX` (cy 0.7375) and `NET_BOX` (cy 0.83625) never collided, so balls fell straight through the table and the "bounces" counted were floor bounces. Fixed by offsetting the sphere start into each box's local frame (`p = P − box.c`) in BOTH the slab loop and the inside-pop-out branch; rewritten with scalar locals (zero per-call allocation).
- Consequence discovered immediately: with CCD working, the machine's no-drag `solveFlight` arcs die in the net tape (drag pulls the real flight short/low). Replaced the parabolic solver with a **drag-aware shooting solver**: each candidate T seeds `v = (dx/T, (dy+½gT²)/T, dz/T)` and is verified by simulating the REAL integrator (`simFlight` reuses `stepBall` on module scratch — drag, Magnus, spin, CCD); iterate T on net-hit (raise arc) / landing error (short → more carry, long → flatter), keep the best candidate, bounded 12 iterations. New signature `solveFlight(from, to, speed, margin, spin)` — callers: machine (passes its planned spin so Magnus is accounted) and the assist arc (passes the post-damping spin).
- `machine.solveLaunch` reordered: spin computed BEFORE the flight solve.

### 2. Hit model (`TableTennisSport.js`)
- Swept ball-vs-blade contact at the 240 Hz step: blade transforms interpolated between the render frame's start/end transforms (`_phPos/_phQ/_phVel` per slot, fraction `(step+1)/batch`); per-face (±R along the blade normal) plane crossing of the ball segment + blade-local disc radius test (`BLADE_R` 0.08); one contact per substep.
- Impulse response = `surfaceBounce` against the MOVING blade (extended with surface velocity params; relative approach + tangential slide), rubber e = 0.78, mu = 0.55; sweet-spot power falloff `1 − 0.35·offNorm²`; hit cooldown 0.09 s; separation along the normal.
- **Swing boost**: the phone's SWING_EVENT rising edge sets `boostVel = swingDir_world · 0.55 · power` (unit dir × swing speed, power clamped 1..16 → a 12 m/s swing is ≈6.6 m/s blade at contact) decaying over `BOOST_T` 0.16 s; the boost adds to the effective blade velocity in `physicsStep` and suppresses the assisted arc (`boostFactor`), so a live swing overrides the assist.
- FX: particles, `audio.bounce(strength·1.05)`, racket kick-back, HUD popups **HIT / SMASH / SWEET / EDGE** (SMASH keys off `boosted && impact > 5.5` — the outgoing-speed test was dead code under assist since arcs cap ≈38 km/h), `stats.hits`, `lastHit { slot, kmh, offNorm, boosted, impact, spinRpm, t }`.
- **Removed** the per-swing `SWING x m/s` popup — it buried the hit payoff under fast swing streams (the swing already has arrow flash + audio + HUD chip readout).

### 3. Assisted rallies
- **Targeting** = take the ball at the top of the bounce: `_scanIntercept` predicts with the real integrator (`predictBall`), detects the table bounce inside the prediction (vy flip > 0.5 with y ∈ [0.74, 0.95] — rising pre-bounce arcs pass the net band ABOVE this window), then picks the post-bounce point with the LEAST |vy| inside the reach band (REACH_Y [0.84, 1.42], REACH_X 0.75, own-half depth ≥ 0.45) — the "hang" point, most forgiving timing. `predictBall` returns a SHARED singleton: scan + copy immediately.
- Positioning: while a hittable ball is incoming the assist target owns the blade position OUTRIGHT (diluting toward home puts the blade below the ball's flight path — measured 0.25 m vertical miss); exp smoothing τ 0.12 s. Assist off/low/medium/high = 0/0.45/0.75/0.95 from Settings.
- **Return arc is legal by construction**: `_applyAssistArc` no longer blends velocities (a 40 % raw component dives into the tape — measured 12 net hits/run). `solveFlight` REPLACES the velocity; the hit's character survives through speed (`4.2 + (raw−4.2)·(0.35+0.5w)`, clamp 4.2..10.5 m/s) and lateral aim, scaled by the assist level; spin damped to the value the solve accounted for.
- Machine aims deep (tz ∈ [−1.15, −0.6]) so the post-bounce rise lands inside blade reach.

### 4. Debug/E2E APIs
- `debugAssistScan(slot, ballIdx)` — predicts one ball and annotates every point `{ t, x, y, z, vy, post, deep, band, pick }` (pick = the selected hang point). Kept as a legit debug API (it found the CCD bug and tuned the scan).
- `lastHit` + `stats { launches, bounces, netHits, hits, maxSpeed }` power the E2E assertions; the practice panel now shows `hits`.

### Verification performed
- `npm run build` clean after every change; `npm run smoke` — ALL TESTS PASSED (protocol 21/21).
- **Table bounce proof**: `debugAssistScan` on a live machine ball — trajectory now bounces at y = 0.78 (table top + radius) instead of falling through to the floor; traced a full flight: net cleared at y 1.44, landed z −0.58 (inside the target range), rebound apex 1.36 m, bounces/rolls/despawns — physically honest end to end.
- **Practice E2E** (final build): 32 hits, **zero net hits**, 259 table bounces, peak 13.8 m/s, boosted smash logged at impact 11.1 (49.5 km/h), demo rackets sustain the rally with assist medium.
- **2P debug E2E with a fake phone** (hand-packed 56-byte MOTION frames through the real server, room code from `.room-code`): 1112 frames streamed → slot 1, live HUD chip ("P1 E2E Phone · 9 ms · 🔋76% · 63/s · swing 12.0 m/s"), **212 hits by the streamed racket**, `lastHit.slot === 1`, SWEET/SMASH popups observed live ("SMASH 46 km/h"), perfect 1,2,1,2 rally alternation vs the demo slot, netHits 0.
- **Boost path verified**: swing pulses (3-on/5-off flag pattern = real rising edges) → boosted hits with impact 7.4-7.7; required two real fixes (below).
- **Assist settings plumbing**: off → `_assistValue` 0 + `assistValid` false (blade parks home); high → 0.95; restored medium. Pause/B-guard/resume + quit-to-lobby re-verified with the phone streaming. **Zero console errors** across the whole session.

### Bugs found & fixed during Phase 3 E2E
- **`sweepSphereBox` origin-center bug** (the reported pass-through) — see §1; the fix surfaced the latent drag-arc problem that the broken CCD had been hiding.
- **Swing boost magnitude** — `dir` is a UNIT vector, so `0.55·(power/9)` produced 0.73 m/s blade speed (boost effectively dead); now `0.55·power` (clamped) ≈ 6.6 m/s at a 12 m/s swing.
- **Zero-direction swings** — a swing event with zero accel left `dir = (0,0,0)`: boostT set but boostVel zero (and no lunge). The input source now falls back to the device's forward (+Z screen normal).
- **SMASH dead code** — outgoing-speed test (> 40 km/h) unreachable under assist; keys off impact now.
- **Swing popup spam** overwriting hit popups (see §2).
- Measurement note: `stats.launches`/`hits` are cumulative per mode entry — early E2E windows right after a reload under-count while the first feeds/pool churn settle; steady-state machine rate verified at 0.67-0.75 launches/s for interval 1.4.

### Next up — Phase 4
Phone-driven singles match: real scoring/serve rotation on the phone-controlled game, ball in/out + edge-ball detection, match UI (score, serve indicator, set/match end), rematch flow.

---

## Phase 4 — Phone-driven singles match ✅

**Goal:** a real match on the phone-driven game — legal-by-construction serves, ITTF serve rotation (every 2 points, every point at deuce), rally legality (bounce order, volley rule, lets), real scoring with win-by-2, and the full match UI (scoreboard, serve dot, banners, end screen, rematch with server swap).

### 1. Generic match state (`client/core/match.js`)
- Sport-agnostic `MatchState { rules {pointsToWin, winBy}, firstServer, points[2], winner }` with `pointFor(slot)` → `{ over, winner, deuce, gamePointFor }`, `serveSlot()`, `reset()`.
- Rotation policy in one place: `serveSlot = floor(total/every) % 2 ? 3−firstServer : firstServer`, `every = 1` at deuce (both ≥ pointsToWin−1) else 2 — the ITTF rule. Doubles sports subclass or wrap it.

### 2. Serve system (`TableTennisSport.js`, `physics.js`)
- `traceServe(from, vx, vy, vz, spin)` — simulates a serve with the REAL integrator and reports `{netHit, floor, nBounces, z1, z2, x2}`.
- `solveServe(from, dirZ, power, spinW)` — shooting search over own-side bounce depth (0.44–0.83 m) and speed factors (±14 %), topspin ω = ŷ × û; legality-scored (net hit +10, floor +6, first bounce wrong half +6, wrong bounce count +4, exit beyond +0.45 beyond the table +3); best-effort candidate even when nothing scores 0 → **a serve is always launched, and it is legal by construction**.
- `_startServe` — parks a fresh ball at the server's home BEFORE any blade pin (`HOME + 0.03y + dirZ·SERVE_HOLD`, SERVE_HOLD 0.1 toward the net), marks the phase `'serve-wait'` and emits the event. `debugForceServe()` skips the wait (E2E).
- **Serve trigger** in the per-slot `update()` loop: the SERVER's `input.swingEvent` during `'serve-wait'` calls `_executeServe(power)` — a serve is a hit with a solved-legal arc on top (swing FX + blade boost still apply; power shapes pace + spin). 6 s timeout auto-serves at power 9.
- `_executeServe` pins the held ball to the blade face, solves, launches, hands the rally to the server; the serve-wait triggers only once per point.

### 3. Rally adjudication (`TableTennisSport.js`)
- `_rally = { toucher, serveFlight, ownBounced, oppBounced, letServe, done }` tracked per point; ball 0 is the single live match ball (`_matchBallIdx` via `_freeBall` after `resetBalls`).
- **Bounce side rule**: `ownSign = −dirZ`, side = sign of the bounce z (z<0 → slot 1 half). `debugAssistScan`/`_scanIntercept` fixed the same way (`p.z * sideSign >= 0.45`) so a serve's first (server-half) bounce doesn't park the receiver at the net.
- **Legality** on each bounce: own-side double-bounce fault, serve must clear the opponent half (no bounce → no legal return → point when it goes out), opponent must bounce on their half before hitting.
- **Volley rule**: pre-hit `predictBall` prediction — if the predicted first table bounce inside the reach window belongs to the *striker's own* half, a volley hit is a fault (`VOLLEY`), unless the ball was already heading out (`VOLLEY OUT` = striker wins).
- **Rally handover**: at the end of `_hitRespond`, the opponent's legal strike makes them the toucher and clears the serve/bounce flags for this flight.
- **Let**: serve flight with `netHit && oppBounced` → `LET` event, point replayed (no score change).
- `_onMatchBallGone` — the single adjudication path for a ball that despawns without decisive events: derives the outcome from the last known position/velocity (past table plane, on whose half, moving away).
- `_matchStep` phases: `'serve-wait'` (6 s timeout → auto-serve), `'rally'` (+ **watchdog**: match ball missing/inactive ≥ 0.05 s → `_onMatchBallGone`, closing the stranded-rally bug class), `'point'` (1 s ban → `_startServe`), `'over'`.

### 4. Debug/E2E APIs
`debugMatchInfo()`, `debugMatchRules({pointsToWin, winBy})` (short games), `debugForceServe()`, `debugAwardPoint(slot)` (fast-forwards through the real MatchState so rotation/deuce/end logic is exercised, skipping the point ban). `window.__RP` now exposes `{ net, slots, sport, state, settings, hud }`.

### 5. Match UI
- **Scoreboard** (`hud.js`, `ui.css`): `#score` top-centre — names, points, serve dot (`sc-serve.on` glows), game-point side glow (`.gp`); `showScore/setScoreNames/updateScore/banner`.
- **Banners**: `MATCH START`, `DEUCE`, `GAME POINT — <name>`, `<NAME> WINS a–b` (kind `win`); `LET — serve clipped the net, replaying…` toast; `POINT — OUT/NO RETURN/OWN SIDE/SERVE FAULT/VOLLEY/VOLLEY OUT` popups on the scoring side.
- **End screen** (`screens.js`): `Rematch` (server swaps) + `Back to lobby`; menu 2P badge now "MATCH", lobby action is **Start match**, menu footer "Phase 4 build".
- `main.js`: state `'match'` (replaces `'debug2p'`), `onStartMatch` (layout 2, `setMode('match')`, machine hidden, `startMatch(1)`), `onRematch`, quit hooks call `sport.exitMatch()` + `hud.showScore(false)`; `slotName()` shows the phone's name when connected; end screen appears 1.8 s after the `end` event (guarded by still being in the match). Space/B are practice-only; pause quit label "Quit to lobby" (match) / "Quit to menu" (practice).
- `machine.setVisible(v)` — the machine only exists in practice; hidden (and stopped) in match.

### Verification performed
- `npm run build` clean (109 modules); `npm run smoke` — ALL TESTS PASSED (21/21).
- **Demo-vs-demo match, real rules (11, win by 2)**, prod build on :8080, in-app browser, assist off so demo rigs miss and points flow: 28 points with **ZERO serve-rotation errors** (verified against the formula, pre- and post-deuce); deuce reached at 10-10 and serves alternate every point through 17-15; `end screen "Player 1 wins"` with the correct score; **rematch swaps firstServer** (start event srv2); ESC pause opens, physics halts (simTime frozen), resumes; quit-to-lobby mid-match → state lobby, match null, scoreboard hidden, balls cleared.
- **Fake-phone E2E** (in-page WebSocket → `ctrl_hello {room, name:'TestPhone'}`, 56-byte LE MOTION at 16 ms, 3-on/5-off SWING_EVENT pulses with swing 9 + accel (2,0,12) on edges): joins a slot, `serve` events fired **from phone swings**, points adjudicate; scoreboard shows the phone's name; end screen "TestPhone wins 14 — 12"; quit mid-match clean with the phone connected.
- **Stranded-rally bug found & fixed** (one natural repro + deterministic closure): the match ball deactivated ~20 ms after a serve launched from `update()` in the same frame `_startServe` ran (held ball never pinned → `solveServe` read the dead ball's stale position), and there was no net for ball-gone-without-events. Fixed: (1) `_startServe` parks the held ball before the first blade pin; (2) rally watchdog in `_matchStep`. Post-fix: 19 points with maxRallyBalllessMs = 0.
- **Practice regression**: machine visible + auto-running, Space toss works, B toggles, quit-to-menu works. Zero console errors across the whole session.

### Next up — Stage B
Sensor-only "Hand Motion Positioning": gravity-removed accel rotated by the calibrated orientation, leaky double integration with spring-damper return, ZUPT stillness snap, dead zone + reach clamp, per-player Positioning/Sensitivity/Return/Dead-zone settings, no-gyro tilt fallback, F3 hand-motion debug, estimator in its own module (interface ready for camera fusion).
