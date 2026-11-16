/**
 * Fusion KF unit tests (Stage C) — `npm run test:fusion`.
 *
 * Runs in plain Node (the filter is DOM/THREE-free by design). Timestamps
 * use real epoch-ms magnitudes (~1.7e12) to mirror production conditions.
 */

import { HandFusionFilter } from '../client/core/tracker/fusionKF.js';

let pass = 0, fail = 0;
function ok(cond, name, detail = '') {
  if (cond) { pass++; console.log('  PASS ' + name); }
  else { fail++; console.log('  FAIL ' + name + (detail ? ' — ' + detail : '')); }
}
const near = (a, b, eps) => Math.abs(a - b) <= eps;

const T0 = 1794900000000; // epoch ms

/* 1. Convergence: silent IMU, static target. */
{
  const f = new HandFusionFilter();
  const tx = 10, ty = 5, tz = 20, sx = 0.01, sy = 0.01, sz = 0.05;
  for (let i = 0; i <= 60; i++) {          // 2 s of 30 Hz camera
    const t = T0 + i * 33.33;
    f.predict(t, 0, 0, 0);
    f.fuse(t, tx + sx * ((i % 3) - 1), ty, tz, sx, sy, sz);
  }
  const d = f.debug();
  ok(near(d.p[0], tx, 3 * sx) && near(d.p[1], ty, 3 * sy) && near(d.p[2], tz, 3 * sz),
    'static target converged', JSON.stringify(d.p));
  ok(Math.abs(d.v[0]) + Math.abs(d.v[1]) + Math.abs(d.v[2]) < 0.03,
    'velocity settled to ~0', JSON.stringify(d.v));
  ok(d.nMeas === 61 && d.nRejected === 0, 'all measurements accepted', `${d.nMeas}/${d.nRejected}`);
}

/* 2. Velocity tracking: silent IMU, target moving at 2 m/s. */
{
  const f = new HandFusionFilter();
  for (let i = 0; i <= 45; i++) {          // 1.5 s
    const t = T0 + i * 33.33;
    f.predict(t, 0, 0, 0);
    f.fuse(t, 2 * (i * 33.33) / 1000, 0, 0, 0.008, 0.008, 0.05);
  }
  const d = f.debug();
  ok(near(d.v[0], 2, 0.08), 'velocity tracks 2 m/s', `v=${d.v[0].toFixed(4)}`);
  ok(near(d.p[0], 3, 0.05), 'position tracks the mover', `p=${d.p[0].toFixed(4)}`);
}

/* 3. Delayed measurement + replay: exact determinism vs inline fusion. */
{
  const u = 2;                             // m/s² constant accel
  const truth = (dt) => 0.5 * u * dt * dt; // x(t) from rest
  // Run A: measurement fused inline exactly at t = 500 ms.
  const a = new HandFusionFilter();
  for (let i = 1; i <= 100; i++) {
    const t = T0 + i * 10;
    a.predict(t, u, 0, 0);
    if (i === 50) a.fuse(t, truth(0.5), 0, 0, 0.005, 0.005, 0.05);
  }
  // Run B: identical IMU stream, but the measurement arrives at t = 1000 ms
  // dated 500 ms (real webcam pipeline latency) — handled by replay.
  const b = new HandFusionFilter();
  for (let i = 1; i <= 100; i++) {
    const t = T0 + i * 10;
    b.predict(t, u, 0, 0);
    if (i === 100) b.fuse(t - 500, truth(0.5), 0, 0, 0.005, 0.005, 0.05);
  }
  const da = a.debug(), db = b.debug();
  ok(near(da.p[0], truth(1.0), 0.02) && near(db.p[0], truth(1.0), 0.02),
    'both runs end near ground truth', `inline=${da.p[0].toFixed(4)} replay=${db.p[0].toFixed(4)} truth=${truth(1.0)}`);
  ok(near(da.p[0], db.p[0], 1e-6) && near(da.v[0], db.v[0], 1e-6),
    'replay is bit-exact vs inline fusion', `dp=${Math.abs(da.p[0] - db.p[0]).toExponential(2)}`);
}

/* 4. Two back-to-back late measurements: the second must build on the
 *    first correction (write-back), not on the stale timeline. */
{
  const f = new HandFusionFilter();
  for (let i = 1; i <= 100; i++) f.predict(T0 + i * 10, 0, 0, 0); // silent IMU, drift-free
  // Two corrections for the same static position, both dated in the past.
  const r1 = f.fuse(T0 + 300, 1, 0, 0, 0.005, 0.005, 0.05);
  const r2 = f.fuse(T0 + 400, 1, 0, 0, 0.005, 0.005, 0.05);
  const d = f.debug();
  ok(r1 && r2, 'both late measurements accepted');
  ok(near(d.p[0], 1, 0.01) && near(d.p[1], 0, 0.01) && near(d.p[2], 0, 0.05),
    'state carries both corrections', JSON.stringify(d.p));
}

/* 5. Outlier gate: a wildly wrong measurement is rejected whole. */
{
  const f = new HandFusionFilter();
  for (let i = 0; i <= 60; i++) {
    const t = T0 + i * 33.33;
    f.predict(t, 0, 0, 0);
    f.fuse(t, 1, 1, 1, 0.01, 0.01, 0.05);
  }
  const before = f.debug().p.slice();
  const accepted = f.fuse(T0 + 2400, 5, 1, 1, 0.01, 0.01, 0.05); // 4 m off
  const d = f.debug();
  ok(!accepted && d.nRejected === 1, 'outlier rejected', `accepted=${accepted}`);
  ok(near(d.p[0], before[0], 1e-9), 'state unchanged by the outlier',
    `dp=${Math.abs(d.p[0] - before[0]).toExponential(2)}`);
}

/* 6. Measurement predating the buffer: clamped, no crash, applied. */
{
  const f = new HandFusionFilter();
  for (let i = 1; i <= 120; i++) f.predict(T0 + i * 10, 0, 0, 0); // 1.2 s @100 Hz > 96-slot ring
  const applied = f.fuse(T0, 0.5, 0, 0, 0.01, 0.01, 0.05);
  const d = f.debug();
  ok(applied && d.nClamped === 1, 'stale measurement clamped to buffer start', `applied=${applied}`);
  ok(near(d.p[0], 0.5, 0.05), 'clamped measurement still corrects', `p=${d.p[0].toFixed(3)}`);
}

/* 7. dt clamp: a 500 ms stream gap integrates at most 50 ms of motion. */
{
  const f = new HandFusionFilter();
  f.predict(T0, 2, 0, 0);
  f.predict(T0 + 500, 2, 0, 0);            // gap → dt clamped to 0.05
  f.predict(T0 + 510, 0, 0, 0);
  const d = f.debug();
  ok(d.v[0] < 0.11, 'gap clamp bounds integration', `v=${d.v[0].toFixed(4)}`);
}

/* 8. Camera measurement before any IMU: seeds the filter. */
{
  const f = new HandFusionFilter();
  const applied = f.fuse(T0, 0.2, 0.1, 1.5, 0.01, 0.01, 0.05);
  const d = f.debug();
  ok(applied && d.started && near(d.p[2], 1.5, 1e-9), 'first measurement seeds', JSON.stringify(d.p));
  f.predict(T0 + 16, 0, 0, 0);
  ok(f.debug().started, 'predict after seed stays consistent');
}

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
