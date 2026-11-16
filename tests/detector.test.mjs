/**
 * Node round-trip: synthesize the same 640×480 frame the browser E2E draws
 * (two ARUCO_4X4_1000 markers on a mid-grey background) and run the vendored
 * detector on it directly — isolates "bad synthetic image" vs "worker path".
 */
import { AR } from '../client/core/tracker/vendor/aruco.js';
import '../client/core/tracker/vendor/aruco_4x4_1000.js';

const W = 640, H = 480;
const data = new Uint8ClampedArray(W * H * 4);
for (let i = 0; i < W * H; i++) {
  data[i * 4] = 128; data[i * 4 + 1] = 128; data[i * 4 + 2] = 128; data[i * 4 + 3] = 255;
}

function drawMarker(code, x, y, side) {
  const u = side / 6;
  const px = (xx, yy, c) => {
    if (xx < 0 || yy < 0 || xx >= W || yy >= H) return;
    const i = (yy * W + xx) * 4;
    data[i] = c; data[i + 1] = c; data[i + 2] = c; data[i + 3] = 255;
  };
  for (let yy = y - u; yy < y + side + u; yy++)
    for (let xx = x - u; xx < x + side + u; xx++) px(xx, yy, 255); // quiet zone
  for (let yy = y; yy < y + side; yy++)
    for (let xx = x; xx < x + side; xx++) px(xx, yy, 0);            // black square
  for (let r = 0; r < 4; r++)
    for (let c = 0; c < 4; c++)
      if (code[r * 4 + c] === '1')
        for (let yy = y + (r + 1) * u; yy < y + (r + 2) * u; yy++)
          for (let xx = x + (c + 1) * u; xx < x + (c + 2) * u; xx++) px(xx, yy, 255);
}

drawMarker('1011010100110010', 200, 180, 60); // id 0
drawMarker('0000111110011010', 420, 240, 48); // id 1

const detector = new AR.Detector({ dictionaryName: 'ARUCO_4X4_1000' });
const markers = detector.detect({ width: W, height: H, data });
console.log('found', markers.length);
for (const m of markers) {
  const cx = m.corners.reduce((s, c) => s + c.x, 0) / 4;
  const cy = m.corners.reduce((s, c) => s + c.y, 0) / 4;
  console.log(`id=${m.id} center=(${cx.toFixed(1)},${cy.toFixed(1)}) ham=${m.hammingDistance}`);
}
