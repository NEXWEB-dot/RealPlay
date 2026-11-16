/**
 * Marker detection worker (Stage C).
 *
 * Module worker: receives an ImageBitmap per video frame, draws it to a
 * reused OffscreenCanvas, runs the vendored js-aruco2 detector (ARUCO_4X4_1000
 * dictionary, marker IDs 0/1) and posts back a compact per-marker summary:
 *
 *   { t, w, h, markers: [{ id, cx, cy, side, ham }] }
 *
 * cx/cy = marker center in pixels, side = mean edge length in pixels
 * (the OUTER black square), ham = dictionary hamming distance. All pose
 * math happens on the main thread (cameraTracker) — the worker stays dumb.
 */

import { AR } from './vendor/aruco.js';
import './vendor/aruco_4x4_1000.js'; // registers ARUCO_4X4_1000 in AR.DICTIONARIES

const DICTIONARY = 'ARUCO_4X4_1000';

let detector = null;
let canvas = null;
let ctx = null;

self.onmessage = (ev) => {
  const { bmp, t, w, h } = ev.data;
  if (!bmp) return;

  try {
    if (!detector) detector = new AR.Detector({ dictionaryName: DICTIONARY });
    if (!canvas) {
      canvas = new OffscreenCanvas(w, h);
      ctx = canvas.getContext('2d', { willReadFrequently: true });
    }
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }

    ctx.drawImage(bmp, 0, 0, w, h);
  } finally {
    if (bmp.close) bmp.close(); // release the transferred bitmap's memory
  }

  let found;
  try {
    found = detector.detect(ctx.getImageData(0, 0, w, h));
  } catch {
    found = []; // never let a bad frame kill the worker
  }

  const out = [];
  for (let i = 0; i < found.length; i++) {
    const m = found[i];
    let cx = 0, cy = 0, side = 0;
    for (let k = 0; k < 4; k++) {
      cx += m.corners[k].x;
      cy += m.corners[k].y;
      const a = m.corners[k], b = m.corners[(k + 1) & 3];
      const dx = b.x - a.x, dy = b.y - a.y;
      side += Math.sqrt(dx * dx + dy * dy);
    }
    out.push({ id: m.id, cx: cx / 4, cy: cy / 4, side: side / 4, ham: m.hammingDistance });
  }

  self.postMessage({ t, w, h, markers: out });
};
