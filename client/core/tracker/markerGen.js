/**
 * ArUco marker generation (Stage C) — the DRAW side of the tracker.
 *
 * The detector (detectWorker) and this generator share the same vendored
 * dictionary (ARUCO_4X4_1000, 4×4 bits), so what we print/rasterize is
 * bit-identical to what the detector reads back.
 *
 * Geometry convention (same as js-aruco2's Dictionary.generateSVG):
 *   the marker is markSize(=6) units of BLACK square (1-unit black border +
 *   4×4 data bits), surrounded by a 1-unit WHITE quiet zone on each side,
 *   i.e. 8×8 units total. The DETECTED quad is the OUTER BLACK SQUARE —
 *   that is the "marker side" every consumer talks about:
 *
 *     black side   = markSize / (markSize + 4)   of the full tile  (6/8)
 *     quiet zone   = 1 unit on all four sides
 *
 *   All sizing below therefore treats `side` as the black-square side.
 */

import { AR } from './vendor/aruco.js';
import './vendor/aruco_4x4_1000.js';

const dict = new AR.Dictionary('ARUCO_4X4_1000');
export const MARK_SIZE = dict.markSize;        // 6 — black square, in units
export const UNITS_TOTAL = dict.markSize + 2;  // 8 — incl. 1-unit quiet zone

/**
 * SVG string for marker `id` with the BLACK square side = sideMm
 * (print-ready; unit suffix included). IDs 0 and 1 are the game markers
 * (P1/P2); the dictionary holds 1000.
 */
export function markerSVG(id, sideMm) {
  const total = (sideMm * UNITS_TOTAL) / MARK_SIZE;
  return dict.generateSVG(id).replace(
    '<svg ',
    `<svg width="${total.toFixed(2)}mm" height="${total.toFixed(2)}mm" `,
  );
}

/**
 * Rasterize marker `id` onto a 2D canvas context. (x, y) = TOP-LEFT of the
 * BLACK square; its side = sidePx. The caller must ensure enough canvas
 * margin for the quiet zone (1 unit = sidePx/6). Returns the full tile rect
 * (including the quiet zone) in pixels.
 *
 * For clean detection pass an integer sidePx divisible by MARK_SIZE (no
 * anti-aliased unit edges) — e.g. 48, 60, 96.
 */
export function drawMarker(ctx, id, x, y, sidePx) {
  const u = sidePx / MARK_SIZE;
  const code = dict.codeList[id];
  if (code == null) throw new Error(`marker id ${id} is not in the dictionary`);

  // quiet zone (white) around the black square
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(x - u, y - u, sidePx + 2 * u, sidePx + 2 * u);

  // black square (border + data region)
  ctx.fillStyle = '#000000';
  ctx.fillRect(x, y, sidePx, sidePx);

  // data bits: '1' cells are WHITE, drawn inside the 1-unit black border
  ctx.fillStyle = '#ffffff';
  const size = MARK_SIZE - 2; // 4×4 data bits
  for (let r = 0; r < size; r++) {
    for (let c = 0; c < size; c++) {
      if (code[r * size + c] === '1') {
        ctx.fillRect(x + (c + 1) * u, y + (r + 1) * u, u, u);
      }
    }
  }
  return { x: x - u, y: y - u, w: sidePx + 2 * u, h: sidePx + 2 * u };
}
