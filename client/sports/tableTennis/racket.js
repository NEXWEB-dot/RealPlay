/**
 * Procedural racket model.
 *
 * Local frame convention (MUST match the phone mapping):
 *   +Z = racket face normal (points at the opponent when held correctly)
 *   +Y = from handle toward the top edge of the blade
 *   +X = blade width
 * A phone held "like a racket" maps device axes exactly like this
 * (screen normal +Z, phone top +Y), which is why the calibrated relative
 * quaternion can be applied 1:1 to this group.
 */

import * as THREE from 'three';

export const RACKET = {
  bladeRadius: 0.075,   // ~15 cm blade diameter -> 7.5 cm radius
  bladeThickness: 0.012,
  handleLength: 0.10,
};

/**
 * @param {{ accent: number }} opts accent = player color (handle ring, trail)
 * @returns {THREE.Group}
 */
export function buildRacket({ accent = 0xff8a1e } = {}) {
  const g = new THREE.Group();
  g.name = 'racket';

  const R = RACKET.bladeRadius;
  const T = RACKET.bladeThickness;

  const bladeMat = new THREE.MeshStandardMaterial({
    color: 0x8a6b45, // wood core edge
    roughness: 0.6,
  });
  const blade = new THREE.Mesh(
    new THREE.CylinderGeometry(R, R, T, 48),
    bladeMat,
  );
  blade.rotation.x = Math.PI / 2; // cylinder axis Y -> Z (face normal ±Z)
  blade.castShadow = true;
  g.add(blade);

  // Rubber faces: red on the +Z side, black on the -Z side (official kit).
  const rubberGeo = new THREE.CylinderGeometry(R - 0.0012, R - 0.0012, 0.0032, 48);
  const redMat = new THREE.MeshStandardMaterial({ color: 0xd8342c, roughness: 0.85, metalness: 0 });
  const blackMat = new THREE.MeshStandardMaterial({ color: 0x141414, roughness: 0.85, metalness: 0 });
  const rubberFront = new THREE.Mesh(rubberGeo, redMat);
  rubberFront.rotation.x = Math.PI / 2;
  rubberFront.position.z = T / 2 + 0.0016;
  rubberFront.castShadow = true;
  g.add(rubberFront);
  const rubberBack = new THREE.Mesh(rubberGeo, blackMat);
  rubberBack.rotation.x = Math.PI / 2;
  rubberBack.position.z = -(T / 2 + 0.0016);
  rubberBack.castShadow = true;
  g.add(rubberBack);

  // Edge tape
  const edge = new THREE.Mesh(
    new THREE.TorusGeometry(R, 0.0022, 10, 48),
    new THREE.MeshStandardMaterial({ color: 0xe9e4d8, roughness: 0.55 }),
  );
  edge.castShadow = true;
  g.add(edge); // torus lies in XY plane by default — exactly around the blade

  // Handle: along -Y, wooden, with the player-color ring.
  const woodMat = new THREE.MeshStandardMaterial({ color: 0xa9713f, roughness: 0.5 });
  const handle = new THREE.Mesh(
    new THREE.BoxGeometry(0.028, RACKET.handleLength, 0.02),
    woodMat,
  );
  handle.position.y = -(R + RACKET.handleLength / 2 - 0.004);
  handle.castShadow = true;
  g.add(handle);

  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.017, 0.003, 8, 24),
    new THREE.MeshStandardMaterial({
      color: accent,
      emissive: accent,
      emissiveIntensity: 0.9,
      roughness: 0.4,
    }),
  );
  ring.rotation.x = Math.PI / 2;
  ring.position.y = handle.position.y + RACKET.handleLength - 0.02;
  g.add(ring);

  // Invisible marker at the blade centre (front face) for the trail emitter.
  const marker = new THREE.Object3D();
  marker.position.z = T / 2 + 0.003;
  marker.name = 'bladeCenter';
  g.add(marker);

  return g;
}
