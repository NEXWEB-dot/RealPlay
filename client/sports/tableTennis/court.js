/**
 * Table tennis court + sports hall (Phase 1 visual target).
 *
 * Real dimensions (metres): table 2.74 long (Z) x 1.525 wide (X), top at
 * 0.76, net 0.1525 high, net overhang 0.1525 each side.
 *
 * Everything is procedural (no asset files yet): plank floor texture, net
 * grid texture and the banner are generated on canvases; materials are PBR
 * (MeshPhysicalMaterial with clearcoat for the glossy table top) and get
 * reflections from the RoomEnvironment PMREM applied by the sport.
 */

import * as THREE from 'three';

export const TABLE = {
  length: 2.74,   // along Z
  width: 1.525,   // along X
  height: 0.76,   // top surface
  thickness: 0.045,
  netHeight: 0.1525,
  netOverhang: 0.1525,
  lineW: 0.02,
};

/* ------------------------------------------------------------------ */
/* Procedural textures                                                 */
/* ------------------------------------------------------------------ */

function makeCanvas(w, h) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  return [c, c.getContext('2d')];
}

function plankTexture() {
  const [c, ctx] = makeCanvas(512, 512);
  const plankW = 64;
  for (let x = 0; x < 512; x += plankW) {
    const l = 0.32 + Math.random() * 0.1;
    ctx.fillStyle = `hsl(28, 42%, ${Math.round(l * 100)}%)`;
    ctx.fillRect(x, 0, plankW, 512);
    // grain streaks
    ctx.globalAlpha = 0.12;
    for (let i = 0; i < 26; i++) {
      ctx.fillStyle = Math.random() > 0.5 ? '#000' : '#fff';
      const y = Math.random() * 512;
      ctx.fillRect(x + Math.random() * plankW, y, 2, 20 + Math.random() * 60);
    }
    ctx.globalAlpha = 1;
    // seam
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.fillRect(x, 0, 2, 512);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(6, 6);
  tex.anisotropy = 4;
  return tex;
}

function netTexture() {
  const [c, ctx] = makeCanvas(64, 64);
  ctx.clearRect(0, 0, 64, 64);
  ctx.strokeStyle = 'rgba(235,235,235,0.85)';
  ctx.lineWidth = 2;
  for (let i = 0; i <= 64; i += 8) {
    ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, 64); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(64, i); ctx.stroke();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(92, 8); // ~2 cm squares across 1.83 x 0.1525 m
  return tex;
}

function bannerTexture(text) {
  const [c, ctx] = makeCanvas(1024, 128);
  ctx.fillStyle = '#101623';
  ctx.fillRect(0, 0, 1024, 128);
  ctx.strokeStyle = '#233150';
  ctx.lineWidth = 6;
  ctx.strokeRect(3, 3, 1018, 122);
  ctx.font = 'bold 64px Arial, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = '#ff8a1e';
  ctx.fillText(text, 512, 68);
  ctx.fillStyle = 'rgba(255,255,255,0.16)';
  ctx.font = 'bold 30px Arial, sans-serif';
  ctx.fillText('· TABLE TENNIS ·', 512, 108);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/* ------------------------------------------------------------------ */
/* Court builder                                                       */
/* ------------------------------------------------------------------ */

/**
 * @param {THREE.Scene} scene
 * @returns {{ lights: LightRig }}
 */
export function buildCourt(scene) {
  const group = new THREE.Group();
  group.name = 'court';
  scene.add(group);

  /* ---- floor ---- */
  const floorMat = new THREE.MeshStandardMaterial({
    map: plankTexture(),
    roughness: 0.55,
    metalness: 0.0,
    envMapIntensity: 0.35,
  });
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(24, 24), floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  group.add(floor);

  /* ---- walls + banners ---- */
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x131a26,
    roughness: 0.9,
    metalness: 0,
    envMapIntensity: 0.15,
  });
  const wallGeo = new THREE.PlaneGeometry(24, 7);
  for (const [z, ry] of [[-8, 0], [8, Math.PI]]) {
    const wall = new THREE.Mesh(wallGeo, wallMat);
    wall.position.set(0, 3.5, z);
    wall.rotation.y = ry;
    group.add(wall);
  }
  for (const [x, ry] of [[-10, Math.PI / 2], [10, -Math.PI / 2]]) {
    const wall = new THREE.Mesh(wallGeo, wallMat);
    wall.position.set(x, 3.5, 0);
    wall.rotation.y = ry;
    group.add(wall);
  }

  const bannerMat = new THREE.MeshBasicMaterial({
    map: bannerTexture('R E A L P L A Y'),
  });
  const banner = new THREE.Mesh(new THREE.PlaneGeometry(7, 0.875), bannerMat);
  banner.position.set(0, 2.6, -7.9);
  group.add(banner);
  const banner2 = banner.clone();
  banner2.position.z = 7.9;
  banner2.rotation.y = Math.PI;
  group.add(banner2);

  /* ---- ceiling light panels (emissive, decorative) ---- */
  const lightPanelMat = new THREE.MeshBasicMaterial({ color: 0xf5f0e0 });
  const panelGeo = new THREE.PlaneGeometry(2.2, 1.1);
  for (let i = 0; i < 4; i++) {
    const panel = new THREE.Mesh(panelGeo, lightPanelMat);
    panel.position.set(-3.6 + i * 2.4, 5.6, 0);
    panel.rotation.x = Math.PI / 2;
    group.add(panel);
  }

  /* ---- table ---- */
  const topMat = new THREE.MeshPhysicalMaterial({
    color: 0x1667c7,
    roughness: 0.22,
    metalness: 0.0,
    clearcoat: 0.9,
    clearcoatRoughness: 0.18,
    envMapIntensity: 0.8,
  });
  const tableTop = new THREE.Mesh(
    new THREE.BoxGeometry(TABLE.width, TABLE.thickness, TABLE.length),
    topMat,
  );
  tableTop.position.y = TABLE.height - TABLE.thickness / 2;
  tableTop.castShadow = true;
  tableTop.receiveShadow = true;
  group.add(tableTop);

  // dark apron under the top
  const apronMat = new THREE.MeshStandardMaterial({ color: 0x0d3b78, roughness: 0.5 });
  const apron = new THREE.Mesh(new THREE.BoxGeometry(TABLE.width - 0.04, 0.05, TABLE.length - 0.04), apronMat);
  apron.position.y = TABLE.height - TABLE.thickness - 0.025;
  group.add(apron);

  // metal legs
  const legMat = new THREE.MeshStandardMaterial({ color: 0x2a2f38, roughness: 0.35, metalness: 0.7 });
  const legGeo = new THREE.BoxGeometry(0.06, TABLE.height - TABLE.thickness - 0.05, 0.06);
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const leg = new THREE.Mesh(legGeo, legMat);
      leg.position.set(sx * (TABLE.width / 2 - 0.25), (TABLE.height - TABLE.thickness - 0.05) / 2, sz * (TABLE.length / 2 - 0.45));
      leg.castShadow = true;
      group.add(leg);
    }
  }

  // white lines
  const lineMat = new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.4 });
  const yLine = TABLE.height + 0.0006;
  const addLine = (w, d, x, z) => {
    const line = new THREE.Mesh(new THREE.BoxGeometry(w, 0.0012, d), lineMat);
    line.position.set(x, yLine, z);
    group.add(line);
  };
  addLine(TABLE.lineW, TABLE.length, -(TABLE.width / 2 - TABLE.lineW / 2), 0);
  addLine(TABLE.lineW, TABLE.length, TABLE.width / 2 - TABLE.lineW / 2, 0);
  addLine(TABLE.width, TABLE.lineW, 0, -(TABLE.length / 2 - TABLE.lineW / 2));
  addLine(TABLE.width, TABLE.lineW, 0, TABLE.length / 2 - TABLE.lineW / 2);
  addLine(0.003, TABLE.length, 0, 0); // centre line (doubles)

  /* ---- net ---- */
  const netGroup = new THREE.Group();
  const netW = TABLE.width + TABLE.netOverhang * 2;
  const netMat = new THREE.MeshBasicMaterial({
    map: netTexture(),
    transparent: true,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  const netMesh = new THREE.Mesh(new THREE.PlaneGeometry(netW, TABLE.netHeight), netMat);
  netMesh.position.y = TABLE.height + TABLE.netHeight / 2;
  netGroup.add(netMesh);

  const tapeMat = new THREE.MeshStandardMaterial({ color: 0xf5f5f5, roughness: 0.5 });
  const tape = new THREE.Mesh(new THREE.BoxGeometry(netW, 0.008, 0.006), tapeMat);
  tape.position.y = TABLE.height + TABLE.netHeight - 0.004;
  tape.castShadow = true;
  netGroup.add(tape);

  const postMat = new THREE.MeshStandardMaterial({ color: 0x1c222c, roughness: 0.4, metalness: 0.6 });
  const postGeo = new THREE.CylinderGeometry(0.008, 0.01, TABLE.netHeight + 0.02, 12);
  for (const sx of [-1, 1]) {
    const post = new THREE.Mesh(postGeo, postMat);
    post.position.set(sx * (TABLE.width / 2 + TABLE.netOverhang), TABLE.height + (TABLE.netHeight + 0.02) / 2 - 0.01, 0);
    post.castShadow = true;
    netGroup.add(post);
    const foot = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.012, 0.16), postMat);
    foot.position.set(post.position.x, TABLE.height - TABLE.thickness + 0.006, 0);
    netGroup.add(foot);
  }
  group.add(netGroup);

  /* ---- lighting ---- */
  const lights = buildLighting(group);

  return { lights };
}

/**
 * @returns {{ key: DirectionalLight, setShadows(enabled, size): void }}
 */
function buildLighting(parent) {
  const hemi = new THREE.HemisphereLight(0x8fb4d9, 0x2e2418, 0.5);
  parent.add(hemi);

  const key = new THREE.DirectionalLight(0xfff1dd, 1.7);
  key.position.set(3.2, 6.0, 2.6);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -3.6;
  key.shadow.camera.right = 3.6;
  key.shadow.camera.top = 3.6;
  key.shadow.camera.bottom = -3.6;
  key.shadow.camera.near = 0.5;
  key.shadow.camera.far = 16;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.02;
  parent.add(key);
  parent.add(key.target);

  const fill = new THREE.DirectionalLight(0x9db8ff, 0.4);
  fill.position.set(-2.5, 4.0, -3.0);
  parent.add(fill);

  return {
    key,
    setShadows(enabled, size) {
      key.castShadow = enabled;
      const s = Math.max(256, size | 0);
      if (key.shadow.mapSize.x !== s) {
        key.shadow.mapSize.set(s, s);
        if (key.shadow.map) {
          key.shadow.map.dispose();
          key.shadow.map = null;
        }
      }
    },
  };
}
