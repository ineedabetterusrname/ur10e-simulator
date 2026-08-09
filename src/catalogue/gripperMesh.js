import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';

/**
 * Official OnRobot 2FG7 mesh (TraceParts STEP AP242 -> glTF, meshopt +
 * quantization, 212k triangles in 1.07 MB).
 *
 * Unlike the UR export this one has no assembly tree: it is a flattened part
 * of 70 sibling solids whose node names carry no meaning ("empty_2".."empty_71").
 * So the moving parts are found by WHERE THEY SIT rather than what they are
 * called -- which also survives a re-export that renumbers the nodes.
 *
 * Authoring frame is Y-up with the tool axis along +Y: the mounting face is
 * the topmost geometry and the fingertips reach y = -47 mm. Exactly eight
 * solids lie below y = 0, four per finger, and they split cleanly by the sign
 * of their x centre.
 */
const CAD_WIDTH = 0.0518; // pad-face to pad-face opening the CAD was exported at
const FINGER_SOLIDS = 8;

/**
 * Everything above the blue mounting ring is the coupling interior -- 50
 * fasteners, four standoff posts and the adapter plate -- which on the real
 * cell is buried inside the robot's tool flange and never seen. The cut plane
 * is taken from the ring's own top face rather than a constant, so the ring
 * survives its own test.
 */
const COVER_EPS = 1e-4;

/**
 * Recessed panel on the housing front that carries the OnRobot wordmark, in
 * the mesh's authoring frame (metres). Node positions are meshopt-quantized,
 * so this has to be converted into each mesh's local units before use.
 */
const LOGO_PANEL_Z = 0.030;
const LOGO_PLANE_EPS = 5e-5;
/** Guards against stray coplanar slivers on other solids being read as text. */
const MIN_PANEL_TRIS = 200;
const MIN_MARK_TRIS = 100;

const FINGER_COLOR = 0x1c1c1e; // black-oxide fingers and grip pads

let cachedRig = null;

/**
 * Loads and assembles the rig ONCE per session: catalogue toggles rebuild the
 * part, and each instance clones this tree (geometry and materials shared).
 */
function loadRig(url) {
  if (!cachedRig) {
    const loader = new GLTFLoader();
    loader.setMeshoptDecoder(MeshoptDecoder);
    cachedRig = loader.loadAsync(url)
      .then((gltf) => assemble(gltf.scene, url))
      // Don't cache a failure: the part can be re-added from the catalogue, and
      // a single bad fetch would otherwise pin it to the placeholder all session.
      .catch((err) => { cachedRig = null; throw err; });
  }
  return cachedRig;
}

const isBlue = ({ r, g, b }) => b > 0.3 && b > r * 2 && b > g * 1.5;
const isGreen = ({ r, g, b }) => g > 0.85 && r < 0.15 && b < 0.15;

function assemble(content, url) {
  content.updateMatrixWorld(true);

  content.traverse((o) => {
    if (!o.isMesh) return;
    o.castShadow = o.receiveShadow = true;
    // CAD export materials are matte; shade them like the real anodized
    // aluminium and black-oxide fingers.
    o.material = o.material.clone();
    o.material.metalness = 0.25;
    o.material.roughness = 0.5;
  });

  const box = new THREE.Box3();
  const centre = new THREE.Vector3();

  // The mounting ring is the topmost blue solid, and its top face is the plane
  // the coupling interior sits above. Found before any recolouring, so the
  // green band recoloured below cannot be mistaken for it.
  let blue = new THREE.Color(0x1f6fb8);
  let coverY = Infinity;
  for (const node of content.children) {
    const mesh = node.isMesh ? node : node.children.find((o) => o.isMesh);
    if (!mesh || !isBlue(mesh.material.color)) continue;
    box.setFromObject(node);
    if (coverY === Infinity || box.max.y > coverY) {
      coverY = box.max.y;
      blue = mesh.material.color.clone();
    }
  }

  // The CAD marks one band in pure green as a construction colour; the real
  // 2FG7 wears OnRobot blue there.
  content.traverse((o) => {
    if (o.isMesh && isGreen(o.material.color)) o.material.color.copy(blue);
  });

  const housing = new THREE.Group();
  const fingerNeg = new THREE.Group();
  const fingerPos = new THREE.Group();
  housing.name = 'housing';
  fingerNeg.name = 'fingerNeg';
  fingerPos.name = 'fingerPos';

  let topY = -Infinity;
  for (const node of [...content.children]) {
    box.setFromObject(node).getCenter(centre);
    if (box.max.y > coverY + COVER_EPS) continue; // dropped: coupling interior
    if (box.max.y > topY) topY = box.max.y;
    if (centre.y >= 0) housing.add(node);
    else (centre.x < 0 ? fingerNeg : fingerPos).add(node);
  }

  const found = fingerNeg.children.length + fingerPos.children.length;
  if (found !== FINGER_SOLIDS || fingerNeg.children.length !== fingerPos.children.length) {
    throw new Error(
      `2FG7 mesh: expected ${FINGER_SOLIDS} finger solids split evenly, got ` +
      `${fingerNeg.children.length}/${fingerPos.children.length} from ${url}`
    );
  }

  for (const finger of [fingerNeg, fingerPos]) {
    finger.traverse((o) => { if (o.isMesh) o.material.color.setHex(FINGER_COLOR); });
  }
  housing.traverse((o) => { if (o.isMesh) paintLogo(o, blue); });

  // Y-up tool axis -> the scene's +Z-out-of-flange. Seated on what REMAINS
  // rather than on the CAD's original mounting face: with the coupling gone
  // that face no longer exists, and seating on it left the gripper floating
  // 9.2 mm clear of the flange. The mounting ring now meets the flange, which
  // also brings the visible height to 147 mm against a 144 mm datasheet
  // figure -- the coupling was never part of that number.
  // Rotating about X leaves finger travel on X, so the groups below still
  // translate along the scene's X.
  const rig = new THREE.Group();
  rig.name = 'gripper2fg7';
  rig.rotation.x = -Math.PI / 2;
  rig.position.z = topY;
  rig.add(housing, fingerNeg, fingerPos);
  return rig;
}

/**
 * Recolours the OnRobot wordmark.
 *
 * It is engraved into the housing solid as coplanar faces on a recessed panel,
 * and the source STEP does style it in OnRobot blue -- but that file carries 122
 * colour assignments for only 70 solids, and the STEP->glTF conversion collapses
 * them to one material per solid. The wordmark therefore arrives the same
 * near-white as the body and is invisible against it.
 *
 * Recovering it geometrically: on the panel plane, the wordmark is every
 * connected triangle island EXCEPT the panel background itself (the single
 * largest one). Those triangles move to a second material group.
 *
 * A no-op on any mesh without panel-plane geometry, so a future re-export that
 * preserves face colours simply skips this.
 */
function paintLogo(mesh, blue) {
  const geom = mesh.geometry;
  const index = geom.getIndex();
  const pos = geom.getAttribute('position');
  if (!index || !pos) return;

  // Positions are meshopt-quantized: the node carries the scale and offset, so
  // the panel plane has to be expressed in this mesh's local units.
  const scaleZ = mesh.scale.z || 1;
  const eps = LOGO_PLANE_EPS / Math.abs(scaleZ);
  const idx = index.array;
  const triCount = idx.length / 3;

  const isMark = new Uint8Array(triCount);
  let markTris = 0;

  // The wordmark is engraved on both faces of the housing. Each panel is
  // classified separately: with both at once, one panel's background would
  // lose the "largest island" test and get painted as if it were lettering.
  for (const side of [1, -1]) {
    const panelZ = (side * LOGO_PANEL_Z - mesh.position.z) / scaleZ;
    const onPanel = new Uint8Array(triCount);
    let panelTris = 0;
    for (let t = 0; t < triCount; t++) {
      let flat = true;
      for (let k = 0; k < 3; k++) {
        if (Math.abs(pos.getZ(idx[t * 3 + k]) - panelZ) > eps) { flat = false; break; }
      }
      if (flat) { onPanel[t] = 1; panelTris++; }
    }
    if (panelTris < MIN_PANEL_TRIS) continue;

    // union-find over the panel triangles' vertices -> connected islands
    const parent = new Int32Array(pos.count);
    for (let i = 0; i < parent.length; i++) parent[i] = i;
    const find = (x) => { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
    const union = (a, b) => { a = find(a); b = find(b); if (a !== b) parent[b] = a; };
    for (let t = 0; t < triCount; t++) {
      if (!onPanel[t]) continue;
      union(idx[t * 3], idx[t * 3 + 1]);
      union(idx[t * 3 + 1], idx[t * 3 + 2]);
    }
    // The background is the island that SPANS the panel -- picked by extent,
    // not by triangle count: the background is a single flat face of ~200
    // triangles while one curved glyph can carry seven times that.
    const span = new Map();
    for (let t = 0; t < triCount; t++) {
      if (!onPanel[t]) continue;
      const root = find(idx[t * 3]);
      let s = span.get(root);
      if (!s) span.set(root, (s = { x0: Infinity, x1: -Infinity, y0: Infinity, y1: -Infinity }));
      for (let k = 0; k < 3; k++) {
        const v = idx[t * 3 + k];
        const x = pos.getX(v);
        const y = pos.getY(v);
        if (x < s.x0) s.x0 = x;
        if (x > s.x1) s.x1 = x;
        if (y < s.y0) s.y0 = y;
        if (y > s.y1) s.y1 = y;
      }
    }
    let background = -1;
    let biggest = -1;
    for (const [root, s] of span) {
      const area = (s.x1 - s.x0) * (s.y1 - s.y0);
      if (area > biggest) { biggest = area; background = root; }
    }

    for (let t = 0; t < triCount; t++) {
      if (!onPanel[t] || isMark[t] || find(idx[t * 3]) === background) continue;
      isMark[t] = 1;
      markTris++;
    }
  }
  if (markTris < MIN_MARK_TRIS) return;

  // reorder indices so the wordmark triangles form one contiguous range
  const body = [];
  const mark = [];
  for (let t = 0; t < triCount; t++) {
    (isMark[t] ? mark : body).push(idx[t * 3], idx[t * 3 + 1], idx[t * 3 + 2]);
  }

  index.set(Uint32Array.from([...body, ...mark]));
  index.needsUpdate = true;
  geom.clearGroups();
  geom.addGroup(0, body.length, 0);
  geom.addGroup(body.length, mark.length, 1);

  const logoMat = mesh.material.clone();
  logoMat.color.copy(blue);
  mesh.material = [mesh.material, logoMat];
}

/**
 * Adds the gripper to `group` (a part's tool-frame subtree, +Z out of the
 * flange) and returns { setWidth } to drive the fingers.
 *
 * The real fingers rest at the CAD's 51.8 mm opening rather than the primitive
 * skin's symmetric-about-zero layout, so the caller must swap drivers rather
 * than keep using its own.
 */
export async function applyGripperMesh(group, url) {
  const rig = (await loadRig(url)).clone(true);
  const fingerNeg = rig.getObjectByName('fingerNeg');
  const fingerPos = rig.getObjectByName('fingerPos');
  group.add(rig);

  return {
    setWidth(w) {
      const d = (w - CAD_WIDTH) / 2;
      fingerPos.position.x = d;
      fingerNeg.position.x = -d;
    },
  };
}
