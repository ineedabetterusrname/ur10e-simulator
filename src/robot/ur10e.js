import * as THREE from 'three';

const rad = (d) => (d * Math.PI) / 180;

/**
 * UR10e specification.
 * DH parameters are the official Universal Robots values (standard DH,
 * T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)), so the scene-graph
 * kinematics match the real robot exactly. Link meshes carry small cosmetic
 * lateral offsets so the silhouette reads like a real UR arm; those offsets
 * do not affect kinematics.
 */
export const SPECS = {
  name: 'UR10e',
  reach: 1.3,
  payload: 12.5,
  dh: [
    { d: 0.1807, a: 0, alpha: Math.PI / 2 },
    { d: 0, a: -0.6127, alpha: 0 },
    { d: 0, a: -0.57155, alpha: 0 },
    { d: 0.17415, a: 0, alpha: Math.PI / 2 },
    { d: 0.11985, a: 0, alpha: -Math.PI / 2 },
    { d: 0.11655, a: 0, alpha: 0 },
  ],
  joints: [
    { name: 'Base', min: rad(-360), max: rad(360), vmax: rad(120), amax: rad(300) },
    { name: 'Shoulder', min: rad(-360), max: rad(360), vmax: rad(120), amax: rad(300) },
    { name: 'Elbow', min: rad(-180), max: rad(180), vmax: rad(180), amax: rad(300) },
    { name: 'Wrist 1', min: rad(-360), max: rad(360), vmax: rad(180), amax: rad(350) },
    { name: 'Wrist 2', min: rad(-360), max: rad(360), vmax: rad(180), amax: rad(350) },
    { name: 'Wrist 3', min: rad(-360), max: rad(360), vmax: rad(180), amax: rad(350) },
  ],
  home: [0, -90, -90, -90, 90, 0].map(rad),
};

const MAT = {
  body: () => new THREE.MeshStandardMaterial({ color: 0xd8dce1, metalness: 0.35, roughness: 0.45 }),
  cap: () => new THREE.MeshStandardMaterial({ color: 0x33373d, metalness: 0.5, roughness: 0.5 }),
  blue: () => new THREE.MeshStandardMaterial({ color: 0x1794d1, metalness: 0.3, roughness: 0.35 }),
  black: () => new THREE.MeshStandardMaterial({ color: 0x17181b, metalness: 0.4, roughness: 0.6 }),
};

function cylZ(r, h, mat) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 40), mat);
  m.rotation.x = Math.PI / 2;
  m.castShadow = m.receiveShadow = true;
  return m;
}
function cylX(r, h, mat) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 40), mat);
  m.rotation.z = Math.PI / 2;
  m.castShadow = m.receiveShadow = true;
  return m;
}
function at(mesh, x, y, z) {
  mesh.position.set(x, y, z);
  return mesh;
}

/**
 * Builds the UR10e as a THREE.Group hierarchy.
 * Each joint i is a Group rotating about its local Z axis; the static link
 * transform Tz(d)·Tx(a)·Rx(alpha) sits between consecutive joints. The root
 * group converts the robot's Z-up convention to three.js Y-up.
 */
export function buildUR10e() {
  const root = new THREE.Group();
  root.name = 'ur10e';
  root.rotation.x = -Math.PI / 2; // Z-up robot base -> Y-up world

  const body = MAT.body();
  const cap = MAT.cap();
  const blue = MAT.blue();
  const black = MAT.black();

  const joints = [];
  let parent = root;
  for (let i = 0; i < 6; i++) {
    const j = new THREE.Group();
    j.name = `joint${i + 1}`;
    parent.add(j);
    joints.push(j);
    const { d, a, alpha } = SPECS.dh[i];
    const fix = new THREE.Group();
    fix.name = `link${i + 1}`;
    fix.position.set(a, 0, d);
    fix.rotation.x = alpha;
    j.add(fix);
    parent = fix;
  }
  const flange = parent; // frame after fix6, Z = tool axis
  const tool0 = new THREE.Group();
  tool0.name = 'tool0';
  flange.add(tool0);
  const toolPoint = new THREE.Object3D();
  toolPoint.name = 'tcp';
  tool0.add(toolPoint);

  // ---- cosmetic meshes -------------------------------------------------
  // base pedestal (static)
  root.add(at(cylZ(0.125, 0.036, black), 0, 0, 0.018));
  root.add(at(cylZ(0.105, 0.02, blue), 0, 0, 0.042));

  // J1: column up to the shoulder
  joints[0].add(at(cylZ(0.088, 0.14, body), 0, 0, 0.115));

  // J2 frame: shoulder housing along joint axis + upper-arm tube
  const armZ = 0.075; // cosmetic lateral offset of the upper-arm tube
  joints[1].add(at(cylZ(0.095, 0.22, body), 0, 0, 0));
  joints[1].add(at(cylZ(0.097, 0.016, blue), 0, 0, 0.112));
  joints[1].add(at(cylZ(0.078, 0.13, cap), 0, 0, armZ + 0.02));
  joints[1].add(at(cylX(0.07, 0.52, body), -0.306, 0, armZ));
  joints[1].add(at(cylZ(0.072, 0.11, cap), -0.6127, 0, armZ - 0.01));

  // J3 frame: elbow housing + forearm tube
  joints[2].add(at(cylZ(0.07, 0.17, body), 0, 0, 0.01));
  joints[2].add(at(cylZ(0.072, 0.014, blue), 0, 0, 0.098));
  joints[2].add(at(cylX(0.055, 0.47, body), -0.285, 0, 0));
  joints[2].add(at(cylZ(0.052, 0.1, cap), -0.57155, 0, 0));

  // J4 (wrist 1): housing + connector along d4
  joints[3].add(at(cylZ(0.05, 0.1, body), 0, 0, 0.0));
  joints[3].add(at(cylZ(0.047, 0.15, cap), 0, 0, 0.095));
  joints[3].add(at(cylZ(0.052, 0.012, blue), 0, 0, 0.052));

  // J5 (wrist 2)
  joints[4].add(at(cylZ(0.05, 0.1, body), 0, 0, 0));
  joints[4].add(at(cylZ(0.047, 0.11, cap), 0, 0, 0.07));
  joints[4].add(at(cylZ(0.052, 0.012, blue), 0, 0, 0.052));

  // J6 (wrist 3) + flange disc
  joints[5].add(at(cylZ(0.045, 0.08, body), 0, 0, 0.0));
  tool0.add(at(cylZ(0.0455, 0.012, black), 0, 0, -0.006));

  // ---- collision capsules ---------------------------------------------
  // Endpoints are kept clear of the joints they hinge on so straight poses
  // do not self-trigger, while genuine folds still overlap. Only used until
  // the official mesh loads — realMesh.js then installs exact per-link BVH
  // colliders and the checker switches to true mesh-distance tests.
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const capsules = {
    base: { node: root, p1: V(0, 0, 0.02), p2: V(0, 0, 0.2), r: 0.115 },
    shoulder: { node: joints[1], p1: V(0, 0, -0.115), p2: V(0, 0, 0.115), r: 0.1 },
    upperarm: { node: joints[1], p1: V(-0.09, 0, armZ), p2: V(-0.52, 0, armZ), r: 0.078 },
    forearm: { node: joints[2], p1: V(-0.09, 0, 0), p2: V(-0.5, 0, 0), r: 0.062 },
    wrist: { node: joints[3], p1: V(0, 0, -0.04), p2: V(0, 0, 0.15), r: 0.058 },
    hand: { node: joints[4], p1: V(0, 0, -0.02), p2: V(0, 0, 0.11), r: 0.055 },
    tool: { node: tool0, p1: V(0, 0, -0.005), p2: V(0, 0, 0.03), r: 0.05 },
  };

  // Everything added above is a placeholder "primitive skin": it renders
  // instantly and is swapped out when the official UR mesh loads (realMesh.js).
  const cosmetics = [];
  root.traverse((o) => { if (o.isMesh) cosmetics.push(o); });

  const robot = {
    specs: SPECS,
    root,
    joints,
    flange,
    tool0,
    toolPoint,
    capsules,
    /**
     * Hidden while the official mesh loads, so the placeholder never flashes
     * on screen; shown again only if that load fails. A no-op once
     * removeCosmetics() has run, which is the success path.
     */
    setCosmeticsVisible(v) {
      for (const m of cosmetics) m.visible = v;
    },
    removeCosmetics() {
      for (const m of cosmetics) {
        m.parent?.remove(m);
        m.geometry.dispose();
        m.material.dispose();
      }
      cosmetics.length = 0;
    },
    setAngles(q) {
      for (let i = 0; i < 6; i++) joints[i].rotation.z = q[i];
    },
    getAngles() {
      return joints.map((j) => j.rotation.z);
    },
    /** Called by the attachment manager when the tool chain changes. */
    setToolCapsule(len, r) {
      capsules.tool.p2.set(0, 0, Math.max(len, 0.03));
      capsules.tool.r = r;
    },
    setTCP(z) {
      toolPoint.position.z = z;
    },
  };
  robot.setAngles(SPECS.home);
  return robot;
}
