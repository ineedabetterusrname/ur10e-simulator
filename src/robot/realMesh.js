import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { MeshBVH } from 'three-mesh-bvh';

const rad = (d) => (d * Math.PI) / 180;

/**
 * Official Universal Robots UR10e mesh (exported from the UR "cojt" graphical
 * documentation, see UR10e_cojt/). The GLB stores one node per kinematic link,
 * posed at joint angles (0, -90, +90, 0, 0, 0)° in millimetres, Z-up — the
 * same base-frame convention as our DH scene graph. Verified against exact DH
 * landmarks: elbow height 793.4 mm (d1+|a2|), wrist x -571.55 (a3), flange
 * face y -290.7 (published zero-pose value).
 */
const GLB_POSE = [0, -90, 90, 0, 0, 0].map(rad);

/** GLB link node name -> joint group index (-1 = static robot root). */
const LINK_TO_JOINT = {
  GROUND: -1,
  BASE: 0,
  SHOULDER: 1,
  ELBOW: 2,
  WRIST1: 3,
  WRIST2: 4,
  FLANGE: 5,
};

/**
 * Loads the real UR10e mesh and swaps it in for the primitive placeholder
 * skin. Registration needs no hand-tuned offsets: the DH robot is posed at
 * the GLB's authoring configuration, the GLB content is inserted in the robot
 * base frame, and Object3D.attach() re-parents each link into its joint group
 * while preserving its world transform — so every link lands exactly and
 * rotates with the correct joint from then on.
 *
 * Also builds exact per-link collision BVHs from a simplified copy of the
 * same mesh (`<url dir>/ur10e-collision.glb`, ±1.5 mm) and installs them as
 * robot.linkColliders — CollisionChecker then switches from the capsule
 * approximation to true mesh-distance tests.
 */
export async function applyRealMesh(robot, url) {
  const loader = new GLTFLoader();
  loader.setMeshoptDecoder(MeshoptDecoder);
  const collisionUrl = url.replace(/ur10e\.glb$/, 'ur10e-collision.glb');
  const [gltf, collisionGltf] = await Promise.all([
    loader.loadAsync(url),
    loader.loadAsync(collisionUrl),
  ]);
  const content = gltf.scene;

  // CAD export materials are matte (metalness 0, roughness 1); keep the
  // official colors but shade them like the anodized aluminium of a real UR.
  content.traverse((o) => {
    if (!o.isMesh) return;
    o.castShadow = o.receiveShadow = true;
    o.material.metalness = 0.25;
    o.material.roughness = 0.55;
  });

  const prevQ = robot.getAngles();
  robot.setAngles(GLB_POSE);
  robot.root.add(content);
  robot.root.updateMatrixWorld(true);
  for (const [name, ji] of Object.entries(LINK_TO_JOINT)) {
    const node = content.getObjectByName(name);
    if (!node) throw new Error(`link node "${name}" missing from ${url}`);
    (ji < 0 ? robot.root : robot.joints[ji]).attach(node);
  }
  robot.root.remove(content);

  const linkColliders = buildLinkColliders(robot, collisionGltf.scene);
  robot.setAngles(prevQ);

  robot.removeCosmetics();
  robot.linkColliders = linkColliders; // installed last: atomic switch-over
}

/**
 * Per-link collision data, each baked into the local frame of the node it is
 * measured against (so a single matrixWorld gives its world pose):
 *   { node, geometry (position-only, boundsTree set), sphere, legacy[], floorExempt }
 * `legacy` lists the old capsule names a link stands in for — static-obstacle
 * exemption sets (e.g. the track rail's) are written in those names.
 * Caller must have the robot posed at GLB_POSE with world matrices current.
 */
function buildLinkColliders(robot, collisionScene) {
  const meta = {
    GROUND: { node: robot.root, legacy: ['base'], floorExempt: true },
    BASE: { node: robot.joints[0], legacy: ['base'], floorExempt: true },
    SHOULDER: { node: robot.joints[1], legacy: ['shoulder', 'upperarm'], floorExempt: false },
    ELBOW: { node: robot.joints[2], legacy: ['forearm'], floorExempt: false },
    WRIST1: { node: robot.joints[3], legacy: ['wrist'], floorExempt: false },
    WRIST2: { node: robot.joints[4], legacy: ['hand'], floorExempt: false },
    FLANGE: { node: robot.joints[5], legacy: ['flange'], floorExempt: false },
  };
  robot.root.add(collisionScene);
  robot.root.updateMatrixWorld(true);

  const colliders = {};
  const toLocal = new THREE.Matrix4();
  const m = new THREE.Matrix4();
  for (const [name, { node, legacy, floorExempt }] of Object.entries(meta)) {
    const linkNode = collisionScene.getObjectByName(name);
    if (!linkNode) throw new Error(`link node "${name}" missing from collision mesh`);
    toLocal.copy(node.matrixWorld).invert();

    // merge all of the link's triangles into one position-only geometry,
    // expressed in the collider node's local frame
    const arrays = [];
    let vertCount = 0;
    linkNode.traverse((o) => {
      if (!o.isMesh) return;
      m.multiplyMatrices(toLocal, o.matrixWorld);
      const src = o.geometry.index
        ? o.geometry.toNonIndexed().getAttribute('position')
        : o.geometry.getAttribute('position');
      const out = new Float32Array(src.count * 3);
      const v = new THREE.Vector3();
      for (let i = 0; i < src.count; i++) {
        v.fromBufferAttribute(src, i).applyMatrix4(m);
        out.set([v.x, v.y, v.z], i * 3);
      }
      arrays.push(out);
      vertCount += src.count;
    });
    const merged = new Float32Array(vertCount * 3);
    let off = 0;
    for (const a of arrays) { merged.set(a, off); off += a.length; }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(merged, 3));
    geometry.boundsTree = new MeshBVH(geometry);
    geometry.computeBoundingSphere();

    colliders[name] = { node, geometry, legacy, floorExempt };
  }
  robot.root.remove(collisionScene);
  return colliders;
}
