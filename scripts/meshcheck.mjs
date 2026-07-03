/**
 * Headless verification that the official UR10e GLB registers exactly onto
 * the DH joint hierarchy via applyRealMesh(). Ground truth: per-link world
 * AABBs parsed straight out of the GLB's own node transforms (mm, Z-up) at
 * its authoring pose q = (0, -90, +90, 0, 0, 0)°. After attach(), re-posing
 * the robot at that q must reproduce those AABBs.
 */
import * as THREE from 'three';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { buildUR10e } from '../src/robot/ur10e.js';
import { applyRealMesh } from '../src/robot/realMesh.js';
import { CollisionChecker } from '../src/robot/collision.js';

const glbPath = fileURLToPath(new URL('../public/ur10e.glb', import.meta.url));
const collisionGlbPath = fileURLToPath(new URL('../public/ur10e-collision.glb', import.meta.url));

// Node lacks the DOM ProgressEvent that FileLoader fires while streaming.
globalThis.ProgressEvent ??= class ProgressEvent extends Event {
  constructor(type, init = {}) { super(type); Object.assign(this, init); }
};

// three's FileLoader goes through fetch(); serve it the local files.
const realFetch = globalThis.fetch;
globalThis.fetch = async (url, opts) => {
  const u = typeof url === 'string' ? url : url.url;
  if (u.includes('ur10e-collision.glb')) return new Response(await readFile(collisionGlbPath));
  if (u.includes('ur10e.glb')) return new Response(await readFile(glbPath));
  return realFetch(url, opts);
};

const rad = (d) => (d * Math.PI) / 180;
const GLB_POSE = [0, -90, 90, 0, 0, 0].map(rad);

// Expected link AABBs in robot-base coords (metres), from the GLB itself.
const EXPECTED = {
  GROUND:   [[-0.0950, -0.0950,  0.0000], [ 0.0950,  0.0950, 0.0993]],
  BASE:     [[-0.0764, -0.0946,  0.0993], [ 0.0764,  0.0764, 0.2749]],
  SHOULDER: [[-0.0764, -0.2692,  0.1043], [ 0.0763, -0.0946, 0.8534]],
  ELBOW:    [[-0.6184, -0.1126,  0.7354], [ 0.0581,  0.0192, 0.8514]],
  WRIST1:   [[-0.6185, -0.2211,  0.7365], [-0.5246, -0.0957, 0.8505]],
  WRIST2:   [[-0.6185, -0.2311,  0.6266], [-0.5246, -0.1170, 0.7365]],
  FLANGE:   [[-0.6165, -0.2907,  0.6219], [-0.5265, -0.2310, 0.7186]],
};
const TOL = 0.003; // metres; weld + meshopt quantization headroom

const robot = buildUR10e();
await applyRealMesh(robot, 'http://localhost/ur10e.glb');

// measure in the Z-up robot base frame, not the Y-up render frame
robot.root.rotation.x = 0;
robot.setAngles(GLB_POSE);
robot.root.updateMatrixWorld(true);

let failed = 0;
const check = (name, cond, detail) => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  if (!cond) failed++;
};

let meshCount = 0;
for (const [name, [emin, emax]] of Object.entries(EXPECTED)) {
  const node = robot.root.getObjectByName(name);
  check(`${name} present`, !!node);
  if (!node) continue;
  node.traverse((o) => { if (o.isMesh) meshCount++; });
  const box = new THREE.Box3().setFromObject(node);
  const err = Math.max(
    ...emin.map((v, k) => Math.abs(box.min.getComponent(k) - v)),
    ...emax.map((v, k) => Math.abs(box.max.getComponent(k) - v)),
  );
  check(`${name} AABB registration`, err < TOL, `max err ${(err * 1000).toFixed(2)} mm`);
}
// GLTFLoader emits one THREE.Mesh per glTF primitive (105 across 30 meshes)
check('mesh nodes attached', meshCount === 105, `${meshCount}/105`);

// primitives swapped out: no leftover placeholder meshes outside the 7 links
let strays = 0;
robot.root.traverse((o) => {
  if (o.isMesh && !Object.keys(EXPECTED).some((n) => o.parent === robot.root.getObjectByName(n) || hasAncestor(o, n))) strays++;
});
function hasAncestor(o, name) {
  for (let p = o.parent; p; p = p.parent) if (p.name === name) return true;
  return false;
}
check('primitive skin removed', strays === 0, `${strays} stray meshes`);

// sanity at home pose: flange mesh centre must track the flange frame
robot.setAngles(robot.specs.home);
robot.root.updateMatrixWorld(true);
const fBox = new THREE.Box3().setFromObject(robot.root.getObjectByName('FLANGE'));
const fc = fBox.getCenter(new THREE.Vector3());
const ff = new THREE.Vector3().setFromMatrixPosition(robot.flange.matrixWorld);
check('FLANGE tracks flange frame at home', fc.distanceTo(ff) < 0.08,
  `centre-to-frame ${(fc.distanceTo(ff) * 1000).toFixed(1)} mm`);

/* ------------------------------------------------------------------ */
/* Mesh-collision behaviour: the checker must fire exactly when the    */
/* real surfaces come together — no early stops, no interpenetration.  */
/* ------------------------------------------------------------------ */

// back to the app's render orientation: floor is world y = 0
robot.root.rotation.x = -Math.PI / 2;
robot.root.updateMatrixWorld(true);

const collider = new CollisionChecker(robot);
const cols = robot.linkColliders;
check('mesh colliders installed', !!cols && Object.keys(cols).length === 7
  && Object.values(cols).every((c) => c.geometry.boundsTree),
  cols ? `${Object.keys(cols).length}/7 links` : 'missing');

// true lowest surface point (world y) across all links, from collider verts
const _v = new THREE.Vector3();
function trueMinY() {
  let minY = Infinity;
  for (const col of Object.values(cols)) {
    const box = new THREE.Box3()
      .copy(col.geometry.boundsTree.getBoundingBox(new THREE.Box3()))
      .applyMatrix4(col.node.matrixWorld);
    if (box.min.y > minY || box.min.y > 0.05) { minY = Math.min(minY, box.min.y); continue; }
    const pos = col.geometry.getAttribute('position');
    for (let i = 0; i < pos.count; i++) {
      const y = _v.fromBufferAttribute(pos, i).applyMatrix4(col.node.matrixWorld).y;
      if (y < minY) minY = y;
    }
  }
  return minY;
}

const pose = (q) => { robot.setAngles(q.map(rad)); robot.root.updateMatrixWorld(true); };
const HOME = [0, -90, -90, -90, 90, 0];
const times = [];
const timedCheck = () => {
  const t0 = performance.now();
  const r = collider.check();
  times.push(performance.now() - t0);
  return r;
};

pose(HOME);
check('home pose is collision-free (mesh)', timedCheck() === null, String(collider.check()));

// user scenario: jogging the elbow from home must not stop early
let earlyHit = null;
for (let e = -45; e >= -150 && !earlyHit; e -= 1) {
  pose([0, -90, e, -90, 90, 0]);
  const hit = timedCheck();
  if (hit) earlyHit = `${hit} at elbow ${e}°`;
}
check('elbow jog from home clear through -150°', earlyHit === null, String(earlyHit));

// folding the forearm all the way back must still protect
let fold = null;
for (let e = -90; e <= 179 && !fold; e += 1) {
  pose([0, -90, e, 0, 0, 0]);
  const hit = timedCheck();
  if (hit) fold = { e, hit };
}
check('deep elbow fold triggers', fold !== null, fold && `${fold.hit} at elbow ${fold.e}°`);

// shoulder sweeps in BOTH directions: while the checker is quiet the mesh
// must never be below the floor, and when it fires near the ground it must
// be genuinely close (no mid-air floor stops)
for (const dir of [+1, -1]) {
  let penetrated = null;
  let stop = null;
  for (let s = 1; s <= 135 && !stop; s += 1) {
    pose([0, -90 + dir * s, -90, -90, 90, 0]);
    const hit = timedCheck();
    const minY = trueMinY();
    if (!hit && minY < -0.002) { penetrated = `minY ${(minY * 1000).toFixed(1)} mm at shoulder ${-90 + dir * s}°`; break; }
    if (hit) stop = { s, hit, minY };
  }
  check(`shoulder ${dir > 0 ? '+' : '-'} sweep never penetrates floor unnoticed`,
    penetrated === null, String(penetrated));
  check(`shoulder ${dir > 0 ? '+' : '-'} sweep stops`, stop !== null,
    stop && `${stop.hit} at shoulder ${-90 + dir * stop.s}°, lowest surface ${(stop.minY * 1000).toFixed(1)} mm`);
  if (stop && stop.hit.startsWith('floor')) {
    check(`shoulder ${dir > 0 ? '+' : '-'} floor stop is genuine (not mid-air)`,
      stop.minY < 0.03, `lowest surface ${(stop.minY * 1000).toFixed(1)} mm above ground`);
  }
}

const avg = times.reduce((s, t) => s + t, 0) / times.length;
check('collision check performance', avg < 2 && Math.max(...times) < 25,
  `avg ${avg.toFixed(2)} ms, max ${Math.max(...times).toFixed(2)} ms over ${times.length} checks`);

console.log(failed === 0 ? '\nAll mesh registration checks passed.' : `\n${failed} CHECK(S) FAILED`);
process.exit(failed === 0 ? 0 : 1);
