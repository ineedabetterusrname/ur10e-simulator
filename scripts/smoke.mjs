/**
 * Headless smoke test — validates the kinematic model without a browser.
 *   1. Scene-graph FK at q = 0 must match the analytic UR10e DH solution
 *      (published zero-pose flange position: [-1184.25, -290.7, 60.85] mm).
 *   2. A DLS IK step must reduce Cartesian error.
 *   3. The self-collision checker must be quiet at home and fire on an
 *      elbow fold, and the floor check must fire when the tool is driven down.
 */
import * as THREE from 'three';
import { buildUR10e, SPECS } from '../src/robot/ur10e.js';
import { Kinematics } from '../src/robot/kinematics.js';
import { CollisionChecker } from '../src/robot/collision.js';

let failures = 0;
const check = (name, ok, detail = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ` — ${detail}` : ''}`);
  if (!ok) failures++;
};

const robot = buildUR10e();
const kin = new Kinematics(robot);
const collider = new CollisionChecker(robot);

// --- 1. FK vs analytic DH -------------------------------------------------
robot.setAngles([0, 0, 0, 0, 0, 0]);
robot.setTCP(0);
const { pos } = kin.tcpInBase();
const expected = new THREE.Vector3(-1.18425, -0.2907, 0.06085);
check(
  'FK zero pose matches UR10e DH solution',
  pos.distanceTo(expected) < 1e-6,
  `got [${pos.toArray().map((v) => (v * 1000).toFixed(2)).join(', ')}] mm`
);

// analytic DH product for a random pose, compared against the scene graph
function analyticFK(q) {
  const T = new THREE.Matrix4();
  const tmp = new THREE.Matrix4();
  SPECS.dh.forEach((p, i) => {
    T.multiply(tmp.makeRotationZ(q[i]));
    T.multiply(tmp.makeTranslation(0, 0, p.d));
    T.multiply(tmp.makeTranslation(p.a, 0, 0));
    T.multiply(tmp.makeRotationX(p.alpha));
  });
  return new THREE.Vector3().setFromMatrixPosition(T);
}
const qRand = [0.4, -1.1, 1.3, -0.7, 0.9, 0.3];
robot.setAngles(qRand);
const graphPos = kin.tcpInBase().pos;
const dhPos = analyticFK(qRand);
check(
  'FK random pose matches analytic DH product',
  graphPos.distanceTo(dhPos) < 1e-6,
  `graph [${graphPos.toArray().map((v) => v.toFixed(5)).join(', ')}] vs DH [${dhPos.toArray().map((v) => v.toFixed(5)).join(', ')}]`
);

// --- 2. IK step reduces error ----------------------------------------------
robot.setAngles(SPECS.home);
const start = kin.tcpInBase().pos.clone();
const goalDx = [0.02, 0.01, -0.015, 0, 0, 0];
const dq = kin.dlsStep(goalDx);
check('DLS IK returns a solution', Array.isArray(dq));
if (dq) {
  robot.setAngles(robot.getAngles().map((q, i) => q + dq[i]));
  const after = kin.tcpInBase().pos;
  const target = start.clone().add(new THREE.Vector3(...goalDx.slice(0, 3)));
  const errBefore = start.distanceTo(target);
  const errAfter = after.distanceTo(target);
  check(
    'IK step reduces Cartesian error',
    errAfter < errBefore * 0.25,
    `${(errBefore * 1000).toFixed(2)} mm -> ${(errAfter * 1000).toFixed(2)} mm`
  );
}

// --- 3. collision checker ---------------------------------------------------
const deg = (d) => (d * Math.PI) / 180;
robot.setAngles(SPECS.home);
robot.setToolCapsule(0.03, 0.05);
robot.root.updateMatrixWorld(true);
check('home pose is collision-free', collider.check() === null, String(collider.check()));

robot.setAngles([0, deg(-90), deg(175), 0, 0, 0]); // fold forearm onto upper arm
robot.root.updateMatrixWorld(true);
const fold = collider.check();
check('elbow fold triggers self-collision', fold !== null, String(fold));

// tilt the straight arm downward (positive shoulder angle) until it dips below the floor
let floorHit = null;
for (let s = 5; s <= 80 && !floorHit; s += 5) {
  robot.setAngles([0, deg(s), 0, 0, 0, 0]);
  robot.root.updateMatrixWorld(true);
  floorHit = collider.check();
}
check('driving the arm down triggers floor collision', floorHit !== null, String(floorHit));

console.log(failures === 0 ? '\nAll smoke tests passed.' : `\n${failures} test(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
