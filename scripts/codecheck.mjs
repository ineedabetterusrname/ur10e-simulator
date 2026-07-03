/**
 * Headless verification of the Code Lab pipeline (everything except Pyodide,
 * whose mocks are thin pass-throughs to the pieces tested here):
 *   SimBridge  — recording, joint limits, FK/IK accuracy, moveL linearity
 *   CodeRunner — playback through the real MotionController, speed caps,
 *                protective-stop abort, vmax restoration
 *   ghimport   — JSON + CSV toolpath parsing
 */
import * as THREE from 'three';
import { buildUR10e } from '../src/robot/ur10e.js';
import { Kinematics } from '../src/robot/kinematics.js';
import { MotionController } from '../src/robot/motion.js';
import { SimBridge } from '../src/code/bridge.js';
import { CodeRunner } from '../src/code/runner.js';
import { parseProgramFile } from '../src/code/ghimport.js';

let failed = 0;
const check = (name, cond, detail = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
  if (!cond) failed++;
};
const rad = (d) => (d * Math.PI) / 180;
const HOME = [0, -90, -90, -90, 90, 0].map(rad);

const robot = buildUR10e();
const kin = new Kinematics(robot);
const motion = new MotionController(robot);
const bridge = new SimBridge(robot, kin);

// ---------------------------------------------------------------- bridge
robot.setAngles(HOME);
robot.root.updateMatrixWorld(true);

bridge.begin();
const tcp0 = bridge.getTCP();
bridge.moveJ([0.3, -1.4, -1.2, -1.0, 1.4, 0.2], 1.0);
check('moveJ records an event', bridge.events.length === 1);
check('getActualQ propagates', Math.abs(bridge.getQ()[0] - 0.3) < 1e-12);

let limitErr = null;
try { bridge.moveJ([0, -1.5, rad(200), 0, 0, 0]); } catch (e) { limitErr = e.message; }
check('moveJ rejects out-of-limit target', /joint 3.*outside limits/.test(limitErr ?? ''), limitErr);

// moveL: straight-line descent 20 cm from home
bridge.end();
bridge.begin();
const start = bridge.getTCP();
const target = [...start];
target[2] -= 0.2;
bridge.moveL(target, 0.25);
const evs = bridge.events;
check('moveL subdivides for linearity', evs.length >= 8, `${evs.length} segments`);

// end pose accuracy: FK of the final segment's joints vs the commanded pose
// (rotation compared as quaternion angle — rotation vectors are ambiguous mod 2pi)
const rvQuat = (p) => {
  const ang = Math.hypot(p[3], p[4], p[5]);
  return ang < 1e-12
    ? new THREE.Quaternion()
    : new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(p[3] / ang, p[4] / ang, p[5] / ang), ang);
};
const endPose = bridge.fk(evs[evs.length - 1].q);
const posErr = Math.hypot(endPose[0] - target[0], endPose[1] - target[1], endPose[2] - target[2]);
const rotErr = rvQuat(endPose).angleTo(rvQuat(target));
check('moveL end pose accurate', posErr < 1e-3 && rotErr < 5e-3,
  `pos err ${(posErr * 1000).toFixed(2)} mm, rot err ${(rotErr * 1000).toFixed(2)} mrad`);

// linearity: every intermediate TCP must lie near the straight line
let maxDev = 0;
const A = new THREE.Vector3(...start.slice(0, 3));
const B = new THREE.Vector3(...target.slice(0, 3));
const line = new THREE.Line3(A, B);
const tmp = new THREE.Vector3();
for (const ev of evs) {
  const p = bridge.fk(ev.q);
  line.closestPointToPoint(new THREE.Vector3(p[0], p[1], p[2]), true, tmp);
  maxDev = Math.max(maxDev, tmp.distanceTo(new THREE.Vector3(p[0], p[1], p[2])));
}
check('moveL path is straight', maxDev < 0.004, `max deviation ${(maxDev * 1000).toFixed(2)} mm`);

bridge.sleep(0.4);
check('sleep records', evs[evs.length - 1].type === 'sleep');

// IK round-trip on a fresh pose
const q6 = bridge.ik(start);
const rt = bridge.fk(q6);
const ikErr = Math.hypot(rt[0] - start[0], rt[1] - start[1], rt[2] - start[2]);
check('ik/fk round-trip', ikErr < 1e-3, `${(ikErr * 1000).toFixed(2)} mm`);

const eventsForPlayback = bridge.end();
check('end() restores live pose', robot.getAngles().every((v, i) => Math.abs(v - HOME[i]) < 1e-12));

// ---------------------------------------------------------------- runner
motion.setPositions(HOME);
motion.apply();
motion.speed = 1.0;
const logs = [];
let doneOk = null;
const runner = new CodeRunner(motion);
runner.play(eventsForPlayback, {
  onLog: (t) => logs.push(t),
  onDone: (ok) => { doneOk = ok; },
});
const vmaxBefore = motion.axes.slice(0, 6).map((a) => a.vmax);
let steps = 0;
while (runner.active && steps++ < 40000) {
  motion.update(1 / 120);
  motion.apply();
  runner.update(1 / 120);
}
const qEnd = motion.getPositions();
const qTarget = eventsForPlayback.filter((e) => e.type === 'movej').pop().q;
check('playback completes', doneOk === true, `${steps} sim steps, logs: ${logs.length}`);
check('playback reaches final target', qTarget.every((q, i) => Math.abs(qEnd[i] - q) < 0.004));
check('vmax restored after playback',
  motion.axes.slice(0, 6).every((a, i) => a.vmax === robot.specs.joints[i].vmax),
  `caps during run differed: ${vmaxBefore.some((v, i) => v !== motion.axes[i].vmax)}`);

// protective stop aborts playback and reports the tripping command
motion.setPositions(HOME);
const logs2 = [];
let done2 = null;
runner.play(eventsForPlayback, { onLog: (t, c) => logs2.push({ t, c }), onDone: (ok) => { done2 = ok; } });
for (let i = 0; i < 30; i++) { motion.update(1 / 120); motion.apply(); runner.update(1 / 120); }
motion.protectiveStop('self-collision: test / test');
runner.update(1 / 120);
check('protective stop aborts playback', done2 === false && !runner.active,
  logs2.at(-1)?.t);
check('vmax restored after abort',
  motion.axes.slice(0, 6).every((a, i) => a.vmax === robot.specs.joints[i].vmax));
motion.reset();

// ---------------------------------------------------------------- ghimport
robot.setAngles(HOME);
// a reachable pose target: 10 cm below the TCP of the second waypoint
const wp2deg = [20, -80, -100, -90, 90, 10];
bridge.begin();
const wp2pose = bridge.fk(wp2deg.map(rad));
bridge.end();
wp2pose[2] -= 0.1;
const json = JSON.stringify({
  units: 'deg',
  moves: [
    { type: 'joints', q: [0, -90, -90, -90, 90, 0], v: 90 },
    { type: 'joints', q: wp2deg },
    { type: 'pose', pose: wp2pose, v: 0.2 },
  ],
});
const ghEvents = parseProgramFile('toolpath.json', json, bridge);
check('GH JSON parses', ghEvents.length >= 3, `${ghEvents.length} events`);
check('GH deg->rad conversion', Math.abs(ghEvents[0].q[1] + Math.PI / 2) < 1e-9);

const csv = '# j1..j6 in degrees\n0,-90,-90,-90,90,0\n15,-75,-95,-90,90,0,45\n';
const csvEvents = parseProgramFile('toolpath.csv', csv, bridge);
check('CSV parses', csvEvents.length === 2 && Math.abs(csvEvents[1].q[0] - rad(15)) < 1e-9,
  `${csvEvents.length} events, wp2 speed cap ${csvEvents[1].vcap.toFixed(2)} rad/s`);

let badErr = null;
try { parseProgramFile('bad.json', '{"moves":[{"type":"pose","pose":[1,2]}]}', bridge); } catch (e) { badErr = e.message; }
check('malformed file rejected', /6 numbers/.test(badErr ?? ''), badErr);
check('bridge idle after imports', !bridge.recording);

console.log(failed === 0 ? '\nAll code-lab checks passed.' : `\n${failed} CHECK(S) FAILED`);
process.exit(failed === 0 ? 0 : 1);
