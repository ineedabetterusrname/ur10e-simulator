/**
 * End-to-end check of the Code Lab Python path on real Pyodide (npm package,
 * same major version as the browser CDN pin):
 *   1. The Robo Lab's ur10e_basic_template.py (with its __main__ guard) and
 *      the three shipped sample projects must record motions and then play
 *      back to completion through the real MotionController with the exact
 *      mesh-collision system live — the full browser pipeline, headless.
 *   2. Beginner failure modes must produce clear, teaching-quality errors:
 *      degrees vs radians, millimetres vs metres, input(), desktop-only
 *      imports, runaway loops.
 */
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { existsSync } from 'node:fs';
import { buildUR10e } from '../src/robot/ur10e.js';
import { applyRealMesh } from '../src/robot/realMesh.js';
import { Kinematics } from '../src/robot/kinematics.js';
import { MotionController } from '../src/robot/motion.js';
import { CollisionChecker } from '../src/robot/collision.js';
import { SimBridge } from '../src/code/bridge.js';
import { PyRunner } from '../src/code/pyrunner.js';
import { CodeRunner } from '../src/code/runner.js';
import { EXAMPLES } from '../src/code/examples.js';

let failed = 0;
const check = (name, cond, detail = '') => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
  if (!cond) failed++;
};

// ---- full browser pipeline, headless: real mesh + mesh collision ---------
globalThis.ProgressEvent ??= class ProgressEvent extends Event {
  constructor(type, init = {}) { super(type); Object.assign(this, init); }
};
const glb = fileURLToPath(new URL('../public/ur10e.glb', import.meta.url));
const cglb = fileURLToPath(new URL('../public/ur10e-collision.glb', import.meta.url));
globalThis.fetch = async (url) => {
  const u = String(url?.url ?? url);
  return new Response(await readFile(u.includes('collision') ? cglb : glb));
};

const robot = buildUR10e();
await applyRealMesh(robot, 'http://localhost/ur10e.glb');
const kin = new Kinematics(robot);
const motion = new MotionController(robot);
const collider = new CollisionChecker(robot);
const bridge = new SimBridge(robot, kin);
const runner = new CodeRunner(motion);
const py = new PyRunner(bridge, { importPyodide: () => import('pyodide'), indexURL: null });
console.log('loading pyodide (npm)…');
await py.ensure();

async function record(source) {
  motion.setPositions(robot.specs.home);
  motion.apply();
  robot.root.updateMatrixWorld(true);
  const out = [];
  bridge.begin();
  const res = await py.run(source, (line, cls) => out.push({ line, cls }));
  const events = bridge.end();
  return { res, events, out };
}

/** Plays events exactly like main.js does: motion + collision every tick. */
function playback(events) {
  motion.speed = 1.0;
  let doneOk = null;
  let abortMsg = null;
  runner.play(events, {
    onLog: (t, c) => { if (c === 'err') abortMsg = t; },
    onDone: (ok) => { doneOk = ok; },
  });
  let steps = 0;
  let safeQ = motion.getPositions();
  while (runner.active && steps++ < 200000) {
    motion.update(1 / 120);
    motion.apply();
    robot.root.updateMatrixWorld(true);
    const hit = collider.check();
    if (hit) {
      if (motion.state === 'RUNNING') {
        motion.setPositions(safeQ);
        motion.apply();
        robot.root.updateMatrixWorld(true);
        motion.protectiveStop(hit);
      }
    } else if (motion.state !== 'EMERGENCY_STOP') {
      safeQ = motion.getPositions();
    }
    runner.update(1 / 120);
  }
  if (motion.state !== 'RUNNING') motion.reset();
  return { doneOk, steps, abortMsg };
}

// ---- 1. the Robo Lab basic template (the script the students start with) --
const labTemplate = 'C:/FOLDER/00.TH_OWL/00_RA_JOB/1.Robo_Lab_Projects/UR10e_Documentation/Python_Template/ur10e_basic_template.py';
if (existsSync(labTemplate)) {
  const { res, events, out } = await record(await readFile(labTemplate, 'utf8'));
  check('lab template: runs (main guard honoured)', res.ok && events.length > 0,
    `${events.length} events, first line: ${out[0]?.line}`);
  check('lab template: prints its progress',
    out.some((o) => o.line.includes('Connecting')) && out.some((o) => o.line.includes('Movement complete')));
  const pb = playback(events);
  check('lab template: plays to completion (mesh collision live)', pb.doneOk === true,
    pb.abortMsg ?? `${pb.steps} ticks`);
} else {
  console.log('SKIP  lab template not found at expected path');
}

// ---- 2. the three shipped sample projects ---------------------------------
for (const ex of EXAMPLES) {
  const { res, events, out } = await record(ex.code);
  check(`${ex.name}: records motion`, res.ok && events.length > 0,
    res.ok ? `${events.length} events` : res.error?.split('\n').at(-2));
  const pb = playback(events);
  check(`${ex.name}: plays to completion (collision live)`, pb.doneOk === true,
    pb.abortMsg ?? `${pb.steps} ticks, prints: ${out.filter((o) => !o.cls).length}`);
}

// ---- 3. speedL streaming (main_real.py pattern) ---------------------------
{
  const { res, events } = await record(`
from rtde_control import RTDEControlInterface
c = RTDEControlInterface("x")
c.moveJ([0.0, -1.31, -1.75, -1.66, 1.57, 0.0])
for i in range(40):
    c.speedL([0.05, 0, -0.02, 0, 0, 0], 0.1, 0.05)
c.speedStop()
c.stopScript()
`);
  const stream = events.filter((e) => e.label?.startsWith('speedL stream'));
  check('speedL streams are recorded', res.ok && stream.length === 40,
    res.ok ? `${stream.length} stream steps, label "${stream[0]?.label}"` : res.error?.split('\n').at(-2));
}

// ---- 4. beginner failure modes must teach, not baffle ---------------------
const cases = [
  ['degrees passed to moveJ', 'from rtde_control import *\nRTDEControlInterface("x").moveJ([0, -90, -90, -90, 90, 0])',
    /degrees.*radians/s],
  ['millimetres passed to moveL', 'from rtde_control import *\nRTDEControlInterface("x").moveL([400, -200, 300, 0, 3.14, 0])',
    /millimetres.*metres/s],
  ['input() used', 'x = input("ip? ")', /input\(\) is not available/],
  ['runaway read loop', 'from rtde_receive import *\nr = RTDEReceiveInterface("x")\nwhile True:\n    q = r.getActualQ()',
    /robot calls in one run/],
];
bridge.maxCalls = 5000; // keep the runaway-loop case fast
for (const [name, src, expect] of cases) {
  const { res } = await record(src);
  check(`error hint: ${name}`, !res.ok && expect.test(res.error ?? ''),
    (res.error ?? 'no error').split('\n').filter((l) => l.trim()).at(-1)?.slice(0, 110));
}
bridge.maxCalls = 250000;

// desktop-only import → warning + module error
{
  const out = [];
  motion.setPositions(robot.specs.home); motion.apply();
  bridge.begin();
  const res = await py.run('import cv2\nprint("nope")', (l, c) => out.push({ l, c }));
  bridge.end();
  check('cv2 import warns and fails cleanly',
    !res.ok && out.some((o) => o.l.includes('cannot run in the browser')),
    out[0]?.l?.slice(0, 100));
}

console.log(failed === 0 ? '\nAll Python e2e checks passed.' : `\n${failed} CHECK(S) FAILED`);
process.exit(failed === 0 ? 0 : 1);
