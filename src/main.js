import './style.css';
import * as THREE from 'three';
import { createScene } from './scene.js';
import { buildUR10e } from './robot/ur10e.js';
import { applyRealMesh } from './robot/realMesh.js';
import { MotionController } from './robot/motion.js';
import { Kinematics } from './robot/kinematics.js';
import { CollisionChecker } from './robot/collision.js';
import { AttachmentManager } from './catalogue/manager.js';
import { Pendant } from './ui/pendant.js';
import { CodeLab } from './code/codelab.js';
import { BrickSystem } from './world/bricks.js';
import { CataloguePanel } from './ui/cataloguePanel.js';
import { StatusBar } from './ui/statusbar.js';
import { buildBaseFrameGuide, TrajectoryVisualizer } from './ui/trajectory.js';
import { h } from './ui/dom.js';

const world = createScene(document.getElementById('app'));
const robot = buildUR10e();
world.scene.add(robot.root);

// Swap the primitive placeholder skin for the official UR10e mesh once it
// loads; on failure (e.g. offline) the primitives simply stay.
// Hidden meanwhile: the render loop starts immediately but the swap takes a
// GLB fetch plus seven main-thread BVH builds, and without this the old
// placeholder robot was visibly on screen for that whole window.
robot.setCosmeticsVisible(false);
applyRealMesh(robot, `${import.meta.env.BASE_URL}ur10e.glb`)
  .catch((err) => console.warn('Real UR10e mesh unavailable, keeping primitive skin.', err))
  .finally(() => robot.setCosmeticsVisible(true)); // no-op on success: already disposed

const motion = new MotionController(robot);
const kin = new Kinematics(robot);
const collider = new CollisionChecker(robot);
world.bricks = new BrickSystem(world, robot, () => manager.gripper);
const manager = new AttachmentManager(robot, motion, world);

// Base frame guide (+X, +Y, +Z in UR10e base frame + 1.3m workspace reach ring)
const baseGuide = buildBaseFrameGuide(robot.root);
const trajectory = new TrajectoryVisualizer(robot, kin);

const pendantEl = document.getElementById('pendant');
const pendant = new Pendant(pendantEl, { motion, kin, robot, manager });
const codeLab = new CodeLab(pendant.panes.code, { motion, kin, robot, manager, trajectory });
new CataloguePanel(document.getElementById('catalogue'), manager);
const status = new StatusBar(document.getElementById('statusbar'), { motion, kin, manager });

// TCP marker triad
const triad = new THREE.AxesHelper(0.09);
robot.toolPoint.add(triad);

// Top-center viewport bar: camera presets (Iso / Top / Front / Side) + guide toggles
const setCam = (pos, target = [0, 0.55, 0]) => {
  world.camera.position.set(...pos);
  world.controls.target.set(...target);
  world.controls.update();
};
const axesToggleBtn = h('button.vp-btn.active', {
  text: 'Axes',
  title: 'Toggle Base Frame (+X/+Y/+Z) & 1.3m reach rings',
  onclick: () => {
    baseGuide.visible = !baseGuide.visible;
    triad.visible = baseGuide.visible;
    axesToggleBtn.classList.toggle('active', baseGuide.visible);
  },
});
const pathToggleBtn = h('button.vp-btn.active', {
  text: 'Path',
  title: 'Toggle 3D TCP toolpath & live trail',
  onclick: () => {
    trajectory.setVisible(!trajectory.visible);
    pathToggleBtn.classList.toggle('active', trajectory.visible);
  },
});
const vpBar = h('div.viewport-bar', {},
  h('button.vp-btn', { text: 'Iso', title: 'Isometric 3D view', onclick: () => setCam([2.4, 1.9, 2.6]) }),
  h('button.vp-btn', { text: 'Top', title: 'Top-down XY base-plane view', onclick: () => setCam([0, 3.8, 0.01], [0, 0, 0]) }),
  h('button.vp-btn', { text: 'Front', title: 'Front view (+X right)', onclick: () => setCam([0, 0.8, 3.2]) }),
  h('button.vp-btn', { text: 'Side', title: 'Side view (+X towards camera)', onclick: () => setCam([3.2, 0.8, 0]) }),
  h('span.vp-sep'),
  axesToggleBtn,
  pathToggleBtn,
);
document.body.appendChild(vpBar);

// Compact embed mode (?embed=1): used when the simulator is shown inside a
// small iframe/preview pane. Collapse both side panels, hide the status bar,
// and gently auto-rotate the view until the user interacts.
if (new URLSearchParams(location.search).has('embed')) {
  document.body.classList.add('embed');
  document.getElementById('catalogue').classList.add('collapsed');
  pendantEl.classList.add('collapsed');
  world.controls.autoRotate = true;
  world.controls.autoRotateSpeed = 0.6;
  const stopRotate = () => {
    world.controls.autoRotate = false;
    world.controls.removeEventListener('start', stopRotate);
  };
  world.controls.addEventListener('start', stopRotate);
}

let safeQ = motion.getPositions();
const clock = new THREE.Clock();

function tick() {
  requestAnimationFrame(tick);
  const dt = Math.min(clock.getDelta(), 0.05);

  pendant.applyCartesianJog(dt);
  codeLab.update(dt);
  motion.update(dt);
  motion.apply();
  robot.root.updateMatrixWorld(true);

  const hit = collider.check(manager.staticObstacles);
  if (hit) {
    if (motion.state === 'RUNNING') {
      // revert to the last safe pose and latch a protective stop
      motion.setPositions(safeQ);
      motion.apply();
      robot.root.updateMatrixWorld(true);
      motion.protectiveStop(hit);
    }
  } else if (motion.state !== 'EMERGENCY_STOP') {
    safeQ = motion.getPositions();
  }

  manager.update(dt);
  world.bricks.update(dt);
  trajectory.update();
  pendant.updateReadouts(dt);
  status.update(dt);
  world.controls.update();
  const pipCamera = manager.pipCamera;
  // raise the pendant's bottom edge while the camera preview is visible
  pendantEl.classList.toggle('pip-clear', !!pipCamera);
  world.render(pipCamera);
}
tick();
