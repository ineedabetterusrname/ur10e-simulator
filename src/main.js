import './style.css';
import * as THREE from 'three';
import { createScene } from './scene.js';
import { buildUR10e } from './robot/ur10e.js';
import { MotionController } from './robot/motion.js';
import { Kinematics } from './robot/kinematics.js';
import { CollisionChecker } from './robot/collision.js';
import { AttachmentManager } from './catalogue/manager.js';
import { Pendant } from './ui/pendant.js';
import { CataloguePanel } from './ui/cataloguePanel.js';
import { StatusBar } from './ui/statusbar.js';

const world = createScene(document.getElementById('app'));
const robot = buildUR10e();
world.scene.add(robot.root);

const motion = new MotionController(robot);
const kin = new Kinematics(robot);
const collider = new CollisionChecker(robot);
const manager = new AttachmentManager(robot, motion, world);

const pendantEl = document.getElementById('pendant');
const pendant = new Pendant(pendantEl, { motion, kin, robot, manager });
new CataloguePanel(document.getElementById('catalogue'), manager);
const status = new StatusBar(document.getElementById('statusbar'), { motion, kin, manager });

// TCP marker triad
const triad = new THREE.AxesHelper(0.09);
robot.toolPoint.add(triad);

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
  pendant.updateReadouts(dt);
  status.update(dt);
  world.controls.update();
  const pipCamera = manager.pipCamera;
  // raise the pendant's bottom edge while the camera preview is visible
  pendantEl.classList.toggle('pip-clear', !!pipCamera);
  world.render(pipCamera);
}
tick();
