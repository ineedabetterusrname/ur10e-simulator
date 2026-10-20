import * as THREE from 'three';

/**
 * Creates a crisp sprite text label for 3D axes/rings.
 * Safe in headless Node environments (returns a dummy Object3D if no document).
 */
function makeTextSprite(text, color = '#ffffff', scale = 0.09) {
  if (typeof document === 'undefined') return new THREE.Object3D();
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 96;
  const ctx = canvas.getContext('2d');
  ctx.font = 'bold 42px "Segoe UI", system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = 'rgba(13, 17, 22, 0.78)';
  ctx.beginPath();
  ctx.roundRect(24, 14, 208, 68, 20);
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = 4;
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.fillText(text, 128, 50);

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(scale * (256 / 96), scale, 1);
  sprite.renderOrder = 999;
  return sprite;
}

/**
 * Builds the UR10e Base Frame coordinate guide (+X, +Y, +Z in robot base frame,
 * where Z is up) and the 1.3 m workspace reach ring. Attached to `robot.root`
 * so it is always exact in the robot's base coordinate system.
 */
export function buildBaseFrameGuide(robotRoot) {
  const group = new THREE.Group();
  group.name = 'baseFrameGuide';
  group.position.z = 0.004; // slightly above floor in robot Z-up frame

  const axisLen = 0.35;
  const xArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0),
    axisLen,
    0xff4d58,
    0.045,
    0.024
  );
  const yArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, 0),
    axisLen,
    0x38d976,
    0.045,
    0.024
  );
  const zArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 0, 0),
    0.26,
    0x3ca4ff,
    0.045,
    0.024
  );
  group.add(xArrow, yArrow, zArrow);

  const xLbl = makeTextSprite('+X (base)', '#ff6b74', 0.055);
  xLbl.position.set(axisLen + 0.08, 0, 0.03);
  const yLbl = makeTextSprite('+Y (base)', '#4ce887', 0.055);
  yLbl.position.set(0, axisLen + 0.08, 0.03);
  const zLbl = makeTextSprite('+Z', '#5cb4ff', 0.05);
  zLbl.position.set(0, 0, 0.32);
  group.add(xLbl, yLbl, zLbl);

  // Reach rings in the XY plane of robot.root (radius 0.5 m and 1.3 m max reach)
  for (const [radius, hex, label] of [
    [0.5, 0x2c3644, null],
    [1.0, 0x2c3644, null],
    [1.3, 0x1794d1, '1.3m max reach'],
  ]) {
    const segs = 96;
    const pts = [];
    for (let i = 0; i <= segs; i++) {
      const a = (i / segs) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(a) * radius, Math.sin(a) * radius, 0.001));
    }
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineDashedMaterial({
      color: hex,
      dashSize: radius === 1.3 ? 0.05 : 0.03,
      gapSize: 0.03,
      transparent: true,
      opacity: radius === 1.3 ? 0.55 : 0.35,
    });
    const ring = new THREE.Line(geom, mat);
    ring.computeLineDistances();
    group.add(ring);

    if (label) {
      const rLbl = makeTextSprite(label, '#49b6eb', 0.05);
      rLbl.position.set(radius * Math.cos(-Math.PI / 4), radius * Math.sin(-Math.PI / 4), 0.025);
      group.add(rLbl);
    }
  }

  robotRoot.add(group);
  return group;
}

/**
 * TrajectoryVisualizer — renders both:
 *   1. The planned 3D toolpath curve + waypoint dots from recorded events
 *      (in robot.root base frame so it moves with the base track if mounted).
 *   2. A live trailing ribbon behind the TCP as the robot moves.
 */
export class TrajectoryVisualizer {
  constructor(robot, kin) {
    this.robot = robot;
    this.kin = kin;
    this.group = new THREE.Group();
    this.group.name = 'trajectories';
    robot.root.add(this.group);

    // Planned toolpath line + waypoint spheres
    this.plannedLine = null;
    this.plannedDots = new THREE.Group();
    this.group.add(this.plannedDots);

    // Live TCP trail (ring buffer of recent positions in robot base frame)
    this.maxTrail = 220;
    this.trailPts = [];
    this.trailGeom = new THREE.BufferGeometry();
    this.trailMat = new THREE.LineBasicMaterial({
      color: 0xf5b82e,
      transparent: true,
      opacity: 0.7,
    });
    this.trailLine = new THREE.Line(this.trailGeom, this.trailMat);
    this.group.add(this.trailLine);
    this._lastSample = new THREE.Vector3(Infinity, Infinity, Infinity);
    this._tp = { pos: new THREE.Vector3(), quat: new THREE.Quaternion() };
  }

  setVisible(v) {
    this.group.visible = v;
  }

  get visible() {
    return this.group.visible;
  }

  clearPlanned() {
    if (this.plannedLine) {
      this.plannedLine.removeFromParent();
      this.plannedLine.geometry.dispose();
      this.plannedLine.material.dispose();
      this.plannedLine = null;
    }
    while (this.plannedDots.children.length) {
      const c = this.plannedDots.children[0];
      c.removeFromParent();
      c.geometry?.dispose();
      c.material?.dispose();
    }
  }

  clearTrail() {
    this.trailPts.length = 0;
    this.trailGeom.setFromPoints([]);
    this._lastSample.set(Infinity, Infinity, Infinity);
  }

  /**
   * Computes and displays the 3D TCP path for a recorded event list.
   */
  showPlannedEvents(events, bridge) {
    this.clearPlanned();
    if (!events || events.length === 0) return;

    const savedQ = this.robot.getAngles();
    const pts = [];
    const keyPts = [];

    try {
      // Start at current robot pose
      const p0 = bridge.fk(savedQ);
      pts.push(new THREE.Vector3(p0[0], p0[1], p0[2]));
      keyPts.push(new THREE.Vector3(p0[0], p0[1], p0[2]));

      for (const ev of events) {
        if (ev.type !== 'movej' || !ev.q) continue;
        // Interpolate joint steps for pure moveJ so curved paths look smooth
        const prevQ = pts._lastQ ?? savedQ;
        const maxDq = Math.max(...ev.q.map((v, i) => Math.abs(v - prevQ[i])));
        const sub = ev.fine && maxDq > 0.15 ? Math.min(12, Math.ceil(maxDq / 0.12)) : 1;
        for (let s = 1; s <= sub; s++) {
          const t = s / sub;
          const qInterp = prevQ.map((v, i) => v + (ev.q[i] - v) * t);
          const pose = bridge.fk(qInterp);
          pts.push(new THREE.Vector3(pose[0], pose[1], pose[2]));
        }
        pts._lastQ = ev.q;
        if (ev.fine) {
          const endPose = pts[pts.length - 1];
          keyPts.push(endPose.clone());
        }
      }
    } finally {
      this.robot.setAngles(savedQ);
      this.robot.root.updateMatrixWorld(true);
    }

    if (pts.length < 2) return;

    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color: 0x22d3ee, transparent: true, opacity: 0.9 });
    this.plannedLine = new THREE.Line(geom, mat);
    this.group.add(this.plannedLine);

    const dotGeom = new THREE.SphereGeometry(0.007, 12, 10);
    const dotMat = new THREE.MeshBasicMaterial({ color: 0x22d3ee });
    for (const kp of keyPts) {
      const m = new THREE.Mesh(dotGeom, dotMat);
      m.position.copy(kp);
      this.plannedDots.add(m);
    }
  }

  /** Samples live TCP position each frame and appends to trail when moving. */
  update() {
    if (!this.group.visible) return;
    const { pos } = this.kin.tcpInBase(this._tp);
    if (pos.distanceToSquared(this._lastSample) > 0.006 * 0.006) {
      this._lastSample.copy(pos);
      this.trailPts.push(pos.clone());
      if (this.trailPts.length > this.maxTrail) this.trailPts.shift();
      if (this.trailPts.length >= 2) {
        this.trailGeom.setFromPoints(this.trailPts);
      }
    }
  }
}
