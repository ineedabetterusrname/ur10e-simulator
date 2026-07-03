import { partById } from './parts.js';

/**
 * AttachmentManager — mounts catalogue parts onto the robot "smartly":
 *   - end effectors are exclusive (selecting one replaces the current one)
 *   - the F/T sensor mounts inline and shifts the tool stack by its length
 *   - add-ons (camera, proximity sensor) stack on flange brackets
 *   - the base track reparents the robot onto a carriage and registers an
 *     external axis with the motion controller
 * After every change it recomputes the TCP offset, the tool collision
 * capsule and the payload, then notifies the UI so pendant controls adapt.
 */
export class AttachmentManager {
  constructor(robot, motion, world) {
    this.robot = robot;
    this.motion = motion;
    this.world = world;
    this.active = new Map(); // partId -> built instance
    this.staticObstacles = [];
    this._listeners = [];
  }

  onChange(fn) { this._listeners.push(fn); }
  _emit() { this._listeners.forEach((f) => f()); }

  has(id) { return this.active.has(id); }
  activeIds() { return [...this.active.keys()]; }

  /** Toggle semantics: clicking an active part removes it; end effectors replace each other. */
  toggle(id) {
    if (this.active.has(id)) this.remove(id);
    else this.add(id);
  }

  add(id) {
    const def = partById(id);
    if (!def) return;
    if (def.category === 'endEffector') {
      for (const [aid] of this.active) {
        if (partById(aid).category === 'endEffector') this._dispose(aid);
      }
    }
    const inst = def.build();
    inst.def = def;
    this.active.set(id, inst);
    if (def.category === 'base') this._mountTrack(inst);
    else if (def.category === 'world') inst.attachWorld?.(this.world);
    this._rebuildToolChain();
    this._emit();
  }

  remove(id) {
    if (!this.active.has(id)) return;
    this._dispose(id);
    this._rebuildToolChain();
    this._emit();
  }

  _dispose(id) {
    const inst = this.active.get(id);
    this.active.delete(id);
    if (inst.def.category === 'base') this._unmountTrack(inst);
    else if (inst.def.category === 'world') inst.detachWorld?.(this.world);
    else inst.group.removeFromParent();
  }

  _mountTrack(inst) {
    this.world.scene.add(inst.group);
    this.robot.root.removeFromParent();
    inst.carriage.add(this.robot.root);
    this.robot.root.position.set(0, inst.carriageTopY - inst.carriage.position.y, 0);
    this.staticObstacles.push(inst.obstacle);
    this.motion.addExternalAxis({
      ...inst.axis,
      q: 0,
      apply: (q) => { inst.carriage.position.x = q; },
    });
  }

  _unmountTrack(inst) {
    this.robot.root.removeFromParent();
    this.robot.root.position.set(0, 0, 0);
    this.world.scene.add(this.robot.root);
    inst.group.removeFromParent();
    this.staticObstacles = this.staticObstacles.filter((o) => o !== inst.obstacle);
    this.motion.removeExternalAxis(inst.axis.name);
  }

  /** Restack flange-mounted parts: [F/T sensor] -> end effector, plus side add-ons. */
  _rebuildToolChain() {
    const tool0 = this.robot.tool0;
    let zOff = 0;
    const ft = this._byCategory('inline')[0];
    if (ft) {
      tool0.add(ft.group);
      ft.group.position.z = 0;
      zOff = ft.inlineLen;
    }
    const ee = this._byCategory('endEffector')[0];
    if (ee) {
      tool0.add(ee.group);
      ee.group.position.z = zOff;
    }
    for (const addon of this._byCategory('addon')) tool0.add(addon.group);

    const tcpZ = zOff + (ee ? ee.tcpZ : 0);
    this.robot.setTCP(tcpZ);
    const capLen = zOff + (ee ? ee.capsule.len : 0.03);
    const capR = ee ? Math.max(ee.capsule.r, 0.05) : 0.05;
    this.robot.setToolCapsule(capLen, capR);
    this._eeInstance = ee;
    this._zOff = zOff;
  }

  _byCategory(cat) {
    return [...this.active.values()].filter((i) => i.def.category === cat);
  }

  /** Total tool payload in kg (flange-mounted parts only). */
  payload() {
    return [...this.active.values()]
      .filter((i) => i.def.category !== 'base' && i.def.category !== 'world')
      .reduce((s, i) => s + i.def.mass, 0);
  }

  /** Mounted parallel-gripper API (OnRobot 2FG7), or null. */
  get gripper() {
    return this._byCategory('endEffector')[0]?.gripperApi ?? null;
  }

  /**
   * Width command from Code-lab playback [m]; returns the motion duration
   * so the runner can wait for the fingers.
   */
  commandGripper(width) {
    const g = this.gripper;
    return g ? g.command(width) : 0;
  }

  /** Active wrist camera to render as picture-in-picture, or null. */
  get pipCamera() {
    const cam = this.active.get('wristcam');
    return cam && cam.state.pip ? cam.camera : null;
  }

  /** Control groups for the pendant Tool tab. */
  controlGroups() {
    const groups = [];
    for (const inst of this.active.values()) {
      if (inst.controls && inst.controls.length) {
        groups.push({ title: inst.def.name, controls: inst.controls });
      }
    }
    return groups;
  }

  update(dt) {
    const ee = this._eeInstance;
    const massBeyondFT = (ee ? ee.def.mass : 0);
    const comBeyondFT = ee ? this._zOff + ee.tcpZ * 0.45 : 0.01;
    const ctx = { massBeyondFT, comBeyondFT, manager: this };
    for (const inst of this.active.values()) inst.update?.(dt, ctx);
  }
}
