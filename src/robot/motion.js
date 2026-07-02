/**
 * MotionController — per-axis motion with real velocity/acceleration limits.
 *
 * Every axis (6 arm joints + optional external axes such as a linear track)
 * integrates with a trapezoidal profile: jogging commands a velocity,
 * targets are approached with sqrt-decel so the axis arrives without
 * overshoot. A speed override (0..1) scales all velocities, like the real
 * pendant. Safety states mirror a real UR controller: RUNNING,
 * PROTECTIVE_STOP (self-collision, requires Reset) and EMERGENCY_STOP.
 */
export class MotionController {
  constructor(robot) {
    this.robot = robot;
    this.axes = robot.specs.joints.map((spec, i) => ({
      name: spec.name,
      unit: 'rad',
      q: robot.specs.home[i],
      v: 0,
      target: null,
      jogDir: 0,
      min: spec.min,
      max: spec.max,
      vmax: spec.vmax,
      amax: spec.amax,
      atLimit: false,
      external: false,
      apply: null,
    }));
    this.speed = 0.5;
    this.state = 'RUNNING';
    this.stopReason = null;
    this.program = [];
    this.playing = false;
    this.playIndex = 0;
    this.loopProgram = false;
    this._axesListeners = [];
    this._stateListeners = [];
  }

  onAxesChanged(fn) { this._axesListeners.push(fn); }
  onStateChanged(fn) { this._stateListeners.push(fn); }
  _emitAxes() { this._axesListeners.forEach((f) => f()); }
  _emitState() { this._stateListeners.forEach((f) => f()); }

  addExternalAxis(def) {
    this.axes.push({
      name: def.name, unit: def.unit || 'm', q: def.q ?? 0, v: 0, target: null,
      jogDir: 0, min: def.min, max: def.max, vmax: def.vmax, amax: def.amax,
      atLimit: false, external: true, apply: def.apply,
    });
    this.program = []; // waypoints are invalid once the axis set changes
    this.playing = false;
    this._emitAxes();
  }

  removeExternalAxis(name) {
    const i = this.axes.findIndex((a) => a.external && a.name === name);
    if (i >= 0) {
      this.axes.splice(i, 1);
      this.program = [];
      this.playing = false;
      this._emitAxes();
    }
  }

  jog(i, dir) {
    if (this.state !== 'RUNNING') return;
    const ax = this.axes[i];
    if (!ax) return;
    ax.jogDir = dir;
    if (dir !== 0) ax.target = null;
  }

  setTarget(i, value) {
    if (this.state !== 'RUNNING') return;
    const ax = this.axes[i];
    if (!ax) return;
    ax.target = Math.min(ax.max, Math.max(ax.min, value));
    ax.jogDir = 0;
  }

  /** Incremental targets from Cartesian jog / IK. */
  nudge(dq) {
    if (this.state !== 'RUNNING') return;
    for (let i = 0; i < 6; i++) {
      const ax = this.axes[i];
      if (Math.abs(dq[i]) < 1e-12) continue;
      const base = ax.target ?? ax.q;
      ax.target = Math.min(ax.max, Math.max(ax.min, base + dq[i]));
    }
  }

  goHome() {
    if (this.state !== 'RUNNING') return;
    this.robot.specs.home.forEach((q, i) => this.setTarget(i, q));
  }

  estop() {
    this.state = 'EMERGENCY_STOP';
    this.stopReason = 'Emergency stop pressed';
    this.playing = false;
    this.axes.forEach((a) => { a.v = 0; a.jogDir = 0; a.target = null; });
    this._emitState();
  }

  protectiveStop(reason) {
    this.state = 'PROTECTIVE_STOP';
    this.stopReason = reason;
    this.playing = false;
    this.axes.forEach((a) => { a.v = 0; a.jogDir = 0; a.target = null; });
    this._emitState();
  }

  reset() {
    this.state = 'RUNNING';
    this.stopReason = null;
    this._emitState();
  }

  getPositions() { return this.axes.map((a) => a.q); }
  setPositions(qs) {
    qs.forEach((q, i) => { if (this.axes[i]) { this.axes[i].q = q; this.axes[i].v = 0; } });
  }

  saveWaypoint() {
    this.program.push(this.getPositions());
  }
  clearProgram() { this.program = []; this.playing = false; }
  playProgram(loop = false) {
    if (this.state !== 'RUNNING' || this.program.length === 0) return;
    this.loopProgram = loop;
    this.playing = true;
    this.playIndex = 0;
    this._commandWaypoint();
  }
  stopProgram() {
    this.playing = false;
    this.axes.forEach((a) => { a.target = null; });
  }
  _commandWaypoint() {
    const wp = this.program[this.playIndex];
    if (!wp) { this.playing = false; return; }
    wp.forEach((q, i) => this.setTarget(i, q));
  }
  _waypointReached() {
    const wp = this.program[this.playIndex];
    return wp.every((q, i) => Math.abs(this.axes[i].q - q) < 0.003 && Math.abs(this.axes[i].v) < 0.02);
  }

  update(dt) {
    if (this.playing) {
      if (this._waypointReached()) {
        this.playIndex++;
        if (this.playIndex >= this.program.length) {
          if (this.loopProgram) this.playIndex = 0;
          else { this.playing = false; }
        }
        if (this.playing) this._commandWaypoint();
      }
    }

    const ov = this.speed;
    const running = this.state === 'RUNNING';
    for (const ax of this.axes) {
      let vDes = 0;
      if (running) {
        if (ax.jogDir !== 0) {
          vDes = ax.jogDir * ax.vmax * ov;
        } else if (ax.target != null) {
          const err = ax.target - ax.q;
          if (Math.abs(err) < 5e-4 && Math.abs(ax.v) < 0.02) {
            ax.q = ax.target;
            ax.target = null;
            ax.v = 0;
            continue;
          }
          // decel-limited approach speed toward target
          const vCap = Math.sqrt(2 * ax.amax * Math.abs(err));
          vDes = Math.sign(err) * Math.min(vCap, ax.vmax * ov);
        }
      }
      // accelerate toward desired velocity
      const dv = vDes - ax.v;
      const maxDv = ax.amax * dt * (running ? 1 : 3);
      ax.v += Math.abs(dv) > maxDv ? Math.sign(dv) * maxDv : dv;
      ax.q += ax.v * dt;

      ax.atLimit = false;
      if (ax.q <= ax.min) { ax.q = ax.min; ax.v = 0; ax.atLimit = true; }
      if (ax.q >= ax.max) { ax.q = ax.max; ax.v = 0; ax.atLimit = true; }
    }
  }

  /** Applies axis positions to the scene graph (arm joints + external axes). */
  apply() {
    for (let i = 0; i < 6; i++) this.robot.joints[i].rotation.z = this.axes[i].q;
    for (const ax of this.axes) if (ax.external && ax.apply) ax.apply(ax.q);
  }
}
