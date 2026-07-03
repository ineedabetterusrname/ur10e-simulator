import * as THREE from 'three';

const TAU = Math.PI * 2;

/** UR rotation-vector (axis-angle) -> quaternion. */
function rotvecToQuat(rx, ry, rz, out = new THREE.Quaternion()) {
  const angle = Math.hypot(rx, ry, rz);
  if (angle < 1e-12) return out.set(0, 0, 0, 1);
  return out.setFromAxisAngle(new THREE.Vector3(rx / angle, ry / angle, rz / angle), angle);
}

/** Quaternion -> UR rotation vector [rx, ry, rz]. */
function quatToRotvec(q) {
  const w = q.w < 0 ? -q.w : q.w;
  const s = q.w < 0 ? -1 : 1;
  const angle = 2 * Math.acos(Math.min(1, w));
  const den = Math.sqrt(Math.max(0, 1 - w * w));
  if (den < 1e-9) return [0, 0, 0];
  const k = (angle / den) * s;
  return [q.x * k, q.y * k, q.z * k];
}

/**
 * SimBridge — the JS side of the Code Lab's ur_rtde mock.
 *
 * Student code runs to completion instantly while the bridge maintains a
 * *virtual* robot state: motion commands validate against joint limits,
 * moveL targets are IK-solved (subdivided so the TCP path is straight), and
 * state reads return the kinematically propagated pose. The result is an
 * event list the CodeRunner then plays back through the real motion
 * controller — with velocity/acceleration limits and collision protection.
 *
 * All scene-graph mutation happens synchronously inside a single JS task and
 * is restored afterwards, so the live view never flickers.
 */
export class SimBridge {
  constructor(robot, kin) {
    this.robot = robot;
    this.kin = kin;
    this.maxEvents = 3000;
    this.maxCalls = 250000;
    this.events = null;
    this._virtualQ = null;
    this._liveQ = null;
    this._cmd = 0;
    this._time = 0;
    this._tp = { pos: new THREE.Vector3(), quat: new THREE.Quaternion() };
  }

  get recording() { return this.events !== null; }

  begin() {
    this._liveQ = this.robot.getAngles();
    this._virtualQ = [...this._liveQ];
    this.events = [];
    this.notes = [];
    this._cmd = 0;
    this._time = 0;
    this._calls = 0;
    this._speedLabel = null;
    this._gripW = 0.073; // virtual 2FG7 width, starts fully open
  }

  /** Guards every bridge call against scripts that loop forever on reads. */
  _tick() {
    if (this.recording && ++this._calls > this.maxCalls) {
      throw new Error(
        `script made over ${this.maxCalls} robot calls in one run — likely an infinite loop. ` +
        'The simulator records the whole script before playing it, so unbounded ' +
        'while-loops never finish.');
    }
  }

  /** Ends recording, restores the live pose, returns the event list. */
  end() {
    const events = this.events ?? [];
    this.events = null;
    if (this._liveQ) {
      this.robot.setAngles(this._liveQ);
      this.robot.root.updateMatrixWorld(true);
    }
    return events;
  }

  _record(ev) {
    if (!this.recording) throw new Error('no recording in progress');
    if (this.events.length >= this.maxEvents) {
      throw new Error(`program too long (limit ${this.maxEvents} motion events) — infinite loop?`);
    }
    this.events.push(ev);
  }

  _limits() { return this.robot.specs.joints; }

  _checkLimits(q, what) {
    const lim = this._limits();
    for (let i = 0; i < 6; i++) {
      if (!Number.isFinite(q[i])) throw new Error(`${what}: joint ${i + 1} value is not a number`);
      if (q[i] < lim[i].min - 1e-9 || q[i] > lim[i].max + 1e-9) {
        const hint = Math.abs(q[i]) > TAU * 1.05
          ? ' (values look like degrees — ur_rtde expects radians, use math.radians())'
          : '';
        throw new Error(
          `${what}: joint ${i + 1} target ${(q[i] * 180 / Math.PI).toFixed(1)}° outside ` +
          `limits [${(lim[i].min * 180 / Math.PI).toFixed(0)}°, ${(lim[i].max * 180 / Math.PI).toFixed(0)}°]${hint}`);
      }
    }
  }

  /** Runs fn with the scene graph at the virtual pose, then restores it. */
  _withVirtual(fn) {
    this.robot.setAngles(this._virtualQ);
    this.robot.root.updateMatrixWorld(true);
    try {
      return fn();
    } finally {
      this.robot.setAngles(this._liveQ);
      this.robot.root.updateMatrixWorld(true);
    }
  }

  _tcpOfVirtual() {
    return this._withVirtual(() => {
      const { pos, quat } = this.kin.tcpInBase(this._tp);
      return { pos: pos.clone(), quat: quat.clone() };
    });
  }

  // ------------------------------------------------------------- commands

  moveJ(q, speed = 1.05) {
    this._tick();
    this._speedLabel = null;
    this._checkLimits(q, 'moveJ');
    const vcap = THREE.MathUtils.clamp(speed, 0.05, Math.PI);
    const label = `moveJ #${++this._cmd}`;
    const maxDq = Math.max(...q.map((v, i) => Math.abs(v - this._virtualQ[i])));
    this._time += maxDq / vcap;
    this._record({ type: 'movej', q: [...q], vcap, fine: true, label });
    this._virtualQ = [...q];
  }

  servoJ(q) {
    this._tick();
    this._speedLabel = null;
    this._checkLimits(q, 'servoJ');
    this._record({ type: 'movej', q: [...q], vcap: Math.PI, fine: false, label: `servoJ #${++this._cmd}` });
    this._time += 0.05;
    this._virtualQ = [...q];
  }

  /**
   * speedL/speedJ velocity streaming, approximated for the record-then-play
   * model: each call advances the virtual robot by v*dt (dt = the command's
   * `time` parameter). Consecutive calls share one label so playback logs
   * "speedL stream #n" once instead of spamming.
   */
  speedL(xd, dt = 0.05) {
    this._tick();
    if (xd.every((v) => Math.abs(v) < 1e-9)) { this._speedLabel = null; return; }
    const step = THREE.MathUtils.clamp(dt, 0.002, 0.2);
    const { pos, quat } = this._tcpOfVirtual();
    pos.x += xd[0] * step;
    pos.y += xd[1] * step;
    pos.z += xd[2] * step;
    quat.premultiply(rotvecToQuat(xd[3] * step, xd[4] * step, xd[5] * step));
    const label = this._speedLabel ??= `speedL stream #${++this._cmd}`;
    const q = this._solveIK(pos, quat, [...this._virtualQ], label);
    this._record({ type: 'movej', q, vcap: Math.PI, fine: false, label });
    this._time += step;
    this._virtualQ = q;
  }

  speedJ(qd, dt = 0.05) {
    this._tick();
    if (qd.every((v) => Math.abs(v) < 1e-9)) { this._speedLabel = null; return; }
    const step = THREE.MathUtils.clamp(dt, 0.002, 0.2);
    const lim = this._limits();
    const q = this._virtualQ.map((v, i) =>
      THREE.MathUtils.clamp(v + qd[i] * step, lim[i].min, lim[i].max));
    const label = this._speedLabel ??= `speedJ stream #${++this._cmd}`;
    this._record({ type: 'movej', q, vcap: Math.PI, fine: false, label });
    this._time += step;
    this._virtualQ = q;
  }

  speedStop() {
    this._tick();
    this._speedLabel = null;
  }

  moveL(pose, speed = 0.25) {
    this._tick();
    this._speedLabel = null;
    if (Math.max(...pose.slice(0, 3).map(Math.abs)) > 5) {
      throw new Error(
        `moveL: target [${pose.slice(0, 3).map((v) => v.toFixed(0)).join(', ')}] is far outside ` +
        'the 1.3 m reach (positions look like millimetres — ur_rtde expects metres)');
    }
    const label = `moveL #${++this._cmd}`;
    const v = THREE.MathUtils.clamp(speed, 0.01, 3);
    const start = this._tcpOfVirtual();
    const endPos = new THREE.Vector3(pose[0], pose[1], pose[2]);
    const endQuat = rotvecToQuat(pose[3], pose[4], pose[5]);

    const dist = start.pos.distanceTo(endPos);
    const ang = start.quat.angleTo(endQuat);
    const steps = Math.min(120, Math.max(1, Math.ceil(Math.max(dist / 0.02, ang / 0.15))));
    const tStep = Math.max(dist / steps / v, ang / steps / 1.5, 0.02);

    const p = new THREE.Vector3();
    const rot = new THREE.Quaternion();
    let seed = [...this._virtualQ];
    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      p.lerpVectors(start.pos, endPos, t);
      rot.slerpQuaternions(start.quat, endQuat, t);
      const q = this._solveIK(p, rot, seed, label);
      const maxDq = Math.max(...q.map((v2, i) => Math.abs(v2 - seed[i])));
      const vcap = THREE.MathUtils.clamp(maxDq / tStep, 0.05, Math.PI);
      this._record({ type: 'movej', q, vcap, fine: s === steps, label: `${label} (${s}/${steps})` });
      seed = q;
      this._time += tStep;
    }
    this._virtualQ = seed;
  }

  /** OnRobot 2FG7 width command, in millimetres (external grip 35–73). */
  gripMove(widthMm) {
    this._tick();
    this._speedLabel = null;
    if (!Number.isFinite(widthMm)) throw new Error('gripper: width is not a number');
    const w = THREE.MathUtils.clamp(widthMm, 35, 73) / 1000;
    this._record({
      type: 'grip', width: w,
      label: `gripper #${++this._cmd} → ${(w * 1000).toFixed(0)} mm`,
    });
    this._time += Math.abs(w - this._gripW) / 0.15 + 0.1;
    this._gripW = w;
  }

  gripWidth() {
    this._tick();
    return this._gripW * 1000;
  }

  sleep(s) {
    this._tick();
    this._speedLabel = null;
    const dur = THREE.MathUtils.clamp(s, 0, 30);
    this._time += dur;
    this._record({ type: 'sleep', s: dur, label: `sleep ${dur.toFixed(2)}s #${++this._cmd}` });
  }

  // ------------------------------------------------------------- queries

  getQ() { this._tick(); return [...this._virtualQ]; }

  getTCP() {
    this._tick();
    const { pos, quat } = this._tcpOfVirtual();
    return [pos.x, pos.y, pos.z, ...quatToRotvec(quat)];
  }

  fk(q) {
    this._tick();
    this._checkLimits(q, 'getForwardKinematics');
    const saved = this._virtualQ;
    this._virtualQ = [...q];
    try {
      return this.getTCP();
    } finally {
      this._virtualQ = saved;
    }
  }

  ik(pose) {
    this._tick();
    const pos = new THREE.Vector3(pose[0], pose[1], pose[2]);
    const quat = rotvecToQuat(pose[3], pose[4], pose[5]);
    return this._solveIK(pos, quat, [...this._virtualQ], 'getInverseKinematics');
  }

  timestamp() { return this._time; }

  note(text) { this.notes?.push(text); }

  // ------------------------------------------------------------- IK

  /**
   * Damped-least-squares IK from a seed configuration. Mutates the scene
   * graph during iteration; callers restore the live pose afterwards
   * (moveL/_withVirtual paths) — everything stays within one JS task.
   */
  _solveIK(targetPos, targetQuat, seed, what) {
    const robot = this.robot;
    const kin = this.kin;
    const lim = this._limits();
    const q = [...seed];
    const cur = this._tp;
    const qe = new THREE.Quaternion();
    try {
      for (let iter = 0; iter < 80; iter++) {
        robot.setAngles(q);
        robot.root.updateMatrixWorld(true);
        kin.tcpInBase(cur);

        const ex = targetPos.x - cur.pos.x;
        const ey = targetPos.y - cur.pos.y;
        const ez = targetPos.z - cur.pos.z;
        qe.copy(targetQuat).multiply(cur.quat.clone().invert());
        const [rx, ry, rz] = quatToRotvec(qe);
        const posErr = Math.hypot(ex, ey, ez);
        const rotErr = Math.hypot(rx, ry, rz);
        if (posErr < 3e-4 && rotErr < 2e-3) {
          return q.map((v, i) => THREE.MathUtils.clamp(v, lim[i].min, lim[i].max));
        }

        // clamp the step so DLS stays in its linear regime
        const pk = posErr > 0.08 ? 0.08 / posErr : 1;
        const rk = rotErr > 0.3 ? 0.3 / rotErr : 1;
        const k = Math.min(pk, rk);
        const dq = kin.dlsStep([ex * k, ey * k, ez * k, rx * k, ry * k, rz * k]);
        if (!dq) throw new Error(`${what}: near-singular configuration, no IK solution`);
        for (let i = 0; i < 6; i++) {
          q[i] = THREE.MathUtils.clamp(q[i] + dq[i], lim[i].min, lim[i].max);
          // keep revolute values in a sane band to avoid limit windup
          if (q[i] > TAU) q[i] -= TAU;
          if (q[i] < -TAU) q[i] += TAU;
        }
      }
      throw new Error(
        `${what}: target pose unreachable (IK did not converge) — ` +
        `[${targetPos.toArray().map((v) => v.toFixed(3)).join(', ')}] m`);
    } finally {
      robot.setAngles(this._liveQ ?? seed);
      robot.root.updateMatrixWorld(true);
    }
  }
}
