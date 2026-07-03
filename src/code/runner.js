/**
 * CodeRunner — plays a recorded event list (from SimBridge or a Grasshopper
 * export) through the MotionController, one event at a time. Motion runs
 * with the controller's real trapezoidal profiles; per-event velocity caps
 * reproduce the commanded moveJ/moveL speeds (still scaled by the pendant's
 * speed override). A protective stop or e-stop aborts the run and reports
 * which command tripped it.
 */
export class CodeRunner {
  constructor(motion) {
    this.motion = motion;
    this.events = [];
    this.i = 0;
    this.active = false;
    this._sleepLeft = 0;
    this._vmax0 = null;
    this._lastCmd = null;
    this._onLog = null;
    this._onDone = null;
    /** Optional hook: (widthMetres) => animation duration in seconds. */
    this.onGripper = null;
  }

  get progress() { return this.active ? `${Math.min(this.i + 1, this.events.length)}/${this.events.length}` : ''; }

  play(events, { onLog, onDone } = {}) {
    if (this.active) this.stop('restarted');
    if (this.motion.state !== 'RUNNING') {
      onLog?.(`cannot start: robot is in ${this.motion.state} — press Reset first`, 'err');
      onDone?.(false);
      return false;
    }
    this.events = events;
    this.i = 0;
    this.active = true;
    this._lastCmd = null;
    this._onLog = onLog ?? null;
    this._onDone = onDone ?? null;
    this._vmax0 = this.motion.axes.slice(0, 6).map((a) => a.vmax);
    this._startEvent();
    return true;
  }

  stop(reason = 'stopped by user') {
    if (!this.active) return;
    this._finish(false, reason);
    for (let i = 0; i < 6; i++) {
      const ax = this.motion.axes[i];
      if (ax) { ax.target = null; }
    }
  }

  _finish(ok, msg) {
    this.active = false;
    if (this._vmax0) {
      this._vmax0.forEach((v, i) => { if (this.motion.axes[i]) this.motion.axes[i].vmax = v; });
      this._vmax0 = null;
    }
    if (msg) this._onLog?.(msg, ok ? 'sys' : 'err');
    this._onDone?.(ok);
  }

  _startEvent() {
    const ev = this.events[this.i];
    if (!ev) {
      this._finish(true, 'program finished');
      return;
    }
    // one log line per user-level command (moveL subdivisions share a label)
    const cmd = ev.label?.split(' (')[0];
    if (cmd && cmd !== this._lastCmd) {
      this._lastCmd = cmd;
      this._onLog?.(`▶ ${cmd}`, 'sys');
    }
    if (ev.type === 'sleep') {
      this._sleepLeft = ev.s;
      return;
    }
    if (ev.type === 'grip') {
      // drive the mounted gripper and wait for the fingers to finish
      this._sleepLeft = Math.max(0.05, this.onGripper?.(ev.width) ?? 0.3);
      return;
    }
    // movej
    for (let i = 0; i < 6; i++) {
      this.motion.axes[i].vmax = Math.min(this._vmax0[i], ev.vcap ?? this._vmax0[i]);
      this.motion.setTarget(i, ev.q[i]);
    }
  }

  _reached(ev) {
    for (let i = 0; i < 6; i++) {
      const ax = this.motion.axes[i];
      const err = Math.abs(ax.q - ev.q[i]);
      if (ev.fine) {
        if (err > 0.002 || Math.abs(ax.v) > 0.02) return false;
      } else if (err > 0.006) {
        return false;
      }
    }
    return true;
  }

  update(dt) {
    if (!this.active) return;
    if (this.motion.state !== 'RUNNING') {
      const ev = this.events[this.i];
      this._finish(false,
        `aborted at ${ev?.label ?? 'start'}: ${this.motion.stopReason ?? this.motion.state}`);
      return;
    }
    const ev = this.events[this.i];
    if (!ev) { this._finish(true, 'program finished'); return; }

    if (ev.type === 'sleep' || ev.type === 'grip') {
      this._sleepLeft -= dt;
      if (this._sleepLeft > 0) return;
    } else if (!this._reached(ev)) {
      return;
    }
    this.i++;
    this._startEvent();
  }
}
