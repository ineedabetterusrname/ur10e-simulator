import { h } from './dom.js';

export class StatusBar {
  constructor(container, { motion, kin, manager }) {
    this.motion = motion;
    this.kin = kin;
    this.manager = manager;
    this._t = 0;
    this._frames = 0;
    this._fps = 0;

    this.stateEl = h('span.sb-state.ok', { text: 'RUNNING' });
    this.tcpEl = h('span');
    this.payloadEl = h('span');
    this.speedEl = h('span');
    this.fpsEl = h('span.dim');
    container.append(
      h('div.sb-inner', {},
        this.stateEl,
        h('span.sep'), this.tcpEl,
        h('span.sep'), this.payloadEl,
        h('span.sep'), this.speedEl,
        h('span.sep'), this.fpsEl,
      ),
    );
  }

  update(dt) {
    this._t += dt;
    this._frames++;
    if (this._t < 0.25) return;
    this._fps = Math.round(this._frames / this._t);
    this._t = 0;
    this._frames = 0;

    const m = this.motion;
    this.stateEl.textContent = m.state.replace('_', ' ');
    this.stateEl.className = 'sb-state ' +
      (m.state === 'RUNNING' ? 'ok' : m.state === 'PROTECTIVE_STOP' ? 'warn' : 'err');

    const { pos } = this.kin.tcpInBase();
    this.tcpEl.textContent = `TCP ${(pos.x * 1000).toFixed(0)}, ${(pos.y * 1000).toFixed(0)}, ${(pos.z * 1000).toFixed(0)} mm`;
    const payload = this.manager.payload();
    this.payloadEl.textContent = `Payload ${payload.toFixed(2)} kg`;
    this.payloadEl.classList.toggle('warn-text', payload > 12.5);
    this.speedEl.textContent = `Speed ${(m.speed * 100).toFixed(0)}%`;
    this.fpsEl.textContent = `${this._fps} fps`;
  }
}
