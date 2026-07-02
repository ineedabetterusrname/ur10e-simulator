import { h, hold } from './dom.js';

const R2D = 180 / Math.PI;

/**
 * Teach-pendant style control panel.
 * Tabs: Move (joint + Cartesian jog), Tool (adaptive controls contributed by
 * mounted attachments), Program (waypoints), Status.
 */
export class Pendant {
  constructor(container, { motion, kin, robot, manager }) {
    this.container = container;
    this.motion = motion;
    this.kin = kin;
    this.robot = robot;
    this.manager = manager;
    this.cartDir = null; // active Cartesian jog: [vx vy vz wx wy wz] unit
    this.linSpeed = 0.15; // m/s at 100% override
    this.angSpeed = 0.7; // rad/s at 100% override
    this._readoutT = 0;
    this._readouts = [];
    this._jointRows = [];

    container.append(this._buildShell());
    this._buildMoveTab();
    this._buildToolTab();
    this._buildProgramTab();
    this._buildStatusTab();
    this._selectTab('move');

    manager.onChange(() => { this._buildToolTab(); this._refreshCatalogDependent(); });
    motion.onAxesChanged(() => { this._buildMoveTab(); this._buildProgramTab(); });
    motion.onStateChanged(() => this._updateStatePill());
  }

  // ------------------------------------------------------------- shell
  _buildShell() {
    this.statePill = h('span.state-pill', { text: 'RUNNING' });
    this.resetBtn = h('button.btn.reset-btn', {
      text: 'Reset',
      onclick: () => this.motion.reset(),
    });
    this.estopBtn = h('button.estop', {
      title: 'Emergency stop',
      onclick: () => this.motion.estop(),
    }, h('span', { text: 'STOP' }));

    this.speedLabel = h('span.speed-val', { text: '50%' });
    const speed = h('input', {
      type: 'range', min: 1, max: 100, value: 50,
      oninput: (e) => {
        this.motion.speed = e.target.value / 100;
        this.speedLabel.textContent = `${e.target.value}%`;
      },
    });

    this.tabs = {};
    this.panes = {};
    const tabBar = h('div.tabbar');
    for (const [id, label] of [['move', 'Move'], ['tool', 'Tool'], ['prog', 'Program'], ['status', 'Status']]) {
      this.tabs[id] = h('button.tab', { text: label, onclick: () => this._selectTab(id) });
      tabBar.append(this.tabs[id]);
      this.panes[id] = h('div.pane');
    }

    const collapseBtn = h('button.pendant-toggle', {
      text: '›',
      title: 'Collapse / expand panel',
      onclick: () => this.container.classList.toggle('collapsed'),
    });

    return h('div.pendant-shell', {},
      h('header.pendant-head', {},
        h('div.brand', {}, collapseBtn, h('span.brand-dot'), h('span', { text: 'UR10e' }), h('span.brand-sub', { text: 'Teach Pendant' })),
        h('div.head-right', {}, this.statePill, this.resetBtn, this.estopBtn),
      ),
      h('div.speed-row', {}, h('span.lbl', { text: 'Speed' }), speed, this.speedLabel),
      tabBar,
      h('div.pane-wrap', {}, ...Object.values(this.panes)),
    );
  }

  _selectTab(id) {
    for (const [k, tab] of Object.entries(this.tabs)) {
      tab.classList.toggle('active', k === id);
      this.panes[k].classList.toggle('active', k === id);
    }
  }

  _updateStatePill() {
    const s = this.motion.state;
    this.statePill.textContent = s.replace('_', ' ');
    this.statePill.className = 'state-pill ' +
      (s === 'RUNNING' ? 'ok' : s === 'PROTECTIVE_STOP' ? 'warn' : 'err');
    this.resetBtn.classList.toggle('attention', s !== 'RUNNING');
    if (this.stopReasonEl) {
      this.stopReasonEl.textContent = this.motion.stopReason || '';
      this.stopReasonEl.classList.toggle('hidden', !this.motion.stopReason);
    }
  }

  // ------------------------------------------------------------- move tab
  _buildMoveTab() {
    const pane = this.panes.move;
    pane.replaceChildren();
    this._jointRows = [];

    this.stopReasonEl = h('div.stop-reason.hidden');
    pane.append(this.stopReasonEl);

    pane.append(h('h3', { text: 'Joint jog' }));
    this.motion.axes.forEach((ax, i) => {
      const isDeg = ax.unit === 'rad';
      const val = h('span.jval');
      const slider = h('input', {
        type: 'range',
        min: isDeg ? ax.min * R2D : ax.min * 1000,
        max: isDeg ? ax.max * R2D : ax.max * 1000,
        step: 1,
        value: isDeg ? ax.q * R2D : ax.q * 1000,
        oninput: (e) => this.motion.setTarget(i, isDeg ? e.target.value / R2D : e.target.value / 1000),
      });
      const minus = hold(h('button.jog', { text: '−' }), () => this.motion.jog(i, -1), () => this.motion.jog(i, 0));
      const plus = hold(h('button.jog', { text: '+' }), () => this.motion.jog(i, 1), () => this.motion.jog(i, 0));
      pane.append(h('div.jog-row', {},
        h('span.jname', { text: ax.name }), minus, slider, plus, val));
      this._jointRows.push({ ax, val, slider, isDeg });
    });

    pane.append(h('h3', { text: 'TCP jog — base frame' }));
    const grid = h('div.cart-grid');
    const axes = [
      ['X', 0], ['Y', 1], ['Z', 2],
      ['RX', 3], ['RY', 4], ['RZ', 5],
    ];
    for (const [label, idx] of axes) {
      const dirMinus = new Array(6).fill(0); dirMinus[idx] = -1;
      const dirPlus = new Array(6).fill(0); dirPlus[idx] = 1;
      grid.append(h('div.cart-cell', {},
        hold(h('button.jog', { text: '−' }), () => { this.cartDir = dirMinus; }, () => { this.cartDir = null; }),
        h('span.cart-lbl', { text: label }),
        hold(h('button.jog', { text: '+' }), () => { this.cartDir = dirPlus; }, () => { this.cartDir = null; }),
      ));
    }
    pane.append(grid);

    this.tcpReadout = h('div.tcp-readout');
    pane.append(this.tcpReadout);
    pane.append(h('div.row-btns', {},
      h('button.btn', { text: 'Home pose', onclick: () => this.motion.goHome() }),
    ));
    this._updateStatePill();
  }

  // ------------------------------------------------------------- tool tab
  _buildToolTab() {
    const pane = this.panes.tool;
    pane.replaceChildren();
    this._readouts = [];
    const groups = this.manager.controlGroups();
    if (groups.length === 0) {
      pane.append(h('div.empty', {
        text: 'No attachments mounted. Open the catalogue (left) to add end effectors, sensors, a camera or a base track — their controls appear here automatically.',
      }));
      return;
    }
    for (const g of groups) {
      pane.append(h('h3', { text: g.title }));
      for (const c of g.controls) pane.append(this._renderControl(c));
    }
  }

  _renderControl(c) {
    switch (c.type) {
      case 'slider': {
        const val = h('span.jval', { text: c.fmt ? c.fmt(c.get()) : c.get() });
        const input = h('input', {
          type: 'range', min: c.min, max: c.max, step: c.step ?? 1, value: c.get(),
          oninput: (e) => { c.set(Number(e.target.value)); val.textContent = c.fmt ? c.fmt(Number(e.target.value)) : e.target.value; },
        });
        return h('div.ctl-row', {}, h('span.jname', { text: c.label }), input, val);
      }
      case 'toggle': {
        const btn = h('button.toggle', {
          text: c.get() ? 'ON' : 'OFF',
          onclick: () => { c.set(!c.get()); btn.textContent = c.get() ? 'ON' : 'OFF'; btn.classList.toggle('on', c.get()); },
        });
        btn.classList.toggle('on', c.get());
        return h('div.ctl-row', {}, h('span.jname', { text: c.label }), btn);
      }
      case 'button':
        return h('div.ctl-row', {}, h('span.jname'), h('button.btn', { text: c.label, onclick: c.onClick }));
      case 'jog': {
        const i = this.motion.axes.findIndex((a) => a.name === c.axisName);
        const minus = hold(h('button.jog', { text: '−' }), () => this.motion.jog(i, -1), () => this.motion.jog(i, 0));
        const plus = hold(h('button.jog', { text: '+' }), () => this.motion.jog(i, 1), () => this.motion.jog(i, 0));
        return h('div.ctl-row', {}, h('span.jname', { text: c.label }), minus, plus);
      }
      case 'readout': {
        const val = h('span.ro-val', { text: '—' });
        this._readouts.push({ el: val, get: c.get });
        return h('div.ctl-row', {}, h('span.jname', { text: c.label }), val);
      }
      default:
        return h('div');
    }
  }

  // ------------------------------------------------------------- program tab
  _buildProgramTab() {
    const pane = this.panes.prog;
    pane.replaceChildren();
    pane.append(h('h3', { text: 'Waypoint program' }));
    this.wpList = h('div.wp-list');
    const refresh = () => {
      this.wpList.replaceChildren();
      this.motion.program.forEach((wp, i) => {
        this.wpList.append(h('div.wp-item', {},
          h('span', { text: `WP ${i + 1}` }),
          h('span.wp-q', { text: wp.slice(0, 6).map((q) => (q * R2D).toFixed(0) + '°').join(' ') }),
          h('button.btn.small', { text: 'Go', onclick: () => wp.forEach((q, k) => this.motion.setTarget(k, q)) }),
        ));
      });
      if (this.motion.program.length === 0) {
        this.wpList.append(h('div.empty', { text: 'No waypoints yet. Jog the robot, then press "Save waypoint".' }));
      }
    };
    pane.append(h('div.row-btns', {},
      h('button.btn', { text: 'Save waypoint', onclick: () => { this.motion.saveWaypoint(); refresh(); } }),
      h('button.btn', { text: '▶ Play', onclick: () => this.motion.playProgram(false) }),
      h('button.btn', { text: '⟳ Loop', onclick: () => this.motion.playProgram(true) }),
      h('button.btn', { text: '■ Stop', onclick: () => this.motion.stopProgram() }),
      h('button.btn.danger', { text: 'Clear', onclick: () => { this.motion.clearProgram(); refresh(); } }),
    ));
    pane.append(this.wpList);
    refresh();
  }

  // ------------------------------------------------------------- status tab
  _buildStatusTab() {
    const pane = this.panes.status;
    pane.replaceChildren();
    pane.append(h('h3', { text: 'Robot status' }));
    this.statusBody = h('div.status-body');
    pane.append(this.statusBody);
    pane.append(h('h3', { text: 'About' }));
    pane.append(h('div.about', {
      text: 'UR10e — 6-axis collaborative robot. Reach 1300 mm, payload 12.5 kg. ' +
        'Kinematics use official UR DH parameters; motion respects per-joint velocity and ' +
        'acceleration limits; capsule-based self-collision triggers a protective stop.',
    }));
  }

  _refreshCatalogDependent() { /* joint rows rebuilt via onAxesChanged */ }

  // ------------------------------------------------------------- per-frame
  /** Called before motion.update: turns held Cartesian jog into IK nudges. */
  applyCartesianJog(dt) {
    if (!this.cartDir || this.motion.state !== 'RUNNING') return;
    const ov = this.motion.speed;
    const dx = this.cartDir.map((d, i) =>
      d * (i < 3 ? this.linSpeed : this.angSpeed) * ov * dt);
    const dq = this.kin.dlsStep(dx);
    if (dq) this.motion.nudge(dq);
  }

  updateReadouts(dt) {
    this._readoutT += dt;
    if (this._readoutT < 0.1) return;
    this._readoutT = 0;

    for (const row of this._jointRows) {
      const { ax, val, slider, isDeg } = row;
      val.textContent = isDeg ? `${(ax.q * R2D).toFixed(1)}°` : `${(ax.q * 1000).toFixed(0)} mm`;
      val.classList.toggle('limit', ax.atLimit);
      if (document.activeElement !== slider) slider.value = isDeg ? ax.q * R2D : ax.q * 1000;
    }

    const { pos } = this.kin.tcpInBase();
    const rpy = this.kin.tcpRPY();
    if (this.tcpReadout) {
      this.tcpReadout.textContent =
        `TCP  X ${(pos.x * 1000).toFixed(1)}  Y ${(pos.y * 1000).toFixed(1)}  Z ${(pos.z * 1000).toFixed(1)} mm   ` +
        `RX ${(rpy[0] * R2D).toFixed(1)}  RY ${(rpy[1] * R2D).toFixed(1)}  RZ ${(rpy[2] * R2D).toFixed(1)}°`;
    }

    for (const r of this._readouts) r.el.textContent = r.get();

    if (this.panes.status.classList.contains('active')) {
      const m = this.motion;
      this.statusBody.replaceChildren(
        ...m.axes.map((ax) => h('div.status-row', {},
          h('span', { text: ax.name }),
          h('span', { text: ax.unit === 'rad' ? `${(ax.q * R2D).toFixed(2)}°` : `${(ax.q * 1000).toFixed(1)} mm` }),
          h('span.dim', { text: ax.unit === 'rad' ? `${(ax.v * R2D).toFixed(0)}°/s` : `${(ax.v * 1000).toFixed(0)} mm/s` }),
        )),
        h('div.status-row', {}, h('span', { text: 'Payload' }), h('span', { text: `${this.manager.payload().toFixed(2)} / 12.5 kg` })),
        h('div.status-row', {}, h('span', { text: 'State' }), h('span', { text: m.state })),
        m.stopReason ? h('div.status-row', {}, h('span', { text: 'Reason' }), h('span', { text: m.stopReason })) : null,
      );
    }
  }
}
