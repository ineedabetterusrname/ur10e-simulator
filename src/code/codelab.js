import { h } from '../ui/dom.js';
import { SimBridge } from './bridge.js';
import { PyRunner } from './pyrunner.js';
import { CodeRunner } from './runner.js';
import { parseProgramFile } from './ghimport.js';
import { EXAMPLES } from './examples.js';

/**
 * CodeLab — the pendant's Code tab. Students paste or upload a ur_rtde
 * Python script (executed in-browser via Pyodide) or a Grasshopper-exported
 * toolpath (JSON/CSV) and watch it run on the simulator with real motion
 * physics and collision protection.
 */
export class CodeLab {
  constructor(pane, { motion, kin, robot }) {
    this.motion = motion;
    this.bridge = new SimBridge(robot, kin);
    this.py = new PyRunner(this.bridge);
    this.runner = new CodeRunner(motion);
    this.busy = false;
    this.pending = null; // { name, events } from an uploaded toolpath

    this.runBtn = h('button.btn', { text: '▶ Run', title: 'Run (Ctrl+Enter)', onclick: () => this._onRun() });
    this.stopBtn = h('button.btn.danger', { text: '■ Stop', onclick: () => this._onStop() });
    this.status = h('span.code-status', { text: 'idle' });

    this.examples = h('select.code-examples', {
      title: 'Load an example script',
      onchange: () => {
        const ex = EXAMPLES[this.examples.value];
        if (ex) {
          this._clearPending();
          this.editor.value = ex.code;
          this.log(`loaded "${ex.name}" — press Run`, 'sys');
        }
        this.examples.value = ''; // back to placeholder so re-selecting works
      },
    }, h('option', { value: '', text: 'Examples…', disabled: '', selected: '' }),
    ...EXAMPLES.map((ex, i) => h('option', { value: i, text: ex.name })));

    this.file = h('input', {
      type: 'file',
      accept: '.py,.json,.csv,.txt',
      style: 'display:none',
      onchange: (e) => this._onFile(e.target.files[0]),
    });
    const loadBtn = h('button.btn', { text: 'Load file…', onclick: () => { this.file.value = ''; this.file.click(); } });
    const clearBtn = h('button.btn', { text: 'Clear', title: 'Clear console', onclick: () => this.consoleEl.replaceChildren() });
    const wideBtn = h('button.btn', {
      text: '⛶ Wide',
      title: 'Toggle a wide editor (more room for code)',
      onclick: () => pane.closest('#pendant')?.classList.toggle('wide'),
    });

    this.editor = h('textarea.code-editor', { spellcheck: 'false' });
    this.editor.value = EXAMPLES[0].code;
    this.editor.addEventListener('input', () => this._clearPending());
    this.editor.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        this._onRun();
      } else if (e.key === 'Tab') {
        e.preventDefault();
        const { selectionStart: s, selectionEnd: t, value } = this.editor;
        this.editor.value = value.slice(0, s) + '    ' + value.slice(t);
        this.editor.selectionStart = this.editor.selectionEnd = s + 4;
      }
    });

    this.banner = h('div.code-banner.hidden');
    this.consoleEl = h('div.code-console');

    pane.append(
      h('h3', { text: 'Code lab — Python (ur_rtde) & toolpath upload' }),
      h('div.code-toolbar', {}, this.runBtn, this.stopBtn, this.examples, loadBtn, clearBtn, wideBtn, this.file, this.status),
      this.editor,
      this.banner,
      this.consoleEl,
    );
    this.log('Run ur_rtde Python on the simulated UR10e — the same script works on the real robot.', 'sys');
    this.log('Pick an example, paste your own code, or load a .py / .json / .csv file. Ctrl+Enter runs.', 'sys');
  }

  log(text, cls) {
    for (const line of String(text).split('\n')) {
      this.consoleEl.append(h(`div.cline${cls ? '.' + cls : ''}`, { text: line }));
    }
    while (this.consoleEl.childElementCount > 400) this.consoleEl.firstChild.remove();
    this.consoleEl.scrollTop = this.consoleEl.scrollHeight;
  }

  _setStatus(t) { this.status.textContent = t; }

  _clearPending() {
    if (!this.pending) return;
    this.pending = null;
    this.banner.classList.add('hidden');
    this.banner.replaceChildren();
  }

  // --------------------------------------------------------------- events

  async _onFile(f) {
    if (!f) return;
    const text = await f.text();
    if (f.name.toLowerCase().endsWith('.py') || (!f.name.match(/\.(json|csv)$/i) && text.includes('import'))) {
      this._clearPending();
      this.editor.value = text;
      this.log(`loaded ${f.name} into the editor — press Run`, 'sys');
      return;
    }
    try {
      const events = parseProgramFile(f.name, text, this.bridge);
      this.pending = { name: f.name, events };
      this.banner.classList.remove('hidden');
      this.banner.replaceChildren(
        h('span', { text: `Toolpath "${f.name}" loaded — ${events.length} motion events. Run plays it.` }),
        h('button.btn.small', { text: '✕', title: 'Discard toolpath', onclick: () => this._clearPending() }),
      );
      this.log(`parsed ${f.name}: ${events.length} motion events — press Run`, 'sys');
    } catch (err) {
      this.log(`could not parse ${f.name}: ${err.message}`, 'err');
    }
  }

  async _onRun() {
    if (this.busy || this.runner.active) return;
    if (this.motion.state !== 'RUNNING') {
      this.log(`robot is in ${this.motion.state} — press Reset first`, 'err');
      return;
    }
    if (this.pending) {
      this._play(this.pending.events);
      return;
    }
    this.busy = true;
    this.runBtn.disabled = true;
    try {
      if (!this.py.ready) {
        await this.py.ensure((s) => { this._setStatus(s); this.log(s, 'sys'); });
      }
      this._setStatus('running script…');
      this.bridge.begin();
      let events;
      try {
        const res = await this.py.run(this.editor.value, (line, cls) => this.log(line, cls));
        events = this.bridge.end();
        for (const n of this.bridge.notes ?? []) this.log(n, 'sys');
        if (!res.ok) {
          this.log(res.error, 'err');
          if (events.length) this.log(`(${events.length} motion events recorded before the error were discarded)`, 'sys');
          this._setStatus('error');
          return;
        }
      } catch (err) {
        this.bridge.end();
        throw err;
      }
      if (events.length === 0) {
        this.log('script finished but commanded no motion', 'sys');
        this._setStatus('idle');
        return;
      }
      this._play(events);
    } catch (err) {
      this.log(String(err?.message ?? err), 'err');
      this._setStatus('error');
    } finally {
      this.busy = false;
      this.runBtn.disabled = false;
    }
  }

  _play(events) {
    const ok = this.runner.play(events, {
      onLog: (t, cls) => this.log(t, cls),
      onDone: (success) => this._setStatus(success ? 'finished' : 'stopped'),
    });
    if (ok) this._setStatus('playing…');
  }

  _onStop() {
    if (this.runner.active) this.runner.stop();
    else this._setStatus('idle');
  }

  update(dt) {
    this.runner.update(dt);
    if (this.runner.active) this._setStatus(`playing ${this.runner.progress}`);
  }
}
