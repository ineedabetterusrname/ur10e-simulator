import { h } from '../ui/dom.js';
import { SimBridge } from './bridge.js';
import { PyRunner } from './pyrunner.js';
import { CodeRunner } from './runner.js';
import { parseProgramFile, isURScript } from './ghimport.js';
import { EXAMPLES } from './examples.js';

const STORAGE_KEY = 'ur10e_codelab_source_v1';

/**
 * CodeLab — the pendant's Code tab. Students paste or upload a ur_rtde
 * Python script (executed in-browser via Pyodide), a URScript program from
 * the Grasshopper "Robots" plugin (.script), or a Grasshopper-exported
 * toolpath (.json/.csv/.ghx) and watch it run on the simulator with real
 * motion physics, 3D trajectory preview, and collision protection.
 */
export class CodeLab {
  constructor(pane, { motion, kin, robot, manager, trajectory }) {
    this.motion = motion;
    this.manager = manager;
    this.trajectory = trajectory ?? null;
    this.bridge = new SimBridge(robot, kin);
    this.py = new PyRunner(this.bridge);
    this.runner = new CodeRunner(motion);
    this.runner.onGripper = (w) => manager?.commandGripper(w) ?? 0;
    this.busy = false;
    this.pending = null; // { name, events } from an uploaded toolpath
    this._watchHandle = null;
    this._watchLastMod = 0;
    this._watchTimer = 0;

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
          this._saveToStorage();
          this.log(`loaded "${ex.name}" — press Run`, 'sys');
        }
        this.examples.value = ''; // back to placeholder so re-selecting works
      },
    }, h('option', { value: '', text: 'Examples…', disabled: '', selected: '' }),
    ...EXAMPLES.map((ex, i) => h('option', { value: i, text: ex.name })));

    this.file = h('input', {
      type: 'file',
      accept: '.py,.script,.urp,.json,.csv,.ghx,.gh,.txt',
      style: 'display:none',
      onchange: (e) => this._onFile(e.target.files[0]),
    });
    const loadBtn = h('button.btn', {
      text: 'Load file…',
      title: 'Load .py, .script (Robots plugin), .json, .csv, or .ghx',
      onclick: () => { this.file.value = ''; this.file.click(); },
    });
    this.watchBtn = h('button.btn', {
      text: '🔄 Live Sync',
      title: 'Watch a Grasshopper output file (.script / .json) on disk and auto-update when it changes',
      onclick: () => this._toggleWatch(),
    });
    const ghNodeBtn = h('button.btn', {
      text: '⬇ GH Node',
      title: 'Download the ready-to-use Grasshopper sync component (drag onto your Grasshopper canvas)',
      onclick: () => this._downloadGHNode(),
    });
    const savePyBtn = h('button.btn', {
      text: '⬇ .py',
      title: 'Download current editor code as a .py file',
      onclick: () => this._downloadEditorCode(),
    });
    const clearBtn = h('button.btn', {
      text: 'Clear',
      title: 'Clear console & path preview',
      onclick: () => {
        this.consoleEl.replaceChildren();
        this.trajectory?.clearPlanned();
        this.trajectory?.clearTrail();
      },
    });
    const wideBtn = h('button.btn', {
      text: '⛶ Wide',
      title: 'Toggle a wide editor (more room for code)',
      onclick: () => pane.closest('#pendant')?.classList.toggle('wide'),
    });

    this.editor = h('textarea.code-editor', { spellcheck: 'false' });
    const saved = this._loadFromStorage();
    this.editor.value = saved || EXAMPLES[0].code;
    this.editor.addEventListener('input', () => {
      this._clearPending();
      this._saveToStorage();
    });
    this.editor.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        this._onRun();
      } else if (e.key === 'Tab') {
        e.preventDefault();
        const { selectionStart: s, selectionEnd: t, value } = this.editor;
        this.editor.value = value.slice(0, s) + '    ' + value.slice(t);
        this.editor.selectionStart = this.editor.selectionEnd = s + 4;
        this._saveToStorage();
      }
    });

    this.banner = h('div.code-banner.hidden');
    this.consoleEl = h('div.code-console');

    pane.append(
      h('h3', { text: 'Code lab — Python (ur_rtde), Grasshopper & Robots (.script)' }),
      h('div.code-toolbar', {},
        this.runBtn, this.stopBtn, this.examples,
        loadBtn, this.watchBtn, ghNodeBtn, savePyBtn, clearBtn, wideBtn,
        this.file, this.status,
      ),
      this.editor,
      this.banner,
      this.consoleEl,
    );
    this.log('Run ur_rtde Python or Grasshopper (Robots plugin .script / .json / .csv) on the simulated UR10e.', 'sys');
    this.log('Grasshopper workflow: click "⬇ GH Node", drop it onto your GH canvas, wire Create Program → Code, then click "🔄 Live Sync".', 'sys');
  }

  _loadFromStorage() {
    try { return localStorage.getItem(STORAGE_KEY); } catch { return null; }
  }

  _saveToStorage() {
    try { localStorage.setItem(STORAGE_KEY, this.editor.value); } catch { /* ignore */ }
  }

  _downloadGHNode() {
    const a = document.createElement('a');
    a.href = `${import.meta.env.BASE_URL}UR10e_Sim_Sync.ghx`;
    a.download = 'UR10e_Sim_Sync.ghx';
    document.body.appendChild(a);
    a.click();
    a.remove();
    this.log('Downloaded "UR10e_Sim_Sync.ghx"!', 'sys');
    this.log('1) Drag UR10e_Sim_Sync.ghx directly onto your open Grasshopper canvas.', 'sys');
    this.log('2) Connect your Robots plugin "Create Program → Code" (or Planes) to the node.', 'sys');
    this.log('3) Click "🔄 Live Sync" above and select "ur10e_sim_live.script" on your Desktop.', 'sys');
  }

  _downloadEditorCode() {
    const isScript = isURScript(this.editor.value);
    const ext = isScript ? 'script' : 'py';
    const blob = new Blob([this.editor.value], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `ur10e_program.${ext}`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    this.log(`Saved editor contents as ur10e_program.${ext}`, 'sys');
  }

  async _toggleWatch() {
    if (this._watchHandle) {
      this._stopWatch();
      this.log('Live Sync stopped.', 'sys');
      return;
    }
    if (!window.showOpenFilePicker) {
      this.log('Live Sync uses the File System Access API (supported in Chrome, Edge, or Brave). Use "Load file…" in this browser.', 'err');
      return;
    }
    try {
      const [handle] = await window.showOpenFilePicker({
        id: 'ur10e-gh-sync',
        types: [{
          description: 'Robot Programs & Toolpaths (.script, .json, .csv, .py)',
          accept: { 'text/plain': ['.script', '.urp', '.json', '.csv', '.py', '.ghx', '.txt'] },
        }],
      });
      this._watchHandle = handle;
      this._watchLastMod = 0;
      this.watchBtn.classList.add('attention');
      this.watchBtn.textContent = '⏹ Syncing…';
      this.log(`Live Sync active on "${handle.name}" — move any slider in Grasshopper to update automatically!`, 'sys');
      await this._pollWatchedFile(true);
    } catch (err) {
      if (err?.name !== 'AbortError') {
        this.log(`Could not start Live Sync: ${err.message}`, 'err');
      }
    }
  }

  _stopWatch() {
    this._watchHandle = null;
    this._watchLastMod = 0;
    this.watchBtn.classList.remove('attention');
    this.watchBtn.textContent = '🔄 Live Sync';
  }

  async _pollWatchedFile(initial = false) {
    if (!this._watchHandle) return;
    try {
      const file = await this._watchHandle.getFile();
      if (file.lastModified <= this._watchLastMod) return;
      this._watchLastMod = file.lastModified;
      const text = await file.text();
      if (!text.trim()) return;

      const events = parseProgramFile(file.name, text, this.bridge);
      this.pending = { name: file.name, events };
      this.trajectory?.showPlannedEvents(events, this.bridge);
      this.banner.classList.remove('hidden');
      this.banner.replaceChildren(
        h('span', { text: `🔄 Live Sync: "${file.name}" (${events.length} events)` }),
        h('button.btn.small', { text: 'Stop sync', onclick: () => this._stopWatch() }),
      );
      this.log(`${initial ? 'Loaded' : '🔄 Synced update from'} ${file.name} (${events.length} motion events)`, 'sys');
      if (this.motion.state === 'RUNNING') {
        this._play(events);
      }
    } catch (err) {
      this.log(`Live Sync (${this._watchHandle?.name}): ${err.message}`, 'err');
    }
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
    const lower = f.name.toLowerCase();
    if (lower.endsWith('.py') || (!lower.match(/\.(json|csv|script|urp|ghx|gh)$/i) && text.includes('import') && !isURScript(text))) {
      this._clearPending();
      this.editor.value = text;
      this._saveToStorage();
      this.log(`loaded ${f.name} into the editor — press Run`, 'sys');
      return;
    }
    try {
      const events = parseProgramFile(f.name, text, this.bridge);
      this.pending = { name: f.name, events };
      this.trajectory?.showPlannedEvents(events, this.bridge);
      for (const n of this.bridge.notes ?? []) this.log(n, 'sys');
      this.banner.classList.remove('hidden');
      this.banner.replaceChildren(
        h('span', { text: `Toolpath "${f.name}" loaded — ${events.length} motion events. Run plays it.` }),
        h('button.btn.small', { text: '✕', title: 'Discard toolpath', onclick: () => this._clearPending() }),
      );
      this.log(`parsed ${f.name}: ${events.length} motion events (3D path preview shown) — press Run`, 'sys');
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
      this.trajectory?.showPlannedEvents(this.pending.events, this.bridge);
      this._play(this.pending.events);
      return;
    }

    // If the student pasted URScript (from Grasshopper's Robots plugin) directly
    // into the editor, parse & play it directly without needing Pyodide!
    if (isURScript(this.editor.value)) {
      try {
        const events = parseProgramFile('editor.script', this.editor.value, this.bridge);
        for (const n of this.bridge.notes ?? []) this.log(n, 'sys');
        this.trajectory?.showPlannedEvents(events, this.bridge);
        this.log(`parsed URScript from editor (${events.length} motion events)`, 'sys');
        this._play(events);
      } catch (err) {
        this.log(`URScript error: ${err.message}`, 'err');
        this._setStatus('error');
      }
      return;
    }

    this.busy = true;
    this.runBtn.disabled = true;
    try {
      if (!this.py.ready) {
        await this.py.ensure((s) => { this._setStatus(s); this.log(s, 'sys'); });
      }
      // gripper scripts need the 2FG7 mounted BEFORE recording so the TCP
      // (and every pose the script reads) already includes the tool length
      if (this.manager && !this.manager.gripper && /TwoFG7|from\s+onrobot|import\s+onrobot/.test(this.editor.value)) {
        this.manager.add('gripper2f');
        this.log('mounted the OnRobot 2FG7 gripper automatically', 'sys');
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
      this.trajectory?.showPlannedEvents(events, this.bridge);
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
    this.trajectory?.clearTrail();
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
    if (this._watchHandle) {
      this._watchTimer += dt;
      if (this._watchTimer >= 0.4) {
        this._watchTimer = 0;
        this._pollWatchedFile(false);
      }
    }
  }
}
