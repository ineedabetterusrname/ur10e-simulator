import { RTDE_CONTROL_PY, RTDE_RECEIVE_PY, DASHBOARD_PY, ONROBOT_PY, PRELUDE_PY } from './pymocks.js';

const PYODIDE_BASE = 'https://cdn.jsdelivr.net/pyodide/v314.0.2/full/';

/**
 * PyRunner — runs student Python in the browser via Pyodide (CPython/WASM,
 * lazy-loaded from CDN on first Run) with the simulator's ur_rtde mocks
 * installed, so the same script that drives the real UR10e over RTDE runs
 * unmodified against the SimBridge.
 *
 * `importPyodide`/`indexURL` can be overridden to run against the npm
 * pyodide package instead of the CDN (used by scripts/pycheck.mjs).
 */
export class PyRunner {
  constructor(bridge, { importPyodide, indexURL = PYODIDE_BASE } = {}) {
    this.bridge = bridge;
    this.pyodide = null;
    this._loading = null;
    this._indexURL = indexURL;
    this._import = importPyodide
      ?? (() => import(/* @vite-ignore */ `${PYODIDE_BASE}pyodide.mjs`));
  }

  get ready() { return !!this.pyodide; }

  /** Loads and configures Pyodide once; safe to call repeatedly. */
  ensure(onStatus) {
    this._loading ??= this._load(onStatus).catch((err) => {
      this._loading = null; // allow retry (e.g. after a network hiccup)
      throw err;
    });
    return this._loading;
  }

  async _load(onStatus) {
    onStatus?.('downloading Python runtime (~7 MB, first run only)…');
    const { loadPyodide } = await this._import();
    onStatus?.('starting Python…');
    const pyodide = await loadPyodide(this._indexURL ? { indexURL: this._indexURL } : {});

    // primitive-only bridge: joint/pose values are spread as plain numbers
    const b = this.bridge;
    pyodide.registerJsModule('_urbridge', {
      move_j: (q0, q1, q2, q3, q4, q5, v) => b.moveJ([q0, q1, q2, q3, q4, q5], v),
      move_l: (x, y, z, rx, ry, rz, v) => b.moveL([x, y, z, rx, ry, rz], v),
      servo_j: (q0, q1, q2, q3, q4, q5) => b.servoJ([q0, q1, q2, q3, q4, q5]),
      speed_l: (x, y, z, rx, ry, rz, t) => b.speedL([x, y, z, rx, ry, rz], t),
      speed_j: (q0, q1, q2, q3, q4, q5, t) => b.speedJ([q0, q1, q2, q3, q4, q5], t),
      speed_stop: () => b.speedStop(),
      grip_move: (mm) => b.gripMove(mm),
      grip_width: () => b.gripWidth(),
      sleep: (s) => b.sleep(s),
      get_q: () => b.getQ(),
      get_tcp: () => b.getTCP(),
      fk: (q0, q1, q2, q3, q4, q5) => b.fk([q0, q1, q2, q3, q4, q5]),
      ik: (x, y, z, rx, ry, rz) => b.ik([x, y, z, rx, ry, rz]),
      timestamp: () => b.timestamp(),
      note: (t) => b.note(String(t)),
    });

    pyodide.FS.writeFile('rtde_control.py', RTDE_CONTROL_PY);
    pyodide.FS.writeFile('rtde_receive.py', RTDE_RECEIVE_PY);
    pyodide.FS.writeFile('dashboard_client.py', DASHBOARD_PY);
    pyodide.FS.writeFile('onrobot.py', ONROBOT_PY);
    pyodide.runPython(PRELUDE_PY);

    this.pyodide = pyodide;
    onStatus?.('Python ready');
  }

  /**
   * Executes a script in a fresh namespace. stdout/stderr lines stream to
   * onOutput. Resolves { ok, error } — motion events land in the bridge.
   */
  async run(source, onOutput) {
    const py = this.pyodide;
    warnDesktopOnlyImports(source, onOutput);
    py.setStdout({ batched: (line) => onOutput?.(line) });
    py.setStderr({ batched: (line) => onOutput?.(line, 'err') });
    // scripts must behave like `python script.py` — the ubiquitous
    // `if __name__ == "__main__":` guard has to be truthy
    const ns = py.toPy({ __name__: '__main__' });
    try {
      await py.loadPackagesFromImports(source); // e.g. numpy, on demand
      await py.runPythonAsync(source, { globals: ns });
      return { ok: true };
    } catch (err) {
      return { ok: false, error: tidyTraceback(err) };
    } finally {
      ns.destroy();
      py.setStdout();
      py.setStderr();
    }
  }
}

/** Libraries that need a desktop machine (cameras, GUIs, physics, hardware). */
const DESKTOP_ONLY = {
  cv2: 'OpenCV needs a camera and a desktop Python',
  mediapipe: 'Mediapipe needs a camera and a desktop Python',
  pybullet: 'PyBullet is a desktop physics engine — this simulator replaces it here',
  serial: 'pyserial talks to physical hardware',
  rospy: 'ROS runs on a robot PC',
  tkinter: 'tkinter opens desktop windows',
};

function warnDesktopOnlyImports(source, onOutput) {
  for (const m of source.matchAll(/^[ \t]*(?:import|from)[ \t]+([A-Za-z_]\w*)/gm)) {
    const why = DESKTOP_ONLY[m[1]];
    if (why) {
      onOutput?.(
        `⚠ "${m[1]}" cannot run in the browser (${why}). ` +
        'Remove or guard that part — the ur_rtde robot calls themselves are fully simulated.', 'err');
    }
  }
}

/** Trims Pyodide's internal frames off a Python traceback. */
function tidyTraceback(err) {
  const msg = String(err?.message ?? err);
  const lines = msg.split('\n');
  const start = lines.findIndex((l) => l.includes('File "<exec>"'));
  if (start > 0) return ['Traceback (most recent call last):', ...lines.slice(start)].join('\n');
  return msg;
}
