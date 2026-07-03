/**
 * Grasshopper / toolpath import: converts an exported program file into
 * simulator events by replaying it through the SimBridge (so pose targets
 * get the same IK + straight-line subdivision as Python moveL calls).
 *
 * JSON format (see docs/grasshopper-export.md, exported by the provided
 * GhPython snippet — no plugins required):
 *   {
 *     "units": "deg" | "rad",          // joint values (default deg)
 *     "moves": [
 *       { "type": "joints", "q": [j1..j6], "v": 60 },      // v in units/s
 *       { "type": "pose", "pose": [x,y,z,rx,ry,rz], "v": 0.25 }  // m + rotvec(rad), v in m/s
 *     ]
 *   }
 * A bare top-level array is treated as the moves list.
 *
 * CSV format: one move per line, 6 joint values in degrees with an optional
 * 7th speed column (deg/s). Lines starting with # and a header row are
 * skipped.
 */

const D2R = Math.PI / 180;

export function parseProgramFile(name, text, bridge) {
  const lower = name.toLowerCase();
  bridge.begin();
  try {
    if (lower.endsWith('.json')) parseJSON(text, bridge);
    else parseCSV(text, bridge);
    return bridge.end();
  } catch (err) {
    bridge.end();
    throw err;
  }
}

function parseJSON(text, bridge) {
  const doc = JSON.parse(text);
  const moves = Array.isArray(doc) ? doc : doc.moves;
  if (!Array.isArray(moves) || moves.length === 0) {
    throw new Error('no "moves" array found in the JSON file');
  }
  const jointScale = (Array.isArray(doc) ? 'deg' : (doc.units ?? 'deg')) === 'rad' ? 1 : D2R;

  moves.forEach((mv, idx) => {
    const type = mv.type ?? (mv.q ? 'joints' : mv.pose ? 'pose' : null);
    if (type === 'joints' || type === 'j') {
      const q = num6(mv.q, idx, 'q');
      bridge.moveJ(q.map((v) => v * jointScale), (mv.v ?? 60) * jointScale);
    } else if (type === 'pose' || type === 'p') {
      const pose = num6(mv.pose, idx, 'pose');
      // GH docs in millimetres are a classic slip — auto-detect and convert
      if (Math.max(...pose.slice(0, 3).map(Math.abs)) > 10) {
        pose[0] /= 1000; pose[1] /= 1000; pose[2] /= 1000;
      }
      bridge.moveL(pose, mv.v ?? 0.25);
    } else {
      throw new Error(`move ${idx + 1}: unknown type "${mv.type}" (use "joints" or "pose")`);
    }
  });
}

function parseCSV(text, bridge) {
  const rows = text.split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
  let count = 0;
  for (const [li, line] of rows.entries()) {
    const vals = line.split(/[,;\t]+/).map((s) => Number(s.trim()));
    if (vals.some(Number.isNaN)) {
      if (li === 0) continue; // header row
      throw new Error(`line ${li + 1}: "${line}" is not numeric`);
    }
    if (vals.length < 6) throw new Error(`line ${li + 1}: expected 6 joint values (deg), got ${vals.length}`);
    bridge.moveJ(vals.slice(0, 6).map((v) => v * D2R), (vals[6] ?? 60) * D2R);
    count++;
  }
  if (count === 0) throw new Error('no joint rows found in the CSV file');
}

function num6(arr, idx, what) {
  if (!Array.isArray(arr) || arr.length < 6 || arr.slice(0, 6).some((v) => typeof v !== 'number' || Number.isNaN(v))) {
    throw new Error(`move ${idx + 1}: "${what}" must be an array of 6 numbers`);
  }
  return arr.slice(0, 6);
}
