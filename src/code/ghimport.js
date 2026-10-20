/**
 * Grasshopper & URScript toolpath import: converts an exported program file
 * into simulator events by replaying it through the SimBridge (so pose targets
 * get the same IK + straight-line subdivision as Python moveL calls).
 *
 * Supported formats:
 * 1. URScript (.script, .txt, or pasted from the Grasshopper "Robots" plugin's
 *    CreateProgram -> Code output):
 *      set_tcp(p[x, y, z, rx, ry, rz])
 *      movej([j1, j2, j3, j4, j5, j6], a=..., v=...)
 *      movej(p[x, y, z, rx, ry, rz], a=..., v=...)
 *      movel(p[x, y, z, rx, ry, rz], a=..., v=...)
 *      movep(p[x, y, z, rx, ry, rz], a=..., v=...)
 *      sleep(t)
 *      twofg_grip_external(..., width, ...)
 *
 * 2. Grasshopper XML (.ghx): extracts internalised URScript from Panels or
 *    internalised Planes from Plane parameters.
 *
 * 3. JSON format (see docs/grasshopper-export.md):
 *      { "units": "deg" | "rad", "moves": [ ... ] }
 *
 * 4. CSV format: one move per line, 6 joint values in degrees with an optional
 *    7th speed column (deg/s).
 */

import * as THREE from 'three';

const D2R = Math.PI / 180;

/**
 * Returns true if a string looks like a URScript program (e.g. output from the
 * Grasshopper "Robots" plugin's Create Program -> Code).
 */
export function isURScript(text) {
  return /\b(?:move[jlp]|set_tcp|speed[jl]|servoj)\s*\(/.test(text) &&
    !/\brtde_control\b/.test(text);
}

export function parseProgramFile(name, text, bridge) {
  const lower = (name || '').toLowerCase();
  if (lower.endsWith('.gh')) {
    throw new Error(
      'Binary .gh files only store the node graph recipe, not evaluated planes. ' +
      'Use the "⬇ GH Node" button in the toolbar to drop our sync node into Grasshopper, ' +
      'or upload the .script file from the Robots plugin\'s Save Program component.'
    );
  }
  bridge.begin();
  try {
    if (lower.endsWith('.ghx') || text.trimStart().startsWith('<?xml')) {
      parseGHX(text, bridge);
    } else if (lower.endsWith('.json') || text.trimStart().startsWith('{') || text.trimStart().startsWith('[')) {
      parseJSON(text, bridge);
    } else if (lower.endsWith('.script') || lower.endsWith('.urp') || isURScript(text)) {
      parseURScript(text, bridge);
    } else {
      parseCSV(text, bridge);
    }
    return bridge.end();
  } catch (err) {
    bridge.end();
    throw err;
  }
}

/**
 * Parses Universal Robots URScript — specifically supporting the output of the
 * Grasshopper "Robots" plugin (by Vicente Soler) as well as standard Polyscope
 * .script files.
 */
function parseURScript(text, bridge) {
  const lines = text.split(/\r?\n/);
  let count = 0;

  // Track simple variable assignments like `q0 = [0, -1.57, ...]` or `p0 = p[...]`
  const vars = new Map();

  for (let li = 0; li < lines.length; li++) {
    // Strip comments
    const raw = lines[li].replace(/#.*$/, '').trim();
    if (!raw) continue;

    // Variable assignment: name = p[...] or [...]
    const assignMatch = raw.match(/^([A-Za-z_]\w*)\s*=\s*(p?\s*\[[^\]]+\])\s*$/);
    if (assignMatch) {
      vars.set(assignMatch[1], assignMatch[2]);
      continue;
    }

    // set_tcp(p[x, y, z, rx, ry, rz]) — record note if non-zero Z offset
    const tcpMatch = raw.match(/\bset_tcp\s*\(\s*p\s*\[\s*([^\]]+)\]\s*\)/);
    if (tcpMatch) {
      const nums = tcpMatch[1].split(',').map((s) => Number(s.trim()));
      if (nums.length >= 3 && nums.every(Number.isFinite)) {
        const tcpZ = Math.hypot(nums[0], nums[1], nums[2]);
        bridge.note(`URScript set_tcp: offset ${(tcpZ * 1000).toFixed(1)} mm`);
      }
      continue;
    }

    // sleep(seconds)
    const sleepMatch = raw.match(/\bsleep\s*\(\s*([0-9.+eE-]+)\s*\)/);
    if (sleepMatch) {
      const s = Number(sleepMatch[1]);
      if (Number.isFinite(s) && s > 0) {
        bridge.sleep(s);
        count++;
      }
      continue;
    }

    // OnRobot 2FG7 gripper commands in URScript (twofg_grip_external / twofg_grip / grip_move)
    const gripMatch = raw.match(/\b(?:twofg_grip(?:_external|_internal)?|grip_move)\s*\(\s*(?:\d+\s*,\s*)?([0-9.+eE-]+)/);
    if (gripMatch) {
      const w = Number(gripMatch[1]);
      if (Number.isFinite(w)) {
        bridge.gripMove(w);
        count++;
      }
      continue;
    }

    // movej / movel / movep / servoj
    const moveMatch = raw.match(/\b(movej|movel|movep|servoj)\s*\(\s*(.+)\)\s*$/);
    if (!moveMatch) continue;

    const cmd = moveMatch[1];
    let argsStr = moveMatch[2];

    // Resolve variable reference if first argument is an identifier
    const firstIdent = argsStr.match(/^([A-Za-z_]\w*)(?=\s*(?:,|$))/);
    if (firstIdent && vars.has(firstIdent[1])) {
      argsStr = vars.get(firstIdent[1]) + argsStr.slice(firstIdent[0].length);
    }

    // Check if target is a pose `p[x, y, z, rx, ry, rz]` or joint list `[j1..j6]`
    const targetMatch = argsStr.match(/^(p)?\s*\[\s*([^\]]+)\](.*)$/);
    if (!targetMatch) continue;

    const isPose = targetMatch[1] === 'p';
    const vals = targetMatch[2].split(',').map((s) => Number(s.trim()));
    if (vals.length < 6 || vals.slice(0, 6).some((v) => !Number.isFinite(v))) {
      throw new Error(`line ${li + 1}: expected 6 numbers in ${cmd} target`);
    }
    const vec = vals.slice(0, 6);
    const rest = targetMatch[3] || '';

    // Extract speed `v=...` or positional speed argument
    let speed = null;
    const namedV = rest.match(/\bv\s*=\s*([0-9.+eE-]+)/);
    if (namedV) {
      speed = Number(namedV[1]);
    } else {
      // Positional arguments after `], a, v, ...`
      const posArgs = rest.split(',').map((s) => s.trim()).filter(Boolean);
      // In URScript: movej(q, a=1.4, v=1.05, t=0, r=0) -> posArgs[0] is a, posArgs[1] is v
      if (posArgs.length >= 2 && !posArgs[1].includes('=')) {
        const vNum = Number(posArgs[1]);
        if (Number.isFinite(vNum) && vNum > 0) speed = vNum;
      }
    }

    if (cmd === 'movej' || cmd === 'servoj') {
      if (isPose) {
        const q = bridge.ik(vec);
        bridge.moveJ(q, speed ?? 1.05);
      } else {
        bridge.moveJ(vec, speed ?? 1.05);
      }
      count++;
    } else if (cmd === 'movel' || cmd === 'movep') {
      if (isPose) {
        bridge.moveL(vec, speed ?? 0.25);
      } else {
        const pose = bridge.fk(vec);
        bridge.moveL(pose, speed ?? 0.25);
      }
      count++;
    }
  }

  if (count === 0) {
    throw new Error('no URScript motion commands (movej / movel / movep) found in file');
  }
}

/**
 * Extracts internalised URScript text or internalised GH_Plane targets from a
 * Grasshopper XML (.ghx) archive.
 */
function parseGHX(xmlText, bridge) {
  // 1. Check if any Panel or string item contains URScript commands
  const scriptBlock = xmlText.match(/(?:move[jlp]|set_tcp)\s*\([\s\S]*?(?=<\/item>|<\/string>)/);
  if (scriptBlock) {
    const decoded = scriptBlock[0]
      .replace(/&amp;/g, '&')
      .replace(/&lt;/g, '<')
      .replace(/&gt;/g, '>')
      .replace(/&quot;/g, '"')
      .replace(/&#xD;/g, '\r')
      .replace(/&#xA;/g, '\n');
    parseURScript(decoded, bridge);
    return;
  }

  // 2. Look for internalised GH_Plane nodes (<item name="Origin"> + XAxis + YAxis)
  const planeRegex = /<chunk name="Plane"[\s\S]*?<\/chunk>/g;
  const planeChunks = xmlText.match(planeRegex) || [];
  const poses = [];
  for (const chunk of planeChunks) {
    const ox = matchXmlNum(chunk, 'Origin', 'X');
    const oy = matchXmlNum(chunk, 'Origin', 'Y');
    const oz = matchXmlNum(chunk, 'Origin', 'Z');
    const xx = matchXmlNum(chunk, 'XAxis', 'X');
    const xy = matchXmlNum(chunk, 'XAxis', 'Y');
    const xz = matchXmlNum(chunk, 'XAxis', 'Z');
    const yx = matchXmlNum(chunk, 'YAxis', 'X');
    const yy = matchXmlNum(chunk, 'YAxis', 'Y');
    const yz = matchXmlNum(chunk, 'YAxis', 'Z');
    if ([ox, oy, oz, xx, xy, xz, yx, yy, yz].some((v) => v === null)) continue;
    // Skip default world XY origin planes from empty parameters
    if (ox === 0 && oy === 0 && oz === 0 && xx === 1 && yy === 1) continue;

    const xAxis = new THREE.Vector3(xx, xy, xz).normalize();
    const yAxis = new THREE.Vector3(yx, yy, yz).normalize();
    const zAxis = new THREE.Vector3().crossVectors(xAxis, yAxis).normalize();
    yAxis.crossVectors(zAxis, xAxis).normalize();

    const m = new THREE.Matrix4().makeBasis(xAxis, yAxis, zAxis);
    const q = new THREE.Quaternion().setFromRotationMatrix(m);
    const w = q.w < 0 ? -q.w : q.w;
    const s = q.w < 0 ? -1 : 1;
    const angle = 2 * Math.acos(Math.min(1, w));
    const den = Math.sqrt(Math.max(0, 1 - w * w));
    const rv = den < 1e-9 ? [0, 0, 0] : [(q.x * angle / den) * s, (q.y * angle / den) * s, (q.z * angle / den) * s];

    let px = ox, py = oy, pz = oz;
    if (Math.max(Math.abs(px), Math.abs(py), Math.abs(pz)) > 10) {
      px /= 1000; py /= 1000; pz /= 1000;
    }
    poses.push([px, py, pz, ...rv]);
  }

  if (poses.length > 0) {
    for (const pose of poses) bridge.moveL(pose, 0.25);
    return;
  }

  throw new Error(
    'This .ghx file has no internalised URScript or Planes. ' +
    'Click "⬇ GH Node" in the toolbar to drop our sync node into Grasshopper, ' +
    'or export a .script file from the Robots plugin\'s Save Program component.'
  );
}

function matchXmlNum(chunk, section, coord) {
  const sec = chunk.match(new RegExp(`<item name="${section}"[^>]*>[\\s\\S]*?<\\/${section}|<chunk name="${section}"[\\s\\S]*?<\\/chunk>`));
  const scope = sec ? sec[0] : chunk;
  const m = scope.match(new RegExp(`<item name="${coord}"[^>]*>([0-9.+eE-]+)<\\/item>`));
  return m ? Number(m[1]) : null;
}

function parseJSON(text, bridge) {
  const doc = JSON.parse(text);
  const moves = Array.isArray(doc) ? doc : doc.moves;
  if (!Array.isArray(moves) || moves.length === 0) {
    throw new Error('no "moves" array found in the JSON file');
  }
  const jointScale = (Array.isArray(doc) ? 'deg' : (doc.units ?? 'deg')) === 'rad' ? 1 : D2R;

  moves.forEach((mv, idx) => {
    const type = mv.type ?? (mv.q ? 'joints' : mv.pose ? 'pose' : mv.grip != null ? 'grip' : null);
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
    } else if (type === 'grip') {
      bridge.gripMove(Number(mv.grip ?? mv.width ?? 73));
    } else if (type === 'sleep') {
      bridge.sleep(Number(mv.s ?? mv.time ?? 0.5));
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
