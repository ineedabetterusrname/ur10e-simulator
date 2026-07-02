import * as THREE from 'three';

const std = (color, opts = {}) =>
  new THREE.MeshStandardMaterial({ color, metalness: 0.4, roughness: 0.5, ...opts });

function box(w, h, d, mat) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.castShadow = m.receiveShadow = true;
  return m;
}
function cylZ(r, h, mat, rTop = r) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(rTop, r, h, 32), mat);
  m.rotation.x = Math.PI / 2;
  m.castShadow = m.receiveShadow = true;
  return m;
}
const at = (m, x, y, z) => { m.position.set(x, y, z); return m; };

/**
 * Part catalogue. Each part's build() returns:
 *   group      — mesh subtree (tool parts are built in flange frame, +Z = out)
 *   tcpZ       — TCP distance past the mount face (end effectors only)
 *   inlineLen  — stack-up length for inline parts (F/T sensor)
 *   capsule    — {len, r} collision capsule contribution along tool Z
 *   controls   — pendant control specs shown in the Tool tab
 *   update(dt, ctx) — per-frame behaviour
 * Categories: endEffector (exclusive), inline (between flange and tool),
 * addon (camera/sensors, stackable), base (linear track).
 */
export const PARTS = [
  // ---------------- end effectors ----------------
  {
    id: 'gripper2f',
    name: '2-Finger Gripper',
    category: 'endEffector',
    desc: 'Parallel electric gripper, 46 mm stroke. Adds open/close controls.',
    mass: 0.9,
    build() {
      const group = new THREE.Group();
      const bodyM = std(0x2b2e33);
      const accent = std(0x1794d1);
      group.add(at(cylZ(0.038, 0.02, accent), 0, 0, 0.01));
      group.add(at(box(0.09, 0.05, 0.07, bodyM), 0, 0, 0.055));
      const fingerM = std(0x8f959c, { metalness: 0.7, roughness: 0.3 });
      const f1 = at(box(0.014, 0.03, 0.075, fingerM), 0.03, 0, 0.125);
      const f2 = at(box(0.014, 0.03, 0.075, fingerM), -0.03, 0, 0.125);
      group.add(f1, f2);
      const state = { open: 1, targetOpen: 1 };
      return {
        group,
        tcpZ: 0.165,
        capsule: { len: 0.17, r: 0.055 },
        state,
        controls: [
          { type: 'slider', label: 'Grip opening', min: 0, max: 100, step: 1,
            get: () => Math.round(state.targetOpen * 100),
            set: (v) => { state.targetOpen = v / 100; },
            fmt: (v) => `${v}%` },
          { type: 'button', label: 'Open', onClick: () => { state.targetOpen = 1; } },
          { type: 'button', label: 'Close', onClick: () => { state.targetOpen = 0; } },
          { type: 'readout', label: 'Opening', get: () => `${(4 + state.open * 46).toFixed(0)} mm` },
        ],
        update(dt) {
          state.open += Math.max(-dt * 1.6, Math.min(dt * 1.6, state.targetOpen - state.open));
          const x = 0.007 + state.open * 0.023;
          f1.position.x = x;
          f2.position.x = -x;
        },
      };
    },
  },
  {
    id: 'vacuum',
    name: 'Vacuum Gripper',
    category: 'endEffector',
    desc: 'Single-cup vacuum gripper with on/off suction and status lamp.',
    mass: 0.75,
    build() {
      const group = new THREE.Group();
      group.add(at(cylZ(0.038, 0.05, std(0x2b2e33)), 0, 0, 0.025));
      for (let i = 0; i < 3; i++) {
        group.add(at(cylZ(0.03 - i * 0.004, 0.014, std(0x17181b, { roughness: 0.9 })), 0, 0, 0.06 + i * 0.016));
      }
      group.add(at(cylZ(0.02, 0.02, std(0x0d0e10, { roughness: 0.95 }), 0.034), 0, 0, 0.108));
      const lampM = new THREE.MeshStandardMaterial({ color: 0x1d3324, emissive: 0x000000 });
      group.add(at(new THREE.Mesh(new THREE.SphereGeometry(0.008, 16, 12), lampM), 0.033, 0, 0.02));
      const state = { on: false };
      return {
        group,
        tcpZ: 0.12,
        capsule: { len: 0.125, r: 0.045 },
        state,
        controls: [
          { type: 'toggle', label: 'Vacuum', get: () => state.on,
            set: (v) => { state.on = v; lampM.emissive.setHex(v ? 0x22cc66 : 0x000000); } },
          { type: 'readout', label: 'Pressure', get: () => (state.on ? '-75 kPa' : '0 kPa') },
        ],
        update() {},
      };
    },
  },
  {
    id: 'torch',
    name: 'Welding Torch',
    category: 'endEffector',
    desc: 'MIG torch with 25° neck. Arc toggle with flicker glow.',
    mass: 1.2,
    build() {
      const group = new THREE.Group();
      group.add(at(cylZ(0.036, 0.04, std(0x2b2e33)), 0, 0, 0.02));
      const neck = new THREE.Group();
      neck.position.set(0, 0, 0.04);
      neck.rotation.x = -0.44; // ~25 degree bend
      group.add(neck);
      neck.add(at(cylZ(0.014, 0.14, std(0xb8860b, { metalness: 0.8, roughness: 0.35 })), 0, 0, 0.07));
      const tipM = new THREE.MeshStandardMaterial({ color: 0x9a5b0a, emissive: 0x000000 });
      neck.add(at(cylZ(0.01, 0.03, tipM, 0.006), 0, 0, 0.155));
      const light = new THREE.PointLight(0x88bbff, 0, 0.6);
      light.position.set(0, 0, 0.17);
      neck.add(light);
      const state = { arc: false, t: 0 };
      return {
        group,
        tcpZ: 0.2, // approximate wire tip along the bent neck
        capsule: { len: 0.21, r: 0.05 },
        state,
        controls: [
          { type: 'toggle', label: 'Arc', get: () => state.arc, set: (v) => { state.arc = v; } },
          { type: 'readout', label: 'Wire feed', get: () => (state.arc ? '8.2 m/min' : '—') },
        ],
        update(dt) {
          state.t += dt;
          const glow = state.arc ? 0.7 + 0.3 * Math.sin(state.t * 37) : 0;
          tipM.emissive.setRGB(glow, glow * 0.7, glow * 1.2);
          light.intensity = state.arc ? 2 + Math.random() * 2 : 0;
        },
      };
    },
  },
  // ---------------- inline ----------------
  {
    id: 'ftsensor',
    name: 'Force-Torque Sensor',
    category: 'inline',
    desc: 'Mounts between flange and tool (+40 mm stack-up). Live wrench readout.',
    mass: 0.3,
    build() {
      const group = new THREE.Group();
      group.add(at(cylZ(0.0455, 0.036, std(0x4a5057, { metalness: 0.6, roughness: 0.35 })), 0, 0, 0.02));
      group.add(at(cylZ(0.047, 0.008, std(0x1794d1)), 0, 0, 0.02));
      const state = { wrench: [0, 0, 0, 0, 0, 0] };
      const q = new THREE.Quaternion();
      const f = new THREE.Vector3();
      const r = new THREE.Vector3();
      const t = new THREE.Vector3();
      return {
        group,
        inlineLen: 0.04,
        capsule: { len: 0.04, r: 0.05 },
        state,
        controls: [
          { type: 'readout', label: 'Force [N]', get: () => state.wrench.slice(0, 3).map((v) => v.toFixed(1)).join('  ') },
          { type: 'readout', label: 'Torque [Nm]', get: () => state.wrench.slice(3).map((v) => v.toFixed(2)).join('  ') },
        ],
        update(dt, ctx) {
          // static gravity wrench of everything mounted past the sensor,
          // expressed in the sensor frame
          group.getWorldQuaternion(q);
          f.set(0, -9.81 * ctx.massBeyondFT, 0).applyQuaternion(q.invert());
          r.set(0, 0, ctx.comBeyondFT);
          t.crossVectors(r, f);
          state.wrench = [f.x, f.y, f.z, t.x, t.y, t.z];
        },
      };
    },
  },
  // ---------------- add-ons ----------------
  {
    id: 'wristcam',
    name: 'Wrist Camera',
    category: 'addon',
    desc: 'Camera on a flange-side bracket. Adds a live picture-in-picture view.',
    mass: 0.3,
    build() {
      const group = new THREE.Group();
      group.position.set(0.075, 0, 0.028);
      group.add(at(box(0.012, 0.02, 0.05, std(0x33373d)), -0.022, 0, 0));
      group.add(at(box(0.03, 0.034, 0.056, std(0x17181b)), 0, 0, 0));
      const lens = at(cylZ(0.011, 0.014, std(0x0a0c10, { metalness: 0.2, roughness: 0.1 })), 0, 0, 0.033);
      group.add(lens);
      group.add(at(cylZ(0.013, 0.004, std(0x1794d1)), 0, 0, 0.028));
      const camera = new THREE.PerspectiveCamera(55, 16 / 9, 0.03, 12);
      // three.js cameras look down -Z; pitch 180° so it looks along the tool's
      // +Z axis with the image upright (a 180° yaw would render it upside down)
      camera.rotation.x = Math.PI;
      group.add(camera);
      const state = { pip: true };
      return {
        group,
        camera,
        state,
        controls: [
          { type: 'toggle', label: 'Camera view', get: () => state.pip, set: (v) => { state.pip = v; } },
          { type: 'slider', label: 'Field of view', min: 30, max: 90, step: 1,
            get: () => camera.fov,
            set: (v) => { camera.fov = v; camera.updateProjectionMatrix(); },
            fmt: (v) => `${v}°` },
        ],
        update() {},
      };
    },
  },
  {
    id: 'proxsensor',
    name: 'Proximity Sensor (ToF)',
    category: 'addon',
    desc: 'Time-of-flight distance sensor beside the flange, range 1000 mm.',
    mass: 0.05,
    build() {
      const group = new THREE.Group();
      group.position.set(-0.062, 0, 0.024);
      group.add(at(cylZ(0.009, 0.045, std(0xc7cbd1, { metalness: 0.8, roughness: 0.3 })), 0, 0, 0));
      group.add(at(cylZ(0.0095, 0.006, std(0xcc4422)), 0, 0, 0.017));
      const state = { mm: null };
      const origin = new THREE.Vector3();
      const dir = new THREE.Vector3();
      return {
        group,
        state,
        controls: [
          { type: 'readout', label: 'Distance', get: () => (state.mm == null ? 'out of range' : `${state.mm.toFixed(0)} mm`) },
        ],
        update() {
          // analytic ray vs floor plane (y = 0), along the sensor's +Z
          group.getWorldPosition(origin);
          group.getWorldDirection(dir); // +Z in world
          let d = null;
          if (dir.y < -1e-4) {
            const t = origin.y / -dir.y;
            if (t >= 0 && t <= 1.0) d = t * 1000;
          }
          state.mm = d;
        },
      };
    },
  },
  // ---------------- base ----------------
  {
    id: 'track',
    name: 'Linear Base Track (2 m)',
    category: 'base',
    desc: 'Floor rail with a motorised carriage. Adds a 7th jog axis (E1) to the pendant.',
    mass: 0,
    build() {
      const group = new THREE.Group(); // world-space, added to the scene
      const railM = std(0x3a3f46, { metalness: 0.55, roughness: 0.45 });
      const rail = at(box(2.1, 0.09, 0.3, railM), 0, 0.045, 0);
      group.add(rail);
      group.add(at(box(2.1, 0.012, 0.05, std(0x8f959c, { metalness: 0.8, roughness: 0.25 })), 0, 0.096, 0.09));
      group.add(at(box(2.1, 0.012, 0.05, std(0x8f959c, { metalness: 0.8, roughness: 0.25 })), 0, 0.096, -0.09));
      group.add(at(box(0.06, 0.14, 0.34, std(0xd7a20a)), 1.06, 0.07, 0));
      group.add(at(box(0.06, 0.14, 0.34, std(0xd7a20a)), -1.06, 0.07, 0));
      const carriage = new THREE.Group();
      carriage.position.y = 0.102;
      carriage.add(at(box(0.36, 0.03, 0.3, std(0x22262b)), 0, 0.015, 0));
      group.add(carriage);
      return {
        group,
        carriage,
        carriageTopY: 0.132, // robot base sits here
        axis: { name: 'Track E1', unit: 'm', min: -0.85, max: 0.85, vmax: 0.6, amax: 1.0 },
        // world-space obstacle capsule approximating the rail body
        obstacle: {
          name: 'track rail',
          p1: new THREE.Vector3(-1.05, 0.05, 0),
          p2: new THREE.Vector3(1.05, 0.05, 0),
          r: 0.12,
          exempt: new Set(['base', 'shoulder']),
        },
        controls: [
          { type: 'jog', label: 'Carriage E1', axisName: 'Track E1' },
        ],
        update() {},
      };
    },
  },
];

export const partById = (id) => PARTS.find((p) => p.id === id);
