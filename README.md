# UR10e Simulator

![license](https://img.shields.io/badge/license-MIT-blue) ![three.js](https://img.shields.io/badge/built%20with-three.js-000000)

Interactive, user-controllable 3D simulation of a **Universal Robots UR10e** built with
[Three.js](https://threejs.org/) — real joint kinematics, motion physics, self-collision
protection, a teach-pendant UI, and a smart attachment catalogue.

**Live demo:** https://ineedabetterusrname.github.io/ur10e-simulator/

## Run it

```bash
npm install
npm run dev        # open the printed http://localhost:5173 URL
```

Other scripts:

```bash
npm run build      # production bundle -> dist/
npm run preview    # serve the production bundle
npm run smoke      # headless kinematics + collision test suite (no browser)
```

## What's simulated

### Kinematics — exact, not approximate
The joint chain is built from the **official UR10e DH parameters**
(d₁ 0.1807, a₂ −0.6127, a₃ −0.57155, d₄ 0.17415, d₅ 0.11985, d₆ 0.11655 m), so forward
kinematics match the real controller. The smoke test verifies the scene graph against the
analytic DH product and the published zero-pose flange position (−1184.25, −290.7, 60.85 mm).

### Motion physics
- Per-joint **velocity limits** (120 °/s base/shoulder, 180 °/s elbow/wrists) and
  acceleration-limited trapezoidal profiles — joints ramp up, cruise, and decelerate
  into targets without overshoot.
- **Joint limits** (±360°, elbow ±180°) with limit flagging on the pendant.
- Global **speed override** slider (1–100%), like the real pendant.

### Safety
- **Self-collision detection** via capsule sweeps over all non-adjacent link pairs,
  plus floor and (when mounted) track-rail collision.
- A predicted collision reverts to the last safe pose and latches a
  **PROTECTIVE STOP** — press *Reset* to resume, exactly like a real UR.
- Physical-style **emergency stop** button.

### Teach pendant (right panel)
| Tab | Contents |
|---|---|
| **Move** | Joint jog (hold −/+ or drag sliders) and Cartesian **TCP jog** in the base frame (X/Y/Z/RX/RY/RZ) driven by damped-least-squares IK; live TCP readout; Home pose |
| **Tool** | *Adaptive* — controls contributed by whatever is currently mounted |
| **Program** | Save waypoints, play once or loop, jump to any waypoint |
| **Status** | Joint positions/velocities, payload, machine state |

### Catalogue (left panel)
Click a card to mount/unmount. Parts apply **smartly**:

- **End effectors** *(exclusive — a new one replaces the current one)*
  - 2-Finger Gripper — open/close buttons + stroke slider
  - Vacuum Gripper — suction toggle with status lamp
  - Welding Torch — arc toggle with flicker glow
- **Inline** — Force-Torque Sensor mounts *between* flange and tool, shifts the whole
  tool stack by 40 mm, and streams a live gravity wrench computed from the mounted payload
- **Wrist add-ons** — Wrist Camera (live picture-in-picture view + FOV control) and a
  ToF Proximity Sensor (distance readout)
- **Base** — 2 m Linear Track: reparents the robot onto a motorised carriage and registers
  a 7th axis (**E1**) that appears in the pendant's jog list automatically

Every change recomputes the TCP offset (so Cartesian jog stays accurate), the tool
collision capsule, and the payload figure. The 6 base robot-arm controls never change.

## Architecture

```
src/
  main.js               bootstrap + fixed-order sim loop (jog→motion→FK→collision→UI)
  scene.js              renderer, lights, ground, orbit camera, PiP viewport
  robot/
    ur10e.js            DH-exact joint hierarchy, meshes, collision capsules, specs
    motion.js           per-axis trapezoidal motion, jog/targets, safety states, program
    kinematics.js       scene-graph-derived FK, numeric Jacobian, DLS IK
    collision.js        capsule-capsule / capsule-floor / static-obstacle checks
  catalogue/
    parts.js            part definitions (meshes, controls, behaviours)
    manager.js          smart mounting, TCP/capsule/payload recompute, control routing
  ui/
    pendant.js          teach pendant (tabs, jog, adaptive Tool tab)
    cataloguePanel.js   part cards
    statusbar.js        state / TCP / payload / fps strip
```

Kinematics are computed **from the scene graph itself** (numerically differentiated), so
the math can never drift from what you see on screen.

## Controls summary

- **Orbit / zoom / pan** — left-drag / wheel / right-drag on the 3D view
- **Jog a joint** — hold the − / + buttons, or drag its slider to a target
- **Move the TCP** — hold a Cartesian jog button (base frame)
- **E-stop** — red button, then *Reset* to recover

## License

[MIT](LICENSE)
