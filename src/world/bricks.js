import * as THREE from 'three';

/** Simulator bricks: 120 x 60 x 60 mm — graspable by the 2FG7 (35–73 mm). */
export const BRICK = { l: 0.12, w: 0.06, h: 0.06 };
const MAX_BRICKS = 12;
const GRAVITY = 9.81;

/**
 * BrickSystem — free-standing white cuboids the robot can manipulate.
 *
 * Interaction: drag a brick to move it on the floor plane (orbit controls
 * pause while dragging); the Tool tab's rotation slider turns the selected
 * brick (last added or last dragged).
 *
 * Physics (deliberately simple but believable):
 *  - released bricks fall under gravity and land on the floor or on top of
 *    any brick whose footprint they overlap (yaw-aware 2D OBB test), so
 *    walls can be stacked;
 *  - a closing parallel gripper stops at the brick's width (fingers contact
 *    the faces) and the brick then rides the TCP; opening beyond the brick
 *    width + margin releases it.
 *
 * The gripper is read through `getGripper()` (the attachment manager's
 * mounted 2FG7 API) so grasping works identically for pendant buttons and
 * Code-lab playback.
 */
export class BrickSystem {
  constructor(world, robot, getGripper) {
    this.world = world;
    this.robot = robot;
    this.getGripper = getGripper;
    this.bricks = [];
    this.held = null;
    this.dragging = null;
    this.hovered = null;
    this.selected = null; // rotation-slider target: last added or dragged
    this._ray = new THREE.Raycaster();
    this._ndc = new THREE.Vector2();
    this._v = new THREE.Vector3();
    this._v2 = new THREE.Vector3();
    this._plane = new THREE.Plane();
    this._mat = new THREE.MeshStandardMaterial({ color: 0xf2f1ec, roughness: 0.85, metalness: 0.02 });
    this._bindPointer();
  }

  get count() { return this.bricks.length; }

  /** First brick appears at the sample scripts' pick station. */
  addBrick() {
    if (this.bricks.length >= MAX_BRICKS) return null;
    const n = this.bricks.length;
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(BRICK.l, BRICK.h, BRICK.w), this._mat);
    mesh.castShadow = mesh.receiveShadow = true;
    const spot = n === 0
      ? [0.69, 0.32]
      : [0.45 + 0.16 * ((n - 1) % 3), 0.55 + 0.14 * Math.floor((n - 1) / 3)];
    mesh.position.set(spot[0], BRICK.h / 2, spot[1]);
    mesh.userData.brick = { yaw: 0, vy: 0, falling: false };
    this.world.scene.add(mesh);
    this.bricks.push(mesh);
    this.selected = mesh;
    return mesh;
  }

  /** Yaw of the selected brick in degrees (Tool-tab rotation slider). */
  getYawDeg() {
    const d = this.selected?.userData.brick;
    return d ? Math.round(((d.yaw * 180 / Math.PI) % 360 + 360) % 360) : 0;
  }

  setYawDeg(deg) {
    const b = this.selected;
    if (!b || b === this.held) return; // can't twist a brick inside the gripper
    b.userData.brick.yaw = (deg * Math.PI) / 180;
    b.rotation.set(0, b.userData.brick.yaw, 0);
  }

  clear() {
    if (this.held) this._release(false);
    for (const b of this.bricks) {
      b.removeFromParent();
      b.geometry.dispose();
    }
    this.bricks.length = 0;
    this.dragging = null;
  }

  // --------------------------------------------------------------- physics

  update(dt) {
    this._updateGrasp();
    for (const b of this.bricks) {
      const d = b.userData.brick;
      if (b === this.held || b === this.dragging || !d.falling) continue;
      d.vy -= GRAVITY * dt;
      b.position.y += d.vy * dt;
      const support = this._supportY(b);
      if (b.position.y - BRICK.h / 2 <= support + 1e-4) {
        b.position.y = support + BRICK.h / 2;
        d.vy = 0;
        d.falling = false;
      }
    }
  }

  _updateGrasp() {
    const g = this.getGripper?.();
    if (this.held) {
      // opening past the brick width + margin lets go; losing the gripper does too
      if (!g || g.width > BRICK.w + 0.008) this._release(true);
      return;
    }
    if (!g) return;
    if (g.target >= g.width - 1e-5) { g.setContact(0); return; } // not closing
    g.gripCenter(this._v);
    const candidate = this.bricks.find((b) => {
      if (b === this.dragging) return false;
      this._v2.copy(b.position);
      const horiz = Math.hypot(this._v2.x - this._v.x, this._v2.z - this._v.z);
      return horiz < 0.045 && Math.abs(this._v2.y - this._v.y) < BRICK.h / 2 + 0.02;
    });
    if (!candidate) { g.setContact(0); return; }
    g.setContact(BRICK.w); // fingers stop on the brick faces
    if (g.width <= BRICK.w + 0.0015) {
      this.robot.toolPoint.attach(candidate);
      this.held = candidate;
      candidate.userData.brick.falling = false;
    }
  }

  _release(fromGripper) {
    const b = this.held;
    if (!b) return;
    this.held = null;
    this.world.scene.attach(b);
    // bricks settle flat: keep only the yaw of the current orientation
    this._v.set(1, 0, 0).applyQuaternion(b.quaternion);
    const yaw = Math.atan2(-this._v.z, this._v.x);
    b.rotation.set(0, yaw, 0);
    b.userData.brick.yaw = yaw;
    b.userData.brick.vy = 0;
    b.userData.brick.falling = fromGripper;
    if (!fromGripper) b.position.y = this._supportY(b) + BRICK.h / 2;
  }

  /** Height of whatever is directly underneath: floor or the tallest overlapped brick. */
  _supportY(brick) {
    let top = 0;
    for (const other of this.bricks) {
      if (other === brick || other === this.held) continue;
      const oTop = other.position.y + BRICK.h / 2;
      if (oTop > brick.position.y - BRICK.h / 2 + 0.02) continue; // not below
      if (oTop <= top) continue;
      if (footprintsOverlap(brick, other)) top = oTop;
    }
    return top;
  }

  // ----------------------------------------------------------- interaction

  _bindPointer() {
    const dom = this.world.renderer?.domElement;
    if (!dom?.addEventListener) return; // headless tests
    dom.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      const hit = this._pick(e);
      if (!hit || hit === this.held) return;
      this.dragging = hit;
      this.selected = hit;
      hit.userData.brick.falling = false;
      this.world.controls.enabled = false;
      dom.setPointerCapture(e.pointerId);
    });
    dom.addEventListener('pointermove', (e) => {
      if (this.dragging) {
        // slide on a horizontal plane at the brick's current height
        this._plane.set(new THREE.Vector3(0, 1, 0), -this.dragging.position.y);
        this._castRay(e);
        if (this._ray.ray.intersectPlane(this._plane, this._v)) {
          const r = Math.hypot(this._v.x, this._v.z);
          const k = r > 1.6 ? 1.6 / r : 1; // stay in the robot's neighbourhood
          this.dragging.position.x = this._v.x * k;
          this.dragging.position.z = this._v.z * k;
        }
        return;
      }
      if (this.bricks.length) {
        this.hovered = this._pick(e);
        dom.style.cursor = this.hovered && this.hovered !== this.held ? 'grab' : '';
      }
    });
    const drop = () => {
      if (!this.dragging) return;
      this.dragging.userData.brick.falling = true; // resettle (stack or floor)
      this.dragging = null;
      this.world.controls.enabled = true;
    };
    dom.addEventListener('pointerup', drop);
    dom.addEventListener('pointercancel', drop);
  }

  _castRay(e) {
    const rect = this.world.renderer.domElement.getBoundingClientRect();
    this._ndc.set(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this._ray.setFromCamera(this._ndc, this.world.camera);
  }

  _pick(e) {
    this._castRay(e);
    const hit = this._ray.intersectObjects(this.bricks, false)[0];
    return hit?.object ?? null;
  }
}

/** Yaw-aware 2D rectangle overlap on the floor plane (separating axes). */
function footprintsOverlap(a, b) {
  const rect = (m) => {
    const yaw = m.userData.brick.yaw;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return {
      x: m.position.x, z: m.position.z,
      ax: [c, -s], az: [s, c], // local axes in the floor plane
      hx: BRICK.l / 2, hz: BRICK.w / 2,
    };
  };
  const A = rect(a);
  const B = rect(b);
  const axes = [A.ax, A.az, B.ax, B.az];
  const dx = B.x - A.x;
  const dz = B.z - A.z;
  for (const [ux, uz] of axes) {
    const dist = Math.abs(dx * ux + dz * uz);
    const ra = A.hx * Math.abs(A.ax[0] * ux + A.ax[1] * uz) + A.hz * Math.abs(A.az[0] * ux + A.az[1] * uz);
    const rb = B.hx * Math.abs(B.ax[0] * ux + B.ax[1] * uz) + B.hz * Math.abs(B.az[0] * ux + B.az[1] * uz);
    if (dist > ra + rb) return false;
  }
  return true;
}
