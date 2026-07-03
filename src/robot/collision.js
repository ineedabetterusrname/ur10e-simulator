import * as THREE from 'three';

/** Closest squared distance between segments p1q1 and p2q2 (Ericson, RTCD 5.1.9). */
function segSegDist2(p1, q1, p2, q2) {
  const d1 = new THREE.Vector3().subVectors(q1, p1);
  const d2 = new THREE.Vector3().subVectors(q2, p2);
  const r = new THREE.Vector3().subVectors(p1, p2);
  const a = d1.dot(d1);
  const e = d2.dot(d2);
  const f = d2.dot(r);
  let s, t;
  const EPS = 1e-10;
  if (a <= EPS && e <= EPS) {
    return r.dot(r);
  }
  if (a <= EPS) {
    s = 0;
    t = THREE.MathUtils.clamp(f / e, 0, 1);
  } else {
    const c = d1.dot(r);
    if (e <= EPS) {
      t = 0;
      s = THREE.MathUtils.clamp(-c / a, 0, 1);
    } else {
      const b = d1.dot(d2);
      const denom = a * e - b * b;
      s = denom > EPS ? THREE.MathUtils.clamp((b * f - c * e) / denom, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = THREE.MathUtils.clamp(-c / a, 0, 1);
      } else if (t > 1) {
        t = 1;
        s = THREE.MathUtils.clamp((b - c) / a, 0, 1);
      }
    }
  }
  const c1 = p1.clone().addScaledVector(d1, s);
  const c2 = p2.clone().addScaledVector(d2, t);
  return c1.distanceToSquared(c2);
}

/* ------------------------------------------------------------------ */
/* Legacy capsule model — only used until the official mesh (and its   */
/* per-link BVH colliders, see realMesh.js) has loaded.                */
/* ------------------------------------------------------------------ */

/** Capsule pairs eligible for self-collision (non-adjacent, geometrically able to touch). */
const CAPSULE_PAIRS = [
  ['base', 'forearm'], ['base', 'wrist'], ['base', 'hand'], ['base', 'tool'],
  ['shoulder', 'forearm'], ['shoulder', 'wrist'], ['shoulder', 'hand'], ['shoulder', 'tool'],
  ['upperarm', 'forearm'], ['upperarm', 'wrist'], ['upperarm', 'hand'], ['upperarm', 'tool'],
  ['forearm', 'hand'], ['forearm', 'tool'],
];

/** Capsules that never reach the floor (or sit on it by design). */
const CAPSULE_GROUND_EXEMPT = new Set(['base', 'shoulder']);

/* ------------------------------------------------------------------ */
/* Mesh model — exact distance tests between the real link meshes.     */
/* ------------------------------------------------------------------ */

/**
 * Link pairs eligible for self-collision. Chain-adjacent links (bolted
 * together at a joint) are excluded; so are pairs that cannot physically
 * reach each other (e.g. GROUND/SHOULDER).
 */
const LINK_PAIRS = [
  ['GROUND', 'ELBOW'], ['GROUND', 'WRIST1'], ['GROUND', 'WRIST2'], ['GROUND', 'FLANGE'],
  ['BASE', 'ELBOW'], ['BASE', 'WRIST1'], ['BASE', 'WRIST2'], ['BASE', 'FLANGE'],
  ['SHOULDER', 'WRIST1'], ['SHOULDER', 'WRIST2'], ['SHOULDER', 'FLANGE'],
  ['ELBOW', 'WRIST2'], ['ELBOW', 'FLANGE'],
];

/** Links the mounted-tool capsule is checked against (wrist links are adjacent). */
const TOOL_VS_LINKS = ['GROUND', 'BASE', 'SHOULDER', 'ELBOW'];

/** Human-readable names for protective-stop reasons. */
const LINK_LABEL = {
  GROUND: 'base', BASE: 'base', SHOULDER: 'shoulder / upper arm', ELBOW: 'forearm',
  WRIST1: 'wrist 1', WRIST2: 'wrist 2', FLANGE: 'wrist 3',
};

export class CollisionChecker {
  constructor(robot) {
    this.robot = robot;
    this.groundY = 0;
    /**
     * Protective-stop margin for the mesh model: stop when real surfaces come
     * within this distance. Must stay above the collision mesh's ±1.5 mm
     * simplification error so surfaces can never visibly interpenetrate.
     */
    this.clearance = 0.005;
    this._a = new THREE.Vector3();
    this._b = new THREE.Vector3();
    this._c = new THREE.Vector3();
    this._d = new THREE.Vector3();
    this._m = new THREE.Matrix4();
    this._inv = new THREE.Matrix4();
    this._line = new THREE.Line3();
    this._ray = new THREE.Ray();
    this._plane = new THREE.Plane();
    this._t1 = {};
    this._t2 = {};
  }

  _world(cap, outP, outQ) {
    outP.copy(cap.p1).applyMatrix4(cap.node.matrixWorld);
    outQ.copy(cap.p2).applyMatrix4(cap.node.matrixWorld);
  }

  /**
   * Returns a human-readable reason string when robot links come within the
   * safety clearance of each other, the floor, or a static world obstacle
   * ({p1,p2,r,name} in world coords). Null when clear. Uses exact per-link
   * mesh distances once realMesh.js has installed robot.linkColliders;
   * approximate capsules before that.
   */
  check(staticObstacles = []) {
    return this.robot.linkColliders
      ? this._checkMesh(staticObstacles)
      : this._checkCapsules(staticObstacles);
  }

  /* ---------------------------- mesh path ---------------------------- */

  _checkMesh(staticObstacles) {
    const cols = this.robot.linkColliders;
    const caps = this.robot.capsules;

    // self-collision: exact mesh-to-mesh distance, sphere broad phase first
    for (const [n1, n2] of LINK_PAIRS) {
      const A = cols[n1];
      const B = cols[n2];
      const sA = A.geometry.boundingSphere;
      const sB = B.geometry.boundingSphere;
      this._a.copy(sA.center).applyMatrix4(A.node.matrixWorld);
      this._b.copy(sB.center).applyMatrix4(B.node.matrixWorld);
      const reach = sA.radius + sB.radius + this.clearance;
      if (this._a.distanceToSquared(this._b) > reach * reach) continue;

      // B expressed in A's local frame
      this._m.copy(A.node.matrixWorld).invert().multiply(B.node.matrixWorld);
      const res = A.geometry.boundsTree.closestPointToGeometry(
        B.geometry, this._m, this._t1, this._t2, 0, this.clearance);
      if (res && res.distance <= this.clearance) {
        return `self-collision: ${LINK_LABEL[n1]} / ${LINK_LABEL[n2]}`;
      }
    }

    // floor: any link surface dipping below ground level
    for (const [name, col] of Object.entries(cols)) {
      if (col.floorExempt) continue;
      if (this._meshBelowFloor(col)) return `floor collision: ${LINK_LABEL[name]}`;
    }

    // mounted tool (capsule, sized by the attachment manager) vs link meshes
    const tool = caps.tool;
    this._world(tool, this._c, this._d);
    for (const name of TOOL_VS_LINKS) {
      if (this._segmentNearMesh(cols[name], this._c, this._d, tool.r)) {
        return `self-collision: tool / ${LINK_LABEL[name]}`;
      }
    }

    // static world obstacles (world-space capsules, e.g. the track rail)
    for (const obs of staticObstacles) {
      for (const [name, col] of Object.entries(cols)) {
        if (obs.exempt && col.legacy.every((n) => obs.exempt.has(n))) continue;
        if (this._segmentNearMesh(col, obs.p1, obs.p2, obs.r)) {
          return `collision: ${LINK_LABEL[name]} / ${obs.name}`;
        }
      }
      if (!(obs.exempt && obs.exempt.has('tool'))) {
        this._world(tool, this._a, this._b);
        const rr = tool.r + obs.r;
        if (segSegDist2(this._a, this._b, obs.p1, obs.p2) < rr * rr) {
          return `collision: tool / ${obs.name}`;
        }
      }
    }
    return null;
  }

  /** True when any triangle of the collider pokes below groundY (+2 mm margin). */
  _meshBelowFloor(col) {
    this._plane.normal.set(0, 1, 0);
    this._plane.constant = -(this.groundY + 0.002);
    this._inv.copy(col.node.matrixWorld).invert();
    this._plane.applyMatrix4(this._inv); // world plane -> collider local
    const plane = this._plane;
    const p = this._a;
    return col.geometry.boundsTree.shapecast({
      intersectsBounds: (box) => {
        // lowest corner of the box along the plane normal
        p.set(
          plane.normal.x > 0 ? box.min.x : box.max.x,
          plane.normal.y > 0 ? box.min.y : box.max.y,
          plane.normal.z > 0 ? box.min.z : box.max.z,
        );
        return plane.distanceToPoint(p) < 0;
      },
      intersectsTriangle: (tri) =>
        plane.distanceToPoint(tri.a) < 0 ||
        plane.distanceToPoint(tri.b) < 0 ||
        plane.distanceToPoint(tri.c) < 0,
    });
  }

  /** True when a world-space capsule (p1,p2,r) comes within clearance of the collider mesh. */
  _segmentNearMesh(col, p1World, p2World, radius) {
    this._inv.copy(col.node.matrixWorld).invert();
    const line = this._line;
    line.start.copy(p1World).applyMatrix4(this._inv);
    line.end.copy(p2World).applyMatrix4(this._inv);
    const rr = radius + this.clearance;
    const rr2 = rr * rr;
    const c = this._a;
    const q = this._b;
    const ray = this._ray;
    const segLen = line.distance();
    ray.origin.copy(line.start);
    line.delta(ray.direction).normalize();
    return col.geometry.boundsTree.shapecast({
      intersectsBounds: (box) => {
        box.getCenter(c);
        const halfDiag = c.distanceTo(box.max);
        line.closestPointToPoint(c, true, q);
        return q.distanceTo(c) <= rr + halfDiag;
      },
      intersectsTriangle: (tri) => {
        // segment piercing the triangle counts as contact
        if (segLen > 0 && ray.intersectTriangle(tri.a, tri.b, tri.c, false, q)) {
          if (q.distanceToSquared(ray.origin) <= segLen * segLen) return true;
        }
        if (segSegDist2(line.start, line.end, tri.a, tri.b) <= rr2) return true;
        if (segSegDist2(line.start, line.end, tri.b, tri.c) <= rr2) return true;
        if (segSegDist2(line.start, line.end, tri.c, tri.a) <= rr2) return true;
        tri.closestPointToPoint(line.start, q);
        if (q.distanceToSquared(line.start) <= rr2) return true;
        tri.closestPointToPoint(line.end, q);
        return q.distanceToSquared(line.end) <= rr2;
      },
    });
  }

  /* --------------------------- capsule path -------------------------- */

  _checkCapsules(staticObstacles) {
    const caps = this.robot.capsules;
    for (const [n1, n2] of CAPSULE_PAIRS) {
      const c1 = caps[n1];
      const c2 = caps[n2];
      this._world(c1, this._a, this._b);
      this._world(c2, this._c, this._d);
      const rr = c1.r + c2.r;
      if (segSegDist2(this._a, this._b, this._c, this._d) < rr * rr) {
        return `self-collision: ${n1} / ${n2}`;
      }
    }
    for (const name of Object.keys(caps)) {
      if (CAPSULE_GROUND_EXEMPT.has(name)) continue;
      const c = caps[name];
      this._world(c, this._a, this._b);
      const low = Math.min(this._a.y, this._b.y) - c.r;
      if (low < this.groundY + 0.002) return `floor collision: ${name}`;
    }
    for (const obs of staticObstacles) {
      for (const name of Object.keys(caps)) {
        if (obs.exempt && obs.exempt.has(name)) continue;
        const c = caps[name];
        this._world(c, this._a, this._b);
        const rr = c.r + obs.r;
        if (segSegDist2(this._a, this._b, obs.p1, obs.p2) < rr * rr) {
          return `collision: ${name} / ${obs.name}`;
        }
      }
    }
    return null;
  }
}
