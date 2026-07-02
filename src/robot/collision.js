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

/** Link pairs eligible for self-collision (non-adjacent, geometrically able to touch). */
const PAIRS = [
  ['base', 'forearm'], ['base', 'wrist'], ['base', 'hand'], ['base', 'tool'],
  ['shoulder', 'forearm'], ['shoulder', 'wrist'], ['shoulder', 'hand'], ['shoulder', 'tool'],
  ['upperarm', 'forearm'], ['upperarm', 'wrist'], ['upperarm', 'hand'], ['upperarm', 'tool'],
  ['forearm', 'hand'], ['forearm', 'tool'],
];

/** Links that never reach the floor (or sit on it by design). */
const GROUND_EXEMPT = new Set(['base', 'shoulder']);

export class CollisionChecker {
  constructor(robot) {
    this.robot = robot;
    this.groundY = 0;
    this._a = new THREE.Vector3();
    this._b = new THREE.Vector3();
    this._c = new THREE.Vector3();
    this._d = new THREE.Vector3();
  }

  _world(cap, outP, outQ) {
    outP.copy(cap.p1).applyMatrix4(cap.node.matrixWorld);
    outQ.copy(cap.p2).applyMatrix4(cap.node.matrixWorld);
  }

  /**
   * Returns a human-readable reason string when any capsule pair intersects,
   * a robot capsule dips below the floor, or a robot capsule hits one of the
   * static world obstacles ({p1,p2,r,name} in world coords). Null when clear.
   */
  check(staticObstacles = []) {
    const caps = this.robot.capsules;
    for (const [n1, n2] of PAIRS) {
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
      if (GROUND_EXEMPT.has(name)) continue;
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
