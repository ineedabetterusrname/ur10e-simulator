import * as THREE from 'three';

/** Solve A x = b for a dense n x n system (Gaussian elimination, partial pivot). */
export function solveLinear(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    [M[col], M[piv]] = [M[piv], M[col]];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col] / M[col][col];
      if (f === 0) continue;
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return M.map((row, i) => row[n] / M[i][i]);
}

/**
 * Kinematics for the 6 arm joints, derived numerically from the scene graph
 * itself so the math can never drift from the visuals. Poses are expressed
 * in the robot base frame (Z-up).
 */
export class Kinematics {
  constructor(robot) {
    this.robot = robot;
    this.eps = 1e-4;
    this.lambda = 0.08; // DLS damping
    this._inv = new THREE.Matrix4();
    this._m = new THREE.Matrix4();
    this._pos = new THREE.Vector3();
    this._quat = new THREE.Quaternion();
    this._scale = new THREE.Vector3();
  }

  /** TCP pose in the robot base frame: { pos: Vector3, quat: Quaternion }. */
  tcpInBase(out = { pos: new THREE.Vector3(), quat: new THREE.Quaternion() }) {
    this.robot.root.updateMatrixWorld(true);
    this._inv.copy(this.robot.root.matrixWorld).invert();
    this._m.multiplyMatrices(this._inv, this.robot.toolPoint.matrixWorld);
    this._m.decompose(out.pos, out.quat, this._scale);
    return out;
  }

  /** RPY (XYZ intrinsic) of the TCP in the base frame, radians. */
  tcpRPY() {
    const { quat } = this.tcpInBase();
    const e = new THREE.Euler().setFromQuaternion(quat, 'XYZ');
    return [e.x, e.y, e.z];
  }

  /**
   * 6x6 geometric Jacobian (rows: vx vy vz wx wy wz in base frame,
   * cols: arm joints) via central-friendly forward differences.
   */
  jacobian() {
    const robot = this.robot;
    const q0 = robot.getAngles();
    const ref = this.tcpInBase();
    const refPos = ref.pos.clone();
    const refQuat = ref.quat.clone();
    const J = Array.from({ length: 6 }, () => new Array(6).fill(0));
    const p = { pos: new THREE.Vector3(), quat: new THREE.Quaternion() };
    const dq = new THREE.Quaternion();
    for (let i = 0; i < 6; i++) {
      const q = [...q0];
      q[i] += this.eps;
      robot.setAngles(q);
      this.tcpInBase(p);
      J[0][i] = (p.pos.x - refPos.x) / this.eps;
      J[1][i] = (p.pos.y - refPos.y) / this.eps;
      J[2][i] = (p.pos.z - refPos.z) / this.eps;
      // orientation delta as a rotation vector
      dq.copy(p.quat).multiply(refQuat.clone().invert());
      if (dq.w < 0) dq.set(-dq.x, -dq.y, -dq.z, -dq.w);
      const angle = 2 * Math.acos(Math.min(1, dq.w));
      const s = Math.sqrt(Math.max(0, 1 - dq.w * dq.w));
      const k = s > 1e-9 ? angle / s : 2; // small-angle: 2*(x,y,z)
      J[3][i] = (dq.x * k) / this.eps;
      J[4][i] = (dq.y * k) / this.eps;
      J[5][i] = (dq.z * k) / this.eps;
    }
    robot.setAngles(q0);
    this.robot.root.updateMatrixWorld(true);
    return J;
  }

  /**
   * Damped-least-squares step: joint deltas that move the TCP by dx
   * ([dx dy dz drx dry drz] in base frame). Returns Array(6) or null
   * when close to a singularity.
   */
  dlsStep(dx) {
    const J = this.jacobian();
    const n = 6;
    // A = J J^T + lambda^2 I
    const A = Array.from({ length: n }, (_, r) =>
      Array.from({ length: n }, (_, c) => {
        let s = 0;
        for (let k = 0; k < 6; k++) s += J[r][k] * J[c][k];
        return r === c ? s + this.lambda * this.lambda : s;
      })
    );
    const y = solveLinear(A, dx);
    if (!y) return null;
    const dq = new Array(6).fill(0);
    for (let i = 0; i < 6; i++) {
      for (let r = 0; r < n; r++) dq[i] += J[r][i] * y[r];
    }
    return dq;
  }
}
