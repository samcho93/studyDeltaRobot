// kinematics.js — rotary delta robot kinematics.
// Mirror of python/deltarobot/kinematics.py (same conventions, same solution choice).
// Positions are the effector centre [m]; theta: 0 = upper arm horizontal, + = down.

export const PHI = [0, 2 * Math.PI / 3, 4 * Math.PI / 3];
export const ELBOW_MIN = 20 * Math.PI / 180;
export const ELBOW_MAX = 165 * Math.PI / 180;

export class Unreachable extends Error {}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.sqrt(dot(a, a));
export const vec = { sub, add, scale, dot, cross, norm };

export const det3 = (m) => dot(m[0], cross(m[1], m[2]));
export function inv3(m) {
  const d = det3(m);
  if (Math.abs(d) < 1e-15) throw new Error('singular matrix');
  const c0 = cross(m[1], m[2]), c1 = cross(m[2], m[0]), c2 = cross(m[0], m[1]);
  return [[c0[0] / d, c1[0] / d, c2[0] / d], [c0[1] / d, c1[1] / d, c2[1] / d], [c0[2] / d, c1[2] / d, c2[2] / d]];
}
const transpose = (m) => [0, 1, 2].map((i) => [0, 1, 2].map((j) => m[j][i]));
const matmul = (a, b) => { const bt = transpose(b); return [0, 1, 2].map((i) => [0, 1, 2].map((j) => dot(a[i], bt[j]))); };

function symEigvals(a) {
  const p1 = a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2;
  const q = (a[0][0] + a[1][1] + a[2][2]) / 3;
  if (p1 < 1e-30) return [a[0][0], a[1][1], a[2][2]].sort((x, y) => x - y);
  const p2 = (a[0][0] - q) ** 2 + (a[1][1] - q) ** 2 + (a[2][2] - q) ** 2 + 2 * p1;
  const p = Math.sqrt(p2 / 6);
  const b = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (a[i][j] - (i === j ? q : 0)) / p));
  const rr = Math.max(-1, Math.min(1, det3(b) / 2));
  const phi = Math.acos(rr) / 3;
  const e1 = q + 2 * p * Math.cos(phi);
  const e3 = q + 2 * p * Math.cos(phi + 2 * Math.PI / 3);
  const e2 = 3 * q - e1 - e3;
  return [e3, e2, e1].sort((x, y) => x - y);
}

export const armAxis = (i) => [Math.cos(PHI[i]), Math.sin(PHI[i]), 0];
export const motorPoint = (d, i) => scale(armAxis(i), d.base_radius);

export function elbow(d, i, theta) {
  const rho = d.base_radius + d.upper_arm * Math.cos(theta);
  return [rho * Math.cos(PHI[i]), rho * Math.sin(PHI[i]), -d.upper_arm * Math.sin(theta)];
}
export function elbowDerivative(d, i, theta) {
  const L = d.upper_arm, c = Math.cos(PHI[i]), s = Math.sin(PHI[i]);
  return [-L * Math.sin(theta) * c, -L * Math.sin(theta) * s, -L * Math.cos(theta)];
}
export const ballJoint = (d, i, p) => add(p, scale(armAxis(i), d.effector_radius));

export function ikArm(d, i, p) {
  const c = Math.cos(PHI[i]), s = Math.sin(PHI[i]);
  const [x, y, z] = p;
  const xa = x * c + y * s, ya = -x * s + y * c;
  const a = xa + d.effector_radius - d.base_radius;
  const L = d.upper_arm, l = d.forearm;
  const K = (l * l - L * L - a * a - ya * ya - z * z) / (2 * L);
  const rho = Math.hypot(a, z);
  if (rho < 1e-12 || Math.abs(K) > rho) throw new Unreachable('arm ' + (i + 1) + ' cannot reach');
  const alpha = Math.atan2(a, z);
  const asn = Math.asin(K / rho);
  const t1 = alpha + asn, t2 = alpha + Math.PI - asn;
  const t = Math.cos(t1) > Math.cos(t2) ? t1 : t2;
  return Math.atan2(Math.sin(t), Math.cos(t));
}
export const ik = (d, p) => [ikArm(d, 0, p), ikArm(d, 1, p), ikArm(d, 2, p)];

/** IK that returns null instead of throwing. */
export function tryIk(d, p) { try { return ik(d, p); } catch (e) { return null; } }

export function fk(d, theta) {
  const l = d.forearm;
  const c = [0, 1, 2].map((i) => sub(elbow(d, i, theta[i]), scale(armAxis(i), d.effector_radius)));
  const exv = sub(c[1], c[0]);
  const dd = norm(exv);
  if (dd < 1e-12) throw new Unreachable('degenerate sphere centres');
  const ex = scale(exv, 1 / dd);
  const t = sub(c[2], c[0]);
  const i_ = dot(ex, t);
  const eyv = sub(t, scale(ex, i_));
  const eyn = norm(eyv);
  if (eyn < 1e-12) throw new Unreachable('sphere centres are collinear');
  const ey = scale(eyv, 1 / eyn);
  const ez = cross(ex, ey);
  const j = dot(ey, t);
  const x = dd / 2;
  const y = (i_ * i_ + j * j - 2 * i_ * x) / (2 * j);
  const h2 = l * l - x * x - y * y;
  if (h2 < 0) throw new Unreachable('no forward kinematic solution');
  const h = Math.sqrt(h2);
  const base = add(c[0], add(scale(ex, x), scale(ey, y)));
  const p1 = add(base, scale(ez, h)), p2 = sub(base, scale(ez, h));
  return p1[2] < p2[2] ? p1 : p2;
}
export function tryFk(d, theta) { try { return fk(d, theta); } catch (e) { return null; } }

export function jacobianParts(d, theta, p) {
  if (!p) p = fk(d, theta);
  const jx = [], jt = [];
  for (let i = 0; i < 3; i++) {
    const s = sub(ballJoint(d, i, p), elbow(d, i, theta[i]));
    jx.push(s);
    jt.push(dot(s, elbowDerivative(d, i, theta[i])));
  }
  return [jx, jt];
}
export function jacobian(d, theta, p) {
  const [jx, jt] = jacobianParts(d, theta, p);
  const inv = inv3(jx);
  return [0, 1, 2].map((r) => [0, 1, 2].map((c) => inv[r][c] * jt[c]));
}
export function jointVelocity(d, theta, v, p) {
  const [jx, jt] = jacobianParts(d, theta, p);
  return [0, 1, 2].map((i) => dot(jx[i], v) / jt[i]);
}
export function singularValues(j) {
  return symEigvals(matmul(transpose(j), j)).map((e) => Math.sqrt(Math.max(0, e)));
}
export function conditionNumber(d, theta, p) {
  let sv;
  try { sv = singularValues(jacobian(d, theta, p)); } catch (e) { return Infinity; }
  return sv[0] > 1e-12 ? sv[2] / sv[0] : Infinity;
}

export function forearmDirection(d, i, theta, p) {
  const s = sub(ballJoint(d, i, p), elbow(d, i, theta));
  return scale(s, 1 / norm(s));
}
export function passiveAngles(d, theta, p) {
  if (!p) p = fk(d, theta);
  const out = [];
  for (let i = 0; i < 3; i++) {
    const dv = forearmDirection(d, i, theta[i], p);
    const c = Math.cos(PHI[i]), s = Math.sin(PHI[i]);
    const dx = dv[0] * c + dv[1] * s, dy = -dv[0] * s + dv[1] * c, dz = dv[2];
    const ct = Math.cos(theta[i]), st = Math.sin(theta[i]);
    const lx = ct * dx - st * dz, lz = st * dx + ct * dz, ly = dy;
    const yaw = Math.asin(Math.max(-1, Math.min(1, ly)));
    out.push([Math.atan2(-lz, lx), yaw]);
  }
  return out;
}
export function elbowAngle(d, i, theta, p) {
  const e = elbow(d, i, theta);
  const a = sub(motorPoint(d, i), e), b = sub(ballJoint(d, i, p), e);
  return Math.acos(Math.max(-1, Math.min(1, dot(a, b) / (norm(a) * norm(b)))));
}

export function limitReport(d, p) {
  let th;
  try { th = ik(d, p); } catch (e) { return { ok: false, theta: null, problems: ['unreachable'], ball: [], elbow: [] }; }
  const problems = [], ball = [], elb = [];
  const pa = passiveAngles(d, th, p);
  for (let i = 0; i < 3; i++) {
    if (th[i] < d.theta_min - 1e-9 || th[i] > d.theta_max + 1e-9) problems.push('theta' + (i + 1));
    const yaw = pa[i][1];
    ball.push(Math.abs(yaw));
    if (Math.abs(yaw) > d.ball_joint_limit) problems.push('ball' + (i + 1));
    const ea = elbowAngle(d, i, th[i], p);
    elb.push(ea);
    if (ea < ELBOW_MIN || ea > ELBOW_MAX) problems.push('elbow' + (i + 1));
  }
  return { ok: problems.length === 0, theta: th, problems, ball, elbow: elb };
}
export const reachable = (d, p) => limitReport(d, p).ok;

export function workspaceBounds(d) {
  return [d.base_radius - d.effector_radius + d.upper_arm + d.forearm, 0, -(d.upper_arm + d.forearm)];
}

/** Grid-sample reachable positions. Returns flat Float32Array [x,y,z,...]. */
export function workspacePoints(d, step = 0) {
  const [rad, ztop, zbot] = workspaceBounds(d);
  const h = step || rad / 14;
  const n = Math.floor(rad / h), nz = Math.floor((ztop - zbot) / h);
  const pts = [];
  for (let iz = 0; iz <= nz; iz++) {
    const z = ztop - iz * h;
    for (let ix = -n; ix <= n; ix++) {
      for (let iy = -n; iy <= n; iy++) {
        const x = ix * h, y = iy * h;
        if (x * x + y * y > rad * rad) continue;
        if (reachable(d, [x, y, z])) pts.push(x, y, z);
      }
    }
  }
  return new Float32Array(pts);
}

export function maxRadiusAt(d, z, rmax, nDir = 24, tol = 5e-4) {
  if (!reachable(d, [0, 0, z])) return 0;
  const ok = (rho) => {
    for (let k = 0; k < nDir; k++) {
      const a = 2 * Math.PI * k / nDir;
      if (!reachable(d, [rho * Math.cos(a), rho * Math.sin(a), z])) return false;
    }
    return true;
  };
  const step = rmax / 20;
  let lo = 0, hi = null, rho = step;
  while (rho <= rmax) {
    if (ok(rho)) { lo = rho; rho += step; } else { hi = rho; break; }
  }
  if (hi === null) return lo;
  while (hi - lo > tol) {
    const mid = (lo + hi) / 2;
    if (ok(mid)) lo = mid; else hi = mid;
  }
  return lo;
}

export function workCylinder(d, height, nz = 40) {
  const [rad, ztop, zbot] = workspaceBounds(d);
  const dz = (ztop - zbot) / nz;
  const profile = [];
  for (let k = 0; k <= nz; k++) {
    const z = ztop - k * dz;
    profile.push([z, maxRadiusAt(d, z, rad)]);
  }
  const win = Math.max(1, Math.round(height / dz));
  let best = [0, 0];
  for (let k = 0; k < profile.length - win; k++) {
    let m = Infinity;
    for (let j = k; j <= k + win; j++) m = Math.min(m, profile[j][1]);
    if (m > best[0]) best = [m, k];
  }
  const zTop = profile[best[1]][0];
  return { diameter: 2 * best[0], height: win * dz, z_top: zTop, z_bottom: zTop - win * dz, profile };
}
