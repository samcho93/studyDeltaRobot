// dynamics.js — lumped dynamics for motor sizing. Mirror of python/deltarobot/dynamics.py.
import { fk, jacobian } from './kinematics.js';
import { G } from './design.js';

export const STEPPER_MARGIN = 1.5;

export function jointTorques(d, theta, thetaDd, p, a = [0, 0, 0]) {
  if (!p) p = fk(d, theta);
  const J = jacobian(d, theta, p);
  const m = d.movingPlateMass;
  const f = [m * a[0], m * a[1], m * (a[2] + G)];
  const I = d.armInertia, k = d.gravityMoment;
  return [0, 1, 2].map((i) => I * thetaDd[i] - k * G * Math.cos(theta[i])
    + J[0][i] * f[0] + J[1][i] * f[1] + J[2][i] * f[2]);
}

export function availableTorque(d, omegaMotor) {
  const m = d.motorSpec;
  const w = Math.abs(omegaMotor), wmax = Number(m.max_speed), peak = Number(m.peak_torque);
  if (w >= wmax) return 0;
  if (m.kind === 'rc_servo' || m.kind === 'smart_servo') return peak * (1 - w / wmax);
  if (m.kind === 'stepper') {
    const tmax = Number(m.torque_at_max_speed !== undefined ? m.torque_at_max_speed : 0.25 * peak);
    return peak - (peak - tmax) * w / wmax;
  }
  return peak;
}

export function inertiaRatio(d) {
  const m = d.motorSpec;
  if (m.integrated_gear || Number(m.rotor_inertia) <= 0) return 0;
  const n = d.ratio;
  const load = d.armInertia - d.driveInertia + d.movingPlateMass * d.upper_arm ** 2 / 3;
  return load / (n * n) / Number(m.rotor_inertia);
}

/** frames: [{t, q, tool}] -> samples with theta_d, theta_dd, p, v, a, tau. */
export function analyze(d, frames) {
  const n = frames.length;
  if (!n) return [];
  const ts = frames.map((f) => Number(f.t));
  const qs = frames.map((f) => f.q.slice());
  const ps = qs.map((q) => fk(d, q));
  const deriv = (vals, k) => {
    const w = vals[0].length;
    if (n === 1) return new Array(w).fill(0);
    const lo = Math.max(0, k - 1), hi = Math.min(n - 1, k + 1);
    const dt = ts[hi] - ts[lo];
    if (dt <= 0) return new Array(w).fill(0);
    return vals[0].map((_, j) => (vals[hi][j] - vals[lo][j]) / dt);
  };
  const qd = qs.map((_, k) => deriv(qs, k));
  const v = ps.map((_, k) => deriv(ps, k));
  const qdd = qd.map((_, k) => deriv(qd, k));
  const acc = v.map((_, k) => deriv(v, k));
  return qs.map((q, k) => ({
    t: ts[k], theta: q, theta_d: qd[k], theta_dd: qdd[k], p: ps[k], v: v[k], a: acc[k],
    tau: jointTorques(d, q, qdd[k], ps[k], acc[k]), tool: frames[k].tool || 0,
  }));
}

export function evaluate(d, samples, safetyFactor = 1.2) {
  const m = d.motorSpec;
  const n = d.ratio, eta = d.efficiency;
  const peak = [0, 0, 0], wmax = [0, 0, 0], sq = [0, 0, 0];
  let worst = 0, total = 0, prev = null;
  for (const s of samples) {
    const t = Number(s.t);
    const dt = prev === null ? 0 : t - prev;
    prev = t;
    total += dt;
    for (let i = 0; i < 3; i++) {
      const tm = Math.abs(s.tau[i]) / (n * eta);
      const wm = Math.abs(s.theta_d[i]) * n;
      peak[i] = Math.max(peak[i], tm);
      wmax[i] = Math.max(wmax[i], wm);
      sq[i] += tm * tm * dt;
      let avail = availableTorque(d, wm);
      if (m.kind === 'stepper') avail /= STEPPER_MARGIN;
      worst = Math.max(worst, avail > 0 ? tm * safetyFactor / avail : Infinity);
    }
  }
  const rms = sq.map((v) => (total > 0 ? Math.sqrt(v / total) : 0));
  const peakRatio = Math.max(...peak) * safetyFactor / Number(m.peak_torque);
  const rmsRatio = Math.max(...rms) * safetyFactor / Number(m.rated_torque);
  const speedRatio = Math.max(...wmax) / Number(m.max_speed);
  const problems = [];
  if (worst > 1) problems.push('속도-토크 곡선 초과: 해당 속도에서 낼 수 있는 토크보다 큰 토크가 필요합니다');
  if (peakRatio > 1) problems.push('피크 토크 초과 (안전율 ' + safetyFactor.toFixed(1) + ' 포함)');
  if (rmsRatio > 1) problems.push('RMS(연속) 토크가 정격을 넘습니다 — 과열 위험');
  if (speedRatio > 1) problems.push('모터 최고 속도 초과');
  return {
    peak_torque_motor: peak, rms_torque_motor: rms, max_speed_motor: wmax,
    peak_ratio: peakRatio, rms_ratio: rmsRatio, speed_ratio: speedRatio, curve_ratio: worst,
    inertia_ratio: inertiaRatio(d), ok: problems.length === 0, problems,
  };
}
