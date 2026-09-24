// planner.js — JS twin of deltarobot.DeltaRobot with the record backend.
// Builds a timeline {frames:[{t,q,tool}], ...} for the simulator's demo programs.
import { fk, limitReport } from '../assets/js/delta/kinematics.js';
import { Path, linePath, archPath, samplePath, sampleJoint, Profile } from '../assets/js/delta/trajectory.js';
import { SceneState } from '../assets/js/delta/scene.js';

export const DT = 0.01;
const HOLD_DT = 0.05;

export class WorkspaceError extends Error {}

const WHY = { unreachable: '팔 길이로 닿지 않음', theta: '모터 각도 한계', ball: '볼조인트 각도 한계', elbow: '팔꿈치 각도 한계' };
export function explain(p, problems) {
  const why = [...new Set(problems.map((x) => WHY[x.replace(/[123]$/, '')] || x))].sort();
  return `작업영역 밖: (${p.map((v) => v.toFixed(3)).join(', ')}) m — ${why.join(', ')}`;
}

export class Planner {
  constructor(design, sceneData, opts = {}) {
    this.d = design;
    this.scene = new SceneState(sceneData);
    this.sceneData = sceneData;
    this.speed = opts.speed || 0.25 * (design.upper_arm + design.forearm) / 0.43;
    this.accel = opts.accel || 10 * this.speed;
    this.profile = opts.profile || 'scurve';
    this.jointSpeed = 3.0;
    this.jointAccel = 30.0;
    this.t = 0;
    this.tool = 0;
    this.q = [design.homeTheta, design.homeTheta, design.homeTheta];
    this.frames = [{ t: 0, q: this.q.slice(), tool: 0 }];
    this.timeLimit = opts.timeLimit || 600;
    this.archH = opts.archH || 0.03;   // default arch height for the demo programs
  }

  get tl() { return this.d.toolLength; }
  eff(p) { return [p[0], p[1], p[2] + this.tl]; }
  get effector() { return fk(this.d, this.q); }
  get position() { const p = this.effector; return [p[0], p[1], p[2] - this.tl]; }

  ik(p) {
    const rep = limitReport(this.d, this.eff(p));
    if (!rep.ok) throw new WorkspaceError(explain(p, rep.problems));
    return rep.theta;
  }
  reachable(p) { return limitReport(this.d, this.eff(p)).ok; }

  advance(t) {
    if (t > this.timeLimit) throw new Error('가상 시간 제한을 넘었습니다');
    this.t = t;
  }

  runCartesian(path, o = {}) {
    const samples = samplePath(path, o.profile || this.profile, o.speed || this.speed, o.accel || this.accel, DT, this.t);
    for (const [t, pe] of samples) {
      const rep = limitReport(this.d, pe);
      if (!rep.ok) throw new WorkspaceError('경로 중간 ' + explain([pe[0], pe[1], pe[2] - this.tl], rep.problems));
      this.frames.push({ t, q: rep.theta, tool: this.tool });
      this.q = rep.theta;
    }
    if (samples.length) this.advance(samples[samples.length - 1][0]);
  }

  moveTo(p, o) { this.ik(p); this.runCartesian(linePath(this.effector, this.eff(p)), o); }
  /** Polyline through all points with ONE speed profile (no stop at the corners). */
  movePath(points, o) {
    const path = new Path();
    let prev = this.effector;
    for (const pt of points) { this.ik(pt); const e = this.eff(pt); path.line(prev, e); prev = e; }
    this.runCartesian(path, o);
  }
  archTo(p, height = 0.03, o = {}) {
    this.ik(p);
    this.runCartesian(archPath(this.effector, this.eff(p), height, o.radius || 0), o);
  }
  moveJoints(q1, o = {}) {
    const s = sampleJoint(this.q, q1, o.profile || this.profile, o.speed || this.jointSpeed, o.accel || this.jointAccel, DT, this.t);
    for (const [t, q] of s) this.frames.push({ t, q, tool: this.tool });
    this.q = q1.slice();
    if (s.length) this.advance(s[s.length - 1][0]);
  }
  home() { const h = this.d.homeTheta; this.moveJoints([h, h, h]); }

  pickZ(part) { return this.d.toolSpec.kind === 'gripper' ? part.z + 0.5 * part.h : part.top; }

  /** Stream TCP positions fn(tau), tau in (0, duration], last sample exactly at duration. */
  runTcpSamples(fn, duration) {
    const n = Math.max(1, Math.ceil(duration / DT - 1e-9));
    const t0 = this.t;
    for (let k = 1; k <= n; k++) {
      const tau = duration * k / n;
      const tcp = fn(tau);
      const rep = limitReport(this.d, this.eff(tcp));
      if (!rep.ok) throw new WorkspaceError('트래킹 중 ' + explain(tcp, rep.problems));
      this.frames.push({ t: t0 + tau, q: rep.theta, tool: this.tool });
      this.q = rep.theta;
    }
    this.advance(t0 + duration);
  }

  /** Conveyor tracking pick — mirror of DeltaRobot.track_pick (python/deltarobot/robot.py). */
  trackPick(pid, o = {}) {
    const hover = o.hover ?? 0.02, td = Math.max(DT, o.descentTime ?? 0.3), dwell = o.dwell ?? 0.1, height = o.height ?? this.archH;
    const find = () => this.parts().find((q) => q.id === pid);
    const p = find();
    if (!p) throw new WorkspaceError(`부품 ${pid} 가 지금 보이지 않습니다`);
    const conv = this.sceneData.conveyor;
    const v = conv && p.on === 'conveyor' ? conv.speed : 0;
    const zc = o.z ?? this.pickZ(p);
    let travel = 0, xh = p.x;
    for (let it = 0; it < 6; it++) {
      xh = p.x + v * (travel + 0.5 * td);
      const path = archPath(this.effector, this.eff([xh, p.y, zc + hover]), height);
      travel = new Profile(this.profile, path.length, this.speed, this.accel).T;
    }
    xh = p.x + v * (travel + 0.5 * td);
    this.archTo([xh, p.y, zc + hover], height);
    const y = p.y;
    const q5 = new Profile('quintic', 1, 1, 1);
    const s = (u) => q5.at(u * q5.T)[0];
    this.runTcpSamples((tau) => [xh + 0.5 * v * (tau - td / Math.PI * Math.sin(Math.PI * tau / td)), y, zc + hover * (1 - s(tau / td))], td);
    const got = this.toolOn();
    const x1 = xh + 0.5 * v * td;
    if (dwell > 0) this.runTcpSamples((tau) => [x1 + v * tau, y, zc], dwell);
    const x2 = x1 + v * Math.max(0, dwell);
    this.runTcpSamples((tau) => [x2 + 0.5 * v * (tau + td / Math.PI * Math.sin(Math.PI * tau / td)), y, zc + hover * s(tau / td)], td);
    return got;
  }
  wait(sec) {
    const end = this.t + sec;
    let t = this.t;
    while (t < end - 1e-9) { t = Math.min(end, t + HOLD_DT); this.frames.push({ t, q: this.q.slice(), tool: this.tool }); }
    this.advance(end);
  }
  toolOn() {
    this.tool = 1;
    const id = this.scene.grab(this.position, this.t, this.d.toolSpec);
    this.frames.push({ t: this.t, q: this.q.slice(), tool: 1 });
    return id;
  }
  toolOff() {
    this.tool = 0;
    const id = this.scene.release(this.position, this.t);
    this.frames.push({ t: this.t, q: this.q.slice(), tool: 0 });
    return id;
  }
  parts() {
    return this.scene.visible(this.t).map(({ part, x, y, z }) => ({ id: part.id, x, y, z, top: z + part.h, size: part.size,
      h: part.h, color: part.color, material: part.material, on: part.mode }));
  }
  /** Estimated duration of an arch move to p (used for conveyor interception). */
  archDuration(p, height) {
    const path = archPath(this.effector, this.eff(p), height);
    return new Profile(this.profile, path.length, this.speed, this.accel).T;
  }
  timeline(name) {
    return { type: 'timeline', name, design: this.d.toDict(), scene: this.sceneData, frames: this.frames,
      events: this.scene.events, duration: this.t };
  }
}
