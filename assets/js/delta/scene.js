// scene.js — work cell (table, parts, bins, conveyor). Mirror of python/deltarobot/scene.py.
import { workCylinder } from './kinematics.js';

export const GRAB_Z_TOL = 0.012;
const r4 = (v) => Math.round(v * 1e4) / 1e4;

export const SCENE_KINDS = { empty: '빈 작업대', pick_place: '분류 (pick & place)', conveyor: '컨베이어 트래킹', drawing: '그리기 (펜)' };

export function defaultScene(d, kind = 'pick_place', cyl = null) {
  const height = 0.25 * (d.upper_arm + d.forearm);
  cyl = cyl || workCylinder(d, height);
  const rc = Math.max(0.02, cyl.diameter / 2);
  const surface = cyl.z_bottom - d.toolLength;
  const size = Math.min(0.06, Math.max(0.015, 0.12 * rc));
  const h = 0.6 * size;
  const scene = { kind, surface_z: r4(surface), work_radius: r4(rc), work_top: r4(cyl.z_top - d.toolLength),
    parts: [], bins: [], conveyor: null };
  const colors = ['red', 'blue', 'green'];
  if (kind === 'pick_place') {
    let k = 0;
    for (let row = 0; row < 2; row++) {
      for (let col = 0; col < 3; col++) {
        scene.parts.push({ id: 'p' + (k + 1), x: r4(-0.55 * rc + row * 0.22 * rc), y: r4((col - 1) * 0.3 * rc),
          size: r4(size), h: r4(h), color: colors[col], material: row === 1 ? 'steel' : 'plastic' });
        k++;
      }
    }
    colors.forEach((c, j) => scene.bins.push({ id: 'ABC'[j], x: r4(0.5 * rc), y: r4((j - 1) * 0.36 * rc),
      w: r4(0.3 * rc), d: r4(0.3 * rc), h: r4(0.8 * h), color: c }));
  } else if (kind === 'conveyor') {
    const offs = [0, 0.12, -0.12, 0.06, -0.06];
    const spawn = [];
    for (let n = 0; n < 30; n++) {
      spawn.push({ t: r4(1 + 2 * n), y_off: r4(offs[n % 5] * rc), color: colors[n % 2], material: 'plastic' });
    }
    scene.conveyor = { y: r4(-0.45 * rc), width: r4(0.45 * rc), x_start: r4(-1.3 * rc), x_end: r4(1.3 * rc),
      speed: r4(0.2 * rc), size: r4(size), h: r4(h), spawn };
    colors.slice(0, 2).forEach((c, j) => scene.bins.push({ id: 'AB'[j], x: r4((j - 0.5) * 0.7 * rc), y: r4(0.5 * rc),
      w: r4(0.35 * rc), d: r4(0.3 * rc), h: r4(0.8 * h), color: c }));
  } else if (kind === 'drawing') {
    scene.paper = { x: 0, y: 0, w: r4(1.4 * rc), d: r4(1.4 * rc) };
  }
  return scene;
}

export class SceneState {
  constructor(data) {
    this.data = data ? JSON.parse(JSON.stringify(data)) : { kind: 'empty', surface_z: -1, parts: [], bins: [], conveyor: null };
    this.surface_z = Number(this.data.surface_z ?? -1);
    this.parts = (this.data.parts || []).map((p) => ({ ...p, mode: 'table', z: this.surface_z }));
    const conv = this.data.conveyor;
    if (conv) {
      (conv.spawn || []).forEach((s, n) => this.parts.push({
        id: 'c' + (n + 1), size: conv.size, h: conv.h, color: s.color || 'red', material: s.material || 'plastic',
        mode: 'conveyor', x0: conv.x_start, t0: s.t, y: conv.y + (s.y_off || 0), z: this.surface_z,
      }));
    }
    this.held = null;
    this.events = [];
  }

  pos(p, t) {
    if (p.mode === 'gone' || p.mode === 'held') return null;
    if (p.mode === 'conveyor') {
      if (t < p.t0) return null;
      const conv = this.data.conveyor;
      const x = p.x0 + conv.speed * (t - p.t0);
      if (x > conv.x_end) return null;
      return [x, p.y, p.z];
    }
    return [p.x, p.y, p.z];
  }

  visible(t) {
    const out = [];
    for (const p of this.parts) {
      const q = this.pos(p, t);
      if (q) out.push({ part: p, x: q[0], y: q[1], z: q[2] });
    }
    return out;
  }

  grab(tcp, t, tool) {
    if (this.held || tool.kind === 'pen') return null;
    let best = null, bestD = 1e9;
    for (const p of this.parts) {
      const q = this.pos(p, t);
      if (!q) continue;
      if (tool.ferrous_only && p.material !== 'steel') continue;
      const dxy = Math.hypot(tcp[0] - q[0], tcp[1] - q[1]);
      const top = q[2] + p.h;
      const zok = tool.kind === 'gripper'
        ? (q[2] - 0.002 <= tcp[2] && tcp[2] <= top + GRAB_Z_TOL)
        : Math.abs(tcp[2] - top) <= GRAB_Z_TOL;
      if (zok && dxy <= Math.max(0.012, 0.45 * p.size) && dxy < bestD) { best = p; bestD = dxy; }
    }
    if (!best) { this.events.push({ t, type: 'grab_miss' }); return null; }
    best.mode = 'held';
    this.held = best;
    this.events.push({ t, type: 'grab', id: best.id });
    return best.id;
  }

  release(tcp, t) {
    const p = this.held;
    if (!p) return null;
    this.held = null;
    const x = tcp[0], y = tcp[1];
    p.x = x; p.y = y;
    for (const b of this.data.bins || []) {
      if (Math.abs(x - b.x) <= b.w / 2 && Math.abs(y - b.y) <= b.d / 2) {
        Object.assign(p, { mode: 'bin', z: this.surface_z + 0.003, bin: b.id, bin_color: b.color });
        this.events.push({ t, type: 'place', id: p.id, bin: b.id });
        return p.id;
      }
    }
    const conv = this.data.conveyor;
    if (conv && Math.abs(y - conv.y) <= conv.width / 2 && x >= conv.x_start && x <= conv.x_end) {
      Object.assign(p, { mode: 'conveyor', x0: x, t0: t, y, z: this.surface_z });
    } else {
      Object.assign(p, { mode: 'table', z: this.surface_z });
    }
    this.events.push({ t, type: 'drop', id: p.id });
    return p.id;
  }

  score() {
    const per = {};
    for (const b of this.data.bins || []) per[b.id] = 0;
    let correct = 0, total = 0;
    for (const p of this.parts) {
      if (p.mode === 'bin') {
        per[p.bin] = (per[p.bin] || 0) + 1;
        total++;
        if (p.bin_color === p.color) correct++;
      }
    }
    return { per_bin: per, correct, total };
  }
}
