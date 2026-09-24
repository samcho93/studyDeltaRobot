// programs.js — demo programs for the 작업 tab (built with the JS Planner).
import { Planner, WorkspaceError } from './planner.js';

export const PROGRAMS = {
  adept: { name: 'Adept 사이클 (왕복 3회)', scene: null },
  sort: { name: '분류 데모 (색깔별 상자)', scene: 'pick_place' },
  conveyor: { name: '컨베이어 트래킹 데모', scene: 'conveyor' },
  circle: { name: '원 그리기', scene: 'drawing' },
  star: { name: '별 그리기', scene: 'drawing' },
};

/** Standard cycle used by the design check: rise h, across a, descend, and back. */
export function adeptGeometry(scene) {
  const rc = scene.work_radius || 0.1;
  const across = Math.min(0.305, 1.4 * rc);
  const up = Math.min(0.025, 0.25 * across);
  const z = (scene.surface_z || -0.3) + 0.02;
  return { across, up, z };
}

export function runAdept(p, scene, cycles = 3) {
  const { across, up, z } = adeptGeometry(scene);
  const a = [-across / 2, 0, z], b = [across / 2, 0, z];
  p.moveTo(a);
  const t0 = p.t;
  for (let k = 0; k < cycles; k++) {
    p.archTo(b, up);
    p.archTo(a, up);
  }
  return { cycleTime: (p.t - t0) / cycles, across, up };
}

function sortDemo(p) {
  const tool = p.d.toolSpec;
  if (tool.kind === 'pen') throw new WorkspaceError('펜 툴로는 부품을 집을 수 없습니다 — 설계 탭에서 흡착컵·그리퍼·전자석을 고르세요');
  const bins = p.sceneData.bins || [];
  const clear = 0.02;
  let done = 0;
  for (const part of p.parts()) {
    if (tool.ferrous_only && part.material !== 'steel') continue;
    const bin = bins.find((b) => b.color === part.color) || bins[0];
    if (!bin) break;
    const pick = [part.x, part.y, part.top];
    if (tool.kind === 'gripper') pick[2] = part.z + part.h * 0.5;
    p.archTo(pick, p.archH);
    p.toolOn();
    p.wait(0.1);
    p.archTo([bin.x, bin.y, p.sceneData.surface_z + bin.h + part.h + clear], p.archH);
    p.toolOff();
    p.wait(0.1);
    done++;
  }
  p.home();
  return { picked: done };
}

function conveyorDemo(p, until = 30) {
  const tool = p.d.toolSpec;
  if (tool.kind === 'pen') throw new WorkspaceError('펜 툴로는 부품을 집을 수 없습니다');
  if (tool.ferrous_only) throw new WorkspaceError('전자석은 철(steel) 부품만 잡을 수 있는데 컨베이어 부품은 플라스틱입니다 — 흡착컵이나 그리퍼를 고르세요');
  const conv = p.sceneData.conveyor;
  const bins = p.sceneData.bins || [];
  const rc = p.sceneData.work_radius;
  const tried = new Set();
  const lead = 1.2;   // rough look-ahead [s] to decide which part to chase
  while (p.t < until) {
    // most downstream part that will still be reachable a little later
    const target = p.parts().filter((q) => q.on === 'conveyor' && !tried.has(q.id)).sort((a, b) => b.x - a.x)
      .find((q) => { const x = q.x + conv.speed * lead; return x < 0.9 * rc && p.reachable([x, q.y, p.pickZ(q)]); });
    if (!target) { p.wait(0.1); continue; }
    tried.add(target.id);
    let got = null;
    try { got = p.trackPick(target.id); } catch (e) { if (!(e instanceof WorkspaceError)) throw e; continue; }
    if (!got) { p.toolOff(); continue; }
    const bin = bins.find((b) => b.color === target.color) || bins[0];
    p.archTo([bin.x, bin.y, p.sceneData.surface_z + bin.h + target.h + 0.02], p.archH);
    p.toolOff();
  }
  return { score: p.scene.score() };
}

function drawShape(p, shape) {
  const rc = p.sceneData.work_radius;
  const z = p.sceneData.surface_z + 0.001;
  const R = 0.55 * rc;
  const pts = [];
  if (shape === 'circle') {
    for (let k = 0; k <= 72; k++) { const a = 2 * Math.PI * k / 72; pts.push([R * Math.cos(a), R * Math.sin(a), z]); }
  } else {
    for (let k = 0; k <= 10; k++) {
      const a = Math.PI / 2 + Math.PI * k / 5;
      const r = k % 2 === 0 ? R : 0.4 * R;
      pts.push([r * Math.cos(a), r * Math.sin(a), z]);
    }
  }
  p.archTo(pts[0], 0.02);
  p.toolOn();
  const slow = { speed: Math.min(p.speed, 0.1), accel: Math.min(p.accel, 1.0) };
  if (shape === 'circle') p.movePath(pts.slice(1), slow);            // one continuous constant-speed pass
  else for (let k = 1; k < pts.length; k++) p.moveTo(pts[k], slow);  // stop at every corner
  p.toolOff();
  p.moveTo([pts[0][0], pts[0][1], z + 0.03]);
  return { points: pts.length };
}

/** Build a timeline for one demo; throws WorkspaceError with a readable message. */
export function buildProgram(name, design, scene, opts) {
  const p = new Planner(design, scene, opts);
  p.home();
  let info = {};
  if (name === 'adept') info = runAdept(p, scene);
  else if (name === 'sort') info = sortDemo(p);
  else if (name === 'conveyor') info = conveyorDemo(p);
  else if (name === 'circle' || name === 'star') info = drawShape(p, name);
  const tl = p.timeline(name);
  tl.info = info;
  tl.finalScore = p.scene.score();
  return tl;
}
