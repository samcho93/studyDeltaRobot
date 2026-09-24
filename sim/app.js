// app.js — delta robot simulator: design / jog / task / analysis / link tabs.
import { Design, loadCatalog, summary, G } from '../assets/js/delta/design.js';
import * as K from '../assets/js/delta/kinematics.js';
import { analyze, evaluate, jointTorques } from '../assets/js/delta/dynamics.js';
import { PROFILES, PROFILE_KO } from '../assets/js/delta/trajectory.js';
import { defaultScene, SceneState, SCENE_KINDS } from '../assets/js/delta/scene.js';
import { generate as generateUrdf } from '../assets/js/delta/urdf.js';
import { DeltaView } from './render.js';
import { TimeChart } from './charts.js';
import { buildProgram, PROGRAMS, runAdept } from './programs.js';
import { Planner } from './planner.js';
import { Joystick, stepTcp } from '../assets/js/joystick.js';

const LS_KEY = 'studydelta.design.v1';
const LS_OPTS = 'studydelta.sim.opts.v1';
const DEG = 180 / Math.PI;
const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const EMBED = params.get('embed') === '1';
if (EMBED) document.body.classList.add('embed');

const lsGet = (k) => { try { return JSON.parse(localStorage.getItem(k) || 'null'); } catch (e) { return null; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* private mode */ } };
const b64url = (s) => btoa(String.fromCharCode(...new TextEncoder().encode(s))).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
const unb64url = (s) => { let b = s.replace(/-/g, '+').replace(/_/g, '/'); while (b.length % 4) b += '='; return new TextDecoder().decode(Uint8Array.from(atob(b), (c) => c.charCodeAt(0))); };
const fmt = (v, n = 3) => (Number.isFinite(v) ? v.toFixed(n) : '—');

// ------------------------------------------------------------------ state
const S = {
  cat: null, design: null, presetName: null, sceneKind: 'pick_place', sceneData: null, cyl: null,
  q: [0.35, 0.35, 0.35], tool: 0,
  timeline: null, t: 0, playing: false, rate: 1, frameIdx: 0, sceneState: null,
  analysis: null, evalRes: null,
  jogTime: 0, live: null, ws: null,
  opts: { profile: 'scurve', speed: null, accel: null, archH: 30 },
};

let view;
function toast(msg, bad = false, ms = 3500) {
  const t = $('toast');
  t.textContent = msg; t.className = 'd-toast' + (bad ? ' bad' : ''); t.hidden = false;
  clearTimeout(toast._h);
  toast._h = setTimeout(() => { t.hidden = true; }, ms);
}

// ------------------------------------------------------------------ boot
async function boot() {
  try {
    const probe = document.createElement('canvas');
    if (!(probe.getContext('webgl2') || probe.getContext('webgl'))) throw new Error('WebGL unavailable');
  } catch (e) {
    const box = $('noGL'); box.hidden = false;
    box.textContent = '이 브라우저에서 WebGL을 사용할 수 없어 3D 화면을 그릴 수 없습니다. Chrome/Edge 최신 버전에서 하드웨어 가속을 켜 주세요.';
    return;
  }
  S.cat = await loadCatalog('../python/deltarobot/data/catalog.json');
  view = new DeltaView($('viewport'));
  if (params.get('model') === 'mesh') $('optUrdf').checked = false;
  view.useUrdf = $('optUrdf').checked;
  $('optUrdf').addEventListener('change', () => {
    view.useUrdf = $('optUrdf').checked;
    view.build(S.design, S.sceneData);
    refreshOverlays();
    toast(view.useUrdf ? (view.urdfError ? 'URDF 모델을 만들지 못해 기본 모델로 그립니다: ' + view.urdfError
      : 'URDF 모델로 그립니다 — URDF 뷰어·ROS 2 RViz와 같은 모델') : '기본(three.js 도형) 모델로 그립니다');
  });
  window.deltaSim = { view, state: S };   // debugging handle (browser console)

  // initial design: #design= > ?preset= > localStorage > default
  let init = null;
  const m = location.hash.match(/design=([^&]+)/);
  if (m) { try { init = JSON.parse(unb64url(m[1])); } catch (e) { toast('공유 링크의 설계를 읽지 못했습니다', true); } }
  if (!init && params.get('preset') && S.cat.presets[params.get('preset')]) {
    init = { preset: params.get('preset'), design: S.cat.presets[params.get('preset')].design };
  }
  if (!init) init = lsGet(LS_KEY);
  if (!init || !init.design) init = { preset: 'edu_dynamixel', design: S.cat.presets.edu_dynamixel.design };
  S.presetName = init.preset && S.cat.presets[init.preset] ? init.preset : null;
  S.sceneKind = SCENE_KINDS[params.get('scene')] ? params.get('scene') : (SCENE_KINDS[init.scene] ? init.scene : 'pick_place');
  const saved = lsGet(LS_OPTS);
  if (saved) Object.assign(S.opts, saved);

  buildDesignForm();
  buildTaskForm();
  setupTabs();
  setupPlayer();
  setupLink();
  setupOverlays();
  setupJoystick();
  applyDesign(new Design(init.design), { fit: true });
  requestAnimationFrame(loop);
  if (EMBED) setupEmbed();
}

// ------------------------------------------------------------------ tabs
let activeTab = 'design';
function setupTabs() {
  document.querySelectorAll('.tabs button').forEach((btn) => btn.addEventListener('click', () => {
    activeTab = btn.dataset.tab;
    document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('active', b === btn));
    document.querySelectorAll('.tab-body').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + activeTab));
    if (activeTab === 'jog') enterJog();
    if (activeTab === 'analysis') { drawCylProfile(); drawCharts(); }
  }));
  $('btnFit').addEventListener('click', () => view.fit());
}

// ------------------------------------------------------------------ design form
const DIMS = [
  ['base_radius', 'R', '베이스 반지름', 30, 400],
  ['effector_radius', 'r', '이펙터 반지름', 15, 150],
  ['upper_arm', 'L', '위팔', 50, 600],
  ['forearm', 'l', '아래팔 로드', 100, 1500],
  ['forearm_spacing', 'w', '로드 간격', 20, 150],
];
const LIMS = [
  ['theta_min', 'θmin', '모터각 하한 (°)', -80, 0],
  ['theta_max', 'θmax', '모터각 상한 (°)', 30, 110],
  ['ball_joint_limit', 'β', '볼조인트 한계 (°)', 15, 60],
];

function sliderRow(host, key, sym, ko, lo, hi, step, onChange) {
  const row = document.createElement('div');
  row.className = 'joint-row';
  row.innerHTML = `<div class="jr-head"><span class="jr-name">${sym}</span><span class="jr-ko">${ko}</span>
    <input class="d-num" type="number" step="${step}" min="${lo}" max="${hi}"></div>
    <input type="range" min="${lo}" max="${hi}" step="${step}">`;
  const num = row.querySelector('.d-num'), rng = row.querySelector('input[type=range]');
  const set = (v) => { num.value = String(Math.round(v / step) * step).replace(/(\.\d*?)0+$/, '$1').replace(/\.$/, ''); rng.value = v; };
  rng.addEventListener('input', () => { num.value = rng.value; onChange(Number(rng.value), false); });
  rng.addEventListener('change', () => onChange(Number(rng.value), true));
  num.addEventListener('change', () => { const v = Math.min(hi, Math.max(lo, Number(num.value))); set(v); onChange(v, true); });
  host.appendChild(row);
  return { row, set, rng, num };
}

const formCtl = {};
let designTimer = null;
function scheduleDesign(changes, commit) {
  S.pending = { ...(S.pending || {}), ...changes };
  clearTimeout(designTimer);
  designTimer = setTimeout(() => {
    const d = S.design.copy(S.pending);
    S.pending = null;
    S.presetName = null;
    $('preset').value = '';
    applyDesign(d);
  }, commit ? 0 : 180);
}

function buildDesignForm() {
  const cat = S.cat;
  const pre = $('preset');
  pre.innerHTML = '<option value="">— 사용자 정의 —</option>' +
    Object.entries(cat.presets).map(([k, p]) => `<option value="${k}">${p.name}</option>`).join('');
  pre.addEventListener('change', () => {
    if (!pre.value) return;
    S.presetName = pre.value;
    applyDesign(new Design(cat.presets[pre.value].design), { fit: true });
  });
  for (const [key, sym, ko, lo, hi] of DIMS) {
    formCtl[key] = sliderRow($('dimSliders'), key, sym, ko, lo, hi, 1, (v, c) => scheduleDesign({ [key]: v / 1000 }, c));
  }
  for (const [key, sym, ko, lo, hi] of LIMS) {
    formCtl[key] = sliderRow($('limSliders'), key, sym, ko, lo, hi, 1, (v, c) => scheduleDesign({ [key]: v / DEG }, c));
  }
  const fill = (sel, obj, label = (v) => v.name) => {
    $(sel).innerHTML = Object.entries(obj).map(([k, v]) => `<option value="${k}">${label(v)}</option>`).join('');
  };
  fill('matUpper', cat.materials, (m) => `${m.name} · ${(m.linear_density * 1000).toFixed(0)} g/m`);
  fill('matFore', cat.materials, (m) => `${m.name} · ${(m.linear_density * 1000).toFixed(0)} g/m`);
  fill('motor', cat.motors);
  fill('gearbox', cat.gearboxes);
  fill('tool', cat.tools);
  $('matUpper').addEventListener('change', (e) => scheduleDesign({ upper_arm_material: e.target.value }, true));
  $('matFore').addEventListener('change', (e) => scheduleDesign({ forearm_material: e.target.value }, true));
  $('motor').addEventListener('change', (e) => {
    const m = cat.motors[e.target.value];
    const ch = { motor: e.target.value };
    if (m.integrated_gear) Object.assign(ch, { gearbox: 'none', gear_ratio: 1 });
    scheduleDesign(ch, true);
  });
  $('gearbox').addEventListener('change', (e) => {
    const gb = cat.gearboxes[e.target.value];
    const r = gb.ratios.includes(Number(S.design.gear_ratio)) ? Number(S.design.gear_ratio) : gb.ratios[Math.floor(gb.ratios.length / 2)];
    scheduleDesign({ gearbox: e.target.value, gear_ratio: r }, true);
  });
  $('ratio').addEventListener('change', (e) => scheduleDesign({ gear_ratio: Number(e.target.value) }, true));
  $('tool').addEventListener('change', (e) => scheduleDesign({ tool: e.target.value }, true));
  $('payload').addEventListener('change', (e) => scheduleDesign({ payload: Math.max(0, Number(e.target.value)) }, true));
  $('effMass').addEventListener('change', (e) => scheduleDesign({ effector_mass: Math.max(0.005, Number(e.target.value)) }, true));
  $('btnCheck').addEventListener('click', runDesignCheck);
  $('btnResetDesign').addEventListener('click', () => {
    const k = S.presetName || 'edu_dynamixel';
    S.presetName = k;
    applyDesign(new Design(S.cat.presets[k].design), { fit: true });
  });
}

function paintDesignForm() {
  const d = S.design;
  $('preset').value = S.presetName || '';
  $('presetDesc').textContent = S.presetName ? S.cat.presets[S.presetName].desc : '값을 직접 바꾼 설계입니다.';
  for (const [key] of DIMS) formCtl[key].set(Math.round(d[key] * 1000));
  for (const [key] of LIMS) formCtl[key].set(Math.round(d[key] * DEG));
  $('matUpper').value = d.upper_arm_material;
  $('matFore').value = d.forearm_material;
  $('motor').value = d.motor;
  $('gearbox').value = d.gearbox;
  const integrated = d.motorSpec.integrated_gear;
  $('gearbox').disabled = !!integrated;
  $('ratio').disabled = !!integrated;
  $('ratio').innerHTML = d.gearboxSpec.ratios.map((r) => `<option value="${r}">${r} : 1</option>`).join('');
  $('ratio').value = String(d.gear_ratio);
  $('tool').value = d.tool;
  $('payload').value = d.payload;
  $('effMass').value = d.effector_mass;
  const m = d.motorSpec;
  $('motorDesc').textContent = m.desc + ' (' + m.control + ', ' + m.voltage + ') — 대표값, 데이터시트 확인';
  $('toolDesc').textContent = d.toolSpec.desc;
  const n = d.ratio, eta = d.efficiency;
  const rpm = (w) => (w * 60 / (2 * Math.PI)).toFixed(0);
  $('driveTable').innerHTML =
    '<tr><th></th><th>모터축</th><th>출력축(×감속)</th></tr>' +
    `<tr><td>정격 토크</td><td>${m.rated_torque.toFixed(2)} N·m</td><td>${(m.rated_torque * n * eta).toFixed(2)} N·m</td></tr>` +
    `<tr><td>피크 토크</td><td>${m.peak_torque.toFixed(2)} N·m</td><td>${(m.peak_torque * n * eta).toFixed(2)} N·m</td></tr>` +
    `<tr><td>최고 속도</td><td>${rpm(m.max_speed)} rpm</td><td>${rpm(m.max_speed / n)} rpm</td></tr>` +
    `<tr><td>분해능</td><td>${m.resolution_deg}°</td><td>${(m.resolution_deg / n).toPrecision(2)}°</td></tr>` +
    (integrated ? '<tr><td colspan="3">일체형 서보: 감속기 내장 — 외부 감속기 없음</td></tr>' :
      `<tr><td>감속기</td><td colspan="2">${d.gearboxSpec.name} ${n}:1, 효율 ${(eta * 100).toFixed(0)}%, 백래시 ${d.gearboxSpec.backlash_arcmin}′</td></tr>`);
}

function paintDerived() {
  const d = S.design, s = summary(d);
  const cyl = S.cyl;
  const staticTau = (() => {
    try { const p = K.fk(d, [d.homeTheta, d.homeTheta, d.homeTheta]); return jointTorques(d, [d.homeTheta, d.homeTheta, d.homeTheta], [0, 0, 0], p)[0]; } catch (e) { return NaN; }
  })();
  const kv = [
    ['위팔 질량', (s.upper_arm_mass * 1000).toFixed(0) + ' g'],
    ['로드 한 쌍', (s.forearm_pair_mass * 1000).toFixed(0) + ' g'],
    ['이동부 질량 m_p', (s.moving_plate_mass * 1000).toFixed(0) + ' g'],
    ['팔 관성 (출력축)', s.arm_inertia.toExponential(2) + ' kg·m²'],
    ['홈 자세 정적 토크', fmt(staticTau, 3) + ' N·m (출력)'],
    ['감속비·효율', `${s.ratio} : 1 · ${(s.efficiency * 100).toFixed(0)}%`],
    ['작업 실린더 D×H', cyl ? `${(cyl.diameter * 1000).toFixed(0)} × ${(cyl.height * 1000).toFixed(0)} mm` : '—'],
    ['툴 길이 (TCP)', (d.toolLength * 1000).toFixed(0) + ' mm'],
  ];
  $('derived').innerHTML = kv.map(([k, v]) => `<div><span>${k}</span><code>${v}</code></div>`).join('');
}

// ------------------------------------------------------------------ apply design
function applyDesign(d, o = {}) {
  const errs = d.validate();
  const hard = errs.filter((e) => !e.includes('무시') && !e.includes('짧으면'));
  const box = $('designErr');
  if (errs.length) { box.hidden = false; box.textContent = errs.join(' · '); } else box.hidden = true;
  if (hard.length) {
    if (S.design) { paintDesignForm(); }
    toast('설계 오류: ' + hard[0], true);
    return false;
  }
  const oldSize = S.design ? S.design.upper_arm + S.design.forearm : 0;
  S.design = d;
  S.cyl = K.workCylinder(d, 0.25 * (d.upper_arm + d.forearm));
  S.sceneData = o.sceneData || defaultScene(d, S.sceneKind, S.cyl);
  S.sceneKind = S.sceneData.kind || S.sceneKind;
  view.build(d, S.sceneData);
  if (o.fit || Math.abs(oldSize - (d.upper_arm + d.forearm)) > 0.02) view.fit();
  S.q = [d.homeTheta, d.homeTheta, d.homeTheta];
  S.tool = 0;
  S.sceneState = new SceneState(S.sceneData);
  if (!o.keepTimeline) clearTimeline();
  if (!S.opts.speedUser) { S.opts.speed = +(0.25 * (d.upper_arm + d.forearm) / 0.43).toFixed(3); S.opts.accel = +(10 * S.opts.speed).toFixed(2); }
  paintTaskForm();
  paintDesignForm();
  paintDerived();
  buildJogSliders();
  refreshOverlays();
  $('checkResult').innerHTML = '';
  $('subTitle').textContent = `${S.presetName ? S.cat.presets[S.presetName].name : '사용자 설계'} · ${d.motorSpec.name} · ${d.toolSpec.name}`;
  lsSet(LS_KEY, { preset: S.presetName, design: d.toDict(), scene: S.sceneKind });
  paintWorkspaceInfo();
  selectedPart = null;
  if ($('binBtns')) paintBins();
  if (EMBED) postParent({ type: 'sim-design', design: d.toDict(), scene: S.sceneData });
  if (S.ws && S.ws.readyState === 1 && !o.fromLink) S.ws.send(JSON.stringify({ type: 'scene', scene: S.sceneData }));
  return true;
}

// ------------------------------------------------------------------ overlays
let cloudCache = null;
function setupOverlays() {
  ['optCloud', 'optCyl', 'optPath', 'optTrail', 'optFrames'].forEach((id) => $(id).addEventListener('change', refreshOverlays));
  if (params.get('frames') === '1') $('optFrames').checked = true;
}
function refreshOverlays() {
  if (!view || !S.design) return;
  if ($('optCloud').checked) {
    if (!cloudCache || cloudCache.d !== S.design) {
      toast('작업영역을 계산하는 중…', false, 1200);
      setTimeout(() => { cloudCache = { d: S.design, pts: K.workspacePoints(S.design, 0) }; view.showWorkspace(cloudCache.pts); paintWorkspaceInfo(); }, 30);
    } else view.showWorkspace(cloudCache.pts);
  } else view.showWorkspace(null);
  view.showCylinder($('optCyl').checked ? S.cyl : null);
  view.framesOn = $('optFrames').checked;
  view.showJointFrames(view.framesOn);
  showPathPreview();
}
function showPathPreview() {
  if (!$('optPath').checked || !S.timeline) { view.setPathPreview(null); return; }
  const d = S.design, pts = [];
  const fr = S.timeline.frames;
  const step = Math.max(1, Math.floor(fr.length / 1500));
  for (let i = 0; i < fr.length; i += step) {
    const p = K.tryFk(d, fr[i].q);
    if (p) pts.push([p[0], p[1], p[2] - d.toolLength]);
  }
  view.setPathPreview(pts);
}

function paintWorkspaceInfo() {
  const c = S.cyl, d = S.design;
  const items = [
    ['작업 실린더 지름', (c.diameter * 1000).toFixed(0) + ' mm'],
    ['높이', (c.height * 1000).toFixed(0) + ' mm'],
    ['TCP z 범위', `${((c.z_top - d.toolLength) * 1000).toFixed(0)} ~ ${((c.z_bottom - d.toolLength) * 1000).toFixed(0)} mm`],
    ['작업대 높이', (S.sceneData.surface_z * 1000).toFixed(0) + ' mm'],
  ];
  if (cloudCache && cloudCache.d === d) {
    const [rad] = K.workspaceBounds(d);
    const h = rad / 14;
    items.push(['작업영역 부피', ((cloudCache.pts.length / 3) * h ** 3 * 1e3).toFixed(1) + ' L (격자 근사)']);
  }
  $('wsInfo').innerHTML = items.map(([k, v]) => `<div><span>${k}</span><code>${v}</code></div>`).join('');
}

function drawCylProfile() {
  const cv = $('chartCyl');
  const ch = new TimeChart(cv, { title: '높이 z(TCP)별 최대 반지름 — 가로축 z', unit: 'mm', labels: ['반지름'], xUnit: ' mm' });
  const prof = S.cyl.profile.slice().reverse();
  const tl = S.design.toolLength;
  ch.setData(prof.map((p) => (p[0] - tl) * 1000), [prof.map((p) => p[1] * 1000)]);
  ch.setLimits([{ v: S.cyl.diameter / 2 * 1000, label: '작업 실린더 반지름', color: '#38bdf8' }]);
  ch.draw();
}

// ------------------------------------------------------------------ jog
const jog = { xyz: [], q: [] };
function buildJogSliders() {
  const d = S.design;
  const [rad] = K.workspaceBounds(d);
  const tl = d.toolLength;
  const zTop = Math.round((S.cyl.z_top - tl + 0.4 * S.cyl.height) * 1000);
  const zBot = Math.round((S.sceneData.surface_z - 0.02) * 1000);
  $('xyzSliders').innerHTML = '';
  $('qSliders').innerHTML = '';
  jog.xyz = ['x', 'y', 'z'].map((ax, i) => {
    const lo = i < 2 ? -Math.round(rad * 1000) : zBot, hi = i < 2 ? Math.round(rad * 1000) : Math.min(zTop, -10);
    return sliderRow($('xyzSliders'), ax, ax, 'TCP ' + ax, lo, hi, 1, (v) => jogXYZ(i, v / 1000));
  });
  jog.q = [0, 1, 2].map((i) => sliderRow($('qSliders'), 'q' + i, 'θ' + (i + 1), '모터 ' + (i + 1),
    Math.round(d.theta_min * DEG), Math.round(d.theta_max * DEG), 0.1, (v) => jogQ(i, v / DEG)));
  paintJog();
}
let framesAuto = false;
function enterJog() {
  if (!framesAuto && !$('optFrames').checked) {   // show the axis conventions the first time
    framesAuto = true;
    $('optFrames').checked = true;
    refreshOverlays();
  }
  if (S.live) return;
  pause();
  startJogMode();
  paintBins();
  paintJog();
}
/** Jog owns the pose (a loaded timeline no longer overrides it) until ▶ is pressed again. */
function startJogMode() {
  if (S.jogMode) return;
  S.jogMode = true;
  if (S.timeline) S.jogTime = S.t;         // keep conveyor parts where the playback left them
}
function currentTcp() { const p = K.tryFk(S.design, S.q); return p ? [p[0], p[1], p[2] - S.design.toolLength] : null; }
function jogXYZ(i, v) {
  if (S.live) { toast('PC가 로봇을 제어 중입니다', true); return; }
  pause();
  startJogMode();
  S.anim = null;
  const tcp = currentTcp() || [0, 0, -0.3];
  tcp[i] = v;
  const rep = K.limitReport(S.design, [tcp[0], tcp[1], tcp[2] + S.design.toolLength]);
  if (rep.theta) S.q = rep.theta;
  const msg = $('jogMsg');
  if (!rep.ok) {
    msg.className = 'd-msg bad';
    msg.textContent = rep.theta ? '한계 초과: ' + rep.problems.join(', ') + ' — 실제 로봇은 이 위치로 갈 수 없습니다' : '팔 길이로 닿지 않는 위치입니다';
  } else { msg.className = 'd-msg ok'; msg.textContent = '도달 가능'; }
  S.jogBad = rep.ok ? null : rep;
  paintJog(i);
}
function jogQ(i, v) {
  if (S.live) return;
  pause();
  startJogMode();
  S.anim = null;
  const q = S.q.slice(); q[i] = v;
  if (!K.tryFk(S.design, q)) { $('jogMsg').className = 'd-msg bad'; $('jogMsg').textContent = '이 모터각 조합으로는 조립할 수 없습니다 (순기구학 해 없음)'; return; }
  S.q = q;
  const tcp = currentTcp();
  const rep = K.limitReport(S.design, [tcp[0], tcp[1], tcp[2] + S.design.toolLength]);
  S.jogBad = rep.ok ? null : rep;
  $('jogMsg').className = rep.ok ? 'd-msg ok' : 'd-msg bad';
  $('jogMsg').textContent = rep.ok ? '도달 가능' : '한계 초과: ' + rep.problems.join(', ');
  paintJog(null, i);
}
function paintJog(skipXyz = null, skipQ = null) {
  if (!jog.q.length) return;
  const d = S.design;
  const tcp = currentTcp();
  if (tcp) jog.xyz.forEach((c, i) => { if (i !== skipXyz) c.set(Math.round(tcp[i] * 1000)); });
  jog.q.forEach((c, i) => { if (i !== skipQ) c.set(Math.round(S.q[i] * DEG * 10) / 10); });
  if (!tcp) return;
  const p = [tcp[0], tcp[1], tcp[2] + d.toolLength];
  const rep = K.limitReport(d, p);
  const cond = K.conditionNumber(d, S.q, p);
  const pa = K.passiveAngles(d, S.q, p);
  let tau = [NaN, NaN, NaN];
  try { tau = jointTorques(d, S.q, [0, 0, 0], p).map((v) => v / (d.ratio * d.efficiency)); } catch (e) { /* singular */ }
  const bad = (i, key) => rep.problems.includes(key + (i + 1)) ? ' style="color:var(--danger)"' : '';
  let rows = '<tr><th></th><th>팔 1</th><th>팔 2</th><th>팔 3</th></tr>';
  rows += '<tr><td>θ (°)</td>' + [0, 1, 2].map((i) => `<td${bad(i, 'theta')}>${(S.q[i] * DEG).toFixed(1)}</td>`).join('') + '</tr>';
  rows += '<tr><td>볼조인트 (°)</td>' + [0, 1, 2].map((i) => `<td${bad(i, 'ball')}>${(Math.abs(pa[i][1]) * DEG).toFixed(1)}</td>`).join('') + '</tr>';
  rows += '<tr><td>팔꿈치 (°)</td>' + [0, 1, 2].map((i) => `<td${bad(i, 'elbow')}>${(K.elbowAngle(d, i, S.q[i], p) * DEG).toFixed(1)}</td>`).join('') + '</tr>';
  rows += '<tr><td>정적 토크 (모터축, N·m)</td>' + tau.map((v) => `<td>${fmt(v, 3)}</td>`).join('') + '</tr>';
  rows += `<tr><td>조건수 κ</td><td colspan="3">${Number.isFinite(cond) ? cond.toFixed(2) : '∞ (특이점)'}</td></tr>`;
  $('jogTable').innerHTML = rows;
  $('btnJogTool').textContent = S.tool ? '툴 OFF' : '툴 ON';
}
$('btnHome').addEventListener('click', () => {
  if (S.live) return;
  pause();
  startJogMode();
  S.anim = null;
  const h = S.design.homeTheta; S.q = [h, h, h]; S.jogBad = null;
  $('jogMsg').textContent = '';
  paintJog();
});
$('btnJogTool').addEventListener('click', () => {
  if (S.live) return;
  pause();
  startJogMode();
  const tcp = currentTcp();
  if (!S.tool) {
    S.tool = 1;
    const id = S.sceneState.grab(tcp, S.jogTime, S.design.toolSpec);
    toast(id ? `부품 ${id}를 잡았습니다` : '잡힌 부품이 없습니다 — 부품 윗면(흡착·전자석) 또는 부품 높이 안(그리퍼)에 TCP를 맞추세요');
  } else {
    S.tool = 0;
    const id = S.sceneState.release(tcp, S.jogTime);
    if (id) toast(`부품 ${id}를 놓았습니다`);
  }
  paintJog();
});

// ------------------------------------------------------------------ jog: pick & place helpers
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
/** Smoothly move the TCP in a straight line (IK every frame). Resolves true when it arrived. */
function animateTcp(to, dur) {
  const from = currentTcp();
  if (!from) return Promise.resolve(false);
  const rep = K.limitReport(S.design, [to[0], to[1], to[2] + S.design.toolLength]);
  if (!rep.ok) {
    $('jogMsg').className = 'd-msg bad';
    $('jogMsg').textContent = '갈 수 없는 위치: ' + (rep.theta ? rep.problems.join(', ') : '팔 길이로 닿지 않음');
    return Promise.resolve(false);
  }
  return new Promise((resolve) => { S.anim = { from, to, t0: performance.now(), dur: Math.max(0.05, dur) * 1000, resolve }; });
}
function stepAnim(now) {
  const a = S.anim;
  if (!a) return;
  const u = Math.min(1, (now - a.t0) / a.dur);
  const s = u * u * (3 - 2 * u);                       // smooth start / stop
  const p = [0, 1, 2].map((k) => a.from[k] + (a.to[k] - a.from[k]) * s);
  const rep = K.limitReport(S.design, [p[0], p[1], p[2] + S.design.toolLength]);
  if (rep.theta) S.q = rep.theta;
  if (u >= 1) { S.anim = null; S.jogBad = null; a.resolve(true); }
}
const jogDur = () => Math.min(3, Math.max(0.1, Number($('jogDur').value) || 0.6));
function pickZ(part) { return S.design.toolSpec.kind === 'gripper' ? part.z + 0.5 * part.part.h : part.z + part.part.h; }
let busy = false, selectedPart = null;
function visibleParts() { return S.sceneState.visible(S.jogTime); }
async function pickPart(id) {
  if (busy || S.live) return;
  const tool = S.design.toolSpec;
  if (tool.kind === 'pen') { toast('펜 툴은 부품을 집을 수 없습니다 — 설계 탭에서 그리퍼·흡착컵을 고르세요', true); return; }
  if (S.sceneState.held) { toast('이미 ' + S.sceneState.held.id + '를 잡고 있습니다 — 먼저 놓으세요', true); return; }
  pause(); startJogMode();
  const tcp = currentTcp();
  let cands = visibleParts().filter((v) => !tool.ferrous_only || v.part.material === 'steel');
  if (id) cands = cands.filter((v) => v.part.id === id);
  if (!cands.length) { toast(tool.ferrous_only ? '전자석으로 잡을 수 있는 철(steel) 부품이 없습니다' : '잡을 부품이 없습니다', true); return; }
  cands.sort((a, b) => Math.hypot(a.x - tcp[0], a.y - tcp[1]) - Math.hypot(b.x - tcp[0], b.y - tcp[1]));
  const target = cands[0];
  busy = true;
  view.highlightPart(target.part.id);
  try {
    const up = 0.03, z = pickZ(target);
    const conv = target.part.mode === 'conveyor' ? S.sceneData.conveyor.speed : 0;
    const lead = conv * jogDur() * 2;                   // conveyor parts keep moving while we approach
    if (!await animateTcp([target.x + lead, target.y, Math.max(tcp[2], z + up)], jogDur())) return;
    const now = visibleParts().find((v) => v.part.id === target.part.id);
    if (!now) { toast('부품이 사라졌습니다', true); return; }
    if (!await animateTcp([now.x + conv * jogDur() * 0.6, now.y, z], jogDur() * 0.6)) return;
    S.tool = 1;
    const got = S.sceneState.grab(currentTcp(), S.jogTime, tool);
    await sleep(150);
    if (!got) { S.tool = 0; toast('잡지 못했습니다 — 위치가 조금 어긋났습니다. 다시 시도하세요', true); }
    await animateTcp([currentTcp()[0], currentTcp()[1], z + up], jogDur() * 0.6);
    if (got) toast(`부품 ${got} 를 잡았습니다 — 방향키/슬라이더로 옮기거나 상자 버튼을 누르세요`);
  } finally {
    busy = false;
    selectedPart = null;
    $('btnPick').textContent = '가까운 부품 집기';
    view.highlightPart(null);
    paintJog();
  }
}
async function placeAt(x, y, surface) {
  if (busy || S.live) return;
  const held = S.sceneState.held;
  if (!held) { toast('잡고 있는 부품이 없습니다', true); return; }
  pause(); startJogMode();
  busy = true;
  try {
    const up = 0.03, tcp = currentTcp();
    const zDrop = surface + held.h + 0.004;
    const zTop = Math.max(tcp[2], zDrop + up);
    if (!await animateTcp([tcp[0], tcp[1], zTop], jogDur() * 0.5)) return;
    if (!await animateTcp([x, y, zTop], jogDur())) return;
    if (!await animateTcp([x, y, zDrop], jogDur() * 0.6)) return;
    S.tool = 0;
    const id = S.sceneState.release(currentTcp(), S.jogTime);
    await sleep(120);
    await animateTcp([x, y, zTop], jogDur() * 0.6);
    const sc = S.sceneState.score();
    toast(`부품 ${id} 를 놓았습니다` + (sc.total ? ` — 상자 안 ${sc.total}개 (색 일치 ${sc.correct})` : ''));
  } finally {
    busy = false;
    paintJog();
  }
}
function paintBins() {
  const bins = S.sceneData.bins || [];
  const name = { red: '빨강', blue: '파랑', green: '초록' };
  $('binBtns').innerHTML = bins.map((b) => `<button class="s-btn" type="button" data-bin="${b.id}">상자 ${b.id}(${name[b.color] || b.color})로 옮기기</button>`).join('');
  $('binBtns').querySelectorAll('button').forEach((btn) => btn.addEventListener('click', () => {
    const b = bins.find((x) => x.id === btn.dataset.bin);
    placeAt(b.x, b.y, S.sceneData.surface_z + 0.003);
  }));
}
function surfaceUnder(x, y) {
  for (const b of S.sceneData.bins || []) if (Math.abs(x - b.x) <= b.w / 2 && Math.abs(y - b.y) <= b.d / 2) return S.sceneData.surface_z + 0.003;
  return S.sceneData.surface_z;
}
$('btnPick').addEventListener('click', () => pickPart(selectedPart));
$('btnPlace').addEventListener('click', () => { const t = currentTcp(); if (t) placeAt(t[0], t[1], surfaceUnder(t[0], t[1])); });
// click a part in the 3D view to choose it
$('viewport').addEventListener('click', (e) => {
  if (activeTab !== 'jog' || busy) return;
  const id = view.pickPart(e.clientX, e.clientY);
  if (!id) return;
  selectedPart = id;
  view.highlightPart(id);
  $('btnPick').textContent = `부품 ${id} 집기`;
  $('jogMsg').className = 'd-msg ok';
  $('jogMsg').textContent = `부품 ${id} 선택 — [부품 ${id} 집기]를 누르세요`;
});
// keyboard jog (jog tab only, not while typing)
document.addEventListener('keydown', (e) => {
  if (activeTab !== 'jog' || S.live || busy || e.target.matches('input, textarea, select')) return;
  const st = Number($('jogStep').value) / 1000;
  const d = { ArrowLeft: [-st, 0, 0], ArrowRight: [st, 0, 0], ArrowUp: [0, st, 0], ArrowDown: [0, -st, 0],
    q: [0, 0, st], Q: [0, 0, st], PageUp: [0, 0, st], e: [0, 0, -st], E: [0, 0, -st], PageDown: [0, 0, -st] }[e.key];
  if (e.key === 'g' || e.key === 'G') {
    e.preventDefault();
    if (S.sceneState.held) $('btnPlace').click(); else $('btnPick').click();
    return;
  }
  if (!d) return;
  e.preventDefault();
  pause(); startJogMode(); S.anim = null;
  const t = currentTcp();
  const to = [t[0] + d[0], t[1] + d[1], t[2] + d[2]];
  const rep = K.limitReport(S.design, [to[0], to[1], to[2] + S.design.toolLength]);
  if (!rep.ok) { $('jogMsg').className = 'd-msg bad'; $('jogMsg').textContent = '더 갈 수 없습니다: ' + (rep.theta ? rep.problems.join(', ') : '닿지 않음'); return; }
  S.q = rep.theta;
  $('jogMsg').className = 'd-msg ok'; $('jogMsg').textContent = '도달 가능';
  paintJog();
});

// ------------------------------------------------------------------ joystick (screen widget + gamepad)
let joy = null, joyWasActive = false;
function setupJoystick() {
  if (EMBED) return;
  joy = new Joystick($('joyHost'), {
    buttonLabel: '집기', speed: 80, minSpeed: 5, maxSpeed: 400, step: 5,
    onButton: () => { if (S.sceneState && S.sceneState.held) $('btnPlace').click(); else $('btnPick').click(); },
  });
  const show = () => { $('joyHost').hidden = !$('optJoy').checked; };
  $('optJoy').addEventListener('change', show);
  show();
}
/** Joystick control step (called every frame). Returns true if it moved the robot. */
function joystickStep(dt) {
  if (!joy || S.live || busy || $('joyHost').hidden) return false;
  const v = joy.value();
  const active = v.x !== 0 || v.y !== 0 || v.z !== 0;
  if (!active) {
    if (joyWasActive) { joyWasActive = false; paintJog(); }
    return false;
  }
  if (!joyWasActive) { pause(); startJogMode(); S.anim = null; joyWasActive = true; }
  const tcp = currentTcp();
  if (!tcp) return false;
  const res = stepTcp(tcp, v, joy.speed(), dt, (p) => {
    const rep = K.limitReport(S.design, [p[0], p[1], p[2] + S.design.toolLength]);
    if (!rep.ok) return false;
    S.q = rep.theta;
    return true;
  });
  S.jogBad = null;
  const msg = $('jogMsg');
  if (res === 'blocked' || res === 'slide') {
    msg.className = 'd-msg bad';
    msg.textContent = res === 'blocked' ? '작업영역 경계 — 더 갈 수 없습니다' : '작업영역 경계를 따라 미끄러지는 중';
  } else if (msg.textContent.startsWith('작업영역 경계')) { msg.className = 'd-msg ok'; msg.textContent = ''; }
  return true;
}

// ------------------------------------------------------------------ task
function buildTaskForm() {
  $('sceneKind').innerHTML = Object.entries(SCENE_KINDS).map(([k, v]) => `<option value="${k}">${v}</option>`).join('');
  $('program').innerHTML = Object.entries(PROGRAMS).map(([k, v]) => `<option value="${k}">${v.name}</option>`).join('');
  $('profile').innerHTML = PROFILES.map((p) => `<option value="${p}">${PROFILE_KO[p]} (${p})</option>`).join('');
  $('sceneKind').addEventListener('change', (e) => { S.sceneKind = e.target.value; applyDesign(S.design); });
  $('program').addEventListener('change', () => {
    const want = PROGRAMS[$('program').value].scene;
    if (want && want !== S.sceneKind) { S.sceneKind = want; applyDesign(S.design); }
  });
  const saveOpts = () => lsSet(LS_OPTS, S.opts);
  $('profile').addEventListener('change', (e) => { S.opts.profile = e.target.value; saveOpts(); });
  $('archH').addEventListener('change', (e) => { S.opts.archH = Math.max(5, Number(e.target.value)); saveOpts(); });
  $('speed').addEventListener('change', (e) => { S.opts.speed = Math.max(0.01, Number(e.target.value)); S.opts.speedUser = true; saveOpts(); });
  $('accel').addEventListener('change', (e) => { S.opts.accel = Math.max(0.1, Number(e.target.value)); S.opts.speedUser = true; saveOpts(); });
  $('btnRun').addEventListener('click', runProgram);
}
function paintTaskForm() {
  $('sceneKind').value = S.sceneKind;
  $('profile').value = S.opts.profile;
  $('archH').value = S.opts.archH;
  $('speed').value = S.opts.speed;
  $('accel').value = S.opts.accel;
}
function planOpts() { return { profile: S.opts.profile, speed: S.opts.speed, accel: S.opts.accel }; }

function runProgram() {
  if (S.live) { toast('PC 연결을 끊은 뒤 실행하세요', true); return; }
  const name = $('program').value;
  const want = PROGRAMS[name].scene;
  if (want && want !== S.sceneKind) { S.sceneKind = want; applyDesign(S.design); }
  const msg = $('taskMsg');
  try {
    const tl = buildProgramWithArch(name);
    loadTimeline(tl);
    msg.className = 'd-msg ok';
    msg.textContent = `생성 완료 — ${tl.duration.toFixed(2)} s, 프레임 ${tl.frames.length}개`;
    play();
  } catch (e) {
    msg.className = 'd-msg bad';
    msg.textContent = String(e.message || e);
  }
}
function buildProgramWithArch(name) {
  return buildProgram(name, S.design, S.sceneData, { ...planOpts(), archH: S.opts.archH / 1000 });
}

function paintTaskInfo() {
  const tl = S.timeline;
  if (!tl) { $('taskInfo').innerHTML = ''; return; }
  const items = [['총 시간', tl.duration.toFixed(2) + ' s']];
  if (tl.info && tl.info.cycleTime) {
    items.push(['사이클 시간', tl.info.cycleTime.toFixed(3) + ' s'], ['분당 사이클', (60 / tl.info.cycleTime).toFixed(1) + ' cpm'],
      ['사이클 폭×높이', `${(tl.info.across * 1000).toFixed(0)} × ${(tl.info.up * 1000).toFixed(0)} mm`]);
  }
  const sc = S.sceneState.score();
  if (S.sceneData.bins && S.sceneData.bins.length) {
    const fin = tl.finalScore ? ` / 최종 ${tl.finalScore.total}개 (정답 ${tl.finalScore.correct})` : ` (정답 ${sc.correct})`;
    items.push(['분류 결과', `지금 ${sc.total}개${fin}`]);
  }
  if (S.evalRes) items.push(['모터 검증', S.evalRes.ok ? '✔ 통과' : '✖ ' + S.evalRes.problems.length + '건']);
  $('taskInfo').innerHTML = items.map(([k, v]) => `<div><span>${k}</span><code>${v}</code></div>`).join('');
}

// ------------------------------------------------------------------ timeline playback
function clearTimeline() {
  S.timeline = null; S.analysis = null; S.evalRes = null; S.t = 0; S.playing = false;
  view.setPathPreview(null);
  view.clearTrail();
  paintPlayer();
  paintEval();
  paintTaskInfo();
  drawCharts();
}
function loadTimeline(tl) {
  S.timeline = tl;
  S.t = 0;
  resetPlayback();
  try {
    S.analysis = analyze(S.design, tl.frames);
    S.evalRes = evaluate(S.design, S.analysis);
  } catch (e) {
    S.analysis = null; S.evalRes = null;
  }
  paintEval();
  paintTaskInfo();
  showPathPreview();
  drawCharts();
  if (EMBED) postParent({ type: 'sim-analysis', eval: S.evalRes, duration: tl.duration });
}
function resetPlayback() {
  S.frameIdx = 0;
  S.tool = 0;
  S.sceneState = new SceneState(S.sceneData);
  view.clearTrail();
}
function frameAt(t) {
  const fr = S.timeline.frames;
  // advance tool events in order, starting from the last processed frame
  if (t < (S.lastT || 0)) resetPlayback();
  S.lastT = t;
  while (S.frameIdx < fr.length && fr[S.frameIdx].t <= t) {
    const f = fr[S.frameIdx];
    if ((f.tool || 0) !== S.tool) {
      const p = K.tryFk(S.design, f.q);
      if (p) {
        const tcp = [p[0], p[1], p[2] - S.design.toolLength];
        if (f.tool) S.sceneState.grab(tcp, f.t, S.design.toolSpec); else S.sceneState.release(tcp, f.t);
      }
      S.tool = f.tool || 0;
    }
    S.frameIdx++;
  }
  // interpolate joint angles
  const i = Math.max(1, Math.min(fr.length - 1, S.frameIdx));
  const a = fr[i - 1], b = fr[Math.min(i, fr.length - 1)];
  if (!b || b.t <= a.t) return a.q;
  const u = Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t)));
  return [0, 1, 2].map((k) => a.q[k] + (b.q[k] - a.q[k]) * u);
}
function play() {
  if (S.jogMode) { S.jogMode = false; S.anim = null; S.lastT = 0; resetPlayback(); }
  if (!S.timeline) { toast('재생할 동작이 없습니다 — 작업 탭에서 프로그램을 실행하세요'); return; }
  if (S.t >= S.timeline.duration) { S.t = 0; S.lastT = 0; resetPlayback(); }
  S.playing = true; paintPlayer();
}
function pause() { S.playing = false; paintPlayer(); }
function setupPlayer() {
  $('btnPlay').addEventListener('click', () => (S.playing ? pause() : play()));
  $('btnRestart').addEventListener('click', () => { S.t = 0; S.lastT = 0; resetPlayback(); if (S.timeline) play(); });
  $('seek').addEventListener('input', () => {
    if (!S.timeline) return;
    S.t = S.timeline.duration * Number($('seek').value) / 1000;
    S.playing = false;
    paintPlayer();
  });
  $('rate').addEventListener('change', () => { S.rate = Number($('rate').value); });
  document.addEventListener('keydown', (e) => {
    if (e.target.matches('input, textarea, select')) return;
    if (e.key === 'Escape') estop();
    if (e.key === 'p' || e.key === 'P') (S.playing ? pause() : play());
  });
  $('estop').addEventListener('click', estop);
}
function paintPlayer() {
  const dur = S.timeline ? S.timeline.duration : 0;
  $('btnPlay').innerHTML = S.playing ? '&#10074;&#10074;' : '&#9654;';
  $('timeLabel').textContent = `${S.t.toFixed(2)} / ${dur.toFixed(2)} s`;
  $('seek').value = dur ? String(Math.round(1000 * S.t / dur)) : '0';
}
function estop() {
  pause();
  if (S.ws && S.ws.readyState === 1) { S.ws.send(JSON.stringify({ type: 'estop' })); linkLog('estop 전송'); }
  toast('비상정지 — 재생을 멈췄습니다' + (S.ws ? ' (PC에 estop 전송)' : ''), true);
}

// ------------------------------------------------------------------ analysis
const charts = {};
function drawCharts() {
  if (activeTab !== 'analysis' && !charts.q) return;
  if (!charts.q) {
    charts.q = new TimeChart($('chartQ'), { title: '모터각 θ', unit: '°', labels: ['θ1', 'θ2', 'θ3'] });
    charts.w = new TimeChart($('chartW'), { title: '모터 속도 ω (모터축)', unit: 'rpm', labels: ['ω1', 'ω2', 'ω3'], symmetric: true });
    charts.t = new TimeChart($('chartT'), { title: '모터 토크 τ (모터축)', unit: 'N·m', labels: ['τ1', 'τ2', 'τ3'], symmetric: true });
  }
  const d = S.design, m = d.motorSpec, n = d.ratio, eta = d.efficiency;
  charts.q.setLimits([{ v: d.theta_min * DEG, label: 'θmin', color: '#ef4444' }, { v: d.theta_max * DEG, label: 'θmax', color: '#ef4444' }]);
  const rpm = m.max_speed * 60 / (2 * Math.PI);
  charts.w.setLimits([{ v: rpm, label: '최고 속도', color: '#ef4444' }, { v: -rpm, color: '#ef4444' }]);
  charts.t.setLimits([{ v: m.peak_torque, label: '피크', color: '#ef4444' }, { v: -m.peak_torque, color: '#ef4444' },
    { v: m.rated_torque, label: '정격', color: '#f59e0b' }, { v: -m.rated_torque, color: '#f59e0b' }]);
  const A = S.analysis;
  if (A && A.length) {
    const t = A.map((s) => s.t);
    charts.q.setData(t, [0, 1, 2].map((i) => A.map((s) => s.theta[i] * DEG)));
    charts.w.setData(t, [0, 1, 2].map((i) => A.map((s) => s.theta_d[i] * n * 60 / (2 * Math.PI))));
    charts.t.setData(t, [0, 1, 2].map((i) => A.map((s) => s.tau[i] / (n * eta))));
  } else {
    for (const c of Object.values(charts)) c.setData([], [[], [], []]);
  }
  const span = S.live ? 8 : null;
  for (const c of Object.values(charts)) c.draw(S.timeline ? S.t : null, span);
}
function paintEval() {
  const ev = S.evalRes;
  if (!ev) { $('evalBox').innerHTML = '<p class="hint">작업 탭에서 프로그램을 실행하면 결과가 여기에 표시됩니다.</p>'; return; }
  const pct = (v) => (v * 100).toFixed(0) + '%';
  const li = (ok, label, val) => `<li class="${ok ? 'ok' : 'bad'}"><b>${ok ? '✔' : '✖'}</b><span>${label}</span><code>${val}</code></li>`;
  const d = S.design;
  const ir = ev.inertia_ratio;
  const irLi = d.motorSpec.integrated_gear ? '<li class="ok"><b>–</b><span>관성비</span><code>일체형 서보</code></li>'
    : `<li class="${ir <= 10 ? 'ok' : ir <= 30 ? 'warn' : 'bad'}"><b>${ir <= 10 ? '✔' : ir <= 30 ? '!' : '✖'}</b><span>관성비 (부하/로터, 권장 ≤ 10)</span><code>${ir.toFixed(1)}</code></li>`;
  $('evalBox').innerHTML = '<ul class="d-check">' +
    li(ev.peak_ratio <= 1, '피크 토크 / 모터 피크', pct(ev.peak_ratio)) +
    li(ev.rms_ratio <= 1, 'RMS 토크 / 모터 정격', pct(ev.rms_ratio)) +
    li(ev.speed_ratio <= 1, '최고 속도 / 모터 최고 속도', pct(ev.speed_ratio)) +
    li(ev.curve_ratio <= 1, '속도-토크 곡선 여유' + (d.motorSpec.kind === 'stepper' ? ' (스테퍼 ×1.5 여유)' : ''), pct(ev.curve_ratio)) +
    irLi + '</ul>' +
    (ev.problems.length ? `<p class="d-msg bad">${ev.problems.join('<br>')}</p>` : '<p class="d-msg ok">이 동작은 선택한 모터로 수행할 수 있습니다.</p>');
}

// ------------------------------------------------------------------ design check (standard cycle)
function holdingCheck(d, amax) {
  const t = d.toolSpec;
  const need = d.payload * (G + amax) * 2.0;       // safety factor 2 on the holding force
  if (t.kind === 'pen') return { ok: true, text: '펜 — 집기 없음' };
  const have = t.kind === 'gripper' ? 2 * t.friction * t.force : t.force;
  return { ok: have >= need, text: `필요 ${need.toFixed(1)} N (m·(g+a)·SF2) / 툴 ${have.toFixed(1)} N` };
}
function runDesignCheck() {
  const d = S.design;
  const out = $('checkResult');
  try {
    const p = new Planner(d, S.sceneData, planOpts());
    p.home();
    const info = runAdept(p, S.sceneData, 3);
    const tl = p.timeline('adept');
    tl.info = info;
    const an = analyze(d, tl.frames);
    const ev = evaluate(d, an);
    let amax = 0;
    for (const s of an) amax = Math.max(amax, Math.hypot(...s.a));
    const hold = holdingCheck(d, amax);
    const c = S.cyl;
    const li = (ok, label, val) => `<li class="${ok ? 'ok' : 'bad'}"><b>${ok ? '✔' : '✖'}</b><span>${label}</span><code>${val}</code></li>`;
    out.innerHTML = '<ul class="d-check">' +
      `<li class="ok"><b>ⓘ</b><span>작업 실린더</span><code>Ø${(c.diameter * 1000).toFixed(0)} × ${(c.height * 1000).toFixed(0)} mm</code></li>` +
      `<li class="ok"><b>ⓘ</b><span>표준 사이클 ${(info.across * 1000).toFixed(0)}×${(info.up * 1000).toFixed(0)} mm</span><code>${info.cycleTime.toFixed(2)} s · ${(60 / info.cycleTime).toFixed(0)} cpm</code></li>` +
      li(ev.peak_ratio <= 1, '피크 토크', (ev.peak_ratio * 100).toFixed(0) + '%') +
      li(ev.rms_ratio <= 1, 'RMS 토크', (ev.rms_ratio * 100).toFixed(0) + '%') +
      li(ev.speed_ratio <= 1 && ev.curve_ratio <= 1, '속도 / 속도-토크 곡선', `${(ev.speed_ratio * 100).toFixed(0)}% / ${(ev.curve_ratio * 100).toFixed(0)}%`) +
      li(hold.ok, '툴 유지력 (최대 가속 ' + amax.toFixed(1) + ' m/s²)', hold.text) +
      '</ul>' +
      `<p class="hint">표준 사이클 = 위로 ${(info.up * 1000).toFixed(0)} mm → 옆으로 ${(info.across * 1000).toFixed(0)} mm → 아래로, 왕복 1회 (작업 탭의 속도·가속도·프로파일 사용).
      [실행]으로 같은 동작을 재생하려면 작업 탭에서 "Adept 사이클"을 고르세요.</p>`;
    loadTimeline(tl);
    S.t = 0;
  } catch (e) {
    out.innerHTML = `<p class="d-msg bad">${e.message || e}</p>`;
  }
}

// ------------------------------------------------------------------ link (PC Python / ROS 2)
function linkLog(msg) {
  const box = $('linkLog');
  const line = document.createElement('div');
  line.textContent = new Date().toLocaleTimeString() + '  ' + msg;
  box.appendChild(line);
  while (box.childElementCount > 200) box.firstElementChild.remove();
  box.scrollTop = box.scrollHeight;
}
function setLinkStatus(text, cls) { const s = $('linkStatus'); s.textContent = text; s.className = 'vstat' + (cls ? ' ' + cls : ''); }
function setupLink() {
  $('btnConnect').addEventListener('click', () => (S.ws ? disconnect() : connect()));
  $('btnGoal').addEventListener('click', () => {
    const tcp = currentTcp();
    if (S.ws && tcp) { S.ws.send(JSON.stringify({ type: 'goal', xyz: tcp })); linkLog('goal ' + tcp.map((v) => v.toFixed(3)).join(', ')); }
  });
  $('btnExport').addEventListener('click', () => download('delta_design.json', JSON.stringify({ design: S.design.toDict(), scene: S.sceneKind }, null, 2)));
  $('btnUrdf').addEventListener('click', () => download('delta_robot.urdf', generateUrdf(S.design)));
  $('btnShare').addEventListener('click', async () => {
    const url = location.origin + location.pathname + '#design=' + b64url(JSON.stringify({ design: S.design.toDict(), scene: S.sceneKind }));
    try { await navigator.clipboard.writeText(url); toast('공유 링크를 복사했습니다'); } catch (e) { prompt('링크를 복사하세요', url); }
  });
  $('fileDesign').addEventListener('change', async (e) => {
    const f = e.target.files[0]; if (!f) return;
    try {
      const j = JSON.parse(await f.text());
      if (SCENE_KINDS[j.scene]) S.sceneKind = j.scene;
      S.presetName = null;
      applyDesign(new Design(j.design || j), { fit: true });
      toast('설계를 불러왔습니다');
    } catch (err) { toast('설계 JSON을 읽지 못했습니다: ' + err.message, true); }
    e.target.value = '';
  });
  $('fileTimeline').addEventListener('change', async (e) => {
    const f = e.target.files[0]; if (!f) return;
    try { acceptTimeline(JSON.parse(await f.text())); toast('타임라인을 불러왔습니다'); } catch (err) { toast('타임라인 JSON을 읽지 못했습니다: ' + err.message, true); }
    e.target.value = '';
  });
}
function download(name, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'application/octet-stream' }));
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
function connect() {
  const url = $('wsUrl').value.trim();
  let ws;
  try { ws = new WebSocket(url); } catch (e) { setLinkStatus('주소 오류: ' + e.message, 'bad'); return; }
  S.ws = ws;
  setLinkStatus('연결 중… ' + url);
  $('btnConnect').textContent = '연결 끊기';
  ws.onopen = () => {
    setLinkStatus('연결됨 — ' + url, 'ok');
    $('linkChip').hidden = false;
    $('btnGoal').disabled = false;
    ws.send(JSON.stringify({ type: 'hello', client: 'delta-sim', version: 1, design: S.design.toDict(), scene: S.sceneData }));
    linkLog('hello 전송 (현재 설계·장면)');
    pause();
    S.live = { frames: [], t0: null };
    S.timeline = null;
  };
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch (e) { return; }
    if (msg.type === 'design') {
      if (JSON.stringify(msg.design) !== JSON.stringify(S.design.toDict()) || msg.scene) {
        S.presetName = null;
        if (msg.scene && msg.scene.kind) S.sceneKind = msg.scene.kind;
        applyDesign(new Design(msg.design), { sceneData: msg.scene || undefined, fromLink: true, fit: true });
        linkLog('PC가 보낸 설계를 적용했습니다');
      }
      S.live = { frames: [], t0: null };
    } else if (msg.type === 'state') {
      onLiveState(msg);
    } else if (msg.type === 'log') {
      linkLog(String(msg.text || ''));
    }
  };
  ws.onerror = () => { setLinkStatus('연결 실패 — PC에서 websim 스크립트나 ROS 2 web_bridge가 실행 중인지 확인하세요', 'bad'); };
  ws.onclose = (e) => {
    if (S.ws === ws) {
      S.ws = null; S.live = null;
      $('btnConnect').textContent = 'PC Python 연결';
      $('linkChip').hidden = true;
      $('btnGoal').disabled = true;
      setLinkStatus('연결 안 됨' + (e.code && e.code !== 1000 ? ` (code ${e.code})` : ''), e.code === 1000 ? '' : 'bad');
      linkLog('연결 종료');
    }
  };
}
function disconnect() { if (S.ws) S.ws.close(1000); }
function onLiveState(msg) {
  const L = S.live || (S.live = { frames: [], t0: null });
  const t = Number(msg.t) || 0;
  const q = msg.q.map(Number);
  const tool = msg.tool ? 1 : 0;
  if (tool !== S.tool) {
    const p = K.tryFk(S.design, q);
    if (p) {
      const tcp = [p[0], p[1], p[2] - S.design.toolLength];
      if (tool) S.sceneState.grab(tcp, t, S.design.toolSpec); else S.sceneState.release(tcp, t);
    }
    S.tool = tool;
  }
  S.q = q;
  S.liveT = t;
  L.frames.push({ t, q, tool });
  if (L.frames.length > 3000) L.frames.splice(0, L.frames.length - 3000);
  const now = performance.now();
  if (activeTab === 'analysis' && (!L.lastAn || now - L.lastAn > 250) && L.frames.length > 5) {
    L.lastAn = now;
    const recent = L.frames.filter((f) => f.t >= t - 8);
    try { S.analysis = analyze(S.design, recent); S.evalRes = evaluate(S.design, S.analysis); paintEval(); } catch (e) { /* ignore */ }
  }
}

// ------------------------------------------------------------------ acceptance of external timelines (file / Playground)
function acceptTimeline(tl) {
  if (!tl || !Array.isArray(tl.frames) || !tl.frames.length) throw new Error('frames가 없습니다');
  if (tl.design) {
    S.presetName = null;
    if (tl.scene && tl.scene.kind) S.sceneKind = tl.scene.kind;
    applyDesign(new Design(tl.design), { sceneData: tl.scene || undefined, fit: EMBED ? false : true });
  }
  if (tl.duration === undefined) tl.duration = tl.frames[tl.frames.length - 1].t;
  loadTimeline(tl);
  play();
}

// ------------------------------------------------------------------ embed (Playground)
function postParent(msg) { try { window.parent.postMessage({ source: 'delta-sim', ...msg }, '*'); } catch (e) { /* no parent */ } }
function setupEmbed() {
  window.addEventListener('message', (ev) => {
    if (ev.source !== window.parent) return;
    const m = ev.data || {};
    if (m.type === 'timeline') {
      try { acceptTimeline(m.timeline); } catch (e) { postParent({ type: 'sim-error', error: String(e.message || e) }); }
    } else if (m.type === 'get-design') {
      postParent({ type: 'sim-design', design: S.design.toDict(), scene: S.sceneData });
    } else if (m.type === 'stop') {
      pause();
    }
  });
  postParent({ type: 'sim-ready', design: S.design.toDict(), scene: S.sceneData });
}

// ------------------------------------------------------------------ render loop
let lastFrame = performance.now(), lastChart = 0, lastTrail = null;
function loop(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  const joyMoved = joystickStep(dt);
  let tSim = S.jogTime;
  if (S.live) {
    tSim = S.liveT || 0;
  } else if (S.timeline && !S.jogMode) {
    if (S.playing) {
      S.t += dt * S.rate;
      if (S.t >= S.timeline.duration) { S.t = S.timeline.duration; S.playing = false; paintPlayer(); }
    }
    S.q = frameAt(S.t);
    tSim = S.t;
  } else {
    S.jogTime += dt;
    tSim = S.jogTime;
    stepAnim(now);
  }
  const bad = S.jogBad && (!S.timeline || S.jogMode) && !S.live ? [0, 1, 2].map((i) => S.jogBad.problems.some((p) => p.endsWith(String(i + 1)))) : null;
  const res = view.setPose(S.q, S.tool, S.sceneState, tSim, bad);
  if (res) {
    const draws = S.design.toolSpec.kind === 'pen' || S.sceneKind === 'drawing';
    if ((draws && S.tool) || $('optTrail').checked) {
      if (!lastTrail || Math.hypot(res.tcp[0] - lastTrail[0], res.tcp[1] - lastTrail[1], res.tcp[2] - lastTrail[2]) > 0.001) {
        view.addTrail(res.tcp); lastTrail = res.tcp;
      }
    }
    $('hudTcp').textContent = res.tcp.map((v) => (v * 1000).toFixed(1).padStart(7)).join(' ') + ' mm';
    $('hudQ').textContent = S.q.map((v) => (v * DEG).toFixed(1).padStart(6)).join(' ') + ' °';
    $('hudT').textContent = S.live ? `PC ${tSim.toFixed(2)} s` : S.timeline && !S.jogMode ? `${S.t.toFixed(2)} / ${S.timeline.duration.toFixed(2)} s` : '조그';
    $('hudTool').textContent = (S.tool ? 'ON' : 'off') + (S.sceneState && S.sceneState.held ? ' · ' + S.sceneState.held.id : '');
  }
  if (now - lastChart > 100) {
    lastChart = now;
    if (S.timeline && S.playing) paintPlayer();
    if (activeTab === 'analysis') drawCharts();
    if (activeTab === 'task' && S.timeline && S.playing) paintTaskInfo();
    if (activeTab === 'jog' && (S.live || S.anim || joyMoved || (S.timeline && !S.jogMode))) paintJog();
    if (joy) joy.setButtonLabel(S.sceneState && S.sceneState.held ? '놓기' : '집기');
  }
  view.render();
  requestAnimationFrame(loop);
}

boot().catch((e) => {
  console.error(e);
  const box = $('noGL'); box.hidden = false;
  box.textContent = '시뮬레이터를 시작하지 못했습니다: ' + (e.message || e);
});
