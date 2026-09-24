// urdf-viewer.js — delta robot URDF viewer (three.js 0.186 + urdf-loader 0.13.1).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { Design, loadCatalog } from './delta/design.js';
import { tryFk, PHI, limitReport, workspaceBounds } from './delta/kinematics.js';
import { generate, jointState } from './delta/urdf.js';
import { overlayArrow, rotationArrow, textSprite } from '../../sim/render.js';
import { Joystick, stepTcp } from './joystick.js';

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const LS_KEY = 'studydelta.design.v1';

document.querySelectorAll('.tabs button').forEach((btn) => btn.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('active', b === btn));
  document.querySelectorAll('.tab-body').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + btn.dataset.tab));
}));

const cat = await loadCatalog('../python/deltarobot/data/catalog.json');

// ------------------------------------------------------------------ three.js
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const container = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
container.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.005, 50);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.5));
const sun = new THREE.DirectionalLight(0xffffff, 2);
sun.position.set(1, -1, 2);
scene.add(sun);
const grid = new THREE.GridHelper(2, 40, 0x5c6773, 0x5c6773);
grid.rotation.x = Math.PI / 2;
grid.material.transparent = true; grid.material.opacity = 0.3;
scene.add(grid);
function bg() { scene.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#0c1015'); }
bg();
document.addEventListener('themechange', bg);
function resize() {
  const w = container.clientWidth || 800, h = container.clientHeight || 600;
  renderer.setSize(w, h, false);
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
new ResizeObserver(resize).observe(container);
resize();

// ------------------------------------------------------------------ design source
const saved = (() => { try { return JSON.parse(localStorage.getItem(LS_KEY) || 'null'); } catch (e) { return null; } })();
const src = $('source');
src.innerHTML = (saved && saved.design ? '<option value="__current">시뮬레이터의 현재 설계</option>' : '') +
  Object.entries(cat.presets).map(([k, p]) => `<option value="${k}">프리셋: ${p.name}</option>`).join('');
const qp = new URLSearchParams(location.search).get('preset');
if (qp && cat.presets[qp]) src.value = qp;

let design, robot, urdfText, q = [0.35, 0.35, 0.35], sliders = [];
const loader = new URDFLoader();

function load() {
  const key = src.value;
  design = new Design(key === '__current' ? saved.design : cat.presets[key].design);
  $('srcNote').textContent = key === '__current' ? '시뮬레이터 설계 탭에서 마지막으로 쓴 설계입니다.' : cat.presets[key].desc;
  urdfText = generate(design);
  $('urdfText').value = urdfText;
  if (robot) { scene.remove(robot); robot.traverse((o) => { if (o.geometry) o.geometry.dispose(); }); }
  robot = loader.parse(urdfText);
  scene.add(robot);
  robot.traverse((o) => { if (o.isMesh) { o.castShadow = true; } });
  const links = Object.keys(robot.links).length, joints = Object.values(robot.joints);
  $('nLinks').textContent = String(links);
  const grip = joints.filter((j) => j.name.startsWith('gripper')).length;
  $('nJoints').textContent = `${joints.length} (능동 3 · 수동 ${joints.filter((j) => j.name.startsWith('elbow')).length} · 가상 3${grip ? ' · 그리퍼 ' + grip : ''} · 고정 ${joints.filter((j) => j.jointType === 'fixed').length})`;
  $('toolWrap').hidden = !grip;
  buildSliders();
  buildMultiSliders();
  paintTree();
  const h = design.homeTheta;
  q = [h, h, h];
  apply();
  fit();
  axesVisible();
  jointAxes();
}

/** Generic slider row: returns {set(v)} ; onInput(v) gets the slider value. */
function makeRow(host, name, ko, lo, hi, step, unit, onInput) {
  const row = document.createElement('div');
  row.className = 'joint-row';
  row.innerHTML = `<div class="jr-head"><span class="jr-name">${name}</span><span class="jr-ko">${ko}</span><span class="jr-val"></span></div>
    <input type="range" min="${lo}" max="${hi}" step="${step}">`;
  const rng = row.querySelector('input'), val = row.querySelector('.jr-val');
  const digits = step < 1 ? 1 : 0;
  rng.addEventListener('input', () => { val.textContent = Number(rng.value).toFixed(digits) + unit; onInput(Number(rng.value)); });
  host.appendChild(row);
  return { set: (v) => { rng.value = v; val.textContent = Number(v).toFixed(digits) + unit; } };
}

let tcpRows = [], allRow = null, manualRows = {}, manual = {};
function tcpNow() {
  const p = tryFk(design, q);
  return p ? [p[0], p[1], p[2] - design.toolLength] : null;
}
function buildMultiSliders() {
  $('tcpSliders').innerHTML = ''; $('allSlider').innerHTML = '';
  const [rad] = workspaceBounds(design);
  const R = Math.round(rad * 1000), zlo = Math.round(-(design.upper_arm + design.forearm + design.toolLength) * 1000);
  tcpRows = ['x', 'y', 'z'].map((ax, i) => makeRow($('tcpSliders'), ax, 'TCP ' + ax, i < 2 ? -R : zlo, i < 2 ? R : -10, 1, ' mm', (v) => {
    const tcp = tcpNow() || [0, 0, -0.3];
    tcp[i] = v / 1000;
    const rep = limitReport(design, [tcp[0], tcp[1], tcp[2] + design.toolLength]);
    const msg = $('tcpMsg');
    if (rep.theta) { q = rep.theta.slice(); apply(false); }
    msg.className = 'u-msg' + (rep.ok ? '' : ' bad');
    msg.textContent = !rep.theta ? '팔 길이로 닿지 않는 위치입니다' : rep.ok ? 'θ = ' + q.map((t) => (t * DEG).toFixed(1) + '°').join(', ')
      : '한계 초과(' + rep.problems.join(', ') + ') — 실제 로봇은 갈 수 없는 자세';
  }));
  allRow = makeRow($('allSlider'), 'θ', '세 모터 공통', (design.theta_min * DEG).toFixed(1), (design.theta_max * DEG).toFixed(1), 0.1, '°', (v) => {
    const old = q.slice();
    q = [v / DEG, v / DEG, v / DEG];
    if (!apply()) { q = old; apply(); }
  });
  buildManual();
}
function buildManual() {
  $('manualSliders').innerHTML = '';
  manualRows = {};
  const js = jointState(design, q, 0);
  const [rad] = workspaceBounds(design);
  for (const name of Object.keys(js)) {
    if (name.startsWith('motor')) continue;
    const lin = name.startsWith('effector') || name.startsWith('gripper');
    let lo, hi;
    if (name.startsWith('effector')) { lo = name === 'effector_z' ? -(design.upper_arm + design.forearm) * 1000 : -rad * 1000; hi = name === 'effector_z' ? 0 : rad * 1000; }
    else if (name.startsWith('gripper')) { lo = -6; hi = 0; }
    else { lo = -180; hi = 180; }
    manualRows[name] = makeRow($('manualSliders'), name, lin ? '직동' : '회전', Math.round(lo), Math.round(hi), lin ? 0.5 : 0.5, lin ? ' mm' : '°', (v) => {
      manual[name] = lin ? v / 1000 : v / DEG;
      apply(false);
    });
  }
  loadManualFromFk();
}
function loadManualFromFk() {
  const js = jointState(design, q, $('optTool').checked ? 1 : 0);
  for (const [n, v] of Object.entries(js)) {
    if (n.startsWith('motor')) continue;
    manual[n] = v;
    const lin = n.startsWith('effector') || n.startsWith('gripper');
    if (manualRows[n]) manualRows[n].set(lin ? v * 1000 : v * DEG);
  }
}

function buildSliders() {
  $('sliders').innerHTML = '';
  sliders = [0, 1, 2].map((i) => {
    const row = document.createElement('div');
    row.className = 'joint-row';
    const lo = design.theta_min * DEG, hi = design.theta_max * DEG;
    row.innerHTML = `<div class="jr-head"><span class="jr-name">motor${i + 1}_joint</span><span class="jr-ko">θ${i + 1}</span><span class="jr-val"></span></div>
      <input type="range" min="${lo.toFixed(1)}" max="${hi.toFixed(1)}" step="0.1">
      <div class="jr-limits"><span>${lo.toFixed(0)}°</span><span>${hi.toFixed(0)}°</span></div>`;
    const rng = row.querySelector('input');
    rng.addEventListener('input', () => { const old = q.slice(); q[i] = Number(rng.value) / DEG; if (!apply()) { q = old; apply(); } });
    $('sliders').appendChild(row);
    return { row, rng, val: row.querySelector('.jr-val') };
  });
}

function apply(syncTcp = true) {
  const p = tryFk(design, q);
  sliders.forEach((s, i) => { s.rng.value = (q[i] * DEG).toFixed(1); s.val.textContent = (q[i] * DEG).toFixed(1) + '°'; });
  if (!p) { $('closure').textContent = '순기구학 해 없음'; return false; }
  if (syncTcp && tcpRows.length) {
    const t = [p[0], p[1], p[2] - design.toolLength];
    tcpRows.forEach((r, i) => r.set(Math.round(t[i] * 1000)));
    $('tcpMsg').textContent = '';
  }
  if (allRow && q[0] === q[1] && q[1] === q[2]) allRow.set((q[0] * DEG).toFixed(1));
  const js = jointState(design, q, $('optTool').checked ? 1 : 0);
  const passive = $('optPassive').checked;
  for (const [name, v] of Object.entries(js)) {
    const isActive = name.startsWith('motor') || name.startsWith('gripper');
    robot.setJointValue(name, isActive || passive ? v : (manual[name] ?? 0));
  }
  robot.updateMatrixWorld(true);
  // closure error: forearm tip vs where the effector's ball joint is
  let err = 0;
  const tip = new THREE.Vector3();
  const eff = robot.links.effector.getWorldPosition(new THREE.Vector3());
  for (let i = 0; i < 3; i++) {
    for (const [k, s] of [['a', 1], ['b', -1]]) {
      const link = robot.links[`forearm${i + 1}${k}`];
      tip.set(design.forearm, 0, 0).applyMatrix4(link.matrixWorld);
      const want = new THREE.Vector3(
        eff.x + design.effector_radius * Math.cos(PHI[i]) - s * Math.sin(PHI[i]) * design.forearm_spacing / 2,
        eff.y + design.effector_radius * Math.sin(PHI[i]) + s * Math.cos(PHI[i]) * design.forearm_spacing / 2,
        eff.z);
      err = Math.max(err, tip.distanceTo(want));
    }
  }
  $('closure').textContent = (err * 1000).toFixed(3) + ' mm' + (err > 0.001 ? '  ← 사슬이 풀림' : '');
  $('closure').style.color = err > 0.001 ? 'var(--danger)' : '';
  $('passiveTable').innerHTML = '<tr><th>관절</th><th>값</th></tr>' + Object.entries(js).filter(([n]) => !n.startsWith('motor')).map(([n, v]) =>
    `<tr><td>${n}</td><td>${n.startsWith('effector') || n.startsWith('gripper') ? (v * 1000).toFixed(1) + ' mm' : (v * DEG).toFixed(2) + '°'}</td></tr>`).join('');
  return true;
}

function paintTree() {
  const children = {};
  for (const j of Object.values(robot.joints)) {
    const parent = j.parent.name;
    (children[parent] = children[parent] || []).push(j);
  }
  const cls = (j) => j.name.startsWith('motor') ? 'j-active' : j.name.startsWith('elbow') ? 'j-passive' : j.name.startsWith('effector') ? 'j-virtual' : 'j-fixed';
  const lines = [];
  const walk = (link, depth) => {
    lines.push('  '.repeat(depth) + '▸ ' + link);
    for (const j of children[link] || []) {
      const child = j.children.find((c) => c.isURDFLink);
      const mimic = j.mimicJoint ? ` = ${j.mimicJoint}` : '';
      lines.push('  '.repeat(depth + 1) + `<span class="${cls(j)}">⟳ ${j.name} (${j.jointType}${mimic})</span>`);
      if (child) walk(child.name, depth + 2);
    }
  };
  walk('base_link', 0);
  $('tree').innerHTML = lines.join('\n');
}

let jointMarks = [];
function jointAxes() {
  jointMarks.forEach((m) => m.parent && m.parent.remove(m));
  jointMarks = [];
  if (!$('optJointAxes').checked || !robot) return;
  const s = design.upper_arm + design.forearm;
  for (const j of Object.values(robot.joints)) {
    if (j.jointType === 'fixed') continue;
    const active = j.name.startsWith('motor');
    const virt = j.name.startsWith('effector');
    if (!active && !virt && !j.name.includes('a_')) continue;          // show the 'a' rod only (b mimics it)
    const color = active ? 0x38bdf8 : virt ? 0xc084fc : 0xf59e0b;
    const len = active ? 0.5 * design.upper_arm : virt ? 0.1 * s : 0.3 * design.upper_arm;
    const ax = [j.axis.x, j.axis.y, j.axis.z];
    // fixed to the joint's zero pose in the parent link, so the marks do not spin with the joint value
    const holder = new THREE.Group();
    holder.position.copy(j.origPosition || j.position);
    holder.quaternion.copy(j.origQuaternion || j.quaternion);
    j.parent.add(holder); jointMarks.push(holder);
    const a = overlayArrow(ax, [0, 0, 0], len, color);
    holder.add(a);
    if (active) {
      // +θ sense: rotate the joint's x axis toward -z about +y (right-hand rule)
      const r = rotationArrow([0, 0, 0], [1, 0, 0], [0, 0, -1], 0.45 * design.upper_arm, 1.1, color);
      holder.add(r);
    }
    if (active || virt || j.name.startsWith('elbow1')) {
      const t = textSprite(j.name + (active ? '  (+θ ↓)' : ''), active ? '#7dd3fc' : virt ? '#d8b4fe' : '#fcd34d', s * 0.022);
      t.position.set(ax[0] * len * 1.15, ax[1] * len * 1.15, ax[2] * len * 1.15);
      holder.add(t);
    }
  }
}

function fit() {
  const s = design.upper_arm + design.forearm;
  controls.target.set(0, 0, -0.5 * s);
  camera.position.set(1.4 * s, -1.8 * s, 0.1 * s);
  camera.near = s / 200; camera.far = s * 40;
  camera.updateProjectionMatrix();
  grid.position.z = -(s + 0.1 * s);
  grid.scale.setScalar(Math.max(0.3, s * 1.5));
}

let axes = [];
function axesVisible() {
  axes.forEach((a) => a.parent && a.parent.remove(a));
  axes = [];
  if (!$('optAxes').checked) return;
  const size = (design.upper_arm + design.forearm) * 0.08;
  for (const link of Object.values(robot.links)) { const a = new THREE.AxesHelper(size); link.add(a); axes.push(a); }
}

src.addEventListener('change', load);
$('optPassive').addEventListener('change', () => {
  $('manualBox').hidden = $('optPassive').checked;
  if (!$('optPassive').checked) { for (const n of Object.keys(manual)) manual[n] = 0; loadManualZero(); }
  apply();
});
function loadManualZero() { for (const [n, r] of Object.entries(manualRows)) { manual[n] = 0; r.set(0); } }
$('btnManualFk').addEventListener('click', () => { loadManualFromFk(); apply(false); });
$('btnManualZero').addEventListener('click', () => { loadManualZero(); apply(false); });
$('optTool').addEventListener('change', apply);
$('optAxes').addEventListener('change', axesVisible);
$('optJointAxes').addEventListener('change', jointAxes);
$('btnFit').addEventListener('click', fit);
$('btnHome').addEventListener('click', () => { const h = design.homeTheta; q = [h, h, h]; apply(); });
let wiggle = false;
$('btnWiggle').addEventListener('click', () => { wiggle = !wiggle; $('btnWiggle').textContent = wiggle ? '멈추기' : '자동 움직이기'; });
$('btnDownload').addEventListener('click', () => {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([urdfText], { type: 'application/xml' }));
  a.download = 'delta_robot.urdf';
  a.click();
});
$('btnCopy').addEventListener('click', () => navigator.clipboard.writeText(urdfText).then(() => { $('btnCopy').textContent = '복사됨'; }));

load();

// ------------------------------------------------------------------ joystick: move the effector with IK
const joy = new Joystick($('joyHost'), {
  buttonLabel: '툴 ON/OFF', speed: 80, minSpeed: 5, maxSpeed: 400, step: 5,
  onButton: () => { $('optTool').checked = !$('optTool').checked; apply(false); },
});
$('optJoy').addEventListener('change', () => { $('joyHost').hidden = !$('optJoy').checked; });
let lastJoy = performance.now();
function joyStep() {
  const now = performance.now();
  const dt = Math.min(0.1, (now - lastJoy) / 1000);
  lastJoy = now;
  if ($('joyHost').hidden) return;
  const v = joy.value();
  if (!v.x && !v.y && !v.z) return;
  wiggle = false; $('btnWiggle').textContent = '자동 움직이기';
  const p = tryFk(design, q);
  if (!p) return;
  const res = stepTcp([p[0], p[1], p[2] - design.toolLength], v, joy.speed(), dt, (t) => {
    const rep = limitReport(design, [t[0], t[1], t[2] + design.toolLength]);
    if (!rep.ok) return false;
    q = rep.theta.slice();
    return true;
  });
  apply();
  $('tcpMsg').className = 'u-msg' + (res === 'blocked' || res === 'slide' ? ' bad' : '');
  $('tcpMsg').textContent = res === 'blocked' ? '작업영역 경계 — 더 갈 수 없습니다' : res === 'slide' ? '작업영역 경계를 따라 이동 중' :
    'θ = ' + q.map((t) => (t * DEG).toFixed(1) + '°').join(', ');
}

const t0 = performance.now();
(function loop() {
  joyStep();
  if (wiggle) {
    const t = (performance.now() - t0) / 1000, h = design.homeTheta;
    q = [h + 0.3 * Math.sin(t * 1.3), h + 0.3 * Math.sin(t * 1.3 + 2.1), h + 0.3 * Math.sin(t * 1.3 + 4.2)];
    apply();
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
})();
