// urdf-viewer.js — delta robot URDF viewer (three.js 0.186 + urdf-loader 0.13.1).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { Design, loadCatalog } from './delta/design.js';
import { tryFk, PHI } from './delta/kinematics.js';
import { generate, jointState } from './delta/urdf.js';

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
  $('nJoints').textContent = `${joints.length} (능동 3 · 수동 ${joints.filter((j) => j.name.startsWith('elbow')).length} · 가상 3 · 고정 ${joints.filter((j) => j.jointType === 'fixed').length})`;
  buildSliders();
  paintTree();
  const h = design.homeTheta;
  q = [h, h, h];
  apply();
  fit();
  axesVisible();
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

function apply() {
  const p = tryFk(design, q);
  sliders.forEach((s, i) => { s.rng.value = (q[i] * DEG).toFixed(1); s.val.textContent = (q[i] * DEG).toFixed(1) + '°'; });
  if (!p) { $('closure').textContent = '순기구학 해 없음'; return false; }
  const js = jointState(design, q);
  const passive = $('optPassive').checked;
  for (const [name, v] of Object.entries(js)) {
    const isActive = name.startsWith('motor');
    robot.setJointValue(name, isActive || passive ? v : 0);
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
  $('closure').textContent = (err * 1000).toFixed(3) + ' mm' + (passive ? '' : '  ← 사슬이 풀림');
  $('closure').style.color = err > 0.001 ? 'var(--danger)' : '';
  $('passiveTable').innerHTML = '<tr><th>관절</th><th>값</th></tr>' + Object.entries(js).filter(([n]) => !n.startsWith('motor')).map(([n, v]) =>
    `<tr><td>${n}</td><td>${n.startsWith('effector') ? (v * 1000).toFixed(1) + ' mm' : (v * DEG).toFixed(2) + '°'}</td></tr>`).join('');
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
$('optPassive').addEventListener('change', apply);
$('optAxes').addEventListener('change', axesVisible);
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
const t0 = performance.now();
(function loop() {
  if (wiggle) {
    const t = (performance.now() - t0) / 1000, h = design.homeTheta;
    q = [h + 0.3 * Math.sin(t * 1.3), h + 0.3 * Math.sin(t * 1.3 + 2.1), h + 0.3 * Math.sin(t * 1.3 + 4.2)];
    apply();
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
})();
