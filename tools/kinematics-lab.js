// kinematics-lab.js — 2D views of delta IK (circle ∩ circle in the arm plane) and FK (three spheres).
import { Design, loadCatalog } from '../assets/js/delta/design.js';
import { PHI, elbow, armAxis, tryIk, limitReport, vec } from '../assets/js/delta/kinematics.js';

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const cat = await loadCatalog('../python/deltarobot/data/catalog.json');
const css = (n, fb) => getComputedStyle(document.documentElement).getPropertyValue(n).trim() || fb;
const COL = ['#3b82f6', '#f59e0b', '#10b981'];

let d, mode = 'ik', arm = 0, p = [0, 0, -0.3], q = [0.35, 0.35, 0.35];
const LS = 'studydelta.design.v1';
const saved = (() => { try { return JSON.parse(localStorage.getItem(LS) || 'null'); } catch (e) { return null; } })();
$('preset').innerHTML = (saved && saved.design ? '<option value="__current">시뮬레이터의 현재 설계</option>' : '') +
  Object.entries(cat.presets).map(([k, v]) => `<option value="${k}">${v.name}</option>`).join('');
$('preset').addEventListener('change', setDesign);

function slider(host, label, lo, hi, step, get, set) {
  const row = document.createElement('div');
  row.className = 'joint-row';
  row.innerHTML = `<div class="jr-head"><span class="jr-name">${label}</span><span class="jr-val"></span></div><input type="range" min="${lo}" max="${hi}" step="${step}">`;
  const r = row.querySelector('input'), v = row.querySelector('.jr-val');
  r.addEventListener('input', () => { set(Number(r.value)); draw(); });
  host.appendChild(row);
  return () => { r.value = get(); v.textContent = Number(get()).toFixed(step < 1 ? 1 : 0); };
}
let paints = [];
function setDesign() {
  const k = $('preset').value;
  d = new Design(k === '__current' ? saved.design : cat.presets[k].design);
  const h = d.homeTheta;
  q = [h, h, h];
  p = [0, 0, fkBoth(q).low[2]];
  const reach = d.base_radius - d.effector_radius + d.upper_arm + d.forearm;
  $('pSliders').innerHTML = ''; $('qSliders').innerHTML = '';
  paints = [
    slider($('pSliders'), 'x', -reach * 1000, reach * 1000, 1, () => p[0] * 1000, (v) => { p[0] = v / 1000; }),
    slider($('pSliders'), 'y', -reach * 1000, reach * 1000, 1, () => p[1] * 1000, (v) => { p[1] = v / 1000; }),
    slider($('pSliders'), 'z', -(d.upper_arm + d.forearm) * 1000, 0, 1, () => p[2] * 1000, (v) => { p[2] = v / 1000; }),
    ...[0, 1, 2].map((i) => slider($('qSliders'), 'θ' + (i + 1), d.theta_min * DEG, d.theta_max * DEG, 0.1, () => q[i] * DEG, (v) => { q[i] = v / DEG; })),
  ];
  draw();
}

// ---- FK with both solutions (same construction as kinematics.fk)
function fkBoth(th) {
  const l = d.forearm;
  const c = [0, 1, 2].map((i) => vec.sub(elbow(d, i, th[i]), vec.scale(armAxis(i), d.effector_radius)));
  const exv = vec.sub(c[1], c[0]), dd = vec.norm(exv), ex = vec.scale(exv, 1 / dd);
  const t = vec.sub(c[2], c[0]), i_ = vec.dot(ex, t);
  const eyv = vec.sub(t, vec.scale(ex, i_)), ey = vec.scale(eyv, 1 / vec.norm(eyv)), ez = vec.cross(ex, ey);
  const j = vec.dot(ey, t);
  const x = dd / 2, y = (i_ * i_ + j * j - 2 * i_ * x) / (2 * j);
  const h2 = l * l - x * x - y * y;
  const base = vec.add(c[0], vec.add(vec.scale(ex, x), vec.scale(ey, y)));
  if (h2 < 0) return { c, ok: false, dd, i_, j, x, y, h2 };
  const h = Math.sqrt(h2);
  const p1 = vec.add(base, vec.scale(ez, h)), p2 = vec.sub(base, vec.scale(ez, h));
  return { c, ok: true, dd, i_, j, x, y, h2, low: p1[2] < p2[2] ? p1 : p2, high: p1[2] < p2[2] ? p2 : p1 };
}

// ---- IK details for one arm
function ikDetail(i, pt) {
  const c = Math.cos(PHI[i]), s = Math.sin(PHI[i]);
  const xa = pt[0] * c + pt[1] * s, ya = -pt[0] * s + pt[1] * c, z = pt[2];
  const a = xa + d.effector_radius - d.base_radius;
  const L = d.upper_arm, l = d.forearm;
  const K = (l * l - L * L - a * a - ya * ya - z * z) / (2 * L);
  const rho = Math.hypot(a, z);
  const out = { xa, ya, z, a, K, rho, ok: Math.abs(K) <= rho && rho > 1e-12 };
  if (!out.ok) return out;
  const alpha = Math.atan2(a, z), asn = Math.asin(K / rho);
  const norm = (t) => Math.atan2(Math.sin(t), Math.cos(t));
  out.alpha = alpha; out.asn = asn;
  out.t1 = norm(alpha + asn); out.t2 = norm(alpha + Math.PI - asn);
  out.pick = Math.cos(out.t1) > Math.cos(out.t2) ? out.t1 : out.t2;
  out.other = out.pick === out.t1 ? out.t2 : out.t1;
  return out;
}

// ---- canvas helpers
function setup(cv) {
  const dpr = Math.min(devicePixelRatio || 1, 2), w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  return { g, w, h };
}
function line(g, a, b, col, wd = 2, dash = null) { g.strokeStyle = col; g.lineWidth = wd; g.setLineDash(dash || []); g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); g.setLineDash([]); }
function dot(g, a, r, col) { g.fillStyle = col; g.beginPath(); g.arc(a[0], a[1], r, 0, 2 * Math.PI); g.fill(); }
function circle(g, a, r, col, dash = [4, 4]) { g.strokeStyle = col; g.lineWidth = 1.2; g.setLineDash(dash); g.beginPath(); g.arc(a[0], a[1], r, 0, 2 * Math.PI); g.stroke(); g.setLineDash([]); }
function label(g, a, text, col) { g.fillStyle = col; g.font = `11px ${css('--mono', 'monospace')}`; g.fillText(text, a[0] + 6, a[1] - 6); }

let topMap = null;
function drawTop(pt, th) {
  const cv = $('top'), { g, w, h } = setup(cv);
  const reach = d.base_radius + d.upper_arm + d.forearm * 0.6;
  const s = Math.min(w, h) / (2.3 * reach), cx = w / 2, cy = h / 2 + 10;
  const M = (v) => [cx + v[0] * s, cy - v[1] * s];
  topMap = { s, cx, cy };
  const faint = css('--text-faint', '#64748b'), text = css('--text', '#e4eaf2');
  line(g, M([-reach, 0]), M([reach, 0]), css('--border', '#253040'), 1);
  line(g, M([0, -reach]), M([0, reach]), css('--border', '#253040'), 1);
  circle(g, M([0, 0]), d.base_radius * s, faint);
  label(g, M([reach * 0.92, 0]), 'x', faint); label(g, M([0, reach * 0.92]), 'y', faint);
  const ok = th && th.every(Number.isFinite);
  for (let i = 0; i < 3; i++) {
    const mp = vec.scale(armAxis(i), d.base_radius);
    dot(g, M(mp), 5, COL[i]);
    label(g, M(mp), '모터' + (i + 1), COL[i]);
    if (ok) {
      const e = elbow(d, i, th[i]);
      const b = vec.add(pt, vec.scale(armAxis(i), d.effector_radius));
      line(g, M(mp), M(e), COL[i], 4);
      line(g, M(e), M(b), COL[i], 1.5);
      dot(g, M(e), 3.5, COL[i]);
      dot(g, M(b), 3, COL[i]);
    }
  }
  // effector hexagon
  g.strokeStyle = text; g.lineWidth = 1.5; g.beginPath();
  for (let k = 0; k <= 6; k++) { const a = k * Math.PI / 3; const v = M([pt[0] + (d.effector_radius + 0.008) * Math.cos(a), pt[1] + (d.effector_radius + 0.008) * Math.sin(a)]); k ? g.lineTo(...v) : g.moveTo(...v); }
  g.stroke();
  dot(g, M(pt), 6, ok ? css('--accent', '#38bdf8') : css('--danger', '#f87171'));
  label(g, M(pt), `P (${(pt[0] * 1000).toFixed(0)}, ${(pt[1] * 1000).toFixed(0)}, ${(pt[2] * 1000).toFixed(0)}) mm`, text);
  if (mode === 'ik') {
    const mp = vec.scale(armAxis(arm), d.base_radius);
    line(g, M([0, 0]), M(vec.scale(armAxis(arm), reach)), COL[arm], 1, [2, 4]);
    label(g, M(vec.scale(armAxis(arm), reach * 0.8)), `x′ (팔 ${arm + 1} 방향 φ=${(PHI[arm] * DEG).toFixed(0)}°)`, COL[arm]);
    void mp;
  }
}

function drawSideIk(pt, det) {
  const cv = $('side'), { g, w, h } = setup(cv);
  const L = d.upper_arm, l = d.forearm;
  const span = d.base_radius + L + l * 0.8;
  const s = Math.min(w / (1.5 * span), h / (1.3 * (L + l)));
  const ox = w * 0.15, oy = h * 0.14;
  const M = (x, z) => [ox + x * s, oy - z * s];
  const faint = css('--text-faint', '#64748b'), text = css('--text', '#e4eaf2'), acc = css('--accent', '#38bdf8');
  line(g, M(-0.02, 0), M(span, 0), css('--border', '#253040'), 1);
  line(g, M(0, 0.02), M(0, -(L + l)), css('--border', '#253040'), 1);
  label(g, M(span * 0.95, 0), 'x′', faint); label(g, M(0, -(L + l) * 0.97), 'z', faint);
  const mot = M(d.base_radius, 0);
  dot(g, mot, 5, COL[arm]); label(g, mot, '모터 (R, 0)', COL[arm]);
  circle(g, mot, L * s, COL[arm]);                       // elbow locus
  const B = [det.xa + d.effector_radius, det.z];           // ball joint pair in the arm plane
  const rr = Math.sqrt(Math.max(0, l * l - det.ya * det.ya));
  dot(g, M(...B), 4, text); label(g, M(...B), 'B′ (x′+r, z)', text);
  circle(g, M(...B), rr * s, faint);
  label(g, [M(...B)[0] + rr * s * 0.7, M(...B)[1] - rr * s * 0.7], `반지름 √(l²−y′²) = ${(rr * 1000).toFixed(1)} mm`, faint);
  if (det.ok) {
    for (const [t, chosen] of [[det.other, false], [det.pick, true]]) {
      const e = [d.base_radius + L * Math.cos(t), -L * Math.sin(t)];
      line(g, mot, M(...e), chosen ? COL[arm] : faint, chosen ? 5 : 2, chosen ? null : [5, 4]);
      line(g, M(...e), M(...B), chosen ? COL[arm] : faint, chosen ? 2 : 1.2, chosen ? null : [5, 4]);
      dot(g, M(...e), 4, chosen ? acc : faint);
      label(g, M(...e), `θ=${(t * DEG).toFixed(1)}° ${chosen ? '✔ (cos 큼)' : '✖'}`, chosen ? acc : faint);
    }
  } else {
    g.fillStyle = css('--danger', '#f87171'); g.font = '13px sans-serif';
    g.fillText('두 원이 만나지 않음 → |K| > ρ, 도달 불가', 14, h - 16);
  }
  $('capSide').textContent = `팔 ${arm + 1}의 평면 (x′–z): 원(위팔 L) ∩ 원(로드의 투영)`;
}

function drawSideFk(res) {
  const cv = $('side'), { g, w, h } = setup(cv);
  const L = d.upper_arm, l = d.forearm;
  const span = d.base_radius + L + l;
  const s = Math.min(w / (2.4 * span), h / (2.4 * (L + l)));
  const cx = w / 2, cy = h * 0.45;
  const M = (v) => [cx + v[0] * s, cy - v[2] * s];
  const faint = css('--text-faint', '#64748b'), text = css('--text', '#e4eaf2'), acc = css('--accent', '#38bdf8');
  line(g, M([-span, 0, 0]), M([span, 0, 0]), css('--border', '#253040'), 1);
  label(g, M([span * 0.9, 0, 0]), 'x', faint); label(g, [cx + 4, 14], 'z', faint);
  res.c.forEach((c, i) => { dot(g, M(c), 4, COL[i]); circle(g, M(c), l * s, COL[i], [3, 5]); label(g, M(c), 'C' + (i + 1), COL[i]); });
  if (res.ok) {
    dot(g, M(res.low), 6, acc); label(g, M(res.low), `아래 해 (선택) z=${(res.low[2] * 1000).toFixed(1)}`, acc);
    dot(g, M(res.high), 5, faint); label(g, M(res.high), `위 해 z=${(res.high[2] * 1000).toFixed(1)}`, faint);
  } else {
    g.fillStyle = css('--danger', '#f87171'); g.font = '13px sans-serif';
    g.fillText('세 구가 한 점에서 만나지 않음 (h² < 0) — 조립 불가', 14, h - 16);
  }
  $('capSide').textContent = '정면 투영 (x–z): 반지름 l인 세 구 — 중심 Cᵢ = Eᵢ − r·uᵢ';
}

const f3 = (v) => (v * 1000).toFixed(2);
function draw() {
  if (!d) return;
  paints.forEach((f) => f());
  if (mode === 'ik') {
    const th = tryIk(d, p);
    const rep = limitReport(d, p);
    const det = ikDetail(arm, p);
    drawTop(p, th);
    drawSideIk(p, det);
    let s = `<b>팔 ${arm + 1}</b>, φ = ${(PHI[arm] * DEG).toFixed(0)}°\n`;
    s += `x′ =  x cosφ + y sinφ = ${f3(det.xa)} mm\n`;
    s += `y′ = −x sinφ + y cosφ = ${f3(det.ya)} mm\n`;
    s += `a  = x′ + r − R        = ${f3(det.a)} mm\n`;
    s += `K  = (l² − L² − a² − y′² − z²)/(2L) = ${f3(det.K)} mm\n`;
    s += `ρ  = √(a² + z²)         = ${f3(det.rho)} mm\n`;
    if (det.ok) {
      s += `α  = atan2(a, z)        = ${(det.alpha * DEG).toFixed(2)}°\n`;
      s += `asin(K/ρ)               = ${(det.asn * DEG).toFixed(2)}°\n`;
      s += `θa = α + asin           = ${(det.t1 * DEG).toFixed(2)}°  (cos ${Math.cos(det.t1).toFixed(3)})\n`;
      s += `θb = α + π − asin       = ${(det.t2 * DEG).toFixed(2)}°  (cos ${Math.cos(det.t2).toFixed(3)})\n`;
      s += `<b>선택 θ${arm + 1} = ${(det.pick * DEG).toFixed(2)}°</b>\n`;
    } else s += '|K| > ρ → 도달 불가 (Unreachable)\n';
    s += `\n세 팔: ${th ? th.map((v) => (v * DEG).toFixed(2) + '°').join(', ') : '해 없음'}\n`;
    s += `한계 검사: ${rep.ok ? 'OK' : rep.problems.join(', ')}`;
    $('steps').innerHTML = s;
  } else {
    const res = fkBoth(q);
    drawTop(res.ok ? res.low : [0, 0, 0], res.ok ? q : null);
    drawSideFk(res);
    let s = res.c.map((c, i) => `C${i + 1} = (${f3(c[0])}, ${f3(c[1])}, ${f3(c[2])}) mm`).join('\n') + '\n';
    s += `d  = |C2 − C1| = ${f3(res.dd)} mm\ni  = ex·(C3 − C1) = ${f3(res.i_)} mm\nj  = ey·(C3 − C1) = ${f3(res.j)} mm\n`;
    s += `x  = d/2 = ${f3(res.x)} mm\ny  = (i² + j² − 2ix)/(2j) = ${f3(res.y)} mm\nh² = l² − x² − y² = ${(res.h2 * 1e6).toFixed(1)} mm²\n`;
    s += res.ok ? `<b>P = (${f3(res.low[0])}, ${f3(res.low[1])}, ${f3(res.low[2])}) mm</b>  (z가 작은 해)` : 'h² < 0 → 해 없음';
    $('steps').innerHTML = s;
  }
}

// ---- interactions
$('modeIk').addEventListener('click', () => { mode = 'ik'; $('modeIk').classList.add('on'); $('modeFk').classList.remove('on'); $('ikBox').hidden = false; $('fkBox').hidden = true; const th = tryIk(d, p); if (th) q = th; draw(); });
$('modeFk').addEventListener('click', () => { mode = 'fk'; $('modeFk').classList.add('on'); $('modeIk').classList.remove('on'); $('ikBox').hidden = true; $('fkBox').hidden = false; const th = tryIk(d, p); if (th) q = th; draw(); });
document.querySelectorAll('#armSeg button').forEach((b) => b.addEventListener('click', () => {
  arm = Number(b.dataset.arm);
  document.querySelectorAll('#armSeg button').forEach((x) => x.classList.toggle('on', x === b));
  draw();
}));
let dragging = false;
const topCv = $('top');
function dragTo(ev) {
  if (!topMap || mode !== 'ik') return;
  const r = topCv.getBoundingClientRect();
  p[0] = (ev.clientX - r.left - topMap.cx) / topMap.s;
  p[1] = -(ev.clientY - r.top - topMap.cy) / topMap.s;
  draw();
}
topCv.addEventListener('pointerdown', (e) => { dragging = true; topCv.setPointerCapture(e.pointerId); dragTo(e); });
topCv.addEventListener('pointermove', (e) => { if (dragging) dragTo(e); });
topCv.addEventListener('pointerup', () => { dragging = false; });
window.addEventListener('resize', draw);
document.addEventListener('themechange', draw);
setDesign();
