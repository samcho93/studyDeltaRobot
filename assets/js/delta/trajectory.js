// trajectory.js — time scaling profiles and paths. Mirror of python/deltarobot/trajectory.py.

export const PROFILES = ['trapezoid', 'scurve', 'quintic', 'cycloidal'];
export const PROFILE_KO = { trapezoid: '사다리꼴', scurve: 'S-커브', quintic: '5차 다항식', cycloidal: '사이클로이드' };

export class Profile {
  constructor(kind, d, vmax, amax) {
    if (!PROFILES.includes(kind)) throw new Error('profile must be one of ' + PROFILES.join(', '));
    if (!(vmax > 0) || !(amax > 0)) throw new Error('vmax and amax must be positive');
    this.kind = kind; this.d = Math.max(0, d); this.vmax = vmax; this.amax = amax;
    d = this.d;
    if (d === 0) { this.T = 0; return; }
    if (kind === 'trapezoid') {
      let ta = vmax / amax;
      if (amax * ta * ta >= d) { ta = Math.sqrt(d / amax); this.vp = amax * ta; this.ta = ta; this.tc = 0; }
      else { this.vp = vmax; this.ta = ta; this.tc = (d - vmax * ta) / vmax; }
      this.T = 2 * this.ta + this.tc;
    } else if (kind === 'scurve') {
      const A = amax;
      let ta = 2 * vmax / A;
      if (A * ta * ta / 2 >= d) { ta = Math.sqrt(2 * d / A); this.vp = A * ta / 2; this.ta = ta; this.tc = 0; }
      else { this.vp = vmax; this.ta = ta; this.tc = (d - vmax * ta) / vmax; }
      this.A = A;
      this.T = 2 * this.ta + this.tc;
    } else if (kind === 'quintic') {
      this.T = Math.max(15 * d / (8 * vmax), Math.sqrt(10 * d / (Math.sqrt(3) * amax)));
    } else {
      this.T = Math.max(2 * d / vmax, Math.sqrt(2 * Math.PI * d / amax));
    }
  }

  at(t) {
    const d = this.d, T = this.T;
    if (T <= 0) return [0, 0, 0];
    t = Math.min(Math.max(t, 0), T);
    const k = this.kind;
    if (k === 'trapezoid') {
      const { ta, tc, vp } = this;
      const a = vp / ta;
      if (t < ta) return [0.5 * a * t * t, a * t, a];
      if (t < ta + tc) return [0.5 * a * ta * ta + vp * (t - ta), vp, 0];
      const u = T - t;
      return [d - 0.5 * a * u * u, a * u, t < T ? -a : 0];
    }
    if (k === 'scurve') {
      const { ta, tc, vp } = this;
      const A = 2 * vp / ta;
      const w = Math.PI / ta;
      const acc = (u) => [
        A * (u * u / 4 - (1 - Math.cos(2 * w * u)) / (8 * w * w)),
        A * (u / 2 - Math.sin(2 * w * u) / (4 * w)),
        A * Math.sin(w * u) ** 2,
      ];
      if (t < ta) return acc(t);
      const sa = acc(ta)[0];
      if (t < ta + tc) return [sa + vp * (t - ta), vp, 0];
      const [s, v, a] = acc(T - t);
      return [d - s, v, -a];
    }
    const x = t / T;
    if (k === 'quintic') {
      return [d * (10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5),
        d * (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / T,
        d * (60 * x - 180 * x ** 2 + 120 * x ** 3) / (T * T)];
    }
    return [d * (x - Math.sin(2 * Math.PI * x) / (2 * Math.PI)),
      d * (1 - Math.cos(2 * Math.PI * x)) / T,
      d * 2 * Math.PI * Math.sin(2 * Math.PI * x) / (T * T)];
  }
}

const lerp = (a, b, u) => [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

export class Path {
  constructor() { this.segs = []; this.length = 0; }
  line(p0, p1) {
    const n = dist(p0, p1);
    if (n > 1e-12) { this.segs.push({ type: 'line', p0: p0.slice(), p1: p1.slice(), len: n }); this.length += n; }
    return this;
  }
  arc(c, u, w, radius, angle) {
    const n = radius * Math.abs(angle);
    if (n > 1e-12) { this.segs.push({ type: 'arc', c, u, w, radius, angle, len: n }); this.length += n; }
    return this;
  }
  point(s) {
    s = Math.min(Math.max(s, 0), this.length);
    for (let i = 0; i < this.segs.length; i++) {
      const g = this.segs[i];
      if (s <= g.len || i === this.segs.length - 1) {
        if (g.type === 'line') return lerp(g.p0, g.p1, g.len ? s / g.len : 0);
        const a = g.angle * (s / g.len), ca = Math.cos(a), sa = Math.sin(a);
        return [0, 1, 2].map((k) => g.c[k] + g.radius * (ca * g.u[k] + sa * g.w[k]));
      }
      s -= g.len;
    }
    return [0, 0, 0];
  }
}

export const linePath = (p0, p1) => new Path().line(p0, p1);

export function archPath(p0, p1, height, radius = 0) {
  const ztop = Math.max(p0[2], p1[2]) + height;
  const a = [p0[0], p0[1], ztop], b = [p1[0], p1[1], ztop];
  const horiz = dist(a, b);
  const path = new Path();
  if (horiz < 1e-9) return height > 0 ? path.line(p0, a).line(a, p1) : path.line(p0, p1);
  let rad = radius || height / 2;
  rad = Math.max(0, Math.min(rad, ztop - p0[2], ztop - p1[2], horiz / 2));
  const h = [(b[0] - a[0]) / horiz, (b[1] - a[1]) / horiz, 0];
  const up = [0, 0, 1];
  path.line(p0, [a[0], a[1], ztop - rad]);
  if (rad > 0) path.arc([a[0] + h[0] * rad, a[1] + h[1] * rad, ztop - rad], [-h[0], -h[1], 0], up, rad, Math.PI / 2);
  path.line([a[0] + h[0] * rad, a[1] + h[1] * rad, ztop], [b[0] - h[0] * rad, b[1] - h[1] * rad, ztop]);
  if (rad > 0) path.arc([b[0] - h[0] * rad, b[1] - h[1] * rad, ztop - rad], up, h, rad, Math.PI / 2);
  path.line([b[0], b[1], ztop - rad], p1);
  return path;
}

export function samplePath(path, kind, vmax, amax, dt, t0 = 0) {
  const prof = new Profile(kind, path.length, vmax, amax);
  const out = [];
  const n = Math.max(1, Math.ceil(prof.T / dt));
  for (let k = 1; k <= n; k++) {
    const t = Math.min(prof.T, k * dt);
    out.push([t0 + t, path.point(prof.at(t)[0])]);
  }
  return out;
}

export function sampleJoint(q0, q1, kind, wmax, alpha, dt, t0 = 0) {
  const dq = [0, 1, 2].map((i) => q1[i] - q0[i]);
  const big = Math.max(...dq.map(Math.abs));
  const prof = new Profile(kind, big, wmax, alpha);
  const out = [];
  const n = Math.max(1, Math.ceil(prof.T / dt));
  for (let k = 1; k <= n; k++) {
    const t = Math.min(prof.T, k * dt);
    const u = big > 0 ? prof.at(t)[0] / big : 1;
    out.push([t0 + t, [0, 1, 2].map((i) => q0[i] + dq[i] * u)]);
  }
  return out;
}
