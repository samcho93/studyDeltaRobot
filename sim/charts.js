// charts.js — small time-series charts on <canvas> (no chart library).
// Series colours: motor 1/2/3 = blue / orange / green (distinguishable for colour-blind users).

export const SERIES_COLORS = ['#3b82f6', '#f59e0b', '#10b981'];

function css(name, fb) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb;
}

export class TimeChart {
  /** opts: { title, unit, labels, limits: [{v, label, color}], symmetric } */
  constructor(canvas, opts) {
    this.canvas = canvas;
    this.opts = opts;
    this.t = [];
    this.ch = [[], [], []];
  }

  setData(t, channels) { this.t = t; this.ch = channels; }
  setLimits(limits) { this.opts.limits = limits; }

  draw(cursor = null, span = null) {
    const cv = this.canvas;
    const dpr = Math.min(window_dpr(), 2);
    const w = cv.clientWidth || 300, h = cv.clientHeight || 120;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    }
    const g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    const padL = 40, padR = 8, padT = 18, padB = 16;
    const pw = w - padL - padR, ph = h - padT - padB;
    const n = this.t.length;
    const grid = css('--border-soft', '#1c2532'), faint = css('--text-faint', '#64748b'), text = css('--text-dim', '#9aa8bd');
    g.font = `10px ${css('--mono', 'monospace')}`;
    g.fillStyle = text;
    g.fillText(`${this.opts.title} [${this.opts.unit}]`, padL, 12);
    if (n < 2) {
      g.fillStyle = faint;
      g.fillText('데이터 없음 — 작업 탭에서 프로그램을 실행하세요', padL, padT + ph / 2);
      return;
    }
    let t0 = this.t[0], t1 = this.t[n - 1];
    if (span) { t0 = Math.max(t0, t1 - span); }
    let lo = Infinity, hi = -Infinity;
    for (const c of this.ch) for (let i = 0; i < n; i++) {
      if (this.t[i] < t0) continue;
      const v = c[i];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    for (const L of this.opts.limits || []) {
      if (L.fit === false) continue;
      lo = Math.min(lo, L.v); hi = Math.max(hi, L.v);
    }
    if (this.opts.symmetric) { const m = Math.max(Math.abs(lo), Math.abs(hi)); lo = -m; hi = m; }
    if (!(hi > lo)) { hi = lo + 1; }
    const pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
    const X = (t) => padL + pw * (t - t0) / Math.max(1e-9, t1 - t0);
    const Yv = (v) => padT + ph * (1 - (v - lo) / (hi - lo));

    g.strokeStyle = grid; g.lineWidth = 1; g.fillStyle = faint;
    for (let k = 0; k <= 2; k++) {
      const v = hi - (hi - lo) * k / 2, y = Yv(v);
      g.beginPath(); g.moveTo(padL, y + 0.5); g.lineTo(w - padR, y + 0.5); g.stroke();
      g.fillText(fmt(v), 2, y + 3);
    }
    const xu = this.opts.xUnit === undefined ? ' s' : this.opts.xUnit;
    g.fillText(fmt(t0) + xu, padL, h - 3);
    g.textAlign = 'right'; g.fillText(fmt(t1) + xu, w - padR, h - 3); g.textAlign = 'left';
    if (lo < 0 && hi > 0) {
      g.strokeStyle = faint; g.globalAlpha = 0.5;
      g.beginPath(); g.moveTo(padL, Yv(0)); g.lineTo(w - padR, Yv(0)); g.stroke();
      g.globalAlpha = 1;
    }
    for (const L of this.opts.limits || []) {
      g.strokeStyle = L.color || '#ef4444'; g.setLineDash([5, 4]); g.lineWidth = 1.2;
      g.beginPath(); g.moveTo(padL, Yv(L.v)); g.lineTo(w - padR, Yv(L.v)); g.stroke();
      g.setLineDash([]);
      if (L.label) { g.fillStyle = L.color || '#ef4444'; g.fillText(L.label, padL + 4, Yv(L.v) - 3); }
    }
    this.ch.forEach((c, ci) => {
      g.strokeStyle = SERIES_COLORS[ci % 3]; g.lineWidth = 1.5;
      g.beginPath();
      let started = false;
      const step = Math.max(1, Math.floor(n / (pw * 2)));
      for (let i = 0; i < n; i += step) {
        if (this.t[i] < t0) continue;
        const x = X(this.t[i]), y = Yv(c[i]);
        if (started) g.lineTo(x, y); else { g.moveTo(x, y); started = true; }
      }
      g.stroke();
    });
    if (this.opts.labels) {
      let x = w - padR;
      g.textAlign = 'right';
      for (let ci = this.opts.labels.length - 1; ci >= 0; ci--) {
        const lab = this.opts.labels[ci];
        g.fillStyle = SERIES_COLORS[ci % 3];
        g.fillText(lab, x, 12);
        x -= g.measureText(lab).width + 8;
      }
      g.textAlign = 'left';
    }
    if (cursor !== null && cursor >= t0 && cursor <= t1) {
      g.strokeStyle = css('--text', '#e4eaf2'); g.globalAlpha = 0.6;
      g.beginPath(); g.moveTo(X(cursor), padT); g.lineTo(X(cursor), padT + ph); g.stroke();
      g.globalAlpha = 1;
    }
  }
}

function window_dpr() { return (typeof window !== 'undefined' && window.devicePixelRatio) || 1; }
function fmt(v) {
  const a = Math.abs(v);
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  return v.toFixed(2);
}
