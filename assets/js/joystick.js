// joystick.js — on-screen joystick (XY pad + Z lever + button) with Gamepad API support.
// Velocity-style control: value() returns {x, y, z} in [-1, 1] while held, 0 when released.
// Used by the simulator (sim/app.js) and the URDF viewer (assets/js/urdf-viewer.js).

const DEAD = 0.08;           // dead zone (screen and gamepad)

function shape(v) {
  // dead zone + gentle curve so small deflections allow fine positioning
  const a = Math.abs(v);
  if (a < DEAD) return 0;
  const u = (a - DEAD) / (1 - DEAD);
  return Math.sign(v) * u * u * 0.6 + Math.sign(v) * u * 0.4;
}

export class Joystick {
  /**
   * @param host      element to put the widget in (positioned by the caller's CSS)
   * @param opts      { buttonLabel, onButton(), speedLabel, speed, minSpeed, maxSpeed, step }
   */
  constructor(host, opts = {}) {
    this.opts = opts;
    this.pad = { x: 0, y: 0 };
    this.lever = 0;
    this.gp = { x: 0, y: 0, z: 0 };
    this.gpBtn = false;
    this.gpName = '';
    const el = document.createElement('div');
    el.className = 'joy';
    el.innerHTML = `
      <div class="joy-head"><b>조이스틱</b><span class="joy-gp" hidden></span>
        <button type="button" class="joy-hide" title="접기/펴기" aria-label="조이스틱 접기">▾</button></div>
      <div class="joy-body">
        <div class="joy-pad" role="application" aria-label="XY 이동 패드 — 드래그">
          <span class="joy-ax joy-ax-x">+x</span><span class="joy-ax joy-ax-y">+y</span>
          <div class="joy-knob"></div>
        </div>
        <div class="joy-z" role="application" aria-label="Z 레버 — 드래그 (위 = +z)">
          <span class="joy-ax joy-ax-z">+z</span>
          <div class="joy-zknob"></div>
        </div>
      </div>
      <div class="joy-foot">
        <label class="joy-speed" title="최대 속도">속도 <input type="range" min="${opts.minSpeed ?? 5}" max="${opts.maxSpeed ?? 300}" step="${opts.step ?? 5}" value="${opts.speed ?? 80}"><output></output></label>
        ${opts.buttonLabel ? `<button type="button" class="joy-btn">${opts.buttonLabel}</button>` : ''}
      </div>`;
    host.appendChild(el);
    this.el = el;
    this.knob = el.querySelector('.joy-knob');
    this.zknob = el.querySelector('.joy-zknob');
    this.speedIn = el.querySelector('.joy-speed input');
    this.speedOut = el.querySelector('.joy-speed output');
    const paintSpeed = () => { this.speedOut.textContent = this.speedIn.value + ' mm/s'; };
    this.speedIn.addEventListener('input', paintSpeed);
    paintSpeed();
    el.querySelector('.joy-hide').addEventListener('click', () => {
      el.classList.toggle('folded');
      el.querySelector('.joy-hide').textContent = el.classList.contains('folded') ? '▴' : '▾';
    });
    const btn = el.querySelector('.joy-btn');
    if (btn) btn.addEventListener('click', () => opts.onButton && opts.onButton());
    this.button = btn;
    this.drag(el.querySelector('.joy-pad'), (u, v) => { this.pad = { x: u, y: -v }; this.paint(); },
      () => { this.pad = { x: 0, y: 0 }; this.paint(); });
    this.drag(el.querySelector('.joy-z'), (u, v) => { this.lever = -v; this.paint(); },
      () => { this.lever = 0; this.paint(); }, true);
    window.addEventListener('gamepadconnected', (e) => this.showGamepad(e.gamepad.id));
    window.addEventListener('gamepaddisconnected', () => this.showGamepad(''));
    this.paint();
  }

  /** Pointer drag inside `area`; cb(u, v) with u, v in [-1, 1] relative to the centre. */
  drag(area, cb, release, vertical = false) {
    let id = null;
    const at = (e) => {
      const r = area.getBoundingClientRect();
      let u = ((e.clientX - r.left) / r.width) * 2 - 1;
      let v = ((e.clientY - r.top) / r.height) * 2 - 1;
      if (vertical) u = 0;
      const m = Math.hypot(u, v);
      if (m > 1) { u /= m; v /= m; }
      cb(u, v);
    };
    area.addEventListener('pointerdown', (e) => { id = e.pointerId; area.setPointerCapture(id); area.classList.add('on'); at(e); e.preventDefault(); });
    area.addEventListener('pointermove', (e) => { if (e.pointerId === id) at(e); });
    const up = (e) => { if (e.pointerId !== id) return; id = null; area.classList.remove('on'); release(); };
    area.addEventListener('pointerup', up);
    area.addEventListener('pointercancel', up);
    area.addEventListener('lostpointercapture', () => { if (id !== null) { id = null; area.classList.remove('on'); release(); } });
  }

  paint() {
    const x = this.pad.x || this.gp.x, y = this.pad.y || this.gp.y, z = this.lever || this.gp.z;
    this.knob.style.transform = `translate(${x * 34}px, ${-y * 34}px)`;
    this.zknob.style.transform = `translateY(${-z * 34}px)`;
  }

  showGamepad(name) {
    this.gpName = name;
    const tag = this.el.querySelector('.joy-gp');
    tag.hidden = !name;
    tag.textContent = name ? '🎮 게임패드' : '';
    tag.title = name;
  }

  /** Read a connected gamepad (left stick = x/y, right stick vertical = z, A / cross = button). */
  pollGamepad() {
    const pads = (navigator.getGamepads && navigator.getGamepads()) || [];
    const g = [...pads].find((p) => p && p.connected);
    if (!g) { this.gp = { x: 0, y: 0, z: 0 }; return; }
    if (!this.gpName) this.showGamepad(g.id);
    const ax = (i) => (g.axes[i] ?? 0);
    this.gp = { x: shape(ax(0)), y: shape(-ax(1)), z: shape(-ax(3)) };
    const pressed = !!(g.buttons[0] && g.buttons[0].pressed);
    if (pressed && !this.gpBtn && this.opts.onButton) this.opts.onButton();
    this.gpBtn = pressed;
    this.paint();
  }

  /** Current command in [-1, 1] per axis (screen widget wins over the gamepad). */
  value() {
    this.pollGamepad();
    return {
      x: this.pad.x ? shape(this.pad.x) : this.gp.x,
      y: this.pad.y ? shape(this.pad.y) : this.gp.y,
      z: this.lever ? shape(this.lever) : this.gp.z,
    };
  }

  /** Maximum TCP speed [m/s]. */
  speed() { return Number(this.speedIn.value) / 1000; }

  get active() { const v = this.value(); return v.x !== 0 || v.y !== 0 || v.z !== 0; }

  setButtonLabel(t) { if (this.button) this.button.textContent = t; }
}

/**
 * One control step: move the TCP by velocity * dt, sliding along the reachable axes.
 * tryTcp(p) must return true (and apply the pose) if the TCP position p is allowed.
 */
export function stepTcp(tcp, v, speed, dt, tryTcp) {
  const d = [v.x * speed * dt, v.y * speed * dt, v.z * speed * dt];
  if (!d[0] && !d[1] && !d[2]) return 'idle';
  if (tryTcp([tcp[0] + d[0], tcp[1] + d[1], tcp[2] + d[2]])) return 'ok';
  // blocked: try the axes one at a time so the effector slides along the workspace boundary
  let moved = false;
  const p = tcp.slice();
  for (let k = 0; k < 3; k++) {
    if (!d[k]) continue;
    const q = p.slice(); q[k] += d[k];
    if (tryTcp(q)) { p[k] = q[k]; moved = true; }
  }
  return moved ? 'slide' : 'blocked';
}
