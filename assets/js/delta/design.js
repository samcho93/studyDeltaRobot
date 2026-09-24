// design.js — delta robot design parameters + derived masses/inertias.
// Mirror of python/deltarobot/design.py (keep formulas identical; tests/test_js_parity.py).

export const HUB_MASS = 0.02;
export const ELBOW_MASS = 0.01;
export const BALL_CUP_MASS = 0.004;
export const G = 9.81;

export const DEFAULTS = {
  base_radius: 0.1, effector_radius: 0.035, upper_arm: 0.13, forearm: 0.3, forearm_spacing: 0.05,
  theta_min: -0.7, theta_max: 1.5, ball_joint_limit: 0.6,
  upper_arm_material: 'al_tube', forearm_material: 'cfrp_tube',
  motor: 'dxl_xm430_w350', gearbox: 'none', gear_ratio: 1,
  tool: 'gripper_servo', payload: 0.2, effector_mass: 0.06,
};
export const KEYS = Object.keys(DEFAULTS);

let CATALOG = null;

/** Load python/deltarobot/data/catalog.json once (url relative to the calling page). */
export async function loadCatalog(url) {
  if (CATALOG) return CATALOG;
  const res = await fetch(url, { cache: 'no-cache' });
  if (!res.ok) throw new Error('catalog.json → HTTP ' + res.status);
  CATALOG = await res.json();
  return CATALOG;
}
export function setCatalog(cat) { CATALOG = cat; }
export function catalog() {
  if (!CATALOG) throw new Error('catalog not loaded');
  return CATALOG;
}

export class Design {
  constructor(params = {}) {
    for (const k of KEYS) this[k] = params[k] !== undefined ? params[k] : DEFAULTS[k];
  }
  static preset(name) {
    const p = catalog().presets[name];
    if (!p) throw new Error('unknown preset ' + name);
    return new Design(p.design);
  }
  toDict() { const o = {}; for (const k of KEYS) o[k] = this[k]; return o; }
  copy(changes = {}) { return new Design({ ...this.toDict(), ...changes }); }

  get motorSpec() { return catalog().motors[this.motor]; }
  get gearboxSpec() { return catalog().gearboxes[this.gearbox]; }
  get toolSpec() { return catalog().tools[this.tool]; }
  get ratio() { return this.motorSpec.integrated_gear ? 1 : Number(this.gear_ratio); }
  get efficiency() { return this.motorSpec.integrated_gear ? 1 : Number(this.gearboxSpec.efficiency); }
  get toolLength() { return Number(this.toolSpec.length); }

  density(mat) { return Number(catalog().materials[mat].linear_density); }
  get upperArmMass() { return this.upper_arm * this.density(this.upper_arm_material) + HUB_MASS; }
  get elbowMass() { return this.forearm_spacing * this.density(this.forearm_material) + ELBOW_MASS; }
  get forearmPairMass() { return 2 * this.forearm * this.density(this.forearm_material) + 4 * BALL_CUP_MASS; }
  get movingPlateMass() {
    return this.effector_mass + Number(this.toolSpec.mass) + this.payload + 3 * 0.5 * this.forearmPairMass;
  }
  get driveInertia() {
    const m = this.motorSpec;
    if (m.integrated_gear) return Number(m.output_inertia || 0);
    const n = this.ratio;
    return (Number(m.rotor_inertia) + Number(this.gearboxSpec.inertia)) * n * n;
  }
  get armInertia() {
    const L = this.upper_arm;
    return this.upperArmMass * L * L / 3 + (this.elbowMass + 0.5 * this.forearmPairMass) * L * L + this.driveInertia;
  }
  get gravityMoment() {
    const L = this.upper_arm;
    return this.upperArmMass * L / 2 + (this.elbowMass + 0.5 * this.forearmPairMass) * L;
  }
  get homeTheta() { return Math.max(this.theta_min, Math.min(this.theta_max, 0.35)); }

  /** Human readable problems (Korean), same rules as Python validate(). */
  validate() {
    const cat = catalog();
    const errs = [];
    for (const k of ['base_radius', 'effector_radius', 'upper_arm', 'forearm', 'forearm_spacing']) {
      if (!(this[k] > 0)) errs.push(k + ' 는 0보다 커야 합니다');
    }
    if (!cat.motors[this.motor]) errs.push('알 수 없는 모터: ' + this.motor);
    if (!cat.gearboxes[this.gearbox]) errs.push('알 수 없는 감속기: ' + this.gearbox);
    if (!cat.tools[this.tool]) errs.push('알 수 없는 툴: ' + this.tool);
    for (const m of [this.upper_arm_material, this.forearm_material]) {
      if (!cat.materials[m]) errs.push('알 수 없는 재질: ' + m);
    }
    if (this.theta_min >= this.theta_max) errs.push('theta_min < theta_max 이어야 합니다');
    if (this.forearm <= this.upper_arm) errs.push('forearm(l)이 upper_arm(L)보다 짧으면 작업영역이 매우 좁아집니다 (보통 l ≈ 2~2.5 L)');
    if (this.effector_radius >= this.base_radius) errs.push('effector_radius(r)는 base_radius(R)보다 작아야 합니다');
    if (this.payload < 0) errs.push('payload는 음수가 될 수 없습니다');
    if (!errs.length && this.motorSpec.integrated_gear && this.gearbox !== 'none') {
      errs.push('일체형 서보(' + this.motor + ')에는 외부 감속기를 달지 않습니다 — gearbox는 무시됩니다');
    }
    if (!errs.length && !this.motorSpec.integrated_gear) {
      const ratios = this.gearboxSpec.ratios;
      if (!ratios.includes(Number(this.gear_ratio))) errs.push(this.gearbox + ' 의 감속비는 [' + ratios.join(', ') + '] 중에서 고르세요');
    }
    return errs;
  }
}

export function summary(d) {
  return {
    upper_arm_mass: d.upperArmMass, elbow_mass: d.elbowMass, forearm_pair_mass: d.forearmPairMass,
    moving_plate_mass: d.movingPlateMass, arm_inertia: d.armInertia, drive_inertia: d.driveInertia,
    gravity_moment: d.gravityMoment, ratio: d.ratio, efficiency: d.efficiency,
    static_torque_horizontal: d.gravityMoment * G,
  };
}
