// Node script used by tests/test_js_parity.py: evaluates the JS core on the cases given on stdin.
import { readFileSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '..', '..');
const imp = (p) => import(pathToFileURL(path.join(root, p)).href);

const designMod = await imp('assets/js/delta/design.js');
const kin = await imp('assets/js/delta/kinematics.js');
const dyn = await imp('assets/js/delta/dynamics.js');
const tr = await imp('assets/js/delta/trajectory.js');
const urdf = await imp('assets/js/delta/urdf.js');
const scene = await imp('assets/js/delta/scene.js');

designMod.setCatalog(JSON.parse(readFileSync(path.join(root, 'python/deltarobot/data/catalog.json'), 'utf8')));
const cases = JSON.parse(readFileSync(0, 'utf8'));
const out = {};
for (const [name, c] of Object.entries(cases)) {
  const d = designMod.Design.preset(c.preset);
  const r = {};
  r.summary = designMod.summary(d);
  r.validate = d.validate();
  r.fk = c.thetas.map((t) => kin.tryFk(d, t));
  r.ik = c.points.map((p) => kin.tryIk(d, p));
  r.limits = c.points.map((p) => kin.limitReport(d, p));
  r.jac = kin.jacobian(d, c.thetas[0]);
  r.cond = kin.conditionNumber(d, c.thetas[0]);
  r.passive = kin.passiveAngles(d, c.thetas[0]);
  r.torque = dyn.jointTorques(d, c.thetas[0], [1, -2, 0.5], null, [0.5, -1, 2]);
  r.avail = [0, 1, 5, 50].map((w) => dyn.availableTorque(d, w));
  r.inertia_ratio = dyn.inertiaRatio(d);
  r.profiles = tr.PROFILES.map((k) => { const p = new tr.Profile(k, 0.2, 0.8, 6); return [p.T, p.at(p.T * 0.3), p.at(p.T * 0.8)]; });
  const ap = tr.archPath(c.points[0], c.points[1], 0.03);
  r.arch = [ap.length, ap.point(ap.length * 0.37), ap.point(ap.length * 0.81)];
  r.cyl = kin.workCylinder(d, 0.25 * (d.upper_arm + d.forearm));
  r.cyl.profile = r.cyl.profile.slice(0, 12);
  r.scene = scene.defaultScene(d, 'pick_place');
  r.scene_conv = scene.defaultScene(d, 'conveyor');
  r.urdf = urdf.generate(d);
  r.jstate = urdf.jointState(d, c.thetas[0]);
  r.jstate_tool = urdf.jointState(d, c.thetas[0], 1);
  const frames = [];
  for (let k = 0; k <= 40; k++) { const t = k * 0.01; frames.push({ t, q: [0.3 + 0.2 * Math.sin(3 * t), 0.35, 0.4 - 0.1 * t], tool: 0 }); }
  r.evaluate = dyn.evaluate(d, dyn.analyze(d, frames));
  out[name] = r;
}
process.stdout.write(JSON.stringify(out));
