// Runs every simulator demo program for every preset (tests/test_sim_programs.py).
import { readFileSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const imp = (p) => import(pathToFileURL(path.join(root, p)).href);
const { Design, setCatalog } = await imp('assets/js/delta/design.js');
const { defaultScene } = await imp('assets/js/delta/scene.js');
const { buildProgram, PROGRAMS } = await imp('sim/programs.js');
const { analyze, evaluate } = await imp('assets/js/delta/dynamics.js');

const cat = JSON.parse(readFileSync(path.join(root, 'python/deltarobot/data/catalog.json'), 'utf8'));
setCatalog(cat);
const out = {};
for (const preset of Object.keys(cat.presets)) {
  const base = Design.preset(preset);
  for (const [name, meta] of Object.entries(PROGRAMS)) {
    let d = base;
    if (meta.scene === 'drawing') d = base.copy({ tool: 'pen' });
    else if (name !== 'adept' && d.toolSpec.kind === 'pen') d = base.copy({ tool: 'suction_20' });
    const scene = defaultScene(d, meta.scene || 'pick_place');
    const key = preset + '/' + name;
    try {
      const tl = buildProgram(name, d, scene, {});
      const ev = evaluate(d, analyze(d, tl.frames));
      out[key] = { ok: true, duration: tl.duration, frames: tl.frames.length, info: tl.info,
        motor_ok: ev.ok, peak: ev.peak_ratio, rms: ev.rms_ratio, speed: ev.speed_ratio };
    } catch (e) {
      out[key] = { ok: false, error: String(e.message || e) };
    }
  }
}
process.stdout.write(JSON.stringify(out, null, 1));
