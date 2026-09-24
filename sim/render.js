// render.js — three.js view of the delta robot and its work cell (z-up, metres).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { PHI, elbow, ballJoint, fk } from '../assets/js/delta/kinematics.js';
import { generate as generateUrdf, jointState } from '../assets/js/delta/urdf.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const PART_COLORS = { red: 0xe5484d, blue: 0x3e7bfa, green: 0x30a46c };
const BIN_COLORS = { red: 0xe5484d, blue: 0x3e7bfa, green: 0x30a46c };
const Y = new THREE.Vector3(0, 1, 0);

function cssVar(name, fb) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb;
}

/** Cylinder mesh whose ends can be placed at two points (unit height along Y). */
function strut(radius, material) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 14), material);
  m.castShadow = true;
  return m;
}
const _a = new THREE.Vector3(), _b = new THREE.Vector3(), _d = new THREE.Vector3();
function placeStrut(mesh, p0, p1) {
  _a.set(p0[0], p0[1], p0[2]); _b.set(p1[0], p1[1], p1[2]);
  _d.subVectors(_b, _a);
  const len = _d.length();
  mesh.position.addVectors(_a, _b).multiplyScalar(0.5);
  mesh.scale.set(1, Math.max(len, 1e-6), 1);
  mesh.quaternion.setFromUnitVectors(Y, _d.normalize());
}


/** Text label that always faces the camera. */
export function textSprite(text, color = '#ffffff', height = 0.02) {
  const cv = document.createElement('canvas');
  const g = cv.getContext('2d');
  const font = 'bold 44px "Segoe UI", "Malgun Gothic", sans-serif';
  g.font = font;
  const w = Math.ceil(g.measureText(text).width) + 20;
  cv.width = w; cv.height = 64;
  g.font = font;
  g.fillStyle = 'rgba(0,0,0,0.55)';
  g.fillRect(0, 6, w, 52);
  g.fillStyle = color;
  g.textBaseline = 'middle';
  g.fillText(text, 10, 33);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sp.scale.set(height * w / 64, height, 1);
  sp.renderOrder = 10;
  return sp;
}

/** Arrow drawn on top of the robot (no depth test). */
export function overlayArrow(dir, origin, len, color) {
  const a = new THREE.ArrowHelper(new THREE.Vector3(...dir).normalize(), new THREE.Vector3(...origin), len, color, len * 0.22, len * 0.12);
  a.traverse((o) => { if (o.material) { o.material.depthTest = false; o.material.transparent = true; } o.renderOrder = 9; });
  return a;
}

/** Curved arrow showing the positive rotation sense: center + radius (cos a u + sin a w), a in [0, sweep]. */
export function rotationArrow(center, u, w, radius, sweep, color) {
  const pts = [];
  for (let k = 0; k <= 24; k++) {
    const a = sweep * k / 24;
    pts.push(new THREE.Vector3(
      center[0] + radius * (Math.cos(a) * u[0] + Math.sin(a) * w[0]),
      center[1] + radius * (Math.cos(a) * u[1] + Math.sin(a) * w[1]),
      center[2] + radius * (Math.cos(a) * u[2] + Math.sin(a) * w[2])));
  }
  const g = new THREE.Group();
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true }));
  line.renderOrder = 9;
  g.add(line);
  const end = pts[pts.length - 1], prev = pts[pts.length - 3];
  const head = new THREE.Mesh(new THREE.ConeGeometry(radius * 0.12, radius * 0.3, 12),
    new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true }));
  head.position.copy(end);
  head.quaternion.setFromUnitVectors(Y, end.clone().sub(prev).normalize());
  head.renderOrder = 9;
  g.add(head);
  return g;
}

export class DeltaView {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.005, 50);
    this.camera.position.set(0.8, -1.0, 0.3);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.4));
    this.sun = new THREE.DirectionalLight(0xffffff, 2.2);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(1024, 1024);
    this.scene.add(this.sun, this.sun.target);

    this.mat = {
      frame: new THREE.MeshStandardMaterial({ color: 0x8a94a6, metalness: 0.4, roughness: 0.5 }),
      motor: new THREE.MeshStandardMaterial({ color: 0x23272e, metalness: 0.5, roughness: 0.45 }),
      arm: new THREE.MeshStandardMaterial({ color: 0xe8912d, metalness: 0.2, roughness: 0.5 }),
      armBad: new THREE.MeshStandardMaterial({ color: 0xe5484d, metalness: 0.2, roughness: 0.5 }),
      rod: new THREE.MeshStandardMaterial({ color: 0x1d1f24, metalness: 0.3, roughness: 0.35 }),
      ball: new THREE.MeshStandardMaterial({ color: 0xc9ced6, metalness: 0.8, roughness: 0.25 }),
      plate: new THREE.MeshStandardMaterial({ color: 0x3e7bfa, metalness: 0.3, roughness: 0.4 }),
      tool: new THREE.MeshStandardMaterial({ color: 0x30a46c, metalness: 0.2, roughness: 0.5 }),
      toolOn: new THREE.MeshStandardMaterial({ color: 0xffb224, emissive: 0x6b3d00, metalness: 0.2, roughness: 0.4 }),
      table: new THREE.MeshStandardMaterial({ color: 0x9aa3ad, roughness: 0.9 }),
      bin: new THREE.MeshStandardMaterial({ color: 0x777777, roughness: 0.7 }),
      belt: new THREE.MeshStandardMaterial({ color: 0x2a2d33, roughness: 0.8 }),
      paper: new THREE.MeshStandardMaterial({ color: 0xf6f4ee, roughness: 1 }),
    };
    this.robot = null;
    this.cell = null;
    this.partMeshes = new Map();
    this.trail = null;
    this.trailCount = 0;
    this.extras = new THREE.Group();
    this.scene.add(this.extras);
    this.applyTheme();
    document.addEventListener('themechange', () => this.applyTheme());
    this.resize();
    window.addEventListener('resize', () => this.resize());
    if ('ResizeObserver' in window) new ResizeObserver(() => this.resize()).observe(container);
  }

  applyTheme() {
    this.scene.background = new THREE.Color(cssVar('--bg', '#0c1015'));
  }

  resize() {
    const w = this.container.clientWidth || 800, h = this.container.clientHeight || 600;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  // ------------------------------------------------------------------ robot
  build(d, sceneData) {
    this.d = d;
    if (this.robot) {
      for (const obj of [this.robot.group, this.robot.meshGroup]) {
        if (obj) { this.scene.remove(obj); this.dispose(obj); }
      }
    }
    const g = new THREE.Group();
    const R = d.base_radius, L = d.upper_arm, l = d.forearm, w = d.forearm_spacing, r = d.effector_radius;
    const size = L + l;
    const motorR = Math.min(0.05, Math.max(0.014, 0.18 * R));
    const motorLen = Math.min(0.12, Math.max(0.03, 0.35 * R));
    const armW = Math.max(0.008, 0.08 * L);
    const rodR = Math.max(0.0025, 0.01 * l);
    const ballR = rodR * 1.9;

    const plate = new THREE.Mesh(new THREE.CylinderGeometry(R + motorR + 0.015, R + motorR + 0.015, 0.012, 6), this.mat.frame);
    plate.rotation.x = Math.PI / 2;
    plate.position.z = 0.02 + motorR * 0.4;
    plate.castShadow = true;
    g.add(plate);
    // ceiling mount posts so the robot does not look like it floats
    const postH = 0.25 * size;
    for (let i = 0; i < 3; i++) {
      const a = PHI[i] + Math.PI / 3;
      const post = strut(Math.max(0.006, 0.03 * R), this.mat.frame);
      const pr = R + motorR + 0.005;
      placeStrut(post, [pr * Math.cos(a), pr * Math.sin(a), plate.position.z], [pr * Math.cos(a), pr * Math.sin(a), plate.position.z + postH]);
      g.add(post);
    }

    const arms = [];
    for (let i = 0; i < 3; i++) {
      const phi = PHI[i];
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(motorR, motorR, motorLen, 24), this.mat.motor);
      motor.position.set(R * Math.cos(phi), R * Math.sin(phi), 0);
      motor.rotation.z = phi;           // cylinder axis (Y) -> tangent after rotating by phi
      motor.castShadow = true;
      g.add(motor);

      const pivot = new THREE.Group();
      pivot.position.copy(motor.position);
      pivot.rotation.z = phi;
      const swing = new THREE.Group();   // rotation about local y = theta
      pivot.add(swing);
      const armMesh = new THREE.Mesh(new THREE.BoxGeometry(L, armW, armW * 0.8), this.mat.arm);
      armMesh.position.x = L / 2;
      armMesh.castShadow = true;
      swing.add(armMesh);
      const bar = new THREE.Mesh(new THREE.CylinderGeometry(armW * 0.35, armW * 0.35, w + 2 * ballR, 12), this.mat.arm);
      bar.position.x = L;
      swing.add(bar);
      g.add(pivot);

      const rods = [strut(rodR, this.mat.rod), strut(rodR, this.mat.rod)];
      const balls = [0, 1, 2, 3].map(() => new THREE.Mesh(new THREE.SphereGeometry(ballR, 14, 10), this.mat.ball));
      rods.forEach((m) => g.add(m));
      balls.forEach((m) => g.add(m));
      arms.push({ swing, armMesh, bar, rods, balls });
    }

    const eff = new THREE.Group();
    const effPlate = new THREE.Mesh(new THREE.CylinderGeometry(r + ballR * 1.5, r + ballR * 1.5, 0.01, 6), this.mat.plate);
    effPlate.rotation.x = Math.PI / 2;
    effPlate.castShadow = true;
    eff.add(effPlate);
    const tool = this.buildTool(d);
    eff.add(tool.group);
    g.add(eff);

    this.scene.add(g);
    this.robot = { group: g, arms, eff, tool, size, kind: 'mesh' };
    if (this.useUrdf) {
      try {
        this.buildFromUrdf(d, g);
      } catch (e) {
        console.warn('[sim] URDF model failed, using the built-in model:', e);
        this.urdfError = String(e.message || e);
      }
    }
    this.buildCell(sceneData);
    this.clearTrail();
    this.frames = null;
    if (this.framesOn) this.showJointFrames(true);
  }

  buildTool(d) {
    const spec = d.toolSpec, tl = d.toolLength;
    const group = new THREE.Group();
    const parts = { group, kind: spec.kind, on: [] };
    const mk = (geo, mat, z) => { const m = new THREE.Mesh(geo, mat); m.position.z = z; m.castShadow = true; group.add(m); return m; };
    if (spec.kind === 'suction') {
      const cup = Math.max(0.01, Math.sqrt(Number(spec.force) / 60e3 / Math.PI));
      mk(new THREE.CylinderGeometry(0.006, 0.006, tl - 0.008, 12), this.mat.tool, -(tl - 0.008) / 2).rotation.x = Math.PI / 2;
      const c = mk(new THREE.CylinderGeometry(cup, cup * 0.7, 0.008, 20), this.mat.tool, -tl + 0.004);
      c.rotation.x = -Math.PI / 2;
      parts.on.push(c);
    } else if (spec.kind === 'gripper') {
      mk(new THREE.BoxGeometry(0.06, 0.03, 0.03), this.mat.tool, -0.015);
      const fl = tl - 0.03;
      parts.fingers = [-1, 1].map((s) => {
        const f = mk(new THREE.BoxGeometry(0.006, 0.02, fl), this.mat.tool, -0.03 - fl / 2);
        f.position.x = s * 0.02;
        f.userData.side = s;
        return f;
      });
      parts.on.push(...parts.fingers);
    } else if (spec.kind === 'magnet') {
      const m = mk(new THREE.CylinderGeometry(0.0125, 0.0125, tl, 24), this.mat.tool, -tl / 2);
      m.rotation.x = Math.PI / 2;
      parts.on.push(m);
    } else {
      const m = mk(new THREE.CylinderGeometry(0.004, 0.004, tl - 0.01, 10), this.mat.tool, -(tl - 0.01) / 2 - 0.005);
      m.rotation.x = Math.PI / 2;
      const tip = mk(new THREE.ConeGeometry(0.004, 0.01, 10), this.mat.rod, -tl + 0.005);
      tip.rotation.x = -Math.PI / 2;
      parts.on.push(m);
    }
    group.position.z = -0.005;
    return parts;
  }

  buildCell(s) {
    if (this.cell) { this.scene.remove(this.cell); this.dispose(this.cell); }
    this.partMeshes.clear();
    const g = new THREE.Group();
    this.cell = g;
    this.sceneData = s;
    const z0 = s ? s.surface_z : -(this.d.upper_arm + this.d.forearm);
    const rc = s && s.work_radius ? s.work_radius : 0.2;
    const span = Math.max(3.2 * rc, 0.25);
    const table = new THREE.Mesh(new THREE.BoxGeometry(span, span, 0.02), this.mat.table);
    table.position.set(0, 0, z0 - 0.01 - 0.0005);
    table.receiveShadow = true;
    g.add(table);
    const grid = new THREE.GridHelper(span, Math.round(span / 0.05), 0x5c6773, 0x5c6773);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = z0 + 0.0005;
    grid.material.transparent = true;
    grid.material.opacity = 0.25;
    g.add(grid);
    this.gridHelper = grid;

    if (s) {
      for (const b of s.bins || []) {
        const bg = new THREE.Group();
        const t = 0.003;
        const mat = new THREE.MeshStandardMaterial({ color: BIN_COLORS[b.color] || 0x777777, roughness: 0.7, transparent: true, opacity: 0.85 });
        const floor = new THREE.Mesh(new THREE.BoxGeometry(b.w, b.d, t), mat);
        floor.position.z = t / 2;
        bg.add(floor);
        for (const [sx, sy, w, dd] of [[0, 1, b.w, t], [0, -1, b.w, t], [1, 0, t, b.d], [-1, 0, t, b.d]]) {
          const wall = new THREE.Mesh(new THREE.BoxGeometry(w, dd, b.h), mat);
          wall.position.set(sx * (b.w / 2), sy * (b.d / 2), b.h / 2);
          bg.add(wall);
        }
        bg.position.set(b.x, b.y, z0);
        g.add(bg);
      }
      const c = s.conveyor;
      if (c) {
        const len = c.x_end - c.x_start;
        const tex = this.stripeTexture();
        tex.repeat.set(len / (c.width * 0.6), 1);
        const beltMat = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.85 });
        const belt = new THREE.Mesh(new THREE.BoxGeometry(len, c.width, 0.01), beltMat);
        belt.position.set((c.x_start + c.x_end) / 2, c.y, z0 - 0.005);
        belt.receiveShadow = true;
        g.add(belt);
        for (const sgn of [-1, 1]) {
          const rail = new THREE.Mesh(new THREE.BoxGeometry(len, 0.006, 0.02), this.mat.frame);
          rail.position.set((c.x_start + c.x_end) / 2, c.y + sgn * (c.width / 2 + 0.003), z0 + 0.0);
          g.add(rail);
        }
        this.beltTex = tex;
        this.beltLen = len;
      } else {
        this.beltTex = null;
      }
      if (s.paper) {
        const paper = new THREE.Mesh(new THREE.PlaneGeometry(s.paper.w, s.paper.d), this.mat.paper);
        paper.position.set(s.paper.x, s.paper.y, z0 + 0.0008);
        paper.receiveShadow = true;
        g.add(paper);
      }
    }
    this.scene.add(g);
    // light & camera target follow the robot size
    const size = this.robot ? this.robot.size : 0.5;
    this.sun.position.set(size * 0.8, -size * 0.6, size * 1.6);
    this.sun.target.position.set(0, 0, z0);
    const sc = this.sun.shadow.camera;
    sc.left = sc.bottom = -span; sc.right = sc.top = span; sc.near = 0.01; sc.far = size * 5;
    sc.updateProjectionMatrix();
  }

  stripeTexture() {
    const cv = document.createElement('canvas');
    cv.width = 64; cv.height = 16;
    const x = cv.getContext('2d');
    x.fillStyle = '#2a2d33'; x.fillRect(0, 0, 64, 16);
    x.fillStyle = '#3a3e46'; x.fillRect(0, 0, 8, 16);
    const tex = new THREE.CanvasTexture(cv);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }

  partMesh(p) {
    let m = this.partMeshes.get(p.id);
    if (!m) {
      const steel = p.material === 'steel';
      const mat = new THREE.MeshStandardMaterial({ color: PART_COLORS[p.color] || 0xaaaaaa,
        metalness: steel ? 0.7 : 0.05, roughness: steel ? 0.3 : 0.6 });
      m = new THREE.Mesh(new THREE.BoxGeometry(p.size, p.size, p.h), mat);
      m.castShadow = true;
      if (steel) {
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0xdddddd }));
        m.add(edges);
      }
      this.cell.add(m);
      this.partMeshes.set(p.id, m);
    }
    return m;
  }

  /**
   * Draw the robot from the URDF generated for this design (assets/js/delta/urdf.js) —
   * the same model the URDF viewer and ROS 2 (robot_state_publisher / RViz) use.
   * Every frame only the joint values change: motors + passive joints from jointState().
   */
  buildFromUrdf(d, meshGroup) {
    this.urdfText = generateUrdf(d);
    const robot = new URDFLoader().parse(this.urdfText);
    robot.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    const byLink = (name) => {
      const out = [];
      const link = robot.links[name];
      if (link) link.traverse((o) => { if (o.isMesh && o.parent && (o.parent === link || o.parent.parent === link)) out.push(o); });
      return out;
    };
    // keep each mesh's own URDF material so we can restore it after tinting
    robot.traverse((o) => { if (o.isMesh) o.userData.baseMat = o.material; });
    const arms = [0, 1, 2].map((i) => ({ swing: robot.links[`upper_arm${i + 1}`], meshes: byLink(`upper_arm${i + 1}`) }));
    meshGroup.visible = false;                       // hide the hand-built model
    this.scene.add(robot);
    this.robot = {
      group: robot, meshGroup, urdf: robot, arms, eff: robot.links.effector,
      toolMeshes: byLink('tool_link'), size: this.robot.size, kind: 'urdf',
      tool: { kind: d.toolSpec.kind, on: [] },
    };
    this.urdfError = null;
  }

  /** Update robot pose, tool state, and parts (sceneState: SceneState or null). */
  setPose(q, tool, sceneState, t, bad = false) {
    const d = this.d;
    if (!this.robot || !d) return null;
    let p;
    try { p = fk(d, q); } catch (e) { return null; }
    if (this.robot.kind === 'urdf') this.poseUrdf(q, tool, bad);
    else this.poseMesh(q, tool, bad, p);
    return this.poseScene(p, sceneState, t);
  }

  poseUrdf(q, tool, bad) {
    const r = this.robot;
    const js = jointState(this.d, q);
    for (const name in js) r.urdf.setJointValue(name, js[name]);
    r.arms.forEach((arm, i) => arm.meshes.forEach((m) => { m.material = bad && bad[i] ? this.mat.armBad : m.userData.baseMat; }));
    const on = tool && r.tool.kind !== 'pen';
    r.toolMeshes.forEach((m) => { m.material = on ? this.mat.toolOn : m.userData.baseMat; });
  }

  poseMesh(q, tool, bad, p) {
    const d = this.d;
    const w = d.forearm_spacing;
    this.robot.arms.forEach((arm, i) => {
      arm.swing.rotation.y = q[i];
      arm.armMesh.material = bad && bad[i] ? this.mat.armBad : this.mat.arm;
      const e = elbow(d, i, q[i]);
      const b = ballJoint(d, i, p);
      const tx = -Math.sin(PHI[i]) * w / 2, ty = Math.cos(PHI[i]) * w / 2;
      [1, -1].forEach((s, k) => {
        const e2 = [e[0] + s * tx, e[1] + s * ty, e[2]];
        const b2 = [b[0] + s * tx, b[1] + s * ty, b[2]];
        placeStrut(arm.rods[k], e2, b2);
        arm.balls[2 * k].position.set(...e2);
        arm.balls[2 * k + 1].position.set(...b2);
      });
    });
    this.robot.eff.position.set(p[0], p[1], p[2]);
    const tl = this.robot.tool;
    tl.on.forEach((m) => { m.material = tool ? this.mat.toolOn : this.mat.tool; });
    if (tl.fingers) tl.fingers.forEach((f) => { f.position.x = f.userData.side * (tool ? 0.011 : 0.02); });
    if (tl.kind === 'pen') this.robot.tool.on.forEach((m) => { m.material = this.mat.tool; });
  }

  poseScene(p, sceneState, t) {
    const d = this.d;
    const tl = this.robot.tool;
    const tcp = [p[0], p[1], p[2] - d.toolLength];
    if (sceneState) {
      const seen = new Set();
      for (const v of sceneState.visible(t)) {
        const m = this.partMesh(v.part);
        m.visible = true;
        m.position.set(v.x, v.y, v.z + v.part.h / 2);
        seen.add(v.part.id);
      }
      const held = sceneState.held;
      if (held) {
        const m = this.partMesh(held);
        m.visible = true;
        const zc = tl.kind === 'gripper' ? tcp[2] + 0.004 : tcp[2] - held.h / 2;
        m.position.set(tcp[0], tcp[1], zc);
        seen.add(held.id);
      }
      for (const [id, m] of this.partMeshes) if (!seen.has(id)) m.visible = false;
      if (this.beltTex && sceneState.data.conveyor) {
        const c = sceneState.data.conveyor;
        this.beltTex.offset.x = -((c.speed * t) / (c.width * 0.6)) % 1;
      }
    }
    return { p, tcp };
  }

  // ------------------------------------------------------------------ overlays
  clearTrail() {
    if (this.trail) { this.extras.remove(this.trail); this.trail.geometry.dispose(); }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 30000), 3));
    geo.setDrawRange(0, 0);
    this.trail = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0x1b64f2 }));
    this.trail.frustumCulled = false;
    this.trailCount = 0;
    this.trailBreak = true;
    this.extras.add(this.trail);
  }
  addTrail(p) {
    if (!this.trail || this.trailCount >= 30000) return;
    const arr = this.trail.geometry.attributes.position.array;
    arr.set([p[0], p[1], p[2] + 0.0006], 3 * this.trailCount);
    this.trailCount++;
    this.trail.geometry.setDrawRange(0, this.trailCount);
    this.trail.geometry.attributes.position.needsUpdate = true;
  }

  setPathPreview(points) {
    if (this.pathLine) { this.extras.remove(this.pathLine); this.pathLine.geometry.dispose(); this.pathLine = null; }
    if (!points || points.length < 2) return;
    const geo = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(p[0], p[1], p[2])));
    this.pathLine = new THREE.Line(geo, new THREE.LineDashedMaterial({ color: 0x38bdf8, dashSize: 0.006, gapSize: 0.004, transparent: true, opacity: 0.8 }));
    this.pathLine.computeLineDistances();
    this.extras.add(this.pathLine);
  }

  showWorkspace(flat) {
    if (this.cloud) { this.extras.remove(this.cloud); this.cloud.geometry.dispose(); this.cloud = null; }
    if (!flat) return;
    const tl = this.d.toolLength;
    const n = flat.length / 3;
    const pos = new Float32Array(flat.length), col = new Float32Array(flat.length);
    let zmin = Infinity, zmax = -Infinity;
    for (let i = 0; i < n; i++) { zmin = Math.min(zmin, flat[3 * i + 2]); zmax = Math.max(zmax, flat[3 * i + 2]); }
    const c = new THREE.Color();
    for (let i = 0; i < n; i++) {
      pos[3 * i] = flat[3 * i]; pos[3 * i + 1] = flat[3 * i + 1]; pos[3 * i + 2] = flat[3 * i + 2] - tl;
      c.setHSL(0.6 - 0.6 * (flat[3 * i + 2] - zmin) / Math.max(1e-6, zmax - zmin), 0.8, 0.55);
      col.set([c.r, c.g, c.b], 3 * i);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    this.cloud = new THREE.Points(geo, new THREE.PointsMaterial({ size: Math.max(0.003, this.robot.size / 120), vertexColors: true, transparent: true, opacity: 0.55 }));
    this.extras.add(this.cloud);
  }

  /** Joint frames: base XYZ, motor axes with the +θ sense, elbow passive axes, effector frame, TCP. */
  showJointFrames(on) {
    if (this.frames) {
      for (const o of this.frames) if (o.parent) o.parent.remove(o);
      this.frames = null;
    }
    if (!on || !this.robot) return;
    const d = this.d, L = d.upper_arm, s = this.robot.size;
    const list = [];
    const add = (parent, obj) => { parent.add(obj); list.push(obj); };
    const lab = (parent, text, pos, color, h = s * 0.028) => { const t = textSprite(text, color, h); t.position.set(...pos); add(parent, t); };
    const g = this.robot.group;
    // base frame (REP-103: x red, y green, z blue)
    const ax = s * 0.22;
    add(g, overlayArrow([1, 0, 0], [0, 0, 0], ax, 0xef4444)); lab(g, 'X', [ax * 1.12, 0, 0], '#ef4444');
    add(g, overlayArrow([0, 1, 0], [0, 0, 0], ax, 0x22c55e)); lab(g, 'Y', [0, ax * 1.12, 0], '#22c55e');
    add(g, overlayArrow([0, 0, 1], [0, 0, 0], ax, 0x3b82f6)); lab(g, 'Z', [0, 0, ax * 1.12], '#3b82f6');
    lab(g, 'base', [0, 0, -s * 0.04], '#e5e7eb', s * 0.022);
    const colors = [0x3b82f6, 0xf59e0b, 0x10b981], css = ['#60a5fa', '#fbbf24', '#34d399'];
    this.robot.arms.forEach((arm, i) => {
      const phi = PHI[i];
      const u = [Math.cos(phi), Math.sin(phi), 0], t = [-Math.sin(phi), Math.cos(phi), 0];
      const m = [d.base_radius * u[0], d.base_radius * u[1], 0];
      // motor joint axis = arm-frame +y (tangent); +θ turns the arm DOWN (right-hand rule about +y)
      const alen = Math.max(0.05, 0.5 * L);
      add(g, overlayArrow(t, m, alen, colors[i]));
      lab(g, `모터${i + 1} 축`, [m[0] + t[0] * alen * 1.2, m[1] + t[1] * alen * 1.2, m[2]], css[i], s * 0.022);
      const rad = 0.45 * L;
      add(g, rotationArrow(m, u, [0, 0, -1], rad, 1.1, colors[i]));
      const a = 0.9;
      lab(g, `+θ${i + 1}`, [m[0] + rad * 1.3 * Math.cos(a) * u[0], m[1] + rad * 1.3 * Math.cos(a) * u[1], -rad * 1.3 * Math.sin(a)], css[i]);
      // θ = 0 reference: upper arm horizontal
      const zl = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...m),
        new THREE.Vector3(m[0] + L * u[0], m[1] + L * u[1], 0)]),
      new THREE.LineDashedMaterial({ color: colors[i], dashSize: L * 0.06, gapSize: L * 0.04, depthTest: false, transparent: true }));
      zl.computeLineDistances(); zl.renderOrder = 9;
      add(g, zl);
      lab(g, 'θ=0', [m[0] + L * 1.08 * u[0], m[1] + L * 1.08 * u[1], 0.012 * s], css[i], s * 0.018);
      // passive elbow axes ride on the swinging upper arm (arm frame: x along the arm, y = motor axis)
      const el = Math.max(0.03, 0.3 * L);
      add(arm.swing, overlayArrow([0, 1, 0], [L, 0, 0], el, 0xf59e0b));
      add(arm.swing, overlayArrow([0, 0, 1], [L, 0, 0], el * 0.8, 0xc084fc));
      if (i === 0) {
        lab(arm.swing, 'pitch 축(y)', [L, el * 1.25, 0], '#fbbf24', s * 0.018);
        lab(arm.swing, 'yaw 축(z)', [L, 0, el * 1.0], '#c084fc', s * 0.018);
      }
    });
    // effector frame: translation only, always parallel to the base frame
    const e = this.robot.eff, el = s * 0.1;
    add(e, overlayArrow([1, 0, 0], [0, 0, 0], el, 0xef4444));
    add(e, overlayArrow([0, 1, 0], [0, 0, 0], el, 0x22c55e));
    add(e, overlayArrow([0, 0, 1], [0, 0, 0], el, 0x3b82f6));
    lab(e, '이펙터 (base와 평행)', [el * 0.3, el * 0.3, el * 1.25], '#e5e7eb', s * 0.02);
    const tcp = new THREE.Mesh(new THREE.SphereGeometry(s * 0.008, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff4d8d, depthTest: false }));
    tcp.position.z = -d.toolLength - 0.005; tcp.renderOrder = 10;
    add(e, tcp);
    lab(e, 'TCP', [s * 0.035, 0, -d.toolLength - 0.005], '#ff4d8d', s * 0.02);
    this.frames = list;
  }

  showCylinder(cyl) {
    if (this.cyl) { this.extras.remove(this.cyl); this.cyl.geometry.dispose(); this.cyl = null; }
    if (!cyl || !(cyl.diameter > 0)) return;
    const tl = this.d.toolLength;
    const geo = new THREE.CylinderGeometry(cyl.diameter / 2, cyl.diameter / 2, cyl.height, 48, 1, true);
    this.cyl = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo, 30), new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.7 }));
    this.cyl.add(edges);
    this.cyl.rotation.x = Math.PI / 2;
    this.cyl.position.z = (cyl.z_top + cyl.z_bottom) / 2 - tl;
    this.extras.add(this.cyl);
  }

  fit() {
    if (!this.robot) return;
    const s = this.robot.size;
    const zc = -0.42 * s;
    this.controls.target.set(0, 0, zc);
    this.camera.position.set(1.55 * s, -2.0 * s, zc + 0.75 * s);
    this.camera.near = s / 200;
    this.camera.far = s * 40;
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose(obj) {
    obj.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material && !Object.values(this.mat).includes(o.material)) {
        if (o.material.map) o.material.map.dispose();
        o.material.dispose();
      }
    });
  }
}
