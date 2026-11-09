// PhysicsClay · browser physics lab
// MuJoCo (official Google DeepMind WASM bindings, @mujoco/mujoco@3.17.0) stepping in the browser,
// three.js@0.160.0 rendering, React 18 + htm control panel. No build step, no Python server.
import React, { useEffect, useState } from 'react';
import { createRoot } from 'react-dom/client';
import htm from 'htm';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from 'mujoco';

const html = htm.bind(React.createElement);
const PKG = '@mujoco/mujoco@3.17.0 (single-threaded WASM, jsDelivr)';

const MJCF = `
<mujoco model="physicsclay_lab">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <default>
    <geom friction="0.9 0.01 0.001" solref="0.004 1"/>
    <joint damping="0.4"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.93 0.93 0.95 1"/>
    <body name="table" pos="0 0 0.375">
      <geom name="tabletop" type="box" size="0.5 0.35 0.02" pos="0 0 0.355" rgba="0.80 0.69 0.55 1"/>
      <geom type="box" size="0.02 0.02 0.335" pos=" 0.45  0.3 0" rgba="0.62 0.52 0.40 1"/>
      <geom type="box" size="0.02 0.02 0.335" pos="-0.45  0.3 0" rgba="0.62 0.52 0.40 1"/>
      <geom type="box" size="0.02 0.02 0.335" pos=" 0.45 -0.3 0" rgba="0.62 0.52 0.40 1"/>
      <geom type="box" size="0.02 0.02 0.335" pos="-0.45 -0.3 0" rgba="0.62 0.52 0.40 1"/>
    </body>
    <body name="mug" pos="0.15 -0.12 0.801">
      <freejoint name="mug_free"/>
      <geom name="mug_body" type="cylinder" size="0.04 0.05" mass="0.3" rgba="0.0 0.44 0.89 1"/>
      <geom name="mug_handle" type="box" size="0.012 0.008 0.03" pos="0.05 0 0" mass="0.02" rgba="0.0 0.38 0.78 1"/>
    </body>
    <body name="vase" pos="-0.12 0.08 0.831">
      <freejoint name="vase_free"/>
      <geom name="vase_body" type="cylinder" size="0.055 0.08" mass="0.8" rgba="0.92 0.55 0.45 1"/>
      <geom name="vase_neck" type="cylinder" size="0.03 0.035" pos="0 0 0.115" mass="0.1" rgba="0.88 0.48 0.38 1"/>
    </body>
    <body name="block_a" pos="0.3 0.15 0.781">
      <freejoint name="block_a_free"/>
      <geom name="block_a" type="box" size="0.03 0.03 0.03" mass="0.1" rgba="0.20 0.75 0.45 1"/>
    </body>
    <body name="block_b" pos="0.31 0.15 0.842">
      <freejoint name="block_b_free"/>
      <geom name="block_b" type="box" size="0.03 0.03 0.03" mass="0.1" rgba="0.98 0.78 0.20 1"/>
    </body>
    <body name="arm_base" pos="-0.38 -0.22 0.77">
      <geom type="cylinder" size="0.045 0.02" rgba="0.25 0.25 0.28 1"/>
      <body name="link1" pos="0 0 0.02">
        <joint name="j1" type="hinge" axis="0 0 1" range="-2.5 2.5"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.15" size="0.02" rgba="0.55 0.55 0.6 1"/>
        <body name="link2" pos="0 0 0.15">
          <joint name="j2" type="hinge" axis="0 1 0" range="-2 2"/>
          <geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.018" rgba="0.45 0.45 0.5 1"/>
          <body name="link3" pos="0.15 0 0">
            <joint name="j3" type="hinge" axis="0 1 0" range="-2 2"/>
            <geom type="capsule" fromto="0 0 0 0.12 0 0" size="0.016" rgba="0.35 0.35 0.4 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position joint="j1" kp="8"/>
    <position joint="j2" kp="8"/>
    <position joint="j3" kp="4"/>
  </actuator>
</mujoco>`;

// ---------------- simulation + rendering (outside React) ----------------
const sim = {
  mujoco: null, model: null, data: null,
  playing: true, armWave: true, steps: 0, gravity: 9.81,
  bodies: [], // {id, group}
  error: null,
};
window.__lab = sim; // for headless verification

function bodyId(name) {
  return sim.mujoco.mj_name2id(sim.model, sim.mujoco.mjtObj.mjOBJ_BODY.value, name);
}

function setFreeBody(name, pos, quat = [1, 0, 0, 0]) {
  const { model, data, mujoco } = sim;
  const b = bodyId(name);
  const j = model.body_jntadr[b];
  const qa = model.jnt_qposadr[j], da = model.jnt_dofadr[j];
  const qpos = data.qpos, qvel = data.qvel;
  qpos[qa] = pos[0]; qpos[qa + 1] = pos[1]; qpos[qa + 2] = pos[2];
  for (let k = 0; k < 4; k++) qpos[qa + 3 + k] = quat[k];
  for (let k = 0; k < 6; k++) qvel[da + k] = 0;
  mujoco.mj_forward(model, data);
}

function randQuat(maxAngle) {
  const ax = new THREE.Vector3(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5).normalize();
  const a = (Math.random() * 2 - 1) * maxAngle;
  const s = Math.sin(a / 2);
  return [Math.cos(a / 2), ax.x * s, ax.y * s, ax.z * s];
}

const actions = {
  reset() {
    const { mujoco, model, data } = sim;
    mujoco.mj_resetData(model, data);
    actions.setGravity(sim.gravity);
    mujoco.mj_forward(model, data);
  },
  dropBlock() {
    const x = (Math.random() - 0.5) * 0.5, y = (Math.random() - 0.5) * 0.35;
    setFreeBody(Math.random() < 0.5 ? 'block_a' : 'block_b', [x, y, 1.25], randQuat(Math.PI));
  },
  spawnMugAboveVase() {
    const v = sim.data.xpos; const b = bodyId('vase');
    setFreeBody('mug', [v[3 * b] + 0.01, v[3 * b + 1], v[3 * b + 2] + 0.4], randQuat(0.5));
  },
  setGravity(g) {
    sim.gravity = g;
    const grav = sim.model.opt.gravity;
    grav[0] = 0; grav[1] = 0; grav[2] = -g;
  },
};
window.__labActions = actions;

function buildScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#f2f2f6');
  scene.fog = new THREE.Fog('#f2f2f6', 4, 9);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 50);
  camera.up.set(0, 0, 1); // MuJoCo is Z-up
  camera.position.set(1.25, -1.35, 1.45);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.8);
  controls.update();

  scene.add(new THREE.HemisphereLight('#ffffff', '#d8d8e0', 1.1));
  const sun = new THREE.DirectionalLight('#ffffff', 1.6);
  sun.position.set(1.5, -1.0, 3.0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -1.2, right: 1.2, top: 1.2, bottom: -1.2, near: 0.5, far: 8 });
  scene.add(sun);

  const resize = () => {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / Math.max(h, 1);
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(container);
  resize();
  return { renderer, scene, camera, controls };
}

// Build one THREE.Group per MuJoCo body, with child meshes at each geom's local pos/quat.
function buildBodies(scene) {
  const { model } = sim;
  const groups = new Map();
  for (let b = 0; b < model.nbody; b++) {
    const g = new THREE.Group();
    scene.add(g);
    groups.set(b, g);
  }
  const T = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6 };
  for (let i = 0; i < model.ngeom; i++) {
    const type = model.geom_type[i];
    const s = [model.geom_size[3 * i], model.geom_size[3 * i + 1], model.geom_size[3 * i + 2]];
    const rgba = [0, 1, 2, 3].map((k) => model.geom_rgba[4 * i + k]);
    let geo;
    if (type === T.PLANE) geo = new THREE.PlaneGeometry(2 * s[0], 2 * s[1]);
    else if (type === T.BOX) geo = new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    else if (type === T.SPHERE) geo = new THREE.SphereGeometry(s[0], 24, 16);
    else if (type === T.CYLINDER) geo = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 40).rotateX(Math.PI / 2);
    else if (type === T.CAPSULE) geo = new THREE.CapsuleGeometry(s[0], 2 * s[1], 8, 20).rotateX(Math.PI / 2);
    else if (type === T.ELLIPSOID) geo = new THREE.SphereGeometry(1, 24, 16).scale(s[0], s[1], s[2]);
    else continue;
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]), roughness: type === T.PLANE ? 0.95 : 0.55, metalness: 0.0,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = type !== T.PLANE;
    mesh.receiveShadow = true;
    mesh.position.set(model.geom_pos[3 * i], model.geom_pos[3 * i + 1], model.geom_pos[3 * i + 2]);
    const q = model.geom_quat; // MuJoCo (w,x,y,z) -> three (x,y,z,w)
    mesh.quaternion.set(q[4 * i + 1], q[4 * i + 2], q[4 * i + 3], q[4 * i]);
    groups.get(model.geom_bodyid[i]).add(mesh);
  }
  sim.bodies = [...groups.entries()].map(([id, group]) => ({ id, group }));
}

function syncBodies() {
  const xpos = sim.data.xpos, xquat = sim.data.xquat;
  for (const { id, group } of sim.bodies) {
    group.position.set(xpos[3 * id], xpos[3 * id + 1], xpos[3 * id + 2]);
    group.quaternion.set(xquat[4 * id + 1], xquat[4 * id + 2], xquat[4 * id + 3], xquat[4 * id]);
  }
}

async function startSim(container, onStats) {
  const overlay = document.getElementById('overlay');
  const t0 = performance.now();
  const mujoco = await loadMujoco();
  sim.mujoco = mujoco;
  sim.model = mujoco.MjModel.from_xml_string(MJCF);
  sim.data = new mujoco.MjData(sim.model);
  mujoco.mj_forward(sim.model, sim.data);
  sim.loadMs = Math.round(performance.now() - t0);
  console.log(`[lab] MuJoCo WASM ready: ${PKG}, nbody=${sim.model.nbody} ngeom=${sim.model.ngeom} in ${sim.loadMs} ms`);

  const view = buildScene(container);
  buildBodies(view.scene);
  syncBodies();
  let nm = 0; view.scene.traverse((o) => { if (o.isMesh) nm++; });
  const sz = view.renderer.getSize(new THREE.Vector2());
  console.log(`[lab] meshes=${nm} canvas=${sz.x}x${sz.y} geom_type0=${sim.model.geom_type[0]} (${typeof sim.model.geom_type[0]})`);
  overlay.textContent = `MuJoCo WASM · ${sim.model.nbody} bodies · ${sim.model.ngeom} geoms`;

  // Fixed 60 Hz physics tick: each tick advances 1/60 s of sim time (about 8 x 2 ms mj_step).
  const TICK = 1 / 60;
  const substeps = Math.max(1, Math.round(TICK / sim.model.opt.timestep));
  let last = performance.now(), acc = 0, lastStats = 0, lastLog = 0;
  const frame = (now) => {
    acc += Math.min((now - last) / 1000, 0.25);
    last = now;
    while (acc >= TICK) {
      acc -= TICK;
      if (sim.playing) {
        if (sim.armWave) {
          const t = sim.data.time, ctrl = sim.data.ctrl;
          ctrl[0] = 0.9 * Math.sin(0.8 * t);
          ctrl[1] = -0.6 + 0.4 * Math.sin(1.3 * t);
          ctrl[2] = 0.8 * Math.sin(1.7 * t + 1);
        }
        for (let k = 0; k < substeps; k++) mujoco.mj_step(sim.model, sim.data);
        sim.steps += substeps;
      }
    }
    syncBodies();
    view.controls.update();
    view.renderer.render(view.scene, view.camera);
    if (now - lastStats > 100) { lastStats = now; onStats(); }
    if (now - lastLog > 1000) {
      lastLog = now;
      const m = bodyId('mug');
      console.log(`[lab] t=${sim.data.time.toFixed(3)} steps=${sim.steps} ncon=${sim.data.ncon} mug_z=${sim.data.xpos[3 * m + 2].toFixed(3)} gz=${sim.model.opt.gravity[2]}`);
    }
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

// ---------------- React control panel ----------------
function Stat({ label, value }) {
  return html`<div class="stat"><span>${label}</span><b>${value}</b></div>`;
}

function App() {
  const [status, setStatus] = useState('loading');
  const [err, setErr] = useState(null);
  const [playing, setPlaying] = useState(true);
  const [armWave, setArmWave] = useState(true);
  const [gravity, setGravity] = useState(9.81);
  const [stats, setStats] = useState({ time: 0, ncon: 0, steps: 0 });

  useEffect(() => {
    const onStats = () => setStats({ time: sim.data.time, ncon: sim.data.ncon, steps: sim.steps });
    startSim(document.getElementById('view'), onStats)
      .then(() => {
        setStatus('ready');
        if (location.search.includes('gtest')) actions.setGravity(3.7); // headless check of the write path
        // demo: show a contact event shortly after load
        setTimeout(() => actions.spawnMugAboveVase(), 1500);
      })
      .catch((e) => {
        console.error('[lab] MuJoCo WASM failed:', e);
        sim.error = String((e && e.stack) || e);
        setErr(String((e && e.message) || e)); setStatus('error');
        document.getElementById('overlay').textContent = 'MuJoCo WASM failed to load: no physics is running';
      });
  }, []);

  const ready = status === 'ready';
  const badge = ready ? html`<span class="badge ok">MuJoCo WASM running in browser</span>`
    : status === 'error' ? html`<span class="badge err">WASM load failed</span>`
    : html`<span class="badge wait">loading WASM…</span>`;

  return html`
    <div>
      <div class="row">${badge}</div>
      <h2>Simulation</h2>
      <div class="row">
        <button class="pill primary" disabled=${!ready}
          onClick=${() => { sim.playing = !playing; setPlaying(!playing); }}>${playing ? 'Pause' : 'Play'}</button>
        <button class="pill" disabled=${!ready} onClick=${() => actions.reset()}>Reset</button>
      </div>
      <h2>Interact</h2>
      <div class="row">
        <button class="pill" disabled=${!ready} onClick=${() => actions.dropBlock()}>Drop a block</button>
        <button class="pill" disabled=${!ready} onClick=${() => actions.spawnMugAboveVase()}>Spawn mug above vase</button>
        <button class="pill" disabled=${!ready}
          onClick=${() => { sim.armWave = !armWave; setArmWave(!armWave); }}>${armWave ? 'Hold arm' : 'Wave arm'}</button>
      </div>
      <h2>Gravity · ${gravity.toFixed(2)} m/s²</h2>
      <div class="row">
        <input type="range" min="0" max="20" step="0.01" value=${gravity} disabled=${!ready}
          onInput=${(e) => { const g = parseFloat(e.target.value); setGravity(g); actions.setGravity(g); }} />
      </div>
      <h2>Live</h2>
      <${Stat} label="Sim time" value=${stats.time.toFixed(2) + ' s'} />
      <${Stat} label="Contacts (data.ncon)" value=${stats.ncon} />
      <${Stat} label="mj_step calls" value=${stats.steps.toLocaleString()} />
      <${Stat} label="Physics tick" value="60 Hz (8 × 2 ms)" />
      ${err && html`<p class="note" style="color:#b3261e">${err}</p>`}
      <p class="note">Engine: <code>${PKG}</code>, the official DeepMind JavaScript/WASM bindings.
        MJCF is compiled in-page via <code>MjModel.from_xml_string</code>; bodies are drawn from${' '}
        <code>data.xpos</code> / <code>data.xquat</code> every frame. Drag to orbit.</p>
    </div>`;
}

createRoot(document.getElementById('root')).render(html`<${App} />`);
