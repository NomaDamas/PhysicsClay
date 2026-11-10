// PhysicsClay 3D spatial editor (client-side three.js view; physics/IK stays on the server).
// window.PC3D = { mount(container), setState(state), active(bool) }. Uses window.PC.send / PC.runTask from app.js.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

const TABLE = { z: 0.74, x0: 0.22, x1: 0.82, y0: -0.5, y1: 0.5 };
const HOVER_Z = 0.82, ZMIN = 0.75, ZMAX = 1.45;
const COL = { ok: 0x1f8f55, bad: 0xd6453a, cur: 0x1da8e5, wp: 0x5c77df, sel: 0xf59145, marker: 0x5c77df, markerP: 0xf59145, ray: 0xf59145 };
const arr = (x) => (Array.isArray(x) ? x : []);
const isXYZ = (p) => Array.isArray(p) && p.length >= 3 && p.slice(0, 3).every((v) => typeof v === "number" && isFinite(v));
const PC = () => window.PC || {};
const send = (o) => (PC().send ? PC().send(o) : false);

let root = null, renderer, scene, camera, orbit, tc, raycaster, msgEl, listEl, sumEl, hintEl;
let needs = true; const redraw = () => { needs = true; };
let isActive = false, loopOn = false, sceneState = "idle", retryTimer = 0;
let state = {};
const geomObjs = new Map(); // id -> Object3D (dynamic ones get matrix updates)
const meshCache = new Map();
let hand = "auto", editing = false; // UX 2.0: clicks add waypoints only while the path editor is open
const wps = []; // THREE.Mesh waypoints
let selected = null, dirty = true, gizmoPress = false, down = null;
const wpGroup = new THREE.Group(), linkGroup = new THREE.Group(), srvGroup = new THREE.Group(), mkGroup = new THREE.Group(), objGroup = new THREE.Group(), rayGroup = new THREE.Group();
const memo = {};
const changed = (k, v) => { const j = JSON.stringify(v); if (memo[k] === j) return false; memo[k] = j; return true; };

// ------------------------------------------------------------------ helpers
function disposeGroup(g) {
  for (const c of [...g.children]) {
    g.remove(c);
    c.traverse((o) => { if (o.geometry && !o.userData.sharedGeo) o.geometry.dispose(); if (o.material) { if (o.material.map) o.material.map.dispose(); o.material.dispose(); } });
  }
}
function setPose(obj, pos, mat) {
  const m = mat && mat.length >= 9 ? mat : [1, 0, 0, 0, 1, 0, 0, 0, 1];
  obj.matrix.set(m[0], m[1], m[2], pos[0], m[3], m[4], m[5], pos[1], m[6], m[7], m[8], pos[2], 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
}
function colorOf(rgba) {
  const [r, g, b] = rgba && rgba.length >= 3 ? rgba : [0.5, 0.5, 0.5];
  const grey = Math.abs(r - g) < 0.07 && Math.abs(g - b) < 0.07 && r > 0.25 && r < 0.9;
  return grey ? new THREE.Color(0xc8cacf) : new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace);
}
function segmentMesh(a, b, radius, color, opacity = 1) {
  const d = new THREE.Vector3().subVectors(b, a), len = d.length();
  const g = new THREE.CylinderGeometry(radius, radius, Math.max(len, 1e-4), 8, 1);
  const m = new THREE.MeshBasicMaterial({ color, transparent: opacity < 1, opacity, depthWrite: opacity >= 1 });
  const c = new THREE.Mesh(g, m);
  c.position.copy(a).addScaledVector(d, 0.5);
  if (len > 1e-6) c.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  return c;
}
function labelSprite(text, bg = "#5c77df", fg = "#ffffff", h = 0.045) {
  const cv = document.createElement("canvas"), ctx = cv.getContext("2d"), fs = 44;
  ctx.font = `600 ${fs}px -apple-system, system-ui, sans-serif`;
  const w = Math.max(64, Math.ceil(ctx.measureText(text).width) + 36); cv.width = w; cv.height = 64;
  ctx.font = `600 ${fs}px -apple-system, system-ui, sans-serif`;
  ctx.fillStyle = bg; ctx.beginPath(); ctx.roundRect(2, 2, w - 4, 60, 30); ctx.fill();
  ctx.fillStyle = fg; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(text, w / 2, 34);
  const tex = new THREE.CanvasTexture(cv); tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sp.scale.set(h * w / 64, h, 1); sp.renderOrder = 10;
  return sp;
}
function say(text, cls = "") { if (!msgEl) return; msgEl.hidden = !text; msgEl.textContent = text || ""; msgEl.className = "v3d-msg " + cls; }

// ------------------------------------------------------------------ scene loading
async function loadMesh(id, url) {
  if (meshCache.has(id)) return meshCache.get(id);
  const p = (async () => {
    const r = await fetch(url); if (!r.ok) throw new Error(`mesh ${id}: HTTP ${r.status}`);
    const buf = await r.arrayBuffer(), dv = new DataView(buf);
    const nv = dv.getUint32(0, true), nf = dv.getUint32(4, true);
    const pos = new Float32Array(buf.slice(8, 8 + nv * 12)), idx = new Uint32Array(buf.slice(8 + nv * 12, 8 + nv * 12 + nf * 12));
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(pos, 3)); g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.computeVertexNormals(); g.computeBoundingSphere();
    return g;
  })();
  meshCache.set(id, p); p.catch(() => meshCache.delete(id));
  return p;
}
// RGB u8 per vertex (an object imported from a photo): only when the count matches the mesh the server sent.
async function addColors(geo, url) {
  if (geo.getAttribute("color")) return true;
  try {
    const r = await fetch(url); if (!r.ok) return false;
    const c = new Uint8Array(await r.arrayBuffer());
    if (c.length !== geo.getAttribute("position").count * 3) return false;
    geo.setAttribute("color", new THREE.BufferAttribute(c, 3, true));
    return true;
  } catch (e) { return false; }
}
function primitive(gm) {
  const s = arr(gm.size).map(Number), a = s[0] || 0.01, b = s[1] || a, c = s[2] || a;
  switch (gm.kind) {
    case "box": return new THREE.BoxGeometry(2 * a, 2 * b, 2 * c);
    case "sphere": return new THREE.SphereGeometry(a, 24, 16);
    case "ellipsoid": { const g = new THREE.SphereGeometry(1, 24, 16); g.scale(a, b, c); return g; }
    case "cylinder": { const g = new THREE.CylinderGeometry(a, a, 2 * b, 28); g.rotateX(Math.PI / 2); return g; }
    case "capsule": { const g = new THREE.CapsuleGeometry(a, 2 * b, 6, 16); g.rotateX(Math.PI / 2); return g; }
    default: return null;
  }
}
async function loadScene() {
  clearTimeout(retryTimer);
  if (sceneState === "loading" || sceneState === "ready") return;
  sceneState = "loading"; say("Loading 3D scene…");
  const gen = loadGen;
  try {
    const r = await fetch("/scene.json", { cache: "no-store" }); if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const sj = await r.json(), meshes = sj.meshes || {};
    const sf = sj.surface; // UX 3: the work surface of the current world (table, kitchen counter, shelf, floor)
    if (sf && [sf.z, sf.x0, sf.x1, sf.y0, sf.y1].every((v) => typeof v === "number" && isFinite(v))) { Object.assign(TABLE, sf); tablePlane.constant = -TABLE.z; }
    const built = await Promise.all(arr(sj.geoms).map(async (gm) => {
      if (!gm || (Array.isArray(gm.rgba) && gm.rgba[3] === 0)) return null;
      if (gm.kind === "plane") return floor(gm);
      let geo = null, shared = false, vcol = false;
      if (gm.kind === "mesh" && gm.mesh != null) {
        const ent = meshes[gm.mesh] || {}, url = ent.url || `/mesh/${gm.mesh}.bin`;
        try { geo = await loadMesh(url, url); shared = true; } catch (e) { console.warn("[3d]", e); return null; }
        if (ent.colors) vcol = await addColors(geo, ent.colors);   // objects imported from photos (pps/recon)
      } else geo = primitive(gm);
      if (!geo) return null;
      const alpha = Array.isArray(gm.rgba) && gm.rgba.length > 3 ? gm.rgba[3] : 1;
      const mat = new THREE.MeshStandardMaterial({ color: vcol ? 0xffffff : colorOf(gm.rgba), vertexColors: vcol, roughness: 0.62, metalness: 0.04, transparent: alpha < 1, opacity: alpha });
      const m = new THREE.Mesh(geo, mat); m.userData.sharedGeo = shared;
      m.userData.gm = { name: gm.name || "", kind: gm.kind, dynamic: !!gm.dynamic };   // P2b: MR ghosts the static furniture
      m.castShadow = true; m.receiveShadow = true; m.matrixAutoUpdate = false;
      setPose(m, arr(gm.pos).length >= 3 ? gm.pos : [0, 0, 0], gm.mat);
      return [String(gm.id), m];
    }));
    if (gen !== loadGen) return; // superseded by reload()
    const g = new THREE.Group(); g.name = "geoms";
    for (const b of built) { if (!b) continue; if (Array.isArray(b)) { geomObjs.set(b[0], b[1]); g.add(b[1]); } else g.add(b); }
    scene.add(g);
    sceneState = "ready"; say(""); needs = true; console.info("[3d] scene ready:", geomObjs.size, "geoms", Math.round(performance.now()) + " ms");
    applyPoses();
    if (loadGen > 0) frameScene(g);   // another robot / scene: frame it instead of the default table preset
  } catch (e) {
    if (gen !== loadGen) return;
    console.warn("[3d] scene load failed:", e && e.message || e);
    sceneState = "error"; say("Loading 3D scene… (server scene not ready, retrying)", "warn");
    retryTimer = setTimeout(loadScene, 3000);
  }
}
let loadGen = 0;
// Fit the camera to the robot + work surface after a world change (ignore huge things like the floor plane).
function frameScene(g) {
  if (!camera || !g) return;
  const box = new THREE.Box3(), b = new THREE.Box3(), size = new THREE.Vector3();
  g.traverse((o) => { if (!o.isMesh) return; b.setFromObject(o); b.getSize(size); if (size.x < 4 && size.y < 4 && size.z < 4) box.union(b); });
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3()), r = Math.max(0.6, box.getSize(size).length() * 0.5);
  const dir = new THREE.Vector3(0.25, -1.0, 0.8).normalize();   // from the side and above: back walls (kitchen backsplash, shelf) never block the view
  CAMS.reset = [[c.x + dir.x * r * 2.2, c.y + dir.y * r * 2.2, c.z + dir.z * r * 2.2], [c.x, c.y, c.z]];
  CAMS.stage = CAMS.reset;
  camPreset("reset");
}
function reload() { // robot body switched: /scene.json changed
  loadGen++; clearTimeout(retryTimer);
  if (scene) { const old = scene.getObjectByName("geoms"); if (old) { scene.remove(old); old.traverse((o) => { if (o.material) o.material.dispose(); if (o.geometry) o.geometry.dispose(); }); } }
  geomObjs.clear(); meshCache.clear(); floors.length = 0; sceneState = "idle"; needs = true;
  if (root && isActive) loadScene();
}
// Stage colours follow the app theme (CSS tokens --stage-bg / --stage-floor / --stage-grid / --stage-grid2).
let hemi = null;
const floors = [];
function themeColors() {
  const cs = getComputedStyle(document.documentElement), c = (k, d) => (cs.getPropertyValue(k) || "").trim() || d;
  return { bg: c("--stage-bg", "#f5f5f7"), floor: c("--stage-floor", "#eceef2"), grid: c("--stage-grid", "#c4c7cf"), grid2: c("--stage-grid2", "#dcdfe5"),
    dark: document.documentElement.dataset.theme === "dark" };
}
function makeGrid(z, t) {
  const grid = new THREE.GridHelper(14, 56, new THREE.Color(t.grid), new THREE.Color(t.grid2)); grid.rotation.x = Math.PI / 2; grid.position.z = z + 0.001;
  return grid;
}
function applyTheme() {
  if (!scene) return;
  const t = themeColors();
  scene.background = new THREE.Color(t.bg); scene.fog.color.set(t.bg);
  if (hemi) { hemi.groundColor.set(t.dark ? 0x3a3c44 : 0xd9dbe2); hemi.intensity = t.dark ? 1.0 : 1.15; }
  for (const f of floors) {
    f.pl.material.color.set(t.floor);
    if (f.grid) { f.g.remove(f.grid); f.grid.geometry.dispose(); f.grid.material.dispose(); }
    f.grid = makeGrid(f.z, t); f.g.add(f.grid);
  }
  needs = true;
}
function floor(gm) {
  const g = new THREE.Group(), z = isXYZ(gm.pos) ? gm.pos[2] : 0, t = themeColors();
  g.userData.floor = true;            // P2b: hidden in passthrough (MR), where the real floor is
  const pl = new THREE.Mesh(new THREE.PlaneGeometry(14, 14), new THREE.MeshStandardMaterial({ color: new THREE.Color(t.floor), roughness: 0.95 }));
  pl.position.z = z; pl.receiveShadow = true; g.add(pl);
  const grid = makeGrid(z, t); g.add(grid);
  floors.push({ g, pl, grid, z });
  return g;
}
function applyPoses() {
  const gp = state.geom_poses; if (!gp || typeof gp !== "object") return;
  for (const k in gp) {
    const o = geomObjs.get(k), v = gp[k]; if (!o || !Array.isArray(v) || v.length < 12) continue;
    setPose(o, v, v.slice(3, 12));
  }
}

// ------------------------------------------------------------------ waypoints
const wpGeo = new THREE.SphereGeometry(0.016, 20, 14);
function addWaypoint(p) {
  const m = new THREE.Mesh(wpGeo, new THREE.MeshStandardMaterial({ color: COL.wp, roughness: 0.4 }));
  m.userData.sharedGeo = true; m.position.copy(p); m.castShadow = true;
  wpGroup.add(m); wps.push(m); dirty = true; rebuildLinks(); return m;
}
function select(m) {
  selected = m || null;
  for (const w of wps) w.material.color.setHex(w === selected ? COL.sel : COL.wp);
  if (selected) tc.attach(selected); else tc.detach();
  rebuildLinks();
}
function removeSelected() {
  if (!selected) return false;
  const i = wps.indexOf(selected); if (i >= 0) wps.splice(i, 1);
  wpGroup.remove(selected); selected.material.dispose(); select(null); dirty = true; rebuildLinks(); return true;
}
function clearLocal() { select(null); for (const w of wps) { wpGroup.remove(w); w.material.dispose(); } wps.length = 0; dirty = true; rebuildLinks(); }
function rebuildLinks() {
  needs = true;
  disposeGroup(linkGroup);
  for (let i = 1; i < wps.length; i++) linkGroup.add(segmentMesh(wps[i - 1].position, wps[i].position, 0.005, COL.wp, 0.85));
  for (const w of wps) { // drop line to the table shows the hover height
    const base = new THREE.Vector3(w.position.x, w.position.y, TABLE.z + 0.002);
    linkGroup.add(segmentMesh(base, w.position, 0.0018, 0x1d1d1f, 0.35));
    const dot = new THREE.Mesh(new THREE.CircleGeometry(0.012, 20), new THREE.MeshBasicMaterial({ color: 0x1d1d1f, transparent: true, opacity: 0.18, depthWrite: false }));
    dot.position.copy(base); linkGroup.add(dot);
  }
  renderHud();
}
const pointsOut = () => wps.map((w) => [w.position.x, w.position.y, w.position.z].map((v) => Math.round(v * 1000) / 1000));
function check() {
  if (wps.length < 2) { hintEl.textContent = "Add at least 2 waypoints on the table first."; return false; }
  const msg = { type: "path3d", points: pointsOut(), hand };
  if (send(msg)) { dirty = false; renderHud(); return true; }
  hintEl.textContent = "Not connected to the robot."; return false;
}
function run() {
  if (wps.length >= 2 && dirty) check();
  if (PC().runTask) PC().runTask("follow_path", {}, "3d");
}
function carry() {
  if (wps.length >= 2 && dirty) check();
  if (PC().runTask) PC().runTask("pick_place", { object: "mug", target: "along my path" }, "3d");
}
const CAMS = { stage: [[1.9, -1.3, 1.15], [0.1, 0, 0.65]], orbit: [[2.6, 1.8, 2.0], [0.3, 0, 0.7]],
  reset: [[2.0, -1.6, 1.7], [0.3, 0, 0.75]], top: [[0.52, -0.001, 3.0], [0.52, 0, 0.74]],
  front: [[2.6, 0, 1.05], [0.45, 0, 0.82]], side: [[0.5, -2.5, 0.98], [0.5, 0, 0.84]],
};
function camPreset(k, ms = 380) {
  const c = CAMS[k] || CAMS.reset; if (!camera) return; needs = true;
  zoomGoal = null; camera.up.set(0, 0, 1);
  flyTo(new THREE.Vector3(...c[0]), new THREE.Vector3(...c[1]), ms);
}
// Smooth camera move (presets, focus on a proposed path). Any user input (wheel, drag) cancels it.
let flyAnim = 0;
function cancelFly() { if (flyAnim) { cancelAnimationFrame(flyAnim); flyAnim = 0; } }
function flyTo(p1, t1, ms) {
  cancelFly();
  const p0 = camera.position.clone(), t0 = orbit.target.clone(), start = performance.now();
  const step = (now) => {
    const u = ms > 0 ? Math.min(1, (now - start) / ms) : 1, e = u * u * (3 - 2 * u);
    camera.position.lerpVectors(p0, p1, e); orbit.target.lerpVectors(t0, t1, e); orbit.update(); needs = true;
    flyAnim = u < 1 ? requestAnimationFrame(step) : 0;
  };
  if (ms > 0) flyAnim = requestAnimationFrame(step); else step(start);
}

// ------------------------------------------------------------------ wheel / pinch zoom
// We zoom ourselves instead of OrbitControls: three r160 scales each wheel step by 1 / (devicePixelRatio | 0), which is
// 1/0 on a page zoomed below 100 % on a standard display (every notch slams to the min or max distance: "two steps")
// and half speed on Retina. Here a step is proportional to the wheel distance in CSS px (line / page deltas
// normalised), the same on every display, eased over a few frames, and centred on the point under the pointer.
const ZOOM = { min: 0.2, max: 8, perPx: 0.0011, pinchPerPx: 0.012, maxStep: 0.6, tau: 70 };
let zoomGoal = null;                 // {r, t}: the distance and orbit target the camera eases toward
const wheelCaptures = [];            // feature modules (a trajectory handle under the pointer) may take the wheel
function wheelPx(ev) {
  const k = ev.deltaMode === 1 ? 18 : ev.deltaMode === 2 ? (root && root.clientHeight) || 600 : 1;
  return ev.deltaY * k;
}
function onWheel(ev) {
  if (!renderer || ev.target !== renderer.domElement) return;       // panels over the stage keep their own wheel
  if (wheelCaptures.some((fn) => { try { return fn({ ev, hit: tableHit(ev) }) === true; } catch (e) { console.error(e); return false; } })) {
    ev.preventDefault(); ev.stopPropagation(); return;
  }
  ev.preventDefault(); ev.stopPropagation();                      // OrbitControls' own (broken) wheel zoom never runs
  if (!orbit.enabled) return;
  cancelFly();
  const step = Math.max(-ZOOM.maxStep, Math.min(ZOOM.maxStep, wheelPx(ev) * (ev.ctrlKey ? ZOOM.pinchPerPx : ZOOM.perPx)));
  zoomAt(Math.exp(step), ev);
}
// Scale the camera distance by f (>1 = out), keeping the point under the pointer where it is on screen.
function zoomAt(f, ev) {
  if (!camera) return;
  const r0 = zoomGoal ? zoomGoal.r : camera.position.distanceTo(orbit.target);
  const t0 = zoomGoal ? zoomGoal.t.clone() : orbit.target.clone();
  const r1 = Math.min(ZOOM.max, Math.max(ZOOM.min, r0 * f)), s = r1 / r0;
  let t1 = t0;
  if (ev && Math.abs(s - 1) > 1e-6) {
    raycaster.setFromCamera(ndc(ev), camera);
    const p = new THREE.Vector3(), plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -t0.z);
    if (raycaster.ray.intersectPlane(plane, p) && p.distanceTo(t0) < 2.5 * r0) t1 = p.clone().add(t0.clone().sub(p).multiplyScalar(s));
  }
  zoomGoal = { r: r1, t: t1 }; needs = true;
}
function easeZoom(dt) {
  if (!zoomGoal) return false;
  const a = 1 - Math.exp(-dt / ZOOM.tau), off = camera.position.clone().sub(orbit.target), r = off.length();
  orbit.target.lerp(zoomGoal.t, a);
  const nr = r + (zoomGoal.r - r) * a, done = Math.abs(nr - zoomGoal.r) < 1e-4 && orbit.target.distanceTo(zoomGoal.t) < 1e-4;
  camera.position.copy(orbit.target).addScaledVector(off.normalize(), done ? zoomGoal.r : nr);
  if (done) { orbit.target.copy(zoomGoal.t); zoomGoal = null; }
  return true;
}
function clearAll() { clearLocal(); send({ type: "clear" }); }

function renderHud() {
  if (!listEl) return;
  listEl.innerHTML = "";
  wps.forEach((w, i) => {
    const li = document.createElement("li"); li.className = w === selected ? "sel" : "";
    const h = (w.position.z - TABLE.z) * 100;
    li.innerHTML = `<b>${i + 1}</b><span class="mono">x ${w.position.x.toFixed(2)} · y ${w.position.y.toFixed(2)}</span><span class="mono h">+${h.toFixed(1)} cm</span>`;
    li.addEventListener("click", () => select(w));
    listEl.appendChild(li);
  });
  const p = state.path && typeof state.path === "object" ? state.path : null, ok = p ? arr(p.ok) : [];
  if (!wps.length && !(p && arr(p.pts).length)) sumEl.textContent = "No waypoints yet.";
  else if (wps.length && dirty) { sumEl.textContent = `${wps.length} waypoint${wps.length === 1 ? "" : "s"} · not checked`; sumEl.dataset.v = ""; }
  else if (p && ok.length) {
    const bad = ok.filter((x) => x === false).length;
    sumEl.textContent = bad ? `Physics: ${bad} of ${ok.length} unreachable` : `Physics: all ${ok.length} reachable`;
    sumEl.dataset.v = bad ? "bad" : "ok";
  } else { sumEl.textContent = "Checking…"; sumEl.dataset.v = ""; }
}

// ------------------------------------------------------------------ server overlays
function renderServer() {
  const s = state, p = s.path && typeof s.path === "object" ? s.path : null;
  const pts = p ? arr(p.pts) : [], ok = p ? arr(p.ok) : [], running = s.mode === "PATH", cur = running && p ? Number(p.i) : -1;
  if (changed("srv", [pts, ok, cur])) {
    disposeGroup(srvGroup);
    const v = pts.map((q) => (isXYZ(q) ? new THREE.Vector3(q[0], q[1], q[2]) : null));
    for (let i = 1; i < v.length; i++) {
      if (!v[i - 1] || !v[i]) continue;
      srvGroup.add(segmentMesh(v[i - 1], v[i], 0.0035, ok[i] === false || ok[i - 1] === false ? COL.bad : COL.ok, 0.7));
    }
    v.forEach((q, i) => {
      if (!q) return;
      const isCur = i === cur, r = isCur ? 0.028 : 0.022;
      const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({
        color: isCur ? COL.cur : ok[i] === false ? COL.bad : COL.ok, transparent: !isCur, opacity: isCur ? 1 : 0.5, depthWrite: isCur }));
      m.position.copy(q); srvGroup.add(m);
    });
    renderHud();
  }
  const mk = arr(s.markers).filter((m) => m && isXYZ(m.pos)).map((m) => [String(m.id ?? "?"), m.pos.slice(0, 3)]);
  if (changed("mk", mk)) {
    disposeGroup(mkGroup);
    for (const [id, q] of mk) {
      const col = id === "P" ? COL.markerP : COL.marker, base = new THREE.Vector3(q[0], q[1], q[2]), top = base.clone().add(new THREE.Vector3(0, 0, 0.09));
      mkGroup.add(segmentMesh(base, top, 0.0025, col));
      const disc = new THREE.Mesh(new THREE.CircleGeometry(0.018, 24), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.45, depthWrite: false }));
      disc.position.copy(base).add(new THREE.Vector3(0, 0, 0.002)); mkGroup.add(disc);
      const lab = labelSprite(id, id === "P" ? "#f59145" : "#5c77df", "#fff", 0.04); lab.position.copy(top).add(new THREE.Vector3(0, 0, 0.022)); mkGroup.add(lab);
    }
  }
  const ob = arr(s.scene_objects).filter((o) => o && isXYZ(o.pos)).map((o) => [String(o.name || "object"), o.pos.slice(0, 3).map((x) => Math.round(x * 200) / 200)]);
  if (changed("obj", ob)) {
    disposeGroup(objGroup);
    for (const [name, q] of ob) {
      const lab = labelSprite(name, "rgba(29,29,31,.82)", "#fff", 0.034); lab.position.set(q[0], q[1], q[2] + 0.13); objGroup.add(lab);
    }
  }
  const ray = s.gesture && Array.isArray(s.gesture.ray) && isXYZ(s.gesture.ray[0]) && isXYZ(s.gesture.ray[1]) ? s.gesture.ray : null;
  if (changed("ray", ray ? ray.map((q) => q.map((x) => Math.round(x * 100) / 100)) : null)) {
    disposeGroup(rayGroup);
    if (ray) {
      const a = new THREE.Vector3(...ray[0].slice(0, 3)), b = new THREE.Vector3(...ray[1].slice(0, 3));
      rayGroup.add(segmentMesh(a, b, 0.004, COL.ray, 0.75));
      const end = new THREE.Mesh(new THREE.SphereGeometry(0.02, 16, 12), new THREE.MeshBasicMaterial({ color: COL.ray })); end.position.copy(b); rayGroup.add(end);
    }
  }
}

// ------------------------------------------------------------------ interaction
const tablePlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -TABLE.z);
let hoverGhost = null;
function ndc(ev) { const r = renderer.domElement.getBoundingClientRect(); return new THREE.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1); }
function tableHit(ev) {
  raycaster.setFromCamera(ndc(ev), camera);
  const p = new THREE.Vector3(); if (!raycaster.ray.intersectPlane(tablePlane, p)) return null;
  return p.x >= TABLE.x0 && p.x <= TABLE.x1 && p.y >= TABLE.y0 && p.y <= TABLE.y1 ? p : null;
}
// Feature tools (web/ui/*.js) can take over the pointer: PC3D.capture(fn) where fn({type, ev, hit}) returns true to consume.
const captures = [];
function captured(type, ev) {
  if (!captures.length) return false;
  const hit = tableHit(ev);
  return captures.some((fn) => { try { return fn({ type, ev, hit }) === true; } catch (e) { console.error(e); return false; } });
}
function onDown(ev) {
  cancelFly();                                                     // grabbing the view stops a fly-to
  if (captured("down", ev)) { orbit.enabled = false; down = null; return; }
  gizmoPress = !!(tc.dragging || tc.axis); down = { x: ev.clientX, y: ev.clientY, b: ev.button };
}
function onUp(ev) {
  if (captured("up", ev)) { orbit.enabled = true; down = null; return; }
  const d = down; down = null;
  if (!d || d.b !== 0 || gizmoPress || Math.hypot(ev.clientX - d.x, ev.clientY - d.y) > 5) { gizmoPress = false; return; }
  raycaster.setFromCamera(ndc(ev), camera);
  const hit = raycaster.intersectObjects(wps, false)[0];
  if (hit) { select(hit.object); return; }
  const p = tableHit(ev);
  if (p && editing) { addWaypoint(new THREE.Vector3(p.x, p.y, HOVER_Z)); select(null); hintEl.textContent = "Click a waypoint to move it — the blue (Z) arrow sets its height."; }
  else select(null);
}
function onMove(ev) {
  if (captured("move", ev)) { if (hoverGhost) hoverGhost.visible = false; return; }
  if (!hoverGhost || down || !editing) { if (hoverGhost) hoverGhost.visible = false; return; }
  const p = tableHit(ev); hoverGhost.visible = !!p && !tc.axis;
  if (p) hoverGhost.position.set(p.x, p.y, HOVER_Z);
  needs = true;
}
function onKey(ev) {
  if (!isActive) return;
  const tg = ev.target;
  if (tg && (tg.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(tg.tagName))) return;
  if (ev.metaKey || ev.ctrlKey || ev.altKey) return;
  const dlg = document.getElementById("settings"); if (dlg && dlg.open) return;
  if ((ev.key === "Delete" || ev.key === "Backspace") && selected) { ev.preventDefault(); ev.stopImmediatePropagation(); removeSelected(); }
  else if (ev.key === "Escape" && selected) { ev.stopImmediatePropagation(); select(null); } // app's Esc = stop only when nothing selected
  else if (ev.key === "c" || ev.key === "C") clearLocal(); // app.js also sends {type:"clear"}
  else if ((ev.key === "f" || ev.key === "F") && dirty && wps.length >= 2) check(); // app.js then runs follow_path
}

// ------------------------------------------------------------------ mount / loop
function buildUi(container) {
  container.innerHTML = `
    <button type="button" class="btn sm v3d-toggle" aria-expanded="false" title="Show the 3D path editor">Edit path</button>
    <div class="v3d-panel" role="group" aria-label="3D path editor" hidden>
      <div class="v3d-row"><span class="label mono">3D PATH</span><span class="grow"></span>
        <div class="seg" role="group" aria-label="Hand">
          <button type="button" class="seg-btn" data-hand="auto" aria-pressed="true">Auto</button>
          <button type="button" class="seg-btn" data-hand="left" aria-pressed="false">Left</button>
          <button type="button" class="seg-btn" data-hand="right" aria-pressed="false">Right</button>
        </div></div>
      <div class="v3d-row">
        <button type="button" class="btn sm" data-v3d="check">Check</button>
        <button type="button" class="btn sm primary" data-v3d="run">Run</button>
        <button type="button" class="btn sm primary" data-v3d="carry" title="pick_place mug along my path">Carry mug along path</button>
        <button type="button" class="btn sm ghost" data-v3d="clear">Clear</button>
      </div>
      <div class="v3d-sum mono"></div>
      <ol class="v3d-list"></ol>
      <details class="v3d-hint muted small"><summary>How to</summary>Click the table to add a waypoint · click one to move it (Z = height) · Del removes · Esc deselects · drag to orbit, right-drag to pan</details>
      <div class="v3d-tip muted small"></div>
    </div>
    <div class="v3d-views seg" role="group" aria-label="Camera presets">
      <button type="button" class="seg-btn" data-cam="stage">Stage</button>
      <button type="button" class="seg-btn" data-cam="orbit">Orbit</button>
      <button type="button" class="seg-btn" data-cam="top">Top</button>
      <button type="button" class="seg-btn" data-cam="side">Side</button>
      <button type="button" class="seg-btn" data-cam="reset" aria-label="Reset view" title="Reset view"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.75" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" style="display:block"><path d="M4 12a8 8 0 1 0 2.4-5.7M4 4v4h4"/></svg></button>
    </div>
    <div class="v3d-msg" hidden></div>`;
  msgEl = container.querySelector(".v3d-msg"); listEl = container.querySelector(".v3d-list");
  sumEl = container.querySelector(".v3d-sum"); hintEl = container.querySelector(".v3d-tip");
  const panel = container.querySelector(".v3d-panel");
  const tog = container.querySelector(".v3d-toggle");
  ["pointerdown", "pointerup", "pointermove"].forEach((t) => tog.addEventListener(t, (e) => e.stopPropagation()));
  tog.addEventListener("click", () => editPath(panel.hidden));
  ["pointerdown", "pointerup", "pointermove", "wheel"].forEach((t) => panel.addEventListener(t, (e) => e.stopPropagation()));
  panel.querySelectorAll("[data-hand]").forEach((b) => b.addEventListener("click", () => {
    hand = b.dataset.hand; dirty = true;
    panel.querySelectorAll("[data-hand]").forEach((x) => x.setAttribute("aria-pressed", String(x === b))); renderHud();
  }));
  panel.querySelector('[data-v3d="check"]').addEventListener("click", check);
  panel.querySelector('[data-v3d="run"]').addEventListener("click", run);
  panel.querySelector('[data-v3d="clear"]').addEventListener("click", clearAll);
  panel.querySelector('[data-v3d="carry"]').addEventListener("click", carry);
  const views = container.querySelector(".v3d-views");
  ["pointerdown", "pointerup", "pointermove", "wheel"].forEach((t) => views.addEventListener(t, (e) => e.stopPropagation()));
  views.querySelectorAll("[data-cam]").forEach((b) => b.addEventListener("click", () => camPreset(b.dataset.cam)));
}
function editPath(on) { // open / close the waypoint editor (the "Path" tool calls this too)
  if (!root) return;
  const panel = root.querySelector(".v3d-panel"), tog = root.querySelector(".v3d-toggle");
  editing = !!on; panel.hidden = !editing;
  tog.setAttribute("aria-expanded", String(editing)); tog.textContent = editing ? "Done editing" : "Edit path";
  if (!editing && hoverGhost) hoverGhost.visible = false;
  needs = true; window.dispatchEvent(new CustomEvent("pc-path-edit", { detail: editing }));
}
function mount(container) {
  if (root || !container) return;
  root = container; root.classList.add("view3d");
  buildUi(root);
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.className = "v3d-canvas";
  root.prepend(renderer.domElement);
  scene = new THREE.Scene(); scene.background = new THREE.Color(0xf5f5f7); scene.fog = new THREE.Fog(0xf5f5f7, 7, 16);
  camera = new THREE.PerspectiveCamera(42, 4 / 3, 0.02, 60); camera.up.set(0, 0, 1); camera.position.set(2.0, -1.6, 1.7);
  orbit = new OrbitControls(camera, renderer.domElement); orbit.target.set(0.3, 0, 0.75); orbit.enableDamping = true; orbit.dampingFactor = 0.12;
  orbit.enableRotate = orbit.enablePan = orbit.enableZoom = true; orbit.screenSpacePanning = true;   // enableZoom: touch pinch
  orbit.minDistance = ZOOM.min; orbit.maxDistance = ZOOM.max; orbit.update(); // orbit (left drag) · pan (right drag / shift) · zoom (wheel: onWheel)
  root.addEventListener("wheel", onWheel, { capture: true, passive: false });
  hemi = new THREE.HemisphereLight(0xffffff, 0xd9dbe2, 1.15); hemi.position.set(0, 0, 1); scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 1.9); dir.position.set(1.6, -1.8, 4.2); dir.castShadow = true;
  dir.shadow.mapSize.set(1024, 1024); Object.assign(dir.shadow.camera, { left: -1.6, right: 1.6, top: 1.6, bottom: -1.6, near: 0.5, far: 10 }); dir.shadow.bias = -0.0004;
  scene.add(dir, dir.target);
  // editable zone outline on the table top
  const zone = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([[TABLE.x0, TABLE.y0], [TABLE.x1, TABLE.y0], [TABLE.x1, TABLE.y1], [TABLE.x0, TABLE.y1]]
    .map(([x, y]) => new THREE.Vector3(x, y, TABLE.z + 0.003))), new THREE.LineBasicMaterial({ color: 0x5c77df, transparent: true, opacity: 0.45 }));
  scene.add(zone, wpGroup, linkGroup, srvGroup, mkGroup, objGroup, rayGroup, ...layers.values());
  hoverGhost = new THREE.Mesh(wpGeo, new THREE.MeshBasicMaterial({ color: COL.wp, transparent: true, opacity: 0.35, depthWrite: false })); hoverGhost.visible = false; scene.add(hoverGhost);
  raycaster = new THREE.Raycaster();
  tc = new TransformControls(camera, renderer.domElement); tc.setMode("translate"); tc.setSpace("world"); tc.setSize(0.6);
  tc.addEventListener("dragging-changed", (e) => { orbit.enabled = !e.value; if (e.value) gizmoPress = true; });
  tc.addEventListener("objectChange", () => {
    if (!selected) return;
    selected.position.z = Math.min(ZMAX, Math.max(ZMIN, selected.position.z));
    selected.position.x = Math.min(TABLE.x1 + 0.15, Math.max(TABLE.x0 - 0.15, selected.position.x));
    selected.position.y = Math.min(TABLE.y1 + 0.15, Math.max(TABLE.y0 - 0.15, selected.position.y));
    dirty = true; rebuildLinks();
  });
  scene.add(tc);
  tc.addEventListener("change", redraw); orbit.addEventListener("change", redraw);
  const cv = renderer.domElement;
  cv.addEventListener("pointerdown", onDown); cv.addEventListener("pointerup", onUp); cv.addEventListener("pointermove", onMove);
  cv.addEventListener("pointerleave", () => { if (hoverGhost) hoverGhost.visible = false; });
  cv.addEventListener("contextmenu", (e) => e.preventDefault());
  document.addEventListener("keydown", onKey, true);
  new ResizeObserver(resize).observe(root); resize();
  new MutationObserver(applyTheme).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] }); applyTheme();
  renderHud(); renderServer();
  if (isActive) { loadScene(); startLoop(); }
}
function resize() {
  if (!root || !renderer || (renderer.xr && renderer.xr.isPresenting)) return;   // P2b: the headset owns the size while in VR
  const w = Math.max(1, root.clientWidth), h = Math.max(1, root.clientHeight);
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); needs = true;
}
function startLoop() {
  if (loopOn) return; loopOn = true;
  let last = performance.now();
  const tick = (now) => {
    if (!isActive || !root) { loopOn = false; return; }
    if (renderer.xr && renderer.xr.isPresenting) { needs = true; requestAnimationFrame(tick); return; }   // P2b: web/xr.js renders in VR
    const dt = Math.min(100, Math.max(0, (now || performance.now()) - last)); last = now || performance.now();
    const zoomed = easeZoom(dt);
    if (orbit.update() || needs || zoomed) { needs = false; renderer.render(scene, camera); } // render on demand (cheap when idle)
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}
function active(on) {
  isActive = !!on;
  if (root) root.hidden = !isActive;
  if (!isActive || !root) return;
  resize(); loadScene(); startLoop();
}
function setState(s) {
  state = s && typeof s === "object" ? s : {};
  if (!root) return;
  if (sceneState === "ready") applyPoses();
  renderServer(); needs = true;
}

function project(x, y, z) { // world -> client px (debug / tests)
  const v = new THREE.Vector3(x, y, z).project(camera), r = renderer.domElement.getBoundingClientRect();
  return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height];
}
// Named layers feature modules draw into (cleared and rebuilt by their owner).
const layers = new Map();
function layer(name) {
  if (!layers.has(name)) { const g = new THREE.Group(); g.name = "layer:" + name; layers.set(name, g); if (scene) scene.add(g); }
  return layers.get(name);
}
function capture(fn) { captures.push(fn); return () => { const i = captures.indexOf(fn); if (i >= 0) captures.splice(i, 1); }; }
// Fly the camera to frame these world points, keeping the current viewing direction (P2: a proposed trajectory).
function focus(points, opts = {}) {
  if (!camera || !orbit) return;
  const box = new THREE.Box3();
  for (const p of arr(points)) if (isXYZ(p)) box.expandByPoint(new THREE.Vector3(p[0], p[1], p[2]));
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3()), r = Math.max(opts.min || 0.22, box.getSize(new THREE.Vector3()).length() / 2);
  const vf = THREE.MathUtils.degToRad(camera.fov), hf = 2 * Math.atan(Math.tan(vf / 2) * camera.aspect);
  const dir = new THREE.Vector3().subVectors(camera.position, orbit.target).normalize();
  const dist = Math.min(ZOOM.max, Math.max(ZOOM.min, r / Math.sin(Math.min(vf, hf) / 2) * (opts.pad || 1.1)));
  zoomGoal = null;
  flyTo(c.clone().addScaledVector(dir, dist), c, opts.ms == null ? 450 : opts.ms);
}
// Is every point on screen (inside a margin) and is the group big enough to work with? (callers skip a fly-to then)
function inView(points, margin = 0.08, minSpan = 0.18) {
  if (!camera) return false;
  let x0 = 1, x1 = -1, y0 = 1, y1 = -1, n = 0;
  for (const p of arr(points)) {
    if (!isXYZ(p)) continue;
    const v = new THREE.Vector3(p[0], p[1], p[2]).project(camera); n++;
    if (v.z > 1 || Math.abs(v.x) > 1 - margin || Math.abs(v.y) > 1 - margin) return false;
    x0 = Math.min(x0, v.x); x1 = Math.max(x1, v.x); y0 = Math.min(y0, v.y); y1 = Math.max(y1, v.y);
  }
  return n > 0 && Math.max(x1 - x0, y1 - y0) / 2 >= minSpan;
}
window.PC3D = {
  mount, setState, active, reload, editPath, get editing() { return editing; }, project, layer, capture, redraw, disposeGroup, labelSprite, segmentMesh, TABLE, THREE, focus, inView,
  // wheel: fn({ev, hit}) -> true to take the wheel (a trajectory handle under the pointer); zoomBy(f) scales the distance (tests, keys)
  wheelCapture(fn) { wheelCaptures.push(fn); return () => { const i = wheelCaptures.indexOf(fn); if (i >= 0) wheelCaptures.splice(i, 1); }; },
  zoomBy(f) { zoomAt(f, null); }, get zoomLimits() { return { min: ZOOM.min, max: ZOOM.max }; },
  get orbitTarget() { return orbit ? orbit.target : null; }, get distance() { return camera && orbit ? camera.position.distanceTo(orbit.target) : null; },
  get scene() { return scene; }, get camera() { return camera; }, get mounted() { return !!root; }, get isActive() { return isActive; },
  get renderer() { return renderer; }, get geoms() { return scene ? scene.getObjectByName("geoms") : null; }, get sceneReady() { return sceneState === "ready"; },
  tableHit: (ev) => (renderer ? tableHit(ev) : null),
  get waypoints() { return pointsOut(); }, get selectedIndex() { return wps.indexOf(selected); },
};
window.dispatchEvent(new Event("pc3d-ready"));
