// P2 · 3D trajectories: say it, see it, fix only what's wrong.
// The robot proposes a path (from words, a demonstration, a stroke or keyframes) and pps/features/traj.py sends it as
// state.traj: a ghost gripper rides along the checked path (blue = fine, red = not), every point has a handle.
//   drag a handle .............. move it on its horizontal plane
//   Alt-drag / wheel / ↕ chip ... height, with a guide line down to the surface and a height label
//   the ring around a point ..... turn the wrist (yaw); tilt, speed, force, grip and tolerance are chips in the popover
//   Path tool (dock) ............ click 2-5 spots to drop ghost grippers: they connect into a checked path
// Keyboard (a point selected): arrows move it 1 cm (Shift 5 cm), PageUp/PageDown height, [ ] turn the wrist,
// Delete removes it, Esc deselects. Messages: {type:"traj", op:"propose"|"edit"|"run"|"clear"} (docs/PROTOCOL.md v9).
"use strict";
(function () {
  const P = window.PCUI;
  if (!P) return;
  const arr = (x) => (Array.isArray(x) ? x : []);
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const r3 = (v) => Math.round(v * 1000) / 1000;
  const SRC = { language: "From your words", draw: "Drawn", keyframes: "Keyframes", demo: "Shown by you", vr: "VR", skill: "Skill" };
  const MAX_KEYS = 5;
  const SPEEDS = [["Slow", 0.06], ["Normal", null], ["Fast", 0.3]];
  const FORCES = [0, 2, 5, 10];
  const TOLS = [[null, "Auto"], [0.005, "5 mm"], [0.01, "1 cm"], [0.02, "2 cm"]];
  const IC = {
    path: '<path d="M4 18c3 0 4-3 6-6s4-6 7-6"/><circle cx="4" cy="18" r="1.6"/><circle cx="17" cy="6" r="1.6"/>',
    plus: '<path d="M12 5v14M5 12h14"/>', minus: '<path d="M5 12h14"/>', trash: '<path d="M5 7h14M10 7V5h4v2M7 7l1 12h8l1-12"/>',
    up: '<path d="m7 14 5-5 5 5"/>', down: '<path d="m7 10 5 5 5-5"/>', rot: '<path d="M19 12a7 7 0 1 1-2.05-4.95"/><path d="M19 4v4h-4"/>',
    x: '<path d="M6 6l12 12M18 6 6 18"/>', play: '<path d="M8 5v14l11-7z"/>', stop: '<rect x="6.5" y="6.5" width="11" height="11" rx="2"/>',
    help: '<circle cx="12" cy="12" r="8.5"/><path d="M9.8 9.5a2.3 2.3 0 0 1 4.4.9c0 1.6-2.2 1.9-2.2 3.4M12 16.8v.2"/>',
    focus: '<path d="M4 9V5.5A1.5 1.5 0 0 1 5.5 4H9M15 4h3.5A1.5 1.5 0 0 1 20 5.5V9M20 15v3.5a1.5 1.5 0 0 1-1.5 1.5H15M9 20H5.5A1.5 1.5 0 0 1 4 18.5V15"/><circle cx="12" cy="12" r="2.5"/>',
  };
  const svg = (n, s = 16) => `<svg width="${s}" height="${s}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.75" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${IC[n] || ""}</svg>`;

  // ------------------------------------------------------------------ styles
  const CSS = `
.tj-card { width: 100%; box-sizing: border-box; background: var(--raised, #fff); color: var(--t1, #1d1d1f); border: 1px solid var(--line, rgba(29,29,31,.1));
  border-radius: var(--r3, 20px); box-shadow: var(--shadow-md, 0 12px 32px rgba(0,0,0,.1)); padding: 14px 16px; display: flex; flex-direction: column; gap: 10px;
  font: 13px/1.4 var(--sans, system-ui); min-width: 0; }
.tj-card[hidden], .tj-pop[hidden], .tj-prof[hidden] { display: none !important; }
.tj-head { display: flex; align-items: center; gap: 10px; min-width: 0; }
.tj-ico { width: 32px; height: 32px; border-radius: 10px; display: grid; place-items: center; color: var(--accent-ink); background: var(--accent-tint, rgba(92,119,223,.12)); flex: none; }
.tj-title { flex: 1; min-width: 0; }
.tj-title b { display: block; font-size: 15px; font-weight: 600; letter-spacing: 0; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.tj-title span { color: var(--t2, rgba(29,29,31,.64)); font-size: 12px; }
.tj-state { --c: var(--accent, #5c77df); --ci: var(--accent-ink); font-size: 12px; font-weight: 600; padding: 4px 10px; border-radius: var(--r-ctl, 10px); color: var(--ci); white-space: nowrap;
  background: color-mix(in srgb, var(--c) 12%, transparent); }
.tj-state[data-s="blocked"] { --c: var(--bad, #d6453a); --ci: var(--bad-ink); } .tj-state[data-s="ready"] { --c: var(--ok, #1f8f55); --ci: var(--ok-ink); } .tj-state[data-s="running"] { --c: var(--hot, #f59145); --ci: var(--hot-ink); }
.tj-sum { margin: 0; color: var(--t2, rgba(29,29,31,.64)); }
.tj-sum.bad { color: var(--bad-ink, #b3342b); background: color-mix(in srgb, var(--bad, #d6453a) 8%, transparent); padding: 8px 10px; border-radius: 12px; }
.tj-strip { display: flex; gap: 6px; overflow-x: auto; padding: 2px; scrollbar-width: thin; min-width: 0; }
.tj-pt { flex: none; min-width: 36px; height: 36px; padding: 0 6px; border-radius: var(--r-ctl, 10px); border: 1.5px solid var(--line2, rgba(29,29,31,.16)); background: var(--raised, #fff);
  color: var(--t1, #1d1d1f); font: 600 13px var(--sans, system-ui); cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 4px; }
.tj-pt:hover { background: var(--active, #f0f0f3); }
.tj-pt[data-bad="true"] { border-color: var(--bad, #d6453a); color: var(--bad-ink, #b3342b); }
.tj-pt[aria-pressed="true"] { background: var(--hot, #f59145); border-color: var(--hot, #f59145); color: #fff; }
.tj-pt i { width: 6px; height: 6px; border-radius: 50%; background: currentColor; opacity: .7; }
.tj-pt:focus-visible, .tj-btn:focus-visible, .tj-chip:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.tj-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
.tj-btn { min-height: 44px; padding: 0 14px; border-radius: var(--r-ctl, 10px); border: 1px solid var(--line2, rgba(29,29,31,.16)); background: var(--raised, #fff);
  color: var(--t1, #1d1d1f); font: 600 13.5px var(--sans, system-ui); display: inline-flex; align-items: center; justify-content: center; gap: 6px; cursor: pointer; }
.tj-btn:hover:not(:disabled) { background: var(--active, #f0f0f3); }
.tj-btn:disabled { opacity: .45; cursor: default; }
.tj-btn.primary { background: var(--accent, #5c77df); border-color: transparent; color: var(--on-accent, #fff); flex: 1; }
.tj-btn.primary:hover:not(:disabled) { filter: brightness(1.06); background: var(--accent, #5c77df); }
.tj-btn.danger { color: var(--bad-ink, #b3342b); }
.tj-btn.icon { padding: 0; width: 44px; }
.tj-tip { margin: 0; color: var(--t2, rgba(29,29,31,.64)); font-size: 12px; }
.tj-tip kbd { font: 600 11px var(--mono, monospace); padding: 1px 4px; border-radius: 5px; background: var(--field, #f5f5f7); border: 1px solid var(--line, rgba(29,29,31,.1)); }
/* popover next to the selected point (stage overlay) */
.tj-pop { position: absolute; z-index: 9; width: 316px; max-width: calc(100% - 20px); box-sizing: border-box; padding: 8px 12px 12px; border-radius: 18px;
  background: var(--glass-strong); color: var(--t1, #1d1d1f); box-shadow: var(--shadow-lg);
  border: 1px solid var(--line, rgba(29,29,31,.1)); font: 12.5px/1.35 var(--sans, system-ui); display: flex; flex-direction: column; gap: 8px; }
.tj-pop header { display: flex; align-items: center; gap: 6px; }
.tj-pop header b { flex: 1; font-size: 13.5px; }
.tj-pop header .tj-x { width: 32px; height: 32px; border: 0; border-radius: 50%; background: transparent; color: var(--t2, rgba(29,29,31,.64)); cursor: pointer; display: grid; place-items: center; }
.tj-pop header .tj-x:hover { background: var(--active, #f0f0f3); }
.tj-line { display: grid; grid-template-columns: 66px 1fr; align-items: center; gap: 6px; min-height: 32px; }
.tj-line > span { color: var(--t2, rgba(29,29,31,.64)); font-size: 12px; }
.tj-chips { display: flex; flex-wrap: nowrap; gap: 6px; align-items: center; min-width: 0; }
.tj-seg { display: inline-flex; gap: 2px; padding: 2px; border-radius: var(--r-ctl, 10px); background: var(--field, #f5f5f7); min-width: 0; }
.tj-chip { height: 28px; min-width: 28px; padding: 0 8px; border-radius: var(--r-ctl, 10px); border: 0; background: transparent; white-space: nowrap;
  color: var(--t1, #1d1d1f); font: 500 12px var(--sans, system-ui); cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 4px; }
.tj-chip:hover { background: var(--active, #e9e9ee); }
.tj-chip[aria-pressed="true"] { background: var(--raised, #fff); color: var(--t1, #1d1d1f); font-weight: 600; box-shadow: 0 1px 3px rgba(0,0,0,.12); }
.tj-chip.icon { padding: 0; width: 28px; }
.tj-val { font: 600 12px/1 var(--mono, monospace); min-width: 46px; height: 28px; display: inline-flex; align-items: center; justify-content: center; white-space: nowrap; }
.tj-pop footer { display: flex; gap: 6px; padding-top: 2px; }
.tj-pop footer .tj-chip { flex: 1; height: 34px; border: 1px solid var(--line2, rgba(29,29,31,.16)); background: var(--raised, #fff); }
.tj-pop footer .tj-chip:hover { background: var(--active, #f0f0f3); }
.tj-pop footer .tj-chip.danger { color: var(--bad-ink, #b3342b); }
.tj-pop .tj-bad { color: var(--bad-ink, #b3342b); font-size: 12px; margin: 0; }
/* 2D fallback: height profile for a stroke on the robot's view */
.tj-prof { display: inline-flex; align-items: center; gap: 6px; font-size: var(--fs-xs, 12px); font-weight: 500; color: var(--t2, rgba(29,29,31,.64)); }
.tj-prof input { width: 84px; accent-color: var(--accent, #5c77df); }
.tj-prof output { font: 600 11px var(--mono, monospace); min-width: 44px; color: var(--t1, #1d1d1f); }
.path-bar:has(.tj-prof:not([hidden])) { flex-wrap: wrap; justify-content: center; white-space: normal; max-width: calc(100% - 24px); border-radius: 22px; }
@media (pointer: coarse) { .tj-chip { height: 36px; min-width: 36px; } .tj-chip.icon { width: 36px; } .tj-pt { height: 40px; min-width: 40px; } }
.tj-card .tj-pop { position: static; width: auto; max-width: none; box-shadow: none; background: var(--field, #f5f5f7); border: 0; padding: 10px; }
.tj-card .tj-seg { background: var(--raised, #fff); }
@media (max-width: 760px) {
  .tj-line { grid-template-columns: 62px 1fr; }
  .tj-chips { flex-wrap: wrap; }
  .tj-prof input { width: 64px; }
}`;
  const st = document.createElement("style"); st.id = "pcui-traj-css"; st.textContent = CSS; document.head.appendChild(st);

  // ------------------------------------------------------------------ DOM: card (side slot) + popover (stage slot)
  const card = document.createElement("section");
  card.className = "tj-card"; card.hidden = true; card.setAttribute("aria-label", "Path in 3D");
  P.slot("side").prepend(card);           // right under the robot's plan
  const body = document.createElement("div"), popHost = document.createElement("div");
  body.style.cssText = "display:flex;flex-direction:column;gap: 10px;min-width:0";
  card.append(body, popHost);
  const phone = window.matchMedia ? matchMedia("(max-width: 760px)") : { matches: false };
  const pop = document.createElement("div");
  pop.className = "tj-pop"; pop.hidden = true; pop.setAttribute("role", "dialog"); pop.setAttribute("aria-label", "Edit the selected point");
  // the point editor floats next to its point over the 3D stage; on a phone the stage is too small, so it docks in the card
  function dockPop() { const host = phone.matches ? popHost : P.slot("stage"); if (pop.parentNode !== host) host.appendChild(pop); }
  dockPop();
  if (phone.addEventListener) phone.addEventListener("change", () => { dockPop(); renderPop(); });
  ["pointerdown", "pointerup", "pointermove", "wheel", "click", "dblclick"].forEach((t) => pop.addEventListener(t, (e) => e.stopPropagation()));

  // ------------------------------------------------------------------ state
  let tj = {}, g = null, tr = null, which = null;   // state.traj, its ghost, the trajectory shown, "proposed" | "current"
  let sel = -1, hover = -1, drag = null, press = null, seq = 0;
  // local previews of moved points until the server has the edit that carries them (then its ghost takes over)
  const pending = new Map(), pendingAt = new Map(), sentAt = new Map();
  let modSeq = 0;
  function preview(i, k) { pending.set(i, k); pendingAt.set(i, ++modSeq); }
  function settle(id) {
    const upto = sentAt.get(id); if (upto == null) return;
    sentAt.delete(id);
    for (const [i, m] of [...pendingAt]) if (m <= upto && !(drag && drag.i === i)) { pending.delete(i); pendingAt.delete(i); }
  }
  const coarse = window.matchMedia ? matchMedia("(pointer: coarse)") : { matches: false };
  let keyMode = false, kfirst = null, kfId = null;
  let V = null, layer = null, dyn = null, prev = null, handles = [], ringPick = null, tubeMeshes = [];
  const showHelp = (() => { try { return localStorage.getItem("pc.trajHelp") !== "0"; } catch (e) { return true; } })();
  let helpOn = showHelp;

  const pts = () => arr(g && g.keys);
  const keyPos = (i) => (pending.has(i) ? pending.get(i) : pts()[i]);
  const point = (i) => (tr && tr.points && tr.points[i]) || {};
  const bad = (i) => !!(tr && tr.check && arr(tr.check.ok)[i] === false);
  const surfaceZ = () => (tj.surface != null ? tj.surface : (V && V.TABLE ? V.TABLE.z : 0.74));
  function under(x, y) {          // the surface right below a point: an object's top or the table
    let z = surfaceZ(), name = null;
    for (const o of arr(tj.objects)) {
      if (Math.hypot(x - o.pos[0], y - o.pos[1]) < o.r && o.top > z) { z = o.top; name = o.name; }
    }
    return { z, name };
  }
  const send = (msg) => { const ok = P.send(msg); if (!ok) P.toast("Not connected to the robot."); return ok; };
  function edit(fields) {
    const id = `ui-tj-${++seq}`; sentAt.set(id, modSeq);
    return send(Object.assign({ type: "traj", op: "edit", world: true, id }, fields));
  }

  // ------------------------------------------------------------------ 3D: static ghost (from the server)
  const COL = { ok: 0x5c77df, bad: 0xd6453a, sel: 0xf59145, white: 0xffffff, ink: 0x1d1d1f, approach: 0x8e93a6 };
  let ghostKey = "";
  function gripperMesh(T, opacity, color) {
    // a parallel-jaw gripper drawn in the end effector's site frame: fingers reach toward +axis, the TCP is the origin
    const outer = new T.Group(), inner = new T.Group();
    const ax = g && Array.isArray(g.axis) ? new T.Vector3(...g.axis).normalize() : new T.Vector3(0, 0, 1);
    inner.quaternion.setFromUnitVectors(new T.Vector3(0, 0, 1), ax);
    const mat = new T.MeshStandardMaterial({ color, transparent: true, opacity, depthWrite: false, roughness: 0.45, metalness: 0.05 });
    const edge = new T.LineBasicMaterial({ color: COL.ok, transparent: true, opacity: Math.min(0.9, opacity + 0.35) });
    const box = (sx, sy, sz, x, y, z) => {
      const geo = new T.BoxGeometry(sx, sy, sz), m = new T.Mesh(geo, mat); m.position.set(x, y, z);
      const e = new T.LineSegments(new T.EdgesGeometry(geo), edge); e.position.copy(m.position);
      inner.add(m, e);
    };
    box(0.064, 0.022, 0.016, 0, 0, -0.054);              // palm
    box(0.009, 0.018, 0.046, 0.024, 0, -0.023);          // fingers
    box(0.009, 0.018, 0.046, -0.024, 0, -0.023);
    box(0.016, 0.016, 0.05, 0, 0, -0.087);               // wrist
    const tip = new T.Mesh(new T.SphereGeometry(0.005, 12, 8), new T.MeshBasicMaterial({ color: COL.ok, transparent: true, opacity: Math.min(1, opacity + 0.4) }));
    inner.add(tip);
    outer.add(inner);
    return outer;
  }
  const quat = (T, q) => (Array.isArray(q) && q.length === 4 ? new T.Quaternion(q[1], q[2], q[3], q[0]) : new T.Quaternion());

  function buildStatic() {
    if (!V || !layer) return;
    const key = g ? JSON.stringify([g.id, g.pts, g.ok, g.kq, g.approach_ok, tj.running, which]) : "";
    if (key === ghostKey) return;
    ghostKey = key; dynKey = "";
    if (dyn) { clearDyn(); layer.remove(dyn); }         // labels share cached materials: never dispose them
    V.disposeGroup(layer); handles = []; tubeMeshes = []; prev = null;
    layer.add(dyn = new V.THREE.Group());
    if (!g) { V.redraw(); return; }
    const T = V.THREE, fade = tj.running ? 0.35 : 1;
    const ps = arr(g.pts).map((q) => new T.Vector3(q[0], q[1], q[2])), ok = arr(g.ok);
    const tube = new T.Group(); tube.name = "tube";
    for (let i = 0; i + 1 < ps.length; i++) {
      const good = ok[i] !== false && ok[i + 1] !== false;
      const m = V.segmentMesh(ps[i], ps[i + 1], good ? 0.0042 : 0.0055, good ? COL.ok : COL.bad, (good ? 0.72 : 0.9) * fade);
      m.userData.seg = i; tube.add(m); tubeMeshes.push(m);
    }
    if (ps.length) {   // round joints so the tube reads as one smooth line
      const jg = new T.SphereGeometry(0.0042, 10, 8), jm = new T.MeshBasicMaterial({ color: COL.ok, transparent: true, opacity: 0.72 * fade });
      const inst = new T.InstancedMesh(jg, jm, ps.length), mx = new T.Matrix4();
      ps.forEach((v, i) => { mx.makeTranslation(v.x, v.y, v.z); inst.setMatrixAt(i, mx); if (ok[i] === false) inst.setColorAt(i, new T.Color(COL.bad)); });
      tube.add(inst);
    }
    layer.add(tube);
    const a = arr(g.approach);
    if (a.length === 2 && !tj.running) {    // dashed: how the hand gets to the start
      const geo = new T.BufferGeometry().setFromPoints(a.map((q) => new T.Vector3(q[0], q[1], q[2])));
      const line = new T.Line(geo, new T.LineDashedMaterial({ color: g.approach_ok === false ? COL.bad : COL.approach, dashSize: 0.012, gapSize: 0.009, transparent: true, opacity: 0.85 }));
      line.computeLineDistances(); layer.add(line);
    }
    // ghost grippers at the points (the wrist as it will be there)
    if (!tj.running) {
      const keys = pts(), every = keys.length > 12 ? Math.ceil(keys.length / 8) : 1;
      keys.forEach((k, i) => {
        if (i % every && i !== keys.length - 1) return;
        const gm = gripperMesh(T, 0.16, COL.white); gm.position.set(k[0], k[1], k[2]); gm.quaternion.copy(quat(T, arr(g.kq)[i])); gm.userData.ghostOf = i;
        layer.add(gm);
      });
    }
    // handles
    const small = pts().length > 14;
    pts().forEach((k, i) => {
      const geo = new T.SphereGeometry(small ? 0.009 : 0.013, 20, 14);
      const m = new T.Mesh(geo, new T.MeshStandardMaterial({ color: COL.white, emissive: COL.ok, emissiveIntensity: 0.25, roughness: 0.35 }));
      m.position.set(k[0], k[1], k[2]); m.userData.handle = i; m.renderOrder = 6;
      m.visible = !tj.running; layer.add(m); handles.push(m);
    });
    prev = gripperMesh(T, 0.42, 0xdfe5ff); prev.visible = false; layer.add(prev);   // rides along the path (preview)
    paintDyn();
  }

  // ------------------------------------------------------------------ 3D: selection, drag, guides, ring (cheap, often)
  let labelCache = new Map(), dynKey = "";
  function clearDyn() {
    for (const c of [...dyn.children]) {
      dyn.remove(c);
      if (c.isSprite) continue;
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
  }
  function label(text, bg) {
    const k = text + "|" + bg;
    if (!labelCache.has(k)) { if (labelCache.size > 80) labelCache = new Map(); labelCache.set(k, V.labelSprite(text, bg, "#fff", 0.03)); }
    return labelCache.get(k).clone();
  }
  function paintDyn() {
    if (!V || !dyn) return;
    const T = V.THREE;
    const dk = JSON.stringify([ghostKey.length, g && g.id, sel, hover, drag && [drag.i, drag.mode, drag.yaw], [...pending.entries()], kfirst,
      sel >= 0 ? point(sel).yaw : null, tj.running]);
    if (dk === dynKey) return;
    dynKey = dk;
    clearDyn();
    ringPick = null;
    if (!g || tj.running) { V.redraw(); return; }
    const keys = pts(), small = keys.length > 14;
    handles.forEach((m, i) => {
      const k = keyPos(i); if (!k) return;
      m.position.set(k[0], k[1], k[2]);
      const isSel = i === sel, isHover = i === hover && !isSel;
      m.material.color.setHex(isSel ? COL.sel : bad(i) ? COL.bad : COL.white);
      m.material.emissive.setHex(isSel ? COL.sel : bad(i) ? COL.bad : COL.ok);
      m.material.emissiveIntensity = isSel ? 0.5 : bad(i) ? 0.45 : isHover ? 0.6 : 0.25;
      m.scale.setScalar(isHover ? 1.3 : isSel ? 1.15 : 1);
      if (!small || isSel || isHover || i === 0 || i === keys.length - 1) {
        const lab = label(String(i + 1), isSel ? "#f59145" : bad(i) ? "#d6453a" : "#5c77df");
        lab.position.set(k[0], k[1], k[2] + 0.034); dyn.add(lab);
      }
    });
    // local preview while a handle moves: straight lines between the handles (the server smooths and checks on release)
    if (pending.size) {
      const line = new T.Line(new T.BufferGeometry().setFromPoints(keys.map((_, i) => new T.Vector3(...keyPos(i)))),
        new T.LineBasicMaterial({ color: COL.sel, transparent: true, opacity: 0.9 }));
      dyn.add(line);
      tubeMeshes.forEach((m) => { m.material.opacity = 0.25; });
    } else tubeMeshes.forEach((m) => { m.material.opacity = m.material.color.getHex() === COL.bad ? 0.9 : 0.72; });
    // height guide + label for the point in focus
    const f = drag ? drag.i : sel >= 0 ? sel : hover;
    if (f >= 0 && keyPos(f)) {
      const k = keyPos(f), u = under(k[0], k[1]);
      const top = new T.Vector3(k[0], k[1], k[2]), base = new T.Vector3(k[0], k[1], u.z + 0.001);
      if (top.z - base.z > 0.004) dyn.add(V.segmentMesh(base, top, 0.0016, drag && drag.mode === "height" ? COL.sel : COL.ink, 0.55));
      const disc = new T.Mesh(new T.RingGeometry(0.009, 0.014, 28), new T.MeshBasicMaterial({ color: COL.ink, transparent: true, opacity: 0.35, depthWrite: false, side: T.DoubleSide }));
      disc.position.copy(base); dyn.add(disc);
      const cm = (k[2] - u.z) * 100;
      const lab = label(`${cm.toFixed(cm < 10 ? 1 : 0)} cm${u.name ? " over the " + u.name.replace(/_/g, " ") : ""}`, "rgba(29,29,31,.82)");
      lab.position.set(k[0], k[1], (top.z + base.z) / 2); lab.center.set(-0.08, 0.5); dyn.add(lab);
    }
    // wrist ring around the selected point: drag the knob to turn the gripper about the vertical
    if (sel >= 0 && keyPos(sel)) {
      kick();
      const k = keyPos(sel), R = 0.055;
      const ring = new T.Mesh(new T.TorusGeometry(R, 0.0022, 8, 64), new T.MeshBasicMaterial({ color: drag && drag.mode === "yaw" ? COL.sel : COL.ok, transparent: true, opacity: 0.8 }));
      ring.position.set(k[0], k[1], k[2]); dyn.add(ring);
      const yaw = (drag && drag.mode === "yaw" ? drag.yaw : point(sel).yaw || 0) * Math.PI / 180;
      const knob = new T.Mesh(new T.SphereGeometry(0.009, 16, 12), new T.MeshStandardMaterial({ color: COL.sel, emissive: COL.sel, emissiveIntensity: 0.4 }));
      knob.position.set(k[0] + R * Math.cos(yaw), k[1] + R * Math.sin(yaw), k[2]); dyn.add(knob);
      dyn.add(V.segmentMesh(new T.Vector3(k[0], k[1], k[2]), knob.position.clone(), 0.0012, COL.sel, 0.8));
      ringPick = new T.Mesh(new T.TorusGeometry(R, 0.014, 6, 48), new T.MeshBasicMaterial({ visible: false }));
      ringPick.position.copy(ring.position); dyn.add(ringPick);
      if (drag && drag.mode === "yaw") {
        const lab = label(`wrist ${Math.round(drag.yaw)}°`, "#f59145"); lab.position.set(k[0] + R * Math.cos(yaw), k[1] + R * Math.sin(yaw), k[2] + 0.03); dyn.add(lab);
      }
    }
    // the first keyframe, before there are two to connect
    if (kfirst) {
      const gm = gripperMesh(T, 0.35, 0xdfe5ff); gm.position.set(...kfirst); gm.quaternion.set(1, 0, 0, 0); dyn.add(gm);
      const lab = label("1", "#5c77df"); lab.position.set(kfirst[0], kfirst[1], kfirst[2] + 0.04); dyn.add(lab);
    }
    V.redraw();
  }

  // ------------------------------------------------------------------ preview: a ghost gripper rides along at the planned speed
  let animT0 = performance.now(), animOn = false, lastFrame = 0, animUntil = 0;
  function animate(now) {
    animOn = false;
    placePop();
    const live = which === "proposed" || now < animUntil;
    if (!V || !prev || !g || tj.running || drag || pending.size || document.hidden || !V.isActive || !live) {
      if (prev && prev.visible) { prev.visible = false; V.redraw(); }
      if (!pop.hidden) { animOn = true; requestAnimationFrame(animate); }     // keep the popover on its point while orbiting
      return;
    }
    animOn = true; requestAnimationFrame(animate);
    if (now - lastFrame < 33) return;
    lastFrame = now;
    const ts = arr(g.t), ps = arr(g.pts), qs = arr(g.q); if (ps.length < 2) { prev.visible = false; return; }
    const T = V.THREE, dur = ts[ts.length - 1] || 1, cyc = dur + 1.2;
    const t = Math.min(dur, ((now - animT0) / 1000) % cyc);
    let k = 0; while (k + 1 < ts.length - 1 && ts[k + 1] <= t) k++;
    const span = (ts[k + 1] - ts[k]) || 1, u = Math.max(0, Math.min(1, (t - ts[k]) / span));
    const a = ps[k], b = ps[Math.min(k + 1, ps.length - 1)];
    prev.position.set(a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u);
    if (qs.length) prev.quaternion.copy(quat(T, qs[k])).slerp(quat(T, qs[Math.min(k + 1, qs.length - 1)]), u);
    else prev.quaternion.copy(quat(T, arr(g.kq)[0]));      // the wrist does not turn along this path
    prev.visible = true;
    V.redraw();
  }
  function kick() { if (!animOn) { animOn = true; requestAnimationFrame(animate); } }

  // ------------------------------------------------------------------ pointer (PC3D.capture)
  function ray(ev) {
    const cv = V.scene && document.querySelector("#view3d canvas") || ev.target, r = cv.getBoundingClientRect(), T = V.THREE;
    const rc = new T.Raycaster(); rc.setFromCamera(new T.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1), V.camera);
    return rc;
  }
  function pickHandle(ev, tol) {
    if (!g || tj.running) return -1;
    const hit = ray(ev).intersectObjects(handles, false)[0];
    if (hit) return hit.object.userData.handle;
    let best = -1, bd = tol || (ev.pointerType && ev.pointerType !== "mouse" ? 22 : 12);   // a finger may land near a handle
    pts().forEach((_, i) => {
      const k = keyPos(i), s = V.project(k[0], k[1], k[2]), d = Math.hypot(s[0] - ev.clientX, s[1] - ev.clientY);
      if (d < bd) { bd = d; best = i; }
    });
    return best;
  }
  const onRing = (ev) => !!(ringPick && ray(ev).intersectObject(ringPick, false)[0]);
  function planeHit(ev, plane) { const p = new V.THREE.Vector3(); return ray(ev).ray.intersectPlane(plane, p) ? p : null; }
  function clampXY(x, y) {
    const T = V.TABLE || { x0: 0.22, x1: 0.82, y0: -0.5, y1: 0.5 };
    return [Math.min(T.x1 + 0.15, Math.max(T.x0 - 0.3, x)), Math.min(T.y1 + 0.15, Math.max(T.y0 - 0.15, y))];
  }
  function setCursor(c) { const cv = document.querySelector("#view3d canvas"); if (cv) cv.style.cursor = c || ""; }

  function onPointer({ type, ev, hit }) {
    if (!V) return false;
    const T = V.THREE;
    if (type === "down") {
      press = { x: ev.clientX, y: ev.clientY, t: performance.now() };
      if (ev.button !== 0 && ev.pointerType === "mouse") return false;
      if (sel >= 0 && onRing(ev)) {
        const k = keyPos(sel);
        drag = { i: sel, mode: "yaw", plane: new T.Plane(new T.Vector3(0, 0, 1), -k[2]), yaw: point(sel).yaw || 0, moved: false };
        setCursor("grabbing"); paintDyn(); return true;
      }
      const i = pickHandle(ev);
      if (i < 0) return false;
      const k = keyPos(i);
      sel = i; hover = -1;
      if (ev.altKey) {
        const n = new T.Vector3().subVectors(V.camera.position, new T.Vector3(...k)); n.z = 0;
        if (n.lengthSq() < 1e-9) n.set(1, 0, 0);
        const plane = new T.Plane().setFromNormalAndCoplanarPoint(n.normalize(), new T.Vector3(...k)), h = planeHit(ev, plane);
        drag = { i, mode: "height", plane, dz: h ? k[2] - h.z : 0, moved: false };
        setCursor("ns-resize");
      } else {
        const plane = new T.Plane(new T.Vector3(0, 0, 1), -k[2]), h = planeHit(ev, plane);
        drag = { i, mode: "plane", plane, off: h ? [k[0] - h.x, k[1] - h.y] : [0, 0], moved: false };
        setCursor("grabbing");
      }
      render(); paintDyn(); return true;
    }
    if (type === "move") {
      if (!drag) {
        if (!g || tj.running) { setCursor(keyMode && hit ? "copy" : ""); return false; }
        const i = pickHandle(ev), r = i < 0 && sel >= 0 && onRing(ev);
        if (i !== hover) { hover = i; paintDyn(); }
        setCursor(i >= 0 ? (ev.altKey ? "ns-resize" : "grab") : r ? "ew-resize" : keyMode && hit ? "copy" : "");
        return false;
      }
      const h = planeHit(ev, drag.plane); if (!h) return true;
      const k = keyPos(drag.i).slice();
      if (drag.mode === "plane") { const xy = clampXY(h.x + drag.off[0], h.y + drag.off[1]); k[0] = r3(xy[0]); k[1] = r3(xy[1]); }
      else if (drag.mode === "height") { k[2] = r3(Math.min(surfaceZ() + 0.8, Math.max(surfaceZ() - 0.03, h.z + drag.dz))); }
      else {
        const c = keyPos(drag.i); let deg = Math.atan2(h.y - c[1], h.x - c[0]) * 180 / Math.PI;
        deg = Math.round(deg / (ev.shiftKey ? 15 : 5)) * (ev.shiftKey ? 15 : 5);
        if (deg !== drag.yaw) { drag.yaw = deg; drag.moved = true; paintDyn(); renderPop(); }
        return true;
      }
      drag.moved = true; preview(drag.i, k); paintDyn(); renderPop();
      return true;
    }
    if (type === "up") {
      const tap = press && Math.hypot(ev.clientX - press.x, ev.clientY - press.y) < 6; press = null;
      if (drag) {
        const d = drag; drag = null; setCursor("grab");
        if (d.mode === "yaw") { if (d.moved) edit({ index: d.i, point: { yaw: d.yaw } }); }
        else if (d.moved && pending.has(d.i)) edit({ index: d.i, point: { p: pending.get(d.i) } });
        render(); paintDyn(); renderPop(); kick();
        return true;
      }
      if (tap && keyMode && hit) { dropKeyframe(hit); return true; }
      if (tap && sel >= 0) { sel = -1; render(); paintDyn(); renderPop(); }
      return false;
    }
    return false;
  }

  // wheel with the pointer on a handle: height, 5 mm a notch (Shift: 2 cm). Anywhere else the wheel zooms the view.
  // The handle is picked fresh under the pointer (never a stale hover: after a fly-to the path sits where the pointer
  // rests, and a remembered hover used to eat every wheel turn there).
  let wheelTimer = 0, wheelAcc = 0;
  function onWheel({ ev }) {
    if (!g || tj.running || !V || ev.ctrlKey) return false;        // ctrl+wheel = trackpad pinch: always zoom
    const i = pickHandle(ev, 10);
    if (i < 0) return false;
    if (hover !== i) { hover = i; paintDyn(); }
    wheelAcc += ev.deltaMode === 1 ? ev.deltaY * 18 : ev.deltaY;   // trackpads send many small deltas: one step per ~50 px
    if (Math.abs(wheelAcc) < 50 && ev.deltaMode === 0 && Math.abs(ev.deltaY) < 50) return true;
    const dir = Math.sign(wheelAcc); wheelAcc = 0;
    const k = keyPos(i).slice(), step = ev.shiftKey ? 0.02 : 0.005;
    k[2] = r3(Math.min(surfaceZ() + 0.8, Math.max(surfaceZ() - 0.03, k[2] - dir * step)));
    preview(i, k); sel = i; paintDyn(); renderPop(); render();
    clearTimeout(wheelTimer); wheelTimer = setTimeout(() => { if (pending.has(i)) edit({ index: i, point: { p: pending.get(i) } }); }, 380);
    return true;
  }

  // ------------------------------------------------------------------ keyframes (the Path tool)
  function dropKeyframe(hit) {
    let z = surfaceZ() + 0.10;
    for (const o of arr(tj.objects)) if (Math.hypot(hit.x - o.pos[0], hit.y - o.pos[1]) < o.r + 0.01) z = Math.max(z, o.top + 0.07);
    const p = [r3(hit.x), r3(hit.y), r3(z)];
    const mine = tr && tr.source === "keyframes" && kfId && tr.id === kfId;
    if (mine) {
      if (pts().length >= MAX_KEYS) { P.toast(`That's ${MAX_KEYS} keyframes. Drag them to adjust, or add a point from the card.`); return; }
      edit({ insert: { after: pts().length - 1, p } });
      if (pts().length + 1 >= MAX_KEYS) P.toast("Last keyframe. Drag any of them to adjust; Run when it's blue.");
      return;
    }
    if (!kfirst) { kfirst = p; paintDyn(); P.toast("Keyframe 1 placed. Click a second spot to connect them."); return; }
    const id = `ui-tj-${++seq}`;
    kfId = "pending";
    send({ type: "traj", op: "propose", id, traj: { source: "keyframes", label: "your keyframes", points: [{ p: kfirst }, { p }] } });
    kfirst = null; paintDyn();
  }
  function setKeyMode(on) {
    keyMode = !!on;
    if (keyMode) {
      if (window.PC && window.PC.setStageView) window.PC.setStageView("3d");
      P.with3D((v) => { if (v.editing && v.editPath) v.editPath(false); });
      P.toast(`Click ${tr && tr.source === "keyframes" ? "more spots to add keyframes" : "2 to " + MAX_KEYS + " spots on the table: ghost grippers connect into a path"}. Drag them to adjust.`);
    } else if (kfirst) { kfirst = null; paintDyn(); }
    render();
  }
  P.tool({ id: "path", label: "Path", icon: "path", title: "Make a path in 3D: drop 2–5 ghost grippers, then drag them",
    onSelect: () => setKeyMode(true), onDeselect: () => setKeyMode(false) });

  // ------------------------------------------------------------------ keyboard (a point selected)
  document.addEventListener("keydown", (ev) => {
    if (sel < 0 || !g || tj.running) return;
    const tg = ev.target;
    if (tg && (tg.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(tg.tagName))) return;
    if (ev.metaKey || ev.ctrlKey) return;
    const dlg = document.getElementById("settings"); if (dlg && dlg.open) return;
    const k = keyPos(sel).slice(), step = ev.shiftKey ? 0.05 : 0.01;
    let moved = false;
    const camDir = () => {   // arrow keys follow the view: up = away from the camera
      if (!V || !V.camera) return [1, 0];
      const d = new V.THREE.Vector3(); V.camera.getWorldDirection(d); d.z = 0;
      if (d.lengthSq() < 1e-6) return [1, 0];
      d.normalize(); return [d.x, d.y];
    };
    const [fx, fy] = camDir(), rx = fy, ry = -fx;
    switch (ev.key) {
      case "ArrowUp": k[0] += fx * step; k[1] += fy * step; moved = true; break;
      case "ArrowDown": k[0] -= fx * step; k[1] -= fy * step; moved = true; break;
      case "ArrowRight": k[0] += rx * step; k[1] += ry * step; moved = true; break;
      case "ArrowLeft": k[0] -= rx * step; k[1] -= ry * step; moved = true; break;
      case "PageUp": k[2] += step; moved = true; break;
      case "PageDown": k[2] -= step; moved = true; break;
      case "[": case "]": {
        const y = (point(sel).yaw || 0) + (ev.key === "]" ? 15 : -15);
        edit({ index: sel, point: { yaw: ((y + 540) % 360) - 180 } }); break;
      }
      case "Delete": case "Backspace": removePoint(sel); break;
      case "Escape": sel = -1; render(); paintDyn(); renderPop(); break;
      default: return;
    }
    ev.preventDefault(); ev.stopImmediatePropagation();
    if (moved) {
      const i = sel;
      preview(i, k.map(r3)); paintDyn(); renderPop();
      clearTimeout(wheelTimer); wheelTimer = setTimeout(() => { if (pending.has(i)) edit({ index: i, point: { p: pending.get(i) } }); }, 380);
    }
  }, true);

  function removePoint(i) {
    if (pts().length <= 1) { P.toast("A path needs at least one point. Clear it instead."); return; }
    edit({ remove: i }); sel = Math.min(i, pts().length - 2); render(); renderPop();
  }

  // ------------------------------------------------------------------ popover (the selected point)
  function popPos() {
    if (!V || sel < 0 || !keyPos(sel)) return null;
    const host = P.slot("stage").getBoundingClientRect(), k = keyPos(sel), s = V.project(k[0], k[1], k[2]);
    const w = pop.offsetWidth || 316, h = pop.offsetHeight || 300, hx = s[0] - host.left, hy = s[1] - host.top;
    const avoid = ["#pip", ".stage-tl", ".v3d-views", "#taskHud"].map((q) => document.querySelector(q))
      .filter((e) => e && e.offsetParent !== null && !e.hidden).map((e) => e.getBoundingClientRect())
      .map((r) => ({ l: r.left - host.left - 6, t: r.top - host.top - 6, r: r.right - host.left + 6, b: r.bottom - host.top + 6 }));
    const area = (x, y, a) => Math.max(0, Math.min(x + w, a.r) - Math.max(x, a.l)) * Math.max(0, Math.min(y + h, a.b) - Math.max(y, a.t));
    const cost = (x, y) => avoid.reduce((c, a) => c + area(x, y, a), 0)
      + (x < hx + 14 && x + w > hx - 14 && y < hy + 14 && y + h > hy - 14 ? 1e5 : 0);       // never on top of its own point
    const fit = (x, y) => [Math.max(10, Math.min(host.width - w - 10, x)), Math.max(10, Math.min(host.height - h - 10, y))];
    const tries = [fit(hx + 28, hy - h / 2), fit(hx - w - 28, hy - h / 2), fit(hx + 28, hy + 24), fit(hx - w - 28, hy + 24),
      fit(hx - w / 2, hy + 30), fit(hx - w / 2, hy - h - 30), fit(10, host.height - h - 54), fit(host.width - w - 10, host.height - h - 54)];
    let best = tries[0], bc = Infinity;
    for (const t of tries) { const c = cost(t[0], t[1]); if (c < bc - 1) { best = t; bc = c; } }
    return best;
  }
  let popKey = "";
  function renderPop() {
    const show = sel >= 0 && !!g && !tj.running && !!keyPos(sel) && !(drag && drag.mode !== "yaw");
    pop.hidden = !show;
    if (!show) { popKey = ""; return; }
    const p = point(sel), k = keyPos(sel), u = under(k[0], k[1]), n = pts().length;
    const yaw = drag && drag.mode === "yaw" ? drag.yaw : p.yaw || 0;
    const key = JSON.stringify([sel, n, p, k, yaw, bad(sel), tr && tr.check && tr.check.why]);
    if (key !== popKey) {
      popKey = key;
      const chip = (act, val, text, on, aria) => `<button type="button" class="tj-chip" data-a="${act}" data-v="${esc(val)}" aria-pressed="${!!on}"${aria ? ` aria-label="${esc(aria)}"` : ""}>${text}</button>`;
      const sp = p.speed == null ? null : p.speed;
      const issue = arr(tj.issues).find((x) => x.span ? sel >= x.span[0] && sel <= x.span[1] : x.i === sel);
      const seg = (inner) => `<span class="tj-seg">${inner}</span>`;
      const ic = (html) => html.replace('class="tj-chip"', 'class="tj-chip icon"');
      pop.innerHTML = `
        <header><b>Point ${sel + 1} of ${n}</b>
          <button type="button" class="tj-x" data-a="prev" aria-label="Previous point">${svg("up", 16)}</button>
          <button type="button" class="tj-x" data-a="next" aria-label="Next point">${svg("down", 16)}</button>
          <button type="button" class="tj-x" data-a="close" aria-label="Close">${svg("x", 16)}</button></header>
        ${bad(sel) && issue ? `<p class="tj-bad">${esc(issue.why[0].toUpperCase() + issue.why.slice(1))}.</p>` : ""}
        <div class="tj-line"><span>Height</span><div class="tj-chips">${seg(ic(chip("dz", -0.01, svg("minus", 14), false, "Lower 1 cm"))
          + `<span class="tj-val" aria-live="polite">${((k[2] - u.z) * 100).toFixed(1)} cm</span>` + ic(chip("dz", 0.01, svg("plus", 14), false, "Raise 1 cm")))}</div></div>
        <div class="tj-line"><span>Speed</span><div class="tj-chips">${seg(SPEEDS.map(([t, v]) => chip("speed", v == null ? "" : v, t, v == null ? sp == null : sp != null && Math.abs(sp - v) < 1e-6)).join(""))}
          ${sp != null && !SPEEDS.some(([, v]) => v != null && Math.abs(sp - v) < 1e-6) ? `<span class="tj-val">${Math.round(sp * 100)} cm/s</span>` : ""}</div></div>
        <div class="tj-line"><span>Gripper</span><div class="tj-chips">${seg(chip("grip", "", "Keep", p.grip == null) + chip("grip", 0, "Open", p.grip === 0) + chip("grip", 1, "Close", p.grip === 1))}</div></div>
        <div class="tj-line"><span>Press</span><div class="tj-chips">${seg(FORCES.map((f) => chip("force", f === 0 ? "" : f, f === 0 ? "No" : f + " N", f === 0 ? !p.force : p.force === f)).join(""))}</div></div>
        <div class="tj-line"><span>Wrist</span><div class="tj-chips">${seg(ic(chip("yaw", -15, svg("rot", 14), false, "Turn the wrist clockwise 15 degrees"))
          + `<span class="tj-val" title="Turn about the vertical">${Math.round(yaw)}°</span>`
          + ic(chip("yaw", 15, svg("rot", 14).replace("<svg", '<svg style="transform:scaleX(-1)"'), false, "Turn the wrist counter-clockwise 15 degrees")))}
          ${seg(ic(chip("tilt", -10, svg("minus", 14), false, "Lean the tool back 10 degrees"))
          + `<span class="tj-val" title="Tilt: lean toward the direction of travel" style="min-width:38px">${Math.round(p.tilt || 0)}°</span>`
          + ic(chip("tilt", 10, svg("plus", 14), false, "Lean the tool forward 10 degrees")))}</div></div>
        <div class="tj-line"><span>Precision</span><div class="tj-chips">${seg(TOLS.map(([v, t]) => chip("tol", v == null ? "" : v, t, v == null ? p.tol == null : p.tol === v)).join(""))}</div></div>
        <footer>${chip("insert", "", svg("plus", 14) + "Point after", false)}${chip("remove", "", svg("trash", 14) + "Remove", false).replace('class="tj-chip"', 'class="tj-chip danger"')}</footer>`;
    }
    placePop();
  }
  function placePop() {
    if (pop.hidden || phone.matches) return;
    const xy = popPos();
    if (xy && window.innerWidth > 760) { pop.style.left = xy[0] + "px"; pop.style.top = xy[1] + "px"; }
  }
  pop.addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-a]"); if (!b || sel < 0) return;
    const a = b.dataset.a, raw = b.dataset.v, v = raw === "" || raw == null ? null : Number(raw), p = point(sel);
    switch (a) {
      case "close": sel = -1; render(); paintDyn(); renderPop(); return;
      case "prev": sel = (sel - 1 + pts().length) % pts().length; render(); paintDyn(); renderPop(); return;
      case "next": sel = (sel + 1) % pts().length; render(); paintDyn(); renderPop(); return;
      case "dz": { const k = keyPos(sel).slice(); k[2] = r3(k[2] + v); preview(sel, k); paintDyn(); renderPop(); edit({ index: sel, point: { p: k } }); return; }
      case "speed": edit({ index: sel, point: { speed: v } }); return;
      case "grip": edit({ index: sel, point: { grip: v } }); return;
      case "force": edit({ index: sel, point: { force: v } }); return;
      case "tol": edit({ index: sel, point: { tol: v } }); return;
      case "yaw": { const y = (p.yaw || 0) + v; edit({ index: sel, point: { yaw: ((y + 540) % 360) - 180 } }); return; }
      case "tilt": edit({ index: sel, point: { tilt: Math.max(-60, Math.min(60, (p.tilt || 0) + v)) } }); return;
      case "insert": edit({ insert: { after: sel } }); sel = sel + 1; return;
      case "remove": removePoint(sel); return;
    }
  });

  // ------------------------------------------------------------------ card
  function phase() {
    if (!tr) return null;
    if (tj.running) return "running";
    if (tj.ok === false) return "blocked";
    return which === "proposed" ? "proposed" : "ready";
  }
  const PH = { proposed: "Waiting for go", ready: "Ready", blocked: "Needs a fix", running: "Moving" };
  let cardKey = "";
  function render() {
    const ph = phase();
    card.hidden = !tr && !keyMode;
    const key = JSON.stringify([ph, tr && tr.id, tr && tr.label, tr && tr.source, tr && tr.check, sel, keyMode, kfirst, helpOn, pts().length, tj.progress > 0]);
    if (key === cardKey) return;
    cardKey = key;
    if (!tr) {
      body.innerHTML = `<div class="tj-head"><span class="tj-ico">${svg("path", 18)}</span><div class="tj-title"><b>Make a path</b><span>Keyframes</span></div></div>
        <p class="tj-tip">${kfirst ? "Click a second spot: the two ghost grippers connect into a path." : `Click 2 to ${MAX_KEYS} spots on the table. A ghost gripper appears at each; they connect into a path I check for reach and collisions.`}
        Or just say it: <i>“go over the vase to the left of the bowl”</i>.</p>`;
      return;
    }
    const n = arr(tr.points).length, ch = tr.check || {}, dur = ch.duration != null ? `${Number(ch.duration).toFixed(1)} s` : "";
    const badN = arr(ch.ok).filter((x) => x === false).length;
    const sum = ph === "blocked" ? `<p class="tj-sum bad">${esc(tj.why || ch.why || "Part of the path is blocked.")}</p>`
      : `<p class="tj-sum">${n} point${n === 1 ? "" : "s"}${dur ? " · " + dur : ""}${ph === "running" ? " · moving" : " · every point checked"}</p>`;
    const strip = arr(tr.points).map((p, i) => `<button type="button" class="tj-pt" data-i="${i}" aria-pressed="${i === sel}" data-bad="${arr(ch.ok)[i] === false}"
      aria-label="Point ${i + 1}${arr(ch.ok)[i] === false ? ", blocked" : ""}${p.grip != null ? p.grip >= 0.5 ? ", closes the gripper" : ", opens the gripper" : ""}">${i + 1}${p.grip != null || p.force || p.yaw ? "<i></i>" : ""}</button>`).join("");
    const run = ph === "running" ? `<button type="button" class="tj-btn danger" data-c="stop">${svg("stop")}Stop</button>`
      : `<button type="button" class="tj-btn primary" data-c="run" ${ph === "blocked" ? "disabled" : ""}>${svg("play")}${ph === "proposed" ? "Go" : "Run"}</button>`;
    body.innerHTML = `
      <div class="tj-head"><span class="tj-ico">${svg("path", 18)}</span>
        <div class="tj-title"><b>${esc(tr.label || "Path")}</b><span>${esc(SRC[tr.source] || tr.source)}${badN ? ` · ${badN} to fix` : ""}</span></div>
        <span class="tj-state" data-s="${ph}">${PH[ph]}</span></div>
      ${sum}
      ${ph !== "running" ? `<div class="tj-strip" role="group" aria-label="Points (select one to edit it)">${strip}</div>` : ""}
      ${helpOn && ph !== "running" ? (coarse.matches ? `<p class="tj-tip">Drag a point on the table · tap a number to set its height, speed and wrist · or say “higher there”.</p>`
        : `<p class="tj-tip">Drag a point · <kbd>Alt</kbd>-drag or scroll: height · ring: wrist · or say “higher there”.</p>`) : ""}
      <div class="tj-row">${run}
        ${ph !== "running" ? `<button type="button" class="tj-btn icon" data-c="add" aria-label="Add a point at the end" title="Add a point">${svg("plus", 18)}</button>
        <button type="button" class="tj-btn icon" data-c="focus" aria-label="Frame the path in the 3D view" title="Frame the path">${svg("focus", 18)}</button>
        <button type="button" class="tj-btn icon" data-c="help" aria-pressed="${helpOn}" aria-label="Show or hide tips" title="Tips">${svg("help", 18)}</button>
        <button type="button" class="tj-btn" data-c="clear">Clear</button>` : ""}</div>`;
  }
  card.addEventListener("click", (ev) => {
    const pt = ev.target.closest("button[data-i]");
    if (pt) { const i = +pt.dataset.i; sel = sel === i ? -1 : i; render(); paintDyn(); renderPop(); if (sel >= 0) P.with3D(() => kick()); return; }
    const b = ev.target.closest("button[data-c]"); if (!b) return;
    switch (b.dataset.c) {
      case "run":
        if (which === "proposed" && tj.plan_id) send({ type: "plan", op: "confirm", id: tj.plan_id });
        else send({ type: "traj", op: "run", id: `ui-tj-${++seq}` });
        sel = -1; renderPop(); break;
      case "stop": if (window.PC && window.PC.runTask) window.PC.runTask("stop", {}, "button"); else send({ type: "task", id: `tj-stop-${Date.now()}`, action: "stop" }); break;
      case "add": edit({ insert: { after: pts().length - 1 } }); sel = pts().length; break;
      case "clear": send({ type: "traj", op: "clear", id: `ui-tj-${++seq}` }); sel = -1; kfId = null; renderPop(); break;
      case "focus": focusPath(); break;
      case "help": helpOn = !helpOn; try { localStorage.setItem("pc.trajHelp", helpOn ? "1" : "0"); } catch (e) { /* private mode */ } render(); break;
    }
  });

  // ------------------------------------------------------------------ 2D fallback: height profile for a stroke (robot's view)
  const bar = document.getElementById("pathBar");
  let prof = null, profTimer = 0;
  if (bar) {
    prof = document.createElement("span");
    prof.className = "tj-prof"; prof.hidden = true;
    prof.innerHTML = `<label>Lift <input type="range" min="-4" max="20" step="1" value="0" data-p="lift" aria-label="Lift the stroke, centimetres"></label><output data-o="lift">+0 cm</output>
      <label>Arch <input type="range" min="0" max="20" step="1" value="0" data-p="arch" aria-label="Arch the middle of the stroke, centimetres"></label><output data-o="arch">0 cm</output>`;
    bar.insertBefore(prof, bar.querySelector("#runPath"));
    ["pointerdown", "pointerup", "pointermove"].forEach((t) => prof.addEventListener(t, (e) => e.stopPropagation()));
    prof.addEventListener("input", (ev) => {
      const lift = +prof.querySelector('[data-p="lift"]').value, arch = +prof.querySelector('[data-p="arch"]').value;
      prof.querySelector('[data-o="lift"]').textContent = `${lift >= 0 ? "+" : ""}${lift} cm`;
      prof.querySelector('[data-o="arch"]').textContent = `${arch} cm`;
      prof.dataset.busy = "1";
      clearTimeout(profTimer);
      profTimer = setTimeout(() => { edit({ profile: { lift: lift / 100, arch: arch / 100 } }); prof.dataset.busy = ""; }, 260);
    });
  }
  function renderProf() {
    if (!prof) return;
    const on = !!(tr && tr.source === "draw" && which === "current" && !tj.running);
    prof.hidden = !on;
    if (!on || prof.dataset.busy) return;
    const pr = (tr.meta && tr.meta.profile) || { lift: 0, arch: 0 };
    const L = prof.querySelector('[data-p="lift"]'), A = prof.querySelector('[data-p="arch"]');
    if (document.activeElement !== L) { L.value = Math.round(pr.lift * 100); prof.querySelector('[data-o="lift"]').textContent = `${pr.lift >= 0 ? "+" : ""}${Math.round(pr.lift * 100)} cm`; }
    if (document.activeElement !== A) { A.value = Math.round(pr.arch * 100); prof.querySelector('[data-o="arch"]').textContent = `${Math.round(pr.arch * 100)} cm`; }
  }

  function focusPath(auto) {
    if (!g) return;
    if (window.PC && window.PC.setStageView && !(V && V.isActive)) window.PC.setStageView("3d");
    const pts3 = arr(g.pts).concat(arr(g.approach).slice(0, 1));
    P.with3D((v) => {
      if (auto && v.inView && v.inView(pts3)) return;            // already on screen at a workable size: keep the user's view
      if (v.focus) v.focus(pts3, { pad: 1.6, min: 0.35 });
    });
  }

  // ------------------------------------------------------------------ server state
  let lastReply = null, lastGhost = "", focused = "";
  P.onState((s) => {
    tj = (s && s.traj) || {};
    const nt = tj.proposed || tj.current || null;
    which = tj.proposed ? "proposed" : tj.current ? "current" : null;
    const changedTraj = (nt && nt.id) !== (tr && tr.id);
    tr = nt; g = tj.ghost || null;
    const gk = g ? `${g.id}|${g.duration}|${arr(g.pts).length}` : "";
    if (gk !== lastGhost) {           // a new or changed path: replay the preview twice from the start
      lastGhost = gk; animT0 = performance.now(); animUntil = animT0 + ((tj.ghost && tj.ghost.duration) || 3) * 2000 + 2600;
    }
    if (changedTraj && sel >= pts().length) sel = -1;
    if (g && tr && which === "proposed" && focused !== tr.id && ["language", "demo", "skill"].includes(tr.source) && V && V.isActive) {
      focused = tr.id; setTimeout(() => focusPath(true), 120);   // a path from words: bring it into view once, only if it isn't already
    }
    if (!tr) { sel = -1; hover = -1; }
    if (sel >= pts().length) sel = pts().length - 1;
    if (tj.reply && tj.reply.id) settle(tj.reply.id);
    if (changedTraj) { pending.clear(); pendingAt.clear(); }
    if (tr && tr.source === "keyframes" && kfId === "pending") kfId = tr.id;
    if (tr && tr.source !== "keyframes") kfId = null;
    const r = tj.reply;
    if (r && r.t !== (lastReply && lastReply.t) && String(r.id || "").startsWith("ui-tj-")) {
      if (!r.ok && r.message && !r.traj) P.toast(r.message);       // a failed check is already on the card and the plan
      if (r.op === "propose" && r.ok && keyMode) P.toast("Connected. Click more spots to add keyframes, or drag them.");
    }
    lastReply = r;
    render(); renderProf(); buildStatic(); paintDyn(); renderPop();
    if (g && !tj.running) kick();
  });
  P.with3D((v) => {
    V = v; layer = v.layer("traj"); v.capture(onPointer);
    if (v.wheelCapture) v.wheelCapture(onWheel);
    ghostKey = ""; buildStatic(); paintDyn(); kick();
  });
  window.addEventListener("pc3d-ready", () => { ghostKey = ""; });
  // P2b VR (web/xr.js): a hand grabbing a handle uses the same local preview as a mouse drag; the edit it sends settles it
  window.PCTraj = {
    preview(i, k) { if (!g || !pts()[i] || !Array.isArray(k)) return; preview(i, k.slice(0, 3).map(r3)); hover = i; paintDyn(); },
    track(id) { if (id) sentAt.set(id, modSeq); },
    cancel(i) { pending.delete(i); pendingAt.delete(i); if (hover === i) hover = -1; paintDyn(); },
    end(i) { if (hover === i) hover = -1; paintDyn(); },
    get keys() { return pts(); },
  };
  window.addEventListener("resize", () => renderPop());
  render();
})();
