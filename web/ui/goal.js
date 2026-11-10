// Goal (#E): "make it look like this". Show the end state instead of the steps.
//   Arrange: drag ghost copies of every movable object on the 3D table, then Make it so.
//   Photo:   take or choose a photo of the layout you want; the server reads it (Gemini Robotics-ER, or an offline
//            colour reader) and proposes the goal as ghosts you can still drag.
// The server (pps/features/goal.py) orders the moves, skips what is already in place and runs them one by one.
// Protocol: {type:"goal", op:"set"|"photo"|"run"|"cancel"|"clear"}; state.goal (docs/PROTOCOL.md, v4 "Goal").
"use strict";
(function () {
  const PCUI = window.PCUI;
  if (!PCUI) return;

  const CSS = `
.goal-card { width: 100%; box-sizing: border-box; background: var(--raised); color: var(--t1); border: 1px solid var(--line);
  border-radius: var(--r3); box-shadow: var(--shadow-md); padding: 16px; display: flex; flex-direction: column; gap: 12px;
  font: 13px/1.4 var(--sans); max-height: calc(100vh - 120px); overflow: auto; }
.goal-card[hidden] { display: none; }
.goal-head { display: flex; align-items: center; gap: 10px; }
.goal-head h3 { margin: 0; font-size: 15px; font-weight: 650; flex: 1; letter-spacing: -.01em; }
.goal-head .goal-ico { width: 32px; height: 32px; border-radius: 10px; display: grid; place-items: center; color: var(--accent-ink);
  background: var(--accent-tint); flex: none; }
.goal-btn { min-height: 44px; min-width: 44px; padding: 0 16px; border-radius: 999px; border: 1px solid var(--line2);
  background: var(--raised); color: var(--t1); font: 500 13.5px var(--sans); cursor: pointer; display: inline-flex;
  align-items: center; justify-content: center; gap: 8px; }
.goal-btn:hover:not(:disabled) { background: var(--active); }
.goal-btn:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.goal-btn:disabled { opacity: .45; cursor: default; }
.goal-btn.primary { background: var(--accent); border-color: transparent; color: var(--on-accent); font-weight: 600; }
.goal-btn.primary:hover:not(:disabled) { background: var(--inv-bg); color: var(--inv-fg); }
.goal-btn.danger { color: var(--bad-ink); border-color: color-mix(in srgb, var(--bad) 40%, transparent); }
.goal-btn.icon { padding: 0; border-color: transparent; background: transparent; }
.goal-seg { display: flex; gap: 4px; padding: 4px; border-radius: 999px; background: var(--field); }
.goal-seg button { flex: 1; min-height: 44px; border: 0; border-radius: 999px; background: transparent; color: var(--t2);
  font: 500 13px var(--sans); cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 6px; }
.goal-seg button[aria-pressed="true"] { background: var(--raised); color: var(--t1); box-shadow: var(--shadow); }
.goal-seg button:focus-visible { outline: 2px solid var(--accent); outline-offset: 1px; }
.goal-seg.small button { min-height: 44px; font-size: 12px; }
.goal-hint { margin: 0; color: var(--t2); }
.goal-objs { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; gap: 6px; }
.goal-objs li { display: flex; align-items: center; gap: 10px; min-height: 44px; padding: 0 4px 0 12px; border-radius: 14px;
  background: var(--field); }
.goal-objs li[data-sel="true"] { box-shadow: inset 0 0 0 2px var(--accent); }
.goal-objs .sw { width: 12px; height: 12px; border-radius: 4px; flex: none; background: var(--accent); }
.goal-objs .nm { flex: 1; font-weight: 550; text-transform: capitalize; }
.goal-objs .dt { color: var(--t2); font-family: var(--mono); font-size: 11.5px; }
.goal-file { position: absolute; width: 1px; height: 1px; opacity: 0; overflow: hidden; clip: rect(0 0 0 0); }
.goal-preview { position: relative; border-radius: 16px; overflow: hidden; background: var(--field); aspect-ratio: 4 / 3;
  display: grid; place-items: center; color: var(--t3); }
.goal-preview img { width: 100%; height: 100%; object-fit: contain; display: block; }
.goal-preview .dot { position: absolute; transform: translate(-50%, -50%); padding: 2px 8px; border-radius: 999px;
  background: var(--accent); color: var(--on-accent); font: 600 11px var(--sans); white-space: nowrap; box-shadow: var(--shadow); }
.goal-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
.goal-row .grow { flex: 1; }
.goal-label { color: var(--t2); font-size: 12px; }
.goal-status { padding: 10px 12px; border-radius: 14px; background: var(--field); color: var(--t1); min-height: 20px; }
.goal-status[data-s="failed"], .goal-status[data-s="stopped"] { background: var(--bad-tint); color: var(--bad-ink); }
.goal-status[data-s="done"] { background: var(--ok-tint); color: var(--ok-ink); }
.goal-status[data-s="running"], .goal-status[data-s="locating"] { background: var(--accent-tint); }
.goal-status:empty { display: none; }
.goal-steps { margin: 0; padding: 0; list-style: none; display: flex; flex-direction: column; gap: 4px; }
.goal-steps li { display: flex; align-items: center; gap: 10px; min-height: 36px; padding: 0 10px; border-radius: 12px; }
.goal-steps li .n { width: 24px; height: 24px; border-radius: 50%; display: grid; place-items: center; flex: none;
  font: 600 11px var(--mono); border: 1.5px solid var(--line2); color: var(--t2); }
.goal-steps li[data-s="running"] { background: var(--accent-tint); }
.goal-steps li[data-s="running"] .n { border-color: var(--accent); color: var(--accent-ink); }
.goal-steps li[data-s="done"] .n { border-color: var(--ok); background: var(--ok); color: var(--on-accent); }
.goal-steps li[data-s="failed"] .n { border-color: var(--bad); background: var(--bad); color: var(--on-accent); }
.goal-steps li .t { flex: 1; }
.goal-steps li .e { color: var(--t2); font: 11px var(--mono); }
.goal-foot { display: flex; gap: 8px; flex-wrap: wrap; }
.goal-foot .goal-btn { flex: 1; }
@media (max-width: 760px) { .goal-card { max-height: 60vh; } }`;
  const st = document.createElement("style"); st.id = "goal-css"; st.textContent = CSS; document.head.appendChild(st);

  const ICON = {
    upload: '<path d="M12 16V4m0 0-4.5 4.5M12 4l4.5 4.5"/><path d="M4 15v3.5A1.5 1.5 0 0 0 5.5 20h13a1.5 1.5 0 0 0 1.5-1.5V15"/>',
    cube: '<path d="m12 3 8 4.5v9L12 21l-8-4.5v-9z"/><path d="m4 7.5 8 4.5 8-4.5M12 12v9"/>',
    reset: '<path d="M4 12a8 8 0 1 0 2.6-5.9"/><path d="M4 4v4.5h4.5"/>',
  };
  const svg = (body, size = 18) => `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;
  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const arr = (x) => (Array.isArray(x) ? x : []);

  // ------------------------------------------------------------------ panel
  const card = document.createElement("section");
  card.className = "goal-card"; card.hidden = true; card.setAttribute("aria-label", "Goal: make it look like this");
  card.innerHTML = `
    <div class="goal-head"><span class="goal-ico">${PCUI.icon("goal", 18)}</span><h3>Make it look like this</h3>
      <button type="button" class="goal-btn icon" data-g="close" aria-label="Close goal panel" title="Close">${PCUI.icon("x", 18)}</button></div>
    <div class="goal-seg" role="group" aria-label="How to show the goal">
      <button type="button" data-mode="arrange" aria-pressed="true">${svg(ICON.cube, 16)}Arrange in 3D</button>
      <button type="button" data-mode="photo" aria-pressed="false">${PCUI.icon("show", 16)}From a photo</button></div>
    <div data-pane="arrange">
      <p class="goal-hint">Drag the ghosts on the table to where each object should end up. Tap a ghost, then tap a spot, works too.</p>
      <ul class="goal-objs" aria-label="Movable objects"></ul>
      <div class="goal-row"><span class="grow"></span>
        <button type="button" class="goal-btn" data-g="reset">${svg(ICON.reset, 16)}Reset ghosts</button></div>
    </div>
    <div data-pane="photo" hidden>
      <p class="goal-hint">A photo of the table the way you want it. I find each object and map it onto my table.</p>
      <input type="file" class="goal-file" accept="image/*" capture="environment" aria-label="Goal photo">
      <div class="goal-preview" aria-live="polite"><span>No photo yet</span></div>
      <div class="goal-row"><span class="goal-label">Photo taken from</span></div>
      <div class="goal-seg small" role="group" aria-label="Photo taken from">
        <button type="button" data-view="user" aria-pressed="true">Your side (robot at the top)</button>
        <button type="button" data-view="robot" aria-pressed="false">Robot's side</button></div>
      <div class="goal-row">
        <button type="button" class="goal-btn" data-g="pick">${svg(ICON.upload, 16)}Take or choose a photo</button>
        <button type="button" class="goal-btn" data-g="reread" hidden>Read again</button></div>
    </div>
    <div class="goal-status" role="status" aria-live="polite"></div>
    <ol class="goal-steps" aria-label="Planned moves"></ol>
    <div class="goal-foot">
      <button type="button" class="goal-btn" data-g="preview">Preview plan</button>
      <button type="button" class="goal-btn primary" data-g="make">${PCUI.icon("check", 16)}Make it so</button>
      <button type="button" class="goal-btn danger" data-g="cancel" hidden>${PCUI.icon("x", 16)}Cancel</button></div>`;
  PCUI.slot("side").appendChild(card);
  const $ = (sel) => card.querySelector(sel);
  const el = {
    objs: $(".goal-objs"), status: $(".goal-status"), steps: $(".goal-steps"), file: $(".goal-file"), preview: $(".goal-preview"),
    make: $('[data-g="make"]'), preview_btn: $('[data-g="preview"]'), cancel: $('[data-g="cancel"]'), reread: $('[data-g="reread"]'),
  };

  let open = false, mode = "arrange", photoView = "user", photo = null; // photo: {b64, url}
  let goal = {}, lastRev = -1, lastSaid = "", objects = [];
  const ghosts = new Map();      // name -> {x, y, touched}
  let selected = null;

  function setMode(m) {
    mode = m === "photo" ? "photo" : "arrange";
    card.querySelectorAll("[data-mode]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.mode === mode)));
    card.querySelectorAll("[data-pane]").forEach((p) => { p.hidden = p.dataset.pane !== mode; });
    if (mode === "arrange") show3D();
    render(); draw();
  }
  function show3D() {
    const P = window.PC3D;
    if (P && P.isActive) return;
    const b = document.querySelector('.seg-btn[data-view="3d"]');
    if (b) b.click();
  }
  function setOpen(on) {
    open = !!on; card.hidden = !open && !running();
    if (open) { setMode(mode); }
    draw();
  }
  const running = () => goal.status === "running" || goal.status === "locating";

  PCUI.tool({ id: "goal", label: "Goal", icon: "goal", title: "Show the end state: arrange ghosts in 3D or use a photo",
    onSelect: () => setOpen(true), onDeselect: () => setOpen(false) });

  card.addEventListener("click", (e) => {
    const b = e.target.closest("button"); if (!b || !card.contains(b)) return;
    if (b.dataset.mode) return setMode(b.dataset.mode);
    if (b.dataset.view) {
      photoView = b.dataset.view;
      card.querySelectorAll("[data-view]").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      if (photo) sendPhoto();
      return;
    }
    if (b.dataset.obj) { selected = selected === b.dataset.obj ? null : b.dataset.obj; render(); draw(); return; }
    switch (b.dataset.g) {
      case "close": PCUI.activeTool === "goal" ? PCUI.selectTool(null) : setOpen(false); break;
      case "reset": resetGhosts(); break;
      case "pick": el.file.click(); break;
      case "reread": sendPhoto(); break;
      case "preview": submit(false); break;
      case "make": submit(true); break;
      case "cancel": PCUI.send({ type: "goal", op: "cancel" }); break;
    }
  });

  // ------------------------------------------------------------------ sending
  function targetsFromGhosts() {
    const out = {};
    for (const o of objects) { const g = ghosts.get(o.name); if (g) out[o.name] = [round3(g.x), round3(g.y)]; }
    return out;
  }
  const round3 = (v) => Math.round(v * 1000) / 1000;
  // The ghosts always hold the goal (a photo's reading shows up as ghosts too), so both modes send them.
  function submit(run) {
    const ok = PCUI.send({ type: "goal", op: "set", targets: targetsFromGhosts(), run: !!run, source: mode === "photo" ? "photo" : "arrange" });
    if (!ok) { PCUI.toast("Not connected to the robot."); return; }
    if (run) PCUI.say("you", "Make it look like this.");
  }
  const photoRead = () => String(goal.source || "").startsWith("photo") && goal.targets && Object.keys(goal.targets).length > 0;

  el.file.addEventListener("change", async () => {
    const f = el.file.files && el.file.files[0]; el.file.value = "";
    if (!f) return;
    if (!/^image\//.test(f.type)) { PCUI.toast("Please choose an image."); return; }
    try {
      photo = await downscale(f);
      el.preview.innerHTML = `<img alt="Your goal photo" src="${photo.url}">`;
      el.reread.hidden = false;
      sendPhoto();
    } catch (e) { PCUI.toast("Couldn't read that image."); console.error(e); }
  });
  function sendPhoto() {
    if (!photo) return;
    if (!PCUI.send({ type: "goal", op: "photo", jpg_b64: photo.b64, view: photoView, key: (() => { try { return localStorage.getItem("pc.geminiKey") || undefined; } catch (e) { return undefined; } })() })) { PCUI.toast("Not connected to the robot."); return; }
    PCUI.say("you", "Make it look like this photo.");
    for (const g of ghosts.values()) g.touched = false;
  }
  // Downscale on a canvas so the websocket message stays well under 1.5 MB.
  async function downscale(file) {
    const url = URL.createObjectURL(file);
    try {
      const img = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = url; });
      let side = 1280, q = 0.85, data = "";
      for (let k = 0; k < 6; k++) {
        const s = Math.min(1, side / Math.max(img.naturalWidth, img.naturalHeight));
        const cv = document.createElement("canvas");
        cv.width = Math.max(1, Math.round(img.naturalWidth * s)); cv.height = Math.max(1, Math.round(img.naturalHeight * s));
        cv.getContext("2d").drawImage(img, 0, 0, cv.width, cv.height);
        data = cv.toDataURL("image/jpeg", q);
        if (data.length < 1_400_000) break;
        side = Math.round(side * 0.75); q = Math.max(0.6, q - 0.08);
      }
      return { url: data, b64: data.slice(data.indexOf(",") + 1) };
    } finally { URL.revokeObjectURL(url); }
  }

  // ------------------------------------------------------------------ ghosts
  function syncGhosts() {
    const names = new Set(objects.map((o) => o.name));
    for (const k of [...ghosts.keys()]) if (!names.has(k)) ghosts.delete(k);
    for (const o of objects) {
      const g = ghosts.get(o.name);
      if (!g) ghosts.set(o.name, { x: o.pos[0], y: o.pos[1], touched: false });
      else if (!g.touched) { g.x = o.pos[0]; g.y = o.pos[1]; }
    }
  }
  function resetGhosts() {
    for (const o of objects) ghosts.set(o.name, { x: o.pos[0], y: o.pos[1], touched: false });
    selected = null; render(); draw();
  }
  const objOf = (n) => objects.find((o) => o.name === n);
  const movedCm = (n) => { const o = objOf(n), g = ghosts.get(n); return o && g ? Math.hypot(g.x - o.pos[0], g.y - o.pos[1]) * 100 : 0; };

  function clampTable(x, y, r) {
    const T = (window.PC3D && window.PC3D.TABLE) || { x0: 0.22, x1: 0.82, y0: -0.5, y1: 0.5 }, m = 0.04 + r * 0.5;
    return [Math.min(T.x1 - m, Math.max(T.x0 + m, x)), Math.min(T.y1 - m, Math.max(T.y0 + m, y))];
  }
  function moveGhost(n, x, y) {
    const o = objOf(n), g = ghosts.get(n); if (!o || !g) return;
    [g.x, g.y] = clampTable(x, y, o.r || 0.04); g.touched = true;
    draw(); renderObjs();
  }

  // 3D drawing in PC3D.layer("goal")
  let P = null, layer = null, meshes = [], drawKey = "";
  const showGhosts = () => open && mode === "arrange" || running() || goal.status === "ready";
  function draw() {
    if (!P || !layer) return;
    const vis = showGhosts();
    const key = JSON.stringify([vis, selected, objects.map((o) => [o.name, o.pos.map((v) => Math.round(v * 500))]),
      [...ghosts.entries()].map(([k, g]) => [k, Math.round(g.x * 1000), Math.round(g.y * 1000)]), goal.status, goal.i]);
    if (key === drawKey) return;
    drawKey = key;
    P.disposeGroup(layer); meshes = [];
    if (!vis) { P.redraw(); return; }
    const THREE = P.THREE, z0 = (P.TABLE && P.TABLE.z) || 0.74;
    const css = getComputedStyle(document.documentElement);
    const accent = new THREE.Color((css.getPropertyValue("--accent") || "#5c77df").trim());
    const hot = new THREE.Color((css.getPropertyValue("--hot") || "#f59145").trim());
    for (const o of objects) {
      const g = ghosts.get(o.name); if (!g) continue;
      const r = o.r || 0.04, h = o.h || 0.06, sel = selected === o.name, col = sel ? hot : accent;
      const geo = o.shape === "box" ? new THREE.BoxGeometry(2 * r, 2 * r, h) : new THREE.CylinderGeometry(r, r, h, 32).rotateX(Math.PI / 2);
      const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: col, transparent: true, opacity: 0.38, depthWrite: false, roughness: 0.5 }));
      m.position.set(g.x, g.y, z0 + h / 2 + 0.001); m.userData.goalName = o.name; m.renderOrder = 5;
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo, 30), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.9 }));
      m.add(edges);
      layer.add(m); meshes.push(m);
      const ring = new THREE.Mesh(new THREE.RingGeometry(r + 0.008, r + 0.016, 40), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.7, depthWrite: false }));
      ring.position.set(g.x, g.y, z0 + 0.003); layer.add(ring);
      const lab = P.labelSprite(o.label || o.name.replace(/_/g, " "), sel ? "#f59145" : "#5c77df", "#fff", 0.032);
      lab.position.set(g.x, g.y, z0 + h + 0.05); layer.add(lab);
      const from = new THREE.Vector3(o.pos[0], o.pos[1], z0 + 0.004), to = new THREE.Vector3(g.x, g.y, z0 + 0.004);
      if (from.distanceTo(to) > 0.02) {
        layer.add(P.segmentMesh(from, to, 0.0035, col, 0.8));
        const tip = new THREE.Mesh(new THREE.ConeGeometry(0.012, 0.03, 16), new THREE.MeshBasicMaterial({ color: col }));
        const dir = to.clone().sub(from).normalize();
        tip.position.copy(to).addScaledVector(dir, -(r + 0.03)); tip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        layer.add(tip);
      }
    }
    P.redraw();
  }

  // pointer: drag a ghost; a tap on the table moves the selected ghost there (and never drops a path waypoint)
  let drag = null, press = null;
  function pickGhost(ev, hit) {
    const THREE = P.THREE, cv = ev.target && ev.target.getBoundingClientRect ? ev.target : null;
    if (cv && P.camera) {
      const r = cv.getBoundingClientRect();
      const ndc = new THREE.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
      const rc = new THREE.Raycaster(); rc.setFromCamera(ndc, P.camera);
      const h = rc.intersectObjects(meshes, false)[0];
      if (h) return h.object.userData.goalName;
    }
    if (!hit) return null;
    let best = null, bd = 1e9;
    for (const o of objects) {                 // touch-friendly: near the ghost's footprint counts
      const g = ghosts.get(o.name); if (!g) continue;
      const d = Math.hypot(hit.x - g.x, hit.y - g.y);
      if (d < (o.r || 0.04) + 0.025 && d < bd) { best = o.name; bd = d; }
    }
    return best;
  }
  function onPointer({ type, ev, hit }) {
    if (!(open && mode === "arrange") || running()) return false;
    if (type === "down") {
      const n = pickGhost(ev, hit);
      press = { x: ev.clientX, y: ev.clientY };
      if (!n) return false;
      const g = ghosts.get(n);
      drag = { name: n, dx: hit ? g.x - hit.x : 0, dy: hit ? g.y - hit.y : 0 };
      selected = n; renderObjs(); draw();
      return true;
    }
    if (type === "move") {
      if (!drag) return false;
      if (hit) moveGhost(drag.name, hit.x + drag.dx, hit.y + drag.dy);
      return true;
    }
    if (type === "up") {
      if (drag) { drag = null; press = null; return true; }
      const tap = press && Math.hypot(ev.clientX - press.x, ev.clientY - press.y) < 6; press = null;
      if (!tap) return false;
      if (hit && selected) moveGhost(selected, hit.x, hit.y);
      return !!hit;
    }
    return false;
  }
  PCUI.with3D((v) => { P = v; layer = v.layer("goal"); v.capture(onPointer); draw(); });

  // ------------------------------------------------------------------ rendering from state
  function renderObjs() {
    el.objs.innerHTML = objects.map((o) => {
      const cm = movedCm(o.name), g = ghosts.get(o.name);
      const dt = g && cm >= 1 ? `moves ${cm.toFixed(0)} cm` : "stays";
      return `<li data-sel="${selected === o.name}"><span class="sw" aria-hidden="true"></span><span class="nm">${esc(o.label || o.name)}</span>
        <span class="dt">${dt}</span><button type="button" class="goal-btn icon" data-obj="${esc(o.name)}" aria-pressed="${selected === o.name}"
        aria-label="Select the ${esc(o.label || o.name)} ghost, then tap the table">${PCUI.icon("point", 18)}</button></li>`;
    }).join("") || `<li><span class="nm">No movable objects</span></li>`;
  }
  const STEP_TXT = (s) => (s.kind === "aside" ? `Move ${s.label} out of the way` : `Put ${s.label} in its spot`);
  function render() {
    const g = goal, s = g.status || "idle";
    el.status.dataset.s = s; el.status.textContent = s === "idle" ? "" : (g.message || "");
    el.steps.innerHTML = arr(g.sequence).map((x, k) => `<li data-s="${esc(x.status)}"><span class="n">${x.status === "done" ? PCUI.icon("check", 14) : x.status === "failed" ? PCUI.icon("x", 14) : k + 1}</span>
      <span class="t">${esc(STEP_TXT(x))}</span>${x.err_cm != null && x.kind === "place" ? `<span class="e">${Number(x.err_cm).toFixed(1)} cm</span>` : ""}</li>`).join("");
    const busy = running();
    el.cancel.hidden = s !== "running";
    el.make.hidden = s === "running";
    el.make.disabled = busy || (mode === "photo" && !photoRead());
    el.preview_btn.hidden = s === "running";
    el.preview_btn.disabled = busy;
    card.querySelectorAll("[data-g='pick'],[data-g='reread'],[data-view]").forEach((b) => { b.disabled = busy; });
    renderObjs();
    renderPhotoDots();
  }
  function renderPhotoDots() {
    el.preview.querySelectorAll(".dot").forEach((d) => d.remove());
    const ph = goal.photo; if (!photo || !ph || !arr(ph.found).length) return;
    const img = el.preview.querySelector("img"); if (!img || !img.naturalWidth) return;
    const W = el.preview.clientWidth, H = el.preview.clientHeight, s = Math.min(W / img.naturalWidth, H / img.naturalHeight);
    const ox = (W - img.naturalWidth * s) / 2, oy = (H - img.naturalHeight * s) / 2;
    for (const f of ph.found) {
      const d = document.createElement("span"); d.className = "dot"; d.textContent = String(f.name).replace(/_/g, " ");
      d.style.left = `${ox + f.u * img.naturalWidth * s}px`; d.style.top = `${oy + f.v * img.naturalHeight * s}px`;
      el.preview.appendChild(d);
    }
  }

  PCUI.onState((s) => {
    const g = s && s.goal; if (!g || typeof g !== "object") return;
    const prevStatus = goal.status;
    goal = g;
    objects = arr(g.objects).filter((o) => o && Array.isArray(o.pos) && o.pos.length >= 2);
    syncGhosts();
    if (g.rev !== lastRev) {           // a new goal from the server (a photo, or the spots it kept clear of obstacles)
      lastRev = g.rev;
      for (const [n, p] of Object.entries(g.targets || {})) {
        const gh = ghosts.get(n);
        if (gh && Array.isArray(p) && !(drag && drag.name === n)) { gh.x = p[0]; gh.y = p[1]; gh.touched = true; }
      }
    }
    if (g.status === "done" && prevStatus !== "done") for (const gh of ghosts.values()) gh.touched = false;   // follow the objects again
    if (!open) card.hidden = !(g.status === "running" || g.status === "locating");
    const sayKey = `${g.rev}|${g.status}`;
    if (["ready", "done", "failed", "stopped"].includes(g.status) && sayKey !== lastSaid && prevStatus !== undefined && g.message) {
      lastSaid = sayKey; PCUI.say("robot", g.message);
    }
    render(); draw();
  });
})();
