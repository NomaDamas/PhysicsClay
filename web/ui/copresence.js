// UX 3 co-presence (#20): you are in the robot's world, across the table.
// state.copresence (pps/features/copresence.py) -> a soft grey mannequin in the 3D stage (PC3D.layer("copresence"))
// with your pointing ray (orange) and its hit ring on the table, the robot's attention (indigo ring), and the spot it
// will hand things to you. The robot eye draws the same mannequin on the server.
// "Shared space" (stage chip): robot-eye stage with you across the table, wider robot eye that turns toward what it
// attends to, your camera smaller. Deixis: "pick this up", "put it there", "give it to me" (이거/저거/여기/저기/줘)
// are resolved on the server against where you pointed when you said it, then run as normal tasks (plan preview,
// "which one?" and the feed all apply).
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const pretty = (n) => String(n || "").replace(/_/g, " ");
  const store = { get(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }, set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } } };

  // ------------------------------------------------------------------ deixis (mirror of the server's classify)
  const OBJ = /(^|[^a-z'])(this|that|these|those)([^a-z]|$)|이거|저거|그거|이것|저것|그것|요거/;
  const PLACE = /(^|[^a-z'])(here|there)([^a-z]|$)|여기|저기|거기|이쪽|저쪽/;
  const HANDOVER = /\b(give|hand|pass|bring)\b[^.?!]*\bme\b|\bhand (it|this|that) over\b|건네|넘겨|나한테|저한테|내게|제게/;
  const HANDOVER_KO = /(^|\s)(줘|줘요|주세요|줄래)/;
  const VERB = /\b(put|place|set|move|drop|bring|leave|pick|grab|take|lift|get|hold)\b|\bup\b|놔|놓|둬|올려|옮겨|내려|집어|들어|잡아|가져/;
  const KO_VERB = /놔|놓|둬|올려|옮겨|내려|집어|들어|잡아|가져/;
  function kindOf(text) {
    const t = String(text || "").toLowerCase().trim();
    if (!t) return null;
    if (HANDOVER.test(t) || (HANDOVER_KO.test(t) && !KO_VERB.test(t))) return "handover";
    const o = OBJ.test(t), p = PLACE.test(t);
    if (p && (VERB.test(t) || /\bit\b/.test(t) || t.split(/\s+/).length <= 3)) return "place";
    if (o && VERB.test(t)) return "object";
    return null;
  }

  let seq = 0;
  const waiting = new Map(); // reply id -> resolve
  U.onState((s) => {
    const r = s.copresence && s.copresence.reply;
    if (r && waiting.has(r.id)) { const fn = waiting.get(r.id); waiting.delete(r.id); fn(r); }
  });
  function ask(text) {
    const id = `cp${++seq}-${Date.now() % 100000}`;
    return new Promise((res) => {
      if (!U.send({ type: "copresence", op: "utter", text, id })) { res(null); return; }
      waiting.set(id, res);
      setTimeout(() => { if (waiting.has(id)) { waiting.delete(id); res(null); } }, 3000);
    });
  }
  async function hear(text) {
    const r = await ask(text);
    if (!r) { U.say("robot", "I couldn't reach the robot to work out what you pointed at."); return; }
    if (!r.ok || !r.task) { U.say("robot", r.message || "I didn't see what you meant. Point at it for a moment, then say it again."); return; }
    const args = Object.assign({}, r.task); const action = args.action; delete args.action;
    if (window.PC && window.PC.runTask) {
      const res = await window.PC.runTask(action, args, "voice");
      if (res && res.message) U.say("robot", res.message);
    } else U.send(Object.assign({ type: "task", id: `cp-${Date.now()}`, action }, args));
  }
  window.addEventListener("pc-hear", (ev) => {
    if (ev.defaultPrevented) return;                 // a "which one?" answer, a plan "yes", a zone… got it first
    const text = ev.detail && ev.detail.text, c = U.state.copresence;
    if (!c || !c.on) return;
    U.send({ type: "copresence", op: "heard" });     // the robot looks at you when you speak
    if (!c.joints) return;
    const k = kindOf(text);
    if (!k || (k !== "handover" && !c.recent_point)) return;   // no pointing to resolve: let the normal parser have it
    ev.preventDefault();
    hear(text);
  });

  // ------------------------------------------------------------------ stage chips: Shared space + attention
  const css = document.createElement("style");
  css.id = "pc-copresence-css";
  css.textContent = `
.cp-bar { position: absolute; left: 14px; top: 58px; z-index: 6; display: flex; align-items: center; gap: 8px; flex-wrap: wrap; max-width: calc(100% - 220px); pointer-events: none; }
.cp-bar > * { pointer-events: auto; }
.cp-toggle { display: inline-flex; align-items: center; gap: 8px; min-height: 36px; padding: 0 14px 0 10px; border-radius: 999px; border: 1px solid var(--line2, rgba(29,29,31,.16));
  background: var(--glass-strong, rgba(251,251,253,.9)); color: var(--t1, #1d1d1f); font: 600 var(--fs-sm, 13px) var(--sans, system-ui); cursor: pointer; box-shadow: var(--shadow, 0 8px 24px rgba(0,0,0,.08)); backdrop-filter: blur(10px); }
.cp-toggle::before { content: ""; position: absolute; inset: -4px; }
.cp-toggle { position: relative; }
.cp-toggle:hover { background: var(--active, #f0f0f3); }
.cp-toggle:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.32)); }
.cp-toggle[aria-pressed="true"] { background: var(--inv-bg, #1d1d1f); color: var(--inv-fg, #fff); border-color: var(--inv-bg, #1d1d1f); }
.cp-sw { width: 26px; height: 16px; border-radius: 999px; background: var(--line2, rgba(29,29,31,.16)); position: relative; transition: background var(--fast, .14s); flex: none; }
.cp-sw::after { content: ""; position: absolute; left: 2px; top: 2px; width: 12px; height: 12px; border-radius: 50%; background: #fff; transition: transform var(--fast, .14s); box-shadow: 0 1px 2px rgba(0,0,0,.2); }
.cp-toggle[aria-pressed="true"] .cp-sw { background: var(--accent, #5c77df); }
.cp-toggle[aria-pressed="true"] .cp-sw::after { transform: translateX(10px); }
.cp-look { display: inline-flex; align-items: center; gap: 7px; height: 30px; padding: 0 12px 0 9px; border-radius: 999px; background: var(--glass, rgba(255,255,255,.92)); color: var(--t1, #1d1d1f);
  font: 600 12px var(--sans, system-ui); white-space: nowrap; box-shadow: 0 1px 2px rgba(0,0,0,.06); max-width: 100%; overflow: hidden; text-overflow: ellipsis; }
.cp-look[hidden], .cp-demo[hidden] { display: none; }
.cp-look i { width: 8px; height: 8px; border-radius: 50%; background: var(--accent, #5c77df); flex: none; box-shadow: 0 0 0 3px var(--accent-tint, rgba(92,119,223,.12)); }
.cp-look[data-kind="object"] i, .cp-look[data-kind="spot"] i { background: var(--hot, #f59145); box-shadow: 0 0 0 3px var(--hot-tint, rgba(245,145,69,.16)); }
.cp-look small { font-weight: 500; color: var(--t2, rgba(29,29,31,.64)); }
.cp-demo { height: 30px; padding: 0 12px; border-radius: 999px; border: 1px dashed var(--line2, rgba(29,29,31,.16)); background: var(--glass, rgba(255,255,255,.8)); color: var(--t2, rgba(29,29,31,.64));
  font: 600 12px var(--sans, system-ui); cursor: pointer; position: relative; }
.cp-demo::before { content: ""; position: absolute; inset: -7px -2px; }
.cp-demo:hover { color: var(--t1, #1d1d1f); }
.cp-demo:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.32)); }
body.pc-shared #camPanel { width: 78%; align-self: flex-start; transition: width var(--med, .26s) var(--ease, ease); }
@media (max-width: 760px) { .cp-bar { top: auto; bottom: 52px; left: 10px; max-width: calc(100% - 20px); gap: 6px; } .cp-toggle { min-height: 32px; font-size: 12px; padding: 0 10px 0 8px; } .cp-look { height: 28px; font-size: 11px; } .cp-demo { display: none !important; } body.pc-shared #camPanel { width: auto; } }
@media (prefers-reduced-motion: reduce) { .cp-sw, .cp-sw::after, body.pc-shared #camPanel { transition: none; } }
`;
  document.head.appendChild(css);

  const bar = document.createElement("div");
  bar.className = "cp-bar";
  bar.innerHTML = `<button type="button" class="cp-toggle" aria-pressed="false" title="Robot eye with you across the table: point, then say “pick this up”, “put it there” or “give it to me”"><span class="cp-sw" aria-hidden="true"></span><span>Shared space</span></button>
    <span class="cp-look" hidden aria-live="polite"><i aria-hidden="true"></i><span class="cp-look-t"></span></span>
    <button type="button" class="cp-demo" hidden title="No camera: a synthetic person points at the mug, at a spot, then holds out a hand">Show a pointing person</button>`;
  U.slot("stage").appendChild(bar);
  const tog = bar.querySelector(".cp-toggle"), look = bar.querySelector(".cp-look"), lookT = bar.querySelector(".cp-look-t"), demo = bar.querySelector(".cp-demo");

  let shared = store.get("pc.shared") === "1" || new URLSearchParams(location.search).get("shared") === "1";
  let synced = null;
  function applyShared(on, user) {
    shared = !!on; tog.setAttribute("aria-pressed", String(shared));
    document.body.classList.toggle("pc-shared", shared);
    if (user) store.set("pc.shared", shared ? "1" : "0");
    if (U.send({ type: "copresence", op: "view", on: shared })) synced = shared;
    if (shared && window.PC && window.PC.setStageView) window.PC.setStageView("robot");
    if (user) U.say("sys", shared ? "Shared space: you're across the table in the robot's view. Point at something and say “pick this up”, “put it there” or “give it to me”."
      : "Shared space off.");
  }
  tog.addEventListener("click", () => applyShared(!shared, true));
  demo.addEventListener("click", () => { if (U.send({ type: "copresence", op: "demo" })) U.say("sys", "A synthetic person points at the mug, then at a spot, then holds out a hand (about 6 s each). Say “pick this up”, “put it there”, then “give it to me”."); });
  if (shared) { document.body.classList.add("pc-shared"); tog.setAttribute("aria-pressed", "true"); }

  function lookText(a, c) {
    if (!a) return null;
    const p = c.pointing && c.pointing.armed ? c.pointing : null;
    if (a.kind === "object") return [`Looking at: ${pretty(a.name)}`, p ? "say “pick this up”" : ""];
    if (a.kind === "spot") return ["Looking at: that spot", "say “put it here”"];
    if (a.name === "your hand") return ["Looking at: your hand", ""];
    return ["Looking at: you", ""];
  }

  // ------------------------------------------------------------------ 3D: mannequin, ray, rings
  const BONES = [["l_shoulder", "l_elbow", 0.045], ["l_elbow", "l_wrist", 0.038], ["r_shoulder", "r_elbow", 0.045], ["r_elbow", "r_wrist", 0.038],
    ["l_hip", "l_knee", 0.065], ["l_knee", "l_ankle", 0.05], ["r_hip", "r_knee", 0.065], ["r_knee", "r_ankle", 0.05],
    ["l_shoulder", "r_shoulder", 0.06], ["l_hip", "r_hip", 0.075], ["neck", "pelvis", 0.13]];
  const BALLS = { l_shoulder: 0.05, r_shoulder: 0.05, l_elbow: 0.045, r_elbow: 0.045, l_knee: 0.065, r_knee: 0.065, l_hip: 0.075, r_hip: 0.075,
    l_wrist: 0.038, r_wrist: 0.038, l_ankle: 0.05, r_ankle: 0.05, neck: 0.06 };
  let V = null, G = null, P = null;
  function build(v) {
    V = v; const T = v.THREE; G = v.layer("copresence");
    // soft grey, a little translucent: the person is there, but never hides the table from the 3D camera behind them
    const grey = new T.MeshStandardMaterial({ color: 0xc4c8d1, roughness: 0.85, metalness: 0, transparent: true, opacity: 0.62, depthWrite: false });
    const cyl = new T.CylinderGeometry(1, 1, 1, 16, 1), sph = new T.SphereGeometry(1, 20, 14);
    const mesh = (geo, mat) => { const m = new T.Mesh(geo, mat); m.castShadow = true; m.userData.sharedGeo = true; m.matrixAutoUpdate = true; G.add(m); return m; };
    P = { bones: BONES.map(([a, b, r]) => ({ a, b, r, m: mesh(cyl, grey) })), balls: {}, T };
    for (const k in BALLS) { const m = mesh(sph, grey); m.scale.setScalar(BALLS[k]); P.balls[k] = m; }
    P.head = mesh(sph, grey); P.head.scale.set(0.085, 0.08, 0.11);
    P.neck = mesh(cyl, grey);
    P.hands = { l: mesh(sph, grey), r: mesh(sph, grey) };
    P.feet = { l: mesh(cyl, grey), r: mesh(cyl, grey) };
    const orange = new T.MeshBasicMaterial({ color: 0xf59145, transparent: true, opacity: 0.95, depthWrite: false });
    const orangeSoft = new T.MeshBasicMaterial({ color: 0xf59145, transparent: true, opacity: 0.35, depthWrite: false });
    P.ray = new T.Mesh(cyl, orange); P.ray.userData.sharedGeo = true; G.add(P.ray);
    P.raySoft = new T.Mesh(cyl, orangeSoft); P.raySoft.userData.sharedGeo = true; G.add(P.raySoft);
    P.ring = new T.Mesh(new T.RingGeometry(0.03, 0.04, 48), orange.clone()); P.ring.material.side = T.DoubleSide; G.add(P.ring);
    P.dot = new T.Mesh(new T.CircleGeometry(0.012, 24), orange.clone()); G.add(P.dot);
    const indigo = new T.MeshBasicMaterial({ color: 0x5c77df, transparent: true, opacity: 0.85, depthWrite: false, side: T.DoubleSide });
    P.att = new T.Mesh(new T.RingGeometry(0.9, 1, 64), indigo); G.add(P.att);
    P.give = new T.Mesh(new T.TorusGeometry(0.05, 0.006, 10, 40), indigo.clone()); G.add(P.give);
    hideAll();
  }
  function hideAll() { if (G) for (const c of G.children) c.visible = false; }
  const vec = (p) => new P.T.Vector3(p[0], p[1], p[2]);
  const UP = () => new P.T.Vector3(0, 1, 0);
  function seg(m, a, b, r) {
    const A = vec(a), B = vec(b), d = new P.T.Vector3().subVectors(B, A), len = d.length();
    if (len < 1e-5) { m.visible = false; return; }
    m.position.copy(A).addScaledVector(d, 0.5); m.quaternion.setFromUnitVectors(UP(), d.normalize()); m.scale.set(r, len, r); m.visible = true;
  }
  function draw(c) {
    if (!P) return;
    const J = c && c.on && c.visible !== false ? c.joints : null;   // no person drawn for the synthetic no-camera dancer
    if (!J) { hideAll(); V.redraw(); return; }
    for (const b of P.bones) { if (J[b.a] && J[b.b]) seg(b.m, J[b.a], J[b.b], b.r); else b.m.visible = false; }
    for (const k in P.balls) { const m = P.balls[k]; if (J[k]) { m.position.set(J[k][0], J[k][1], J[k][2]); m.visible = true; } else m.visible = false; }
    if (J.head) { P.head.position.set(J.head[0], J.head[1], J.head[2]); P.head.visible = true; }
    if (J.neck && J.head) seg(P.neck, J.neck, [J.head[0], J.head[1], J.head[2] - 0.06], 0.05);
    for (const s of ["l", "r"]) {
      const w = J[`${s}_wrist`], i = J[`${s}_index`] || w, h = P.hands[s];
      if (w) { h.position.set((w[0] + i[0]) / 2, (w[1] + i[1]) / 2, (w[2] + i[2]) / 2); h.scale.setScalar(0.042); h.visible = true; } else h.visible = false;
      const a = J[`${s}_ankle`];
      if (a) seg(P.feet[s], a, [a[0] - 0.12, a[1], a[2] - 0.03], 0.04); else P.feet[s].visible = false;
    }
    const p = c.pointing;
    P.ray.visible = P.raySoft.visible = P.ring.visible = P.dot.visible = false;
    if (p && p.from && p.dir) {
      const end = p.hit || [p.from[0] + p.dir[0] * 0.6, p.from[1] + p.dir[1] * 0.6, p.from[2] + p.dir[2] * 0.6];
      seg(p.armed ? P.ray : P.raySoft, p.from, end, p.armed ? 0.005 : 0.003);
      if (p.hit) {
        P.ring.position.set(p.hit[0], p.hit[1], p.hit[2] + 0.003); P.ring.material.opacity = p.armed ? 0.95 : 0.35; P.ring.visible = true;
        if (p.armed) { P.dot.position.set(p.hit[0], p.hit[1], p.hit[2] + 0.004); P.dot.visible = true; }
      }
    }
    const a = c.attention;
    P.att.visible = false;
    if (a && a.pos) {
      if (a.kind === "object") { P.att.position.set(a.pos[0], a.pos[1], V.TABLE.z + 0.004); P.att.scale.setScalar(0.1); P.att.visible = true; }
      else if (a.kind === "person" && J.pelvis) { P.att.position.set(J.pelvis[0], J.pelvis[1], 0.006); P.att.scale.setScalar(0.42); P.att.visible = true; }
    }
    const ho = c.handover;
    P.give.visible = !!(ho && ho.target && ho.phase !== "given");
    if (P.give.visible) { P.give.position.set(ho.target[0], ho.target[1], ho.target[2]); P.give.rotation.set(0, Math.PI / 2, 0); }
    V.redraw();
  }
  U.with3D(build);

  // ------------------------------------------------------------------ state
  let lastLook = "";
  U.onState((s) => {
    const c = s.copresence;
    if (!c) { bar.hidden = true; return; }
    bar.hidden = false;
    if (synced === null && shared) applyShared(true, false);   // remembered from last time: robot eye, you across the table
    else if (synced !== shared && U.send({ type: "copresence", op: "view", on: shared })) synced = shared;   // after a reconnect
    const lt = c.on && c.joints ? lookText(c.attention, c) : null;
    const key = lt ? lt.join("|") + (c.attention && c.attention.kind) : "";
    if (key !== lastLook) {
      lastLook = key; look.hidden = !lt;
      if (lt) { look.dataset.kind = c.attention.kind; lookT.innerHTML = `${esc(lt[0])}${lt[1] ? ` <small>· ${esc(lt[1])}</small>` : ""}`; }
    }
    demo.hidden = !(c.on && c.source === "synthetic");
    if (P) draw(c);
  });
  window.PCCopresence = { kindOf, hear: (t) => hear(t), get shared() { return shared; }, setShared: (on) => applyShared(on, true) };
})();
