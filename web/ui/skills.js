// v1 P4 Skills from demonstration (#32) + demo packs (#35). State: state.skillbook (pps/features/skills.py).
//   3D stage (layer "skills"): every weld bead as a growing tube along the torch tip's actual path (weave included; the
//     hot end glows while it runs), and the scene's seams as dashed guides with a label.
//   Plan card: when the proposal is a skill, a row of quick corrections (Wider · Narrower · Slower · Faster …) that edit
//     its parameters ({type:"skill", op:"edit", text}); the server recompiles and re-proposes.
//   Side slot: the loaded demo pack (what to say, what needs physics, "Play a demo" for people without a camera).
//   Voice (when the LLM agent is off): "wider", "a bit slower", "more push", "6 mm", "3 Hz" correct the skill on screen;
//     "now the L seam" / "the other seam" applies it there. With the agent on, its tools do the same.
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;
  const arr = (x) => (Array.isArray(x) ? x : []);
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const send = (m) => { const ok = U.send(m); if (!ok) U.toast("Not connected to the robot."); return ok; };
  let SB = {}, S = {};

  const QUICK = {
    weave_seam: [["Wider", "wider"], ["Narrower", "narrower"], ["Slower", "slower"], ["Faster", "faster"], ["More push", "more push"], ["Closer", "closer"]],
    wipe: [["Harder", "harder"], ["Softer", "softer"], ["Tighter", "narrower"], ["Spiral", "spiral"], ["Slower", "slower"]],
    stir: [["Wider", "wider"], ["Smaller", "smaller"], ["Slower", "slower"], ["Faster", "faster"], ["Deeper", "deeper"]],
    press: [["Harder", "harder"], ["Softer", "softer"], ["Longer", "longer"], ["Shorter", "shorter"]],
    spot: [["Help earlier", "help earlier"], ["Let me grind", "help later"], ["More force", "harder"], ["Less force", "softer"]],
  };

  const css = `
.sk-row { display: flex; flex-wrap: wrap; gap: 6px; align-items: center; margin-top: 8px; padding: 10px 12px; border-radius: var(--r-inner, 16px);
  background: var(--raised, #fff); border: 1px solid var(--line, rgba(29,29,31,.1)); box-shadow: var(--shadow, none); }
.sk-row[hidden], .sk-pack[hidden] { display: none !important; }
.sk-row b { font: 600 12px var(--sans, system-ui); color: var(--t2, rgba(29,29,31,.64)); margin-right: 4px; white-space: nowrap; }
.sk-chip { min-height: 36px; padding: 0 12px; border-radius: 999px; border: 1px solid var(--line2, rgba(29,29,31,.16)); background: var(--field, #f5f5f7);
  color: var(--t1, #1d1d1f); font: 600 12.5px var(--sans, system-ui); cursor: pointer; position: relative; }
.sk-chip::before { content: ""; position: absolute; inset: -4px -1px; }
.sk-chip:hover { background: var(--active, #f0f0f3); }
.sk-chip:focus-visible, .sk-btn:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.sk-desc { flex-basis: 100%; font: 500 12.5px/1.35 var(--sans, system-ui); color: var(--t1, #1d1d1f); }
.sk-pack { container-type: inline-size; background: var(--raised, #fff); border: 1px solid var(--line, rgba(29,29,31,.1)); border-radius: var(--r-inner, 16px);
  box-shadow: var(--shadow, none); padding: 10px 14px 12px; display: flex; flex-direction: column; gap: 8px; margin-top: 10px; min-width: 0; }
.sk-pack-head { display: flex; align-items: center; gap: 8px; min-height: 36px; }
.sk-pack-head svg { color: var(--hot, #f59145); flex: none; }
.sk-pack-head b { font-size: var(--fs-sm, 13px); font-weight: 650; }
.sk-pack-head span { margin-left: auto; font-size: 11px; color: var(--t3, rgba(29,29,31,.45)); }
.sk-pack p { margin: 0; font-size: 12px; line-height: 1.4; color: var(--t2, rgba(29,29,31,.64)); }
.sk-ex { display: flex; flex-wrap: wrap; gap: 6px; }
.sk-btn { min-height: 40px; padding: 0 14px; border-radius: 999px; border: 1px solid var(--accent, #5c77df); background: var(--accent, #5c77df);
  color: var(--on-accent, #fff); font: 600 13px var(--sans, system-ui); cursor: pointer; align-self: flex-start; }
.sk-btn:disabled { opacity: .5; cursor: default; }
.sk-skills { font-size: 12px; color: var(--t2, rgba(29,29,31,.64)); line-height: 1.4; }
`;
  const st = document.createElement("style"); st.id = "pcui-skills-css"; st.textContent = css; document.head.appendChild(st);

  // ------------------------------------------------------------------ plan card: quick corrections
  const row = document.createElement("div");
  row.className = "sk-row"; row.hidden = true; row.setAttribute("role", "group"); row.setAttribute("aria-label", "Correct the skill");
  let rowMounted = false, rowKey = "";
  function mountRow() {
    if (rowMounted) return;
    U.slot("plan").appendChild(row); rowMounted = true;
  }
  function renderRow() {
    mountRow();
    const p = S.plan, sk = p && p.skill;
    const show = !!(sk && (p.phase === "proposing" || p.phase === "blocked"));
    row.hidden = !show;
    if (!show) { rowKey = ""; return; }
    const key = JSON.stringify([sk.name, sk.describe, p.phase]);
    if (key === rowKey) return;
    rowKey = key;
    const q = QUICK[sk.kind] || [];
    row.innerHTML = `<span class="sk-desc">${esc(sk.label)} · ${esc(sk.describe)}</span><b>Correct it:</b>` +
      q.map(([l, t]) => `<button type="button" class="sk-chip" data-t="${esc(t)}">${esc(l)}</button>`).join("");
  }
  row.addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-t]"); if (!b) return;
    if (send({ type: "skill", op: "edit", text: b.dataset.t, id: `ui-sk-${Date.now() % 1e7}` })) U.say("user", b.textContent + ".");
  });

  // ------------------------------------------------------------------ side slot: the demo pack
  const pack = document.createElement("section");
  pack.className = "sk-pack"; pack.hidden = true; pack.setAttribute("aria-label", "Demo pack");
  let packMounted = false, packKey = "";
  const ICON = '<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><rect x="3.5" y="7" width="17" height="12" rx="2"/><path d="M8 7V5h8v2M3.5 12h17"/></svg>';
  function renderPack() {
    if (!packMounted) { U.slot("side").appendChild(pack); packMounted = true; }
    const pk = SB.pack, skills = arr(SB.skills).filter((k) => k.ok !== false && !k.example);
    pack.hidden = !pk;
    if (!pk) { packKey = ""; return; }
    const key = JSON.stringify([pk, skills.map((k) => k.name + k.describe), (S.watch || {}).mode]);
    if (key === packKey) return;
    packKey = key;
    pack.innerHTML = `<div class="sk-pack-head">${ICON}<b>${esc(pk.label)}</b><span>demo pack</span></div>
      ${pk.physics ? `<p>${esc(pk.physics)}</p>` : ""}
      <div class="sk-ex">${arr(pk.examples).map((e) => `<button type="button" class="sk-chip" data-ex="${esc(e)}">“${esc(e)}”</button>`).join("")}</div>
      ${pk.demo ? `<button type="button" class="sk-btn" data-demo>No camera? Play a demo</button>` : ""}
      ${skills.length ? `<div class="sk-skills">Your skills: ${skills.slice(0, 4).map((k) => `<b>${esc(k.name.replace(/_/g, " "))}</b> (${esc(k.describe)})`).join(", ")}</div>` : ""}`;
  }
  pack.addEventListener("click", (ev) => {
    const b = ev.target.closest("button"); if (!b) return;
    if (b.hasAttribute("data-demo")) { if (send({ type: "skill", op: "pack_demo", id: `ui-pd-${Date.now() % 1e7}` })) U.say("sys", "Playing a synthetic person for this pack…"); return; }
    if (b.dataset.ex && U.runExample) U.runExample(b.dataset.ex);
  });

  // examples on the idle plan card follow the pack (and go back to the robot's own when it is unloaded)
  let saved = null, exPack = null;
  function applyExamples() {
    const pk = SB.pack ? SB.pack.id : null;
    if (pk === exPack) return;
    if (pk && !exPack) saved = U.examples;
    exPack = pk;
    U.examples = pk ? arr(SB.pack.examples) : saved;
    window.dispatchEvent(new CustomEvent("pc-examples", { detail: U.examples }));
  }

  // ------------------------------------------------------------------ voice (agent off) + pack examples offline
  const EDIT = /\b(wider|narrower|tighter|broader|smaller|bigger|slower|faster|quicker|more push|less push|push|drag|closer|further|farther|harder|softer|gentler|deeper|shallower|spiral|raster|zig ?zag|circles?|figure ?(8|eight)|help (earlier|later|sooner)|let me grind|longer|shorter|more turns|fewer turns|no weave)\b|\d+(\.\d+)?\s*(mm|cm|hz|n|newtons?|deg|degrees|°|s|sec|seconds|turns|passes)\b|넓게|좁게|천천히|빨리|세게|살살/i;
  const APPLY = /\b(now|then|also|do|weld|apply|same|like that|too)\b.*\b(l[ -]?seam|other seam|straight seam|seam ?[a-z]\b|the seam)/i;
  function targetOf(t) {
    if (/\bl[ -]?seam\b|\bcorner\b/i.test(t)) return "seam_l";
    if (/straight/i.test(t)) return "seam_a";
    if (/other|next/i.test(t)) return "other";
    return null;
  }
  function hear(text) {
    const t = String(text || "").trim(); if (!t) return false;
    const cur = SB.current;
    if (APPLY.test(t)) {
      const name = (cur && cur.name) || (arr(SB.skills).find((k) => k.kind === "weave_seam" && !k.example) || {}).name;
      if (!name) return false;
      return send({ type: "skill", op: "apply", name, target: targetOf(t), id: `ui-sk-${Date.now() % 1e7}` });
    }
    if (cur && (cur.phase === "proposed" || cur.phase === "blocked") && EDIT.test(t)) {
      return send({ type: "skill", op: "edit", text: t, id: `ui-sk-${Date.now() % 1e7}` });
    }
    return false;
  }
  window.addEventListener("pc-hear", (ev) => {
    if (ev.defaultPrevented || (U.agent && U.agent.active)) return;
    if (hear(ev.detail && ev.detail.text)) ev.preventDefault();
  });
  function wrapRun() {                                       // pack examples ("now weld the L seam like that") work offline
    const r = U.runExample;
    if (!r || r.__sk) return;
    const w = (text) => { if (!(U.agent && U.agent.active) && hear(text)) { U.say("user", text); return; } r(text); };
    w.__sk = true; U.runExample = w;
  }
  wrapRun();
  window.addEventListener("load", wrapRun);                  // in case robots.js defines runExample after us

  // ------------------------------------------------------------------ 3D: beads and seams
  let V = null, layer = null, seamKey = "", seamGroup = null;
  const beadObj = new Map();     // id -> {group, n, t}
  function buildSeams() {
    if (!V || !layer) return;
    const seams = arr(SB.seams), key = JSON.stringify(seams.map((s) => [s.name, s.points]));
    if (key === seamKey) return;
    seamKey = key;
    if (seamGroup) { layer.remove(seamGroup); V.disposeGroup(seamGroup); }
    seamGroup = new V.THREE.Group(); seamGroup.name = "seams"; layer.add(seamGroup);
    const T = V.THREE;
    for (const s of seams) {
      const pts = arr(s.points).map((p) => new T.Vector3(p[0], p[1], p[2] + 0.0015));
      if (pts.length < 2) continue;
      const line = new T.Line(new T.BufferGeometry().setFromPoints(pts), new T.LineDashedMaterial({ color: 0xf59145, dashSize: 0.008, gapSize: 0.006 }));
      line.computeLineDistances(); seamGroup.add(line);
      const mid = pts[Math.floor(pts.length / 2)].clone(); mid.z += 0.03;
      const lab = V.labelSprite(String(s.label || s.name).replace(/^the /, ""), "#f59145", "#fff", 0.022); lab.position.copy(mid); seamGroup.add(lab);
    }
    V.redraw();
  }
  function buildBead(b) {
    const T = V.THREE, pts = arr(b.pts).map((p) => new T.Vector3(p[0], p[1], p[2]));
    const g = new T.Group(); g.name = "bead:" + b.id;
    const r = Math.max(0.0015, +b.r || 0.003);
    if (pts.length >= 2) {
      const curve = new T.CatmullRomCurve3(pts, false, "centripetal");
      const tube = new T.Mesh(new T.TubeGeometry(curve, Math.min(1400, pts.length * 3), r, 8, false),
        new T.MeshStandardMaterial({ color: 0x8a7560, roughness: 0.45, metalness: 0.55 }));
      g.add(tube);
      if (b.live) {
        const tail = pts.slice(-14);
        if (tail.length >= 2) {
          const hot = new T.Mesh(new T.TubeGeometry(new T.CatmullRomCurve3(tail, false, "centripetal"), tail.length * 3, r * 1.08, 8, false),
            new T.MeshBasicMaterial({ color: 0xff8c2a }));
          g.add(hot);
        }
        const glow = new T.Mesh(new T.SphereGeometry(r * 2.2, 14, 10), new T.MeshBasicMaterial({ color: 0xffd27a, transparent: true, opacity: 0.85 }));
        glow.position.copy(pts[pts.length - 1]); g.add(glow);
      }
    }
    return g;
  }
  function buildBeads() {
    if (!V || !layer) return;
    const now = performance.now(), seen = new Set();
    for (const b of arr(SB.beads)) {
      seen.add(b.id);
      const cur = beadObj.get(b.id), n = arr(b.pts).length;
      if (cur && cur.n === n && cur.live === !!b.live) continue;
      if (cur && b.live && now - cur.t < 150) continue;          // a growing bead is rebuilt at most ~7 times a second
      if (cur) { layer.remove(cur.group); V.disposeGroup(cur.group); }
      const group = buildBead(b); layer.add(group);
      beadObj.set(b.id, { group, n, t: now, live: !!b.live });
    }
    for (const [id, o] of beadObj) if (!seen.has(id)) { layer.remove(o.group); V.disposeGroup(o.group); beadObj.delete(id); }
    V.redraw();
  }
  U.with3D((v) => { V = v; layer = v.layer("skills"); seamKey = ""; beadObj.clear(); buildSeams(); buildBeads(); });
  window.addEventListener("pc3d-ready", () => { seamKey = ""; beadObj.clear(); });

  // ------------------------------------------------------------------ state
  let lastReply = 0;
  U.onState((s) => {
    S = s || {}; SB = S.skillbook && typeof S.skillbook === "object" ? S.skillbook : {};
    renderRow(); renderPack(); applyExamples(); buildSeams(); buildBeads();
    const r = SB.reply;
    if (r && r.t !== lastReply) {
      lastReply = r.t;
      if (String(r.id || "").startsWith("ui-") && !r.ok && r.message) U.toast(r.message);
    }
  });
  window.PCSkills = { hear, get state() { return SB; } };
})();
