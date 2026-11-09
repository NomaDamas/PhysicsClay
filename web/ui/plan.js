// UX 2.0 #B · Plan preview: the robot proposes, you confirm.
// Motion commands you type, say or pick from a chip are sent with preview:true. The server answers with state.plan
// (see pps/features/plan.py); this module draws it as the plan card in the "plan" slot and as a translucent ghost
// (path, target ring, carried object) in PC3D.layer("plan"). "yes / go / do it" confirms, "no / cancel" cancels.
"use strict";
(function () {
  const P = window.PCUI;
  if (!P) return;
  const MOTION = new Set(["pick_place", "reach", "follow_path", "play_skill"]);
  const VIA = new Set(["typed", "voice", "gemini", "chip", "plan"]);   // buttons and keys stay direct
  const WAITING = new Set(["proposing", "needs_you", "blocked"]);
  const PHASE = { idle: "Idle", proposing: "Proposing", needs_you: "Needs you", blocked: "Can't do this", running: "Running", done: "Done", failed: "Didn't finish" };
  const EXAMPLES = [
    { label: "Put the mug in the center", action: "pick_place", args: { object: "mug", target: "center" } },
    { label: "Move the red block to the left", action: "pick_place", args: { object: "red block", target: "left" } },
    { label: "Reach over the front", action: "reach", args: { target: "front" } },
  ];
  // P.examples (strings, set per robot by robots.js) replace these; a string chip goes to P.runExample(text)
  const examples = () => (Array.isArray(P.examples) && P.examples.length ? P.examples : EXAMPLES);
  const exLabel = (e) => (typeof e === "string" ? e : e.label);
  if (new URLSearchParams(location.search).get("preview") === "0") P.planPreview = false;

  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const IC = {
    target: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3.2"/>',
    stop: '<rect x="6.5" y="6.5" width="11" height="11" rx="2"/>',
    alert: '<path d="M12 4.5 3 19.5h18z"/><path d="M12 10.5v4M12 17.3v.2"/>',
    edit: '<path d="M4.5 19.5h4l10-10-4-4-10 10z"/><path d="m13 7 4 4"/>',
    route: '<path d="M6 18.5h8.5a3.5 3.5 0 0 0 0-7h-5a3.5 3.5 0 0 1 0-7H18"/><circle cx="6" cy="18.5" r="1.6"/><circle cx="18" cy="4.5" r="1.6"/>',
  };
  const icon = (n, size = 18) => (IC[n]
    ? `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${IC[n]}</svg>`
    : P.icon(n, size));

  // ------------------------------------------------------------------ hooks app.js calls
  P.taskFilter = (action, args, source) => {
    if (!MOTION.has(action) || !VIA.has(source) || P.planPreview === false || P.state.demo) return args;
    return Object.assign({}, args, { preview: true });
  };
  const YES = /^(yes|yeah|yep|yup|sure|ok|okay|go|go ahead|go for it|do it|run|run it|start|confirm|let'?s go|let'?s do it|sounds good|looks good|perfect)( please| now)?$/;
  const YES_KO = /^(응|네|예|그래|좋아|해|해줘|해 줘|실행|실행해|진행|진행해|가자|고)$/;
  const NO = /^(no|nope|nah|cancel|cancel it|never ?mind|don'?t|do not|abort|forget it)( please| thanks)?$/;
  const NO_KO = /^(아니|아니야|아니요|아뇨|취소|취소해|하지 ?마|됐어)$/;
  P.hear = (text) => {
    const p = P.state.plan; if (!p) return false;
    const t = String(text || "").toLowerCase().replace(/[.!?,。]+/g, " ").replace(/\s+/g, " ").trim();
    const yes = YES.test(t) || YES_KO.test(t), no = NO.test(t) || NO_KO.test(t);
    if (no && WAITING.has(p.phase)) { act("cancel"); return true; }
    if (yes && p.phase === "proposing" && p.ok) { act("confirm"); return true; }
    if (yes && p.phase === "needs_you") { act("alt"); return true; }
    return false;
  };
  window.addEventListener("pc-hear", (ev) => { if (P.hear(ev.detail && ev.detail.text)) ev.preventDefault(); });

  // ------------------------------------------------------------------ actions
  function run(action, args, label) {
    if (!window.PC || !window.PC.runTask) return;
    if (label) P.say("user", label);
    window.PC.runTask(action, args, "chip").then((r) => { if (r && r.message) P.say("robot", r.message); });
  }
  function act(what) {
    const p = P.state.plan;
    if (what === "fix") {
      const inp = document.querySelector("#cmd") || document.querySelector('[data-slot="dock"] input, input[type="text"]');
      if (inp) { inp.focus(); inp.placeholder = "Tell me what to change…"; }
      P.say("sys", "Tell me what to change, e.g. “put it on the left instead”, or point at the spot.");
      return;
    }
    if (!p) return;
    if (what === "confirm") {
      if (P.send({ type: "plan", op: "confirm", id: p.id })) P.say("sys", "Confirmed. Running the plan.");
      else P.toast("Not connected to the robot.");
    } else if (what === "cancel") {
      P.send({ type: "plan", op: "cancel", id: p.id });
    } else if (what === "alt") {
      const a = p.alternative && p.alternative.task; if (!a) return;
      const args = Object.assign({}, a); delete args.action;
      run(a.action, args, p.alternative.label || "Use that instead");
    }
  }

  // ------------------------------------------------------------------ card
  const css = `
.plan-card { background: var(--raised, #fff); border-radius: var(--r-card, 24px); box-shadow: var(--shadow, 0 12px 32px rgba(40,30,90,.08));
  border: 1px solid var(--line, rgba(29,29,31,.1)); padding: 18px; display: flex; flex-direction: column; gap: 12px; color: var(--t1, #1d1d1f);
  font: 14px/1.45 var(--sans, system-ui); width: 100%; box-sizing: border-box; }
.plan-head { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
.plan-eyebrow { font-size: 12px; font-weight: 600; letter-spacing: .06em; text-transform: uppercase; color: var(--t2, rgba(29,29,31,.62)); }
.plan-phase { --c: var(--idle, #9a9aa0); font-size: 12px; font-weight: 600; padding: 3px 10px; border-radius: 999px; color: var(--c);
  background: color-mix(in srgb, var(--c) 12%, transparent); }
.plan-card[data-phase="proposing"] .plan-phase, .plan-card[data-phase="running"] .plan-phase { --c: var(--accent, #5c77df); }
.plan-card[data-phase="needs_you"] .plan-phase { --c: var(--hot, #f59145); }
.plan-card[data-phase="blocked"] .plan-phase, .plan-card[data-phase="failed"] .plan-phase { --c: var(--bad, #d6453a); }
.plan-card[data-phase="done"] .plan-phase { --c: var(--ok, #1f8f55); }
.plan-intent { margin: 0; font-size: 17px; font-weight: 600; line-height: 1.35; }
.plan-card[data-phase="idle"] .plan-intent { font-weight: 500; color: var(--t2, rgba(29,29,31,.62)); }
.plan-target { display: inline-flex; align-items: center; gap: 6px; align-self: flex-start; padding: 5px 12px 5px 9px; border-radius: 999px;
  background: var(--accent-tint, rgba(92,119,223,.12)); color: var(--accent, #5c77df); font-weight: 600; font-size: 13px; }
.plan-steps { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; gap: 6px; }
.plan-steps li { display: flex; align-items: center; gap: 10px; color: var(--t1, #1d1d1f); }
.plan-steps .n { flex: none; width: 24px; height: 24px; border-radius: 50%; display: grid; place-items: center; font-size: 12px; font-weight: 600;
  background: var(--field, #f5f5f7); color: var(--t2, rgba(29,29,31,.62)); }
.plan-steps li.now .n { background: var(--accent, #5c77df); color: var(--on-accent, #fff); }
.plan-steps li.past { color: var(--t3, rgba(29,29,31,.45)); }
.plan-steps li.past .n { background: color-mix(in srgb, var(--ok, #1f8f55) 14%, transparent); color: var(--ok, #1f8f55); }
.plan-chips { display: flex; flex-wrap: wrap; gap: 6px; }
.plan-chip { font-size: 12px; padding: 4px 10px; border-radius: 999px; border: 1px solid var(--line2, rgba(29,29,31,.16)); color: var(--t2, rgba(29,29,31,.62)); }
.plan-verdict { display: flex; gap: 10px; align-items: flex-start; padding: 12px; border-radius: var(--r-inner, 18px);
  --c: var(--ok, #1f8f55); background: color-mix(in srgb, var(--c) 9%, transparent); color: var(--c); }
.plan-verdict.bad { --c: var(--bad, #d6453a); }
.plan-verdict svg { flex: none; margin-top: 1px; }
.plan-verdict b { display: block; font-size: 15px; }
.plan-verdict ul { margin: 4px 0 0; padding-left: 16px; color: var(--t2, rgba(29,29,31,.62)); font-size: 13px; }
.plan-alt { font-size: 13px; color: var(--t1, #1d1d1f); padding: 10px 12px; border-radius: var(--r-inner, 18px); background: var(--field, #f5f5f7); }
.plan-alt b { color: var(--hot, #f59145); }
.plan-progress { height: 6px; border-radius: 999px; background: var(--field, #f5f5f7); overflow: hidden; }
.plan-progress i { display: block; height: 100%; width: 0; background: var(--accent, #5c77df); border-radius: inherit; transition: width .3s; }
.plan-actions, .plan-examples { display: flex; flex-wrap: wrap; gap: 8px; }
.plan-btn { min-height: 44px; padding: 0 16px; border-radius: 999px; border: 1px solid var(--line2, rgba(29,29,31,.16)); background: var(--raised, #fff);
  color: var(--t1, #1d1d1f); font: 600 14px var(--sans, system-ui); display: inline-flex; align-items: center; gap: 7px; cursor: pointer; }
.plan-btn:hover { background: var(--active, #f0f0f3); }
.plan-btn:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.plan-btn.primary { background: var(--accent, #5c77df); border-color: var(--accent, #5c77df); color: var(--on-accent, #fff); flex: 1; justify-content: center; }
.plan-btn.primary:hover { filter: brightness(1.06); }
.plan-btn.danger { color: var(--bad, #d6453a); }
.plan-example { min-height: 44px; padding: 0 14px; border-radius: 999px; border: 1px solid var(--line, rgba(29,29,31,.1)); background: var(--field, #f5f5f7);
  color: var(--t1, #1d1d1f); font: 500 13px var(--sans, system-ui); cursor: pointer; text-align: left; }
.plan-example:hover { background: var(--active, #f0f0f3); }
.plan-example:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.card[data-status="proposed"] .pill { color: var(--accent, #5c77df); }
`;
  const st = document.createElement("style"); st.id = "pcui-plan-css"; st.textContent = css; document.head.appendChild(st);

  const host = P.slot("plan");
  const fallback = !!(host && host.dataset && host.dataset.slotFallback);
  const card = document.createElement("section");
  card.className = "plan-card"; card.setAttribute("aria-live", "polite"); card.setAttribute("aria-label", "Robot's plan");
  host.appendChild(card);
  card.addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-act], button[data-ex]"); if (!b) return;
    if (b.dataset.ex != null) {
      const e = examples()[+b.dataset.ex]; if (!e) return;
      if (typeof e === "string" || !e.action) {
        const text = exLabel(e), cmd = document.querySelector("#cmd");
        if (P.runExample) P.runExample(text); else if (cmd) { cmd.value = text; cmd.focus(); }
        return;
      }
      run(e.action, e.args, e.label); return;
    }
    act(b.dataset.act);
  });

  const btn = (act, label, ic, cls = "") => `<button type="button" class="plan-btn ${cls}" data-act="${act}">${icon(ic)}<span>${esc(label)}</span></button>`;
  function body(p) {
    if (!p) {
      return `<header class="plan-head"><span class="plan-eyebrow">Robot's plan</span><span class="plan-phase">${PHASE.idle}</span></header>
        <p class="plan-intent">Tell me, point, or show me.</p>
        <div class="plan-examples">${examples().map((e, i) => `<button type="button" class="plan-example" data-ex="${i}">${esc(exLabel(e))}</button>`).join("")}</div>`;
    }
    const ph = p.phase, bad = ph === "blocked" || ph === "needs_you" || ph === "failed";
    const steps = (p.steps || []).map((s, i) => `<li data-i="${i}"><span class="n">${i + 1}</span><span>${esc(s.text)}</span></li>`).join("");
    let verdict = p.verdict, reasons = p.reasons || [];
    if (ph === "done") { verdict = "Done"; reasons = [p.message]; }
    if (ph === "failed") { verdict = "It didn't finish"; reasons = [p.message]; }
    if (ph === "running") { verdict = ""; }
    const alt = p.alternative && (ph === "needs_you" || ph === "blocked")
      ? `<div class="plan-alt">${ph === "needs_you" && p.question ? `<b>${esc(p.question)}</b><br>` : ""}${esc(p.alternative.text)}</div>` : "";
    let actions = "";
    if (ph === "proposing") actions = btn("confirm", "Run", "play", "primary") + btn("fix", "Fix", "edit") + btn("cancel", "Cancel", "x");
    else if (ph === "needs_you") actions = btn("alt", (p.alternative && p.alternative.label) || "Use that instead", "check", "primary") + btn("cancel", "Cancel", "x");
    else if (ph === "blocked") actions = btn("fix", "Fix", "edit", "primary") + btn("cancel", "Cancel", "x");
    else if (ph === "running") actions = btn("cancel", "Stop", "stop", "danger");
    else actions = btn("cancel", "OK", "check", "primary");
    return `<header class="plan-head"><span class="plan-eyebrow">Robot's plan</span><span class="plan-phase">${esc(PHASE[ph] || ph)}</span></header>
      <p class="plan-intent">${esc(p.intent)}</p>
      ${p.target ? `<span class="plan-target">${icon("target", 16)}<span>${esc(p.target.label)}</span></span>` : ""}
      ${steps ? `<ol class="plan-steps">${steps}</ol>` : ""}
      ${(p.constraints || []).length ? `<div class="plan-chips">${p.constraints.map((c) => `<span class="plan-chip">${esc(c)}</span>`).join("")}</div>` : ""}
      ${ph === "running" || ph === "done" ? '<div class="plan-progress" role="progressbar" aria-label="Progress"><i></i></div>' : ""}
      ${verdict ? `<div class="plan-verdict ${bad ? "bad" : ""}">${icon(bad ? "alert" : "check", 20)}<div><b>${esc(verdict)}</b>${reasons.filter(Boolean).length ? `<ul>${reasons.filter(Boolean).map((r) => `<li>${esc(r)}</li>`).join("")}</ul>` : ""}</div></div>` : ""}
      ${alt}
      <div class="plan-actions">${actions}</div>`;
  }
  let lastKey = null;
  function render(p) {
    const key = JSON.stringify(p ? Object.assign({}, p, { progress: 0, step_i: 0 }) : examples().map(exLabel));
    if (key !== lastKey) {
      lastKey = key;
      card.dataset.phase = p ? p.phase : "idle";
      card.innerHTML = body(p);
      card.hidden = !p && fallback;   // old shell without a plan column: stay out of the way until there is a plan
    }
    if (p && (p.phase === "running" || p.phase === "done")) {
      const bar = card.querySelector(".plan-progress i"); if (bar) bar.style.width = `${Math.round((p.progress || 0) * 100)}%`;
      const cur = p.step_i || 0;
      card.querySelectorAll(".plan-steps li").forEach((li, i) => {
        li.classList.toggle("past", i < cur || p.phase === "done");
        li.classList.toggle("now", i === cur && p.phase === "running");
      });
    }
  }

  // ------------------------------------------------------------------ 3D ghost
  let ghostKey = null;
  function ghost(p) {
    const V = window.PC3D; if (!V || !V.mounted) return;
    const show = p && p.phase !== "done" && p.phase !== "failed" && (p.waypoints || []).length;
    const key = show ? JSON.stringify([p.id, p.phase, p.waypoints, p.wp_ok, p.target, p.carry]) : null;
    if (key === ghostKey) return;
    ghostKey = key;
    const g = V.layer("plan"), T = V.THREE;
    V.disposeGroup(g);
    if (show) {
      const fade = p.phase === "running" ? 0.45 : 1;
      const OK = 0x5c77df, BAD = 0xd6453a, ok = !!p.ok;
      const pts = p.waypoints.map((w) => new T.Vector3(w[0], w[1], w[2]));
      for (let i = 0; i + 1 < pts.length; i++) {
        const good = p.wp_ok[i] !== false && p.wp_ok[i + 1] !== false;
        g.add(V.segmentMesh(pts[i], pts[i + 1], good ? 0.0045 : 0.006, good ? OK : BAD, (good ? 0.6 : 0.85) * fade));
      }
      const dot = new T.SphereGeometry(0.009, 12, 8);
      pts.forEach((v, i) => { if (p.wp_ok[i] === false) { const m = new T.Mesh(dot, new T.MeshBasicMaterial({ color: BAD, transparent: true, opacity: 0.9 * fade })); m.position.copy(v); g.add(m); } });
      (p.steps || []).forEach((s, i) => {   // numbered step badges along the path, like the card
        const v = pts[Math.min(pts.length - 1, Math.round(s.at * (pts.length - 1)))];
        const sp = V.labelSprite(String(i + 1), ok ? "#5c77df" : "#d6453a", "#ffffff", 0.026); sp.position.set(v.x, v.y, v.z + 0.03);
        sp.material.opacity = fade; g.add(sp);
      });
      if (p.target && p.target.pos) {
        const tp = p.target.pos;
        const ring = new T.Mesh(new T.RingGeometry(0.038, 0.052, 48), new T.MeshBasicMaterial({ color: ok ? OK : BAD, transparent: true, opacity: 0.85 * fade, side: T.DoubleSide, depthWrite: false }));
        ring.position.set(tp[0], tp[1], tp[2] + 0.003); g.add(ring);
        const inner = new T.Mesh(new T.CircleGeometry(0.038, 48), new T.MeshBasicMaterial({ color: ok ? OK : BAD, transparent: true, opacity: 0.12 * fade, side: T.DoubleSide, depthWrite: false }));
        inner.position.set(tp[0], tp[1], tp[2] + 0.002); g.add(inner);
      }
      const c = p.carry;
      if (c && c.to && p.phase !== "running") {   // where the object will end up
        const geo = c.shape === "cyl" ? new T.CylinderGeometry(c.r, c.r, 2 * c.half_h, 32) : new T.BoxGeometry(2 * c.r, 2 * c.r, 2 * c.half_h);
        const mesh = new T.Mesh(geo, new T.MeshBasicMaterial({ color: ok ? OK : BAD, transparent: true, opacity: 0.22, depthWrite: false }));
        if (c.shape === "cyl") mesh.rotation.x = Math.PI / 2;   // three.js cylinders stand on Y; the world is Z-up
        mesh.position.set(c.to[0], c.to[1], c.to[2]);
        const edges = new T.LineSegments(new T.EdgesGeometry(geo, 30), new T.LineBasicMaterial({ color: ok ? OK : BAD, transparent: true, opacity: 0.7 }));
        edges.position.copy(mesh.position); edges.rotation.copy(mesh.rotation);
        g.add(mesh, edges);
      }
    }
    V.redraw();
  }

  function onState(s) { const p = s && s.plan ? s.plan : null; render(p); ghost(p); }
  P.onState(onState);
  onState(P.state);
  window.addEventListener("pc3d-ready", () => { ghostKey = undefined; onState(P.state); });
  window.addEventListener("pc-examples", () => onState(P.state));   // robot switched: new example chips
})();
