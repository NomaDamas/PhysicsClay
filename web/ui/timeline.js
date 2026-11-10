// Teach timeline (UX 2.0 #F) — slot "timeline".
// While teaching, every step the robot understood is a card: object -> target, plus the constraint chips it will obey.
// Cards can be reordered (drag, or the move buttons / arrow keys), deleted, re-shown (record just that step again) and
// their constraint chips toggled. Every edit goes through the existing {type:"teach", cmd:"set_spec"}; "show again"
// uses {type:"timeline", op:"reshow"} (pps/features/timeline.py). Saved skills are a compact strip of cards below:
// play (task play_skill) and, for motion skills, edit a time range of one body part (task edit_skill).
"use strict";
(function () {
  if (!window.PCUI) return;
  const U = window.PCUI;

  // ------------------------------------------------------------------ icons (stroke, same frame as PCUI.icon)
  const P = {
    grip: '<circle cx="9" cy="6" r="1.2"/><circle cx="15" cy="6" r="1.2"/><circle cx="9" cy="12" r="1.2"/><circle cx="15" cy="12" r="1.2"/><circle cx="9" cy="18" r="1.2"/><circle cx="15" cy="18" r="1.2"/>',
    left: '<path d="M15 5l-7 7 7 7"/>',
    right: '<path d="M9 5l7 7-7 7"/>',
    to: '<path d="M4 12h15M13 6l6 6-6 6"/>',
    trash: '<path d="M4 7h16M10 11v6M14 11v6M6 7l1 12a2 2 0 0 0 2 2h6a2 2 0 0 0 2-2l1-12M9 7V4h6v3"/>',
    redo: '<path d="M4 8h3l2-2.5h6L17 8h3v11H4z"/><path d="M14.6 11.2a3.4 3.4 0 1 0 .6 3"/><path d="M15.4 9.6v2.2h-2.2"/>',
    play: '<path d="M8 5v14l11-7z"/>',
    edit: '<path d="M4 7h10M18 7h2M4 17h4M12 17h8"/><circle cx="16" cy="7" r="2"/><circle cx="10" cy="17" r="2"/>',
    check: '<path d="m5 12 4.5 4.5L19 7"/>',
    x: '<path d="M6 6l12 12M18 6 6 18"/>',
    dot: '<circle cx="12" cy="12" r="4"/>',
  };
  const ic = (n, s = 20) => `<svg width="${s}" height="${s}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${P[n] || ""}</svg>`;
  const esc = (x) => String(x == null ? "" : x).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const arr = (x) => (Array.isArray(x) ? x : []);
  const num = (x, d = 0) => (typeof x === "number" && isFinite(x) ? x : d);

  // ------------------------------------------------------------------ styles (existing tokens from style.css :root)
  const css = `
.tl { display: flex; flex-direction: column; gap: 10px; font-family: var(--sans); color: var(--t1); min-width: 0; }
.tl[hidden], .tl [hidden] { display: none !important; }
.tl-sec { background: var(--panel); border: 1px solid var(--line); border-radius: var(--r-card, 24px); box-shadow: var(--shadow);
  padding: 10px 12px 12px; backdrop-filter: blur(18px); -webkit-backdrop-filter: blur(18px); min-width: 0; }
.tl-head { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; min-height: 44px; }
.tl-title { font-size: var(--fs-sm, 13px); font-weight: 650; letter-spacing: .01em; white-space: nowrap; }
.tl-phase { font-size: var(--fs-xs, 12px); padding: 3px 9px; border-radius: 999px; background: var(--accent-tint); color: var(--accent); font-weight: 600; white-space: nowrap; }
.tl-phase[data-ph="recording"] { background: rgba(214, 69, 58, .12); color: var(--bad); }
.tl-phase[data-ph="done"] { background: rgba(31, 143, 85, .12); color: var(--ok); }
.tl-q { flex: 1 1 220px; min-width: 0; font-size: var(--fs-sm, 13px); color: var(--t2); line-height: 1.35;
  display: -webkit-box; -webkit-line-clamp: 2; -webkit-box-orient: vertical; overflow: hidden; }
.tl-acts { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; margin-left: auto; }
.tl-btn { display: inline-flex; align-items: center; justify-content: center; gap: 6px; min-height: 44px; min-width: 44px; padding: 0 14px;
  border-radius: 999px; border: 1px solid var(--line2); background: var(--raised); color: var(--t1); font: 600 var(--fs-sm, 13px)/1 var(--sans);
  cursor: pointer; transition: background var(--fast) var(--ease), border-color var(--fast) var(--ease); }
.tl-btn:hover { background: var(--active); }
.tl-btn:focus-visible, .tl-ib:focus-visible, .tl-chip:focus-visible, .tl-grip:focus-visible, .tl-seg button:focus-visible, .tl-track:focus-visible { outline: none; box-shadow: var(--ring); }
.tl-btn.primary { background: var(--accent); border-color: var(--accent); color: var(--on-accent); }
.tl-btn.primary:hover { filter: brightness(1.06); }
.tl-btn.rec { background: var(--bad); border-color: var(--bad); color: #fff; }
.tl-btn:disabled, .tl-ib:disabled { opacity: .38; cursor: default; }
.tl-save { display: inline-flex; gap: 6px; align-items: center; }
.tl-save input { min-height: 44px; width: 150px; border-radius: 999px; border: 1px solid var(--line2); background: var(--field); padding: 0 14px;
  font: 500 var(--fs-sm, 13px) var(--sans); color: var(--t1); }
.tl-save input:focus { outline: none; box-shadow: var(--ring); }
.tl-row { user-select: none; -webkit-user-select: none; display: flex; gap: 10px; overflow-x: auto; overscroll-behavior-x: contain; padding: 8px 2px 4px; margin: 0; list-style: none;
  scroll-snap-type: x proximity; scrollbar-width: thin; }
.tl-card { position: relative; flex: 0 0 auto; width: 236px; scroll-snap-align: start; background: var(--raised); border: 1px solid var(--line);
  border-radius: var(--r-inner, 18px); padding: 6px 8px 8px; display: flex; flex-direction: column; gap: 6px;
  transition: transform var(--med) var(--ease), box-shadow var(--fast) var(--ease), border-color var(--fast) var(--ease); }
.tl-card[data-status="running"] { border-color: var(--accent); box-shadow: 0 0 0 3px var(--accent-tint); }
.tl-card[data-status="done"] { border-color: rgba(31, 143, 85, .45); }
.tl-card[data-status="failed"] { border-color: var(--bad); box-shadow: 0 0 0 3px rgba(214, 69, 58, .12); }
.tl-card.reshow { border-color: var(--bad); border-style: dashed; }
.tl-card.dragging { transition: none; z-index: 2; box-shadow: var(--shadow-lg); cursor: grabbing; }
.tl-row.drag .tl-card:not(.dragging) { transition: transform var(--med) var(--ease); }
.tl-top { display: flex; align-items: center; gap: 4px; }
.tl-grip { display: inline-flex; align-items: center; gap: 2px; min-height: 44px; min-width: 44px; padding: 0 6px 0 2px; border: 0; background: none;
  color: var(--t3); border-radius: 12px; cursor: grab; touch-action: none; font: 700 var(--fs-md, 15px) var(--sans); }
.tl-grip b { color: var(--t1); }
.tl-grip:disabled { cursor: default; }
.tl-grip:disabled svg { visibility: hidden; }
.tl-st { margin-left: auto; font: 600 var(--fs-xs, 12px) var(--mono); color: var(--t3); display: inline-flex; align-items: center; gap: 4px; padding-right: 4px; }
.tl-card[data-status="running"] .tl-st { color: var(--accent); }
.tl-card[data-status="done"] .tl-st { color: var(--ok); }
.tl-card[data-status="failed"] .tl-st, .tl-card.reshow .tl-st { color: var(--bad); }
.tl-what { display: flex; align-items: center; gap: 6px; padding: 0 6px; font-size: var(--fs-md, 15px); font-weight: 650; min-width: 0; }
.tl-what span { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.tl-what svg { flex: none; color: var(--t3); }
.tl-what .tgt { color: var(--accent); }
.tl-chips { display: flex; flex-wrap: wrap; gap: 0 4px; padding: 0 2px; }
.tl-chip { min-height: 44px; padding: 0; border: 0; background: none; cursor: pointer; display: inline-flex; align-items: center; }
.tl-chip span { display: inline-flex; align-items: center; gap: 4px; height: 28px; padding: 0 10px 0 7px; border-radius: 999px; font: 600 var(--fs-xs, 12px) var(--sans);
  background: rgba(245, 145, 69, .14); color: #a85a14; border: 1px solid rgba(245, 145, 69, .35); }
.tl-chip[aria-pressed="false"] span { background: none; color: var(--t3); border: 1px dashed var(--line2); text-decoration: line-through; }
.tl-chip:disabled { cursor: default; }
.tl-chip:focus-visible { box-shadow: none; }
.tl-chip:focus-visible span { box-shadow: var(--ring); }
.tl-msg { font-size: var(--fs-xs, 12px); color: var(--t2); padding: 0 6px; line-height: 1.35; }
.tl-card[data-status="failed"] .tl-msg { color: var(--bad); }
.tl-tools { display: flex; gap: 2px; justify-content: space-between; border-top: 1px solid var(--divider); padding-top: 4px; }
.tl-ib { display: inline-flex; align-items: center; justify-content: center; min-width: 44px; min-height: 44px; border: 0; border-radius: 12px; background: none;
  color: var(--t2); cursor: pointer; }
.tl-ib:hover:not(:disabled) { background: var(--active); color: var(--t1); }
.tl-ib.del:hover:not(:disabled) { color: var(--bad); }
.tl-empty { flex: 0 0 auto; display: flex; align-items: center; gap: 8px; min-height: 80px; padding: 0 14px; color: var(--t2); font-size: var(--fs-sm, 13px);
  border: 1px dashed var(--line2); border-radius: var(--r-inner, 18px); }
.tl-empty .dot { color: var(--bad); animation: tl-blink 1.2s infinite; }
@keyframes tl-blink { 50% { opacity: .25; } }
.tl-sk { width: auto; min-width: 150px; max-width: 220px; flex-direction: row; align-items: center; gap: 2px; padding: 4px 4px 4px 12px; }
.tl-sk .nm { min-width: 0; flex: 1; display: flex; flex-direction: column; }
.tl-sk .nm b { font-size: var(--fs-sm, 13px); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.tl-sk .nm small { font-size: var(--fs-xs, 12px); color: var(--t3); }
.tl-sk.on { border-color: var(--accent); }
.tl-sk.playing { border-color: var(--accent); box-shadow: 0 0 0 3px var(--accent-tint); }
.tl-ib.play { color: var(--accent); }
.tl-ib[aria-expanded="true"] { background: var(--accent-tint); color: var(--accent); }
.tl-ed { margin-top: 6px; display: flex; flex-direction: column; gap: 8px; border-top: 1px solid var(--divider); padding-top: 8px; }
.tl-ed-head { display: flex; align-items: baseline; gap: 8px; font-size: var(--fs-sm, 13px); }
.tl-ed-head small { color: var(--t3); font-family: var(--mono); }
.tl-track { position: relative; height: 44px; border-radius: 12px; background: var(--field); cursor: crosshair; touch-action: none; user-select: none; -webkit-user-select: none; }
.tl-track i { position: absolute; top: 6px; bottom: 6px; border-radius: 9px; background: var(--accent-tint); border: 2px solid var(--accent); pointer-events: none; }
.tl-track b { position: absolute; bottom: 2px; transform: translateX(-50%); font: 500 10px var(--mono); color: var(--t3); pointer-events: none; }
.tl-seg { display: flex; gap: 4px; flex-wrap: wrap; }
.tl-seg button:not(.tl-btn) { min-height: 44px; padding: 0 12px; border-radius: 999px; border: 1px solid var(--line2); background: var(--raised); color: var(--t2);
  font: 600 var(--fs-xs, 12px) var(--sans); cursor: pointer; }
.tl-seg button:not(.tl-btn)[aria-pressed="true"] { background: var(--ink, #1d1d1f); border-color: var(--ink, #1d1d1f); color: #fff; }
.tl-sr { position: absolute; width: 1px; height: 1px; overflow: hidden; clip: rect(0 0 0 0); white-space: nowrap; }
.pcui-fallback-timeline { left: 50%; bottom: 150px; transform: translateX(-50%); width: min(980px, calc(100vw - 32px)); max-height: 48vh; overflow-y: auto; display: block; border-radius: var(--r-card, 24px); }
@media (max-width: 760px) { .pcui-fallback-timeline { bottom: 150px; } .tl-card { width: 210px; } .tl-q { flex-basis: 100%; order: 3; } }
@media (prefers-reduced-motion: reduce) { .tl-card, .tl-row.drag .tl-card:not(.dragging) { transition: none; } .tl-empty .dot { animation: none; } }
`;
  const style = document.createElement("style"); style.id = "pcui-timeline-css"; style.textContent = css; document.head.appendChild(style);

  // ------------------------------------------------------------------ dom
  const root = document.createElement("section");
  root.className = "tl"; root.setAttribute("aria-label", "Teach timeline and saved skills"); root.hidden = true;
  root.innerHTML = `
    <div class="tl-sec tl-teach" hidden>
      <div class="tl-head">
        <span class="tl-title">Steps</span><span class="tl-phase" data-ph="idle">ready</span>
        <span class="tl-q"></span><span class="tl-acts"></span>
      </div>
      <ol class="tl-row tl-steps" aria-label="Taught steps"></ol>
    </div>
    <div class="tl-sec tl-skills" hidden>
      <div class="tl-head"><span class="tl-title">Skills</span><span class="tl-q tl-skq">Tap play to run one (skills you showed me are proposed on this scene first). Motion skills can be edited.</span></div>
      <ul class="tl-row tl-sklist" aria-label="Saved skills"></ul>
      <div class="tl-ed" hidden></div>
    </div>
    <div class="tl-sr" aria-live="polite"></div>`;
  U.slot("timeline").appendChild(root);
  const $ = (s) => root.querySelector(s);
  const ui = { teach: $(".tl-teach"), phase: $(".tl-phase"), q: $(".tl-q"), acts: $(".tl-acts"), steps: $(".tl-steps"),
    skills: $(".tl-skills"), sklist: $(".tl-sklist"), ed: $(".tl-ed"), live: $(".tl-sr") };
  const announce = (t) => { ui.live.textContent = ""; setTimeout(() => { ui.live.textContent = t; }, 30); };

  // ------------------------------------------------------------------ helpers
  const PRETTY = { mug: "mug", red_block: "red block", blue_block: "blue block", vase: "vase" };
  const objName = (o) => PRETTY[o] || String(o || "?").replace(/_/g, " ");
  const tgtName = (t) => { t = String(t || "?"); return t.length === 1 ? `marker ${t}` : t === "there" ? "the spot you showed" : t.replace(/_/g, " "); };
  const conKey = (c) => `${c.type}|${c.object || ""}`;
  function conLabel(c) {
    const o = objName(c.object);
    return { avoid: `clear of ${o}`, keep_still: `leave ${o}`, upright: "keep upright", slow: "slowly",
      grasp_top: "grab from top", grasp_side: "grab from side" }[c.type] || String(c.type || "").replace(/_/g, " ");
  }
  const plainSteps = (steps) => arr(steps).filter((s) => s && typeof s === "object").map((s) => ({ object: s.object, target: s.target, status: s.status || "pending", message: s.message || "" }));
  const send = (m) => { const ok = U.send(m); if (!ok) U.toast("Not connected to the robot."); return ok; };
  const teachCmd = (cmd, extra) => send(Object.assign({ type: "teach", cmd }, extra || {}));
  function runTask(action, args) {
    if (window.PC && typeof window.PC.runTask === "function") return window.PC.runTask(action, args, "button");
    send(Object.assign({ type: "task", id: `tl${Date.now() % 1e7}`, action }, args));
    return Promise.resolve(null);
  }

  // ------------------------------------------------------------------ teach state (+ local edits until the server echoes them)
  let off = [];          // constraints switched off on a chip (kept so they can be switched back on)
  let order = [];        // stable chip order by key
  let pending = null;    // {steps, cons, base, t} optimistic edit
  let lastPhase = "idle";
  const specKey = (spec) => JSON.stringify(spec ? [arr(spec.steps).map((s) => [s.object, s.target, s.status]), spec.constraints] : null);

  function teachModel(s) {
    const T = s.teach && typeof s.teach === "object" ? s.teach : {}, R = s.timeline && s.timeline.reshow;
    const ph = ["idle", "recording", "confirm", "practice", "done"].includes(T.phase) ? T.phase : "idle";
    const spec = T.spec && typeof T.spec === "object" ? T.spec : null;
    if (ph !== lastPhase) {
      if ((ph === "recording" && !R) || ph === "idle") { off = []; order = []; }
      lastPhase = ph;
    }
    let steps = plainSteps(R ? R.steps : spec && spec.steps), cons = arr(R ? R.constraints : spec && spec.constraints).filter((c) => c && c.type);
    if (pending) {
      if (specKey(spec) !== pending.base || Date.now() - pending.t > 2500) pending = null;
      else { steps = pending.steps; cons = pending.cons; }
    }
    const onKeys = new Set(cons.map(conKey));
    off = off.filter((c) => !onKeys.has(conKey(c)));        // re-added by voice: no longer off
    for (const c of cons.concat(off)) if (!order.includes(conKey(c))) order.push(conKey(c));
    const all = order.map((k) => cons.find((c) => conKey(c) === k) || off.find((c) => conKey(c) === k)).filter(Boolean);
    return { ph, T, spec, R, steps, cons, chips: all.map((c) => ({ c, on: onKeys.has(conKey(c)) })),
      editable: !R && (ph === "confirm" || ph === "done") && steps.length > 0 };
  }

  function commit(steps, cons, say) {
    const T = U.state.teach || {}, spec = T.spec || {};
    const out = { goal: spec.goal, steps: steps.map((s) => ({ object: s.object, target: s.target })), constraints: cons.map((c) => Object.assign({ type: c.type }, c.object ? { object: c.object } : {})) };
    if (!teachCmd("set_spec", { spec: out })) return;
    pending = { steps: steps.map((s) => Object.assign({}, s, { status: "pending", message: "" })), cons: out.constraints, base: specKey(spec), t: Date.now() };
    if (say) announce(say);
    render(true);
  }
  function moveStep(i, j) {
    const m = teachModel(U.state); if (!m.editable || j < 0 || j >= m.steps.length || i === j) return;
    const st = m.steps.slice(); const [x] = st.splice(i, 1); st.splice(j, 0, x);
    focusAfter = { key: null, idx: j, act: lastAct };
    commit(st, m.cons, `Step moved to position ${j + 1}.`);
  }
  function deleteStep(i) {
    const m = teachModel(U.state); if (!m.editable || m.steps.length < 2) return;
    const st = m.steps.slice(); const [x] = st.splice(i, 1);
    focusAfter = { idx: Math.min(i, st.length - 1), act: "grip" };
    commit(st, m.cons, `Deleted step ${i + 1}: ${objName(x.object)} to ${tgtName(x.target)}.`);
  }
  function toggleChip(key) {
    const m = teachModel(U.state); if (!m.editable) return;
    const on = m.cons.find((c) => conKey(c) === key);
    if (on) { off.push(on); commit(m.steps, m.cons.filter((c) => c !== on), `${conLabel(on)} off.`); }
    else {
      const c = off.find((x) => conKey(x) === key); if (!c) return;
      off = off.filter((x) => x !== c);
      let cons = m.cons.slice();
      if (c.type === "grasp_top" || c.type === "grasp_side") { cons.filter((x) => x.type === "grasp_top" || x.type === "grasp_side").forEach((x) => off.push(x)); cons = cons.filter((x) => x.type !== "grasp_top" && x.type !== "grasp_side"); }
      commit(m.steps, cons.concat([c]), `${conLabel(c)} on.`);
    }
  }
  function reshow(i) {
    const m = teachModel(U.state); if (!m.editable) return;
    if (send({ type: "timeline", op: "reshow", index: i })) {
      U.say("sys", `Show me step ${i + 1} again on your camera and say what you're doing. Say “done” when finished.`);
      announce(`Recording step ${i + 1} again.`);
    }
  }

  // ------------------------------------------------------------------ "Show" tool in the dock (start / finish a demonstration)
  const showTool = { id: "show", label: "Show", icon: "show", group: "teach", title: "Show the robot a task on your camera while you explain it",
    onSelect() {
      const ph = (U.state.teach || {}).phase;
      if (ph === "recording") { teachCmd("stop"); U.say("sys", "Got it — working out what you showed me…"); }
      else if (ph !== "practice" && teachCmd("start")) U.say("sys", "Teaching started — show the task on your camera and explain it out loud. Say “done” when finished.");
      setTimeout(() => { if (U.activeTool === "show") U.selectTool(null); }, 0);
    } };
  U.tool(showTool);

  // ------------------------------------------------------------------ render: teach steps
  let lastKey = "", actsKey = "", focusAfter = null, lastAct = "grip";
  function actionsHTML(m) {
    const n = m.steps.length;
    if (m.R) return `<button type="button" class="tl-btn rec" data-a="stop">${ic("check", 18)}Done showing</button><button type="button" class="tl-btn" data-a="reshow_cancel">Keep old step</button>`;
    switch (m.ph) {
      case "recording": return `<button type="button" class="tl-btn rec" data-a="stop">${ic("check", 18)}Done showing</button><button type="button" class="tl-ib" data-a="cancel" aria-label="Cancel teaching">${ic("x")}</button>`;
      case "confirm": return `<button type="button" class="tl-btn primary" data-a="practice">${ic("play", 18)}Practice</button><button type="button" class="tl-ib" data-a="cancel" aria-label="Cancel teaching">${ic("x")}</button>`;
      case "practice": { const k = m.steps.findIndex((s) => s.status === "running"); return `<span class="tl-phase">step ${k < 0 ? n : k + 1} of ${n}</span>`; }
      case "done": return `<form class="tl-save"><input name="n" placeholder="Name this task" aria-label="Skill name" autocomplete="off"><button class="tl-btn primary" type="submit">Save</button></form><button type="button" class="tl-btn" data-a="practice">${ic("play", 18)}Again</button><button type="button" class="tl-ib" data-a="cancel" aria-label="Close teaching">${ic("x")}</button>`;
      default: return "";
    }
  }
  const PHASE = { idle: "ready", recording: "watching you", confirm: "check my steps", practice: "practicing in sim", done: "practice passed" };

  function cardHTML(m, s, i) {
    const n = m.steps.length, ed = m.editable, re = m.R && m.R.index === i;
    const u = arr(m.T.utterances), heard = re && u.length ? `“${u[u.length - 1]}”` : "";
    const st = re ? `${ic("dot", 12)}REC ${num(m.T.seconds).toFixed(0)} s` : s.status === "done" ? `${ic("check", 14)}done` : s.status === "failed" ? "failed" : s.status === "running" ? "running" : m.ph === "practice" ? "waiting" : "";
    const chips = m.chips.map(({ c, on }) => `<button type="button" class="tl-chip" data-a="chip" data-k="${esc(conKey(c))}" aria-pressed="${on}" ${ed ? "" : "disabled"}
      title="${on ? "Applies to every step. Tap to turn off." : "Off. Tap to turn back on."}"><span>${ic(on ? "check" : "x", 12)}${esc(conLabel(c))}</span></button>`).join("");
    return `<li class="tl-card${re ? " reshow" : ""}" data-i="${i}" data-status="${esc(s.status)}" aria-label="Step ${i + 1}: ${esc(objName(s.object))} to ${esc(tgtName(s.target))}">
      <div class="tl-top"><button type="button" class="tl-grip" data-a="grip" ${ed && n > 1 ? "" : "disabled"} aria-label="Step ${i + 1}. Drag, or use the left and right arrow keys, to reorder">${ic("grip", 16)}<b>${i + 1}</b></button>
        <span class="tl-st">${st}</span></div>
      <div class="tl-what"><span>${esc(objName(s.object))}</span>${ic("to", 16)}<span class="tgt">${esc(tgtName(s.target))}</span></div>
      ${chips ? `<div class="tl-chips">${chips}</div>` : ""}
      ${(heard || s.message) ? `<div class="tl-msg">${esc(heard || s.message)}</div>` : ""}
      ${ed ? `<div class="tl-tools">
        <button type="button" class="tl-ib" data-a="left" ${i === 0 ? "disabled" : ""} aria-label="Move step ${i + 1} earlier" title="Move earlier">${ic("left")}</button>
        <button type="button" class="tl-ib" data-a="right" ${i === n - 1 ? "disabled" : ""} aria-label="Move step ${i + 1} later" title="Move later">${ic("right")}</button>
        <button type="button" class="tl-ib" data-a="reshow" aria-label="Show step ${i + 1} again" title="Show this step again">${ic("redo")}</button>
        <button type="button" class="tl-ib del" data-a="delete" ${n < 2 ? "disabled" : ""} aria-label="Delete step ${i + 1}" title="Delete step">${ic("trash")}</button>
      </div>` : ""}
    </li>`;
  }

  function renderTeach(m, force) {
    const show = m.ph !== "idle" || !!m.R;
    ui.teach.hidden = !show;
    if (!show) { lastKey = actsKey = ""; return; }
    const T = m.T, secs = num(T.seconds);
    const key = JSON.stringify([m.ph, m.steps, m.chips.map((x) => [conKey(x.c), x.on]), m.R && m.R.index, m.editable, m.ph === "recording" ? Math.floor(secs) : 0, arr(T.utterances).length]);
    const q = m.ph === "practice" ? "Practicing in simulation with my own body. Cards light up as each step runs."
      : String(T.question || (m.ph === "recording" ? "Show the task on your camera and say what you're doing." : ""));
    if (ui.q.textContent !== q) { ui.q.textContent = q; ui.q.title = q; }
    if (key === lastKey && !force) return;
    if (drag) return;   // never rebuild under the pointer
    lastKey = key;
    ui.phase.dataset.ph = m.ph; ui.phase.textContent = m.R ? `showing step ${m.R.index + 1} again` : PHASE[m.ph];
    const typing = document.activeElement && document.activeElement.closest && document.activeElement.closest(".tl-save");
    const ak = JSON.stringify([m.ph, m.R && m.R.index, m.steps.length, m.steps.findIndex((s) => s.status === "running")]);
    if (!typing && (ak !== actsKey || force)) { actsKey = ak; ui.acts.innerHTML = actionsHTML(m); }
    const fa = focusAfter || rememberFocus();
    if (m.ph === "recording" && !m.R) {
      const u = arr(T.utterances), last = u.length ? `“${esc(u[u.length - 1])}”` : "watching your hands…";
      ui.steps.innerHTML = `<li class="tl-empty"><span class="dot">${ic("dot", 14)}</span><span>REC ${secs.toFixed(0)} s · ${last}</span></li>`;
    } else {
      ui.steps.innerHTML = m.steps.map((s, i) => cardHTML(m, s, i)).join("") || `<li class="tl-empty">No steps yet.</li>`;
    }
    focusAfter = null;
    if (fa) {
      const card = ui.steps.querySelector(`.tl-card[data-i="${fa.idx}"]`);
      const b = card && (card.querySelector(`[data-a="${fa.act}"]:not(:disabled)`) || card.querySelector(`[data-a="grip"]:not(:disabled)`) || card.querySelector("button:not(:disabled)"));
      if (b) b.focus({ preventScroll: false });
    }
  }
  function rememberFocus() {
    const a = document.activeElement; if (!a || !ui.steps.contains(a)) return null;
    const card = a.closest(".tl-card"); return card ? { idx: +card.dataset.i, act: a.dataset.a || "grip" } : null;
  }

  ui.acts.addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-a]"); if (!b) return;
    const a = b.dataset.a;
    if (a === "stop") { teachCmd("stop"); U.say("sys", "Got it — working out what you showed me…"); }
    else if (a === "practice") { teachCmd("practice"); U.say("sys", "Practicing in simulation with my own body…"); }
    else if (a === "cancel") { teachCmd("cancel"); U.say("sys", "Teaching closed."); }
    else if (a === "reshow_cancel") send({ type: "timeline", op: "reshow_cancel" });
  });
  ui.acts.addEventListener("submit", (ev) => {
    ev.preventDefault(); const inp = ev.target.querySelector("input"), name = (inp && inp.value || "").trim(); if (!name) { inp && inp.focus(); return; }
    teachCmd("save", { name }); U.say("sys", `Saving skill “${name}”…`); inp.value = ""; inp.blur();
  });
  ui.steps.addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-a]"); if (!b || b.disabled) return;
    const card = b.closest(".tl-card"), i = card ? +card.dataset.i : -1, a = b.dataset.a;
    lastAct = a;
    if (a === "left") moveStep(i, i - 1);
    else if (a === "right") moveStep(i, i + 1);
    else if (a === "delete") deleteStep(i);
    else if (a === "reshow") reshow(i);
    else if (a === "chip") { focusAfter = { idx: i, act: "chip" }; toggleChip(b.dataset.k); }
  });
  ui.steps.addEventListener("keydown", (ev) => {
    const b = ev.target.closest && ev.target.closest('[data-a="grip"]'); if (!b) return;
    const i = +b.closest(".tl-card").dataset.i;
    if (ev.key === "ArrowLeft" || ev.key === "ArrowRight") { ev.preventDefault(); lastAct = "grip"; moveStep(i, i + (ev.key === "ArrowLeft" ? -1 : 1)); }
    else if (ev.key === "Delete" || ev.key === "Backspace") { ev.preventDefault(); deleteStep(i); }
  });

  // drag to reorder (mouse: anywhere on the card; touch/pen: the numbered grip, so the strip still scrolls)
  let drag = null;
  ui.steps.addEventListener("pointerdown", (ev) => {
    const card = ev.target.closest(".tl-card"); if (!card || ev.button > 0) return;
    const grip = ev.target.closest('[data-a="grip"]'), other = ev.target.closest("button:not([data-a='grip'])");
    if (other || (ev.pointerType !== "mouse" && !grip)) return;
    const m = teachModel(U.state); if (!m.editable || m.steps.length < 2) return;
    const cards = [...ui.steps.querySelectorAll(".tl-card")];
    drag = { card, i: +card.dataset.i, to: +card.dataset.i, x0: ev.clientX, on: false, id: ev.pointerId, cards, rects: cards.map((c) => c.getBoundingClientRect()) };
  });
  window.addEventListener("pointermove", (ev) => {
    if (!drag || ev.pointerId !== drag.id) return;
    const dx = ev.clientX - drag.x0;
    if (!drag.on) {
      if (Math.abs(dx) < 6) return;
      drag.on = true; drag.card.classList.add("dragging"); ui.steps.classList.add("drag");
      try { drag.card.setPointerCapture(ev.pointerId); } catch (e) { /* ok */ }
    }
    ev.preventDefault();
    drag.card.style.transform = `translateX(${dx}px)`;
    const r = drag.rects[drag.i], cx = r.left + r.width / 2 + dx, w = r.width + 10;
    let to = drag.i;
    drag.rects.forEach((q, k) => { const mid = q.left + q.width / 2; if (k < drag.i && cx < mid) to = Math.min(to, k); if (k > drag.i && cx > mid) to = Math.max(to, k); });
    drag.to = to;
    drag.cards.forEach((c, k) => {
      if (k === drag.i) return;
      const shift = drag.i < to && k > drag.i && k <= to ? -w : drag.i > to && k >= to && k < drag.i ? w : 0;
      c.style.transform = shift ? `translateX(${shift}px)` : "";
    });
  }, { passive: false });
  const dragEnd = (ev) => {
    if (!drag || (ev && ev.pointerId !== drag.id)) return;
    const d = drag; drag = null;
    if (!d.on) return;
    d.cards.forEach((c) => { c.style.transform = ""; }); d.card.classList.remove("dragging"); ui.steps.classList.remove("drag");
    const suppress = (e) => { e.stopPropagation(); e.preventDefault(); };
    window.addEventListener("click", suppress, { capture: true, once: true }); setTimeout(() => window.removeEventListener("click", suppress, { capture: true }), 0);
    if (ev && ev.type === "pointercancel") { render(true); return; }
    if (d.to !== d.i) { lastAct = "grip"; moveStep(d.i, d.to); } else render(true);
  };
  window.addEventListener("pointerup", dragEnd); window.addEventListener("pointercancel", dragEnd);

  // ------------------------------------------------------------------ render: skills strip + editor
  const PARTS = [["body", "Whole body"], ["arms", "Arms"], ["left_arm", "Left arm"], ["right_arm", "Right arm"], ["legs", "Legs"]];
  const EDITS = [["faster", { speed: 1.5 }], ["slower", { speed: 0.7 }], ["bigger", { amplitude: 1.3 }], ["smaller", { amplitude: 0.75 }]];
  const ed = { name: "", dur: 0, t0: 0, t1: 0, part: "body", drag: null };
  let playing = "", playT = 0, skKey = "";
  function skillModel(s) {
    const meta = new Map(arr(s.skills_meta).filter((x) => x && x.name).map((x) => [String(x.name), num(+x.duration, 0)]));
    // whole-body motion skills (dance, video imitation) only play on bodies that can imitate; taught tasks work everywhere
    const w = s.world && s.world.robot, caps = w && Array.isArray(w.capabilities) ? w.capabilities : null;
    const canMotion = !caps || caps.includes("imitate_video");
    const out = arr(s.skills).map(String).map((name) => ({ name, motion: meta.has(name), dur: meta.get(name) || 0 })).filter((k) => canMotion || !k.motion);
    // P4: parametric skills learned from demonstrations (state.skillbook), shown for every body that can run them
    const sb = s.skillbook && typeof s.skillbook === "object" ? s.skillbook : {};
    for (const k of arr(sb.skills)) {
      if (!k || !k.name || k.ok === false || k.fits === false || out.some((x) => x.name === k.name)) continue;
      out.push({ name: k.name, param: true, kind: k.kind, describe: k.describe || "", example: !!k.example, dur: 0 });
    }
    return out;
  }
  function playState(s) {
    const t = s.task || {}, ph = (s.teach || {}).phase;
    if (!playing) return "";
    const busy = (t.action === "play_skill" && (t.status === "running" || t.status === "planning")) || ph === "practice";
    if (busy) return playing;
    if (Date.now() - playT < 1500) return playing;     // the server has not answered yet
    playing = ""; return "";
  }
  function renderSkills(s, force) {
    const list = skillModel(s), now = playState(s);
    ui.skills.hidden = !list.length;
    if (ed.name && !list.some((k) => k.name === ed.name && k.motion)) ed.name = "";
    const key = JSON.stringify([list, now, ed.name]);
    if (key !== skKey || force) {
      skKey = key;
      const keep = document.activeElement && ui.sklist.contains(document.activeElement) ? [document.activeElement.closest(".tl-card").dataset.n, document.activeElement.dataset.a] : null;
      ui.sklist.innerHTML = list.map((k) => `<li class="tl-card tl-sk${k.name === ed.name ? " on" : ""}${k.name === now ? " playing" : ""}" data-n="${esc(k.name)}">
        <span class="nm"><b title="${esc(k.param ? `${k.name}: ${k.describe}` : k.name)}">${esc(k.name.replace(/_/g, " "))}</b><small title="${esc(k.describe || "")}">${k.name === now ? "playing…" : k.param ? `${esc(String(k.kind || "").replace(/_/g, " "))}${k.example ? " · example" : ""}` : k.motion ? `motion · ${k.dur.toFixed(1)} s` : "taught task"}</small></span>
        <button type="button" class="tl-ib play" data-a="play" aria-label="Play ${esc(k.name)}" title="Play">${ic("play", 18)}</button>
        ${k.motion ? `<button type="button" class="tl-ib" data-a="edit" aria-expanded="${k.name === ed.name}" aria-label="Edit ${esc(k.name)}" title="Edit a part of this motion">${ic("edit", 18)}</button>` : ""}
      </li>`).join("");
      if (keep) { const c = [...ui.sklist.children].find((x) => x.dataset.n === keep[0]); const b = c && c.querySelector(`[data-a="${keep[1]}"]`); if (b) b.focus(); }
    }
    renderEditor(list, force);
  }
  let edKey = "";
  function renderEditor(list, force) {
    const k = list.find((x) => x.name === ed.name);
    ui.ed.hidden = !k;
    if (!k) { edKey = ""; return; }
    if (Math.abs(k.dur - ed.dur) > 1e-3) { ed.dur = k.dur; ed.t0 = 0; ed.t1 = k.dur; }
    const key = JSON.stringify([ed.name, ed.dur, ed.part]);
    if (key !== edKey || force) {
      edKey = key;
      const step = [1, 2, 5, 10, 15, 30, 60, 120].find((x) => x >= ed.dur / 6) || 300;
      let ticks = ""; for (let x = 0; ed.dur && x <= ed.dur + 1e-6; x += step) ticks += `<b style="left:${(x / ed.dur) * 100}%">${x}s</b>`;
      ui.ed.innerHTML = `<div class="tl-ed-head"><b>${esc(ed.name.replace(/_/g, " "))}</b><small class="tl-range"></small></div>
        <div class="tl-track" tabindex="0" role="slider" aria-label="Time range to edit. Drag to select, arrow keys to move it" aria-valuemin="0" aria-valuemax="${ed.dur.toFixed(1)}"><i></i>${ticks}</div>
        <div class="tl-seg" role="group" aria-label="Body part">${PARTS.map(([v, l]) => `<button type="button" data-part="${v}" aria-pressed="${ed.part === v}">${l}</button>`).join("")}</div>
        <div class="tl-seg" role="group" aria-label="Change">${EDITS.map(([v]) => `<button type="button" class="tl-btn" data-edit="${v}">${v[0].toUpperCase() + v.slice(1)}</button>`).join("")}</div>`;
    }
    const sel = ui.ed.querySelector(".tl-track i"), full = ed.t0 <= 0.001 && ed.t1 >= ed.dur - 0.001;
    if (sel && ed.dur) { sel.style.left = (ed.t0 / ed.dur) * 100 + "%"; sel.style.width = ((ed.t1 - ed.t0) / ed.dur) * 100 + "%"; }
    const txt = full ? `whole skill · ${ed.dur.toFixed(1)} s` : `${ed.t0.toFixed(1)}–${ed.t1.toFixed(1)} s of ${ed.dur.toFixed(1)} s`;
    const r = ui.ed.querySelector(".tl-range"); if (r) r.textContent = txt;
    const tr = ui.ed.querySelector(".tl-track"); if (tr) { tr.setAttribute("aria-valuetext", txt); tr.setAttribute("aria-valuenow", ed.t0.toFixed(1)); }
  }
  ui.sklist.addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-a]"); if (!b) return;
    const name = b.closest(".tl-card").dataset.n;
    const pk = skillModel(U.state).find((k) => k.name === name && k.param);
    if (b.dataset.a === "play" && pk) {      // a parametric skill: compile it on this scene and propose it (plan card)
      if (send({ type: "skill", op: "apply", name, id: `ui-sk-${Date.now() % 1e7}` })) announce(`Proposing ${name.replace(/_/g, " ")}: ${pk.describe}.`);
      return;
    }
    if (b.dataset.a === "play") { playing = name; playT = Date.now(); runTask("play_skill", { name }); announce(`Playing ${name}.`); renderSkills(U.state, true); }
    else if (b.dataset.a === "edit") {
      if (ed.name === name) ed.name = "";
      else { ed.name = name; ed.dur = -1; ed.part = "body"; }
      renderSkills(U.state, true);
    }
  });
  ui.ed.addEventListener("click", (ev) => {
    const p = ev.target.closest("[data-part]"); if (p) { ed.part = p.dataset.part; renderSkills(U.state, true); const nb = ui.ed.querySelector(`[data-part="${ed.part}"]`); if (nb) nb.focus(); return; }
    const e = ev.target.closest("[data-edit]"); if (!e || !ed.name) return;
    const a = Object.assign({ name: ed.name, part: ed.part }, EDITS.find((x) => x[0] === e.dataset.edit)[1]);
    if (!(ed.t0 <= 0.001 && ed.t1 >= ed.dur - 0.001)) { a.t0 = +ed.t0.toFixed(2); a.t1 = +ed.t1.toFixed(2); }
    Promise.resolve(runTask("edit_skill", a)).then((r) => { if (r && r.message) U.toast(r.message); });
  });
  const tAt = (tr, ev) => { const r = tr.getBoundingClientRect(); return Math.max(0, Math.min(ed.dur, ((ev.clientX - r.left) / r.width) * ed.dur)); };
  ui.ed.addEventListener("pointerdown", (ev) => {
    const tr = ev.target.closest(".tl-track"); if (!tr || !ed.dur) return;
    tr.setPointerCapture(ev.pointerId);
    const t = tAt(tr, ev), r = tr.getBoundingClientRect(), px = (x) => (x / ed.dur) * r.width, x = ev.clientX - r.left;
    if (Math.abs(x - px(ed.t0)) < 12) ed.drag = { edge: "t0" }; else if (Math.abs(x - px(ed.t1)) < 12) ed.drag = { edge: "t1" };
    else { ed.drag = { anchor: t }; ed.t0 = ed.t1 = t; }
    renderEditor(skillModel(U.state));
  });
  ui.ed.addEventListener("pointermove", (ev) => {
    const tr = ev.target.closest(".tl-track"); if (!tr || !ed.drag) return;
    const t = tAt(tr, ev);
    if (ed.drag.edge === "t0") ed.t0 = Math.min(t, ed.t1 - 0.05); else if (ed.drag.edge === "t1") ed.t1 = Math.max(t, ed.t0 + 0.05);
    else { ed.t0 = Math.min(ed.drag.anchor, t); ed.t1 = Math.max(ed.drag.anchor, t); }
    renderEditor(skillModel(U.state));
  });
  const edEnd = () => { if (!ed.drag) return; ed.drag = null; if (ed.t1 - ed.t0 < 0.1) { ed.t0 = 0; ed.t1 = ed.dur; } renderEditor(skillModel(U.state)); };
  ui.ed.addEventListener("pointerup", edEnd); ui.ed.addEventListener("pointercancel", edEnd);
  ui.ed.addEventListener("keydown", (ev) => {
    if (!ev.target.closest(".tl-track") || !ed.dur) return;
    const w0 = ed.t1 - ed.t0, d = Math.max(0.1, ed.dur / 20);
    if (ev.key === "ArrowLeft" || ev.key === "ArrowRight") {
      ev.preventDefault();
      const whole = w0 >= ed.dur - 0.001, left = ev.key === "ArrowLeft";
      const w = whole ? Math.min(ed.dur, Math.max(0.5, ed.dur / 4)) : w0;   // first press narrows "whole skill" to a window
      const base = whole ? (left ? ed.dur - w : 0) : ed.t0 + (left ? -d : d);
      ed.t0 = Math.max(0, Math.min(ed.dur - w, base)); ed.t1 = ed.t0 + w;
    } else if (ev.key === "Home" || ev.key === "Escape") { ev.preventDefault(); ed.t0 = 0; ed.t1 = ed.dur; }
    else return;
    renderEditor(skillModel(U.state));
  });

  // ------------------------------------------------------------------ state
  function render(force) {
    const s = U.state || {};
    renderTeach(teachModel(s), force);
    renderSkills(s, force);
    root.hidden = ui.teach.hidden && ui.skills.hidden;
  }
  U.onState(() => render(false));
  render(true);
  window.PCTimeline = { render, model: () => teachModel(U.state) };   // debug hook
})();
