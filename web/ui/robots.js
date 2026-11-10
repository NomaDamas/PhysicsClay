// UX 3 · Robot UX: every robot gets its own examples, dock tools and look, and you can pick robots and build scenes.
//   - the top-bar robot pill becomes a button that opens "Robot & scene": robot cards (kind, tagline, capability chips,
//     experimental badge) and a scene panel (template, objects with add / remove). Both send `world` messages and
//     show progress until state.world.rev changes (docs/WORLD.md).
//   - per robot (pps/world/prompts/<id>.json, served as /static/robots/catalog.json): idle plan-card example chips
//     (PCUI.examples, read by plan.js), dock tools filtered by the JSON `tools` and the robot's capabilities, and
//     Handover / Drive tools that ask the agent.
//   - state.world may be absent (older server): the robot is then G1 with its default capabilities, the picker lists
//     G1 and shows the others as "coming with the world update", and the scene is read-only.
// Dev: window.PC.ingest({...}) renders a fake state (this module also forwards it to PCUI.onState listeners).
"use strict";
(function () {
  const U = window.PCUI;
  if (!U) return;
  const $ = (id) => document.getElementById(id);
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const arr = (v) => (Array.isArray(v) ? v : []);

  // ------------------------------------------------------------------ vocabulary
  const ORDER = ["g1", "panda", "ur5e", "xarm7", "iiwa14", "aloha", "apollo", "stretch3", "tidybot", "spot", "h2"];
  const KIND = { humanoid: "Humanoid", arm: "Arm", bimanual: "Two arms", mobile_manipulator: "Mobile", quadruped_arm: "Legged + arm" };
  const CAP = {
    pick_place: "Pick & place", reach: "Reach", follow_path: "Paths", bimanual: "Two arms", handover: "Handover",
    imitate_video: "Video imitation", follow_me: "Follow me", teach_by_showing: "Teach by showing", balance: "Balance", drive_base: "Drives",
  };
  const G1_CAPS = ["pick_place", "reach", "follow_path", "imitate_video", "follow_me", "teach_by_showing", "balance"];
  const TEMPLATES = ["table", "kitchen", "shelf", "floor"];
  const TPL = {
    table: ["Table", "One table, robot at the near edge"],
    kitchen: ["Kitchen", "Counter with a sink side"],
    shelf: ["Shelf", "Shelf levels above a table"],
    floor: ["Floor", "Objects on the floor"],
    gym: ["Gym", "Rubber floor for lifting"],
  };
  const KINDS = ["mug", "block", "bowl", "cup", "bottle", "can", "box", "plate", "fruit"];
  const COLORS = { red: "#e0574f", orange: "#f59145", yellow: "#e8c44e", green: "#4caf6e", blue: "#4a7fe0", purple: "#9b6fd6", white: "#f4f4f6", black: "#2a2a2e", grey: "#9a9aa0" };
  const WHERE = ["left", "center", "right", "front", "back"];
  const DOCK_TOOLS = ["point", "zone", "goal", "show", "path", "handover", "drive"];   // the ones a robot JSON can switch
  // offline fallback so the UI never breaks if the catalog can't be fetched
  const FALLBACK = {
    g1: { label: "Unitree G1", kind: "humanoid", tagline: "Humanoid at the table: picks with either hand, copies you, learns by watching.",
      examples: ["Put the mug on the right", "Carry the red block over the vase to the left", "Let me show you how to stack the blocks", "Follow me"],
      tools: ["point", "zone", "goal", "show", "path"] },
  };
  let catalog = FALLBACK;

  // ------------------------------------------------------------------ icons (stroke, 24 px grid)
  const IC = {
    humanoid: '<circle cx="12" cy="4.8" r="2"/><path d="M12 7.5v7m0 0-3 6.5m3-6.5 3 6.5M6.5 10.5 12 9l5.5 1.5"/>',
    arm: '<path d="M5 20h8M9 20v-4l4-6 5 2M13 10l-2-5"/><circle cx="13" cy="10" r="1.2"/>',
    bimanual: '<path d="M3 20h6M6 20v-4l3-5 3 1.5M21 20h-6M18 20v-4l-3-5-3 1.5"/>',
    mobile_manipulator: '<rect x="4" y="14" width="13" height="4.5" rx="1.5"/><circle cx="7" cy="20" r="1.2"/><circle cx="14" cy="20" r="1.2"/><path d="M10 14V6h4l3 3"/>',
    quadruped_arm: '<path d="M5 12h11l2-3h2M7 12l-2 7M9 12l1 7M14 12l-1 7M16 12l2 7M12 12V8l4-3"/>',
    drive: '<rect x="4" y="9" width="13" height="7" rx="2"/><circle cx="7.5" cy="18" r="1.6"/><circle cx="13.5" cy="18" r="1.6"/><path d="M19 12.5h3m-1.5-1.5 1.5 1.5-1.5 1.5"/>',
    handover: '<path d="M3 15h4l3-3h4M21 9h-4l-3 3"/><path d="m11 7 3-3 3 3M13 17l-3 3-3-3"/>',
    scene: '<path d="M3 10h18M5 10v9M19 10v9"/><rect x="7" y="5.5" width="4" height="4.5" rx=".8"/><path d="M14 10V7.5a2 2 0 0 1 4 0V10"/>',
    chev: '<path d="M6 9l6 6 6-6"/>',
    plus: '<path d="M12 5v14M5 12h14"/>',
    trash: '<path d="M5 7h14M10 7V5h4v2M7 7l1 12h8l1-12"/>',
    reset: '<path d="M4 12a8 8 0 1 0 2.4-5.7M4 4v4h4"/>',
    x: '<path d="M6 6l12 12M18 6 6 18"/>',
    check: '<path d="m5 12 4.5 4.5L19 7"/>',
    table: '<path d="M3 9h18M5 9v10M19 9v10"/>',
    kitchen: '<rect x="3" y="9" width="18" height="10" rx="1.5"/><path d="M3 13h18M8 9V6h4"/>',
    shelf: '<rect x="5" y="3" width="14" height="18" rx="1.5"/><path d="M5 9h14M5 15h14"/>',
    floor: '<path d="M2 18h20M6 18l2-6h8l2 6"/><circle cx="12" cy="9" r="2"/>',
  };
  const icon = (n, size = 20) => (IC[n]
    ? `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${IC[n]}</svg>`
    : U.icon(n, size));

  // ------------------------------------------------------------------ world model (contract or fallback)
  function model(s) {
    s = s && typeof s === "object" ? s : {};
    const W = s.world && typeof s.world === "object" && s.world.robot && typeof s.world.robot === "object" ? s.world : null;
    if (W) {
      const robot = Object.assign({ capabilities: [] }, W.robot);
      const robots = arr(W.robots).filter((r) => r && r.id);
      if (!robots.some((r) => r.id === robot.id)) robots.unshift(robot);
      const scene = W.scene && typeof W.scene === "object" ? W.scene : { template: "table", objects: [] };
      return { world: true, rev: W.rev, id: robot.id, robot, robots, scene, packs: arr(W.packs).filter((p) => p && p.id), pack: W.pack || scene.pack || null,
        templates: arr(W.templates).length ? arr(W.templates).map((t) => (t && typeof t === "object" ? String(t.id || "") : String(t))).filter(Boolean) : TEMPLATES };
    }
    const robot = { id: "g1", label: "Unitree G1", kind: "humanoid", base: "legged", capabilities: G1_CAPS };
    const objects = arr(s.scene_objects).map((o) => ({ name: String(o.name || ""), kind: "", color: "", pos: o.pos, movable: !/vase|blue block/.test(String(o.name)) }));
    return { world: false, rev: null, id: "g1", robot, robots: [robot], scene: { template: "table", objects }, templates: TEMPLATES };
  }
  const caps = (m) => new Set(arr(m.robot.capabilities).length ? m.robot.capabilities : (m.id === "g1" ? G1_CAPS : []));
  function profile(m) {
    const p = catalog[m.id] || {};
    return {
      label: m.robot.label || p.label || m.id, kind: m.robot.kind || p.kind || "", tagline: p.tagline || m.robot.note || "",
      examples: arr(p.examples).filter((e) => typeof e === "string" && e.trim()),
      tools: arr(p.tools).length ? p.tools : DOCK_TOOLS,
    };
  }
  const agentOn = () => !!(U.state.agent && U.state.agent.available);

  // ------------------------------------------------------------------ talk to the robot through the normal command path
  // Submitting the command box is exactly "typing it": the Brain routes it to the LLM agent when one is available,
  // otherwise the UX 2.0 parser handles it.
  function submit(text) {
    const cmd = $("cmd"), form = $("cmdForm");
    if (!cmd || !form) return false;
    cmd.value = text;
    if (form.requestSubmit) form.requestSubmit(); else form.dispatchEvent(new Event("submit", { cancelable: true }));
    return true;
  }
  function fill(text) {
    const cmd = $("cmd"); if (!cmd) return;
    cmd.value = text; cmd.focus();
    try { cmd.setSelectionRange(text.length, text.length); } catch (e) { /* noop */ }
  }
  function understood(text) {
    if (agentOn()) return true;
    if (/\b(show you|teach|watch me|let me show)\b/i.test(text)) return true;   // teach-by-showing phrases (app.js)
    try { return !!(window.PC && window.PC.parseIntent && window.PC.parseIntent(text)); } catch (e) { return false; }
  }
  U.runExample = (text) => {
    document.body.classList.remove("sheet-open");
    if (understood(text)) { submit(text); return; }
    fill(text);
    U.toast("This one needs the AI brain. Edit it, or connect an LLM in Settings.");
  };
  function askAgent(what, text) {
    if (!agentOn()) { U.toast(`${what} needs the AI brain (LLM agent), which isn't connected. Say it step by step instead.`); return false; }
    return submit(text);
  }

  // ------------------------------------------------------------------ catalog
  fetch("/static/robots/catalog.json").then((r) => (r.ok ? r.json() : null)).then((c) => {
    if (c && typeof c === "object" && c.g1) { catalog = c; refresh(true); }
  }).catch(() => { /* offline: built-in G1 fallback */ });

  // ------------------------------------------------------------------ dev: PC.ingest also feeds PCUI.onState listeners
  (function wrapIngest() {
    const P = window.PC;
    if (!P || typeof P.ingest !== "function") { setTimeout(wrapIngest, 200); return; }
    if (P.ingest.pcuiWrapped) return;
    const orig = P.ingest;
    const wrapped = function (s) { orig.call(P, s); window.dispatchEvent(new CustomEvent("pc-state", { detail: P.state })); };
    wrapped.pcuiWrapped = true;
    P.ingest = wrapped;
  })();

  // ------------------------------------------------------------------ CSS
  const css = `
/* ---- UX 3 robot UX (web/ui/robots.js) ---- */
.rb-pill-btn { display: inline-flex; align-items: center; min-width: 0; max-width: 100%; min-height: 44px; padding: 0; border: 0; background: none; border-radius: 999px; color: inherit; }
.rb-pill-btn .status-pill { cursor: pointer; transition: background var(--fast), border-color var(--fast); padding-right: 10px; }
.rb-pill-btn:hover .status-pill { background: var(--active); }
.rb-pill-btn:focus-visible { outline: none; }
.rb-pill-btn:focus-visible .status-pill { box-shadow: var(--ring); }
.rb-pill-btn[aria-expanded="true"] .status-pill { border-color: rgba(92,119,223,.45); }
.sp-kind { font-size: 11px; font-weight: 600; color: var(--accent-ink); background: var(--accent-tint); padding: 2px 8px; border-radius: 999px; flex: none; }
.sp-kind[data-busy] { color: var(--hot-ink); background: rgba(245,145,69,.14); }
.sp-chev { width: 14px; height: 14px; flex: none; color: var(--t2); }
.status-pill[data-switching] > i { background: var(--hot) !important; animation: pulse 1s infinite; }

.rb-dlg { width: min(760px, calc(100vw - 32px)); max-height: min(86vh, 820px); padding: 0; display: none; flex-direction: column; overflow: hidden; }
.rb-dlg[open] { display: flex; }
.rb-head { display: flex; align-items: center; gap: 12px; padding: 12px 12px 12px 20px; border-bottom: 1px solid var(--divider); flex: none; }
.rb-head h2 { margin: 0; font-size: var(--fs-md); font-weight: 650; }
.rb-tabs { display: inline-flex; padding: 3px; border-radius: 999px; background: var(--field); border: 1px solid var(--line-soft); }
.rb-tab { min-height: 38px; padding: 0 16px; border: 0; border-radius: 999px; background: transparent; font-size: var(--fs-sm); font-weight: 600; color: var(--t2); }
.rb-tab[aria-selected="true"] { background: #fff; color: var(--t1); box-shadow: 0 1px 3px rgba(0,0,0,.08); }
.rb-tab:focus-visible { outline: none; box-shadow: var(--ring); }
.rb-body { padding: 16px 20px 20px; overflow-y: auto; display: flex; flex-direction: column; gap: 14px; min-height: 0; }
.rb-body[hidden] { display: none; }
.rb-body > * { flex: none; }
.rb-sec-title { margin: 4px 0 -4px; font-size: var(--fs-xs); font-weight: 650; letter-spacing: .06em; text-transform: uppercase; color: var(--t2); }
.rb-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
.rb-card { position: relative; display: flex; gap: 12px; align-items: flex-start; min-height: 44px; padding: 14px; border-radius: 18px; text-align: left;
  border: 1px solid var(--line); background: #fff; color: var(--t1); transition: border-color var(--fast), box-shadow var(--fast), background var(--fast); }
.rb-card:hover:not(:disabled) { border-color: rgba(92,119,223,.45); box-shadow: var(--shadow); }
.rb-card:focus-visible { outline: none; box-shadow: var(--ring); }
.rb-card[aria-pressed="true"] { border: 2px solid var(--accent); padding: 13px; background: linear-gradient(0deg, rgba(92,119,223,.04), rgba(92,119,223,.04)), #fff; }
.rb-card:disabled { cursor: default; background: var(--s0); border-style: dashed; }
.rb-card:disabled .rb-ico, .rb-card:disabled .rb-name, .rb-card:disabled .rb-tag { opacity: .62; }
.rb-ico { flex: none; width: 40px; height: 40px; border-radius: 12px; display: grid; place-items: center; background: var(--field); color: var(--t1); }
.rb-card[aria-pressed="true"] .rb-ico { background: var(--accent); color: #fff; }
.rb-main { min-width: 0; flex: 1; display: flex; flex-direction: column; gap: 5px; }
.rb-top { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
.rb-name { font-size: var(--fs-md); font-weight: 650; letter-spacing: -.01em; }
.rb-kind { font-size: 11px; font-weight: 600; color: var(--t2); background: var(--field); padding: 2px 8px; border-radius: 999px; }
.rb-exp { font-size: 11px; font-weight: 650; color: var(--hot-ink); background: rgba(245,145,69,.16); padding: 2px 8px; border-radius: 999px; }
.rb-tag { font-size: var(--fs-sm); color: var(--ink-2); line-height: 1.4; }
.rb-caps { display: flex; flex-wrap: wrap; gap: 4px; margin-top: 2px; }
.rb-cap { font-size: 11px; font-weight: 500; padding: 2px 8px; border-radius: 999px; border: 1px solid var(--line); color: var(--t2); }
.rb-note { font-size: var(--fs-xs); color: var(--t2); line-height: 1.4; }
.rb-note.warn { color: var(--hot-ink); }
.rb-state { margin-left: auto; display: inline-flex; align-items: center; gap: 5px; font-size: 11px; font-weight: 650; color: var(--accent-ink); }
.rb-state.busy { color: var(--hot-ink); }
.rb-spin { width: 12px; height: 12px; border-radius: 50%; border: 2px solid currentColor; border-right-color: transparent; animation: spin .8s linear infinite; }
.rb-hint { margin: 0; font-size: var(--fs-sm); color: var(--ink-2); line-height: 1.5; padding: 10px 12px; border-radius: 12px; background: var(--field); }
.rb-hint.warn { background: rgba(245,145,69,.12); color: var(--hot-ink); }

.sc-tpl { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 8px; }
.sc-tpl-btn { min-height: 76px; display: flex; flex-direction: column; align-items: flex-start; gap: 4px; padding: 10px 12px; border-radius: 14px; border: 1px solid var(--line); background: #fff; text-align: left; color: var(--t1); }
.sc-tpl-btn b { font-size: var(--fs-sm); font-weight: 650; }
.sc-tpl-btn span { font-size: 11px; color: var(--t2); line-height: 1.3; }
.sc-tpl-btn svg { color: var(--t2); }
.sc-tpl-btn:hover:not(:disabled) { border-color: rgba(92,119,223,.45); }
.sc-tpl-btn[aria-pressed="true"] { border: 2px solid var(--accent); padding: 9px 11px; }
.sc-tpl-btn[aria-pressed="true"] svg { color: var(--accent); }
.sc-tpl-btn:disabled { cursor: default; opacity: .6; }
.sc-tpl-btn:focus-visible { outline: none; box-shadow: var(--ring); }
.sc-packs { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
.sc-pack { min-height: 96px; display: flex; flex-direction: column; align-items: flex-start; gap: 4px; padding: 10px 12px; border-radius: 14px;
  border: 1px solid var(--line); background: #fff; text-align: left; color: var(--t1); cursor: pointer; }
.sc-pack b { font-size: var(--fs-sm); font-weight: 650; display: flex; align-items: center; gap: 6px; }
.sc-pack span { font-size: 11px; color: var(--t2); line-height: 1.3; display: -webkit-box; -webkit-line-clamp: 3; -webkit-box-orient: vertical; overflow: hidden; }
.sc-pack small { font-size: 11px; color: var(--t3); }
.sc-pack:hover:not(:disabled) { border-color: rgba(245,145,69,.6); }
.sc-pack[aria-pressed="true"] { border: 2px solid var(--hot, #f59145); padding: 9px 11px; }
.sc-pack:disabled { cursor: default; opacity: .6; }
.sc-pack:focus-visible { outline: none; box-shadow: var(--ring); }
@media (max-width: 640px) { .sc-packs { grid-template-columns: minmax(0, 1fr); } .sc-pack { min-height: 0; } }
.sc-list { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; border: 1px solid var(--line); border-radius: 16px; background: #fff; overflow: hidden; }
.sc-list li { display: flex; align-items: center; gap: 10px; min-height: 52px; padding: 4px 6px 4px 14px; border-top: 1px solid var(--divider); }
.sc-list li:first-child { border-top: 0; }
.sc-list li.empty { color: var(--t2); font-size: var(--fs-sm); padding: 14px; }
.sc-sw { flex: none; width: 16px; height: 16px; border-radius: 5px; border: 1px solid rgba(29,29,31,.18); background: var(--field); }
.sc-name { font-weight: 600; font-size: var(--fs-sm); min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.sc-meta { font-size: var(--fs-xs); color: var(--t2); white-space: nowrap; }
.sc-fixed { font-size: 11px; color: var(--t2); border: 1px solid var(--line); border-radius: 999px; padding: 1px 7px; }
.sc-del { margin-left: auto; width: 44px; height: 44px; flex: none; display: grid; place-items: center; border-radius: 12px; border: 0; background: transparent; color: var(--t2); }
.sc-del:hover:not(:disabled) { background: rgba(214,69,58,.08); color: var(--bad-ink); }
.sc-del:disabled { opacity: .35; cursor: default; }
.sc-del:focus-visible { outline: none; box-shadow: var(--ring); }
.sc-add { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)) auto; gap: 8px; align-items: end; padding: 12px; border-radius: 16px; background: var(--s0); border: 1px solid var(--divider); }
.sc-add label { display: flex; flex-direction: column; gap: 4px; min-width: 0; }
.sc-add label > span { font-size: var(--fs-xs); color: var(--t2); font-weight: 500; }
.sc-add select { height: 44px; padding: 0 10px; border-radius: 12px; border: 1px solid var(--line); background: #fff; font-size: var(--fs-sm); min-width: 0; }
.sc-add select:focus { outline: none; border-color: rgba(92,119,223,.55); box-shadow: var(--ring); }
.sc-add .btn { height: 44px; padding: 0 16px; }
.sc-add .btn svg, .sc-foot .btn svg { width: 16px; height: 16px; }
.sc-foot { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.sc-foot .btn { height: 44px; }
.sc-status { font-size: var(--fs-sm); color: var(--t2); display: inline-flex; align-items: center; gap: 8px; }
.sc-status.busy { color: var(--hot-ink); }
.dock .pcui-tool[hidden] { display: none !important; }
html.pc-no-teach #teachPanel, html.pc-no-follow .menu-item[data-act="follow"], html.pc-no-video .menu-item[data-act="video"], html.pc-no-video #videoBtn { display: none !important; }

@media (max-width: 760px) {
  .status-pill .sp-robot { display: inline; max-width: 112px; overflow: hidden; text-overflow: ellipsis; color: var(--t1); font-weight: 600; }
  .status-pill .sp-sub, .status-pill .sp-kind { display: none; }
  .rb-pill-btn .status-pill { padding-right: 6px; }
  .rb-dlg { width: 100vw; max-width: 100vw; max-height: 88vh; margin: auto 0 0; border-radius: 24px 24px 0 0; border-bottom: 0; }
  .rb-head { padding: 10px 8px 10px 16px; gap: 8px; }
  .rb-head h2 { display: none; }
  .rb-tabs { flex: 1; }
  .rb-tab { flex: 1; }
  .rb-body { padding: 14px 16px calc(20px + env(safe-area-inset-bottom)); }
  .rb-grid { grid-template-columns: minmax(0, 1fr); }
  .sc-tpl { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .sc-add { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  .sc-add .btn { grid-column: 1 / -1; }
}
`;
  const st = document.createElement("style"); st.id = "pcui-robots-css"; st.textContent = css; document.head.appendChild(st);

  // ------------------------------------------------------------------ top-bar pill -> button
  const pill = $("stats"), label = $("robotLabel");
  let pillBtn = null, kindEl = null;
  if (pill && pill.parentNode) {
    pillBtn = document.createElement("button");
    pillBtn.type = "button"; pillBtn.className = "rb-pill-btn"; pillBtn.id = "robotPick";
    pillBtn.setAttribute("aria-haspopup", "dialog"); pillBtn.setAttribute("aria-expanded", "false");
    pill.parentNode.insertBefore(pillBtn, pill); pillBtn.appendChild(pill);
    kindEl = document.createElement("span"); kindEl.className = "sp-kind";
    if (label) label.after(kindEl); else pill.appendChild(kindEl);
    pill.insertAdjacentHTML("beforeend", `<svg class="sp-chev" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true">${IC.chev}</svg>`);
    pillBtn.addEventListener("click", () => openDlg("robot"));
  }

  // ------------------------------------------------------------------ dialog
  const dlg = document.createElement("dialog");
  dlg.className = "rb-dlg"; dlg.id = "worldDlg"; dlg.setAttribute("aria-labelledby", "worldDlgTitle");
  dlg.innerHTML = `
    <header class="rb-head">
      <h2 id="worldDlgTitle">Robot &amp; scene</h2>
      <div class="rb-tabs" role="tablist" aria-label="Robot or scene">
        <button type="button" class="rb-tab" role="tab" id="rbTabRobot" data-tab="robot" aria-controls="rbRobot" aria-selected="true">Robot</button>
        <button type="button" class="rb-tab" role="tab" id="rbTabScene" data-tab="scene" aria-controls="rbScene" aria-selected="false">Scene</button>
      </div>
      <span class="grow"></span>
      <button type="button" class="icon-btn" data-close aria-label="Close">${icon("x", 18)}</button>
    </header>
    <div class="rb-body" id="rbRobot" role="tabpanel" aria-labelledby="rbTabRobot"></div>
    <div class="rb-body" id="rbScene" role="tabpanel" aria-labelledby="rbTabScene" hidden></div>`;
  document.body.appendChild(dlg);
  const robotPane = dlg.querySelector("#rbRobot"), scenePane = dlg.querySelector("#rbScene");
  let tab = "robot";
  function setTab(t) {
    tab = t;
    dlg.querySelectorAll(".rb-tab").forEach((b) => b.setAttribute("aria-selected", String(b.dataset.tab === t)));
    robotPane.hidden = t !== "robot"; scenePane.hidden = t !== "scene";
    render(true);
  }
  function openDlg(t) {
    setTab(t || "robot");
    if (!dlg.open) { try { dlg.showModal(); } catch (e) { dlg.setAttribute("open", ""); } }
    if (pillBtn) pillBtn.setAttribute("aria-expanded", "true");
    const q = (a, b) => dlg.querySelector(a) || dlg.querySelector(b);   // a selector list would return document order
    const first = t === "scene" ? q(".sc-tpl-btn[aria-pressed='true']", ".sc-tpl-btn") : q(".rb-card[aria-pressed='true']", ".rb-card:not(:disabled)");
    if (first) { first.focus(); requestAnimationFrame(() => first.scrollIntoView({ block: "nearest" })); }
  }
  function closeDlg() { if (dlg.open) dlg.close(); }
  dlg.addEventListener("close", () => { if (pillBtn) pillBtn.setAttribute("aria-expanded", "false"); });
  dlg.addEventListener("click", (ev) => {
    if (ev.target === dlg) { closeDlg(); return; }   // backdrop
    const b = ev.target.closest("button"); if (!b || !dlg.contains(b)) return;
    if (b.hasAttribute("data-close")) closeDlg();
    else if (b.dataset.tab) setTab(b.dataset.tab);
    else if (b.dataset.robot) pickRobot(b.dataset.robot);
    else if (b.dataset.tpl) pickTemplate(b.dataset.tpl);
    else if (b.dataset.pack) pickPack(b.dataset.pack);
    else if (b.dataset.del) removeObject(b.dataset.del);
    else if (b.dataset.sceneReset != null) resetScene();
  });
  dlg.addEventListener("keydown", (ev) => {   // arrow keys move between the two tabs
    if (!ev.target.classList || !ev.target.classList.contains("rb-tab") || !/^Arrow(Left|Right)$/.test(ev.key)) return;
    const t = tab === "robot" ? "scene" : "robot"; setTab(t); dlg.querySelector(`.rb-tab[data-tab="${t}"]`).focus();
  });

  // More menu: "Robot & scene…"
  const menu = document.querySelector("#moreMenu .more-menu");
  if (menu) {
    const item = document.createElement("button");
    item.type = "button"; item.className = "menu-item"; item.setAttribute("role", "menuitem"); item.id = "sceneMenuItem";
    item.innerHTML = `${icon("scene", 18)}Robot &amp; scene…`;
    item.addEventListener("click", () => { const d = $("moreMenu"); if (d) d.open = false; openDlg("scene"); });
    menu.insertBefore(item, menu.firstChild);
  }

  // ------------------------------------------------------------------ actions
  let pending = null;   // {kind: "robot"|"scene", id?, rev, t0, label}
  const PENDING_MS = 45000;
  function begin(kind, msg, extra) {
    const m = model(U.state);
    if (!m.world) { U.toast("Robots and scenes arrive with the world update on the server."); return false; }
    if (!U.send(msg)) { U.toast("Not connected to the robot."); return false; }
    pending = Object.assign({ kind, rev: m.rev, t0: Date.now() }, extra || {});
    render(true);
    return true;
  }
  function pickRobot(id) {
    const m = model(U.state);
    if (id === m.id || (pending && pending.kind === "robot")) return;
    const r = m.robots.find((x) => x.id === id) || {}, lab = r.label || (catalog[id] || {}).label || id;
    if (begin("robot", { type: "world", op: "set", robot: id }, { id, label: lab })) U.say("sys", `Switching to ${lab}…`);
  }
  function pickTemplate(t) {
    const m = model(U.state);
    if (t === m.scene.template || pending) return;
    const objects = arr(m.scene.objects);
    if (begin("scene", { type: "world", op: "set", scene: { template: t, objects } }, { label: `${(TPL[t] || [t])[0]} scene` })) U.say("sys", `Building the ${(TPL[t] || [t])[0].toLowerCase()} scene…`);
  }
  function pickPack(id) {
    const m = model(U.state);
    if (pending) return;
    const p = arr(m.packs).find((x) => x.id === id) || { label: id };
    if (begin("scene", { type: "world", op: "pack", name: id, id: `ui-pack-${Date.now() % 1e7}` }, { label: `Setting up the ${String(p.label).toLowerCase()}` })) U.say("sys", `Setting up the ${String(p.label).toLowerCase()}…`);
  }
  function removeObject(name) {
    if (pending) return;
    if (begin("scene", { type: "world", op: "remove_object", name }, { label: `Removing ${name.replace(/_/g, " ")}` })) U.say("sys", `Removed ${name.replace(/_/g, " ")} from the scene.`);
  }
  function resetScene() {
    if (pending) return;
    if (begin("scene", { type: "world", op: "reset" }, { label: "Resetting the scene" })) U.say("sys", "Resetting the scene…");
  }
  function addObject(kind, color, where) {
    if (pending) return;
    const m = model(U.state), names = new Set(arr(m.scene.objects).map((o) => o.name));
    const base = [color, kind].filter(Boolean).join("_");
    let name = base, k = 2;
    while (names.has(name)) name = `${base}_${k++}`;
    const object = { name, kind, color: color || null, size: null, pos: where, movable: true };
    const nice = [color, kind].filter(Boolean).join(" ");
    if (begin("scene", { type: "world", op: "add_object", object }, { label: `Adding the ${nice}` })) U.say("sys", `Adding a ${nice} on the ${where}.`);
  }
  scenePane.addEventListener("submit", (ev) => {
    ev.preventDefault();
    const f = ev.target; if (!f.classList.contains("sc-add")) return;
    addObject(f.elements.kind.value, f.elements.color.value, f.elements.where.value);
  });

  // ------------------------------------------------------------------ dock tools: Handover (bimanual) and Drive (drive_base)
  function momentary(id) { setTimeout(() => { if (U.activeTool === id) U.selectTool(null); }, 0); }
  U.tool({
    id: "handover", label: "Handover", icon: icon("handover", 18), title: "Pass an object from one arm to the other",
    onSelect() {
      momentary("handover");
      const h = U.state.holding && typeof U.state.holding === "object" ? U.state.holding : {};
      const name = (o) => String(o).replace(/_/g, " ");
      const text = h.left ? `Hand the ${name(h.left)} from the left arm to the right arm`
        : h.right ? `Hand the ${name(h.right)} from the right arm to the left arm`
          : "Pick something up with one arm and hand it over to the other arm";
      askAgent("Handover", text);
    },
  });
  U.tool({
    id: "drive", label: "Drive", icon: icon("drive", 18), title: "Drive the base: point at a spot first, then press Drive",
    onSelect() {
      momentary("drive");
      const mk = arr(U.state.markers), last = mk[mk.length - 1];
      if (!last) {
        const point = document.querySelector('.pcui-tool[data-tool="point"]:not([hidden])');
        setTimeout(() => {   // after Point's own "click a spot" toast, so this one stays
          if (point) U.selectTool("point");
          U.toast("Tap where to drive, then press Drive.");
        }, 0);
        return;
      }
      askAgent("Driving the base", `Drive the base next to marker ${last.id}, then stop`);
    },
  });

  // ------------------------------------------------------------------ per-robot dock tools, examples, extras
  const GATE = {
    show: (c) => c.has("teach_by_showing"),
    handover: (c) => c.has("bimanual") || c.has("handover"),
    drive: (c) => c.has("drive_base"),
  };
  let toolKey = "";
  function applyTools(force) {
    const m = model(U.state), c = caps(m), want = new Set(profile(m).tools);
    const host = U.slot("tools");
    const btns = [...host.querySelectorAll(".pcui-tool[data-tool]")];
    const key = JSON.stringify([m.id, [...c], [...want], btns.map((b) => b.dataset.tool)]);
    if (key === toolKey && !force) return;
    toolKey = key;
    for (const b of btns) {
      const id = b.dataset.tool;
      const show = !DOCK_TOOLS.includes(id) || (want.has(id) && (!GATE[id] || GATE[id](c)));
      if (b.hidden === !show) continue;
      b.hidden = !show;
      if (!show && U.activeTool === id) U.selectTool(null);
    }
    const root = document.documentElement.classList;
    root.toggle("pc-no-teach", !c.has("teach_by_showing"));
    root.toggle("pc-no-follow", !c.has("follow_me"));
    root.toggle("pc-no-video", !c.has("imitate_video"));
  }
  const toolHost = U.slot("tools");
  if (toolHost && "MutationObserver" in window) new MutationObserver(() => applyTools()).observe(toolHost, { childList: true });

  let exKey = "";
  function applyExamples() {
    const m = model(U.state), ex = profile(m).examples.slice(0, 4);
    const key = JSON.stringify([m.id, ex]);
    if (key === exKey) return;
    exKey = key;
    U.examples = ex.length ? ex : null;
    window.dispatchEvent(new CustomEvent("pc-examples", { detail: U.examples }));
    // the shell's own idle card (shown only when plan.js isn't loaded)
    const box = document.querySelector("#planIdle .pi-examples");
    if (box && ex.length) {
      box.innerHTML = ex.slice(0, 3).map((e) => `<button type="button" class="pi-ex" data-rb-example="${esc(e)}">“${esc(e)}”</button>`).join("");
    }
  }
  document.addEventListener("click", (ev) => {
    const b = ev.target.closest && ev.target.closest("[data-rb-example]");
    if (b) U.runExample(b.dataset.rbExample);
  });

  // ------------------------------------------------------------------ pill
  let pillKey = "";
  function applyPill() {
    if (!pill) return;
    const m = model(U.state), p = profile(m);
    const sw = pending && pending.kind === "robot" ? pending : null;
    const fixLabel = () => { if (label && label.textContent !== p.label) label.textContent = p.label; };
    fixLabel(); requestAnimationFrame(fixLabel);   // app.js writes its own label on its next frame; ours wins
    const key = JSON.stringify([m.id, p.label, p.kind, sw && sw.label]);
    if (key === pillKey) return;
    pillKey = key;
    if (kindEl) {
      kindEl.textContent = sw ? "Switching…" : (KIND[p.kind] || p.kind || "");
      kindEl.hidden = !kindEl.textContent;
      if (sw) kindEl.dataset.busy = ""; else delete kindEl.dataset.busy;
    }
    if (sw) pill.dataset.switching = ""; else delete pill.dataset.switching;
    if (pillBtn) {
      pillBtn.setAttribute("aria-label", sw ? `Switching to ${sw.label}. Open robot and scene.` : `Robot: ${p.label}${p.kind ? ", " + (KIND[p.kind] || p.kind).toLowerCase() : ""}. Change robot or scene.`);
      pillBtn.title = sw ? `Switching to ${sw.label}…` : `${p.label} · change robot or scene`;
    }
  }

  // ------------------------------------------------------------------ dialog content
  function robotCard(r, m, available) {
    const p = catalog[r.id] || {}, cur = r.id === m.id;
    const lab = r.label || p.label || r.id, kind = r.kind || p.kind || "";
    const cs = arr(r.capabilities).length ? r.capabilities : (r.id === "g1" && !m.world ? G1_CAPS : []);
    const busy = pending && pending.kind === "robot" && pending.id === r.id;
    const state = busy ? '<span class="rb-state busy"><span class="rb-spin" aria-hidden="true"></span>Switching…</span>'
      : cur ? `<span class="rb-state">${icon("check", 14)}Current</span>` : "";
    const note = !available ? '<span class="rb-note">Coming with the world update</span>'
      : r.note ? `<span class="rb-note ${r.experimental ? "warn" : ""}">${esc(r.note)}</span>` : "";
    return `<button type="button" class="rb-card" data-robot="${esc(r.id)}" aria-pressed="${cur}" ${available ? "" : "disabled"}>
      <span class="rb-ico">${icon(IC[kind] ? kind : "arm", 22)}</span>
      <span class="rb-main">
        <span class="rb-top"><span class="rb-name">${esc(lab)}</span>${kind ? `<span class="rb-kind">${esc(KIND[kind] || kind)}</span>` : ""}${r.experimental ? '<span class="rb-exp">Experimental</span>' : ""}${state}</span>
        ${p.tagline ? `<span class="rb-tag">${esc(p.tagline)}</span>` : ""}
        ${cs.length ? `<span class="rb-caps">${cs.map((c) => `<span class="rb-cap">${esc(CAP[c] || c)}</span>`).join("")}</span>` : ""}
        ${note}
      </span></button>`;
  }
  function robotsHTML(m) {
    const avail = new Map(m.robots.map((r) => [r.id, r]));
    const rank = (id) => { const i = ORDER.indexOf(id); return i < 0 ? 99 : i; };
    const on = [...avail.values()].sort((a, b) => rank(a.id) - rank(b.id));
    const off = ORDER.filter((id) => !avail.has(id) && catalog[id]).map((id) => Object.assign({ id }, catalog[id], { capabilities: [] }));
    if (!m.world) {   // older server: only G1 lives; show the rest so people know what's coming
      ORDER.filter((id) => id !== "g1" && !catalog[id]).forEach((id) => off.push({ id, label: id, kind: "" }));
    }
    const busy = pending && pending.kind === "robot";
    return `${m.world ? "" : '<p class="rb-hint">This server runs the Unitree G1 only. The other robots arrive with the world update.</p>'}
      ${busy ? `<p class="rb-hint warn" role="status">Switching to ${esc(pending.label)}… the stage reloads when it's ready.</p>` : ""}
      <div class="rb-grid">${on.map((r) => robotCard(r, m, true)).join("")}</div>
      ${off.length ? `<h3 class="rb-sec-title">${m.world ? "Not on this server" : "Coming with the world update"}</h3><div class="rb-grid">${off.map((r) => robotCard(r, m, false)).join("")}</div>` : ""}`;
  }
  function where(o) {
    if (typeof o.pos === "string") return o.pos;
    if (Array.isArray(o.pos) && o.pos.length >= 2) return `${(+o.pos[0]).toFixed(2)}, ${(+o.pos[1]).toFixed(2)} m`;
    return "";
  }
  function sceneHTML(m) {
    const objs = arr(m.scene.objects), tpl = m.scene.template || "table", busy = !!pending, ro = !m.world;
    const st = pending ? `<span class="sc-status busy" role="status"><span class="rb-spin" aria-hidden="true"></span>${esc(pending.label)}…</span>`
      : `<span class="sc-status" role="status">${objs.length} object${objs.length === 1 ? "" : "s"} · ${esc((TPL[tpl] || [tpl])[0])}</span>`;
    const opts = (list, sel, lab) => list.map((v) => `<option value="${esc(v)}" ${v === sel ? "selected" : ""}>${esc(lab ? lab(v) : v)}</option>`).join("");
    const cap = (s) => s.charAt(0).toUpperCase() + s.slice(1);
    const packs = arr(m.packs);
    const packsHTML = packs.length ? `<h3 class="rb-sec-title">Demo packs</h3>
      <div class="sc-packs">${packs.map((p) => `<button type="button" class="sc-pack" data-pack="${esc(p.id)}" aria-pressed="${p.id === m.pack}" ${ro || busy ? "disabled" : ""}>
        <b>${esc(p.label || p.id)}</b><span>${esc(p.summary || "")}</span><small>${esc((p.robots || []).length > 1 ? `${p.robot} (or ${p.robots.filter((r) => r !== p.robot).slice(0, 2).join(", ")})` : (p.robot || ""))}</small></button>`).join("")}</div>` : "";
    return `${ro ? '<p class="rb-hint">Building scenes arrives with the world update. This is the current G1 table.</p>' : '<p class="rb-hint">Or just say it: “a kitchen counter with a bowl and two cups”, or “set up a weld cell”.</p>'}
      ${packsHTML}
      <h3 class="rb-sec-title">Template</h3>
      <div class="sc-tpl">${m.templates.map((t) => `<button type="button" class="sc-tpl-btn" data-tpl="${esc(t)}" aria-pressed="${t === tpl}" ${ro || busy ? "disabled" : ""}>${icon(IC[t] ? t : "table", 20)}<b>${esc((TPL[t] || [cap(t)])[0])}</b><span>${esc((TPL[t] || [, ""])[1])}</span></button>`).join("")}</div>
      <h3 class="rb-sec-title">Objects</h3>
      <ul class="sc-list">${objs.length ? objs.map((o) => {
        const nm = String(o.name || ""), sw = COLORS[o.color] || COLORS[nm.split(/[ _]/)[0]] || "";
        const meta = [o.kind, where(o)].filter(Boolean).join(" · ");
        return `<li><span class="sc-sw" style="${sw ? `background:${sw}` : ""}" aria-hidden="true"></span><span class="sc-name">${esc(nm.replace(/_/g, " "))}</span>
          ${meta ? `<span class="sc-meta">${esc(meta)}</span>` : ""}${o.movable === false ? '<span class="sc-fixed">fixed</span>' : ""}
          <button type="button" class="sc-del" data-del="${esc(nm)}" aria-label="Remove ${esc(nm.replace(/_/g, " "))}" ${ro || busy ? "disabled" : ""}>${icon("trash", 18)}</button></li>`;
      }).join("") : '<li class="empty">No objects yet. Add one below.</li>'}</ul>
      <form class="sc-add" autocomplete="off" aria-label="Add an object">
        <label><span>Object</span><select name="kind" ${ro ? "disabled" : ""}>${opts(KINDS, "cup", cap)}</select></label>
        <label><span>Colour</span><select name="color" ${ro ? "disabled" : ""}>${opts(Object.keys(COLORS), "red", cap)}</select></label>
        <label><span>Where</span><select name="where" ${ro ? "disabled" : ""}>${opts(WHERE, "center", cap)}</select></label>
        <button type="submit" class="btn primary" ${ro || busy ? "disabled" : ""}>${icon("plus", 16)}Add</button>
      </form>
      <div class="sc-foot">${st}<span class="grow"></span><button type="button" class="btn" data-scene-reset ${ro || busy ? "disabled" : ""}>${icon("reset", 16)}Reset scene</button></div>`;
  }
  let dlgKey = "";
  function render(force) {
    if (!dlg.open && !force) return;
    const m = model(U.state);
    const key = JSON.stringify([tab, m.world, m.id, m.rev, m.robots, m.scene, m.templates, pending && [pending.kind, pending.id, pending.label], Object.keys(catalog).length]);
    if (key === dlgKey && !force) return;
    dlgKey = key;
    const pane = tab === "robot" ? robotPane : scenePane;
    // keep the add-object choices across re-renders
    const f = scenePane.querySelector(".sc-add"), keep = f ? { kind: f.elements.kind.value, color: f.elements.color.value, where: f.elements.where.value } : null;
    const focusSel = document.activeElement && pane.contains(document.activeElement) ? document.activeElement.dataset : null;
    pane.innerHTML = tab === "robot" ? robotsHTML(m) : sceneHTML(m);
    if (keep && tab === "scene") { const g = scenePane.querySelector(".sc-add"); if (g) for (const k in keep) g.elements[k].value = keep[k]; }
    if (focusSel) {   // don't drop keyboard focus when the list re-renders
      const k = Object.keys(focusSel)[0], el = k ? pane.querySelector(`[data-${k.replace(/[A-Z]/g, (c) => "-" + c.toLowerCase())}="${CSS.escape(focusSel[k])}"]`) : null;
      if (el) el.focus();
    }
  }

  // ------------------------------------------------------------------ state
  function settle(m) {
    if (!pending) return;
    if (m.world && m.rev !== pending.rev) {
      const p = pending; pending = null;
      if (p.kind === "robot") {
        if (m.id === p.id) { U.toast(`Now using ${profile(m).label}.`); U.say("sys", `Robot is now ${profile(m).label}.`); closeDlg(); }
        else U.toast(`Couldn't switch to ${p.label}${m.robot.note ? ": " + m.robot.note : "."}`);
      }
    } else if (Date.now() - pending.t0 > PENDING_MS) {
      const p = pending; pending = null;
      U.toast(`${p.kind === "robot" ? `Switching to ${p.label}` : p.label} is taking too long. Try again.`);
    }
  }
  function refresh(force) {
    const m = model(U.state);
    settle(m);
    applyPill(); applyTools(force); applyExamples(); render(force && dlg.open);
  }
  U.onState(() => refresh(false));
  setInterval(() => { if (pending) refresh(false); }, 1000);   // time out a switch even if the server goes quiet
  refresh(true);
  window.addEventListener("load", () => setTimeout(() => applyTools(true), 1700));   // after shell.js registers "path"

  U.robots = { open: openDlg, close: closeDlg, model: () => model(U.state), get catalog() { return catalog; } };
})();
