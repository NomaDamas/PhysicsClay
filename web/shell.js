// PhysicsClay UX 2.0 shell behaviour: the parts of the layout that are not app logic.
//   - robot-eye picture-in-picture over the 3D stage (copied from the robot view frames)
//   - "Robot's plan" header chip and the idle card's example chips (they fill the command box)
//   - phone: plan bottom sheet + dock height for padding
//   - a "Path" tool in the dock that opens the 3D waypoint editor (only if no feature module registered one)
// The idle plan card hides itself (CSS :has) as soon as a feature module puts its own card into the plan slot.
"use strict";
(function () {
  const $ = (id) => document.getElementById(id);
  const PC = () => window.PC || {};

  // ---------------------------------------------------------------- robot-eye PiP (3D mode)
  const robotImg = $("robotImg"), eye = $("eyePip"), frame = $("robotFrame");
  let lastEye = 0;
  function drawEye() {
    if (!eye || !robotImg || !frame.classList.contains("v3d") || !robotImg.naturalWidth) return;
    const now = performance.now(); if (now - lastEye < 120) return; lastEye = now;
    const r = eye.getBoundingClientRect(); if (!r.width) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2), w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
    if (eye.width !== w || eye.height !== h) { eye.width = w; eye.height = h; }
    const iw = robotImg.naturalWidth, ih = robotImg.naturalHeight, sc = Math.max(w / iw, h / ih); // cover
    const ctx = eye.getContext("2d");
    try { ctx.drawImage(robotImg, (w - iw * sc) / 2, (h - ih * sc) / 2, iw * sc, ih * sc); } catch (e) { /* frame revoked mid-draw */ }
  }
  if (robotImg) robotImg.addEventListener("load", drawEye);

  // ---------------------------------------------------------------- plan header chip
  const chip = $("planChip");
  let lastTask = "", taskAt = 0;
  function planStatus(s) {
    if (!s || typeof s !== "object") return ["off", "Connecting"];
    const T = s.teach && typeof s.teach === "object" ? s.teach : {};
    if (T.phase === "recording") return ["busy", "Watching you"];
    if (T.phase === "confirm") return ["wait", "Needs you"];
    if (T.phase === "practice") return ["busy", "Practicing"];
    const P = s.plan && typeof s.plan === "object" ? s.plan : null;   // UX2 plan preview (web/ui/plan.js)
    if (P) {
      const m = { proposing: ["wait", "Waiting for you"], needs_you: ["wait", "Needs you"], blocked: ["bad", "Can't do this"],
                  running: ["busy", "Working"], done: ["ok", "Done"], failed: ["bad", "Didn't finish"] }[P.phase];
      if (m) return m;
    }
    const t = s.task && typeof s.task === "object" ? s.task : null;
    if (t) {
      const key = `${t.id}|${t.status}`;
      if (key !== lastTask) { lastTask = key; taskAt = Date.now(); }
      if (t.status === "planning") return ["busy", "Thinking"];
      if (t.status === "proposed" || t.status === "asking") return ["wait", "Waiting for you"];
      if (t.status === "running") return ["busy", "Working"];
      if (Date.now() - taskAt < 5000 && t.status === "done") return ["ok", "Done"];
      if (Date.now() - taskAt < 5000 && t.status === "failed") return ["bad", "Couldn't do it"];
    }
    if (s.mode === "FOLLOW") return ["busy", "Following you"];
    if (s.mode === "PATH") return ["busy", "Following a path"];
    return ["off", "Ready"];
  }
  function renderChip(s) {
    if (!chip || chip.dataset.owner) return; // a feature module may take the chip over by setting data-owner
    const [tone, text] = planStatus(s);
    if (chip.dataset.tone !== tone) chip.dataset.tone = tone;
    if (chip.textContent !== text) chip.textContent = text;
  }
  window.addEventListener("pc-state", (e) => renderChip(e.detail));
  setInterval(() => renderChip(PC().state), 1000); // let "Done" fade back to "Waiting for you"

  // ---------------------------------------------------------------- idle card examples fill the command box
  document.querySelectorAll(".pi-ex[data-example]").forEach((b) => b.addEventListener("click", () => {
    const cmd = $("cmd"); if (!cmd) return;
    cmd.value = b.dataset.example; cmd.focus();
    try { cmd.setSelectionRange(cmd.value.length, cmd.value.length); } catch (e) { /* noop */ }
    document.body.classList.remove("sheet-open"); syncGrab();
  }));

  // ---------------------------------------------------------------- phone: plan sheet + dock height
  const grab = $("sheetGrab");
  function syncGrab() {
    if (!grab) return;
    const open = document.body.classList.contains("sheet-open");
    grab.setAttribute("aria-expanded", String(open));
    grab.querySelector(".sr").textContent = open ? "Collapse the robot's plan" : "Expand the robot's plan";
  }
  if (grab) grab.addEventListener("click", () => { document.body.classList.toggle("sheet-open"); syncGrab(); });
  document.addEventListener("keydown", (ev) => { if (ev.key === "Escape" && document.body.classList.contains("sheet-open")) { document.body.classList.remove("sheet-open"); syncGrab(); } });
  const dock = $("dock");
  if (dock && "ResizeObserver" in window) {
    new ResizeObserver(() => {
      const fixed = getComputedStyle(dock).position === "fixed";
      document.documentElement.style.setProperty("--dock-h", fixed ? dock.offsetHeight + "px" : "0px");
    }).observe(dock);
  }
  const phone = window.matchMedia ? matchMedia("(max-width: 760px)") : null;
  const cmdIn = $("cmd"), longHint = cmdIn ? cmdIn.placeholder : "";
  function onPhone() {
    if (cmdIn) cmdIn.placeholder = phone && phone.matches ? "Type or say it…" : longHint;
    if (!phone || !phone.matches) { document.body.classList.remove("sheet-open"); syncGrab(); }
  }
  if (phone && phone.addEventListener) phone.addEventListener("change", onPhone);
  onPhone();

  // ---------------------------------------------------------------- "Path" tool -> the 3D waypoint editor
  function registerPath() {
    const UI = window.PCUI; if (!UI || document.querySelector('.pcui-tool[data-tool="path"]')) return;
    UI.tool({
      id: "path", label: "Path", icon: "path", title: "Draw a route in 3D (waypoints)",
      onSelect() { if (PC().setStageView) PC().setStageView("3d"); UI.with3D((v) => v.editPath && v.editPath(true)); },
      onDeselect() { const v = window.PC3D; if (v && v.editPath && v.editing) v.editPath(false); },
    });
  }
  let syncing = false; // keep the tool button and the editor's own "Edit path" toggle in step
  window.addEventListener("pc-path-edit", (e) => {
    const UI = window.PCUI; if (syncing || !UI || !document.querySelector('.pcui-tool[data-tool="path"]')) return;
    syncing = true;
    try {
      if (!e.detail && UI.activeTool === "path") UI.selectTool(null);
      else if (e.detail && UI.activeTool !== "path") UI.selectTool("path");
    } finally { syncing = false; }
  });
  // feature modules load from /ui/modules.json after core.js; give them first claim on the "path" id
  window.addEventListener("load", () => setTimeout(registerPath, 1500));
})();
