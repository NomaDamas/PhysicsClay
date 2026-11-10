// #G hand gestures (UX 2.0): a "Gestures" button on the camera opens a guide card that says what each gesture does,
// that the camera must be on (with a button to turn it on), and lights up the gesture the robot sees right now.
// A chip on the camera confirms a recognised gesture. Server: pps/features/gestures.py.
// state.gestures = {live, active, last: {id, name, label, effect}, last_age}; state.source says whether a real camera
// feeds the server ("phone" = a browser camera, "webcam"), state.pose_ok whether a person is in view.
// The card lives in <body> (fixed, placed next to its button and kept inside the viewport; a bottom sheet on phones),
// so the small camera frame (overflow: hidden) can never clip it. Also opened from the dock's More menu.
"use strict";
(function () {
  const P = window.PCUI; if (!P) return;
  const svg = (body, size = 18) => `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;
  const ICON = {
    stop: '<path d="M7 11V6a1.5 1.5 0 0 1 3 0v4m0-5a1.5 1.5 0 0 1 3 0v5m0-4a1.5 1.5 0 0 1 3 0v6m0-3a1.5 1.5 0 0 1 3 0v4a7 7 0 0 1-7 7h-1a7 7 0 0 1-6-3.5L5 14"/>',
    slower: '<path d="M4 8h16"/><path d="M8 12v7m-3-3 3 3 3-3"/><path d="M16 12v7m-3-3 3 3 3-3"/>',
    come_here: '<path d="M20 12H7"/><path d="m12 7-5 5 5 5"/><path d="M4 5v14"/>',
    go_on: '<path d="M7 11v9H4v-9z"/><path d="M7 11l4-7a2 2 0 0 1 3 2l-1 4h5a2 2 0 0 1 2 2.3l-1.2 6A2 2 0 0 1 16.8 20H7"/>',
    legend: '<path d="M7 11V6a1.5 1.5 0 0 1 3 0v4m0-5a1.5 1.5 0 0 1 3 0v5m0-4a1.5 1.5 0 0 1 3 0v6m0-3a1.5 1.5 0 0 1 3 0v4a7 7 0 0 1-7 7h-1a7 7 0 0 1-6-3.5L5 14"/>',
    cam: '<path d="M4 8h3l2-2.5h6L17 8h3v11H4z"/><circle cx="12" cy="13.2" r="3.4"/>',
    x: '<path d="M6 6l12 12M18 6 6 18"/>',
  };
  // [id, name, how to do it, what the robot does]
  const ROWS = [
    ["stop", "Stop", "Raise both hands above your head", "I stop whatever I'm doing"],
    ["slower", "Slower", "Both palms pressing down, in front of your waist", "I slow down (×0.7 each time)"],
    ["come_here", "Come here", "Beckon toward you with one hand", "I reach toward you, with what I'm holding"],
    ["go_on", "Go on", "Thumb up, or an open palm pushed forward", "I run the proposed plan, or carry on"],
  ];
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

  const css = document.createElement("style");
  css.id = "pc-gestures-css";
  css.textContent = `
.pcg-host { position: relative; }
.pcg-toggle { position: absolute; left: 8px; bottom: 8px; z-index: 6; display: inline-flex; align-items: center; gap: 6px; min-height: 36px; padding: 0 12px 0 10px;
  border-radius: 999px; border: 0; background: var(--glass, rgba(255,255,255,.92)); color: var(--t1, #1d1d1f); font: 600 12px var(--sans, system-ui); cursor: pointer;
  box-shadow: 0 2px 8px rgba(0,0,0,.18); }
.pcg-toggle::before { content: ""; position: absolute; inset: -4px; }
.pcg-toggle:hover { background: var(--glass-strong, #fff); }
.pcg-toggle:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.pcg-toggle[aria-expanded="true"] { background: var(--inv-bg, #1d1d1f); color: var(--inv-fg, #fff); }
.pcg-toggle .pcg-live { width: 7px; height: 7px; border-radius: 50%; background: var(--idle, #9a9aa0); flex: none; }
.pcg-toggle[data-cam="on"] .pcg-live { background: var(--ok, #1b7f4b); }
.pcg-card { position: fixed; z-index: 45; width: min(340px, calc(100vw - 24px)); max-height: calc(100vh - 24px); overflow-y: auto; display: flex; flex-direction: column; gap: 10px;
  padding: 14px 14px 12px; border-radius: 20px; background: var(--raised, #fff); color: var(--t1, #1d1d1f); border: 1px solid var(--line, rgba(29,29,31,.1));
  box-shadow: var(--shadow-lg, 0 22px 48px rgba(40,30,90,.16)); font: 13px/1.4 var(--sans, system-ui); animation: rise var(--med, .26s) var(--ease, ease); }
.pcg-card[hidden] { display: none; }
.pcg-head { display: flex; align-items: center; gap: 8px; }
.pcg-head h2 { margin: 0; font-size: 15px; font-weight: 650; flex: 1; }
.pcg-x { width: 36px; height: 36px; display: grid; place-items: center; border-radius: 50%; border: 0; background: transparent; color: var(--t2, rgba(29,29,31,.66)); cursor: pointer; position: relative; }
.pcg-x::before { content: ""; position: absolute; inset: -4px; }
.pcg-x:hover { background: var(--active, #f0f0f3); color: var(--t1, #1d1d1f); }
.pcg-x:focus-visible { outline: none; box-shadow: var(--ring); }
.pcg-status { display: flex; align-items: flex-start; gap: 10px; padding: 10px 12px; border-radius: 14px; background: var(--field, #f5f5f7); }
.pcg-status i { width: 8px; height: 8px; border-radius: 50%; margin-top: 5px; flex: none; background: var(--idle, #9a9aa0); }
.pcg-status[data-s="watching"] i { background: var(--ok, #1b7f4b); box-shadow: 0 0 0 3px var(--ok-tint, rgba(31,143,85,.12)); animation: pulse 1.6s ease-in-out infinite; }
.pcg-status[data-s="nobody"] i, .pcg-status[data-s="busy"] i { background: var(--warn, #f59145); }
.pcg-status[data-s="off"] { background: var(--accent-tint, rgba(92,119,223,.12)); }
.pcg-status[data-s="off"] i { background: var(--accent, #5269d6); }
.pcg-status .pcg-st { flex: 1; min-width: 0; }
.pcg-status b { display: block; font-weight: 650; }
.pcg-status span { color: var(--t2, rgba(29,29,31,.66)); }
.pcg-cam-on { align-self: flex-start; display: inline-flex; align-items: center; gap: 6px; min-height: 36px; margin-top: 8px; padding: 0 14px; border-radius: 999px; border: 0;
  background: var(--accent, #5269d6); color: var(--on-accent, #fff); font: 600 13px var(--sans, system-ui); cursor: pointer; }
.pcg-cam-on:focus-visible { outline: none; box-shadow: var(--ring); }
.pcg-list { list-style: none; margin: 0; padding: 0; display: grid; gap: 4px; }
.pcg-row { display: grid; grid-template-columns: 34px minmax(0, 1fr); column-gap: 10px; align-items: center; padding: 8px 10px 8px 8px; border-radius: 14px;
  border: 1px solid transparent; transition: background var(--fast, .14s), border-color var(--fast, .14s); }
.pcg-ic { grid-row: span 3; display: inline-flex; width: 34px; height: 34px; border-radius: 12px; align-items: center; justify-content: center;
  background: var(--accent-tint, rgba(92,119,223,.12)); color: var(--accent-ink, #3f59c4); }
.pcg-row b { font-weight: 650; font-size: 13px; }
.pcg-row .pcg-how { color: var(--t1, #1d1d1f); font-size: 12px; }
.pcg-row .pcg-eff { color: var(--t2, rgba(29,29,31,.66)); font-size: 12px; }
.pcg-row .pcg-eff::before { content: "→ "; }
.pcg-row.on { background: var(--accent-tint, rgba(92,119,223,.12)); border-color: color-mix(in srgb, var(--accent, #5269d6) 45%, transparent); }
.pcg-row.on .pcg-ic { background: var(--accent, #5269d6); color: var(--on-accent, #fff); }
.pcg-row.fired { background: var(--ok-tint, rgba(31,143,85,.12)); border-color: color-mix(in srgb, var(--ok, #1b7f4b) 45%, transparent); }
.pcg-row .pcg-seen { display: none; margin-left: 6px; font-size: 11px; font-weight: 650; color: var(--accent-ink, #3f59c4); }
.pcg-row.on .pcg-seen { display: inline; }
.pcg-row.fired .pcg-seen { display: inline; color: var(--ok-ink, #17703f); }
.pcg-tip { margin: 0; color: var(--t2, rgba(29,29,31,.66)); font-size: 12px; line-height: 1.45; }
.pcg-chip { position: absolute; left: 50%; top: 10px; transform: translate(-50%, -6px); z-index: 7; display: inline-flex; align-items: center; gap: 8px; padding: 8px 14px 8px 10px;
  border-radius: 999px; background: var(--accent, #5269d6); color: var(--on-accent, #fff); font: 600 13px var(--sans, system-ui); box-shadow: var(--shadow-md, 0 10px 30px rgba(0,0,0,.18));
  opacity: 0; transition: opacity .2s, transform .2s; pointer-events: none; white-space: nowrap; max-width: calc(100% - 20px); overflow: hidden; text-overflow: ellipsis; }
.pcg-chip[hidden] { display: none; }
.pcg-chip.in { opacity: 1; transform: translate(-50%, 0); }
.pcg-chip .pcg-eff { font-weight: 500; opacity: .9; }
@media (max-width: 1099px) { .pcg-toggle span.pcg-lbl { display: none; } .pcg-toggle { left: 6px; bottom: 6px; padding: 0 9px; } }
@media (max-width: 760px) {
  .pcg-toggle, .pcg-chip { display: none; }
  .pcg-card { left: 8px !important; right: 8px; top: auto !important; bottom: calc(var(--dock-h, 120px) + 8px); width: auto; max-height: calc(100vh - var(--dock-h, 120px) - var(--topbar, 56px) - 24px); }
}
@media (prefers-reduced-motion: reduce) { .pcg-chip, .pcg-row { transition: none; } .pcg-card { animation: none; } }
`;
  document.head.appendChild(css);

  const host = document.getElementById("camFrame") || P.slot("stage");
  host.classList.add("pcg-host");

  // ---------------------------------------------------------------- button on the camera + the guide card
  const toggle = document.createElement("button");
  toggle.type = "button"; toggle.className = "pcg-toggle"; toggle.id = "gesturesBtn";
  toggle.setAttribute("aria-expanded", "false"); toggle.setAttribute("aria-controls", "pcgCard"); toggle.setAttribute("aria-haspopup", "dialog");
  toggle.title = "Hand gestures the robot understands";
  toggle.innerHTML = `${svg(ICON.legend, 16)}<span class="pcg-lbl">Gestures</span><span class="pcg-live" aria-hidden="true"></span>`;
  host.appendChild(toggle);

  const card = document.createElement("section");
  card.className = "pcg-card"; card.id = "pcgCard"; card.hidden = true;
  card.setAttribute("role", "dialog"); card.setAttribute("aria-labelledby", "pcgTitle");
  card.innerHTML = `
    <header class="pcg-head">${svg(ICON.legend, 18)}<h2 id="pcgTitle">Hand gestures</h2>
      <button type="button" class="pcg-x" aria-label="Close">${svg(ICON.x, 16)}</button></header>
    <div class="pcg-status" role="status" aria-live="polite"><i aria-hidden="true"></i><div class="pcg-st"><b></b><span></span>
      <button type="button" class="pcg-cam-on" hidden>${svg(ICON.cam, 16)}Turn on camera</button></div></div>
    <ul class="pcg-list" aria-label="Gestures">
      ${ROWS.map(([k, n, how, eff]) => `<li class="pcg-row" data-g="${k}"><span class="pcg-ic">${svg(ICON[k], 18)}</span>
        <span><b>${n}</b><span class="pcg-seen" data-g-seen>seen</span></span><span class="pcg-how">${how}</span><span class="pcg-eff">${eff}</span></li>`).join("")}
    </ul>
    <p class="pcg-tip">Face the camera with your upper body in view. Hold a pose still for about half a second; each gesture fires once, so lower your hands before repeating it.</p>`;
  document.body.appendChild(card);
  const statusEl = card.querySelector(".pcg-status"), stB = statusEl.querySelector("b"), stS = statusEl.querySelector(".pcg-st > span");
  const camOn = card.querySelector(".pcg-cam-on");

  function place() {
    if (card.hidden || window.matchMedia("(max-width: 760px)").matches) return;
    const a = toggle.getBoundingClientRect(), w = card.offsetWidth, h = card.offsetHeight, vw = window.innerWidth, vh = window.innerHeight;
    const anchored = a.width > 0 && a.height > 0;
    let x = anchored ? a.left : 12, y = anchored ? a.bottom + 8 : 72;
    if (anchored && y + h > vh - 12) y = Math.max(12, a.top - h - 8);      // no room below: open upwards
    x = Math.max(12, Math.min(vw - w - 12, x)); y = Math.max(12, Math.min(vh - h - 12, y));
    card.style.left = x + "px"; card.style.top = y + "px";
  }
  let lastFocus = null;
  function setOpen(open) {
    if (open === !card.hidden) { place(); return; }
    card.hidden = !open; toggle.setAttribute("aria-expanded", String(open));
    if (open) { lastFocus = document.activeElement; render(); place(); card.querySelector(".pcg-x").focus({ preventScroll: true }); }
    else if (lastFocus && document.contains(lastFocus) && lastFocus.focus) { try { lastFocus.focus({ preventScroll: true }); } catch (e) { /* noop */ } }
  }
  toggle.addEventListener("click", (ev) => { ev.stopPropagation(); setOpen(card.hidden); });
  card.querySelector(".pcg-x").addEventListener("click", () => setOpen(false));
  document.addEventListener("keydown", (ev) => { if (ev.key === "Escape" && !card.hidden) { ev.stopImmediatePropagation(); setOpen(false); } }, true);
  document.addEventListener("pointerdown", (ev) => { if (!card.hidden && !card.contains(ev.target) && !toggle.contains(ev.target) && !ev.target.closest("[data-act='gestures']")) setOpen(false); }, true);
  window.addEventListener("resize", place);
  camOn.addEventListener("click", () => {
    const c = window.PC && window.PC.camera;
    if (c && c.start) c.start(); else P.toast("Turn the camera on in Settings → Camera.");
    render();
  });
  // phones: the camera is a tiny picture-in-picture, so the dock's More menu opens the guide
  const menu = document.querySelector(".more-menu");
  if (menu && !menu.querySelector("[data-act='gestures']")) {
    const b = document.createElement("button");
    b.type = "button"; b.className = "menu-item"; b.dataset.act = "gestures"; b.setAttribute("role", "menuitem");
    b.innerHTML = `${svg(ICON.legend, 18)}Hand gestures…`;
    b.addEventListener("click", () => { const d = document.getElementById("moreMenu"); if (d) d.open = false; setOpen(true); });
    menu.insertBefore(b, menu.children[2] || null);
  }
  window.PCGestures = { open() { setOpen(true); }, close() { setOpen(false); }, get isOpen() { return !card.hidden; } };

  // ---------------------------------------------------------------- live state
  let g = {}, s = {};
  function camState() {
    const c = window.PC && window.PC.camera, src = s.source;
    const streaming = src === "phone" || src === "webcam";
    if (!streaming) {
      const starting = c && c.status && /starting/.test(c.status);
      return starting ? "starting" : "off";
    }
    if (g.active === false) return "busy";
    return s.pose_ok ? "watching" : "nobody";
  }
  const TXT = {
    off: ["Your camera is off", "The robot needs to see you to read gestures."],
    starting: ["Starting your camera…", "Allow the camera if the browser asks."],
    nobody: ["I can't see you yet", "Step back so your head, shoulders and hands are in view."],
    busy: ["Paused while I'm busy", "Gestures are ignored during the demo, teaching or a video."],
    watching: ["Watching for gestures", "Try one: the matching row lights up."],
  };
  function render() {
    const st = camState();
    toggle.dataset.cam = st === "watching" ? "on" : "off";
    if (card.hidden) return;
    statusEl.dataset.s = st;
    const [b, t] = TXT[st]; if (stB.textContent !== b) stB.textContent = b; if (stS.textContent !== t) stS.textContent = t;
    camOn.hidden = st !== "off";
    const live = st === "watching" ? g.live : null;
    card.querySelectorAll(".pcg-row").forEach((r) => r.classList.toggle("on", r.dataset.g === live && !r.classList.contains("fired")));
  }

  const chip = document.createElement("div");
  chip.className = "pcg-chip"; chip.hidden = true; chip.setAttribute("role", "status"); chip.setAttribute("aria-live", "polite");
  host.appendChild(chip);
  let lastId = null, chipT = 0, hideT = 0, firedT = 0;
  function showChip(l) {
    chip.innerHTML = `${svg(ICON[l.name] || ICON.legend, 18)}<span>${esc(l.label || l.name)}</span>${l.effect ? `<span class="pcg-eff">· ${esc(l.effect)}</span>` : ""}`;
    chip.hidden = false; void chip.offsetWidth; chip.classList.add("in");
    clearTimeout(chipT); clearTimeout(hideT);
    chipT = setTimeout(() => { chip.classList.remove("in"); hideT = setTimeout(() => { chip.hidden = true; }, 250); }, 2600);
    if (window.matchMedia("(max-width: 760px)").matches) P.toast(`Gesture: ${l.label || l.name}${l.effect ? " · " + l.effect : ""}`);
    const row = card.querySelector(`.pcg-row[data-g="${l.name}"]`);
    if (row) {
      card.querySelectorAll(".pcg-row.fired").forEach((r) => { r.classList.remove("fired"); r.querySelector("[data-g-seen]").textContent = "seen"; });
      row.classList.add("fired"); row.querySelector("[data-g-seen]").textContent = l.effect ? `done · ${l.effect}` : "done";
      clearTimeout(firedT); firedT = setTimeout(() => { row.classList.remove("fired"); row.querySelector("[data-g-seen]").textContent = "seen"; render(); }, 3000);
    }
  }

  P.onState((st) => {
    s = st || {}; g = s.gestures && typeof s.gestures === "object" ? s.gestures : {};
    const l = g.last;
    if (l && l.id != null && l.id !== lastId) {
      const first = lastId === null; lastId = l.id;
      if (!first || (g.last_age != null && g.last_age < 2)) showChip(l);
    } else if (!l && lastId === null) lastId = 0;
    render();
  });
})();
