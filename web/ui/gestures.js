// #G gesture legend + recognised-gesture chip on the camera panel (UX 2.0). Server: pps/features/gestures.py.
// state.gestures = {live, active, last: {id, name, label, effect}, last_age}
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
  };
  const ROWS = [
    ["stop", "Stop", "both hands above your head"],
    ["slower", "Slower", "both palms pressing down"],
    ["come_here", "Come here", "beckon with one hand"],
    ["go_on", "Go on", "thumb up, or open palm forward"],
  ];

  const css = document.createElement("style");
  css.id = "pc-gestures-css";
  css.textContent = `
.pcg-host { position: relative; }
.pcg-legend { position: absolute; left: 8px; bottom: 8px; z-index: 6; display: flex; flex-direction: column; align-items: flex-start; gap: 6px; max-width: calc(100% - 16px); pointer-events: none; }
.pcg-legend > * { pointer-events: auto; }
.pcg-toggle { display: inline-flex; align-items: center; gap: 6px; min-height: 44px; min-width: 44px; padding: 0 12px; border-radius: 999px; border: 1px solid var(--line2, rgba(0,0,0,.16));
  background: var(--panel, rgba(255,255,255,.86)); color: var(--t1, #1d1d1f); font: 600 12px var(--sans, system-ui); cursor: pointer; box-shadow: var(--shadow, 0 4px 14px rgba(0,0,0,.08)); backdrop-filter: blur(10px); }
.pcg-toggle:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.pcg-toggle .pcg-strip { display: inline-flex; gap: 2px; }
.pcg-toggle .pcg-strip span { display: inline-flex; width: 22px; height: 22px; align-items: center; justify-content: center; border-radius: 999px; color: var(--t2, rgba(29,29,31,.62)); }
.pcg-toggle .pcg-strip span.on, .pcg-row.on .pcg-ic { background: var(--accent, #5c77df); color: var(--on-accent, #fff); }
.pcg-list { margin: 0; padding: 8px; list-style: none; border-radius: 16px; background: var(--panel, rgba(255,255,255,.92)); box-shadow: var(--shadow-md, 0 10px 30px rgba(0,0,0,.12));
  backdrop-filter: blur(12px); display: grid; gap: 2px; font: 12px/1.3 var(--sans, system-ui); color: var(--t1, #1d1d1f); }
.pcg-list[hidden], .pcg-chip[hidden], .pcg-legend[hidden] { display: none; }
.pcg-row { display: grid; grid-template-columns: 26px auto; column-gap: 8px; align-items: center; padding: 3px 6px 3px 2px; border-radius: 10px; }
.pcg-row b { font-weight: 600; } .pcg-row small { grid-column: 2; color: var(--t2, rgba(29,29,31,.62)); font-size: 11px; }
.pcg-ic { grid-row: span 2; display: inline-flex; width: 26px; height: 26px; border-radius: 999px; align-items: center; justify-content: center; background: var(--accent-tint, rgba(92,119,223,.12)); color: var(--accent, #5c77df); }
.pcg-off { color: var(--t3, rgba(29,29,31,.45)); font-size: 11px; padding: 2px 6px; }
.pcg-chip { position: absolute; left: 50%; top: 10px; transform: translate(-50%, -6px); z-index: 7; display: inline-flex; align-items: center; gap: 8px; padding: 8px 14px 8px 10px;
  border-radius: 999px; background: var(--accent, #5c77df); color: var(--on-accent, #fff); font: 600 13px var(--sans, system-ui); box-shadow: var(--shadow-md, 0 10px 30px rgba(0,0,0,.18));
  opacity: 0; transition: opacity .2s, transform .2s; pointer-events: none; white-space: nowrap; max-width: calc(100% - 20px); overflow: hidden; text-overflow: ellipsis; }
.pcg-chip.in { opacity: 1; transform: translate(-50%, 0); }
.pcg-chip .pcg-eff { font-weight: 500; opacity: .9; }
@media (prefers-reduced-motion: reduce) { .pcg-chip { transition: none; } }
`;
  document.head.appendChild(css);

  function host() {
    const cf = document.getElementById("camFrame") || document.querySelector('[data-slot="camera"], .cam-frame');
    return cf || P.slot("stage");
  }
  const h = host();
  if (h.id === "camFrame" || h.classList.contains("cam-frame")) h.classList.add("pcg-host");

  const legend = document.createElement("div");
  legend.className = "pcg-legend";
  legend.innerHTML = `
    <ul class="pcg-list" id="pcgList" hidden aria-label="Gestures the robot understands">
      ${ROWS.map(([k, n, how]) => `<li class="pcg-row" data-g="${k}"><span class="pcg-ic">${svg(ICON[k], 16)}</span><b>${n}</b><small>${how}</small></li>`).join("")}
      <li class="pcg-off" id="pcgOff" hidden>Paused while I'm busy (demo, teaching or a video).</li>
    </ul>
    <button type="button" class="pcg-toggle" aria-expanded="false" aria-controls="pcgList" title="Gestures I understand">
      ${svg(ICON.legend, 16)}<span>Gestures</span><span class="pcg-strip">${ROWS.map(([k]) => `<span data-g="${k}">${svg(ICON[k], 14)}</span>`).join("")}</span>
    </button>`;
  h.appendChild(legend);
  const list = legend.querySelector(".pcg-list"), toggle = legend.querySelector(".pcg-toggle"), off = legend.querySelector("#pcgOff");
  function setOpen(open) {
    list.hidden = !open; toggle.setAttribute("aria-expanded", String(open));
    try { localStorage.setItem("pc.gestureLegend", open ? "1" : "0"); } catch (e) { /* storage blocked */ }
  }
  toggle.addEventListener("click", () => setOpen(list.hidden));
  let open0 = false;
  try { open0 = localStorage.getItem("pc.gestureLegend") === "1"; } catch (e) { /* storage blocked */ }
  setOpen(open0);

  const chip = document.createElement("div");
  chip.className = "pcg-chip"; chip.hidden = true; chip.setAttribute("role", "status"); chip.setAttribute("aria-live", "polite");
  h.appendChild(chip);
  let lastId = null, chipT = 0, hideT = 0, live = null;
  function showChip(g) {
    chip.innerHTML = `${svg(ICON[g.name] || ICON.legend, 18)}<span>${esc(g.label || g.name)}</span>${g.effect ? `<span class="pcg-eff">· ${esc(g.effect)}</span>` : ""}`;
    chip.hidden = false; void chip.offsetWidth; chip.classList.add("in");
    clearTimeout(chipT); clearTimeout(hideT);
    chipT = setTimeout(() => { chip.classList.remove("in"); hideT = setTimeout(() => { chip.hidden = true; }, 250); }, 2600);
  }
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

  P.onState((s) => {
    const g = s && s.gestures; if (!g || typeof g !== "object") return;
    const l = g.last;
    if (l && l.id != null && l.id !== lastId) {
      const first = lastId === null; lastId = l.id;
      if (!first || (g.last_age != null && g.last_age < 2)) showChip(l);
    } else if (!l && lastId === null) lastId = 0;
    if (g.live !== live) {
      live = g.live || null;
      legend.querySelectorAll("[data-g]").forEach((n) => n.classList.toggle("on", n.dataset.g === live));
    }
    off.hidden = g.active !== false;
  });
})();
