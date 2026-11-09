// #G nudge pad (UX 2.0): while a task runs or a plan / marker / drawn path exists, arrows move the active target 2 cm
// and a speed control sets the motion speed. Server: pps/features/gestures.py.
// state.nudge = {active, target, kind, offset: [dx, dy, dz] metres, last: {id, message, ok}}; state.speed_scale
// sends {type: "nudge", dx, dy, dz} (robot frame: x forward, y left, z up) and {type: "speed", scale}
"use strict";
(function () {
  const P = window.PCUI; if (!P) return;
  const STEP = 0.02;
  const svg = (body, size = 20) => `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;
  const ARROW = {
    up: '<path d="M12 19V5"/><path d="m6 11 6-6 6 6"/>', down: '<path d="M12 5v14"/><path d="m6 13 6 6 6-6"/>',
    left: '<path d="M19 12H5"/><path d="m11 6-6 6 6 6"/>', right: '<path d="M5 12h14"/><path d="m13 6 6 6-6 6"/>',
    raise: '<path d="M12 20V8"/><path d="m7 13 5-5 5 5"/><path d="M5 4h14"/>', lower: '<path d="M12 4v12"/><path d="m7 11 5 5 5-5"/><path d="M5 20h14"/>',
  };
  // [key, label, dx, dy, dz, grid area]  arrows as seen from the robot's eye: up = forward (+x), left = robot's left (+y)
  const BTNS = [
    ["up", "Forward 2 cm", STEP, 0, 0, "u"], ["left", "Left 2 cm", 0, STEP, 0, "l"], ["right", "Right 2 cm", 0, -STEP, 0, "r"],
    ["down", "Back 2 cm", -STEP, 0, 0, "d"], ["raise", "Up 2 cm", 0, 0, STEP, "zu"], ["lower", "Down 2 cm", 0, 0, -STEP, "zd"],
  ];
  const SPEEDS = [[0.3, "Careful"], [0.6, "Steady"], [1.0, "Normal"]];

  const css = document.createElement("style");
  css.id = "pc-nudge-css";
  css.textContent = `
.pcn { position: absolute; right: 10px; bottom: 10px; z-index: 8; width: 212px; padding: 10px; border-radius: 20px; background: var(--panel, rgba(255,255,255,.9));
  box-shadow: var(--shadow-md, 0 10px 30px rgba(0,0,0,.14)); backdrop-filter: blur(14px); font: 12px/1.3 var(--sans, system-ui); color: var(--t1, #1d1d1f); }
.pcn[hidden] { display: none; }
.pcn.pcn-fixed { position: fixed; right: 16px; bottom: 120px; }
.pcn-head { display: flex; align-items: baseline; gap: 6px; margin: 0 2px 8px; }
.pcn-head b { font-weight: 650; } .pcn-head span { color: var(--t2, rgba(29,29,31,.62)); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.pcn-grid { display: grid; grid-template-columns: repeat(3, 44px) 8px 44px; grid-template-rows: repeat(3, 44px); gap: 4px;
  grid-template-areas: ". u . . zu" "l c r . ." ". d . . zd"; justify-content: center; }
.pcn button { display: inline-flex; align-items: center; justify-content: center; min-width: 44px; min-height: 44px; border-radius: 14px; border: 1px solid var(--line, rgba(0,0,0,.1));
  background: var(--raised, #fff); color: var(--t1, #1d1d1f); cursor: pointer; font: 600 12px var(--sans, system-ui); padding: 0; touch-action: manipulation; }
.pcn button:hover { background: var(--active, #f0f0f3); }
.pcn button:active { transform: scale(.96); }
.pcn button:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.28)); }
.pcn-c { grid-area: c; display: flex; align-items: center; justify-content: center; text-align: center; font: 600 10.5px var(--mono, ui-monospace, monospace); color: var(--t2, rgba(29,29,31,.62)); }
.pcn-speed { display: flex; gap: 4px; margin-top: 10px; }
.pcn-speed button { flex: 1; flex-direction: column; gap: 1px; min-height: 44px; border-radius: 12px; }
.pcn-speed button small { font: 500 10px var(--mono, ui-monospace, monospace); color: var(--t2, rgba(29,29,31,.62)); }
.pcn-speed button[aria-pressed="true"] { background: var(--accent, #5c77df); border-color: var(--accent, #5c77df); color: var(--on-accent, #fff); }
.pcn-speed button[aria-pressed="true"] small { color: inherit; opacity: .85; }
.pcn-msg { margin: 8px 2px 0; min-height: 15px; color: var(--t2, rgba(29,29,31,.62)); font-size: 11px; }
.pcn-msg.bad { color: var(--bad, #d6453a); }
.pcn-hint { margin: 6px 2px 0; color: var(--t3, rgba(29,29,31,.45)); font-size: 10.5px; }
@media (max-width: 760px) { .pcn { right: 8px; bottom: 8px; transform-origin: bottom right; } }
`;
  document.head.appendChild(css);

  const pad = document.createElement("section");
  pad.className = "pcn"; pad.hidden = true; pad.setAttribute("aria-label", "Nudge pad");
  pad.innerHTML = `
    <div class="pcn-head"><b>Nudge</b><span class="pcn-target">the target</span></div>
    <div class="pcn-grid">
      ${BTNS.map(([k, label, dx, dy, dz, area]) => `<button type="button" style="grid-area:${area}" data-dx="${dx}" data-dy="${dy}" data-dz="${dz}" aria-label="${label}" title="${label}">${svg(ARROW[k])}</button>`).join("")}
      <div class="pcn-c" aria-live="polite">0 cm</div>
    </div>
    <div class="pcn-speed" role="group" aria-label="Speed">
      ${SPEEDS.map(([v, l]) => `<button type="button" data-speed="${v}" aria-pressed="false">${l}<small>×${v.toFixed(1)}</small></button>`).join("")}
    </div>
    <p class="pcn-msg" role="status"></p>
    <p class="pcn-hint">Arrows as the robot sees the table.</p>`;

  function mount() {
    const stage = document.querySelector('[data-slot="stage"]');
    const host = stage || document.getElementById("robotFrame") || document.querySelector(".robot-frame");
    if (host) { host.appendChild(pad); if (getComputedStyle(host).position === "static") host.style.position = "relative"; }
    else { pad.classList.add("pcn-fixed"); P.slot("stage").appendChild(pad); }
  }
  mount();

  const targetEl = pad.querySelector(".pcn-target"), offEl = pad.querySelector(".pcn-c"), msgEl = pad.querySelector(".pcn-msg");
  pad.querySelectorAll("button[data-dx]").forEach((b) => b.addEventListener("click", () => {
    if (!P.send({ type: "nudge", dx: +b.dataset.dx, dy: +b.dataset.dy, dz: +b.dataset.dz })) P.toast("Offline: can't nudge right now.");
  }));
  pad.querySelectorAll("button[data-speed]").forEach((b) => b.addEventListener("click", () => {
    if (!P.send({ type: "speed", scale: +b.dataset.speed })) P.toast("Offline: can't change speed right now.");
  }));

  const cm = (o) => {
    const n = [["forward", "back"], ["left", "right"], ["up", "down"]];
    const parts = (o || []).map((v, k) => (Math.abs(v) > 1e-4 ? `${Math.round(Math.abs(v) * 100)} ${n[k][v > 0 ? 0 : 1]}` : null)).filter(Boolean);
    return parts.length ? parts.join(" · ") : "0 cm";
  };
  let lastMsgId = null, msgT = 0;
  P.onState((s) => {
    const n = s && s.nudge; if (!n || typeof n !== "object") return;
    pad.hidden = !n.active;
    const z = n.kind !== "marker";
    pad.querySelectorAll('button[data-dz]:not([data-dz="0"])').forEach((b) => { b.hidden = !z; });
    targetEl.textContent = n.target || "the target";
    offEl.textContent = cm(n.offset);
    const sp = Number(s.speed_scale);
    pad.querySelectorAll("button[data-speed]").forEach((b) => b.setAttribute("aria-pressed", String(Math.abs(+b.dataset.speed - sp) < 0.051)));
    const l = n.last;
    if (l && l.id !== lastMsgId) {
      const first = lastMsgId === null; lastMsgId = l.id;
      if (!first) {
        msgEl.textContent = l.message || ""; msgEl.classList.toggle("bad", !l.ok);
        clearTimeout(msgT); msgT = setTimeout(() => { msgEl.textContent = ""; }, 4000);
      }
    }
  });
})();
