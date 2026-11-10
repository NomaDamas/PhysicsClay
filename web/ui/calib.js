// Calibration (P6, docs/calibration.md): "this table is slippery, recalibrate". A calm card in the side slot that runs
// the probe routine (swing, press, lift, slide), shows each probe's progress and the fit, and lists the fitted values
// against the simulator's defaults. Server side: pps/features/calib.py ({type: "calib", op}).
"use strict";
(function () {
  const UI = window.PCUI;
  if (!UI) return;

  const css = `
.pcc { background: var(--raised); border: 1px solid var(--line); border-radius: var(--r-inner, 18px); box-shadow: var(--shadow);
  padding: 12px 12px 10px 16px; display: flex; flex-direction: column; gap: 6px; font-family: var(--sans); color: var(--t1); }
.pcc-head { display: flex; align-items: center; gap: 8px; min-height: 44px; }
.pcc-head svg { color: var(--t2); flex: none; }
.pcc-title { font-size: var(--fs-sm, 13px); font-weight: 600; }
.pcc-chip { font-size: var(--fs-xs, 12px); color: var(--t3); padding: 2px 8px; border-radius: 999px; border: 1px solid var(--line); white-space: nowrap; }
.pcc-chip[data-s="probing"], .pcc-chip[data-s="fitting"] { color: var(--t1); border-color: var(--t3); }
.pcc-chip[data-s="applied"] { color: var(--ok); border-color: currentColor; }
.pcc-chip[data-s="failed"] { color: var(--bad); border-color: currentColor; }
.pcc-head .grow { flex: 1; }
.pcc-btn { min-height: 44px; min-width: 44px; padding: 0 12px; border-radius: 999px; border: 1px solid var(--line); background: transparent;
  color: var(--t2); font: 500 var(--fs-xs, 12px) var(--sans); cursor: pointer; }
.pcc-btn:hover:not(:disabled) { background: var(--active); color: var(--t1); }
.pcc-btn:focus-visible { outline: none; box-shadow: var(--ring); }
.pcc-btn:disabled { opacity: .45; cursor: default; }
.pcc-btn.primary { color: var(--t1); border-color: var(--t3); }
.pcc-msg { font-size: var(--fs-xs, 12px); color: var(--t2); line-height: 1.45; }
.pcc-probes { display: flex; gap: 6px; flex-wrap: wrap; margin: 2px 0; }
.pcc-probe { font-size: var(--fs-xs, 12px); color: var(--t3); display: inline-flex; align-items: center; gap: 5px; padding: 2px 8px 2px 6px;
  border-radius: 999px; background: var(--active); }
.pcc-dot { width: 7px; height: 7px; border-radius: 50%; background: var(--line); flex: none; }
.pcc-probe[data-s="running"] { color: var(--t1); }
.pcc-probe[data-s="running"] .pcc-dot { background: var(--t1); animation: pcc-pulse 1.1s ease-in-out infinite; }
.pcc-probe[data-s="done"] .pcc-dot { background: var(--ok); }
.pcc-probe[data-s="skipped"] { text-decoration: line-through; }
@keyframes pcc-pulse { 50% { opacity: .3; } }
@media (prefers-reduced-motion: reduce) { .pcc-probe[data-s="running"] .pcc-dot { animation: none; } }
.pcc-bar { height: 3px; border-radius: 3px; background: var(--active); overflow: hidden; }
.pcc-bar > i { display: block; height: 100%; background: var(--t2); width: 0; transition: width .3s; }
.pcc-tab { width: 100%; border-collapse: collapse; font-size: var(--fs-xs, 12px); font-variant-numeric: tabular-nums; }
.pcc-tab td { padding: 5px 0; border-top: 1px solid var(--divider); }
.pcc-tab td:first-child { color: var(--t2); width: 38%; }
.pcc-tab td.v { text-align: right; font-weight: 600; width: 22%; }
.pcc-tab td.d { text-align: right; color: var(--t3); width: 18%; }
.pcc-scale { position: relative; height: 6px; margin: 0 10px; border-radius: 3px; background: var(--active); }
.pcc-scale b { position: absolute; top: -2px; width: 2px; height: 10px; background: var(--t3); left: 50%; }
.pcc-scale i { position: absolute; top: -1px; width: 8px; height: 8px; border-radius: 50%; background: var(--t1); transform: translateX(-4px); }
.pcc-err { font-size: var(--fs-xs, 12px); color: var(--t3); }
.pcc-actions { display: flex; gap: 6px; flex-wrap: wrap; }
.pcc [hidden] { display: none !important; }`;
  const st = document.createElement("style"); st.id = "pcui-calib-css"; st.textContent = css; document.head.appendChild(st);

  const NAMES = { friction_scale: "Friction", mass_scale: "Object mass", damping_scale: "Joint damping", kp_scale: "Servo stiffness",
    latency_ms: "Latency", armature_scale: "Rotor inertia", frictionloss_scale: "Joint friction" };
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const fmt = (p) => (p.name === "latency_ms" ? `${Math.round(p.value)} ms` : `×${(+p.value).toFixed(2)}`);
  const dflt = (p) => (p.name === "latency_ms" ? `${Math.round(p.default)} ms` : `×${(+p.default).toFixed(2)}`);
  function pos(p) { // where the value sits in its search range (log for scales), 0..1
    const lo = +p.lo, hi = +p.hi, v = +p.value;
    if (p.name === "latency_ms" || lo <= 0) return Math.min(1, Math.max(0, (v - lo) / (hi - lo)));
    return Math.min(1, Math.max(0, Math.log(v / lo) / Math.log(hi / lo)));
  }
  function posDefault(p) { return pos({ ...p, value: p.default }); }

  let card = null, memo = "", open = false;
  function ensure() {
    if (card) return card;
    card = document.createElement("section"); card.className = "pcc"; card.setAttribute("aria-label", "Calibration");
    card.innerHTML = `<div class="pcc-head"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"
      stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 7h10M18 7h2M4 17h4M12 17h8"/><circle cx="16" cy="7" r="2"/><circle cx="10" cy="17" r="2"/></svg>
      <span class="pcc-title">Calibration</span><span class="pcc-chip" data-s="idle">idle</span><span class="grow"></span>
      <button type="button" class="pcc-btn primary" data-op="run" title="Run the probes, fit the simulator to them and apply the fit">Calibrate here</button></div>
      <div class="pcc-body"><div class="pcc-probes" aria-label="Probes"></div><div class="pcc-bar" hidden><i></i></div>
      <div class="pcc-msg" role="status" aria-live="polite"></div>
      <table class="pcc-tab" hidden><tbody></tbody></table><div class="pcc-err" hidden></div></div>
      <div class="pcc-actions">
        <button type="button" class="pcc-btn" data-op="probe" title="Run the probe routine only">Probe</button>
        <button type="button" class="pcc-btn" data-op="fit" title="Fit the simulator to the last probes">Fit</button>
        <button type="button" class="pcc-btn" data-op="apply" title="Use the fitted values in the simulator">Apply</button>
        <button type="button" class="pcc-btn" data-op="reset" title="Back to the simulator's defaults">Reset</button>
        <button type="button" class="pcc-btn" data-op="stop" title="Stop probing / fitting">Stop</button></div>`;
    card.addEventListener("click", (e) => {
      const b = e.target.closest("[data-op]"); if (!b || b.disabled) return;
      open = true;
      UI.send({ type: "calib", op: b.dataset.op, id: `calib-${b.dataset.op}-${Date.now() % 100000}` });
    });
    UI.slot("side").appendChild(card);
    return card;
  }

  function render(c) {
    const s = c.status || "idle", busy = s === "probing" || s === "fitting";
    const el = ensure();
    const quiet = s === "idle" && !(c.per_param && c.per_param.length) && !open;   // idle: just the header and its button
    el.querySelector(".pcc-body").hidden = quiet;
    el.querySelector(".pcc-actions").hidden = quiet;
    const chip = el.querySelector(".pcc-chip"); chip.dataset.s = s; chip.textContent = s;
    el.querySelector('[data-op="run"]').disabled = busy;
    el.querySelector('[data-op="probe"]').disabled = busy;
    el.querySelector('[data-op="fit"]').disabled = busy || !(c.logs && c.logs.length);
    el.querySelector('[data-op="apply"]').disabled = busy || !c.params || c.applied;
    el.querySelector('[data-op="stop"]').disabled = !busy;
    el.querySelector(".pcc-probes").innerHTML = (c.probes || []).map((p) =>
      `<span class="pcc-probe" data-s="${esc(p.status)}"><span class="pcc-dot"></span>${esc(p.label)}</span>`).join("");
    const bar = el.querySelector(".pcc-bar"), pr = c.progress;
    bar.hidden = !(s === "fitting" && pr);
    if (pr) bar.firstChild.style.width = `${Math.round(100 * (pr.stage_index + pr.gen / Math.max(1, pr.gens)) / Math.max(1, pr.stages))}%`;
    el.querySelector(".pcc-msg").textContent = c.summary && s !== "probing" && s !== "fitting" ? c.summary : (c.message || "Probe the table and objects, then fit the simulator to what the robot did.");
    const tab = el.querySelector(".pcc-tab"), rows = c.per_param || [];
    tab.hidden = !rows.length;
    tab.querySelector("tbody").innerHTML = rows.map((p) => `<tr><td>${esc(NAMES[p.name] || p.name)}</td>
      <td><div class="pcc-scale" title="search range ${esc(p.lo)} – ${esc(p.hi)}"><b style="left:${(100 * posDefault(p)).toFixed(1)}%"></b>
      <i style="left:${(100 * pos(p)).toFixed(1)}%"></i></div></td><td class="v">${fmt(p)}</td><td class="d">${dflt(p)}</td></tr>`).join("");
    const err = el.querySelector(".pcc-err"), f = c.fit;
    err.hidden = !f;
    if (f) err.textContent = `Fit error ${(+f.err_before).toFixed(2)} → ${(+f.err_after).toFixed(2)} · ${f.evals} rollouts · ${Math.round(f.seconds)} s${c.applied ? " · applied" : ""}`;
  }

  UI.onState((s) => {
    const c = s && s.calib; if (!c) return;
    const physical = (s.physics && s.physics.mode) === "dynamic";
    if (!physical) { if (card) { card.remove(); card = null; memo = ""; } return; }   // kinematic mode: nothing to calibrate
    const j = JSON.stringify(c); if (j === memo) return; memo = j;
    if (c.status === "applied" && c.summary) UI.toast(c.summary);
    render(c);
  });

  // "this table is slippery, recalibrate" / "calibrate" typed or spoken (the agent has a calibrate_here tool too)
  const RE = /\b(?:re-?)?calibrat(?:e|ion)\b|캘리브레이션|보정해/;
  window.addEventListener("pc-hear", (e) => {
    const t = String((e.detail && e.detail.text) || "").toLowerCase();
    if (!RE.test(t) || (UI.state.agent && UI.state.agent.available)) return; // the agent handles it when it's on
    open = true;
    UI.send({ type: "calib", op: "run", id: `calib-run-${Date.now() % 100000}` });
    e.preventDefault();
  });
  window.PCCalib = { render, get open() { return open; } };
})();
