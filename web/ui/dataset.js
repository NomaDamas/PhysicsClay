// v1 P1 · Datasets (pps/features/dataset.py). A record control in the top bar: a dot, "Record" / "REC · 3", and a
// small panel with the dataset name, start / stop, the episodes recorded so far (task, success, length) and
// "Discard last". While recording, every task the robot runs becomes a LeRobot episode in datasets/<name>/<robot>/.
// Messages: {type: "dataset", op: "start" | "stop" | "discard_last", name?, auto: true}. State: state.dataset.
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

  const css = document.createElement("style");
  css.id = "pc-dataset-css";
  css.textContent = `
.ds-wrap { position: relative; flex: none; }
.ds-btn { display: inline-flex; align-items: center; gap: 8px; height: 36px; padding: 0 14px 0 12px; border-radius: 999px; border: 1px solid var(--line, rgba(29,29,31,.1));
  background: var(--raised, #fff); color: var(--t1, #1d1d1f); font: 500 var(--fs-sm, 13px) var(--sans, system-ui); cursor: pointer; white-space: nowrap;
  transition: background var(--fast, .14s); font-variant-numeric: tabular-nums; }
.ds-btn:hover { background: var(--active, #f0f0f3); }
.ds-btn:focus-visible, .ds-pop button:focus-visible, .ds-pop input:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.32)); }
.ds-btn i { width: 9px; height: 9px; border-radius: 50%; background: var(--idle, #9a9aa0); flex: none; }
.ds-btn[data-rec="1"] { border-color: color-mix(in srgb, var(--bad) 35%, transparent); }
.ds-btn[data-rec="1"] i { background: var(--bad, #d6453a); box-shadow: 0 0 0 3px color-mix(in srgb, var(--bad) 16%, transparent); animation: pulse 1.4s ease-in-out infinite; }
.ds-btn b { font-weight: 700; letter-spacing: .04em; color: var(--bad-ink, #b3342b); }
.ds-btn small { color: var(--t2, rgba(29,29,31,.64)); font-size: var(--fs-xs, 12px); }
.ds-pop { position: absolute; right: 0; top: calc(100% + 8px); width: 340px; max-width: calc(100vw - 32px); z-index: 40; padding: 14px; border-radius: 18px;
  background: var(--panel, #fff); border: 1px solid var(--line, rgba(29,29,31,.1)); box-shadow: var(--shadow-md, 0 22px 48px rgba(40,30,90,.12));
  font: 400 var(--fs-sm, 13px) var(--sans, system-ui); color: var(--t1, #1d1d1f); display: flex; flex-direction: column; gap: 10px; }
.ds-pop[hidden] { display: none; }
.ds-head { display: flex; align-items: baseline; gap: 8px; }
.ds-head strong { font-size: var(--fs-md, 15px); font-weight: 650; }
.ds-head span { color: var(--t2, rgba(29,29,31,.64)); font-size: var(--fs-xs, 12px); margin-left: auto; font-variant-numeric: tabular-nums; }
.ds-note { color: var(--t2, rgba(29,29,31,.64)); font-size: var(--fs-xs, 12px); line-height: 1.45; }
.ds-row { display: flex; gap: 8px; }
.ds-row input { flex: 1 1 auto; min-width: 0; height: 36px; padding: 0 12px; border-radius: 10px; border: 1px solid var(--line2, rgba(29,29,31,.16));
  background: var(--field, #f5f5f7); color: var(--t1, #1d1d1f); font: 500 var(--fs-sm, 13px) var(--mono, ui-monospace, monospace); }
.ds-row input:disabled { opacity: .7; }
.ds-go { flex: none; height: 36px; padding: 0 14px; border-radius: 999px; border: 0; background: var(--accent, #5c77df); color: var(--on-accent, #fff);
  font: 600 var(--fs-sm, 13px) var(--sans, system-ui); cursor: pointer; }
.ds-go[data-rec="1"] { background: var(--bad, #d6453a); }
.ds-cur { display: flex; align-items: center; gap: 8px; padding: 8px 10px; border-radius: 10px; background: var(--bad-tint); font-size: var(--fs-xs, 12px); }
.ds-cur[hidden] { display: none; }
.ds-cur i { width: 7px; height: 7px; border-radius: 50%; background: var(--bad, #d6453a); flex: none; animation: pulse 1.2s infinite; }
.ds-cur span { flex: 1 1 auto; min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.ds-list { list-style: none; margin: 0; padding: 0; max-height: 220px; overflow-y: auto; border-top: 1px solid var(--line-soft, rgba(29,29,31,.08)); }
.ds-list li { display: grid; grid-template-columns: 28px 16px minmax(0, 1fr) auto; align-items: center; gap: 6px; padding: 7px 2px;
  border-bottom: 1px solid var(--line-soft, rgba(29,29,31,.08)); font-size: var(--fs-xs, 12px); }
.ds-list .n { color: var(--t3, rgba(29,29,31,.6)); font-variant-numeric: tabular-nums; }
.ds-list .ok { color: var(--ok-ink, #17703f); font-weight: 700; }
.ds-list .bad { color: var(--bad-ink, #b3342b); font-weight: 700; }
.ds-list .t { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.ds-list .d { color: var(--t2, rgba(29,29,31,.64)); font-variant-numeric: tabular-nums; }
.ds-list .ds-empty { display: block; color: var(--t2, rgba(29,29,31,.64)); padding: 10px 2px; border: 0; }
.ds-foot { display: flex; align-items: center; gap: 8px; }
.ds-foot span { color: var(--t2, rgba(29,29,31,.64)); font-size: var(--fs-xs, 12px); flex: 1 1 auto; min-width: 0; }
.ds-foot button { flex: none; height: 32px; padding: 0 12px; border-radius: 999px; border: 1px solid var(--line2, rgba(29,29,31,.16)); background: var(--raised, #fff);
  color: var(--t1, #1d1d1f); font: 600 12px var(--sans, system-ui); cursor: pointer; }
.ds-foot button:disabled { opacity: .45; cursor: default; }
@media (max-width: 760px) {
  .ds-btn { height: 32px; padding: 0 10px; border: 0; background: transparent; gap: 6px; }
  .ds-btn .ds-lbl small { display: none; }
  .ds-pop { position: fixed; left: 12px; right: auto; top: calc(var(--topbar, 56px) + 6px); width: calc(100vw - 24px); max-width: none; max-height: calc(100vh - var(--topbar, 56px) - 24px); overflow-y: auto; }
}
`;
  document.head.appendChild(css);

  const host = document.querySelector(".bar-right");
  const wrap = document.createElement("div");
  wrap.className = "ds-wrap";
  wrap.innerHTML = `
<button type="button" class="ds-btn" aria-haspopup="dialog" aria-expanded="false" title="Record tasks as a robot-learning dataset (LeRobot format)">
  <i aria-hidden="true"></i><span class="ds-lbl">Record</span>
</button>
<div class="ds-pop" role="dialog" aria-label="Dataset recording" hidden>
  <div class="ds-head"><strong>Dataset</strong><span class="ds-sum"></span></div>
  <div class="ds-note">While recording, every task the robot runs becomes a training episode (LeRobot format): what you said, joint states, actions, two cameras. Failed tasks are kept and labelled.</div>
  <div class="ds-row"><input class="ds-name" type="text" spellcheck="false" autocomplete="off" aria-label="Dataset name" placeholder="dataset name"><button type="button" class="ds-go">Start</button></div>
  <div class="ds-cur" hidden><i aria-hidden="true"></i><span></span><b class="ds-cur-t"></b></div>
  <ul class="ds-list" aria-label="Episodes"></ul>
  <div class="ds-foot"><span class="ds-where"></span><button type="button" class="ds-discard">Discard last</button></div>
</div>`;
  if (host) host.insertBefore(wrap, document.getElementById("demoPill") || document.getElementById("demoBtn") || null);
  else U.slot("side").prepend(wrap);
  const btn = wrap.querySelector(".ds-btn"), pop = wrap.querySelector(".ds-pop"), lbl = wrap.querySelector(".ds-lbl");
  const nameIn = wrap.querySelector(".ds-name"), go = wrap.querySelector(".ds-go"), list = wrap.querySelector(".ds-list");
  const cur = wrap.querySelector(".ds-cur"), sum = wrap.querySelector(".ds-sum"), where = wrap.querySelector(".ds-where");
  const discard = wrap.querySelector(".ds-discard");

  const stamp = () => { const d = new Date(), p = (n) => String(n).padStart(2, "0"); return `session_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`; };
  nameIn.value = stamp();
  let D = {}, open = false;

  function setOpen(v) {
    open = v; pop.hidden = !v; btn.setAttribute("aria-expanded", String(v));
    if (v) { render(D, true); if (!D.recording) setTimeout(() => nameIn.focus(), 0); }
  }
  btn.addEventListener("click", () => setOpen(!open));
  document.addEventListener("pointerdown", (e) => { if (open && !wrap.contains(e.target)) setOpen(false); });
  document.addEventListener("keydown", (e) => { if (open && e.key === "Escape") { setOpen(false); btn.focus(); } });

  go.addEventListener("click", () => {
    if (D.recording) { U.send({ type: "dataset", op: "stop", id: `ds-${Date.now()}` }); return; }
    const name = nameIn.value.trim() || stamp();
    if (!U.send({ type: "dataset", op: "start", name, auto: true, id: `ds-${Date.now()}` })) U.toast("Not connected to the robot.");
  });
  nameIn.addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); go.click(); } });
  discard.addEventListener("click", () => {
    const l = D.last; if (!l) return;
    if (!confirm(`Discard episode ${l.i == null ? "" : l.i + " "}(“${l.task}”)? Its files are deleted.`)) return;
    U.send({ type: "dataset", op: "discard_last", id: `ds-${Date.now()}` });
  });

  let lastKey = "", lastReply = null;
  function render(d, force) {
    const rec = !!d.recording, n = d.episodes || 0;
    btn.dataset.rec = rec ? "1" : "0";
    const l = rec ? `<b>REC</b> <small>${n} ep</small>` : (n ? `Record <small>${n} ep</small>` : "Record");
    if (lbl.innerHTML !== l) lbl.innerHTML = l;
    btn.title = rec ? `Recording every task into datasets/${d.name || ""}` : "Record tasks as a robot-learning dataset (LeRobot format)";
    if (!open && !force) return;
    const key = JSON.stringify([rec, n, d.name, d.current, d.list, d.message, d.error, d.refused]);
    if (key === lastKey && !force) return;
    lastKey = key;
    go.textContent = rec ? "Stop" : "Start"; go.dataset.rec = rec ? "1" : "0";
    nameIn.disabled = rec;
    if (rec && d.name && document.activeElement !== nameIn) nameIn.value = d.name;
    const ok = d.succeeded || 0;
    sum.textContent = n ? `${n} episode${n === 1 ? "" : "s"} · ${ok} succeeded` : (rec ? "waiting for a task" : "");
    cur.hidden = !d.current;
    if (d.current) { cur.querySelector("span").textContent = d.current.task; cur.querySelector(".ds-cur-t").textContent = `${(d.current.seconds || 0).toFixed(1)} s`; }
    const items = (d.list || []).slice().reverse();
    list.innerHTML = items.length ? items.map((e) => `<li title="${esc(e.robot)} · ${esc(e.outcome)} · ${e.frames} frames">
      <span class="n">${e.i == null ? "…" : "#" + e.i}</span><span class="${e.success ? "ok" : "bad"}" aria-label="${e.success ? "succeeded" : "failed"}">${e.success ? "✓" : "✕"}</span>
      <span class="t">${esc(e.task)}</span><span class="d">${(e.seconds || 0).toFixed(1)} s</span></li>`).join("")
      : `<li class="ds-empty">${rec ? "Ask the robot to do something: each task is one episode." : "No episodes yet."}</li>`;
    discard.disabled = !d.last || d.last.i == null;
    const msg = d.error ? `Error: ${d.error}` : (d.message || (d.root ? d.root.replace(/^.*\/datasets\//, "datasets/") : ""));
    where.textContent = msg + (d.refused ? ` · ${d.refused} refused before moving` : "");
  }

  if (location.hash === "#dataset") setOpen(true);   // deep link: open the panel

  U.onState((st) => {
    D = st.dataset || {};
    render(D, false);
    const r = D.reply;
    if (r && r.id && r.id !== lastReply) {
      lastReply = r.id;
      if (String(r.id).startsWith("ds-")) { if (!r.ok) U.toast(r.message); else if (r.op !== "discard_last") U.say("sys", r.message); }
    }
  });
})();
