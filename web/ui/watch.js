// v1 P3 Watching the person (#31): the robot understands what you do (state.human) and acts on it (state.watch).
// A compact "Watch" card in the side slot: mode chips (Off · Mirror · Learn · Respond · Assist · Coach), the live
// reading (activity, phase, reps, held tool, one-line summary), the mode's own line (recording, last demo + Repeat,
// coach cue, assist state), the rule list with delete, and a one-line "when …, …" rule box.
// With this device's camera on and a mode on, MediaPipe Hands runs in the browser and sends 9 keypoints per hand
// ({type:"hands"}, ~8 Hz): pinch and palm orientation for mirroring. Typed or spoken "copy me", "watch and learn",
// "when I put my hand out, give me the wrench", "coach me", "spot me", "stop watching" are handled here when the
// LLM agent is off (when it is on, the agent has the same abilities as tools).
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const pretty = (s) => String(s || "").replace(/_/g, " ");
  const MODES = [["off", "Off"], ["mirror", "Mirror"], ["learn", "Learn"], ["respond", "Respond"], ["assist", "Assist"], ["coach", "Coach"]];
  const HINT = {
    off: "Pick a mode and the robot watches you through your camera.",
    mirror: "Copies you live. Move your hand; pinch or make a fist to close the gripper.",
    learn: "Recording. Show it once, then press Done.",
    respond: "Acts on your rules. Try “when I put my hand out, give me the mug”.",
    assist: "Follows under your hands and pushes when you get stuck.",
    coach: "Counts your reps and calls out form.",
  };
  const VERB = { squat: "Squatting", press: "Pressing", curl: "Curling", wave: "Waving", weave: "Weaving", reach: "Reaching",
    point: "Pointing", still: "Still", moving: "Moving", none: "No one in view" };
  const EYE = '<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12z"/><circle cx="12" cy="12" r="3"/></svg>';

  const css = `
.pcw { container-type: inline-size; background: var(--raised); border: 1px solid var(--line); border-radius: var(--r-inner, 16px); box-shadow: var(--shadow); padding: 4px 14px 12px; display: flex; flex-direction: column; gap: 8px; min-width: 0; }
.pcw-head { display: flex; align-items: center; gap: 8px; min-height: 44px; }
.pcw-head > svg { color: var(--accent); flex: none; }
.pcw-title { font-size: var(--fs-sm, 13px); font-weight: 600; }
.pcw-live { display: inline-flex; align-items: center; gap: 6px; font-size: var(--fs-xs, 12px); color: var(--t3); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; min-width: 0; }
.pcw-live i { width: 7px; height: 7px; border-radius: 50%; background: var(--idle); flex: none; }
.pcw-live[data-on="1"] i { background: var(--ok); box-shadow: 0 0 0 3px rgba(31, 143, 85, .14); }
.pcw-live[data-rec="1"] i { background: var(--bad); box-shadow: 0 0 0 3px rgba(214, 69, 58, .16); animation: pcw-pulse 1.2s ease-in-out infinite; }
@keyframes pcw-pulse { 50% { opacity: .35; } }
.pcw-head .grow { flex: 1; }
.pcw-ai { min-height: 32px; padding: 0 10px; border-radius: 999px; border: 1px solid var(--line2); background: transparent; color: var(--t2); font-size: var(--fs-xs, 12px); font-weight: 600; position: relative; }
.pcw-ai::before { content: ""; position: absolute; inset: -6px -2px; }
.pcw-ai[aria-pressed="true"] { background: var(--accent-tint); color: var(--accent-ink); border-color: transparent; }
.pcw-ai:focus-visible, .pcw-chip:focus-visible, .pcw-btn:focus-visible { outline: none; box-shadow: var(--ring); }
.pcw-modes { display: flex; gap: 2px; padding: 3px; background: var(--field); border-radius: 12px; }
@container (max-width: 290px) { .pcw-modes { flex-wrap: wrap; } .pcw-chip { flex-basis: 30%; } }
.pcw-chip { flex: 1 1 auto; min-width: 0; overflow: hidden; text-overflow: ellipsis; min-height: 36px; padding: 0 5px; border: 0; border-radius: 9px; background: transparent; color: var(--t2); font-size: var(--fs-xs, 12px); font-weight: 600; white-space: nowrap; transition: background var(--fast), color var(--fast); }
.pcw-chip:hover { color: var(--t1); }
.pcw-chip[aria-checked="true"] { background: var(--raised); color: var(--t1); box-shadow: 0 1px 2px rgba(0,0,0,.08), 0 0 0 1px var(--line-soft); }
.pcw-chip[data-mode="off"][aria-checked="true"] { color: var(--t2); }
.pcw-now { display: flex; align-items: center; gap: 12px; min-height: 48px; }
.pcw-reps { flex: none; width: 52px; height: 48px; border-radius: 12px; background: var(--deep); display: flex; flex-direction: column; align-items: center; justify-content: center; line-height: 1; }
.pcw-reps b { font-size: 22px; font-weight: 650; font-variant-numeric: tabular-nums; }
.pcw-reps small { font-size: 10px; color: var(--t3); margin-top: 3px; letter-spacing: .02em; }
.pcw-reps[data-zero="1"] b { color: var(--t3); }
.pcw-txt { min-width: 0; flex: 1; display: flex; flex-direction: column; gap: 2px; }
.pcw-act { display: flex; align-items: center; gap: 6px; font-size: var(--fs-md, 15px); font-weight: 600; min-width: 0; flex-wrap: wrap; }
.pcw-tag { font-size: 11px; font-weight: 600; padding: 2px 8px; border-radius: 999px; background: var(--accent-tint); color: var(--accent-ink); white-space: nowrap; }
.pcw-tag[data-k="hold"] { background: rgba(245, 145, 69, .14); color: var(--hot-ink); }
.pcw-tag[data-k="stuck"] { background: rgba(214, 69, 58, .12); color: var(--bad-ink); }
.pcw-sum { font-size: var(--fs-xs, 12px); color: var(--t2); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.pcw-line { display: flex; align-items: center; gap: 8px; min-height: 36px; font-size: var(--fs-xs, 12px); color: var(--t2); border-top: 1px solid var(--divider); padding-top: 8px; }
.pcw-line .t { flex: 1; min-width: 0; }
.pcw-line b { color: var(--t1); font-weight: 600; }
.pcw-btn { min-height: 36px; padding: 0 14px; border-radius: 999px; border: 1px solid var(--line2); background: var(--raised); color: var(--t1); font-size: var(--fs-xs, 12px); font-weight: 600; white-space: nowrap; flex: none; }
.pcw-btn:hover { background: var(--active); }
.pcw-btn.primary { background: var(--ink); color: #fff; border-color: var(--ink); }
.pcw-btn.primary:hover { background: #000; }
.pcw-rules { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; }
.pcw-rule { display: flex; align-items: center; gap: 8px; min-height: 44px; border-top: 1px solid var(--divider); }
.pcw-rule .w { flex: 1; min-width: 0; font-size: var(--fs-xs, 12px); line-height: 1.35; }
.pcw-rule .w b { font-weight: 600; }
.pcw-rule .w span { color: var(--t3); }
.pcw-rule .n { font-size: 11px; color: var(--t3); font-variant-numeric: tabular-nums; white-space: nowrap; }
.pcw-rule .n[data-hot="1"] { color: var(--ok-ink); font-weight: 600; }
.pcw-rm { width: 44px; height: 44px; flex: none; border: 0; border-radius: 999px; background: transparent; color: var(--t3); display: grid; place-items: center; }
.pcw-rm:hover { background: var(--active); color: var(--bad); }
.pcw-rm:focus-visible { outline: none; box-shadow: var(--ring); }
.pcw-add { display: flex; gap: 6px; }
.pcw-add input { flex: 1; min-width: 0; min-height: 40px; border-radius: 12px; border: 1px solid var(--line); background: var(--field); padding: 0 12px; font-size: var(--fs-xs, 12px); }
.pcw-add input:focus { outline: none; border-color: var(--accent); background: var(--raised); box-shadow: var(--ring); }
.pcw-msg { font-size: var(--fs-xs, 12px); color: var(--t2); min-height: 0; }
.pcw-msg[data-bad="1"] { color: var(--bad-ink); }
@media (prefers-reduced-motion: reduce) { .pcw-live[data-rec="1"] i { animation: none; } .pcw-chip { transition: none; } }`;
  const st = document.createElement("style"); st.id = "pcui-watch-css"; st.textContent = css; document.head.appendChild(st);

  const card = document.createElement("section");
  card.className = "pcw"; card.setAttribute("aria-label", "Watch the person");
  card.innerHTML = `<div class="pcw-head">${EYE}<span class="pcw-title">Watch</span><span class="pcw-live" aria-live="polite"><i aria-hidden="true"></i><span class="lt">no one in view</span></span><span class="grow"></span>
      <button type="button" class="pcw-ai" aria-pressed="false" title="Also describe what you're doing with a vision model (sends a camera frame every few seconds; needs a server key)">AI</button></div>
    <div class="pcw-modes" role="radiogroup" aria-label="Watch mode">${MODES.map(([m, l]) => `<button type="button" class="pcw-chip" role="radio" data-mode="${m}" aria-checked="${m === "off"}">${l}</button>`).join("")}</div>
    <div class="pcw-now"><div class="pcw-reps" data-zero="1"><b>0</b><small>REPS</small></div>
      <div class="pcw-txt"><div class="pcw-act"></div><div class="pcw-sum"></div></div></div>
    <div class="pcw-line"><span class="t"></span></div>
    <ul class="pcw-rules" aria-label="Rules"></ul>
    <form class="pcw-add" autocomplete="off"><input type="text" aria-label="New rule" placeholder="When I hold out my hand, give me the mug"><button type="submit" class="pcw-btn">Add</button></form>
    <div class="pcw-msg" aria-live="polite" hidden></div>`;
  U.slot("side").prepend(card);
  const $ = (s) => card.querySelector(s);
  const live = $(".pcw-live"), liveT = live.querySelector(".lt"), ai = $(".pcw-ai"), reps = $(".pcw-reps"), act = $(".pcw-act"),
    sum = $(".pcw-sum"), line = $(".pcw-line"), lineT = line.querySelector(".t"), rules = $(".pcw-rules"), form = $(".pcw-add"),
    input = form.querySelector("input"), note = $(".pcw-msg");

  let W = {}, H = {}, msgTimer = 0;
  const send = (m) => U.send(Object.assign({ type: "watch", id: `ui${Date.now() % 1e7}` }, m));
  function say(text, bad) { note.textContent = text || ""; note.hidden = !text; note.dataset.bad = bad ? "1" : "0"; clearTimeout(msgTimer); if (text) msgTimer = setTimeout(() => { note.hidden = true; }, 6000); }
  function setMode(m) { if (send({ op: "mode", mode: m })) { for (const b of card.querySelectorAll(".pcw-chip")) b.setAttribute("aria-checked", String(b.dataset.mode === m)); } }

  card.querySelector(".pcw-modes").addEventListener("click", (e) => { const b = e.target.closest(".pcw-chip"); if (b) setMode(b.dataset.mode); });
  card.querySelector(".pcw-modes").addEventListener("keydown", (e) => {   // radio group: arrows move the choice
    if (!/^Arrow(Left|Right)$/.test(e.key)) return;
    const bs = [...card.querySelectorAll(".pcw-chip")], i = bs.findIndex((b) => b.getAttribute("aria-checked") === "true");
    const n = bs[(i + (e.key === "ArrowRight" ? 1 : bs.length - 1)) % bs.length]; n.focus(); setMode(n.dataset.mode); e.preventDefault();
  });
  ai.addEventListener("click", () => send({ op: "summary", on: ai.getAttribute("aria-pressed") !== "true" }));
  form.addEventListener("submit", (e) => { e.preventDefault(); addRule(input.value); });
  rules.addEventListener("click", (e) => { const b = e.target.closest("[data-rm]"); if (b) send({ op: "rule_remove", rule: b.dataset.rm }); });
  line.addEventListener("click", (e) => {
    const b = e.target.closest("[data-act]"); if (!b) return;
    if (b.dataset.act === "done") send({ op: "learn_stop" });
    else if (b.dataset.act === "replay") send({ op: "replay" });
  });
  function addRule(text) {
    text = String(text || "").trim(); if (!text) return false;
    const pending = `ui${Date.now() % 1e7}`;
    if (!U.send({ type: "watch", op: "rule", text, id: pending })) { say("Not connected.", true); return false; }
    waitReply(pending, (r) => { say(r.message, !r.ok); if (r.ok) { input.value = ""; U.say("robot", r.message); } });
    return true;
  }
  const waiters = new Map();
  function waitReply(id, fn) { waiters.set(id, fn); setTimeout(() => waiters.delete(id), 4000); }

  // ------------------------------------------------------------------ render
  let memo = {};
  function changed(k, v) { const j = JSON.stringify(v); if (memo[k] === j) return false; memo[k] = j; return true; }
  function render() {
    const mode = W.mode || "off";
    if (changed("mode", mode)) {
      for (const b of card.querySelectorAll(".pcw-chip")) b.setAttribute("aria-checked", String(b.dataset.mode === mode));
      card.dataset.mode = mode;
    }
    const rec = mode === "learn" && W.learn && W.learn.recording;
    const handsOn = !!(window.PC && window.PC.camVideo) && HT.ready && mode !== "off";
    const liveTxt = !H.visible ? "no one in view" : rec ? `recording ${(W.learn.seconds || 0).toFixed(1)} s`
      : H.src === "synthetic" ? "synthetic person" : handsOn ? "watching · hands" : mode === "off" ? "sees you" : "watching";
    if (changed("live", [liveTxt, H.visible, rec])) { liveT.textContent = liveTxt; live.dataset.on = H.visible && mode !== "off" ? "1" : "0"; live.dataset.rec = rec ? "1" : "0"; }
    const s = W.summary || {};
    if (changed("ai", s.on)) ai.setAttribute("aria-pressed", String(!!s.on));
    const n = H.reps || 0;
    if (changed("reps", n)) { reps.querySelector("b").textContent = String(n); reps.dataset.zero = n ? "0" : "1"; }
    const tags = [];
    if (H.phase && ["squat", "press", "curl"].includes(H.activity)) tags.push(["phase", H.phase]);
    if (H.stalled) tags.push(["stuck", "stuck"]);
    if (H.holding) tags.push(["hold", pretty(H.holding)]);
    const actTxt = H.visible ? (VERB[H.activity] || pretty(H.activity)) : VERB.none;
    if (changed("act", [actTxt, tags])) act.innerHTML = `<span>${esc(actTxt)}</span>${tags.map(([k, t]) => `<span class="pcw-tag" data-k="${k}">${esc(t)}</span>`).join("")}`;
    const sumTxt = !H.visible ? "Turn on this device's camera in Settings, or step into view."
      : H.summary_src && H.summary_src !== "rules" ? `${H.summary} · ${H.summary_src}` : detail();
    if (changed("sum", sumTxt)) sum.textContent = sumTxt;
    renderLine(mode);
    renderRules();
  }
  function detail() {      // what the big label doesn't say already (the rule-based summary would repeat it)
    const out = [], w = H.weave, hs = H.hands || {}, off = H.offer && hs[H.offer];
    if (w) out.push(`${(w.amp * 100).toFixed(1)} cm side to side at ${w.freq.toFixed(1)} Hz`);
    if (off && off.open) out.push(`${H.offer} hand held out`);
    if (H.fatigue > 0.25) out.push("slowing down");
    if (H.src === "synthetic") out.push("synthetic person (no camera)");
    out.push(`${(H.speed || 0).toFixed(2)} m/s`);
    return out.join(" · ");
  }
  function renderLine(mode) {
    let html = esc(HINT[mode] || ""), btn = "";
    const L = W.learn || {}, last = L.last;
    if (mode === "learn" && L.recording) { html = `<b>Recording</b> ${(L.seconds || 0).toFixed(1)} s · show it once`; btn = `<button type="button" class="pcw-btn primary" data-act="done">Done</button>`; }
    else if (mode === "mirror" && W.mirror) {
      const m = W.mirror;
      if (m.kind === "whole_body") html = m.paused ? "Copying your whole body (starting…)" : "<b>Copying your whole body</b>";
      else if (m.paused) html = "Paused while a task runs, or no one in view.";
      else if (m.pairs && m.pairs.length) html = m.pairs.map((p) => `Your ${p.person} hand → my ${p.robot} hand${p.grip === 1 ? " · <b>closed</b>" : ""}`).join(" · ");
    } else if (mode === "assist" && W.assist) {
      const a = W.assist;
      html = a.phase === "helping" ? `<b>Helping</b> · pushing up, capped at ${Math.round(a.max_force || 0)} N` : a.phase === "following" ? "Following under your hands" : "Waiting for you";
    } else if (mode === "coach" && W.coach && W.coach.last) {
      const c = W.coach.last;
      html = c.text ? `<b>Rep ${esc(c.rep)}</b> · ${esc(c.text)}` : `<b>Rep ${esc(c.rep)}</b> · good`;
    }
    if (!(mode === "learn" && L.recording) && last && mode !== "mirror" && mode !== "assist" && !(mode === "coach" && W.coach && W.coach.last)) {
      const p = last.pattern;
      html = `Learned <b>${esc(pretty(last.name))}</b> · ${esc(last.activity)}${p ? ` · ${(p.amp * 100).toFixed(1)} cm at ${p.freq.toFixed(1)} Hz` : last.reps ? ` · ${last.reps} reps` : ""}`;
      btn = `<button type="button" class="pcw-btn" data-act="replay" title="The robot proposes its version as a plan">Repeat</button>`;
    }
    if (changed("line", [html, btn])) line.innerHTML = `<span class="t">${html}</span>${btn}`;
  }
  function renderRules() {
    const list = Array.isArray(W.rules) ? W.rules : [];
    const view = list.map((r) => [r.id, r.when_text || r.when || r.expr, r.do, r.fired, r.last_age !== null && r.last_age < 4]);
    if (!changed("rules", view)) return;
    rules.innerHTML = list.map((r) => `<li class="pcw-rule"><span class="w"><span>When</span> <b>${esc(r.when_text || r.when || r.expr)}</b> <span>→</span> ${esc(r.do)}</span>
      <span class="n" data-hot="${r.last_age !== null && r.last_age < 4 ? 1 : 0}">${r.fired ? `×${r.fired}` : ""}</span>
      <button type="button" class="pcw-rm" data-rm="${esc(r.id)}" aria-label="Delete rule: when ${esc(r.when_text || r.when)}" title="Delete">${U.icon("x", 18)}</button></li>`).join("");
  }

  let lastEv = null;
  U.onState((s) => {
    W = (s && s.watch) || {}; H = (s && s.human) || {};
    const r = W.reply;
    if (r && waiters.has(r.id)) { const fn = waiters.get(r.id); waiters.delete(r.id); fn(r); }
    const ev = (W.events || [])[W.events ? W.events.length - 1 : 0];
    if (ev && lastEv && ev.t !== lastEv && (ev.kind === "rule" || ev.kind === "demo") && !/^New rule/.test(ev.text)) U.say("sys", ev.text);
    if (ev) lastEv = ev.t;
    render();
    hands.sync(W.mode || "off");
  });

  // ------------------------------------------------------------------ MediaPipe Hands in the browser (camera on + a mode on)
  const KEYS = [0, 4, 5, 8, 9, 12, 16, 17, 20];          // wrist, thumb tip, index mcp/tip, middle mcp/tip, ring tip, pinky mcp/tip
  const TV = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14";
  const MODEL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task";
  const HT = { lm: null, loading: false, failed: false, timer: 0, ready: false };
  const r3 = (v) => Math.round(v * 1000) / 1000;
  async function loadHands() {
    if (HT.lm || HT.loading || HT.failed) return;
    HT.loading = true;
    try {
      const vision = await import(`${TV}/vision_bundle.mjs`);
      const files = await vision.FilesetResolver.forVisionTasks(`${TV}/wasm`);
      HT.lm = await vision.HandLandmarker.createFromOptions(files, { baseOptions: { modelAssetPath: MODEL, delegate: "GPU" }, runningMode: "VIDEO", numHands: 2 });
      HT.ready = true;
    } catch (e) { HT.failed = true; console.warn("hand tracking unavailable; using the pose's hand points", e); }
    HT.loading = false;
  }
  function tick() {
    const v = window.PC && window.PC.camVideo;
    if (!HT.lm || !v || v.readyState < 2) return;
    let res;
    try { res = HT.lm.detectForVideo(v, performance.now()); } catch (e) { return; }
    const out = [];
    (res.worldLandmarks || []).forEach((wl, i) => {
      const im = res.landmarks && res.landmarks[i];
      if (!im || !wl) return;
      out.push({ uv: [r3(im[0].x), r3(im[0].y)], lm: KEYS.map((k) => [r3(wl[k].x), r3(wl[k].y), r3(wl[k].z)]),
        score: r3(((res.handedness || [])[i] || [{}])[0].score || 0) });
    });
    U.send({ type: "hands", hands: out });
  }
  const hands = {
    sync(mode) {
      const want = mode !== "off" && !!(window.PC && window.PC.camVideo);
      if (want && !HT.lm) loadHands();
      if (want && HT.lm && !HT.timer) HT.timer = setInterval(tick, 125);
      if (!want && HT.timer) { clearInterval(HT.timer); HT.timer = 0; }
    },
  };

  // ------------------------------------------------------------------ hearing (when the LLM agent is off)
  const RX = {
    mirror: /\b(do it like me|copy me|mirror me|follow my hands?|move like me|imitate me)\b|따라\s*해/i,
    learn: /\b(watch (me|and learn|this)|learn (this|from me)|i'?ll show you how)\b|보고\s*배워/i,
    coach: /\b(coach me|count my (squats|reps|curls|presses)|check my form|count (for me|reps))\b|자세\s*봐/i,
    assist: /\b(spot me|help me lift|assist me)\b/i,
    off: /\b(stop watching|stop copying|stop mirroring|stop coaching|don'?t watch)\b|그만\s*봐/i,
    done: /^(ok(ay)?[, ]+)?(done|that'?s it|finished|i'?m done)\b|^(끝|다\s*했어)/i,
    replay: /\b(your turn|now you( do it)?|do what i did|repeat (it|that|what i did)|now try)\b|이제\s*해\s*봐/i,
    rule: /^\s*(when(ever)?|if|once|as soon as)\b.+(,|\bthen\b)|하면\s/i,
    me: /\b(i|i'?m|my|me)\b|내가|제가|손/i,           // a rule is about the person: "when I …", "if my hand …"
  };
  window.addEventListener("pc-hear", (ev) => {
    if (ev.defaultPrevented) return;
    if (U.agent && U.agent.active) return;              // the agent has set_watch_mode / add_watch_rule / learn_from_demo
    const t = String((ev.detail && ev.detail.text) || "").trim(); if (!t) return;
    const mode = W.mode || "off";
    let handled = true;
    if (mode === "learn" && RX.done.test(t)) send({ op: "learn_stop" });
    else if (RX.rule.test(t) && RX.me.test(t.split(/,|\bthen\b|하면/)[0])) addRule(t);
    else if (RX.off.test(t)) setMode("off");
    else if (RX.mirror.test(t)) setMode("mirror");
    else if (RX.learn.test(t)) setMode("learn");
    else if (RX.coach.test(t)) setMode("coach");
    else if (RX.assist.test(t)) setMode("assist");
    else if (RX.replay.test(t) && W.learn && W.learn.last) send({ op: "replay" });
    else handled = false;
    if (handled) ev.preventDefault();
  });

  window.PCWatch = { setMode, addRule, get state() { return { watch: W, human: H }; } };
})();
