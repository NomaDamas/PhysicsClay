// UX 3 · Brain: typed or spoken requests go to the real LLM agent (pps/agent.py) instead of the regex parser.
// app.js fires "pc-hear" first (plan "go", "which one?" answers and zone phrases are handled there by other modules),
// then a cancelable "pc-intent"; this module takes it when an agent is available and sends {type:"agent", text}.
// Otherwise the UX 2.0 parser runs as before. The agent's steps show in the conversation feed: tool calls as compact
// cards with ok/fail, a "thinking · <model>" indicator while it works; its replies arrive as robot lines via state.say
// (feed + voice). A chip in the feed header says which brain is active. Settings: use the agent, confirm before moving,
// and (public --byok servers) the visitor's own LLM endpoint/key/model, stored only in this browser.
"use strict";
(function () {
  const P = window.PCUI;
  if (!P) return;
  const LS = {
    get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } },
  };
  const cfg = {
    get on() { return LS.get("pc.agentOn", "1") === "1"; },
    get confirm() { return LS.get("pc.agentConfirm", "1") === "1"; },
    get base() { return LS.get("pc.llmBase", ""); },
    get key() { return LS.get("pc.llmKey", ""); },
    get model() { return LS.get("pc.llmModel", ""); },
    get effort() { return LS.get("pc.llmEffort", "medium"); },
  };
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const ag = () => (P.state && P.state.agent && typeof P.state.agent === "object" ? P.state.agent : null);
  const byok = () => !!(P.state && P.state.byok);
  // which brain answers typed/spoken requests right now
  function llmReady() { const a = ag(); return !!a && (a.available || (byok() && !!cfg.key)); }
  function active() { return cfg.on && llmReady(); }
  function modelName() { const a = ag(); return (byok() && cfg.key ? cfg.model || "gpt-6.1-terra" : a && a.model) || "LLM"; }

  // ------------------------------------------------------------------ styles (existing tokens only)
  const css = `
.pca-brain { display: inline-flex; align-items: center; gap: 6px; min-height: 28px; padding: 0 10px; border-radius: 999px; border: 1px solid var(--line);
  background: var(--raised); color: var(--t2); font: 500 11px var(--sans); white-space: nowrap; cursor: pointer; flex: 0 1 auto; min-width: 0; max-width: 170px; }
.pca-brain .lbl { min-width: 0; overflow: hidden; text-overflow: ellipsis; }
.pca-brain:hover { color: var(--t1); border-color: var(--line2); }
.pca-brain:focus-visible { outline: none; box-shadow: var(--ring); }
.pca-brain .dot { width: 7px; height: 7px; border-radius: 50%; background: var(--idle); flex: none; }
.pca-brain[data-brain="llm"] { color: var(--accent-ink); border-color: color-mix(in srgb, var(--accent) 35%, transparent); background: var(--accent-tint); }
.pca-brain[data-brain="llm"] .dot { background: var(--accent); }
.pca-brain[data-busy="true"] .dot { animation: pulse 1s infinite; }
.pca-reset { min-height: 28px; min-width: 28px; border-radius: 999px; border: 0; background: transparent; color: var(--t2); cursor: pointer; font: 500 11px var(--sans); }
.pca-reset:hover { background: var(--active); color: var(--t1); }
.pca-reset[hidden] { display: none; }
.feed-list > .pca-tool { flex: none; align-self: flex-start; max-width: 92%; display: grid; grid-template-columns: auto 1fr auto; column-gap: 8px; row-gap: 2px; align-items: center;
  padding: 7px 10px 7px 10px; border-radius: 12px; border: 1px solid var(--line); background: var(--field); font-family: var(--sans); animation: rise var(--med) var(--ease); }
.pca-tool .ic { width: 18px; height: 18px; border-radius: 6px; display: grid; place-items: center; background: var(--deep); color: var(--t2); font: 600 10px var(--mono); }
.pca-tool .nm { font: 600 12px var(--mono); color: var(--t1); min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.pca-tool .nm span { font-weight: 400; color: var(--t2); }
.pca-tool .st { font: 600 10.5px var(--sans); padding: 1px 7px; border-radius: 999px; background: var(--deep); color: var(--t2); white-space: nowrap; }
.pca-tool .rs { grid-column: 2 / 4; font-size: 12px; line-height: 1.4; color: var(--t2); overflow-wrap: anywhere; }
.pca-tool .rs:empty { display: none; }
.pca-tool[data-ok="true"] .ic { background: color-mix(in srgb, var(--ok) 16%, #fff); color: var(--ok-ink); }
.pca-tool[data-ok="true"] .st { background: color-mix(in srgb, var(--ok) 14%, #fff); color: var(--ok-ink); }
.pca-tool[data-ok="false"] .ic { background: color-mix(in srgb, var(--bad) 14%, #fff); color: var(--bad-ink); }
.pca-tool[data-ok="false"] .st { background: color-mix(in srgb, var(--bad) 12%, #fff); color: var(--bad-ink); }
.pca-tool[data-ok="false"] .rs { color: var(--bad-ink); }
.pca-tool:not([data-ok]) .st::before { content: ""; display: inline-block; width: 5px; height: 5px; border-radius: 50%; background: var(--info); margin-right: 5px; vertical-align: 1px; animation: pulse 1s infinite; }
.feed-list > .pca-note { flex: none; font-size: var(--fs-xs); color: var(--t2); padding-left: 2px; }
.feed-list > .pca-note.err { color: var(--bad-ink); }
.feed-list > .pca-thinking { flex: none; align-self: flex-start; display: inline-flex; align-items: center; gap: 8px; padding: 6px 12px; border-radius: 999px;
  background: var(--accent-tint); color: var(--accent-ink); font: 500 12px var(--sans); }
.pca-thinking i { display: inline-flex; gap: 3px; }
.pca-thinking i b { width: 5px; height: 5px; border-radius: 50%; background: currentColor; opacity: .35; animation: pca-dot 1.2s infinite; }
.pca-thinking i b:nth-child(2) { animation-delay: .15s; } .pca-thinking i b:nth-child(3) { animation-delay: .3s; }
@keyframes pca-dot { 0%, 60%, 100% { opacity: .25; transform: none; } 30% { opacity: 1; transform: translateY(-2px); } }
.feed-list > .card[data-id^="ag"] { display: none; }   /* app.js task cards for agent-launched tasks: the tool card covers them */
.pca-set .row2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.pca-set [data-byok][hidden] { display: none; }
.pca-set .status { margin: 0; }
@media (prefers-reduced-motion: reduce) { .pca-thinking i b, .pca-tool:not([data-ok]) .st::before { animation: none; } }`;
  const st = document.createElement("style"); st.id = "pcui-agent-css"; st.textContent = css; document.head.appendChild(st);

  // ------------------------------------------------------------------ feed helpers
  const feed = () => document.getElementById("feed");
  function append(node) {
    const f = feed(); if (!f) return;
    const atBottom = f.scrollHeight - f.scrollTop - f.clientHeight < 60;
    const empty = f.querySelector(".feed-empty"); if (empty) empty.remove();
    if (thinking && thinking.parentNode === f && node !== thinking) f.insertBefore(node, thinking); else f.appendChild(node);
    if (atBottom) f.scrollTop = f.scrollHeight;
  }
  const LABEL = {
    observe: "look", pick_place: "pick & place", reach: "reach", follow_path: "follow path", play_skill: "play skill", go_home: "home",
    stop: "stop", reset_scene: "reset scene", set_speed: "speed", add_zone: "keep-out zone", remove_zone: "remove zone", clear_zones: "clear zones",
    set_goal_layout: "goal layout", run_goal: "run goal", choose_candidate: "answer", confirm_plan: "confirm plan", cancel_plan: "cancel plan",
    follow_me: "follow me", start_teaching: "teach", set_robot: "robot", build_scene: "build scene", add_object: "add object", remove_object: "remove object",
  };
  function argText(tool, a) {
    a = a && typeof a === "object" ? a : {};
    const xyz = (p) => (Array.isArray(p) ? `(${p.map((v) => (+v).toFixed(2)).join(", ")})` : "");
    switch (tool) {
      case "pick_place": return `${a.object || "?"} → ${a.target || (a.marker ? "marker " + a.marker : "") || xyz(a.xyz) || "?"}`;
      case "reach": return a.target || (a.marker ? "marker " + a.marker : "") || xyz(a.xyz);
      case "add_zone": return a.near ? `near ${a.near}` : a.rect ? xyz(a.rect) : "";
      case "set_goal_layout": return Object.keys(a.targets || {}).join(", ");
      case "set_robot": return a.id || "";
      case "build_scene": return (a.spec && a.spec.template) || "";
      case "add_object": return (a.object && a.object.name) || "";
      default: { const k = Object.keys(a); return k.map((x) => (typeof a[x] === "object" ? x : `${x} ${a[x]}`)).join(" · "); }
    }
  }
  function toolNode(e) {
    const n = document.createElement("div"); n.className = "pca-tool"; n.dataset.eid = e.id;
    n.innerHTML = '<span class="ic" aria-hidden="true">ƒ</span><span class="nm"></span><span class="st"></span><span class="rs"></span>';
    return n;
  }
  function paintTool(n, e) {
    const a = argText(e.tool, e.args);
    n.querySelector(".nm").innerHTML = `${esc(LABEL[e.tool] || e.tool)}${a ? ` <span>${esc(a)}</span>` : ""}`;
    const done = typeof e.ok === "boolean";
    if (done) n.dataset.ok = String(e.ok); else delete n.dataset.ok;
    n.querySelector(".st").textContent = done ? (e.ok ? "ok" : "failed") : "running";
    n.querySelector(".ic").textContent = done ? (e.ok ? "✓" : "!") : "ƒ";
    n.querySelector(".rs").textContent = done && e.text !== e.tool ? e.text : "";
  }

  // ------------------------------------------------------------------ events -> feed
  const nodes = new Map();       // event id -> {node, sig}
  let primed = false, thinking = null;
  function renderEvents(a) {
    const evs = Array.isArray(a.events) ? a.events : [];
    if (!primed) { primed = true; for (const e of evs) nodes.set(e.id, { old: true }); return; }   // turns from before this page loaded stay unrendered
    const ids = new Set(evs.map((e) => e.id));
    for (const k of [...nodes.keys()]) if (!ids.has(k) && nodes.size > 80) nodes.delete(k);
    for (const e of evs) {
      const have = nodes.get(e.id);
      const sig = `${e.ok}|${e.text}`;
      if (have && (have.old || have.sig === sig)) continue;
      if (e.kind === "tool") {
        const n = (have && have.node) || toolNode(e);
        paintTool(n, e);
        if (!have || !have.node) append(n);
        nodes.set(e.id, { node: n, sig });
      } else if (!have && (e.kind === "error" || (e.kind === "thinking" && !/^thinking · /.test(e.text)))) {
        const n = document.createElement("div"); n.className = "pca-note" + (e.kind === "error" ? " err" : ""); n.textContent = e.text;
        append(n); nodes.set(e.id, { node: n, sig });
      } else {
        nodes.set(e.id, { node: null, sig });   // user (app.js shows it), reply/result (state.say shows + speaks them)
      }
    }
  }
  function renderThinking(a) {
    const f = feed(); if (!f) return;
    if (a && a.busy) {
      if (!thinking) {
        thinking = document.createElement("div"); thinking.className = "pca-thinking"; thinking.setAttribute("role", "status");
        thinking.innerHTML = '<i aria-hidden="true"><b></b><b></b><b></b></i><span></span>';
      }
      thinking.querySelector("span").textContent = `thinking · ${a.model || modelName()}`;
      if (thinking.parentNode !== f || f.lastElementChild !== thinking) { const atB = f.scrollHeight - f.scrollTop - f.clientHeight < 60; f.appendChild(thinking); if (atB) f.scrollTop = f.scrollHeight; }
    } else if (thinking) { thinking.remove(); thinking = null; }
  }
  // ------------------------------------------------------------------ brain chip
  let chip = null, resetBtn = null;
  function mountChip() {
    if (chip) return;
    const head = document.querySelector("#feedPanel .panel-head"); if (!head) return;
    chip = document.createElement("button"); chip.type = "button"; chip.className = "pca-brain";
    chip.innerHTML = '<span class="dot" aria-hidden="true"></span><span class="lbl"></span>';
    chip.addEventListener("click", openSettings);
    resetBtn = document.createElement("button"); resetBtn.type = "button"; resetBtn.className = "pca-reset"; resetBtn.textContent = "↺";
    resetBtn.title = "New conversation (the agent forgets this chat)"; resetBtn.setAttribute("aria-label", resetBtn.title);
    resetBtn.addEventListener("click", () => { if (P.send({ type: "agent", op: "reset" })) P.say("sys", "New conversation."); });
    const count = head.querySelector("#feedCount");
    head.insertBefore(chip, count || null); head.insertBefore(resetBtn, count || null);
  }
  let chipSig = "";
  function paintChip(a) {
    mountChip(); if (!chip) return;
    const on = active();
    const sig = [on, llmReady(), byok(), modelName(), !!(a && a.busy), document.activeElement && document.activeElement.id].join("|");
    if (sig === chipSig) return; chipSig = sig;
    chip.dataset.brain = on ? "llm" : "parser"; chip.dataset.busy = String(!!(a && a.busy));
    chip.querySelector(".lbl").textContent = on ? modelName() : "offline parser";
    chip.setAttribute("aria-label", on ? `Brain: LLM agent ${modelName()}` : "Brain: offline rule parser");
    chip.title = on ? `Requests go to the LLM agent (${modelName()}), which calls robot tools. Click for settings.`
      : llmReady() ? "The LLM agent is switched off in Settings: the rule parser answers." : byok() ? "Add your own LLM key in Settings to use the LLM agent." : "No LLM configured on this server: the offline rule parser answers.";
    resetBtn.hidden = !on;
    const inp = document.getElementById("cmd");
    if (inp && !inp.dataset.pcaPh) inp.dataset.pcaPh = inp.placeholder;
    if (inp && document.activeElement !== inp) inp.placeholder = on ? "ask anything: put the mug next to the vase" : inp.dataset.pcaPh;
  }

  // ------------------------------------------------------------------ settings (injected into the dialog)
  let sec = null;
  function mountSettings() {
    if (sec) return;
    const ai = document.getElementById("secAI"); if (!ai) return;
    sec = document.createElement("section"); sec.className = "set-sec pca-set"; sec.id = "secAgent";
    sec.innerHTML = `<h3>Agent brain</h3>
      <label class="check"><input id="pcaOn" type="checkbox"><span>Use the LLM agent for typed and spoken requests</span></label>
      <label class="check"><input id="pcaConfirm" type="checkbox"><span>Confirm before moving (the agent proposes a plan; say “go” to run it)</span></label>
      <div data-byok hidden>
        <p class="small muted">Your own OpenAI-compatible Responses API. Stored only in this browser and sent to this server with each request, for that request only.</p>
        <label class="field"><span>LLM base URL</span><input id="pcaBase" type="url" spellcheck="false" placeholder="https://api.openai.com/v1"></label>
        <label class="field"><span>LLM API key</span><input id="pcaKey" type="password" autocomplete="off" spellcheck="false" placeholder="sk-…"></label>
        <div class="row2">
          <label class="field"><span>Model</span><input id="pcaModel" type="text" spellcheck="false" placeholder="gpt-6.1-terra"></label>
          <label class="field"><span>Reasoning effort</span><select id="pcaEffort"><option value="">none</option><option value="low">low</option><option value="medium">medium</option><option value="high">high</option></select></label>
        </div>
      </div>
      <p class="status small muted" id="pcaStatus"></p>`;
    ai.parentNode.insertBefore(sec, ai);
    const dlg = document.getElementById("settings");
    if (dlg) {
      dlg.addEventListener("close", () => {
        if (dlg.returnValue !== "save") return;
        LS.set("pc.agentOn", sec.querySelector("#pcaOn").checked ? "1" : "0");
        LS.set("pc.agentConfirm", sec.querySelector("#pcaConfirm").checked ? "1" : "0");
        if (byok()) {
          LS.set("pc.llmBase", sec.querySelector("#pcaBase").value.trim()); LS.set("pc.llmKey", sec.querySelector("#pcaKey").value.trim());
          LS.set("pc.llmModel", sec.querySelector("#pcaModel").value.trim()); LS.set("pc.llmEffort", sec.querySelector("#pcaEffort").value);
        }
        chipSig = ""; paintChip(ag());
      });
      new MutationObserver(() => { if (dlg.open) fillSettings(); }).observe(dlg, { attributes: true, attributeFilter: ["open"] });
    }
  }
  function fillSettings() {
    if (!sec) return;
    sec.querySelector("#pcaOn").checked = cfg.on; sec.querySelector("#pcaConfirm").checked = cfg.confirm;
    sec.querySelector("[data-byok]").hidden = !byok();
    sec.querySelector("#pcaBase").value = cfg.base; sec.querySelector("#pcaKey").value = cfg.key;
    sec.querySelector("#pcaModel").value = cfg.model; sec.querySelector("#pcaEffort").value = cfg.effort;
    const a = ag();
    sec.querySelector("#pcaStatus").textContent = !a ? "This server has no agent." : a.available ? `Server LLM: ${a.model}.`
      : byok() ? (cfg.key ? "Using your key." : "No key yet: the offline parser answers.") : "No LLM configured (LLM_BASE_URL / LLM_API_KEY in .env): the offline parser answers.";
  }
  function openSettings() {
    const b = document.getElementById("settingsBtn") || document.getElementById("settingsBtnM"); if (b) b.click();
    setTimeout(() => { const s = document.getElementById("secAgent"); if (s) s.scrollIntoView({ block: "start" }); }, 60);
  }

  // ------------------------------------------------------------------ taking requests
  const STOP = /^(please\s+)?(stop|halt|freeze|wait|hold on)\b|^(멈춰|정지|스톱|그만)/i;
  window.addEventListener("pc-intent", (ev) => {
    const text = String((ev.detail && ev.detail.text) || "").trim();
    if (!text || !active()) return;
    if (STOP.test(text)) return;          // safety: stop goes straight to the robot, never waits on a model
    const msg = { type: "agent", text, confirm: cfg.confirm };
    if (byok() && cfg.key) msg.llm = { base_url: cfg.base || undefined, key: cfg.key, model: cfg.model || undefined, effort: cfg.effort || undefined };
    if (P.send(msg)) ev.preventDefault();
  });

  P.agent = { get active() { return active(); }, send(text) { return window.dispatchEvent(new CustomEvent("pc-intent", { detail: { text }, cancelable: true })); } };

  P.onState((s) => {
    const a = s && s.agent && typeof s.agent === "object" ? s.agent : null;
    mountSettings(); paintChip(a);
    if (!a) return;
    renderEvents(a); renderThinking(a);
  });
})();
