// Candidates (UX 2.0 #C): "Which one?". When the server finds a reference ambiguous it sets state.choices
// {id, question, options:[{key, name, label, color, pos, uv}]}. This module shows the question with numbered chips in
// the stage overlay, numbered badges in the 3D stage (layer "choices") and on the robot-eye view, and answers with
// {type:"choose", id, key}. You can pick by click/tap, by typing or saying "two" / "the red one", by pressing 1-4,
// or by pointing (the server resolves a pointed marker near a candidate on its own).
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;

  const SWATCH = { red: "#d6453a", green: "#1f8f55", blue: "#1da8e5" };
  const ICON_Q = '<svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="12" cy="12" r="9"/><path d="M9.6 9.3a2.5 2.5 0 0 1 4.8.9c0 1.7-2.4 2.2-2.4 3.6"/><path d="M12 17h.01"/></svg>';

  const css = document.createElement("style");
  css.textContent = `
.cand-card { pointer-events: auto; width: min(380px, calc(100vw - 32px)); padding: 16px; border-radius: var(--r-card, 24px);
  background: var(--raised, #fff); color: var(--t1, #1d1d1f); box-shadow: var(--shadow-md); border: 1px solid var(--line);
  font: 400 var(--fs-md, 15px)/1.4 var(--sans); animation: cand-in var(--med, .26s) var(--ease, ease) both; }
.cand-card[hidden] { display: none; }
@keyframes cand-in { from { opacity: 0; transform: translateY(-6px); } }
.cand-head { display: flex; gap: 10px; align-items: flex-start; }
.cand-head svg { flex: none; color: var(--hot, #f59145); margin-top: 1px; }
.cand-kicker { margin: 0 0 2px; font: 600 var(--fs-xs, 12px)/1.2 var(--sans); letter-spacing: .04em; text-transform: uppercase; color: var(--hot, #f59145); }
.cand-q { margin: 0; font-weight: 500; }
.cand-chips { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 10px; }
.cand-chip { display: inline-flex; align-items: center; gap: 8px; min-height: 44px; padding: 0 14px 0 6px; border-radius: 999px;
  border: 1px solid var(--line2); background: var(--field, #f5f5f7); color: var(--t1); font: 500 var(--fs-md, 15px) var(--sans); cursor: pointer;
  transition: background var(--fast, .14s), border-color var(--fast, .14s); }
.cand-chip:hover, .cand-chip.is-hot { background: var(--accent-tint); border-color: var(--accent); }
.cand-chip:focus-visible, .cand-cancel:focus-visible, .cand-badge:focus-visible { outline: none; box-shadow: var(--ring); }
.cand-num { display: inline-grid; place-items: center; width: 32px; height: 32px; border-radius: 50%; background: var(--accent); color: var(--on-accent, #fff);
  font: 600 15px var(--sans); }
.cand-dot { width: 10px; height: 10px; border-radius: 3px; background: var(--idle); box-shadow: inset 0 0 0 1px rgba(0,0,0,.12); }
.cand-foot { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
.cand-hint { color: var(--t2); font-size: var(--fs-sm, 13px); }
.cand-cancel { display: inline-flex; align-items: center; gap: 6px; min-height: 44px; padding: 0 12px; border-radius: 999px; border: 0;
  background: transparent; color: var(--t2); font: 500 var(--fs-sm, 13px) var(--sans); cursor: pointer; }
.cand-cancel:hover { background: var(--active); color: var(--t1); }
.cand-eye { position: absolute; inset: 0; pointer-events: none; z-index: 5; }
.cand-eye[hidden] { display: none; }
.cand-badge { position: absolute; left: 0; top: 0; pointer-events: auto; width: 44px; height: 44px; margin: -60px 0 0 -22px; padding: 0;
  border-radius: 50%; border: 3px solid var(--raised, #fff); background: var(--accent); color: var(--on-accent, #fff); font: 700 18px var(--sans);
  box-shadow: var(--shadow-md); cursor: pointer; display: grid; place-items: center; animation: cand-in var(--med, .26s) var(--ease, ease) both; }
.cand-badge::after { content: ""; position: absolute; left: 50%; top: 100%; width: 2px; height: 14px; margin-left: -1px; background: var(--accent); }
.cand-badge:hover, .cand-badge.is-hot { background: var(--hot, #f59145); }
.cand-badge:hover::after, .cand-badge.is-hot::after { background: var(--hot, #f59145); }
.cand-ring { position: absolute; left: 0; top: 0; width: 34px; height: 16px; margin: -8px 0 0 -17px; border-radius: 50%;
  border: 2px solid var(--accent); pointer-events: none; animation: cand-pulse 1.4s ease-in-out infinite; }
@keyframes cand-pulse { 50% { transform: scale(1.25); opacity: .55; } }
.card[data-status="asking"] { --sc: var(--hot); }
@media (prefers-reduced-motion: reduce) { .cand-card, .cand-badge, .cand-ring { animation: none; } }
`;
  document.head.appendChild(css);

  let cur = null, sig = "", lastPos = "", answered = null, hot = null;

  // ---------------------------------------------------------------- matching what you say to an option
  const COLORS = { red: ["red", "빨간", "빨강"], green: ["green", "초록", "녹색"], blue: ["blue", "파란", "파랑"] };
  const KINDS = { mug: ["mug", "cup", "컵", "머그"], block: ["block", "cube", "블록", "큐브"] };
  const ORD = [["1", "one", "first", "하나", "첫"], ["2", "two", "second", "둘", "두 번째", "두번째"],
    ["3", "three", "third", "셋", "세 번째", "세번째"], ["4", "four", "fourth", "넷", "네 번째", "네번째"]];
  const CANCEL = /^(cancel|never ?mind|forget it|neither|none)\b|취소|됐어/;
  const word = (t, w) => (/^[\x00-\x7f]+$/.test(w) ? new RegExp(`(^|[^a-z])${w}([^a-z]|$)`).test(t) : t.includes(w));
  const kindOf = (name) => Object.keys(KINDS).find((k) => String(name).includes(k)) || null;
  function match(text, options) {
    const t = String(text || "").toLowerCase().replace(/[.!?。]+$/g, "").trim();
    if (!t) return null;
    if (CANCEL.test(t)) return "cancel";
    let hits = options.filter((o) => (COLORS[o.color] || []).some((w) => word(t, w)));
    if (!hits.length && new Set(options.map((o) => kindOf(o.name))).size > 1) hits = options.filter((o) => (KINDS[kindOf(o.name)] || []).some((w) => word(t, w)));
    if (!hits.length) hits = options.filter((o, i) => (ORD[i] || []).some((w) => word(t, w)));
    if (!hits.length && /\blast\b|마지막/.test(t)) hits = options.slice(-1);
    if (!hits.length && /\bleft\b|왼/.test(t)) hits = options.slice(0, 1);
    if (!hits.length && /\bright\b|오른/.test(t)) hits = options.slice(-1);
    return hits.length === 1 ? hits[0].key : null;
  }

  function choose(key, how) {
    if (!cur) return;
    const o = cur.options.find((x) => x.key === key);
    if (!U.send({ type: "choose", id: cur.id, key })) { U.toast("Not connected to the robot."); return; }
    answered = cur.id;
    if (how !== "voice") U.say("user", key === "cancel" ? "Never mind." : `${o ? o.label : key}.`);
  }

  // typed or spoken answers ("two", "the red one", "취소") while a question is open
  window.addEventListener("pc-hear", (ev) => {
    if (!cur) return;
    const k = match(ev.detail, cur.options);
    if (k) { ev.preventDefault(); choose(k, "voice"); }
  });
  // number keys 1-4 answer too (not while typing)
  window.addEventListener("keydown", (ev) => {
    if (!cur || ev.metaKey || ev.ctrlKey || ev.altKey) return;
    const tg = ev.target, typing = tg && (tg.tagName === "INPUT" || tg.tagName === "TEXTAREA" || tg.isContentEditable);
    if (typing) return;
    const o = cur.options.find((x) => x.key === ev.key);
    if (o) { ev.preventDefault(); ev.stopPropagation(); choose(o.key, "key"); }
  }, true);

  // ---------------------------------------------------------------- question card (stage overlay slot)
  const card = document.createElement("section");
  card.className = "cand-card"; card.hidden = true;
  card.setAttribute("role", "group"); card.setAttribute("aria-live", "polite"); card.setAttribute("aria-label", "The robot asks which one");
  card.innerHTML = `<div class="cand-head">${ICON_Q}<div><p class="cand-kicker">Which one?</p><p class="cand-q"></p></div></div>
    <div class="cand-chips"></div>
    <div class="cand-foot"><span class="cand-hint">Tap one, say “two” or “the red one”, or point.</span>
    <button type="button" class="cand-cancel">${U.icon("x", 16)}<span>Cancel</span></button></div>`;
  card.querySelector(".cand-cancel").addEventListener("click", () => choose("cancel"));
  let mounted = false;
  function mountCard() { if (!mounted) { U.slot("stage").appendChild(card); mounted = true; } }

  function renderCard() {
    mountCard();
    card.hidden = !cur;
    if (!cur) return;
    card.querySelector(".cand-q").textContent = cur.question || "Which one do you mean?";
    const chips = card.querySelector(".cand-chips"); chips.textContent = "";
    for (const o of cur.options) {
      const b = document.createElement("button");
      b.type = "button"; b.className = "cand-chip"; b.dataset.key = o.key;
      b.setAttribute("aria-label", `${o.key}: ${o.label}`);
      b.innerHTML = `<span class="cand-num" aria-hidden="true">${o.key}</span><span class="cand-dot" aria-hidden="true"></span><span></span>`;
      b.lastChild.textContent = o.label;
      const sw = SWATCH[o.color] || (kindOf(o.name) === "mug" ? "#b22e33" : null);
      if (sw) b.querySelector(".cand-dot").style.background = sw;
      b.addEventListener("click", () => choose(o.key));
      b.addEventListener("pointerenter", () => setHot(o.key)); b.addEventListener("pointerleave", () => setHot(null));
      chips.appendChild(b);
    }
  }
  function setHot(key) {
    hot = key;
    for (const n of document.querySelectorAll(".cand-chip, .cand-badge")) n.classList.toggle("is-hot", n.dataset.key === key);
    paint3D();
  }

  // ---------------------------------------------------------------- robot-eye badges (HTML over #robotFrame)
  let eye = null;
  function eyeLayer() {
    const frame = document.getElementById("robotFrame");
    if (!frame) return null;
    if (!eye || eye.parentNode !== frame) {
      eye = document.createElement("div"); eye.className = "cand-eye"; eye.hidden = true;
      for (const t of ["pointerdown", "pointerup", "pointermove", "click"]) eye.addEventListener(t, (ev) => { if (ev.target !== eye) ev.stopPropagation(); });
      frame.appendChild(eye);
    }
    return eye;
  }
  function placeEye() {
    const layer = eyeLayer(); if (!layer) return;
    const img = document.getElementById("robotImg"), ov = document.getElementById("robotOverlay") || img;
    const v3d = document.getElementById("view3d"), in3d = !!(v3d && !v3d.hidden);
    const show = !!cur && !!ov && !in3d && (!U.state.view || U.state.view === "robot");
    layer.hidden = !show;
    if (!show) return;
    const fr = layer.getBoundingClientRect(), r = ov.getBoundingClientRect();
    const iw = (img && img.naturalWidth) || 640, ih = (img && img.naturalHeight) || 480, sc = Math.min(r.width / iw, r.height / ih);
    const cw = iw * sc, ch = ih * sc, cx = r.left - fr.left + (r.width - cw) / 2, cy = r.top - fr.top + (r.height - ch) / 2;
    for (const o of cur.options) {
      const b = layer.querySelector(`.cand-badge[data-key="${o.key}"]`), ring = layer.querySelector(`.cand-ring[data-key="${o.key}"]`);
      const ok = Array.isArray(o.uv) && o.uv.length >= 2 && o.uv.every(isFinite);
      if (!b) continue;
      b.hidden = ring.hidden = !ok;
      if (!ok) continue;
      const x = cx + o.uv[0] * cw, y = cy + o.uv[1] * ch;
      for (const n of [b, ring]) { n.style.left = `${x.toFixed(1)}px`; n.style.top = `${y.toFixed(1)}px`; }
    }
  }
  function renderEye() {
    const layer = eyeLayer(); if (!layer) return;
    layer.textContent = "";
    if (!cur) { layer.hidden = true; return; }
    for (const o of cur.options) {
      const ring = document.createElement("span"); ring.className = "cand-ring"; ring.dataset.key = o.key;
      const b = document.createElement("button");
      b.type = "button"; b.className = "cand-badge"; b.dataset.key = o.key; b.textContent = o.key;
      b.setAttribute("aria-label", `Choose ${o.key}: ${o.label}`); b.title = o.label;
      b.addEventListener("click", (ev) => { ev.stopPropagation(); choose(o.key); });
      b.addEventListener("pointerenter", () => setHot(o.key)); b.addEventListener("pointerleave", () => setHot(null));
      layer.append(ring, b);
    }
    placeEye();
  }
  window.addEventListener("resize", () => { if (cur) placeEye(); });

  // ---------------------------------------------------------------- 3D badges (PC3D layer "choices") + picking
  let V = null, down3d = null;
  const BADGE_UP = 0.11; // m above the object centre
  function paint3D() {
    if (!V) return;
    const g = V.layer("choices"), T = V.THREE;
    V.disposeGroup(g);
    if (cur) {
      for (const o of cur.options) {
        if (!Array.isArray(o.pos)) continue;
        const [x, y, z] = o.pos, isHot = hot === o.key;
        const col = isHot ? "#f59145" : "#5c77df";
        const sp = V.labelSprite(o.key, col, "#ffffff", 0.06); sp.position.set(x, y, z + BADGE_UP); g.add(sp);
        g.add(V.segmentMesh(new T.Vector3(x, y, z + 0.035), new T.Vector3(x, y, z + BADGE_UP - 0.03), 0.0025, isHot ? 0xf59145 : 0x5c77df, 0.9));
        const ring = new T.Mesh(new T.RingGeometry(0.05, 0.062, 40), new T.MeshBasicMaterial({ color: isHot ? 0xf59145 : 0x5c77df, transparent: true, opacity: 0.85, depthWrite: false, side: T.DoubleSide }));
        ring.position.set(x, y, (V.TABLE && isFinite(V.TABLE.z) ? V.TABLE.z : 0.74) + 0.004); ring.renderOrder = 9; g.add(ring);
      }
    }
    V.redraw();
  }
  function nearest3D(ev) {
    if (!cur || !V) return null;
    let best = null, bd = 40;
    for (const o of cur.options) {
      if (!Array.isArray(o.pos)) continue;
      for (const dz of [BADGE_UP, 0]) {
        const p = V.project(o.pos[0], o.pos[1], o.pos[2] + dz), d = Math.hypot(p[0] - ev.clientX, p[1] - ev.clientY);
        if (d < bd) { bd = d; best = o; }
      }
    }
    return best;
  }
  U.with3D((v) => {
    V = v; paint3D();
    v.capture(({ type, ev }) => {
      if (!cur) { down3d = null; return false; }
      if (type === "down") { const o = nearest3D(ev); down3d = o ? o.key : null; return !!o; }
      if (type === "up" && down3d) { const k = down3d; down3d = null; if (nearest3D(ev)?.key === k) choose(k); return true; }
      if (type === "move" && !down3d) { const o = nearest3D(ev); if ((o && o.key) !== hot) setHot(o ? o.key : null); }
      return false;
    });
  });

  // ---------------------------------------------------------------- state
  U.onState((s) => {
    const c = s && s.choices && Array.isArray(s.choices.options) && s.choices.options.length ? s.choices : null;
    const nsig = c ? c.id + "|" + c.question + "|" + c.options.map((o) => o.key + ":" + o.name).join(",") : "";
    const posSig = c ? c.options.map((o) => (o.pos || []).map((x) => x.toFixed(2)).join(",")).join(";") : "";
    cur = c;
    if (nsig !== sig) {
      sig = nsig; lastPos = posSig; hot = null; renderCard(); renderEye(); paint3D();
    } else if (c) {
      placeEye();
      if (posSig !== lastPos) { lastPos = posSig; paint3D(); }
    }
    // after an answer, say what the robot is doing now (the task card was already settled by the question)
    const t = s && s.task;
    if (answered && !c && t && t.id === answered && t.status !== "planning") {
      answered = null;
      if (t.message) U.say("robot", t.message);
    }
  });
})();
