// In-app guide: the "?" Help button in the top bar (or the ? key) opens a compact, scannable sheet: the 3 ways to
// instruct, the plan card, dock tools, 3D view, trajectories, Watch modes, camera & phone, gestures, packs, datasets,
// VR, safety, voice and keys. English / 한국어 (navigator.language picks; a toggle in the sheet switches and is
// remembered). A first-run tip strip under the top bar points to it until dismissed. Content: web/help-content.js
// (window.PCHelpContent, plain text only); full guide: docs/USAGE.md, docs/USAGE.ko.md.
"use strict";
(function () {
  const P = window.PCUI; if (!P) return;
  const C = window.PCHelpContent; if (!C || !C.en) return;
  const LS = {
    get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode: this visit only */ } },
  };
  const ACT = {   // buttons that act on a section (labels per language)
    en: { camera: [["cam", "Turn on camera"], ["phone", "Use my phone"]], gestures: [["gestures", "Open the gestures guide"]], view: [["reset", "Reset the 3D view"]] },
    ko: { camera: [["cam", "카메라 켜기"], ["phone", "휴대폰을 카메라로"]], gestures: [["gestures", "제스처 안내 열기"]], view: [["reset", "3D 시점 초기화"]] },
  };
  let lang = LS.get("pc.helpLang", "") || (/^ko/i.test(navigator.language || "") ? "ko" : "en");
  if (!C[lang]) lang = "en";

  const css = document.createElement("style");
  css.id = "pc-help-css";
  css.textContent = `
.help-dlg { width: min(940px, calc(100vw - 24px)); max-height: calc(100vh - 24px); }
.help-dlg[open] { display: flex; flex-direction: column; }
.help-dlg > .dlg-head { gap: 10px; flex: none; }
.help-dlg > .dlg-head h2 { flex: 1; min-width: 0; }
.help-lang { flex: none; }
.help-lang .seg-btn { height: 32px; font-size: var(--fs-xs); padding: 0 12px; }
.help-body { flex: 1 1 auto; min-height: 0; overflow-y: auto; padding: 14px 20px 20px; }
.help-lead { margin: 0 0 14px; font-size: var(--fs-md); line-height: 1.45; color: var(--t1); text-wrap: pretty; }
.help-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(min(100%, 270px), 1fr)); gap: 10px; align-items: start; }
.help-sec { margin: 0; padding: 12px 14px; border-radius: 16px; background: var(--s0); border: 1px solid var(--divider); }
.help-sec h3 { margin: 0 0 8px; font-size: var(--fs-sm); font-weight: 600; color: var(--t1); }
.help-sec dl { margin: 0; display: flex; flex-direction: column; gap: 8px; }
.help-sec dt { font-size: var(--fs-sm); font-weight: 600; color: var(--t1); display: flex; align-items: center; gap: 6px; }
.help-sec dd { margin: 1px 0 0; font-size: var(--fs-sm); line-height: 1.42; color: var(--t2); text-wrap: pretty; }
.help-sec kbd { margin-left: 0; font-weight: 600; color: var(--t1); background: var(--raised); }
.help-acts { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 10px; }
.help-foot { margin: 14px 0 0; font-size: var(--fs-xs); color: var(--t2); }
.tip-strip { flex: none; display: flex; align-items: center; gap: 10px; min-height: 44px; padding: 6px 10px 6px 20px; background: var(--accent-tint);
  border-bottom: 1px solid var(--line-soft); color: var(--t1); font-size: var(--fs-sm); position: relative; z-index: 15; }
.tip-strip[hidden] { display: none; }
.tip-strip .tip-txt { flex: 1; min-width: 0; line-height: 1.35; }
.tip-strip .btn { flex: none; }
.tip-x { flex: none; width: 36px; height: 36px; display: grid; place-items: center; border: 0; border-radius: 50%; background: transparent; color: var(--t2); padding: 0; position: relative; }
.tip-x::before { content: ""; position: absolute; inset: -4px; }
.tip-x:hover { background: var(--active); color: var(--t1); }
.tip-x:focus-visible { outline: none; box-shadow: var(--ring); }
.tip-x svg { width: 16px; height: 16px; }
@media (max-width: 760px) {
  .tip-strip { padding: 6px 6px 6px 14px; font-size: var(--fs-xs); }
  .tip-strip .btn { height: 36px; padding: 0 10px; }
  .help-body { padding: 12px 14px 16px; }
  .help-dlg > .dlg-head { padding-left: 14px; }
  .help-dlg > .dlg-head h2 { font-size: var(--fs-sm); }
}
`;
  document.head.appendChild(css);

  const svgX = '<svg viewBox="0 0 24 24" aria-hidden="true"><path fill="none" stroke="currentColor" stroke-width="1.75" stroke-linecap="round" d="M6 6l12 12M18 6 6 18"/></svg>';
  const dlg = document.createElement("dialog");
  dlg.className = "help-dlg"; dlg.id = "helpDlg"; dlg.setAttribute("aria-labelledby", "helpTitle");
  dlg.innerHTML = `
    <header class="dlg-head"><h2 id="helpTitle"></h2>
      <div class="seg help-lang" role="radiogroup" aria-label="Language / 언어">
        <button type="button" class="seg-btn" role="radio" data-lang="en">English</button><button type="button" class="seg-btn" role="radio" data-lang="ko">한국어</button>
      </div>
      <button type="button" class="icon-btn help-close">${svgX}</button></header>
    <div class="help-body"><p class="help-lead"></p><div class="help-grid"></div><p class="help-foot"></p></div>`;
  document.body.appendChild(dlg);
  const body = dlg.querySelector(".help-body"), grid = dlg.querySelector(".help-grid");

  function act(kind) {
    const PC = window.PC || {};
    if (kind === "cam") { dlg.close(); if (PC.camera) PC.camera.start(); }
    else if (kind === "phone") { dlg.close(); if (PC.camera) PC.camera.phone(); }
    else if (kind === "gestures") { dlg.close(); if (window.PCGestures) window.PCGestures.open(); }
    else if (kind === "reset") { dlg.close(); const b = document.querySelector('.v3d-views [data-cam="reset"]'); if (PC.setStageView) PC.setStageView("3d"); if (b) b.click(); }
  }
  function render() {
    const T = C[lang] || C.en;
    document.getElementById("helpTitle").textContent = T.title;
    dlg.querySelector(".help-lead").textContent = T.lead;
    dlg.querySelector(".help-foot").textContent = T.footer || "";
    const close = dlg.querySelector(".help-close"); close.setAttribute("aria-label", T.close || "Close"); close.title = T.close || "Close";
    dlg.querySelectorAll("[data-lang]").forEach((b) => { const on = b.dataset.lang === lang; b.setAttribute("aria-checked", String(on)); b.setAttribute("aria-pressed", String(on)); });
    dlg.setAttribute("lang", lang);
    grid.innerHTML = "";
    for (const sec of T.sections || []) {
      const box = document.createElement("section"); box.className = "help-sec"; box.dataset.sec = sec.id;
      const h = document.createElement("h3"); h.textContent = sec.title; box.appendChild(h);
      const dl = document.createElement("dl");
      for (const it of sec.items || []) {
        const dt = document.createElement("dt"); dt.textContent = it.k;
        if (it.kbd) { const k = document.createElement("kbd"); k.textContent = it.kbd; dt.appendChild(k); }
        const dd = document.createElement("dd"); dd.textContent = it.v;
        dl.append(dt, dd);
      }
      box.appendChild(dl);
      const acts = (ACT[lang] || ACT.en)[sec.id];
      if (acts) {
        const row = document.createElement("div"); row.className = "help-acts";
        for (const [k, label] of acts) { const b = document.createElement("button"); b.type = "button"; b.className = "btn sm"; b.textContent = label; b.addEventListener("click", () => act(k)); row.appendChild(b); }
        box.appendChild(row);
      }
      grid.appendChild(box);
    }
    renderTip();
  }
  function open(section) {
    render();
    if (typeof dlg.showModal === "function") { if (!dlg.open) dlg.showModal(); } else dlg.setAttribute("open", "");
    body.scrollTop = 0;
    if (section) { const s = grid.querySelector(`[data-sec="${section}"]`); if (s) s.scrollIntoView({ block: "start" }); }
    dismissTip();
  }
  dlg.querySelector(".help-close").addEventListener("click", () => dlg.close());
  dlg.addEventListener("click", (ev) => { if (ev.target === dlg) dlg.close(); });   // backdrop
  dlg.querySelectorAll("[data-lang]").forEach((b) => b.addEventListener("click", () => { lang = b.dataset.lang; LS.set("pc.helpLang", lang); render(); }));
  const btn = document.getElementById("helpBtn");
  if (btn) btn.addEventListener("click", () => open());
  document.addEventListener("keydown", (ev) => {
    const tg = ev.target;
    if (ev.key !== "?" || ev.metaKey || ev.ctrlKey || ev.altKey || (tg && (tg.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(tg.tagName)))) return;
    if (document.querySelector("dialog[open]")) return;
    ev.preventDefault(); open();
  });

  // ---------------------------------------------------------------- first-run tip strip (dismissible, remembered)
  const tip = document.createElement("div");
  tip.className = "tip-strip"; tip.id = "tipStrip"; tip.setAttribute("role", "note");
  tip.innerHTML = `<span class="tip-txt"></span><button type="button" class="btn sm primary tip-go"></button><button type="button" class="tip-x">${svgX}</button>`;
  const top = document.querySelector(".topbar");
  if (top && top.parentNode) top.parentNode.insertBefore(tip, top.nextSibling); else document.body.prepend(tip);
  tip.hidden = LS.get("pc.tipSeen", "0") === "1" || new URLSearchParams(location.search).get("tip") === "0";
  function renderTip() {
    const T = C[lang] || C.en;
    tip.setAttribute("lang", lang);
    tip.querySelector(".tip-txt").textContent = T.tip;
    tip.querySelector(".tip-go").textContent = T.tipCta;
    const x = tip.querySelector(".tip-x"); x.setAttribute("aria-label", T.close || "Close"); x.title = T.close || "Close";
  }
  function dismissTip() { if (!tip.hidden) { tip.hidden = true; LS.set("pc.tipSeen", "1"); } }
  tip.querySelector(".tip-go").addEventListener("click", () => open());
  tip.querySelector(".tip-x").addEventListener("click", dismissTip);
  renderTip();

  window.PCHelp = { open, close() { dlg.close(); }, get lang() { return lang; }, setLang(l) { if (C[l]) { lang = l; LS.set("pc.helpLang", l); render(); } } };
})();
