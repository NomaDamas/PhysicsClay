// Real (docs/RECON.md): bring real things into the simulation from photos.
//   An object:  take / choose 1-4 photos (or grab the live phone / browser camera), say what it is and how big if you
//               know, pick how to reconstruct it, check the outline and the 3D preview, adjust the size, Add to scene.
//   My table:   one photo of the table from where the robot stands -> the surface and the objects on it -> Use this layout.
// Server: POST /recon/import · /recon/scan (multipart), GET /recon/job/<id>[/mesh.bin|/overlay.jpg], /recon/info.json;
// websocket {type:"recon", op:"add"|"use_layout"|"import"|"scan"|"add_library"|"delete_library"}; state.recon.
// Keys (Gemini for recognition, fal.ai / Meshy for cloud meshes) stay in this browser and ride one request each.
"use strict";
(function () {
  const PCUI = window.PCUI;
  if (!PCUI) return;
  const LS = { get(k) { try { return localStorage.getItem(k) || ""; } catch (e) { return ""; } }, set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private */ } } };
  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const arr = (x) => (Array.isArray(x) ? x : []);
  const svg = (body, size = 18) => `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.75" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;
  const ICON = {
    real: '<path d="M4 8.5h3l1.6-2h6.8l1.6 2h3V19H4z"/><path d="m12 10.2 3 1.7v3.4l-3 1.7-3-1.7v-3.4z"/><path d="m9 11.9 3 1.7 3-1.7M12 13.6V17"/>',
    upload: '<path d="M12 16V4m0 0-4.5 4.5M12 4l4.5 4.5"/><path d="M4 15v3.5A1.5 1.5 0 0 0 5.5 20h13a1.5 1.5 0 0 0 1.5-1.5V15"/>',
    cam: '<path d="M4 8h3l2-2.5h6L17 8h3v11H4z"/><circle cx="12" cy="13.2" r="3.4"/>',
    table: '<path d="M3 9h18M5 9v10M19 9v10M7.5 9V6.5h3V9M14 9V5.5h2.5V9"/>',
  };

  const CSS = `
.rc-card { width: 100%; box-sizing: border-box; background: var(--raised); color: var(--t1); border: 1px solid var(--line);
  border-radius: var(--r3); box-shadow: var(--shadow-md); padding: 16px; display: flex; flex-direction: column; gap: 12px;
  font: 13px/1.45 var(--sans); max-height: calc(100vh - 120px); overflow: auto; }
.rc-card[hidden], .rc-card [hidden] { display: none !important; }
.rc-head { display: flex; align-items: center; gap: 10px; }
.rc-head h3 { margin: 0; font-size: 15px; font-weight: 600; flex: 1; }
.rc-ico { width: 32px; height: 32px; border-radius: 10px; display: grid; place-items: center; color: var(--accent-ink); background: var(--accent-tint); flex: none; }
.rc-btn { min-height: 44px; padding: 0 16px; border-radius: var(--r-ctl, 10px); border: 1px solid var(--line2); background: var(--raised); color: var(--t1);
  font: 500 13.5px var(--sans); cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 8px; }
.rc-btn:hover:not(:disabled) { background: var(--active); }
.rc-btn:focus-visible, .rc-seg button:focus-visible, .rc-in:focus-visible, .rc-sel:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.rc-btn:disabled { opacity: .45; cursor: default; }
.rc-btn.primary { background: var(--accent); border-color: transparent; color: var(--on-accent); font-weight: 600; }
.rc-btn.primary:hover:not(:disabled) { background: var(--inv-bg); color: var(--inv-fg); }
.rc-btn.icon { min-width: 44px; padding: 0; border-color: transparent; background: transparent; }
.rc-btn.danger { color: var(--bad-ink); }
.rc-seg { display: flex; gap: 4px; padding: 4px; border-radius: var(--r-ctl, 10px); background: var(--field); }
.rc-seg button { flex: 1; min-height: 40px; border: 0; border-radius: 8px; background: transparent; color: var(--t2); font: 500 13px var(--sans);
  cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 6px; }
.rc-seg button[aria-pressed="true"] { background: var(--raised); color: var(--t1); box-shadow: var(--shadow); }
.rc-hint { margin: 0; color: var(--t2); }
.rc-hint a { color: var(--accent-ink); }
.rc-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
.rc-row > .rc-btn { flex: 1 1 auto; }
.rc-file { position: absolute; width: 1px; height: 1px; opacity: 0; overflow: hidden; clip: rect(0 0 0 0); }
.rc-photo { position: relative; border-radius: var(--r-inner, 12px); overflow: hidden; background: var(--field); aspect-ratio: 4 / 3;
  display: grid; place-items: center; color: var(--t3); touch-action: none; user-select: none; }
.rc-photo img { width: 100%; height: 100%; object-fit: contain; display: block; pointer-events: none; }
.rc-crop { position: absolute; border: 2px solid var(--hot); border-radius: 4px; background: var(--hot-tint); pointer-events: none; }
.rc-thumbs { display: flex; gap: 6px; }
.rc-thumbs img { width: 52px; height: 52px; object-fit: cover; border-radius: 8px; border: 1px solid var(--line); }
.rc-field { display: flex; flex-direction: column; gap: 4px; flex: 1 1 140px; min-width: 0; }
.rc-field span { color: var(--t2); font-size: 12px; }
.rc-in, .rc-sel { height: 44px; padding: 0 12px; border-radius: var(--r-ctl, 10px); border: 1px solid var(--line2); background: var(--field); color: var(--t1);
  font: 500 13px var(--sans); min-width: 0; width: 100%; box-sizing: border-box; }
.rc-in:focus { background: var(--field-focus); }
.rc-lic { color: var(--t3); font-size: 11.5px; margin: -4px 0 0; }
.rc-keys { border-top: 1px solid var(--line-soft); padding-top: 8px; }
.rc-keys summary { cursor: pointer; color: var(--t2); min-height: 32px; display: flex; align-items: center; }
.rc-keys .rc-row { margin-top: 6px; }
.rc-steps { margin: 0; padding: 0; list-style: none; display: flex; flex-direction: column; gap: 2px; }
.rc-steps li { display: flex; align-items: center; gap: 10px; min-height: 30px; padding: 0 8px; border-radius: 10px; color: var(--t2); }
.rc-steps li .n { width: 18px; height: 18px; border-radius: 50%; border: 1.5px solid var(--line2); flex: none; display: grid; place-items: center; font: 600 10px var(--sans); }
.rc-steps li[data-s="active"] { background: var(--accent-tint); color: var(--t1); }
.rc-steps li[data-s="active"] .n { border-color: var(--accent); }
.rc-steps li[data-s="done"] .n { border-color: var(--ok); background: var(--ok); color: var(--on-accent); }
.rc-steps li[data-s="failed"] .n { border-color: var(--bad); background: var(--bad); color: var(--on-accent); }
.rc-steps li[data-s="skipped"] { opacity: .55; }
.rc-status { padding: 10px 12px; border-radius: 12px; background: var(--field); }
.rc-status[data-s="failed"] { background: var(--bad-tint); color: var(--bad-ink); }
.rc-status[data-s="ready"], .rc-status[data-s="added"] { background: var(--ok-tint); color: var(--ok-ink); }
.rc-status:empty { display: none; }
.rc-view { position: relative; border-radius: var(--r-inner, 12px); overflow: hidden; background: var(--stage-bg); aspect-ratio: 4 / 3; touch-action: none; }
.rc-view canvas { width: 100%; height: 100%; display: block; }
.rc-view .rc-tag { position: absolute; left: 8px; bottom: 8px; padding: 2px 8px; border-radius: 6px; background: var(--glass); color: var(--t2); font-size: 11.5px; }
.rc-facts { display: grid; grid-template-columns: auto 1fr; gap: 4px 12px; margin: 0; }
.rc-facts dt { color: var(--t2); }
.rc-facts dd { margin: 0; font-variant-numeric: tabular-nums; }
.rc-meter { display: inline-block; width: 56px; height: 6px; border-radius: 6px; background: var(--track); vertical-align: middle; margin-left: 6px; overflow: hidden; }
.rc-meter i { display: block; height: 100%; background: var(--ok); }
.rc-meter i.mid { background: var(--warn); } .rc-meter i.low { background: var(--bad); }
.rc-objs { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; gap: 4px; }
.rc-objs li { display: flex; align-items: center; gap: 10px; min-height: 36px; padding: 0 10px; border-radius: 10px; background: var(--field); }
.rc-objs .sw { width: 12px; height: 12px; border-radius: 4px; flex: none; }
.rc-objs .nm { flex: 1; text-transform: capitalize; }
.rc-objs .dt { color: var(--t2); font-size: 12px; font-variant-numeric: tabular-nums; }
.rc-map { width: 100%; height: auto; border-radius: 12px; background: var(--field); }
.rc-lib { display: grid; grid-template-columns: repeat(auto-fill, minmax(92px, 1fr)); gap: 8px; }
.rc-lib button { display: flex; flex-direction: column; gap: 4px; padding: 6px; border-radius: 12px; border: 1px solid var(--line); background: var(--raised);
  color: var(--t1); font: 500 12px var(--sans); cursor: pointer; text-align: left; min-height: 44px; }
.rc-lib img { width: 100%; aspect-ratio: 1; object-fit: cover; border-radius: 8px; background: var(--field); }
.rc-lib small { color: var(--t2); }
.rc-check { display: flex; align-items: center; gap: 8px; min-height: 44px; }
.rc-check input { width: 18px; height: 18px; }
@media (max-width: 760px) { .rc-card { max-height: 70vh; } .rc-row > .rc-btn { flex: 1 1 100%; } }`;
  const st = document.createElement("style"); st.id = "recon-css"; st.textContent = CSS; document.head.appendChild(st);

  const card = document.createElement("section");
  card.className = "rc-card"; card.hidden = true; card.setAttribute("aria-label", "Bring real things in from photos");
  card.innerHTML = `
  <div class="rc-head"><span class="rc-ico">${svg(ICON.real, 18)}</span><h3>From a real photo</h3>
    <button type="button" class="rc-btn icon" data-r="close" aria-label="Close" title="Close">${PCUI.icon("x", 18)}</button></div>
  <div class="rc-seg" role="group" aria-label="What to bring in">
    <button type="button" data-mode="object" aria-pressed="true">${svg(ICON.cam, 16)}An object</button>
    <button type="button" data-mode="space" aria-pressed="false">${svg(ICON.table, 16)}My table</button></div>

  <div data-pane="object">
    <p class="rc-hint">Photos of one thing, on a table if you can. Saying how big it is makes the size exact; an
      <a href="/recon/marker.png" target="_blank" rel="noopener">ArUco marker</a> printed at 5 cm next to it works too.</p>
    <input type="file" class="rc-file" data-f="object" accept="image/*" capture="environment" multiple aria-label="Photos of the object">
    <div class="rc-photo" data-ph="object" aria-label="Photo. Drag a box around the object to help me find it."><span>No photo yet</span></div>
    <div class="rc-thumbs" data-th="object"></div>
    <div class="rc-row">
      <button type="button" class="rc-btn" data-r="pick-object">${svg(ICON.upload, 16)}Take or choose photos</button>
      <button type="button" class="rc-btn" data-r="cam-object" title="Use the camera that is streaming (phone or browser)">${svg(ICON.cam, 16)}Live camera</button></div>
    <div class="rc-row">
      <label class="rc-field"><span>What is it? (optional)</span><input class="rc-in" data-i="label" placeholder="e.g. coffee mug" autocomplete="off"></label>
      <label class="rc-field"><span>How big? (optional)</span><input class="rc-in" data-i="size" placeholder="e.g. 9 cm tall" autocomplete="off"></label></div>
    <label class="rc-field"><span>Reconstruct with</span><select class="rc-sel" data-i="provider"></select></label>
    <p class="rc-lic" data-lic></p>
    <details class="rc-keys"><summary>Keys for cloud and recognition (stay in this browser)</summary>
      <div class="rc-row"><label class="rc-field"><span>fal.ai key</span><input class="rc-in" type="password" data-k="pc.falKey" autocomplete="off"></label>
        <label class="rc-field"><span>Meshy key</span><input class="rc-in" type="password" data-k="pc.meshyKey" autocomplete="off"></label></div>
      <p class="rc-lic">Recognition uses the Gemini key from Settings (or the server's). Hunyuan3D is not offered: its license excludes the EU, the UK and South Korea.</p></details>
    <div class="rc-row"><button type="button" class="rc-btn primary" data-r="go-object" disabled>Reconstruct</button></div>
  </div>

  <div data-pane="space" hidden>
    <p class="rc-hint">A photo of your table taken from where the robot would stand (its near edge at the bottom). A
      <a href="/recon/marker.png" target="_blank" rel="noopener">5 cm ArUco marker</a> lying on the table makes the sizes exact.</p>
    <input type="file" class="rc-file" data-f="space" accept="image/*" capture="environment" aria-label="Photo of the table">
    <div class="rc-photo" data-ph="space"><span>No photo yet</span></div>
    <div class="rc-row">
      <button type="button" class="rc-btn" data-r="pick-space">${svg(ICON.upload, 16)}Take or choose a photo</button>
      <button type="button" class="rc-btn" data-r="cam-space">${svg(ICON.cam, 16)}Live camera</button></div>
    <div class="rc-row"><button type="button" class="rc-btn primary" data-r="go-space" disabled>Scan the table</button></div>
  </div>

  <div class="rc-status" role="status" aria-live="polite"></div>
  <ol class="rc-steps" aria-label="Progress"></ol>

  <div data-res="object" hidden>
    <div class="rc-view"><canvas aria-label="3D preview of the reconstructed object (drag to turn)"></canvas><span class="rc-tag">3D preview · drag to turn</span></div>
    <div class="rc-photo" data-ov="object"></div>
    <dl class="rc-facts"></dl>
    <div class="rc-row"><label class="rc-field"><span>Height (cm)</span><input class="rc-in" data-i="height" type="number" min="1" max="60" step="0.1" inputmode="decimal"></label></div>
    <div class="rc-row">
      <button type="button" class="rc-btn" data-r="retry">Try again</button>
      <button type="button" class="rc-btn primary" data-r="add">${PCUI.icon("check", 16)}Add to scene</button></div>
  </div>

  <div data-res="space" hidden>
    <div class="rc-photo" data-ov="space"></div>
    <svg class="rc-map" viewBox="0 0 320 180" role="img" aria-label="Top view of the layout in front of the robot"></svg>
    <ul class="rc-objs" aria-label="Objects found"></ul>
    <div class="rc-row"><label class="rc-field"><span>Surface</span><select class="rc-sel" data-i="template">
      <option value="table">Table</option><option value="kitchen">Kitchen counter</option><option value="shelf">Shelf</option><option value="floor">Floor</option></select></label></div>
    <label class="rc-check"><input type="checkbox" data-i="real"> Use their real shapes (slower; saved to your objects)</label>
    <div class="rc-row"><button type="button" class="rc-btn primary" data-r="use">${PCUI.icon("check", 16)}Use this layout</button></div>
  </div>

  <details class="rc-keys" data-lib-wrap><summary data-lib-sum>Your objects</summary><div class="rc-lib" style="margin-top:8px"></div></details>`;
  PCUI.slot("side").appendChild(card);
  const $ = (s) => card.querySelector(s);
  const el = {
    status: $(".rc-status"), steps: $(".rc-steps"), prov: $('[data-i="provider"]'), lic: $("[data-lic]"),
    goObject: $('[data-r="go-object"]'), goSpace: $('[data-r="go-space"]'), facts: $(".rc-facts"), height: $('[data-i="height"]'),
    canvas: $(".rc-view canvas"), objs: $(".rc-objs"), map: $(".rc-map"), lib: $(".rc-lib"), libSum: $("[data-lib-sum]"),
    template: $('[data-i="template"]'), real: $('[data-i="real"]'), add: $('[data-r="add"]'), use: $('[data-r="use"]'),
  };

  let open = false, mode = "object", info = null, R = {};
  const photos = { object: [], space: [] };     // [{blob, url}]
  let crop = null;                               // [u0, v0, u1, v1] normalised, on the first object photo
  let job = { object: null, space: null }, detail = { object: null, space: null }, shown = { object: null, space: null };

  function setMode(m) {
    mode = m === "space" ? "space" : "object";
    card.querySelectorAll("[data-mode]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.mode === mode)));
    card.querySelectorAll("[data-pane]").forEach((p) => { p.hidden = p.dataset.pane !== mode; });
    render();
  }
  function setOpen(on) {
    open = !!on; card.hidden = !open;
    if (open) { loadInfo(); setMode(mode); }
  }
  PCUI.tool({ id: "real", label: "Real", icon: svg(ICON.real, 20), title: "Bring a real object or your table into the simulation from a photo",
    onSelect: () => setOpen(true), onDeselect: () => setOpen(false) });
  if (location.hash === "#real") setTimeout(() => PCUI.selectTool("real"), 400);

  // ------------------------------------------------------------------ info: providers, depth model, library
  async function loadInfo() {
    try {
      const r = await fetch("/recon/info.json", { headers: keyHeaders() }); info = await r.json();
    } catch (e) { info = null; }
    const cur = el.prov.value || LS.get("pc.reconProvider") || "primitive";
    el.prov.innerHTML = arr(info && info.providers).map((p) => {
      const ok = p.ok || (p.key && LS.get(p.key === "fal" ? "pc.falKey" : "pc.meshyKey"));
      return `<option value="${esc(p.id)}" ${ok ? "" : "disabled"}>${esc(p.label)}${ok ? "" : " — " + esc(p.why || "unavailable")}</option>`;
    }).join("") || `<option value="primitive">Quick shape (offline)</option>`;
    el.prov.value = [...el.prov.options].some((o) => o.value === cur && !o.disabled) ? cur : "primitive";
    showLicense(); renderLibrary();
  }
  function showLicense() {
    const p = arr(info && info.providers).find((x) => x.id === el.prov.value);
    const d = info && info.depth;
    el.lic.textContent = p ? `${p.note || ""} Model license: ${p.license}.` + (p.id === "primitive" && d ? ` Depth: Depth Anything V2 Small (Apache-2.0), ${d.ready ? "ready" : d.can_download ? `downloads once (~${d.size_mb} MB)` : "off (outline only)"}.` : "") : "";
  }
  el.prov.addEventListener("change", () => { LS.set("pc.reconProvider", el.prov.value); showLicense(); });
  card.querySelectorAll("[data-k]").forEach((inp) => {
    inp.value = LS.get(inp.dataset.k);
    inp.addEventListener("change", () => { LS.set(inp.dataset.k, inp.value.trim()); loadInfo(); });
  });
  function keyHeaders() {
    const h = {}, g = LS.get("pc.geminiKey"), f = LS.get("pc.falKey"), m = LS.get("pc.meshyKey");
    if (g) h["X-Gemini-Key"] = g; if (f) h["X-Fal-Key"] = f; if (m) h["X-Meshy-Key"] = m;
    return h;
  }
  function renderLibrary() {
    const objs = arr(info && info.library);
    el.libSum.textContent = objs.length ? `Your objects (${objs.length})` : "Your objects (none yet)";
    el.lib.innerHTML = objs.map((o) => `<button type="button" data-lib="${esc(o.id)}" title="Add the ${esc(o.label)} to the scene">
      <img alt="" loading="lazy" src="/recon/library/${encodeURIComponent(o.id)}/photo.jpg" onerror="this.style.visibility='hidden'">
      <span>${esc(o.label || o.id)}</span><small>${arr(o.size).map((v) => (v * 100).toFixed(0)).join("×")} cm · ${esc(o.provider || "")}</small></button>`).join("")
      || `<p class="rc-hint">Objects you bring in are kept here and can be added again.</p>`;
  }

  // ------------------------------------------------------------------ photos and the crop box
  async function downscale(file, side = 1600) {
    const url = URL.createObjectURL(file);
    try {
      const img = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = url; });
      const s = Math.min(1, side / Math.max(img.naturalWidth, img.naturalHeight));
      const cv = document.createElement("canvas");
      cv.width = Math.max(1, Math.round(img.naturalWidth * s)); cv.height = Math.max(1, Math.round(img.naturalHeight * s));
      cv.getContext("2d").drawImage(img, 0, 0, cv.width, cv.height);
      const blob = await new Promise((res) => cv.toBlob(res, "image/jpeg", 0.88));
      return { blob, url: URL.createObjectURL(blob) };
    } finally { URL.revokeObjectURL(url); }
  }
  card.querySelectorAll(".rc-file").forEach((inp) => inp.addEventListener("change", async () => {
    const which = inp.dataset.f, files = [...(inp.files || [])].filter((f) => /^image\//.test(f.type)).slice(0, which === "object" ? 4 : 1);
    inp.value = "";
    if (!files.length) { PCUI.toast("Please choose an image."); return; }
    try {
      for (const p of photos[which]) URL.revokeObjectURL(p.url);
      photos[which] = await Promise.all(files.map((f) => downscale(f)));
      if (which === "object") crop = null;
      showPhotos(which);
    } catch (e) { PCUI.toast("Couldn't read that image."); }
  }));
  function showPhotos(which) {
    const ph = $(`[data-ph="${which}"]`), first = photos[which][0];
    ph.innerHTML = first ? `<img alt="Your photo" src="${first.url}">${which === "object" ? '<div class="rc-crop" hidden></div>' : ""}` : "<span>No photo yet</span>";
    if (which === "object") {
      $('[data-th="object"]').innerHTML = photos.object.slice(1).map((p) => `<img alt="Another view" src="${p.url}">`).join("");
      drawCrop();
    }
    el.goObject.disabled = !photos.object.length; el.goSpace.disabled = !photos.space.length;
  }
  // drag a box on the first photo: the object is inside it (helps when there's clutter)
  (function cropper() {
    const ph = $('[data-ph="object"]'); let start = null;
    const rel = (ev) => {
      const img = ph.querySelector("img"); if (!img || !img.naturalWidth) return null;
      const r = ph.getBoundingClientRect(), s = Math.min(r.width / img.naturalWidth, r.height / img.naturalHeight);
      const w = img.naturalWidth * s, h = img.naturalHeight * s, ox = (r.width - w) / 2, oy = (r.height - h) / 2;
      return [Math.min(1, Math.max(0, (ev.clientX - r.left - ox) / w)), Math.min(1, Math.max(0, (ev.clientY - r.top - oy) / h))];
    };
    ph.addEventListener("pointerdown", (ev) => { const p = rel(ev); if (!p) return; start = p; ph.setPointerCapture(ev.pointerId); });
    ph.addEventListener("pointermove", (ev) => { if (!start) return; const p = rel(ev); if (!p) return; crop = [Math.min(start[0], p[0]), Math.min(start[1], p[1]), Math.max(start[0], p[0]), Math.max(start[1], p[1])]; drawCrop(); });
    ph.addEventListener("pointerup", () => { if (crop && (crop[2] - crop[0] < 0.03 || crop[3] - crop[1] < 0.03)) crop = null; start = null; drawCrop(); });
  })();
  function drawCrop() {
    const ph = $('[data-ph="object"]'), box = ph.querySelector(".rc-crop"), img = ph.querySelector("img");
    if (!box || !img) return;
    box.hidden = !crop; if (!crop || !img.naturalWidth) return;
    const r = ph.getBoundingClientRect(), s = Math.min(r.width / img.naturalWidth, r.height / img.naturalHeight);
    const w = img.naturalWidth * s, h = img.naturalHeight * s, ox = (r.width - w) / 2, oy = (r.height - h) / 2;
    Object.assign(box.style, { left: `${ox + crop[0] * w}px`, top: `${oy + crop[1] * h}px`, width: `${(crop[2] - crop[0]) * w}px`, height: `${(crop[3] - crop[1]) * h}px` });
  }

  // ------------------------------------------------------------------ sending
  async function upload(which) {
    const fd = new FormData();
    for (const p of photos[which]) fd.append("photo", p.blob, "photo.jpg");
    const hints = which === "object" ? { label: $('[data-i="label"]').value.trim() || undefined, size: $('[data-i="size"]').value.trim() || undefined, crop: crop || undefined } : {};
    fd.append("meta", JSON.stringify({ provider: el.prov.value || "primitive", hints, tag: "ui" }));
    setStatus("running", which === "object" ? "Sending your photos…" : "Sending your photo…");
    try {
      const r = await fetch(which === "object" ? "/recon/import" : "/recon/scan", { method: "POST", body: fd, headers: keyHeaders() });
      const j = await r.json();
      if (!j.ok) { setStatus("failed", j.message || "The upload failed."); return; }
      startJob(which, j.job);
      PCUI.say("you", which === "object" ? "Bring this into the simulation." : "Here's my table.");
    } catch (e) { setStatus("failed", "Couldn't reach the robot server."); }
  }
  function fromCamera(which) {
    const k = { gemini: LS.get("pc.geminiKey") || undefined, fal: LS.get("pc.falKey") || undefined, meshy: LS.get("pc.meshyKey") || undefined };
    const id = `rc-${Date.now()}`;
    const msg = which === "object"
      ? { type: "recon", op: "import", source: "camera", id, provider: el.prov.value || "primitive", keys: k, hints: { label: $('[data-i="label"]').value.trim() || undefined, size: $('[data-i="size"]').value.trim() || undefined } }
      : { type: "recon", op: "scan", source: "camera", id, keys: k };
    if (!PCUI.send(msg)) { PCUI.toast("Not connected to the robot."); return; }
    pendingCam = { id, which };
    setStatus("running", "Taking a picture with the live camera…");
  }
  let pendingCam = null;
  function startJob(which, id) { job[which] = id; detail[which] = null; shown[which] = null; render(); }

  card.addEventListener("click", (e) => {
    const b = e.target.closest("button"); if (!b || !card.contains(b)) return;
    if (b.dataset.mode) return setMode(b.dataset.mode);
    if (b.dataset.lib) return addFromLibrary(b.dataset.lib);
    switch (b.dataset.r) {
      case "close": PCUI.activeTool === "real" ? PCUI.selectTool(null) : setOpen(false); break;
      case "pick-object": $('[data-f="object"]').click(); break;
      case "pick-space": $('[data-f="space"]').click(); break;
      case "cam-object": fromCamera("object"); break;
      case "cam-space": fromCamera("space"); break;
      case "go-object": upload("object"); break;
      case "go-space": upload("space"); break;
      case "retry": job.object = null; detail.object = null; render(); setStatus("", ""); $('[data-ph="object"]').scrollIntoView({ block: "nearest" }); break;
      case "add": addObject(); break;
      case "use": useLayout(); break;
    }
  });
  function addObject() {
    const j = detail.object; if (!j) return;
    const h = parseFloat(el.height.value), cur = j.summary && j.summary.size_cm ? j.summary.size_cm[2] : null;
    const msg = { type: "recon", op: "add", job: j.id, id: `rc-add-${Date.now()}` };
    if (isFinite(h) && h > 0 && cur && Math.abs(h - cur) > 0.05) msg.height_m = h / 100;
    if (!PCUI.send(msg)) PCUI.toast("Not connected to the robot.");
  }
  function useLayout() {
    const j = detail.space; if (!j) return;
    if (!PCUI.send({ type: "recon", op: "use_layout", job: j.id, shapes: el.real.checked ? "real" : "primitive", template: el.template.value, id: `rc-use-${Date.now()}` })) PCUI.toast("Not connected to the robot.");
  }
  function addFromLibrary(id) {
    if (!PCUI.send({ type: "recon", op: "add_library", id, id_task: `rc-lib-${Date.now()}` })) PCUI.toast("Not connected to the robot.");
  }

  // ------------------------------------------------------------------ progress and results
  function setStatus(s, text) { el.status.dataset.s = s || ""; el.status.textContent = text || ""; }
  const pct = (v) => Math.round((v || 0) * 100);
  const meter = (c) => `<span class="rc-meter" aria-hidden="true"><i class="${c < 0.4 ? "low" : c < 0.7 ? "mid" : ""}" style="width:${pct(c)}%"></i></span>`;
  function render() {
    const jid = job[mode], js = arr(R.jobs).find((x) => x.id === jid);
    el.steps.innerHTML = js ? arr(js.steps).map((s, i) => `<li data-s="${esc(s.state)}"><span class="n">${s.state === "done" ? "✓" : s.state === "failed" ? "!" : i + 1}</span>${esc(s.label)}</li>`).join("") : "";
    el.steps.hidden = !js || js.status === "ready" || js.status === "added";
    if (js) setStatus(js.status, js.status === "failed" ? js.error || js.message : js.message);
    card.querySelector('[data-res="object"]').hidden = !(mode === "object" && detail.object);
    card.querySelector('[data-res="space"]').hidden = !(mode === "space" && detail.space);
  }
  async function fetchDetail(which, id) {
    try {
      const r = await fetch(`/recon/job/${id}`, { cache: "no-store" }); if (!r.ok) return;
      detail[which] = await r.json(); shown[which] = id;
      if (which === "object") showObject(detail.object); else showSpace(detail.space);
      render();
    } catch (e) { /* try again on the next state */ }
  }
  function showObject(j) {
    const s = j.summary || {}, sc = s.scale || {};
    $('[data-ov="object"]').innerHTML = `<img alt="The outline I found" src="/recon/job/${j.id}/overlay.jpg?t=${Date.now()}">`;
    const dims = arr(s.size_cm);
    el.facts.innerHTML = `
      <dt>It is</dt><dd>${esc(s.label)} · ${esc(s.material)}${s.hollow ? " (hollow)" : ""}</dd>
      <dt>Shape</dt><dd>${esc(s.shape)} by ${esc((s.provider || {}).label)}</dd>
      <dt>Size</dt><dd>${dims.map((v) => Number(v).toFixed(1)).join(" × ")} cm</dd>
      <dt>Size from</dt><dd>${esc(sc.label || "?")} ${meter(sc.confidence || 0)} ${pct(sc.confidence)}%</dd>
      <dt>Mass</dt><dd>${s.mass_g} g · friction ${s.friction}</dd>
      <dt>Collision</dt><dd>${s.hulls} convex piece${s.hulls === 1 ? "" : "s"} (${esc(s.collision)})${s.watertight ? "" : " · repaired"}</dd>
      <dt>Confidence</dt><dd>${meter(s.confidence || 0)} ${pct(s.confidence)}%${s.views > 1 ? ` · ${s.views} views` : ""}</dd>
      ${arr(s.notes).length ? `<dt>Notes</dt><dd>${arr(s.notes).map(esc).join("; ")}</dd>` : ""}`;
    el.height.value = dims[2] ? Number(dims[2]).toFixed(1) : "";
    preview(j.id);
  }
  function showSpace(j) {
    const s = j.summary || {}, lay = s.layout || {}, objs = arr(lay.objects);
    $('[data-ov="space"]').innerHTML = `<img alt="What I found on your table" src="/recon/job/${j.id}/overlay.jpg?t=${Date.now()}">`;
    el.template.value = s.template || "table";
    const C = { red: "#eb5c54", green: "#4db873", blue: "#3d8ced", yellow: "#fad140", orange: "#fa9433", purple: "#9466d9", pink: "#f58cb8", white: "#f0f0eb", black: "#26262b", gray: "#8c8f99", silver: "#bfc4cc", brown: "#8c613d" };
    el.objs.innerHTML = objs.map((o) => `<li><span class="sw" style="background:${C[o.color] || "#999"}"></span><span class="nm">${esc(o.color)} ${esc(o.label)}</span>
      <span class="dt">${arr(o.size_m).map((v) => (v * 100).toFixed(0)).join("×")} cm</span></li>`).join("") || `<li>No objects found.</li>`;
    // top view in front of the robot: robot at the bottom, x forward = up on screen, y left = left
    const spec = s.spec || { objects: [] }, sx = 320 / 1.2, cx = 160, by = 170;
    const pts = arr(spec.objects).map((o) => {
      const [x, y] = o.pos || [0, 0], [dx, dy] = o.size || [0.05, 0.05];
      return `<rect x="${cx - y * sx - dy * sx / 2}" y="${by - (x - 0.1) * sx - dx * sx / 2}" width="${Math.max(4, dy * sx)}" height="${Math.max(4, dx * sx)}" rx="3" fill="${C[o.color] || "#999"}" opacity=".9"><title>${esc(o.name)}</title></rect>`;
    }).join("");
    el.map.innerHTML = `<rect x="${cx - 0.5 * sx}" y="${by - 0.72 * sx}" width="${sx}" height="${0.6 * sx}" rx="6" fill="var(--stage-floor)" stroke="var(--line2)"/>
      <circle cx="${cx}" cy="${by}" r="7" fill="var(--accent)"/><text x="${cx + 12}" y="${by + 4}" font-size="11" fill="var(--t2)">robot</text>${pts}`;
  }

  // ------------------------------------------------------------------ 3D preview (three.js from the stage module)
  let tt = null;
  function preview(id) {
    const THREE = window.PC3D && window.PC3D.THREE;
    if (!THREE) { window.addEventListener("pc3d-ready", () => preview(id), { once: true }); return; }
    if (!tt) {
      const renderer = new THREE.WebGLRenderer({ canvas: el.canvas, antialias: true, alpha: true });
      renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
      const scene = new THREE.Scene(), cam = new THREE.PerspectiveCamera(32, 4 / 3, 0.01, 20);
      cam.up.set(0, 0, 1);
      scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.6));
      const sun = new THREE.DirectionalLight(0xffffff, 1.4); sun.position.set(1, -1.5, 2); scene.add(sun);
      const group = new THREE.Group(); scene.add(group);
      tt = { THREE, renderer, scene, cam, group, yaw: 0.6, drag: null, mesh: null, raf: 0, size: 0.2 };
      el.canvas.addEventListener("pointerdown", (ev) => { tt.drag = ev.clientX; el.canvas.setPointerCapture(ev.pointerId); });
      el.canvas.addEventListener("pointermove", (ev) => { if (tt.drag == null) return; tt.yaw += (ev.clientX - tt.drag) * 0.012; tt.drag = ev.clientX; });
      el.canvas.addEventListener("pointerup", () => { tt.drag = null; });
    }
    fetch(`/recon/job/${id}/mesh.bin`, { cache: "no-store" }).then((r) => r.arrayBuffer()).then((buf) => {
      const T = tt.THREE, dv = new DataView(buf), nv = dv.getUint32(0, true), nf = dv.getUint32(4, true);
      const g = new T.BufferGeometry();
      g.setAttribute("position", new T.BufferAttribute(new Float32Array(buf.slice(8, 8 + nv * 12)), 3));
      g.setIndex(new T.BufferAttribute(new Uint32Array(buf.slice(8 + nv * 12, 8 + nv * 12 + nf * 12)), 1));
      const off = 8 + nv * 12 + nf * 12, hasCol = buf.byteLength >= off + nv * 3;
      if (hasCol) g.setAttribute("color", new T.BufferAttribute(new Uint8Array(buf.slice(off, off + nv * 3)), 3, true));
      g.computeVertexNormals(); g.computeBoundingSphere();
      if (tt.mesh) { tt.group.remove(tt.mesh); tt.mesh.geometry.dispose(); tt.mesh.material.dispose(); }
      tt.mesh = new T.Mesh(g, new T.MeshStandardMaterial({ vertexColors: hasCol, color: hasCol ? 0xffffff : 0xb8bcc6, roughness: 0.7, metalness: 0.02 }));
      tt.group.add(tt.mesh);
      tt.size = Math.max(0.02, g.boundingSphere.radius);
      if (!tt.raf) loop();
    }).catch(() => {});
  }
  const reduce = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  function loop() {
    tt.raf = requestAnimationFrame(loop);
    if (card.hidden || card.querySelector('[data-res="object"]').hidden) return;
    const c = el.canvas, w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) return;
    if (c.width !== Math.round(w * tt.renderer.getPixelRatio())) { tt.renderer.setSize(w, h, false); tt.cam.aspect = w / h; tt.cam.updateProjectionMatrix(); }
    if (tt.drag == null && !reduce) tt.yaw += 0.006;
    const d = tt.size * 3.4;
    tt.cam.position.set(Math.cos(tt.yaw) * d, Math.sin(tt.yaw) * d, d * 0.55); tt.cam.lookAt(0, 0, 0);
    tt.renderer.render(tt.scene, tt.cam);
  }

  // ------------------------------------------------------------------ state
  let lastReply = null;
  PCUI.onState((s) => {
    R = s.recon || {};
    const rep = R.reply;
    if (rep && rep.id !== lastReply) {
      lastReply = rep.id;
      if (pendingCam && rep.id === pendingCam.id) { if (rep.ok && rep.job) startJob(pendingCam.which, rep.job); pendingCam = null; }
      if (!rep.ok) { setStatus("failed", rep.message); PCUI.toast(rep.message); }
      else if (rep.op === "add" || rep.op === "use_layout" || rep.op === "add_library") { PCUI.say("robot", rep.message); setTimeout(loadInfo, 1500); }
    }
    for (const which of ["object", "space"]) {
      const js = arr(R.jobs).find((x) => x.id === job[which]);
      if (js && (js.status === "ready" || js.status === "added") && shown[which] !== js.id) fetchDetail(which, js.id);
      if (js && js.status === "ready" && detail[which] && detail[which].status !== "ready" && which === "space" && shown[which] === js.id) fetchDetail(which, js.id);
    }
    const busy = arr(R.jobs).some((x) => x.status === "running" || x.status === "queued");
    el.goObject.disabled = busy || !photos.object.length; el.goSpace.disabled = busy || !photos.space.length;
    if (open) render();
  });
})();
