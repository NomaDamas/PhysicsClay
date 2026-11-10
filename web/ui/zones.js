// Keep-out zones (UX 2.0 #D): "not here". Zone tool: drag a rectangle on the table in the 3D stage. Saying or typing
// "stay away from the vase" / "don't go near the mug" adds a zone around that object. Zones show as red translucent
// volumes in PC3D.layer("zones") and are listed (removable) in the "side" slot. Server side: pps/features/zones.py.
"use strict";
(function () {
  const UI = window.PCUI;
  if (!UI) return;
  const RED = 0xd6453a;
  const MIN = 0.03; // smallest painted side (m)

  // ------------------------------------------------------------------ styles (existing tokens only)
  const css = `
.pcz { background: var(--raised); border: 1px solid var(--line); border-radius: var(--r-inner, 18px); box-shadow: var(--shadow);
  padding: 12px 12px 8px 16px; display: flex; flex-direction: column; gap: 4px; font-family: var(--sans); color: var(--t1); }
.pcz-head { display: flex; align-items: center; gap: 8px; min-height: 44px; }
.pcz-head svg { color: var(--bad); flex: none; }
.pcz-title { font-size: var(--fs-sm, 13px); font-weight: 600; }
.pcz-count { font-size: var(--fs-xs, 12px); color: var(--t3); font-variant-numeric: tabular-nums; }
.pcz-head .grow { flex: 1; }
.pcz-btn { min-height: 44px; min-width: 44px; padding: 0 12px; border-radius: 999px; border: 1px solid transparent; background: transparent;
  color: var(--t2); font: 500 var(--fs-xs, 12px) var(--sans); cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 6px; }
.pcz-btn:hover { background: var(--active); color: var(--t1); }
.pcz-btn:focus-visible { outline: none; box-shadow: var(--ring); }
.pcz-list { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; }
.pcz-row { display: flex; align-items: center; gap: 10px; min-height: 44px; border-top: 1px solid var(--divider); }
.pcz-row:first-child { border-top: 0; }
.pcz-sw { width: 14px; height: 14px; border-radius: 4px; flex: none; background: color-mix(in srgb, var(--bad) 22%, transparent); border: 1.5px solid var(--bad); }
.pcz-row[data-shape="circle"] .pcz-sw { border-radius: 50%; }
.pcz-txt { display: flex; flex-direction: column; min-width: 0; flex: 1; }
.pcz-nm { font-size: var(--fs-sm, 13px); font-weight: 600; }
.pcz-meta { font-size: var(--fs-xs, 12px); color: var(--t3); font-variant-numeric: tabular-nums; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.pcz-row .pcz-btn { padding: 0; width: 44px; }
.pcz-row .pcz-btn:hover { color: var(--bad-ink); }
.pcz-hint { background: var(--raised); border: 1px solid var(--line); border-left: 3px solid var(--bad); border-radius: 14px; box-shadow: var(--shadow);
  padding: 10px 14px; max-width: 320px; font: 500 var(--fs-sm, 13px) var(--sans); color: var(--t1); }
.pcz-hint b { color: var(--bad-ink); font-weight: 600; }`;
  const st = document.createElement("style"); st.id = "pcui-zones-css"; st.textContent = css; document.head.appendChild(st);

  // ------------------------------------------------------------------ state
  let zones = [], memo = "", hot = null, draft = null, release = null, hintEl = null, seen = null;
  const fmt = (m) => `${Math.round(m * 100)}`;
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

  // ------------------------------------------------------------------ 3D volumes
  let waiting = false;
  function draw() { // paint now if the 3D stage is up, else once when it mounts
    const v = window.PC3D;
    if (v && v.mounted) { paint(v); return; }
    if (!waiting) { waiting = true; UI.with3D((p) => { waiting = false; paint(p); }); }
  }
  function paint(v) {
    {
      const T = v.THREE, g = v.layer("zones"); v.disposeGroup(g);
      const zt = v.TABLE.z;
      for (const z of zones) {
        const [cx, cy] = z.center, h = Math.max(0.01, +z.height || 0.3), [w, d] = z.size, on = z.id === hot;
        let geo;
        if (z.shape === "circle") { geo = new T.CylinderGeometry(w / 2, w / 2, h, 48, 1, true); geo.rotateX(Math.PI / 2); }
        else geo = new T.BoxGeometry(w, d, h);
        const vol = new T.Mesh(geo, new T.MeshBasicMaterial({ color: RED, transparent: true, opacity: on ? 0.3 : 0.16, depthWrite: false, side: T.DoubleSide }));
        vol.position.set(cx, cy, zt + h / 2); vol.renderOrder = 2; g.add(vol);
        const edges = new T.LineSegments(new T.EdgesGeometry(z.shape === "circle" ? new T.CylinderGeometry(w / 2, w / 2, h, 48).rotateX(Math.PI / 2) : geo, 20),
          new T.LineBasicMaterial({ color: RED, transparent: true, opacity: on ? 1 : 0.65 }));
        edges.position.copy(vol.position); g.add(edges);
        const foot = new T.Mesh(z.shape === "circle" ? new T.CircleGeometry(w / 2, 48) : new T.PlaneGeometry(w, d),
          new T.MeshBasicMaterial({ color: RED, transparent: true, opacity: on ? 0.4 : 0.26, depthWrite: false }));
        foot.position.set(cx, cy, zt + 0.003); g.add(foot);
        const lab = v.labelSprite(z.name || z.id, "#d6453a", "#ffffff", 0.036); lab.position.set(cx, cy, zt + h + 0.03); g.add(lab);
      }
      if (draft) {
        const [x0, y0, x1, y1] = draft.rect(), w = Math.max(0.005, x1 - x0), d = Math.max(0.005, y1 - y0), h = 0.3;
        const box = new T.BoxGeometry(w, d, h), ok = w >= MIN && d >= MIN;
        const vol = new T.Mesh(box, new T.MeshBasicMaterial({ color: ok ? RED : 0x9a9aa0, transparent: true, opacity: 0.12, depthWrite: false }));
        vol.position.set((x0 + x1) / 2, (y0 + y1) / 2, zt + h / 2); g.add(vol);
        const e = new T.LineSegments(new T.EdgesGeometry(box), new T.LineBasicMaterial({ color: ok ? RED : 0x9a9aa0 }));
        e.position.copy(vol.position); g.add(e);
      }
      v.redraw();
    }
  }

  // ------------------------------------------------------------------ list (side slot)
  let card = null;
  function renderList() {
    if (!zones.length) { if (card) { card.remove(); card = null; } return; }
    if (!card) {
      card = document.createElement("section"); card.className = "pcz"; card.setAttribute("aria-label", "Keep-out zones");
      card.innerHTML = `<div class="pcz-head">${UI.icon("zone", 18)}<span class="pcz-title">Keep-out zones</span><span class="pcz-count"></span>
        <span class="grow"></span><button type="button" class="pcz-btn" data-zone-clear title="Remove every keep-out zone">Clear all</button></div><ul class="pcz-list"></ul>`;
      card.querySelector("[data-zone-clear]").addEventListener("click", () => UI.send({ type: "zone", op: "clear" }));
      card.addEventListener("click", (e) => {
        const b = e.target.closest("[data-zone-rm]"); if (b) UI.send({ type: "zone", op: "remove", id: b.dataset.zoneRm });
      });
      card.addEventListener("pointerover", (e) => { const r = e.target.closest(".pcz-row"); setHot(r ? r.dataset.id : null); });
      card.addEventListener("pointerleave", () => setHot(null));
      UI.slot("side").appendChild(card);
    }
    card.querySelector(".pcz-count").textContent = String(zones.length);
    card.querySelector(".pcz-list").innerHTML = zones.map((z) => {
      const dims = z.shape === "circle" ? `${fmt(z.size[0])} cm across` : `${fmt(z.size[0])} × ${fmt(z.size[1])} cm`;
      const how = z.source === "paint" ? "painted" : z.source === "voice" ? "said" : z.source === "text" ? "typed" : esc(z.source || "");
      return `<li class="pcz-row" data-id="${esc(z.id)}" data-shape="${esc(z.shape)}"><span class="pcz-sw" aria-hidden="true"></span>
        <span class="pcz-txt"><span class="pcz-nm">${esc(z.name || "Zone " + z.id)}</span><span class="pcz-meta">${dims} · ${fmt(z.height)} cm tall · ${how}</span></span>
        <button type="button" class="pcz-btn" data-zone-rm="${esc(z.id)}" aria-label="Remove ${esc(z.name || "zone " + z.id)}" title="Remove">${UI.icon("x", 18)}</button></li>`;
    }).join("");
  }
  function setHot(id) { if (id === hot) return; hot = id; draw(); }

  UI.onState((s) => {
    const list = Array.isArray(s && s.zones) ? s.zones.filter((z) => z && Array.isArray(z.center) && Array.isArray(z.size)) : [];
    const j = JSON.stringify(list); if (j === memo) return; memo = j;
    if (seen) for (const z of list) if (!seen.has(z.id) && z.source === "paint") UI.say("robot", `Got it, I'll keep out of ${z.label}.`);
    seen = new Set(list.map((z) => z.id));
    zones = list; renderList(); draw();
  });

  // ------------------------------------------------------------------ Zone tool: drag a rectangle on the table
  function clampT(p, T) { return [Math.min(T.x1, Math.max(T.x0, p.x)), Math.min(T.y1, Math.max(T.y0, p.y))]; }
  function onPointer({ type, ev, hit }) {
    const v = window.PC3D;
    if (type === "down") {
      if (ev.button !== 0 || !hit || !v) return false; // off the table: orbit as usual
      const a = clampT(hit, v.TABLE);
      draft = { a, b: a.slice(), rect() { return [Math.min(this.a[0], this.b[0]), Math.min(this.a[1], this.b[1]), Math.max(this.a[0], this.b[0]), Math.max(this.a[1], this.b[1])]; } };
      try { ev.target.setPointerCapture(ev.pointerId); } catch (e) { /* noop */ }
      draw(); return true;
    }
    if (!draft) return false;
    if (hit) draft.b = clampT(hit, v.TABLE);
    if (type === "move") { draw(); return true; }
    if (type === "up") {
      const r = draft.rect(); draft = null;
      if (r[2] - r[0] < MIN || r[3] - r[1] < MIN) { draw(); hint("Drag a bigger area: at least 3 cm on each side."); return true; }
      UI.send({ type: "zone", op: "add", shape: "rect", rect: r.map((x) => Math.round(x * 1000) / 1000), source: "paint" });
      draw(); hint(); return true;
    }
    return false;
  }
  function hint(extra) {
    if (UI.activeTool !== "zone") { if (hintEl) { hintEl.remove(); hintEl = null; } return; }
    if (!hintEl) { hintEl = document.createElement("div"); hintEl.className = "pcz-hint"; hintEl.setAttribute("role", "status"); UI.slot("stage").appendChild(hintEl); }
    hintEl.innerHTML = `<b>Keep-out zone</b> · drag on the table to mark where I must not go. ${extra ? esc(extra) : "Or say “stay away from the vase”."}`;
  }
  UI.tool({
    id: "zone", label: "Zone", icon: "zone", title: "Keep-out zone: drag a rectangle on the table in 3D",
    onSelect() {
      const v = window.PC3D, btn = document.querySelector('.seg-btn[data-view="3d"]');
      if (!(v && v.isActive) && btn) btn.click(); // painting happens on the 3D stage
      UI.with3D((p) => { if (UI.activeTool === "zone" && !release) release = p.capture(onPointer); });
      hint();
    },
    onDeselect() { if (release) release(); release = null; draft = null; draw(); hint(); },
  });

  // ------------------------------------------------------------------ "stay away from the vase" (typed or spoken)
  const ADD = [
    /\b(?:stay|keep|get|move)\s+(?:well\s+|far\s+)?(?:away|clear)\s+(?:from|of)\s+(.+)$/,
    /\b(?:don'?t|do not|never|dont)\s+(?:go|come|get|move|reach)\s+(?:near|close to|next to|by|over)\s+(.+)$/,
    /\b(?:don'?t|do not|never|dont)\s+(?:touch|hit|bump(?:\s+into)?|knock(?:\s+over)?)\s+(.+)$/,
    /\bwithout\s+(?:touching|hitting|going near|bumping into|knocking over)\s+(.+)$/,
    /\b(?:avoid|keep out of)\s+(.+)$/,
  ];
  const KO_ADD = /(꽃병|화병|컵|머그|빨간\s*블록|파란\s*블록|빨간|파란)\S*\s*(?:근처|가까이|옆|쪽)?\S*\s*(?:가지\s*마|가면\s*안|오지\s*마|피해|건드리지\s*마|닿지\s*마)/;
  const CLEAR = /\b(?:clear|remove|delete|forget)\s+(?:all\s+)?(?:the\s+|my\s+)?(?:keep[- ]?out\s+)?zones\b|\b(?:clear|remove|delete)\s+(?:all\s+)?(?:the\s+)?keep[- ]?out zones?\b|구역\s*(?:다\s*)?(?:지워|삭제)/;
  const REMOVE = /\b(?:remove|delete|clear)\s+(?:the\s+)?(?:keep[- ]?out\s+)?zone\s+([a-z])\b/;
  const PLACE = /\b(pick|put|move|place|bring|carry|set|drop|take)\b|옮겨|놓아|놔|올려|가져/;
  function objectOf(s) {
    s = s.split(/\s*(?:,|;|\band\b|\bthen\b|\bwhile\b|\bbut\b|\bplease\b|\bwhen\b|\bas\b)\s*/)[0];
    s = s.replace(/^(?:the|that|this|my|your|a|an)\s+/, "").replace(/\s+(?:again|anymore|at all|too)$/, "").trim();
    return s && !/^(it|them|that|this|there|here|me)$/.test(s) ? s : null;
  }
  function hear(text) {
    const tp = (UI.state.teach && UI.state.teach.phase) || "idle";
    if (tp !== "idle") return false; // while teaching, "don't touch the vase" is a constraint for the taught task
    const raw = String(text || "").trim(), t = raw.toLowerCase().replace(/[.!?。]+$/g, "").trim();
    if (!t) return false;
    let m = t.match(REMOVE);
    if (m) { UI.send({ type: "zone", op: "remove", id: m[1].toUpperCase(), source: "text" }); return true; }
    if (CLEAR.test(t)) { UI.send({ type: "zone", op: "clear", source: "text" }); return true; }
    let obj = null, rest = "";
    for (const re of ADD) {
      m = t.match(re); if (!m || !(obj = objectOf(m[1]))) continue;
      rest = t.slice(0, m.index) + " " + m[1].slice(m[1].indexOf(obj) + obj.length); break;
    }
    if (!obj) { m = raw.match(KO_ADD); if (m) { obj = m[1].replace(/\s+/g, ""); rest = raw.replace(m[0], " "); } }
    if (!obj) return false;
    UI.send({ type: "zone", op: "add", near: obj, source: "voice" });
    // "put the mug on the left but stay away from the vase": the zone goes first, then the app runs the task as usual
    return !PLACE.test(rest);
  }
  window.addEventListener("pc-hear", (e) => { if (e.detail && hear(e.detail.text)) e.preventDefault(); });
  window.PCZones = { hear, get zones() { return zones; } }; // tests / other modules
})();
