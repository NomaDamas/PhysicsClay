// PhysicsClay UI core: the small registry UX 2.0 feature modules plug into, so a feature is one file in web/ui/
// and never edits index.html or app.js. Loads every module listed by /ui/modules.json after app.js is up.
//
//   PCUI.state                      latest server state (same object app.js renders)
//   PCUI.onState(fn)                fn(state) on every state frame; returns an unsubscribe function
//   PCUI.send(msg)                  WebSocket message to the server (false when offline)
//   PCUI.say(kind, text)            line in the conversation feed (kind: "sys" | "robot" | "you")
//   PCUI.toast(text)
//   PCUI.slot(name)                 container element the shell provides: "tools" | "plan" | "stage" | "timeline" | "side"
//                                   (a fallback container is created when the current shell has no such slot)
//   PCUI.tool({id, label, icon, title, onSelect, onDeselect, group})
//                                   input tool button in the dock; tools in the same group are exclusive (default "input")
//   PCUI.selectTool(id | null)      PCUI.activeTool
//   PCUI.with3D(fn)                 fn(PC3D) once the three.js stage is mounted (PC3D.layer, capture, tableHit, THREE…)
//   PCUI.icon(name)                 inline stroke SVG string from a tiny shared set
"use strict";
(function () {
  const listeners = new Set();
  const tools = new Map();
  const fallbacks = {};
  let active = null;

  function slot(name) {
    const el = document.querySelector(`[data-slot="${name}"]`);
    if (el) return el;
    if (!fallbacks[name]) {
      const f = document.createElement("div");
      f.className = `pcui-fallback pcui-fallback-${name}`;
      f.dataset.slotFallback = name;
      document.body.appendChild(f);
      fallbacks[name] = f;
    }
    return fallbacks[name];
  }

  const ICONS = {
    point: '<path d="M9 11V5.5a1.5 1.5 0 0 1 3 0V11m0-1.5a1.5 1.5 0 0 1 3 0V12m0-1a1.5 1.5 0 0 1 3 0v4a6 6 0 0 1-6 6h-1a6 6 0 0 1-5-2.7L4.5 15a1.5 1.5 0 0 1 2.4-1.8L9 15"/>',
    path: '<path d="M4 18c3 0 4-3 6-6s4-6 7-6"/><circle cx="4" cy="18" r="1.6"/><circle cx="18" cy="6" r="1.6"/>',
    zone: '<rect x="4" y="4" width="16" height="16" rx="3"/><path d="M8 16 16 8"/>',
    goal: '<rect x="3.5" y="5" width="17" height="14" rx="2.5"/><path d="m7 15 3.5-4 3 3 2-2 2.5 3"/><circle cx="9" cy="9" r="1.2"/>',
    show: '<path d="M4 8h3l2-2.5h6L17 8h3v11H4z"/><circle cx="12" cy="13.2" r="3.4"/>',
    sort: '<path d="M4 7h7M4 12h5M4 17h3"/><path d="m15 6 3-3 3 3M18 3v18"/>',
    hand: '<path d="M7 11V6a1.5 1.5 0 0 1 3 0v4m0-5a1.5 1.5 0 0 1 3 0v5m0-4a1.5 1.5 0 0 1 3 0v6m0-3a1.5 1.5 0 0 1 3 0v4a7 7 0 0 1-7 7h-1a7 7 0 0 1-6-3.5L5 14"/>',
    check: '<path d="m5 12 4.5 4.5L19 7"/>',
    x: '<path d="M6 6l12 12M18 6 6 18"/>',
    play: '<path d="M8 5v14l11-7z"/>',
  };
  function icon(name, size = 20) {
    const body = ICONS[name] || ICONS.point;
    return `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;
  }

  function renderTool(t) {
    const b = document.createElement("button");
    b.type = "button"; b.className = "pcui-tool"; b.dataset.tool = t.id;
    b.setAttribute("aria-pressed", "false");
    b.title = t.title || t.label;
    b.innerHTML = `${t.icon && t.icon.startsWith("<") ? t.icon : icon(t.icon || t.id)}<span>${t.label}</span>`;
    b.addEventListener("click", () => selectTool(active === t.id ? null : t.id));
    slot("tools").appendChild(b);
    t.button = b;
  }
  function tool(t) {
    if (!t || !t.id || tools.has(t.id)) return;
    t.group = t.group || "input";
    tools.set(t.id, t);
    renderTool(t);
  }
  function selectTool(id) {
    const prev = active ? tools.get(active) : null;
    const next = id ? tools.get(id) : null;
    if (prev && prev !== next) {
      prev.button && prev.button.setAttribute("aria-pressed", "false");
      try { prev.onDeselect && prev.onDeselect(); } catch (e) { console.error(e); }
    }
    active = next ? next.id : null;
    if (next) {
      next.button && next.button.setAttribute("aria-pressed", "true");
      try { next.onSelect && next.onSelect(); } catch (e) { console.error(e); }
    }
    window.dispatchEvent(new CustomEvent("pc-tool", { detail: active }));
  }

  function with3D(fn) {
    const go = () => { const v = window.PC3D; if (v && v.mounted) { fn(v); return true; } return false; };
    if (go()) return;
    const iv = setInterval(() => { if (go()) clearInterval(iv); }, 300);
  }

  window.addEventListener("pc-state", (e) => { for (const fn of listeners) { try { fn(e.detail); } catch (err) { console.error(err); } } });

  window.PCUI = {
    get state() { return (window.PC && window.PC.state) || {}; },
    onState(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    send(msg) { return !!(window.PC && window.PC.send && window.PC.send(msg)); },
    say(kind, text) { if (window.PC && window.PC.addLine) window.PC.addLine(kind, text); },
    toast(text) { if (window.PC && window.PC.toast) window.PC.toast(text); else console.log(text); },
    slot, tool, selectTool, with3D, icon,
    get activeTool() { return active; },
  };

  // Load feature modules (sorted, one file each). Classic scripts; each wraps itself in an IIFE.
  fetch("/ui/modules.json").then((r) => (r.ok ? r.json() : [])).then((list) => {
    for (const src of Array.isArray(list) ? list : []) {
      const s = document.createElement("script"); s.src = src; s.async = false; document.body.appendChild(s);
    }
  }).catch(() => { /* offline or old server: no feature modules */ });
})();
