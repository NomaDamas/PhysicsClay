// v1 P5 · Safety around people (pps/features/safety.py). Status only: the browser can't change safety.
// state.safety -> the person's safety capsules in the 3D stage (PC3D.layer("safety")): faint when clear, amber while
// the robot slows down, red when it has stopped; a calm line above the plan card and in the plan chip ("Slowing:
// you're 30 cm away", "Stopped: too close to your head"). The only message it sends is {type: "safety", op: "resume"},
// to confirm a latched stop (force over the limit) once you're clear.
"use strict";
(function () {
  const U = window.PCUI; if (!U) return;
  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

  const css = document.createElement("style");
  css.id = "pc-safety-css";
  css.textContent = `
.sf-note { display: flex; align-items: center; gap: 10px; padding: 10px 12px; border-radius: 14px; background: var(--raised, #fff);
  border: 1px solid var(--line, rgba(29,29,31,.1)); font: 600 var(--fs-sm, 13px) var(--sans, system-ui); color: var(--t1, #1d1d1f); }
.sf-note[hidden] { display: none; }
.sf-note i { width: 8px; height: 8px; border-radius: 50%; flex: none; background: var(--warn, #f59145); box-shadow: 0 0 0 3px rgba(245,145,69,.16); }
.sf-note[data-zone="stop"] i { background: var(--bad, #d6453a); box-shadow: 0 0 0 3px rgba(214,69,58,.16); }
.sf-note .sf-t { flex: 1 1 auto; min-width: 0; }
.sf-note small { display: block; font-weight: 500; color: var(--t2, rgba(29,29,31,.64)); margin-top: 2px; }
.sf-note button { flex: none; min-height: 32px; padding: 0 12px; border-radius: 999px; border: 1px solid var(--line2, rgba(29,29,31,.16));
  background: var(--bar, #fbfbfd); color: var(--t1, #1d1d1f); font: 600 12px var(--sans, system-ui); cursor: pointer; }
.sf-note button:focus-visible { outline: none; box-shadow: var(--ring, 0 0 0 3px rgba(92,119,223,.32)); }
`;
  document.head.appendChild(css);

  const note = document.createElement("div");
  note.className = "sf-note"; note.hidden = true; note.setAttribute("role", "status"); note.setAttribute("aria-live", "polite");
  note.innerHTML = `<i aria-hidden="true"></i><span class="sf-t"></span><button type="button" hidden>I'm clear</button>`;
  const host = U.slot("plan");
  host.insertBefore(note, host.firstChild);
  const txt = note.querySelector(".sf-t"), ok = note.querySelector("button");
  ok.addEventListener("click", () => { if (U.send({ type: "safety", op: "resume", id: `sf-${Date.now()}` })) ok.disabled = true; });

  // ------------------------------------------------------------------ plan chip (shell.js lets a module own it)
  const chip = document.getElementById("planChip");
  function ownChip(z) {
    if (!chip) return;
    if (!z) { if (chip.dataset.owner === "safety") delete chip.dataset.owner; return; }
    if (chip.dataset.owner && chip.dataset.owner !== "safety") return;
    chip.dataset.owner = "safety";
    const [tone, text] = z === "stop" ? ["bad", "Stopped for safety"] : ["wait", "Slowing near you"];
    if (chip.dataset.tone !== tone) chip.dataset.tone = tone;
    if (chip.textContent !== text) chip.textContent = text;
  }

  // ------------------------------------------------------------------ 3D capsules
  let V = null, G = null, mats = null, pool = [], lastKey = "";
  const COLORS = { clear: [0x8a93a6, 0.05], slow: [0xf59145, 0.11], stop: [0xd6453a, 0.16] };   // faint: pieces overlap
  function build(v) {
    V = v; const T = v.THREE; G = v.layer("safety");
    mats = {};
    for (const k in COLORS) mats[k] = new T.MeshBasicMaterial({ color: COLORS[k][0], transparent: true, opacity: COLORS[k][1], depthWrite: false });
    mats.link = new T.MeshBasicMaterial({ color: 0xd6453a, transparent: true, opacity: 0.8, depthWrite: false });
  }
  // one capsule = a cylinder + two sphere caps (shared unit geometries, scaled)
  let cyl = null, sph = null;
  function piece(i, kind) {
    const T = V.THREE;
    if (!cyl) { cyl = new T.CylinderGeometry(1, 1, 1, 14, 1, true); sph = new T.SphereGeometry(1, 14, 10); }
    while (pool.length <= i) {
      const g = { c: new T.Mesh(cyl, mats.clear), a: new T.Mesh(sph, mats.clear), b: new T.Mesh(sph, mats.clear) };
      for (const m of [g.c, g.a, g.b]) { m.userData.sharedGeo = true; m.renderOrder = 3; G.add(m); }
      pool.push(g);
    }
    const g = pool[i];
    for (const m of [g.c, g.a, g.b]) m.material = mats[kind];
    return g;
  }
  function draw(s) {
    if (!G) return;
    const caps = s && s.person ? s.capsules || [] : [];
    const zone = s && s.zone ? s.zone : "clear";
    const key = zone + "|" + caps.map((c) => c.slice(0, 7).map((x) => x.toFixed(2)).join(",")).join(";") +
      "|" + JSON.stringify((s && s.closest && zone !== "clear") ? [s.closest.p_robot, s.closest.p_human] : null);
    if (key === lastKey) return;
    lastKey = key;
    const T = V.THREE, UP = new T.Vector3(0, 1, 0);
    caps.forEach((c, i) => {
      const g = piece(i, zone === "stop" ? "stop" : zone === "slow" ? "slow" : "clear");
      const A = new T.Vector3(c[0], c[1], c[2]), B = new T.Vector3(c[3], c[4], c[5]), r = c[6];
      const d = new T.Vector3().subVectors(B, A), len = d.length();
      g.a.position.copy(A); g.a.scale.setScalar(r); g.b.position.copy(B); g.b.scale.setScalar(r);
      g.c.position.copy(A).addScaledVector(d, 0.5); g.c.scale.set(r, Math.max(len, 1e-4), r);
      if (len > 1e-6) g.c.quaternion.setFromUnitVectors(UP, d.normalize());
      g.a.visible = g.b.visible = true; g.c.visible = len > 1e-4;
    });
    for (let i = caps.length; i < pool.length; i++) pool[i].a.visible = pool[i].b.visible = pool[i].c.visible = false;
    // the closest pair, as a thin line, while slowing or stopped
    if (!G.userData.link) {
      const m = new T.Mesh(new T.CylinderGeometry(0.003, 0.003, 1, 8), mats.link); m.renderOrder = 4; G.add(m); G.userData.link = m;
    }
    const link = G.userData.link, cl = s && s.closest;
    link.visible = false;
    if (cl && cl.p_robot && cl.p_human && zone !== "clear" && caps.length) {
      const A = new T.Vector3(...cl.p_robot), B = new T.Vector3(...cl.p_human), d = new T.Vector3().subVectors(B, A), len = d.length();
      if (len > 1e-3) {
        link.position.copy(A).addScaledVector(d, 0.5); link.scale.set(1, len, 1); link.quaternion.setFromUnitVectors(UP, d.normalize());
        link.material.color.set(zone === "stop" ? 0xd6453a : 0xf59145); link.visible = true;
      }
    }
    V.redraw();
  }
  U.with3D(build);

  // ------------------------------------------------------------------ state
  let lastNote = "";
  U.onState((st) => {
    const s = st.safety;
    if (!s) return;
    const z = s.zone === "stop" || s.zone === "slow" ? s.zone : null;
    const sub = z === "stop" ? (s.latched ? "Step back, then confirm." : "I'll carry on when you step back.")
      : z === "slow" ? `Moving at ${Math.round((s.speed_scale || 0) * 100)}% speed.` : "";
    const key = [z, s.message, sub, s.latched].join("|");
    if (key !== lastNote) {
      lastNote = key;
      note.hidden = !z; note.dataset.zone = z || "";
      txt.innerHTML = z ? `${esc(s.message || (z === "stop" ? "Stopped: you're very close" : "Slowing: you're close"))}<small>${esc(sub)}</small>` : "";
      ok.hidden = !(z === "stop" && s.latched); ok.disabled = false;
    }
    ownChip(z);
    draw(s);
  });
})();
