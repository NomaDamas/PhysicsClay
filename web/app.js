// PhysicsClay HUD: receives state (JSON) + camera/sim frames (JPEG) over WebSocket; sends intent commands back.
const $ = (id) => document.getElementById(id);
const camImg = $("cam"), simImg = $("sim"), overlay = $("overlay"), camFrame = $("camFrame"), simFrame = $("simFrame");
const ctx = overlay.getContext("2d");
let ws, state = {}, localPath = null, drawing = null, dragIdx = -1, lastSend = 0;

const SKELETON = [[11,13],[13,15],[12,14],[14,16],[11,12],[11,23],[12,24],[23,24],[15,19],[15,21],[15,17],[16,20],[16,22],[16,18],[0,11],[0,12]];

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = "arraybuffer";
  ws.onmessage = (ev) => {
    if (typeof ev.data === "string") { state = JSON.parse(ev.data); render(); return; }
    const u8 = new Uint8Array(ev.data), img = u8[0] === 1 ? camImg : simImg;
    const url = URL.createObjectURL(new Blob([u8.subarray(1)], { type: "image/jpeg" }));
    const old = img.src; img.onload = () => { if (old.startsWith("blob:")) URL.revokeObjectURL(old); };
    img.src = url;
  };
  ws.onclose = () => { $("stats").textContent = "disconnected, retrying…"; setTimeout(connect, 1000); };
}
const send = (obj) => { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); };

// ---------------------------------------------------------------- state -> DOM
function render() {
  const s = state, mode = s.mode || "—";
  const modeEl = $("mode"); modeEl.textContent = mode; modeEl.className = "chip mode " + mode;
  const clutch = $("clutch"); clutch.textContent = s.engaged ? "Engaged" : "Engage"; clutch.classList.toggle("on", !!s.engaged);
  camFrame.classList.toggle("engaged", !!s.engaged);
  const feas = Math.round((s.feasibility || 0) * 100);
  const fill = $("feas"); fill.style.width = feas + "%"; fill.classList.toggle("low", feas < 80);
  $("feasTxt").textContent = `feasible ${feas}%`;
  $("speed").textContent = `speed x${s.speed_scale ?? 1}`;
  $("mirror").textContent = `mirror ${s.mirror === false ? "off" : "on"}`;
  $("stats").textContent = `${s.source || ""} · ${s.fps || 0} fps · IK ${Math.round(s.ik_hz || 0)} Hz`;
  $("source").textContent = s.source || "";
  $("noPose").classList.toggle("hidden", !!s.pose_ok || s.source === "synthetic");

  const says = $("says"); says.innerHTML = "";
  const add = (text, cls) => { const li = document.createElement("li"); li.textContent = text; li.className = cls; says.appendChild(li); };
  if (s.path_msg) add(s.path_msg, /all reachable/.test(s.path_msg) ? "ok" : "warn");
  const msgs = s.messages || [];
  if (!msgs.length && !s.path_msg) add(s.engaged ? "OK. Following your intent." : "Paused. Draw a path, or press space to follow your body.", s.engaged ? "ok" : "info");
  for (const m of msgs) add(m, /Too fast/.test(m) ? "warn" : "");
  if (mode === "REC") add(`Recording ${s.rec_duration ?? 0}s · ${s.rec_frames ?? 0} frames`, "info");
  if (mode === "PREVIEW") add(`Preview ${s.preview_t ?? 0}s / ${s.rec_duration ?? 0}s at x${s.speed_scale ?? 1}`, "info");

  const chips = $("context"); chips.innerHTML = "";
  const chip = (t, hot) => { const d = document.createElement("div"); d.className = "chip" + (hot ? " hot" : ""); d.textContent = t; chips.appendChild(d); };
  const g = s.grip || {}, h = s.holding || {};
  chip(`L hand: ${g.l ? "fist" : "open"}${h.l ? " on " + h.l : ""}`, !!h.l);
  chip(`R hand: ${g.r ? "fist" : "open"}${h.r ? " on " + h.r : ""}`, !!h.r);
  for (const o of (s.scene || [])) chip(o, true);
  if (s.path_n) chip(`path: ${s.path_n} waypoints · ${s.path_hand === "l" ? "left" : "right"} hand`);
  chip(`source: ${s.source || "?"}`);
  $("saved").textContent = s.saved ? `saved ${s.saved.split("/").slice(-2).join("/")}  (intent.json · trajectory.json · report.json)` : "";
  if (dragIdx < 0 && !drawing) localPath = (s.path_uv && s.path_uv.length) ? s.path_uv.map((p) => [p[0], p[1]]) : null;
  drawOverlay();
}

// ---------------------------------------------------------------- overlay (camera view)
function fit() {
  const r = camFrame.getBoundingClientRect();
  if (overlay.width !== Math.round(r.width) || overlay.height !== Math.round(r.height)) { overlay.width = Math.round(r.width); overlay.height = Math.round(r.height); }
}
// the camera frame is 16:9 drawn with object-fit: contain inside the (16:9) frame -> full box
const toPx = (u, v) => [u * overlay.width, v * overlay.height];
const toUV = (ev) => { const r = overlay.getBoundingClientRect(); return [(ev.clientX - r.left) / r.width, (ev.clientY - r.top) / r.height]; };

function drawOverlay() {
  fit();
  ctx.clearRect(0, 0, overlay.width, overlay.height);
  const s = state;
  if (s.pose) {
    ctx.lineWidth = 3; ctx.strokeStyle = s.engaged ? "rgba(95,208,138,.9)" : "rgba(200,200,200,.7)"; ctx.lineCap = "round";
    for (const [a, b] of SKELETON) { const p = toPx(...s.pose[a]), q = toPx(...s.pose[b]); ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); }
    ctx.fillStyle = "#fff";
    for (const i of [11, 12, 13, 14, 15, 16]) { const p = toPx(...s.pose[i]); ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, 7); ctx.fill(); }
  }
  for (const o of (s.objects || [])) {
    const [lab, sc, u1, v1, u2, v2] = o, a = toPx(u1, v1), b = toPx(u2, v2);
    ctx.strokeStyle = "#f2a33a"; ctx.lineWidth = 2; ctx.strokeRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
    ctx.fillStyle = "#f2a33a"; ctx.font = "13px JetBrains Mono, monospace"; ctx.fillText(`${lab} ${sc}`, a[0] + 4, a[1] + 15);
  }
  const path = (dragIdx >= 0 || drawing) ? (drawing || localPath) : localPath;
  if (path && path.length > 1) {
    ctx.lineWidth = 2.5; ctx.strokeStyle = "rgba(255,220,80,.95)"; ctx.beginPath();
    path.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); ctx.stroke();
    if (!drawing) {
      const ok = s.path_ok || [], cur = s.path_i ?? -1;
      path.forEach((p, i) => {
        const q = toPx(p[0], p[1]), good = i < ok.length ? ok[i] : true, isCur = i === cur && s.mode === "FOLLOW";
        ctx.fillStyle = isCur ? "#4fd1e8" : good ? "#5fd08a" : "#ff6b5b";
        ctx.beginPath(); ctx.arc(q[0], q[1], isCur ? 9 : (i === dragIdx ? 8 : 5.5), 0, 7); ctx.fill();
        if (i === dragIdx) { ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke(); }
      });
    }
  }
}

overlay.addEventListener("pointerdown", (ev) => {
  overlay.setPointerCapture(ev.pointerId);
  const uv = toUV(ev);
  if (localPath) {
    const p = toPx(...uv);
    let best = -1, bd = 14;
    localPath.forEach((w, i) => { const q = toPx(w[0], w[1]); const d = Math.hypot(q[0] - p[0], q[1] - p[1]); if (d < bd) { bd = d; best = i; } });
    if (best >= 0) { dragIdx = best; drawOverlay(); return; }
  }
  drawing = [uv];
});
overlay.addEventListener("pointermove", (ev) => {
  const uv = toUV(ev);
  if (dragIdx >= 0) { localPath[dragIdx] = uv; drawOverlay(); return; }
  if (drawing) { const last = drawing[drawing.length - 1], p = toPx(...last), q = toPx(...uv); if (Math.hypot(q[0] - p[0], q[1] - p[1]) > 4) { drawing.push(uv); drawOverlay(); } }
});
const depth = () => Number($("depth").value) / 100;
const finish = (ev) => {
  if (dragIdx >= 0) { send({ type: "path", points: localPath, resample: false, hand: state.path_hand || undefined, depth: depth() }); dragIdx = -1; return; }
  if (drawing) { if (drawing.length >= 2) { localPath = drawing; send({ type: "path", points: drawing, depth: depth() }); } drawing = null; drawOverlay(); }
};
$("depth").addEventListener("input", () => { $("depthTxt").textContent = `${$("depth").value} cm`; });
$("depth").addEventListener("change", () => { if (localPath && localPath.length > 1) send({ type: "path", points: localPath, resample: false, hand: state.path_hand || undefined, depth: depth() }); });
overlay.addEventListener("pointerup", finish); overlay.addEventListener("pointercancel", finish);

// ---------------------------------------------------------------- sim orbit
let orbit = null;
simFrame.addEventListener("pointerdown", (ev) => { orbit = [ev.clientX, ev.clientY]; simFrame.setPointerCapture(ev.pointerId); });
simFrame.addEventListener("pointermove", (ev) => { if (!orbit) return; const now = performance.now(); if (now - lastSend < 33) return; lastSend = now; send({ type: "camera", dazimuth: (ev.clientX - orbit[0]) * 0.4, delevation: -(ev.clientY - orbit[1]) * 0.4 }); orbit = [ev.clientX, ev.clientY]; });
const endOrbit = () => { orbit = null; };
simFrame.addEventListener("pointerup", endOrbit); simFrame.addEventListener("pointercancel", endOrbit);
simFrame.addEventListener("wheel", (ev) => { ev.preventDefault(); send({ type: "camera", zoom: Math.exp(ev.deltaY * 0.0015) }); }, { passive: false });

// ---------------------------------------------------------------- controls
$("clutch").onclick = () => send({ type: "clutch", engaged: !state.engaged });
$("mirror").onclick = () => send({ type: "key", key: "M" });
document.querySelectorAll("[data-key]").forEach((b) => b.onclick = () => { const k = b.dataset.key; if (k === "C") localPath = null; send({ type: "key", key: k }); });
$("cmdForm").onsubmit = (ev) => { ev.preventDefault(); const t = $("cmd").value.trim(); if (t) send({ type: "cmd", text: t }); $("cmd").value = ""; $("cmd").blur(); };
document.addEventListener("keydown", (ev) => {
  const typing = ev.target && /^(INPUT|TEXTAREA)$/.test(ev.target.tagName);
  if (typing) { if (ev.key === "Escape") ev.target.blur(); return; }
  if (ev.key === " ") { ev.preventDefault(); send({ type: "clutch", engaged: !state.engaged }); }
  else if (ev.key === "/") { ev.preventDefault(); $("cmd").focus(); }
  else if (/^[fcrpahm]$/i.test(ev.key)) { if (ev.key.toUpperCase() === "C") localPath = null; send({ type: "key", key: ev.key.toUpperCase() }); }
});
new ResizeObserver(drawOverlay).observe(camFrame);
connect();
