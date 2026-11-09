// PhysicsClay HUD v2 — talk to the robot, tap/draw spatial intent on its view, read the physics verdict.
// Protocol: docs/PROTOCOL.md. Gemini (optional): web/gemini.js -> window.PCGemini.
"use strict";
(function () {
  const $ = (id) => document.getElementById(id);
  const el = {
    mode: $("mode"), talk: $("talk"), talkBig: $("talkBig"), talkLabel: $("talkLabel"), talkSub: $("talkSub"), talkBigLabel: $("talkBigLabel"),
    feasFill: $("feasFill"), feasTxt: $("feasTxt"), speed: $("speed"), stats: $("stats"), statsTxt: $("statsTxt"),
    robotFrame: $("robotFrame"), robotImg: $("robotImg"), robotOv: $("robotOverlay"), robotEmpty: $("robotEmpty"),
    taskHud: $("taskHud"), taskHudTxt: $("taskHudTxt"), pathBar: $("pathBar"), pathInfo: $("pathInfo"), orbitHint: $("orbitHint"),
    holding: $("holding"), camFrame: $("camFrame"), camImg: $("camImg"), camOv: $("camOverlay"), camEmpty: $("camEmpty"),
    noPose: $("noPose"), followBadge: $("followBadge"), recBadge: $("recBadge"), source: $("source"),
    feed: $("feed"), feedPanel: $("feedPanel"), feedToggle: $("feedToggle"), feedCount: $("feedCount"),
    says: $("says"), saysFeas: $("saysFeas"), saved: $("saved"), skillSel: $("skillSel"), playBtn: $("playBtn"),
    videoBar: $("videoBar"), videoName: $("videoName"), videoFill: $("videoFill"), videoPct: $("videoPct"), videoBtn: $("videoBtn"), videoFile: $("videoFile"), dropHint: $("dropHint"),
    skillEdit: $("skillEdit"), seName: $("seName"), seRange: $("seRange"), sePart: $("sePart"), seTrack: $("seTrack"), seSel: $("seSel"), seTicks: $("seTicks"),
    cmdForm: $("cmdForm"), cmd: $("cmd"), toast: $("toast"), dlg: $("settings"),
  };
  const camPanel = el.camFrame.closest(".panel");
  let view3d = new URLSearchParams(location.search).get("view") === "3d";
  const P3 = () => window.PC3D;
  function mount3d() { const v = P3(); if (!v) return; v.mount($("view3d")); v.active(view3d); v.setState(state); }
  window.addEventListener("pc3d-ready", mount3d);
  const SKELETON = [[11, 13], [13, 15], [12, 14], [14, 16], [11, 12], [11, 23], [12, 24], [23, 24], [0, 11], [0, 12]];
  const REGIONS = ["left", "right", "center", "front", "back"];
  const C = { ok: "#1f8f55", bad: "#d6453a", cyan: "#1da8e5", accent: "#5c77df", hot: "#f59145" };
  const arr = (x) => (Array.isArray(x) ? x : []);
  const num = (x, d = 0) => (typeof x === "number" && isFinite(x) ? x : d);
  const isUV = (p) => Array.isArray(p) && p.length >= 2 && isFinite(p[0]) && isFinite(p[1]);

  // ------------------------------------------------------------------ settings
  const LS = {
    get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } },
  };
  const G = () => window.PCGemini;
  const settings = {
    get key() { return LS.get("pc.geminiKey", ""); },
    get liveModel() { return LS.get("pc.liveModel", "") || (G() && G().DEFAULT_LIVE_MODEL) || ""; },
    get erModel() { return LS.get("pc.erModel", "") || (G() && G().DEFAULT_ER_MODEL) || ""; },
    get sendCam() { return LS.get("pc.sendCam", "1") === "1"; },
    get sendRobot() { return LS.get("pc.sendRobot", "0") === "1"; },
    get lang() { return LS.get("pc.lang", "en-US"); },
    get mock() { return LS.get("pc.mock", "0") === "1"; },
  };
  const useProxy = () => !settings.key && !!(typeof state === "object" && state && state.server_gemini);
  const aiOpts = () => (settings.key ? { apiKey: settings.key } : { apiKey: "", proxy: true });
  const haveAI = () => !!(settings.key || useProxy());
  const geminiReady = () => !!(G() && typeof G().connectLive === "function" && haveAI());

  // ------------------------------------------------------------------ websocket
  let ws = null, connected = false, state = {}, gotState = false;
  const latest = { cam: null, robot: null }; // ArrayBuffer of the last frame (byte 0 = tag)
  const blobUrls = { cam: [], robot: [] , pip: [] };

  function connect() {
    const url = `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`;
    try { ws = new WebSocket(url); } catch (e) { setTimeout(connect, 1500); return; }
    ws.binaryType = "arraybuffer";
    ws.onopen = () => { connected = true; if (LS.get("pc.mock", null) !== null) send({ type: "mock", enabled: settings.mock }); schedule(); maybeAutoDemo(); };
    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") {
        let s; try { s = JSON.parse(ev.data); } catch (e) { return; }
        if (s && typeof s === "object" && !s.type) { state = s; gotState = true; onState(); schedule(); if (P3()) P3().setState(s); window.dispatchEvent(new CustomEvent("pc-state", { detail: s })); }
        return;
      }
      const buf = ev.data; if (!(buf instanceof ArrayBuffer) || buf.byteLength < 2) return;
      const tag = new Uint8Array(buf, 0, 1)[0];
      if (tag === 1) { latest.cam = buf; showFrame(el.camImg, "cam", buf); }
      else if (tag === 2) { latest.robot = buf; showFrame(el.robotImg, "robot", buf); }
      else if (tag === 3) { showFrame(document.getElementById("pipImg"), "pip", buf); }
    };
    ws.onclose = () => { connected = false; schedule(); setTimeout(connect, 1000); };
    ws.onerror = () => { try { ws.close(); } catch (e) { /* noop */ } };
  }
  const send = (obj) => { if (ws && ws.readyState === 1) { ws.send(JSON.stringify(obj)); return true; } return false; };

  function showFrame(img, key, buf) {
    const url = URL.createObjectURL(new Blob([new Uint8Array(buf, 1)], { type: "image/jpeg" }));
    const list = blobUrls[key]; list.push(url);
    img.onload = () => {
      while (list.length > 1 && list[0] !== img.src) URL.revokeObjectURL(list.shift());
      if (!img.classList.contains("live")) { img.classList.add("live"); schedule(); }
    };
    img.src = url;
    if (list.length > 6) URL.revokeObjectURL(list.shift());
  }
  function b64Of(buf) {
    if (!buf) return null;
    const u8 = new Uint8Array(buf, 1); let s = "";
    for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
    return btoa(s);
  }

  // ------------------------------------------------------------------ render (once per animation frame)
  let rafPending = false;
  function schedule() { if (!rafPending) { rafPending = true; requestAnimationFrame(render); } }
  const memo = {};
  const changed = (k, v) => { const j = typeof v === "string" ? v : JSON.stringify(v); if (memo[k] === j) return false; memo[k] = j; return true; };

  function render() {
    rafPending = false;
    const s = state, mode = String(s.mode || "IDLE");
    if (changed("mode", mode)) { el.mode.dataset.mode = mode; el.mode.querySelector("b").textContent = mode; }
    const feas = Math.round(Math.max(0, Math.min(1, num(s.feasibility, 0))) * 100);
    if (changed("feas", gotState ? feas : -1)) {
      el.feasFill.style.width = (gotState ? feas : 0) + "%";
      el.feasFill.className = "meter-fill" + (feas < 50 ? " low" : feas < 85 ? " mid" : "");
      el.feasTxt.textContent = gotState ? feas + "%" : "--";
      el.saysFeas.textContent = gotState ? `feasible ${feas}%` : "";
    }
    const sp = num(s.speed_scale, 1);
    if (changed("speed", sp)) el.speed.textContent = `×${sp.toFixed(sp < 1 && sp * 10 % 1 ? 2 : 1)}`;
    const stats = connected ? `${num(s.fps).toFixed(0)} fps · IK ${Math.round(num(s.ik_hz))} Hz` : "offline · reconnecting";
    if (changed("stats", stats + connected)) { el.statsTxt.textContent = stats; el.stats.dataset.conn = connected ? "on" : "off"; }
    if (changed("src", s.source || "")) el.source.textContent = s.source || "—";
    el.robotEmpty.hidden = el.robotImg.classList.contains("live");
    el.camEmpty.hidden = el.camImg.classList.contains("live");
    el.noPose.hidden = !gotState || !!s.pose_ok || s.source === "synthetic" || !el.camImg.classList.contains("live");

    const following = mode === "FOLLOW";
    el.followBadge.hidden = !following; camPanel.classList.toggle("following", following);
    const rec = s.recording && typeof s.recording === "object" ? s.recording : {};
    el.recBadge.hidden = !rec.active; if (rec.active) el.recBadge.textContent = `REC ${num(rec.seconds).toFixed(1)}s`;
    document.querySelectorAll('[data-act="follow"]').forEach((b) => b.setAttribute("aria-pressed", String(following)));

    const view = ["orbit", "stage"].includes(s.view) ? s.view : "robot";
    const pipLabel = document.getElementById("pipLabel"); if (pipLabel) pipLabel.textContent = view === "robot" ? "Full body" : "Robot eye";
    if (changed("view", view + view3d)) {
      const shown = view3d ? "3d" : "robot"; // only Robot eye | 3D buttons; server view may be stage/orbit
      document.querySelectorAll(".seg-btn[data-view]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.view === shown)));
      el.robotFrame.classList.toggle("orbit", view !== "robot" && !view3d); el.orbitHint.hidden = view === "robot" || view3d;
      el.robotFrame.classList.toggle("v3d", view3d); $("view3d").hidden = !view3d;
      if (P3()) P3().active(view3d);
    }
    const mockOn = typeof s.mock === "boolean" ? s.mock : settings.mock;
    if (changed("mock", mockOn)) $("mockPill").hidden = !mockOn;
    $("teachCancel").hidden = !gotState || !s.teach || !s.teach.phase || s.teach.phase === "idle";
    const gname = s.gesture && typeof s.gesture === "object" && s.gesture.name ? String(s.gesture.name) : "";
    if (changed("gesture", gname)) { const gc = $("gestureChip"); gc.hidden = !gname; gc.textContent = gname ? "gesture · " + gname : ""; }
    const h = s.holding && typeof s.holding === "object" ? s.holding : {};
    const hold = ["left", "right"].filter((k) => h[k]).map((k) => `${k[0].toUpperCase()} hand: ${h[k]}`).join(" · ");
    if (changed("hold", hold)) { el.holding.textContent = hold || "hands empty"; el.holding.classList.toggle("hot", !!hold); el.holding.classList.toggle("subtle", !hold); }

    // skills
    const skills = arr(s.skills).map(String);
    if (changed("skills", skills)) {
      const cur = el.skillSel.value; el.skillSel.innerHTML = "";
      if (!skills.length) el.skillSel.add(new Option("no skills yet", ""));
      for (const k of skills) el.skillSel.add(new Option(k, k));
      if (skills.includes(cur)) el.skillSel.value = cur;
      el.skillSel.disabled = el.playBtn.disabled = !skills.length;
    }

    // task HUD + path bar
    const t = s.task && typeof s.task === "object" ? s.task : null;
    const hudOn = !!(t && (t.status === "planning" || t.status === "running" || Date.now() - lastTaskChange < 4000));
    el.taskHud.hidden = !hudOn;
    if (hudOn && changed("hud", [t.status, t.action, t.message, t.step])) {
      el.taskHud.dataset.status = t.status || "";
      el.taskHudTxt.textContent = `${actionTitle(t.action)} · ${t.step || t.message || t.status || ""}`;
    }
    const p = s.path && typeof s.path === "object" ? s.path : null, pn = p ? arr(p.uv).length : 0;
    el.pathBar.hidden = !pn || stroke !== null || mode === "PATH" || view3d;
    if (pn) {
      const bad = arr(p.ok).filter((x) => x === false).length;
      el.pathInfo.textContent = `${pn} waypoints · ${bad ? bad + " unreachable" : "all reachable"} · ${p.hand || "right"} hand`;
      el.pathInfo.style.color = bad ? C.bad : C.ok;
    }

    // robot says
    const msgs = arr(s.messages).map(String);
    const sayKey = [msgs, t && t.message, t && t.status, gotState, connected];
    if (changed("says", sayKey)) {
      el.says.innerHTML = "";
      const add = (text, cls) => { const li = document.createElement("li"); li.textContent = text; li.className = cls; el.says.appendChild(li); };
      if (!connected) add("Not connected to the robot. Reconnecting…", "info");
      else if (!gotState) add("Waiting for the robot…", "info");
      else {
        if (t && t.message) add(t.message, t.status === "failed" ? "" : t.status === "done" ? "ok" : "warn");
        for (const m of msgs) add(m, /too fast|slow/i.test(m) ? "warn" : "");
        if (!msgs.length && !(t && t.message)) add(mode === "FOLLOW" ? "Following you. Every pose is checked against my joints." : "Ready. Tap the table, draw a path, or just tell me what you want.", "ok");
      }
    }
    if (changed("saved", s.saved || "")) { el.saved.hidden = !s.saved; el.saved.textContent = s.saved ? `Saved → ${s.saved} (intent.json · trajectory.json · report.json)` : ""; }

    // video imitation progress
    const vid = s.video && typeof s.video === "object" ? s.video : null;
    el.videoBar.hidden = !vid;
    if (vid) {
      const pr = Math.max(0, Math.min(1, num(vid.progress)));
      el.videoName.textContent = String(vid.name || "video"); el.videoFill.style.width = (pr * 100).toFixed(1) + "%"; el.videoPct.textContent = Math.round(pr * 100) + "%";
    }
    // v3: demo pill, phone camera chip, robot label, overlays text
    const dm = s.demo && typeof s.demo === "object" ? s.demo : null;
    const dtxt = dm ? `DEMO · ${num(dm.step)}/${num(dm.total)} · ${dm.title || dm.name || ""}` : "";
    if (changed("demo", dtxt)) { $("demoPill").hidden = !dm; $("demoPillTxt").textContent = dtxt; }
    const pcOn = !!phoneCam.stream;
    if (changed("pcam", [pcOn, s.source])) { $("phoneCamChip").hidden = !pcOn; $("phoneCamChip").classList.toggle("live", s.source === "phone"); }
    const rlab = (arr(s.robots).find((r) => r && r.id === s.robot) || {}).label || (s.robot === "ur5e" ? "UR5e arm" : s.robot ? "Unitree G1" : "");
    if (rlab && changed("rlab", rlab)) $("robotLabel").textContent = `${rlab} · MuJoCo`;
    if (el.dlg.open) renderSettingsLive();
    if (changed("sg", !!s.server_gemini) && (live.state === "off" || live.state === "error")) setTalk("off");
    renderSkillEdit();
    renderTeach();
    drawRobot(); drawCam();
    if (tapFx.length || pendingTap) schedule(); // keep animating ripples
  }

  // ------------------------------------------------------------------ overlays
  function fitCanvas(cv) {
    const r = cv.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    const ctx = cv.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, r.width, r.height);
    return { ctx, W: r.width, H: r.height };
  }
  // image content rect inside an object-fit:contain box
  function contentRect(img, W, H, dw, dh) {
    const iw = img.naturalWidth || dw, ih = img.naturalHeight || dh, sc = Math.min(W / iw, H / ih);
    const w = iw * sc, h = ih * sc; return { x: (W - w) / 2, y: (H - h) / 2, w, h };
  }
  const robotRect = () => { const r = el.robotOv.getBoundingClientRect(); return { box: r, c: contentRect(el.robotImg, r.width, r.height, 640, 480) }; };
  function evUV(ev) {
    const { box, c } = robotRect();
    return [(ev.clientX - box.left - c.x) / c.w, (ev.clientY - box.top - c.y) / c.h];
  }

  function pillLabel(ctx, x, y, text, fg, bg) { // drawn below the anchor so it never collides with marker badges above
    ctx.font = "500 11px 'JetBrains Mono', ui-monospace, monospace";
    const w = ctx.measureText(text).width + 14, h = 19, top = y + 12;
    ctx.fillStyle = bg; roundRect(ctx, x - w / 2, top, w, h, 9.5); ctx.fill();
    ctx.strokeStyle = "rgba(242,163,58,.45)"; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = fg; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(text, x, top + h / 2 + .5);
  }
  function roundRect(ctx, x, y, w, h, r) { ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r); ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath(); }

  function drawRobot() {
    const { ctx, W, H } = fitCanvas(el.robotOv), c = contentRect(el.robotImg, W, H, 640, 480), s = state;
    const P = (uv) => [c.x + uv[0] * c.w, c.y + uv[1] * c.h];
    ctx.lineCap = "round"; ctx.lineJoin = "round";
    // scene objects
    for (const o of arr(s.scene_objects)) {
      if (!o || !isUV(o.uv)) continue;
      const [x, y] = P(o.uv);
      ctx.fillStyle = "rgba(242,163,58,.95)"; ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill();
      ctx.strokeStyle = "rgba(242,163,58,.35)"; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(x, y, 9, 0, 7); ctx.stroke();
      pillLabel(ctx, x, y, String(o.name || "object"), "#f2a33a", "rgba(12,12,13,.82)");
    }
    // path
    const p = s.path && typeof s.path === "object" ? s.path : null, pts = p ? arr(p.uv) : [];
    if (pts.some(isUV) && !stroke) {
      const ok = arr(p.ok), cur = num(p.i, -1), running = s.mode === "PATH";
      for (let i = 1; i < pts.length; i++) { // null uv = behind the camera: break the line there
        if (!isUV(pts[i - 1]) || !isUV(pts[i])) continue;
        const a = P(pts[i - 1]), b = P(pts[i]);
        ctx.lineWidth = 5; ctx.strokeStyle = "rgba(0,0,0,.35)"; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        ctx.lineWidth = 2; ctx.strokeStyle = ok[i] === false || ok[i - 1] === false ? "rgba(255,107,91,.85)" : "rgba(95,208,138,.85)";
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      }
      pts.forEach((q, i) => {
        if (!isUV(q)) return;
        const [x, y] = P(q), isCur = running && i === cur;
        ctx.fillStyle = isCur ? C.cyan : ok[i] === false ? C.bad : C.ok;
        ctx.beginPath(); ctx.arc(x, y, isCur ? 8 : 4.5, 0, 7); ctx.fill();
        if (isCur) { ctx.strokeStyle = "rgba(79,209,232,.4)"; ctx.lineWidth = 6; ctx.beginPath(); ctx.arc(x, y, 13, 0, 7); ctx.stroke(); }
      });
    }
    // live stroke
    if (stroke && stroke.pts.length > 1) {
      ctx.strokeStyle = C.hot; ctx.lineWidth = 3; ctx.shadowColor = "rgba(255,90,31,.6)"; ctx.shadowBlur = 8; ctx.beginPath();
      stroke.pts.forEach((q, i) => { const [x, y] = P(q); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); ctx.shadowBlur = 0;
    }
    // markers
    for (const m of arr(s.markers)) {
      if (!m || !isUV(m.uv)) continue;
      const [x, y] = P(m.uv), id = String(m.id ?? "?");
      ctx.strokeStyle = "rgba(0,0,0,.5)"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y - 14); ctx.stroke();
      ctx.strokeStyle = C.accent; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y - 14); ctx.stroke();
      ctx.fillStyle = C.accent; ctx.beginPath(); ctx.ellipse(x, y, 5, 2.4, 0, 0, 7); ctx.fill();
      ctx.beginPath(); ctx.arc(x, y - 24, 11, 0, 7); ctx.fillStyle = C.accent; ctx.shadowColor = "rgba(0,0,0,.5)"; ctx.shadowBlur = 6; ctx.fill(); ctx.shadowBlur = 0;
      ctx.fillStyle = "#1a1204"; ctx.font = "600 12px 'IBM Plex Sans', system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(id, x, y - 23.5);
    }
    drawPins(ctx, P, "robot");
    // pending tap ghost + ripples
    const now = performance.now();
    if (pendingTap && now - pendingTap.t < 1500) {
      const [x, y] = P(pendingTap.uv); ctx.setLineDash([3, 3]); ctx.strokeStyle = C.accent; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y - 24, 11, 0, 7); ctx.stroke(); ctx.setLineDash([]);
    } else pendingTap = null;
    for (let i = tapFx.length - 1; i >= 0; i--) {
      const f = tapFx[i], k = (now - f.t) / 500; if (k >= 1) { tapFx.splice(i, 1); continue; }
      const [x, y] = P(f.uv); ctx.strokeStyle = `rgba(242,163,58,${1 - k})`; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 6 + k * 22, 0, 7); ctx.stroke();
    }
  }

  const SOFT = [205, 145, 28, 280, 350, 175, 50, 235];
  function drawPins(ctx, P, target) {
    const sp = state.spatial && typeof state.spatial === "object" ? state.spatial : null, er = sp && sp.er && typeof sp.er === "object" ? sp.er : null;
    if (!er || (er.target || "camera") !== target) return;
    arr(er.points).forEach((p, i) => {
      if (!p || !isFinite(p.u) || !isFinite(p.v)) return;
      const [x, y] = P([p.u, p.v]), n = String(i + 1);
      ctx.strokeStyle = "rgba(92,119,223,.9)"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y - 16); ctx.stroke();
      ctx.fillStyle = "#5c77df"; ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.fill();
      ctx.beginPath(); ctx.arc(x, y - 26, 11, 0, 7); ctx.fillStyle = "#fff"; ctx.shadowColor = "rgba(40,30,90,.3)"; ctx.shadowBlur = 8; ctx.fill(); ctx.shadowBlur = 0;
      ctx.strokeStyle = "#5c77df"; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = "#3a4fb0"; ctx.font = "700 12px -apple-system, system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(n, x, y - 25.5);
      if (p.label) {
        ctx.font = "600 11px -apple-system, system-ui, sans-serif"; const t = String(p.label), tw = ctx.measureText(t).width + 12;
        ctx.fillStyle = "rgba(255,255,255,.92)"; roundRect(ctx, x + 14, y - 35, tw, 18, 9); ctx.fill();
        ctx.fillStyle = "#1d1d1f"; ctx.textAlign = "left"; ctx.fillText(t, x + 20, y - 25.5);
      }
    });
  }
  function drawCam() {
    const { ctx, W, H } = fitCanvas(el.camOv), c = contentRect(el.camImg, W, H, 960, 540), s = state;
    if (!el.camImg.classList.contains("live")) return;
    const P = (uv) => [c.x + uv[0] * c.w, c.y + uv[1] * c.h];
    for (const o of arr(s.objects)) {
      if (!Array.isArray(o) || o.length < 6) continue;
      const [lab, sc, u1, v1, u2, v2] = o, a = P([u1, v1]), b = P([u2, v2]);
      ctx.strokeStyle = "rgba(242,163,58,.9)"; ctx.lineWidth = 1.5; roundRect(ctx, a[0], a[1], b[0] - a[0], b[1] - a[1], 4); ctx.stroke();
      ctx.font = "500 10px 'JetBrains Mono', monospace"; ctx.textAlign = "left"; ctx.textBaseline = "top";
      const t = `${lab} ${typeof sc === "number" ? sc.toFixed(2) : sc}`, tw = ctx.measureText(t).width + 8;
      ctx.fillStyle = "rgba(242,163,58,.92)"; ctx.fillRect(a[0], a[1] - 15, tw, 15); ctx.fillStyle = "#1a1204"; ctx.fillText(t, a[0] + 4, a[1] - 13);
    }
    const sp = s.spatial && typeof s.spatial === "object" ? s.spatial : null;
    arr(sp && sp.objects).forEach((o, i) => {
      if (!o || typeof o !== "object") return;
      const hue = SOFT[i % SOFT.length], poly = arr(o.poly).filter(isUV), box = arr(o.box);
      ctx.lineJoin = "round"; let ax, ay;
      if (poly.length >= 3) {
        ctx.beginPath(); poly.forEach((q, j) => { const [x, y] = P(q); j ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.closePath();
        ctx.fillStyle = `hsla(${hue}, 80%, 62%, .28)`; ctx.fill(); ctx.strokeStyle = `hsla(${hue}, 75%, 55%, .9)`; ctx.lineWidth = 1.5; ctx.stroke();
        let mi = 0; poly.forEach((q, j) => { if (q[1] < poly[mi][1]) mi = j; }); [ax, ay] = P(poly[mi]);
      } else if (box.length >= 4) {
        const a = P([box[0], box[1]]), b = P([box[2], box[3]]);
        roundRect(ctx, a[0], a[1], b[0] - a[0], b[1] - a[1], 6); ctx.fillStyle = `hsla(${hue}, 80%, 62%, .2)`; ctx.fill();
        ctx.strokeStyle = `hsla(${hue}, 75%, 55%, .9)`; ctx.lineWidth = 1.5; ctx.stroke(); [ax, ay] = a;
      } else return;
      ctx.font = "600 11px -apple-system, system-ui, sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "middle";
      const t = `${o.label || "object"}${typeof o.score === "number" ? " " + Math.round(o.score * 100) + "%" : ""}`, tw = ctx.measureText(t).width + 12;
      const ly = Math.max(10, ay - 12); ctx.fillStyle = `hsla(${hue}, 70%, 96%, .94)`; roundRect(ctx, ax, ly - 9, tw, 18, 9); ctx.fill();
      ctx.strokeStyle = `hsla(${hue}, 70%, 50%, .6)`; ctx.lineWidth = 1; ctx.stroke(); ctx.fillStyle = `hsl(${hue}, 55%, 28%)`; ctx.fillText(t, ax + 6, ly + .5);
    });
    drawPins(ctx, P, "camera");
    const tch = s.teach && typeof s.teach === "object" ? s.teach : null, ev = tch && tch.evidence && typeof tch.evidence === "object" ? tch.evidence : null;
    const hp = ev && (tch.phase === "recording" || tch.phase === "confirm") ? arr(ev.hand_uv).filter(isUV) : [];
    if (hp.length > 1) { // demonstrated hand path (teach evidence)
      ctx.lineCap = "round"; ctx.lineJoin = "round";
      ctx.strokeStyle = "rgba(0,0,0,.35)"; ctx.lineWidth = 6; ctx.beginPath(); hp.forEach((q, i) => { const [x, y] = P(q); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
      for (let i = 1; i < hp.length; i++) {
        const a = P(hp[i - 1]), b = P(hp[i]), k = i / (hp.length - 1);
        ctx.strokeStyle = `hsla(${25 + k * 200}, 85%, 60%, ${0.45 + 0.55 * k})`; ctx.lineWidth = 3.5;
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      }
      const [ex, ey] = P(hp[hp.length - 1]); ctx.fillStyle = "#f59145"; ctx.beginPath(); ctx.arc(ex, ey, 6, 0, 7); ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
    }
    const pose = arr(s.pose);
    if (pose.length >= 25) {
      const follow = s.mode === "FOLLOW", col = follow ? "rgba(79,209,232,.95)" : "rgba(228,228,231,.7)";
      ctx.lineCap = "round"; ctx.lineWidth = follow ? 3 : 2.2; ctx.strokeStyle = col;
      for (const [a, b] of SKELETON) {
        if (!isUV(pose[a]) || !isUV(pose[b])) continue;
        const p = P(pose[a]), q = P(pose[b]); ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
      }
      for (const i of [0, 11, 12, 13, 14, 15, 16, 23, 24]) {
        if (!isUV(pose[i])) continue; const p = P(pose[i]);
        ctx.fillStyle = i === 15 || i === 16 ? C.accent : "#fff"; ctx.beginPath(); ctx.arc(p[0], p[1], i === 15 || i === 16 ? 4.5 : 3, 0, 7); ctx.fill();
      }
    }
  }

  // ------------------------------------------------------------------ robot-view interaction
  let stroke = null, orbitDrag = null, pendingTap = null, lastCam = 0;
  const tapFx = [], pointers = new Map();
  const ov = el.robotOv;
  const inImage = (uv) => uv[0] >= 0 && uv[0] <= 1 && uv[1] >= 0 && uv[1] <= 1;
  const clampUV = (uv) => [Math.min(1, Math.max(0, uv[0])), Math.min(1, Math.max(0, uv[1]))];

  ov.addEventListener("contextmenu", (e) => e.preventDefault());
  ov.addEventListener("pointerdown", (ev) => {
    try { ov.setPointerCapture(ev.pointerId); } catch (e) { /* noop */ }
    pointers.set(ev.pointerId, [ev.clientX, ev.clientY]);
    const orbit = state.view === "orbit" || state.view === "stage";
    if (orbit && (ev.button === 2 || pointers.size >= 2)) {
      stroke = null; orbitDrag = centroid(); el.robotFrame.classList.add("orbiting"); schedule(); return;
    }
    if (ev.button !== 0 || pointers.size > 1) { stroke = null; return; }
    const uv = evUV(ev); if (!inImage(uv)) return;
    stroke = { pts: [uv], t: performance.now(), x: ev.clientX, y: ev.clientY, travel: 0 };
  });
  ov.addEventListener("pointermove", (ev) => {
    if (!pointers.has(ev.pointerId)) return;
    const prev = pointers.get(ev.pointerId); pointers.set(ev.pointerId, [ev.clientX, ev.clientY]);
    if (orbitDrag) {
      const c = centroid(), now = performance.now();
      if (now - lastCam < 33) return;
      const dx = c[0] - orbitDrag[0], dy = c[1] - orbitDrag[1]; if (!dx && !dy) return;
      lastCam = now; send({ type: "camera", dazimuth: dx * 0.4, delevation: -dy * 0.4 }); orbitDrag = c; return;
    }
    if (!stroke) return;
    stroke.travel += Math.hypot(ev.clientX - prev[0], ev.clientY - prev[1]);
    const last = stroke.pts[stroke.pts.length - 1], uv = clampUV(evUV(ev)), { c } = robotRect();
    if (Math.hypot((uv[0] - last[0]) * c.w, (uv[1] - last[1]) * c.h) > 5) { stroke.pts.push(uv); schedule(); }
  });
  function endPointer(ev) {
    pointers.delete(ev.pointerId);
    if (orbitDrag) { if (!pointers.size) { orbitDrag = null; el.robotFrame.classList.remove("orbiting"); } else orbitDrag = centroid(); return; }
    const s = stroke; stroke = null; if (!s || ev.type === "pointercancel") { schedule(); return; }
    if (s.travel < 8 && performance.now() - s.t < 600) {
      const uv = s.pts[0]; send({ type: "tap", u: uv[0], v: uv[1] });
      tapFx.push({ uv, t: performance.now() }); pendingTap = { uv, t: performance.now(), n: arr(state.markers).length };
      contextToModel(`The user tapped the robot view at (u=${uv[0].toFixed(2)}, v=${uv[1].toFixed(2)}); a new marker letter is being created there.`);
    } else if (s.pts.length >= 2) {
      const pts = decimate(s.pts, 64);
      send({ type: "draw", points: pts.map((p) => [round3(p[0]), round3(p[1])]) });
      addLine("sys", `Drew a path with ${pts.length} points — checking reachability…`);
      contextToModel("The user drew a path on the robot view. It is pre-checked by physics; call follow_path() if they ask to run it.");
    }
    schedule();
  }
  ov.addEventListener("pointerup", endPointer); ov.addEventListener("pointercancel", endPointer);
  ov.addEventListener("wheel", (ev) => {
    if (state.view !== "orbit" && state.view !== "stage") return; ev.preventDefault();
    const now = performance.now(); if (now - lastCam < 30) return; lastCam = now;
    send({ type: "camera", zoom: Math.exp(Math.max(-200, Math.min(200, ev.deltaY)) * 0.0015) });
  }, { passive: false });
  function centroid() { let x = 0, y = 0; for (const p of pointers.values()) { x += p[0]; y += p[1]; } const n = pointers.size || 1; return [x / n, y / n]; }
  const round3 = (x) => Math.round(x * 1000) / 1000;
  function decimate(pts, max) { if (pts.length <= max) return pts; const out = []; for (let i = 0; i < max; i++) out.push(pts[Math.round(i * (pts.length - 1) / (max - 1))]); return out; }
  new ResizeObserver(schedule).observe(el.robotFrame); new ResizeObserver(schedule).observe(el.camFrame);

  document.querySelectorAll(".seg-btn[data-view]").forEach((b) => b.addEventListener("click", () => {
    if (b.dataset.view === "3d") { view3d = true; schedule(); return; } // client-side view: the server keeps rendering as is
    view3d = false;
    send({ type: "view", name: b.dataset.view }); state = Object.assign({}, state, { view: b.dataset.view }); schedule();
  }));
  $("runPath").addEventListener("click", () => runTask("follow_path", {}, "button"));
  $("clearPath").addEventListener("click", clearAll);
  function clearAll() { send({ type: "clear" }); addLine("sys", "Cleared markers and path."); }

  // ------------------------------------------------------------------ tasks + intent feed
  const TAG = Math.random().toString(36).slice(2, 5);
  let seq = 0, lastTaskKey = "", lastTaskChange = 0, intentCount = 0;
  const tasks = new Map();
  const TITLES = { pick_place: "Pick & place", reach: "Reach", follow_path: "Follow path", follow_me: "Follow me", learn_skill: "Learn skill",
    play_skill: "Play skill", propose_task: "Task understood", set_speed: "Set speed", stop: "Stop", home: "Home", reset_scene: "Reset scene", export: "Export", point: "Point on robot view",
    edit_skill: "Edit skill", compile_for: "Compile for robot", show_video: "Imitate a video" };
  function actionTitle(a) { return TITLES[a] || String(a || "Task").replace(/_/g, " "); }
  function argsSummary(action, a) {
    a = a || {};
    switch (action) {
      case "pick_place": return `${a.object || "mug"} → ${a.target_uv ? `(${a.target_uv.map((x) => x.toFixed(2)).join(", ")})${a.target ? " “" + a.target + "”" : ""}` : a.target || "auto"}`;
      case "reach": return a.target_uv ? `→ (${a.target_uv.map((x) => x.toFixed(2)).join(", ")})${a.target ? " “" + a.target + "”" : ""}` : `→ ${a.target || "?"}`;
      case "follow_me": return a.enabled === false ? "off" : "on";
      case "learn_skill": case "play_skill": return `“${a.name || "?"}”`;
      case "set_speed": return String(a.value ?? "");
      case "point": return `“${a.query || ""}”`;
      case "show_video": return String(a.name || "");
      case "compile_for": return `→ ${a.robot || "?"}`;
      case "edit_skill": {
        const r = a.t0 != null && a.t1 != null ? ` · ${num(+a.t0).toFixed(1)}–${num(+a.t1).toFixed(1)} s` : "";
        const m = a.speed != null ? ` · speed ×${a.speed}` : a.amplitude != null ? ` · size ×${a.amplitude}` : "";
        return `“${a.name || "?"}” · ${String(a.part || "body").replace("_", " ")}${r}${m}`;
      }
      default: { const k = Object.keys(a); return k.length ? k.map((x) => `${x}=${JSON.stringify(a[x])}`).join(" ") : ""; }
    }
  }
  const clock = () => new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false });

  function feedAppend(node) {
    const f = el.feed, atBottom = f.scrollHeight - f.scrollTop - f.clientHeight < 60;
    const empty = f.querySelector(".feed-empty"); if (empty) empty.remove();
    f.appendChild(node);
    while (f.children.length > 120) f.firstElementChild.remove();
    if (atBottom) f.scrollTop = f.scrollHeight;
  }
  function addLine(kind, text) {
    const d = document.createElement("div"); d.className = "line " + kind;
    const who = document.createElement("span"); who.className = "who"; who.textContent = kind === "user" ? "YOU" : kind === "robot" ? "ROBOT" : "·";
    const t = document.createElement("span"); t.className = "txt"; t.textContent = text;
    d.append(who, t); feedAppend(d); return d;
  }
  function makeCard(id, action, args, source) {
    const card = document.createElement("article"); card.className = "card"; card.dataset.status = "planning";
    card.innerHTML = '<div class="card-top"><span class="card-title"></span><span class="card-act"></span><span class="pill">planning</span></div><div class="card-args"></div><div class="card-msg"></div><div class="card-meta"></div>';
    card.querySelector(".card-title").textContent = actionTitle(action);
    card.querySelector(".card-act").textContent = action;
    card.querySelector(".card-args").textContent = argsSummary(action, args);
    card.dataset.meta = `${source} · ${clock()}`; card.querySelector(".card-meta").textContent = `${card.dataset.meta} · ${id}`;
    feedAppend(card); intentCount++; el.feedCount.textContent = `${intentCount} intent${intentCount === 1 ? "" : "s"}`;
    return card;
  }
  function updateCard(rec) {
    const c = rec.card; if (!c) return;
    c.dataset.status = rec.status; c.querySelector(".pill").textContent = rec.status;
    c.querySelector(".card-msg").textContent = rec.message || "";
    if (rec.step) c.querySelector(".card-meta").textContent = `${c.dataset.meta || rec.source} · ${rec.step}`;
  }

  function runTask(action, args = {}, source = "you") {
    const id = `t${++seq}${TAG}`;
    const rec = { id, action, args, source, status: "planning", message: "", step: "", settled: false, resolve: null, timer: 0 };
    rec.card = makeCard(id, action, args, source); tasks.set(id, rec);
    const p = new Promise((res) => { rec.resolve = res; });
    const finish = (r) => { if (rec.settled) return; rec.settled = true; clearTimeout(rec.timer); rec.resolve(r); };
    rec.finish = finish;
    if (!send(Object.assign({ type: "task", id, action }, args))) {
      rec.status = "failed"; rec.message = "Not connected to the robot."; updateCard(rec);
      finish({ ok: false, status: "failed", message: rec.message });
      return p;
    }
    rec.timer = setTimeout(() => {
      if (rec.settled) return;
      if (rec.status === "planning") { rec.status = "failed"; rec.message = "No response from the robot."; updateCard(rec); }
      finish({ ok: false, status: "timeout", message: "no response" });
    }, 8000);
    if (tasks.size > 60) { const k = tasks.keys().next().value; tasks.delete(k); }
    return p;
  }
  // local-only card (e.g. Gemini ER pointing)
  function localCard(action, args, source) {
    const id = `l${++seq}${TAG}`, rec = { id, action, args, source, status: "running", message: "", card: null };
    rec.card = makeCard(id, action, args, source); updateCard(rec);
    return (status, message) => { rec.status = status; rec.message = message || ""; updateCard(rec); };
  }

  let lastRobot = null;
  function onState() {
    const t = state.task && typeof state.task === "object" ? state.task : null;
    if (t && t.id != null) {
      const key = [t.id, t.status, t.message, t.step].join("|");
      if (key !== lastTaskKey) {
        lastTaskKey = key;
        let rec = tasks.get(String(t.id));
        const stale = !lastTaskChange && (t.status === "done" || t.status === "failed"); // finished before this page loaded
        lastTaskChange = stale ? 1 : Date.now();
        if (!rec && t.action && !stale) { // task started elsewhere (another client / server)
          rec = { id: String(t.id), action: t.action, args: {}, source: "server", status: "planning", settled: true };
          rec.card = makeCard(rec.id, t.action, {}, "server"); tasks.set(rec.id, rec);
        }
        if (rec) {
          const prevStatus = rec.status;
          rec.status = ["planning", "running", "done", "failed"].includes(t.status) ? t.status : rec.status;
          rec.message = t.message || rec.message || ""; rec.step = t.step || "";
          updateCard(rec);
          if (!rec.settled && ["running", "done", "failed"].includes(rec.status)) rec.finish({ ok: rec.status !== "failed", status: rec.status, message: rec.message });
          else if (rec.viaModel && prevStatus !== rec.status && (rec.status === "done" || rec.status === "failed")) {
            contextToModel(`[robot update] ${rec.action} ${rec.status}: ${rec.message || ""}`);
          }
        }
      }
    }
    trackVideo();
    onSay();
    if (state.robot) {
      if (lastRobot && state.robot !== lastRobot) {
        const v = P3();
        if (v && typeof v.reload === "function") { try { v.reload(); } catch (e) { /* noop */ } }
        else if (view3d) { location.href = location.pathname + "?view=3d"; }
      }
      lastRobot = state.robot;
    }
    // marker created after our tap
    if (pendingTap && arr(state.markers).length !== pendingTap.n) pendingTap = null;
  }

  // ------------------------------------------------------------------ teach by showing (Show & Tell)
  const TEACH_START = /\b(start teaching|let me show you|i'?ll show you|watch me|let me teach you|i'?ll teach you|teach (you|it) (something|a task))\b|보여\s?줄게|가르쳐\s?줄게/;
  const TEACH_STOP = /^(ok(ay)?[, ]+)?(done|that'?s it|that is it|finished|i'?m done|all done|done showing)\b|\bthat'?s all\b|^끝|끝났어|다\s?했어/;
  const TEACH_YES = /^(yes|yeah|yep|yup|correct|right|exactly|sure|ok(ay)?|go ahead|do it|try it|practice|sounds good|perfect)\b|^(맞아|응|네|예|그래|좋아|해봐|연습해)/;
  const TEACH_SAVE = /\b(?:save|remember|call|name)\s+(?:it|this|that)\s+(?:as\s+)?["“']?(.+?)["”']?$|["“']?(.+?)["”']?\s*(?:으로|로|라고)\s*저장/;
  const teachPhase = () => (state.teach && typeof state.teach === "object" && state.teach.phase) || "idle";
  const teachCmd = (cmd, extra) => send(Object.assign({ type: "teach", cmd }, extra || {}));
  function teachStart() { if (teachCmd("start")) addLine("sys", "Teaching started — show the task on your camera and explain it out loud. Say “done” when finished."); else toast("Not connected to the robot."); }
  function teachStop() { teachCmd("stop"); addLine("sys", "Got it — working out what you showed me…"); }
  function teachPractice() { teachCmd("practice"); addLine("sys", "Practicing in simulation with my own body…"); }
  function teachCorrect(text) { teachCmd("correct", { text }); }
  function teachSave(name) { name = String(name || "").trim(); if (!name) return; teachCmd("save", { name }); addLine("sys", `Saving skill “${name}”…`); }
  // returns true when the utterance was consumed by the teaching flow
  function teachHear(text) {
    const t = String(text).trim(), low = t.toLowerCase().replace(/[.!?。]+$/g, "").trim(), ph = teachPhase();
    if (!low) return false;
    if (!window.dispatchEvent(new CustomEvent("pc-text", { detail: { text: t }, cancelable: true }))) return true; // web/ui modules hear every final utterance first
    if (ph === "recording") {
      if (TEACH_STOP.test(low)) { teachStop(); return true; }
      teachCmd("utterance", { text: t }); return true;
    }
    if (TEACH_START.test(low)) { teachStart(); return true; }
    if (ph === "confirm" || ph === "done") {
      const sv = t.match(TEACH_SAVE); if (sv && (sv[1] || sv[2])) { teachSave(sv[1] || sv[2]); return true; }
      if (TEACH_YES.test(low)) { teachPractice(); return true; }
      if (/^(stop|halt|멈춰|정지)\b/.test(low)) return false;
      teachCorrect(t); return true;
    }
    if (ph === "practice") {
      if (/\b(stop|halt|wait)\b|멈춰|정지|그만/.test(low)) return false;
      teachCorrect(t); return true;
    }
    return false;
  }
  const PHASE_LABEL = { idle: "ready", recording: "watching you", confirm: "please confirm", practice: "practicing in sim", done: "done" };
  const STRIP_ON = { idle: "show", recording: "show", confirm: "understand", practice: "practice", done: "correct" };
  let lastQ = "";
  function chip(text, cls) { const c = document.createElement("span"); c.className = "t-chip " + (cls || ""); c.textContent = text; return c; }
  function renderTeach() {
    const T = state.teach && typeof state.teach === "object" ? state.teach : {}, ph = ["idle", "recording", "confirm", "practice", "done"].includes(T.phase) ? T.phase : "idle";
    const panel = $("teachPanel");
    if (changed("tph", ph)) {
      panel.dataset.phase = ph; $("teachPhase").textContent = PHASE_LABEL[ph];
      panel.querySelectorAll(".teach-strip li").forEach((li) => li.classList.toggle("on", li.dataset.k === STRIP_ON[ph]));
      $("teachCorrectForm").hidden = true;
    }
    $("teachRec").hidden = ph !== "recording";
    if (ph === "recording") $("teachRec").textContent = `REC ${num(T.seconds).toFixed(1)}s`;
    const evd = T.evidence && typeof T.evidence === "object" ? T.evidence : {};
    const evTxt = [num(evd.grasps) ? `${num(evd.grasps)} grasp${num(evd.grasps) === 1 ? "" : "s"}` : "", evd.moved ? `moved ${evd.moved}` : "",
      arr(evd.objects_seen).length ? "sees " + arr(evd.objects_seen).join(", ") : "", arr(evd.hand_uv).length ? `hand path ${arr(evd.hand_uv).length} pts` : ""].filter(Boolean);
    if (changed("tev", evTxt)) { const e = $("teachEvidence"); e.innerHTML = ""; (evTxt.length ? evTxt : ["watching your hands…"]).forEach((x) => e.appendChild(chip(x, "ev"))); }
    const utt = arr(T.utterances).map(String);
    if (changed("tutt", utt)) {
      const ul = $("teachUtt"); ul.innerHTML = "";
      if (!utt.length) ul.innerHTML = '<li class="muted">Talk while you show: “pick up the mug, carry it over, don\'t touch the vase”.</li>';
      for (const u of utt.slice(-6)) { const li = document.createElement("li"); li.textContent = u; ul.appendChild(li); }
    }
    const spec = T.spec && typeof T.spec === "object" ? T.spec : {};
    if (changed("tspec", spec)) {
      $("teachGoal").textContent = spec.goal ? String(spec.goal) : "";
      const ol = $("teachSteps"); ol.innerHTML = "";
      const steps = arr(spec.steps);
      steps.forEach((st, i) => {
        if (!st || typeof st !== "object") return;
        const li = document.createElement("li"); li.dataset.status = st.status || "pending";
        const n = document.createElement("b"); n.textContent = String(i + 1);
        const body = document.createElement("div"); body.className = "t-step-body";
        const row = document.createElement("div"); row.className = "t-step-row";
        row.append(chip(String(st.action || "step").replace(/_/g, " "), "act"));
        if (st.object) row.append(chip(String(st.object), "obj"));
        if (st.target) { row.append(Object.assign(document.createElement("span"), { className: "t-arrow", textContent: "→" }), chip(String(st.target), "tgt")); }
        const stx = document.createElement("span"); stx.className = "t-status mono"; stx.textContent = st.status || "pending"; row.append(stx);
        body.append(row);
        if (st.message) { const m = document.createElement("div"); m.className = "t-msg"; m.textContent = String(st.message); body.append(m); }
        li.append(n, body); ol.appendChild(li);
      });
      const cons = $("teachCons"); cons.innerHTML = "";
      for (const c of arr(spec.constraints)) {
        if (!c) continue;
        const txt = c.text || [String(c.type || "").replace(/_/g, " "), c.object].filter(Boolean).join(" ");
        cons.appendChild(chip(txt, "con " + String(c.type || "")));
      }
      const done = steps.filter((x) => x && x.status === "done").length, bad = steps.filter((x) => x && x.status === "failed").length;
      $("teachSummary").textContent = steps.length ? `${steps.length} step${steps.length === 1 ? "" : "s"}${done ? ` · ${done} done` : ""}${bad ? ` · ${bad} failed` : ""}` : "";
    }
    const q = T.question ? String(T.question) : "";
    if (changed("tq", q)) {
      $("teachQ").hidden = !q; $("teachQ").textContent = q;
      if (q && q !== lastQ && ph !== "idle") { addLine("robot", q); if (!live.session) speak(q, "en-US"); }
      lastQ = q;
    }
    $("teachConfirmBtns").hidden = !(ph === "confirm" || ph === "done");
    $("teachYes").textContent = ph === "done" ? "Practice again" : "Yes, practice";
    $("teachSaveForm").hidden = ph !== "done";
  }
  $("teachStart").addEventListener("click", teachStart);
  $("teachCancel").addEventListener("click", () => { teachCmd("cancel"); addLine("sys", "Teaching cancelled."); });
  const more = $("moreMenu");
  more.addEventListener("click", (ev) => { if (ev.target.closest(".more-menu .btn")) more.open = false; });
  document.addEventListener("pointerdown", (ev) => { if (more.open && !more.contains(ev.target)) more.open = false; });
  $("teachStop").addEventListener("click", teachStop);
  $("teachYes").addEventListener("click", teachPractice);
  $("teachCorrectBtn").addEventListener("click", () => { const f = $("teachCorrectForm"); f.hidden = !f.hidden; if (!f.hidden) $("teachCorrectIn").focus(); });
  $("teachCorrectForm").addEventListener("submit", (ev) => {
    ev.preventDefault(); const v = $("teachCorrectIn").value.trim(); if (!v) return; $("teachCorrectIn").value = "";
    addLine("user", v); teachCorrect(v); if (live.session) contextToModel(`The user corrected the taught task: "${v}"`);
  });
  $("teachSaveForm").addEventListener("submit", (ev) => { ev.preventDefault(); teachSave($("teachName").value); $("teachName").value = ""; });

  // mock / scripted brain utterances: state.say = {id, text, kind: "robot"|"user"|"vision"}
  let lastSayId = null;
  function onSay() {
    const sy = state.say; if (!sy || typeof sy !== "object" || sy.id == null || !sy.text) { if (gotState && lastSayId === null) lastSayId = ""; return; } // nothing stale: the next line is new
    const id = String(sy.id); if (id === lastSayId) return;
    const first = lastSayId === null; lastSayId = id;
    if (first && !state.mock && !settings.mock && !state.demo) return; // stale line from before this page loaded
    if (first) return; // never replay an old caption / voice line on load
    const text = String(sy.text), k = String(sy.kind || "robot");
    if (k === "caption") { showOverlay("caption", text, 4000); addLine("sys", text); return; }
    if (k === "narrator") { showOverlay("caption", text, 5000); addLine("sys", text); speak(text, "en-US", "alloy"); return; }
    if (k === "user") { addLine("user", text); showOverlay("subtitle", text, 3000); return; }
    const d = addLine("robot", text);
    if (k === "vision") {
      d.classList.add("vision"); const tag = document.createElement("span"); tag.className = "vtag"; tag.textContent = "VISION";
      d.querySelector(".txt").prepend(tag);
    }
    speak(text, "en-US");
  }
  const ovTimers = {};
  function showOverlay(id, text, ms) {
    const n = $(id); if (!n) return;
    n.textContent = text; n.hidden = false; n.classList.remove("out"); void n.offsetWidth; n.classList.add("in");
    clearTimeout(ovTimers[id]); clearTimeout(ovTimers[id + "h"]);
    ovTimers[id] = setTimeout(() => { n.classList.remove("in"); n.classList.add("out"); ovTimers[id + "h"] = setTimeout(() => { n.hidden = true; }, 450); }, ms);
  }

  // ------------------------------------------------------------------ text intents (rule parser, no key needed)
  const EDIT_KIND = { faster: { speed: 1.5 }, slower: { speed: 0.7 }, bigger: { amplitude: 1.3 }, smaller: { amplitude: 0.75 } };
  const KO_REGION = [["오른쪽", "right"], ["우측", "right"], ["왼쪽", "left"], ["좌측", "left"], ["가운데", "center"], ["중앙", "center"], ["앞", "front"], ["뒤", "back"]];
  function findTarget(t, raw) {
    let m = t.match(/\b(?:on|to|onto|at|toward|towards|into|over)\s+(?:the\s+)?(?:marker\s+|point\s+|spot\s+)?(left|right|center|centre|middle|front|back|[a-f])\b(?!')/);
    if (m) return normRegion(m[1]);
    m = t.match(/\b(?:marker|point|spot)\s+([a-f])\b/); if (m) return m[1].toUpperCase();
    m = raw.match(/(?:^|[^A-Za-z])([A-F])(?:[^A-Za-z]|$)/); if (m && /[가-힣]/.test(raw)) return m[1];
    if (ALONG_RE.test(t)) return "along my path";
    if (DEICTIC_RE.test(t)) return "there";
    for (const [k, v] of KO_REGION) if (t.includes(k)) return v;
    m = t.match(/\b(left|right|center|centre|middle|front|back)\b/); if (m) return normRegion(m[1]);
    return null;
  }
  const ALONG_RE = /\b(along|follow(?:ing)?|via|using|by)\s+(?:my|the|this|that)\s+(path|route|line|trajectory)\b|경로대로|내 경로|경로로|경로 따라/;
  const DEICTIC_RE = /\b(over there|there|here|where i'?m pointing|where i point(?:ed)?|that spot|this spot)\b|저기|여기|거기|저쪽|이쪽/;
  function normRegion(x) { x = x.toLowerCase(); if (x === "centre" || x === "middle") return "center"; return x.length === 1 ? x.toUpperCase() : x; }
  const norm = (x) => String(x).toLowerCase().replace(/[^a-z0-9가-힣]+/g, " ").trim();
  function findSkill(name) { const n = norm(name); return arr(state.skills).map(String).find((k) => norm(k) === n) || null; }

  function parseIntent(text) {
    const raw = String(text).trim(), t = raw.toLowerCase().replace(/[.!?。]+$/g, "").trim();
    if (!t) return null;
    if (/\b(stop|quit|end) following\b|unfollow|don'?t follow|그만 따라|따라하지 마/.test(t)) return ["follow_me", { enabled: false }];
    if (/\bfollow me\b|\bcopy me\b|\bmirror me\b|\bimitate me\b|따라\s?해|따라\s?와|따라\s?하/.test(t)) return ["follow_me", { enabled: true }];
    let m = t.match(/\b(?:learn|remember|save|call)\s+(?:this|it|that)\s+(?:as\s+)?["“']?(.+?)["”']?$/) || t.match(/\b(?:learn|remember) as ["“']?(.+?)["”']?$/);
    if (m) return ["learn_skill", { name: m[1].trim() }];
    m = raw.match(/["“']?(.+?)["”']?\s*(?:으로|로|라고)\s*(?:배워|기억해|저장해)/); if (m) return ["learn_skill", { name: m[1].trim() }];
    if (ALONG_RE.test(t) && (/\b(mug|cup|it|that|carry|move|bring|take|put)\b|\bmy\s+(path|route)\b|머그|컵|옮겨|내 경로|경로대로/.test(t))) return ["pick_place", { object: "mug", target: "along my path" }];
    if (/\b(follow|run|execute|go along|trace)\s+(the\s+)?(path|line|route)\b|경로/.test(t)) return ["follow_path", {}];
    const hasObj = /\b(mug|cup)\b|머그|컵/.test(t);
    const place = /\b(pick|put|move|place|bring|carry|set|drop|take)\b|옮겨|놓아|놔|올려|가져/.test(t);
    if (place && (hasObj || /\b(it|that)\b/.test(t) || /[가-힣]/.test(t))) {
      const target = findTarget(t, raw); const a = { object: "mug" }; if (target) a.target = target; return ["pick_place", a];
    }
    if (/\b(reach|touch|point)\b|뻗어|가리켜/.test(t)) { const target = findTarget(t, raw); if (target) return ["reach", { target }]; }
    const kind = /\b(faster|quicker|speed up)\b|빠르게|빨리/.test(t) ? "faster" : /\b(slower|slow down)\b|천천히|느리게/.test(t) ? "slower"
      : /\b(bigger|larger|wider|stronger)\b|크게/.test(t) ? "bigger" : /\b(smaller|narrower|subtler|weaker)\b|작게/.test(t) ? "smaller" : null;
    if (kind && arr(state.skills).length) {
      const part = /left arm|왼팔|왼쪽 팔/.test(t) ? "left_arm" : /right arm|오른팔|오른쪽 팔/.test(t) ? "right_arm" : /\blegs?\b|다리/.test(t) ? "legs"
        : /\barms?\b|팔/.test(t) ? "arms" : /\b(whole body|body)\b|온몸|몸 전체/.test(t) ? "body" : null;
      const rng = t.match(/(\d+(?:\.\d+)?)\s*(?:s|secs?|seconds?|초)?\s*(?:to|-|–|~|until|부터|에서)\s*(\d+(?:\.\d+)?)\s*(?:s|secs?|seconds?|초)?/);
      const thisPart = /\b(this part|only this|that part|selected|selection)\b|이 부분|이부분|여기만/.test(t);
      if (part || rng || thisPart) {
        const d = editDefaults(), named = arr(state.skills).map(String).find((k) => k && t.includes(k.toLowerCase()));
        const a = { name: named || d.name, part: part || d.part };
        if (rng) { a.t0 = Math.min(+rng[1], +rng[2]); a.t1 = Math.max(+rng[1], +rng[2]); } else if (d.t0 != null) { a.t0 = d.t0; a.t1 = d.t1; }
        Object.assign(a, EDIT_KIND[kind]);
        if (a.name) return ["edit_skill", a];
      }
    }
    m = t.match(/^(?:please\s+)?(?:do|play|perform|replay|show me)\s+(?:the\s+|your\s+)?(.+?)(?:\s+skill|\s+again)?$/);
    if (m && findSkill(m[1])) return ["play_skill", { name: findSkill(m[1]) }];
    for (const k of arr(state.skills).map(String)) if (k && t.includes(k.toLowerCase()) && /해|실행|play|do/.test(t)) return ["play_skill", { name: k }];
    if (/\b(slower|slow down|more slowly|carefully)\b|천천히|느리게/.test(t)) return ["set_speed", { value: "slower" }];
    if (/\b(faster|speed up|quicker|hurry)\b|빠르게|빨리/.test(t)) return ["set_speed", { value: "faster" }];
    if (/\b(stop|halt|freeze|hold on|wait)\b|멈춰|정지|스톱|그만/.test(t)) return ["stop", {}];
    if (/\breset\b|리셋|초기화/.test(t)) return ["reset_scene", {}];
    if (/\b(home|go back|start pose|neutral)\b|원위치|처음으로|제자리/.test(t)) return ["home", {}];
    if (/\b(export|save|download)\b|저장|내보내/.test(t)) return ["export", {}];
    if (/\bclear\b|지워/.test(t)) return ["clear", {}];
    return null;
  }

  async function handleText(text, source = "typed") {
    text = String(text || "").trim(); if (!text) return;
    if (live.session) { live.session.sendText(text); return; } // gemini.js echoes it as a final user transcript -> teachHear there
    addLine("user", text);
    if (teachHear(text)) return;
    const it = parseIntent(text);
    if (!it) { const msg = "I didn't catch an action. Try “put the mug on the right”, “follow me”, or tap the table."; addLine("robot", msg); speak(msg); return; }
    if (it[0] === "clear") { clearAll(); speak("Cleared."); return; }
    const r = await runTask(it[0], it[1], source);
    if (r && r.message) { addLine("robot", r.message); speak(r.message); }
  }
  el.cmdForm.addEventListener("submit", (ev) => { ev.preventDefault(); const v = el.cmd.value; el.cmd.value = ""; handleText(v, "typed"); });

  const ttsQ = []; let ttsBusy = false, ttsOK = true;
  function ttsNext() {
    if (ttsBusy || !ttsQ.length) return;
    const item = ttsQ.shift(); ttsBusy = true;
    const text = item.text;
    const a = new Audio("/tts?voice=" + item.voice + "&text=" + encodeURIComponent(text));
    let settled = false, guard = 0;
    const settle = (fallback, bad) => {
      if (settled) return; settled = true; clearTimeout(guard);
      if (bad) ttsOK = false;
      if (fallback) { try { a.pause(); } catch (e) { /* noop */ } speakBrowser(text); }
      ttsBusy = false; ttsNext();
    };
    guard = setTimeout(() => settle(false), 30000); // a stalled stream must not block the queue
    a.onended = () => settle(false); a.onerror = () => settle(true, true);
    a.play().catch(() => settle(true));
  }
  function speak(msg, lang, voice) {
    if (live.session || !msg) return;
    if (ttsOK && (!lang || String(lang).startsWith("en"))) {
      const short = String(msg).split(/(?<=[.!?])\s/).slice(0, 2).join(" ");
      if (ttsQ.length > 2) ttsQ.splice(0, ttsQ.length - 2);
      ttsQ.push({ text: short, voice: voice || "coral" }); ttsNext(); return;
    }
    speakBrowser(msg, lang);
  }
  function speakBrowser(msg, lang) {
    if (!("speechSynthesis" in window) || !msg) return;
    try {
      speechSynthesis.cancel();
      const short = String(msg).split(/(?<=[.!?])\s/).slice(0, 2).join(" ");
      const u = new SpeechSynthesisUtterance(short); u.lang = lang || settings.lang; u.rate = 1.05;
      if (rec.on) { rec.paused = true; try { rec.r.stop(); } catch (e) { /* noop */ } }
      u.onend = u.onerror = () => { if (rec.on && rec.paused) { rec.paused = false; startRecognizer(); } };
      speechSynthesis.speak(u);
    } catch (e) { /* noop */ }
  }

  // ------------------------------------------------------------------ quick actions
  const acts = {
    pick: () => runTask("pick_place", { object: "mug", target: "right" }, "button").then(sayBack),
    follow: () => runTask("follow_me", { enabled: state.mode !== "FOLLOW" }, "button").then(sayBack),
    learn: () => {
      const name = (window.prompt("Name this skill (from your last follow-me recording):", "wave") || "").trim();
      if (name) runTask("learn_skill", { name }, "button").then(sayBack);
    },
    play: () => { const n = el.skillSel.value; if (n) runTask("play_skill", { name: n }, "button").then(sayBack); },
    stop: () => runTask("stop", {}, "button"),
    home: () => runTask("home", {}, "button").then(sayBack),
    reset: () => runTask("reset_scene", {}, "button").then(sayBack),
    export: () => runTask("export", {}, "button").then(sayBack),
    compile: () => runTask("compile_for", { robot: "ur5e" }, "button").then(sayBack),
  };
  function sayBack(r) { if (r && r.message && !live.session) speak(r.message); }
  document.querySelectorAll("[data-act]").forEach((b) => b.addEventListener("click", () => { const f = acts[b.dataset.act]; if (f) f(); }));

  // ------------------------------------------------------------------ skill editor (part + time range -> edit_skill)
  const se = { name: "", dur: 0, t0: 0, t1: 0, drag: null };
  function skillDuration(name) { const m = arr(state.skills_meta).find((x) => x && String(x.name) === name); return m ? num(+m.duration, 0) : 0; }
  function editDefaults() {
    const name = el.skillSel.value || se.name || (arr(state.skills).length ? String(arr(state.skills).slice(-1)[0]) : "");
    const full = !se.dur || (se.t0 <= 0.001 && se.t1 >= se.dur - 0.001);
    return { name, part: el.sePart.value || "body", t0: full ? null : +se.t0.toFixed(2), t1: full ? null : +se.t1.toFixed(2) };
  }
  function renderSkillEdit() {
    const name = el.skillSel.disabled ? "" : el.skillSel.value, dur = name ? skillDuration(name) : 0;
    el.skillEdit.hidden = !name;
    if (!name) return;
    if (name !== se.name || Math.abs(dur - se.dur) > 1e-3) {
      se.name = name; se.dur = dur; se.t0 = 0; se.t1 = dur;
      el.seName.textContent = name; el.seTrack.hidden = !dur; el.seTicks.innerHTML = "";
      if (dur) {
        const step = dur > 20 ? 5 : dur > 8 ? 2 : 1;
        for (let x = 0; x <= dur + 1e-6; x += step) { const p = (x / dur) * 100; el.seTicks.insertAdjacentHTML("beforeend", `<i style="left:${p}%"></i><b style="left:${p}%">${x}s</b>`); }
        el.seTrack.setAttribute("aria-valuemax", dur.toFixed(1));
      }
    }
    if (se.dur) {
      el.seSel.style.left = (se.t0 / se.dur) * 100 + "%"; el.seSel.style.width = ((se.t1 - se.t0) / se.dur) * 100 + "%";
      el.seRange.textContent = `${se.t0.toFixed(1)}–${se.t1.toFixed(1)} s of ${se.dur.toFixed(1)} s`;
      el.seTrack.setAttribute("aria-valuetext", el.seRange.textContent);
    } else el.seRange.textContent = "whole skill";
  }
  const tAt = (ev) => { const r = el.seTrack.getBoundingClientRect(); return Math.max(0, Math.min(se.dur, ((ev.clientX - r.left) / r.width) * se.dur)); };
  el.seTrack.addEventListener("pointerdown", (ev) => {
    if (!se.dur) return; el.seTrack.setPointerCapture(ev.pointerId);
    const t = tAt(ev), r = el.seTrack.getBoundingClientRect(), px = (x) => (x / se.dur) * r.width, x = ev.clientX - r.left;
    if (Math.abs(x - px(se.t0)) < 10) se.drag = { edge: "t0" };
    else if (Math.abs(x - px(se.t1)) < 10) se.drag = { edge: "t1" };
    else { se.drag = { anchor: t }; se.t0 = se.t1 = t; }
    schedule();
  });
  el.seTrack.addEventListener("pointermove", (ev) => {
    if (!se.drag) return; const t = tAt(ev);
    if (se.drag.edge === "t0") se.t0 = Math.min(t, se.t1 - 0.05); else if (se.drag.edge === "t1") se.t1 = Math.max(t, se.t0 + 0.05);
    else { se.t0 = Math.min(se.drag.anchor, t); se.t1 = Math.max(se.drag.anchor, t); }
    schedule();
  });
  const seEnd = () => { if (!se.drag) return; se.drag = null; if (se.t1 - se.t0 < 0.1) { se.t0 = 0; se.t1 = se.dur; } schedule(); };
  el.seTrack.addEventListener("pointerup", seEnd); el.seTrack.addEventListener("pointercancel", seEnd);
  el.seTrack.addEventListener("keydown", (ev) => {
    if (!se.dur || (ev.key !== "ArrowLeft" && ev.key !== "ArrowRight")) return; ev.preventDefault();
    const d = (ev.key === "ArrowLeft" ? -1 : 1) * Math.max(0.1, se.dur / 20), w = se.t1 - se.t0;
    se.t0 = Math.max(0, Math.min(se.dur - w, se.t0 + d)); se.t1 = se.t0 + w; schedule();
  });
  el.skillSel.addEventListener("change", schedule);
  function editSkill(kind, source) {
    const d = editDefaults(); if (!d.name) return Promise.resolve({ ok: false, message: "No skill selected." });
    const a = { name: d.name, part: d.part }; if (d.t0 != null) { a.t0 = d.t0; a.t1 = d.t1; }
    return runTask("edit_skill", Object.assign(a, EDIT_KIND[kind]), source);
  }
  document.querySelectorAll("[data-edit]").forEach((b) => b.addEventListener("click", () => editSkill(b.dataset.edit, "button").then(sayBack)));

  // ------------------------------------------------------------------ show a video (upload -> whole-body imitation -> auto skill)
  let videoJob = null;
  async function uploadVideo(file) {
    if (!file) return;
    if (!/^video\//.test(file.type) && !/\.(mp4|mov|webm|m4v)$/i.test(file.name)) { toast("Please choose an mp4, mov or webm video."); return; }
    const done = localCard("show_video", { name: file.name }, "upload");
    done("planning", "Uploading…");
    try {
      const fd = new FormData(); fd.append("video", file, file.name);
      const r = await fetch("/upload", { method: "POST", body: fd });
      let j = {}; try { j = await r.json(); } catch (e) { /* non-JSON */ }
      if (!r.ok || j.ok === false) throw new Error(j.message || `Upload failed (HTTP ${r.status}).`);
      done("running", j.message || `Imitating ${j.name || file.name}…`);
      videoJob = { done, name: j.name || file.name, seen: false };
    } catch (e) { done("failed", String(e && e.message || e)); }
  }
  el.videoBtn.addEventListener("click", () => el.videoFile.click());
  el.videoFile.addEventListener("change", () => { uploadVideo(el.videoFile.files && el.videoFile.files[0]); el.videoFile.value = ""; });
  let dragDepth = 0;
  const hasFiles = (ev) => ev.dataTransfer && Array.from(ev.dataTransfer.types || []).includes("Files");
  camPanel.addEventListener("dragenter", (ev) => { if (!hasFiles(ev)) return; ev.preventDefault(); dragDepth++; camPanel.classList.add("dragover"); el.dropHint.hidden = false; });
  camPanel.addEventListener("dragover", (ev) => { if (hasFiles(ev)) { ev.preventDefault(); ev.dataTransfer.dropEffect = "copy"; } });
  camPanel.addEventListener("dragleave", () => { if (--dragDepth <= 0) { dragDepth = 0; camPanel.classList.remove("dragover"); el.dropHint.hidden = true; } });
  camPanel.addEventListener("drop", (ev) => {
    ev.preventDefault(); dragDepth = 0; camPanel.classList.remove("dragover"); el.dropHint.hidden = true;
    uploadVideo(ev.dataTransfer && ev.dataTransfer.files && ev.dataTransfer.files[0]);
  });
  function trackVideo() {
    if (!videoJob) return;
    const v = state.video;
    if (v && typeof v === "object") videoJob.seen = true;
    else if (videoJob.seen) {
      const skill = String(videoJob.name).replace(/\.[^.]+$/, "");
      videoJob.done("done", `Video finished. Saved as a skill${arr(state.skills).map(String).includes(skill) ? ` “${skill}”` : ""} — select it below to edit or play.`);
      if (arr(state.skills).map(String).includes(skill)) { el.skillSel.value = skill; }
      videoJob = null;
    }
  }

  // ------------------------------------------------------------------ talk: Gemini Live or Web Speech fallback
  const live = { session: null, state: "off", level: 0, frameTimer: 0, tick: 0, gen: 0, lines: { user: null, model: null } };
  const rec = { on: false, r: null, paused: false, line: null };

  function setTalk(st, sub) {
    live.state = st;
    for (const b of [el.talk, el.talkBig]) { b.dataset.state = st; b.setAttribute("aria-pressed", String(st !== "off" && st !== "error")); }
    const labels = { off: "Talk", connecting: "Connecting…", live: "Live", listening: "Listening", speaking: "Speaking", error: "Talk" };
    el.talkLabel.textContent = labels[st] || st;
    el.talkBigLabel.textContent = st === "off" || st === "error" ? "Tap to talk" : (labels[st] || st) + " · tap to end";
    el.talkSub.textContent = sub || (st === "off" ? (geminiReady() ? "gemini live · space" : "browser speech · space") : st === "error" ? "error" : live.session ? "gemini live" : "browser speech");
  }
  function setLevel(v) { live.level = v; el.talk.style.setProperty("--lvl", v.toFixed(3)); el.talkBig.style.setProperty("--lvl", v.toFixed(3)); }

  const SYSTEM = [
    "You are the voice and brain of a Unitree G1 humanoid robot standing at a table with a mug on it, in a physics simulation that mirrors the real robot.",
    "PhysicsClay turns human intent into spatial instructions the robot can execute. You hear the user, and you receive frames from the user's camera (they show the user, not the robot).",
    "Your job: turn the user's intent into tool calls. Never claim a motion happened or succeeded before you get the tool result.",
    "Tool results come from the physics engine: {ok, status, message}. status 'running' means accepted and in progress — say you're doing it, not that it's done. If ok is false, say plainly that you can't and why, using the message (e.g. out of reach, joint limit, collision), and suggest an alternative if obvious.",
    "Spatial targets: named table regions are left, right, center, front, back (from the robot's point of view). The user can tap the robot view to create lettered markers (A, B, C…) — use those letters as targets. For anything else (\"next to the bowl\", \"the dark spot\"), pass the description as target, or call point_on_robot_view first.",
    "When the user gestures or points in their camera, interpret it semantically (e.g. pointing to their left = the robot's right side of the table, a wave, a thumbs-up meaning go) and act through tools; ask a short question if it's ambiguous.",
    "Following the user's body is a mode: only call follow_me(enabled=true) when they ask you to follow, copy or mirror them; turn it off when they say stop. After following, they may say 'learn this as <name>' -> learn_skill, and later 'do <name>' -> play_skill.",
    "Messages starting with [context] or [robot update] are from the app, not the user: use them silently; speak only if a robot update reports a failure or completion the user is waiting for.",
    "Skills can be edited by body part and time range: 'only this part faster', 'left arm bigger', 'make 2 to 4 seconds slower' -> edit_skill (omit t0/t1 when they mean the range selected on screen). 'Would it work on a UR5e / an arm?' -> compile_for(robot='ur5e').",
    "When the user says 'there' / 'here' / 'over there' while pointing (camera gesture), call pick_place (or reach) with target 'there' — the server knows where they point (marker P). When they say 'carry it along my path' / 'follow my route', call pick_place with target 'along my path' (the route they built in the 3D editor or drew on the view).",
    "If the user asks to wipe or trace a shape, ask them to draw it on the robot view or place waypoints in the 3D view, then run it with follow_path.",
    "TEACH BY SHOWING is the core experience: the user teaches you a task like teaching a friend — they show it on camera and explain it out loud. You do NOT copy their motion live. The app handles 'start teaching' / 'done' / 'yes' phrases itself.",
    "During teaching, watch the demonstration frames and listen; when they finish (or when you understand), call propose_task with the goal, steps (pick_place object -> target) and constraints (avoid the vase, keep something still, keep upright, slow, grasp side/top), then ask them to confirm in ONE sentence. If they correct you, call propose_task again with the fix.",
    "Speak in short, warm sentences — one or two at a time. Reply in the user's language.",
  ].join("\n");
  const S = (d) => ({ type: "STRING", description: d });
  const TOOLS = [{ functionDeclarations: [
    { name: "pick_place", description: "Pick up an object and place it at a target on the table.", parameters: { type: "OBJECT", properties: {
      object: S("Object to pick, e.g. 'mug'."), target: S("Where to place it: region (left/right/center/front/back), marker letter (A-F or P), 'there' when the user points and says there/here, 'along my path' to carry it along the user's drawn/3D route, or a short visual description.") }, required: ["object"] } },
    { name: "reach", description: "Move a hand to hover over a target on the table.", parameters: { type: "OBJECT", properties: { target: S("Region, marker letter, 'there' (where the user points), or visual description.") }, required: ["target"] } },
    { name: "follow_path", description: "Execute the path the user drew on the robot view." },
    { name: "follow_me", description: "Turn imitation (follow the user's body) on or off. Auto-records while on.", parameters: { type: "OBJECT", properties: { enabled: { type: "BOOLEAN", description: "true to start following, false to stop." } }, required: ["enabled"] } },
    { name: "learn_skill", description: "Save the last follow-me recording as a named skill.", parameters: { type: "OBJECT", properties: { name: S("Skill name, e.g. 'wave'.") }, required: ["name"] } },
    { name: "play_skill", description: "Replay a learned skill (validated by physics).", parameters: { type: "OBJECT", properties: { name: S("Name of a learned skill.") }, required: ["name"] } },
    { name: "set_speed", description: "Change motion speed.", parameters: { type: "OBJECT", properties: { value: S("'slower', 'faster', or a number 0.1-1.") }, required: ["value"] } },
    { name: "stop", description: "Stop the current motion and hold the pose." },
    { name: "home", description: "Return to the start pose; the mug stays where it is." },
    { name: "reset_scene", description: "Return home and put the mug back at the start." },
    { name: "export", description: "Save intent, trajectory and physics report files." },
    { name: "edit_skill", description: "Edit part of a learned skill: a body part over a time range, faster/slower or bigger/smaller. Omit t0/t1 to use the range the user selected on the timeline (or the whole skill).", parameters: { type: "OBJECT", properties: {
      name: S("Learned skill name. Omit to use the selected skill."), t0: { type: "NUMBER", description: "Start time in seconds." }, t1: { type: "NUMBER", description: "End time in seconds." },
      part: { type: "STRING", enum: ["left_arm", "right_arm", "arms", "legs", "body"], description: "Body part to edit." },
      speed: { type: "NUMBER", description: "Time scale: 1.5 = faster, 0.7 = slower." }, amplitude: { type: "NUMBER", description: "Motion size: 1.3 = bigger, 0.75 = smaller." } }, required: ["part"] } },
    { name: "compile_for", description: "Check the last recorded/approved motion or task path on a different robot and report what transfers.", parameters: { type: "OBJECT", properties: { robot: S("Target robot, e.g. 'ur5e'.") }, required: ["robot"] } },
    { name: "propose_task", description: "During teaching (Teach by Showing): propose the task you understood from the user's demonstration frames + speech. Shown to the user as a card to confirm.", parameters: { type: "OBJECT", properties: {
      goal: S("One-sentence goal, e.g. 'Move the mug to the right side without touching the vase'."),
      steps: { type: "ARRAY", items: { type: "OBJECT", properties: { action: { type: "STRING", enum: ["pick_place"] }, object: S("Object, e.g. 'mug'."), target: S("Region, marker letter, 'there', or 'along my path'.") }, required: ["action", "object"] } },
      constraints: { type: "ARRAY", items: { type: "OBJECT", properties: { type: { type: "STRING", enum: ["avoid", "keep_still", "upright", "slow", "grasp_side", "grasp_top"] }, object: S("Object the constraint is about, if any.") }, required: ["type"] } } }, required: ["goal", "steps"] } },
    { name: "point_on_robot_view", description: "Find something in the robot's camera view and drop a marker on it. Returns the marker letter.", parameters: { type: "OBJECT", properties: { query: S("What to find, e.g. 'the empty spot near the edge'.") }, required: ["query"] } },
  ] }];

  async function erPoint(query) {
    const img = b64Of(latest.robot);
    if (!img) throw new Error("no robot view frame yet");
    const pts = await G().point({ ...aiOpts(), model: settings.erModel, imageBase64: img, query });
    const p = arr(pts).find((x) => x && isFinite(x.u) && isFinite(x.v));
    if (!p) throw new Error(`couldn't find “${query}” in my view`);
    return p;
  }
  function waitNewMarker(before, ms = 1500) {
    return new Promise((res) => {
      const t0 = Date.now();
      (function poll() {
        const now = arr(state.markers).map((m) => String(m && m.id));
        const fresh = now.find((id) => !before.includes(id));
        if (fresh) return res(fresh);
        if (Date.now() - t0 > ms) return res(now.length ? now[now.length - 1] : null);
        setTimeout(poll, 60);
      })();
    });
  }
  async function pointOnRobotView(query, source) {
    const done = localCard("point", { query }, source);
    try {
      const p = await erPoint(query);
      const before = arr(state.markers).map((m) => String(m && m.id));
      send({ type: "tap", u: p.u, v: p.v }); tapFx.push({ uv: [p.u, p.v], t: performance.now() }); schedule();
      const id = await waitNewMarker(before);
      done("done", `Marked ${id ? "“" + id + "”" : "the spot"} at (${p.u.toFixed(2)}, ${p.v.toFixed(2)})${p.label ? " · " + p.label : ""}.`);
      return { ok: true, marker: id || "the new marker", u: p.u, v: p.v, label: p.label || query };
    } catch (e) {
      done("failed", String(e && e.message || e)); return { ok: false, message: String(e && e.message || e) };
    }
  }
  async function resolveTarget(args, source) {
    const tgt = args.target == null ? "" : String(args.target).trim();
    if (!tgt || args.target_uv) return args;
    const low = tgt.toLowerCase().replace(/^the\s+/, "").replace(/\s+(side|of the table)$/, "");
    if (REGIONS.includes(low)) return Object.assign({}, args, { target: low });
    if (ALONG_RE.test(low) || /^along\b/.test(low)) return Object.assign({}, args, { target: "along my path" });
    if (/^(there|here|over there|that spot|this spot)$/.test(low) || /where (i'?m|i am|they'?re|the user is) pointing|where i point/.test(low)) return Object.assign({}, args, { target: "there" });
    const mk = tgt.match(/^(?:marker\s+|point\s+)?([a-z])$/i);
    if (mk) return Object.assign({}, args, { target: mk[1].toUpperCase() });
    if (!G() || typeof G().point !== "function" || !haveAI()) return args; // let the server try
    const p = await pointOnRobotView(tgt, source);
    if (!p.ok) throw new Error(p.message);
    return Object.assign({}, args, { target_uv: [p.u, p.v] });
  }

  async function onToolCall(call) {
    const name = call && call.name, args = (call && call.args) || {};
    try {
      if (name === "point_on_robot_view") return await pointOnRobotView(String(args.query || ""), "gemini");
      if (name === "propose_task") {
        const spec = { goal: String(args.goal || ""), steps: arr(args.steps).map((x) => Object.assign({ action: "pick_place" }, x || {})), constraints: arr(args.constraints) };
        if (!send({ type: "teach", cmd: "set_spec", spec })) return { ok: false, message: "Not connected to the robot." };
        return { ok: true, message: "The task card is shown to the user. Ask them to confirm in one short sentence (yes = practice, or they correct you)." };
      }
      if (!TITLES[name]) return { ok: false, message: `unknown tool ${name}` };
      let a = Object.assign({}, args);
      if (name === "pick_place" || name === "reach") a = await resolveTarget(a, "gemini");
      if (name === "edit_skill") {
        const d = editDefaults(); if (!a.name) a.name = d.name; if (!a.part) a.part = d.part;
        if (a.t0 == null && d.t0 != null) { a.t0 = d.t0; a.t1 = d.t1; }
        if (!a.name) return { ok: false, message: "There is no learned skill to edit yet." };
      }
      if (name === "compile_for" && !a.robot) a.robot = "ur5e";
      if (name === "play_skill" && a.name && !findSkill(a.name) && arr(state.skills).length) {
        return { ok: false, message: `I don't know a skill called "${a.name}". I know: ${arr(state.skills).join(", ")}.` };
      }
      const p = runTask(name, a, "gemini");
      const rec = [...tasks.values()].pop(); if (rec) rec.viaModel = true;
      return await p;
    } catch (e) {
      return { ok: false, message: String(e && e.message || e) };
    }
  }

  function onTranscript(tr) {
    if (!tr || typeof tr.text !== "string") return;
    const role = tr.role === "user" ? "user" : "model", text = tr.text.trim();
    if (/^\[(context|robot update)\]/.test(text)) return;
    let line = live.lines[role];
    if (!text) { if (tr.final && line) { line.classList.remove("interim"); live.lines[role] = null; } return; }
    if (!line) { line = addLine(role === "user" ? "user" : "robot", text); live.lines[role] = line; }
    line.querySelector(".txt").textContent = text;
    line.classList.toggle("interim", !tr.final);
    if (tr.final) { live.lines[role] = null; if (role === "user") teachHear(text); }
    const f = el.feed; if (f.scrollHeight - f.scrollTop - f.clientHeight < 120) f.scrollTop = f.scrollHeight;
  }
  function contextToModel(text) { if (live.session && live.session.isOpen !== false) { try { live.session.sendText("[context] " + text); } catch (e) { /* noop */ } } }

  async function startGemini() {
    setTalk("connecting");
    const gen = ++live.gen;
    try {
      const session = await G().connectLive({
        ...aiOpts(), model: settings.liveModel, systemInstruction: SYSTEM, tools: TOOLS,
        onStatus: (st) => {
          st = String(st || "");
          if (/^error/i.test(st)) { toast("Gemini: " + st.replace(/^error:?\s*/i, "")); }
          if (/closed|error/i.test(st)) { if (live.session) stopTalk(/error/i.test(st) ? "error" : "off"); return; }
          if (/speak/i.test(st)) setTalk("speaking"); else if (/listen/i.test(st)) setTalk("listening"); else if (/live|ready|open/i.test(st)) setTalk("live");
        },
        onTranscript, onToolCall, onAudioLevel: (v) => setLevel(Math.max(0, Math.min(1, num(v)))),
      });
      if (gen !== live.gen) { try { session.close(); } catch (e) { /* noop */ } return; } // cancelled while connecting
      live.session = session;
      try { await session.startMic(); } catch (e) { toast("Microphone blocked — type below, the robot still listens."); }
      if (live.state === "connecting") setTalk(session.isMicOn === false ? "live" : "listening");
      addLine("sys", "Gemini Live connected. Talk naturally — gestures in your camera count too.");
      live.frameTimer = setInterval(pushFrames, 1050);
    } catch (e) {
      if (gen !== live.gen) return;
      live.session = null; setTalk("error", "connect failed");
      toast("Couldn't connect to Gemini Live: " + (e && e.message || e) + " — using browser speech instead.");
      startRecognizer(true);
    }
  }
  function pushFrames() {
    const s = live.session; if (!s) return;
    live.tick++;
    const robotTurn = settings.sendRobot && (!settings.sendCam || live.tick % 3 === 0); // gemini.js keeps images ≥1 s apart
    const buf = robotTurn ? latest.robot : (settings.sendCam ? latest.cam : null);
    if (buf) { try { s.sendImage(b64Of(buf), "image/jpeg"); } catch (e) { /* noop */ } }
  }

  // Web Speech fallback
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  function startRecognizer(fromFailover) {
    if (!SR) { setTalk("off"); toast("Voice isn't available in this browser — type your intent below."); el.cmd.focus(); return; }
    rec.on = true;
    if (!rec.r) {
      const r = new SR(); rec.r = r;
      r.continuous = true; r.interimResults = true;
      r.onresult = (ev) => {
        for (let i = ev.resultIndex; i < ev.results.length; i++) {
          const res = ev.results[i], text = res[0].transcript.trim(); if (!text) continue;
          if (!rec.line) rec.line = addLine("user", text);
          rec.line.querySelector(".txt").textContent = text; rec.line.classList.toggle("interim", !res.isFinal);
          if (res.isFinal) { rec.line.remove(); rec.line = null; handleText(text, "voice"); }
        }
      };
      r.onerror = (ev) => { if (ev.error === "not-allowed" || ev.error === "service-not-allowed") { rec.on = false; setTalk("error", "mic blocked"); toast("Microphone permission denied."); } };
      r.onend = () => { if (rec.on && !rec.paused) setTimeout(() => { if (rec.on && !rec.paused) startRecognizer(); }, 150); else if (!rec.on) setTalk("off"); };
    }
    rec.r.lang = settings.lang;
    try { rec.r.start(); } catch (e) { /* already started */ }
    el.talk.classList.add("nolevel"); el.talkBig.classList.add("nolevel");
    setTalk("listening", fromFailover ? "browser speech" : undefined);
  }
  function stopRecognizer() { rec.on = false; rec.paused = false; if (rec.line) { rec.line.remove(); rec.line = null; } try { rec.r && rec.r.stop(); } catch (e) { /* noop */ } }

  function stopTalk(final = "off") {
    live.gen++; clearInterval(live.frameTimer); live.frameTimer = 0;
    const s = live.session; live.session = null;
    if (s) { try { s.stopMic(); } catch (e) { /* noop */ } try { s.close(); } catch (e) { /* noop */ } addLine("sys", "Conversation ended."); }
    for (const k of ["user", "model"]) { if (live.lines[k]) live.lines[k].classList.remove("interim"); live.lines[k] = null; }
    stopRecognizer(); setLevel(0); el.talk.classList.remove("nolevel"); el.talkBig.classList.remove("nolevel");
    setTalk(final);
  }
  function toggleTalk() {
    if (live.state !== "off" && live.state !== "error") { stopTalk(); return; }
    if (geminiReady()) startGemini(); else startRecognizer();
  }
  el.talk.addEventListener("click", toggleTalk); el.talkBig.addEventListener("click", toggleTalk);

  // ------------------------------------------------------------------ settings dialog
  function openSettings(section) {
    if (typeof section !== "string") section = "";
    $("setKey").value = settings.key; $("setLive").value = settings.liveModel; $("setEr").value = settings.erModel;
    $("setLive").placeholder = (G() && G().DEFAULT_LIVE_MODEL) || "live model"; $("setEr").placeholder = (G() && G().DEFAULT_ER_MODEL) || "ER model";
    $("setMock").checked = typeof state.mock === "boolean" ? state.mock : settings.mock;
    $("setSendCam").checked = settings.sendCam; $("setSendRobot").checked = settings.sendRobot; $("setLang").value = settings.lang;
    $("geminiStatus").textContent = G() ? "gemini.js loaded · " + (settings.key ? "own key set → Talk uses Gemini Live" : state.server_gemini ? "server key → Talk uses Gemini Live (proxy)" : "no key → Talk uses browser speech") : "gemini.js not loaded → Talk uses browser speech";
    $("serverGemini").hidden = !state.server_gemini;
    $("setPhoneCam").checked = phoneCam.want; renderSettingsLive();
    if (typeof el.dlg.showModal === "function") el.dlg.showModal(); else el.dlg.setAttribute("open", "");
    if (section) { const sec = $(section); if (sec) { sec.scrollIntoView({ block: "start" }); sec.classList.add("flash"); setTimeout(() => sec.classList.remove("flash"), 1200); } }
  }
  el.dlg.addEventListener("close", () => {
    if (el.dlg.returnValue !== "save") return;
    LS.set("pc.geminiKey", $("setKey").value.trim());
    const lm = $("setLive").value.trim(), em = $("setEr").value.trim();
    LS.set("pc.liveModel", G() && lm === G().DEFAULT_LIVE_MODEL ? "" : lm);
    LS.set("pc.erModel", G() && em === G().DEFAULT_ER_MODEL ? "" : em);
    LS.set("pc.sendCam", $("setSendCam").checked ? "1" : "0"); LS.set("pc.sendRobot", $("setSendRobot").checked ? "1" : "0");
    LS.set("pc.lang", $("setLang").value);
    if (rec.r) rec.r.lang = settings.lang;
    if (live.state === "off" || live.state === "error") setTalk("off");
    toast("Settings saved.");
  });
  $("setMock").addEventListener("change", () => {
    const on = $("setMock").checked; LS.set("pc.mock", on ? "1" : "0"); send({ type: "mock", enabled: on }); schedule();
    addLine("sys", on ? "Mock AI on: offline gestures + scripted replies." : "Mock AI off.");
  });
  $("settingsBtn").addEventListener("click", () => openSettings()); $("settingsBtnM").addEventListener("click", () => openSettings());
  $("camBtnM").addEventListener("click", () => openSettings("secCamera"));
  function renderSettingsLive() {
    const rb = String(state.robot || "g1");
    document.querySelectorAll("[data-robot]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.robot === rb)));
    const sp = state.spatial && typeof state.spatial === "object" ? state.spatial : null;
    if (sp && sp.model && document.activeElement !== $("setSpatial")) $("setSpatial").value = String(sp.model);
    const er = sp && sp.er && typeof sp.er === "object" ? sp.er : null;
    $("erStatus").textContent = er ? `ER · ${er.target || ""} · ${arr(er.points).length} points${er.ms ? " · " + Math.round(er.ms) + " ms" : ""}` : "";
    $("serverGemini").hidden = !state.server_gemini;
    $("camStatus").textContent = phoneCam.status;
  }
  // robot body
  document.querySelectorAll("[data-robot]").forEach((b) => b.addEventListener("click", () => {
    if (!send({ type: "robot", name: b.dataset.robot })) { toast("Not connected to the robot."); return; }
    document.querySelectorAll("[data-robot]").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
    addLine("sys", `Switching body → ${b.textContent}.`);
  }));
  // spatial model + Robotics-ER
  $("setSpatial").addEventListener("change", () => { send({ type: "spatial_model", name: $("setSpatial").value }); });
  $("erCam").addEventListener("click", () => { if (send({ type: "spatial", target: "camera" })) { $("erStatus").textContent = "asking Gemini Robotics-ER (camera)…"; addLine("sys", "Asking Gemini Robotics-ER what it sees in your camera…"); } });
  $("erRobot").addEventListener("click", () => { if (send({ type: "spatial", target: "robot" })) { $("erStatus").textContent = "asking Gemini Robotics-ER (robot view)…"; addLine("sys", "Asking Gemini Robotics-ER what it sees in the robot view…"); } });
  // demo autopilot
  document.querySelectorAll("[data-demo]").forEach((b) => b.addEventListener("click", () => {
    if (!send({ type: "demo", name: b.dataset.demo })) { toast("Not connected to the robot."); return; }
    if (b.dataset.demo !== "stop" && el.dlg.open) el.dlg.close("cancel");
  }));
  const demoParam = new URLSearchParams(location.search).get("demo");
  let demoAuto = !!demoParam;
  function maybeAutoDemo() {
    if (!demoAuto) return; demoAuto = false;
    setTimeout(() => { send({ type: "demo", name: demoParam === "1" || demoParam === "true" ? "full" : demoParam }); }, 1500);
  }

  // ------------------------------------------------------------------ this device's camera -> server (0x10 + JPEG)
  const IS_PHONE = /Android|iPhone|iPad|iPod|Mobile/i.test(navigator.userAgent) || (window.matchMedia && matchMedia("(pointer: coarse)").matches);
  const phoneCam = {
    get want() { return LS.get("pc.phoneCam", "0") === "1"; },
    facing: LS.get("pc.facing", IS_PHONE ? "environment" : "user"),
    stream: null, video: null, canvas: null, timer: 0, busy: false, status: "off", sent: 0, gen: 0,
  };
  async function camStart() {
    camStop(); const gen = phoneCam.gen;
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) { phoneCam.status = "camera not available (needs https)"; schedule(); return; }
    phoneCam.status = "starting…"; schedule();
    try {
      const st = await navigator.mediaDevices.getUserMedia({ video: { facingMode: phoneCam.facing, width: 640 }, audio: false });
      if (!phoneCam.want || gen !== phoneCam.gen) { st.getTracks().forEach((t) => t.stop()); return; }
      phoneCam.stream = st;
      const v = phoneCam.video || (phoneCam.video = Object.assign(document.createElement("video"), { muted: true, playsInline: true, autoplay: true }));
      v.setAttribute("playsinline", ""); v.srcObject = st; try { await v.play(); } catch (e) { /* autoplay */ }
      if (gen !== phoneCam.gen) return; // stopped / restarted while starting (camStop already stopped the tracks)
      phoneCam.canvas = phoneCam.canvas || document.createElement("canvas");
      phoneCam.timer = setInterval(camTick, 100);
      phoneCam.status = `streaming · ${phoneCam.facing === "user" ? "front" : "back"}`; schedule();
    } catch (e) { if (gen === phoneCam.gen) { phoneCam.status = "camera blocked: " + (e && e.name || e); schedule(); } }
  }
  function camStop() {
    phoneCam.gen++; clearInterval(phoneCam.timer); phoneCam.timer = 0;
    if (phoneCam.stream) phoneCam.stream.getTracks().forEach((t) => t.stop());
    phoneCam.stream = null; phoneCam.status = "off"; schedule();
  }
  function camTick() {
    const v = phoneCam.video; if (phoneCam.busy || !v || v.readyState < 2 || !v.videoWidth) return;
    if (!ws || ws.readyState !== 1 || ws.bufferedAmount > 300000) return;
    const w = Math.min(640, v.videoWidth), h = Math.round(w * v.videoHeight / v.videoWidth), cv = phoneCam.canvas;
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    cv.getContext("2d").drawImage(v, 0, 0, w, h);
    phoneCam.busy = true;
    cv.toBlob(async (blob) => {
      try {
        if (blob && ws && ws.readyState === 1) {
          const jpg = new Uint8Array(await blob.arrayBuffer()), out = new Uint8Array(jpg.length + 1);
          out[0] = 0x10; out.set(jpg, 1); ws.send(out.buffer); phoneCam.sent++;
        }
      } catch (e) { /* noop */ } finally { phoneCam.busy = false; }
    }, "image/jpeg", 0.6);
  }
  $("setPhoneCam").addEventListener("change", () => {
    LS.set("pc.phoneCam", $("setPhoneCam").checked ? "1" : "0");
    if (phoneCam.want) camStart(); else camStop();
  });
  $("camFlip").addEventListener("click", () => {
    phoneCam.facing = phoneCam.facing === "user" ? "environment" : "user"; LS.set("pc.facing", phoneCam.facing);
    if (phoneCam.want) camStart(); else phoneCam.status = `off · next: ${phoneCam.facing === "user" ? "front" : "back"}`;
    schedule();
  });
  if (phoneCam.want) camStart();

  // feed collapse (mobile mostly)
  el.feedToggle.addEventListener("click", () => {
    const open = el.feedToggle.getAttribute("aria-expanded") !== "true";
    el.feedToggle.setAttribute("aria-expanded", String(open)); el.feedPanel.classList.toggle("collapsed", !open);
  });

  let toastTimer = 0;
  function toast(msg) { el.toast.textContent = msg; el.toast.hidden = false; clearTimeout(toastTimer); toastTimer = setTimeout(() => { el.toast.hidden = true; }, 3800); }

  // ------------------------------------------------------------------ keyboard
  document.addEventListener("keydown", (ev) => {
    const tg = ev.target, typing = tg && (tg.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(tg.tagName));
    if (typing) { if (ev.key === "Escape" && tg === el.cmd) tg.blur(); return; }
    if (el.dlg.open || ev.metaKey || ev.ctrlKey || ev.altKey || ev.repeat) return;
    if (ev.key === " ") { if (tg && tg.tagName === "BUTTON" && tg !== el.talk) return; ev.preventDefault(); toggleTalk(); }
    else if (ev.key === "Escape") { runTask("stop", {}, "key"); }
    else if (ev.key === "f" || ev.key === "F") { runTask("follow_path", {}, "key"); }
    else if (ev.key === "c" || ev.key === "C") { clearAll(); }
    else if (ev.key === "/") { ev.preventDefault(); el.cmd.focus(); }
  });

  // debug/demo hook: window.PC.ingest(stateObject) renders a state without a server
  document.getElementById("pip").addEventListener("click", (ev) => {
    ev.stopPropagation(); send({ type: "view", name: state.view === "robot" ? "stage" : "robot" });
  });
  ["pointerdown", "pointerup", "pointermove"].forEach((t) => document.getElementById("pip").addEventListener(t, (ev) => ev.stopPropagation()));
  window.PC = { toast: (m) => toast(m), ingest(s) { state = s || {}; gotState = true; onState(); schedule(); if (P3()) P3().setState(state); }, runTask, parseIntent, send, addLine, get state() { return state; } };
  if (P3()) mount3d();

  setTalk("off");
  schedule();
  connect();
})();
