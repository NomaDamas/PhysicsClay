// P2b · VR / MR on Meta Quest (WebXR), docs/vr.md. Open the same page in the Quest browser over https on your LAN:
//   Enter VR ... the table floats in front of you at a comfortable height (Recenter puts it there again)
//   Enter MR ... passthrough: pinch your real table's near-left, then near-right corner, and the robot stands on it
// In the headset: pinch and move to draw a trajectory in the air (1:1), pinch a ghost handle to move it and twist your
// wrist to turn the gripper, a quick pinch drops a keyframe, a fist (or the grip button) drives the robot's hand live,
// and Scan turns your room's planes into the scene. The menu sits by your left hand (palm toward you) or left controller.
// The pure parts (alignment, gestures, strokes, the input -> message pipeline, room -> scene) are web/xr-core.js.
import * as X from "./xr-core.js";

const OPT_FEATURES = ["local-floor", "hand-tracking", "plane-detection", "mesh-detection", "anchors", "hit-test", "bounded-floor"];
const JOINTS = ["wrist", "thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip",
  "index-finger-metacarpal", "index-finger-phalanx-proximal", "index-finger-phalanx-intermediate", "index-finger-phalanx-distal", "index-finger-tip",
  "middle-finger-metacarpal", "middle-finger-phalanx-proximal", "middle-finger-phalanx-intermediate", "middle-finger-phalanx-distal", "middle-finger-tip",
  "ring-finger-metacarpal", "ring-finger-phalanx-proximal", "ring-finger-phalanx-intermediate", "ring-finger-phalanx-distal", "ring-finger-tip",
  "pinky-finger-metacarpal", "pinky-finger-phalanx-proximal", "pinky-finger-phalanx-intermediate", "pinky-finger-phalanx-distal", "pinky-finger-tip"];
const LS_MR = "pc.xr.mr", LS_PREFS = "pc.xr.prefs";
const MODE_LABEL = { draw: "Draw", keyframe: "Keyframes", teleop: "Drive", scan: "Scan room", calibrate: "Calibrate" };
const SPEED_LABEL = { off: "Speed: robot's", hand: "Speed: as drawn", pinch: "Speed: pinch" };
const store = {
  get(k) { try { return JSON.parse(localStorage.getItem(k) || "null"); } catch (e) { return null; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* private mode: not remembered */ } },
};

let V = null, T = null;
const P = () => window.PCUI;
let supported = { vr: false, ar: false, checked: false };
let session = null, kind = null, refType = null, rig = null, xrCam = null, vis = null, busy = false, saved = null, curFrame = null;
let al = null, alSource = null, needRecenter = false, head = null, lastZ = null, corners = null;
let anchor = null, anchorRef = null, anchorInfo = null;
let ix = null, st = {};
const prefs = Object.assign({ side: "across", speedFrom: "off" }, store.get(LS_PREFS) || {});
let toastText = "", toastUntil = 0, calibN = 0, roomResult = null, roomRaw = null, lastScan = 0, roomId = null, roomAvailable = true;
const seen = { traj: null, tele: null, task: null };
const info = { frames: 0, errors: [], sent: [], started: 0, ended: 0, features: [] };

// ------------------------------------------------------------------ page: Enter VR / Enter MR buttons
const css = document.createElement("style");
css.textContent = `.xr-enter[hidden] { display: none !important; } .xr-enter .seg-btn { font-weight: 650; }
html[data-xr] .xr-enter .seg-btn { opacity: .5; }`;
document.head.appendChild(css);

async function detect() {
  const xr = navigator.xr;
  if (!xr || typeof xr.isSessionSupported !== "function") { supported = { vr: false, ar: false, checked: true }; renderButtons(); return supported; }
  const sup = async (m) => { try { return !!(await xr.isSessionSupported(m)); } catch (e) { return false; } };
  const [vr, ar] = await Promise.all([sup("immersive-vr"), sup("immersive-ar")]);
  supported = { vr, ar, checked: true };
  renderButtons();
  return supported;
}
function renderButtons() {
  const host = document.querySelector(".stage-tl") || (P() && P().slot("stage"));
  if (!host) return;
  let box = document.getElementById("xrEnter");
  if (!box) {
    box = document.createElement("div"); box.id = "xrEnter"; box.className = "seg xr-enter";
    box.setAttribute("role", "group"); box.setAttribute("aria-label", "Headset");
    ["pointerdown", "pointerup", "pointermove"].forEach((t) => box.addEventListener(t, (e) => e.stopPropagation()));
    host.appendChild(box);
  }
  const b = (k, label, title) => `<button type="button" class="seg-btn" data-xr="${k}" title="${title}">${label}</button>`;
  box.innerHTML = (supported.vr ? b("vr", "Enter VR", "Draw paths in the air, grab ghost grippers, drive the robot (headset)") : "")
    + (supported.ar ? b("ar", "Enter MR", "Passthrough: the robot on your real table (headset)") : "");
  box.hidden = !(supported.vr || supported.ar);
  box.querySelectorAll("[data-xr]").forEach((el) => el.addEventListener("click", () => enter(el.dataset.xr)));
}

// ------------------------------------------------------------------ session
async function enter(k) {
  if (session || busy || !V) return false;
  if (window.PC && window.PC.setStageView) window.PC.setStageView("3d");       // the 3D stage must be live (scene, ghost)
  busy = true;
  let s;
  try {
    s = await navigator.xr.requestSession(k === "ar" ? "immersive-ar" : "immersive-vr", { optionalFeatures: OPT_FEATURES });
  } catch (e) {
    busy = false;
    P().toast(`Couldn't start ${k === "ar" ? "MR" : "VR"}: ${(e && e.message) || e}`);
    return false;
  }
  try { await start(s, k); } catch (e) {
    console.error("[xr]", e); info.errors.push(String((e && e.stack) || e));
    P().toast(`The headset session failed: ${(e && e.message) || e}`);
    try { await s.end(); } catch (e2) { /* already over */ }
    teardown();
    busy = false; return false;
  }
  busy = false;
  return true;
}

async function start(s, k) {
  const R = V.renderer, S = V.scene;
  session = s; kind = k;
  refType = "local-floor";
  try { await s.requestReferenceSpace("local-floor"); } catch (e) { refType = "local"; }
  R.xr.enabled = true;
  R.xr.setReferenceSpaceType(refType);
  saved = { bg: S.background, fog: S.fog, shadows: R.shadowMap.enabled, clear: R.getClearColor(new T.Color()).getHex(), alpha: R.getClearAlpha() };
  if (k === "ar") { S.background = null; S.fog = null; R.setClearColor(0x000000, 0); }
  R.shadowMap.enabled = false;                                   // cheaper on the headset; contact shadows are not worth it here
  rig = new T.Group(); rig.name = "xr-rig"; rig.matrixAutoUpdate = false;
  xrCam = new T.PerspectiveCamera(70, 1, 0.02, 60); rig.add(xrCam);
  S.add(rig);
  vis = buildVisuals();
  ix = X.createInteraction({ send: (m) => { const ok = P().send(m); info.sent.push(m); if (info.sent.length > 200) info.sent.shift(); return ok; },
    now: () => performance.now(), opts: { speedFrom: prefs.speedFrom, view: prefs.side } });
  ix.setState(st);
  ix.on(onIx);
  s.addEventListener("end", onEnd);
  R.setAnimationLoop(onFrame);
  await R.xr.setSession(s);
  info.features = Array.from(s.enabledFeatures || []);
  info.started++;
  al = null; alSource = null; needRecenter = true; corners = null; anchor = null; anchorRef = null; lastZ = V.TABLE.z;
  if (k === "ar") {
    const sv = store.get(LS_MR);
    if (sv && sv.align && typeof sv.align.yaw === "number") restoreSaved(sv);
    else { ix.mode = "calibrate"; calibN = 0; toast("Pinch your table's near-left corner, then its near-right corner."); }
  } else toast("Pinch and move to draw. Menu: turn your left palm toward you.");
  document.documentElement.dataset.xr = k;
  mrLook(k === "ar");
}

function onEnd() {
  if (ix && ix.mode === "teleop") ix.mode = "draw";              // sends teleop stop
  teardown();
}
function teardown() {
  const R = V && V.renderer, S = V && V.scene;
  if (R) R.setAnimationLoop(null);
  if (saved && S && R) {
    S.background = saved.bg; S.fog = saved.fog; R.shadowMap.enabled = saved.shadows; R.setClearColor(saved.clear, saved.alpha);
  }
  mrLook(false);
  if (rig && S) S.remove(rig);
  if (vis) { for (const o of vis.all) disposeObj(o); vis = null; }
  const tj = window.PCTraj; if (tj && grabbing != null) tj.cancel(grabbing);
  grabbing = null;
  session = null; kind = null; rig = null; xrCam = null; saved = null; curFrame = null; anchor = null; ix = null;
  delete document.documentElement.dataset.xr;
  info.ended++;
  if (V) V.redraw();
}
function disposeObj(o) {
  if (!o) return;
  if (o.parent) o.parent.remove(o);
  o.traverse((c) => { if (c.geometry) c.geometry.dispose(); if (c.material) { if (c.material.map) c.material.map.dispose(); c.material.dispose(); } });
}

// ------------------------------------------------------------------ alignment
function recenter() {
  needRecenter = false;
  if (!head) return;
  al = X.vrAlignment(head, V.TABLE, prefs.side);
  alSource = "recenter";
  lastZ = V.TABLE.z;
}
function restoreSaved(sv) {
  al = sv.align; alSource = "saved"; needRecenter = false; corners = null;     // corners were touched in another session's space
  if (sv.side) prefs.side = sv.side;
  if (typeof sv.align.tableY === "number") al = X.withSurface(al, V.TABLE.z);
  const a = sv.anchor;
  anchorInfo = a || null;
  if (a && a.uuid && session && typeof session.restorePersistentAnchor === "function") {
    session.restorePersistentAnchor(a.uuid).then((an) => { anchor = an; anchorRef = { p: a.p, yaw: a.yaw || 0 }; anchorInfo = a; })
      .catch(() => toast("Using your saved table calibration. If the robot isn't on your table, press Calibrate."));
    toast("Welcome back: the robot is on your table (saved calibration).");
  } else toast("Using your saved table calibration. If the robot isn't on your table, press Calibrate.");
}
function saveMR() {
  if (kind !== "ar" || !al) return;
  store.set(LS_MR, { v: 1, align: al, side: prefs.side, corners, anchor: anchorInfo, refType, at: Date.now() });
}
async function makeAnchor(p) {
  anchorInfo = null;
  if (!curFrame || typeof curFrame.createAnchor !== "function" || typeof XRRigidTransform === "undefined") return;
  try {
    const a = await curFrame.createAnchor(new XRRigidTransform({ x: p[0], y: p[1], z: p[2] }, { x: 0, y: 0, z: 0, w: 1 }), V.renderer.xr.getReferenceSpace());
    anchor = a; anchorRef = { p: p.slice(), yaw: 0 };
    let uuid = null;
    if (a && typeof a.requestPersistentHandle === "function") { try { uuid = await a.requestPersistentHandle(); } catch (e) { uuid = null; } }
    anchorInfo = { uuid, p: p.slice(), yaw: 0 };
    saveMR();
  } catch (e) { /* anchors not granted: the alignment holds until the space is recentred */ }
}
function trackAnchor(frame, ref) {
  if (!anchor || !al || !frame.trackedAnchors || !frame.trackedAnchors.has(anchor)) return;
  const pose = frame.getPose(anchor.anchorSpace, ref);
  if (!pose) return;
  const now = { p: X.fromXRPoint(pose.transform.position), yaw: X.yawOfXR(X.fromXRQuat(pose.transform.orientation)) };
  if (!anchorRef) { anchorRef = now; return; }
  if (X.dist(now.p, anchorRef.p) > 0.002 || Math.abs(X.wrapPi(now.yaw - anchorRef.yaw)) > 0.003) {
    al = X.rebaseAlignment(al, anchorRef, now); anchorRef = now; corners = null;
  }
}
function setSide(side) {
  const old = prefs.side;
  prefs.side = side; store.set(LS_PREFS, prefs);
  if (ix) ix.opts.view = side;
  if (al && old !== side) { try { al = X.switchSide(al, V.TABLE, old, side); saveMR(); } catch (e) { /* keep */ } }
  toast(side === "across" ? "The robot faces you across the table." : "You stand on the robot's side (its view).");
}

// ------------------------------------------------------------------ input (WebXR -> plain HandInput objects)
function readInputs(frame, ref) {
  const out = {};
  for (const src of session.inputSources || []) {
    const side = src.handedness;
    if (side !== "left" && side !== "right") continue;
    if (src.hand) {
      const J = {}, Jsim = [];
      let wq = null;
      for (const name of JOINTS) {
        const j = src.hand.get(name), jp = j && frame.getJointPose ? frame.getJointPose(j, ref) : null;
        if (!jp) continue;
        J[name] = X.fromXRPoint(jp.transform.position);
        Jsim.push(X.xrToSim(al, J[name]));
        if (name === "wrist") wq = X.fromXRQuat(jp.transform.orientation);
      }
      const m = X.handMetrics(J);
      if (!m.pinchXR) { out[side] = { side, kind: "hand", tracked: false }; continue; }
      out[side] = { side, kind: "hand", tracked: true, pxr: m.pinchXR, p: X.xrToSim(al, m.pinchXR), palm: X.xrToSim(al, m.palmXR || m.pinchXR),
        q: wq ? X.quatXrToSim(al, wq) : null, wqXR: wq, wristXR: J.wrist || null, pinchDist: m.pinchDist, curl: m.curl,
        tip: J["index-finger-tip"] ? X.xrToSim(al, J["index-finger-tip"]) : null, joints: Jsim };
    } else {
      const gp = src.gripSpace ? frame.getPose(src.gripSpace, ref) : null, rp = src.targetRaySpace ? frame.getPose(src.targetRaySpace, ref) : null;
      const base = gp || rp;
      if (!base) { out[side] = { side, kind: "controller", tracked: false }; continue; }
      const gpos = X.fromXRPoint(base.transform.position), gq = X.fromXRQuat(base.transform.orientation);
      const rpos = rp ? X.fromXRPoint(rp.transform.position) : gpos, rq = rp ? X.fromXRQuat(rp.transform.orientation) : gq;
      const dir = X.qrot(rq, [0, 0, -1]), tool = X.add(rpos, X.scale(dir, 0.05));
      const btn = (i) => { const b = src.gamepad && src.gamepad.buttons && src.gamepad.buttons[i]; return b ? (typeof b.value === "number" ? b.value : b.pressed ? 1 : 0) : 0; };
      const o = X.xrToSim(al, rpos);
      out[side] = { side, kind: "controller", tracked: true, pxr: tool, p: X.xrToSim(al, tool), palm: X.xrToSim(al, gpos), q: X.quatXrToSim(al, gq),
        trigger: btn(0), squeeze: btn(1), tip: X.xrToSim(al, tool), ray: { o, d: X.sub(X.xrToSim(al, X.add(rpos, dir)), o) } };
    }
  }
  return out;
}

// ------------------------------------------------------------------ panels (canvas textures) and the menu
function makePanel(wPx, hPx, wM) {
  const cv = document.createElement("canvas"); cv.width = wPx; cv.height = hPx;
  const tex = new T.CanvasTexture(cv); tex.colorSpace = T.SRGBColorSpace;
  const hM = wM * hPx / wPx;
  const mesh = new T.Mesh(new T.PlaneGeometry(wM, hM), new T.MeshBasicMaterial({ map: tex, transparent: true, depthTest: false, side: T.DoubleSide }));
  mesh.renderOrder = 30; mesh.up.set(0, 0, 1); mesh.frustumCulled = false;
  return { cv, ctx: cv.getContext("2d"), tex, mesh, w: wM, h: hM, W: wPx, H: hPx, buttons: [], key: "", press: {}, hover: null };
}
function rrect(c, x, y, w, h, r) { c.beginPath(); c.roundRect(x, y, w, h, r); }
function menuButtons() {
  const tj = st.traj || {}, has = !!(tj.proposed || tj.current), m = ix ? ix.mode : "draw";
  const B = [
    { id: "go", label: "▶ Go", on: false, dis: !has || tj.ok === false || tj.running, accent: true },
    { id: "undo", label: "↶ Undo", dis: !(ix && ix.undoDepth) },
    { id: "clear", label: "✕ Clear", dis: !has },
    { id: "exit", label: "Exit", dis: false },
    { id: "mode:draw", label: "Draw", on: m === "draw" }, { id: "mode:keyframe", label: "Keys", on: m === "keyframe" },
    { id: "mode:teleop", label: "Drive", on: m === "teleop" }, { id: "mode:scan", label: "Scan", on: m === "scan" },
    kind === "ar" ? { id: "calibrate", label: "Calibrate", on: m === "calibrate" } : { id: "recenter", label: "Recenter" },
    { id: "side", label: prefs.side === "across" ? "Side: facing" : "Side: robot's" },
    { id: "speed", label: SPEED_LABEL[prefs.speedFrom] || "Speed" },
    { id: "talk", label: "🎤 Talk", dis: !document.getElementById("talk") },
  ];
  if (m === "scan") B.push({ id: "room", label: "Use my room", accent: true, dis: !(roomResult && roomResult.ok) });
  if (st.dataset && typeof st.dataset === "object") B.push({ id: "record", label: st.dataset.recording ? "■ Stop rec" : "● Record", on: !!st.dataset.recording });
  return B;
}
function paintMenu(pn) {
  const B = menuButtons();
  const key = JSON.stringify([B, pn.hover]);
  if (key === pn.key) return;
  pn.key = key;
  const c = pn.ctx, cols = 4, pad = 14, bw = (pn.W - pad * (cols + 1)) / cols, rows = Math.ceil(B.length / cols);
  const bh = Math.min(110, (pn.H - pad * (rows + 1)) / Math.max(rows, 4));
  c.clearRect(0, 0, pn.W, pn.H);
  c.fillStyle = "rgba(24,26,32,0.88)"; rrect(c, 0, 0, pn.W, pn.H, 36); c.fill();
  pn.buttons = [];
  B.forEach((b, i) => {
    const x = pad + (i % cols) * (bw + pad), y = pad + Math.floor(i / cols) * (bh + pad);
    c.fillStyle = b.dis ? "rgba(255,255,255,0.06)" : b.on ? "#f59145" : b.accent ? "#5c77df" : pn.hover === b.id ? "rgba(255,255,255,0.28)" : "rgba(255,255,255,0.14)";
    rrect(c, x, y, bw, bh, 22); c.fill();
    c.fillStyle = b.dis ? "rgba(255,255,255,0.35)" : "#fff"; c.font = "600 38px -apple-system, system-ui, sans-serif";
    c.textAlign = "center"; c.textBaseline = "middle"; c.fillText(b.label, x + bw / 2, y + bh / 2, bw - 16);
    pn.buttons.push({ id: b.id, dis: !!b.dis, x, y, w: bw, h: bh });
  });
  pn.tex.needsUpdate = true;
}
function statusLines() {
  const tj = st.traj || {}, tr = tj.proposed || tj.current, tp = st.teleop || {}, m = ix ? ix.mode : "draw";
  const L = [];
  const hint = {
    draw: "Pinch and move to draw a path in the air. Pinch a handle to move it, twist to turn the wrist. A quick pinch adds a keyframe.",
    keyframe: "Pinch where the gripper should go: keyframes connect into a path. Pinch a handle to move it.",
    teleop: "Make a fist (or hold the grip button) to drive my hand. Pinch / trigger closes the gripper. Open your hand to let go.",
    scan: !roomAvailable ? NO_ROOM : roomResult ? (roomResult.ok ? roomResult.notes.join(" ") : roomResult.notes[0]) : "Look around your room. I use the planes and furniture your Quest knows (Space Setup).",
    calibrate: `Touch and pinch your table's near-${calibN === 0 ? "LEFT" : "RIGHT"} corner (${calibN}/2).`,
  }[m];
  L.push({ t: hint, c: "#e8eaf2" });
  if (tr) {
    const n = (tr.points || []).length, dur = tr.check && tr.check.duration != null ? `${Number(tr.check.duration).toFixed(1)} s` : "";
    if (tj.running) L.push({ t: `Moving along it… ${Math.round((tj.progress || 0) * 100)}%`, c: "#f5b27a" });
    else if (tj.ok === false) L.push({ t: `✕ ${tj.why || "Part of the path is blocked (red)."}`, c: "#ff8a80" });
    else L.push({ t: `✓ ${n} point${n === 1 ? "" : "s"}${dur ? ", " + dur : ""}. Press Go or say “go”.`, c: "#9be3b4" });
  }
  if (grabView) L.push({ t: `Point ${grabView.i + 1}${grabView.twist > 10 ? ` · wrist ${grabView.twist}°` : ""}`, c: "#f5b27a" });
  if (m === "teleop" || tp.active) {
    if (tp.active) {
      const driving = !!(ix && ix.view && ix.view.clutch && ix.view.clutch.length);   // the echo is our own command's clock
      const rtt = driving && typeof tp.echo === "number" ? Math.max(0, Math.round(performance.now() - tp.echo)) : null;
      L.push({ t: `Driving my ${tp.role || "right"} hand · ${tp.rate || 0}/s${rtt != null && rtt < 5000 ? ` · ${rtt} ms` : ""}${tp.clamped ? " · edge of my reach" : ""}`, c: "#9be3b4" });
      if (tp.held) L.push({ t: `Holding: ${tp.held}`, c: "#ff8a80" });
    } else L.push({ t: "Not driving.", c: "#c4c8d4" });
  }
  if (st.dataset && st.dataset.recording) L.push({ t: "● Recording a demo", c: "#ff6b6b" });
  if (toastText && performance.now() < toastUntil) L.push({ t: toastText, c: "#ffd27a" });
  return { title: `${MODE_LABEL[m] || m}${kind === "ar" ? " · MR" : " · VR"}`, lines: L };
}
function wrapText(c, text, maxW) {
  const words = String(text).split(/\s+/), lines = [];
  let cur = "";
  for (const w of words) { const t = cur ? cur + " " + w : w; if (c.measureText(t).width > maxW && cur) { lines.push(cur); cur = w; } else cur = t; }
  if (cur) lines.push(cur);
  return lines;
}
function paintStatus(pn) {
  const now = performance.now();
  if (now - (pn.at || 0) < 200) return;
  const s = statusLines(), key = JSON.stringify(s);
  if (key === pn.key) return;
  pn.key = key; pn.at = now;
  const c = pn.ctx;
  c.clearRect(0, 0, pn.W, pn.H);
  c.fillStyle = "rgba(24,26,32,0.84)"; rrect(c, 0, 0, pn.W, pn.H, 34); c.fill();
  c.textAlign = "left"; c.textBaseline = "top";
  c.fillStyle = "#fff"; c.font = "700 44px -apple-system, system-ui, sans-serif"; c.fillText(s.title, 30, 22);
  let y = 84;
  c.font = "500 32px -apple-system, system-ui, sans-serif";
  for (const l of s.lines) {
    c.fillStyle = l.c;
    for (const line of wrapText(c, l.t, pn.W - 60)) { if (y > pn.H - 40) break; c.fillText(line, 30, y); y += 40; }
    y += 6;
  }
  pn.tex.needsUpdate = true;
}
function toast(text) { toastText = String(text || ""); toastUntil = performance.now() + 4500; }

const NO_ROOM = "This session has no room data (plane / mesh detection). On Quest: Settings → Physical space → Space Setup, and allow it when asked.";
function setMode(m) {
  if (!ix) return;
  ix.mode = m;
  if (m === "scan") { roomResult = null; lastScan = 0; roomAvailable = !!(curFrame && (curFrame.detectedPlanes || curFrame.detectedMeshes)); if (!roomAvailable) toast(NO_ROOM); }
  if (m === "calibrate") calibN = 0;
}
function action(id) {
  if (!ix) return;
  if (id.startsWith("mode:")) { const m = id.slice(5); setMode(ix.mode === m && m !== "draw" ? "draw" : m); return; }
  switch (id) {
    case "go": ix.go(); break;
    case "undo": ix.undo(); break;
    case "clear": ix.clear(); break;
    case "exit": if (session) session.end(); break;
    case "recenter": needRecenter = true; toast("Recentred: the table is in front of you."); break;
    case "calibrate": setMode("calibrate"); toast("Pinch your table's near-left corner, then its near-right corner."); break;
    case "side": setSide(prefs.side === "across" ? "behind" : "across"); break;
    case "speed": {
      const order = ["off", "hand", "pinch"];
      prefs.speedFrom = order[(order.indexOf(prefs.speedFrom) + 1) % order.length]; ix.opts.speedFrom = prefs.speedFrom; store.set(LS_PREFS, prefs);
      toast({ off: "Paths use the robot's default speed.", hand: "Paths move as fast as you draw them.", pinch: "A firmer pinch (or trigger) draws a faster path." }[prefs.speedFrom]);
      break;
    }
    case "talk": { const b = document.getElementById("talk"); if (b) { b.click(); toast("Listening (the page's microphone). Say what to change."); } break; }
    case "room": useRoom(); break;
    case "record": P().send({ type: "dataset", op: st.dataset && st.dataset.recording ? "stop" : "start", source: "vr_teleop", id: `vr-rec-${Date.now() % 100000}` }); break;
  }
}

// ------------------------------------------------------------------ room scan
function applyM(M, x, y, z) { return [M[0] * x + M[4] * y + M[8] * z + M[12], M[1] * x + M[5] * y + M[9] * z + M[13], M[2] * x + M[6] * y + M[10] * z + M[14]]; }
function collectRoom(frame, ref) {
  const planes = [], meshes = [];
  if (frame.detectedPlanes) {
    for (const pl of frame.detectedPlanes) {
      const pose = frame.getPose(pl.planeSpace, ref);
      if (!pose || !pl.polygon) continue;
      const M = pose.transform.matrix;
      planes.push({ orientation: String(pl.orientation || "").toLowerCase(), label: pl.semanticLabel || pl.label || "",
        polygon: Array.from(pl.polygon).map((q) => applyM(M, q.x, q.y, q.z)) });
    }
  }
  if (frame.detectedMeshes) {
    for (const ms of frame.detectedMeshes) {
      const pose = frame.getPose(ms.meshSpace, ref);
      if (!pose || !ms.vertices) continue;
      const M = pose.transform.matrix, v = ms.vertices, n = Math.floor(v.length / 3), step = Math.max(1, Math.floor(n / 400)), pts = [];
      for (let i = 0; i < n; i += step) pts.push(applyM(M, v[3 * i], v[3 * i + 1], v[3 * i + 2]));
      meshes.push({ label: ms.semanticLabel || ms.label || "", points: pts });
    }
  }
  return { planes, meshes, viewer: head ? head.p : [0, 1.6, 0], floorY: refType === "local-floor" ? 0 : undefined };
}
function scanRoom(frame, ref) {
  roomAvailable = !!(frame.detectedPlanes || frame.detectedMeshes);
  if (!roomAvailable) { roomResult = null; return null; }
  roomRaw = collectRoom(frame, ref);
  const trusted = alSource === "calibrated" || alSource === "saved" ? al : null;
  const keep = (((st.world || {}).scene || {}).objects || []).filter((o) => o && o.movable !== false);
  roomResult = X.roomToScene(roomRaw, trusted, V.TABLE, { side: prefs.side, keep });
  return roomResult;
}
function useRoom() {
  if (!roomResult || !roomResult.ok) { toast(roomResult ? roomResult.notes[0] : "Look around first: I need your room's planes."); return false; }
  roomId = `vr-room-${Date.now() % 100000}`;
  const ok = P().send({ type: "world", op: "set", scene: roomResult.spec, id: roomId });
  if (roomResult.align) { al = roomResult.align; if (alSource !== "calibrated" && alSource !== "saved") alSource = "room"; lastZ = null; saveMR(); }
  toast(ok ? "Building your room…" : "Not connected to the robot.");
  return ok;
}

// ------------------------------------------------------------------ visuals
let grabbing = null, grabView = null;
function buildVisuals() {
  const all = [];
  COL_HOT = new T.Color(0xf59145); COL_IDLE = new T.Color(0x5c77df);
  const add = (o, parent) => { (parent || V.scene).add(o); all.push(o); return o; };
  const jointGeo = new T.SphereGeometry(0.0065, 10, 8);
  const hands = add(new T.InstancedMesh(jointGeo, new T.MeshBasicMaterial({ color: 0xdfe5ff, transparent: true, opacity: 0.85, depthTest: true }), 60));
  hands.instanceMatrix.setUsage(T.DynamicDrawUsage); hands.count = 0; hands.frustumCulled = false;
  const tips = add(new T.InstancedMesh(new T.SphereGeometry(0.011, 14, 10), new T.MeshBasicMaterial({ color: 0xffffff }), 4));
  tips.instanceMatrix.setUsage(T.DynamicDrawUsage); tips.count = 0; tips.frustumCulled = false;
  const strokeGeo = new T.BufferGeometry(); strokeGeo.setAttribute("position", new T.BufferAttribute(new Float32Array(3 * 2400), 3));
  const stroke = add(new T.Line(strokeGeo, new T.LineBasicMaterial({ color: 0xf59145 }))); stroke.frustumCulled = false; stroke.visible = false;
  const rayGeo = new T.BufferGeometry(); rayGeo.setAttribute("position", new T.BufferAttribute(new Float32Array(12), 3));
  const rays = add(new T.LineSegments(rayGeo, new T.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.6 }))); rays.frustumCulled = false;
  const kf = add(new T.Mesh(new T.SphereGeometry(0.014, 16, 12), new T.MeshBasicMaterial({ color: 0x5c77df, transparent: true, opacity: 0.8 }))); kf.visible = false;
  const calib = add(new T.Group(), rig);                       // XR-space markers (the corners you touched)
  const scan = add(new T.Group());
  const menu = makePanel(1024, 600, 0.24); add(menu.mesh);
  const status = makePanel(1024, 400, 0.46); add(status.mesh);
  return { all, hands, tips, stroke, rays, kf, calib, scan, menu, status, scanKey: "" };
}
const _m = () => new T.Matrix4();
let COL_HOT = null, COL_IDLE = null;
function placeFacing(mesh, p, look) {
  mesh.position.set(p[0], p[1], p[2]);
  mesh.lookAt(look[0], look[1], look[2]);
  mesh.updateMatrixWorld(true);
}
function tableFrame() {
  const t = V.TABLE, across = prefs.side !== "behind";
  const near = across ? t.x1 : t.x0, inward = across ? -1 : 1, right = across ? 1 : -1;   // y sign of the viewer's right
  return { t, near, inward, right };
}
function headSim() { return head ? X.xrToSim(al, head.p) : [V.TABLE.x1 + 0.6, 0, V.TABLE.z + 0.6]; }
function placePanels(inputs) {
  const f = tableFrame(), hs = headSim();
  // status: by the near edge on your right, a little above the table
  placeFacing(vis.status.mesh, [f.near + f.inward * 0.06, f.right * (Math.max(Math.abs(f.t.y0), Math.abs(f.t.y1)) + 0.18), f.t.z + 0.36], hs);
  // menu: by your left palm when it faces you (hands), on the left controller, else on your left by the near edge
  const L = inputs.left;
  let spot = null;
  if (L && L.tracked && L.kind === "hand" && L.wqXR && L.wristXR && head) {
    const n = X.qrot(L.wqXR, [0, -1, 0]), toHead = X.sub(head.p, L.wristXR), d = X.len(toHead) || 1;
    if (X.dot(n, X.scale(toHead, 1 / d)) > 0.55) {
      spot = X.xrToSim(al, X.add(X.add(L.wristXR, X.scale(n, 0.05)), [0, 0.11, 0]));
      L.overUI = true;                                         // the hand holding the menu doesn't draw
    }
  } else if (L && L.tracked && L.kind === "controller") spot = X.add(L.palm, [0, 0, 0.12]);
  if (!spot) spot = [f.near + f.inward * 0.04, -f.right * (Math.max(Math.abs(f.t.y0), Math.abs(f.t.y1)) + 0.16), f.t.z + 0.30];
  placeFacing(vis.menu.mesh, spot, hs);
}
const raycaster = { rc: null };
function uiInput(inputs) {
  const panels = [vis.menu, vis.status];
  vis.menu.hover = null;
  const rayPts = [];
  for (const side of ["left", "right"]) {
    const inp = inputs[side];
    if (!inp || !inp.tracked) continue;
    const pn = vis.menu, key = side;
    if (inp.kind === "hand" && inp.tip) {                      // poke with the index fingertip
      const loc = pn.mesh.worldToLocal(new T.Vector3(...inp.tip));
      const inside = Math.abs(loc.x) <= pn.w / 2 + 0.01 && Math.abs(loc.y) <= pn.h / 2 + 0.01;
      if (inside && loc.z > -0.03 && loc.z < 0.06) inp.overUI = true;
      const b = inside ? hitButton(pn, loc.x / pn.w + 0.5, loc.y / pn.h + 0.5) : null;
      if (b) pn.hover = b.id;
      const pr = pn.press[key] || (pn.press[key] = { down: false, t: 0 });
      if (inside && loc.z < 0.012 && loc.z > -0.04 && !pr.down) {
        pr.down = true;
        if (b && !b.dis && performance.now() - pr.t > 350) { pr.t = performance.now(); action(b.id); }
      } else if (!inside || loc.z > 0.025) pr.down = false;
    } else if (inp.kind === "controller" && inp.ray) {          // point the ray, pull the trigger
      if (!raycaster.rc) raycaster.rc = new T.Raycaster();
      const rc = raycaster.rc, dl = X.len(inp.ray.d) || 1;
      rc.set(new T.Vector3(...inp.ray.o), new T.Vector3(...X.scale(inp.ray.d, 1 / dl)));
      const hit = rc.intersectObjects(panels.map((p) => p.mesh), false)[0];
      let end = X.add(inp.ray.o, X.scale(inp.ray.d, 0.6 / dl));
      const pr = vis.menu.press[key + "-t"] || (vis.menu.press[key + "-t"] = { h: X.hysteresis(X.TRIGGER.on, X.TRIGGER.off, false) });
      const edge = pr.h.update(inp.trigger).changed;
      if (hit) {
        end = [hit.point.x, hit.point.y, hit.point.z];
        inp.overUI = true;
        if (hit.object === vis.menu.mesh && hit.uv) {
          const b = hitButton(vis.menu, hit.uv.x, hit.uv.y);
          if (b) { vis.menu.hover = b.id; if (edge === "start" && !b.dis) action(b.id); }
        }
      }
      rayPts.push(inp.ray.o, end);
    }
  }
  const pos = vis.rays.geometry.attributes.position;
  for (let i = 0; i < 4; i++) { const p = rayPts[i] || [0, 0, -10]; pos.setXYZ(i, p[0], p[1], p[2]); }
  pos.needsUpdate = true;
  vis.rays.visible = rayPts.length > 0;
}
function hitButton(pn, u, v) {
  const x = u * pn.W, y = (1 - v) * pn.H;
  return pn.buttons.find((b) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) || null;
}
function drawVisuals(inputs, view) {
  // hands: joints + the pinch point (orange while pinching / driving)
  const m4 = _m();
  let n = 0, k = 0;
  for (const side of ["left", "right"]) {
    const inp = inputs[side];
    if (!inp || !inp.tracked) continue;
    for (const j of inp.joints || []) { if (n >= 60) break; m4.makeTranslation(j[0], j[1], j[2]); vis.hands.setMatrixAt(n++, m4); }
    if (inp.kind === "controller") { m4.makeTranslation(...inp.palm); vis.hands.setMatrixAt(n++, m4); }
    const active = inp.kind === "controller" ? (inp.trigger || 0) > X.TRIGGER.on : inp.pinchDist != null && inp.pinchDist < X.PINCH.on;
    m4.makeTranslation(...inp.p); vis.tips.setMatrixAt(k, m4);
    vis.tips.setColorAt(k++, active ? COL_HOT : COL_IDLE);
  }
  vis.hands.count = n; vis.hands.instanceMatrix.needsUpdate = true;
  vis.tips.count = k; vis.tips.instanceMatrix.needsUpdate = true; if (vis.tips.instanceColor) vis.tips.instanceColor.needsUpdate = true;
  // the stroke being drawn
  const S = view && view.stroke;
  if (S && S.length > 1) {
    const pos = vis.stroke.geometry.attributes.position, m = Math.min(S.length, 2400);
    for (let i = 0; i < m; i++) pos.setXYZ(i, S[i][0], S[i][1], S[i][2]);
    pos.needsUpdate = true; vis.stroke.geometry.setDrawRange(0, m); vis.stroke.visible = true;
  } else vis.stroke.visible = false;
  vis.kf.visible = !!(view && view.kfirst);
  if (view && view.kfirst) vis.kf.position.set(...view.kfirst);
  // calibration corners (XR space, under the rig)
  if (vis.calib.userData.n !== (corners ? 2 : 0) + calibPending.length) {
    disposeChildren(vis.calib);
    for (const c of (corners || []).concat(calibPending)) {
      const s = new T.Mesh(new T.SphereGeometry(0.012, 14, 10), new T.MeshBasicMaterial({ color: 0x1f8f55 })); s.position.set(...c); vis.calib.add(s);
    }
    vis.calib.userData.n = (corners ? 2 : 0) + calibPending.length;
  }
  vis.calib.visible = ix.mode === "calibrate" || performance.now() < toastUntil;
  // room scan preview: what becomes a fixed box (blue), what is only shown (grey), the table (green)
  const sk = roomResult && ix.mode === "scan" ? JSON.stringify([roomResult.boxes, roomResult.table, roomResult.align && roomResult.align.yaw]) : "";
  if (sk !== vis.scanKey) {
    vis.scanKey = sk;
    disposeChildren(vis.scan);
    if (roomResult && ix.mode === "scan") {
      for (const b of roomResult.boxes || []) {
        const box = new T.Mesh(new T.BoxGeometry(Math.max(0.01, b.size[0]), Math.max(0.01, b.size[1]), Math.max(0.01, b.size[2])),
          new T.MeshBasicMaterial({ color: b.used ? 0x5c77df : 0x9aa0ae, transparent: true, opacity: b.used ? 0.35 : 0.12, depthWrite: false }));
        // shown where it really is: under the alignment the result will use
        const cx = roomResult.align && al ? X.xrToSim(al, X.simToXr(roomResult.align, b.c)) : b.c;
        box.position.set(...cx); box.rotation.z = roomResult.align && al ? al.yaw - roomResult.align.yaw : 0; vis.scan.add(box);
      }
    }
  }
  vis.scan.visible = ix.mode === "scan";
  paintMenu(vis.menu);
  paintStatus(vis.status);
}
function disposeChildren(g) { for (const c of [...g.children]) disposeObj(c); }
const calibPending = [];

// MR: the real table and floor are there; ghost the sim's furniture and hide its floor
const mrSaved = new Map();
let mrGeoms = null;
function mrLook(on) {
  const g = V && V.geoms;
  if (!on) {
    for (const [o, s] of mrSaved) { if (o.material) { o.material.opacity = s.opacity; o.material.transparent = s.transparent; o.material.depthWrite = s.depthWrite; } o.visible = s.visible; }
    mrSaved.clear(); mrGeoms = null; return;
  }
  if (!g || g === mrGeoms) return;
  mrSaved.clear(); mrGeoms = g;
  g.traverse((o) => {
    if (o === g) return;
    if (o.userData && o.userData.floor) { mrSaved.set(o, { visible: o.visible }); o.visible = false; return; }
    const gm = o.userData && o.userData.gm;
    if (o.isMesh && gm && !gm.dynamic && o.material) {
      mrSaved.set(o, { opacity: o.material.opacity, transparent: o.material.transparent, depthWrite: o.material.depthWrite, visible: o.visible });
      o.material.transparent = true; o.material.opacity = 0.16; o.material.depthWrite = false;
    }
  });
}

// ------------------------------------------------------------------ interaction events
function onIx(e) {
  const tj = window.PCTraj;
  switch (e.type) {
    case "toast": toast(e.text); break;
    case "preview": if (tj) tj.preview(e.i, e.p); grabbing = e.i; break;
    case "preview-end": if (tj) { if (e.id) { tj.track(e.id); tj.end(e.i); } else tj.cancel(e.i); } grabbing = null; break;
    case "calib-point": calibN = e.n; calibPending.push(e.xr); toast(e.n === 1 ? "Got the left corner. Now the near-right corner." : "Got it."); break;
    case "calibrated": {
      calibPending.length = 0;
      const [sa, sb] = X.calibTargets(V.TABLE, prefs.side);
      try {
        al = X.solveAlignment(e.a, e.b, sa, sb); alSource = "calibrated"; corners = [e.a, e.b]; calibN = 0; lastZ = V.TABLE.z;
        saveMR(); makeAnchor(e.a);
        toast(`Calibrated. Your table's edge is ${Math.round(al.width_xr * 100)} cm (the robot's is ${Math.round(al.width_sim * 100)} cm), centred.`);
        ix.mode = "draw";
      } catch (err) { calibN = 0; toast(err.message); ix.mode = "calibrate"; }
      break;
    }
    case "teleop": toast(e.on ? `Driving my ${e.role} hand.` : "Let go: my hand holds still."); break;
  }
}

function onFrame(time, frame) {
  if (!frame || !session || !vis) return;
  curFrame = frame;
  try {
    const ref = V.renderer.xr.getReferenceSpace();
    const pose = ref ? frame.getViewerPose(ref) : null;
    if (pose) head = { p: X.fromXRPoint(pose.transform.position), q: X.fromXRQuat(pose.transform.orientation) };
    if (head && (needRecenter || !al)) recenter();
    if (!al) al = { yaw: 0, t: [V.TABLE.x1 + 0.6, 0, 0] };
    trackAnchor(frame, ref);
    if (lastZ !== V.TABLE.z && al && al.tableY != null) { al = X.withSurface(al, V.TABLE.z); lastZ = V.TABLE.z; }
    if (kind === "ar") mrLook(true);                            // the world may have been rebuilt
    rig.matrix.fromArray(X.alignMatrix(al)); rig.matrixWorldNeedsUpdate = true;
    const inputs = readInputs(frame, ref);
    placePanels(inputs);
    uiInput(inputs);
    ix.setState(st);
    const view = ix.update(performance.now(), inputs);
    grabView = view.grabs && view.grabs[0] || null;
    if (ix.mode === "scan" && performance.now() - lastScan > 500) { lastScan = performance.now(); scanRoom(frame, ref); }
    drawVisuals(inputs, view);
    info.frames++;
  } catch (e) {
    if (info.errors.length < 20) info.errors.push(String((e && e.stack) || e));
    console.error("[xr]", e);
  }
  V.renderer.render(V.scene, xrCam);
}

// ------------------------------------------------------------------ server state: replies become toasts
function onState(s) {
  st = s || {};
  if (ix) ix.setState(st);
  const r = (st.traj || {}).reply;
  if (r && r.id && String(r.id).startsWith("vr-") && r.id !== seen.traj) {
    seen.traj = r.id;
    if (!r.ok && r.message) toast(r.message);
    else if (r.op === "propose" && r.message) toast(r.message);
  }
  const tr = (st.teleop || {}).reply;
  if (tr && tr.id && String(tr.id).startsWith("vr-") && tr.id !== seen.tele) { seen.tele = tr.id; if (!tr.ok) toast(tr.message); }
  const t = st.task;
  if (roomId && t && t.id === roomId && t.status !== "planning" && seen.task !== `${t.id}:${t.status}`) {
    seen.task = `${t.id}:${t.status}`;
    toast(t.status === "done" ? "Your room is the scene now." : (t.message || "I couldn't build your room."));
  }
}

// ------------------------------------------------------------------ boot
function boot(v) {
  V = v; T = v.THREE;
  if (!V.renderer) return;
  V.renderer.xr.addEventListener("sessionend", () => { if (V && V.renderer) V.renderer.setAnimationLoop(null); });
  if (P()) P().onState(onState);
  detect();
  if (navigator.xr && typeof navigator.xr.addEventListener === "function") navigator.xr.addEventListener("devicechange", detect);
}
function waitFor3D() {
  const v = window.PC3D;
  if (v && v.mounted && v.renderer) { boot(v); return; }
  setTimeout(waitFor3D, 250);
}
waitFor3D();

// tests and the console: window.PCXR (docs/vr.md, scripts/checks/xr.py)
window.PCXR = {
  core: X,
  get supported() { return supported; }, detect, enter, exit: () => (session ? session.end() : Promise.resolve()),
  get active() { return !!session; }, get kind() { return kind; }, get refType() { return refType; },
  get mode() { return ix ? ix.mode : null; }, setMode,
  get align() { return al; }, get alSource() { return alSource; }, set align(a) { al = a; alSource = "test"; needRecenter = false; },
  simToXr: (p) => X.simToXr(al, p), xrToSim: (p) => X.xrToSim(al, p),
  get info() { return info; }, get room() { return roomResult; }, get roomRaw() { return roomRaw; }, useRoom, press: action,
  get menu() { return vis ? vis.menu.buttons.map((b) => b.id) : []; }, get status() { return statusLines(); }, get prefs() { return prefs; },
};
