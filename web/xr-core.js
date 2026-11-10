// PhysicsClay XR core: the pure part of VR / MR authoring (no DOM, no three.js), so node can test it
// (tests/xr_core.test.mjs) and web/xr.js only glues it to WebXR and the 3D stage.
//
// Frames. XR reference space: metres, y up, -z ahead of where the session started. Sim world: metres, z up, the robot
// at the origin facing +x, the work surface's near edge at x0 (docs/WORLD.md). An alignment {yaw, t} maps one to the
// other:  sim = Rz(yaw) · R0 · xr + t,  where R0 turns XR axes into z-up ones (ahead -> +x, left -> +y, up -> +z).
// Quaternions are [w, x, y, z] everywhere here (the server's convention); XR's {x, y, z, w} are converted at the edge.
//
// Pipeline (all plain objects, so a test can feed fake joint poses):
//   hand / controller poses -> HandInput {side, kind, p, pxr, palm, q, pinchDist | trigger, curl | squeeze, overUI}
//   -> createInteraction().update(t, hands) -> messages through send(): traj propose / edit / insert, teleop, …
"use strict";

// ------------------------------------------------------------------ small vector / quaternion kit
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const len = (a) => Math.hypot(a[0], a[1], a[2]);
export const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
export const lerp = (a, b, u) => [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const r3 = (v) => Math.round(v * 1000) / 1000;
const r4 = (v) => Math.round(v * 10000) / 10000;
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite3 = (p) => Array.isArray(p) && p.length >= 3 && p.slice(0, 3).every((v) => typeof v === "number" && isFinite(v));
export const wrapPi = (a) => { let x = (a + Math.PI) % (2 * Math.PI); if (x < 0) x += 2 * Math.PI; return x - Math.PI; };

export function qnorm(q) { const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]; }
export function qmul(a, b) {
  return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
}
export const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
export function qaxis(axis, ang) { const s = Math.sin(ang / 2), n = len(axis) || 1; return [Math.cos(ang / 2), axis[0] / n * s, axis[1] / n * s, axis[2] / n * s]; }
export function qrot(q, v) { const p = qmul(qmul(q, [0, v[0], v[1], v[2]]), qconj(q)); return [p[1], p[2], p[3]]; }
export function qangle(a, b) { const d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]); return 2 * Math.acos(Math.min(1, d)); }
export const fromXRQuat = (o) => [o.w, o.x, o.y, o.z];
export const fromXRPoint = (o) => [o.x, o.y, o.z];

// ------------------------------------------------------------------ XR <-> sim
// R0: XR (x right, y up, z back) -> z-up (x ahead, y left, z up):  (x, y, z) -> (-z, -x, y)
export const R0 = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]];
export const Q0 = [0.5, 0.5, -0.5, -0.5];                 // the same rotation as a quaternion
const rot0 = (p) => [-p[2], -p[0], p[1]];
const unrot0 = (s) => [-s[1], s[2], -s[0]];
const rotz = (p, a) => { const c = Math.cos(a), s = Math.sin(a); return [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]; };

export function xrToSim(al, p) { return add(rotz(rot0(p), al.yaw), al.t); }
export function simToXr(al, s) { return unrot0(rotz(sub(s, al.t), -al.yaw)); }
export function quatXrToSim(al, q) { return qnorm(qmul(qmul(qaxis([0, 0, 1], al.yaw), Q0), q)); }
export function quatSimToXr(al, q) { return qnorm(qmul(qconj(qmul(qaxis([0, 0, 1], al.yaw), Q0)), q)); }
/** 4x4 column-major (THREE.Matrix4.fromArray) of the XR -> sim transform: the "rig" holding the XR camera. */
export function alignMatrix(al) {
  const c = Math.cos(al.yaw), s = Math.sin(al.yaw);
  const M = [[c * R0[0][0] - s * R0[1][0], c * R0[0][1] - s * R0[1][1], c * R0[0][2] - s * R0[1][2]],
    [s * R0[0][0] + c * R0[1][0], s * R0[0][1] + c * R0[1][1], s * R0[0][2] + c * R0[1][2]], R0[2]];
  return [M[0][0], M[1][0], M[2][0], 0, M[0][1], M[1][1], M[2][1], 0, M[0][2], M[1][2], M[2][2], 0, al.t[0], al.t[1], al.t[2], 1];
}

/** Sim points the calibration corners stand for. side "across": you face the robot from the far side of the surface
 * (your near edge is its far edge x1); "behind": you stand where the robot is (your near edge is its near edge x0). */
export function calibTargets(table, side = "across") {
  const z = table.z;
  return side === "behind" ? [[table.x0, table.y1, z], [table.x0, table.y0, z]] : [[table.x1, table.y0, z], [table.x1, table.y1, z]];
}

/** Two-point calibration: real near-left / near-right corners (XR) -> the alignment that puts the sim corners on them
 * (yaw + translation; the surface height comes from the touched points). The midpoint is matched, so a real table of
 * a different width is centred. */
export function solveAlignment(xrA, xrB, simA, simB) {
  const a = rot0(xrA), b = rot0(xrB);
  const dx = [b[0] - a[0], b[1] - a[1]], ds = [simB[0] - simA[0], simB[1] - simA[1]];
  const wx = Math.hypot(dx[0], dx[1]), ws = Math.hypot(ds[0], ds[1]);
  if (wx < 0.05) throw new Error("the two corners are too close together (touch both ends of the table's edge)");
  const yaw = wrapPi(Math.atan2(ds[1], ds[0]) - Math.atan2(dx[1], dx[0]));
  const m = rotz([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0], yaw);
  const zs = (simA[2] + simB[2]) / 2, zx = (a[2] + b[2]) / 2;
  const t = [(simA[0] + simB[0]) / 2 - m[0], (simA[1] + simB[1]) / 2 - m[1], zs - zx];
  return { yaw, t, width_xr: r3(wx), width_sim: r3(ws), tableY: r4(zx), tilt: r3(Math.abs(a[2] - b[2])) };
}

/** VR: put the surface in front of the viewer at a comfortable height. head = {p: [x,y,z], q: [w,x,y,z]} (XR). */
export function vrAlignment(head, table, side = "across", opts = {}) {
  let f = qrot(head.q || [1, 0, 0, 0], [0, 0, -1]); f = [f[0], 0, f[2]];
  if (len(f) < 1e-6) f = [0, 0, -1];
  f = scale(f, 1 / len(f));
  const right = cross(f, [0, 1, 0]);
  const y = clamp(opts.height != null ? opts.height : head.p[1] - 0.62, 0.55, 1.15);
  const d = opts.dist != null ? opts.dist : side === "behind" ? table.x0 + 0.15 : 0.32;   // behind: over the robot's shoulder
  const c = add([head.p[0], 0, head.p[2]], scale(f, d)), w = (table.y1 - table.y0) / 2;
  const A = add(c, scale(right, -w)), B = add(c, scale(right, w));
  A[1] = B[1] = y;
  const [sa, sb] = calibTargets(table, side);
  return solveAlignment(A, B, sa, sb);
}

/** Switch which side of the table you are on, keeping the table where it is in your room (the robot moves to the
 * other edge): the edge you calibrated on stays your edge. */
export function switchSide(al, table, from, to) {
  if (from === to) return al;
  const [oa, ob] = calibTargets(table, from), [sa, sb] = calibTargets(table, to);
  return Object.assign({}, al, solveAlignment(simToXr(al, oa), simToXr(al, ob), sa, sb));
}

/** A new surface height (another template) keeps the real table under the sim surface: only t.z changes. */
export function withSurface(al, tableZ) {
  if (!al || al.tableY == null) return al;
  return Object.assign({}, al, { t: [al.t[0], al.t[1], tableZ - al.tableY] });
}

/** The reference space moved (recentre, a new session) and an anchor tells us how: anchor pose then {p, yaw} and now
 * {p, yaw} (yaw about XR +y). -> the alignment that keeps the sim where it was in the room. */
export function rebaseAlignment(al, then, now) {
  // a point x' in the new space is x = Ry(then - now) (x' - now.p) + then.p in the old one, and R0 Ry(a) = Rz(a) R0
  const dyaw = then.yaw - now.yaw;
  const yaw2 = wrapPi(al.yaw + dyaw);
  // sim = Rz(al.yaw) R0 (Ry(dyaw)(x' - a') + a) + t = Rz(al.yaw + dyaw) R0 x' + [Rz(al.yaw) R0 a - Rz(yaw2) R0 a'] + t
  const k1 = rotz(rot0(then.p), al.yaw), k2 = rotz(rot0(now.p), yaw2);
  return Object.assign({}, al, { yaw: yaw2, t: [al.t[0] + k1[0] - k2[0], al.t[1] + k1[1] - k2[1], al.t[2] + k1[2] - k2[2]] });
}
/** yaw (about XR +y) of an XR orientation. */
export function yawOfXR(q) { const f = qrot(q, [0, 0, -1]); return Math.atan2(-f[0], -f[2]); }

// ------------------------------------------------------------------ gesture detectors
/** Threshold with hysteresis. below=true: "on" when the value drops under `on` (pinch distance), off above `off`. */
export function hysteresis(on, off, below = true) {
  let state = false;
  return {
    get on() { return state; },
    reset() { state = false; },
    update(v) {
      if (v == null || !isFinite(v)) { const was = state; state = false; return { on: false, changed: was ? "end" : null }; }
      const was = state;
      state = below ? (state ? v < off : v < on) : (state ? v > off : v > on);
      return { on: state, changed: state === was ? null : state ? "start" : "end" };
    },
  };
}
export const PINCH = { on: 0.017, off: 0.030 };            // index tip <-> thumb tip, metres (joint centres)
export const TRIGGER = { on: 0.6, off: 0.35 };
export const CURL = { on: 1.25, off: 1.45 };               // fingertips-to-wrist / knuckle-to-wrist: a fist is ~0.8, open ~1.9

/** Hand joints (XR space, {name: [x,y,z]}) -> {pinchDist, curl, pinchXR (between the tips), palmXR}. */
export function handMetrics(J) {
  const g = (n) => J[n];
  const it = g("index-finger-tip"), tt = g("thumb-tip"), w = g("wrist"), k = g("middle-finger-phalanx-proximal");
  const out = { pinchDist: null, curl: null, pinchXR: null, palmXR: w || null };
  if (it && tt) { out.pinchDist = dist(it, tt); out.pinchXR = lerp(it, tt, 0.5); }
  const tips = ["middle-finger-tip", "ring-finger-tip", "pinky-finger-tip"].map(g).filter(Boolean);
  if (w && k && tips.length) {
    const ref = dist(k, w);
    if (ref > 1e-3) out.curl = tips.reduce((s, p) => s + dist(p, w), 0) / tips.length / ref;
  }
  const mc = g("middle-finger-metacarpal");
  if (w && mc) out.palmXR = lerp(w, mc, 0.5);
  return out;
}

// ------------------------------------------------------------------ strokes
export function pathLength(P) { let L = 0; for (let i = 1; i < P.length; i++) L += dist(P[i - 1], P[i]); return L; }
/** Moving average (window 2k+1), endpoints kept. */
export function smooth(P, k = 2) {
  if (P.length < 3 || k < 1) return P.map((p) => p.slice());
  return P.map((p, i) => {
    if (i === 0 || i === P.length - 1) return p.slice();
    const a = Math.max(0, i - k), b = Math.min(P.length - 1, i + k), n = Math.min(i - a, b - i);
    let s = [0, 0, 0];
    for (let j = i - n; j <= i + n; j++) s = add(s, P[j]);
    return scale(s, 1 / (2 * n + 1));
  });
}
/** Points every `step` metres along the polyline (first and last kept). Returns [{p, at}] with `at` = arc length. */
export function resample(P, step) {
  if (!P.length) return [];
  const L = pathLength(P);
  if (L < 1e-9) return [{ p: P[0].slice(), at: 0 }];
  const n = Math.max(1, Math.round(L / step));
  const out = [];
  let i = 0, acc = 0;
  for (let k = 0; k <= n; k++) {
    const s = (k / n) * L;
    while (i < P.length - 2 && acc + dist(P[i], P[i + 1]) < s) { acc += dist(P[i], P[i + 1]); i++; }
    const seg = dist(P[i], P[i + 1]) || 1e-9;
    out.push({ p: lerp(P[i], P[i + 1], clamp((s - acc) / seg, 0, 1)), at: s });
  }
  return out;
}
export const STROKE = { step: 0.015, max: 60, minLen: 0.03, minSample: 0.002, smoothK: 2, vmin: 0.02, vmax: 0.4 };
export function pinchSpeed(amount) { return r3(0.05 + 0.25 * clamp(amount, 0, 1)); }   // light pinch / trigger = slow … firm = fast

/** Raw samples [{p (sim), t (ms), amount? 0..1}] -> {ok, points: [{p, speed?}], length, duration, why}.
 * speedFrom: "off" (the robot's default speed) | "hand" (as fast as you drew) | "pinch" (pinch / trigger pressure). */
export function buildStroke(samples, opts = {}) {
  const o = Object.assign({}, STROKE, opts);
  const S = samples.filter((s) => s && finite3(s.p));
  const raw = S.map((s) => s.p);
  const L = pathLength(raw), dur = S.length > 1 ? (S[S.length - 1].t - S[0].t) / 1000 : 0;
  if (S.length < 2 || L < o.minLen) return { ok: false, points: [], length: L, duration: dur, why: "too short" };
  const sm = smooth(raw, o.smoothK);
  let rs = resample(sm, o.step);
  if (rs.length > o.max) rs = resample(sm, pathLength(sm) / (o.max - 1));
  if (rs.length > o.max) rs = rs.slice(0, o.max - 1).concat([rs[rs.length - 1]]);
  // cumulative arc length of the raw samples, to look up time / pressure at each resampled point
  const cum = [0]; for (let i = 1; i < sm.length; i++) cum.push(cum[i - 1] + dist(sm[i - 1], sm[i]));
  const at = (s) => { let i = 0; while (i < cum.length - 2 && cum[i + 1] < s) i++; return i; };
  const pts = rs.map((r) => {
    const pt = { p: r.p.map(r3) };
    if (o.speedFrom === "hand" && S.length > 2) {
      const i = at(r.at), a = Math.max(0, i - 3), b = Math.min(S.length - 1, i + 3), dt = (S[b].t - S[a].t) / 1000;
      if (dt > 0.02) pt.speed = r3(clamp((cum[b] - cum[a]) / dt, o.vmin, o.vmax));
    } else if (o.speedFrom === "pinch") {
      const i = at(r.at), a = S[i] && S[i].amount;
      if (a != null) pt.speed = pinchSpeed(a);
    }
    return pt;
  });
  return { ok: pts.length >= 2, points: pts, length: L, duration: dur, why: pts.length >= 2 ? null : "too short" };
}

// ------------------------------------------------------------------ trajectories (server's ghost, docs/PROTOCOL.md v9)
/** Nearest key point within radius -> index, or -1. */
export function nearestHandle(keys, p, radius = 0.035) {
  let best = -1, bd = radius;
  (keys || []).forEach((k, i) => { if (finite3(k)) { const d = dist(k, p); if (d < bd) { bd = d; best = i; } } });
  return best;
}
/** Where a new keyframe at p belongs: `after` index for {op:"edit", insert:{after, p}} (closest segment; -1 = before
 * the first point, n-1 = after the last). */
export function insertIndex(keys, p) {
  const K = (keys || []).filter(finite3);
  if (!K.length) return -1;
  if (K.length === 1) return 0;
  let best = { d: Infinity, after: K.length - 1 };
  for (let i = 0; i + 1 < K.length; i++) {
    const a = K[i], b = K[i + 1], ab = sub(b, a), L2 = dot(ab, ab) || 1e-12, u = dot(sub(p, a), ab) / L2;
    const d = dist(p, add(a, scale(ab, clamp(u, 0, 1))));
    let after = i;
    if (u < 0 && i === 0) after = -1;
    else if (u > 1 && i === K.length - 2) after = K.length - 1;
    if (d < best.d - 1e-9) best = { d, after };
  }
  return best.after;
}
/** Which robot hand a trajectory drawn around p belongs to: left when it's on the robot's left (y > 0) and it has one. */
export function roleFor(points, roles) {
  const R = roles && roles.length ? roles : ["right"];
  if (!R.includes("left")) return R[0];
  if (!R.includes("right")) return "left";
  const y = points.reduce((s, q) => s + q.p[1], 0) / Math.max(1, points.length);
  return y > 0 ? "left" : "right";
}
/** Your hand -> the robot's hand you drive: across the table you face it, so your right hand drives its left (a mirror,
 * like P3); standing on its side, your right drives its right. One arm: whichever hand clutches drives it. */
export function teleopRole(side, view, roles) {
  const R = roles && roles.length ? roles : ["right"];
  if (R.length < 2) return R[0];
  const want = view === "across" ? (side === "right" ? "left" : "right") : side;
  return R.includes(want) ? want : R[0];
}

// ------------------------------------------------------------------ the interaction (input frames -> messages)
export const MODES = ["draw", "keyframe", "teleop", "scan", "calibrate"];
const TAP_MOVE = 0.015, GRAB_MOVE = 0.004, TWIST_DEG = 10, TELEOP_MS = 33;

/**
 * createInteraction({send, now?, opts?}) -> {mode, setState(s), update(tMs, hands), undo(), go(), clear(), on(fn), view}
 *  send(msg) -> bool: a WebSocket message (PCUI.send). Events through on(fn): {type: "toast", text} | {type: "preview",
 *  i, p} (a handle being moved: draw it there) | {type: "preview-end"} | {type: "calib-point", n, xr} |
 *  {type: "calibrated", a, b} | {type: "teleop", on, role} | {type: "sent", msg}.
 */
export function createInteraction({ send, now, opts } = {}) {
  const clock = now || (() => Date.now());
  const o = Object.assign({ speedFrom: "off", view: "across", grabRadius: 0.035 }, opts || {});
  const listeners = new Set();
  const emit = (e) => { for (const fn of listeners) { try { fn(e); } catch (err) { /* a broken listener must not stop input */ } } };
  let seq = 0;
  const id = () => `vr-${++seq}`;
  const out = (msg) => { const ok = !!send(msg); emit({ type: "sent", msg, ok }); return ok; };
  let state = {}, mode = "draw";
  const undoStack = [];
  const hands = {};
  let kfirst = null;
  const calib = [];
  const tele = { lastSend: 0, active: false, roles: {} };

  const traj = () => (state && state.traj) || {};
  const shown = () => traj().proposed || traj().current || null;
  const ghost = () => traj().ghost || null;
  const keys = () => ((ghost() && ghost().keys) || []);
  const roles = () => Object.keys((state.teleop && state.teleop.hands) || {}).length ? Object.keys(state.teleop.hands) : ["right"];
  const remember = () => { undoStack.push(shown() ? JSON.parse(JSON.stringify(shown())) : null); if (undoStack.length > 20) undoStack.shift(); };

  function H(side) {
    if (!hands[side]) {
      hands[side] = { pinch: hysteresis(PINCH.on, PINCH.off), trig: hysteresis(TRIGGER.on, TRIGGER.off, false),
        fist: hysteresis(CURL.on, CURL.off), squeeze: hysteresis(TRIGGER.on, TRIGGER.off, false), st: "idle", samples: [], grab: null, clutch: null };
    }
    return hands[side];
  }

  function pressed(h, inp) {          // pinch (hand) / trigger (controller), with hysteresis
    return inp.kind === "controller" ? h.trig.update(inp.trigger) : h.pinch.update(inp.pinchDist);
  }
  function gripped(h, inp) {          // a fist (hand) / the grip button (controller)
    return inp.kind === "controller" ? h.squeeze.update(inp.squeeze) : h.fist.update(inp.curl);
  }
  const amount = (inp) => (inp.kind === "controller" ? clamp(((inp.trigger || 0) - TRIGGER.on) / (1 - TRIGGER.on), 0, 1)
    : inp.pinchDist == null ? null : 1 - clamp((inp.pinchDist - 0.004) / 0.022, 0, 1));

  function dropKeyframe(p) {
    const g = ghost(), tr = shown();
    if (tr && g && !traj().running) {
      remember();
      out({ type: "traj", op: "edit", id: id(), world: true, insert: { after: insertIndex(keys(), p), p: p.map(r3) } });
      emit({ type: "toast", text: `Keyframe added (${keys().length + 1} points).` });
      return;
    }
    if (!kfirst) { kfirst = p.map(r3); emit({ type: "toast", text: "Keyframe 1. Pinch another spot to connect them." }); return; }
    remember();
    out({ type: "traj", op: "propose", id: id(), traj: { source: "keyframes", frame: "world", label: "your keyframes (VR)",
      role: roleFor([{ p: kfirst }, { p }], roles()), points: [{ p: kfirst }, { p: p.map(r3) }] } });
    kfirst = null;
  }

  function finishStroke(h) {
    const s = buildStroke(h.samples, { speedFrom: o.speedFrom });
    h.samples = [];
    if (!s.ok) { emit({ type: "toast", text: "That stroke was too short. Pinch and move a few centimetres." }); return null; }
    remember();
    const msg = { type: "traj", op: "propose", id: id(), traj: { source: "vr", frame: "world", label: "drawn in VR",
      role: roleFor(s.points, roles()), points: s.points } };
    out(msg);
    return msg;
  }

  function grabStart(h, inp, i) {
    const g = ghost(), k0 = keys()[i];
    h.grab = { i, k0: k0.slice(), h0: inp.p.slice(), qh0: inp.q ? inp.q.slice() : null,
      kq0: g && g.kq && g.kq[i] ? g.kq[i].slice() : null, k: k0.slice(), q: null, twist: 0 };
    emit({ type: "preview", i, p: h.grab.k, grab: true });
  }
  function grabMove(h, inp) {
    const gb = h.grab;
    gb.k = add(gb.k0, sub(inp.p, gb.h0));
    if (gb.qh0 && inp.q && gb.kq0) {
      const d = qmul(inp.q, qconj(gb.qh0));            // the wrist's turn since the grab, in the world frame
      gb.twist = qangle([1, 0, 0, 0], d) * 180 / Math.PI;
      gb.q = qnorm(qmul(d, gb.kq0));
    }
    emit({ type: "preview", i: gb.i, p: gb.k, q: gb.twist > TWIST_DEG ? gb.q : null });
  }
  function grabEnd(h) {
    const gb = h.grab; h.grab = null;
    const moved = dist(gb.k, gb.k0) > GRAB_MOVE, turned = gb.twist > TWIST_DEG && gb.q;
    if (!moved && !turned) { emit({ type: "preview-end", i: gb.i, id: null }); return null; }
    const point = { p: gb.k.map(r3) };
    if (turned) point.q = gb.q.map(r4);
    remember();
    const msg = { type: "traj", op: "edit", id: id(), world: true, index: gb.i, point };
    out(msg);
    emit({ type: "preview-end", i: gb.i, id: msg.id });     // the preview stays until the server's reply to this edit
    return msg;
  }

  function teleopStep(side, inp, t) {
    const h = H(side), g = gripped(h, inp), anchorP = inp.palm || inp.p;
    if (g.changed === "start") {
      const role = teleopRole(side, o.view, roles()), cur = (state.teleop && state.teleop.hands && state.teleop.hands[role]) || {};
      h.clutch = { role, e0: finite3(cur.p) ? cur.p.slice() : anchorP.slice(), qe0: cur.q || null, h0: anchorP.slice(), qh0: inp.q ? inp.q.slice() : null };
      out({ type: "teleop", op: "start", role, src: "vr", id: id() });
      tele.active = true;
      emit({ type: "teleop", on: true, role });
    }
    if (g.changed === "end" && h.clutch) { emit({ type: "teleop", on: false, role: h.clutch.role }); h.clutch = null; }
    const pr = pressed(h, inp);
    if (h.clutch && t - (tele.roles[h.clutch.role] || 0) >= TELEOP_MS) {
      const c = h.clutch, p = add(c.e0, sub(anchorP, c.h0));
      const msg = { type: "teleop", role: c.role, p: p.map(r4), t: Math.round(t), src: "vr" };
      if (c.qe0 && c.qh0 && inp.q) msg.q = qnorm(qmul(qmul(inp.q, qconj(c.qh0)), c.qe0)).map(r4);
      msg.grip = inp.kind === "controller" ? r3(clamp(inp.trigger || 0, 0, 1)) : pr.on ? 1 : 0;
      tele.roles[c.role] = t;
      out(msg);
    }
  }

  function update(t, input) {
    const view = { stroke: null, grabs: [], clutch: [], calib: calib.slice(), kfirst };
    for (const side of ["left", "right"]) {
      const inp = input && input[side];
      const h = H(side);
      if (!inp || !inp.tracked) {               // lost tracking: end what this hand was doing, cleanly
        if (h.grab) grabEnd(h);
        if (h.st === "drawing") finishStroke(h);
        h.st = "idle"; h.samples = []; h.pinch.reset(); h.trig.reset(); h.fist.reset(); h.squeeze.reset(); h.clutch = null;
        continue;
      }
      if (mode === "teleop") { teleopStep(side, inp, t); if (h.clutch) view.clutch.push({ side, role: h.clutch.role }); continue; }
      const pr = pressed(h, inp);
      if (pr.changed === "start") {
        if (inp.overUI) { h.st = "ui"; continue; }
        const i = (mode === "draw" || mode === "keyframe") && !traj().running ? nearestHandle(keys(), inp.p, o.grabRadius) : -1;
        if (i >= 0) { grabStart(h, inp, i); h.st = "grab"; }
        else { h.st = "press"; h.t0 = t; h.p0 = inp.p.slice(); h.pxr0 = inp.pxr ? inp.pxr.slice() : null; h.samples = [{ p: inp.p.slice(), t, amount: amount(inp) }]; }
      } else if (pr.on) {
        if (h.st === "grab") grabMove(h, inp);
        else if (h.st === "press" || h.st === "drawing") {
          const last = h.samples[h.samples.length - 1];
          if (!last || dist(last.p, inp.p) >= STROKE.minSample) h.samples.push({ p: inp.p.slice(), t, amount: amount(inp) });
          if (h.st === "press" && mode === "draw" && dist(inp.p, h.p0) > TAP_MOVE) h.st = "drawing";
        }
      } else if (pr.changed === "end") {
        if (h.st === "grab") grabEnd(h);
        else if ((h.st === "drawing" || h.st === "press") && mode === "draw" && h.samples.some((x) => dist(x.p, h.p0) > TAP_MOVE)) finishStroke(h);
        else if (h.st === "press" || h.st === "drawing") {      // it didn't travel: a tap, however long you held it
          if (mode === "calibrate") {
            calib.push(h.pxr0 || inp.pxr);
            emit({ type: "calib-point", n: calib.length, xr: calib[calib.length - 1] });
            if (calib.length >= 2) { const [a, b] = calib.splice(0, 2); emit({ type: "calibrated", a, b }); }
          } else if (mode === "draw" || mode === "keyframe") dropKeyframe(mode === "keyframe" ? inp.p : h.p0);
        }
        h.st = "idle"; h.samples = [];
      }
      if (mode === "draw" && (h.st === "drawing" || (h.st === "press" && h.samples.length > 1))) view.stroke = h.samples.map((s) => s.p);
      if (h.grab) view.grabs.push({ side, i: h.grab.i, p: h.grab.k, twist: Math.round(h.grab.twist) });
    }
    api.view = view;
    return view;
  }

  function setMode(m) {
    if (!MODES.includes(m) || m === mode) return;
    if (mode === "teleop" && tele.active) { out({ type: "teleop", op: "stop", id: id() }); tele.active = false; }
    for (const s of Object.keys(hands)) { const h = hands[s]; if (h.grab) grabEnd(h); h.st = "idle"; h.samples = []; h.clutch = null; }
    if (m === "calibrate") calib.length = 0;
    kfirst = null;
    mode = m;
  }
  const api = {
    get mode() { return mode; }, set mode(m) { setMode(m); },
    get opts() { return o; }, view: null,
    setState(s) { state = s || {}; },
    update,
    on(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    go() { return out({ type: "traj", op: "run", id: id() }); },
    clear() { remember(); kfirst = null; return out({ type: "traj", op: "clear", id: id() }); },
    undo() {
      if (!undoStack.length) { emit({ type: "toast", text: "Nothing to undo." }); return false; }
      const prev = undoStack.pop();
      if (!prev) return out({ type: "traj", op: "clear", id: id() });
      const tr = Object.assign({}, prev); delete tr.check; delete tr.id;
      return out({ type: "traj", op: "propose", id: id(), traj: tr });
    },
    get undoDepth() { return undoStack.length; },
    cancelCalibration() { calib.length = 0; },
  };
  return api;
}

// ------------------------------------------------------------------ room scan -> scene spec (docs/WORLD.md)
const slug = (s) => String(s || "").toLowerCase().replace(/[^a-z0-9]+/g, "_").replace(/^_+|_+$/g, "").slice(0, 20);
function bbox(P) {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of P) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); }
  return { lo, hi, c: lerp(lo, hi, 0.5), size: sub(hi, lo) };
}
function polyArea(P) {   // XR polygon on a horizontal plane: area in x-z
  let a = 0; for (let i = 0; i < P.length; i++) { const p = P[i], q = P[(i + 1) % P.length]; a += p[0] * q[2] - q[0] * p[2]; }
  return Math.abs(a) / 2;
}
const TABLE_WORDS = ["table", "desk", "counter", "workbench"];
/** The work surface among the detected planes / meshes (XR): a plane or mesh labelled table / desk, else the largest
 * horizontal plane between 0.4 and 1.3 m above the floor. -> {polygon (XR), y, area, label} | null */
export function findTable(planes, meshes, floorY = 0) {
  const cand = [];
  for (const pl of planes || []) {
    if (!pl || pl.orientation !== "horizontal" || !(pl.polygon && pl.polygon.length >= 3)) continue;
    const y = pl.polygon.reduce((s, p) => s + p[1], 0) / pl.polygon.length, h = y - floorY;
    const named = TABLE_WORDS.includes(String(pl.label || "").toLowerCase());
    if ((named && h > 0.2) || (h > 0.4 && h < 1.3)) cand.push({ polygon: pl.polygon, y, area: polyArea(pl.polygon), label: pl.label || "plane", named });
  }
  for (const m of meshes || []) {
    if (!m || !TABLE_WORDS.includes(String(m.label || "").toLowerCase()) || !(m.points && m.points.length)) continue;
    const b = bbox(m.points);
    const poly = [[b.lo[0], b.hi[1], b.lo[2]], [b.hi[0], b.hi[1], b.lo[2]], [b.hi[0], b.hi[1], b.hi[2]], [b.lo[0], b.hi[1], b.hi[2]]];
    cand.push({ polygon: poly, y: b.hi[1], area: b.size[0] * b.size[2], label: m.label, named: true, mesh: true });
  }
  if (!cand.length) return null;
  cand.sort((a, b) => (b.named - a.named) || (b.area - a.area));
  return cand[0];
}
/** The table edge nearest the viewer, as near-left / near-right corners (XR), for an automatic calibration. */
export function tableEdge(table, viewerXR) {
  const P = table.polygon, n = P.length;
  let best = null;
  for (let i = 0; i < n; i++) {
    const a = P[i], b = P[(i + 1) % n], m = lerp(a, b, 0.5), L = Math.hypot(b[0] - a[0], b[2] - a[2]);
    if (L < 0.2) continue;
    const d = Math.hypot(m[0] - viewerXR[0], m[2] - viewerXR[2]);
    if (!best || d < best.d) best = { a, b, d };
  }
  if (!best) return null;
  // left / right as seen from the viewer: the cross product's sign (XR: y up)
  const to = [best.a[0] - viewerXR[0], 0, best.a[2] - viewerXR[2]], ab = [best.b[0] - best.a[0], 0, best.b[2] - best.a[2]];
  const leftFirst = cross(to, ab)[1] < 0;
  const [L, R] = leftFirst ? [best.a, best.b] : [best.b, best.a];
  return { left: [L[0], table.y, L[2]], right: [R[0], table.y, R[2]] };
}
export const TEMPLATE_TOPS = { table: 0.74, kitchen: 0.90, floor: 0.01 };
export function templateFor(height) { return height < 0.25 ? "floor" : height >= 0.84 ? "kitchen" : "table"; }

/**
 * Room scan -> a `world set` scene spec (docs/WORLD.md): the real table becomes the work surface (template by its
 * height), things standing on it become fixed boxes (movable:false), the current movable objects are kept. Walls,
 * floor and furniture off the surface are previewed but can't be part of the spec (it only describes the surface).
 * room = {planes: [{orientation, label, polygon: [[x,y,z] XR]}], meshes: [{label, points: [[x,y,z] XR]}], viewer: [x,y,z],
 *         floorY?}; align = the current XR->sim alignment (null: derived from the table's nearest edge);
 * table = the sim surface {z, x0, x1, y0, y1}; keep = objects to keep (state.world.scene.objects).
 * -> {ok, spec, align, template, height, boxes: [{label, c (sim), size, used, why?}], notes, table: {...} | null}
 */
export function roomToScene(room, align, table, opts = {}) {
  const notes = [], boxes = [];
  const floors = (room.planes || []).filter((p) => p && p.orientation === "horizontal" && String(p.label || "").toLowerCase() === "floor" && p.polygon && p.polygon.length);
  const floorY = room.floorY != null ? room.floorY : floors.length ? Math.min(...floors.map((p) => p.polygon.reduce((s, q) => s + q[1], 0) / p.polygon.length)) : 0;
  const tb = findTable(room.planes, room.meshes, floorY);
  if (!tb) return { ok: false, spec: null, align, boxes, notes: ["I couldn't find a table in your room scan. Run Space Setup on the Quest (mark your desk), then scan again."], table: null };
  const height = tb.y - floorY, template = opts.template || templateFor(height);
  const top = TEMPLATE_TOPS[template] != null ? TEMPLATE_TOPS[template] : table.z;
  const surf = Object.assign({}, table, { z: top });
  let al = align;
  if (!al) {
    const e = tableEdge(tb, room.viewer || [0, 1.6, 0]);
    if (!e) return { ok: false, spec: null, align, boxes, notes: ["Your table's edges are too short to line the robot up with."], table: tb };
    const [sa, sb] = calibTargets(surf, opts.side || "across");
    al = solveAlignment(e.left, e.right, sa, sb);
    notes.push("Lined the robot's table up with the edge of your table nearest to you.");
  } else al = withSurface(al, top);
  const inside = (c, r = 0) => c[0] > surf.x0 - r && c[0] < surf.x1 + r && c[1] > surf.y0 - r && c[1] < surf.y1 + r;
  const items = [];
  for (const pl of room.planes || []) {
    if (!pl || !pl.polygon || pl.polygon.length < 3 || pl.polygon === tb.polygon) continue;
    items.push({ label: pl.label || (pl.orientation === "vertical" ? "wall" : "surface"), pts: pl.polygon, plane: pl.orientation });
  }
  for (const m of room.meshes || []) {
    if (!m || !m.points || !m.points.length || (tb.mesh && TABLE_WORDS.includes(String(m.label || "").toLowerCase()))) continue;
    if (String(m.label || "").toLowerCase() === "global mesh") continue;
    items.push({ label: m.label || "object", pts: m.points });
  }
  const objects = [];
  const counts = {};
  for (const it of items) {
    const sim = it.pts.map((p) => xrToSim(al, p)), b = bbox(sim);
    let size = b.size.slice(), c = b.c.slice();
    if (it.plane === "horizontal") { size[2] = Math.max(0.02, c[2] - top); c[2] = top + size[2] / 2; }   // a plane on top of something
    const lab = String(it.label).toLowerCase();
    const onTop = inside(c) && b.lo[2] > top - 0.08 && b.lo[2] < top + 0.25 && size[0] < 1.0 && size[1] < 1.0 && lab !== "wall" && lab !== "floor" && lab !== "ceiling";
    const box = { label: it.label, c: c.map(r3), size: size.map(r3), used: onTop };
    if (!onTop) box.why = inside(c) ? "too big / not standing on the table" : "off the table (the scene only describes the work surface)";
    boxes.push(box);
    if (onTop && objects.length < (opts.maxObjects || 10)) {
      const base = slug(it.label) || "box";
      counts[base] = (counts[base] || 0) + 1;
      objects.push({ name: counts[base] > 1 ? `${base}_${counts[base]}` : base, kind: "box", color: "gray",
        size: [clamp(size[0], 0.02, 0.4), clamp(size[1], 0.02, 0.4), clamp(size[2], 0.02, 0.4)].map(r3), pos: [r3(c[0]), r3(c[1])], movable: false });
    }
  }
  const keep = (opts.keep || []).filter((o) => o && o.movable !== false).slice(0, 16 - objects.length);
  const spec = { template, objects: keep.map((o) => Object.assign({}, o)).concat(objects) };
  const used = objects.length, off = boxes.filter((b) => !b.used).length;
  notes.push(`Your table is ${Math.round(height * 100)} cm high: using the ${template === "kitchen" ? "kitchen counter" : template} template.`);
  notes.push(used ? `${used} thing${used === 1 ? "" : "s"} on your table become${used === 1 ? "s" : ""} fixed obstacle${used === 1 ? "" : "s"}.` : "Nothing standing on your table was detected.");
  if (off) notes.push(`${off} other surface${off === 1 ? "" : "s"} (walls, floor, furniture) shown for reference only.`);
  return { ok: true, spec, align: al, template, height: r3(height), boxes, notes, table: { y: tb.y, label: tb.label, area: r3(tb.area) } };
}
