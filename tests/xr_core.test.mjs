// Unit tests for web/xr-core.js (the pure part of VR / MR authoring). Run: node --test tests/
import test from "node:test";
import assert from "node:assert/strict";
import * as X from "../web/xr-core.js";

const TABLE = { z: 0.74, x0: 0.22, x1: 0.82, y0: -0.5, y1: 0.5 };
const near = (a, b, eps = 1e-6) => assert.ok(Math.abs(a - b) <= eps, `${a} != ${b} (±${eps})`);
const near3 = (a, b, eps = 1e-6) => { for (let k = 0; k < 3; k++) near(a[k], b[k], eps); };
const nearQ = (a, b, eps = 1e-6) => assert.ok(X.qangle(X.qnorm(a), X.qnorm(b)) <= eps, `quat ${a} vs ${b}`);
let seed = 7;
const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
const rv = (s = 1) => [(rnd() - 0.5) * 2 * s, (rnd() - 0.5) * 2 * s, (rnd() - 0.5) * 2 * s];
const rq = () => X.qnorm([rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, rnd() - 0.5]);

test("Q0 is the same rotation as R0 (XR y-up -> sim z-up)", () => {
  for (let i = 0; i < 20; i++) {
    const v = rv();
    const m = X.R0.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
    near3(X.qrot(X.Q0, v), m, 1e-9);
  }
  near3(X.xrToSim({ yaw: 0, t: [0, 0, 0] }, [0, 0, -1]), [1, 0, 0]);    // XR ahead -> sim +x
  near3(X.xrToSim({ yaw: 0, t: [0, 0, 0] }, [-1, 0, 0]), [0, 1, 0]);    // XR left -> sim +y
  near3(X.xrToSim({ yaw: 0, t: [0, 0, 0] }, [0, 1, 0]), [0, 0, 1]);     // XR up -> sim +z
});

test("alignment: points and orientations round-trip, the matrix agrees", () => {
  for (let i = 0; i < 20; i++) {
    const al = { yaw: (rnd() - 0.5) * 6, t: rv(2) };
    const p = rv(3), q = rq();
    near3(X.simToXr(al, X.xrToSim(al, p)), p, 1e-9);
    nearQ(X.quatSimToXr(al, X.quatXrToSim(al, q)), q, 1e-6);
    const M = X.alignMatrix(al), s = X.xrToSim(al, p);
    const m = [0, 1, 2].map((r) => M[r] * p[0] + M[4 + r] * p[1] + M[8 + r] * p[2] + M[12 + r]);
    near3(m, s, 1e-9);
    // an orientation maps like the frame: rotating a vector then mapping == mapping then rotating with the mapped quat
    const v = rv(), A = X.qrot(X.quatXrToSim(al, q), v);
    const B = X.sub(X.xrToSim(al, X.qrot(q, v)), X.xrToSim(al, [0, 0, 0]));
    near3(A, B, 1e-9);
  }
});

test("two-point calibration recovers yaw, translation and height", () => {
  for (const side of ["across", "behind"]) {
    for (let i = 0; i < 10; i++) {
      const al = { yaw: (rnd() - 0.5) * 6, t: [rv(1)[0], rv(1)[1], 0.74 - (0.6 + rnd() * 0.5)] };
      const [sa, sb] = X.calibTargets(TABLE, side);
      const a = X.simToXr(al, sa), b = X.simToXr(al, sb);
      const got = X.solveAlignment(a, b, sa, sb);
      near(X.wrapPi(got.yaw - al.yaw), 0, 1e-9);
      near3(got.t, al.t, 1e-9);
      near(got.width_xr, 1.0, 1e-3);
    }
  }
  assert.throws(() => X.solveAlignment([0, 0.7, -0.4], [0.01, 0.7, -0.4], [0.82, -0.5, 0.74], [0.82, 0.5, 0.74]));
});

test("calibration sides: across puts you beyond the far edge facing the robot; behind puts you behind it", () => {
  // you stand at the XR origin looking -z; your table's near edge is 40 cm ahead, 1.2 m wide, 76 cm high
  const L = [-0.6, 0.76, -0.4], R = [0.6, 0.76, -0.4];
  const across = X.solveAlignment(L, R, ...X.calibTargets(TABLE, "across"));
  const you = X.xrToSim(across, [0, 1.6, 0]), ahead = X.sub(X.xrToSim(across, [0, 1.6, -1]), you);
  assert.ok(you[0] > TABLE.x1 + 0.3, `you at x=${you[0]}`);
  near(you[1], 0, 1e-9); near(ahead[0], -1, 1e-9);                 // facing the robot (-x)
  near(X.xrToSim(across, L)[2], TABLE.z, 1e-9);                     // the real table top is the sim surface
  near(X.xrToSim(across, L)[1], -0.6, 1e-9);                        // your left = the robot's right (-y), centred
  const behind = X.solveAlignment(L, R, ...X.calibTargets(TABLE, "behind"));
  const you2 = X.xrToSim(behind, [0, 1.6, 0]);
  assert.ok(you2[0] < TABLE.x0, `you at x=${you2[0]}`);
  near(X.sub(X.xrToSim(behind, [0, 1.6, -1]), you2)[0], 1, 1e-9);  // facing +x like the robot
});

test("VR recentre: the surface lands in front of you at a comfortable height", () => {
  const yaw = 0.5, head = { p: [0.3, 1.65, 0.2], q: X.qaxis([0, 1, 0], yaw) };
  const al = X.vrAlignment(head, TABLE, "across");
  const edge = X.simToXr(al, [TABLE.x1, 0, TABLE.z]);                // your near edge's middle
  const f = [-Math.sin(yaw), 0, -Math.cos(yaw)];
  const d = [edge[0] - head.p[0], 0, edge[2] - head.p[2]];
  near(X.len(d), 0.32, 1e-9);
  near(X.dot(d, f) / X.len(d), 1, 1e-9);                             // straight ahead
  near(edge[1], 1.65 - 0.62, 1e-9);
  const seated = X.vrAlignment({ p: [0, 1.1, 0], q: [1, 0, 0, 0] }, TABLE);
  near(X.simToXr(seated, [TABLE.x1, 0, TABLE.z])[1], 0.55, 1e-9);   // clamped to a sensible minimum
  near(X.yawOfXR(head.q), yaw, 1e-9);
});

test("switching sides keeps your table edge and puts the robot on the other side", () => {
  const L = [-0.6, 0.76, -0.4], R = [0.6, 0.76, -0.4];
  const across = X.solveAlignment(L, R, ...X.calibTargets(TABLE, "across"));
  const behind = X.switchSide(across, TABLE, "across", "behind");
  near3(X.simToXr(behind, [TABLE.x0, 0, TABLE.z]), [0, 0.76, -0.4], 1e-9);       // the robot's near edge is now your edge
  near3(X.simToXr(across, [TABLE.x1, 0, TABLE.z]), [0, 0.76, -0.4], 1e-9);
  assert.ok(X.xrToSim(behind, [0, 1.6, 0])[0] < TABLE.x0);
  const back = X.switchSide(behind, TABLE, "behind", "across");
  near(X.wrapPi(back.yaw - across.yaw), 0, 1e-9); near3(back.t, across.t, 1e-9);
});

test("withSurface and rebaseAlignment keep the sim where it was in the room", () => {
  const al = X.solveAlignment([-0.6, 0.76, -0.4], [0.6, 0.76, -0.4], ...X.calibTargets(TABLE, "across"));
  const k = X.withSurface(al, 0.90);
  near(X.xrToSim(k, [0, 0.76, -0.4])[2], 0.90, 1e-9);
  // the reference space is recentred: new = Ry(phi) (old - c) ; an anchor at A tells us how
  const phi = 0.7, c = [0.4, 0, -0.3];
  const toNew = (x) => { const v = X.sub(x, c); return X.qrot(X.qaxis([0, 1, 0], phi), v); };
  const A = [0.2, 0.8, -0.5];
  const then = { p: A, yaw: 0 }, now = { p: toNew(A), yaw: phi };
  const rb = X.rebaseAlignment(al, then, now);
  for (let i = 0; i < 10; i++) {
    const x = rv(2);
    near3(X.xrToSim(rb, toNew(x)), X.xrToSim(al, x), 1e-9);
  }
});

test("pinch / trigger / fist hysteresis", () => {
  const h = X.hysteresis(X.PINCH.on, X.PINCH.off);
  const seq = [0.05, 0.02, 0.016, 0.022, 0.028, 0.031, 0.02, null].map((v) => h.update(v).changed);
  assert.deepEqual(seq, [null, null, "start", null, null, "end", null, null]);
  const t = X.hysteresis(X.TRIGGER.on, X.TRIGGER.off, false);
  assert.deepEqual([0.1, 0.7, 0.5, 0.3, 0.9].map((v) => t.update(v).changed), [null, "start", null, "end", "start"]);
});

// a synthetic right hand (XR): wrist at w, fingers along -z; curled fingers fold back toward the wrist
function hand(w, { pinch = 0.06, fist = false } = {}) {
  const J = { wrist: w, "middle-finger-metacarpal": X.add(w, [0, 0, -0.045]), "middle-finger-phalanx-proximal": X.add(w, [0, 0, -0.095]) };
  const tip = (dx) => (fist ? X.add(w, [dx, -0.03, -0.06]) : X.add(w, [dx, 0, -0.185]));
  J["middle-finger-tip"] = tip(0); J["ring-finger-tip"] = tip(-0.02); J["pinky-finger-tip"] = tip(-0.04);
  J["index-finger-tip"] = X.add(w, [0.02, 0, -0.17]);
  J["thumb-tip"] = X.add(J["index-finger-tip"], [pinch, 0, 0]);
  return J;
}

test("hand metrics: pinch distance, the pinch point and a fist", () => {
  const open = X.handMetrics(hand([0, 1, -0.3]));
  near(open.pinchDist, 0.06, 1e-9);
  assert.ok(open.curl > X.CURL.off, `open curl ${open.curl}`);
  const fist = X.handMetrics(hand([0, 1, -0.3], { fist: true, pinch: 0.01 }));
  assert.ok(fist.curl < X.CURL.on, `fist curl ${fist.curl}`);
  near3(fist.pinchXR, X.add([0, 1, -0.3], [0.025, 0, -0.17]), 1e-9);
});

test("strokes are smoothed, resampled every 1.5 cm, capped at 60 points; speed from the hand", () => {
  const S = [];
  for (let i = 0; i <= 120; i++) {           // a 30 cm arc drawn in 1.33 s (90 Hz) with 2 mm jitter
    const a = (i / 120) * Math.PI / 2, j = (rnd() - 0.5) * 0.004;
    S.push({ p: [0.5 + 0.19 * Math.cos(a) + j, 0.19 * Math.sin(a) + j, 0.9], t: i * 11.1 });
  }
  const s = X.buildStroke(S);
  assert.ok(s.ok && s.points.length >= 15 && s.points.length <= 25, `${s.points.length} points`);
  near3(s.points[0].p, S[0].p, 2e-3);
  near3(s.points[s.points.length - 1].p, S[S.length - 1].p, 2e-3);
  for (let i = 1; i < s.points.length; i++) near(X.dist(s.points[i - 1].p, s.points[i].p), 0.015, 0.004);
  assert.ok(s.points.every((p) => p.speed === undefined));
  const long = []; for (let i = 0; i <= 400; i++) long.push({ p: [0.3 + i * 0.003, 0, 0.9], t: i * 30 });
  const L = X.buildStroke(long, { speedFrom: "hand" });
  assert.equal(L.points.length, 60);
  assert.ok(L.points.every((p) => Math.abs(p.speed - 0.1) < 0.01), "drawn at 10 cm/s");
  const P = X.buildStroke(long.map((x) => Object.assign({ amount: 1 }, x)), { speedFrom: "pinch" });
  assert.ok(P.points.every((p) => p.speed === 0.3));
  assert.equal(X.buildStroke([{ p: [0, 0, 1], t: 0 }, { p: [0.01, 0, 1], t: 100 }]).ok, false);
});

test("handles and keyframe insertion", () => {
  const K = [[0.4, -0.2, 0.9], [0.5, 0, 0.95], [0.4, 0.2, 0.9]];
  assert.equal(X.nearestHandle(K, [0.51, 0.01, 0.95]), 1);
  assert.equal(X.nearestHandle(K, [0.6, 0.3, 1.2]), -1);
  assert.equal(X.insertIndex(K, [0.46, -0.1, 0.93]), 0);
  assert.equal(X.insertIndex(K, [0.46, 0.1, 0.93]), 1);
  assert.equal(X.insertIndex(K, [0.35, 0.35, 0.9]), 2);
  assert.equal(X.insertIndex(K, [0.35, -0.35, 0.9]), -1);
  assert.equal(X.roleFor([{ p: [0.4, 0.3, 1] }], ["right", "left"]), "left");
  assert.equal(X.roleFor([{ p: [0.4, 0.3, 1] }], ["right"]), "right");
  assert.equal(X.teleopRole("right", "across", ["right", "left"]), "left");
  assert.equal(X.teleopRole("right", "behind", ["right", "left"]), "right");
});

// ------------------------------------------------------------------ the input -> message pipeline
function rig() {
  const sent = [], events = [];
  let now = 0;
  const ix = X.createInteraction({ send: (m) => { sent.push(m); return true; }, now: () => now });
  ix.on((e) => events.push(e));
  const frame = (side, inp) => { now += 11; return ix.update(now, { [side]: Object.assign({ side, kind: "hand", tracked: true }, inp) }); };
  return { ix, sent, events, frame, get now() { return now; } };
}

test("pipeline: a pinch stroke becomes a traj propose with world-frame points", () => {
  const r = rig();
  r.frame("right", { p: [0.4, -0.2, 0.9], pxr: [0, 1, -0.4], pinchDist: 0.06 });
  for (let i = 0; i <= 40; i++) r.frame("right", { p: [0.4, -0.2 + i * 0.01, 0.9 + 0.05 * Math.sin(i / 8)], pinchDist: 0.01 });
  assert.equal(r.sent.length, 0, "nothing is sent while drawing");
  const v = r.ix.view;
  assert.ok(v.stroke && v.stroke.length > 20, "the stroke is shown while drawing");
  r.frame("right", { p: [0.4, 0.2, 0.9], pinchDist: 0.05 });
  assert.equal(r.sent.length, 1);
  const m = r.sent[0];
  assert.equal(m.type, "traj"); assert.equal(m.op, "propose");
  assert.equal(m.traj.source, "vr"); assert.equal(m.traj.frame, "world");
  assert.ok(m.traj.points.length >= 20 && m.traj.points.length <= 60);
  near3(m.traj.points[0].p, [0.4, -0.2, 0.9], 0.003);
  assert.ok(m.traj.points.every((p) => p.p.every(Number.isFinite)));
});

test("pipeline: grab a handle, move it 5 cm and twist the wrist -> one op:edit with p and q", () => {
  const r = rig();
  const keys = [[0.4, -0.2, 0.9], [0.5, 0, 0.95], [0.4, 0.2, 0.9]];
  r.ix.setState({ traj: { proposed: { id: "t1", points: keys.map((p) => ({ p })) }, ghost: { keys, kq: keys.map(() => [0, 1, 0, 0]) } } });
  const q0 = [1, 0, 0, 0];
  r.frame("right", { p: [0.51, 0, 0.95], q: q0, pinchDist: 0.05 });
  r.frame("right", { p: [0.51, 0, 0.95], q: q0, pinchDist: 0.01 });
  for (let i = 1; i <= 10; i++) r.frame("right", { p: [0.51, 0.005 * i, 0.95], q: X.qaxis([0, 0, 1], (30 * Math.PI / 180) * i / 10), pinchDist: 0.01 });
  assert.ok(r.events.some((e) => e.type === "preview" && e.i === 1));
  r.frame("right", { p: [0.51, 0.05, 0.95], q: X.qaxis([0, 0, 1], 30 * Math.PI / 180), pinchDist: 0.05 });
  assert.equal(r.sent.length, 1);
  const m = r.sent[0];
  assert.equal(m.op, "edit"); assert.equal(m.index, 1); assert.equal(m.world, true);
  near3(m.point.p, [0.5, 0.05, 0.95], 1e-3);
  nearQ(m.point.q, X.qmul(X.qaxis([0, 0, 1], 30 * Math.PI / 180), [0, 1, 0, 0]), 1e-3);
  // a small grab (no move, no twist) sends nothing
  r.frame("right", { p: [0.4, 0.2, 0.9], q: q0, pinchDist: 0.01 });
  r.frame("right", { p: [0.4, 0.2, 0.9], q: q0, pinchDist: 0.05 });
  assert.equal(r.sent.length, 1);
});

test("pipeline: a short pinch-tap drops a keyframe (two connect into a path, then inserts); undo", () => {
  const r = rig();
  const tap = (p) => { r.frame("right", { p, pinchDist: 0.05 }); r.frame("right", { p, pinchDist: 0.01 }); r.frame("right", { p, pinchDist: 0.05 }); };
  tap([0.4, -0.2, 0.9]);
  assert.equal(r.sent.length, 0);
  tap([0.5, 0.1, 0.95]);
  assert.equal(r.sent[0].op, "propose"); assert.equal(r.sent[0].traj.source, "keyframes"); assert.equal(r.sent[0].traj.points.length, 2);
  const keys = [[0.4, -0.2, 0.9], [0.5, 0.1, 0.95]];
  const tr = { id: "t2", source: "keyframes", points: keys.map((p) => ({ p })), check: { ok: [true, true] } };
  r.ix.setState({ traj: { proposed: tr, ghost: { keys, kq: [[1, 0, 0, 0], [1, 0, 0, 0]] } } });
  tap([0.6, 0.3, 0.95]);
  assert.deepEqual(r.sent[1].insert, { after: 1, p: [0.6, 0.3, 0.95] });
  assert.ok(r.ix.undo());
  assert.equal(r.sent[2].op, "propose"); assert.equal(r.sent[2].traj.points.length, 2); assert.equal(r.sent[2].traj.check, undefined);
  r.ix.go(); assert.equal(r.sent[3].op, "run");
  r.ix.clear(); assert.equal(r.sent[4].op, "clear");
});

test("pipeline: a long still pinch is still a tap; Keys mode drops a keyframe at every release", () => {
  const r = rig();
  r.frame("right", { p: [0.4, -0.2, 0.9], pinchDist: 0.05 });
  for (let i = 0; i < 60; i++) r.frame("right", { p: [0.4, -0.2 + (i % 2) * 0.002, 0.9], pinchDist: 0.01 });   // held still for 0.66 s
  r.frame("right", { p: [0.4, -0.2, 0.9], pinchDist: 0.05 });
  assert.equal(r.sent.length, 0);
  assert.ok(r.events.some((e) => e.type === "toast" && /Keyframe 1/.test(e.text)));
  r.ix.mode = "keyframe";
  r.frame("right", { p: [0.4, 0, 0.9], pinchDist: 0.01 });
  for (let i = 0; i < 10; i++) { r.frame("right", { p: [0.4, i * 0.01, 0.9], pinchDist: 0.01 }); assert.equal(r.ix.view.stroke, null); }
  r.frame("right", { p: [0.4, 0.1, 0.9], pinchDist: 0.01 });
  r.frame("right", { p: [0.4, 0.1, 0.9], pinchDist: 0.05 });
  r.frame("right", { p: [0.5, 0.1, 0.9], pinchDist: 0.01 });
  r.frame("right", { p: [0.5, 0.1, 0.9], pinchDist: 0.05 });
  assert.equal(r.sent.length, 1);
  assert.deepEqual(r.sent[0].traj.points.map((q) => q.p), [[0.4, 0.1, 0.9], [0.5, 0.1, 0.9]]);   // at the release points
});

test("pipeline: UI over-hover suppresses drawing; lost tracking ends a stroke", () => {
  const r = rig();
  r.frame("right", { p: [0.4, 0, 0.9], pinchDist: 0.05, overUI: true });
  r.frame("right", { p: [0.4, 0, 0.9], pinchDist: 0.01, overUI: true });
  for (let i = 0; i < 10; i++) r.frame("right", { p: [0.4, i * 0.01, 0.9], pinchDist: 0.01 });
  r.frame("right", { p: [0.4, 0.1, 0.9], pinchDist: 0.05 });
  assert.equal(r.sent.length, 0);
  r.frame("right", { p: [0.4, 0, 0.9], pinchDist: 0.01 });
  for (let i = 0; i < 20; i++) r.frame("right", { p: [0.4, i * 0.01, 0.9], pinchDist: 0.01 });
  r.ix.update(r.now + 11, {});                                  // the hand left the view
  assert.equal(r.sent.length, 1); assert.equal(r.sent[0].traj.source, "vr");
});

test("pipeline: teleop — a fist clutches, commands follow the hand at <= 30 Hz, pinch closes the gripper", () => {
  const r = rig();
  r.ix.setState({ teleop: { hands: { right: { p: [0.45, -0.2, 0.95], q: [1, 0, 0, 0] } } } });
  r.ix.mode = "teleop";
  const w = (dx, fist, pinch) => ({ p: [0.3, 0, 1], palm: [0.3 + dx, 0, 1], q: X.qaxis([0, 0, 1], dx), curl: fist ? 0.8 : 1.9, pinchDist: pinch ? 0.01 : 0.06 });
  r.frame("right", w(0, false, false));
  r.frame("right", w(0, true, false));
  assert.equal(r.sent[0].op, "start"); assert.equal(r.sent[0].role, "right");
  for (let i = 1; i <= 30; i++) r.frame("right", w(i * 0.002, true, i > 20));
  const cmds = r.sent.filter((m) => m.type === "teleop" && !m.op);
  assert.ok(cmds.length >= 8 && cmds.length <= 12, `${cmds.length} commands in 330 ms`);
  const last = cmds[cmds.length - 1];
  near3(last.p, [0.45 + 0.06, -0.2, 0.95], 0.006);         // the last frame may fall between two 30 Hz sends
  assert.equal(last.grip, 1); assert.equal(cmds[0].grip, 0);
  nearQ(last.q, X.qaxis([0, 0, 1], 0.06), 0.01);
  r.ix.mode = "draw";
  assert.equal(r.sent[r.sent.length - 1].op, "stop");
});

test("pipeline: calibration — two pinch-taps give the corners (XR)", () => {
  const r = rig();
  r.ix.mode = "calibrate";
  const tap = (pxr) => { r.frame("right", { p: [0, 0, 0], pxr, pinchDist: 0.05 }); r.frame("right", { p: [0, 0, 0], pxr, pinchDist: 0.01 }); r.frame("right", { p: [0, 0, 0], pxr, pinchDist: 0.05 }); };
  tap([-0.6, 0.76, -0.4]); tap([0.6, 0.76, -0.4]);
  const done = r.events.find((e) => e.type === "calibrated");
  assert.deepEqual(done.a, [-0.6, 0.76, -0.4]); assert.deepEqual(done.b, [0.6, 0.76, -0.4]);
  assert.equal(r.sent.length, 0);
});

// ------------------------------------------------------------------ room scan
function room(tableY = 0.76) {
  const rect = (x0, x1, z0, z1, y) => [[x0, y, z0], [x1, y, z0], [x1, y, z1], [x0, y, z1]];
  return {
    viewer: [0, 1.6, 0.3],
    planes: [
      { orientation: "horizontal", label: "floor", polygon: rect(-2, 2, -3, 1, 0) },
      { orientation: "horizontal", label: "table", polygon: rect(-0.6, 0.6, -1.1, -0.4, tableY) },
      { orientation: "vertical", label: "wall", polygon: [[-2, 0, -1.5], [2, 0, -1.5], [2, 2.5, -1.5], [-2, 2.5, -1.5]] },
    ],
    meshes: [
      { label: "screen", points: [[-0.25, tableY, -0.95], [0.25, tableY, -0.95], [0.25, tableY + 0.35, -0.9], [-0.25, tableY + 0.35, -0.9]] },
      { label: "couch", points: [[1.2, 0, 0.2], [2.0, 0, 0.2], [2.0, 0.8, 1.0], [1.2, 0.8, 1.0]] },
      { label: "global mesh", points: [[-3, 0, -3], [3, 2.5, 3]] },
    ],
  };
}

test("room scan -> a world set scene spec: the table is the surface, what stands on it becomes fixed boxes", () => {
  const keep = [{ name: "mug", kind: "mug", color: "red", pos: [0.33, -0.18], movable: true }, { name: "vase", kind: "vase", movable: false }];
  const r = X.roomToScene(room(), null, TABLE, { keep });
  assert.ok(r.ok, r.notes.join(" "));
  assert.equal(r.spec.template, "table");
  const scr = r.spec.objects.find((o) => o.name === "screen");
  assert.ok(scr, JSON.stringify(r.spec));
  assert.equal(scr.kind, "box"); assert.equal(scr.movable, false);
  assert.ok(scr.pos[0] > TABLE.x0 && scr.pos[0] < TABLE.x1 && Math.abs(scr.pos[1]) < 0.3, `screen at ${scr.pos}`);
  assert.ok(scr.size.every((v) => v >= 0.02 && v <= 0.4));
  assert.ok(r.spec.objects.some((o) => o.name === "mug") && !r.spec.objects.some((o) => o.name === "vase"), "movable objects are kept");
  assert.ok(r.boxes.some((b) => b.label === "couch" && !b.used) && r.boxes.some((b) => b.label === "wall" && !b.used));
  assert.ok(!r.boxes.some((b) => b.label === "global mesh"));
  // the derived alignment: your table's nearest edge (z = -0.4) is the sim far edge, its top the sim surface
  near3(X.xrToSim(r.align, [0, 0.76, -0.4]), [TABLE.x1, 0, TABLE.z], 1e-6);
  // a calibrated alignment is used as is (only the height follows the template)
  const al = X.solveAlignment([-0.6, 0.76, -0.4], [0.6, 0.76, -0.4], ...X.calibTargets(TABLE, "across"));
  const r2 = X.roomToScene(room(0.92), al, TABLE, {});
  assert.equal(r2.spec.template, "kitchen");
  near(X.xrToSim(r2.align, [0, 0.76, -0.4])[2], 0.90, 1e-9);
  assert.equal(X.roomToScene({ planes: [], meshes: [] }, null, TABLE).ok, false);
});
