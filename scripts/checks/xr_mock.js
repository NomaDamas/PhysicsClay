// A minimal fake WebXR device for headless Chrome (scripts/checks/xr.py injects it before the page loads).
// Enough of XRSystem / XRSession / XRFrame / XRWebGLLayer / hand input / planes / meshes / anchors for three.js r160's
// WebXRManager and web/xr.js. Not a full emulator: one reference space (every space is posed in it), two eye views,
// frames from setTimeout. Driven from the test through window.__xrmock:
//   __xrmock.supported = {vr, ar}            what isSessionSupported answers
//   __xrmock.head = {p: [x,y,z], q: [x,y,z,w]}
//   __xrmock.script([{right: {at, q, pinch, fist}, left: {...} | null, frames: n}, ...]) -> Promise (one step per n frames)
//   __xrmock.setPlanes([{orientation, semanticLabel, polygon: [[x,y,z]...]}]), __xrmock.setMeshes([{semanticLabel, points}])
(function () {
  "use strict";
  const M = (window.__xrmock = { supported: { vr: true, ar: true }, head: { p: [0, 1.6, 0], q: [0, 0, 0, 1] }, sessions: 0, frames: 0,
    planes: null, meshes: null, granted: null, log: [] });

  // ---------------------------------------------------------------- math
  const qmul = (a, b) => [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];   // [x,y,z,w]
  const qrot = (q, v) => { const p = qmul(qmul(q, [v[0], v[1], v[2], 0]), [-q[0], -q[1], -q[2], q[3]]); return [p[0], p[1], p[2]]; };
  function matrix(p, q) {
    const [x, y, z, w] = q, x2 = x + x, y2 = y + y, z2 = z + z;
    const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w * z2;
    return new Float32Array([1 - (yy + zz), xy + wz, xz - wy, 0, xy - wz, 1 - (xx + zz), yz + wx, 0, xz + wy, yz - wx, 1 - (xx + yy), 0, p[0], p[1], p[2], 1]);
  }
  function invert(p, q) { const qi = [-q[0], -q[1], -q[2], q[3]], pi = qrot(qi, p).map((v) => -v); return { p: pi, q: qi }; }
  function transform(p, q) {
    const inv = invert(p, q);
    return { position: { x: p[0], y: p[1], z: p[2], w: 1 }, orientation: { x: q[0], y: q[1], z: q[2], w: q[3] }, matrix: matrix(p, q),
      inverse: { position: { x: inv.p[0], y: inv.p[1], z: inv.p[2], w: 1 }, orientation: { x: inv.q[0], y: inv.q[1], z: inv.q[2], w: inv.q[3] }, matrix: matrix(inv.p, inv.q) } };
  }
  function perspective(fovY, aspect, near, far) {
    const f = 1 / Math.tan(fovY / 2), nf = 1 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
  }
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const lerp = (a, b, u) => [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];

  // ---------------------------------------------------------------- spaces
  class FakeSpace extends EventTarget {
    constructor(pose) { super(); this._pose = pose || (() => ({ p: [0, 0, 0], q: [0, 0, 0, 1] })); }
    getOffsetReferenceSpace() { return this; }
  }
  class XRRigidTransform {
    constructor(position, orientation) {
      const p = position || { x: 0, y: 0, z: 0 }, o = orientation || { x: 0, y: 0, z: 0, w: 1 };
      Object.assign(this, transform([p.x, p.y, p.z], [o.x, o.y, o.z, o.w]));
    }
  }
  class XRWebGLLayer {
    constructor(session, gl) {
      this.framebuffer = null;                                   // the page's canvas: screenshots show the headset view
      this.framebufferWidth = 960; this.framebufferHeight = 480; this.antialias = false; this.ignoreDepthValues = false;
      this.fixedFoveation = 0; this._gl = gl;
    }
    getViewport(view) { return { x: view.eye === "right" ? 480 : 0, y: 0, width: 480, height: 480 }; }
    static getNativeFramebufferScaleFactor() { return 1; }
  }

  // ---------------------------------------------------------------- hands (25 joints) from {at (pinch point), q, pinch, fist}
  const NAMES = { thumb: ["thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip"] };
  for (const f of ["index", "middle", "ring", "pinky"]) NAMES[f] = ["metacarpal", "phalanx-proximal", "phalanx-intermediate", "phalanx-distal", "tip"].map((k) => `${f}-finger-${k}`);
  function handJoints(h, side) {
    const pinch = h.pinch == null ? 0.06 : h.pinch, q = h.q || [0, 0, 0, 1], s = side === "left" ? -1 : 1;
    const L = { wrist: [0, 0, 0] };                               // local, fingers along -z, palm down (-y), thumb toward the body's centre
    const idxTip = [0.02 * s, 0, -0.17];
    const open = { index: idxTip, middle: [0, 0, -0.185], ring: [-0.02 * s, 0, -0.175], pinky: [-0.04 * s, 0, -0.15] };
    for (const f of ["index", "middle", "ring", "pinky"]) {
      const tip = open[f], n = NAMES[f];
      L[n[0]] = lerp(L.wrist, tip, 0.25); L[n[1]] = lerp(L.wrist, tip, 0.5);
      const ftip = h.fist && f !== "index" ? [tip[0], -0.03, -0.06] : tip;
      L[n[2]] = lerp(L[n[1]], ftip, 0.4); L[n[3]] = lerp(L[n[1]], ftip, 0.7); L[n[4]] = ftip;
    }
    const thumbTip = add(idxTip, [pinch * s, 0, 0]);
    NAMES.thumb.forEach((n, i) => { L[n] = lerp([0.03 * s, 0, -0.03], thumbTip, (i + 1) / 4); });
    const m = lerp(idxTip, thumbTip, 0.5);                       // the pinch point is `at`
    const out = {};
    for (const k in L) out[k] = add(h.at, qrot(q, [L[k][0] - m[0], L[k][1] - m[1], L[k][2] - m[2]]));
    return { joints: out, q };
  }
  class FakeHand {
    constructor(src) { this._src = src; this._joints = new Map(); for (const n of ["wrist"].concat(NAMES.thumb, NAMES.index, NAMES.middle, NAMES.ring, NAMES.pinky)) this._joints.set(n, { jointName: n, _hand: this }); }
    get size() { return this._joints.size; }
    get(n) { return this._joints.get(n); }
    values() { return this._joints.values(); }
    keys() { return this._joints.keys(); }
    entries() { return this._joints.entries(); }
    [Symbol.iterator]() { return this._joints.entries(); }
  }
  function makeSource(side) {
    const src = { handedness: side, targetRayMode: "tracked-pointer", profiles: ["generic-hand"], gamepad: null, _pose: null };
    src.hand = new FakeHand(src);
    src.targetRaySpace = new FakeSpace(() => src._pose && { p: src._pose.joints["index-finger-metacarpal"], q: src._pose.q });
    src.gripSpace = new FakeSpace(() => src._pose && { p: src._pose.joints.wrist, q: src._pose.q });
    return src;
  }

  // ---------------------------------------------------------------- session + frames
  class FakeSession extends EventTarget {
    constructor(mode, init) {
      super();
      this.mode = mode; M.sessions++;
      const want = [].concat((init && init.requiredFeatures) || [], (init && init.optionalFeatures) || []);
      const grantable = M.granted || ["local", "local-floor", "viewer", "hand-tracking", "plane-detection", "mesh-detection", "anchors"];
      this.enabledFeatures = want.filter((f) => grantable.includes(f));
      this.renderState = { baseLayer: null, depthNear: 0.1, depthFar: 1000, inlineVerticalFieldOfView: null };
      this.environmentBlendMode = mode === "immersive-ar" ? "alpha-blend" : "opaque";
      this.visibilityState = "visible"; this.interactionMode = "world-space";
      this.inputSources = [];
      this._sources = { left: makeSource("left"), right: makeSource("right") };
      this._cbs = new Map(); this._id = 0; this._ended = false; this._steps = []; this._anchors = new Set();
      this._ref = new FakeSpace();
      this._tick = setInterval(() => this._frame(), 16);
    }
    updateRenderState(s) { Object.assign(this.renderState, s || {}); }
    requestReferenceSpace(type) {
      if (!["local", "local-floor", "viewer", "bounded-floor", "unbounded"].includes(type) || (type !== "local" && type !== "viewer" && !this.enabledFeatures.includes(type)))
        return Promise.reject(new DOMException(`reference space ${type} not supported`, "NotSupportedError"));
      return Promise.resolve(this._ref);
    }
    requestAnimationFrame(cb) { const id = ++this._id; this._cbs.set(id, cb); return id; }
    cancelAnimationFrame(id) { this._cbs.delete(id); }
    restorePersistentAnchor() { return Promise.reject(new DOMException("no persistent anchors in the mock", "NotSupportedError")); }
    end() {
      if (this._ended) return Promise.resolve();
      this._ended = true; clearInterval(this._tick);
      const ev = new Event("end"); ev.session = this;
      this.dispatchEvent(ev);
      return Promise.resolve();
    }
    _setHands(step) {
      const added = [], removed = [];
      for (const side of ["left", "right"]) {
        const src = this._sources[side], h = step && step[side];
        const was = this.inputSources.includes(src);
        src._pose = h ? handJoints(h, side) : null;
        if (h && !was) { this.inputSources.push(src); added.push(src); }
        if (!h && was) { this.inputSources.splice(this.inputSources.indexOf(src), 1); removed.push(src); }
      }
      if (added.length || removed.length) {
        const ev = new Event("inputsourceschange"); ev.added = added; ev.removed = removed; ev.session = this;
        this.dispatchEvent(ev);
      }
    }
    _frame() {
      if (this._ended) return;
      const st = this._steps[0];
      if (st) {
        if (!st._applied) { this._setHands(st); st._applied = true; }
        if (--st.frames <= 0) { this._steps.shift(); if (st._done) st._done(); }   // resolves after this frame's callbacks
      }
      const cbs = this._cbs; this._cbs = new Map();
      const frame = new FakeFrame(this, performance.now());
      M.frames++;
      for (const cb of cbs.values()) { try { cb(performance.now(), frame); } catch (e) { M.log.push(String(e && e.stack || e)); } }
      frame._active = false;
    }
  }
  class FakeFrame {
    constructor(session) {
      this.session = session; this._active = true;
      this.trackedAnchors = new Set(session._anchors);
      if (session.enabledFeatures.includes("plane-detection") && M.planes) this.detectedPlanes = new Set(M.planes.map(plane));
      if (session.enabledFeatures.includes("mesh-detection") && M.meshes) this.detectedMeshes = new Set(M.meshes.map(mesh));
    }
    getViewerPose() {
      const h = M.head, eyes = [["left", -0.032], ["right", 0.032]];
      const views = eyes.map(([eye, dx]) => ({ eye, projectionMatrix: perspective(Math.PI / 2, 1, 0.02, 60),
        transform: transform(add(h.p, qrot(h.q, [dx, 0, 0])), h.q), recommendedViewportScale: 1, requestViewportScale() {} }));
      return Object.assign({ views, emulatedPosition: false }, { transform: transform(h.p, h.q) });
    }
    getPose(space) {
      const p = space && space._pose && space._pose();
      return p ? { transform: transform(p.p, p.q), emulatedPosition: false } : null;
    }
    getJointPose(joint) {
      const src = joint && joint._hand && joint._hand._src, pose = src && src._pose;
      if (!pose) return null;
      return { transform: transform(pose.joints[joint.jointName], pose.q), radius: joint.jointName.endsWith("tip") ? 0.006 : 0.009 };
    }
    fillJointRadii() { return false; }
    createAnchor(rt) {
      if (!this.session.enabledFeatures.includes("anchors")) return Promise.reject(new DOMException("anchors not enabled", "NotSupportedError"));
      const p = [rt.position.x, rt.position.y, rt.position.z], q = [rt.orientation.x, rt.orientation.y, rt.orientation.z, rt.orientation.w];
      const a = { anchorSpace: new FakeSpace(() => ({ p, q })), delete() {}, requestPersistentHandle: () => Promise.resolve("mock-anchor-" + Date.now()) };
      this.session._anchors.add(a);
      return Promise.resolve(a);
    }
  }
  function plane(d) {
    return { orientation: d.orientation, semanticLabel: d.semanticLabel, lastChangedTime: 1,
      planeSpace: new FakeSpace(() => ({ p: [0, 0, 0], q: [0, 0, 0, 1] })), polygon: d.polygon.map((p) => ({ x: p[0], y: p[1], z: p[2], w: 1 })) };
  }
  function mesh(d) {
    return { semanticLabel: d.semanticLabel, lastChangedTime: 1, meshSpace: new FakeSpace(() => ({ p: [0, 0, 0], q: [0, 0, 0, 1] })),
      vertices: new Float32Array(d.points.flat()), indices: new Uint32Array(0) };
  }

  // ---------------------------------------------------------------- XRSystem
  const xr = new EventTarget();
  xr.isSessionSupported = (mode) => Promise.resolve(mode === "immersive-vr" ? !!M.supported.vr : mode === "immersive-ar" ? !!M.supported.ar : mode === "inline");
  xr.requestSession = (mode, init) => {
    if ((mode === "immersive-vr" && !M.supported.vr) || (mode === "immersive-ar" && !M.supported.ar)) return Promise.reject(new DOMException("not supported", "NotSupportedError"));
    const s = new FakeSession(mode, init);
    M.session = s;
    return Promise.resolve(s);
  };
  try { Object.defineProperty(Navigator.prototype, "xr", { get: () => xr, configurable: true }); } catch (e) { /* fall back */ }
  try { Object.defineProperty(navigator, "xr", { value: xr, configurable: true }); } catch (e) { /* ok */ }
  window.XRWebGLLayer = XRWebGLLayer;
  window.XRRigidTransform = XRRigidTransform;
  try { delete window.XRWebGLBinding; } catch (e) { window.XRWebGLBinding = undefined; }
  for (const C of [window.WebGLRenderingContext, window.WebGL2RenderingContext]) if (C) C.prototype.makeXRCompatible = function () { return Promise.resolve(); };

  // ---------------------------------------------------------------- test API
  M.script = (steps) => new Promise((resolve) => {
    const s = M.session;
    if (!s || s._ended) { resolve(false); return; }
    const list = steps.map((x) => Object.assign({ frames: 2 }, x, { _applied: false }));
    if (!list.length) { resolve(true); return; }
    list[list.length - 1]._done = () => resolve(true);
    s._steps.push(...list);
  });
  M.setPlanes = (p) => { M.planes = p; };
  M.setMeshes = (m) => { M.meshes = m; };
})();
