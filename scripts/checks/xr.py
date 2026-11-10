"""P2b VR / MR (docs/vr.md) without a headset: the pure core under node, then the real page in headless Chrome with a
mocked navigator.xr (scripts/checks/xr_mock.js: fake XRSystem / XRSession / frames / hand joints / planes / anchors).

1. node --test tests/xr_core.test.mjs: calibration math, recentring, gestures, stroke resampling, the input -> message
   pipeline, room scan -> scene spec (skipped with a note when node is not installed);
2. the Enter VR / Enter MR buttons appear only when isSessionSupported says so;
3. Enter VR starts a session through three.js's WebXRManager and renders frames without errors;
4. a synthetic pinch stroke (fake hand joints) becomes {traj, op:"propose", source:"vr", frame:"world"} the server accepts;
5. grabbing a ghost handle, lifting it 5 cm and twisting the wrist sends op:"edit" (point moved, wrist turned);
   a short pinch-tap adds a keyframe (insert); Undo on the menu puts the previous path back;
6. a fist drives the robot's hand (teleop start + ~30 Hz commands) and leaving Drive mode stops it;
7. Enter MR: two pinches on the table's corners calibrate (centred, saved per browser), passthrough look;
8. room scan (fake planes + a mesh) -> a world set scene the server builds (table surface + a fixed box);
9. a session without the optional features (no local-floor, no hands, no planes) still starts.
Headless Chrome: PC_CHROME or the usual install paths; skipped (not failed) when there is none. CDP port 9312
(PC_XR_CDP_PORT). Screenshots of the headset view go to PC_XR_SHOTS when set."""
import asyncio
import base64
import json
import math
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

import aiohttp

ROOT = Path(__file__).resolve().parents[2]
MOCK = Path(__file__).with_name("xr_mock.js")
CDP_PORT = int(os.environ.get("PC_XR_CDP_PORT", "9312"))


def _chrome():
    for c in [os.environ.get("PC_CHROME"), "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
              shutil.which("google-chrome"), shutil.which("google-chrome-stable"), shutil.which("chromium"), shutil.which("chromium-browser")]:
        if c and Path(c).exists():
            return c
    return None


class CDP:
    def __init__(self, ws):
        self.ws, self.n, self.wait, self.errors, self.logs = ws, 0, {}, [], []
        self.reader = asyncio.ensure_future(self._read())

    async def _read(self):
        async for m in self.ws:
            if m.type != aiohttp.WSMsgType.TEXT:
                continue
            d = json.loads(m.data)
            if "id" in d and d["id"] in self.wait:
                self.wait.pop(d["id"]).set_result(d)
            elif d.get("method") == "Runtime.exceptionThrown":
                ex = d["params"]["exceptionDetails"]
                self.errors.append((ex.get("exception") or {}).get("description") or ex.get("text"))
            elif d.get("method") == "Runtime.consoleAPICalled" and d["params"].get("type") == "error":
                self.errors.append(" ".join(str(a.get("value", a.get("description", ""))) for a in d["params"].get("args", [])))

    async def call(self, method, params=None, timeout=20):
        self.n += 1
        fut = asyncio.get_event_loop().create_future()
        self.wait[self.n] = fut
        await self.ws.send_str(json.dumps({"id": self.n, "method": method, "params": params or {}}))
        return await asyncio.wait_for(fut, timeout)

    async def eval(self, expr, timeout=30):
        r = await self.call("Runtime.evaluate", {"expression": expr, "awaitPromise": True, "returnByValue": True}, timeout)
        res = r.get("result") or {}
        if res.get("exceptionDetails"):
            ex = res["exceptionDetails"]
            raise RuntimeError((ex.get("exception") or {}).get("description") or ex.get("text"))
        return (res.get("result") or {}).get("value")


async def run(t):
    # ---- 1. the pure core under node
    node = shutil.which("node")
    if node:
        r = await asyncio.get_event_loop().run_in_executor(None, lambda: subprocess.run(
            [node, "--test", str(ROOT / "tests" / "xr_core.test.mjs")], cwd=ROOT, capture_output=True, text=True, timeout=120))
        out = r.stdout + r.stderr
        npass = next((ln.split()[-1] for ln in out.splitlines() if ln.startswith("# pass")), "?")
        nfail = next((ln.split()[-1] for ln in out.splitlines() if ln.startswith("# fail")), "?")
        t.check("xr(unit): calibration, recentring, gestures, strokes, input -> messages, room -> scene (node --test)",
                r.returncode == 0, f"{npass} passed, {nfail} failed")
    else:
        print("SKIP  xr(unit): node is not installed (tests/xr_core.test.mjs)", flush=True)

    chrome = _chrome()
    if not chrome:
        print("SKIP  xr: no Chrome / Chromium for the emulated headset (set PC_CHROME)", flush=True)
        return
    prof = tempfile.mkdtemp(prefix="p2b_chrome_")
    proc = subprocess.Popen([chrome, "--headless=new", "--disable-gpu", "--enable-unsafe-swiftshader", "--hide-scrollbars", "--mute-audio",
                             "--window-size=1280,800", f"--remote-debugging-port={CDP_PORT}", f"--user-data-dir={prof}",
                             "--no-first-run", "--no-default-browser-check", "about:blank"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        await _browser(t, chrome)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(prof, ignore_errors=True)


async def _browser(t, chrome):
    shots = os.environ.get("PC_XR_SHOTS")
    async with aiohttp.ClientSession() as s:
        page = None
        for _ in range(60):
            try:
                async with s.get(f"http://127.0.0.1:{CDP_PORT}/json") as r:
                    page = next(x for x in await r.json() if x.get("type") == "page")
                break
            except Exception:
                await asyncio.sleep(0.5)
        if page is None:
            t.check("xr: headless Chrome starts", False, chrome)
            return
        ws = await s.ws_connect(page["webSocketDebuggerUrl"], max_msg_size=0)
        cdp = CDP(ws)
        try:
            await _flow(t, cdp, shots)
        finally:
            cdp.reader.cancel()
            await ws.close()


async def _until(t, cdp, expr, limit=10.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        try:
            if await cdp.eval(expr):
                return True
        except Exception:
            pass
        await t.pump(0.2)            # keep the app socket read while waiting
    return False


async def _ev(t, cdp, expr, timeout=90):
    """A long in-page script (the mock plays hand poses frame by frame): keep the app socket read meanwhile."""
    task = asyncio.ensure_future(cdp.eval(expr, timeout))
    while not task.done():
        await t.pump(0.2)
    return task.result()


async def _reply(t, pred, limit=8.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.2)
        r = (t.state().get("traj") or {}).get("reply") or {}
        if pred(r):
            return r
    return {}


async def _shot(cdp, shots, name):
    if not shots:
        return
    try:
        r = await cdp.call("Page.captureScreenshot", {"format": "png"})
        Path(shots).mkdir(parents=True, exist_ok=True)
        (Path(shots) / f"{name}.png").write_bytes(base64.b64decode(r["result"]["data"]))
    except Exception:
        pass


async def _flow(t, cdp, shots):
    await cdp.call("Page.enable")
    await cdp.call("Runtime.enable")
    await cdp.call("Page.addScriptToEvaluateOnNewDocument", {"source": MOCK.read_text()})
    await cdp.call("Page.navigate", {"url": t.base + "/?nospeak=1"})
    ok = await _until(t, cdp, "!!(window.PCXR && PCXR.supported.checked && window.PC3D && PC3D.mounted)", 30)
    if not ok:
        t.check("xr: the page loads web/xr.js", False, "PCXR never appeared")
        return
    await _until(t, cdp, "PC3D.sceneReady", 30)
    await t.task("reset_scene", 1)
    await t.send(type="traj", op="clear")

    # ---- 2. buttons only when supported
    both = await cdp.eval("[...document.querySelectorAll('#xrEnter [data-xr]')].map(b => b.dataset.xr + ':' + b.textContent).join(',')")
    none = await cdp.eval("(async () => { __xrmock.supported = {vr: false, ar: false}; await PCXR.detect(); const h = document.getElementById('xrEnter').hidden;"
                          " __xrmock.supported = {vr: true, ar: false}; await PCXR.detect(); const only = document.querySelectorAll('#xrEnter [data-xr]').length;"
                          " __xrmock.supported = {vr: true, ar: true}; await PCXR.detect(); return [h, only]; })()")
    t.check("xr: Enter VR / Enter MR appear only when isSessionSupported says so",
            both == "vr:Enter VR,ar:Enter MR" and none == [True, 1], f"{both}; unsupported hidden={none[0]}, vr-only buttons={none[1]}")

    # ---- 3. Enter VR
    await cdp.eval("document.querySelector('#xrEnter [data-xr=vr]').click()")
    ok = await _until(t, cdp, "PCXR.active && PC3D.renderer.xr.isPresenting && PCXR.info.frames > 3", 40)
    tris = await cdp.eval("PC3D.renderer.info.render.triangles")
    # software GL (SwiftShader) draws the full robot at < 1 frame/s: hide its high-poly link meshes for the rest of the run
    await cdp.eval("PC3D.geoms.traverse((o) => { if (o.isMesh && o.geometry.index && o.geometry.index.count > 15000) o.visible = false; })")
    ok = ok and await _until(t, cdp, "PCXR.info.frames > 20", 15)
    info = await cdp.eval("({frames: PCXR.info.frames, errors: PCXR.info.errors, ref: PCXR.refType, feats: PCXR.info.features, mock: __xrmock.log})")
    edge = await cdp.eval("PCXR.simToXr([PC3D.TABLE.x1, 0, PC3D.TABLE.z]).map(v => +v.toFixed(3))")
    t.check("xr: Enter VR starts the session (three.js WebXRManager, local-floor, hands) and renders without errors",
            ok and not info["errors"] and not info["mock"] and info["ref"] == "local-floor" and "hand-tracking" in info["feats"],
            f"{info['frames']} frames ({tris} triangles a frame, both eyes), ref {info['ref']}, features {info['feats']}, errors {info['errors'][:1] or info['mock'][:1]}")
    t.check("xr: VR recentre puts the table in front of you at a comfortable height",
            edge and abs(edge[0]) < 0.01 and abs(edge[1] - 0.98) < 0.01 and abs(edge[2] + 0.32) < 0.01, f"your near edge's middle at {edge} (XR)")
    await _shot(cdp, shots, "p2b_vr_enter")

    # ---- 4. a pinch stroke in the air -> traj propose
    stroke = await _ev(t, cdp, """(async () => {
      const tb = PC3D.TABLE, pts = [];
      for (let i = 0; i <= 24; i++) { const u = i / 24; pts.push([0.40 + 0.04 * Math.sin(u * Math.PI), -0.26 + 0.22 * u, tb.z + 0.17 + 0.04 * Math.sin(u * Math.PI)]); }
      const xr = pts.map((p) => PCXR.simToXr(p));
      const steps = [{right: {at: xr[0], pinch: 0.06}, frames: 3}, {right: {at: xr[0], pinch: 0.008}, frames: 3}];
      for (const p of xr.slice(1)) steps.push({right: {at: p, pinch: 0.008}, frames: 2});
      steps.push({right: {at: xr[xr.length - 1], pinch: 0.06}, frames: 3});
      await __xrmock.script(steps);
      const m = PCXR.info.sent.filter((x) => x.type === 'traj').pop();
      return {first: pts[0], last: pts[pts.length - 1], msg: m || null};
    })()""", 60)
    msg = stroke["msg"] or {}
    r = await _reply(t, lambda r: r.get("id") == msg.get("id"))
    tr = (t.state().get("traj") or {}).get("proposed") or {}
    p0 = ((tr.get("points") or [{}])[0]).get("p")
    t.check("xr: a pinch stroke (fake hand joints) becomes traj propose, source vr, world frame, accepted by the server",
            msg.get("op") == "propose" and (msg.get("traj") or {}).get("source") == "vr" and (msg.get("traj") or {}).get("frame") == "world"
            and r.get("traj") and tr.get("source") == "vr" and 12 <= len(tr.get("points") or []) <= 60 and p0 and math.dist(p0, stroke["first"]) < 0.01,
            f"{len(tr.get('points') or [])} points, first {p0} vs drawn {[round(v, 3) for v in stroke['first']]}; server: {r.get('message')}")
    await _shot(cdp, shots, "p2b_vr_stroke")

    # ---- 5. grab a handle: lift 5 cm, twist 30 degrees -> op:edit
    await _until(t, cdp, "!!(PCUI.state.traj && PCUI.state.traj.ghost && PCUI.state.traj.ghost.keys.length > 5)", 5)
    grab = await _ev(t, cdp, """(async () => {
      const g = PCUI.state.traj.ghost, i = Math.floor(g.keys.length / 2), k = g.keys[i];
      const at = (u) => PCXR.simToXr([k[0], k[1], k[2] + 0.05 * u]);
      const qy = (a) => [0, Math.sin(a / 2), 0, Math.cos(a / 2)];
      const steps = [{right: {at: at(0), pinch: 0.06}, frames: 3}, {right: {at: at(0), pinch: 0.008}, frames: 3}];
      for (let s = 1; s <= 10; s++) steps.push({right: {at: at(s / 10), q: qy(Math.PI / 6 * s / 10), pinch: 0.008}, frames: 2});
      steps.push({right: {at: at(1), q: qy(Math.PI / 6), pinch: 0.06}, frames: 3});
      await __xrmock.script(steps);
      const m = PCXR.info.sent.filter((x) => x.type === 'traj').pop();
      return {i, k, kq: g.kq[i], msg: m};
    })()""", 60)
    gm = grab["msg"] or {}
    r = await _reply(t, lambda r: r.get("id") == gm.get("id"))
    tr = (t.state().get("traj") or {}).get("proposed") or {}
    pt = (tr.get("points") or [])[grab["i"]] if len(tr.get("points") or []) > grab["i"] else {}
    dz = (pt.get("p") or [0, 0, 0])[2] - grab["k"][2]
    q = pt.get("q")
    turn = None
    if q and grab.get("kq"):                     # how far the point's gripper turned (we twisted 30 degrees about the vertical)
        d = abs(sum(a * b for a, b in zip(q, grab["kq"]))) / (math.sqrt(sum(a * a for a in q)) * math.sqrt(sum(b * b for b in grab["kq"])))
        turn = math.degrees(2 * math.acos(min(1.0, d)))
    t.check("xr: grabbing a ghost handle, lifting it 5 cm and twisting the wrist sends op:edit (moved + turned)",
            gm.get("op") == "edit" and gm.get("index") == grab["i"] and (gm.get("point") or {}).get("q") and r.get("op") == "edit"
            and abs(dz - 0.05) < 0.006 and turn is not None and 25 < turn < 35,
            f"point {grab['i'] + 1}: lifted {dz * 100:.1f} cm, gripper turned {turn and round(turn)} deg; server: {r.get('message')}")

    n_before = len(tr.get("points") or [])
    tap = await _ev(t, cdp, """(async () => {
      const K = PCUI.state.traj.ghost.keys, i = Math.floor(K.length / 3), k = K[i];
      const at = PCXR.simToXr([k[0], k[1], k[2] + 0.07]);
      await __xrmock.script([{right: {at, pinch: 0.06}, frames: 3}, {right: {at, pinch: 0.008}, frames: 3}, {right: {at, pinch: 0.06}, frames: 3}]);
      return PCXR.info.sent.filter((x) => x.type === 'traj').pop();
    })()""", 30)
    r = await _reply(t, lambda r: r.get("id") == (tap or {}).get("id"))
    n_after = len(((t.state().get("traj") or {}).get("proposed") or {}).get("points") or [])
    t.check("xr: a short pinch-tap in empty space inserts a keyframe (op:edit insert)",
            (tap or {}).get("insert") is not None and n_after == n_before + 1, f"{n_before} -> {n_after} points; {r.get('message')}")
    undo = await cdp.eval("(async () => { PCXR.press('undo'); return PCXR.info.sent.filter((x) => x.type === 'traj').pop(); })()")
    r = await _reply(t, lambda r: r.get("id") == (undo or {}).get("id"))
    n_undo = len(((t.state().get("traj") or {}).get("proposed") or {}).get("points") or [])
    t.check("xr: Undo (menu) puts the previous path back", (undo or {}).get("op") == "propose" and n_undo == n_before, f"{n_after} -> {n_undo} points")
    await t.send(type="traj", op="clear")
    await t.pump(0.4)

    # ---- 6. teleop: a fist drives the robot's hand
    await t.task("home", 2)
    await t.pump(0.6)
    hands0 = (t.state().get("teleop") or {}).get("hands") or {}
    tele = await _ev(t, cdp, """(async () => {
      PCXR.setMode('teleop');
      const H = PCUI.state.teleop.hands, h = (H.left || H.right).p, at = (dy) => PCXR.simToXr([h[0] + 0.12, h[1] + dy, h[2] + 0.12]);
      const steps = [{right: {at: at(0), pinch: 0.06}, frames: 3}, {right: {at: at(0), pinch: 0.06, fist: true}, frames: 4}];
      for (let s = 1; s <= 15; s++) steps.push({right: {at: at(0.06 * s / 15), pinch: 0.06, fist: true}, frames: 2});
      steps.push({right: {at: at(0.06), pinch: 0.06, fist: true}, frames: 30});
      await __xrmock.script(steps);
      const sent = PCXR.info.sent.filter((x) => x.type === 'teleop');
      return {n: sent.filter((x) => !x.op).length, start: sent.find((x) => x.op === 'start') || null, last: sent.filter((x) => !x.op).pop() || null};
    })()""", 60)
    await t.pump(0.8)
    tp = t.state().get("teleop") or {}
    role = (tele["start"] or {}).get("role") or "right"            # across the table your right hand drives its left
    start, now = (hands0.get(role) or {}).get("p"), ((tp.get("hands") or {}).get(role) or {}).get("p")
    dy = (now[1] - start[1]) if (now and start) else 0
    t.check("xr: a fist drives the robot's hand live (teleop start + ~30 Hz commands, the hand follows)",
            tele["start"] and tele["n"] >= 10 and tp.get("active") and tp.get("src") == "vr" and abs(dy - 0.06) < 0.015,
            f"your right hand -> its {role} hand: {tele['n']} commands, target moved {dy * 100:.1f} cm (wanted 6), rate {tp.get('rate')}/s, held {tp.get('held')}")
    await _ev(t, cdp, "(async () => { await __xrmock.script([{right: {at: [0, 1, -0.3], pinch: 0.06}, frames: 3}]); PCXR.setMode('draw'); })()")
    await t.pump(0.6)
    t.check("xr: leaving Drive mode stops teleop", (t.state().get("teleop") or {}).get("active") is False, (t.state().get("teleop") or {}).get("reply"))

    # ---- 7. MR: two-point calibration, saved per browser, passthrough look
    await cdp.eval("PCXR.exit()")
    ok = await _until(t, cdp, "!PCXR.active && !PC3D.renderer.xr.isPresenting", 8)
    t.check("xr: exiting VR restores the desktop 3D view", ok and await cdp.eval("PCXR.info.ended >= 1 && !!PC3D.scene.background && PC3D.renderer.shadowMap.enabled"))
    await cdp.eval("localStorage.removeItem('pc.xr.mr'); document.querySelector('#xrEnter [data-xr=ar]').click()")
    ok = await _until(t, cdp, "PCXR.active && PCXR.kind === 'ar' && PCXR.mode === 'calibrate' && PCXR.info.frames > 5", 15)
    cal = await _ev(t, cdp, """(async () => {
      const L = [-0.6, 0.76, -0.5], R = [0.6, 0.76, -0.5], tap = (at) => [{right: {at, pinch: 0.06}, frames: 3}, {right: {at, pinch: 0.008}, frames: 3}, {right: {at, pinch: 0.06}, frames: 3}];
      await __xrmock.script(tap(L).concat(tap(R), [{right: null, frames: 3}]));
      const T = PC3D.TABLE, saved = JSON.parse(localStorage.getItem('pc.xr.mr') || 'null');
      let floorHidden = true; PC3D.geoms.traverse((o) => { if (o.userData.floor && o.visible) floorHidden = false; });
      return {src: PCXR.alSource, mode: PCXR.mode, nl: PCXR.simToXr([T.x1, T.y0, T.z]), nr: PCXR.simToXr([T.x1, T.y1, T.z]), saved: !!(saved && saved.align),
              bg: PC3D.scene.background, floorHidden};
    })()""", 40)
    t.check("xr: MR two-point calibration lines the sim table up with the real one (centred, saved per browser, passthrough)",
            ok and cal["src"] == "calibrated" and cal["mode"] == "draw" and math.dist(cal["nl"], [-0.5, 0.76, -0.5]) < 0.005
            and math.dist(cal["nr"], [0.5, 0.76, -0.5]) < 0.005 and cal["saved"] and cal["bg"] is None and cal["floorHidden"],
            f"sim near-left -> {[round(v, 3) for v in cal['nl']]}, saved={cal['saved']}, background={cal['bg']}, floor hidden={cal['floorHidden']}")
    await _shot(cdp, shots, "p2b_mr_calibrated")

    # ---- 8. room scan -> world set
    room = await _ev(t, cdp, """(async () => {
      const rect = (x0, x1, z0, z1, y) => [[x0, y, z0], [x1, y, z0], [x1, y, z1], [x0, y, z1]];
      __xrmock.setPlanes([{orientation: 'horizontal', semanticLabel: 'floor', polygon: rect(-2, 2, -3, 1, 0)},
                          {orientation: 'horizontal', semanticLabel: 'table', polygon: rect(-0.6, 0.6, -1.1, -0.5, 0.76)},
                          {orientation: 'vertical', semanticLabel: 'wall', polygon: [[-2, 0, -1.6], [2, 0, -1.6], [2, 2.4, -1.6], [-2, 2.4, -1.6]]}]);
      __xrmock.setMeshes([{semanticLabel: 'screen', points: [[-0.2, 0.76, -1.0], [0.2, 0.76, -1.0], [0.2, 1.08, -0.95], [-0.2, 1.08, -0.95]]}]);
      PCXR.setMode('scan');
      await new Promise((r) => setTimeout(r, 900));
      const R = PCXR.room;
      return R && {ok: R.ok, spec: R.spec, notes: R.notes, boxes: R.boxes.map((b) => [b.label, b.used])};
    })()""", 30)
    spec = (room or {}).get("spec") or {}
    scr = next((o for o in spec.get("objects") or [] if o.get("name") == "screen"), None)
    used = await cdp.eval("PCXR.useRoom()")
    t0 = time.time()
    built = None
    while time.time() - t0 < 30:
        await t.pump(0.4)
        w = t.state().get("world") or {}
        objs = (w.get("scene") or {}).get("objects") or []
        built = next((o for o in objs if o.get("name") == "screen"), None)
        if built is not None:
            break
    task = t.state().get("task") or {}
    t.check("xr: room scan (planes + a mesh) -> a world set scene the server builds (table surface + fixed box)",
            room and room["ok"] and scr and scr["movable"] is False and used and built is not None and built.get("movable") is False
            and str(task.get("id", "")).startswith("vr-room") and task.get("status") == "done",
            f"spec {json.dumps(spec)[:160]}; built {built}; task {task.get('status')} {task.get('message', '')[:80]}")
    await _shot(cdp, shots, "p2b_mr_room")
    await cdp.eval("PCXR.exit()")
    await _until(t, cdp, "!PCXR.active", 8)
    await t.send(type="world", op="reset", id="xr-reset")
    t0 = time.time()
    while time.time() - t0 < 30:
        await t.pump(0.4)
        if (((t.state().get("world") or {}).get("robot") or {}).get("id")) == "g1" and not (t.state().get("world") or {}).get("scene", {}).get("objects", [{}])[0].get("name") == "screen":
            break

    # ---- 9. graceful without the optional features
    await _until(t, cdp, "PC3D.sceneReady", 20)
    await cdp.eval("__xrmock.granted = ['local']; __xrmock.setPlanes(null); __xrmock.setMeshes(null); document.querySelector('#xrEnter [data-xr=vr]').click()")
    ok = await _until(t, cdp, "PCXR.active && PCXR.info.frames > 10", 15)
    bare = await _ev(t, cdp, "(async () => { PCXR.setMode('scan'); await new Promise((r) => setTimeout(r, 300)); const s = PCXR.status;"
                          " const out = {ref: PCXR.refType, feats: PCXR.info.features, errors: PCXR.info.errors.length, text: s.lines.map((l) => l.t).join(' ')};"
                          " await PCXR.exit(); __xrmock.granted = null; return out; })()")
    t.check("xr: without local-floor / hand tracking / plane detection the session still starts and says what's missing",
            ok and bare["ref"] == "local" and bare["errors"] == 0 and "room data" in bare["text"], f"ref {bare['ref']}, features {bare['feats']}")
    errs = [e for e in cdp.errors if e and "favicon" not in str(e)]
    t.check("xr: no page errors during the emulated sessions", not errs, "; ".join(str(e)[:120] for e in errs[:2]))
