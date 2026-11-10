"""End-to-end self test: starts an isolated server (synthetic input) and exercises every feature over the real protocol.
Run: .venv/bin/python scripts/selftest.py        Prints PASS/FAIL per check and exits non-zero on any failure.

Environment:
  PC_SELFTEST_PORT=8791      HTTP port of the test server (HTTPS uses the next one); give parallel runs their own ports.
  PC_SELFTEST_OFFLINE=1      CI / no keys: the server runs with --byok (never reads .env keys) and the checks that need
                             a key, the network or a local test clip are reported as SKIP instead of FAIL.
  PC_SELFTEST_LOG=path       write the server's stdout/stderr there (default: discarded)."""
import asyncio
import json
import os
import ssl
import struct
import subprocess
import sys
import time
from pathlib import Path

import aiohttp

ROOT = Path(__file__).resolve().parents[1]
PORT = int(os.environ.get("PC_SELFTEST_PORT", "8791")); HTTPS = PORT + 1   # parallel runs: give each its own port
BASE = f"http://127.0.0.1:{PORT}"
OFFLINE = os.environ.get("PC_SELFTEST_OFFLINE") == "1"
if OFFLINE:
    os.environ["PC_BYOK"] = "1"   # checks in this process see what the --byok server sees: no server keys
results = []
skipped = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


def skip(name, why):
    skipped.append(name)
    print(f"SKIP  {name}  {why}", flush=True)


async def main():
    s = aiohttp.ClientSession()
    for _ in range(80):
        try:
            async with s.get(BASE + "/") as r:
                if r.status == 200:
                    break
        except Exception:
            await asyncio.sleep(0.5)
    ws = await s.ws_connect(BASE + "/ws")
    st = {}
    bins = {}

    async def pump(sec):
        nonlocal st
        t0 = time.time()
        while time.time() - t0 < sec:
            m = await asyncio.wait_for(ws.receive(), 120)   # physics: a refused pick plans every grasp first (seconds; more on a loaded machine)
            if m.type == aiohttp.WSMsgType.TEXT:
                st = json.loads(m.data)
            elif m.type == aiohttp.WSMsgType.BINARY:
                bins[m.data[0]] = len(m.data)
        return st

    async def task(action, wait=1.0, **kw):
        tid = f"{action}-{time.time():.3f}"
        await ws.send_str(json.dumps({"type": "task", "id": tid, "action": action, **kw}))
        t0 = time.time()
        while time.time() - t0 < wait:
            await pump(0.2)
            if st.get("task") and st["task"]["id"] == tid and st["task"]["status"] != "planning":
                break
        return st.get("task") or {}

    async def until_done(limit=25):
        t0 = time.time()
        while time.time() - t0 < limit:
            await pump(0.3)
            if st.get("task", {}).get("status") != "running":
                break
        return st.get("task") or {}

    send = lambda **k: ws.send_str(json.dumps(k))

    # --- serving
    async with s.get(BASE + "/static/app.js") as r:
        check("static files", r.status == 200)
    async with s.get(BASE + "/scene.json") as r:
        sc = await r.json()
    check("scene.json", len(sc["geoms"]) > 30 and len(sc["meshes"]) > 20, f"{len(sc['geoms'])} geoms")
    mid = next(iter(sc["meshes"]))
    async with s.get(f"{BASE}/mesh/{mid}.bin") as r:
        b = await r.read()
    nv, nf = struct.unpack("<II", b[:8])
    check("mesh binary", len(b) == 8 + nv * 12 + nf * 12, f"{nv} verts")
    ctx = ssl.create_default_context(); ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
    try:
        async with s.get(f"https://127.0.0.1:{HTTPS}/", ssl=ctx) as r:
            check("https (phone mic)", r.status == 200)
    except Exception as e:
        check("https (phone mic)", False, str(e)[:60])
    await pump(1.5)
    check("frames: camera / robot view / full body", all(k in bins for k in (1, 2, 3)), f"{sorted(bins)} server {st.get('fps')} fps")
    keys = ("mode", "view", "feasibility", "markers", "path", "task", "holding", "scene_objects", "skills", "teach", "geom_poses", "say", "mock")
    check("state fields", all(k in st for k in keys), ",".join(k for k in keys if k not in st))
    names = sorted(o["name"] for o in st["scene_objects"])
    check("scene objects", names == ["blue block", "green block", "mug", "red block", "vase"], str(names))

    # --- spatial: tap, draw, 3D path
    await send(type="tap", u=0.45, v=0.55); await pump(0.5)
    check("tap -> marker", any(m["id"] == "A" for m in st["markers"]), str([m["id"] for m in st["markers"]]))
    await send(type="draw", points=[[0.4, 0.75], [0.5, 0.72], [0.6, 0.75]]); await pump(1.0)
    check("draw on robot view -> checked path", st["path"] and len(st["path"]["ok"]) > 2, st["messages"][:1])
    await send(type="path3d", points=[[0.28, -0.12, 0.84], [0.30, -0.02, 0.92], [0.30, 0.10, 0.84]], hand="auto"); await pump(1.2)
    check("3D path -> per-waypoint verdict", st["path"] and "pts" in st["path"], f"{sum(st['path']['ok'])}/{len(st['path']['ok'])} ok")
    await send(type="clear"); await pump(0.3)

    # --- voice-level tasks with physics verdicts
    t = await task("pick_place", 2, object="mug", target="back")
    check("refuse unreachable + suggest", t.get("status") == "failed" and "closest spot" in t.get("message", ""), t.get("message", "")[:90])
    t = await task("pick_place", 2, object="mug", target="left")
    check("refuse carry through vase", t.get("status") == "failed" and "vase" in t.get("message", ""), t.get("message", "")[:80])
    t = await task("pick_place", 2, object="mug", target="center")
    t = await until_done()
    check("pick & place mug -> center", t.get("status") == "done", t.get("message", "")[:60])
    route = [[0.28, -0.12, 1.04], [0.29, -0.02, 1.14], [0.29, 0.10, 1.14], [0.29, 0.20, 1.02]]
    await task("reset_scene", 1)
    await send(type="path3d", points=route, hand="right"); await pump(1.2)
    t = await task("pick_place", 2, object="mug", target="along my path")
    t = await until_done()
    check("carry over the vase along a 3D route", t.get("status") == "done", t.get("message", "")[:60])
    await task("reset_scene", 1)
    t = await task("pick_place", 2, object="red block", target="center")
    t = await until_done()
    check("second object (red block)", t.get("status") == "done", t.get("message", "")[:60])
    t = await task("pick_place", 1, object="blue block", target="left")
    check("fixed object is refused", t.get("status") == "failed" and "fixed" in t.get("message", ""), t.get("message", "")[:60])
    await task("reset_scene", 1)

    # --- deictic + mock
    await send(type="mock", enabled=True); await pump(0.5)
    check("mock brain on + speaks", st["mock"] and st["say"] and "Mock AI on" in st["say"]["text"], (st["say"] or {}).get("text", "")[:50])
    await send(type="tap", u=0.60, v=0.66); await pump(0.4)
    t = await task("pick_place", 2, object="mug", target="there")
    check("'there' resolves to the marked spot", t.get("status") in ("running", "done") or "reach" in t.get("message", ""), t.get("message", "")[:70])
    await until_done(); await task("reset_scene", 1)
    await send(type="mock", enabled=False); await pump(0.3)

    # --- follow mode, skills, edit, compile, export
    await task("follow_me", 1, enabled=True); await pump(3)
    check("follow me -> FOLLOW mode", st["mode"] == "FOLLOW")
    t = await task("learn_skill", 1, name="selftest_wave")
    check("learn skill", t.get("status") == "done", t.get("message", "")[:50])
    t = await task("edit_skill", 1, name="selftest_wave", t0=0.5, t1=2.0, part="arms", speed=1.5)
    check("edit skill (part + time range)", t.get("status") == "done", t.get("message", "")[:60])
    t = await task("play_skill", 1, name="selftest_wave")
    t = await until_done()
    check("play skill", t.get("status") == "done" and "Finished" in t.get("message", ""), t.get("message", "")[:40])
    t = await task("compile_for", 15, robot="ur5e")
    check("compile same intent for UR5e", t.get("status") in ("done", "failed") and "UR5e" in t.get("message", ""), t.get("message", "")[:60])
    t = await task("export", 2)
    check("export intent/trajectory/report", t.get("status") == "done" and st.get("saved"), str(st.get("saved"))[-30:])
    p = Path(st["saved"]) if st.get("saved") else None
    check("exported files exist", p and all((p / f).exists() for f in ("intent.json", "trajectory.json", "report.json")))

    # --- teach by showing (core)
    T = lambda **k: send(type="teach", **k)
    await T(cmd="start"); await pump(1)
    check("teach: recording", st["teach"]["phase"] == "recording")
    for u in ["pick up the mug and put it in the middle", "don't touch the vase", "then put the red block on the right", "slowly"]:
        await T(cmd="utterance", text=u); await pump(0.2)
    await T(cmd="stop"); await pump(0.8)
    tc = st["teach"]
    steps = [(x["object"], x["target"]) for x in tc["spec"]["steps"]] if tc["spec"] else []
    check("teach: understood steps", steps == [("mug", "center"), ("red_block", "right")], str(steps))
    cons = sorted(c["type"] for c in tc["spec"]["constraints"]) if tc["spec"] else []
    check("teach: understood constraints", cons == ["avoid", "slow"], str(cons))
    check("teach: asks back", tc["phase"] == "confirm" and "Is that right" in tc["question"], tc["question"][:70])
    await T(cmd="correct", text="no, put the red block on the left instead"); await pump(0.6)
    check("teach: correction edits the spec", st["teach"]["spec"]["steps"][-1]["target"] == "left", str([(x["object"], x["target"]) for x in st["teach"]["spec"]["steps"]]))
    await T(cmd="correct", text="no, the red block goes on the right"); await pump(0.4)
    await T(cmd="correct", text="yes"); t0 = time.time()
    while time.time() - t0 < 60:
        await pump(1)
        if st["teach"]["phase"] != "practice":
            break
    tc = st["teach"]
    check("teach: practice in sim", tc["phase"] == "done", f"{tc['phase']}: " + " | ".join(f"{x['status']}" for x in tc["spec"]["steps"]))
    await T(cmd="save", name="selftest_task"); await pump(0.5)
    check("teach: save as skill", "selftest_task" in st["skills"])
    t = await task("play_skill", 2, name="selftest_task"); await pump(0.5)
    check("teach: replay saved task", st["teach"]["phase"] == "practice", t.get("message", "")[:50])
    await T(cmd="cancel"); await task("reset_scene", 1)

    # --- v3: phone frames, robot switch, spatial model, proxies, tts, demo autopilot
    import cv2
    import numpy as np
    jpg = cv2.imencode(".jpg", np.full((360, 640, 3), 120, np.uint8))[1].tobytes()
    for _ in range(8):
        await ws.send_bytes(b"\x10" + jpg); await pump(0.15)
    check("phone camera frames become the source", st["source"] == "phone", st["source"])
    await pump(1.6)
    check("falls back when phone stops", st["source"] != "phone", st["source"])
    await send(type="robot", name="ur5e"); await pump(1.5)
    check("robot switch -> UR5e verdict", st["robot"] == "ur5e" and "UR5e" in (st.get("task") or {}).get("message", ""), (st.get("task") or {}).get("message", "")[:60])
    await send(type="robot", name="g1"); await pump(0.3)
    await send(type="spatial_model", name="mediapipe"); await pump(0.4)
    check("spatial model switch", st["spatial"]["model"] == "mediapipe")
    await send(type="spatial_model", name="yolo-seg"); await pump(0.3)
    for _ in range(10):
        await ws.send_bytes(b"\x10" + jpg); await pump(0.07)
    ci = st.get("cam_in") or {}
    check("state.cam_in: a browser camera streaming, with its fps", ci.get("fps", 0) > 3 and ci.get("mobile") is False, str(ci))
    async with s.get(BASE + "/phone") as r:
        body = await r.text()
        check("/phone serves the phone camera page", r.status == 200 and "getUserMedia" in body, f"{r.status} {len(body)}B")
    async with s.get(BASE + "/phone/info") as r:
        info = await r.json()
        check("/phone/info gives the https address for the QR code", info.get("https_port") == HTTPS and str(info.get("url", "")).startswith("https://"), str(info))
    async with s.get(BASE + "/tts", params={"voice": "coral", "text": "Self test, quiet fallback."}, headers={"X-OpenAI-Key": ""}) as r:
        check("/tts never answers with an error status (204 = use the browser voice)", r.status in (200, 204), f"{r.status} {r.headers.get('X-TTS', '')}")
    if OFFLINE:
        for name in ("OpenAI TTS endpoint", "Gemini REST proxy reaches Google", "server reports Gemini key"):
            skip(name, "offline: needs an API key and the network")
    else:
        async with s.get(BASE + "/tts", params={"voice": "coral", "text": "Self test."}) as r:
            b = await r.read()
            check("OpenAI TTS endpoint", r.status == 200 and len(b) > 1000, f"{r.status} {r.headers.get('X-TTS', '')} {len(b)}B")

        async def gemini_rest():
            async with s.post(BASE + "/gemini/rest/gemini-3.8-flash:generateContent", json={"contents": [{"parts": [{"text": "Reply OK"}]}]}) as r:
                return r.status, await r.text()
        req = asyncio.ensure_future(gemini_rest())
        while not req.done():          # keep reading the socket so the server heartbeat doesn't drop us (Google can take 35 s)
            await pump(0.3)
        status, txt = req.result()
        check("Gemini REST proxy reaches Google", status in (200, 429) or "error" in txt, f"{status} {txt[:60]}")
        check("server reports Gemini key", st.get("server_gemini") is True)
    await send(type="demo", name="keyfeatures"); t0 = time.time(); kinds = set(); seen = None
    while time.time() - t0 < 120:
        await pump(0.3)
        sy = st.get("say")
        if sy and sy["id"] != seen:
            seen = sy["id"]; kinds.add(sy["kind"])
        if st.get("demo") is None and time.time() - t0 > 5:
            break
    check("demo keyfeatures runs to the end", st.get("demo") is None and {"narrator", "robot", "user", "caption"} <= kinds, f"{round(time.time() - t0)}s {sorted(kinds)}")
    await send(type="demo", name="loop"); await pump(1.0)
    check("demo loop starts", (st.get("demo") or {}).get("name") == "full")
    await send(type="demo", name="stop"); await pump(0.5)
    check("demo stop", st.get("demo") is None)
    await task("reset_scene", 1)

    # --- video upload -> imitation -> skill
    clip = Path("/tmp/pc_clip.mp4")
    if clip.exists():
        form = aiohttp.FormData(); form.add_field("video", open(clip, "rb"), filename="selftest clip.mp4", content_type="video/mp4")
        async with s.post(BASE + "/upload", data=form) as r:
            j = await r.json()
        await pump(1.5)
        playing = st.get("video") is not None and st["view"] == "stage"
        t0 = time.time()
        while st.get("video") is not None and time.time() - t0 < 20:
            await pump(0.5)
        check("video upload -> whole-body imitation -> skill", j["ok"] and playing and "selftest_clip" in st["skills"], j.get("name"))
    elif OFFLINE:
        skip("video upload -> whole-body imitation -> skill", "offline: no /tmp/pc_clip.mp4 test clip")
    else:
        check("video upload", False, "no /tmp/pc_clip.mp4 test clip")

    # --- feature checks: scripts/checks/*.py each define `async def run(t)` (UX 2.0 features add their own file)
    import importlib.util
    from types import SimpleNamespace
    t = SimpleNamespace(send=send, pump=pump, task=task, until_done=until_done, check=check, http=s, base=BASE,
                        state=lambda: st, frames=lambda: bins)
    for f in sorted((ROOT / "scripts" / "checks").glob("*.py")):
        if f.name.startswith("_"):
            continue
        spec = importlib.util.spec_from_file_location(f"checks_{f.stem}", f)
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        try:
            await mod.run(t)
        except Exception as e:
            check(f"checks/{f.name} ran", False, repr(e)[:120])
        await task("reset_scene", 1)

    await ws.close(); await s.close()


if __name__ == "__main__":
    log = open(os.environ["PC_SELFTEST_LOG"], "w") if os.environ.get("PC_SELFTEST_LOG") else subprocess.DEVNULL
    srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--https", "--port", str(PORT),
                            "--https-port", str(HTTPS)] + (["--byok"] if OFFLINE else []),
                           cwd=ROOT, stdout=log, stderr=subprocess.STDOUT if log is not subprocess.DEVNULL else subprocess.DEVNULL)
    try:
        asyncio.run(main())
    finally:
        srv.terminate()
        for f in ("selftest_wave", "selftest_task", "selftest_clip"):
            try:
                os.remove(ROOT / "recordings" / "skills" / f"{f}.json")
            except OSError:
                pass
        for f in list((ROOT / "skills").glob("selftest_*.json")) + list((ROOT / "skills").glob("p4check_*.json")):
            try:                         # P4: skills the checks' demonstrations became (skills/ is user data)
                f.unlink()
            except OSError:
                pass
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed" + (f", {len(skipped)} skipped (offline)" if skipped else ""))
    sys.exit(1 if bad else 0)
