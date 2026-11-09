"""PhysicsClay server: one process.

your camera (webcam / video / synthetic) -> MediaPipe pose (+ objects) -> follow mode / context
robot view (MuJoCo, real Unitree G1 + table + scanned mug) <- tasks (pick_place, reach, paths, skills) <- browser intents
Browser talks to Gemini Live itself (key stays in the browser); intents arrive here as tasks, the physics verdict goes back.
Protocol: docs/PROTOCOL.md.   Run: .venv/bin/python -m pps.app [--video clip.mp4] [--objects] [--fake] [--https]
"""
import argparse
import copy
import os
import asyncio
import json
import socket
import ssl
import subprocess
import threading
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
import aiohttp
from aiohttp import WSMsgType, web

from .fake import fake_landmarks
from .perceive import H, W, Objects, Pose, grip_state, holding
from .protocol import LANDMARKS
from .robot_runtime import HANDS, Runtime
from .compile_other import compile_ur5e
from .mock import MockBrain
from .spatial import ER_MODEL, Spatial, byok, gemini_key
from .teach import Teacher
from .demo import Demo
from .tasks import REGIONS, Tasks, apply_surface, edit_skill, skills_meta
from .world import profiles as world_profiles, scene as world_scene
from .world.runtime import make_runtime
from . import ext

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "web"
IK_ITERS, IK_DT = 4, 1 / 120
SIM_W, SIM_H = 640, 480
ASPECT = SIM_W / SIM_H


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.cam = self.sim = self.pip = None
        self.phone = None          # (time, jpeg bytes) from a browser/phone camera
        self.state = "{}"
        self.fid = 0
        self.cmds = []
        self.clients = set()
        self.running = True
        self.scene = None          # (model, /scene.json payload, mesh key) of the current world, set by the worker
        self.caps = ()             # current robot's capabilities (the upload endpoint checks imitate_video)

    def put(self, cam, sim, state):
        with self.lock:
            self.cam, self.sim, self.state = cam, sim, state
            self.fid += 1

    def get(self):
        with self.lock:
            return self.cam, self.sim, self.state, self.fid

    def push_cmd(self, c):
        with self.lock:
            self.cmds.append(c)

    def pop_cmds(self):
        with self.lock:
            c, self.cmds = self.cmds, []
            return c


# ------------------------------------------------------------------ server
async def ws_handler(request):
    ws = web.WebSocketResponse(max_msg_size=0, heartbeat=20)
    await ws.prepare(request)
    shared = request.app["shared"]
    shared.clients.add(ws)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    shared.push_cmd(json.loads(msg.data))
                except ValueError:
                    pass
            elif msg.type == WSMsgType.BINARY and msg.data[:1] == b"\x10":
                shared.phone = (time.time(), msg.data[1:])
            elif msg.type == WSMsgType.ERROR:
                break
    finally:
        shared.clients.discard(ws)
    return ws


async def broadcaster(shared):
    last = -1
    while True:
        await asyncio.sleep(1 / 30)
        cam, sim, state, fid = shared.get()
        if fid == last or cam is None:
            continue
        last = fid
        for ws in list(shared.clients):
            try:
                await ws.send_str(state)
                await ws.send_bytes(b"\x01" + cam)
                await ws.send_bytes(b"\x02" + sim)
                if shared.pip is not None:
                    await ws.send_bytes(b"\x03" + shared.pip)
            except Exception:
                shared.clients.discard(ws)


async def upload(request):
    reader = await request.multipart()
    field = await reader.next()
    while field is not None and field.name != "video":
        field = await reader.next()
    if field is None:
        return web.json_response({"ok": False, "message": "no 'video' field"}, status=400)
    if "imitate_video" not in request.app["shared"].caps:
        return web.json_response({"ok": False, "message": "Imitating a video needs a humanoid body (the Unitree G1). Switch robots first."},
                                 status=409)
    from urllib.parse import unquote
    stem = "".join(ch for ch in Path(unquote(field.filename or "video")).stem.lower().replace(" ", "_") if ch.isalnum() or ch in "_-")[:28] or "video"
    ext = (Path(field.filename or "v.mp4").suffix or ".mp4").lower()
    d = ROOT / "recordings" / "uploads"
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"{stem}{ext}"
    size = 0
    with open(path, "wb") as f:
        while True:
            chunk = await field.read_chunk(1 << 20)
            if not chunk:
                break
            size += len(chunk)
            if size > 300 * (1 << 20):
                return web.json_response({"ok": False, "message": "video too large (300 MB max)"}, status=413)
            f.write(chunk)
    request.app["shared"].push_cmd({"type": "source", "path": str(path), "name": stem})
    return web.json_response({"ok": True, "name": stem, "message": f"Imitating '{stem}'. It becomes a skill when the video ends."})


def scene_payload(m, world=None, key=None):
    """/scene.json: every visible geom of the current world (+ the work surface, so the 3D view can raycast it)."""
    if world is not None and not world.legacy:
        out = world_payload(m, world, key)
    else:
        out = legacy_payload(m)
    s = world.surface if world is not None else world_scene.surface("table")
    out["surface"] = {"z": s["top"], "x0": s["x"][0], "x1": s["x"][1], "y0": s["y"][0], "y1": s["y"][1]}
    return out


GEOM_KINDS = {int(k): v for k, v in {
    mujoco.mjtGeom.mjGEOM_PLANE: "plane", mujoco.mjtGeom.mjGEOM_BOX: "box", mujoco.mjtGeom.mjGEOM_MESH: "mesh",
    mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder", mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
    mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule", mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid"}.items()}


def world_payload(m, world, key):
    """Any robot / scene: visual geoms (groups 0-2, not the IK target ghosts); robot meshes are served per world."""
    geoms, meshes = [], {}
    d = mujoco.MjData(m)
    if world.home_q is not None:
        d.qpos[:] = world.home_q
    mujoco.mj_forward(m, d)
    helper = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith("_target")}
    for g in range(m.ngeom):
        grp, typ, b = int(m.geom_group[g]), int(m.geom_type[g]), int(m.geom_bodyid[g])
        if grp not in (0, 1, 2) or typ not in GEOM_KINDS or b in helper:
            continue
        rgba = m.geom_rgba[g].tolist()
        mat = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, m.geom_matid[g]) if m.geom_matid[g] >= 0 else None
        if mat == "wood":
            rgba = [0.66, 0.48, 0.31, 1.0]
        elif mat == "pc_mug":
            rgba = [0.70, 0.18, 0.20, 1.0]
        elif mat is not None:
            rgba = m.mat_rgba[m.geom_matid[g]].tolist()
        if typ == int(mujoco.mjtGeom.mjGEOM_PLANE):
            rgba = [0.92, 0.92, 0.94, 1.0]
        if rgba[3] <= 0.01:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        mid = int(m.geom_dataid[g]) if typ == int(mujoco.mjtGeom.mjGEOM_MESH) else None
        if mid is not None:
            meshes[str(mid)] = {"url": f"/mesh/{mid}.bin?w={key}"}
        geoms.append({"id": int(g), "name": name, "kind": GEOM_KINDS[typ], "mesh": None if mid is None else str(mid),
                      "size": [round(float(x), 5) for x in m.geom_size[g]], "rgba": [round(float(x), 3) for x in rgba],
                      "pos": [round(float(x), 5) for x in d.geom_xpos[g]], "mat": [round(float(x), 6) for x in d.geom_xmat[g]],
                      "dynamic": bool(b != 0)})
    return {"geoms": geoms, "meshes": meshes, "world": key}


def legacy_payload(m):
    geoms, meshes = [], {}
    kinds = GEOM_KINDS
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mug = m.body("mug").id
    for g in range(m.ngeom):
        grp, typ = int(m.geom_group[g]), int(m.geom_type[g])
        if grp not in (0, 2) or typ not in kinds:
            continue
        if grp == 0 and m.geom_bodyid[g] != 0:
            continue                                  # group-0 bits on bodies are ghosts / helpers
        rgba = m.geom_rgba[g].tolist()
        if m.geom_matid[g] >= 0:
            rgba = m.mat_rgba[m.geom_matid[g]].tolist()
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if m.geom_bodyid[g] == mug:
            rgba = [0.70, 0.18, 0.20, 1.0]
        elif typ == int(mujoco.mjtGeom.mjGEOM_BOX) and m.geom_bodyid[g] == 0:
            rgba = [0.66, 0.48, 0.31, 1.0]
        elif typ == int(mujoco.mjtGeom.mjGEOM_PLANE):
            rgba = [0.92, 0.92, 0.94, 1.0]
        mid = int(m.geom_dataid[g]) if typ == int(mujoco.mjtGeom.mjGEOM_MESH) else None
        if mid is not None:
            meshes[str(mid)] = {"url": f"/mesh/{mid}.bin"}
        geoms.append({"id": int(g), "name": name, "kind": kinds[typ], "mesh": None if mid is None else str(mid),
                      "size": [round(float(x), 5) for x in m.geom_size[g]], "rgba": [round(float(x), 3) for x in rgba],
                      "pos": [round(float(x), 5) for x in d.geom_xpos[g]], "mat": [round(float(x), 6) for x in d.geom_xmat[g]],
                      "dynamic": bool(m.geom_bodyid[g] != 0)})
    return {"geoms": geoms, "meshes": meshes}


async def scene_json(request):
    cur = request.app["shared"].scene
    return web.json_response(cur[1] if cur is not None else request.app["scene"])


async def mesh_bin(request):
    m = request.app["model"]                 # no ?w= key: the UX 2.0 world's meshes (cached by browsers for a day)
    w = request.query.get("w")
    if w:
        cur = request.app["shared"].scene
        if cur is None or cur[2] != w:
            raise web.HTTPNotFound()         # a mesh of a world that's gone
        m = cur[0]
    try:
        i = int(request.match_info["mid"])
        va, vn, fa, fn = m.mesh_vertadr[i], m.mesh_vertnum[i], m.mesh_faceadr[i], m.mesh_facenum[i]
    except (ValueError, IndexError):
        raise web.HTTPNotFound()
    verts = np.ascontiguousarray(m.mesh_vert[va:va + vn], dtype="<f4")
    faces = np.ascontiguousarray(m.mesh_face[fa:fa + fn], dtype="<u4")
    body = np.array([vn, fn], dtype="<u4").tobytes() + verts.tobytes() + faces.tobytes()
    resp = web.Response(body=body, content_type="application/octet-stream", headers={"Cache-Control": "max-age=86400"})
    resp.enable_compression()   # ~5 MB of meshes: compress for the public tunnel (upload bandwidth is the bottleneck)
    return resp


import certifi  # noqa: E402

SSL_CTX = ssl.create_default_context(cafile=certifi.where())
GEMINI_WS = "wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent"


async def gemini_live_proxy(request):
    """Browser <-> our server <-> Gemini Live. The key never leaves the server."""
    key = gemini_key()
    ws = web.WebSocketResponse(max_msg_size=0)
    await ws.prepare(request)
    if not key:
        await ws.close(code=4001, message=b"no server key")
        return ws
    async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=SSL_CTX)) as sess:
        try:
            up = await sess.ws_connect(GEMINI_WS + "?key=" + key, max_msg_size=0)
        except Exception as e:
            await ws.close(code=4002, message=str(e)[:100].encode())
            return ws

        async def down():
            async for m in up:
                if m.type == WSMsgType.TEXT:
                    await ws.send_str(m.data)
                elif m.type == WSMsgType.BINARY:
                    await ws.send_bytes(m.data)
                else:
                    break
            if not ws.closed:
                await ws.close(code=up.close_code or 1000, message=(str(up.exception() or "") or "upstream closed")[:100].encode())

        task = asyncio.create_task(down())
        async for m in ws:
            if up.closed:
                break
            if m.type == WSMsgType.TEXT:
                await up.send_str(m.data)
            elif m.type == WSMsgType.BINARY:
                await up.send_bytes(m.data)
        await up.close()
        task.cancel()
    return ws


async def gemini_rest_proxy(request):
    key = gemini_key()
    if not key:
        return web.json_response({"error": {"message": "no server key"}}, status=503)
    tail = request.match_info["tail"]
    body = await request.read()
    try:
        async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=SSL_CTX), timeout=aiohttp.ClientTimeout(total=35)) as sess:
            async with sess.post("https://generativelanguage.googleapis.com/v1beta/models/" + tail, data=body,
                                 headers={"Content-Type": "application/json", "x-goog-api-key": key}) as r:
                return web.Response(body=await r.read(), status=r.status, content_type="application/json")
    except asyncio.TimeoutError:
        return web.json_response({"error": {"code": 504, "message": "Gemini did not answer within 35 s (upstream busy)"}}, status=504)


TTS_DIR = ROOT / "recordings" / "tts"


def openai_key():
    if byok():
        return None
    env = ROOT / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.startswith("OPENAI_API_KEY="):
                return line.split("=", 1)[1].strip()
    return None


async def tts_handler(request):
    """OpenAI TTS with a disk cache; the browser plays the mp3 instead of the choppy built-in voice."""
    import hashlib
    text = (request.query.get("text") or "")[:600]
    voice = request.query.get("voice") or "coral"
    if not text:
        raise web.HTTPNotFound()
    TTS_DIR.mkdir(parents=True, exist_ok=True)
    f = TTS_DIR / (hashlib.sha1((voice + "|" + text).encode()).hexdigest() + ".mp3")
    # BYOK: the visitor's own key comes in a header; it is used for this one request and never stored or logged
    key = (request.headers.get("X-OpenAI-Key") or "").strip() or openai_key()
    if not f.exists() and not key:
        raise web.HTTPNotFound()   # no key: the browser falls back to its built-in voice (cached lines still play)
    if not f.exists():
        body = {"model": "gpt-4o-mini-tts", "voice": voice, "input": text, "response_format": "mp3",
                "instructions": "A friendly, natural humanoid robot assistant; calm, clear and precise." if voice == "coral"
                else "A confident startup founder giving a crisp product demo; warm and energetic."}
        async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=SSL_CTX)) as sess:
            async with sess.post("https://api.openai.com/v1/audio/speech", json=body,
                                 headers={"Authorization": "Bearer " + key}) as r:
                if r.status != 200:
                    raise web.HTTPBadGateway(text=(await r.text())[:200])
                f.write_bytes(await r.read())
    return web.FileResponse(f, headers={"Cache-Control": "max-age=86400", "Content-Type": "audio/mpeg"})


async def index(request):
    return web.FileResponse(WEB / "index.html", headers={"Cache-Control": "no-store"})


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


async def ui_modules(request):
    """Front-end feature modules (web/ui/*.js, core.js first) so features never edit index.html."""
    names = sorted(p.name for p in (WEB / "ui").glob("*.js") if p.name != "core.js")
    return web.json_response(["/static/ui/" + n for n in names])


def ensure_cert():
    d = ROOT / "certs"
    d.mkdir(exist_ok=True)
    cert, key = d / "cert.pem", d / "key.pem"
    if not cert.exists():
        subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-keyout", str(key), "-out", str(cert),
                        "-days", "30", "-subj", "/CN=physicsclay"], check=True, capture_output=True)
    ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ctx.load_cert_chain(str(cert), str(key))
    return ctx


def run_server(shared, port, https_port):
    async def main():
        app = web.Application(client_max_size=320 * (1 << 20))
        app["shared"] = shared
        app.router.add_get("/", index)
        app.router.add_get("/ws", ws_handler)
        app.router.add_post("/upload", upload)
        app.router.add_get("/gemini/live", gemini_live_proxy)
        app.router.add_get("/tts", tts_handler)
        app.router.add_post("/gemini/rest/{tail:.*}", gemini_rest_proxy)
        app.router.add_get("/scene.json", scene_json)
        app.router.add_get("/mesh/{mid}.bin", mesh_bin)
        app.router.add_get("/ui/modules.json", ui_modules)
        sm = mujoco.MjModel.from_xml_path(str(ROOT / "assets" / "unitree_g1" / "scene_pc.xml"))
        app["model"], app["scene"] = sm, scene_payload(sm)
        app.router.add_static("/static", WEB)
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        await web.TCPSite(runner, "0.0.0.0", port).start()
        if https_port:
            await web.TCPSite(runner, "0.0.0.0", https_port, ssl_context=ensure_cert()).start()
        asyncio.create_task(broadcaster(shared))
        while True:
            await asyncio.sleep(3600)

    asyncio.run(main())


# ------------------------------------------------------------------ worker (main thread: GL + camera)
def world_target(c, world):
    """{type:"world", op} -> (robot id, scene spec) to build, or raise ValueError."""
    op = c.get("op", "set")
    robot, spec = world.profile.id, copy.deepcopy(world.spec)
    if op == "set":
        if c.get("robot") is not None:
            p = world_profiles.get(c.get("robot"))
            if p is None:
                raise ValueError(f"I don't know the robot '{c.get('robot')}'. I know: {', '.join(world_profiles.PROFILES)}.")
            robot = p.id
        if isinstance(c.get("scene"), dict):
            spec = c["scene"]
        elif isinstance(c.get("template"), str):
            spec["template"] = c["template"]
    elif op == "add_object":
        o = c.get("object") if isinstance(c.get("object"), dict) else {k: c[k] for k in ("name", "kind", "color", "size", "pos", "movable") if k in c}
        if not o:
            raise ValueError("add_object needs an object, e.g. {kind: 'cup', color: 'red'}.")
        spec["objects"] = list(spec["objects"]) + [o]
    elif op == "remove_object":
        key = world_scene._slug(c.get("name"))
        keep = [o for o in spec["objects"] if o["name"] != key]
        if len(keep) == len(spec["objects"]):
            keep = [o for o in spec["objects"] if key not in o["name"]] if key else keep
        if len(keep) == len(spec["objects"]):
            raise ValueError(f"There is no '{c.get('name')}' in the scene. Objects: {', '.join(o['name'] for o in spec['objects']) or 'none'}.")
        spec["objects"] = keep
    elif op == "reset":
        robot, spec = world_profiles.DEFAULT_ROBOT, copy.deepcopy(world_scene.DEFAULT_SCENE)
    else:
        raise ValueError(f"Unknown world op '{op}'. Use set, add_object, remove_object or reset.")
    return robot, spec


def world_state(world, rev):
    return {"rev": rev, "robot": world.profile.public(), "scene": world.spec, "robots": world_profiles.listing(),
            "templates": world_scene.templates(), "surface": {"top": world.surface["top"], "x": list(world.surface["x"]),
                                                              "y": list(world.surface["y"])},
            "regions": {k: list(v) for k, v in world.regions.items()}, "notes": world.notes}


def world_message(world):
    p = world.profile
    objs = [o["name"].replace("_", " ") for o in world.spec["objects"]]
    msg = f"Now {p.label} at the {world_scene.TEMPLATES[world.spec['template']]['label'].lower()}"
    msg += f" with {', '.join(objs)}." if objs else ", nothing on it yet."
    if p.experimental:
        msg += " Experimental: " + p.note
    if world.notes:
        msg += " (" + "; ".join(world.notes) + ")"
    return msg


def worker(args, shared):
    world = world_scene.build(None, None)                # the UX 2.0 world: G1 + today's table (scene_pc.xml)
    world_rev = [1]
    apply_surface(world.surface, world.regions)
    rt = make_runtime(world, mirror=not args.no_mirror)
    tasks = Tasks(rt, {o["name"]: o["kind"] for o in world.spec["objects"]})
    spatial = Spatial()
    teacher = Teacher(tasks)
    TEACHER[0] = teacher
    brain = MockBrain()
    brain.enabled = args.mock
    video_flag = [False]
    demo = Demo(tasks, teacher, brain, rt, push=shared.push_cmd, video_active=lambda: video_flag[0], er=lambda: spatial.er)
    ext.load()
    ctx = ext.make_ctx(rt=rt, tasks=tasks, teacher=teacher, brain=brain, demo=demo, spatial=spatial, aspect=ASPECT, push=shared.push_cmd)
    ctx.world = world
    payload = scene_payload(rt.model, world, f"{world.profile.id}-{world_rev[0]}")
    shared.scene, shared.caps = (rt.model, payload, f"{world.profile.id}-{world_rev[0]}"), tuple(rt.capabilities)
    dyn_geoms = [gm["id"] for gm in payload["geoms"] if gm["dynamic"]]
    m, d = rt.model, rt.data
    renderer = mujoco.Renderer(m, SIM_H, SIM_W)
    pip_renderer = mujoco.Renderer(m, 300, 400)
    robot_cam = m.camera("robot_view").id
    orbit = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, orbit)
    orbit.azimuth, orbit.elevation, orbit.distance = 165, -20, 2.2
    orbit.lookat[:] = (0.35, 0.0, 0.9)
    stage = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, stage)
    stage.azimuth, stage.elevation, stage.distance = 240, -12, 2.4
    stage.lookat[:] = (0.22, 0.0, 0.62)
    full = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, full)
    full.azimuth, full.elevation, full.distance = 240, -12, 2.4
    full.lookat[:] = (0.22, 0.0, 0.62)
    view = "robot"
    cam_params = None

    def switch_world(robot, spec, tid="world"):
        """Rebuild the world in place (robot and/or scene): runtime, tasks, teacher, demo, renderers, /scene.json."""
        nonlocal world, rt, tasks, teacher, demo, m, d, renderer, pip_renderer, robot_cam, dyn_geoms, payload
        if video_flag[0]:
            tasks.set_task(tid, "world", "failed", "I'm imitating your video right now. Let it finish first.")
            return False
        try:
            nw = world_scene.build(robot, spec)
        except ValueError as e:
            tasks.set_task(tid, "world", "failed", f"I couldn't build that world: {e}.")
            return False
        if demo.steps:
            demo.stop()
        apply_surface(nw.surface, nw.regions)
        old_keepout = tasks.keepout
        world, world_rev[0] = nw, world_rev[0] + 1
        rt = make_runtime(nw, mirror=not args.no_mirror)
        tasks = Tasks(rt, {o["name"]: o["kind"] for o in nw.spec["objects"]})
        tasks.keepout = old_keepout                       # the zones feature's hook keeps working (zones are cleared below)
        teacher = Teacher(tasks)
        TEACHER[0] = teacher
        demo = Demo(tasks, teacher, brain, rt, push=shared.push_cmd, video_active=lambda: video_flag[0], er=lambda: spatial.er)
        ctx.rt, ctx.tasks, ctx.teacher, ctx.demo, ctx.world = rt, tasks, teacher, demo, nw
        for k in ("zones",):                              # scene-bound feature state: positions of the old world
            if isinstance(ctx.store.get(k), list):
                ctx.store[k].clear()
        for k in ("plan", "choices"):
            ctx.store.pop(k, None)
        if isinstance(ctx.store.get("goal"), dict):
            ctx.store.pop("goal", None)
        m, d = rt.model, rt.data
        renderer.close()
        pip_renderer.close()
        renderer = mujoco.Renderer(m, SIM_H, SIM_W)
        pip_renderer = mujoco.Renderer(m, 300, 400)
        robot_cam = m.camera("robot_view").id
        key = f"{nw.profile.id}-{world_rev[0]}"
        payload = scene_payload(m, nw, key)
        shared.scene, shared.caps = (m, payload, key), tuple(rt.capabilities)
        dyn_geoms = [gm["id"] for gm in payload["geoms"] if gm["dynamic"]]
        tasks.set_task(tid, "world", "done", world_message(nw))
        return True

    def lacks(cap, tid, action, what):
        """Humanoid-only features on other bodies: a clear refusal instead of a broken attempt."""
        if cap in rt.capabilities:
            return False
        tasks.set_task(tid, action, "failed", f"{what} needs a humanoid body: the {world.profile.label} can't do that. "
                       "Switch to the Unitree G1 for it.")
        return True

    pose = None if args.fake else Pose(str(ROOT / args.model))
    objects = Objects(str(ROOT / args.objects_model)) if (args.objects and not args.fake) else None
    cap, video_dt, source = None, 0.0, "synthetic"
    if args.video:
        cap = cv2.VideoCapture(args.video)
        video_dt, source = 1.0 / (cap.get(cv2.CAP_PROP_FPS) or 30.0), "video"
    elif args.webcam and not args.fake:   # the laptop webcam stays off unless asked for
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        source = "webcam"

    sim_jpg_last = [None]
    video_job = None   # {"name", "frames", "i", "prev_cap", "prev_dt", "prev_source", "prev_view"}
    grip_hist = {"l": [], "r": []}
    t_start, n, fps, t_fps, next_frame_t = time.time(), 0, 0.0, time.time(), time.time()
    t_phys = time.time()
    ip = lan_ip()
    print(f"PhysicsClay  source={source}\n  laptop: http://127.0.0.1:{args.port}\n  phone : http://{ip}:{args.port}"
          + (f"   (mic/camera on phone: https://{ip}:{args.https_port}, accept the certificate warning)" if args.https else ""),
          flush=True)

    while shared.running:
        # ---------------- your camera (a phone/browser camera wins while it streams)
        phone = shared.phone if (shared.phone and time.time() - shared.phone[0] < 1.0 and video_job is None) else None
        if phone is not None:
            frame = cv2.imdecode(np.frombuffer(phone[1], np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            frame = cv2.resize(frame, (W, H))
            if pose is None:
                pose = Pose(str(ROOT / args.model))
            pts, lm = pose(frame)
            source = "phone"
            time.sleep(0.02)
        elif cap is not None:
            if args.video:
                now = time.time()
                if now < next_frame_t:
                    time.sleep(next_frame_t - now)
                next_frame_t = max(next_frame_t + video_dt, time.time() - video_dt)
            if video_job is not None:          # keep real time: skip frames when processing is slower than the video
                behind = int((time.time() - video_job["t0"]) / video_dt) - video_job["i"]
                for _ in range(max(0, min(behind - 1, 8))):
                    if cap.grab():
                        video_job["i"] += 1
            ok, frame = cap.read()
            if ok and video_job is not None:
                video_job["i"] += 1
                if video_job["i"] > video_job["max_frames"]:
                    ok = False
            if not ok:
                if video_job is not None:          # uploaded video finished: save the imitation as a skill
                    tasks.follow_me("vf", False)
                    tasks.learn_skill("video", video_job["name"])
                    if tasks.task and tasks.task["status"] == "done":
                        tasks.task["message"] += f" Say 'do {video_job['name']}' to replay it."
                    cap.release()
                    cap, video_dt, source = video_job["prev_cap"], video_job["prev_dt"], video_job["prev_source"]
                    view = video_job["prev_view"]
                    args.video = video_job["prev_video"]
                    video_job = None
                    continue
                if args.video:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                time.sleep(0.02)
                continue
            if frame.shape[1] != W or frame.shape[0] != H:
                frame = cv2.resize(frame, (W, H))
            pts, lm = pose(frame)
        else:
            frame = np.full((H, W, 3), 40, np.uint8)
            cv2.putText(frame, "synthetic input (no camera)", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (160, 160, 160), 2)
            lm = fake_landmarks(time.time() - t_start)
            pts = [(W / 2, H / 2)] * 33
            for name, i in LANDMARKS.items():
                if name in lm:
                    pts[i] = (W / 2 - lm[name][0] * 700, H * 0.55 + lm[name][1] * 700)
        if source == "phone" and phone is None:
            source = "webcam" if cap is not None and not args.video else ("video" if args.video else "synthetic")
        scene = objects(frame) if objects is not None else []
        spatial.paused = bool(demo.steps) and video_job is None
        if n % 4 == 0 and not spatial.paused:
            spatial.feed(frame)
        hold = holding(scene, pts)
        grip = {}
        for s in ("l", "r"):
            g = grip_state(lm, s) if lm else None
            hist = grip_hist[s]
            hist.append(g if g is not None else 0)
            del hist[:-5]
            grip[s] = 1 if (sum(hist) >= 3 or hold.get(s)) else 0
        if lm:
            rt.last_lm, rt.new_pose = lm, True
        rt.last_grip = grip
        rt.last_context = {"holding": hold, "scene": sorted({o[0] for o in scene}), "source": source}

        # ---------------- intents from the browser
        ctx.cam_params, ctx.frame, ctx.view = cam_params, frame, view
        ctx.lm, ctx.pts, ctx.source = lm, pts, source   # pose landmarks for features (gestures)
        ctx.dispatch = lambda msg, _cp=cam_params: dispatch(tasks, rt, msg, _cp)
        for c in shared.pop_cmds():
            t = c.get("type")
            try:
                if t in ext.HANDLERS:
                    ext.handle(ctx, c)
                elif t == "tap" and cam_params is not None:
                    hit = tasks.raycast(cam_params, float(c["u"]), float(c["v"]), ASPECT)
                    if hit is not None:
                        mid = tasks.add_marker(hit)
                        rt.path_msg = f"Marker {mid} placed. Say 'put the mug on {mid}'."
                    else:
                        rt.path_msg = "That tap missed the scene."
                elif t == "world":
                    try:
                        robot, spec = world_target(c, world)
                    except ValueError as e:
                        tasks.set_task(str(c.get("id", "world")), "world", "failed", str(e))
                        continue
                    switch_world(robot, spec, str(c.get("id", "world")))
                elif t == "teach" and c.get("cmd") == "start" and lacks("teach_by_showing", "teach", "teach", "Teaching by showing"):
                    continue
                elif t == "teach":
                    cmd = c.get("cmd")
                    if cmd == "start":
                        teacher.start()
                        brain.utter("I'm watching. Show me the task and tell me what you're doing.", "robot")
                    elif cmd == "utterance":
                        if teacher.phase == "recording":
                            teacher.utter(c.get("text"))
                        elif teacher.phase in ("confirm", "done"):
                            teacher.correct(c.get("text"))
                    elif cmd == "stop":
                        teacher.stop()
                    elif cmd == "correct":
                        teacher.correct(c.get("text"))
                    elif cmd == "practice":
                        teacher.practice()
                    elif cmd == "save":
                        teacher.save(c.get("name"))
                    elif cmd == "set_spec":
                        teacher.set_spec(c.get("spec") or {})
                    elif cmd == "cancel":
                        teacher.phase = "idle"
                elif t == "demo":
                    if c.get("name") == "stop":
                        demo.stop()
                    elif not world.legacy:
                        tasks.set_task("demo", "demo", "failed", "The scripted demos run in the default world (the G1 at the "
                                       "wooden table). Reset the world first.")
                    else:
                        brain.enabled = True
                        demo.start(c.get("name", "full"))
                elif t == "robot":   # UX 2.0 robot switch: now a real switch of the body (the scene stays)
                    p = world_profiles.get(c.get("name")) or world_profiles.get("g1")
                    verdict = ""
                    if p.id == "ur5e" and world.profile.id != "ur5e":   # first compile what this body just did for the arm
                        dispatch(tasks, rt, {"type": "task", "id": f"robot-{time.time():.0f}", "action": "compile_for", "robot": "ur5e"}, cam_params)
                        verdict, vstatus = (tasks.task or {}).get("message", ""), (tasks.task or {}).get("status", "done")
                    if p.id != world.profile.id:
                        tid = f"robot-{time.time():.0f}"
                        if switch_world(p.id, world.spec, tid) and verdict:
                            tasks.set_task(tid, "compile_for", vstatus, f"{verdict} {tasks.task['message']}")
                    if verdict:
                        brain.utter("Same task, different body: " + verdict, "robot")
                elif t == "spatial_model":
                    spatial.set_model(c.get("name"))
                elif t == "spatial":
                    if c.get("target") == "robot":
                        spatial.ask_er(cv2.imdecode(np.frombuffer(sim_jpg_last[0], np.uint8), cv2.IMREAD_COLOR) if sim_jpg_last[0] else frame,
                                       "robot", c.get("query"), key=c.get("key"))
                    else:
                        spatial.ask_er(frame, "camera", c.get("query"), mirrored=True, key=c.get("key"))
                elif t == "path3d":
                    tasks.set_path_world(c.get("points") or [], c.get("hand") if c.get("hand") in ("left", "right") else None)
                elif t == "mock":
                    brain.enabled = bool(c.get("enabled"))
                    if tasks.task:
                        brain._last_task = (tasks.task.get("id"), tasks.task.get("status"))
                    if brain.enabled:
                        brain.utter("Mock AI on: point at the table, raise both hands to stop, or wave.", "vision")
                elif t == "draw" and cam_params is not None:
                    tasks.draw(c.get("points") or [], cam_params, ASPECT, c.get("hand"))
                elif t == "clear":
                    tasks.markers, tasks.next_marker = [], 0
                    if rt.mode != "FOLLOW":
                        rt.path = None
                    rt.path_msg = None
                elif t == "view":
                    view = c.get("name") if c.get("name") in ("robot", "orbit", "stage") else "robot"
                elif t == "source" and lacks("imitate_video", "video", "video", "Imitating a video"):
                    continue
                elif t == "source":
                    vc = cv2.VideoCapture(c["path"])
                    if not vc.isOpened():
                        tasks.set_task("video", "video", "failed", "I couldn't open that video.")
                        continue
                    vfps = vc.get(cv2.CAP_PROP_FPS) or 30.0
                    nfr = int(vc.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
                    video_job = {"name": c.get("name", "video"), "i": 0, "frames": nfr, "t0": time.time(),
                                 "max_frames": int(min(nfr or 10 ** 9, vfps * min(90.0, float(c.get("seconds") or 90.0)))),
                                 "prev_cap": cap if video_job is None else video_job["prev_cap"],
                                 "prev_dt": video_dt if video_job is None else video_job["prev_dt"],
                                 "prev_source": source if video_job is None else video_job["prev_source"],
                                 "prev_view": view if video_job is None else video_job["prev_view"],
                                 "prev_video": args.video if video_job is None else video_job["prev_video"]}
                    if cap is not None and cap is not video_job["prev_cap"]:
                        cap.release()
                    cap, video_dt, source = vc, 1.0 / vfps, "video"
                    args.video = c["path"]
                    next_frame_t = time.time()
                    view = "stage"
                    if pose is None:
                        pose = Pose(str(ROOT / args.model))
                    tasks.home("vh")
                    tasks.follow_me("video", True)
                    tasks.set_task("video", "video", "running", f"Imitating '{video_job['name']}' from your video, whole body.")
                elif t == "camera":
                    if view == "stage":
                        stage.azimuth += float(c.get("dazimuth", 0.0))
                        stage.elevation = float(np.clip(stage.elevation + float(c.get("delevation", 0.0)), -85, 10))
                        stage.distance = float(np.clip(stage.distance * float(c.get("zoom", 1.0)), 0.8, 6.0))
                        continue
                    orbit.azimuth += float(c.get("dazimuth", 0.0))
                    orbit.elevation = float(np.clip(orbit.elevation + float(c.get("delevation", 0.0)), -85, 10))
                    orbit.distance = float(np.clip(orbit.distance * float(c.get("zoom", 1.0)), 0.8, 6.0))
                elif t == "task":
                    if not ext.intercept_task(ctx, c):
                        dispatch(tasks, rt, c, cam_params)
            except Exception as e:  # never let one bad intent kill the loop
                tasks.set_task(c.get("id", "?"), c.get("action", t), "failed", f"Internal error: {e}")

        # ---------------- physics (real time: the step size follows the wall clock)
        now = time.time()
        dt = float(np.clip(now - t_phys, 1 / 240, 1 / 12)) / IK_ITERS
        t_phys = now
        for _ in range(IK_ITERS):
            rt.step(dt)
            tasks.tick()
        teacher.observe(pts if lm else None, grip, [o[0] for o in scene])
        teacher.tick()
        demo.tick()
        ext.run_ticks(ctx)
        brain.update(lm, rt, tasks, busy=(teacher.phase == "recording" or video_job is not None or bool(demo.steps)
                                          or source == "synthetic"))   # no camera: the synthetic dancer is not a person gesturing
        if not demo.steps and not ctx.store.get("llm_brain"):   # the real agent speaks for itself
            brain.narrate(tasks.task)

        # ---------------- robot view
        rt.paint_model()
        if view == "robot":
            renderer.update_scene(d, camera=robot_cam)
            fovy = float(m.cam_fovy[robot_cam])
        else:
            renderer.update_scene(d, stage if view == "stage" else orbit)
            fovy = float(m.vis.global_.fovy)
        gl = renderer.scene.camera[0]
        fwd, up = np.array(gl.forward, float), np.array(gl.up, float)
        cam_params = (np.array(gl.pos, float), fwd / np.linalg.norm(fwd), up / np.linalg.norm(up), fovy)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        rt.paint_overlays(renderer.scene, reset=False)
        ext.paint_overlays(ctx, renderer.scene)
        sim = renderer.render()
        sim_jpg = cv2.imencode(".jpg", cv2.cvtColor(sim, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 82])[1].tobytes()
        sim_jpg_last[0] = sim_jpg
        # picture-in-picture: the whole robot when the main view is its eye, otherwise the robot's eye
        if view == "robot":
            pip_renderer.update_scene(d, full)
        else:
            pip_renderer.update_scene(d, camera=robot_cam)
        pip_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        pip_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        rt.paint_overlays(pip_renderer.scene, reset=False)
        ext.paint_overlays(ctx, pip_renderer.scene)
        shared.pip = cv2.imencode(".jpg", cv2.cvtColor(pip_renderer.render(), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 78])[1].tobytes()
        shown = cv2.resize(cv2.flip(frame, 1), (960, 540))
        cam_jpg = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 72])[1].tobytes()

        n += 1
        if n % 10 == 0:
            now = time.time()
            fps, t_fps = 10 / max(now - t_fps, 1e-6), now
        st = state(rt, tasks, view, cam_params, fps, source, pts, lm, scene, grip, hold)
        st["say"], st["gesture"], st["mock"] = brain.say, brain.gesture, brain.enabled
        st["teach"] = teacher.state()
        st["demo"] = demo.state()
        video_flag[0] = video_job is not None
        st["robot"] = world.profile.id                     # UX 2.0 aliases of state.world
        st["robots"] = world_profiles.listing()
        st["world"] = world_state(world, world_rev[0])
        st["spatial"] = spatial.state()
        st["server_gemini"] = gemini_key() is not None
        st["byok"] = byok()
        st["geom_poses"] = {str(g): [round(float(x), 4) for x in d.geom_xpos[g]] + [round(float(x), 4) for x in d.geom_xmat[g]] for g in dyn_geoms}
        if st["path"] is not None:
            st["path"]["pts"] = [[round(float(x), 3) for x in p] for p in rt.path["pts"]]
        st.update(ext.extra_state(ctx))
        st["video"] = None if video_job is None else {"name": video_job["name"], "progress": round(min(1.0, video_job["i"] / max(1, video_job["max_frames"])), 3)}
        shared.put(cam_jpg, sim_jpg, json.dumps(st))
        if args.frames and n >= args.frames:
            shared.running = False
    if cap is not None:
        cap.release()


TEACHER = [None]


def SKILL_FILE(key):
    from .tasks import SKILL_DIR
    return SKILL_DIR / f"{key}.json"


HUMANOID_ONLY = {"follow_me": ("follow_me", "Following you"), "learn_skill": ("follow_me", "Learning a motion skill"),
                 "edit_skill": ("follow_me", "Editing a motion skill")}


def dispatch(tasks, rt, c, cam):
    tid, a = str(c.get("id", "t?")), c.get("action")
    caps = getattr(rt, "capabilities", ())
    need = HUMANOID_ONLY.get(a)
    if a == "play_skill":
        key = str(c.get("name") or "").strip().lower().replace(" ", "_")
        f = SKILL_FILE(key)
        taught = f.exists() and json.load(open(f)).get("type") == "task"
        need = ("teach_by_showing", "Replaying a task taught by showing") if taught else ("follow_me", "Replaying a motion skill")
    if need and need[0] not in caps and not (a == "follow_me" and not c.get("enabled", True)):
        label = getattr(getattr(rt, "profile", None), "label", "this robot")
        tasks.set_task(tid, a, "failed", f"{need[1]} needs a humanoid body: the {label} can't do that. Switch to the Unitree G1 for it.")
        return
    if a == "pick_place":
        tasks.pick_place(tid, c.get("object", "mug"), c.get("target"), c.get("target_uv"), cam, ASPECT)
    elif a == "reach":
        tasks.reach(tid, c.get("target"), c.get("target_uv"), cam, ASPECT)
    elif a == "follow_path":
        tasks.follow_path(tid)
    elif a == "follow_me":
        tasks.follow_me(tid, bool(c.get("enabled", True)))
    elif a == "learn_skill":
        tasks.learn_skill(tid, c.get("name"))
    elif a == "play_skill":
        key = str(c.get("name") or "").strip().lower().replace(" ", "_")
        f = SKILL_FILE(key)
        if f.exists() and json.load(open(f)).get("type") == "task":
            TEACHER[0].practice(json.load(open(f))["spec"])
            tasks.set_task(tid, a, "running", f"Running the task '{key}' you taught me.")
        else:
            tasks.play_skill(tid, c.get("name"))
    elif a == "set_speed":
        v = c.get("value", "slower")
        if isinstance(v, str) and "slow" in v.lower():
            rt.speed_scale = max(0.1, rt.speed_scale * 0.7)
        elif isinstance(v, str) and "fast" in v.lower():
            rt.speed_scale = min(1.0, rt.speed_scale / 0.7)
        else:
            rt.speed_scale = float(np.clip(float(v), 0.1, 1.0))
        tasks.set_task(tid, a, "done", f"Speed set to x{rt.speed_scale:.1f}.")
    elif a == "edit_skill":
        edit_skill(tasks, tid, c.get("name"), c.get("t0"), c.get("t1"), c.get("part", "body"), c.get("speed"), c.get("amplitude"))
    elif a == "compile_for":
        if str(c.get("robot", "ur5e")).lower().replace("-", "") not in ("ur5e", "ur5", "arm"):
            tasks.set_task(tid, a, "failed", "I can compile for: ur5e.")
            return
        if rt.path is not None:
            pts = rt.path["pts"]
        else:
            pts = [f["targets"].get("right_palm") or f["targets"].get("left_palm") for f in rt.recorder.frames[::5] if f.get("targets")]
            pts = [p for p in pts if p is not None]
        tasks.set_task(tid, a, "planning", "Compiling for UR5e.")
        r = compile_ur5e(pts)
        tasks.set_task(tid, a, "done" if r["ok"] else "failed", r["message"])
    elif a == "drive":
        tasks.drive(tid, c.get("dx", 0.0), c.get("dy", 0.0), c.get("dyaw", 0.0))
    elif a == "stop":
        tasks.stop(tid)
    elif a == "home":
        tasks.home(tid)
    elif a == "reset_scene":
        tasks.home(tid, reset_scene=True)
    elif a == "export":
        if not rt.recorder.frames:
            tasks.set_task(tid, a, "failed", "Nothing recorded yet.")
        else:
            rt.saved = rt.recorder.save((ROOT / "recordings").as_posix())
            tasks.set_task(tid, a, "done", f"Exported intent, trajectory and report to {Path(rt.saved).name}.")
    else:
        tasks.set_task(tid, str(a), "failed", f"Unknown action '{a}'.")


def public_mode(rt, tasks):
    if rt.mode == "FOLLOW":
        return "TASK" if tasks.kind == "task" else "PATH"
    if rt.mode == "PREVIEW":
        return "PREVIEW"
    if rt.engaged and rt.mode in ("MIRROR", "REC"):
        return "FOLLOW"
    return "IDLE"


def state(rt, tasks, view, cam, fps, source, pts, lm, scene, grip, hold):
    r = rt.last_result
    proj = (lambda p: Tasks.project(cam, p, ASPECT)) if cam is not None else (lambda p: None)
    path = None
    if rt.path is not None:
        path = {"uv": [proj(p) for p in rt.path["pts"]], "ok": [bool(o) for o in rt.path["ok"]],
                "i": int(rt.path["i"]), "hand": rt.path.get("hand") if rt.path.get("hand") in ("left", "right")
                else ("right" if rt.path["palm"].startswith("right") else "left")}
    msgs = list(r["messages"])
    if rt.engaged and rt.mode in ("MIRROR", "REC"):
        need = [k for k in ("l_wrist", "r_wrist", "l_shoulder", "r_shoulder") if not (lm and k in lm)]
        if need:
            msgs = ["I can't see your arms. Step back so your shoulders and hands are in your camera."] + msgs
    if rt.path_msg:
        msgs = [rt.path_msg] + msgs
    objs = [(n, tasks.mug_center(n)) for n in tasks.objs] + [(n, o["pos"]) for n, o in tasks.obstacles.items()]
    return {
        "mode": public_mode(rt, tasks), "view": view,
        "feasibility": round(r["feasibility"], 3), "messages": msgs[:4],
        "speed_scale": round(rt.speed_scale, 2), "mirror": rt.retargeter.mirror,
        "fps": round(fps, 1), "ik_hz": round(fps * IK_ITERS, 1), "source": source,
        "pose": [[1 - x / W, y / H] for x, y in pts] if pts else None, "pose_ok": bool(lm),
        "objects": [[lab, round(sc, 2), 1 - x2 / W, y1 / H, 1 - x1 / W, y2 / H] for lab, sc, (x1, y1, x2, y2) in scene],
        "grip": grip, "user_holding": hold,
        "markers": [{"id": mk["id"], "pos": [round(float(x), 3) for x in mk["pos"]], "uv": proj(mk["pos"])} for mk in tasks.markers],
        "path": path, "task": tasks.task,
        "holding": {("left" if h.startswith("left") else "right"): o for h, o in tasks.holding.items()},
        "scene_objects": [{"name": n.replace("_", " "), "pos": [round(float(x), 3) for x in p], "uv": proj(p)} for n, p in objs],
        "regions": list(REGIONS.keys()), "skills": tasks.skills(), "skills_meta": skills_meta(tasks),
        "recording": {"active": rt.mode == "REC" or (rt.mode == "FOLLOW" and tasks.kind is not None),
                      "seconds": round(rt.recorder.duration, 1)},
        "saved": rt.saved,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--webcam", action="store_true", help="open this computer's webcam (off by default)")
    ap.add_argument("--byok", action="store_true", help="public deployment: never use the server's API keys; visitors bring their own")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", help="video file instead of the webcam (e.g. a dance clip)")
    ap.add_argument("--objects", action="store_true", help="detect objects in your camera for context")
    ap.add_argument("--fake", action="store_true", help="synthetic landmarks, no camera")
    ap.add_argument("--no-mirror", action="store_true")
    ap.add_argument("--https", action="store_true", help="also serve HTTPS (phone mic/camera need it)")
    ap.add_argument("--mock", action="store_true", help="start with the offline mock brain on (gestures + narration)")
    ap.add_argument("--model", default="models/pose_landmarker_full.task")
    ap.add_argument("--objects-model", default="models/efficientdet_lite0.tflite")
    ap.add_argument("--port", type=int, default=8777)
    ap.add_argument("--https-port", type=int, default=8443)
    ap.add_argument("--frames", type=int, default=0, help="exit after N frames (testing)")
    args = ap.parse_args()
    if args.byok:
        os.environ["PC_BYOK"] = "1"
    shared = Shared()
    threading.Thread(target=run_server, args=(shared, args.port, args.https_port if args.https else 0), daemon=True).start()
    try:
        worker(args, shared)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
