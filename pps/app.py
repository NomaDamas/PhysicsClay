"""PhysicsClay server: one process.

your camera (webcam / video / synthetic) -> MediaPipe pose (+ objects) -> follow mode / context
robot view (MuJoCo, real Unitree G1 + table + scanned mug) <- tasks (pick_place, reach, paths, skills) <- browser intents
Browser talks to Gemini Live itself (key stays in the browser); intents arrive here as tasks, the physics verdict goes back.
Protocol: docs/PROTOCOL.md.   Run: .venv/bin/python -m pps.app [--video clip.mp4] [--objects] [--fake] [--https]
"""
import argparse
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
from aiohttp import WSMsgType, web

from .fake import fake_landmarks
from .perceive import H, W, Objects, Pose, grip_state, holding
from .protocol import LANDMARKS
from .robot_runtime import HANDS, Runtime
from .tasks import REGIONS, Tasks

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "web"
IK_ITERS, IK_DT = 4, 1 / 120
SIM_W, SIM_H = 640, 480
ASPECT = SIM_W / SIM_H


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.cam = self.sim = None
        self.state = "{}"
        self.fid = 0
        self.cmds = []
        self.clients = set()
        self.running = True

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
            except Exception:
                shared.clients.discard(ws)


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
        app = web.Application()
        app["shared"] = shared
        app.router.add_get("/", index)
        app.router.add_get("/ws", ws_handler)
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
def worker(args, shared):
    rt = Runtime(mirror=not args.no_mirror)
    tasks = Tasks(rt)
    m, d = rt.model, rt.data
    renderer = mujoco.Renderer(m, SIM_H, SIM_W)
    robot_cam = m.camera("robot_view").id
    orbit = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, orbit)
    orbit.azimuth, orbit.elevation, orbit.distance = 165, -20, 2.2
    orbit.lookat[:] = (0.35, 0.0, 0.9)
    view = "robot"
    cam_params = None

    pose = None if args.fake else Pose(str(ROOT / args.model))
    objects = Objects(str(ROOT / args.objects_model)) if (args.objects and not args.fake) else None
    cap, video_dt, source = None, 0.0, "synthetic"
    if args.video:
        cap = cv2.VideoCapture(args.video)
        video_dt, source = 1.0 / (cap.get(cv2.CAP_PROP_FPS) or 30.0), "video"
    elif not args.fake:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        source = "webcam"

    grip_hist = {"l": [], "r": []}
    t_start, n, fps, t_fps, next_frame_t = time.time(), 0, 0.0, time.time(), time.time()
    t_phys = time.time()
    ip = lan_ip()
    print(f"PhysicsClay  source={source}\n  laptop: http://127.0.0.1:{args.port}\n  phone : http://{ip}:{args.port}"
          + (f"   (mic/camera on phone: https://{ip}:{args.https_port}, accept the certificate warning)" if args.https else ""),
          flush=True)

    while shared.running:
        # ---------------- your camera
        if cap is not None:
            if args.video:
                now = time.time()
                if now < next_frame_t:
                    time.sleep(next_frame_t - now)
                next_frame_t = max(next_frame_t + video_dt, time.time() - video_dt)
            ok, frame = cap.read()
            if not ok:
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
        scene = objects(frame) if objects is not None else []
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
        for c in shared.pop_cmds():
            t = c.get("type")
            try:
                if t == "tap" and cam_params is not None:
                    hit = tasks.raycast(cam_params, float(c["u"]), float(c["v"]), ASPECT)
                    if hit is not None:
                        mid = tasks.add_marker(hit)
                        rt.path_msg = f"Marker {mid} placed. Say 'put the mug on {mid}'."
                    else:
                        rt.path_msg = "That tap missed the scene."
                elif t == "draw" and cam_params is not None:
                    tasks.draw(c.get("points") or [], cam_params, ASPECT, c.get("hand"))
                elif t == "clear":
                    tasks.markers, tasks.next_marker = [], 0
                    if rt.mode != "FOLLOW":
                        rt.path = None
                    rt.path_msg = None
                elif t == "view":
                    view = "orbit" if c.get("name") == "orbit" else "robot"
                elif t == "camera":
                    orbit.azimuth += float(c.get("dazimuth", 0.0))
                    orbit.elevation = float(np.clip(orbit.elevation + float(c.get("delevation", 0.0)), -85, 10))
                    orbit.distance = float(np.clip(orbit.distance * float(c.get("zoom", 1.0)), 0.8, 6.0))
                elif t == "task":
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

        # ---------------- robot view
        rt.paint_model()
        if view == "robot":
            renderer.update_scene(d, camera=robot_cam)
            fovy = float(m.cam_fovy[robot_cam])
        else:
            renderer.update_scene(d, orbit)
            fovy = float(m.vis.global_.fovy)
        gl = renderer.scene.camera[0]
        fwd, up = np.array(gl.forward, float), np.array(gl.up, float)
        cam_params = (np.array(gl.pos, float), fwd / np.linalg.norm(fwd), up / np.linalg.norm(up), fovy)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        rt.paint_overlays(renderer.scene, reset=False)
        sim = renderer.render()
        sim_jpg = cv2.imencode(".jpg", cv2.cvtColor(sim, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 82])[1].tobytes()
        shown = cv2.resize(cv2.flip(frame, 1), (960, 540))
        cam_jpg = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 72])[1].tobytes()

        n += 1
        if n % 10 == 0:
            now = time.time()
            fps, t_fps = 10 / max(now - t_fps, 1e-6), now
        shared.put(cam_jpg, sim_jpg, json.dumps(state(rt, tasks, view, cam_params, fps, source, pts, lm, scene, grip, hold)))
        if args.frames and n >= args.frames:
            shared.running = False
    if cap is not None:
        cap.release()


def dispatch(tasks, rt, c, cam):
    tid, a = str(c.get("id", "t?")), c.get("action")
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
    mug = tasks.mug_center()
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
        "scene_objects": [{"name": "mug", "pos": [round(float(x), 3) for x in mug], "uv": proj(mug)}],
        "regions": list(REGIONS.keys()), "skills": tasks.skills(),
        "recording": {"active": rt.mode == "REC" or (rt.mode == "FOLLOW" and tasks.kind is not None),
                      "seconds": round(rt.recorder.duration, 1)},
        "saved": rt.saved,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", help="video file instead of the webcam (e.g. a dance clip)")
    ap.add_argument("--objects", action="store_true", help="detect objects in your camera for context")
    ap.add_argument("--fake", action="store_true", help="synthetic landmarks, no camera")
    ap.add_argument("--no-mirror", action="store_true")
    ap.add_argument("--https", action="store_true", help="also serve HTTPS (phone mic/camera need it)")
    ap.add_argument("--model", default="models/pose_landmarker_full.task")
    ap.add_argument("--objects-model", default="models/efficientdet_lite0.tflite")
    ap.add_argument("--port", type=int, default=8777)
    ap.add_argument("--https-port", type=int, default=8443)
    ap.add_argument("--frames", type=int, default=0, help="exit after N frames (testing)")
    args = ap.parse_args()
    shared = Shared()
    threading.Thread(target=run_server, args=(shared, args.port, args.https_port if args.https else 0), daemon=True).start()
    try:
        worker(args, shared)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
