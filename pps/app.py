"""PhysicsClay web app, one process.

camera / video / synthetic  ->  MediaPipe pose + objects  ->  retarget + mink IK on the real Unitree G1
->  physics validation  ->  streamed to a browser HUD (camera frame, sim frame, state) over WebSocket.
The browser sends: clutch, keys, typed commands, drawn / edited trajectories, sim camera orbit.
Run:  .venv/bin/python -m pps.app [--video clip.mp4] [--objects] [--fake]   then open http://127.0.0.1:8765
"""
import argparse
import asyncio
import json
import threading
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
from aiohttp import WSMsgType, web

from .fake import fake_landmarks
from .perceive import H, W, Objects, Pose, grip_state, holding, lift_path, nearest_hand, resample
from .protocol import LANDMARKS
from .robot_runtime import Runtime

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "web"
IK_ITERS, IK_DT = 4, 1 / 120


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.cam = self.sim = None
        self.state = b"{}"
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


async def broadcaster(app):
    shared, last = app["shared"], -1
    while True:
        await asyncio.sleep(1 / 30)
        cam, sim, state, fid = shared.get()
        if fid == last or cam is None:
            continue
        last = fid
        for ws in list(shared.clients):
            try:
                await ws.send_str(state.decode("utf-8"))
                await ws.send_bytes(b"\x01" + cam)
                await ws.send_bytes(b"\x02" + sim)
            except Exception:
                shared.clients.discard(ws)


async def index(request):
    return web.FileResponse(WEB / "index.html")


def run_server(shared, port):
    async def start_bg(app):
        app["bg"] = asyncio.create_task(broadcaster(app))

    app = web.Application()
    app["shared"] = shared
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static", WEB)
    app.on_startup.append(start_bg)
    web.run_app(app, host="127.0.0.1", port=port, print=None, handle_signals=False, access_log=None)


# ------------------------------------------------------------------ worker (main thread: GL + camera)
def worker(args, shared):
    rt = Runtime(mirror=not args.no_mirror)
    renderer = mujoco.Renderer(rt.model, 360, 480)
    cam3d = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(rt.model, cam3d)
    cam3d.azimuth, cam3d.elevation, cam3d.distance = 165, -20, 2.2
    cam3d.lookat[:] = (0.35, 0.0, 0.9)

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

    engaged = args.video is not None
    last_pts = last_lm = None
    path_uv = []
    grip_hist = {"l": [], "r": []}
    t0, n, fps, next_frame_t = time.time(), 0, 0.0, time.time()
    print(f"PhysicsClay: source={source}  open http://127.0.0.1:{args.port}", flush=True)

    while shared.running:
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
            lm = fake_landmarks(time.time() - t0)
            pts = [(W / 2, H / 2)] * 33   # synthetic 2D projection so paths can be lifted and the skeleton drawn
            for name, i in LANDMARKS.items():
                if name in lm:
                    pts[i] = (W / 2 - lm[name][0] * 700, H * 0.55 + lm[name][1] * 700)
        if lm:
            last_pts, last_lm = pts, lm
        scene = objects(frame) if objects is not None else []
        hold = holding(scene, pts)
        grip = {}
        for s in ("l", "r"):
            g = grip_state(lm, s) if lm else None
            hist = grip_hist[s]
            hist.append(g if g is not None else 0)
            del hist[:-5]
            grip[s] = 1 if (sum(hist) >= 3 or hold.get(s)) else 0

        key = cmd = path_msg = None
        for c in shared.pop_cmds():
            t = c.get("type")
            if t == "clutch":
                engaged = bool(c.get("engaged"))
            elif t == "key":
                key = str(c.get("key", ""))[:1].upper() or None
                if key == "C":
                    path_uv = []
            elif t == "cmd":
                cmd = c.get("text")
            elif t == "path":
                uv = c.get("points") or []
                if len(uv) >= 2:
                    raw = [((1 - u) * W, v * H) for u, v in uv]
                    if c.get("resample", True):
                        raw = resample(raw, 24)
                    lifted = lift_path(raw, last_pts, last_lm, depth=float(c.get('depth', 0.2))) if last_lm else None
                    if lifted:
                        path_msg = {"points": lifted, "hand": c.get("hand") or nearest_hand(raw[0], last_pts)}
                        path_uv = [[1 - x / W, y / H] for x, y in raw]
                    else:
                        rt.path_msg = "Stand in view first: the path is placed in your body plane"
            elif t == "camera":
                cam3d.azimuth += float(c.get("dazimuth", 0.0))
                cam3d.elevation = float(np.clip(cam3d.elevation + float(c.get("delevation", 0.0)), -85, 20))
                cam3d.distance = float(np.clip(cam3d.distance * float(c.get("zoom", 1.0)), 1.0, 6.0))

        rt.handle_message({"engaged": engaged, "lm": lm, "grip": grip, "holding": hold,
                           "scene": sorted({o[0] for o in scene}), "source": source,
                           "key": key, "cmd": cmd, "path": path_msg})
        for _ in range(IK_ITERS):
            rt.step(IK_DT)

        rt.paint_model()
        renderer.update_scene(rt.data, cam3d)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        rt.paint_overlays(renderer.scene, reset=False)
        sim = renderer.render()
        sim_jpg = cv2.imencode(".jpg", cv2.cvtColor(sim, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 82])[1].tobytes()
        shown = cv2.resize(cv2.flip(frame, 1), (960, 540))
        cam_jpg = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 72])[1].tobytes()

        n += 1
        if n % 10 == 0:
            fps, t0_fps = 10 / max(time.time() - getattr(worker, "_t", time.time() - 0.3), 1e-6), time.time()
            worker._t = t0_fps
        rb = rt.readback(ik_hz=IK_ITERS * fps)
        rb.update({
            "engaged": engaged, "source": source, "fps": round(fps, 1), "grip": grip, "holding": hold,
            "scene": sorted({o[0] for o in scene}),
            "objects": [[lab, round(sc, 2), 1 - x2 / W, y1 / H, 1 - x1 / W, y2 / H] for lab, sc, (x1, y1, x2, y2) in scene],
            "pose": [[1 - x / W, y / H] for x, y in pts] if pts else None,
            "pose_ok": bool(lm), "path_uv": path_uv,
            "path_hand": None if rt.path is None else rt.path.get("hand"),
        })
        shared.put(cam_jpg, sim_jpg, json.dumps(rb).encode("utf-8"))
        if args.frames and n >= args.frames:
            shared.running = False
    if cap is not None:
        cap.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", help="video file instead of the webcam (e.g. a dance clip)")
    ap.add_argument("--objects", action="store_true", help="detect objects for scene context")
    ap.add_argument("--fake", action="store_true", help="synthetic landmarks, no camera")
    ap.add_argument("--no-mirror", action="store_true", help="same-side mapping (default is mirror: the sim camera faces the robot)")
    ap.add_argument("--model", default="models/pose_landmarker_full.task")
    ap.add_argument("--objects-model", default="models/efficientdet_lite0.tflite")
    ap.add_argument("--port", type=int, default=8777)
    ap.add_argument("--frames", type=int, default=0, help="exit after N frames (testing)")
    args = ap.parse_args()
    shared = Shared()
    threading.Thread(target=run_server, args=(shared, args.port), daemon=True).start()
    try:
        worker(args, shared)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
