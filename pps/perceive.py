"""Perception + HUD process (plain python).

Multimodal intent inputs -> UDP -> robot runtime:
  Demonstrate : live webcam pose (MediaPipe Pose)                  --camera N
  Show        : any video file, e.g. a dance clip (same pipeline)  --video path.mp4
  Scene       : object detection (MediaPipe EfficientDet) tags what each hand is holding  --objects
  Say         : typed /commands (voice/LLM slot in here later)
Keys:  Space clutch   R rec   P preview   A approve/save   M mirror   H home   / command   Q quit
"""
import argparse
import time

import cv2
import numpy as np

from .fake import fake_landmarks
from .protocol import LANDMARKS, PERCEIVE_TO_ROBOT, ROBOT_TO_PERCEIVE, make_receiver, make_sender

SKELETON = [(11, 13), (13, 15), (12, 14), (14, 16), (11, 12), (11, 23), (12, 24), (23, 24),
            (15, 19), (15, 21), (15, 17), (16, 20), (16, 22), (16, 18), (0, 11), (0, 12)]
W, H, TOP, BOT = 1280, 720, 56, 170
FONT = cv2.FONT_HERSHEY_SIMPLEX
MODE_COLOR = {"MIRROR": (160, 160, 160), "REC": (60, 60, 230), "PREVIEW": (230, 160, 40), "FOLLOW": (80, 230, 255), "SAVED": (80, 200, 80)}


class Pose:
    def __init__(self, model_path, infer_width=640):
        import mediapipe as mp
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core.base_options import BaseOptions
        self.mp = mp
        opts = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO, num_poses=1,
            min_pose_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.detector = vision.PoseLandmarker.create_from_options(opts)
        self.infer_width = infer_width
        self.t0 = time.monotonic()
        self.last_ts = -1

    def __call__(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        small = cv2.resize(frame_bgr, (self.infer_width, int(h * self.infer_width / w)))
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        ts = max(int((time.monotonic() - self.t0) * 1000), self.last_ts + 1)
        self.last_ts = ts
        res = self.detector.detect_for_video(img, ts)
        if not res.pose_landmarks:
            return None, None
        norm, world = res.pose_landmarks[0], res.pose_world_landmarks[0]
        pts = [(p.x * w, p.y * h) for p in norm]
        vis = [(p.visibility if p.visibility is not None else 1.0) for p in norm]
        lm = {name: [world[i].x, world[i].y, world[i].z] for name, i in LANDMARKS.items() if vis[i] > 0.4}
        return pts, lm


class Objects:
    """Scene context: what objects are in view, and what each hand is touching."""

    def __init__(self, model_path, infer_width=640, every=3):
        import mediapipe as mp
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core.base_options import BaseOptions
        self.mp = mp
        self.detector = vision.ObjectDetector.create_from_options(vision.ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO, max_results=8, score_threshold=0.45))
        self.infer_width, self.every, self.n = infer_width, every, 0
        self.t0, self.last_ts, self.last = time.monotonic(), -1, []

    def __call__(self, frame_bgr):
        self.n += 1
        if self.n % self.every:
            return self.last
        h, w = frame_bgr.shape[:2]
        s = self.infer_width / w
        small = cv2.resize(frame_bgr, (self.infer_width, int(h * s)))
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        ts = max(int((time.monotonic() - self.t0) * 1000), self.last_ts + 1)
        self.last_ts = ts
        out = []
        for det in self.detector.detect_for_video(img, ts).detections:
            b = det.bounding_box
            label = det.categories[0].category_name
            if label == "person":
                continue
            out.append((label, float(det.categories[0].score),
                        (b.origin_x / s, b.origin_y / s, (b.origin_x + b.width) / s, (b.origin_y + b.height) / s)))
        self.last = out
        return out


def holding(objects, pts):
    """Which detected object (if any) contains each wrist -> {"l": label|None, "r": label|None}."""
    out = {"l": None, "r": None}
    if not objects or not pts:
        return out
    for side, idx in (("l", 15), ("r", 16)):
        x, y = pts[idx]
        for label, _, (x1, y1, x2, y2) in objects:
            if (x2 - x1) * (y2 - y1) > 0.12 * W * H:   # hand-held objects are small; skip furniture-sized boxes
                continue
            mx, my = 0.15 * (x2 - x1), 0.15 * (y2 - y1)
            if x1 - mx <= x <= x2 + mx and y1 - my <= y <= y2 + my:
                out[side] = label
                break
    return out


class Sketch:
    """Mouse-drawn trajectory on the camera view (canvas pixel coords)."""

    def __init__(self):
        self.points, self.drawing, self.finished = [], False, False

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points, self.drawing, self.finished = [(x, y)], True, False
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            if not self.points or (abs(x - self.points[-1][0]) + abs(y - self.points[-1][1])) > 6:
                self.points.append((x, y))
        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            self.drawing = False
            self.finished = len(self.points) >= 2


def resample(points, n):
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    if cum[-1] < 1e-6:
        return pts[:1].tolist()
    n = max(2, min(n, len(pts)))
    out = [[float(np.interp(t, cum, pts[:, k])) for k in range(pts.shape[1])] for t in np.linspace(0, cum[-1], n)]
    return out


def lift_path(px_raw, pts2d, lm, depth=0.0):
    """2D sketch (raw video px) -> 3D human-world points in the person's frontal plane (shoulder depth)."""
    try:
        ls, rs = np.array(pts2d[11]), np.array(pts2d[12])
        lsw, rsw = np.array(lm["l_shoulder"]), np.array(lm["r_shoulder"])
    except (KeyError, TypeError, IndexError):
        return None
    px_w, m_w = np.linalg.norm(ls - rs), np.linalg.norm((lsw - rsw)[:2])
    if px_w < 20 or m_w < 0.1:
        return None
    sc = m_w / px_w
    c_px, c_w = (ls + rs) / 2, (lsw + rsw) / 2
    return [[float(c_w[0] + (p[0] - c_px[0]) * sc), float(c_w[1] + (p[1] - c_px[1]) * sc), float(c_w[2] - depth)] for p in px_raw]


def nearest_hand(px_first, pts2d):
    if not pts2d:
        return "r"
    dl = np.hypot(px_first[0] - pts2d[15][0], px_first[1] - pts2d[15][1])
    dr = np.hypot(px_first[0] - pts2d[16][0], px_first[1] - pts2d[16][1])
    return "l" if dl < dr else "r"


def grip_state(lm, side):
    try:
        wr, idx, el = (np.array(lm[f"{side}_{k}"]) for k in ("wrist", "index", "elbow"))
    except KeyError:
        return None
    fore = np.linalg.norm(wr - el)
    if fore < 1e-3:
        return None
    return 1 if np.linalg.norm(idx - wr) / fore < 0.45 else 0


def put(img, text, xy, scale=0.6, color=(235, 235, 235), thick=1):
    cv2.putText(img, text, (int(xy[0]), int(xy[1])), FONT, scale, color, thick, cv2.LINE_AA)


def draw_hud(canvas, rb, engaged, grip, hold, scene, fps, cmd_mode, cmd_buf, pose_ok, source):
    mode = rb.get("mode", "—")
    if rb.get("_saved_flash", 0) > time.time():
        mode = "SAVED"
    cv2.rectangle(canvas, (0, 0), (W, TOP), (28, 28, 28), -1)
    cv2.rectangle(canvas, (12, 10), (150, 46), MODE_COLOR.get(mode, (120, 120, 120)), -1)
    put(canvas, mode, (24, 37), 0.75, (15, 15, 15), 2)
    eng_col = (90, 210, 90) if engaged else (110, 110, 110)
    put(canvas, "ENGAGED" if engaged else "PAUSED  (space)", (170, 37), 0.65, eng_col, 2)
    feas = float(rb.get("feasibility", 0.0))
    cv2.rectangle(canvas, (430, 18), (630, 38), (60, 60, 60), -1)
    cv2.rectangle(canvas, (430, 18), (430 + int(200 * feas), 38), (90, 210, 90) if feas > 0.8 else (60, 160, 230), -1)
    put(canvas, f"feasible {feas * 100:3.0f}%", (640, 37), 0.6)
    put(canvas, f"speed x{rb.get('speed_scale', 1.0)}", (800, 37), 0.6)
    put(canvas, f"mirror {'on' if rb.get('mirror', True) else 'off'}", (930, 37), 0.6)
    put(canvas, f"{source}  {fps:3.0f} fps / ik {rb.get('ik_hz', 0):3.0f} Hz", (1050, 37), 0.5, (170, 170, 170))
    y0 = TOP + H
    if mode == "REC":
        put(canvas, f"REC {rb.get('rec_duration', 0):.1f}s  {rb.get('rec_frames', 0)} frames", (640, y0 + 28), 0.6, (60, 60, 230))
    elif mode == "PREVIEW":
        put(canvas, f"PREVIEW {rb.get('preview_t', 0):.1f}s / {rb.get('rec_duration', 0):.1f}s", (640, y0 + 28), 0.6, (230, 160, 40))
    cv2.rectangle(canvas, (0, y0), (W, y0 + BOT), (22, 22, 22), -1)
    put(canvas, "ROBOT SAYS", (16, y0 + 28), 0.6, (120, 200, 255), 2)
    msgs = list(rb.get("messages", []))
    if rb.get("path_msg"):
        msgs = [rb["path_msg"]] + msgs
    if not pose_ok:
        msgs = ["No person detected"]
    elif not msgs:
        msgs = ["OK. Following your intent." if engaged else "Paused. Drag the ghost targets in the sim to shape the pose."]
    for i, mtxt in enumerate(msgs[:3]):
        col = (90, 210, 90) if mtxt.startswith(("OK", "Paused")) or "all reachable" in mtxt else (80, 120, 255)
        put(canvas, "- " + mtxt, (16, y0 + 58 + 26 * i), 0.62, col, 1)
    put(canvas, "CONTEXT", (900, y0 + 28), 0.6, (120, 200, 255), 2)
    hands = []
    for s, name in (("l", "L"), ("r", "R")):
        state = "FIST" if grip.get(s) else "OPEN"
        hands.append(f"{name}: {state}" + (f" on {hold[s]}" if hold.get(s) else ""))
    put(canvas, "   ".join(hands), (900, y0 + 58), 0.52, (200, 200, 200))
    put(canvas, "scene: " + (", ".join(sorted({o[0] for o in scene})) if scene else "(no objects)"), (900, y0 + 84), 0.5, (170, 170, 170))
    if rb.get("saved"):
        put(canvas, f"saved: {rb['saved'].split('/')[-1]}", (900, y0 + 110), 0.45, (80, 200, 80))
    prompt = "> " + cmd_buf + ("_" if cmd_mode else "")
    put(canvas, prompt if cmd_mode else "draw a path with the mouse, F follow, C clear   |   space clutch   R rec   P preview   A approve   M mirror   H home   / cmd",
        (16, y0 + BOT - 16), 0.55, (200, 200, 200) if cmd_mode else (130, 130, 130))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", help="video file instead of the webcam (e.g. a dance clip)")
    ap.add_argument("--no-loop", action="store_true", help="do not loop the video file")
    ap.add_argument("--objects", action="store_true", help="detect objects for scene context")
    ap.add_argument("--model", default="models/pose_landmarker_full.task")
    ap.add_argument("--objects-model", default="models/efficientdet_lite0.tflite")
    ap.add_argument("--fake", action="store_true", help="synthetic landmarks, no camera")
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="exit after N frames (testing)")
    args = ap.parse_args()

    send = make_sender(PERCEIVE_TO_ROBOT)
    recv = make_receiver(ROBOT_TO_PERCEIVE)
    pose = None if args.fake else Pose(args.model)
    objects = Objects(args.objects_model) if (args.objects and not args.fake) else None
    cap, video_dt, source = None, 0.0, "fake"
    if args.video:
        cap = cv2.VideoCapture(args.video)
        fps_v = cap.get(cv2.CAP_PROP_FPS) or 30.0
        video_dt, source = 1.0 / fps_v, "video"
    elif not args.fake:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        source = "webcam"

    sketch = Sketch()
    if not args.no_window:
        cv2.namedWindow("PhysicsClay")
        cv2.setMouseCallback("PhysicsClay", sketch.on_mouse)
    path_px, pending_path, last_pts, last_lm = [], None, None, None
    engaged, cmd_mode, cmd_buf = args.video is not None, False, ""   # a video file starts engaged
    key_to_send, cmd_to_send = None, None
    rb, grip_hist = {}, {"l": [], "r": []}
    t0, n, fps, saved_seen = time.time(), 0, 0.0, None
    next_frame_t = time.time()
    while True:
        if cap is not None:
            if args.video:
                now = time.time()
                if now < next_frame_t:
                    time.sleep(next_frame_t - now)
                next_frame_t = max(next_frame_t + video_dt, time.time() - video_dt)
            ok, frame = cap.read()
            if not ok:
                if args.video and not args.no_loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                if args.video:
                    break
                time.sleep(0.02)
                continue
            if frame.shape[1] != W or frame.shape[0] != H:
                frame = cv2.resize(frame, (W, H))
            pts, lm = pose(frame)
        else:
            frame = np.full((H, W, 3), 40, np.uint8)
            lm, pts = fake_landmarks(time.time() - t0), None
        if lm:
            last_pts, last_lm = pts, lm
        if sketch.finished:
            sketch.finished = False
            raw = resample([(W - x, y - TOP) for x, y in sketch.points], 24)
            lifted = lift_path(raw, last_pts, last_lm) if last_lm else None
            if lifted:
                pending_path = {"points": lifted, "hand": nearest_hand(raw[0], last_pts)}
                path_px = [(int(W - p[0]), int(p[1])) for p in raw]   # flipped display coords
            else:
                path_px = []
        scene = objects(frame) if objects is not None else []
        hold = holding(scene, pts)
        grip = {}
        for s in ("l", "r"):
            g = grip_state(lm, s) if lm else None
            hist = grip_hist[s]
            hist.append(g if g is not None else 0)
            del hist[:-5]
            grip[s] = 1 if (sum(hist) >= 3 or hold.get(s)) else 0
        for m in recv():
            rb = m
        if rb.get("saved") and rb.get("saved") != saved_seen:
            saved_seen = rb["saved"]
            rb["_saved_flash"] = time.time() + 3.0
        send({"t": time.time(), "engaged": engaged, "lm": lm, "grip": grip, "holding": hold,
              "scene": sorted({o[0] for o in scene}), "source": source, "key": key_to_send, "cmd": cmd_to_send,
              "path": pending_path})
        key_to_send = cmd_to_send = pending_path = None

        n += 1
        if n % 15 == 0:
            fps = 15 / max(time.time() - t0, 1e-6)
            t0 = time.time()
        if args.frames and n >= args.frames:
            break
        if args.no_window:
            continue

        shown = cv2.flip(frame, 1)
        for label, score, (x1, y1, x2, y2) in scene:
            p1, p2 = (int(W - x2), int(y1)), (int(W - x1), int(y2))
            cv2.rectangle(shown, p1, p2, (255, 190, 60), 2)
            put(shown, f"{label} {score:.2f}", (p1[0] + 4, p1[1] + 18), 0.5, (255, 190, 60))
        if pts:
            flip = lambda p: (int(W - p[0]), int(p[1]))
            for a, b in SKELETON:
                cv2.line(shown, flip(pts[a]), flip(pts[b]), (90, 230, 120) if engaged else (160, 160, 160), 2, cv2.LINE_AA)
            for i in (11, 12, 13, 14, 15, 16):
                cv2.circle(shown, flip(pts[i]), 6, (255, 255, 255), -1, cv2.LINE_AA)
        if sketch.drawing and len(sketch.points) > 1:
            cv2.polylines(shown, [np.array([(x, y - TOP) for x, y in sketch.points], np.int32)], False, (255, 220, 80), 3, cv2.LINE_AA)
        if path_px:
            ok = rb.get("path_ok") or []
            cur = rb.get("path_i", -1)
            cv2.polylines(shown, [np.array(path_px, np.int32)], False, (255, 220, 80), 2, cv2.LINE_AA)
            for i, (x, y) in enumerate(path_px):
                good = ok[i] if i < len(ok) else True
                col = (90, 210, 90) if good else (80, 80, 255)
                if i == cur and rb.get("mode") == "FOLLOW":
                    col, r = (80, 230, 255), 9
                else:
                    r = 5
                cv2.circle(shown, (x, y), r, col, -1, cv2.LINE_AA)
        if engaged:
            cv2.rectangle(shown, (0, 0), (W - 1, H - 1), (90, 210, 90), 6)
        canvas = np.zeros((TOP + H + BOT, W, 3), np.uint8)
        canvas[TOP:TOP + H] = shown
        draw_hud(canvas, rb, engaged, grip, hold, scene, fps, cmd_mode, cmd_buf, bool(lm), source)
        cv2.imshow("PhysicsClay", canvas)
        k = cv2.waitKey(1) & 0xFF
        if k == 255:
            continue
        if cmd_mode:
            if k in (13, 10):
                cmd_to_send, cmd_mode, cmd_buf = cmd_buf, False, ""
            elif k == 27:
                cmd_mode, cmd_buf = False, ""
            elif k in (8, 127):
                cmd_buf = cmd_buf[:-1]
            elif 32 <= k < 127:
                cmd_buf += chr(k)
            continue
        if k == ord(" "):
            engaged = not engaged
        elif k in (ord("r"), ord("p"), ord("a"), ord("m"), ord("h"), ord("f")):
            key_to_send = chr(k).upper()
        elif k == ord("c"):
            key_to_send, path_px = "C", []
        elif k == ord("/"):
            cmd_mode, cmd_buf = True, ""
        elif k in (ord("q"), 27):
            break
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
