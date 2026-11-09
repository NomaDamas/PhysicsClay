"""Perception + HUD process (plain python): webcam -> MediaPipe Pose -> UDP; HUD shows robot read-back.

Keys:  Space clutch (engage/pause)   R rec   P preview   A approve/save   M mirror   H home
       /  type a command (slower, faster, mirror off, hold, go, reset) then Enter      Q quit
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
MODE_COLOR = {"MIRROR": (160, 160, 160), "REC": (60, 60, 230), "PREVIEW": (230, 160, 40), "SAVED": (80, 200, 80)}


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


def draw_hud(canvas, rb, engaged, grip, fps, cmd_mode, cmd_buf, pose_ok):
    mode = rb.get("mode", "—")
    if rb.get("saved") and mode == "MIRROR" and rb.get("_saved_flash", 0) > time.time():
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
    put(canvas, f"{fps:4.0f} fps / ik {rb.get('ik_hz', 0):4.0f} Hz", (1060, 37), 0.55, (170, 170, 170))
    if mode == "REC":
        put(canvas, f"REC {rb.get('rec_duration', 0):.1f}s  {rb.get('rec_frames', 0)} frames", (170, 52 + H + 24), 0.6, (60, 60, 230))
    elif mode == "PREVIEW":
        put(canvas, f"PREVIEW {rb.get('preview_t', 0):.1f}s / {rb.get('rec_duration', 0):.1f}s", (170, 52 + H + 24), 0.6, (230, 160, 40))
    # bottom panel
    y0 = TOP + H
    cv2.rectangle(canvas, (0, y0), (W, y0 + BOT), (22, 22, 22), -1)
    put(canvas, "ROBOT SAYS", (16, y0 + 28), 0.6, (120, 200, 255), 2)
    msgs = rb.get("messages", [])
    if not pose_ok:
        msgs = ["No person detected"]
    elif not msgs:
        msgs = ["OK. Following your intent." if engaged else "Paused. Drag the ghost targets in the sim to shape the pose."]
    for i, mtxt in enumerate(msgs[:3]):
        col = (90, 210, 90) if mtxt.startswith(("OK", "Paused")) else (80, 120, 255)
        put(canvas, "- " + mtxt, (16, y0 + 58 + 26 * i), 0.62, col, 1)
    put(canvas, f"L hand: {'FIST' if grip.get('l') else 'OPEN'}   R hand: {'FIST' if grip.get('r') else 'OPEN'}", (900, y0 + 28), 0.55, (170, 170, 170))
    if rb.get("saved"):
        put(canvas, f"saved: {rb['saved']}", (900, y0 + 58), 0.45, (80, 200, 80))
    prompt = "> " + cmd_buf + ("_" if cmd_mode else "")
    put(canvas, prompt if cmd_mode else "/ command   space clutch   R rec   P preview   A approve   M mirror   H home   Q quit",
        (16, y0 + BOT - 16), 0.55, (200, 200, 200) if cmd_mode else (130, 130, 130))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--model", default="models/pose_landmarker_full.task")
    ap.add_argument("--fake", action="store_true", help="synthetic landmarks, no camera")
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="exit after N frames (testing)")
    args = ap.parse_args()

    send = make_sender(PERCEIVE_TO_ROBOT)
    recv = make_receiver(ROBOT_TO_PERCEIVE)
    pose = None if args.fake else Pose(args.model)
    cap = None
    if not args.fake:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

    engaged, cmd_mode, cmd_buf = False, False, ""
    key_to_send, cmd_to_send = None, None
    rb, grip_hist = {}, {"l": [], "r": []}
    t0, n, fps = time.time(), 0, 0.0
    saved_seen = None
    while True:
        if cap is not None:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            frame = cv2.resize(frame, (W, H)) if frame.shape[1] != W else frame
            pts, lm = pose(frame)
        else:
            frame = np.full((H, W, 3), 40, np.uint8)
            lm, pts = fake_landmarks(time.time() - t0), None
        grip = {}
        for s in ("l", "r"):
            g = grip_state(lm, s) if lm else None
            hist = grip_hist[s]
            hist.append(g if g is not None else 0)
            del hist[:-5]
            grip[s] = 1 if sum(hist) >= 3 else 0
        for m in recv():
            rb = m
        if rb.get("saved") and rb.get("saved") != saved_seen:
            saved_seen = rb["saved"]
            rb["_saved_flash"] = time.time() + 3.0
        send({"t": time.time(), "engaged": engaged, "lm": lm, "grip": grip, "key": key_to_send, "cmd": cmd_to_send})
        key_to_send = cmd_to_send = None

        n += 1
        if n % 15 == 0:
            fps = 15 / max(time.time() - t0, 1e-6)
            t0 = time.time()
        if args.frames and n >= args.frames:
            break
        if args.no_window:
            continue

        shown = cv2.flip(frame, 1)
        if pts:
            flip = lambda p: (int(W - p[0]), int(p[1]))
            for a, b in SKELETON:
                cv2.line(shown, flip(pts[a]), flip(pts[b]), (90, 230, 120) if engaged else (160, 160, 160), 2, cv2.LINE_AA)
            for i in (11, 12, 13, 14, 15, 16):
                cv2.circle(shown, flip(pts[i]), 6, (255, 255, 255), -1, cv2.LINE_AA)
        if engaged:
            cv2.rectangle(shown, (0, 0), (W - 1, H - 1), (90, 210, 90), 6)
        canvas = np.zeros((TOP + H + BOT, W, 3), np.uint8)
        canvas[TOP:TOP + H] = shown
        draw_hud(canvas, rb, engaged, grip, fps, cmd_mode, cmd_buf, pose_ok=bool(lm))
        cv2.imshow("Physical Prompt Studio", canvas)
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
        elif k in (ord("r"), ord("p"), ord("a"), ord("m"), ord("h")):
            key_to_send = chr(k).upper()
        elif k == ord("/"):
            cmd_mode, cmd_buf = True, ""
        elif k in (ord("q"), 27):
            break
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
