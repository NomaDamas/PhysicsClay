"""Real-time video understanding, made visible: on a person's camera video draw
YOLO11 segmentation masks, SAM 2.1 masks (box-prompted by YOLO), the 33 MediaPipe Pose joints, and
Gemini Robotics-ER points. Output: recordings/samples/vision_overlay.mp4 (1920x1080).
Usage: .venv/bin/python scripts/vision_clip.py <video> [--seconds 10]
"""
import argparse
import sys
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pps.perceive import Pose  # noqa: E402
from pps.spatial import Spatial  # noqa: E402

W, H = 1920, 1080
BONES = [(11, 13), (13, 15), (12, 14), (14, 16), (11, 12), (11, 23), (12, 24), (23, 24), (23, 25), (25, 27), (24, 26), (26, 28),
         (15, 17), (15, 19), (15, 21), (16, 18), (16, 20), (16, 22), (0, 1), (0, 4), (27, 31), (28, 32)]
COLORS = [(232, 160, 92), (120, 197, 90), (223, 119, 92), (229, 196, 78), (190, 121, 182), (90, 170, 235)]


def badge(img, x, y, text, color):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    cv2.rectangle(img, (x, y - th - 14), (x + tw + 20, y + 8), color, -1)
    cv2.putText(img, text, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--er", action="store_true")
    args = ap.parse_args()
    import torch
    from ultralytics import SAM, YOLO
    torch.set_num_threads(4)
    yolo = YOLO(str(ROOT / "models" / "yolo11n-seg.pt"))
    sam = SAM(str(ROOT / "models" / "sam2.1_t.pt"))
    pose = Pose(str(ROOT / "models" / "pose_landmarker_full.task"))
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    step = max(1, int(round(fps / 10)))          # analyse at 10 fps, write at 20 fps (each frame twice)
    out = ROOT / "recordings" / "samples" / "vision_overlay.mp4"
    wr = imageio.get_writer(str(out), fps=20, codec="libx264", quality=8, macro_block_size=1)
    er_pts, n = None, 0
    while n < args.seconds * fps:
        ok, f = cap.read()
        if not ok:
            break
        n += 1
        if (n - 1) % step:
            continue
        h, w = f.shape[:2]
        s = min((W - 560) / w, (H - 120) / h)
        img = cv2.resize(f, (int(w * s) // 2 * 2, int(h * s) // 2 * 2))
        ih, iw = img.shape[:2]
        over = img.copy()
        # YOLO segmentation
        r = yolo.predict(img, imgsz=480, conf=0.35, verbose=False)[0]
        boxes = []
        if r.masks is not None:
            for k, (poly, b) in enumerate(zip(r.masks.xy, r.boxes)):
                c = COLORS[k % len(COLORS)]
                if len(poly) > 2:
                    cv2.fillPoly(over, [poly.astype(np.int32)], c)
                    cv2.polylines(img, [poly.astype(np.int32)], True, c, 2, cv2.LINE_AA)
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                boxes.append([x1, y1, x2, y2])
                badge(img, int(x1), int(y1) + 4, f"YOLO11-seg: {r.names[int(b.cls[0])]} {float(b.conf[0]):.2f}", c)
        img = cv2.addWeighted(over, 0.28, img, 0.72, 0)
        # SAM 2.1 refines the first box
        if boxes:
            sr = sam(img, bboxes=[boxes[0]], verbose=False)[0]
            if sr.masks is not None and len(sr.masks.xy) and len(sr.masks.xy[0]) > 2:
                cv2.polylines(img, [sr.masks.xy[0].astype(np.int32)], True, (255, 255, 255), 3, cv2.LINE_AA)
                badge(img, int(boxes[0][0]), int(boxes[0][3]) - 6, "SAM 2.1 mask", (60, 60, 60))
        # 33 pose joints
        pts, lm = pose(cv2.resize(f, (1280, 720)))
        if pts:
            P = [(int(x / 1280 * iw), int(y / 720 * ih)) for x, y in pts]
            for a, b in BONES:
                cv2.line(img, P[a], P[b], (95, 208, 138), 4, cv2.LINE_AA)
            for p in P:
                cv2.circle(img, p, 5, (255, 255, 255), -1, cv2.LINE_AA)
        # Gemini Robotics-ER points, asked once mid-clip
        if er_pts is None and n > fps * 2 and "--er" in sys.argv:
            sp = Spatial.__new__(Spatial)
            sp._busy_er, sp.er = False, None
            sp.ask_er(img.copy(), "camera", "Point to the person's hands, head, and any object they interact with.")
            for _ in range(60):
                if sp.er is not None:
                    break
                cv2.waitKey(1)
                import time
                time.sleep(0.25)
            er_pts = (sp.er or {}).get("points") or []
        for i, p in enumerate(er_pts or []):
            x, y = int(p["u"] * iw), int(p["v"] * ih)
            cv2.circle(img, (x, y), 14, (92, 119, 223), -1, cv2.LINE_AA)
            cv2.putText(img, str(i + 1), (x - 7, y + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, p["label"][:22], (x + 18, y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (92, 119, 223), 2, cv2.LINE_AA)
        # canvas
        canvas = np.full((H, W, 3), (253, 251, 251), np.uint8)
        y0 = (H - ih) // 2
        canvas[y0:y0 + ih, 40:40 + iw] = img
        x = 80 + iw
        cv2.putText(canvas, "Real-time video understanding", (x, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.05, (31, 29, 29), 3, cv2.LINE_AA)
        rows = [("YOLO11-seg", "objects + masks", COLORS[0]), ("SAM 2.1", "precise mask", (60, 60, 60)),
                ("MediaPipe Pose", "33 body joints", (95, 208, 138)), ("Gemini Live + Robotics-ER", "meaning + pointing (cloud)", (92, 119, 223))]
        for i, (a, b, c) in enumerate(rows):
            yy = 230 + i * 90
            cv2.circle(canvas, (x + 14, yy - 10), 12, c, -1, cv2.LINE_AA)
            cv2.putText(canvas, a, (x + 40, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (31, 29, 29), 2, cv2.LINE_AA)
            cv2.putText(canvas, b, (x + 40, yy + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (110, 110, 118), 2, cv2.LINE_AA)
        cv2.putText(canvas, "Your camera is part of the prompt.", (x, 640), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (92, 119, 223), 2, cv2.LINE_AA)
        for _ in range(2):
            wr.append_data(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    wr.close()
    print({"out": str(out), "er_points": len(er_pts or [])})


if __name__ == "__main__":
    main()
