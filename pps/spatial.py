"""Spatial understanding models.
- YOLO11 segmentation (ultralytics) on the user's camera, in a background thread: labelled masks -> polygons.
- Gemini Robotics-ER 2 on demand: "what do you see / where is X" -> labelled points on a frame.
"""
import base64
import json
import os
import re
import threading
import time
import ssl
import urllib.request

import certifi
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
API = "https://generativelanguage.googleapis.com/v1beta/models/"
ER_MODEL = "gemini-robotics-er-2-preview"


def gemini_key():
    k = os.environ.get("GEMINI_API_KEY")
    if k:
        return k
    env = ROOT / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.startswith("GEMINI_API_KEY="):
                return line.split("=", 1)[1].strip()
    return None


class Spatial:
    def __init__(self):
        self.model_name = "yolo-seg"
        self.objects = []
        self.er = None
        self._frame = None
        self._lock = threading.Lock()
        self._yolo = None
        self._busy_er = False
        self.paused = False
        threading.Thread(target=self._loop, daemon=True).start()

    def set_model(self, name):
        self.model_name = name if name in ("yolo-seg", "mediapipe", "off") else "yolo-seg"
        if self.model_name != "yolo-seg":
            self.objects = []

    def feed(self, frame_bgr):
        with self._lock:
            self._frame = frame_bgr

    def _loop(self):
        while True:
            time.sleep(0.6)
            if self.model_name != "yolo-seg" or self.paused:
                continue
            with self._lock:
                f, self._frame = self._frame, None
            if f is None:
                continue
            try:
                if self._yolo is None:
                    import torch
                    torch.set_num_threads(2)
                    from ultralytics import YOLO
                    self._yolo = YOLO(str(ROOT / "models" / "yolo11n-seg.pt"))
                h, w = f.shape[:2]
                r = self._yolo.predict(f, imgsz=416, conf=0.35, verbose=False)[0]
                out = []
                names = r.names
                masks = r.masks.xyn if r.masks is not None else [None] * len(r.boxes)
                for b, poly in zip(r.boxes, masks):
                    x1, y1, x2, y2 = (b.xyxyn[0].tolist())
                    label = names[int(b.cls[0])]
                    pts = None
                    if poly is not None and len(poly) > 2:
                        step = max(1, len(poly) // 40)
                        pts = [[round(1 - float(p[0]), 4), round(float(p[1]), 4)] for p in poly[::step]]
                    out.append({"label": label, "score": round(float(b.conf[0]), 2),
                                "box": [round(1 - x2, 4), round(y1, 4), round(1 - x1, 4), round(y2, 4)], "poly": pts})
                self.objects = out
            except Exception as e:  # keep running; report once
                self.objects = [{"label": f"yolo error: {str(e)[:40]}", "score": 0, "box": [0, 0, 0, 0], "poly": None}]
                time.sleep(2)

    # ------------------------------------------------------------------ Gemini Robotics-ER
    def ask_er(self, frame_bgr, target, query=None, mirrored=False):
        key = gemini_key()
        if key is None or self._busy_er:
            return False
        self._busy_er = True

        def run():
            t0 = time.time()
            try:
                ok, jpg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                q = query or "Point to every distinct object relevant to manipulation on the table or in the scene."
                prompt = (f"{q} The answer should follow the json format: "
                          '[{"point": [y, x], "label": <label>}, ...]. The points are in [y, x] format normalized to 0-1000.')
                body = {"contents": [{"parts": [{"inline_data": {"mime_type": "image/jpeg", "data": base64.b64encode(jpg.tobytes()).decode()}},
                                                {"text": prompt}]}],
                        "generationConfig": {"temperature": 0.4}}
                req = urllib.request.Request(API + ER_MODEL + ":generateContent", data=json.dumps(body).encode(),
                                             headers={"Content-Type": "application/json", "x-goog-api-key": key})
                with urllib.request.urlopen(req, timeout=30, context=ssl.create_default_context(cafile=certifi.where())) as resp:
                    data = json.loads(resp.read())
                text = "".join(p.get("text", "") for p in data["candidates"][0]["content"]["parts"])
                m = re.search(r"\[.*\]", text, re.S)
                pts = []
                for it in json.loads(m.group(0)) if m else []:
                    y, x = it["point"]
                    u = x / 1000.0
                    pts.append({"label": str(it.get("label", "?"))[:40], "u": round(1 - u if mirrored else u, 4), "v": round(y / 1000.0, 4)})
                self.er = {"target": target, "query": q, "points": pts[:12], "ms": int((time.time() - t0) * 1000), "model": ER_MODEL}
            except Exception as e:
                self.er = {"target": target, "query": query, "points": [], "ms": int((time.time() - t0) * 1000), "error": str(e)[:120], "model": ER_MODEL}
            finally:
                self._busy_er = False

        threading.Thread(target=run, daemon=True).start()
        return True

    def state(self):
        return {"model": self.model_name, "objects": self.objects, "er": self.er, "busy": self._busy_er}
