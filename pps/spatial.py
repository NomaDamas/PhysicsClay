"""Spatial understanding models.
- YOLO11 segmentation (ultralytics) on the user's camera, in a background thread: labelled masks -> polygons.
  Ultralytics is AGPL-3.0 and an optional extra (`pip install ultralytics`); without it the default is the
  MediaPipe object detector below and the Settings list says YOLO isn't installed (no error label on the camera).
- MediaPipe object detector (EfficientDet-Lite0, Apache-2.0, models/efficientdet_lite0.tflite from setup.sh), same
  background thread: labelled boxes, no masks.
- Gemini Robotics-ER 2 on demand: "what do you see / where is X" -> labelled points on a frame.
"""
import base64
import importlib.util
import json
import os
import re
import ssl
import threading
import time
import urllib.request
from pathlib import Path

import certifi
import cv2

ROOT = Path(__file__).resolve().parents[1]
API = "https://generativelanguage.googleapis.com/v1beta/models/"
ER_MODEL = "gemini-robotics-er-2-preview"
MODELS = ("yolo-seg", "mediapipe", "off")
NO_YOLO = "YOLO isn't installed (optional extra: pip install ultralytics, AGPL-3.0)."


def yolo_available():
    """ultralytics importable (checked without importing it: torch is slow and heavy to load)."""
    try:
        return importlib.util.find_spec("ultralytics") is not None
    except (ImportError, ValueError):
        return False


def byok():
    """Bring-your-own-key mode (public deployments): the server never uses its own API keys; visitors send theirs."""
    return os.environ.get("PC_BYOK") == "1"


def gemini_key():
    if byok():
        return None
    k = os.environ.get("GEMINI_API_KEY")
    if k:
        return k
    env = ROOT / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.startswith("GEMINI_API_KEY="):
                return line.split("=", 1)[1].strip()
    return None


def openai_key():
    """OPENAI_API_KEY from the environment or .env; None in BYOK mode (visitors bring their own)."""
    if byok():
        return None
    k = os.environ.get("OPENAI_API_KEY")
    if k:
        return k.strip()
    env = ROOT / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.startswith("OPENAI_API_KEY="):
                return line.split("=", 1)[1].strip()
    return None


class Spatial:
    def __init__(self):
        self.yolo_ok = yolo_available()
        self.model_name = "yolo-seg" if self.yolo_ok else "mediapipe"
        self.error = None          # the last detector problem, as a calm status line (never drawn as an object)
        self.objects = []
        self.er = None
        self._frame = None
        self._lock = threading.Lock()
        self._yolo = None
        self._mp = None
        self._busy_er = False
        self.paused = False
        threading.Thread(target=self._loop, daemon=True).start()

    def set_model(self, name):
        name = name if name in MODELS else ("yolo-seg" if self.yolo_ok else "mediapipe")
        if name == "yolo-seg" and not self.yolo_ok:
            name = "mediapipe"
        self.error = None
        if name != self.model_name:
            self.objects = []
        self.model_name = name

    def feed(self, frame_bgr):
        with self._lock:
            self._frame = frame_bgr

    def _loop(self):
        while True:
            time.sleep(0.6)
            if self.model_name not in ("yolo-seg", "mediapipe") or self.paused:
                continue
            with self._lock:
                f, self._frame = self._frame, None
            if f is None:
                continue
            if self.model_name == "mediapipe":
                self._mediapipe(f)
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
                self.objects, self.error = out, None
            except ImportError:      # ultralytics (or torch) missing / broken: the MediaPipe detector instead
                self.yolo_ok, self.model_name, self.objects = False, "mediapipe", []
            except Exception as e:  # keep running; a status line, not an object on the camera
                self.objects, self.error = [], f"YOLO is unavailable right now ({str(e)[:60]})."
                time.sleep(2)

    def _mediapipe(self, f):
        """MediaPipe object detector on one frame -> self.objects (boxes, mirrored like the YOLO path)."""
        try:
            if self._mp is None:
                import mediapipe as mp
                from mediapipe.tasks.python import vision
                from mediapipe.tasks.python.core.base_options import BaseOptions
                path = ROOT / "models" / "efficientdet_lite0.tflite"
                if not path.exists():
                    self.objects, self.error = [], "MediaPipe object model missing (run scripts/setup.sh)."
                    time.sleep(5)
                    return
                self._mp = (mp, vision.ObjectDetector.create_from_options(vision.ObjectDetectorOptions(
                    base_options=BaseOptions(model_asset_path=str(path)), running_mode=vision.RunningMode.IMAGE,
                    max_results=8, score_threshold=0.4)))
            mp, det = self._mp
            h, w = f.shape[:2]
            small = cv2.resize(f, (416, max(1, int(h * 416 / w))))
            img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
            sh, sw = small.shape[:2]
            out = []
            for d in det.detect(img).detections:
                b, c = d.bounding_box, d.categories[0]
                x1, y1, x2, y2 = b.origin_x / sw, b.origin_y / sh, (b.origin_x + b.width) / sw, (b.origin_y + b.height) / sh
                out.append({"label": c.category_name, "score": round(float(c.score), 2),
                            "box": [round(1 - x2, 4), round(y1, 4), round(1 - x1, 4), round(y2, 4)], "poly": None})
            if self.model_name == "mediapipe":
                self.objects, self.error = out, None
        except Exception as e:
            self.objects, self.error = [], f"MediaPipe objects unavailable ({str(e)[:60]})."
            time.sleep(2)

    # ------------------------------------------------------------------ Gemini Robotics-ER
    def ask_er(self, frame_bgr, target, query=None, mirrored=False, key=None):
        key = key or gemini_key()
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
                data, last_err, used = None, None, ER_MODEL
                for model in (ER_MODEL, "gemini-3.6-flash", "gemini-3.5-flash"):   # the ER preview is often busy: fall back
                    req = urllib.request.Request(API + model + ":generateContent", data=json.dumps(body).encode(),
                                                 headers={"Content-Type": "application/json", "x-goog-api-key": key, "User-Agent": "curl/8.7.1"})
                    try:
                        with urllib.request.urlopen(req, timeout=20, context=ssl.create_default_context(cafile=certifi.where())) as resp:
                            data, used = json.loads(resp.read()), model
                            break
                    except Exception as e:
                        last_err = e
                if data is None:
                    raise last_err
                text = "".join(p.get("text", "") for p in data["candidates"][0]["content"]["parts"])
                m = re.search(r"\[.*\]", text, re.S)
                pts = []
                for it in json.loads(m.group(0)) if m else []:
                    y, x = it["point"]
                    u = x / 1000.0
                    pts.append({"label": str(it.get("label", "?"))[:40], "u": round(1 - u if mirrored else u, 4), "v": round(y / 1000.0, 4)})
                self.er = {"target": target, "query": q, "points": pts[:12], "ms": int((time.time() - t0) * 1000), "model": used}
            except Exception as e:
                self.er = {"target": target, "query": query, "points": [], "ms": int((time.time() - t0) * 1000), "error": str(e)[:120], "model": ER_MODEL}
            finally:
                self._busy_er = False

        threading.Thread(target=run, daemon=True).start()
        return True

    def state(self):
        return {"model": self.model_name, "objects": self.objects, "er": self.er, "busy": self._busy_er,
                "yolo": self.yolo_ok, "note": self.error or (None if self.yolo_ok else NO_YOLO)}
