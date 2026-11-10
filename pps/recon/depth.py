"""Monocular depth: Depth Anything V2 **Small** (Apache-2.0) as ONNX on onnxruntime's CPU provider.

Only the Small model is used: the Base / Large / Giant weights are CC BY-NC 4.0 and must not be bundled or downloaded
here. The ONNX export (onnx-community/depth-anything-v2-small, Apache-2.0, ~99 MB) is fetched on first use into
models/ (pinned revision + SHA-256); `PC_RECON_DOWNLOAD=0` turns the download off (offline machines, CI). Without
onnxruntime or the file the pipeline still works from the outline alone (pps/recon/fit.py silhouette_shape).

Output: relative inverse depth (bigger = nearer), affine-invariant: shapes, not metres (pps/recon/fit.py turns it into
pseudo depth; the metric scale comes from what the person says, a marker or the VLM). ~1-2 s per photo on a laptop CPU."""
import hashlib
import importlib.util
import os
import threading
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models" / "depth_anything_v2_small.onnx"
REV = "4472b7362082ad9968fee890ca0f1e5aca36b93d"
URL = f"https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/{REV}/onnx/model.onnx"
SHA256 = "afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c"
SIZE_MB = 99
LICENSE = "Apache-2.0 (Depth Anything V2 Small, onnx-community/depth-anything-v2-small)"
MEAN, STD = np.array([0.485, 0.456, 0.406]), np.array([0.229, 0.224, 0.225])

_lock = threading.Lock()
_sess = None
_dl = {"busy": False, "error": None}


def runtime_ok():
    try:
        return importlib.util.find_spec("onnxruntime") is not None
    except (ImportError, ValueError):
        return False


def download_allowed():
    return os.environ.get("PC_RECON_DOWNLOAD", "1").strip().lower() not in ("0", "false", "off", "no")


def available():
    """Ready to run right now (no download needed)."""
    return runtime_ok() and MODEL.exists()


def status():
    return {"model": "Depth Anything V2 Small", "license": "Apache-2.0", "runtime": runtime_ok(), "downloaded": MODEL.exists(),
            "size_mb": SIZE_MB, "downloading": _dl["busy"], "error": _dl["error"],
            "ready": available(), "can_download": runtime_ok() and download_allowed()}


def ensure(progress=None):
    """The model file, downloading it once (verified) if needed. -> Path, or None when it can't be had."""
    if MODEL.exists():
        return MODEL
    if not (runtime_ok() and download_allowed()):
        return None
    with _lock:
        if MODEL.exists():
            return MODEL
        _dl.update(busy=True, error=None)
        tmp = MODEL.with_suffix(".onnx.part")
        try:
            MODEL.parent.mkdir(parents=True, exist_ok=True)
            h = hashlib.sha256()
            req = urllib.request.Request(URL, headers={"User-Agent": "PhysicsClay/recon"})
            import ssl

            import certifi
            with urllib.request.urlopen(req, timeout=60, context=ssl.create_default_context(cafile=certifi.where())) as r, open(tmp, "wb") as f:
                total = int(r.headers.get("Content-Length") or SIZE_MB * 1e6)
                got = 0
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
                    h.update(chunk)
                    got += len(chunk)
                    if progress:
                        progress(min(1.0, got / total))
            if h.hexdigest() != SHA256:
                raise OSError("checksum mismatch")
            tmp.replace(MODEL)
            return MODEL
        except Exception as e:                       # offline, proxy, disk: the outline-only path still works
            _dl["error"] = f"depth model download failed ({str(e)[:80]})"
            try:
                tmp.unlink()
            except OSError:
                pass
            return None
        finally:
            _dl["busy"] = False


def _session():
    global _sess
    if _sess is None:
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = int(os.environ.get("PC_RECON_THREADS", "2"))
        so.log_severity_level = 3
        _sess = ort.InferenceSession(str(MODEL), so, providers=["CPUExecutionProvider"])
    return _sess


def estimate(img_bgr, side=518):
    """BGR image -> relative inverse depth at the image's size (float32, bigger = nearer), or None if unavailable."""
    if not available():
        return None
    h, w = img_bgr.shape[:2]
    s = side / max(h, w)
    nh, nw = max(14, int(round(h * s / 14)) * 14), max(14, int(round(w * s / 14)) * 14)
    x = cv2.resize(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB), (nw, nh), interpolation=cv2.INTER_CUBIC).astype(np.float32) / 255.0
    x = ((x - MEAN) / STD).transpose(2, 0, 1)[None].astype(np.float32)
    with _lock:
        sess = _session()
        y = sess.run(None, {sess.get_inputs()[0].name: x})[0]
    d = np.asarray(y, np.float32).reshape(y.shape[-2], y.shape[-1])
    return cv2.resize(d, (w, h), interpolation=cv2.INTER_LINEAR)
