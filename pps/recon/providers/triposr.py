"""`triposr`: TripoSR (MIT, Stability AI x Tripo; code github.com/VAST-AI-Research/TripoSR, weights stabilityai/TripoSR
~1.7 GB) on this machine: one photo -> a coloured mesh in ~10-60 s on a laptop CPU.

Opt-in only: it needs torch, the TripoSR source (not on PyPI: clone it and point PC_TRIPOSR_DIR at it, or put it at
vendor/TripoSR), its requirements (omegaconf, einops, transformers, torchmcubes) and PC_TRIPOSR=1, which allows the
one-time 1.7 GB weights download from Hugging Face (or PC_TRIPOSR_WEIGHTS = a local folder with config.yaml +
model.ckpt). Nothing is downloaded otherwise, and tests use a stub (`_load` is the only seam)."""
import importlib.util
import os
import sys
import threading
from pathlib import Path

import cv2
import numpy as np

from .. import fit
from ..base import Recon, ReconError

ROOT = Path(__file__).resolve().parents[3]
LICENSE = "MIT (TripoSR code and weights, stabilityai/TripoSR)"
# TripoSR's raw meshes are z-up with the input view looking along +x; this turns them so x runs across the photo
# (the convention every provider follows: x across, y away from the camera, z up)
TO_PC = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_model = {"m": None, "device": None}
_lock = threading.Lock()


def _src():
    d = os.environ.get("PC_TRIPOSR_DIR") or str(ROOT / "vendor" / "TripoSR")
    return Path(d)


def _has(mod):
    try:
        return importlib.util.find_spec(mod) is not None
    except (ImportError, ValueError):
        return False


def enabled():
    return os.environ.get("PC_TRIPOSR", "0").strip().lower() in ("1", "true", "on", "yes")


def installed():
    """-> (ok, why not)."""
    if not _has("torch"):
        return False, "needs PyTorch (pip install 'physicsclay[recon]')"
    if not (_has("tsr") or (_src() / "tsr").exists()):
        return False, "needs the TripoSR source (clone github.com/VAST-AI-Research/TripoSR to vendor/TripoSR)"
    for m in ("omegaconf", "einops", "transformers"):
        if not _has(m):
            return False, f"needs {m} (pip install 'physicsclay[recon]')"
    return True, None


def _load():
    """The TSR model (once). Seam for tests: monkeypatch this to return a stub."""
    with _lock:
        if _model["m"] is None:
            if str(_src()) not in sys.path and (_src() / "tsr").exists():
                sys.path.insert(0, str(_src()))
            import torch
            from tsr.system import TSR
            src = os.environ.get("PC_TRIPOSR_WEIGHTS") or "stabilityai/TripoSR"
            m = TSR.from_pretrained(src, config_name="config.yaml", weight_name="model.ckpt")
            m.renderer.set_chunk_size(8192)
            dev = "mps" if os.environ.get("PC_TRIPOSR_MPS") == "1" and torch.backends.mps.is_available() else "cpu"
            m.to(dev)
            _model.update(m=m, device=dev)
        return _model["m"], _model["device"]


def framed(img, mask, side=512, fill=0.85):
    """TripoSR's input: the object cut out on 50 % grey, centred, filling `fill` of a square -> RGB uint8."""
    ys, xs = np.nonzero(mask)
    x0, y0, x1, y1 = xs.min(), ys.min(), xs.max() + 1, ys.max() + 1
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(float)
    a = mask.astype(float)[..., None]
    comp = (rgb * a + 127.5 * (1 - a))[y0:y1, x0:x1]
    s = fill * side / max(x1 - x0, y1 - y0)
    small = cv2.resize(comp.astype(np.uint8), (max(1, int((x1 - x0) * s)), max(1, int((y1 - y0) * s))), interpolation=cv2.INTER_AREA)
    out = np.full((side, side, 3), 128, np.uint8)
    oy, ox = (side - small.shape[0]) // 2, (side - small.shape[1]) // 2
    out[oy:oy + small.shape[0], ox:ox + small.shape[1]] = small
    return out


class TripoSR:
    name = "triposr"
    label = "Local AI (TripoSR)"
    license = LICENSE
    offline = True
    key = None
    note = "Single photo to a full mesh on this computer. Opt-in: ~1.7 GB of weights, 10-60 s per object on a CPU."

    def available(self, keys=None):
        ok, why = installed()
        if not ok:
            return False, why
        if not enabled() and not os.environ.get("PC_TRIPOSR_WEIGHTS"):
            return False, "off: set PC_TRIPOSR=1 to allow the one-time 1.7 GB weights download"
        return True, None

    def reconstruct(self, img, mask, hints=None, progress=None, resolution=None, **_):
        if mask is None or mask.sum() < 50:
            raise ReconError("I couldn't find the object's outline in that photo. Drag a box around it and try again.")
        model, device = _load()
        from PIL import Image
        pil = Image.fromarray(framed(img, mask))
        if progress:
            progress("shape", 0.3)
        try:
            import torch
            ctx = torch.no_grad()
        except ImportError:                                # the stub in tests
            import contextlib
            ctx = contextlib.nullcontext()
        with ctx:
            codes = model([pil], device=device)
            meshes = model.extract_mesh(codes, True, resolution=int(resolution or os.environ.get("PC_TRIPOSR_RES", 192)))
        mesh = meshes[0]
        cols = None
        vc = getattr(getattr(mesh, "visual", None), "vertex_colors", None)
        if vc is not None and len(vc) == len(mesh.vertices):
            cols = np.asarray(vc, np.uint8)
        mesh.vertices = np.asarray(mesh.vertices) @ TO_PC.T
        fit.centre(mesh)
        ys, xs = np.nonzero(mask)
        ppu = float(xs.max() - xs.min() + 1) / max(float(mesh.extents[0]), 1e-9)
        rgb = fit.mask_color(img, mask)
        return Recon(mesh=mesh, provider=self.name, shape="mesh", colors=cols, rgba=tuple(rgb) + (1.0,), metric=False,
                     px_per_unit=ppu, confidence=0.6, license=LICENSE, notes=[f"TripoSR on {device}"])
