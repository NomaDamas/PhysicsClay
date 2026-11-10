"""Cloud image-to-3D, bring your own key. The key comes with the request (from the browser, or the server's .env outside
--byok), is used for that one job and is never stored, logged or put in state.

* fal_trellis   fal.ai queue API, model `fal-ai/trellis` (Microsoft TRELLIS, MIT weights; fal.ai's terms apply).
                POST https://queue.fal.run/fal-ai/trellis {image_url: data URI} -> status_url / response_url;
                a failed run is still COMPLETED, with `error` set. Output: model_mesh.url (GLB).
* meshy         Meshy image-to-3d (api.meshy.ai/openapi/v1/image-to-3d, Bearer key; data URIs documented).
                Output ownership: free plan -> Meshy owns it, CC BY 4.0; paid plan -> yours. API outputs expire in 3 days.
Not offered as defaults on purpose: Hunyuan3D 2.x (its community license excludes the EU, the UK and South Korea),
Stable Fast 3D / SPAR3D (Stability Community License, revenue-limited).
`fetch` is the only network seam (tests stub it)."""
import base64
import io
import json
import os
import ssl
import time
import urllib.error
import urllib.request
from pathlib import Path

import certifi
import cv2
import numpy as np

from .. import fit
from ..base import Recon, ReconError

ROOT = Path(__file__).resolve().parents[3]
SSL_CTX = ssl.create_default_context(cafile=certifi.where())
MAX_GLB = 80 << 20
Y_UP_TO_Z_UP = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])   # glTF (y up, front +z) -> x across, y away, z up


def fetch(method, url, headers=None, body=None, timeout=60):
    """-> (status, bytes). Never raises for HTTP errors (the caller reads the status)."""
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    h = dict(headers or {})
    if body is not None and not isinstance(body, bytes):
        h.setdefault("Content-Type", "application/json")
    h.setdefault("User-Agent", "PhysicsClay/recon")
    req = urllib.request.Request(url, data=data, headers=h, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout, context=SSL_CTX) as r:
            return r.status, r.read(MAX_GLB + 1)
    except urllib.error.HTTPError as e:
        try:
            return e.code, e.read(4000)
        except Exception:
            return e.code, b""


def env_key(name):
    """The server's own key (never in --byok mode)."""
    from ...spatial import byok
    if byok():
        return None
    v = os.environ.get(name)
    if v:
        return v.strip()
    env = ROOT / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.startswith(name + "="):
                return line.split("=", 1)[1].strip().strip('"').strip("'") or None
    return None


def data_uri(img, mask=None, side=1024, quality=88):
    """The object, cropped with a margin (on white when there's a mask), as a small JPEG data URI."""
    if mask is not None and mask.any():
        ys, xs = np.nonzero(mask)
        x0, y0, x1, y1 = xs.min(), ys.min(), xs.max() + 1, ys.max() + 1
        p = int(0.12 * max(x1 - x0, y1 - y0))
        h, w = img.shape[:2]
        x0, y0, x1, y1 = max(0, x0 - p), max(0, y0 - p), min(w, x1 + p), min(h, y1 + p)
        soft = cv2.GaussianBlur(mask.astype(np.float32), (0, 0), 2)[..., None]
        img = (img * soft + 255 * (1 - soft)).astype(np.uint8)[y0:y1, x0:x1]
    h, w = img.shape[:2]
    s = min(1.0, side / max(h, w))
    if s < 1:
        img = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()
    return "data:image/jpeg;base64," + base64.b64encode(jpg).decode()


def glb_to_recon(blob, provider, license_, mask, img, notes=()):
    """GLB bytes -> Recon: one mesh in this pipeline's frame, colours baked from the texture."""
    import trimesh
    try:
        loaded = trimesh.load(io.BytesIO(blob), file_type="glb", force="scene")
    except Exception as e:
        raise ReconError(f"{provider} sent a mesh I couldn't read ({str(e)[:60]}).") from None
    geoms = loaded.dump() if hasattr(loaded, "dump") else [loaded]
    parts, cols = [], []
    for g in geoms:
        if not isinstance(g, trimesh.Trimesh) or not len(g.faces):
            continue
        try:
            c = np.asarray(g.visual.to_color().vertex_colors, np.uint8)
        except Exception:
            c = None
        if c is None or len(c) != len(g.vertices):
            c = np.tile(np.array([170, 170, 175, 255], np.uint8), (len(g.vertices), 1))
        parts.append(trimesh.Trimesh(g.vertices, g.faces, process=False))
        cols.append(c)
    if not parts:
        raise ReconError(f"{provider} sent an empty mesh.")
    mesh = trimesh.util.concatenate(parts)
    colors = np.vstack(cols)
    mesh.vertices = np.asarray(mesh.vertices) @ Y_UP_TO_Z_UP.T
    fit.centre(mesh)
    rgb = fit.mask_color(img, mask) if mask is not None else colors[:, :3].mean(0) / 255
    ppu = None
    if mask is not None and mask.any():
        xs = np.nonzero(mask)[1]
        ppu = float(xs.max() - xs.min() + 1) / max(float(mesh.extents[0]), 1e-9)
    return Recon(mesh=mesh, provider=provider, shape="mesh", colors=colors, rgba=tuple(rgb) + (1.0,), metric=False,
                 px_per_unit=ppu, confidence=0.7, license=license_, notes=list(notes))


def _err(provider, status, body, key):
    try:
        j = json.loads(body or b"{}")
        d = j.get("detail") or j.get("message") or j.get("error") or j
        if isinstance(d, list) and d and isinstance(d[0], dict):
            d = d[0].get("msg") or d[0].get("type")
    except ValueError:
        d = (body or b"")[:120].decode(errors="replace")
    msg = {401: "the key was rejected", 402: "the account has no credits (Meshy's API needs a paid plan)", 403: "refused (key or content policy)",
           429: "too many requests, try again in a minute"}.get(status, f"HTTP {status}")
    return ReconError(f"{provider}: {msg} ({str(d)[:100]})".replace(key or "\0", "***"))


class FalTrellis:
    name = "fal_trellis"
    label = "Cloud: TRELLIS (fal.ai)"
    license = "MIT (Microsoft TRELLIS weights); fal.ai terms of service apply"
    offline = False
    key = "fal"
    env = "FAL_KEY"
    model = "fal-ai/trellis"
    note = "Full textured mesh from one photo, ~30-90 s. Needs a fal.ai key (pay per use)."

    def available(self, keys=None):
        k = (keys or {}).get(self.key) or env_key(self.env)
        return (True, None) if k else (False, "add a fal.ai key")

    def reconstruct(self, img, mask, hints=None, keys=None, progress=None, timeout=300, poll=2.0, **_):
        key = (keys or {}).get(self.key) or env_key(self.env)
        if not key:
            raise ReconError("TRELLIS on fal.ai needs a fal.ai key (Settings in the Real panel).")
        auth = {"Authorization": "Key " + key}
        st, body = fetch("POST", f"https://queue.fal.run/{self.model}", auth,
                         {"image_url": data_uri(img, mask), "texture_size": "1024", "mesh_simplify": 0.95})
        if st >= 300:
            raise _err("fal.ai", st, body, key)
        sub = json.loads(body)
        status_url, response_url = sub.get("status_url"), sub.get("response_url")
        rid = sub.get("request_id")
        if not status_url:
            status_url = f"https://queue.fal.run/{self.model}/requests/{rid}/status"
            response_url = f"https://queue.fal.run/{self.model}/requests/{rid}"
        t0 = time.time()
        while True:
            st, body = fetch("GET", status_url, auth)
            if st >= 300 and st != 202:
                raise _err("fal.ai", st, body, key)
            s = json.loads(body or b"{}")
            if s.get("status") == "COMPLETED":
                if s.get("error"):
                    raise ReconError(f"fal.ai: the model failed ({str(s.get('error'))[:100]})")
                break
            if progress:
                progress("shape", min(0.9, 0.2 + (time.time() - t0) / 120))
            if time.time() - t0 > timeout:
                raise ReconError("fal.ai didn't finish in time. Try again later or use the offline quick shape.")
            time.sleep(poll)
        st, body = fetch("GET", response_url, auth)
        if st >= 300:
            raise _err("fal.ai", st, body, key)
        out = json.loads(body)
        f = out.get("model_mesh") or out.get("model_glb") or {}
        url = f.get("url") if isinstance(f, dict) else None
        if not url:
            raise ReconError("fal.ai answered without a mesh.")
        st, blob = fetch("GET", url, timeout=120)
        if st >= 300 or len(blob) > MAX_GLB:
            raise ReconError("couldn't download the mesh from fal.ai.")
        return glb_to_recon(blob, self.name, self.license, mask, img, [f"TRELLIS via fal.ai ({self.model})"])


class Meshy:
    name = "meshy"
    label = "Cloud: Meshy"
    license = "Meshy terms: free plan outputs are CC BY 4.0 (owned by Meshy); paid plan outputs are yours"
    offline = False
    key = "meshy"
    env = "MESHY_API_KEY"
    api = "https://api.meshy.ai/openapi/v1/image-to-3d"
    note = "Textured mesh from one photo, ~1-3 min. Needs a Meshy key on a paid plan (API)."

    def available(self, keys=None):
        k = (keys or {}).get(self.key) or env_key(self.env)
        return (True, None) if k else (False, "add a Meshy key")

    def reconstruct(self, img, mask, hints=None, keys=None, progress=None, timeout=420, poll=3.0, **_):
        key = (keys or {}).get(self.key) or env_key(self.env)
        if not key:
            raise ReconError("Meshy needs a Meshy API key (Settings in the Real panel).")
        auth = {"Authorization": "Bearer " + key}
        st, body = fetch("POST", self.api, auth, {"image_url": data_uri(img, mask), "should_texture": True, "enable_pbr": False,
                                                   "should_remesh": True, "topology": "triangle", "target_polycount": 20000,
                                                   "target_formats": ["glb"]})
        if st >= 300:
            raise _err("Meshy", st, body, key)
        tid = json.loads(body).get("result")
        if not tid:
            raise ReconError("Meshy didn't start a task.")
        t0 = time.time()
        while True:
            st, body = fetch("GET", f"{self.api}/{tid}", auth)
            if st >= 300:
                raise _err("Meshy", st, body, key)
            t = json.loads(body)
            s = t.get("status")
            if s == "SUCCEEDED":
                break
            if s in ("FAILED", "CANCELED"):
                raise ReconError(f"Meshy: {((t.get('task_error') or {}).get('message') or s.lower())[:100]}")
            if progress:
                progress("shape", min(0.9, 0.15 + float(t.get("progress") or 0) / 120))
            if time.time() - t0 > timeout:
                raise ReconError("Meshy didn't finish in time. Try again later or use the offline quick shape.")
            time.sleep(poll)
        url = (t.get("model_urls") or {}).get("glb")
        if not url:
            raise ReconError("Meshy answered without a GLB.")
        st, blob = fetch("GET", url, timeout=120)
        if st >= 300 or len(blob) > MAX_GLB:
            raise ReconError("couldn't download the mesh from Meshy.")
        return glb_to_recon(blob, self.name, self.license, mask, img, ["Meshy image-to-3d"])
