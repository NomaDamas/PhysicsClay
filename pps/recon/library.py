"""Local object library: everything imported from photos, reusable in scene specs and data generation.

    library/objects/<id>/meta.json        what it is, where it came from (provider + its model's license, scale source),
                                          physical properties, sizes, file names
                    visual.obj            the visual mesh (metres, z up, centred on its bounding box; vertex colours too)
                    colors.u8             RGB per visual vertex, in the vertex order MuJoCo keeps (the 3D stage reads it)
                    collision_<k>.obj     convex pieces for collision
                    photo.jpg             the photo it came from (a crop: the thumbnail)

`library/` is user data (gitignored). PC_LIBRARY moves it (tests use a temp dir). Scene specs refer to an entry as
{"name": "my_mug", "kind": "cup", "mesh": "<id>", "scale": 1.0} (pps/recon/sim.py)."""
import json
import os
import re
import shutil
import time
import uuid
from pathlib import Path

import numpy as np

from .base import slug

ROOT = Path(__file__).resolve().parents[2]
VERSION = 1


def root():
    return Path(os.environ.get("PC_LIBRARY") or (ROOT / "library")) / "objects"


def valid_id(i):
    return bool(re.fullmatch(r"[a-z0-9_]{1,48}", str(i or "")))


def path(i):
    if not valid_id(i):
        raise ValueError("bad library id")
    return root() / i


def new_id(label):
    return f"{slug(label, 20) or 'thing'}_{uuid.uuid4().hex[:6]}"


def save(prep, meta, photo_jpg=None, lib_id=None):
    """prep: pps/recon/physical.py prepare() output (metres). meta: label, kind, provider, ... -> the saved meta."""
    i = lib_id or new_id(meta.get("label"))
    d = path(i)
    d.mkdir(parents=True, exist_ok=True)
    vis = prep["visual"]
    vis.export(d / "visual.obj")
    cols = prep.get("colors")
    if cols is not None and len(cols) == len(vis.vertices):
        (d / "colors.u8").write_bytes(np.ascontiguousarray(np.asarray(cols)[:, :3], np.uint8).tobytes())
    names = []
    for k, h in enumerate(prep["hulls"]):
        n = f"collision_{k}.obj"
        h.export(d / n)
        names.append(n)
    if photo_jpg:
        (d / "photo.jpg").write_bytes(photo_jpg)
    p = prep["props"]
    ext = np.asarray(vis.extents, float)
    rgba = meta.get("rgba") or (list((np.asarray(cols)[:, :3].mean(0) / 255).round(3)) + [1.0] if cols is not None else [0.6, 0.6, 0.62, 1.0])
    out = dict(meta)
    out.update(id=i, version=VERSION, created=out.get("created") or time.strftime("%Y-%m-%dT%H:%M:%S"),
               size=[round(float(x), 5) for x in ext], visual="visual.obj", colors="colors.u8" if (d / "colors.u8").exists() else None,
               collision=names, photo="photo.jpg" if photo_jpg else None, vertices=int(len(vis.vertices)), faces=int(len(vis.faces)),
               mass=round(p["mass"], 4), volume=float(p["volume"]), density=p["density"], friction=p["friction"], hollow=p["hollow"],
               material=p["material"], mass_source=p["source"], com=[round(float(x), 5) for x in p["com"]],
               inertia=[[float(x) for x in row] for row in np.asarray(p["inertia"])], watertight=prep["watertight"],
               collision_method=prep["method"], rgba=[round(float(x), 3) for x in rgba])
    (d / "meta.json").write_text(json.dumps(out, indent=1))
    return out


def load(i):
    """meta dict (+ "dir") or None."""
    try:
        d = path(i)
        meta = json.loads((d / "meta.json").read_text())
    except (ValueError, OSError):
        return None
    meta["dir"] = str(d)
    return meta


def listing(limit=60):
    out = []
    r = root()
    if not r.exists():
        return out
    for d in sorted(r.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)[:limit]:
        m = load(d.name)
        if m:
            out.append({k: m.get(k) for k in ("id", "label", "kind", "size", "mass", "material", "provider", "scale_source", "created", "rgba")})
    return out


def count():
    """Number of entries (cheap: no meta.json is read; the state frame asks every few seconds)."""
    r = root()
    try:
        return sum(1 for d in r.iterdir() if (d / "meta.json").exists()) if r.exists() else 0
    except OSError:
        return 0


def delete(i):
    d = path(i)
    if d.exists():
        shutil.rmtree(d)
        return True
    return False


def colors(i):
    """RGB uint8 bytes of the visual mesh's vertices, or None."""
    try:
        f = path(i) / "colors.u8"
        return f.read_bytes() if f.exists() else None
    except (ValueError, OSError):
        return None
