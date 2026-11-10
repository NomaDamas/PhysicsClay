"""Geometry a skill is applied to: seams (polylines on the workpiece), regions to wipe, containers to stir, targets to
press. "Geometry gives precision": the seam comes from the scene, never from the person's hand.

The world dict (built by pps/features/skills.py from the live scene, or by hand in tests):
    {"surface": 0.74, "hand": [x, y, z],
     "objects": {name: {"pos": [x, y, z] centre, "r": radius, "h": half height, "kind": "steel_plate", "size": [sx, sy, sz]}},
     "seams": {name: {"points": [[x, y, z], ...] on the work surface, "label": "L seam", "between": [obj, obj]}},
     "welded": [seam names already welded this session]}
"""
import math
import re

import numpy as np

from .core import SkillError

CONTAINERS = ("bowl", "pot", "cup", "mug", "pan", "jar", "can", "box")


def _words(s):
    return [w for w in re.split(r"[^a-z0-9]+", str(s or "").lower()) if w and w not in ("the", "a", "an", "seam", "weld", "joint")]


def polyline_length(P):
    P = np.asarray(P, float)
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))) if len(P) > 1 else 0.0


def resample(P, step):
    """Points every `step` metres along a polyline, keeping its corners."""
    P = np.asarray(P, float)
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def dist_to_polyline(p, P, horizontal=True):
    """Shortest distance from p to the polyline (horizontal only by default) and the arc position there."""
    p, P = np.asarray(p, float), np.asarray(P, float)
    k = 2 if horizontal else 3
    best, s_best, s0 = 1e9, 0.0, 0.0
    for a, b in zip(P[:-1], P[1:]):
        ab = b[:k] - a[:k]
        L2 = float(ab @ ab)
        u = 0.0 if L2 < 1e-12 else float(np.clip((p[:k] - a[:k]) @ ab / L2, 0, 1))
        d = float(np.linalg.norm(p[:k] - (a[:k] + u * ab)))
        if d < best:
            best, s_best = d, s0 + u * math.sqrt(L2)
        s0 += math.sqrt(L2)
    return best, s_best


def seams(world):
    out = {}
    for name, sm in (world.get("seams") or {}).items():
        pts = np.asarray(sm.get("points") or [], float)
        if pts.ndim == 2 and len(pts) >= 2 and pts.shape[1] == 3:
            out[name] = {"points": pts, "label": sm.get("label") or name.replace("_", " "), "between": list(sm.get("between") or [])}
    return out


def _seam_score(name, sm, words):
    have = set(_words(name)) | set(_words(sm["label"]))
    if "l" in words and ("l" in have or "corner" in have):
        return 3
    return len(set(words) & have)


def resolve_seam(target, world, prefer_other=True):
    """target: a seam name / label words ('the L seam', 'straight'), an object name (a plate: its seam, else its long edge
    nearest the robot), [[x, y, z], ...] (a polyline), 'other' / 'next' (one not welded yet) or None (the first seam not
    welded yet). -> (name, polyline (M, 3), label, between)."""
    if isinstance(target, (list, tuple)) and target and isinstance(target[0], (list, tuple)):
        P = np.asarray(target, float)
        if P.ndim != 2 or P.shape[1] != 3 or len(P) < 2:
            raise SkillError("a seam polyline needs at least two [x, y, z] points")
        return "custom", P, "the seam you gave me", []
    sm_all = seams(world)
    welded = list(world.get("welded") or [])
    t = str(target or "").strip().lower()
    if t in ("", "auto", "any", "other", "the other", "the other seam", "next", "the next seam", "next seam"):
        if not sm_all:
            raise SkillError("there is no seam in this scene (load the weld cell, or give the seam as points)")
        order = [n for n in sm_all if n not in welded] or list(sm_all)
        if t in ("other", "the other", "the other seam") and welded:
            order = [n for n in sm_all if n != welded[-1]] or order
        n = order[0]
        return n, sm_all[n]["points"], sm_all[n]["label"], sm_all[n]["between"]
    key = t.replace(" ", "_")
    if key in sm_all:
        return key, sm_all[key]["points"], sm_all[key]["label"], sm_all[key]["between"]
    objs = world.get("objects") or {}
    obj = key if key in objs else next((n for n in objs if key and (key in n or n in key)), None)
    if obj is not None:
        for n, sm in sm_all.items():
            if obj in sm["between"]:
                return n, sm["points"], sm["label"], sm["between"]
        return f"edge_of_{obj}", object_edge(objs[obj], world), f"the edge of the {obj.replace('_', ' ')}", [obj]
    words = _words(t)
    if sm_all and words:
        best = max(sm_all, key=lambda n: _seam_score(n, sm_all[n], words))
        if _seam_score(best, sm_all[best], words) > 0:
            return best, sm_all[best]["points"], sm_all[best]["label"], sm_all[best]["between"]
    have = ", ".join(sm["label"] for sm in sm_all.values()) or "none"
    raise SkillError(f"I don't see a seam called '{target}' (seams here: {have})")


def object_top(o):
    return float(o["pos"][2]) + float(o.get("h", 0.0))


def footprint(o):
    """Axis-aligned rectangle [x0, y0, x1, y1] of an object's top face (a cylinder: the square inside its circle)."""
    c = np.asarray(o["pos"], float)
    size = o.get("size")
    if size and o.get("shape", "box") == "box":
        hx, hy = float(size[0]) / 2, float(size[1]) / 2
    else:
        r = float(o.get("r", 0.05))
        hx = hy = r / math.sqrt(2)
    return [c[0] - hx, c[1] - hy, c[0] + hx, c[1] + hy]


def object_edge(o, world):
    """The long top edge of a box object nearest the robot's hand (or the robot)."""
    x0, y0, x1, y1 = footprint(o)
    z = object_top(o)
    edges = [[[x0, y0, z], [x0, y1, z]], [[x1, y0, z], [x1, y1, z]], [[x0, y0, z], [x1, y0, z]], [[x0, y1, z], [x1, y1, z]]]
    ref = np.asarray(world.get("hand") or [0.0, 0.0, z], float)
    L = [math.dist(e[0], e[1]) for e in edges]
    longest = [e for e, l_ in zip(edges, L) if l_ >= max(L) - 1e-6]
    return np.asarray(min(longest, key=lambda e: dist_to_polyline(ref, e)[0]), float)


def region_rect(target, world):
    """Wipe region -> ([x0, y0, x1, y1], top z, label, object name | None)."""
    objs = world.get("objects") or {}
    surface = float(world.get("surface", 0.74))
    if isinstance(target, (list, tuple)) and len(target) == 4:
        r = [float(v) for v in target]
        return [min(r[0], r[2]), min(r[1], r[3]), max(r[0], r[2]), max(r[1], r[3])], surface, "that area", None
    t = str(target or "").strip().lower().replace(" ", "_")
    name = t if t in objs else next((n for n in objs if t and (t in n or n in t)), None)
    if name is None and not t:
        flat = [n for n, o in objs.items() if str(o.get("kind")) in ("plate", "steel_plate", "box", "board", "tray")]
        name = flat[0] if flat else None
    if name is not None:
        o = objs[name]
        return footprint(o), object_top(o), f"the {name.replace('_', ' ')}", name
    regions = world.get("regions") or {}
    if t in regions:
        x, y = regions[t][:2]
        return [x - 0.08, y - 0.08, x + 0.08, y + 0.08], surface, f"the {t} of the table", None
    raise SkillError(f"I don't know what to wipe: '{target}'")


def container(target, world):
    """-> (name, object dict) of the container to stir in."""
    objs = world.get("objects") or {}
    t = str(target or "").strip().lower().replace(" ", "_")
    if t:
        name = t if t in objs else next((n for n in objs if t in n or n in t), None)
        if name is None:
            raise SkillError(f"I don't see '{target}' to stir in")
        return name, objs[name]
    cands = [n for n, o in objs.items() if str(o.get("kind")) in CONTAINERS]
    if not cands:
        raise SkillError("there's nothing to stir in here (a bowl, pot or cup)")
    hand = np.asarray(world.get("hand") or [0, 0, 0], float)
    name = min(cands, key=lambda n: float(np.linalg.norm(np.asarray(objs[n]["pos"][:2]) - hand[:2])))
    return name, objs[name]


def press_target(target, world):
    """-> (point on top of the target, label, object name | None)."""
    objs = world.get("objects") or {}
    if isinstance(target, (list, tuple)) and len(target) == 3:
        return np.asarray(target, float), "that spot", None
    t = str(target or "").strip().lower().replace(" ", "_")
    name = t if t in objs else next((n for n in objs if t and (t in n or n in t)), None)
    if name is None:
        markers = world.get("markers") or {}
        mk = markers.get(str(target or "").strip().upper()[-1:]) if target else None
        if mk is not None:
            return np.asarray(mk, float), f"marker {str(target).upper()[-1:]}", None
        raise SkillError(f"I don't see '{target}' to press on" if target else "say what to press on")
    o = objs[name]
    return np.array([o["pos"][0], o["pos"][1], object_top(o)]), f"the {name.replace('_', ' ')}", name
