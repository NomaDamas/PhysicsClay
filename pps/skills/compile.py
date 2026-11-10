"""Skill + world -> a P2 trajectory (pps/traj.py schema, source "skill") or, for behaviours, the messages that run them.

    compile(skill, world, target=None) -> {"kind": "trajectory", "traj": {...}, "target": {...}, "say": "..."}
                                        | {"kind": "watch", "msg": {type: "watch", ...}}    (spot)
                                        | {"kind": "task", "msg": {type: "task", ...}}      (mirror_motion)

Trajectory skills keep the scene's geometry exact (the seam polyline, the region, the container, the target) and add the
person's style: speed, weave, angles, force, pattern. Every trajectory carries meta.skill (name, kind, params, the target
and what the runtime needs, e.g. the seam polyline and the arc-on span for the bead) and meta.work_on (objects the tool
works on, so the trajectory check lets it touch or enter them).
"""
import math

import numpy as np

from .. import traj as T
from . import core, geom

APPROACH = 0.06          # m above the work for the approach and retreat
CORNER_LEAD = 0.008      # m: extra points around a seam corner so the path's corner blend stays within ~1 mm


def _r(p, n=4):
    return [round(float(x), n) for x in p]


def _skill_meta(sk, target, **kw):
    return dict({"name": sk["name"], "kind": sk["kind"], "params": sk["params"], "target": target}, **kw)


def _orient(params):
    """(tilt, roll) for P2 points from the weld angles: tilt = travel angle (+ push), roll = 90 - work angle."""
    return float(params["travel_angle"]), 90.0 - float(params["work_angle"])


def weave_seam(sk, world, target=None):
    p = sk["params"]
    name, P, label, between = geom.resolve_seam(target if target is not None else p.get("seam"), world)
    P = np.asarray(P, float)
    hand = np.asarray(world.get("hand") or P[0], float)
    if p.get("direction") == "reverse" or (p.get("direction") == "auto" and np.linalg.norm(hand[:2] - P[-1][:2]) < np.linalg.norm(hand[:2] - P[0][:2])):
        P = P[::-1]
    up = np.array([0.0, 0.0, 1.0])
    so = float(p["standoff"])
    v = float(p["travel_speed"])
    tilt, roll = _orient(p)
    # seam points: the ends, every corner with a short lead in and out (keeps the path's corner rounding ~1 mm)
    seam_pts = [P[0]]
    for i in range(1, len(P) - 1):
        a, c, b = P[i - 1], P[i], P[i + 1]
        u_in, u_out = (c - a) / max(np.linalg.norm(c - a), 1e-9), (b - c) / max(np.linalg.norm(b - c), 1e-9)
        if float(u_in @ u_out) < math.cos(math.radians(8)):
            lead = min(CORNER_LEAD, 0.3 * np.linalg.norm(c - a), 0.3 * np.linalg.norm(b - c))
            seam_pts += [c - u_in * lead, c, c + u_out * lead]
        else:
            seam_pts.append(c)
    seam_pts.append(P[-1])
    pts = [{"p": _r(seam_pts[0] + up * (so + APPROACH)), "speed": 0.08, "note": "approach"}]
    for j, q in enumerate(seam_pts):
        pt = {"p": _r(q + up * so), "speed": v}
        if tilt:
            pt["tilt"] = round(tilt, 1)
        if roll:
            pt["roll"] = round(roll, 1)
        if j == 0:
            pt.update(hold=0.3, note="arc on")
        elif j == len(seam_pts) - 1:
            pt.update(hold=0.4, note="arc off")
        pts.append(pt)
    pts.append({"p": _r(seam_pts[-1] + up * (so + APPROACH)), "speed": 0.08, "note": "retreat"})
    w = p["weave"]
    pattern = None
    if float(w["amp"]) >= 0.0005:
        pattern = {"kind": w["kind"], "amp": float(w["amp"]), "freq": float(w["freq"]), "axis": "lateral", "span": [1, len(pts) - 2]}
    desc = core.describe(sk)
    length = geom.polyline_length(P)
    tr = T.normalize({"id": "skill", "role": "right", "frame": "world", "source": "skill", "label": f"{label}: {desc}",
                      "points": pts, "pattern": pattern,
                      "meta": {"skill": _skill_meta(sk, name, seam=name, seam_label=label, polyline=[_r(x) for x in P],
                                                    standoff=so, arc=[1, len(pts) - 2], bead_r=max(0.0025, 0.5 * float(w["amp"]) + 0.002),
                                                    length=round(length, 4)),
                               "work_on": list(between), "notes": [f"along {label}, {length * 100:.0f} cm"]}})
    return {"kind": "trajectory", "traj": tr, "target": {"seam": name, "label": label},
            "say": f"I'll weld {label} ({length * 100:.0f} cm): {desc}."}


def wipe(sk, world, target=None):
    p = sk["params"]
    rect, top, label, obj = geom.region_rect(target if target is not None else p.get("region"), world)
    x0, y0, x1, y1 = rect
    inset = 0.02
    x0, y0, x1, y1 = x0 + inset, y0 + inset, x1 - inset, y1 - inset
    if x1 - x0 < 0.01 or y1 - y0 < 0.01:
        raise core.SkillError(f"{label} is too small to wipe")
    z = top + 0.003
    sp = float(p["spacing"])
    path = []
    if p["pattern"] == "spiral":
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        rmax = min(x1 - x0, y1 - y0) / 2
        th, step = 0.0, math.radians(20)
        while True:
            r = sp * th / (2 * math.pi)
            if r > rmax:
                break
            path.append([cx + r * math.cos(th), cy + r * math.sin(th), z])
            th += step
    else:
        long_x = (x1 - x0) >= (y1 - y0)            # strokes along the long side
        a0, a1, b0, b1 = (x0, x1, y0, y1) if long_x else (y0, y1, x0, x1)
        n = max(2, int(math.floor((b1 - b0) / sp)) + 1)
        bs = np.linspace(b0, b1, n)
        for i, b in enumerate(bs):
            ends = (a0, a1) if i % 2 == 0 else (a1, a0)
            for a in ends:
                path.append([a, b, z] if long_x else [b, a, z])
    passes = int(p["passes"])
    seq = []
    for k in range(passes):
        seq += path if k % 2 == 0 else path[::-1]
    F, v = float(p["force"]), float(p["speed"])
    pts = [{"p": _r([seq[0][0], seq[0][1], z + APPROACH]), "speed": 0.1, "note": "approach"}]
    for j, q in enumerate(seq):
        pt = {"p": _r(q), "speed": v}
        if j == 0:
            pt.update(force=F, hold=0.25, note="press")       # settle on the surface before the first stroke
        pts.append(pt)
    pts[-1].update(force=0.0, hold=0.1, note="release")
    pts.append({"p": _r([seq[-1][0], seq[-1][1], z + APPROACH]), "speed": 0.1, "note": "lift"})
    if len(pts) > T.MAX_POINTS:
        raise core.SkillError("that wipe needs too many strokes; make the spacing wider")
    desc = core.describe(sk)
    tr = T.normalize({"id": "skill", "role": "right", "frame": "world", "source": "skill", "label": f"{label}: {desc}",
                      "points": pts, "meta": {"skill": _skill_meta(sk, obj or label, rect=_r(rect), top=round(top, 4)),
                                              "work_on": [obj] if obj else [], "notes": [f"over {label}"]}})
    return {"kind": "trajectory", "traj": tr, "target": {"region": obj or rect, "label": label}, "say": f"I'll wipe {label}: {desc}."}


def stir(sk, world, target=None):
    p = sk["params"]
    name, o = geom.container(target if target is not None else p.get("container"), world)
    c = np.asarray(o["pos"], float)
    top = geom.object_top(o)
    bottom = float(o["pos"][2]) - float(o.get("h", 0.0))
    r_in = max(0.006, float(o.get("r", 0.05)) - 0.012)
    R = min(float(p["radius"]), r_in)
    z = max(bottom + 0.012, top - float(p["depth"]))
    turns = int(p["turns"])
    sign = 1.0 if p["direction"] == "ccw" else -1.0
    speed = max(0.005, 2 * math.pi * R * float(p["freq"]))
    m = 16
    pts = [{"p": _r([c[0], c[1], top + APPROACH]), "speed": 0.08, "note": "above"},
           {"p": _r([c[0] + R, c[1], z]), "speed": speed, "note": "in"}]
    for j in range(1, m * turns + 1):
        a = sign * 2 * math.pi * j / m
        pts.append({"p": _r([c[0] + R * math.cos(a), c[1] + R * math.sin(a), z]), "speed": speed})
    pts.append({"p": _r([c[0], c[1], top + APPROACH]), "speed": 0.08, "note": "out"})
    note = f"radius {R * 100:.1f} cm (the {name.replace('_', ' ')} is {r_in * 100:.1f} cm inside)" if R < float(p["radius"]) - 1e-4 else None
    desc = core.describe(sk)
    label = f"the {name.replace('_', ' ')}"
    tr = T.normalize({"id": "skill", "role": "right", "frame": f"object:{name}", "source": "skill", "label": f"stir {label}: {desc}",
                      "points": [dict(q, p=_r(np.asarray(q["p"]) - np.array([c[0], c[1], top]))) for q in pts],
                      "meta": {"skill": _skill_meta(sk, name, radius=round(R, 4), z=round(z, 4)), "work_on": [name],
                               "notes": [n for n in (note, f"inside {label}") if n]}})
    return {"kind": "trajectory", "traj": tr, "target": {"container": name, "label": label}, "say": f"I'll stir {label}: {desc}."}


def press(sk, world, target=None):
    p = sk["params"]
    at, label, obj = geom.press_target(target if target is not None else p.get("target"), world)
    F, hold = float(p["force"]), float(p["hold_s"])
    pts = [{"p": _r(at + [0, 0, 0.08]), "speed": 0.08, "note": "above"},
           {"p": _r(at + [0, 0, 0.003]), "speed": 0.02, "force": F, "hold": hold, "note": "press"},
           {"p": _r(at + [0, 0, 0.08]), "speed": 0.05, "force": 0.0, "note": "release"}]
    desc = core.describe(sk)
    tr = T.normalize({"id": "skill", "role": "right", "frame": "world", "source": "skill", "label": f"{label}: {desc}",
                      "points": pts, "meta": {"skill": _skill_meta(sk, obj or label, at=_r(at), force=F, hold_s=hold),
                                              "work_on": [obj] if obj else [], "notes": [f"on {label}"]}})
    return {"kind": "trajectory", "traj": tr, "target": {"target": obj or _r(at), "label": label},
            "say": f"I'll {desc} on {label}.", "press": {"target": obj or label, "force": F, "hold_s": hold}}


def spot(sk, world, target=None):
    p = sk["params"]
    obj = target if target is not None else p.get("object")
    objs = world.get("objects") or {}
    tgt = "hands"
    if obj and obj != "hands":
        o = str(obj).lower().replace(" ", "_")
        if o not in objs and not any(o in n for n in objs):
            obj = "hands"
    aw = p["assist_when"]
    msg = {"type": "watch", "op": "mode", "mode": "assist", "target": tgt, "max_force": float(p["max_force"]),
           "spot": {"v_min": float(aw["bar_velocity_below"]), "for_s": float(aw["for_s"]), "lift": float(p["lift_cm"]) / 100.0,
                    "bar": None if obj in (None, "hands") else str(obj), "skill": sk["name"]}}
    return {"kind": "watch", "msg": msg, "target": {"object": obj or "hands", "label": "your lift"},
            "say": f"I'll spot you: {core.describe(sk)}."}


def mirror_motion(sk, world, target=None):
    clip = sk["params"].get("clip")
    if not clip:
        raise core.SkillError("this motion skill has no clip")
    return {"kind": "task", "msg": {"type": "task", "action": "play_skill", "name": clip}, "target": {"clip": clip, "label": clip},
            "say": f"I'll {core.describe(sk)}."}


COMPILE = {"weave_seam": weave_seam, "wipe": wipe, "stir": stir, "press": press, "spot": spot, "mirror_motion": mirror_motion}


def compile_skill(sk, world, target=None):
    caps = world.get("caps")
    missing = [c for c in core.needs(sk) if caps is not None and c not in caps]
    if missing:
        raise core.SkillError(f"this robot can't do a {core.KINDS[sk['kind']]['label'].lower()} skill (it needs {', '.join(missing)})")
    return COMPILE[sk["kind"]](sk, world, target)
