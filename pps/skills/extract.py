"""Demonstration (P3 learn file, pps/human/demo.py) -> skill parameters, and which kind of skill it was.

What the person's hand tells us, kind by kind (all in the robot's frame: the person faces the robot, so their motion
is mirrored, pps/human/demo.py mirror_vec; speeds, widths and frequencies don't change, directions do):

* weave_seam: the longest stretch that passes the weave gates (pps/human/periodic.py best_segment), then travel speed
  (least-squares line, its direction checked against the path's PCA axis), weave width and frequency (FFT on the lateral
  offset, band-limited cycles), shape (zigzag / circle / figure8) and the torch angles from the hand orientation. The
  co-presence smoothing (an EMA on every joint) narrows a fast weave a little; its known gain at the weave frequency is
  divided out. The seam itself is NOT taken from the hand: it comes from the scene when the skill is applied.
* wipe: strokes along the path's main axis (reversals with 2 cm hysteresis), their spacing across it, raster or
  spiral (the path winds around its centre with a growing radius), stroke speed.
* stir: a circle fit (Kasa) of the horizontal path, turns and frequency from the unwrapped angle, direction.
* press: the hand comes down and stays still: hold time and where.
* spot: a lift (squats, presses, curls) with reps: the bar (the hands' midpoint) speed sets "help when it slows below".
"""
import math

import numpy as np

from ..human import periodic
from ..human.demo import mirror_vec
from . import core

TORCH = ("torch", "welder", "welding", "gun", "mig", "tig")
WIPE = ("sponge", "cloth", "rag", "towel", "brush", "eraser", "wiper", "squeegee", "pad")
STIR = ("spoon", "whisk", "ladle", "spatula", "stick", "stirrer")
LIFT = ("squat", "press", "curl")
STROKE_HYST = 0.02        # m: a wipe stroke reverses when it moved back this far


def _tool(demo):
    return str(demo.get("tool") or "").lower()


def _has(word, words):
    return any(w in word for w in words)


def hand_path(demo):
    """-> (t (N,), p (N, 3) world, q (N, 4) or None, other (N, 3) or None) of the working hand."""
    fr = [f for f in demo.get("frames") or [] if f.get("p")]
    if len(fr) < 5:
        raise core.SkillError("that demonstration is too short to learn from")
    t = np.array([float(f["t"]) for f in fr])
    p = np.array([f["p"] for f in fr], float)
    q = np.array([f["q"] for f in fr], float) if all(f.get("q") for f in fr) else None
    other = np.array([f["other"] for f in fr], float) if all(f.get("other") for f in fr) else None
    order = np.argsort(t, kind="stable")
    return t[order], p[order], None if q is None else q[order], None if other is None else other[order]


def _mirror(v):
    return mirror_vec(v)


def _smooth(x, k):
    """Moving average over k samples, padded with the end values (zero padding would invent motion at the ends)."""
    x = np.asarray(x, float)
    if k <= 1 or len(x) <= 2 * k:
        return x
    pad = np.concatenate([np.full(k, x[0]), x, np.full(k, x[-1])])
    return np.convolve(pad, np.ones(k) / k, mode="same")[k:-k]


def _smoothing_gain(demo, f, ts=None):
    """Gain of the co-presence smoothing at f, on the demo's own frame times when they are known (ticks are uneven)."""
    sm = demo.get("smoothing") or {}
    a = float(sm.get("ema") or 0.0)
    if ts is not None and len(ts) >= 8:
        return periodic.ema_gain_at(ts, f, a)
    return periodic.ema_gain(f, a, float(sm.get("dt") or 0.0))


# ---------------------------------------------------------------------------- orientation
def _quat_mat(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tool_angles(qs, travel_dir_robot):
    """Torch angles from the person's palm orientation (pps/human/model.py hand frame: x along the hand, y across the
    knuckles toward the index finger). A torch held in a fist leaves it on the index side, so its axis is the palm's y
    (or -y: the end that points down is the tip). In the robot's (mirrored) frame:
      travel angle = lean of the tip along the travel direction (+ push, - drag),
      work angle   = 90 - lean to the left of travel (90 = square to the plate).
    -> {"travel": deg, "work": deg, "spread": deg, "n": frames} or None when the hand doesn't hold a torch pointing down
    steadily (then the defaults are used and the person is told)."""
    if qs is None or len(qs) < 5:
        return None
    t = np.asarray(travel_dir_robot, float)
    up = np.array([0.0, 0.0, 1.0])
    axes = []
    for q in qs:
        y = _quat_mat(q)[:, 1]
        a = _mirror(y)                          # the person's torch direction as the robot copies it
        a = a if a[2] < 0 else -a               # the tip is the end that points down
        axes.append(a / max(np.linalg.norm(a), 1e-9))
    A = np.array(axes)
    down = -A[:, 2]
    if np.mean(down > math.cos(math.radians(60))) < 0.7:
        return None
    m = A.mean(axis=0)
    m /= max(np.linalg.norm(m), 1e-9)
    spread = float(np.degrees(np.median(np.arccos(np.clip(A @ m, -1, 1)))))
    if spread > 25:
        return None
    th = t - (t @ up) * up
    th = th / max(np.linalg.norm(th), 1e-9)
    left = np.cross(up, th)
    travel = math.degrees(math.atan2(float(m @ th), float(-m @ up)))
    lean_left = math.degrees(math.atan2(float(m @ left), float(-m @ up)))
    return {"travel": travel, "work": 90.0 - lean_left, "spread": spread, "n": len(A)}


# ---------------------------------------------------------------------------- per kind
def weave_reading(demo):
    """The weave in a demonstration: (analysis on the best stretch, gate verdict, (i0, i1)) or (None, ...)."""
    t, p, q, _ = hand_path(demo)
    seg = periodic.best_segment(t, p) or (0, len(t) - 1)
    i0, i1 = seg
    a = periodic.analyze(t[i0:i1 + 1], p[i0:i1 + 1])
    ok, why = periodic.is_weave(a)
    return a, ok, why, (i0, i1)


def extract_weave(demo):
    t, p, q, _ = hand_path(demo)
    a, ok, why, (i0, i1) = weave_reading(demo)
    if a is None:
        raise core.SkillError("I didn't see a weave long enough to learn (show it for a few seconds)")
    gain = _smoothing_gain(demo, a["freq"], t[i0:i1 + 1])
    amp = a["amp"] / gain
    d_robot = _mirror(a["dir"])
    angles = tool_angles(None if q is None else q[i0:i1 + 1], d_robot)
    notes = []
    params = {"travel_speed": a["speed"], "weave": {"kind": a["kind"], "amp": amp, "freq": a["freq"]}, "standoff": 0.015,
              "direction": "auto"}
    if angles:
        params.update(travel_angle=angles["travel"], work_angle=angles["work"])
    else:
        notes.append("I couldn't read your torch angle from the camera, so the angles are defaults (10° push, square to the plate)")
    if not ok:
        notes.append("it wasn't a clean weave: " + "; ".join(why[:2]))
    measured = {"amp": round(a["amp"], 5), "amp_corrected": round(amp, 5), "smoothing_gain": round(gain, 3),
                "freq": round(a["freq"], 3), "speed": round(a["speed"], 5), "dir": [round(float(v), 3) for v in d_robot],
                "r2": round(a["r2"], 3), "peak": round(a["peak"], 3), "acf": round(a["acf"], 3), "shape": a["kind"],
                "segment_s": [round(float(t[i0]), 2), round(float(t[i1]), 2)], "weave": bool(ok),
                "angles_from": "hand" if angles else "default"}
    if angles:
        measured["angle_spread"] = round(angles["spread"], 1)
    return params, measured, notes


def _reversals(x):
    """Direction changes of a signal with STROKE_HYST hysteresis (a wipe stroke turning back). -> (count, moved?)"""
    n_rev, direction, hi, lo, ext = 0, 0, x[0], x[0], x[0]
    for s in x:
        if direction == 0:                       # until it has clearly moved one way
            hi, lo = max(hi, s), min(lo, s)
            if s < hi - STROKE_HYST:
                direction, ext = -1, s
            elif s > lo + STROKE_HYST:
                direction, ext = 1, s
        elif direction > 0:
            if s > ext:
                ext = s
            elif s < ext - STROKE_HYST:
                n_rev, direction, ext = n_rev + 1, -1, s
        else:
            if s < ext:
                ext = s
            elif s > ext + STROKE_HYST:
                n_rev, direction, ext = n_rev + 1, 1, s
    return n_rev, direction != 0


def strokes(demo):
    """Wipe reading: {strokes, length, width, spacing, speed, turns, r_trend, pattern}. The strokes run along whichever
    principal axis of the horizontal path turns back most often; the other axis is how far the strokes step."""
    t, p, _, _ = hand_path(demo)
    xy = np.array([_mirror(v) for v in p])[:, :2]
    c = xy.mean(axis=0)
    X = xy - c
    w, V = np.linalg.eigh(X.T @ X / len(X))
    k = max(1, int(round(0.15 / max(np.median(np.diff(t)), 1e-3))))
    reads = []
    for a, b in ((V[:, -1], V[:, 0]), (V[:, 0], V[:, -1])):
        su, sv = X @ a, X @ b
        n_rev, moved = _reversals(_smooth(su, k))
        reads.append((n_rev, moved, su, sv))
    n_rev, moved, su, sv = max(reads, key=lambda r: r[0])
    n = n_rev + 1 if moved else 0
    length, width = float(np.ptp(su)), float(np.ptp(sv))
    ang = np.unwrap(np.arctan2(X[:, 1], X[:, 0]))
    turns = abs(float(ang[-1] - ang[0])) / (2 * math.pi)
    r = np.linalg.norm(X, axis=1)
    r_trend = float(np.corrcoef(t, r)[0, 1]) if np.std(r) > 1e-6 else 0.0
    vel = np.linalg.norm(np.diff(xy, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)
    return {"strokes": n, "length": length, "width": width, "spacing": width / max(1, n - 1), "turns": turns,
            "r_trend": r_trend, "speed": float(np.median(vel)) if len(vel) else 0.0,
            "pattern": "spiral" if turns >= 1.5 and r_trend > 0.5 else "raster"}


def extract_wipe(demo):
    s = strokes(demo)
    params = {"pattern": s["pattern"], "spacing": s["spacing"] if s["pattern"] == "raster" and s["strokes"] > 1 else 0.03,
              "speed": s["speed"], "force": 5.0, "passes": 1}
    if s["pattern"] == "spiral":
        params["spacing"] = max(0.015, s["width"] / max(2.0, 2 * s["turns"]))
    notes = ["I can't feel how hard you pressed, so it presses 5 N (say harder or softer)"]
    return params, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()}, notes


def circle_fit(xy):
    """Kasa algebraic circle fit -> (centre (2,), radius, relative spread of the radius)."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0], sol[1]
    R = math.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-12))
    d = np.hypot(x - cx, y - cy)
    return np.array([cx, cy]), R, float(np.std(d) / max(R, 1e-9))


def circling(demo):
    t, p, _, _ = hand_path(demo)
    xy = np.array([_mirror(v) for v in p])[:, :2]
    c, R, cv = circle_fit(xy)
    ang = np.unwrap(np.arctan2(xy[:, 1] - c[1], xy[:, 0] - c[0]))
    dth = float(ang[-1] - ang[0])
    dur = float(t[-1] - t[0])
    turns = abs(dth) / (2 * math.pi)
    return {"radius": R, "cv": cv, "turns": turns, "freq": turns / dur if dur > 0 else 0.0,
            "direction": "ccw" if dth > 0 else "cw", "center": c.tolist()}


def extract_stir(demo):
    c = circling(demo)
    params = {"radius": c["radius"], "freq": c["freq"], "turns": max(1, int(round(c["turns"]))), "direction": c["direction"],
              "depth": 0.03}
    return params, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in c.items()}, \
        ["I can't see how deep you went into the container, so it stirs 3 cm below the rim (say deeper or shallower)"]


def pressing(demo):
    t, p, _, _ = hand_path(demo)
    z = p[:, 2]
    k = max(1, int(round(0.2 / max(np.median(np.diff(t)), 1e-3))))
    zs = _smooth(z, k)
    ps = np.stack([_smooth(p[:, c], k) for c in range(3)], axis=1)
    v = np.linalg.norm(np.gradient(ps, t, axis=0), axis=1)        # the hand stops, not only stops going down
    low = (zs < zs.min() + 0.012) & (v < 0.03)
    best, cur = (0, -1), None
    for i, m in enumerate(low):
        if m:
            cur = (cur[0], i) if cur else (i, i)
            if cur[1] - cur[0] > best[1] - best[0]:
                best = cur
        else:
            cur = None
    i0, i1 = best
    hold = float(t[i1] - t[i0]) if i1 >= i0 else 0.0
    before = float(zs[: max(1, i0)].max() - zs.min()) if i0 > 0 else 0.0
    return {"hold_s": hold, "drop": before, "at": p[i0:i1 + 1].mean(axis=0).tolist() if i1 >= i0 else None}


def extract_press(demo):
    r = pressing(demo)
    return {"hold_s": max(0.3, r["hold_s"]), "force": 10.0}, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}, \
        ["I can't feel how hard you pressed, so it presses 10 N (say harder or softer)"]


def lifting(demo):
    t, p, _, other = hand_path(demo)
    bar = (p + other) / 2 if other is not None else p
    k = max(1, int(round(0.15 / max(np.median(np.diff(t)), 1e-3))))
    z = _smooth(bar[:, 2], k)
    vz = np.gradient(z, t)
    up = vz[vz > 0.02]
    conc = float(np.percentile(up, 75)) if len(up) else 0.0
    return {"reps": int(demo.get("reps") or 0), "activity": demo.get("activity"), "bar_speed": conc}


def extract_spot(demo):
    r = lifting(demo)
    v_min = float(np.clip(0.25 * r["bar_speed"], 0.02, 0.15)) if r["bar_speed"] > 0 else 0.05
    params = {"assist_when": {"bar_velocity_below": v_min, "for_s": 0.5}, "max_force": 40.0, "lift_cm": 6.0,
              "object": "barbell" if _has(_tool(demo), ("bar", "barbell")) else "hands", "follow": "hands"}
    return params, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}, []


EXTRACT = {"weave_seam": extract_weave, "wipe": extract_wipe, "stir": extract_stir, "press": extract_press, "spot": extract_spot}


# ---------------------------------------------------------------------------- which kind was it?
def rank(demo, world=None):
    """-> [{"kind", "score" 0..1, "why"}], best first. The demonstration's activity, the held tool and the scene decide."""
    world = world or {}
    tool = _tool(demo)
    act = str(demo.get("activity") or "")
    kinds = {str(o.get("kind")) for o in (world.get("objects") or {}).values()}
    out = []
    try:
        a, ok, _, _ = weave_reading(demo)
    except core.SkillError:
        return []
    if ok:
        sc = 0.6 + (0.3 if _has(tool, TORCH) else 0.0) + (0.1 if world.get("seams") else 0.0)
        out.append({"kind": "weave_seam", "score": sc, "why": f"a {a['amp'] * 1000:.0f} mm weave at {a['freq']:.1f} Hz while moving {a['speed'] * 1000:.0f} mm/s"})
    s = strokes(demo)
    c = circling(demo)
    circle = c["turns"] >= 1.5 and c["cv"] < 0.3 and 0.008 <= c["radius"] <= 0.15
    if (s["strokes"] >= 3 and s["length"] >= 0.06 and not circle) or (s["pattern"] == "spiral" and c["cv"] >= 0.3):
        sc = 0.5 + (0.3 if _has(tool, WIPE) else 0.0) + (0.1 if ok and a["amp"] >= 0.02 else 0.0)
        out.append({"kind": "wipe", "score": sc, "why": f"{s['strokes']} strokes of {s['length'] * 100:.0f} cm" if s["pattern"] == "raster"
                    else f"a spiral of {s['turns']:.1f} turns"})
    if circle:
        sc = 0.6 + (0.3 if _has(tool, STIR) else 0.0) + (0.1 if kinds & {"bowl", "pot", "cup", "mug"} else 0.0)
        out.append({"kind": "stir", "score": sc, "why": f"{c['turns']:.1f} circles of {c['radius'] * 100:.0f} cm"})
    pr = pressing(demo)
    if pr["hold_s"] >= 0.5 and pr["drop"] >= 0.03:
        out.append({"kind": "press", "score": 0.65, "why": f"your hand went down {pr['drop'] * 100:.0f} cm and held {pr['hold_s']:.1f} s"})
    if act in LIFT and int(demo.get("reps") or 0) >= 1:
        out.append({"kind": "spot", "score": 0.8 + (0.1 if _has(tool, ("bar",)) else 0.0),
                    "why": f"{demo.get('reps')} {act}{'s' if act != 'press' else 'es'}"})
    return sorted(out, key=lambda r: -r["score"])


def ambiguous(ranked, margin=0.12):
    return len(ranked) >= 2 and ranked[0]["score"] - ranked[1]["score"] < margin


def extract(demo, kind=None, name=None, world=None):
    """-> skill dict (not saved). kind: force a kind; else the best ranked one (raises when nothing fits)."""
    ranked = rank(demo, world) if kind is None else []
    if kind is None:
        if not ranked:
            raise core.SkillError(f"I couldn't find a skill in that demonstration ({demo.get('activity') or 'no clear activity'}): "
                                  "show a weave, a wipe, a stir, a press or a lift")
        kind = ranked[0]["kind"]
    if kind not in EXTRACT:
        raise core.SkillError(f"I can't learn a '{kind}' skill from a hand demonstration")
    params, measured, notes = EXTRACT[kind](demo)
    src = {"demo": demo.get("name"), "robot": demo.get("robot"), "activity": demo.get("activity"), "tool": demo.get("tool"),
           "hand": demo.get("hand"), "duration": demo.get("duration")}
    if ranked:
        src["ranked"] = [{"kind": r["kind"], "score": round(r["score"], 2)} for r in ranked[:3]]
    return core.make(kind, params, name or demo.get("name"), src, measured, notes)
