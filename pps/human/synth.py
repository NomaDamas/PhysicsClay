"""Synthetic people for checks and demos (no camera): MediaPipe pose-world landmark frames of a person standing across
the table, facing the robot, doing something.

    frames("squat", seconds=10, reps=5, valgus=True)     # 5 squats with knees caving in
    frames("weave", seconds=6, amp=0.015, freq=2.0)       # right hand weaving along a seam (welding)
    frames("wave", seconds=3)  frames("curl", ...)  frames("press", ...)  frames("offer", target=[x, y, z])
    frames("lean", 1.0, lean=45, at=[x, y, z], at_l=None)  # leaning over the table, hands reaching in (safety)
    frames("wave", 4, z=0.0)                              # a wave at shoulder height
    frames("jitter", 6, amp=0.02)                         # a still, hanging hand with MediaPipe-like jitter (~0.9 Hz)
    frames("weave", 6, amp=0.005, freq=2.0, travel=0.01, noise=0.001, shape="tri")
    sequence([("rest", 1.0, {}), ("squat", 6.0, {"reps": 3})])

Built in world coordinates (x toward the robot is -x for the person, y left of the robot, z up) and converted with
copresence.from_world, so `to_world` brings them back exactly where they were built. Deterministic.
"""
import math

import numpy as np

from ..features.copresence import ANKLE_Z, HIP_Z, PERSON, from_world, synth_pose

THIGH, SHIN, UPPER, FORE = 0.43, 0.43, 0.30, 0.28
TABLE_TOP = 0.74


def _u(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _arm(sh, wr, sy, pole=None):
    """Two-link arm: elbow position for a wrist target (elbow bends down and out)."""
    sh, wr = np.asarray(sh, float), np.asarray(wr, float)
    d = wr - sh
    n = float(np.clip(np.linalg.norm(d), abs(UPPER - FORE) + 1e-3, UPPER + FORE - 1e-3))
    u = _u(d)
    wr = sh + u * n
    a = (UPPER ** 2 - FORE ** 2 + n ** 2) / (2 * n)
    h = math.sqrt(max(UPPER ** 2 - a ** 2, 0.0))
    p = np.array([0.15, sy * 0.35, -1.0]) if pole is None else np.asarray(pole, float)
    perp = _u(p - np.dot(p, u) * u)
    return sh + a * u + h * perp, wr


def _hand(J, sd, el, wr, sy, open_=False, along=None):
    f = _u(wr - el) if along is None else _u(along)
    side = _u(np.cross(f, [0.0, 0.0, 1.0])) * -sy
    if not np.any(side):
        side = np.array([0.0, sy, 0.0])
    if open_:
        J[f"{sd}_index"] = wr + 0.15 * f
        J[f"{sd}_pinky"] = wr + 0.13 * f + 0.04 * side
        J[f"{sd}_thumb"] = wr + 0.06 * f - 0.05 * side
    else:                                   # a fist (holding a tool): fingers curled near the wrist
        J[f"{sd}_index"] = wr + 0.07 * f
        J[f"{sd}_pinky"] = wr + 0.055 * f + 0.02 * side
        J[f"{sd}_thumb"] = wr + 0.05 * f - 0.02 * side
    J[f"{sd}_elbow"], J[f"{sd}_wrist"] = el, wr


def body(knee=180.0, lean=None, valgus=0.0, anchor=PERSON, arms=None):
    """A standing/squatting body. knee: knee angle (deg, 180 = straight). valgus: 0 (knees track the feet) .. 1 (knees
    cave in at the bottom). lean: torso lean from vertical (deg; default grows with depth). arms: {sd: (wrist xyz,
    open?, pole?)} or None (arms forward at shoulder height, as when squatting / at rest by the sides)."""
    ax, ay = anchor
    bend = math.radians(180.0 - knee)
    a, b = 0.4 * bend, 0.6 * bend
    depth = float(np.clip((180.0 - knee) / 100.0, 0.0, 1.0))
    lean = 8.0 + 30.0 * depth if lean is None else lean
    J = {}
    hip_x = hip_z = None
    for sd, sy in (("l", -1.0), ("r", 1.0)):
        ank = np.array([ax, ay + sy * 0.12, ANKLE_Z])
        ky = ay + sy * (0.12 + 0.03 * depth - 0.12 * valgus * depth)
        knee_p = np.array([ank[0] - SHIN * math.sin(a), ky, ank[2] + SHIN * math.cos(a)])
        hip = np.array([knee_p[0] + THIGH * math.sin(b), ay + sy * 0.10, knee_p[2] + THIGH * math.cos(b)])
        J[f"{sd}_ankle"], J[f"{sd}_knee"], J[f"{sd}_hip"] = ank, knee_p, hip
        hip_x, hip_z = hip[0], hip[2]
    lr = math.radians(lean)
    trunk = np.array([-math.sin(lr), 0.0, math.cos(lr)]) * 0.50
    hip_mid = np.array([hip_x, ay, hip_z])
    sh_mid = hip_mid + trunk
    J["nose"] = sh_mid + _u(trunk) * 0.22 + np.array([-0.08, 0.0, 0.0])
    for sd, sy in (("l", -1.0), ("r", 1.0)):
        sh = sh_mid + np.array([0.0, sy * 0.18, 0.0])
        J[f"{sd}_shoulder"] = sh
        spec = (arms or {}).get(sd)
        if spec is None:
            wr_t = sh + np.array([0.02, sy * 0.03, -0.53])
            el, wr = _arm(sh, wr_t, sy)
            _hand(J, sd, el, wr, sy, open_=True, along=[0, 0, -1])
        else:
            wr_t, open_, pole = (list(spec) + [None, None])[:3]
            el, wr = _arm(sh, wr_t, sy, pole)
            _hand(J, sd, el, wr, sy, open_=bool(open_))
    return J


def _lm(J, anchor=PERSON):
    return from_world({k: np.asarray(v, float) for k, v in J.items()}, anchor, HIP_Z)


def frame_at(kind, t, anchor=PERSON, **kw):
    """One frame of activity `kind` at time t (s)."""
    ax, ay = anchor
    if kind == "squat":
        period = float(kw.get("period", 2.0))
        kmin = float(kw.get("knee_min", 80.0))
        ph = (t % period) / period
        stall = kw.get("stall_at")               # (rep index, fraction of the way up) where the lifter gets stuck
        if stall is not None and t >= stall[0] * period + 0.5 * period:
            ph = 0.5 + 0.5 * float(stall[1])     # frozen on the way up
        knee = 180.0 - (180.0 - kmin) * (1 - math.cos(2 * math.pi * ph)) / 2
        lean = kw.get("lean")
        if lean is not None:
            lean = 8.0 + (float(lean) - 8.0) * (1 - math.cos(2 * math.pi * ph)) / 2
        B = body(knee, lean=lean, valgus=float(kw.get("valgus", 0.0)), anchor=anchor, arms={})
        sh_l, sh_r = B["l_shoulder"], B["r_shoulder"]
        arms = {"l": (sh_l + np.array([-0.50, 0.0, -0.02]), False), "r": (sh_r + np.array([-0.50, 0.0, -0.02]), False)}
        if kw.get("bar"):                        # hands on a bar across the shoulders' front (spotting demo)
            arms = {"l": (sh_l + np.array([-0.12, -0.08, 0.02]), False), "r": (sh_r + np.array([-0.12, 0.08, 0.02]), False)}
        return _lm(body(knee, lean=lean, valgus=float(kw.get("valgus", 0.0)), anchor=anchor, arms=arms), anchor)
    if kind == "curl":
        period = float(kw.get("period", 2.4))
        ph = (t % period) / period
        ang = math.radians(165.0 - 120.0 * (1 - math.cos(2 * math.pi * ph)) / 2)     # elbow 165 -> 45 deg
        B = body(anchor=anchor)
        sd = kw.get("side", "r")
        sy = 1.0 if sd == "r" else -1.0
        sh = B[f"{sd}_shoulder"]
        el = sh + np.array([0.0, sy * 0.02, -UPPER])
        fore = np.array([-math.sin(math.pi - ang), 0.0, -math.cos(math.pi - ang)])   # forearm swings up toward the robot
        wr = el + FORE * fore
        _hand(B, sd, el, wr, sy, open_=False)
        return _lm(B, anchor)
    if kind == "press":
        period = float(kw.get("period", 2.4))
        ph = (t % period) / period
        lift = (1 - math.cos(2 * math.pi * ph)) / 2          # 0 at the shoulders, 1 overhead
        B = body(anchor=anchor, arms={})
        arms = {sd: (B[f"{sd}_shoulder"] + np.array([-0.05, sy * 0.10, 0.03 + 0.48 * lift]), False, [0.2, sy, 0.0])
                for sd, sy in (("l", -1.0), ("r", 1.0))}
        return _lm(body(anchor=anchor, arms=arms), anchor)
    if kind == "wave":
        f = float(kw.get("freq", 1.5))
        B = body(anchor=anchor, arms={})
        sh = B["r_shoulder"]
        z = float(kw.get("z", 0.28))             # wrist height over the shoulder (0 = a wave at shoulder height)
        wr = sh + np.array([-0.12, 0.18 + 0.15 * math.sin(2 * math.pi * f * t), z])
        return _lm(body(anchor=anchor, arms={"r": (wr, True, [0.3, 1.0, -0.6])}), anchor)
    if kind == "jitter":                          # a still hand by the side, as MediaPipe's world landmarks jitter it
        a = float(kw.get("amp", 0.02))
        rng = np.random.default_rng(int(kw.get("seed", 3)))
        ph = rng.uniform(0, 2 * math.pi, (3, 3))
        fr = (0.9, 0.55, 1.7)
        off = np.array([sum(w * math.sin(2 * math.pi * fr[k] * t + ph[k, c]) for k, w in enumerate((1.0, 0.5, 0.25)))
                        for c in range(3)]) * a / 1.75
        B = body(anchor=anchor)
        for k in ("r_wrist", "r_index", "r_pinky", "r_thumb"):
            B[k] = B[k] + off
        B["r_elbow"] = B["r_elbow"] + 0.4 * off
        return _lm(B, anchor)
    if kind == "weave":
        f, amp, travel = float(kw.get("freq", 2.0)), float(kw.get("amp", 0.015)), float(kw.get("travel", 0.03))
        y0 = float(kw.get("y0", ay + 0.12))
        ph = 2 * math.pi * f * t
        wav = (2 / math.pi) * math.asin(math.sin(ph)) if kw.get("shape") == "tri" else math.sin(ph)
        wr = np.array([ax - 0.36 + amp * wav, y0 - travel * t, TABLE_TOP + float(kw.get("z", 0.24))])
        if kw.get("noise"):                       # deterministic per-frame noise (sensor jitter)
            wr = wr + np.random.default_rng(int(round(t * 1000)) + 7).normal(0.0, float(kw["noise"]), 3)
        return _lm(body(lean=15.0, anchor=anchor, arms={"r": (wr, False)}), anchor)
    if kind == "reach":
        tgt = np.asarray(kw.get("target", [ax - 0.6, ay + 0.1, TABLE_TOP + 0.15]), float)
        return synth_pose(offer=tgt, anchor=anchor)
    if kind == "offer":
        return synth_pose(offer=kw.get("target", [0.80, -0.05, TABLE_TOP + 0.25]), anchor=anchor)
    if kind == "lean":                            # leaning over the table toward the robot (P5 safety checks), the right
        arms = {sd: (np.asarray(kw[k], float), bool(kw.get("open", True)))   # hand reaching into its workspace at
                for sd, k in (("r", "at"), ("l", "at_l")) if kw.get(k) is not None}   # `at` (left: `at_l`), else hanging
        return _lm(body(lean=float(kw.get("lean", 40.0)), anchor=anchor, arms=arms), anchor)
    if kind == "hand":                            # the right hand at a world point (mirror tests), open or a fist
        wr = np.asarray(kw["at"], float)
        return _lm(body(anchor=anchor, arms={"r": (wr, bool(kw.get("open", True)))}), anchor)
    return _lm(body(anchor=anchor), anchor)       # rest


def frames(kind, seconds=None, fps=20, anchor=PERSON, **kw):
    if kind == "squat" and seconds is None:
        seconds = float(kw.get("reps", 5)) * float(kw.get("period", 2.0)) + 0.6
    n = int(round((seconds or 3.0) * fps))
    return [frame_at(kind, i / fps, anchor, **kw) for i in range(n)]


def sequence(parts, fps=20, anchor=PERSON):
    """[(kind, seconds, kw), ...] -> one list of frames."""
    out = []
    for kind, sec, kw in parts:
        out += frames(kind, sec, fps, anchor, **(kw or {}))
    return out
