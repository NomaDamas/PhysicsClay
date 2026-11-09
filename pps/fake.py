"""Synthetic human landmarks for testing without a camera."""
import math

import numpy as np


def fake_landmarks(t):
    base = {
        "nose": [0.0, -0.65, -0.05],
        "l_shoulder": [0.18, -0.45, 0.0], "r_shoulder": [-0.18, -0.45, 0.0],
        "l_hip": [0.10, 0.0, 0.0], "r_hip": [-0.10, 0.0, 0.0],
    }
    reach = 0.75 if (t % 8.0) > 6.0 else 0.30  # far-reach phase -> should trip 'unreachable'
    ang = t * 1.2
    for s, sx in (("l", 1.0), ("r", -1.0)):
        sh = np.array(base[f"{s}_shoulder"])
        el = sh + np.array([sx * 0.10, 0.22, -0.05])
        wr = sh + np.array([sx * reach * 0.5 * math.cos(ang), -reach * 0.6 * math.sin(ang), -reach * 0.6])
        base[f"{s}_elbow"] = el.tolist()
        base[f"{s}_wrist"] = wr.tolist()
        base[f"{s}_index"] = (wr + np.array([0.0, -0.12, 0.0])).tolist()
        base[f"{s}_thumb"] = (wr + np.array([sx * -0.05, -0.08, 0.0])).tolist()
        base[f"{s}_pinky"] = (wr + np.array([sx * 0.05, -0.10, 0.0])).tolist()
    ph = t % 8.0
    squat = 0.12 if 2.0 < ph < 3.5 else 0.0
    kick = 0.30 if 4.0 < ph < 5.0 else 0.0
    for s, sx in (("l", 1.0), ("r", -1.0)):
        base[f"{s}_hip"] = [sx * 0.10, 0.0 + squat * 0.0, 0.0]
        base[f"{s}_knee"] = [sx * 0.10, 0.42 - squat, -squat * 0.6]
        base[f"{s}_ankle"] = [sx * 0.10, 0.84 - squat, 0.0]
    if kick:
        base["r_knee"] = [-0.10, 0.30, -0.30]
        base["r_ankle"] = [-0.10, 0.84 - kick, -0.45]
    for k in base:
        if k not in ("l_hip", "r_hip"):
            base[k] = [base[k][0], base[k][1] + squat, base[k][2]] if "ankle" not in k else base[k]
    return base
