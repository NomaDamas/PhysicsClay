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
    return base
