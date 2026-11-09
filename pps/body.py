"""Whole-body retargeting beyond the hands: squat depth, foot lifts (kicks/steps), torso twist.

Human (MediaPipe world, metres): x image-right, y down, z away from the camera, origin at the hips.
Robot: x forward, y left, z up.  Output offsets are applied to the home CoM / foot targets / torso orientation.
"""
import numpy as np


class BodyRetargeter:
    def __init__(self, robot_leg_len=0.62, alpha=0.4):
        self.robot_leg = robot_leg_len
        self.alpha = alpha
        self.h0 = None          # standing hip-above-ankle height (running max)
        self.ema = None

    def reset(self):
        self.h0, self.ema = None, None

    def __call__(self, lm, mirror):
        need = ("l_hip", "r_hip", "l_shoulder", "r_shoulder")
        if not lm or any(k not in lm for k in need):
            return None
        v = {k: np.asarray(x, float) for k, x in lm.items()}
        out = {"squat": 0.0, "lift": {"l": np.zeros(3), "r": np.zeros(3)}, "yaw": 0.0}
        # torso twist from the shoulder line (absolute body rotation: the robot's feet stay planted)
        sv = v["l_shoulder"] - v["r_shoulder"]
        yaw = float(np.arctan2(sv[2], sv[0]))
        out["yaw"] = float(np.clip(-yaw if mirror else yaw, -0.9, 0.9))
        legs = all(k in v for k in ("l_knee", "r_knee", "l_ankle", "r_ankle"))
        if legs:
            hip_y = (v["l_hip"][1] + v["r_hip"][1]) / 2
            ank_y = max(v["l_ankle"][1], v["r_ankle"][1])            # lower ankle = support (y down)
            hh = ank_y - hip_y
            self.h0 = hh if self.h0 is None else max(self.h0 * 0.999, hh)
            leg_h = np.mean([np.linalg.norm(v[f"{s}_knee"] - v[f"{s}_hip"]) + np.linalg.norm(v[f"{s}_ankle"] - v[f"{s}_knee"])
                             for s in ("l", "r")])
            sc = self.robot_leg / max(leg_h, 0.2)
            out["squat"] = float(np.clip((self.h0 - hh) / max(self.h0, 0.2), 0.0, 0.45))
            for s, o in (("l", "r"), ("r", "l")):
                lift = v[f"{o}_ankle"][1] - v[f"{s}_ankle"][1]
                if lift > 0.07:
                    fwd = -(v[f"{s}_ankle"][2] - v[f"{o}_ankle"][2])   # toward the camera = robot forward
                    side = (v[f"{s}_ankle"][0] - v[f"{o}_ankle"][0])
                    d = np.array([np.clip(fwd * sc, -0.15, 0.35), np.clip((-side if mirror else side) * sc * 0.3, -0.1, 0.1),
                                  np.clip(lift * sc, 0.0, 0.35)])
                    out["lift"][s] = d
        if self.ema is None:
            self.ema = out
        else:
            a = self.alpha
            self.ema = {"squat": (1 - a) * self.ema["squat"] + a * out["squat"],
                        "yaw": (1 - a) * self.ema["yaw"] + a * out["yaw"],
                        "lift": {s: (1 - a) * self.ema["lift"][s] + a * out["lift"][s] for s in ("l", "r")}}
        return self.ema
