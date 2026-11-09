"""Human pose-world landmarks -> robot palm targets (robot frame).

Robot frame: x forward (toward the viewer camera), y left, z up.
Human (MediaPipe world): x image-right, y image-down, z depth away from camera.
"""
import numpy as np


class Retargeter:
    def __init__(self, robot_shoulders, robot_arm_len, mirror=True, alpha=0.45):
        # robot_shoulders: {"left_palm": xyz of left shoulder, "right_palm": xyz of right shoulder}
        self.robot_shoulders = {k: np.asarray(v, dtype=float) for k, v in robot_shoulders.items()}
        self.robot_arm_len = float(robot_arm_len)
        self.mirror = mirror
        self.alpha = alpha
        self._ema = {}
        self._arm_len_ema = None

    @staticmethod
    def _v(lm, name):
        return np.asarray(lm[name], dtype=float)

    def human_arm_len(self, lm):
        lens = []
        for s in ("l", "r"):
            try:
                sh, el, wr = self._v(lm, f"{s}_shoulder"), self._v(lm, f"{s}_elbow"), self._v(lm, f"{s}_wrist")
            except KeyError:
                continue
            lens.append(np.linalg.norm(el - sh) + np.linalg.norm(wr - el))
        if not lens:
            return None
        cur = float(np.mean(lens))
        self._arm_len_ema = cur if self._arm_len_ema is None else 0.9 * self._arm_len_ema + 0.1 * cur
        return self._arm_len_ema

    def targets(self, lm):
        """Return {"left_palm": xyz, "right_palm": xyz} in robot frame, or {} if landmarks missing."""
        arm_len = self.human_arm_len(lm)
        if arm_len is None or arm_len < 0.15:
            return {}
        s = self.robot_arm_len / arm_len
        # Mirror: user's right hand (image-left, -x) appears screen-right after display flip,
        # which is the robot's left side (+y) because the robot faces the viewer.
        pairs = (("r", "left_palm"), ("l", "right_palm")) if self.mirror else (("r", "right_palm"), ("l", "left_palm"))
        y_sign = -1.0 if self.mirror else 1.0
        out = {}
        for hs, palm in pairs:
            try:
                sh, wr = self._v(lm, f"{hs}_shoulder"), self._v(lm, f"{hs}_wrist")
            except KeyError:
                continue
            h = wr - sh
            r = np.array([-h[2], y_sign * h[0], -h[1]]) * s
            tgt = self.robot_shoulders[palm] + r
            prev = self._ema.get(palm)
            tgt = tgt if prev is None else (1 - self.alpha) * prev + self.alpha * tgt
            self._ema[palm] = tgt
            out[palm] = tgt
        return out

    def reset(self):
        self._ema.clear()
