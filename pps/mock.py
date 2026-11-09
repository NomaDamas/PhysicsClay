"""Mock brain: an offline stand-in for Gemini Live's vision understanding, so the demo works without a key or network.

It reads the user's pose (MediaPipe world landmarks) and recognises three semantic gestures:
  pointing  -> marks the spot on the table the user points at (marker P); "put the mug there" then uses it
  stop      -> both hands above the head: stop the robot
  wave      -> greets back
and it narrates task verdicts as robot speech (state.say). Real Gemini replaces all of this when a key is set.
"""
import collections
import time

import numpy as np

from .tasks import REGIONS, TABLE_TOP, TABLE_X, TABLE_Y


def _angle(a, b, c):
    v1, v2 = a - b, c - b
    n = np.linalg.norm(v1) * np.linalg.norm(v2)
    return 180.0 if n < 1e-6 else float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / n, -1, 1))))


class MockBrain:
    def __init__(self):
        self.enabled = False
        self.say = None
        self._sid = 0
        self.gesture = None          # {"name", "ray": [[xyz],[xyz]] | None}
        self._cand, self._cand_t = None, 0.0
        self._cool = 0.0
        self._wx = collections.deque(maxlen=45)
        self._last_task = None

    def utter(self, text, kind="robot"):
        self._sid += 1
        self.say = {"id": self._sid, "text": text, "kind": kind}

    # -------------------------------------------------------------- narration
    def narrate(self, task):
        if not self.enabled or not task:
            return
        key = (task.get("id"), task.get("status"))
        if key == self._last_task or task.get("status") == "planning":
            return
        self._last_task = key
        self.utter(task.get("message", ""), "robot")

    # -------------------------------------------------------------- gestures
    def update(self, lm, rt, tasks, busy=False):
        if not self.enabled or not lm or busy or rt.engaged:
            self.gesture = None
            return
        need = ("l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist", "nose")
        if any(k not in lm for k in need):
            self.gesture = None
            return
        v = {k: np.asarray(lm[k], float) for k in need}
        now = time.time()
        name, ray = None, None
        # stop: both wrists above the head (y is down)
        if v["l_wrist"][1] < v["nose"][1] - 0.05 and v["r_wrist"][1] < v["nose"][1] - 0.05:
            name = "stop"
        else:
            for s, o in (("r", "l"), ("l", "r")):
                sh, el, wr = v[f"{s}_shoulder"], v[f"{s}_elbow"], v[f"{s}_wrist"]
                straight = _angle(sh, el, wr) > 150
                reach = np.linalg.norm(wr - sh) > 0.85 * (np.linalg.norm(el - sh) + np.linalg.norm(wr - el))
                level = abs(wr[1] - sh[1]) < 0.30
                other_down = v[f"{o}_wrist"][1] > v[f"{o}_elbow"][1]
                if straight and reach and level and other_down:
                    name = "pointing"
                    d = wr - sh
                    r = np.array([-d[2], (-1 if rt.retargeter.mirror else 1) * d[0], 0.0])
                    n = np.linalg.norm(r[:2])
                    r = r / n if n > 1e-6 else np.array([1.0, 0, 0])
                    origin = np.array([0.10, 0.0, 1.05])
                    tgt = np.array([0.36, 0.0, TABLE_TOP]) + np.array([r[0] * 0.12, r[1] * 0.30, 0.0])
                    tgt[0] = float(np.clip(tgt[0], TABLE_X[0] + 0.05, TABLE_X[1] - 0.05))
                    tgt[1] = float(np.clip(tgt[1], TABLE_Y[0] + 0.06, TABLE_Y[1] - 0.06))
                    ray = [origin.tolist(), tgt.tolist()]
                    break
            if name is None and (v["r_wrist"][1] < v["r_shoulder"][1] or v["l_wrist"][1] < v["l_shoulder"][1]):
                w = v["r_wrist"] if v["r_wrist"][1] < v["l_wrist"][1] else v["l_wrist"]
                self._wx.append((now, w[0]))
                xs = [x for t, x in self._wx if now - t < 1.6]
                if len(xs) > 8 and max(xs) - min(xs) > 0.12:
                    sign = np.sign(np.diff(xs))
                    if np.count_nonzero(np.diff(sign[sign != 0])) >= 3:
                        name = "wave"
        self.gesture = {"name": name, "ray": ray} if name else None
        # confirm a gesture after holding it, then cool down
        if name != self._cand:
            self._cand, self._cand_t = name, now
            return
        if name is None or now < self._cool or now - self._cand_t < (0.5 if name == "stop" else 0.9):
            return
        self._cool = now + 2.5
        if name == "stop":
            tasks.stop("g-stop")
            self.utter("You raised both hands, so I stopped.", "vision")
        elif name == "wave":
            self.utter("I see you waving. Hi! Say 'follow me' if you want me to copy you.", "vision")
        elif name == "pointing" and ray:
            tasks.set_marker("P", np.array(ray[1]))
            x, y = ray[1][0], ray[1][1]
            side = "left" if y > 0.08 else "right" if y < -0.08 else "middle"
            self.utter(f"You're pointing at the {side} of the table. I marked it P. Say 'put the mug there'.", "vision")
