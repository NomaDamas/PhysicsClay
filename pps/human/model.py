"""The person, in the robot's world frame: body pose, hands, held tool, speed, activity (state key `human`).

Built every frame from co-presence's world joints (pps/features/copresence.py `to_world`, which places the person
across the table facing the robot, not mirrored) plus, when the browser camera is on, MediaPipe Hands landmarks
(pinch and palm orientation are much better from 21 hand points than from the 3 coarse hand points of the pose).

* Grounding: co-presence keeps the hips at a fixed height, so in a squat the feet would float. The model shifts the
  body so the lowest ankle stands on the floor; hips really drop (P5 safety capsules and the squat detector use it).
* `pose`: 33 MediaPipe landmarks (index order of MediaPipe Pose) in world metres. The server's pose model gives 19 of
  them; the face points and feet are filled from the head and ankles (listed in `pose_filled`).
* `hands.right|left` (the person's own right/left): wrist position `p`, palm orientation `q` (w, x, y, z; x along
  the hand, y across the knuckles toward the index), `pinch` (thumb-index tip distance, m), `open`, `src`
  ("hands" = browser MediaPipe Hands, "pose" = pose landmarks only).
* `holding`: a tool label the person declared, else a detected label from the camera, else the closest movable sim
  object within HOLD_R of a hand (e.g. something the robot handed over).
"""
import math
import time

import numpy as np

from .activity import ActivityTracker

ANKLE_Z = 0.07
HOLD_R = 0.10
HANDS_FRESH = 0.6          # s: browser hand landmarks older than this are ignored
# MediaPipe Hands keypoints the browser sends (subset of 21): wrist, thumb tip, index mcp, index tip, middle mcp,
# middle tip, ring tip, pinky mcp, pinky tip
HAND_KEYS = (0, 4, 5, 8, 9, 12, 16, 17, 20)
POSE33 = ("nose", "l_eye_inner", "l_eye", "l_eye_outer", "r_eye_inner", "r_eye", "r_eye_outer", "l_ear", "r_ear",
          "mouth_l", "mouth_r", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist", "l_pinky",
          "r_pinky", "l_index", "r_index", "l_thumb", "r_thumb", "l_hip", "r_hip", "l_knee", "r_knee", "l_ankle",
          "r_ankle", "l_heel", "r_heel", "l_foot", "r_foot")
SIDE = {"l": "left", "r": "right"}


def _v(p):
    return np.asarray(p, float)


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v * 0.0


def _r(p, n=3):
    return None if p is None else [round(float(x), n) for x in p]


def mat_to_quat(R):
    """3x3 rotation -> (w, x, y, z)."""
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def hand_frame(along, across):
    """Palm frame from a vector along the hand (wrist -> knuckles) and one across it (pinky -> index)."""
    x = _unit(_v(along))
    y = _v(across) - np.dot(_v(across), x) * x
    if np.linalg.norm(y) < 1e-6:
        y = np.cross([0.0, 0.0, 1.0], x)
    y = _unit(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def ground(J):
    """Shift the body so the lowest ankle is on the floor (co-presence pins the hips at a fixed height)."""
    ank = [J[k][2] for k in ("l_ankle", "r_ankle") if k in J]
    if not ank:
        return J
    dz = ANKLE_Z - min(ank)
    if abs(dz) < 1e-4:
        return J
    off = np.array([0.0, 0.0, dz])
    return {k: _v(p) + off for k, p in J.items()}


def pose33(J):
    """World joints -> ([[x, y, z]] * 33 in MediaPipe order, [indices filled in by estimate])."""
    head, nose = J.get("head"), J.get("nose")
    fw = _unit(np.cross(_v(J["l_shoulder"]) - _v(J["r_shoulder"]), [0.0, 0.0, 1.0]))
    across = _unit(_v(J["l_shoulder"]) - _v(J["r_shoulder"]))
    est = {}
    if nose is not None:
        n = _v(nose)
        for s, sg in (("l", 1.0), ("r", -1.0)):
            est[f"{s}_eye"] = n + np.array([0, 0, 0.035]) + sg * across * 0.032 - fw * 0.02
            est[f"{s}_eye_inner"] = est[f"{s}_eye"] - sg * across * 0.012
            est[f"{s}_eye_outer"] = est[f"{s}_eye"] + sg * across * 0.012
            est[f"{s}_ear"] = n + np.array([0, 0, 0.02]) + sg * across * 0.075 - fw * 0.09
        est["mouth_l"], est["mouth_r"] = n - np.array([0, 0, 0.035]) + across * 0.022, n - np.array([0, 0, 0.035]) - across * 0.022
    elif head is not None:
        est["nose"] = _v(head) + fw * 0.08
    for s in ("l", "r"):
        a = J.get(f"{s}_ankle")
        if a is not None:
            est[f"{s}_heel"] = _v(a) - fw * 0.05 - np.array([0, 0, 0.04])
            est[f"{s}_foot"] = _v(a) + fw * 0.15 - np.array([0, 0, 0.05])
    out, filled = [], []
    for i, k in enumerate(POSE33):
        if k in J:
            out.append(_r(J[k]))
        elif k in est:
            out.append(_r(est[k]))
            filled.append(i)
        else:
            out.append(None)
            filled.append(i)
    return out, filled


def browser_hand(lm9):
    """9 MediaPipe Hands world keypoints (camera axes: x right, y down, z away; metres) -> robot-world vectors, the
    same proper rotation co-presence uses for the pose (world = (z, -x, -y)). -> {pinch, open, R}"""
    P = {k: np.array([p[2], -p[0], -p[1]], float) for k, p in zip(HAND_KEYS, lm9)}
    palm = float(np.linalg.norm(P[9] - P[0]))
    if palm < 1e-3:
        return None
    tips = [float(np.linalg.norm(P[k] - P[0])) for k in (8, 12, 16, 20)]
    return {"pinch": float(np.linalg.norm(P[4] - P[8])), "open": bool(np.mean(tips) / palm > 1.55),
            "R": hand_frame(P[9] - P[0], P[5] - P[17])}


class HumanModel:
    def __init__(self):
        self.tracker = ActivityTracker()
        self.prev, self.prev_t = None, None
        self.speed = 0.0
        self.hands_in = None            # (t, [{uv, lm, score}]) from the browser
        self.declared = None            # a tool the person said they hold
        self.summary_vlm = None         # (t, text, src) from the VLM summarizer
        self.state, self.events = {"visible": False}, []
        self.J = None

    def set_browser_hands(self, hands, now=None):
        out = []
        for h in hands or []:
            try:
                lm = [[float(c) for c in p[:3]] for p in h.get("lm") or []]
                uv = [float(h["uv"][0]), float(h["uv"][1])]
            except (KeyError, TypeError, ValueError, IndexError):
                continue
            if len(lm) == len(HAND_KEYS):
                out.append({"uv": uv, "lm": lm, "score": float(h.get("score") or 0)})
        self.hands_in = (time.time() if now is None else now, out[:2])

    def _match_browser(self, pts, now, size):
        """Browser hands -> {side: hand} by image distance to the pose wrists (handedness labels from an unmirrored
        camera are unreliable, so they are not used)."""
        if not self.hands_in or now - self.hands_in[0] > HANDS_FRESH or not pts or len(pts) < 17:
            return {}
        W, H = size
        out = {}
        for h in self.hands_in[1]:
            hu = np.array([h["uv"][0] * W, h["uv"][1] * H])
            best = min(((float(np.linalg.norm(hu - np.asarray(pts[i][:2], float))), s) for s, i in (("l", 15), ("r", 16))
                        if pts[i] is not None), default=None)
            if best and best[0] < 0.12 * W and best[1] not in out:
                out[best[1]] = h
        return out

    def update(self, J, src, now, objects=None, robot_held=(), detected=None, pointing=False, offer=None,
               hand_open=None, pts=None, frame_size=(1280, 720), near_x=1.0):
        """One frame. J: co-presence world joints (or None). objects: {name: center xyz} of movable sim objects.
        detected: {"l": label, "r": label} from the camera's object detector. offer: co-presence's offered hand side.
        near_x: a hand at or before this x (world) is near the robot's workspace. -> state dict."""
        if J is None:
            self.events = self.tracker.update(now, None)
            self.prev, self.J = None, None
            self.state = {"visible": False, "src": src, "activity": "none", "phase": None, "reps": self.tracker.counter.reps,
                          "holding": None, "hands": {}, "pose": None, "speed": 0.0, "summary": "nobody in view", "t": round(now, 2)}
            return self.state
        J = ground({k: _v(v) for k, v in J.items()})
        if src != getattr(self, "src_prev", src):          # another person / input: the old window means nothing now
            self.tracker.hist.clear()
            self.tracker.counter.phase = None
            self.prev = None
        self.src_prev = src
        self.J = J
        # speed: the fastest of the wrists and the hips (EMA)
        sp = 0.0
        if self.prev is not None and now - self.prev_t > 1e-3:
            dt = now - self.prev_t
            for k in ("l_wrist", "r_wrist", "pelvis", "l_hip"):
                if k in J and k in self.prev:
                    sp = max(sp, float(np.linalg.norm(J[k] - self.prev[k])) / dt)
            self.speed += 0.35 * (min(sp, 5.0) - self.speed)
        self.prev, self.prev_t = J, now
        bh = self._match_browser(pts, now, frame_size)
        hands = {}
        for s in ("l", "r"):
            if f"{s}_wrist" not in J:
                continue
            wr = J[f"{s}_wrist"]
            idx, pk, th = J.get(f"{s}_index"), J.get(f"{s}_pinky"), J.get(f"{s}_thumb")
            h = {"p": _r(wr), "src": "pose"}
            if s in bh and (b := browser_hand(bh[s]["lm"])) is not None:
                h.update(q=_r(mat_to_quat(b["R"]), 4), pinch=round(b["pinch"], 3), open=b["open"], src="hands")
            else:
                if idx is not None and pk is not None:
                    along = (idx + pk) / 2 - wr
                    h["q"] = _r(mat_to_quat(hand_frame(along, idx - pk)), 4)
                h["pinch"] = round(float(np.linalg.norm(th - idx)), 3) if (th is not None and idx is not None) else None
                h["open"] = None if hand_open is None else hand_open.get(s)
            tip = idx if idx is not None else wr
            h["near_robot"] = bool(float(min(wr[0], tip[0])) <= near_x)
            hands[SIDE[s]] = h
        # held tool: declared > detected > closest sim object
        holding, hold_hand = None, None
        if self.declared:
            holding = self.declared
        elif detected and any(detected.get(s) for s in ("l", "r")):
            s = "r" if detected.get("r") else "l"
            holding, hold_hand = detected[s], SIDE[s]
        elif objects:
            best = None
            for s in ("l", "r"):
                if f"{s}_wrist" not in J:
                    continue
                hp = J[f"{s}_wrist"] if f"{s}_index" not in J else (J[f"{s}_wrist"] + J[f"{s}_index"]) / 2
                for name, c in objects.items():
                    if name in robot_held:
                        continue
                    d = float(np.linalg.norm(_v(c) - hp))
                    if d < HOLD_R and (best is None or d < best[0]):
                        best = (d, name, SIDE[s])
            if best:
                holding, hold_hand = best[1], best[2]
        evs = self.tracker.update(now, J, {"speed": self.speed, "pointing": pointing})
        tr = self.tracker
        pose, filled = pose33(J)
        st = {"visible": True, "src": src, "pose": pose, "pose_filled": filled, "hands": hands,
              "holding": holding, "holding_hand": hold_hand,
              "offer": SIDE.get(offer) if offer else None,
              "activity": tr.activity, "phase": tr.phase, "reps": tr.reps,
              "stalled": tr.stalled, "speed": round(self.speed, 3), "fatigue": round(tr.fatigue(), 2),
              "weave": tr.weave(), "t": round(now, 2)}
        st["summary"], st["summary_src"] = self._summary(st, now)
        self.state = st
        self.events = evs
        return st

    def _summary(self, st, now):
        if self.summary_vlm and now - self.summary_vlm[0] < 15.0:
            return self.summary_vlm[1], self.summary_vlm[2]
        return rule_summary(st, self.tracker), "rules"


VERB = {"squat": "squatting", "press": "pressing overhead", "curl": "doing curls", "wave": "waving", "weave": "weaving a hand",
        "reach": "reaching out", "point": "pointing", "still": "standing still", "moving": "moving", "none": "nobody in view"}


def rule_summary(st, tracker=None):
    """'squatting · rep 4 · down · holding the mug' from the rule-based reading."""
    if not st.get("visible"):
        return "nobody in view"
    parts = [VERB.get(st.get("activity"), st.get("activity") or "")]
    if st.get("reps"):
        parts.append(f"rep {st['reps']}")
    if st.get("phase") and st.get("activity") in ("squat", "press", "curl"):
        parts.append(st["phase"])
    if st.get("stalled"):
        parts.append("stuck")
    w = st.get("weave")
    if w:
        parts.append(f"{w['amp'] * 100:.1f} cm at {w['freq']:.1f} Hz")
    if st.get("holding"):
        parts.append("holding " + str(st["holding"]).replace("_", " "))
    if st.get("offer") and (st.get("hands") or {}).get(st["offer"], {}).get("open"):
        parts.append("hand held out")
    return " · ".join(p for p in parts if p)
