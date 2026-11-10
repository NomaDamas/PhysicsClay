"""Demonstrations recorded in `learn` mode, saved for skill extraction (P4), plus a minimal stub extractor.

File: recordings/demos/<name>.json
```
{"version": 1, "name", "created", "robot", "source", "duration", "fps",
 "hand": "right" | "left",                      # the working hand (the one that moved most)
 "tool": "torch" | null,                        # what the person held most of the time
 "activity": "weave" | "squat" | ..., "reps": 4,
 "objects": {name: [x, y, z]},                  # movable objects at the start (world)
 "anchor": {"object": "plate_a" | null, "pos": [x, y, z]},   # the object the hand worked closest to
 "pattern": {"kind": "zigzag", "amp": 0.015, "freq": 2.0, "travel": 0.03, "axis": [..], "dir": [..]} | null,
 "rep_stats": [...],                            # per-rep numbers from the activity tracker (coach references)
 "frames": [{"t", "p": [x, y, z], "q": [w, x, y, z] | null, "pinch", "open", "holding", "activity", "phase",
             "other": [x, y, z] | null}],       # the working hand (world) and the other wrist
 "trajectory": <P2 trajectory object, source "demo", frame "object:<anchor>" or "world">}
```
`to_trajectory` is the stub extractor: the working hand's path, mirrored into the robot's frame (the person faces the
robot, so their forward is the robot's backward), relative to its start and expressed in the anchor object's frame so
it follows the object when it moves. P4 replaces this with parametric skills (weave_seam, wipe, stir, ...).
"""
import json
import re
import time
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEMO_DIR = ROOT / "recordings" / "demos"
MAX_S = 120.0
ANCHOR_R = 0.25        # m: a demo is object-relative when the hand worked this close to an object


def safe_name(name):
    n = re.sub(r"[^a-z0-9_-]+", "_", str(name or "").strip().lower()).strip("_")[:40]
    return n or time.strftime("demo_%Y%m%d_%H%M%S")


def _r(p, n=4):
    return None if p is None else [round(float(x), n) for x in p]


class DemoRecorder:
    def __init__(self, name=None, robot=None, objects=None, now=None):
        self.name = name
        self.robot = robot
        self.t0 = time.time() if now is None else now
        self.objects = {k: _r(v) for k, v in (objects or {}).items()}
        self.rows = []          # (t, human-state subset)
        self.reps, self.rep_stats = 0, []
        self.src = None

    @property
    def seconds(self):
        return (self.rows[-1][0] - self.rows[0][0]) if len(self.rows) > 1 else 0.0

    def add(self, now, h, rep_events=()):
        if not h.get("visible"):
            return
        self.src = h.get("src")
        hands = h.get("hands") or {}
        self.rows.append((now - self.t0, {s: dict(hands[s]) for s in ("right", "left") if s in hands},
                          h.get("holding"), h.get("activity"), h.get("phase"), h.get("weave")))
        for st in rep_events:
            self.rep_stats.append({k: (round(v, 4) if isinstance(v, float) else v) for k, v in st.items()})
        self.reps = h.get("reps") or self.reps

    def finish(self, robot_label=None):
        """-> demo dict (not yet saved) or None when nothing useful was seen."""
        if len(self.rows) < 5:
            return None
        travel = {}
        for s in ("right", "left"):
            ps = np.array([r[1][s]["p"] for r in self.rows if s in r[1] and r[1][s].get("p")])
            travel[s] = float(np.ptp(ps, axis=0).sum()) if len(ps) > 2 else -1.0
        hand = max(travel, key=travel.get)
        other = "left" if hand == "right" else "right"
        frames = []
        for t, hs, holding, act, ph, _ in self.rows:
            h = hs.get(hand)
            if not h or not h.get("p"):
                continue
            frames.append({"t": round(t, 3), "p": h["p"], "q": h.get("q"), "pinch": h.get("pinch"), "open": h.get("open"),
                           "holding": holding, "activity": act, "phase": ph, "other": (hs.get(other) or {}).get("p")})
        if len(frames) < 5:
            return None
        acts = Counter(f["activity"] for f in frames if f["activity"] not in (None, "none", "moving", "still"))
        activity = acts.most_common(1)[0][0] if acts else "moving"
        tools = Counter(f["holding"] for f in frames if f["holding"])
        tool = tools.most_common(1)[0][0] if tools and tools.most_common(1)[0][1] > 0.3 * len(frames) else None
        weaves = [r[5] for r in self.rows if r[5]]
        pattern = None
        if weaves and activity == "weave":
            pattern = {"kind": "zigzag", "amp": round(float(np.median([w["amp"] for w in weaves])), 4),
                       "freq": round(float(np.median([w["freq"] for w in weaves])), 2),
                       "travel": round(float(np.median([w["travel"] for w in weaves])), 4),
                       "axis": weaves[-1]["axis"], "dir": weaves[-1]["dir"]}
        P = np.array([f["p"] for f in frames], float)
        anchor = None
        if self.objects:     # the object the hand worked over (within ANCHOR_R of the path), else a world-frame demo
            def gap(kv):
                return float(np.min(np.linalg.norm(P[:, :2] - np.asarray(kv[1][:2]), axis=1)))
            name, pos = min(self.objects.items(), key=gap)
            if gap((name, pos)) < ANCHOR_R:
                anchor = {"object": name, "pos": pos}
        demo = {"version": 1, "name": safe_name(self.name), "created": round(time.time(), 1), "robot": robot_label or self.robot,
                "source": self.src, "duration": round(frames[-1]["t"] - frames[0]["t"], 2),
                "fps": round(len(frames) / max(frames[-1]["t"] - frames[0]["t"], 1e-3), 1), "hand": hand, "tool": tool,
                "activity": activity, "reps": int(self.reps or 0), "objects": self.objects,
                "anchor": anchor or {"object": None, "pos": _r(P[0])}, "pattern": pattern, "rep_stats": self.rep_stats[-30:],
                "frames": frames}
        demo["trajectory"] = to_trajectory(demo)
        return demo


def save(demo, directory=None):
    d = Path(directory) if directory else DEMO_DIR
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"{demo['name']}.json"
    path.write_text(json.dumps(demo, separators=(",", ":")))
    return path


def load(name, directory=None):
    d = Path(directory) if directory else DEMO_DIR
    p = d / f"{safe_name(name)}.json"
    return json.loads(p.read_text()) if p.exists() else None


def listing(directory=None):
    d = Path(directory) if directory else DEMO_DIR
    return sorted(p.stem for p in d.glob("*.json")) if d.exists() else []


def mirror_vec(v):
    """A displacement of the person's hand -> the same displacement as the robot sees it when copying the person
    face to face (a mirror image): toward the robot (-x) becomes away from the robot's base (+x); left/right and
    up/down stay in the world."""
    v = np.asarray(v, float)
    return np.array([-v[0], v[1], v[2]])


def to_trajectory(demo, max_points=80):
    """Stub extractor: the working hand's path -> a replayable P2 trajectory, relative to the anchor object.

    points[i].p is the offset from the anchor object's position (plus a hover above it), so placing the trajectory
    = adding the anchor's current position. t is time from the start, grip 1 = closed (from pinch / open)."""
    fr = demo["frames"]
    step = max(1, len(fr) // max_points)
    keep = fr[::step]
    if keep[-1] is not fr[-1]:
        keep.append(fr[-1])
    p0 = np.asarray(keep[0]["p"], float)
    pts = []
    for f in keep:
        off = mirror_vec(np.asarray(f["p"], float) - p0)
        grip = None
        if f.get("pinch") is not None:
            grip = round(float(np.clip((0.07 - f["pinch"]) / 0.05, 0.0, 1.0)), 2)
        elif f.get("open") is not None:
            grip = 0.0 if f["open"] else 1.0
        pts.append({"p": _r(off + np.array([0.0, 0.0, 0.08]), 4), "q": f.get("q"), "t": f["t"] - keep[0]["t"], "grip": grip})
    a = (demo.get("anchor") or {}).get("object")
    pat = demo.get("pattern")
    return {"id": f"demo-{demo['name']}", "role": "right", "frame": f"object:{a}" if a else "world", "source": "demo",
            "label": f"{demo.get('activity')} demonstrated by hand" + (f" with the {demo['tool']}" if demo.get("tool") else ""),
            "points": pts,
            "pattern": None if not pat else {"kind": "zigzag", "amp": pat["amp"], "freq": pat["freq"], "axis": "lateral"},
            "check": None}


def place(traj, anchor_pos, start=None):
    """A demo trajectory -> robot-world points [[x, y, z]]: offsets added to the anchor object's current position (or to
    `start`, e.g. the end effector, for a world-frame trajectory)."""
    base = np.asarray(anchor_pos if anchor_pos is not None else start, float)
    return [[round(float(c), 4) for c in base + np.asarray(p["p"], float)] for p in traj["points"]]
