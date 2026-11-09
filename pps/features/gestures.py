"""#G · Live correction without words: gesture vocabulary + nudge pad (UX 2.0).

Gestures (pose landmarks, MediaPipe world frame: metres, y down, z negative toward the camera):
  stop       both hands above the head           -> stop (only when the mock brain is off; it owns stop otherwise)
  slower     both palms pressing down, hands apart, in front of the waist   -> speed_scale x0.7
  come_here  beckon: one hand in front at chest height pulling toward you    -> lean / reach toward you
  go_on      thumb up, or an open palm pushed forward (fingers up)          -> confirm the plan / resume

A static gesture must be held still (0.5-0.7 s); beckon is recognised from its motion. Each recognised gesture fires
once, then the same gesture must end before it can fire again (plus a short global cooldown). Inactive during the
scripted demo, teach recording, video imitation, follow-me mirroring, and on synthetic or video input.

Nudge pad messages: {"type": "nudge", "dx", "dy", "dz"} (metres, robot frame: x forward, y left, z up) moves the active
target; {"type": "speed", "scale"} sets the motion speed. Test-only: {"type": "gesture_feed", "frames": [lm, ...],
"fps": 20} plays synthetic landmark frames through the same detector.

Hooks for other features (optional; everything works without them), set on ctx.store:
  "nudge_hook":   fn(ctx, delta) -> with delta None: label of the target it owns, or None; with a delta (np.array):
                  apply it and return a message.
  "confirm_hook": fn(ctx) -> message if it confirmed something (e.g. ran the proposed plan), else None.
"""
import collections
import time

import numpy as np

from .. import ext

LABELS = {"stop": "Stop", "slower": "Slower", "come_here": "Come here", "go_on": "Go on"}
HOLD = {"stop": 0.5, "slower": 0.7, "go_on": 0.7}
STILL = 0.10          # m: a held gesture's wrists may drift at most this much
COOLDOWN = 2.0        # s between any two fired gestures
REARM = 0.4           # s the gesture must be gone before the same one can fire again
STEP_MAX = 0.05       # m: largest single nudge step per axis
TOTAL_MAX = 0.10      # m: largest total nudge per axis on one target
COME_X = (0.50, 0.44, 0.38)   # m: how far toward the person "come here" tries to reach, farthest first


def _p(lm, k):
    v = lm.get(k)
    return None if v is None else np.asarray(v, float)


def _frame(lm):
    """Common body measures, or None when the upper body isn't visible."""
    need = ("nose", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist")
    if not lm or any(k not in lm for k in need):
        return None
    v = {k: _p(lm, k) for k in lm}
    sh_y = (v["l_shoulder"][1] + v["r_shoulder"][1]) / 2
    sh_z = (v["l_shoulder"][2] + v["r_shoulder"][2]) / 2
    hips = [v[k] for k in ("l_hip", "r_hip") if k in v]
    hip_y = float(np.mean([h[1] for h in hips])) if hips else sh_y + 0.45
    width = float(np.linalg.norm(v["l_shoulder"] - v["r_shoulder"])) or 0.36
    return v, sh_y, sh_z, hip_y, width


def _down(v, s):
    return v[f"{s}_wrist"][1] > v[f"{s}_elbow"][1]


def _flat(v, s):
    """Hand roughly horizontal (palm down): fingertip landmarks near the wrist's height."""
    w = v[f"{s}_wrist"]
    tips = [v[k] for k in (f"{s}_index", f"{s}_pinky") if k in v]
    return bool(tips) and all(abs(t[1] - w[1]) < 0.07 for t in tips)


def _thumb_up(v, s, sh_y, sh_z, hip_y):
    w, th = v[f"{s}_wrist"], v.get(f"{s}_thumb")
    ix, pk = v.get(f"{s}_index"), v.get(f"{s}_pinky")
    if th is None or ix is None:
        return False
    return (th[1] < w[1] - 0.05 and th[1] < ix[1] - 0.03 and (pk is None or th[1] < pk[1] - 0.03)
            and sh_y - 0.10 < w[1] < hip_y - 0.05 and w[2] < sh_z - 0.08)


def _palm_forward(v, s, sh_y, sh_z):
    w, ix = v[f"{s}_wrist"], v.get(f"{s}_index")
    return ix is not None and (sh_z - w[2]) > 0.30 and abs(w[1] - sh_y) < 0.18 and ix[1] < w[1] - 0.05


def classify_static(lm):
    """One frame -> 'stop' | 'slower' | 'go_on' | None (pure; no timing)."""
    f = _frame(lm)
    if f is None:
        return None
    v, sh_y, sh_z, hip_y, width = f
    if v["l_wrist"][1] < v["nose"][1] - 0.05 and v["r_wrist"][1] < v["nose"][1] - 0.05:
        return "stop"
    lw, rw = v["l_wrist"], v["r_wrist"]
    if (all(sh_y + 0.12 < v[f"{s}_wrist"][1] < hip_y + 0.08 and v[f"{s}_wrist"][2] < sh_z - 0.12 and _flat(v, s) for s in ("l", "r"))
            and abs(lw[0] - rw[0]) > 0.8 * width):
        return "slower"
    for s, o in (("r", "l"), ("l", "r")):
        if (_thumb_up(v, s, sh_y, sh_z, hip_y) or _palm_forward(v, s, sh_y, sh_z)) and _down(v, o):
            return "go_on"
    return None


def beckon_pose(lm):
    """-> side ('l'|'r') whose hand is in the beckon posture (in front, chest height, other hand down), or None."""
    f = _frame(lm)
    if f is None:
        return None
    v, sh_y, sh_z, hip_y, _ = f
    for s, o in (("r", "l"), ("l", "r")):
        w = v[f"{s}_wrist"]
        if f"{s}_index" in v and sh_y - 0.15 < w[1] < hip_y and w[2] < sh_z - 0.10 and _down(v, o):
            return s
    return None


class Detector:
    """Landmarks over time -> (live gesture name | None, fired gesture name | None). Pure apart from its own history."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.cand, self.cand_t, self.cand_w = None, 0.0, None
        self.cool = 0.0
        self.gone = {}                 # name -> time it was last seen (re-arm once gone for REARM s)
        self.armed = {k: True for k in LABELS}
        self.hist = collections.deque(maxlen=90)   # (t, side, q, wrist xyz) for beckon

    def _beckon(self, lm, now):
        side = beckon_pose(lm)
        if side is None:
            self.hist.clear()
            return False
        v = {k: _p(lm, k) for k in lm}
        sh_z = (v["l_shoulder"][2] + v["r_shoulder"][2]) / 2
        if self.hist and self.hist[-1][1] != side:
            self.hist.clear()
        self.hist.append((now, side, float(v[f"{side}_index"][2] - sh_z), v[f"{side}_wrist"]))
        win = [h for h in self.hist if now - h[0] < 1.6]
        if len(win) < 6:
            return False
        q = np.array([h[2] for h in win])
        w = np.array([h[3] for h in win])
        amp = q.max() - q.min()
        if amp < 0.06 or (w[:, 1].max() - w[:, 1].min()) > 0.15 or (w[:, 0].max() - w[:, 0].min()) > max(0.10, amp):
            return False
        dq = np.diff(q)
        sign = np.sign(dq[np.abs(dq) > 0.01])
        return np.count_nonzero(np.diff(sign)) >= 2

    def update(self, lm, now):
        name = classify_static(lm) if lm else None
        if name is None and lm and self._beckon(lm, now):
            name = "come_here"
        elif name is not None or not lm:
            self.hist.clear()
        for k in LABELS:
            if k != name:
                if self.gone.get(k) is None:
                    self.gone[k] = now
                if now - self.gone[k] >= REARM:
                    self.armed[k] = True
            else:
                self.gone[k] = None
        if name is None:
            self.cand = None
            return None, None
        wr = np.concatenate([np.asarray(lm["l_wrist"], float), np.asarray(lm["r_wrist"], float)])
        moved = self.cand_w is not None and float(np.max(np.abs(wr - self.cand_w))) > STILL
        if name != self.cand or (moved and name != "come_here"):
            self.cand, self.cand_t, self.cand_w = name, now, wr
            if name != "come_here":
                return name, None
        if not self.armed.get(name, True) or now < self.cool or now - self.cand_t < HOLD.get(name, 0.0):
            return name, None
        self.armed[name] = False
        self.cool = now + COOLDOWN
        return name, name


# ---------------------------------------------------------------------- server side
def _st(ctx):
    s = ctx.store.get("gestures")
    if s is None:
        s = ctx.store["gestures"] = {"det": Detector(), "live": None, "last": None, "seq": 0, "feed": None,
                                     "nudge": {"key": None, "total": np.zeros(3), "last": None, "seq": 0}}
    return s


def _say(ctx, text):
    if getattr(ctx, "brain", None) is not None:
        ctx.brain.utter(text, "vision")


def _busy(ctx):
    demo, teacher = getattr(ctx, "demo", None), getattr(ctx, "teacher", None)
    if demo is not None and (demo.steps or demo.video_active()):
        return True
    if teacher is not None and getattr(teacher, "phase", "") == "recording":
        return True
    return bool(ctx.rt.engaged)


def _running(ctx):
    rt, tasks = ctx.rt, ctx.tasks
    return rt.mode == "FOLLOW" and rt.path is not None and tasks.kind in ("task", "path")


@ext.handler("gesture_feed")
def on_feed(ctx, msg):
    frames = [f for f in (msg.get("frames") or []) if isinstance(f, dict)]
    st = _st(ctx)
    st["feed"] = {"frames": frames, "fps": float(msg.get("fps") or 20), "t0": time.time()} if frames else None
    st["det"].reset()


@ext.tick
def gestures_tick(ctx):
    st, now = _st(ctx), time.time()
    feed, lm = st["feed"], None
    if feed is not None:
        i = int((now - feed["t0"]) * feed["fps"])
        if i >= len(feed["frames"]):
            st["feed"] = None
            st["det"].reset()
        else:
            lm = feed["frames"][i]
    elif getattr(ctx, "source", None) not in ("synthetic", "video"):
        lm = getattr(ctx, "lm", None)
    if _busy(ctx):
        lm = None
    live, fired = st["det"].update(lm, now)
    st["live"] = live
    brain = getattr(ctx, "brain", None)
    if live and live != "stop" and brain is not None and brain.enabled and hasattr(brain, "_cand"):
        brain._cand, brain._cand_t = None, now   # our gesture wins: keep the mock brain from reading it as pointing
    if fired:
        effect = _apply(ctx, fired)
        if effect is not None:
            st["seq"] += 1
            st["last"] = {"id": st["seq"], "name": fired, "label": LABELS[fired], "effect": effect, "t": now}


def _apply(ctx, name):
    rt, tasks = ctx.rt, ctx.tasks
    if name == "stop":
        if ctx.brain is not None and ctx.brain.enabled:
            return None                      # the mock brain owns stop when it's on
        tasks.stop("g-stop")
        _say(ctx, "You raised both hands, so I stopped.")
        return "Stopped"
    if name == "slower":
        rt.speed_scale = max(0.1, rt.speed_scale * 0.7)
        _say(ctx, f"Slowing down to x{rt.speed_scale:.1f}.")
        return f"Speed x{rt.speed_scale:.1f}"
    if name == "come_here":
        return _come_here(ctx)
    if name == "go_on":
        hook = ctx.store.get("confirm_hook")
        if callable(hook):
            msg = hook(ctx)
            if msg:
                _say(ctx, msg)
                return "Confirmed"
        if rt.hold:
            rt.hold = False
            _say(ctx, "Carrying on.")
            return "Resumed"
        if not _running(ctx) and _pending_path(rt) and all(rt.path["ok"]):
            tasks.follow_path("g-go")
            _say(ctx, "Going: following your path.")
            return "Following the path"
        _say(ctx, "Okay, carrying on." if _running(ctx) else "Okay. Tell me, point, or show me what to do.")
        return "Okay"
    return None


def _come_here(ctx):
    """Toward the person (they face the robot across the table, +x): nudge a running target closer, otherwise reach out
    toward them; whatever the robot holds comes along."""
    rt, tasks = ctx.rt, ctx.tasks
    if _running(ctx):
        msg = nudge(ctx, np.array([0.04, 0.0, 0.0]))
        ok = msg.startswith("Moved")
        _say(ctx, "Bringing it 4 cm closer to you." if ok else msg)
        return "4 cm closer" if ok else "Can't come closer"
    from ..tasks import TABLE_TOP, densify
    palm = next((h for h, o in tasks.holding.items() if o), None) or "right_palm"
    y = 0.12 if palm.startswith("left") else -0.12
    cur = rt.data.site_xpos[rt.palm_sid[palm]].copy()
    for x in COME_X:
        p = np.array([x, y, TABLE_TOP + 0.22])
        if all(rt.precheck(densify([cur, p], 0.06), palm)):
            break
    else:
        _say(ctx, "I can't reach any closer to you from here.")
        return "Out of reach"
    held = tasks.holding.get(palm)
    tasks.done_msg = "Here I am, as close as I can reach."
    ok = tasks._launch("g-come", "reach", [cur, p], palm, {}, "task",
                       f"Coming to you{' with ' + tasks.pretty(held) if held else ''}: reaching toward you.")
    _say(ctx, f"Coming closer{' with ' + tasks.pretty(held) if held else ''}." if ok else "I can't reach toward you from here.")
    return "Reaching toward you" if ok else "Out of reach"


# ---------------------------------------------------------------------- nudge pad
def _target(ctx):
    """-> (kind, label) of what a nudge would move, or (None, None)."""
    hook = ctx.store.get("nudge_hook")
    if callable(hook):
        label = hook(ctx, None)
        if label:
            return "hook", str(label)
    rt, tasks = ctx.rt, ctx.tasks
    if _running(ctx):
        rel = _release_idx(tasks)
        if rel is not None and rel in tasks.fired:
            return None, None
        if tasks.task and tasks.task.get("action") == "pick_place":
            return "running", "where it goes"
        return "running", "the hand target" if tasks.kind == "task" else "the rest of the path"
    if tasks.markers:
        return "marker", f"marker {tasks.markers[-1]['id']}"
    if _pending_path(rt):
        return "path", "your path"
    return None, None


def _key(ctx, kind, label):
    """Identity of the active target: the nudge total resets when it changes."""
    return kind, (ctx.tasks.task or {}).get("id") if kind == "running" else label


def _target_pos(ctx, kind):
    """World point of the active target (the release / end waypoint, the marker), for display and checks."""
    rt, tasks = ctx.rt, ctx.tasks
    if kind == "running":
        rel = _release_idx(tasks)
        return rt.path["pts"][rel if rel is not None else -1]
    if kind == "marker":
        return tasks.markers[-1]["pos"]
    if kind == "path":
        return rt.path["pts"][-1]
    return None


def _pending_path(rt):
    """A drawn (or nudged) path that hasn't been followed yet."""
    return rt.path is not None and bool(rt.path_msg) and int(rt.path.get("i", 0)) == 0


def _release_idx(tasks):
    return next((i for i, n in sorted(tasks.events.items()) if n == "release"), None)


def nudge(ctx, delta):
    """Move the active target by delta (robot frame, metres). Returns a message."""
    st = _st(ctx)["nudge"]
    kind, label = _target(ctx)
    if kind is None:
        return "Nothing to nudge: start a task, point at a spot, or draw a path first."
    rt, tasks = ctx.rt, ctx.tasks
    if kind == "hook":
        return str(ctx.store["nudge_hook"](ctx, delta) or f"Moved {label}.")
    key = _key(ctx, kind, label)
    if st["key"] != key:
        st["key"], st["total"] = key, np.zeros(3)
    want = np.clip(st["total"] + delta, -TOTAL_MAX, TOTAL_MAX)
    delta = want - st["total"]
    if not np.any(np.abs(delta) > 1e-6):
        return f"That's as far as I nudge {label} ({TOTAL_MAX * 100:.0f} cm)."
    if kind == "marker":
        from ..tasks import TABLE_TOP
        mk = tasks.markers[-1]
        p = mk["pos"] + np.array([delta[0], delta[1], 0.0])
        if not tasks.on_table(p):
            return f"Marker {mk['id']} would leave the table."
        if abs(delta[0]) < 1e-6 and abs(delta[1]) < 1e-6:
            return "Markers stay on the table: use the arrows, not up/down."
        mk["pos"] = np.array([p[0], p[1], TABLE_TOP])
        st["total"] = st["total"] + np.array([delta[0], delta[1], 0.0])
        return f"Moved marker {mk['id']} by {_cm(delta)}."
    if kind == "path":
        pts = [np.asarray(q, float) + delta for q in rt.path["pts"]]
        tasks.set_path_world(pts, rt.path.get("hand"))
        st["total"] = want
        return f"Moved your path by {_cm(delta)}."
    # running: shift the waypoints ahead of the hand (after the grasp, up to the release; fade out on the way home)
    path, i = rt.path, int(rt.path["i"])
    pts = np.array(path["pts"], float)
    grasp = next((j for j, n in sorted(tasks.events.items()) if n == "grasp" and j not in tasks.fired), None)
    start = max(i + 1, (grasp + 1) if grasp is not None else 0)
    rel = _release_idx(tasks)
    end = rel if rel is not None else len(pts) - 1
    if start > end:
        return "Too late to nudge this one."
    w = np.zeros(len(pts))
    w[start:end + 1] = 1.0
    if rel is not None and end + 1 < len(pts):
        tail = len(pts) - 1 - end
        w[end + 1:] = np.linspace(1.0, 0.0, tail + 1)[1:]
    new = pts + w[:, None] * delta[None, :]
    probe = list(new[start:end + 1:4]) + [new[end]]
    if not all(rt.precheck(np.array(probe), path["palm"])[-1:]):
        return f"I can't reach {label} if it moves {_cm(delta)}."
    seg = np.linalg.norm(np.diff(new, axis=0), axis=1)
    path["pts"], path["cum"] = new, np.concatenate([[0.0], np.cumsum(seg)])
    st["total"] = want
    return f"Moved {label} by {_cm(delta)}."


def _cm(d):
    names = (("forward", "back"), ("left", "right"), ("up", "down"))
    parts = [f"{abs(v) * 100:.0f} cm {names[k][0] if v > 0 else names[k][1]}" for k, v in enumerate(d) if abs(v) > 1e-6]
    return ", ".join(parts) or "0 cm"


@ext.handler("nudge")
def on_nudge(ctx, msg):
    try:
        d = np.array([float(msg.get(k) or 0.0) for k in ("dx", "dy", "dz")])
    except (TypeError, ValueError):
        return
    if not np.all(np.isfinite(d)):
        return
    d = np.clip(d, -STEP_MAX, STEP_MAX)
    text = nudge(ctx, d)
    st = _st(ctx)["nudge"]
    st["seq"] += 1
    st["last"] = {"id": st["seq"], "message": text, "ok": text.startswith("Moved")}


@ext.handler("speed")
def on_speed(ctx, msg):
    try:
        s = float(msg.get("scale"))
    except (TypeError, ValueError):
        return
    if np.isfinite(s):
        ctx.rt.speed_scale = float(np.clip(s, 0.1, 1.0))


@ext.state
def gestures_state(ctx):
    st = _st(ctx)
    kind, label = _target(ctx)
    pos = _target_pos(ctx, kind)
    last = st["last"]
    return {
        "gestures": {"live": st["live"], "active": not _busy(ctx),
                     "last": None if last is None else {k: last[k] for k in ("id", "name", "label", "effect")},
                     "last_age": None if last is None else round(time.time() - last["t"], 2)},
        "nudge": {"active": kind is not None, "target": label, "kind": kind,
                  "offset": [round(float(x), 3) for x in st["nudge"]["total"]] if kind is not None and st["nudge"]["key"] == _key(ctx, kind, label) else [0, 0, 0],
                  "pos": None if pos is None else [round(float(x), 3) for x in pos], "last": st["nudge"]["last"]},
    }
