"""P2 trajectories: one object for drawn, VR, language, demonstrated, keyframed and skill trajectories (docs/V1.md P2).

Pure (numpy only, no server, no MuJoCo), so it is unit tested in tests/test_traj.py:

* `normalize(obj)`: schema validation; returns the canonical trajectory dict (normalize(normalize(x)) == normalize(x)).
* Frames: `world` or `object:<name>` (origin at the top centre of that object, axes parallel to the world), resolved
  against the current object poses with `to_world` / `to_frame`.
* `densify(traj, q0, kind)`: timed samples (30 Hz) with min-jerk-shaped starts and stops, constant speed between, per
  point `speed`, slerped orientation (default: the current approach-down orientation `q0`, turned by per point `yaw` /
  `tilt`), the hand stopping where the gripper changes or a point says `hold`, and the `pattern` superimposed.
* `check(dense, checkers)`: per point verdicts from callbacks (reach / joint limits via IK, zones, collisions, P5 safety)
  -> `{ok[], why, duration}`.
* `from_keypoints(spec, world)`: what the language model writes ({"rel": "above", "of": "red_cup", "dz": 0.1},
  {"via": "over", "of": "vase"}, ...) -> a concrete trajectory, lifted automatically over obstacles along each segment.
* `select` / `edit`: "higher there", "slower near the vase" -> point indices -> changes.
* `follow_step(rt, dt)`: runtime glue (duck-typed): the robot's hand target follows the dense samples in time.

The trajectory object (docs/V1.md P2, docs/PROTOCOL.md v9):
  {"id", "role": "right"|"left", "frame": "world"|"object:<name>", "source": "draw|vr|language|demo|keyframes|skill",
   "label", "points": [{"p": [x,y,z], "q": [w,x,y,z]|null, "t": s|null, "grip": 0..1|null, "speed": m/s|null,
                        "force": N|null, "tol": m|null, "note": str|null, + optional "yaw" (deg), "tilt" (deg), "roll" (deg),
                        "hold" (s)}],
   "pattern": {"kind": "zigzag|circle|figure8", "amp": m, "freq": Hz, "axis": "lateral|normal|vertical",
               "span": [i0, i1]?} | null,
   "check": {"ok": [bool per point], "why": str|null, "duration": s} | null, "meta"?: {...}}
"""
import math
import re

import numpy as np

SOURCES = ("draw", "vr", "language", "demo", "keyframes", "skill")
PATTERNS = ("zigzag", "circle", "figure8")
AXES = ("lateral", "normal", "vertical")
DEFAULT_SPEED = {"arm": 0.25, "bimanual": 0.20, "humanoid": 0.20, "mobile_manipulator": 0.15, "quadruped_arm": 0.15}
FALLBACK_SPEED = 0.20     # m/s
RAMP = 0.5                # s: speeding up from / slowing down to a stop (min-jerk shaped)
GRIP_DWELL = 0.4          # s: the hand stops this long where the gripper opens or closes
DT = 1 / 30               # s between dense samples
MAX_POINTS = 400
CHECK_STEP = 0.025        # m between checked samples
HOVER = 0.10              # m: default height of "above X" over the object's top
HAND_R = 0.04             # m: the hand (gripper) as a sphere, for clearance
CLEARANCE = 0.05          # m: default gap over an obstacle
UP = np.array([0.0, 0.0, 1.0])
DOWN = -UP


class TrajError(ValueError):
    """A trajectory or keypoint spec that can't be used; the message is a sentence for the person."""


# ---------------------------------------------------------------------------- small helpers
def _num(v, lo=None, hi=None, what="value"):
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise TrajError(f"{what} must be a number") from None
    if not math.isfinite(x):
        raise TrajError(f"{what} must be finite")
    if lo is not None:
        x = max(lo, x)
    if hi is not None:
        x = min(hi, x)
    return x


def _vec(v, n, what):
    if not isinstance(v, (list, tuple, np.ndarray)) or len(v) != n:
        raise TrajError(f"{what} must be {n} numbers")
    return [_num(x, what=what) for x in v]


def _r(x, n=4):
    return round(float(x), n)


def smoothstep5(x):
    """Quintic 0..1 ramp with zero velocity and acceleration at both ends (the min-jerk position profile)."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * x * (10 - 15 * x + 6 * x * x)


def _ramp_area(x):
    """Integral of smoothstep5 from 0 to x (x in 0..1); equals 1/2 at x = 1."""
    x = np.clip(x, 0.0, 1.0)
    return x ** 6 - 3 * x ** 5 + 2.5 * x ** 4


def _unit(v, fallback=None):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return None if fallback is None else np.asarray(fallback, float)
    return v / n


# ---------------------------------------------------------------------------- quaternions (w, x, y, z)
def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qaxis(axis, angle):
    a = _unit(axis, [0, 0, 1])
    return np.concatenate([[math.cos(angle / 2)], math.sin(angle / 2) * a])


def qnorm(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = q / n
    return -q if q[0] < 0 else q


def qrot(q, v):
    """Rotate vector v by quaternion q."""
    w, x, y, z = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def slerp(a, b, u):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        return qnorm(a + u * (b - a))
    th = math.acos(min(1.0, d))
    s = math.sin(th)
    return qnorm((math.sin((1 - u) * th) * a + math.sin(u * th) * b) / s)


def qangle(a, b):
    d = abs(float(np.dot(qnorm(a), qnorm(b))))
    return 2 * math.acos(min(1.0, d))


# ---------------------------------------------------------------------------- schema
POINT_KEYS = ("p", "q", "t", "grip", "speed", "force", "tol", "note")
EXTRA_KEYS = ("yaw", "tilt", "roll", "hold")


def normalize_point(pt, i=0):
    if isinstance(pt, (list, tuple)):
        pt = {"p": pt}
    if not isinstance(pt, dict):
        raise TrajError(f"point {i + 1} must be an object with p: [x, y, z]")
    if "p" not in pt:
        raise TrajError(f"point {i + 1} has no position p")
    p = _vec(pt["p"], 3, f"point {i + 1} position")
    if max(abs(x) for x in p) > 10:
        raise TrajError(f"point {i + 1} is more than 10 m away")
    q = pt.get("q")
    if q is not None:
        q = [_r(x, 5) for x in qnorm(_vec(q, 4, f"point {i + 1} orientation"))]
    note = pt.get("note")
    out = {"p": [_r(x) for x in p], "q": q,
           "t": None if pt.get("t") is None else _r(_num(pt["t"], 0.0, 3600.0, "t"), 3),
           "grip": None if pt.get("grip") is None else _r(_num(pt["grip"], 0.0, 1.0, "grip"), 3),
           "speed": None if pt.get("speed") is None else _r(_num(pt["speed"], 0.005, 1.5, "speed"), 4),
           "force": None if pt.get("force") is None else _r(_num(pt["force"], 0.0, 200.0, "force"), 2),
           "tol": None if pt.get("tol") is None else _r(_num(pt["tol"], 0.001, 0.2, "tol"), 4),
           "note": None if note in (None, "") else str(note)[:80]}
    if pt.get("yaw") is not None:
        y = _num(pt["yaw"], what="yaw")
        y = (y + 180.0) % 360.0 - 180.0
        out["yaw"] = _r(180.0 if y == -180.0 else y, 2)
    if pt.get("tilt") is not None:
        out["tilt"] = _r(_num(pt["tilt"], -80.0, 80.0, "tilt"), 2)
    if pt.get("roll"):          # P4: lean sideways about the direction of travel (a weld's work angle)
        out["roll"] = _r(_num(pt["roll"], -60.0, 60.0, "roll"), 2)
    if pt.get("hold") is not None and _num(pt["hold"], 0.0, 30.0, "hold") > 0:
        out["hold"] = _r(_num(pt["hold"], 0.0, 30.0, "hold"), 3)
    return out


def normalize_pattern(pat):
    if pat in (None, False, {}):
        return None
    if not isinstance(pat, dict):
        raise TrajError("pattern must be {kind, amp, freq, axis}")
    kind = str(pat.get("kind") or "zigzag").strip().lower().replace("-", "").replace(" ", "")
    kind = {"weave": "zigzag", "zig": "zigzag", "figureeight": "figure8", "eight": "figure8", "spiral": "circle"}.get(kind, kind)
    if kind not in PATTERNS:
        raise TrajError(f"pattern kind must be one of {', '.join(PATTERNS)}")
    axis = str(pat.get("axis") or "lateral").strip().lower()
    if axis not in AXES:
        raise TrajError(f"pattern axis must be one of {', '.join(AXES)}")
    out = {"kind": kind, "amp": _r(_num(pat.get("amp", 0.005), 0.0005, 0.2, "pattern amp"), 4),
           "freq": _r(_num(pat.get("freq", 2.0), 0.05, 10.0, "pattern freq"), 3), "axis": axis}
    sp = pat.get("span")
    if isinstance(sp, (list, tuple)) and len(sp) == 2:
        out["span"] = [int(_num(sp[0], what="span")), int(_num(sp[1], what="span"))]
    if pat.get("apply") is not None:
        out["apply"] = bool(pat["apply"])
    return out


def normalize_check(ch, n):
    if not isinstance(ch, dict):
        return None
    ok = ch.get("ok")
    ok = [bool(x) for x in ok][:n] if isinstance(ok, (list, tuple)) else None
    out = {"ok": ok, "why": None if not ch.get("why") else str(ch["why"])[:300],
           "duration": None if ch.get("duration") is None else _r(_num(ch["duration"], 0.0, 1e5, "duration"), 2)}
    return out


def normalize_frame(frame):
    f = str(frame or "world").strip()
    if f == "world":
        return f
    m = re.fullmatch(r"object:\s*([\w .-]{1,48})", f)
    if not m:
        raise TrajError("frame must be 'world' or 'object:<name>'")
    return "object:" + m.group(1).strip().lower().replace(" ", "_")


def normalize(obj, default_role="right"):
    """Validate and canonicalise a trajectory object. Raises TrajError with a readable sentence."""
    if not isinstance(obj, dict):
        raise TrajError("a trajectory must be an object with points")
    pts = obj.get("points")
    if not isinstance(pts, (list, tuple)) or not pts:
        raise TrajError("a trajectory needs at least one point")
    if len(pts) > MAX_POINTS:
        raise TrajError(f"a trajectory can have at most {MAX_POINTS} points")
    role = str(obj.get("role") or default_role).strip().lower().replace("_palm", "")
    if role not in ("right", "left"):
        raise TrajError("role must be 'right' or 'left'")
    source = str(obj.get("source") or "draw").strip().lower()
    if source not in SOURCES:
        raise TrajError(f"source must be one of {', '.join(SOURCES)}")
    points = [normalize_point(p, i) for i, p in enumerate(pts)]
    out = {"id": str(obj.get("id") or "t")[:40], "role": role, "frame": normalize_frame(obj.get("frame")), "source": source,
           "label": str(obj.get("label") or "")[:160], "points": points, "pattern": normalize_pattern(obj.get("pattern")),
           "check": normalize_check(obj.get("check"), len(points))}
    if isinstance(obj.get("meta"), dict):
        out["meta"] = obj["meta"]
    return out


def all_timed(traj):
    ts = [p["t"] for p in traj["points"]]
    return len(ts) > 1 and all(t is not None for t in ts) and all(b >= a for a, b in zip(ts, ts[1:])) and ts[-1] > ts[0]


# ---------------------------------------------------------------------------- frames
def _key(name):
    return str(name or "").strip().lower().replace(" ", "_")


def frame_object(frame):
    return frame[7:] if isinstance(frame, str) and frame.startswith("object:") else None


def frame_origin(frame, poses):
    """poses: {object name: [x, y, z] origin (top centre)}. -> np.array(3)."""
    name = frame_object(frame)
    if name is None:
        return np.zeros(3)
    look = {_key(k): v for k, v in (poses or {}).items()}
    if _key(name) not in look:
        raise TrajError(f"the trajectory is relative to the {name.replace('_', ' ')}, which isn't in the scene")
    return np.asarray(look[_key(name)], float)


def to_world(traj, poses):
    """Copy of the trajectory in the world frame (points + the object's current origin)."""
    o = frame_origin(traj["frame"], poses)
    out = dict(traj, frame="world", points=[dict(p, p=[_r(a + b) for a, b in zip(p["p"], o)]) for p in traj["points"]])
    return out


def to_frame(traj, frame, poses):
    """World-frame trajectory -> the same points expressed in `frame`."""
    frame = normalize_frame(frame)
    o = frame_origin(frame, poses)
    return dict(traj, frame=frame, points=[dict(p, p=[_r(a - b) for a, b in zip(p["p"], o)]) for p in traj["points"]])


# ---------------------------------------------------------------------------- orientation of a point
def _tangents(P):
    n = len(P)
    T = []
    for i in range(n):
        a, b = P[max(0, i - 1)], P[min(n - 1, i + 1)]
        T.append(_unit(np.array([b[0] - a[0], b[1] - a[1], 0.0]), [1.0, 0.0, 0.0]))
    return T


def point_quat(pt, q0, tangent):
    """Absolute q wins; otherwise the default orientation q0 turned by yaw (about the vertical) then tilted toward the
    direction of travel (tilt > 0 leans the tool's tip forward along the path) and rolled about it (roll > 0 leans the
    tip to the left of the travel)."""
    if pt.get("q") is not None:
        return qnorm(pt["q"])
    q = qnorm(q0 if q0 is not None else [1, 0, 0, 0])
    if pt.get("yaw"):
        q = qmul(qaxis(UP, math.radians(pt["yaw"])), q)
    if pt.get("tilt"):
        axis = _unit(np.cross(DOWN, tangent), [0, -1, 0])
        q = qmul(qaxis(axis, math.radians(pt["tilt"])), q)
    if pt.get("roll"):
        q = qmul(qaxis(_unit([tangent[0], tangent[1], 0.0], [1, 0, 0]), math.radians(pt["roll"])), q)
    return qnorm(q)


# ---------------------------------------------------------------------------- densify
def default_speed(kind):
    return DEFAULT_SPEED.get(str(kind or ""), FALLBACK_SPEED)


def _stops(points):
    """Indices where the hand comes to rest: first, last, holds, and gripper changes. -> {index: dwell seconds}."""
    n = len(points)
    stops, grip = {0: 0.0, n - 1: 0.0}, None
    for i, p in enumerate(points):
        d = 0.0
        if p.get("grip") is not None:
            if grip is None or abs(p["grip"] - grip) > 1e-6:     # a gripper command: the hand waits while it moves
                d = GRIP_DWELL
            grip = p["grip"]
        if p.get("hold"):
            d = max(d, p["hold"])
        if d > 0 or i in (0, n - 1):
            stops[i] = max(stops.get(i, 0.0), d)
    return stops


def _leg_geometry(P, ds=0.002, blend=0.025):
    """Polyline through P with interior corners rounded by a small quadratic blend (keeps the hand's speed smooth).
    -> pts (M,3), s (M,), s_key (len(P),)."""
    k = len(P)
    pts, s_key = [P[0]], [0.0]
    s = [0.0]

    def emit(q):
        s.append(s[-1] + float(np.linalg.norm(q - pts[-1])))
        pts.append(q)

    def line(a, b):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / ds)))
        for j in range(1, n + 1):
            emit(a + (b - a) * j / n)

    prev = P[0]
    for j in range(1, k):
        c = P[j]
        rounded = False
        if j < k - 1:
            a_in, a_out = c - P[j - 1], P[j + 1] - c
            L_in, L_out = np.linalg.norm(a_in), np.linalg.norm(a_out)
            if L_in > 1e-6 and L_out > 1e-6:
                cosang = float(np.dot(a_in, a_out) / (L_in * L_out))
                b = min(blend, 0.35 * L_in, 0.35 * L_out)
                if cosang < math.cos(math.radians(5)) and b > 1e-4:
                    A, B = c - a_in / L_in * b, c + a_out / L_out * b
                    line(prev, A)
                    n = max(4, int(math.ceil(2 * b / ds)))
                    for m in range(1, n + 1):
                        u = m / n
                        emit((1 - u) ** 2 * A + 2 * u * (1 - u) * c + u * u * B)
                        if m == n // 2:
                            s_key.append(s[-1])
                    prev, rounded = B, True
        if not rounded:
            line(prev, c)
            s_key.append(s[-1])
            prev = c
    return np.array(pts), np.array(s), np.array(s_key)


def _leg_times(Tn, dt):
    """Nominal (constant speed) duration Tn -> actual sample times and the nominal time reached at each (ramps at
    both ends shaped like min-jerk). Returns (t, tau)."""
    if Tn < 1e-6:
        return np.array([0.0]), np.array([0.0])
    Tr = min(RAMP, Tn)
    T = Tn + Tr
    n = max(1, int(math.ceil(T / dt)))
    t = np.linspace(0.0, T, n + 1)
    tau = np.where(t < Tr, Tr * _ramp_area(t / Tr),
                   np.where(t > T - Tr, Tn - Tr * _ramp_area((T - t) / Tr), Tr / 2 + (t - Tr)))
    return t, np.clip(tau, 0.0, Tn)


def densify(traj, q0=None, kind=None, speed=None, dt=DT, apply_pattern=None):
    """World-frame trajectory -> dense timed samples.

    -> {"t": (N,), "p": (N,3), "q": (N,4), "grip": (N,) (nan = no command yet), "force": (N,) (nan = none),
        "key": [sample index of each point], "events": [(t, "grip"|"force", value, point index)], "duration": s,
        "tangent": (N,3)}"""
    pts = traj["points"]
    n = len(pts)
    P = np.array([p["p"] for p in pts], float)
    v0 = float(speed) if speed else default_speed(kind)
    T0 = _tangents(P)
    Q = [point_quat(p, q0, T0[i]) for i, p in enumerate(pts)]
    pat = traj.get("pattern")
    if pat:                       # >= 24 samples per cycle so the follower keeps the pattern's shape
        dt = min(dt, 1.0 / (24 * float(pat["freq"])))
    if all_timed(traj):
        t_out, p_out, q_out, key = _densify_timed(traj, P, Q, dt)
    else:
        t_out, p_out, q_out, key = _densify_speed(pts, P, Q, v0, dt)
    N = len(t_out)
    grip, force = np.full(N, np.nan), np.full(N, np.nan)
    events, g_cur, f_cur = [], None, None
    for i, p in enumerate(pts):
        k = key[i]
        if p.get("grip") is not None and (g_cur is None or abs(p["grip"] - g_cur) > 1e-6):
            g_cur = p["grip"]
            grip[k:] = g_cur
            events.append((float(t_out[k]), "grip", g_cur, i))
        if p.get("force") is not None and (f_cur is None or abs(p["force"] - f_cur) > 1e-6):
            f_cur = p["force"]
            force[k:] = f_cur
            events.append((float(t_out[k]), "force", f_cur, i))
    events.sort(key=lambda e: e[0])
    tang = np.zeros_like(p_out)
    if N > 1:
        d = np.gradient(p_out, axis=0)
        for j in range(N):
            u = _unit(d[j])
            tang[j] = u if u is not None else (tang[j - 1] if j else np.array([1.0, 0, 0]))
    else:
        tang[:] = [1.0, 0, 0]
    if apply_pattern is None:
        apply_pattern = bool(pat) and (pat.get("apply") if pat.get("apply") is not None else traj.get("source") != "demo")
    if pat and apply_pattern and N > 2:
        p_out = p_out + pattern_offsets(pat, t_out, tang, key, n)
    return {"t": t_out, "p": p_out, "q": q_out, "grip": grip, "force": force, "key": list(key), "events": events,
            "duration": float(t_out[-1]), "tangent": tang}


def _densify_speed(pts, P, Q, v0, dt):
    n = len(pts)
    stops = _stops(pts)
    order = sorted(stops)
    speeds = np.array([p["speed"] if p.get("speed") else v0 for p in pts], float)
    t_all, p_all, q_all, key = [0.0], [P[0]], [Q[0]], [0] * n
    t_now = 0.0

    def dwell(sec, i):
        nonlocal t_now
        if sec <= 0:
            return
        m = max(1, int(math.ceil(sec / dt)))
        for j in range(1, m + 1):
            t_all.append(t_now + sec * j / m)
            p_all.append(P[i])
            q_all.append(Q[i])
        t_now += sec

    dwell(stops.get(0, 0.0), 0)
    for a, b in zip(order[:-1], order[1:]):
        geo, s, s_key = _leg_geometry(P[a:b + 1])
        L = float(s[-1])
        v = np.interp(s, s_key, speeds[a:b + 1])
        tau_s = np.concatenate([[0.0], np.cumsum(np.diff(s) / np.maximum(0.5 * (v[1:] + v[:-1]), 1e-4))])
        t, tau = _leg_times(float(tau_s[-1]), dt)
        sq = np.interp(tau, tau_s, s) if L > 1e-9 else np.zeros_like(t)
        base = len(t_all) - 1                       # the sample this leg starts from (point a)
        pos = np.stack([np.interp(sq, s, geo[:, c]) for c in range(3)], axis=1)
        ks = np.clip(np.searchsorted(s_key, sq, side="right") - 1, 0, max(0, len(s_key) - 2))
        for j in range(1, len(t)):
            k = int(ks[j])
            span = s_key[k + 1] - s_key[k] if len(s_key) > 1 else 0.0
            u = 0.0 if span < 1e-9 else float(np.clip((sq[j] - s_key[k]) / span, 0, 1))
            t_all.append(t_now + t[j])
            p_all.append(pos[j])
            q_all.append(slerp(Q[a + k], Q[a + min(k + 1, b - a)], u))
        # each point's sample: where the timeline passes its arc position (point a keeps its arrival sample)
        for m in range(a + 1, b + 1):
            key[m] = base + int(np.argmin(np.abs(sq - s_key[m - a])))
        key[b] = len(t_all) - 1
        t_now += float(t[-1])
        dwell(stops.get(b, 0.0), b)
    return np.array(t_all), np.array(p_all), np.array(q_all), key


def _densify_timed(traj, P, Q, dt):
    ts = np.array([p["t"] for p in traj["points"]], float)
    ts = ts - ts[0]
    T = float(ts[-1])
    m = max(1, int(math.ceil(T / dt)))
    t = np.linspace(0.0, T, m + 1)
    p = np.stack([np.interp(t, ts, P[:, c]) for c in range(3)], axis=1)
    q = []
    for tj in t:
        k = int(np.clip(np.searchsorted(ts, tj, side="right") - 1, 0, len(ts) - 2))
        span = ts[k + 1] - ts[k]
        q.append(slerp(Q[k], Q[k + 1], 0.0 if span < 1e-9 else float(np.clip((tj - ts[k]) / span, 0, 1))))
    key = [int(np.argmin(np.abs(t - x))) for x in ts]
    return t, p, np.array(q), key


def _tri(phi):
    return (2 / math.pi) * np.arcsin(np.sin(phi))


def pattern_offsets(pat, t, tang, key, n_points):
    """Offsets (N,3) of the pattern superimposed on the samples between span points (default: the whole path)."""
    N = len(t)
    off = np.zeros((N, 3))
    i0, i1 = (pat.get("span") or [0, n_points - 1])
    i0, i1 = int(np.clip(i0 if i0 >= 0 else n_points + i0, 0, n_points - 1)), int(np.clip(i1 if i1 >= 0 else n_points + i1, 0, n_points - 1))
    j0, j1 = key[min(i0, i1)], key[max(i0, i1)]
    if j1 - j0 < 2:
        return off
    T = t[j1] - t[j0]
    f, amp = float(pat["freq"]), float(pat["amp"])
    Tw = min(0.5 / f, T / 4)
    for j in range(j0, j1 + 1):
        tau = t[j] - t[j0]
        env = float(smoothstep5(tau / Tw) * smoothstep5((T - tau) / Tw)) if Tw > 1e-6 else 1.0
        tg = tang[j]
        th = _unit([tg[0], tg[1], 0.0])
        lat = _unit(np.cross(UP, th)) if th is not None else np.array([0.0, 1.0, 0.0])
        th = th if th is not None else np.array([1.0, 0.0, 0.0])
        a1, a2 = (UP, th) if pat["axis"] == "vertical" else (lat, th)
        phi = 2 * math.pi * f * tau
        if pat["kind"] == "zigzag":
            o = _tri(phi) * a1
        elif pat["kind"] == "circle":
            o = math.sin(phi) * a1 + math.cos(phi) * a2
        else:
            o = math.sin(phi) * a1 + 0.5 * math.sin(2 * phi) * a2
        off[j] = amp * env * o
    return off


# ---------------------------------------------------------------------------- checks
def check_samples(dense, step=CHECK_STEP, extra=()):
    """Indices of the dense samples to check / draw: every point, gripper events, and one every `step` metres."""
    p = dense["p"]
    keep = set(dense["key"]) | {0, len(p) - 1} | set(extra)
    for e in dense["events"]:
        keep.add(int(np.searchsorted(dense["t"], e[0])))
    acc = 0.0
    for j in range(1, len(p)):
        acc += float(np.linalg.norm(p[j] - p[j - 1]))
        if acc >= step:
            keep.add(j)
            acc = 0.0
    return sorted(k for k in keep if 0 <= k < len(p))


def check(dense, checkers, idx=None, n_points=None, approach=None):
    """Run every checker on the chosen samples.

    checkers: callables fn(pts (M,3), quats (M,4), info) -> [reason | None] * M (None = fine). The first checker that
    objects wins for a sample, so order them by priority (reach first). info = {"idx": sample indices, "dense": dense}.
    approach: optional (M,3) points of the move from the hand to the start, checked too (reported as "on the way").
    -> {"ok": [bool per point], "why": str|None, "duration": s, "idx": [...], "sample_ok": [...], "issues": [...],
        "approach_ok": [...]}"""
    idx = list(idx if idx is not None else check_samples(dense))
    pts, qs = dense["p"][idx], dense["q"][idx]
    a_pts = np.zeros((0, 3)) if approach is None else np.asarray(approach, float).reshape(-1, 3)
    allp = np.concatenate([a_pts, pts]) if len(a_pts) else pts
    allq = np.concatenate([np.repeat(qs[:1], len(a_pts), axis=0), qs]) if len(a_pts) else qs
    reasons = [None] * len(allp)
    info = {"idx": idx, "dense": dense, "n_approach": len(a_pts)}
    for fn in checkers or ():
        try:
            got = fn(allp, allq, info)
        except Exception as e:      # a broken checker must not hide the others
            got = [f"check failed ({e})"] + [None] * (len(allp) - 1)
        for j, r in enumerate(got or []):
            if j < len(reasons) and reasons[j] is None and r:
                reasons[j] = str(r)
    a_reasons, s_reasons = reasons[:len(a_pts)], reasons[len(a_pts):]
    key = dense["key"]
    n_points = n_points or len(key)
    pos = {k: j for j, k in enumerate(idx)}
    ok_pts = [pos.get(key[i]) is None or s_reasons[pos[key[i]]] is None for i in range(n_points)]
    issues = []
    worst_a = _worst([r for r in a_reasons if r])
    if worst_a:
        issues.append({"where": "on the way to the start", "i": None, "span": None, "why": worst_a})
    # consecutive failing samples with the same kind of reason -> one issue over a span of points, with the worst number
    runs, cur = [], None
    for j, r in enumerate(s_reasons):
        kind = _kind_of(r) if r else None
        if kind and cur and cur["kind"] == kind and cur["end"] == j - 1:
            cur["end"], cur["reasons"] = j, cur["reasons"] + [r]
        elif kind:
            cur = {"kind": kind, "start": j, "end": j, "reasons": [r]}
            runs.append(cur)
    for run in runs[:8]:
        k0, k1 = idx[run["start"]], idx[run["end"]]
        i0 = max([i for i in range(n_points) if key[i] <= k0] or [0])
        i1 = min([i for i in range(n_points) if key[i] >= k1] or [n_points - 1])
        at = [i for i in range(n_points) if k0 <= key[i] <= k1]
        if len(at) == 1 and k0 == k1:
            where = f"at point {at[0] + 1}"
        elif i1 - i0 <= 1:
            where = f"between points {i0 + 1} and {i1 + 1}"
        else:
            where = f"from point {i0 + 1} to point {i1 + 1}"
        issues.append({"where": where, "i": at[0] if at else i0, "span": [i0, i1], "why": _worst(run["reasons"])})
    why = None
    if issues:
        why = f"{issues[0]['why'][:1].upper()}{issues[0]['why'][1:]} {issues[0]['where']}"
        if len(issues) > 1:
            why += f", and {len(issues) - 1} more place{'s' if len(issues) > 2 else ''}"
        why += "."
    return {"ok": ok_pts, "why": why, "duration": round(float(dense["duration"]), 2), "idx": idx,
            "sample_ok": [r is None for r in s_reasons], "approach_ok": [r is None for r in a_reasons], "issues": issues}


def _kind_of(reason):
    return re.sub(r"[0-9.]+", "#", str(reason))


def _worst(reasons):
    """The reason with the largest number in it ('out of my reach by 22 cm' over 'by 7 cm')."""
    if not reasons:
        return None
    num = lambda r: max([float(x) for x in re.findall(r"[0-9]+(?:\.[0-9]+)?", r)] or [0.0])
    return max(reasons, key=num)


def surface_checker(top, margin=0.005):
    """Points below the work surface."""
    def fn(pts, qs, info):
        return [f"{(top + margin - p[2]) * 100:.0f} cm below the surface" if p[2] < top + margin else None for p in pts]
    return fn


def obstacle_checker(obstacles, hand_r=0.03, ignore=()):
    """obstacles: [{"name", "pos": [x,y,z centre], "r", "h" (half height)}] as vertical cylinders; the hand is a sphere."""
    obs = [o for o in obstacles if o.get("name") not in set(ignore)]

    def fn(pts, qs, info):
        out = []
        for p in pts:
            hit = None
            for o in obs:
                c = np.asarray(o["pos"], float)
                if np.hypot(p[0] - c[0], p[1] - c[1]) < o["r"] + hand_r and c[2] - o["h"] - hand_r < p[2] < c[2] + o["h"] + hand_r:
                    hit = f"the hand would hit the {str(o['name']).replace('_', ' ')}"
                    break
            out.append(hit)
        return out
    return fn


# ---------------------------------------------------------------------------- approach (hand -> first point)
def approach_time(dist, speed):
    """Min-jerk move of `dist` metres whose peak speed is `speed`."""
    return 0.0 if dist < 1e-4 else max(0.4, 1.875 * dist / max(speed, 1e-3))


def approach_points(p_from, p_to, step=CHECK_STEP):
    a, b = np.asarray(p_from, float), np.asarray(p_to, float)
    n = int(math.ceil(np.linalg.norm(b - a) / step))
    return np.array([a + (b - a) * j / n for j in range(1, n)]) if n > 1 else np.zeros((0, 3))


# ---------------------------------------------------------------------------- runtime glue
def _nudged(path, D, j, k, u):
    """Others may move the displayed points of a running path (the nudge pad, the come-here gesture): follow them by
    adding their offset from where they were installed, interpolated between the displayed points around sample k."""
    P0, P = path.get("pts0"), path.get("pts")
    didx = D.get("didx")
    if P0 is None or P is None or didx is None or len(P0) != len(P) or len(P) == 0:
        return 0.0
    P, P0 = np.asarray(P, float), np.asarray(P0, float)
    j = int(np.clip(j, 0, len(P) - 1))
    if j + 1 >= len(P):
        return P[j] - P0[j]
    span = max(1e-9, float(didx[j + 1] - didx[j]))
    f = float(np.clip((k + u - didx[j]) / span, 0.0, 1.0))
    return (1 - f) * (P[j] - P0[j]) + f * (P[j + 1] - P0[j + 1])


def follow_step(rt, dt):
    """One runtime step of a timed trajectory path (rt.path["dense"]). Duck-typed: rt.data.mocap_pos/quat,
    rt.hands_mid, rt.speed_scale, rt.follow_s (reset to 0 by follow_path = a fresh start), rt.mode.
    Gripper / force changes call path["on_event"](kind, value, point) when set, else rt.set_gripper / rt.set_force."""
    path = rt.path
    D = path["dense"]
    d = rt.data
    mid = rt.hands_mid[path["palm"]]
    run = path.get("_run")
    if run is None or rt.follow_s <= 0.0:
        p0, q0 = d.mocap_pos[mid].copy(), qnorm(d.mocap_quat[mid].copy())
        dist = float(np.linalg.norm(D["p"][0] - p0))
        ta = approach_time(dist, path.get("approach_speed", FALLBACK_SPEED))
        if dist < 0.003:
            ta = 0.25 if qangle(q0, D["q"][0]) > 0.05 else 0.0
        run = path["_run"] = {"p0": p0, "q0": q0, "Ta": ta, "ev": 0}
        rt.follow_s = 0.0
    rt.follow_s += dt * float(getattr(rt, "speed_scale", 1.0))
    t = rt.follow_s - run["Ta"]
    T = float(D["t"][-1])
    if t < 0:
        s = float(smoothstep5(rt.follow_s / run["Ta"]))
        p = run["p0"] + s * (D["p"][0] - run["p0"])
        q = slerp(run["q0"], D["q"][0], s)
        i = 0
    else:
        t = min(t, T)
        N = len(D["t"])
        k = int(np.clip(np.searchsorted(D["t"], t, side="right") - 1, 0, max(0, N - 2)))
        if N == 1:
            p, q = D["p"][0], D["q"][0]
        else:
            span = D["t"][k + 1] - D["t"][k]
            u = 0.0 if span < 1e-9 else float(np.clip((t - D["t"][k]) / span, 0.0, 1.0))
            p = (1 - u) * D["p"][k] + u * D["p"][k + 1]
            q = slerp(D["q"][k], D["q"][k + 1], u)
        evs = D["events"]
        while run["ev"] < len(evs) and evs[run["ev"]][0] <= t + 1e-9:
            _, kind, value, pi = evs[run["ev"]]
            run["ev"] += 1
            cb = path.get("on_event")
            if cb is not None:
                cb(kind, value, pi)
            elif kind == "grip" and hasattr(rt, "set_gripper"):
                rt.set_gripper(path["palm"], value)
            elif kind == "force" and hasattr(rt, "set_force"):
                rt.set_force(path["palm"], value)
        i = int(D["di"][k]) if "di" in D else k
        p = p + _nudged(path, D, i, k, u if N > 1 else 0.0)
    d.mocap_pos[mid] = p
    d.mocap_quat[mid] = q
    path["i"] = i
    path["t_now"] = max(0.0, t)
    if t >= T:
        path["i"] = len(path["pts"]) - 1
        rt.mode = "MIRROR"


# ---------------------------------------------------------------------------- language keypoints -> trajectory
REL = ("above", "over", "at", "on", "left_of", "right_of", "in_front_of", "behind", "around", "beside")
VIA = ("over", "around_left", "around_right")
SIDE = {"left_of": (0.0, 1.0), "right_of": (0.0, -1.0), "in_front_of": (-1.0, 0.0), "behind": (1.0, 0.0)}
PASS_KEYS = ("speed", "force", "grip", "tol", "note", "yaw", "tilt", "hold")
# keypoint forms: `where` says which one a keypoint is, and only that form's fields are read (language models tend to
# fill in every field of a schema, e.g. from: "hand" on every point)
FORMS = {"object": ("of", "rel", "dz", "dist", "dx", "dy", "height", "turns", "dir"),
         "pass": ("of", "via", "clearance", "dx", "dy"),
         "point": ("xyz",), "step": ("dxyz",),
         "marker": ("marker", "dz", "dx", "dy", "height"), "region": ("region", "dz", "dx", "dy", "height"),
         "hand": ("from",)}
COMMON_KEYS = PASS_KEYS + ("pattern",)


def _given(v):
    if v is None or v == "" or v is False:
        return False
    if isinstance(v, (list, tuple)):
        return any(isinstance(x, (int, float)) and x != 0 for x in v)
    return True


def keypoint_form(kp):
    """-> (form, the keypoint with only that form's fields). Uses `where` when given, else guesses from what is filled
    in: xyz, dxyz, via + of, of, marker, region, from: hand."""
    where = str(kp.get("where") or "").strip().lower()
    if where not in FORMS:
        where = ("point" if _given(kp.get("xyz")) else "step" if _given(kp.get("dxyz")) else
                 "pass" if _given(kp.get("via")) and _given(kp.get("of")) else "object" if _given(kp.get("of")) else
                 "marker" if _given(kp.get("marker")) else "region" if _given(kp.get("region")) else
                 "hand" if kp.get("from") in ("hand", "start", "here") or kp.get("at") in ("hand", "start") else "object")
    keep = FORMS[where] + COMMON_KEYS
    out = {k: v for k, v in kp.items() if k in keep}
    if where == "hand":
        out["from"] = "hand"
    return where, out


def _words(s):
    return [w for w in re.split(r"[^a-z0-9]+", str(s or "").lower()) if w and w not in ("the", "a", "an", "my", "your")]


def find_object(name, objects):
    """Fuzzy object lookup: 'red cup' / 'red_cup' / 'the cup' -> 'red_cup' (or None)."""
    if not name:
        return None
    k = _key(name)
    if k.startswith("the_"):
        k = k[4:]
    if k in objects:
        return k
    want = set(_words(name))
    best, bscore = None, 0
    for n in objects:
        have = set(_words(n.replace("_", " "))) | set(_words((objects[n] or {}).get("kind", "")))
        score = len(want & have)
        if want and want <= have:
            score += 2
        if score > bscore:
            best, bscore = n, score
    return best


def _top(o):
    return float(o["pos"][2]) + float(o.get("h", 0.0))


def _resolve_kp(kp, i, world, prev, notes):
    """One language keypoint -> list of (position, attrs)."""
    objs = world.get("objects") or {}
    surface = float(world.get("surface", 0.74))
    hand = world.get("hand")
    attrs = {k: kp[k] for k in PASS_KEYS if kp.get(k) is not None}
    dx, dy = float(kp.get("dx") or 0.0), float(kp.get("dy") or 0.0)

    def finish(p):
        p = np.asarray(p, float).copy()
        p[0] += dx
        p[1] += dy
        if kp.get("height") is not None:
            p[2] = surface + float(kp["height"])
        return p

    if kp.get("from") in ("hand", "start", "here") or kp.get("at") in ("hand", "start"):
        if hand is None:
            raise TrajError("I don't know where my hand is")
        return [(finish(hand), attrs)]
    if kp.get("xyz") is not None:
        return [(finish(_vec(kp["xyz"], 3, "xyz")), attrs)]
    if kp.get("dxyz") is not None:
        base = prev if prev is not None else (np.asarray(hand, float) if hand is not None else None)
        if base is None:
            raise TrajError("a relative step (dxyz) needs a point before it")
        return [(finish(base + np.asarray(_vec(kp["dxyz"], 3, "dxyz"))), attrs)]
    marker = kp.get("marker")
    of = kp.get("of")
    if marker is None and isinstance(of, str) and re.fullmatch(r"(marker\s+)?[a-fp]", of.strip().lower()):
        marker = of.strip()[-1]
    if marker is not None:
        m = (world.get("markers") or {}).get(str(marker).strip().upper()[-1:])
        if m is None:
            raise TrajError(f"there is no marker {str(marker).upper()}; tap the spot first")
        dz = float(kp["dz"]) if kp.get("dz") is not None else HOVER
        return [(finish([m[0], m[1], float(m[2]) + dz]), attrs)]
    region = kp.get("region")
    regions = world.get("regions") or {}
    if region is None and isinstance(of, str) and _key(of) in regions and find_object(of, objs) is None:
        region = of
    if region is not None:
        r = regions.get(_key(region))
        if r is None:
            raise TrajError(f"I don't know the region '{region}'")
        dz = float(kp["dz"]) if kp.get("dz") is not None else HOVER
        return [(finish([r[0], r[1], surface + dz]), attrs)]
    if of is None:
        raise TrajError(f"keypoint {i + 1} needs one of: of (an object), xyz, dxyz, marker, region, from: hand")
    name = find_object(of, objs)
    if name is None:
        have = ", ".join(n.replace("_", " ") for n in objs) or "nothing"
        raise TrajError(f"I don't see '{of}' here (I see {have})")
    o = objs[name]
    c, r, top = np.asarray(o["pos"], float), float(o.get("r", 0.03)), _top(o)
    via = kp.get("via")
    if via:
        via = str(via).strip().lower()
        if via not in VIA:
            raise TrajError(f"via must be one of {', '.join(VIA)}")
        clr = float(kp["clearance"]) if kp.get("clearance") is not None else float(world.get("clearance", CLEARANCE))
        if via == "over":
            z = top + clr + float(world.get("carry_drop", 0.0))
            notes.append(f"over the {name.replace('_', ' ')} with {clr * 100:.0f} cm to spare")
            return [(finish([c[0], c[1], z]), attrs)]
        side = 1.0 if via == "around_left" else -1.0
        z = prev[2] if prev is not None else surface + HOVER
        gap = r + float(world.get("hand_r", HAND_R)) + clr
        notes.append(f"around the {name.replace('_', ' ')} on its {'left' if side > 0 else 'right'}")
        return [(finish([c[0], c[1] + side * gap, z]), attrs)]
    rel = str(kp.get("rel") or "above").strip().lower().replace(" ", "_").replace("-", "_")
    rel = {"over": "above", "on": "at", "on_top_of": "at", "in": "at", "next_to": "beside", "to_the_left_of": "left_of",
           "to_the_right_of": "right_of", "front": "in_front_of", "behind_of": "behind", "around_of": "around"}.get(rel, rel)
    if rel == "above":
        dz = float(kp["dz"]) if kp.get("dz") is not None else HOVER
        return [(finish([c[0], c[1], top + dz]), attrs)]
    if rel == "at":
        dz = float(kp["dz"]) if kp.get("dz") is not None else 0.02
        return [(finish([c[0], c[1], top + dz]), attrs)]
    dist = float(kp["dist"]) if kp.get("dist") is not None else 0.06
    if rel == "around":
        dz = float(kp["dz"]) if kp.get("dz") is not None else None
        z = surface + kp["height"] if kp.get("height") is not None else (top + dz if dz is not None else max(surface + HOVER, top + 0.03))
        R = r + dist
        turns = float(np.clip(float(kp.get("turns") or 1.0), 0.25, 4.0))
        ccw = str(kp.get("dir") or "ccw").lower() != "cw"
        a0 = math.atan2((prev[1] - c[1]), (prev[0] - c[0])) if prev is not None else math.pi
        m = max(6, int(round(12 * turns)))
        out = []
        for j in range(m + 1):
            a = a0 + (1 if ccw else -1) * 2 * math.pi * turns * j / m
            out.append((np.array([c[0] + R * math.cos(a) + dx, c[1] + R * math.sin(a) + dy, z]), dict(attrs)))
        notes.append(f"around the {name.replace('_', ' ')}, {R * 100:.0f} cm from its centre")
        return out
    if rel == "beside":
        ref = prev if prev is not None else (np.asarray(hand, float) if hand is not None else c + np.array([-0.2, 0, 0]))
        v = _unit(np.array([ref[0] - c[0], ref[1] - c[1], 0.0]), [-1.0, 0.0, 0.0])
        sx, sy = v[0], v[1]
    elif rel in SIDE:
        sx, sy = SIDE[rel]
    else:
        raise TrajError(f"rel must be one of {', '.join(REL)}")
    gap = r + dist
    z = surface + (float(kp["dz"]) if kp.get("dz") is not None else HOVER)
    return [(finish([c[0] + sx * gap, c[1] + sy * gap, z]), attrs)]


def _seg_circle(a, b, c, R):
    """Horizontal intersection of segment a->b with the circle (c, R): parameter interval [u0, u1] within [0, 1] or None."""
    d = np.array([b[0] - a[0], b[1] - a[1]])
    f = np.array([a[0] - c[0], a[1] - c[1]])
    A = float(d @ d)
    if A < 1e-12:
        return (0.0, 1.0) if float(f @ f) < R * R else None
    B, C = 2 * float(f @ d), float(f @ f) - R * R
    disc = B * B - 4 * A * C
    if disc <= 0:
        return None
    sq = math.sqrt(disc)
    u0, u1 = (-B - sq) / (2 * A), (-B + sq) / (2 * A)
    lo, hi = max(0.0, u0), min(1.0, u1)
    return (lo, hi) if lo < hi else None


def _bridge(a, b, o, R, need, need_at=lambda p: 0.0):
    """Where segment a->b must be raised to clear one object -> (u, points to insert) or None. need_at(p): the height
    every footprint containing p asks for (a bridge point inside two footprints clears both)."""
    c = np.asarray(o["pos"], float)
    if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-4:
        return None           # a vertical move doesn't pass over anything
    iv = _seg_circle(a, b, c, R)
    if iv is None:
        return None
    u0, u1 = iv
    za, zb = a[2] + (b[2] - a[2]) * u0, a[2] + (b[2] - a[2]) * u1
    if min(za, zb) >= need - 0.005:
        return None
    r = float(o.get("r", 0.03))
    onto = lambda p: math.hypot(p[0] - c[0], p[1] - c[1]) < r + 0.01 and p[2] < need
    if onto(a) or onto(b):
        return None           # going down onto it (touch, grasp, place) on purpose
    ins = []
    if u0 > 1e-3:
        E = a + (b - a) * u0
        ins.append(np.array([E[0], E[1], max(E[2], need, need_at(E))]))
    elif a[2] < need:         # starts low inside the footprint: rise first
        ins.append(np.array([a[0], a[1], max(need, need_at(a))]))
    if u1 < 1 - 1e-3:
        X = a + (b - a) * u1
        ins.append(np.array([X[0], X[1], max(X[2], need, need_at(X))]))
    elif b[2] < need:         # ends low inside the footprint: arrive from above
        ins.append(np.array([b[0], b[1], max(need, need_at(b))]))
    ins = [p for p in ins if np.linalg.norm(p - a) > 1e-3 and np.linalg.norm(p - b) > 1e-3]
    return (u0, ins) if ins else None


def lift_over_obstacles(pts, attrs, world, start=None, passes=16):
    """Automatic clearance: a segment that would pass through or too low over an object is raised where it enters and
    leaves the object's footprint (inflated by the hand), to top + clearance. A segment that ends on the object itself
    (going down to touch, grasp or place) is left alone for that object. One object per segment per pass, nearest
    first, so overlapping footprints still give a clean bridge.
    -> (pts, attrs, lifted names, n_prefix) where n_prefix points were added before the first one (from `start`)."""
    objs = world.get("objects") or {}
    clr = float(world.get("clearance", CLEARANCE))
    hand_r = float(world.get("hand_r", HAND_R))
    drop = float(world.get("carry_drop", 0.0))
    seq = ([np.asarray(start, float)] if start is not None else []) + [np.asarray(p, float) for p in pts]
    att = ([{"_start": True}] if start is not None else []) + list(attrs)
    lifted = []
    R = {n: float(o.get("r", 0.03)) + hand_r + 0.02 for n, o in objs.items()}

    def need_at(p):
        return max([_top(o) + clr + drop for n, o in objs.items()
                    if math.hypot(p[0] - o["pos"][0], p[1] - o["pos"][1]) < R[n] + 1e-3] or [0.0])

    for _ in range(passes):
        changed = False
        out_p, out_a = [seq[0]], [att[0]]
        for a, b, aa, ab in zip(seq[:-1], seq[1:], att[:-1], att[1:]):
            best = None
            for name, o in objs.items():
                got = _bridge(a, b, o, R[name], _top(o) + clr + drop, need_at)
                if got is not None and (best is None or got[0] < best[0]):
                    best = (got[0], got[1], name)
            if best is not None:
                changed = True
                if best[2] not in lifted:
                    lifted.append(best[2])
                for P in best[1]:
                    out_p.append(P)
                    out_a.append({"_lift": best[2], "_pat": bool(aa.get("_pat") and ab.get("_pat")),
                                  **({k: v for k, v in ab.items() if k in ("speed", "force", "tol")})})
            out_p.append(b)
            out_a.append(ab)
        seq, att = out_p, out_a
        if not changed:
            break
    if start is not None:
        n_prefix = next((j for j, x in enumerate(att) if j > 0 and not x.get("_lift")), 1) - 1
        seq, att = seq[1:], att[1:]
    else:
        n_prefix = 0
    return seq, att, lifted, n_prefix


def from_keypoints(spec, world):
    """Language keypoints (written by the LLM) -> (normalized trajectory, notes).

    spec:  {"description"|"label", "object"?: anchor, "role"?, "frame"?: "world"|"object", "keypoints": [...],
            "speed"?, "force"?, "grip"?, "tol"?, "clearance"?, "pattern"?}
    keypoint: "where" (object|pass|point|step|marker|region|hand; guessed when absent) picks one of
              {"of": object, "rel": above|at|left_of|right_of|in_front_of|behind|beside|around, "dz"?, "dist"?},
              {"via": over|around_left|around_right, "of": object, "clearance"?}, {"xyz": [x,y,z]}, {"dxyz": [..]},
              {"marker": "A"}, {"region": "left"}, {"from": "hand"}; plus "dx"/"dy" offsets, "height" (above the surface)
              and per point "speed", "force", "grip", "tol", "yaw", "tilt", "hold", "note", "pattern": true.
    world: {"objects": {name: {"pos": [x,y,z centre], "r", "h" (half height), "kind"?}}, "surface": z, "hand": [x,y,z],
            "markers": {id: [x,y,z]}, "regions": {name: [x, y]}, "clearance"?, "hand_r"?, "carry_drop"?}"""
    if not isinstance(spec, dict):
        raise TrajError("the trajectory spec must be an object")
    kps = spec.get("keypoints")
    if not isinstance(kps, (list, tuple)) or not kps:
        raise TrajError("give at least one keypoint")
    if len(kps) > 40:
        raise TrajError("at most 40 keypoints")
    w = dict(world)
    if spec.get("clearance") is not None:
        w["clearance"] = _num(spec["clearance"], 0.0, 0.3, "clearance")
    notes, pts, attrs = [], [], []
    prev = None
    forms = []
    for i, kp in enumerate(kps):
        if not isinstance(kp, dict):
            raise TrajError(f"keypoint {i + 1} must be an object")
        form, kp = keypoint_form(kp)
        forms.append(form)
        if form == "hand" and i > 0:
            raise TrajError(f"keypoint {i + 1}: only the first point can be where the hand is now")
        for p, a in _resolve_kp(kp, i, w, prev, notes):
            pts.append(p)
            attrs.append(dict(a, _pat=bool(kp.get("pattern"))))
            prev = p
    start = w.get("hand") if forms[0] != "hand" else None
    pts2, attrs2, lifted, n_prefix = lift_over_obstacles(pts, attrs, w, start=start)
    flags2 = [bool(a.get("_pat")) for a in attrs2]
    for name in lifted:
        notes.append(f"lifted over the {name.replace('_', ' ')}")
    # top-level defaults
    for k in ("speed", "force", "tol"):
        if spec.get(k) is not None:
            for a in attrs2:
                a.setdefault(k, spec[k])
    if spec.get("grip") is not None and attrs2:
        attrs2[0].setdefault("grip", spec["grip"])
    points = []
    for p, a in zip(pts2, attrs2):
        points.append({"p": [float(x) for x in p], **{k: v for k, v in a.items() if not k.startswith("_")}})
    pattern = normalize_pattern(spec.get("pattern"))
    if pattern and any(flags2):
        on = [i for i, f in enumerate(flags2) if f]
        pattern["span"] = [on[0], on[-1]]
    role = spec.get("role") or "right"
    traj = normalize({"id": spec.get("id") or "t", "role": role, "frame": "world", "source": spec.get("source") or "language",
                      "label": spec.get("label") or spec.get("description") or "", "points": points, "pattern": pattern,
                      "meta": {"keypoints": list(kps), "notes": notes, "lifted": lifted, "prefix": n_prefix}})
    anchor = spec.get("object")
    if anchor and str(spec.get("frame") or "object") != "world":
        name = find_object(anchor, w.get("objects") or {})
        if name is None:
            raise TrajError(f"I don't see '{anchor}' here")
        poses = {n: [o["pos"][0], o["pos"][1], _top(o)] for n, o in (w.get("objects") or {}).items()}
        traj = to_frame(traj, f"object:{name}", poses)
    return traj, notes


# ---------------------------------------------------------------------------- voice edits
def object_poses(world):
    """Frame origins (top centres) of every object in a world dict."""
    return {n: [float(o["pos"][0]), float(o["pos"][1]), _top(o)] for n, o in (world.get("objects") or {}).items()}


def select(traj_world, index=None, near=None, world=None, radius=0.08):
    """Point indices for an edit: index (int or list, negatives from the end) or near ("start", "end", "middle", "all",
    "there" (marker P or the newest marker), a marker letter, an object name, or [x, y, z])."""
    n = len(traj_world["points"])
    if index is not None:
        ids = index if isinstance(index, (list, tuple)) else [index]
        out = []
        for i in ids:
            i = int(_num(i, what="index"))
            i = n + i if i < 0 else i
            if not 0 <= i < n:
                raise TrajError(f"there is no point {i + 1} (the trajectory has {n})")
            out.append(i)
        return sorted(set(out))
    if near is None:
        raise TrajError("say which point: index or near")
    world = world or {}
    P = np.array([p["p"] for p in traj_world["points"]], float)
    if isinstance(near, (list, tuple)):
        target, r = np.asarray(_vec(near, 3, "near"), float), radius
    else:
        t = str(near).strip().lower()
        if t in ("all", "everywhere", "whole", "the whole path", "전체"):
            return list(range(n))
        if t in ("start", "first", "beginning", "처음", "시작"):
            return [0]
        if t in ("end", "last", "finish", "끝", "마지막"):
            return [n - 1]
        if t in ("middle", "center", "centre", "halfway", "중간"):
            return [n // 2]
        markers = world.get("markers") or {}
        mk = None
        if t in ("there", "here", "that spot", "where i pointed", "저기", "여기", "거기"):
            mk = markers.get("P") or (list(markers.values())[-1] if markers else None)
            if mk is None:
                raise TrajError("point at the spot or tap it first, then say 'there'")
        elif re.fullmatch(r"(marker\s+)?[a-fp]", t):
            mk = markers.get(t[-1].upper())
            if mk is None:
                raise TrajError(f"there is no marker {t[-1].upper()}")
        if mk is not None:
            target, r = np.asarray(mk, float), radius
        else:
            objs = world.get("objects") or {}
            name = find_object(t, objs)
            if name is None:
                raise TrajError(f"I don't see '{near}' near the path")
            o = objs[name]
            target, r = np.asarray(o["pos"], float), float(o.get("r", 0.03)) + radius
    dh = np.hypot(P[:, 0] - target[0], P[:, 1] - target[1])
    ids = [int(i) for i in np.nonzero(dh <= r)[0]]
    return ids or [int(np.argmin(dh))]


SPEED_WORDS = {"slower": 0.5, "much slower": 0.3, "slow": 0.5, "faster": 1.5, "much faster": 2.0, "fast": 1.5}


def edit(traj, ids, changes, surface=None, origin=None):
    """Apply changes to points `ids` of `traj` (any frame; deltas are frame independent, `height` uses the surface and
    the frame origin). changes: dx, dy, dz, height, speed, speed_factor, force, grip, tol, yaw, tilt, hold, note, q, p,
    remove. -> new normalized trajectory."""
    if not isinstance(changes, dict):
        raise TrajError("changes must be an object")
    pts = [dict(p) for p in traj["points"]]
    origin = np.zeros(3) if origin is None else np.asarray(origin, float)
    ids = sorted(set(int(i) for i in ids))
    default_v = changes.get("_default_speed") or FALLBACK_SPEED
    for i in ids:
        p = pts[i]
        xyz = np.asarray(p["p"], float)
        if changes.get("p") is not None:
            xyz = np.asarray(_vec(changes["p"], 3, "p"), float)
        for k, ax in (("dx", 0), ("dy", 1), ("dz", 2)):
            if changes.get(k) is not None:
                xyz[ax] += _num(changes[k], -1.0, 1.0, k)
        if changes.get("height") is not None:
            if surface is None:
                raise TrajError("I don't know where the surface is")
            xyz[2] = surface + _num(changes["height"], -0.2, 1.5, "height") - origin[2]
        p["p"] = [float(x) for x in xyz]
        sp = changes.get("speed")
        if "speed" in changes and sp is None:
            p["speed"] = None                     # back to the robot's default speed
        elif isinstance(sp, str):
            f = SPEED_WORDS.get(sp.strip().lower())
            if f is None:
                raise TrajError("speed must be m/s, or 'slower' / 'faster'")
            p["speed"] = (p.get("speed") or default_v) * f
        elif sp is not None:
            p["speed"] = _num(sp, 0.005, 1.5, "speed")
        if changes.get("speed_factor") is not None:
            p["speed"] = (p.get("speed") or default_v) * _num(changes["speed_factor"], 0.05, 5.0, "speed_factor")
        for k in ("force", "grip", "tol", "yaw", "tilt", "hold", "note", "q"):
            if k in changes:
                p[k] = changes[k]
        if any(k in changes for k in ("yaw", "tilt")) and "q" not in changes:
            p["q"] = None        # turning the wrist by yaw / tilt replaces an absolute orientation
    if changes.get("remove"):
        if len(pts) - len(ids) < 1:
            raise TrajError("a trajectory needs at least one point")
        keep = set(range(len(pts))) - set(ids)
        pts = [p for i, p in enumerate(pts) if i in keep]
    return normalize(dict(traj, points=pts, check=None))


def insert(traj, after, p=None):
    """A new point after index `after` (-1 = before the first): at p, else halfway to the next point."""
    pts = [dict(x) for x in traj["points"]]
    n = len(pts)
    after = int(after)
    if p is None:
        if after < 0:
            a = np.asarray(pts[0]["p"], float)
            p = a + (a - np.asarray(pts[1]["p"], float)) * 0.5 if n > 1 else a + [0, 0, 0.05]
        elif after >= n - 1:
            a = np.asarray(pts[-1]["p"], float)
            p = a + (a - np.asarray(pts[-2]["p"], float)) * 0.5 if n > 1 else a + [0, 0, 0.05]
        else:
            p = (np.asarray(pts[after]["p"], float) + np.asarray(pts[after + 1]["p"], float)) / 2
    src = pts[max(0, min(after, n - 1))]
    new = {"p": [float(x) for x in p], "speed": src.get("speed"), "force": src.get("force"), "tol": src.get("tol")}
    pts.insert(after + 1, new)
    return normalize(dict(traj, points=pts, check=None))


# ---------------------------------------------------------------------------- P3 demonstrations
def from_demo(demo, hand=None):
    """A P3 demo (recordings/demos/<name>.json, pps/human/demo.py) or its `trajectory` -> a normalized trajectory.

    The stub stores the hand path as offsets: from the anchor object's top centre (frame object:<anchor>), or, with no
    anchor, from where the hand started (frame "world" in the file). The second case is placed at `hand` (the robot's
    end effector now). `q` is the person's hand orientation, not the gripper's, so it is dropped (the robot keeps its
    approach orientation); `t` (the person's timing) and `grip` (from the pinch) are kept. The pattern describes the
    weave already in the points, so it is not superimposed again."""
    tr = demo.get("trajectory") if isinstance(demo, dict) and "frames" in demo else demo
    if not isinstance(tr, dict):
        raise TrajError("that demonstration has no trajectory")
    pts = [dict(p, q=None) for p in tr.get("points") or []]
    frame = tr.get("frame") or "world"
    if frame == "world":
        if hand is None:
            raise TrajError("a demonstration without an anchor object needs the hand position to start from")
        h = np.asarray(hand, float)
        pts = [dict(p, p=[float(a + b) for a, b in zip(p["p"], h)]) for p in pts]
    pat = tr.get("pattern")
    if isinstance(pat, dict):
        pat = dict(pat, apply=False)
    out = normalize(dict(tr, points=pts, source="demo", frame=frame, pattern=pat, check=None))
    out.setdefault("meta", {})
    out["meta"] = dict(out.get("meta") or {}, demo=(demo.get("name") if isinstance(demo, dict) else None))
    return out


# ---------------------------------------------------------------------------- summaries
def describe(traj_world, world=None):
    """Short per point info for people and the agent: [{i, near, h (m above the surface), speed}]."""
    world = world or {}
    objs = world.get("objects") or {}
    surface = float(world.get("surface", 0.0))
    out = []
    for i, p in enumerate(traj_world["points"]):
        xyz = p["p"]
        near, bd = None, 1e9
        for n, o in objs.items():
            d = math.hypot(xyz[0] - o["pos"][0], xyz[1] - o["pos"][1]) - float(o.get("r", 0.03))
            if d < bd:
                near, bd = n, d
        e = {"i": i, "h": round(xyz[2] - surface, 3)}
        if near is not None and bd < 0.12:
            e["near"] = near
        for k in ("speed", "force", "grip", "yaw", "tilt", "roll", "hold", "note"):
            if p.get(k) is not None:
                e[k] = p[k]
        out.append(e)
    return out
