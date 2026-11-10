"""Rule-based understanding of what the person is doing, from their 3D pose in the robot's world frame.

No learning, no network: joint angles and the hands' paths over a short sliding window.

* Features per frame (`features(J)`): knee angle, hip height above the ankles, torso lean, knee/ankle width ratio
  (knee valgus), and per arm the elbow angle, the wrist height relative to the shoulder, how straight and how far
  forward the arm is, and the wrist position.
* Activities (`ActivityTracker.update`), in priority order:
  - `squat`  knees bend > 30 deg and the hips drop > 8 cm within the window (periodic, counted)
  - `press`  a wrist travels from shoulder height to overhead (periodic, counted)
  - `curl`   an elbow flexes > 60 deg while the wrist stays below the shoulder (periodic, counted)
  - `wave`   a hand above the shoulder swings sideways (>= 3 direction changes, 5-60 cm)
  - `weave`  a hand below the shoulder oscillates across its own travel direction (welding weave, stirring,
             wiping): the path is detrended (least-squares travel line), the residual's main axis gives the
             oscillation, its zero crossings the frequency and its RMS the amplitude
  - `reach`  an arm straight and well in front of the body
  - `point`  co-presence reports an extended pointing arm
  - `still`  hands and hips move < 5 cm/s for a second
  - `moving` anything else
* Repetitions: the activity's signal (hip height, wrist height, elbow flexion, weave offset) is normalised over the
  window; a hysteresis state machine walks top -> down -> bottom -> up -> top and counts a rep at each return to
  the top. The phase is that state. Per-rep stats (depth, valgus, lean, duration...) feed the coach (coach.py).
* Stall: mid-rep, on the way up, the signal stops moving for STALL_S (a lifter stuck under the bar).
"""
import collections
import math

import numpy as np

PERIODIC = ("squat", "press", "curl", "weave", "wave")
WINDOW_S = 4.0         # s of history the periodic detectors look at
OSC_WINDOW_S = 3.0     # s for the hand-oscillation detectors (wave, weave)
SWITCH_S = 0.3         # s a new activity must be seen before it replaces the current one
KEEP_S = 1.5           # s a periodic activity survives without being detected (the pause at the top of a rep)
STILL_V = 0.05         # m/s
STALL_S = 0.7          # s the signal must stop moving mid-rep to count as a stall
TOP, BOTTOM = 0.72, 0.28


def _v(p):
    return np.asarray(p, float)


def angle(a, b, c):
    """Angle at b (deg) between b->a and b->c."""
    u, w = _v(a) - _v(b), _v(c) - _v(b)
    nu, nw = np.linalg.norm(u), np.linalg.norm(w)
    if nu < 1e-6 or nw < 1e-6:
        return 180.0
    return math.degrees(math.acos(float(np.clip(np.dot(u, w) / (nu * nw), -1.0, 1.0))))


def facing(J):
    across = _v(J["l_shoulder"]) - _v(J["r_shoulder"])
    f = np.cross(across, [0.0, 0.0, 1.0])
    f[2] = 0.0
    n = np.linalg.norm(f)
    return f / n if n > 1e-6 else np.array([-1.0, 0.0, 0.0])


def features(J):
    """World joints {name: xyz} -> flat feature dict (None when the shoulders are missing)."""
    if J is None or "l_shoulder" not in J or "r_shoulder" not in J:
        return None
    f = {}
    fw = facing(J)
    sh_mid = (_v(J["l_shoulder"]) + _v(J["r_shoulder"])) / 2
    hips = [_v(J[k]) for k in ("l_hip", "r_hip") if k in J]
    hip_mid = np.mean(hips, axis=0) if hips else sh_mid - np.array([0, 0, 0.5])
    knees = [angle(J[f"{s}_hip"], J[f"{s}_knee"], J[f"{s}_ankle"]) for s in ("l", "r")
             if all(f"{s}_{k}" in J for k in ("hip", "knee", "ankle"))]
    f["knee"] = float(np.mean(knees)) if knees else 180.0
    ank = [_v(J[k]) for k in ("l_ankle", "r_ankle") if k in J]
    f["hip_h"] = float(hip_mid[2] - (np.mean([a[2] for a in ank]) if ank else 0.07))
    trunk = sh_mid - hip_mid
    f["lean"] = math.degrees(math.acos(float(np.clip(trunk[2] / max(np.linalg.norm(trunk), 1e-6), -1, 1))))
    if all(k in J for k in ("l_knee", "r_knee", "l_ankle", "r_ankle")):
        kd = float(np.linalg.norm((_v(J["l_knee"]) - _v(J["r_knee"]))[:2]))
        ad = float(np.linalg.norm((_v(J["l_ankle"]) - _v(J["r_ankle"]))[:2]))
        f["valgus"] = kd / ad if ad > 0.05 else 1.0
    else:
        f["valgus"] = 1.0
    f["hip"] = hip_mid
    for s in ("l", "r"):
        if not all(f"{s}_{k}" in J for k in ("shoulder", "elbow", "wrist")):
            continue
        sh, el, wr = _v(J[f"{s}_shoulder"]), _v(J[f"{s}_elbow"]), _v(J[f"{s}_wrist"])
        arm = np.linalg.norm(el - sh) + np.linalg.norm(wr - el)
        f[f"{s}_elbow"] = angle(sh, el, wr)
        f[f"{s}_wz"] = float(wr[2] - sh[2])
        f[f"{s}_ext"] = float(np.linalg.norm(wr - sh) / arm) if arm > 1e-3 else 0.0
        f[f"{s}_fwd"] = float(np.dot(wr - sh, fw))
        f[f"{s}_elfwd"] = float(np.dot(el - sh, fw))
        f[f"{s}_w"] = wr
    f["head_z"] = float(_v(J["head"])[2] - sh_mid[2]) if "head" in J else 0.25
    return f


def oscillation(ts, ps):
    """Hand path (times, Nx3) -> {amp (m, peak), freq (Hz), crossings, axis (unit xyz), travel (m/s), dir (unit xyz)}.
    The path is detrended by its least-squares travel line; the residual's principal axis is the oscillation."""
    ts, ps = np.asarray(ts, float), np.asarray(ps, float)
    if len(ts) < 8 or ts[-1] - ts[0] < 0.8:
        return None
    t = ts - ts.mean()
    A = np.column_stack([np.ones_like(t), t])
    coef, *_ = np.linalg.lstsq(A, ps, rcond=None)
    vel = coef[1]
    res = ps - A @ coef
    cov = res.T @ res / len(res)
    w, V = np.linalg.eigh(cov)
    axis = V[:, -1]
    x = res @ axis
    amp = float(np.sqrt(2.0) * np.sqrt(max(w[-1], 0.0)))
    # zero crossings with a dead band (noise below 15% of the amplitude does not count)
    band = 0.15 * amp
    sgn, crossings = 0, 0
    for v in x:
        s = 1 if v > band else (-1 if v < -band else 0)
        if s and sgn and s != sgn:
            crossings += 1
        if s:
            sgn = s
    dur = float(ts[-1] - ts[0])
    sp = float(np.linalg.norm(vel))
    return {"amp": amp, "freq": crossings / (2.0 * dur), "crossings": crossings, "axis": axis,
            "travel": sp, "dir": vel / sp if sp > 1e-6 else np.zeros(3), "x": x}


class RepCounter:
    """Hysteresis rep counter on a normalised signal (1 = top of the movement)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase, self.reps, self.t_top, self.durations = None, 0, None, []
        self.cur = {}

    def update(self, t, s):
        """s in [0, 1] (or None). -> 'rep' when a repetition just completed, else None."""
        if s is None:
            return None
        ev = None
        if self.phase is None:
            self.phase = "top" if s >= 0.5 else "bottom"
            self.t_top = t if self.phase == "top" else None      # joined mid-rep: the first duration is unknown
            return None
        if self.phase == "top" and s < TOP:
            self.phase = "down"
        elif self.phase == "down":
            if s <= BOTTOM:
                self.phase = "bottom"
            elif s >= TOP:
                self.phase = "top"
        elif self.phase == "bottom" and s > BOTTOM:
            self.phase = "up"
        elif self.phase == "up":
            if s >= TOP:
                self.phase = "top"
                self.reps += 1
                if self.t_top is not None:
                    self.durations.append(t - self.t_top)
                self.t_top = t
                ev = "rep"
            elif s <= BOTTOM:
                self.phase = "bottom"
        return ev


class ActivityTracker:
    def __init__(self):
        self.hist = collections.deque()
        self.activity, self.cand, self.cand_t, self.seen_t = "none", None, 0.0, 0.0
        self.side = "r"
        self.counter = RepCounter()
        self.rep_stats, self.cur_stats = [], {}
        self.osc = None
        self.stalled, self.stall_t, self.stall_ref = False, None, None
        self.events = []
        self.signal = None
        self._span = None

    # -------------------------------------------------------------- window helpers
    def _win(self, now, sec):
        return [h for h in self.hist if now - h[0] <= sec]

    @staticmethod
    def _rng(rows, key):
        vals = [r[1][key] for r in rows if key in r[1]]
        return (min(vals), max(vals)) if vals else (0.0, 0.0)

    @staticmethod
    def _swings(rows, key):
        """Half-cycles of a signal in the window: moves between its lower and upper third (a single step, e.g. the
        arm dropping once, is 1; one rep down and up is 2)."""
        vals = [r[1][key] for r in rows if key in r[1]]
        if len(vals) < 4:
            return 0
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-6:
            return 0
        n, state = 0, None
        for v in vals:
            s = (v - lo) / (hi - lo)
            z = 1 if s > 0.7 else (-1 if s < 0.3 else 0)
            if z and state is not None and z != state:
                n += 1
            if z:
                state = z
        return n

    def _detect(self, now, extra):
        rows = self._win(now, WINDOW_S)
        if len(rows) < 6:
            return None, None
        f = rows[-1][1]
        klo, khi = self._rng(rows, "knee")
        hlo, hhi = self._rng(rows, "hip_h")
        if khi - klo > 30 and hhi - hlo > 0.08 and self._swings(rows, "hip_h") >= 2:
            return "squat", None
        best = None
        for s in ("l", "r"):
            if f"{s}_wz" not in f:
                continue
            zlo, zhi = self._rng(rows, f"{s}_wz")
            elo, ehi = self._rng(rows, f"{s}_elbow")
            if zhi > f["head_z"] - 0.02 and zlo < 0.12 and zhi - zlo > 0.25 and self._swings(rows, f"{s}_wz") >= 2:
                best = best or ("press", s)
            elif ehi - elo > 60 and zhi < 0.12 and best is None and self._swings(rows, f"{s}_elbow") >= 2:
                best = ("curl", s)
        if best:
            return best
        for sec in (OSC_WINDOW_S, 2.0, 1.3):     # shorter windows too: the hand may have just moved into place
            osc_rows = self._win(now, sec)
            for s in self._active_sides(osc_rows):
                ts = [r[0] for r in osc_rows if f"{s}_w" in r[1]]
                ps = [r[1][f"{s}_w"] for r in osc_rows if f"{s}_w" in r[1]]
                o = oscillation(ts, ps)
                if o is None:
                    continue
                above = np.median([r[1][f"{s}_wz"] for r in osc_rows if f"{s}_wz" in r[1]]) > 0.05
                if above and 0.05 <= o["amp"] <= 0.6 and o["crossings"] >= 3:
                    self.osc = dict(o, side=s)
                    return "wave", s
                if not above and 0.004 <= o["amp"] <= 0.08 and o["crossings"] >= 4 and 0.6 <= o["freq"] <= 6.0:
                    self.osc = dict(o, side=s)
                    return "weave", s
        if extra.get("pointing"):
            return "point", None
        for s in ("l", "r"):
            if f.get(f"{s}_ext", 0) > 0.85 and f.get(f"{s}_fwd", 0) > 0.25 and f.get(f"{s}_wz", 0) < 0.3:
                return "reach", s
        if extra.get("speed", 1.0) < STILL_V and self._still(now):
            return "still", None
        return "moving", None

    def _active_sides(self, rows):
        """Arms ordered by how much the wrist moved in the window (the working hand first)."""
        out = []
        for s in ("l", "r"):
            ps = [r[1][f"{s}_w"] for r in rows if f"{s}_w" in r[1]]
            if len(ps) > 4:
                out.append((float(np.ptp(np.asarray(ps), axis=0).max()), s))
        return [s for _, s in sorted(out, reverse=True)]

    def _still(self, now):
        rows = self._win(now, 1.0)
        if len(rows) < 4 or rows[-1][0] - rows[0][0] < 0.7:
            return False
        for key in ("l_w", "r_w", "hip"):
            ps = [r[1][key] for r in rows if key in r[1]]
            if len(ps) > 2 and float(np.ptp(np.asarray(ps), axis=0).max()) > 0.06:
                return False
        return True

    # -------------------------------------------------------------- signal for reps
    @staticmethod
    def _key(act, side):
        """-> (feature key, flip (1 = top is the small value), minimum range) of an activity's rep signal."""
        return {"squat": ("hip_h", False, 0.07), "press": (f"{side}_wz", False, 0.2),
                "curl": (f"{side}_elbow", True, 50)}.get(act, (None, False, 0))

    def _norm_s(self):
        """Seconds of history that set the signal's top and bottom: about 1.6 reps (stale deep reps must not keep
        later shallow ones from reaching the bottom)."""
        d = [x for x in self.counter.durations[-4:] if x]
        return float(np.clip(1.6 * (np.median(d) if d else 2.5), 3.0, 6.0))

    def _signal(self, now, act, side):
        """-> (raw value, normalised 0..1 with 1 = top) for the current activity, or (None, None)."""
        rows = self._win(now, self._norm_s())
        if not rows:
            return None, None
        key, flip, need = self._key(act, side)
        if key is None:
            if act not in ("weave", "wave") or self.osc is None or len(self.osc["x"]) < 3:
                return None, None
            x = self.osc["x"]
            lo, hi = float(np.min(x)), float(np.max(x))
            if hi - lo < 1e-4:
                return None, None
            return float(x[-1]), (float(x[-1]) - lo) / (hi - lo)
        vals = [r[1][key] for r in rows if key in r[1]]
        if not vals:
            return None, None
        lo, hi, v = min(vals), max(vals), vals[-1]
        if self.counter.phase in ("bottom", "up") and self._span:
            # on the way back up the range may only grow: a lifter stuck mid-rep must not drift into "top" as the
            # bottom of the rep ages out of the window
            lo, hi = min(lo, self._span[0]), max(hi, self._span[1])
        self._span = (lo, hi)
        if hi - lo < need:
            return v, None
        s = (v - lo) / (hi - lo)
        return v, (1.0 - s) if flip else s

    # -------------------------------------------------------------- per-rep stats (for the coach)
    def _track_stats(self, f, s):
        c = self.cur_stats
        c["knee_min"] = min(c.get("knee_min", 180.0), f["knee"])
        c["lean_max"] = max(c.get("lean_max", 0.0), f["lean"])
        c["hip_min"] = min(c.get("hip_min", 9.0), f["hip_h"])
        c["hip_max"] = max(c.get("hip_max", 0.0), f["hip_h"])
        if s is not None and s < 0.35:
            c["valgus_bottom"] = min(c.get("valgus_bottom", 9.0), f["valgus"])
        sd = self.side
        if f"{sd}_elbow" in f:
            c["elbow_min"] = min(c.get("elbow_min", 180.0), f[f"{sd}_elbow"])
            c["elbow_max"] = max(c.get("elbow_max", 0.0), f[f"{sd}_elbow"])
            c["wz_max"] = max(c.get("wz_max", -9.0), f[f"{sd}_wz"])
            ef = f[f"{sd}_elfwd"]
            c["elfwd_min"], c["elfwd_max"] = min(c.get("elfwd_min", 9.0), ef), max(c.get("elfwd_max", -9.0), ef)

    # -------------------------------------------------------------- main
    def update(self, now, J, extra=None):
        """Feed one frame (world joints, grounded). extra: {speed, pointing}. Returns the events of this frame:
        [('activity', name) | ('rep', stats) | ('stall', None)]."""
        extra = extra or {}
        self.events = []
        f = features(J)
        if f is None:
            self.hist.clear()
            if self.activity != "none":
                self.activity = "none"
                self.events.append(("activity", "none"))
            self.counter.phase = None
            return self.events
        self.hist.append((now, f))
        while self.hist and now - self.hist[0][0] > WINDOW_S + 2.0:
            self.hist.popleft()
        cand, side = self._detect(now, extra)
        if cand is not None:
            if cand == self.activity:
                self.seen_t = now
                if side:
                    self.side = side
            elif cand != self.cand:
                self.cand, self.cand_t = cand, now
            elif now - self.cand_t >= SWITCH_S:
                keep = self.activity in PERIODIC and now - self.seen_t < KEEP_S and cand in ("moving", "still", "reach", "point")
                if not keep:
                    self._switch(cand, side, now)
            if cand == self.activity:
                self.cand = None
        raw, s = self._signal(now, self.activity, self.side)
        self.signal = s
        if self.activity in PERIODIC:
            self._count(now, f, s)
            self._stall(now, raw, s)
        else:
            self.stalled = False
        return self.events

    def _count(self, t, f, s):
        self._track_stats(f, s)
        if self.counter.update(t, s) == "rep":
            st = dict(self.cur_stats, rep=self.counter.reps, activity=self.activity, side=self.side,
                      duration=self.counter.durations[-1] if self.counter.durations else None)
            if self.activity in ("weave", "wave") and self.osc is not None:
                st.update(amp=self.osc["amp"], freq=self.osc["freq"], travel=self.osc["travel"])
            self.rep_stats.append(st)
            del self.rep_stats[:-50]
            self.cur_stats = {}
            self.events.append(("rep", st))

    def _switch(self, act, side, now):
        prev = self.activity
        self.activity, self.seen_t, self.cand = act, now, None
        if side:
            self.side = side
        if act in PERIODIC and act != prev:
            self.counter.reset()
            self._span = None
            self.rep_stats, self.cur_stats = [], {}
            key, flip, need = self._key(act, self.side)
            rows = self._win(now, WINDOW_S)[:-1]       # the activity was recognised after a full swing: count it too
            vals = [r[1][key] for r in rows if key in r[1]] if key else []
            if vals and max(vals) - min(vals) >= need:
                lo, hi = min(vals), max(vals)
                for t, f in rows:
                    if key in f:
                        s = (f[key] - lo) / (hi - lo)
                        self._count(t, f, 1.0 - s if flip else s)
        self.stalled, self.stall_t = False, None
        self.events.append(("activity", act))

    def _stall(self, now, raw, s):
        """Mid-rep on the way up (or anywhere below the top), the signal stops: the person is stuck."""
        if s is None or raw is None or self.counter.phase not in ("up", "bottom") or s > 0.8:
            self.stalled, self.stall_t = False, None
            return
        if self.stall_t is None or abs(raw - self.stall_ref) > self._stall_tol():
            self.stall_t, self.stall_ref = now, raw
            if self.stalled:
                self.stalled = False
            return
        if not self.stalled and now - self.stall_t >= STALL_S and s > 0.05:
            self.stalled = True
            self.events.append(("stall", None))

    def _stall_tol(self):
        return {"squat": 0.015, "press": 0.015, "curl": 4.0}.get(self.activity, 0.01)

    def reset_reps(self):
        """A fresh start (a new coaching set, a new demonstration): forget the window, the activity and the count."""
        self.counter.reset()
        self.rep_stats, self.cur_stats, self._span = [], {}, None
        self.hist.clear()
        self.activity, self.cand, self.stalled = "none", None, False

    # -------------------------------------------------------------- readout
    @property
    def reps(self):
        return self.counter.reps if self.activity in PERIODIC else 0

    @property
    def phase(self):
        if self.activity in PERIODIC:
            return self.counter.phase
        return None

    def fatigue(self):
        """0..1: how much slower the latest reps are than the first ones (a rough tiredness cue)."""
        d = [x for x in self.counter.durations if x]
        if len(d) < 4:
            return 0.0
        first = float(np.median(d[:3]))
        return float(np.clip(d[-1] / first - 1.0, 0.0, 1.0)) if first > 1e-3 else 0.0

    def weave(self):
        if self.activity != "weave" or self.osc is None:
            return None
        o = self.osc
        return {"amp": round(o["amp"], 4), "freq": round(o["freq"], 2), "travel": round(o["travel"], 4),
                "axis": [round(float(v), 3) for v in o["axis"]], "dir": [round(float(v), 3) for v in o["dir"]], "side": o["side"]}
