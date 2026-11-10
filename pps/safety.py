"""Safety around people (v1 P5): speed and separation monitoring in the spirit of ISO/TS 15066. Pure (numpy + an
optional MuJoCo model for the robot's geometry), so every rule here is unit-tested in tests/test_safety.py.

NOT A CERTIFIED SAFETY SYSTEM. It runs on a webcam pose estimate in a simulator. It is a faithful, honest model of
how speed and separation monitoring behaves, so the robot's behaviour around people can be designed and tested here.
It is not a safety function you may rely on near a real robot (docs/safety.md).

Geometry
  * The person is 17 capsules built from the 33 MediaPipe pose landmarks in the robot's world frame (state.human.pose):
    head, neck, torso (spine, shoulder line, hip line), upper arms, forearms, hands, thighs, shins, feet. Each capsule
    belongs to a body region: head | torso | arm | forearm | hand | leg.
  * The robot is one capsule (or sphere) per geom of its bodies: collision geoms where a body has them, its visual
    geoms (mesh bounding boxes) where it doesn't. Cached per model, so it works for every profile (arms, two-arm rigs,
    humanoids, mobile bases, legged bodies). The end-effector links (the site's body, two parents, all children) and
    an object the robot holds are marked `ee`.
  * Distances are capsule surface to capsule surface (segment-segment closest points minus both radii).

Speed and separation (per robot-link / person-part pair, the most restrictive pair wins)
    S_p = v_h * (T_r + T_s) + v_r * T_r + B + C          protective separation distance
    B   = v_r * T_s / 2                                  robot braking distance (constant deceleration)
    C   = d_stop (body), d_stop_critical (head, neck, torso), d_touch (hand/forearm vs the end effector while
          cooperating: assist or mirror mode, or a handover)
  v_h is the person's measured speed toward the robot (capped at v_human_max), v_r the measured speed of that robot link.
    scale = 1                                                  d >= d_slow + (S_p - C)
          = min_scale + (1 - min_scale) * (d - S_p) / band     S_p <= d < d_slow + (S_p - C)
          = 0  (protective stop)                               d < S_p
  so the slow band moves outward by the same distance the robot and the person could cover while it reacts and
  brakes. Hysteresis: a stop is released only after the person is S_p + hysteresis away (with the speed the robot
  had when it stopped) for resume_s; the scale then ramps back up at ramp_up per second. Slowing down is immediate.

Contact policy
  * Head, neck or torso closer than d_stop_critical: immediate stop, in every mode.
  * Cooperating (assist or mirror mode, or a handover): the person's hands and forearms may come up to the end effector. The
    robot moves at most coop_scale there. Actual contact is allowed only when the end-effector force is measured
    (P0 `ee_force`) and stays under f_contact_max; above it -> a latched stop (needs a confirmation to resume).
    Without a force reading contact is not allowed: the robot stops at d_touch.
  * Every other pair: the robot never touches you (stop at S_p with C = d_stop).
"""
import math
from dataclasses import asdict, dataclass, fields

import numpy as np

# ---------------------------------------------------------------------- configuration


@dataclass
class Config:
    d_slow: float = 0.50            # m: start slowing down inside this separation (plus the speed-dependent margin)
    d_stop: float = 0.12            # m: C, protective stop distance for the body (arms, legs, hands when not cooperating)
    d_stop_critical: float = 0.15   # m: C for the head, neck and torso (stop sooner, in every mode)
    d_touch: float = 0.0            # m: C for hands/forearms vs the end effector while cooperating (0 = touching)
    min_scale: float = 0.2          # speed scale at the edge of the stop distance
    coop_scale: float = 0.3         # speed cap while a cooperating hand is inside d_slow of the end effector
    t_react: float = 0.10           # s: T_r, sensing + reaction time
    t_stop: float = 0.20            # s: T_s, time the robot takes to brake to a standstill
    v_human_max: float = 1.6        # m/s: cap on the measured human speed (ISO 13855 walking speed; pose jitter)
    hysteresis: float = 0.05        # m: a stop is released this much beyond the stop distance
    resume_s: float = 1.0           # s the person must stay clear before motion resumes
    ramp_up: float = 1.0            # speed scale per second when speeding back up (slowing down is immediate)
    f_contact_max: float = 40.0     # N: end-effector force cap for allowed contact (ISO/TS 15066 hand limits are higher)
    lost_hold_s: float = 1.5        # s: a camera that loses the person while close keeps the last pose this long

    # floors: config may tune safety, never switch it off
    LIMITS = {"d_slow": (0.20, 3.0), "d_stop": (0.05, 1.0), "d_stop_critical": (0.10, 1.5), "d_touch": (0.0, 0.2),
              "min_scale": (0.05, 1.0), "coop_scale": (0.05, 0.5), "t_react": (0.02, 1.0), "t_stop": (0.02, 2.0),
              "v_human_max": (0.5, 3.0), "hysteresis": (0.01, 0.5), "resume_s": (0.2, 10.0), "ramp_up": (0.1, 5.0),
              "f_contact_max": (5.0, 140.0), "lost_hold_s": (0.0, 5.0)}

    @classmethod
    def from_dict(cls, d=None):
        """Overrides (server config, robot profile) clamped to the floors above. Unknown keys (e.g. "enabled") are
        ignored: there is no switch that turns safety off."""
        c = cls()
        for f in fields(cls):
            if d and f.name in d:
                try:
                    v = float(d[f.name])
                except (TypeError, ValueError):
                    continue
                lo, hi = cls.LIMITS[f.name]
                setattr(c, f.name, float(np.clip(v, lo, hi)))
        c.d_stop_critical = max(c.d_stop_critical, c.d_stop)
        c.d_slow = max(c.d_slow, c.d_stop_critical + 0.1)
        return c

    def public(self):
        return {k: round(v, 3) for k, v in asdict(self).items()}


# ---------------------------------------------------------------------- the person as capsules
CRITICAL = ("head", "torso")
SEVERITY = {"critical": 0, "force": 1, "body": 2, "no_force": 3, "coop": 4}
COOP = ("hand", "forearm")
# MediaPipe Pose indices
NOSE, L_EAR, R_EAR, L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 0, 7, 8, 11, 12, 13, 14, 15, 16
L_PINKY, R_PINKY, L_INDEX, R_INDEX = 17, 18, 19, 20
L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANK, R_ANK, L_HEEL, R_HEEL, L_FOOT, R_FOOT = 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
RADIUS = {"head": 0.11, "neck": 0.06, "spine": None, "shoulders": 0.07, "hips": 0.09, "upper_arm": 0.05,
          "forearm": 0.045, "hand": 0.05, "thigh": 0.075, "shin": 0.055, "foot": 0.04}
PART_WORD = {"head": "head", "torso": "body", "arm": "arm", "forearm": "arm", "hand": "hand", "leg": "leg"}


def _p(pose, i):
    if pose is None or i >= len(pose) or pose[i] is None:
        return None
    return np.asarray(pose[i], float)


def human_capsules(pose):
    """33 world landmarks ([x, y, z] or None) -> [{"part", "region", "a", "b", "r"}]. Missing landmarks drop only the
    capsules that need them."""
    out = []

    def add(part, region, a, b, r):
        if a is not None and b is not None:
            out.append({"part": part, "region": region, "a": np.asarray(a, float), "b": np.asarray(b, float), "r": float(r)})

    ls, rs, lh, rh = _p(pose, L_SH), _p(pose, R_SH), _p(pose, L_HIP), _p(pose, R_HIP)
    sh = None if ls is None or rs is None else (ls + rs) / 2
    hp = None if lh is None or rh is None else (lh + rh) / 2
    le, re, nose = _p(pose, L_EAR), _p(pose, R_EAR), _p(pose, NOSE)
    head = (le + re) / 2 if le is not None and re is not None else nose
    if head is not None:
        up = np.array([0.0, 0.0, 1.0])
        add("head", "head", head - 0.02 * up, head + 0.06 * up, RADIUS["head"])
        add("neck", "head", sh, head - 0.04 * up, RADIUS["neck"])
    if sh is not None and hp is not None:
        add("torso", "torso", sh, hp, float(np.clip(0.35 * np.linalg.norm(ls - rs), 0.10, 0.16)))   # chest depth; width is the shoulder line
    add("shoulders", "torso", ls, rs, RADIUS["shoulders"])
    add("hips", "torso", lh, rh, RADIUS["hips"])
    for s, SH, EL, WR, PK, IX, HIP, KN, AN, HE, FT in (("left", L_SH, L_EL, L_WR, L_PINKY, L_INDEX, L_HIP, L_KNEE, L_ANK, L_HEEL, L_FOOT),
                                                       ("right", R_SH, R_EL, R_WR, R_PINKY, R_INDEX, R_HIP, R_KNEE, R_ANK, R_HEEL, R_FOOT)):
        shp, el, wr = _p(pose, SH), _p(pose, EL), _p(pose, WR)
        add(f"{s}_upper_arm", "arm", shp, el, RADIUS["upper_arm"])
        add(f"{s}_forearm", "forearm", el, wr, RADIUS["forearm"])
        tips = [t for t in (_p(pose, PK), _p(pose, IX)) if t is not None]
        if wr is not None:
            tip = np.mean(tips, axis=0) if tips else (wr + 0.08 * _unit(wr - el) if el is not None else wr)
            add(f"{s}_hand", "hand", wr, wr + 0.6 * (tip - wr), RADIUS["hand"])
        add(f"{s}_thigh", "leg", _p(pose, HIP), _p(pose, KN), RADIUS["thigh"])
        add(f"{s}_shin", "leg", _p(pose, KN), _p(pose, AN), RADIUS["shin"])
        add(f"{s}_foot", "leg", _p(pose, HE), _p(pose, FT), RADIUS["foot"])
    return out


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v * 0.0


# ---------------------------------------------------------------------- distance math
def seg_seg(p0, p1, q0, q1):
    """Closest points between segments p0-p1 and q0-q1, vectorised over leading axes (..., 3).
    -> (distance (...), closest point on p (..., 3), closest point on q (..., 3)). Ericson, Real-Time Collision
    Detection 5.1.9, with the degenerate (point) cases handled."""
    p0, p1, q0, q1 = (np.asarray(x, float) for x in (p0, p1, q0, q1))
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a = np.einsum("...i,...i", d1, d1)
    e = np.einsum("...i,...i", d2, d2)
    f = np.einsum("...i,...i", d2, r)
    c = np.einsum("...i,...i", d1, r)
    b = np.einsum("...i,...i", d1, d2)
    eps = 1e-12
    denom = a * e - b * b
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(denom > eps, np.clip((b * f - c * e) / np.where(denom > eps, denom, 1.0), 0.0, 1.0), 0.0)
        t = np.where(e > eps, (b * s + f) / np.where(e > eps, e, 1.0), 0.0)
        # t outside [0, 1]: clamp and recompute s
        s = np.where(t < 0.0, np.where(a > eps, np.clip(-c / np.where(a > eps, a, 1.0), 0.0, 1.0), 0.0), s)
        s = np.where(t > 1.0, np.where(a > eps, np.clip((b - c) / np.where(a > eps, a, 1.0), 0.0, 1.0), 0.0), s)
        t = np.clip(t, 0.0, 1.0)
    cp = p0 + d1 * s[..., None]
    cq = q0 + d2 * t[..., None]
    return np.linalg.norm(cp - cq, axis=-1), cp, cq


def capsule_distance(a0, a1, ra, b0, b1, rb):
    """Surface distance between two capsules (negative = overlapping)."""
    d, _, _ = seg_seg(a0, a1, b0, b1)
    return d - ra - rb


def _pairs(robot, human):
    """All robot capsules x all person capsules -> (surface distance (n, m), closest points on the robot (n, m, 3),
    closest points on the person (n, m, 3))."""
    ha = np.array([h["a"] for h in human])
    hb = np.array([h["b"] for h in human])
    hr = np.array([h["r"] for h in human])
    d, cp, cq = seg_seg(robot["a"][:, None, :], robot["b"][:, None, :], ha[None, :, :], hb[None, :, :])
    return d - robot["r"][:, None] - hr[None, :], cp, cq


# ---------------------------------------------------------------------- the robot as capsules
def _geom_capsule(m, g):
    """Geom g -> (local a, local b, r) in the geom frame, or None for planes / height fields."""
    import mujoco
    t, s = int(m.geom_type[g]), np.asarray(m.geom_size[g], float)
    z = np.zeros(3)
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return z, z, float(s[0])
    if t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        h = np.array([0.0, 0.0, s[1]])
        return -h, h, float(s[0])
    if t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        return _box_capsule(z, s, ellipsoid=True)
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return _box_capsule(z, s)
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        aabb = np.asarray(m.geom_aabb[g], float)
        return _box_capsule(aabb[:3], aabb[3:])
    return None


def _box_capsule(center, half, ellipsoid=False):
    """A capsule enclosing an axis-aligned box (or ellipsoid) with half sizes `half` around `center` (geom frame)."""
    half = np.abs(np.asarray(half, float))
    k = int(np.argmax(half))
    others = [half[i] for i in range(3) if i != k]
    r = max(others) if ellipsoid else float(math.hypot(*others))
    ax = np.zeros(3)
    ax[k] = max(0.0, half[k] - (r if ellipsoid else 0.0))
    return center - ax, center + ax, float(max(r, 1e-3))


class RobotGeometry:
    """Capsules for every robot body of a MuJoCo model; build once per model, then .capsules(data) per frame.
    robot_bodies: body ids of the robot. ee_sites: {palm name: site id} (end effectors)."""

    def __init__(self, model, robot_bodies, ee_sites=None):
        m = model
        self.model = m
        bodies = set(int(b) for b in robot_bodies)
        coll = {b: [] for b in bodies}
        vis = {b: [] for b in bodies}
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            if b not in bodies:
                continue
            (coll if (m.geom_contype[g] or m.geom_conaffinity[g]) else vis)[b].append(g)
        # end-effector links: the site's body, three parents (wrist and forearm), and everything below the site's body
        ee_of = {}
        for palm, sid in (ee_sites or {}).items():
            b0 = int(m.site_bodyid[sid])
            chain, b = [], b0
            while b > 0 and b in bodies and len(chain) < 4:
                chain.append(b)
                b = int(m.body_parentid[b])
            for c in bodies:
                p = c
                while p > 0:
                    if p == b0:
                        chain.append(c)
                        break
                    p = int(m.body_parentid[p])
            for c in chain:
                ee_of.setdefault(c, palm)
        # bodies on the way from the root to an end effector: "arm"
        arm = set()
        for sid in (ee_sites or {}).values():
            b = int(m.site_bodyid[sid])
            while b > 0 and b in bodies:
                arm.add(b)
                b = int(m.body_parentid[b])
        rows = []
        import mujoco
        for b in sorted(bodies):
            for g in (coll[b] or vis[b]):
                cap = _geom_capsule(m, g)
                if cap is None:
                    continue
                a, bb, r = cap
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or f"body{b}"
                rows.append((g, a, bb, r, b, name, ee_of.get(b)))
        self.geoms = np.array([r[0] for r in rows], int)
        self.la = np.array([r[1] for r in rows], float).reshape(-1, 3)
        self.lb = np.array([r[2] for r in rows], float).reshape(-1, 3)
        self.r = np.array([r[3] for r in rows], float)
        self.body = np.array([r[4] for r in rows], int)
        self.names = [r[5] for r in rows]
        self.ee = [r[6] for r in rows]                 # palm name or None
        self.kind = ["ee" if r[6] else ("arm" if r[4] in arm else "body") for r in rows]

    def capsules(self, data, held=()):
        """-> dict of arrays a (n, 3), b (n, 3), r (n,), plus names / ee / kind lists. held: [(palm, center, radius)]
        objects in the robot's hands (part of its end effector while carried)."""
        if len(self.geoms):
            R = np.asarray(data.geom_xmat, float)[self.geoms].reshape(-1, 3, 3)
            P = np.asarray(data.geom_xpos, float)[self.geoms]
            a = P + np.einsum("nij,nj->ni", R, self.la)
            b = P + np.einsum("nij,nj->ni", R, self.lb)
        else:
            a = b = np.zeros((0, 3))
        r, names, ee, kind = self.r, list(self.names), list(self.ee), list(self.kind)
        if held:
            c = np.array([h[1] for h in held], float).reshape(-1, 3)
            a, b = np.vstack([a, c]), np.vstack([b, c])
            r = np.concatenate([r, [float(h[2]) for h in held]])
            names += ["held object"] * len(held)
            ee += [h[0] for h in held]
            kind += ["ee"] * len(held)
        return {"a": a, "b": b, "r": np.asarray(r, float), "names": names, "ee": ee, "kind": kind}


# ---------------------------------------------------------------------- separation -> speed scale
def protective_distance(cfg, v_robot, v_human, c):
    """S_p = v_h (T_r + T_s) + v_r T_r + v_r T_s / 2 + C."""
    v_h = min(max(float(v_human), 0.0), cfg.v_human_max)
    v_r = max(float(v_robot), 0.0)
    return v_h * (cfg.t_react + cfg.t_stop) + v_r * cfg.t_react + 0.5 * v_r * cfg.t_stop + c


def scale_for(cfg, d, s_p, c, cap=1.0):
    """Speed scale for one pair at surface distance d with protective distance s_p (C = c)."""
    if d < s_p:
        return 0.0
    slow = cfg.d_slow + (s_p - c)
    if d >= slow:
        return 1.0 if cap >= 1.0 else cap
    k = (d - s_p) / max(slow - s_p, 1e-6)
    return min(cap, cfg.min_scale + (1.0 - cfg.min_scale) * k)


def pair_rule(cfg, region, robot_kind, coop, force):
    """Contact policy for one (person region, robot link) pair.
    -> (C, speed cap, contact allowed, why). force: measured end-effector force magnitude in N, or None."""
    if region in CRITICAL:
        return cfg.d_stop_critical, 1.0, False, "critical"
    if coop and region in COOP and robot_kind == "ee":
        if force is None:
            return cfg.d_touch, cfg.coop_scale, False, "no_force"
        return -1.0, cfg.coop_scale, force <= cfg.f_contact_max, "coop"
    return cfg.d_stop, 1.0, False, "body"


class Monitor:
    """Stateful speed and separation monitor: hysteresis, ramp, latching. One per server."""

    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.reset()

    def reset(self):
        self.stopped, self.latched = False, False
        self.reason, self.clear_since, self.stop_pair = None, None, None
        self.scale, self.t = 1.0, None
        self.last = None              # last evaluation (for check_point and the state)
        self.human = []               # last person capsules

    # ---------------------------------------------------------------- one frame
    def evaluate(self, robot, human, v_robot=0.0, v_human=0.0, coop=False, force=None):
        """Pairs -> {d_min, closest, raw_scale, stop, why, binding} without touching the hysteresis state.
        robot: RobotGeometry.capsules(); human: human_capsules(). v_robot: one speed (m/s) or one per robot capsule
        (each pair uses the speed of its own robot link). force: |F| at the end effector (N) or None."""
        cfg = self.cfg
        if not human or len(robot["r"]) == 0:
            return {"d_min": None, "closest": None, "raw": 1.0, "stop": False, "why": None, "binding": None}
        dist, cp, cq = _pairs(robot, human)
        vr = np.broadcast_to(np.asarray(v_robot, float), (dist.shape[0],))
        i, j = np.unravel_index(int(np.argmin(dist)), dist.shape)
        closest = self._pair(robot, human, i, j, dist, cp, cq)
        raw, stop_pair, best_k = 1.0, None, None
        # only pairs that can matter: within the widest slow band
        sp_max = protective_distance(cfg, float(vr.max()), v_human, cfg.d_stop_critical)
        reach = cfg.d_slow + (sp_max - cfg.d_stop_critical) + 0.05
        for (ri, hj) in zip(*np.nonzero(dist < reach)):
            region, kind = human[hj]["region"], robot["kind"][ri]
            c, cap, contact_ok, why = pair_rule(cfg, region, kind, coop, force)
            dd = float(dist[ri, hj])
            if why == "coop" and contact_ok:
                sc = cap if dd < cfg.d_slow else 1.0
            elif why == "coop":                                     # touching, force over the cap
                sc = 0.0 if dd <= 0.0 else (cap if dd < cfg.d_slow else 1.0)
                why = "force"
            else:
                sp = protective_distance(cfg, float(vr[ri]), v_human, c)
                sc = scale_for(cfg, dd, sp, c, cap)
            key = (sc, SEVERITY.get(why, 9), dd)        # the most restrictive pair; among stops the most serious reason
            if best_k is None or key < best_k:
                best_k, raw = key, sc
                stop_pair = (ri, hj, why)
        binding = None
        if stop_pair is not None:
            ri, hj, why = stop_pair
            binding = dict(self._pair(robot, human, ri, hj, dist, cp, cq), why=why)
        return {"d_min": float(dist[i, j]), "closest": closest, "raw": float(raw), "stop": raw <= 0.0,
                "why": binding["why"] if binding else None, "binding": binding}

    @staticmethod
    def _pair(robot, human, i, j, dist, cp, cq):
        return {"d": float(dist[i, j]), "robot_link": robot["names"][i], "robot_kind": robot["kind"][i],
                "robot_ee": robot["ee"][i], "human_part": human[j]["part"], "region": human[j]["region"],
                "p_robot": cp[i, j].tolist(), "p_human": cq[i, j].tolist()}

    def update(self, robot, human, now, v_robot=0.0, v_human=0.0, coop=False, force=None):
        """One frame: evaluate, then hysteresis (stop / clear / resume) and the ramp. -> state dict."""
        cfg = self.cfg
        dt = 0.0 if self.t is None else max(0.0, min(0.5, now - self.t))
        self.t = now
        self.human = human
        ev = self.evaluate(robot, human, v_robot, v_human, coop, force)
        if ev["stop"]:
            self.stopped, self.clear_since = True, None
            self.reason, self.stop_pair = ev["why"], ev["binding"]
            if ev["why"] == "force":
                self.latched = True
        elif self.stopped:
            clear = self._clear(robot, human, coop, force, v_human)
            if clear:
                self.clear_since = self.clear_since if self.clear_since is not None else now
                if now - self.clear_since >= cfg.resume_s and not self.latched:
                    self.stopped, self.reason, self.clear_since, self.stop_pair = False, None, None, None
                    self.scale = 0.0
            else:
                self.clear_since = None
        if self.stopped:
            self.scale = 0.0
        else:
            raw = ev["raw"]
            self.scale = raw if raw <= self.scale else min(raw, self.scale + cfg.ramp_up * dt)
        self.last = ev
        return self.state(ev)

    def _clear(self, robot, human, coop, force, v_human):
        """Every pair is beyond its protective distance plus the hysteresis band. The robot is standing still and
        restarts from zero speed on the ramp, so its own speed term is zero here; the next frames' SSM check covers
        it as it speeds up."""
        if not human or len(robot["r"]) == 0:
            return True
        cfg = self.cfg
        dist, _, _ = _pairs(robot, human)
        for ri, hj in zip(*np.nonzero(dist < cfg.d_slow + 1.0)):
            c, _, contact_ok, why = pair_rule(cfg, human[hj]["region"], robot["kind"][ri], coop, force)
            if why == "coop":
                if not contact_ok and dist[ri, hj] <= cfg.hysteresis:
                    return False
                continue
            if dist[ri, hj] < protective_distance(cfg, 0.0, v_human, c) + cfg.hysteresis:
                return False
        return True

    def confirm(self):
        """A person confirms a latched stop (force over the limit). Takes effect only once they are clear."""
        if self.latched:
            self.latched = False
            return True
        return False

    def state(self, ev=None):
        ev = ev or self.last or {}
        st = {"d_min": None if ev.get("d_min") is None else round(ev["d_min"], 3), "speed_scale": round(self.scale, 3),
              "stop": self.stopped, "latched": self.latched, "reason": self.reason if self.stopped else ev.get("why"),
              "zone": zone_of(self, ev), "closest": None, "binding": None}
        pairs = {"closest": ev.get("closest"), "binding": self.stop_pair if self.stopped and self.stop_pair else ev.get("binding")}
        for k, c in pairs.items():
            if c:
                st[k] = {"robot_link": c["robot_link"], "human_part": c["human_part"], "region": c["region"],
                         "d": round(c["d"], 3), "robot_kind": c["robot_kind"], "robot_ee": c["robot_ee"],
                         "p_robot": [round(x, 3) for x in c["p_robot"]], "p_human": [round(x, 3) for x in c["p_human"]]}
        return st

    # ---------------------------------------------------------------- planning (P2 trajectories, pre-task checks)
    def check_point(self, p, role=None, radius=0.04, coop=False):
        """Is an end-effector at p (sphere of `radius`) outside the person's body zone?
        -> {"ok", "d", "part", "region", "why"}. The body zone is each capsule grown by its stop distance C (head and
        torso: d_stop_critical; hands/forearms while cooperating: d_touch). No person -> ok."""
        cfg = self.cfg
        p = np.asarray(p, float)
        best = None
        for h in self.human:
            d, _, _ = seg_seg(p, p, h["a"], h["b"])
            dd = float(d) - h["r"] - radius
            c, _, _, why = pair_rule(cfg, h["region"], "ee", coop, None)
            margin = dd - c
            if best is None or margin < best[0]:
                best = (margin, dd, h, why)
        if best is None:
            return {"ok": True, "d": None, "part": None, "region": None, "why": None, "role": role}
        margin, dd, h, why = best
        ok = margin >= 0.0
        word = PART_WORD.get(h["region"], "body")
        msg = None if ok else (f"inside your body zone (in your {word})" if dd <= 0.0 else
                               f"inside your body zone: {dd * 100:.0f} cm from your {word}")
        return {"ok": ok, "d": round(dd, 3), "part": h["part"], "region": h["region"], "why": msg, "role": role}


def zone_of(mon, ev):
    """'stop' | 'slow' | 'clear' | None (no person)."""
    if mon.stopped:
        return "stop"
    if ev.get("d_min") is None:
        return None
    return "slow" if mon.scale < 0.999 or ev.get("raw", 1.0) < 0.999 else "clear"


def describe(st, limb="arm"):
    """state.safety -> one calm line for the plan card / status chip, or None when there's nothing to say.
    'Slowing: you're 30 cm away' / 'Stopped: too close to your head'."""
    if not st or st.get("zone") in (None, "clear"):
        return None
    b = st.get("binding") or st.get("closest") or {}
    part = PART_WORD.get(b.get("region"), "body")
    if b.get("human_part") == "neck":
        part = "head"
    d = b.get("d") if st.get("stop") and b.get("d") is not None else st.get("d_min")
    cm = int(round((d or 0.0) * 100))
    if st.get("stop"):
        why = st.get("reason")
        if why == "force":
            return "Stopped: I pressed on you harder than allowed. Say resume when you're clear."
        if why == "critical":
            return f"Stopped: too close to your {part}"
        if why == "no_force":
            return f"Stopped: I'd touch your {part} and I can't feel force here"
        if cm <= 1:
            return f"Stopped: your {part} is touching my {limb}"
        return f"Stopped: you're {cm} cm from my {limb}"
    return f"Slowing: you're {max(cm, 0)} cm away"
