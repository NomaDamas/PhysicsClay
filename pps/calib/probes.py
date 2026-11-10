"""Probe routine (docs/calibration.md): short, safe motions that excite the parameters system identification fits.

    swing   joint step responses on the big arm joints (brisk trapezoids)   -> kp_scale, damping_scale, latency_ms
    press   guarded move onto a free spot of the table, then hold 4 mm in    -> kp_scale (static stiffness), force check
    lift    grasp an object, lift 10 cm and hold still, put it back          -> mass_scale (the wrist feels the weight)
    push    slide an object 10 cm with closed jaws, then a short flick       -> friction_scale (push force, slide-out)

Probes are planned in the current world from what the robot sees (object poses, the work surface) with the existing
planner: the IK (mink) runtime follows hand targets exactly as tasks do (grasps come from `Tasks.grasp_waypoints`), and
joint-space segments drive the planner's configuration directly. Every command goes to the robot through a
`RobotDriver` (pps/bridge): the same routine runs on the simulated plant (SimDriver) and, later, on hardware.

Each probe is recorded as its own Log (pps/calib/log.py), from a short rest at its start, so it can be replayed alone.

Running: `ProbeRunner(rt, tasks, driver, default_probes())` takes over the runtime's plant step (rt.plant_step) while it
runs: call `run_headless(runner)` without a server, or let the server's loop call rt.step as usual (pps/features/calib.py).
"""
import math

import mujoco
import numpy as np

from .. import tasks as task_mod
from ..robot_runtime import slerp
from .log import Recorder

CTRL_DT = 0.01                    # headless control tick (100 Hz); the server uses its own tick
DOWN = np.array([0.0, 0.0, -1.0])


class Skip(Exception):
    """A probe that can't run in this world (no object to push, nothing in reach...)."""


# ------------------------------------------------------------------ segments
def wait(sec):
    return {"kind": "wait", "sec": float(sec)}


def joint(q, vmax=1.2, amax=8.0):
    return {"kind": "joint", "q": np.array(q, float), "vmax": float(vmax), "amax": float(amax)}


def move(palm, p, quat=None, speed=0.15, profile="minjerk"):
    return {"kind": "move", "palm": palm, "p": np.array(p, float), "quat": None if quat is None else np.array(quat, float),
            "speed": float(speed), "profile": profile}


def grip(role, value, sec=0.9):
    return {"kind": "grip", "role": role, "value": float(value), "sec": float(sec)}


def guard(palm, direction, speed=0.01, f_touch=1.5, f_load=None, load_speed=0.002, travel=0.05, hold=1.0):
    """A guarded move: along `direction` at `speed` until the hand feels f_touch N against it (contact), then on at
    `load_speed` until it feels f_load N (if given), then hold that command for `hold` s. The force never runs away:
    the command stops where the force was reached (closed loop on the measured wrist force)."""
    d = np.asarray(direction, float)
    return {"kind": "guard", "palm": palm, "dir": d / np.linalg.norm(d), "speed": float(speed), "f_touch": float(f_touch),
            "f_load": None if f_load is None else float(f_load), "load_speed": float(load_speed), "travel": float(travel),
            "hold": float(hold)}


def _minjerk(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10 - 15 * s + 6 * s * s)


def _ramped(s, edge=0.15):
    """Constant speed with smooth 15 % edges (a steady push)."""
    s = min(max(s, 0.0), 1.0)
    v = 1.0 / (1.0 - edge)                        # cruise speed for unit distance (smoothstep edges cover half their span)
    if s < edge:
        x = s / edge
        return v * edge * (x * x * x - 0.5 * x ** 4)
    if s > 1 - edge:
        return 1.0 - _ramped(1.0 - s, edge)
    return v * edge * 0.5 + v * (s - edge)


def _trapezoid(dist, vmax, amax):
    """Duration and normalized position profile s(t) of a rest-to-rest move over `dist` (velocity / acceleration limits)."""
    if dist < 1e-9:
        return 0.0, lambda t: 1.0
    if dist > vmax * vmax / amax:
        ta = vmax / amax
        T = dist / vmax + ta

        def s(t):
            if t < ta:
                return 0.5 * amax * t * t / dist
            if t < T - ta:
                return (0.5 * amax * ta * ta + vmax * (t - ta)) / dist
            r = max(T - t, 0.0)
            return 1.0 - 0.5 * amax * r * r / dist
        return T, s
    ta = math.sqrt(dist / amax)
    T = 2 * ta

    def s2(t):
        if t < ta:
            return 0.5 * amax * t * t / dist
        r = max(T - t, 0.0)
        return 1.0 - 0.5 * amax * r * r / dist
    return T, s2


# ------------------------------------------------------------------ geometry helpers
def arm_joints(rt, palm):
    """qpos addresses of the arm joints from the base outwards (servoed hinges between the base and the hand), excluding
    fingers, frozen joints (heads, wheels), mobile-base joints and the legs / waist of humanoids."""
    pl, mi = rt.plant, rt.configuration.model
    servoed = {mujoco.mj_id2name(pl.m, mujoco.mjtObj.mjOBJ_JOINT, pl.m.actuator_trnid[a, 0]) for a, _, _ in pl.track}
    freeze = tuple(getattr(getattr(rt, "profile", None), "freeze", ()) or ())
    base = tuple(getattr(getattr(rt, "profile", None), "base_joints", ()) or ())
    skip = ("hip", "knee", "ankle", "waist", "torso", "head", "neck", "wheel")
    b = mi.site_bodyid[mi.site(rt.ee_site[palm]).id]
    chain = []
    while b > 0:
        for k in range(mi.body_jntnum[b]):
            j = mi.body_jntadr[b] + k
            name = mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
            if mi.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE or name.startswith("pc_") or name not in servoed:
                continue
            if any(f in name for f in freeze) or name in base or any(s in name for s in skip):
                continue
            chain.append((int(mi.jnt_qposadr[j]), name, (float(mi.jnt_range[j, 0]), float(mi.jnt_range[j, 1])) if mi.jnt_limited[j] else None))
        b = mi.body_parentid[b]
    return chain[::-1]


def _fk(rt, q):
    mi = rt.configuration.model
    d = mujoco.MjData(mi)
    d.qpos[:] = q
    d.mocap_pos[:], d.mocap_quat[:] = rt.ik_data.mocap_pos, rt.ik_data.mocap_quat
    mujoco.mj_kinematics(mi, d)
    return d


def _hand_low(rt, palm, d=None):
    """How far the lowest point of the hand's collision geoms is below the TCP (current planned pose)."""
    pl = rt.plant
    role = palm[:-5] if palm.endswith("_palm") else palm
    d = d or rt.ik_data
    m = rt.configuration.model
    sid = m.site(rt.ee_site[palm]).id
    tcp = d.site_xpos[sid]
    low = 0.0
    for g in range(m.ngeom):
        if m.geom_bodyid[g] not in pl.ee_bodies.get(role, ()) or not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
        R = d.geom_xmat[g].reshape(3, 3)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    z = (d.geom_xpos[g] + R @ (c + h * np.array([sx, sy, sz])))[2]
                    low = max(low, float(tcp[2] - z))
    return low


def _lead(rt, palm, direction, d=None):
    """How far the jaws reach ahead of the TCP along a horizontal direction (the face that pushes)."""
    pl = rt.plant
    role = palm[:-5] if palm.endswith("_palm") else palm
    d = d or rt.ik_data
    m = rt.configuration.model
    tcp = d.site_xpos[m.site(rt.ee_site[palm]).id]
    g = pl.grip.get(role)
    geoms = [k for side in g["pads"] for k in side] if g else [k for k in range(m.ngeom) if m.geom_bodyid[k] in pl.ee_bodies.get(role, ())]
    lead = 0.0
    for k in geoms:
        c, h = m.geom_aabb[k, :3], m.geom_aabb[k, 3:]
        R = d.geom_xmat[k].reshape(3, 3)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    p = d.geom_xpos[k] + R @ (c + h * np.array([sx, sy, sz]))
                    lead = max(lead, float(np.dot(p - tcp, direction)))
    return lead


def _ik_to(rt, palm, p, quat=None, iters=80, dt=0.03):
    """Where the planner would put the arm to bring the hand to p (scratch configuration). -> (q, distance left)"""
    import mink
    cfg = mink.Configuration(rt.configuration.model)
    cfg.data.mocap_pos[:], cfg.data.mocap_quat[:] = rt.ik_data.mocap_pos, rt.ik_data.mocap_quat
    cfg.update(rt.configuration.q.copy())
    task = rt.hand_tasks[rt.hands.index(palm)]
    saved = task.transform_target_to_world
    q_t = rt.data.mocap_quat[rt.hands_mid[palm]].copy() if quat is None else np.asarray(quat, float)
    task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(q_t), np.asarray(p, float)))
    try:
        for _ in range(iters):
            cfg.integrate_inplace(rt.solve(cfg, dt), dt)
    finally:
        if saved is not None:
            task.set_target(saved)
    return cfg.q.copy(), float(np.linalg.norm(cfg.data.site_xpos[rt.palm_sid[palm]] - p))


def _role(palm):
    return palm[:-5] if palm.endswith("_palm") else palm


def _clear_of_objects(tasks, p, r, skip=()):
    for n in tasks.objs:
        if n in skip:
            continue
        if np.linalg.norm(tasks.mug_center(n)[:2] - p[:2]) < r + tasks.objs[n]["r"]:
            return False
    for o in tasks.obstacles.values():
        if np.linalg.norm(np.asarray(o["pos"])[:2] - p[:2]) < r + o["r"]:
            return False
    return True


def _on_table(p, margin=0.05):
    return (task_mod.TABLE_X[0] + margin < p[0] < task_mod.TABLE_X[1] - margin and
            task_mod.TABLE_Y[0] + margin < p[1] < task_mod.TABLE_Y[1] - margin)


# ------------------------------------------------------------------ probes
class Probe:
    name, label, fits = "probe", "Probe", ()

    def plan(self, rt, tasks):
        raise NotImplementedError


class Swing(Probe):
    name, label, fits = "swing", "Swing", ("kp_scale", "damping_scale", "latency_ms", "armature_scale", "frictionloss_scale")

    def __init__(self, amp=0.3, vmax=1.5, amax=10.0, joints=3):
        self.amp, self.vmax, self.amax, self.n = amp, vmax, amax, joints

    def _swings(self, rt, tasks, palm, q0, chain):
        """Joint targets around q0 that keep the hand clear of the table and the objects: [(adr, q)]."""
        top = task_mod.TABLE_TOP
        sid = rt.configuration.model.site(rt.ee_site[palm]).id
        out = []
        for adr, _name, rng in [chain[i] for i in (0, 1, 3, 2, 4) if i < len(chain)]:
            for sgn in (1.0, -1.0):
                q = q0.copy()
                q[adr] += sgn * self.amp
                if rng is not None and not (rng[0] + 0.05 < q[adr] < rng[1] - 0.05):
                    continue
                d = _fk(rt, q)
                tcp = d.site_xpos[sid]
                if tcp[2] - _hand_low(rt, palm, d) < top + 0.07 or not _clear_of_objects(tasks, tcp, 0.10):
                    continue
                out.append((adr, q))
                break
            if len(out) >= self.n:
                break
        return out

    def plan(self, rt, tasks):
        palm = rt.hands[0]
        q0 = rt.configuration.q.copy()
        chain = arm_joints(rt, palm)
        if not chain:
            raise Skip("no servoed arm joints to swing")
        swings, pre = self._swings(rt, tasks, palm, q0, chain), []
        if not swings:                                 # the hand starts low (humanoids, ALOHA): raise it first, then swing
            tcp = rt.ik_data.site_xpos[rt.palm_sid[palm]].copy()
            for up in (np.array([tcp[0], tcp[1], task_mod.TABLE_TOP + 0.25]), np.array([tcp[0] + 0.08, tcp[1], task_mod.TABLE_TOP + 0.3])):
                q_up, dist = _ik_to(rt, palm, up)
                if dist > 0.03:
                    continue
                swings = self._swings(rt, tasks, palm, q_up, chain)
                if swings:
                    pre, q0 = [move(palm, up, None, 0.15), wait(0.3)], q_up
                    break
        if not swings:
            raise Skip("no joint can swing clear of the table here")
        segs = [wait(0.3)] + pre
        for adr, q in swings:                          # relative to where the arm really is when the swings start
            segs += [{"kind": "swingj", "adr": adr, "delta": float(q[adr] - q0[adr]), "vmax": self.vmax, "amax": self.amax}]
        return segs + ([joint(rt.configuration.q.copy(), 1.0, 4.0), wait(0.2)] if pre else [])


class Press(Probe):
    name, label, fits = "press", "Press", ("kp_scale",)

    def __init__(self, f_load=8.0, hold=1.0):
        self.f_load, self.hold = f_load, hold

    def plan(self, rt, tasks):
        q0 = rt.configuration.q.copy()
        top = task_mod.TABLE_TOP
        regs = task_mod.REGIONS
        spots = [np.array([*regs[k], top]) for k in ("center", "front", "right", "left") if k in regs]
        spots += [np.array([x, y, top]) for x in (0.32, 0.40, 0.28) for y in (0.0, -0.12, 0.12, -0.2, 0.2)]
        for spot in spots:
            if not _on_table(spot) or not _clear_of_objects(tasks, spot, 0.09):
                continue
            palm = tasks.hand_for(spot)
            low = _hand_low(rt, palm)
            hover, near = spot + [0, 0, low + 0.07], spot + [0, 0, low + 0.012]
            if not all(rt.precheck(np.array([hover, near]), palm)):
                continue
            role = _role(palm)
            quat = rt.data.mocap_quat[rt.hands_mid[palm]].copy()
            segs = [wait(0.3)]
            if role in rt.plant.grip:
                segs.append(grip(role, 1.0, 0.8))
            segs += [move(palm, hover, quat, 0.2), move(palm, near, quat, 0.06), wait(0.25),
                     guard(palm, DOWN, 0.01, 1.5, self.f_load, 0.003, 0.04, self.hold), move(palm, hover, quat, 0.08)]
            if role in rt.plant.grip:
                segs.append(grip(role, 0.0, 0.5))
            return segs + [joint(q0, 1.0, 4.0), wait(0.2)]
        raise Skip("no free spot of the table in reach to press on")


GRASPABLE = ("block", "box", "can", "cup", "bottle", "fruit", "mug")
SLIDABLE = ("block", "box", "can", "cup", "mug")


class Lift(Probe):
    name, label, fits = "lift", "Lift", ("mass_scale",)

    def __init__(self, height=0.10, hold=1.2):
        self.height, self.hold = height, hold

    def plan(self, rt, tasks):
        if not rt.plant.grip:
            raise Skip("no gripper")
        q0 = rt.configuration.q.copy()
        kinds = rt.plant.kinds
        cands = sorted(tasks.objs, key=lambda n: (GRASPABLE.index(kinds.get(n, "block")) if kinds.get(n, "block") in GRASPABLE else 99))
        for name in cands:
            if kinds.get(name, "block") not in GRASPABLE:
                continue
            c = tasks.mug_center(name)
            palm = tasks.hand_for(c)
            role = _role(palm)
            if role not in rt.plant.grip:
                continue
            cur = rt.ik_data.site_xpos[rt.palm_sid[palm]].copy()
            kps, quats, gi = tasks.grasp_waypoints(name, palm, cur)
            if quats is None:
                continue
            g, qg = np.asarray(kps[gi - 1], float), np.asarray(quats[gi], float)
            pre = np.asarray(kps[gi - 2], float) if gi >= 2 else g + [0, 0, 0.09]
            up, down = g + [0, 0, self.height], g + [0, 0, 0.006]
            pts, qs, _ = tasks._dense([cur, pre, g, up], [quats[0], qg, qg, qg])
            rt.orient(True)
            if not all(rt.precheck(pts, palm, quats=qs)):
                continue
            segs = [wait(0.3), grip(role, 0.0, 0.3)]
            segs += [move(palm, k, qg, 0.2) for k in kps[:gi - 2]]       # e.g. up over the table's edge first
            segs += [move(palm, pre, qg, 0.2), move(palm, g, qg, 0.05), wait(0.25), grip(role, 1.0, 1.0),
                     move(palm, up, qg, 0.06), wait(self.hold), move(palm, down, qg, 0.05), grip(role, 0.0, 0.9),
                     move(palm, pre, qg, 0.1), joint(q0, 1.0, 4.0), wait(0.2)]
            self.obj = name
            return segs
        raise Skip("nothing I can grasp in reach")


class Slide(Probe):
    """Push an object across the surface while holding it: grasp it, set it down until the table carries its weight,
    slide it 10 cm, let go. The wrist feels the table's friction (horizontal) and the normal force (vertical), so the
    friction coefficient shows up directly. (Free pushes with open contact jammed or tipped light objects in this
    contact model, so the probe holds what it pushes.)"""
    name, label, fits = "slide", "Slide", ("friction_scale",)

    def __init__(self, dist=0.10, speed=0.04, f_touch=3.0):
        self.dist, self.speed, self.f_touch = dist, speed, f_touch

    def plan(self, rt, tasks):
        if not rt.plant.grip:
            raise Skip("no gripper")
        q0 = rt.configuration.q.copy()
        kinds = rt.plant.kinds
        names = sorted(tasks.objs, key=lambda n: SLIDABLE.index(kinds.get(n, "block")) if kinds.get(n, "block") in SLIDABLE else 99)
        for name in names:
            if kinds.get(name, "block") not in SLIDABLE:
                continue
            o = tasks.objs[name]
            c = tasks.mug_center(name)
            palm = tasks.hand_for(c)
            role = _role(palm)
            if role not in rt.plant.grip:
                continue
            dirs = [np.array([0, 1.0, 0]), np.array([0, -1.0, 0]), np.array([1.0, 0, 0]), np.array([-1.0, 0, 0])]
            dirs.sort(key=lambda d: float(np.dot(d[:2], c[:2] - np.array([0.45, 0.0]))))   # towards the middle of the table
            dirs = [d for d in dirs if _on_table(c + d * (self.dist + 0.03), 0.06) and
                    all(_clear_of_objects(tasks, c + d * s_, o["r"] + 0.02, skip=(name,)) for s_ in np.linspace(0.03, self.dist + 0.03, 6))]
            if not dirs:
                continue
            cur = rt.ik_data.site_xpos[rt.palm_sid[palm]].copy()
            kps, quats, gi = tasks.grasp_waypoints(name, palm, cur)
            if quats is None:
                continue
            g, qg = np.asarray(kps[gi - 1], float), np.asarray(quats[gi], float)
            pre = np.asarray(kps[gi - 2], float) if gi >= 2 else g + [0, 0, 0.09]
            for d in dirs:
                up = g + [0, 0, 0.015]
                end = up + d * self.dist
                pts, qs, _ = tasks._dense([cur, pre, g, up, end, end + [0, 0, 0.08]], [quats[0]] + [qg] * 5)
                rt.orient(True)
                if not all(rt.precheck(pts, palm, quats=qs)):
                    continue
                segs = [wait(0.3), grip(role, 0.0, 0.3)]
                segs += [move(palm, k, qg, 0.2) for k in kps[:gi - 2]]
                segs += [move(palm, pre, qg, 0.2), move(palm, g, qg, 0.05), wait(0.25), grip(role, 1.0, 1.0),
                         move(palm, up, qg, 0.03), wait(0.4),
                         guard(palm, DOWN, 0.01, self.f_touch, None, travel=0.03, hold=0.4),
                         {"kind": "slide", "palm": palm, "d": d, "dist": self.dist, "speed": self.speed, "quat": qg},
                         wait(0.3), grip(role, 0.0, 0.9), {"kind": "lift", "palm": palm, "dz": 0.08, "speed": 0.08, "quat": qg},
                         joint(q0, 1.0, 4.0), wait(0.2)]
                self.obj = name
                return segs
        raise Skip("no object I can hold and slide here")


def default_probes(names=None):
    return [PROBES[n]() for n in (names or ORDER) if n in PROBES]


PROBES = {"swing": Swing, "press": Press, "lift": Lift, "slide": Slide}
ORDER = ("swing", "press", "lift", "slide")


# ------------------------------------------------------------------ runner
class ProbeRunner:
    """Runs probes on a runtime through a RobotDriver, one control tick per call of the runtime's plant step."""

    def __init__(self, rt, tasks, driver, probes, meta=None, on_change=None):
        self.rt, self.tasks, self.driver = rt, tasks, driver
        self.queue = list(probes)
        self.total = len(self.queue)
        self.meta = dict(meta or {})
        self.on_change = on_change
        self.logs, self.skipped, self.done_probes = [], [], []
        self.probe, self.segs, self.seg, self.rec = None, [], None, None
        self.q_hold = np.array(rt.configuration.q, float)
        self.prev_kind = None
        self.seg_i = -1
        self.status, self.message = "ready", ""
        self.t = 0.0
        self._installed = False

    # ---------------------------------------------------------------- lifecycle
    def install(self):
        """Route the runtime's plant step through this runner (the IK still runs; the runner decides what is sent)."""
        rt = self.rt
        rt.hold_here()
        self.q_hold = np.array(rt.configuration.q, float)
        rt.plant_step = self.plant_step
        rt.mode = "CALIB"
        self._installed = True
        self.status = "running"
        return self

    def uninstall(self):
        rt = self.rt
        if self._installed:
            if "plant_step" in rt.__dict__:
                del rt.plant_step
            self._installed = False
        rt.mode = "MIRROR"
        rt.orient(False)
        rt.set_obstacles(tuple(getattr(rt.plant, "objects", None) or ()))   # between tasks: fixtures only (Tasks._free_obstacles)
        rt.hold_here()

    def abort(self, why="stopped"):
        if self.rec is not None:
            self.rec = None
        self.driver.stop()
        self.status, self.message = "aborted", why
        self.uninstall()
        self._changed()

    @property
    def finished(self):
        return self.status in ("done", "aborted", "failed")

    def progress(self):
        return {"status": self.status, "probe": getattr(self.probe, "name", None), "index": len(self.done_probes) + len(self.skipped) + (1 if self.probe else 0),
                "total": self.total, "done": [p for p in self.done_probes], "skipped": list(self.skipped), "message": self.message}

    def _changed(self):
        if self.on_change:
            self.on_change(self)

    # ---------------------------------------------------------------- per tick
    def plant_step(self, dt):
        """The runtime's plant step while probing. A probe that breaks must never break the robot loop: it is aborted
        (the robot holds where it is) and the error is reported."""
        try:
            self.tick(dt)
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            self.status, self.message = "failed", f"probe error: {type(e).__name__}: {e}"
            self.rec = None
            try:
                self.driver.stop()
                self.uninstall()
                if hasattr(self.driver, "advance"):
                    self.driver.advance(dt)
            except Exception:  # noqa: BLE001
                pass
            self._changed()

    def tick(self, dt):
        drv = self.driver
        if self.finished:
            drv.send_q_des(self.q_hold, None)
            if hasattr(drv, "advance"):
                drv.advance(dt)
            return
        if self.seg is None and not self._next():
            drv.send_q_des(self.q_hold, None)
            if hasattr(drv, "advance"):
                drv.advance(dt)
            return
        st = drv.read_state()
        settling = self.seg["kind"] == "settle"          # not logged: the replay starts from a robot at rest
        q = self._command(st, dt)
        if self.rec is not None and not settling:
            self.rec.add(st, q, drv.commanded_grip() if hasattr(drv, "commanded_grip") else {}, self.rt.base_pose(), self.seg_i)
        drv.send_q_des(q, st["t"])
        if hasattr(drv, "advance"):
            drv.advance(dt)

    def _next(self):
        """Start the next segment (planning the next probe when the current one is over). -> False when all are done."""
        while not self.segs:
            if self.probe is not None:              # the probe is over: keep its log
                log = self.rec.log() if self.rec is not None else None
                if log is not None and len(log) > 10:
                    self.logs.append(log)
                self.done_probes.append(self.probe.name)
                self.probe, self.rec = None, None
                self._changed()
            if not self.queue:
                self.status, self.message = "done", f"{len(self.logs)} probe logs recorded" + (
                    f"; skipped {', '.join(n for n, _ in self.skipped)}" if self.skipped else "")
                self.uninstall()
                self._changed()
                return False
            p = self.queue.pop(0)
            self.rt.hold_here()
            self.q_hold = np.array(self.rt.configuration.q, float)
            try:
                segs = p.plan(self.rt, self.tasks)
            except Skip as e:
                self.skipped.append((p.name, str(e)))
                self.rt.configuration.update(self.q_hold)
                self._changed()
                continue
            self.rt.configuration.update(self.q_hold)
            for h in self.rt.hands:
                self.rt.mocap_to_site(f"{h}_target", self.rt.ee_site[h])
            self.probe, self.segs = p, [{"kind": "settle", "max": 2.5, "still": 0.0}] + list(segs)   # start each log at rest
            caps = getattr(self.driver, "capabilities", {})
            self.rec = Recorder({"version": 1, "probe": p.name, "label": p.label, "fits": list(p.fits), "roles": list(caps.get("grippers") or caps.get("roles") or []),
                                 "objects": list(self.rt.plant.objects) if self.rt.plant is not None else [], "joints": list(caps.get("joints") or []),
                                 "source": caps.get("name", "sim"), "object": getattr(p, "obj", None), **self.meta})
            self.message = f"{p.label} ({len(self.done_probes) + len(self.skipped) + 1}/{self.total})"
            self._changed()
        self.seg = self.segs.pop(0)
        self.seg["t"] = 0.0
        self.seg_i += 1
        self._start(self.seg)
        return True

    def _start(self, s):
        rt = self.rt
        k = s["kind"]
        hand = self.prev_kind in ("move", "guard")       # the hand target is live: continue from it (no jump)
        self.prev_kind = "move" if k in ("slide", "lift") else k
        if k in ("slide", "lift"):                      # relative moves: from wherever the hand target is now
            mid = rt.hands_mid[s["palm"]]
            p0 = rt.data.mocap_pos[mid].copy() if hand else rt.ik_data.site_xpos[rt.palm_sid[s["palm"]]].copy()
            delta = s["d"] * s["dist"] if k == "slide" else np.array([0.0, 0.0, s["dz"]])
            s.update(kind="move", p=p0 + delta, profile="ramped" if k == "slide" else "minjerk")
            k = "move"
        if k == "swingj":                              # out by delta on one joint, hold, back, hold
            q1 = self.q_hold.copy()
            q1[s["adr"]] += s["delta"]
            self.segs[:0] = [wait(0.35), joint(self.q_hold.copy(), s["vmax"], s["amax"]), wait(0.35)]
            s.update(kind="joint", q=q1)
            k = "joint"
        if k == "joint":
            s["q0"] = self.q_hold.copy()
            dist = float(np.max(np.abs(s["q"] - s["q0"])))
            s["T"], s["prof"] = _trapezoid(dist, s["vmax"], s["amax"])
        elif k == "grip":
            self.driver.send_gripper(s["role"], s["value"])
        elif k in ("move", "guard"):
            palm = s["palm"]
            mid = rt.hands_mid[palm]
            s["p0"] = rt.data.mocap_pos[mid].copy() if hand else rt.ik_data.site_xpos[rt.palm_sid[palm]].copy()
            s["q0"] = rt.data.mocap_quat[mid].copy()
            rt.data.mocap_pos[mid] = s["p0"]
            rt.orient(True)
            if k == "move":
                s["T"] = max(float(np.linalg.norm(s["p"] - s["p0"])) / max(s["speed"], 1e-3), 0.15)
            else:
                s["phase"] = "approach"

    def _command(self, st, dt):
        """The joint command for this tick (and the hand target for the IK's next step)."""
        rt, s = self.rt, self.seg
        k = s["kind"]
        s["t"] += dt
        done = False
        if k == "settle":                               # hold until the joints are still (a light arm rings a while)
            q = self.q_hold
            dq = st.get("dq")
            moving = dq is not None and len(dq) and float(np.max(np.abs(dq))) > 0.02
            s["still"] = 0.0 if moving else s["still"] + dt
            done = s["still"] >= 0.1 or s["t"] >= s["max"]
        elif k in ("wait", "grip"):
            q = self.q_hold
            done = s["t"] >= s["sec"]
        elif k == "joint":
            a = s["prof"](s["t"])
            q = s["q0"] + (s["q"] - s["q0"]) * a
            done = s["t"] >= s["T"]
        else:
            palm = s["palm"]
            mid = rt.hands_mid[palm]
            q = np.array(rt.configuration.q, float)       # the IK's answer to last tick's target
            tcp = rt.ik_data.site_xpos[rt.palm_sid[palm]]
            if k == "move":
                a = (_minjerk if s["profile"] == "minjerk" else _ramped)(s["t"] / s["T"])
                rt.data.mocap_pos[mid] = s["p0"] + (s["p"] - s["p0"]) * a
                if s["quat"] is not None:
                    rt.data.mocap_quat[mid] = slerp(s["q0"], s["quat"], min(1.0, s["t"] / s["T"]))
                done = s["t"] >= s["T"] and (np.linalg.norm(tcp - s["p"]) < 0.004 or s["t"] >= s["T"] + 1.0)
            else:
                done = self._guard(s, st, dt, mid)
            self.q_hold = q
        if k in ("wait", "grip", "joint", "settle"):
            rt.configuration.update(q)
            for h in rt.hands:
                rt.mocap_to_site(f"{h}_target", rt.ee_site[h])
            if k == "joint":
                self.q_hold = q
        if done:
            self.seg = None
        return np.array(q, float)

    def _guard(self, s, st, dt, mid):
        rt = self.rt
        role = _role(s["palm"])
        f = (st.get("ee_force") or {}).get(role)
        push = 0.0 if f is None else float(-np.dot(f, s["dir"]))      # what the surface pushes back with
        s["fmax"] = max(s.get("fmax", 0.0), push)
        tgt = rt.data.mocap_pos[mid]
        if s["phase"] in ("approach", "load"):
            want = s["f_touch"] if s["phase"] == "approach" else s["f_load"]
            if push >= want:
                if s["phase"] == "approach" and s["f_load"] is not None and s["f_load"] > s["f_touch"]:
                    s["phase"] = "load"
                else:
                    s["phase"], s["t1"] = "hold", s["t"]
                return False
            if np.linalg.norm(tgt - s["p0"]) > s["travel"]:              # nothing there: end this segment, say so
                self.message = f"no contact within {s['travel'] * 100:.0f} cm"
                s["phase"] = "missed"
                return True
            v = s["speed"] if s["phase"] == "approach" else s["load_speed"]
            rt.data.mocap_pos[mid] = tgt + s["dir"] * v * dt
            return False
        return s["phase"] != "hold" or s["t"] - s["t1"] >= s["hold"]


class TaskRecorder:
    """Records whatever the runtime does (a task, a skill, a drawn path) as a Log, through a RobotDriver: the IK's
    q_des goes to the driver every tick. Used for the held-out task of `scripts/calibrate.py --demo`, and for logs of
    ordinary tasks to fit from."""

    def __init__(self, rt, driver, meta=None):
        self.rt, self.driver = rt, driver
        caps = getattr(driver, "capabilities", {})
        self.rec = Recorder({"version": 1, "probe": "task", "label": "Task", "fits": [], "roles": list(caps.get("grippers") or caps.get("roles") or []),
                             "objects": list(rt.plant.objects) if rt.plant is not None else [], "joints": list(caps.get("joints") or []),
                             "source": caps.get("name", "sim"), **(meta or {})})
        self._installed = False

    def install(self):
        self.rt.plant_step = self.plant_step
        self._installed = True
        return self

    def uninstall(self):
        if "plant_step" in self.rt.__dict__:
            del self.rt.plant_step
        self._installed = False
        return self.rec.log()

    def plant_step(self, dt):
        drv = self.driver
        st = drv.read_state()
        q = np.array(self.rt.configuration.q, float)
        self.rec.add(st, q, drv.commanded_grip() if hasattr(drv, "commanded_grip") else {}, self.rt.base_pose())
        drv.send_q_des(q, st["t"])
        if hasattr(drv, "advance"):
            drv.advance(dt)


def headless_robot(robot, scene_spec, params=None, noise=None, seed=0):
    """A world, runtime, task layer and SimDriver without a server. params: plant parameters to give the robot (the
    "hidden real" robot of the demo); noise: sensor noise of the driver. -> (world, rt, tasks, driver)"""
    from ..world import scene
    from ..world.runtime import make_runtime
    w = scene.build(robot, scene_spec, physics=True)
    task_mod.apply_surface(w.surface, w.regions)
    rt = make_runtime(w)
    tasks = task_mod.Tasks(rt, w.kinds)
    if params:
        from . import params as P
        P.apply(rt.plant, params)
        rt.plant.reset(rt.configuration.q)
        rt.plant.settle()
    from ..bridge import SimDriver
    drv = SimDriver(rt.plant, base_pose=rt.base_pose, noise=noise, seed=seed)
    return w, rt, tasks, drv


def run_headless(runner, dt=CTRL_DT, limit=180.0):
    """Drive a ProbeRunner without a server: rt.step(dt) per tick (IK, then the runner's plant step)."""
    rt = runner.rt
    if not runner._installed:
        runner.install()
    t = 0.0
    while not runner.finished and t < limit:
        rt.step(dt)
        t += dt
    if not runner.finished:
        runner.abort("time limit")
    return runner.logs
