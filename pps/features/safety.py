"""Safety around people (v1 P5, issue #33): the robot slows down near you and stops before it touches you.

Not a certified safety system (see docs/safety.md). Every frame, while a person is in view (a webcam, the phone, VR,
or a played feed in checks; not the synthetic no-camera dancer and not a replayed video), the person's 33-point pose
becomes capsules and the robot's links become capsules (pps/safety.py), and speed and separation monitoring sets
`rt.safety_scale`, which both runtimes read in step():

    1      full speed           0 < s < 1  everything slower: paths, skills, driving, live mirroring, the IK itself
    0      protective stop: targets, IK and the base hold still; a running task is paused (status stays "running",
           step "paused", message "Paused for safety: …") and resumes on its own once you have been clear for
           resume_s (policy: automatic resume for separation stops; a force-limit stop is latched and needs
           {type: "safety", op: "resume"} once you are clear).

Rules (pps/safety.py has the formula and the defaults):
  * head, neck or torso inside d_stop_critical: stop, in every mode;
  * cooperating (watch mode `assist` or `mirror`, or a handover): your hands and forearms may come up to the end effector at
    coop_scale; contact only with a measured end-effector force (P0: rt.ee_force() / state.physics.ee_force) under
    f_contact_max; without a force reading the robot stops at contact;
  * everything else: slow inside d_slow, stop at the protective separation distance.

Hooks for other features:
  * pre-task guard (runs before every other pre_task): no new motion while stopped; a pick_place / reach whose target
    lies inside your body zone is refused.
  * `tasks.safety.check_point(p, role)` -> {ok, d, part, region, why} for planners (P2 trajectories), and
    `check_point(ctx, p, role)` here.

Config: server side only. Defaults in pps.safety.Config, overridden by the robot profile's `safety` dict and by a JSON
file (`--safety-config path` or PC_SAFETY_CONFIG). Values are clamped to floors; nothing turns safety off.
The browser can only read state.safety and confirm a latched stop: {type: "safety", op: "resume"}. Any other op
(e.g. "disable") is answered with ok: false and changes nothing.

State: safety = {d_min, speed_scale, stop, latched, reason, zone, closest: {robot_link, human_part, region, d, …},
                 binding, message, person, coop, v_robot, v_human, capsules: [[ax, ay, az, bx, by, bz, r, region]],
                 config, reply}
"""
import json
import os
import time

import numpy as np

from .. import ext
from .. import safety as S
from ..human.model import ground, pose33

PERSON_SRC = ("feed", "webcam", "phone", "vr")     # a person in the room (feed = a played person in checks)
CAMERA_SRC = ("webcam", "phone", "vr")            # tracking can drop out: keep the last pose for lost_hold_s
MOTION = ("pick_place", "reach", "follow_path", "play_skill", "handover", "drive", "home", "follow_me")
SPEAK_GAP = 4.0
TELEPORT = 0.15          # m a link may move between two frames before it counts as a jump (reset), not a motion


def _config(ctx):
    over = {}
    prof = getattr(getattr(ctx, "world", None), "profile", None)
    over.update(getattr(prof, "safety", None) or {})
    path = os.environ.get("PC_SAFETY_CONFIG")
    if path:
        try:
            with open(path) as f:
                over.update(json.load(f))
        except (OSError, ValueError) as e:
            print(f"[safety] couldn't read {path}: {e}; using the defaults")
    return S.Config.from_dict(over)


def _st(ctx):
    s = ctx.store.get("safety")
    if s is None:
        s = ctx.store["safety"] = {"mon": S.Monitor(_config(ctx)), "geo": None, "rt_id": None, "prev": None,
                                   "v_robot": 0.0, "v_links": np.zeros(0), "v_human": 0.0, "hprev": None, "seen": None, "human": [],
                                   "src": None, "paused": None, "said": 0.0, "reply": None, "state": None,
                                   "coop": False, "force": None, "person": False}
    return s


def _r(p, n=3):
    return [round(float(x), n) for x in p]


def _say(ctx, text):
    b = getattr(ctx, "brain", None)
    if b is not None and hasattr(b, "utter") and text:
        try:
            b.utter(text, "robot")
        except Exception:
            pass


# ---------------------------------------------------------------------- inputs
def _robot_bodies(ctx, rt):
    w = getattr(ctx, "world", None)
    if w is not None and getattr(w, "model", None) is rt.model and getattr(w, "robot_bodies", None):
        return w.robot_bodies
    m = rt.model
    roots = set(getattr(rt, "robot_roots", ()) or ())
    return {b for b in range(1, m.nbody) if int(m.body_rootid[b]) in roots and m.body_mocapid[m.body_rootid[b]] < 0}


def _person(ctx, now):
    """-> (world joints or None, source). Co-presence's (smoothed) joints; when the person view is switched off in
    co-presence, safety still reads the camera or feed itself (switching the avatar off must not switch safety off)."""
    cp = ctx.store.get("copresence") or {}
    src = cp.get("src") if cp else getattr(ctx, "source", None)
    J = cp.get("J")
    if J is None and cp and not cp.get("on", True):
        try:
            from . import copresence as C
            lm, src = C._landmarks(ctx, C._st(ctx), now)
            J = C.to_world(lm, cp.get("anchor") or C.PERSON) if lm else None
        except Exception:
            J = None
    if src not in PERSON_SRC:
        return None, src
    return J, src


def ee_force(ctx):
    """Largest end-effector force magnitude (N) from P0 physics, or None when nothing measures it."""
    rt = ctx.rt
    f = getattr(rt, "ee_force", None)
    try:
        f = f() if callable(f) else f
    except Exception:
        f = None
    if f is None:
        f = ((ctx.store.get("physics") or {}).get("ee_force")) if isinstance(ctx.store.get("physics"), dict) else None
    if not isinstance(f, dict) or not f:
        return None
    mags = []
    for v in f.values():
        try:
            mags.append(float(np.linalg.norm(np.asarray(v, float)[:3])))
        except (TypeError, ValueError):
            continue
    return max(mags) if mags else None


def _coop(ctx):
    """Cooperating with the person's hands: watch `assist`, `mirror` (their own hand drives the gripper), or a handover
    on its way / being offered."""
    w = ctx.store.get("watch") or {}
    if w.get("mode") in ("assist", "mirror"):
        return True
    t = ctx.tasks.task or {}
    if t.get("action") == "handover" and t.get("status") in ("running", "planning"):
        return True
    ho = (ctx.store.get("copresence") or {}).get("handover") or {}
    return ho.get("phase") in ("carrying", "offering", "given")


def _held(tasks):
    out = []
    for palm, obj in (getattr(tasks, "holding", None) or {}).items():
        if obj and obj in getattr(tasks, "objs", {}):
            o = tasks.objs[obj]
            out.append((palm, tasks.mug_center(obj), float(max(o.get("r", 0.04), o.get("half_h", 0.04))) + 0.005))
    return out


# ---------------------------------------------------------------------- per frame
@ext.tick
def safety_tick(ctx):
    s, now, rt, tasks = _st(ctx), time.time(), ctx.rt, ctx.tasks
    mon = s["mon"]
    if id(rt) != s["rt_id"]:                       # a new world: new geometry, maybe a different profile config
        s["rt_id"], s["prev"], s["v_robot"] = id(rt), None, 0.0
        s["geo"] = S.RobotGeometry(rt.model, _robot_bodies(ctx, rt), dict(getattr(rt, "palm_sid", {}) or {}))
        cfg = _config(ctx)
        if cfg != mon.cfg:
            mon.cfg = cfg
        latched = mon.latched                      # the new body is judged afresh this frame (a force stop stays)
        mon.reset()
        mon.stopped = mon.latched = latched
        s["paused"] = None
    if getattr(tasks, "safety", None) is None:     # the planners' hook (tasks are rebuilt on a world switch)
        tasks.safety = Hook(ctx)
    cfg = mon.cfg
    J, src = _person(ctx, now)
    human = []
    if J is not None:
        pose, _ = pose33(ground({k: np.asarray(v, float) for k, v in J.items()}))
        human = S.human_capsules(pose)
        pts = np.array([p for p in pose if p is not None], float)
        near = ((mon.last or {}).get("closest") or {}).get("p_robot")
        if s["hprev"] is not None and s["hprev"][1].shape == pts.shape and now > s["hprev"][0] and near is not None:
            # the person's speed TOWARD the robot (ISO/TS 15066 uses the directed speed): moving away costs nothing
            u = np.asarray(near, float)[None, :] - pts
            u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-6)
            v = max(0.0, float(np.max(np.einsum("ij,ij->i", pts - s["hprev"][1], u)))) / (now - s["hprev"][0])
            s["v_human"] += 0.35 * (min(v, cfg.v_human_max) - s["v_human"])
        s["hprev"] = (now, pts)
        s["seen"], s["human"], s["src"] = now, human, src
    else:
        s["hprev"] = None
        if (s["src"] in CAMERA_SRC and s["seen"] is not None and now - s["seen"] < cfg.lost_hold_s
                and (mon.stopped or mon.scale < 0.999)):
            human = s["human"]                     # the camera lost you while you were close: assume you're still there
        else:
            s["human"], s["v_human"] = [], 0.0
    held = _held(tasks)
    caps = s["geo"].capsules(rt.data, held)
    mids = 0.5 * (caps["a"] + caps["b"])           # each robot link's own speed (SSM uses the part that approaches)
    if s["prev"] is not None and s["prev"][1].shape == mids.shape and now > s["prev"][0]:
        step = np.linalg.norm(mids - s["prev"][1], axis=1)
        if len(step) and float(step.max()) < TELEPORT:   # home / reset / a new world jump the pose: not a motion
            v = np.minimum(step / (now - s["prev"][0]), 3.0)
            vl = s["v_links"] if isinstance(s.get("v_links"), np.ndarray) and s["v_links"].shape == v.shape else np.zeros_like(v)
            s["v_links"] = vl + 0.5 * (v - vl)
    else:
        s["v_links"] = np.zeros(len(mids))
    s["prev"] = (now, mids)
    vlinks = s["v_links"] if len(s["v_links"]) == len(mids) else np.zeros(len(mids))
    s["v_robot"] = float(vlinks.max()) if len(vlinks) else 0.0
    s["coop"], s["force"], s["person"] = _coop(ctx), ee_force(ctx), bool(human)
    was = mon.stopped
    st = mon.update(caps, human, now, vlinks, s["v_human"], s["coop"], s["force"])
    rt.safety_scale = float(st["speed_scale"])
    st["message"] = S.describe(st, _limb(ctx, st))
    s["state"] = st
    _pause(ctx, s, st, was)


def _limb(ctx, st):
    b = st.get("binding") or st.get("closest") or {}
    if b.get("robot_kind") == "ee":
        side = "left" if str(b.get("robot_ee") or "").startswith("left") else "right"
        try:
            return ctx.tasks.limb(side, "hand")
        except Exception:
            return "hand"
    return "arm" if b.get("robot_kind") == "arm" else "body"


def _pause(ctx, s, st, was):
    """A stop pauses the running task with an honest message; clear again -> the task goes on where it was."""
    tasks, now = ctx.tasks, time.time()
    t = tasks.task
    line = st["message"] or "you're too close"
    if st["stop"]:
        tail = " Say resume once you're clear." if st["latched"] else " I'll go on when you step back."
        if t and t.get("status") == "running":
            p = s["paused"]
            if p is None or p["id"] != t.get("id"):
                s["paused"] = p = {"id": t.get("id"), "message": t.get("message"), "step": t.get("step", "")}
            t["message"] = f"Paused for safety: {line[len('Stopped: '):] if line.startswith('Stopped: ') else line}.{tail}"
            t["step"], t["paused"] = "paused", "safety"
        if not was and now - s["said"] > SPEAK_GAP:
            s["said"] = now
            _say(ctx, line + ".")
        return
    p = s["paused"]
    if p is not None:
        s["paused"] = None
        if t and t.get("id") == p["id"] and t.get("paused"):
            t.pop("paused", None)
            if t.get("status") == "running":
                t["message"], t["step"] = p["message"], p["step"]
                _say(ctx, "You're clear. Carrying on.")


# ---------------------------------------------------------------------- planning hooks
class Hook:
    """tasks.safety: what planners ask before they commit to a point (P2: per trajectory point)."""

    def __init__(self, ctx):
        self.ctx = ctx

    @property
    def scale(self):
        return float(getattr(self.ctx.rt, "safety_scale", 1.0))

    @property
    def stopped(self):
        return _st(self.ctx)["mon"].stopped

    def check_point(self, p, role=None, radius=0.04, coop=None):
        return check_point(self.ctx, p, role, radius, coop)


def check_point(ctx, p, role=None, radius=0.04, coop=None):
    """Is an end effector (`role`: "right" | "left" | palm name) at world point p outside the person's body zone?
    -> {"ok", "d", "part", "region", "why", "role"}. No person in view -> ok. coop: None = the current mode."""
    s = _st(ctx)
    return s["mon"].check_point(p, role, radius, s["coop"] if coop is None else bool(coop))


def _target_points(ctx, msg):
    """World points a pick_place / reach would put the end effector at (no side effects: no markers added)."""
    from .. import tasks as T
    tasks, a = ctx.tasks, msg.get("action")
    out = []
    tgt, uv = msg.get("target"), msg.get("target_uv")
    dest = None
    if isinstance(tgt, (list, tuple)) and len(tgt) == 3:
        dest = np.asarray(tgt, float)
    elif uv is not None and getattr(ctx, "cam_params", None) is not None:
        try:
            dest = tasks.raycast(ctx.cam_params, float(uv[0]), float(uv[1]), getattr(ctx, "aspect", 4 / 3))
        except Exception:
            dest = None
    elif tgt:
        try:
            dest, _ = tasks.resolve(tgt)
        except Exception:
            dest = None
    if dest is not None:
        lift = T.HOVER + 0.04 if a == "reach" else 0.06
        out.append(("target", np.asarray(dest, float) + np.array([0.0, 0.0, lift])))
    if a == "pick_place":
        name = msg.get("object")
        name = name if name in getattr(tasks, "objs", {}) else (tasks.obj_name(name) if name else None)
        if name in getattr(tasks, "objs", {}) and name not in (tasks.holding or {}).values():
            out.append(("object", tasks.mug_center(name)))
    return out


@ext.guard
def safety_first(ctx, msg):
    """Before any other feature sees a task: no new motion during a protective stop, and no target inside you."""
    a = msg.get("action")
    if a not in MOTION or (a == "follow_me" and not msg.get("enabled", True)):
        return None
    s = _st(ctx)
    mon, tid = s["mon"], str(msg.get("id", f"safety-{time.time():.3f}"))
    st = s["state"] or {}
    if mon.stopped:
        line = (st.get("message") or "you're too close").replace("Stopped: ", "")
        ctx.tasks.set_task(tid, a, "failed", f"I can't move right now: {line}. Step back a little and ask again.")
        return True
    if not mon.human or a not in ("pick_place", "reach"):
        return None
    for what, p in _target_points(ctx, msg):
        r = mon.check_point(p, None, 0.04, coop=False)
        if not r["ok"]:
            spot = "that spot is" if what == "target" else "that object is"
            ctx.tasks.set_task(tid, a, "failed", f"I won't reach there: {spot} {r['why']}. Step back, or choose another spot.")
            return True
    return None


# ---------------------------------------------------------------------- messages
@ext.handler("safety")
def on_safety(ctx, msg):
    """The browser can confirm a latched stop. Nothing else: safety is configured on the server."""
    s, op = _st(ctx), msg.get("op")
    if op == "resume":
        mon = s["mon"]
        if not mon.latched:
            ok, text = True, "Nothing to confirm." if not mon.stopped else "I'll go on by myself once you step back."
        else:
            mon.confirm()
            ok, text = True, "Thanks. I'll go on once you're clear."
    else:
        ok, text = False, "Safety is always on. Its distances are set in the server's config, not from the app."
    s["reply"] = {"id": msg.get("id"), "op": op, "ok": ok, "message": text, "t": round(time.time(), 2)}


@ext.state
def safety_state(ctx):
    s = _st(ctx)
    st = dict(s["state"] or s["mon"].state())
    st.setdefault("message", None)
    caps = []
    if s["person"]:
        for h in s["mon"].human:
            caps.append(_r(h["a"]) + _r(h["b"]) + [round(h["r"], 3), h["region"]])
    st.update(person=s["person"], coop=s["coop"], force=None if s["force"] is None else round(s["force"], 1),
              v_robot=round(s["v_robot"], 3), v_human=round(s["v_human"], 3), capsules=caps,
              config=s["mon"].cfg.public(), reply=s["reply"])
    return {"safety": st}


@ext.overlay
def paint_safety(ctx, scn):
    """Robot eye: the person's safety capsules, faint, amber while slowing and red when stopped (only when slowing or
    stopped: the co-presence mannequin already shows the person)."""
    s = _st(ctx)
    st = s["state"] or {}
    if not s["person"] or st.get("zone") not in ("slow", "stop"):
        return
    import mujoco
    rgba = np.array((0.92, 0.26, 0.24, 0.22) if st.get("stop") else (0.96, 0.62, 0.18, 0.16), dtype=np.float32)
    for h in s["mon"].human:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
        a, b = h["a"], h["b"]
        if float(np.linalg.norm(b - a)) < 1e-4:
            b = a + np.array([0.0, 0.0, 1e-4])
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, h["r"], a, b)
        scn.ngeom += 1
