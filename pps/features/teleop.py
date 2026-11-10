"""P2b live teleoperation (docs/vr.md, docs/PROTOCOL.md v10): a VR hand or controller drives one end effector.

{type:"teleop", op:"start", role?, src?, id?}     take over: stops a running path / task / camera follow-me / watch mirror;
                                                 the hand holds where it is until commands arrive
{type:"teleop", role, p:[x,y,z], q?:[w,x,y,z], grip?:0..1, t?:client ms, src?}
                                                 ~30 Hz target for that role's end effector (world frame, metres)
{type:"teleop", op:"stop", id?}                  let go: the hand stays where it is

Every command goes through the same runtime target path P3 mirroring uses (the hand's mocap target), after:
  * clamping: a sphere of the arm's reach around its shoulder, and above the work surface (plus what it carries);
  * rate limiting: at most MAX_SPEED m/s and MAX_TURN rad/s of the robot's time (sim time with physics, so a loaded
    machine running the plant slower than real time doesn't move the target faster than the arm can follow), both
    multiplied by P5's `rt.safety_scale`; with physics the target also waits while the measured hand trails it by more
    than LEAD and is still catching up (a real arm trails a fast target and cuts the corner through whatever is beside
    the path: the G1's open 15 cm gripper knocked the mug over on a move across the table), so the hand goes the way
    the operator points, at the pace the arm can follow;
  * safety: nothing moves during a protective stop (`rt.safety_scale <= 0` or `tasks.safety.stopped`), and a target
    inside the person's body zone (`tasks.safety.check_point`, P5) is not taken. There is no way to bypass this.
`grip` goes to `rt.set_gripper(palm, g)` when the runtime has it (P0); otherwise closing the gripper next to an object
grasps it kinematically (the task layer's grasp) and opening lets go. A bare command (no start) also works: P3's
mirror mode sends those while it copies the person (it only does so while the runtime is free).
Dead man: commands are absolute targets, so when they stop the hand stops at the last one; after IDLE_S without any
command teleop ends by itself.

state.teleop = {active, role, src, rate (commands in the last second), lag_ms (age of the newest command), echo (its
`t`, so a client can measure the round trip), held (why the last command was not applied, or null), clamped,
scale (safety speed scale), hands: {right: {p, q, ee, grip, holding}, ...}, reply: {id, op, ok, message}}
"""
import collections
import time

import numpy as np

from .. import ext
from .. import tasks as tasks_mod
from .. import traj as T

KEY = "teleop"
MAX_SPEED = 1.0          # m/s an end-effector target may move (human hands reach ~1-2 m/s; the IK lags behind anyway)
MAX_TURN = 3.0           # rad/s
REACH = 0.85             # m around the shoulder, when the body doesn't know its arm length
REACH_MOBILE = 1.6       # bodies with a mobile base reach further: the base moves inside the IK
SURFACE_MARGIN = 0.015   # m the end effector stays above the work surface
GRASP_R = 0.07           # m from an object's centre: closing the gripper there holds it (kinematic gripper)
IDLE_S = 3.0             # s without a command -> teleop ends by itself
STALE_S = 0.5            # s without a command -> reported as stale (the hand already holds still)
LEAD = 0.05              # m: physics: how far the target may lead the measured end effector while the arm catches up
CATCH_V = 0.03           # m/s: ... (the hand closing in on the target faster than this is still catching up)


def _r(p, n=4):
    return None if p is None else [round(float(x), n) for x in p]


def _st(ctx):
    s = ctx.store.get(KEY)
    if s is None or s.get("rt") is not ctx.rt:          # first frame, or a new world: start over
        s = {"rt": ctx.rt, "active": False, "role": None, "src": None, "last": 0.0, "times": collections.deque(maxlen=120),
             "echo": None, "held": None, "clamped": False, "grip": {}, "applied": {}, "reply": None, "events": collections.deque(maxlen=6)}
        ctx.store[KEY] = s
    return s


def _reply(s, msg, ok, message):
    s["reply"] = {"id": msg.get("id"), "op": msg.get("op") or "command", "ok": bool(ok), "message": message, "t": round(time.time(), 3)}


def _palm(rt, role):
    role = str(role or "right").replace("_palm", "").lower()
    palm = f"{role}_palm"
    return palm if palm in rt.hands else rt.hands[0]


def _vec(v, n):
    if not isinstance(v, (list, tuple)) or len(v) != n:
        return None
    try:
        a = np.array([float(x) for x in v], float)
    except (TypeError, ValueError):
        return None
    return a if np.all(np.isfinite(a)) else None


# ---------------------------------------------------------------------------- limits
def reach_of(rt, palm):
    """(shoulder xyz, radius) the end effector may go to."""
    sh = getattr(getattr(rt, "retargeter", None), "robot_shoulders", {}) or {}
    c = np.asarray(sh.get(palm, rt.data.site_xpos[rt.palm_sid[palm]]), float)
    if "drive_base" in tuple(getattr(rt, "capabilities", ()) or ()) or "drive_base" in tuple(getattr(getattr(rt, "profile", None), "capabilities", ()) or ()):
        return c, REACH_MOBILE
    arm = getattr(getattr(rt, "retargeter", None), "robot_arm_len", None)
    tcp = float((getattr(rt, "tcp_reach", None) or {}).get(palm, 0.0))   # P0: the gripper's TCP is past the palm
    return c, (float(arm) * 1.02 + tcp if isinstance(arm, (int, float)) and arm > 0.2 else REACH)


def clamp_target(rt, tasks, palm, p):
    """-> (clamped p, clamped?) : inside the reach sphere and above the surface (and what the hand carries)."""
    p = np.asarray(p, float).copy()
    c, R = reach_of(rt, palm)
    v = p - c
    n = float(np.linalg.norm(v))
    clamped = False
    if n > R:
        p, clamped = c + v * (R / n), True
    x0, x1 = tasks_mod.TABLE_X
    y0, y1 = tasks_mod.TABLE_Y
    if x0 - 0.03 <= p[0] <= x1 + 0.03 and y0 - 0.03 <= p[1] <= y1 + 0.03:
        floor = float(tasks_mod.TABLE_TOP) + SURFACE_MARGIN
        held = (getattr(tasks, "holding", None) or {}).get(palm)
        if held in getattr(tasks, "objs", {}):           # the carried object hangs below the hand
            hand_z = float(rt.data.site_xpos[rt.palm_sid[palm]][2])
            bottom = float(tasks.mug_center(held)[2] - tasks.objs[held]["half_h"])
            floor = float(tasks_mod.TABLE_TOP) + max(0.0, hand_z - bottom) + 0.005
        if p[2] < floor:
            p[2], clamped = floor, True
    return p, clamped


def over_edge(rt, palm, want):
    """P0 physics: a hand below the table top that is sent over the table goes up beside the table first (and, when it is
    already under the table, back out from under it): the planner keeps a real arm off the table, so a straight line
    would leave it caught under the edge. -> the waypoint to head for now (want itself when the way is clear)."""
    if getattr(rt, "plant", None) is None:
        return want
    x0, x1 = tasks_mod.TABLE_X
    y0, y1 = tasks_mod.TABLE_Y
    top = float(tasks_mod.TABLE_TOP)
    over = x0 - 0.03 <= want[0] <= x1 + 0.03 and y0 - 0.03 <= want[1] <= y1 + 0.03 and want[2] > top
    ee = np.asarray(rt.data.site_xpos[rt.palm_sid[palm]], float)
    if not over or ee[2] >= top + 0.03:
        return want
    if ee[0] > x0 - 0.04 and y0 - 0.04 <= ee[1] <= y1 + 0.04:   # over the table's footprint:
        if ee[2] >= top:                                         # on it (just above the surface): nothing in the way
            return want
        return np.array([x0 - 0.08, ee[1], min(ee[2], top - 0.04)])   # under it: back out from under the edge
    return np.array([ee[0], ee[1], top + 0.08])                 # beside it: straight up, then over


def robot_clock(rt, now):
    """The clock the hand target's speed limits run on: with physics the robot's own (sim) time, which runs slower
    than the wall clock when a busy machine caps the plant's substeps (rtf < 1: plan and plant share that clock, P0);
    a wall-clock limit would then move the target faster than the arm can follow in its time, so it trails, overshoots
    and sweeps through what is near the path. Kinematic: the wall clock."""
    pl = getattr(rt, "plant", None)
    return float(pl.d.time) if pl is not None else float(now)


def step_dt(rt, clock, last):
    """Time since the last applied command on that clock (capped at 0.1 s). Physics: two commands in one server frame
    share that frame's robot time (the second moves nothing more), and a scene reset (the clock restarts) gives 0."""
    if getattr(rt, "plant", None) is not None:
        return float(np.clip(clock - last, 0.0, 0.1)) if last is not None else 1 / 30
    return float(np.clip(clock - (clock - 1 / 30 if last is None else last), 1 / 120, 0.1))


def leash(ee, cur, new, closing, lead=LEAD, catching=CATCH_V):
    """Physics: hold the target while the arm is still catching up with it. The target goes on from `cur` toward `new`
    unless it already leads the measured end effector `ee` by more than `lead` and the hand is closing in on it faster
    than `catching` m/s (a trailing arm: going on would make it cut the corner). A hand that has stopped short of the
    target (an orientation or a joint limit the IK can't meet) doesn't hold it back, and moving the target back toward
    the hand is always allowed. -> (target, held?)"""
    ee, cur, new = (np.asarray(v, float) for v in (ee, cur, new))
    lag = float(np.linalg.norm(cur - ee))
    if lag > lead and closing is not None and closing > catching and float(np.linalg.norm(new - ee)) > lag:
        return cur, True
    return new, False


def safety_hold(ctx, palm, p):
    """None when the target may be taken, else why not (P5: protective stop, or a target inside the person)."""
    rt, tasks = ctx.rt, ctx.tasks
    scale = float(getattr(rt, "safety_scale", 1.0))
    hook = getattr(tasks, "safety", None)
    if scale <= 0.0 or bool(getattr(hook, "stopped", False)):
        return "safety stop: I hold still while you're this close"
    if hook is not None and hasattr(hook, "check_point"):
        try:
            v = hook.check_point([float(x) for x in p], palm.replace("_palm", ""))
        except Exception:
            v = None
        if isinstance(v, dict) and v.get("ok") is False:
            return "that would put my hand into you" + (f" ({v['why']})" if v.get("why") else "")
    return None


# ---------------------------------------------------------------------------- gripper
def _graspable(hand, c, r, hh):
    try:
        from .traj import graspable
        if graspable(hand, c, r, hh):
            return True
    except Exception:
        pass
    return float(np.linalg.norm(np.asarray(hand) - np.asarray(c))) < GRASP_R


def _gripper(ctx, s, palm, g):
    if g is None:
        return
    rt, tasks = ctx.rt, ctx.tasks
    g = float(np.clip(g, 0.0, 1.0))
    if hasattr(rt, "set_gripper"):                       # P0: a real gripper grasps by contact
        rt.set_gripper(palm, g)
        s["grip"][palm] = g
        return
    closed = g >= 0.5
    prev = s["grip"].get(palm)
    s["grip"][palm] = 1.0 if closed else 0.0
    held = tasks.holding.get(palm)
    if closed and not held and (prev is None or prev < 0.5):
        ee = rt.data.site_xpos[rt.palm_sid[palm]]
        best = min(((float(np.linalg.norm(tasks.mug_center(n) - ee)), n) for n in tasks.objs
                    if n not in tasks.holding.values()
                    and _graspable(ee, tasks.mug_center(n), tasks.objs[n]["r"], tasks.objs[n]["half_h"])), default=None)
        if best:
            tasks.cur = best[1]
            tasks._event("grasp", palm)
            s["events"].append(f"grasped the {best[1].replace('_', ' ')}")
    elif not closed and held:
        tasks._event("release", palm)
        s["events"].append(f"let go of the {held.replace('_', ' ')}")


# ---------------------------------------------------------------------------- ops
def _stop_others(ctx):
    """Teleop takes the hand: stop a running path / task / drive, camera follow-me and P3 mirroring."""
    rt, tasks = ctx.rt, ctx.tasks
    t = tasks.task or {}
    if rt.mode in ("FOLLOW", "PREVIEW") or t.get("status") in ("running", "planning") or getattr(rt, "drive_goal", None) is not None:
        ctx.dispatch({"type": "task", "id": f"teleop-stop-{int(time.time() * 1000) % 100000}", "action": "stop"})
    w = ctx.store.get("watch") or {}
    if w.get("mode") in ("mirror", "assist") and "watch" in ext.HANDLERS:
        ext.handle(ctx, {"type": "watch", "op": "mode", "mode": "off", "id": "teleop"})
    rt.engaged = False                                   # the camera's follow-me would overwrite the hand targets
    if rt.mode not in ("FOLLOW", "PREVIEW", "REC"):
        rt.mode = "MIRROR"


def _orient(rt, on):
    """P0 physics: the operator turns the gripper to grasp, so its orientation is tracked firmly (as in P0's picks)."""
    if hasattr(rt, "orient"):
        rt.orient(on)


def op_start(ctx, s, msg):
    rt = ctx.rt
    _stop_others(ctx)
    _orient(rt, True)
    palm = _palm(rt, msg.get("role") or s.get("role"))
    s.update(active=True, role=palm.replace("_palm", ""), src=str(msg.get("src") or "vr")[:16], last=time.time(), held=None,
             clamped=False)
    s["applied"].pop(palm, None)
    hand = ctx.tasks.limb(s["role"]) if hasattr(ctx.tasks, "limb") else f"{s['role']} hand"
    _reply(s, msg, True, f"Teleop on: your hand drives my {hand}.")


def op_stop(ctx, s, msg):
    was = s["active"]
    if was:
        _orient(ctx.rt, False)
    s.update(active=False, held=None)
    s["applied"].clear()
    _reply(s, msg, True, "Teleop off. I'm holding this pose." if was else "Teleop was already off.")


def command(ctx, s, msg):
    rt, tasks = ctx.rt, ctx.tasks
    p = _vec(msg.get("p"), 3)
    if p is None:
        _reply(s, msg, False, "A teleop command needs p: [x, y, z] in metres.")
        return
    q = _vec(msg.get("q"), 4) if msg.get("q") is not None else None
    grip = msg.get("grip")
    try:
        grip = None if grip is None else float(grip)
    except (TypeError, ValueError):
        grip = None
    palm = _palm(rt, msg.get("role") or s.get("role"))
    now = time.time()
    s["times"].append(now)
    s["last"], s["echo"] = now, msg.get("t")
    if not s["active"]:                                  # a bare command (P3 mirroring, scripts): implicit start
        s.update(active=True, src=str(msg.get("src") or "message")[:16])
        _orient(rt, True)
    s["role"] = palm.replace("_palm", "")
    if rt.mode not in ("MIRROR", "REC"):               # REC: the runtime records what the hand does (P1 can too)
        s["held"] = "busy: I'm running a task (say stop, or start teleop again)"
        return
    want, clamped = clamp_target(rt, tasks, palm, p)
    why = safety_hold(ctx, palm, want)
    clock = robot_clock(rt, now)
    if why:
        s["held"], s["clamped"] = why, clamped
        s["applied"][palm] = clock
        return
    d, mid = rt.data, rt.hands_mid[palm]
    want = over_edge(rt, palm, want)
    scale = float(np.clip(float(getattr(rt, "safety_scale", 1.0)), 0.0, 1.0))
    dt = step_dt(rt, clock, s["applied"].get(palm))
    s["applied"][palm] = clock
    cur = d.mocap_pos[mid].copy()
    delta = want - cur
    n = float(np.linalg.norm(delta))
    step = MAX_SPEED * dt * scale
    new = want if n <= step else cur + delta * (step / n)
    limited = n > step
    if getattr(rt, "plant", None) is not None:          # physics: on a leash to the real hand (see LEAD)
        ee = np.asarray(d.site_xpos[rt.palm_sid[palm]], float).copy()
        prev = s.setdefault("ee", {}).get(palm)
        closing = None
        if prev is not None and clock > prev[0]:
            v = (ee - prev[1]) / (clock - prev[0])
            u = cur - ee
            closing = float(np.dot(v, u / (float(np.linalg.norm(u)) or 1.0)))
        if prev is None or clock != prev[0]:
            s["ee"][palm] = (clock, ee)
        new, leashed = leash(ee, cur, new, closing)
        limited = limited or leashed
    d.mocap_pos[mid] = new
    if q is not None:
        q = T.qnorm(q)
        q0 = T.qnorm(d.mocap_quat[mid].copy())
        ang = T.qangle(q0, q)
        turn = MAX_TURN * dt * scale
        d.mocap_quat[mid] = q if ang <= turn else T.slerp(q0, q, turn / ang)
    _gripper(ctx, s, palm, grip)
    s["held"], s["clamped"] = None, bool(clamped)
    s["limited"] = limited


OPS = {"start": op_start, "stop": op_stop}


@ext.handler("teleop")
def on_teleop(ctx, msg):
    s = _st(ctx)
    op = msg.get("op")
    if op in OPS:
        OPS[op](ctx, s, msg)
    elif op in (None, "", "command", "move"):
        command(ctx, s, msg)
    else:
        _reply(s, msg, False, f"Unknown teleop op '{op}'. Use start, stop, or send {{role, p, q, grip}}.")


@ext.tick
def teleop_tick(ctx):
    s = _st(ctx)
    if s["active"] and s["last"] and time.time() - s["last"] > IDLE_S:
        _orient(ctx.rt, False)
        s.update(active=False, held=None)
        s["applied"].clear()


@ext.state
def teleop_state(ctx):
    s = _st(ctx)
    rt, tasks = ctx.rt, ctx.tasks
    now = time.time()
    hands = {}
    d = rt.data
    for palm in getattr(rt, "hands", []):
        mid = rt.hands_mid[palm]
        g = s["grip"].get(palm)
        hands[palm.replace("_palm", "")] = {"p": _r(d.mocap_pos[mid]), "q": _r(d.mocap_quat[mid]), "ee": _r(d.site_xpos[rt.palm_sid[palm]]),
                                            "grip": None if g is None else round(float(g), 2),
                                            "holding": (getattr(tasks, "holding", None) or {}).get(palm)}
    rate = sum(1 for x in s["times"] if now - x <= 1.0)
    return {KEY: {"active": bool(s["active"]), "role": s["role"], "src": s["src"], "rate": rate,
                  "lag_ms": None if not s["last"] else int(round((now - s["last"]) * 1000)), "stale": bool(s["active"] and now - s["last"] > STALE_S),
                  "echo": s["echo"], "held": s["held"], "clamped": bool(s["clamped"]),
                  "scale": round(float(getattr(rt, "safety_scale", 1.0)), 2), "hands": hands, "reply": s["reply"],
                  "events": list(s["events"])[-3:]}}
