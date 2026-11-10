"""Watching the person (v1 P3, issue #31): the robot understands what you do and acts on it while it watches you.

The person model (pps/human: pose, hands, held tool, speed, activity, phase, reps) is updated every frame from
co-presence's world joints, and one watch mode decides what the robot does with it:

* `mirror`  copy the person live. The G1 retargets the whole body (the existing follow-me); every other body follows
            the person's hand: the wrist's offset from their shoulder, mirrored face to face (they reach toward the
            robot -> the robot reaches toward them; their right hand on the robot's left), moves the end effector from
            where it was when mirroring started; the hand's yaw turns the gripper; pinch / open hand drives the gripper
            (closing next to an object grasps it). Pauses while a task runs. Two-arm bodies follow both hands.
* `learn`   record a demonstration (the working hand's path, palm, pinch, held tool, activity and reps, the objects) and
            save it to recordings/demos/<name>.json for skill extraction (P4). `replay` turns it into a proposed path
            (plan card) placed on the object it was demonstrated over.
* `respond` run rules {when, do}: `when` is a predicate over state.human (pps/human/rules.py: built-in terms such as
            hand_open_near_robot, holding:<tool>, activity:<name>, phase:<name>, reps>=N, stalled, or an expression the
            agent compiled from free text), `do` a task message, a sentence to say, a mode, or an instruction for the
            agent. A rule fires on the rising edge after the condition held for `hold` s, then waits `cooldown` s and
            for the condition to clear before it can fire again, and never while its own task is still running.
* `assist`  follow the person (default: below their hands, as when spotting a bar) and, when they stall mid-rep,
            push up a little with a capped force (the force request is in state.watch.assist for P0 impedance control).
* `coach`   count reps and compare each one with a reference (built-in form rules, or a recorded demo): squat depth,
            knee valgus, back angle, tempo; curl range and elbow drift; press lockout; weave width, frequency and
            travel speed. One short spoken cue per rep at most.

Rules are also evaluated in `assist` and `coach` (not in mirror/learn, where the hands are busy being copied).

Messages ({type: "watch", ...}):
  op:"mode", mode, hand?: "auto"|"left"|"right", target?: "hands"|<object>, max_force?, name? (learn)
  op:"rule", when, do, expr?, cooldown?, hold?, preview?, id?   |  op:"rule", text: "when I put my hand out, give me the wrench"
  op:"rule_remove", id   op:"rules_clear"
  op:"learn_start", name?   op:"learn_stop", name?   op:"replay", name?, preview? (default true)
  op:"tool", name (the person says what they hold; null clears)   op:"coach_ref", demo? | activity?
  op:"summary", on (opt-in VLM description)   op:"reset_reps"
  {type: "hands", hands: [{uv: [u, v], lm: [[x, y, z] * 9], score}]}  MediaPipe Hands from the browser camera
Every op answers in state.watch.reply = {id, op, ok, message, ...}.

State: human (pps/human/model.py) and watch = {mode, rules, events, mirror, learn, assist, coach, demos, reply, summary}.
"""
import collections
import math
import time

import numpy as np

from .. import ext
from .. import tasks as T
from ..human import coach as coach_mod
from ..human import demo as demo_mod
from ..human.model import HumanModel, rule_summary
from ..human.rules import Predicate, RuleError, compile_do, compile_when, split_rule
from ..human.vlm import Summarizer

MODES = ("off", "mirror", "learn", "respond", "assist", "coach")
MIRROR_GAIN = 1.0
MAX_STEP = 0.04          # m an end-effector target may move per frame
ARM_REACH = 0.85         # m from the robot's shoulder (clamp; the IK does its best beyond)
GRASP_R = 0.07           # m: closing the hand this close to an object grasps it (kinematic gripper)
RULE_COOLDOWN, RULE_HOLD = 8.0, 0.4
MAX_RULES = 12
SPEAK_GAP = 1.2
ASSIST_BELOW = 0.10      # m below the person's hands (spotting under the bar)
ASSIST_LIFT, ASSIST_V = 0.06, 0.08


def _st(ctx):
    w = ctx.store.get("watch")
    if w is None:
        w = ctx.store["watch"] = {
            "mode": "off", "model": HumanModel(), "human": {"visible": False}, "rules": [], "rid": 0,
            "events": collections.deque(maxlen=24), "rec": None, "last_demo": None, "cal": {}, "mirror": None,
            "g1_follow": False, "grip": {}, "assist": None, "coach": None, "coach_ref": None, "vlm": Summarizer(),
            "rt_id": None, "opts": {}, "reply": None, "said_t": 0.0, "active_side": "r", "fires": 0}
    return w


def _r(p, n=3):
    return None if p is None else [round(float(x), n) for x in p]


def _say(ctx, text, kind="robot"):
    b = getattr(ctx, "brain", None)
    if b is not None and hasattr(b, "utter") and text:
        try:
            b.utter(text, kind)
        except Exception:
            pass


def _event(w, kind, text, **kw):
    e = {"t": round(time.time(), 2), "kind": kind, "text": str(text)[:200]}
    e.update(kw)
    w["events"].append(e)
    return e


def _reply(w, msg, ok, message, **kw):
    w["reply"] = dict({"id": msg.get("id"), "op": msg.get("op"), "ok": bool(ok), "message": message}, **kw)


def _is_g1(rt):
    return getattr(rt, "profile_id", "g1") == "g1" and "follow_me" in tuple(getattr(rt, "capabilities", ()))


def _palms(rt):
    return list(getattr(rt, "hands", None) or ["right_palm"])


def _objects(tasks):
    out = {}
    for n in getattr(tasks, "objs", {}):
        try:
            out[n] = tasks.mug_center(n)
        except Exception:
            pass
    return out


def _task_busy(tasks, tid):
    t = tasks.task or {}
    return t.get("id") == tid and t.get("status") in ("planning", "running", "proposed", "asking")


def _run_task(ctx, task, tid, preview=False):
    m = dict(task, type="task", id=tid)
    if preview:
        m["preview"] = True
    if not ext.intercept_task(ctx, m):
        ctx.dispatch(m)
    return m


# ---------------------------------------------------------------------- per frame
@ext.tick
def watch_tick(ctx):
    w, now = _st(ctx), time.time()
    rt, tasks = ctx.rt, ctx.tasks
    cp = ctx.store.get("copresence") or {}
    if id(rt) != w["rt_id"]:             # a new world (robot switch): calibrations belong to the old body
        w["rt_id"], w["cal"], w["grip"], w["g1_follow"] = id(rt), {}, {}, False
    J, src = cp.get("J"), cp.get("src")
    ray = cp.get("point")
    held = [o for o in getattr(tasks, "holding", {}).values() if o]
    m = w["model"]
    h = m.update(J, src, now, objects=_objects(tasks), robot_held=held,
                 detected=(getattr(rt, "last_context", None) or {}).get("holding"), pointing=bool(ray and ray.get("armed")),
                 offer=cp.get("offer"), hand_open=cp.get("hand_open"), pts=getattr(ctx, "pts", None),
                 near_x=T.TABLE_X[1] + 0.15)
    w["human"] = h
    # Real camera (#37): the pose's hand points are knuckles, too coarse for open / closed; when MediaPipe Hands in the
    # browser sees a hand, its 21-point reading decides co-presence's hand_open too (state.copresence.hand_open).
    if isinstance(cp.get("hand_open"), dict):
        for side, hd in (h.get("hands") or {}).items():
            if hd.get("src") == "hands" and hd.get("open") is not None:
                cp["hand_open"][side[0]] = bool(hd["open"])
    for kind, val in m.events:
        if kind == "activity" and val not in ("none", "moving", "still") and w["mode"] != "off":
            _event(w, "activity", f"{val}")
    mode = w["mode"]
    if mode == "mirror":
        _mirror(ctx, w, h, now)
    elif mode == "learn":
        rec = w["rec"]
        if rec is not None:
            rec.add(now, h, [v for k, v in m.events if k == "rep"])
            if rec.seconds > demo_mod.MAX_S:
                _set_mode(ctx, w, "off", {})
    elif mode == "assist":
        _assist(ctx, w, h, now)
    elif mode == "coach":
        _coach(ctx, w, h, m.events, now)
    if mode in ("respond", "assist", "coach"):
        _rules(ctx, w, h, now)
    if mode != "off":
        vlm = w["vlm"]                      # opt-in, throttled, background thread (never blocks this loop)
        vlm.maybe_run(getattr(ctx, "frame", None), rule_summary(h), src, mock=bool(getattr(ctx.brain, "enabled", False)), now=now)
        if vlm.last and (m.summary_vlm is None or vlm.last[0] > m.summary_vlm[0]):
            m.summary_vlm = vlm.last


@ext.pre_task
def watch_sees_stop(ctx, msg):
    """'Stop' also stops the robot copying or following you (it would move again on the next frame otherwise)."""
    if msg.get("action") == "stop":
        w = _st(ctx)
        if w["mode"] in ("mirror", "assist"):
            _set_mode(ctx, w, "off", {}, quiet=True)
            _event(w, "mode", "Stopped watching: you said stop.")
    return None


# ---------------------------------------------------------------------- driving the robot's hands
def _drive_hand(ctx, palm, p, q=None, grip=None):
    """Set one end-effector target. Uses the P2 `teleop` message when that feature is present, else the hand's mocap
    target (what the task layer and the 3D ghost drag use)."""
    if "teleop" in ext.HANDLERS:
        ext.handle(ctx, {"type": "teleop", "role": palm.replace("_palm", ""), "p": _r(p, 4), "q": _r(q, 5), "grip": grip})
        return
    rt = ctx.rt
    mid = rt.hands_mid[palm]
    rt.data.mocap_pos[mid] = p
    if q is not None:
        rt.data.mocap_quat[mid] = q


def _gripper(ctx, w, palm, closed):
    """Pinch / fist -> gripper. P0's physical gripper when the runtime has one (rt.set_gripper); otherwise the
    kinematic grasp of the task layer: closing next to an object holds it, opening lets go."""
    if closed is None:
        return
    prev = w["grip"].get(palm)
    w["grip"][palm] = closed
    rt, tasks = ctx.rt, ctx.tasks
    if hasattr(rt, "set_gripper"):
        rt.set_gripper(palm, 1.0 if closed else 0.0)
        return
    if prev is None or prev == closed:
        return
    held = tasks.holding.get(palm)
    if closed and not held:
        ee = rt.data.site_xpos[rt.palm_sid[palm]]
        best = min(((float(np.linalg.norm(c - ee)), n) for n, c in _objects(tasks).items()
                    if n not in tasks.holding.values()), default=None)
        if best and best[0] < GRASP_R:
            tasks.cur = best[1]
            tasks._event("grasp", palm)
            _event(w, "mirror", f"Grasped the {best[1].replace('_', ' ')} with you.")
    elif not closed and held:
        tasks._event("release", palm)
        _event(w, "mirror", f"Let go of the {held.replace('_', ' ')}.")


def _yaw_quat(dyaw, q0):
    c, s = math.cos(dyaw / 2), math.sin(dyaw / 2)
    qz = np.array([c, 0.0, 0.0, s])
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = qz
    return np.array([w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0, w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
                     w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0, w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0])


def _facing_yaw(J):
    across = np.asarray(J["l_shoulder"]) - np.asarray(J["r_shoulder"])
    f = np.cross(across, [0.0, 0.0, 1.0])
    return math.atan2(f[1], f[0]) if np.linalg.norm(f[:2]) > 1e-6 else math.pi


def person_offset(J, sd):
    """The person's wrist relative to their shoulder, as the robot copies it face to face (mirror image):
    expressed as if they faced the robot squarely (-x), then reflected so 'toward the robot' is the robot's +x."""
    d = np.asarray(J[f"{sd}_wrist"], float) - np.asarray(J[f"{sd}_shoulder"], float)
    a = math.pi - _facing_yaw(J)
    c, s = math.cos(a), math.sin(a)
    dc = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
    return np.array([-dc[0], dc[1], dc[2]])


def _hand_yaw(J, sd):
    wr = np.asarray(J[f"{sd}_wrist"], float)
    tip = [np.asarray(J[k], float) for k in (f"{sd}_index", f"{sd}_pinky") if k in J]
    v = (np.mean(tip, axis=0) - wr) if tip else (wr - np.asarray(J[f"{sd}_elbow"], float))
    return math.atan2(v[1], v[0])


def _mirror_pairs(rt, J, w):
    """[(person side, robot palm)]. Two-handed bodies: the person's right hand (on the robot's left, +y) drives the
    robot's left hand, as in a mirror. One arm: the person's working hand (the one raised / held out more, sticky)."""
    palms = _palms(rt)
    if "left_palm" in palms and "right_palm" in palms:
        return [("r", "left_palm"), ("l", "right_palm")]
    pref = w["opts"].get("hand", "auto")
    if pref in ("left", "right"):
        return [(pref[0], palms[0])]

    def score(sd):
        if f"{sd}_wrist" not in J:
            return -9.0
        o = person_offset(J, sd)
        return float(o[0] + 0.6 * o[2])          # forward and up
    cur = w["active_side"]
    other = "l" if cur == "r" else "r"
    if score(other) > score(cur) + 0.15:
        w["active_side"] = cur = other
    return [(cur, palms[0])]


def _clamp(rt, palm, p):
    p = np.asarray(p, float).copy()
    sh = np.asarray(getattr(rt.retargeter, "robot_shoulders", {}).get(palm, p), float)
    v = p - sh
    n = float(np.linalg.norm(v))
    if n > ARM_REACH:
        p = sh + v * (ARM_REACH / n)
    if T.TABLE_X[0] - 0.05 <= p[0] <= T.TABLE_X[1] + 0.05 and T.TABLE_Y[0] - 0.05 <= p[1] <= T.TABLE_Y[1] + 0.05:
        p[2] = max(p[2], T.TABLE_TOP + 0.04)
    return p


def _step_toward(cur, want, max_step=MAX_STEP):
    d = want - cur
    n = float(np.linalg.norm(d))
    return want if n <= max_step else cur + d * (max_step / n)


def _mirror(ctx, w, h, now):
    rt, J = ctx.rt, w["model"].J
    if _is_g1(rt):                        # the humanoid: whole-body retargeting (existing follow-me)
        if not w["g1_follow"] and rt.mode not in ("REC", "FOLLOW", "PREVIEW"):
            _run_task(ctx, {"action": "follow_me", "enabled": True}, "watch-mirror")
            w["g1_follow"] = True
        w["mirror"] = {"kind": "whole_body", "robot_hand": None, "paused": rt.mode != "REC", "person": h.get("visible")}
        return
    if J is None or rt.mode != "MIRROR":
        w["cal"] = {}                     # re-anchor when the person comes back or a task is done
        w["mirror"] = dict(w["mirror"] or {}, paused=True)
        return
    d = rt.data
    info = {"kind": "hands", "paused": False, "pairs": []}
    for sd, palm in _mirror_pairs(rt, J, w):
        if f"{sd}_wrist" not in J or f"{sd}_shoulder" not in J:
            continue
        off = person_offset(J, sd)
        mid = rt.hands_mid[palm]
        cal = w["cal"].get((sd, palm))
        if cal is None:
            cal = w["cal"][(sd, palm)] = {"off0": off, "e0": d.mocap_pos[mid].copy(), "q0": d.mocap_quat[mid].copy(),
                                          "yaw0": _hand_yaw(J, sd)}
        want = _clamp(rt, palm, cal["e0"] + MIRROR_GAIN * (off - cal["off0"]))
        p = _step_toward(d.mocap_pos[mid].copy(), want)
        turn = (_hand_yaw(J, sd) - cal["yaw0"] + math.pi) % (2 * math.pi) - math.pi
        dyaw = float(np.clip(-turn, -1.2, 1.2))          # a mirror image turns the other way
        q = _yaw_quat(dyaw, cal["q0"])
        hand = (h.get("hands") or {}).get("right" if sd == "r" else "left") or {}
        closed = None
        if hand.get("src") == "hands" and hand.get("pinch") is not None:
            closed = hand["pinch"] < 0.035
        elif hand.get("open") is not None:
            closed = not hand["open"]
        _drive_hand(ctx, palm, p, q, None if closed is None else (1.0 if closed else 0.0))
        _gripper(ctx, w, palm, closed)
        ee = d.site_xpos[rt.palm_sid[palm]]
        info["pairs"].append({"person": "right" if sd == "r" else "left", "robot": palm.replace("_palm", ""),
                              "target": _r(p), "ee": _r(ee), "grip": None if closed is None else int(closed),
                              "yaw_deg": round(math.degrees(dyaw))})
    w["mirror"] = info


# ---------------------------------------------------------------------- assist
def _assist(ctx, w, h, now):
    rt, J = ctx.rt, w["model"].J
    a = w["assist"] or {}
    if J is None or rt.mode != "MIRROR":
        a.update(phase="idle" if J is None else "paused", request=None)
        w["assist"] = a
        return
    tgt = w["opts"].get("target") or "hands"
    tasks = ctx.tasks
    if tgt != "hands" and tgt in getattr(tasks, "objs", {}):
        focus = tasks.mug_center(tgt) + np.array([0.0, 0.0, 0.05])
    else:
        ws = [np.asarray(J[k], float) for k in ("l_wrist", "r_wrist") if k in J]
        if not ws:
            return
        focus = np.mean(ws, axis=0) - np.array([0.0, 0.0, ASSIST_BELOW])
    dt = min(0.1, now - a.get("t", now)) if a.get("t") else 0.0
    lift = float(a.get("lift", 0.0))
    if h.get("stalled"):
        if a.get("phase") != "helping":
            mf = float(w["opts"].get("max_force") or 40.0)
            _event(w, "assist", f"You stalled: helping, capped at {mf:.0f} N.")
            _say(ctx, "I've got you. Push.")
        lift = min(ASSIST_LIFT, lift + ASSIST_V * dt)
        phase = "helping"
    else:
        lift = max(0.0, lift - ASSIST_V * dt)
        phase = "following"
    focus = focus + np.array([0.0, 0.0, lift])
    palms = _palms(rt)
    spread = {"left_palm": 0.18, "right_palm": -0.18} if len(palms) > 1 else {palms[0]: 0.0}
    d = rt.data
    ees = {}
    for palm in palms:
        want = _clamp(rt, palm, focus + np.array([0.0, spread.get(palm, 0.0), 0.0]))
        p = _step_toward(d.mocap_pos[rt.hands_mid[palm]].copy(), want, 0.02)
        _drive_hand(ctx, palm, p)
        ees[palm.replace("_palm", "")] = _r(d.site_xpos[rt.palm_sid[palm]])
    mf = float(w["opts"].get("max_force") or 40.0)
    w["assist"] = {"phase": phase, "target": tgt, "focus": _r(focus), "lift": round(lift, 3), "t": now, "ee": ees,
                   "max_force": mf, "request": {"force_n": mf, "dir": [0.0, 0.0, 1.0]} if phase == "helping" else None}


# ---------------------------------------------------------------------- coach
def _coach(ctx, w, h, events, now):
    if h.get("src") == "synthetic":        # the no-camera dancer isn't training
        return
    c = w["coach"]
    if c is None:
        c = w["coach"] = {"activity": None, "reps": 0, "last": None, "cues": {}, "history": [], "ref": None, "weave_t": 0.0}
    act = h.get("activity")
    if act in ("squat", "curl", "press", "weave") and act != c["activity"]:
        c["activity"], c["cues"], c["history"] = act, {}, []
        if h.get("src") != "synthetic":
            _say(ctx, {"squat": "Squats. I'll count.", "curl": "Curls. I'll count.", "press": "Presses. I'll count.",
                       "weave": "I'm watching your weave."}[act])
    custom = w["coach_ref"]
    ref = coach_mod.reference(act, custom)
    c["ref"] = {"activity": act, "source": (custom or {}).get("source") or "built-in"}
    c["reps"] = h.get("reps") or 0
    for kind, st in events:
        if kind != "rep" or st.get("activity") not in ("squat", "curl", "press", "weave"):
            continue
        keys = coach_mod.cues(st, ref)
        if st["activity"] == "weave":            # a weave "rep" is one side-to-side cycle: judge every few seconds
            if now - c["weave_t"] < 4.0:
                continue
            c["weave_t"] = now
        for k in keys:
            c["cues"][k] = c["cues"].get(k, 0) + 1
        line = coach_mod.say(keys)
        n = st.get("rep")
        c["last"] = {"rep": n, "cues": keys, "text": line}
        c["history"] = (c["history"] + [{"rep": n, "cues": keys}])[-20:]
        if keys:
            _event(w, "cue", f"rep {n}: {line}" if st["activity"] != "weave" else line)
        # count every rep aloud; a cue is said the first time and then every third rep it repeats (a coach, not a nag)
        fresh = [k for k in keys if c["cues"][k] % 3 == 1]
        said = coach_mod.say(fresh)
        spoken = (f"{n}. {said}" if st["activity"] != "weave" else said).strip()
        if spoken and h.get("src") != "synthetic" and now - w["said_t"] > SPEAK_GAP:
            w["said_t"] = now
            _say(ctx, spoken)


# ---------------------------------------------------------------------- respond rules
def _rules(ctx, w, h, now):
    if not h.get("visible") or h.get("src") in ("synthetic", None):
        return                        # the no-camera dancer is not a person asking for anything
    for r in w["rules"]:
        if not r["enabled"]:
            continue
        try:
            v = r["_pred"](h)
        except Exception:
            v = False
        if not v:
            r["since"] = None
            if r["latched"]:
                r["false_since"] = r["false_since"] or now
                if now - r["false_since"] > 0.5:
                    r["latched"], r["false_since"] = False, None
            continue
        r["false_since"] = None
        r["since"] = r["since"] or now
        if r["latched"] or now - r["since"] < r["hold"] or now - r["last"] < r["cooldown"]:
            continue
        if r.get("busy") and _task_busy(ctx.tasks, r["busy"]):
            continue
        _fire(ctx, w, r, h, now)


def _fire(ctx, w, r, h, now):
    r["latched"], r["last"] = True, now
    r["fired"] += 1
    w["fires"] += 1
    act = r["act"]
    what = r["when_text"] or r["when"] or r["expr"]
    if "task" in act:
        tid = f"watch-{r['id']}-{r['fired']}"
        r["busy"] = tid
        _run_task(ctx, act["task"], tid, preview=r.get("preview", False))
        _event(w, "rule", f"{what} → {r['do']}", rule=r["id"], task=tid)
    elif "say" in act:
        _say(ctx, act["say"])
        _event(w, "rule", f"{what} → said “{act['say']}”", rule=r["id"])
    elif "mode" in act:
        _set_mode(ctx, w, act["mode"], {})
        _event(w, "rule", f"{what} → {act['mode']}", rule=r["id"])
    else:
        agent = ctx.store.get("agent")
        if "agent" in ext.HANDLERS and agent is not None and agent.available() and not agent.busy:
            ext.handle(ctx, {"type": "agent", "text": f"[watch rule {r['id']}: the person just did this: {what}] {r['do']}"})
            _event(w, "rule", f"{what} → asked the agent: {r['do']}", rule=r["id"])
        else:
            _say(ctx, f"You {what}. I'd {r['do']}, but I need the agent brain for that.")
            _event(w, "rule", f"{what} → (no agent) {r['do']}", rule=r["id"])


def add_rule(ctx, w, msg):
    """-> (ok, message, rule)"""
    when, do, expr = msg.get("when"), msg.get("do"), msg.get("expr")
    if msg.get("text") and not (when and do):
        sp = split_rule(msg["text"])
        if not sp:
            return False, "Say it as 'when I …, do …' (e.g. 'when I put my hand out, give me the wrench').", None
        when, do = sp
    task = msg.get("task") if isinstance(msg.get("task"), dict) and msg["task"].get("action") else None
    if not (when or expr) or not (do or task):
        return False, "A rule needs a when and a do.", None
    if len(w["rules"]) >= MAX_RULES:
        return False, f"I keep at most {MAX_RULES} rules; remove one first.", None
    when_text = str(when or expr).strip()
    try:
        if expr:
            pred = Predicate(expr=expr)
        else:
            try:
                pred = Predicate(when=when)
            except RuleError:
                compiled = compile_when(when)
                if not compiled:
                    return False, (f"I can't watch for “{when}” on my own. Known: hand_open_near_robot, holding:<tool>, "
                                   "activity:<squat|curl|press|wave|weave|reach>, phase:<top|down|bottom|up>, reps>=N, stalled, "
                                   "still, pointing (or an expression over state.human)."), None
                pred = Predicate(when=compiled)
        act = {"task": dict(task)} if task else compile_do(do, list(getattr(ctx.tasks, "objs", {})))
    except RuleError as e:
        return False, f"That rule doesn't work: {e}.", None
    if msg.get("id") and str(msg["id"]).startswith("R") and any(r["id"] == msg["id"] for r in w["rules"]):
        w["rules"] = [r for r in w["rules"] if r["id"] != msg["id"]]
    w["rid"] += 1
    r = {"id": f"R{w['rid']}", "when": pred.when, "expr": pred.expr, "when_text": when_text, "do": str(do or task.get("action")),
         "act": act, "cooldown": float(msg.get("cooldown") or RULE_COOLDOWN), "hold": float(msg.get("hold") if msg.get("hold") is not None else RULE_HOLD),
         "preview": bool(msg.get("preview", False)), "enabled": True, "fired": 0, "last": 0.0, "since": None,
         "latched": False, "false_since": None, "busy": None, "_pred": pred}
    w["rules"].append(r)
    _event(w, "rule", f"New rule {r['id']}: when {when_text} → {r['do']}", rule=r["id"])
    return True, f"Okay: when {_you(when_text)}, I'll {_you(r['do'])}.", r


def _you(text):
    """Echo the person's words back: 'I put my hand out' -> 'you put your hand out'."""
    sw = {"i": "you", "me": "you", "my": "your", "mine": "yours", "i'm": "you're", "myself": "yourself", "am": "are"}
    return " ".join(sw.get(x.lower(), x) for x in str(text).split())


def _rule_public(r):
    return {k: r[k] for k in ("id", "when", "expr", "when_text", "do", "cooldown", "hold", "enabled", "fired")} | {
        "kind": next(iter(r["act"])), "task": r["act"].get("task"), "last_age": round(time.time() - r["last"], 1) if r["last"] else None}


# ---------------------------------------------------------------------- modes
def _set_mode(ctx, w, mode, msg, quiet=False):
    prev = w["mode"]
    for k in ("hand", "target", "max_force"):
        if msg.get(k) is not None:
            w["opts"][k] = msg[k]
    if mode == prev and mode != "learn":
        return True, f"Already in {mode}."
    note = ""
    if prev == "learn":
        note = _finish_demo(ctx, w, msg.get("name"))
    if prev == "mirror":
        if w["g1_follow"]:
            _run_task(ctx, {"action": "follow_me", "enabled": False}, "watch-mirror-off")
        w["g1_follow"], w["cal"], w["mirror"] = False, {}, None
    if prev == "assist":
        w["assist"] = None
    w["mode"] = mode
    tasks = ctx.tasks
    if mode == "learn":
        w["rec"] = demo_mod.DemoRecorder(msg.get("name"), getattr(ctx.rt, "profile_id", "g1"), _objects(tasks))
        w["model"].tracker.reset_reps()
    elif mode == "mirror":
        w["cal"] = {}
    elif mode == "coach":
        w["coach"] = None
        w["model"].tracker.reset_reps()
    elif mode == "assist":
        w["assist"] = {"phase": "following", "lift": 0.0}
    text = {"off": "Stopped watching.", "mirror": "Copying you. Move your hand; I'll follow.",
            "learn": "Watching and recording. Show me, then say done.", "respond": "Watching for your cues.",
            "assist": "Following you; I'll help if you get stuck.", "coach": "Coaching: I'll count and correct your form."}[mode]
    _event(w, "mode", text + (" " + note if note else ""))
    if not quiet:
        _say(ctx, (note + " " + text).strip() if prev == "learn" else text)
    return True, (note + " " + text).strip()


def _finish_demo(ctx, w, name=None):
    rec, w["rec"] = w["rec"], None
    if rec is None:
        return ""
    if name:
        rec.name = name
    demo = rec.finish(getattr(ctx.rt, "profile_id", None))
    if demo is None:
        return "I didn't see enough to learn from."
    path = demo_mod.save(demo)
    w["demos_cache"] = None
    w["last_demo"] = {"name": demo["name"], "path": str(path), "duration": demo["duration"], "hand": demo["hand"],
                      "tool": demo["tool"], "activity": demo["activity"], "reps": demo["reps"], "pattern": demo["pattern"],
                      "anchor": demo["anchor"].get("object"), "frames": len(demo["frames"])}
    p = demo["pattern"]
    detail = f", a {p['amp'] * 100:.1f} cm weave at {p['freq']:.1f} Hz" if p else (f", {demo['reps']} reps" if demo["reps"] else "")
    msg = f"Learned '{demo['name']}': {demo['duration']:.1f} s of {demo['activity']} with your {demo['hand']} hand{detail}."
    _event(w, "demo", msg)
    return msg


def replay(ctx, w, name=None, preview=True, tid=None):
    """Last (or named) demo -> its stub trajectory placed on its anchor object (or at the end effector) -> a path,
    then follow_path (as a plan card when preview). -> (ok, message)"""
    name = name or (w["last_demo"] or {}).get("name")
    demo = demo_mod.load(name) if name else None
    if demo is None:
        return False, "I have no demonstration to repeat yet. Say 'watch and learn' and show me."
    tasks, rt = ctx.tasks, ctx.rt
    anchor = (demo.get("anchor") or {}).get("object")
    tid = tid or f"watch-replay-{int(time.time() * 1000) % 100000}"
    if "traj" in ext.HANDLERS:     # P2: the demo keeps its timing, gripper and object frame (pps/features/traj.py)
        ext.handle(ctx, {"type": "traj", "op": "propose" if preview else "set", "run": not preview, "demo": demo["name"],
                         "task_id": tid, "id": tid})
        r = (ctx.store.get("traj") or {}).get("reply") or {}
        if r.get("id") == tid and (r.get("ok") or r.get("plan")):
            where = f"over the {anchor.replace('_', ' ')}" if anchor in getattr(tasks, "objs", {}) else "from where my hand is"
            why = "" if r.get("ok") else " " + (r.get("message") or "")
            return True, f"My version of '{demo['name']}' {where}: {r.get('n')} points, {r.get('duration') or 0:.1f} s.{why}"
    traj = demo.get("trajectory") or demo_mod.to_trajectory(demo)
    palm = _palms(rt)[0]
    start = rt.data.site_xpos[rt.palm_sid[palm]].copy()
    if anchor in getattr(tasks, "objs", {}):
        top = tasks.mug_center(anchor) + np.array([0.0, 0.0, float(tasks.objs[anchor].get("half_h", 0.04))])
        pts = demo_mod.place(traj, top)
    else:
        pts = demo_mod.place(traj, None, start)
    tasks.set_path_world(pts, palm.replace("_palm", ""))
    _run_task(ctx, {"action": "follow_path"}, tid, preview=preview)
    where = f"over the {anchor.replace('_', ' ')}" if anchor in getattr(tasks, "objs", {}) else "from where my hand is"
    return True, f"My version of '{demo['name']}' {where}: {len(pts)} points."


# ---------------------------------------------------------------------- messages
@ext.handler("watch")
def on_watch(ctx, msg):
    w, op = _st(ctx), msg.get("op")
    if op == "mode":
        mode = str(msg.get("mode") or "").lower()
        if mode not in MODES:
            _reply(w, msg, False, f"Unknown mode '{mode}'. Modes: {', '.join(MODES)}.")
            return
        ok, text = _set_mode(ctx, w, mode, msg)
        _reply(w, msg, ok, text, mode=w["mode"], demo=w["last_demo"] if mode != "learn" else None)
    elif op == "rule":
        ok, text, r = add_rule(ctx, w, msg)
        if ok and w["mode"] in ("off", "learn", "mirror"):
            _set_mode(ctx, w, "respond", {}, quiet=True)
        _reply(w, msg, ok, text, rule=_rule_public(r) if r else None)
        if msg.get("speak"):
            _say(ctx, text)
    elif op in ("rule_remove", "remove_rule"):
        rid = str(msg.get("rule") or msg.get("rule_id") or msg.get("id") or "")
        n = len(w["rules"])
        w["rules"] = [r for r in w["rules"] if r["id"] != rid]
        _reply(w, msg, len(w["rules"]) < n, f"Removed rule {rid}." if len(w["rules"]) < n else f"There's no rule {rid}.")
    elif op == "rules_clear":
        w["rules"] = []
        _reply(w, msg, True, "Cleared all rules.")
    elif op == "learn_start":
        ok, text = _set_mode(ctx, w, "learn", msg)
        _reply(w, msg, ok, text, mode="learn")
    elif op == "learn_stop":
        if w["mode"] != "learn":
            _reply(w, msg, False, "I'm not recording a demonstration.", demo=w["last_demo"])
            return
        ok, text = _set_mode(ctx, w, "off", msg)
        _reply(w, msg, w["last_demo"] is not None, text, demo=w["last_demo"])
    elif op == "replay":
        ok, text = replay(ctx, w, msg.get("name"), msg.get("preview", True) is not False, msg.get("task_id"))
        _reply(w, msg, ok, text)
    elif op == "tool":
        name = msg.get("name")
        w["model"].declared = str(name).strip().lower().replace(" ", "_") if name else None
        _reply(w, msg, True, f"Noted: you're holding the {name}." if name else "Noted: empty hands.")
    elif op == "coach_ref":
        ref = None
        if msg.get("demo"):
            d = demo_mod.load(msg["demo"])
            ref = coach_mod.reference_from_demo(d) if d else None
            if ref:
                ref["source"] = f"demo {d['name']}"
        w["coach_ref"] = ref
        _reply(w, msg, ref is not None or not msg.get("demo"),
               f"Coaching against {ref['source']}." if ref else ("I couldn't use that demo as a reference." if msg.get("demo") else "Coaching against the built-in form rules."))
    elif op == "summary":
        w["vlm"].enabled = bool(msg.get("on", True))
        _reply(w, msg, True, "AI description on." if w["vlm"].enabled else "AI description off.")
    elif op == "reset_reps":
        w["model"].tracker.reset_reps()
        _reply(w, msg, True, "Reps reset.")
    else:
        _reply(w, msg, False, f"Unknown watch op '{op}'.")


@ext.handler("hands")
def on_hands(ctx, msg):
    _st(ctx)["model"].set_browser_hands(msg.get("hands"))


def _demos(w):
    """Saved demo names (the directory is read at most every 3 s)."""
    t, names = w.get("demos_cache") or (0.0, [])
    if time.time() - t > 3.0:
        names = demo_mod.listing()[-12:]
        w["demos_cache"] = (time.time(), names)
    return names


@ext.state
def watch_state(ctx):
    w = _st(ctx)
    rec = w["rec"]
    vlm = w["vlm"]
    return {"human": w["human"], "watch": {
        "mode": w["mode"], "rules": [_rule_public(r) for r in w["rules"]], "events": list(w["events"])[-12:],
        "mirror": w["mirror"] if w["mode"] == "mirror" else None,
        "learn": {"recording": rec is not None, "seconds": round(rec.seconds, 1) if rec else 0.0,
                  "frames": len(rec.rows) if rec else 0, "last": w["last_demo"]},
        "assist": w["assist"] if w["mode"] == "assist" else None,
        "coach": ({k: v for k, v in (w["coach"] or {}).items() if k != "weave_t"} if w["mode"] == "coach" else None),
        "hand": w["opts"].get("hand", "auto"), "fires": w["fires"], "reply": w["reply"],
        "summary": {"on": vlm.enabled, "busy": vlm.busy, "error": vlm.error}, "demos": _demos(w)}}
