"""Plan preview (UX 2.0 #B): the robot proposes, you confirm.

A motion task sent with `preview: true` is not run. Instead the robot works out the whole plan with the same code the
real run uses (target resolution, free-spot search, IK reach precheck, carry-through-obstacle check, nearest reachable
spot) and shows it: intent sentence, target, steps, constraints, verdict + reasons, nearest alternative and the
densified hand waypoints with per-point ok flags. Nothing moves until {type:'plan', op:'confirm'}; {op:'cancel'} drops
it. Tasks without `preview` (demo, self test, buttons) are untouched.

state key `plan` (None when idle), task status `proposed` while it waits. See docs/PROTOCOL.md "v4 (UX 2.0)".
"""
import json
import re
import time

import numpy as np

from .. import ext
from ..tasks import HOVER, SKILL_DIR, TABLE_TOP, densify

KEY = "plan"
MOTION = ("pick_place", "reach", "follow_path", "play_skill")
WAITING = ("proposing", "needs_you", "blocked")
MAX_PTS = 90


def _xyz(p):
    return [round(float(x), 3) for x in p]


def _decimate(pts, ok):
    """At most MAX_PTS points for the 3D ghost; keeps the first, last and every unreachable one."""
    n = len(pts)
    if n <= MAX_PTS:
        return [_xyz(p) for p in pts], [bool(o) for o in ok]
    step = n / (MAX_PTS - 1)
    keep = sorted({int(round(k * step)) for k in range(MAX_PTS - 1)} | {n - 1} | {i for i, o in enumerate(ok) if not o})
    keep = [i for i in keep if i < n]
    return [_xyz(pts[i]) for i in keep], [bool(ok[i]) for i in keep]


def _cap(s):
    return s[:1].upper() + s[1:] if s else s


# ---------------------------------------------------------------------------- dry run
def _dry_run(ctx, msg):
    """Run tasks.pick_place / tasks.reach for real, but with _launch stopped before the robot starts following, then put
    every bit of task/runtime state back. Markers the resolver adds (tap target, nearest reachable spot) are kept so the
    confirmed task and the suggested alternative can name them."""
    tasks, rt = ctx.tasks, ctx.rt
    rt_keys = ("path", "path_msg", "mode", "engaged", "follow_s")
    t_keys = ("task", "kind", "events", "fired", "done_msg", "cur", "cur_off", "fail_reason", "hit_obstacle", "stopped_task",
              "place_at", "fxop", "place_z", "transit_z", "place_check", "place_drop", "cur_action", "release_z")
    snap_rt = {k: getattr(rt, k) for k in rt_keys}
    snap_t = {k: getattr(tasks, k) for k in t_keys if hasattr(tasks, k)}
    snap_q = rt.data.mocap_quat.copy()            # physics plans turn the hand target to the grasp orientation
    grips = {r: g["cmd"] for r, g in tasks.plant.grip.items()} if getattr(tasks, "plant", None) is not None else {}
    before = {id(mk) for mk in tasks.markers}
    cap, caps = {}, []
    real_launch, real_hits = type(tasks)._launch, type(tasks).carry_hits_vase

    def hits(pts, events, palm):
        r = real_hits(tasks, pts, events, palm)
        cap["hit"] = r
        cap["hit_obstacle"] = getattr(tasks, "hit_obstacle", None)
        return r

    def launch(tid, action, keypoints, palm, events, kind, ok_msg, **kw):
        cap.update(keypoints=[np.asarray(k, float).copy() for k in keypoints], palm=palm, events=dict(events),
                   ok_msg=ok_msg, done_msg=getattr(tasks, "done_msg", ""), cur=tasks.cur, cur_off=getattr(tasks, "cur_off", None))
        ok = real_launch(tasks, tid, action, keypoints, palm, events, kind, ok_msg, **kw)
        cap["dists"] = list(getattr(rt, "last_precheck_dists", []) or [])
        cap["path"] = rt.path
        cap["launched"] = ok
        cap["why"] = getattr(tasks, "fail_reason", "reach") if not ok else None
        caps.append(dict(cap))                    # physics tries a few grasps: keep each attempt
        return ok

    tasks._launch, tasks.carry_hits_vase = launch, hits
    rt.recorder.start = lambda *a, **k: None      # _launch starts the recorder; a preview must not wipe a recording
    try:
        a, tid = msg.get("action"), str(msg.get("id", "plan"))
        if a == "pick_place":
            tasks.pick_place(tid, msg.get("object", "mug"), msg.get("target"), msg.get("target_uv"), ctx.cam_params, ctx.aspect)
        elif a in ("open", "close"):              # issue #48 (pps/manip.py): the same code the run uses
            tasks.operate(tid, a, msg.get("fixture") or msg.get("target") or "", msg.get("amount"))
        elif a == "pick_from":
            tasks.pick_from(tid, msg.get("container") or msg.get("target") or "", msg.get("object"))
        elif a == "place_in":
            tasks.place_in(tid, msg.get("object"), msg.get("container") or msg.get("target") or "")
        else:
            tasks.reach(tid, msg.get("target"), msg.get("target_uv"), ctx.cam_params, ctx.aspect)
        verdict = dict(tasks.task or {})
    finally:
        del tasks._launch, tasks.carry_hits_vase
        try:
            del rt.recorder.start
        except AttributeError:
            pass
        for k, v in snap_rt.items():
            setattr(rt, k, v)
        for k, v in snap_t.items():
            setattr(tasks, k, v)
        rt.data.mocap_quat[:] = snap_q
        for r, c in grips.items():
            tasks.plant.grip_cmd(r, c)
    if caps:                                      # the attempt that launched, else the first (the one the verdict names)
        cap.update(next((c for c in caps if c["launched"]), caps[0]))
    cap["new_markers"] = [mk["id"] for mk in tasks.markers if id(mk) not in before]
    return verdict, cap


def _plan_motion(ctx, msg):
    """pick_place / reach -> plan dict."""
    tasks = ctx.tasks
    a = msg["action"]
    verdict, cap = _dry_run(ctx, msg)
    ok = verdict.get("status") == "running"
    message = verdict.get("message", "")
    new = list(cap.get("new_markers", []))
    run = {k: v for k, v in msg.items() if k not in ("type", "preview")}
    if msg.get("target_uv") is not None and new:          # the tap became marker X: confirm by name, no second marker
        run.pop("target_uv", None)
        run["target"] = new.pop(0)
    plan = {"action": a, "ok": ok, "message": message, "run": run, "constraints": [], "reasons": [], "steps": [],
            "waypoints": [], "wp_ok": [], "target": None, "carry": None, "alternative": None, "hand": None}
    kps = cap.get("keypoints")
    if not kps:                                           # failed before any motion was planned (unknown target, ...)
        plan["intent"] = {"pick_place": f"Move {tasks.pretty(tasks.obj_name(msg.get('object', 'mug')) or str(msg.get('object', 'it')))}",
                          "reach": "Reach"}[a] + (f" to {msg['target']}." if msg.get("target") else ".")
        plan["reasons"] = [message]
        plan["verdict"] = "I can't do this yet"
        return plan

    palm = cap["palm"]
    side = "right" if palm.startswith("right") else "left"
    plan["hand"] = side
    path = cap.get("path") or {}
    pts = np.asarray(path.get("pts", []), float)
    okv = list(path.get("ok", [])) or [True] * len(pts)
    plan["waypoints"], plan["wp_ok"] = _decimate(pts, okv)
    # densified index of every keypoint, the same way _launch builds the path
    idx, cnt = [0], 0
    for p0, p1 in zip(kps[:-1], kps[1:]):
        cnt += len(densify([p0, p1])) - 1
        idx.append(cnt)
    n = max(1, len(pts) - 1)
    frac = lambda k: round(min(1.0, idx[min(k, len(idx) - 1)] / n), 3)
    ev = {name: k for k, name in cap["events"].items()}
    speed = getattr(ctx.rt, "speed_scale", 1.0)

    if a == "pick_place":
        name = cap["cur"]
        what = tasks.pretty(name)
        dm = cap.get("done_msg") or ""
        label = dm[len(f"Placed {what} on "):-1] if dm.startswith(f"Placed {what} on ") else "the spot"
        taken = re.search(r" \(beside ([a-z ]+), which is there\)", label)
        if taken:                                         # the spot was occupied: free_spot moved it beside the blocker
            label = label.replace(taken.group(0), f", beside {taken.group(1)}")
            plan["reasons"].append(f"That spot is taken by {taken.group(1)}, so I'll put it right beside.")
        rel = ev.get("release", len(kps) - 3)
        off = cap.get("cur_off") if cap.get("cur_off") is not None else np.zeros(3)
        o = tasks.objs[name]
        to_c = kps[rel] - off
        plan["target"] = {"label": label, "pos": _xyz([to_c[0], to_c[1], TABLE_TOP])}
        plan["carry"] = {"object": name, "label": what, "shape": "cyl" if name == "mug" else "box", "r": round(o["r"], 3),
                         "half_h": round(o["half_h"], 3), "from": _xyz(tasks.mug_center(name)), "to": _xyz(to_c)}
        top = bool(np.allclose(off[:2], 0.0))
        via = "route" in label
        plan["intent"] = f"Pick up {what} with my {side} hand and put it on {label}."
        steps = []
        g = ev.get("grasp")
        # each step starts at the keypoint its motion leaves from
        if g is not None:
            steps += [(f"Reach for {what} with my {side} hand", 0), (f"Grasp it from the {'top' if top else 'side'}", g - 1),
                      ("Lift it 12 cm", g)]
            carry_from = g + 1
        else:
            carry_from = 0
        steps += [("Carry it along your route" if via else f"Carry it over to {label}", min(carry_from, rel - 2)),
                  (f"Set it down on {label}", rel - 1), ("Bring my hand back", rel)]
        plan["constraints"] = ["Keep it upright"] + [f"Clear of {tasks.pretty(ob)}" for ob in tasks.obstacles] + \
                              (["Follow your route"] if via else []) + ["Stay on the table"]
    else:
        dm = cap.get("done_msg") or ""
        label = dm[len("My hand is over "):-1] if dm.startswith("My hand is over ") else "the spot"
        end = kps[-1]
        plan["target"] = {"label": label, "pos": _xyz([end[0], end[1], end[2] - HOVER - 0.04])}
        plan["intent"] = f"Reach over {label} with my {side} hand."
        steps = [(f"Move my {side} hand over {label}", 0), ("Hold it there", len(kps) - 1)]
        plan["constraints"] = ["Hover 12 cm above the table"]
    if speed < 0.99:
        plan["constraints"].append(f"Slow, x{speed:.1f} speed")
    plan["steps"] = [{"text": t, "at": frac(k)} for t, k in steps]

    nbad = sum(1 for x in okv if not x)
    if ok:
        plan["verdict"] = "I can do this"
        plan["reasons"].insert(0, f"All {len(pts)} waypoints are within my {side} arm's reach.")
        if a == "pick_place" and tasks.obstacles:
            plan["reasons"].append(f"{_cap(plan['carry']['label'])} stays clear of " +
                                   " and ".join(tasks.pretty(ob) for ob in tasks.obstacles) + ".")
    elif cap.get("why") == "vase" and cap.get("hit"):
        cm, who = cap["hit"][0], tasks.pretty(cap.get("hit_obstacle") or "vase")
        plan["verdict"] = f"I can't — {cm:.0f} cm too low over {who}"
        plan["reasons"].insert(0, f"Carrying {plan['carry']['label']} straight there would hit {who} ({cm:.0f} cm too low).")
        plan["alternative"] = {"text": f"Draw a route over {who} in 3D, then say 'carry it along my path'.", "task": None}
    else:
        dists = cap.get("dists") or [0.0]
        cm = max(dists) * 100
        plan["verdict"] = f"I can't — {cm:.0f} cm out of reach"
        plan["reasons"].insert(0, f"{nbad} of {len(pts)} waypoints are out of my {side} arm's reach, worst by {cm:.0f} cm.")
        mk = re.search(r"closest spot I can reach is marker ([A-Z]), (\d+) cm nearer", message)
        if mk and a == "pick_place":
            plan["alternative"] = {"text": f"Marker {mk.group(1)} is the closest spot I can reach, {mk.group(2)} cm nearer.",
                                   "label": f"Use marker {mk.group(1)}",
                                   "task": {"action": "pick_place", "object": run.get("object", "mug"), "target": mk.group(1)}}
            plan["question"] = f"I can't reach there. Put it on marker {mk.group(1)} instead?"
    return plan


def _plan_path(ctx, msg):
    rt = ctx.rt
    p = rt.path
    plan = {"action": "follow_path", "run": {k: v for k, v in msg.items() if k not in ("type", "preview")}, "constraints": [],
            "reasons": [], "steps": [], "waypoints": [], "wp_ok": [], "target": None, "carry": None, "alternative": None, "hand": None}
    if p is None or len(p["pts"]) < 2:
        plan.update(ok=False, intent="Follow your route.", verdict="I can't — there is no route yet",
                    message="There is no path yet. Draw one on my view or in 3D first.",
                    reasons=["There is no path yet. Draw one on my view or in 3D first."])
        return plan
    pts, okv = np.asarray(p["pts"], float), [bool(o) for o in p["ok"]]
    side = p.get("hand") if p.get("hand") in ("left", "right") else ("right" if p["palm"].startswith("right") else "left")
    nbad = sum(1 for o in okv if not o)
    plan.update(ok=nbad == 0, hand=side, intent=f"Trace your {len(pts)}-point route with my {side} hand.",
                target={"label": "the end of your route", "pos": _xyz([pts[-1][0], pts[-1][1], TABLE_TOP])},
                steps=[{"text": f"Move my {side} hand to the start", "at": 0.0}, {"text": f"Trace {len(pts)} waypoints", "at": 0.02},
                       {"text": "Stop at the end", "at": 1.0}],
                constraints=["Hand only, body stays put"])
    plan["waypoints"], plan["wp_ok"] = _decimate(pts, okv)
    held = ctx.tasks.holding.get(p["palm"])
    if held:
        plan["constraints"].insert(0, f"Carrying {ctx.tasks.pretty(held)}")
    if nbad == 0:
        plan.update(verdict="I can do this", message=f"Following your path with my {side} hand.",
                    reasons=[f"All {len(pts)} waypoints are within my {side} arm's reach."])
    else:
        plan.update(verdict=f"I can't — {nbad} waypoint{'s' if nbad != 1 else ''} out of reach", message="Part of that path is out of my reach (red). Redraw it closer.",
                    reasons=[f"{nbad} of {len(pts)} waypoints are out of my {side} arm's reach (red)."],
                    alternative={"text": "Drag the red waypoints closer to me in 3D, then ask again.", "task": None})
    return plan


def _plan_skill(ctx, msg):
    key = str(msg.get("name") or "").strip().lower().replace(" ", "_")
    plan = {"action": "play_skill", "run": {k: v for k, v in msg.items() if k not in ("type", "preview")}, "constraints": [],
            "reasons": [], "steps": [], "waypoints": [], "wp_ok": [], "target": None, "carry": None, "alternative": None, "hand": None}
    f = SKILL_DIR / f"{key}.json"
    if not key or not f.exists():
        known = ", ".join(ctx.tasks.skills()) or "nothing yet"
        plan.update(ok=False, intent=f"Play '{msg.get('name') or '?'}'.", verdict="I can't — I don't know that skill",
                    message=f"I don't know '{msg.get('name')}'. I know: {known}.", reasons=[f"I know: {known}."])
        return plan
    data = json.load(open(f))
    frames = data.get("frames") or []
    dur = frames[-1]["t"] if frames else 0.0
    speed = getattr(ctx.rt, "speed_scale", 1.0)
    # ghost: the palm that moves most, sampled along the recording
    best, bspan = None, -1.0
    for palm in ("right_palm", "left_palm"):
        tr = np.array([fr["targets"][palm] for fr in frames if palm in (fr.get("targets") or {})], float)
        if len(tr) > 1:
            span = float(np.ptp(tr, axis=0).max())
            if span > bspan:
                best, bspan = (palm, tr), span
    if best is not None:
        tr = best[1][:: max(1, len(best[1]) // MAX_PTS)]
        plan["waypoints"], plan["wp_ok"] = [_xyz(p) for p in tr], [True] * len(tr)
        plan["hand"] = "right" if best[0].startswith("right") else "left"
    feas = (data.get("report") or {}).get("feasible_fraction")
    plan.update(ok=True, intent=f"Play '{key}' ({dur / max(speed, 0.1):.1f} s) at x{speed:.1f} speed.", verdict="I can do this",
                message=f"Playing '{key}' at x{speed:.1f} speed.",
                steps=[{"text": f"Replay '{key}' with my whole body", "at": 0.0}, {"text": "Return to my start pose", "at": 1.0}],
                constraints=["Feet stay planted"] + ([f"Slow, x{speed:.1f} speed"] if speed < 0.99 else []),
                reasons=[f"Recorded {dur:.1f} s" + (f", {feas * 100:.0f}% feasible when you taught it." if feas is not None else ".")])
    return plan


def _is_taught_task(msg):
    key = str(msg.get("name") or "").strip().lower().replace(" ", "_")
    f = SKILL_DIR / f"{key}.json"
    try:
        return bool(key) and f.exists() and json.load(open(f)).get("type") == "task"
    except Exception:
        return False


# ---------------------------------------------------------------------------- hooks
def _set(ctx, plan):
    ctx.store[KEY] = plan


@ext.pre_task
def preview_first(ctx, msg):
    if not msg.get("preview"):
        return False
    a = msg.get("action")
    if a not in MOTION or (a == "play_skill" and _is_taught_task(msg)):
        return False   # stop/home/speed/... and taught multi-step tasks (they run their own practice card) are not previewed
    tid = str(msg.get("id", f"plan-{time.time():.3f}"))
    try:
        plan = {"follow_path": _plan_path, "play_skill": _plan_skill}.get(a, _plan_motion)(ctx, msg)
    except Exception as e:    # never fall through to running an unconfirmed task
        import traceback
        traceback.print_exc()
        ctx.tasks.set_task(tid, a, "failed", f"I couldn't work out a plan for that ({e}).")
        _set(ctx, None)
        return True
    plan["id"], plan["anchor"], plan["t"] = tid, tid, time.time()
    plan["run"]["id"] = tid
    if plan["ok"]:
        plan["phase"] = "proposing"
        say = plan["intent"] + " Say go, or tap Run."
        ctx.tasks.set_task(tid, a, "proposed", say, "waiting for you")
    else:
        plan["phase"] = "needs_you" if plan.get("alternative") and plan["alternative"].get("task") else "blocked"
        say = plan["message"] if plan["phase"] == "blocked" else f"{plan['reasons'][0]} {plan['question']}"
        ctx.tasks.set_task(tid, a, "failed", say)
    plan["say"] = say
    _set(ctx, plan)
    return True


@ext.handler("plan")
def on_plan(ctx, msg):
    p = ctx.store.get(KEY)
    op = msg.get("op")
    if p is None or (msg.get("id") and str(msg["id"]) != p["id"]):
        return
    if op == "confirm":
        if p["phase"] != "proposing" or not p["ok"]:
            return
        p["phase"], p["progress"], p["step_i"] = "running", 0.0, 0
        p["run"]["type"] = "task"
        ctx.dispatch(dict(p["run"]))
    elif op == "cancel":
        if p["phase"] in WAITING:
            ctx.tasks.set_task(p["id"], p["action"], "failed", "Cancelled. Nothing moved.")
            _set(ctx, None)
        elif p["phase"] == "running":
            ctx.dispatch({"type": "task", "id": p["id"] + "-stop", "action": "stop"})
        else:
            _set(ctx, None)


def _confirm_hook(ctx):
    """A "go on" gesture (pps/features/gestures.py) confirms a waiting plan."""
    p = ctx.store.get(KEY)
    if p and p["phase"] == "proposing" and p["ok"]:
        on_plan(ctx, {"op": "confirm"})
        return "Running the plan"
    return None


@ext.tick
def track(ctx):
    ctx.store.setdefault("confirm_hook", _confirm_hook)
    p = ctx.store.get(KEY)
    if not p or p.get("seq"):                     # a multi-step plan tracks itself (pps/features/fixtures.py)
        return
    t = ctx.tasks.task or {}
    tid = t.get("id")
    if p["phase"] != "running":
        if tid != p["anchor"]:            # another command took over: the proposal (or old result) is stale
            _set(ctx, None)
        return
    if tid == p["id"]:
        st = t.get("status")
        if st == "running":
            rt = ctx.rt
            if p["action"] == "play_skill":
                n = max(1, len(rt.recorder.frames) - 1)
                prog = rt.preview_idx / n
            elif rt.path is not None:
                prog = rt.path["i"] / max(1, len(rt.path["pts"]) - 1)
            else:
                prog = p.get("progress", 0.0)
            p["progress"] = round(float(min(1.0, max(0.0, prog))), 3)
            p["step_i"] = max([i for i, s in enumerate(p["steps"]) if s["at"] <= p["progress"] + 1e-6] or [0])
        elif st in ("done", "failed"):
            p["phase"], p["message"] = st, t.get("message", "")
            if st == "done":
                p["progress"], p["step_i"] = 1.0, len(p["steps"]) - 1
    else:
        stopped = getattr(ctx.tasks, "stopped_task", None) or {}
        p["phase"] = "failed"
        p["message"] = stopped.get("message") if stopped.get("id") == p["id"] else "Interrupted by another command."
        p["anchor"] = tid


@ext.state
def plan_state(ctx):
    p = ctx.store.get(KEY)
    if not p:
        return {KEY: None}
    return {KEY: {k: v for k, v in p.items() if k not in ("anchor", "run", "t")} | {"task": {k: v for k, v in p["run"].items() if k != "type"}}}
