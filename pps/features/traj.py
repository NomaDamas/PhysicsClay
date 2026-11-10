"""P2 trajectories on the server (docs/V1.md P2, docs/PROTOCOL.md v7): the `traj` message, `state.traj`, checks,
the plan card, and execution through the runtime's path follower.

{type:"traj", op:"propose", traj? | spec? | demo?, id?, task_id?}   compile (spec = language keypoints, pps/traj.py
                                                                    from_keypoints), check, show the ghost + plan card;
                                                                    nothing moves until "go" (plan confirm) or op run
{type:"traj", op:"set", traj, run?: bool, id?}                      make it the current trajectory (no card)
{type:"traj", op:"edit", index? | near?, point? | changes?, insert?, remove?, pattern?, profile?, label?, world?: bool}
{type:"traj", op:"run"}  ·  {type:"traj", op:"clear"}

How it plugs in: an installed trajectory *is* `rt.path` (the robot-view overlay, the F key, "Run path", follow_path,
plan preview and "carry it along my path" all keep working), with `timed: True` and the dense samples, which the
runtime follows in time (pps/traj.py follow_step: orientation, speed, gripper and force events). Drawn strokes and 3D
editor paths (`tasks.set_path_world`, marked src "draw") are adopted as trajectories with source "draw". The proposal is
a regular plan card (state.plan, action follow_path), so "go", the go-on gesture and the agent's confirm all work.

P5 and other features add checks with register_checker(fn(ctx, pts, quats, info) -> [reason | None]).
"""
import time

import numpy as np

from .. import ext
from .. import tasks as tasks_mod
from .. import traj as T

KEY = "traj"
MAX_GHOST = 160
CHECKERS = []           # extra per-sample checks (P5 safety): fn(ctx, pts, quats, info) -> [reason | None]


def register_checker(fn):
    CHECKERS.append(fn)
    return fn


def graspable(hand, c, r, hh):
    """The hand is on or at the object (beside it, or just above its top): closing the gripper there takes it."""
    return float(np.hypot(hand[0] - c[0], hand[1] - c[1])) < r + 0.035 and c[2] - hh - 0.02 < hand[2] < c[2] + hh + 0.045


def _r(p, n=4):
    return [round(float(x), n) for x in p]


# ---------------------------------------------------------------------------- store
def _st(ctx):
    s = ctx.store.get(KEY)
    if s is None or s.get("rt") is not ctx.rt:          # first frame, or a new world: start over
        s = {"rt": ctx.rt, "tasks": ctx.tasks, "current": None, "proposed": None, "prep": None, "path": None, "rev": 0,
             "seq": 0, "reply": None, "running": False, "plan_id": None, "prev": None, "last": None, "qhome": {},
             "zones": None, "events": []}
        ctx.store[KEY] = s
    rt = ctx.rt
    for palm in getattr(rt, "hands", []):              # the approach orientation the trajectory's q defaults to
        if palm not in s["qhome"]:
            s["qhome"][palm] = T.qnorm(rt.data.mocap_quat[rt.hands_mid[palm]].copy())
    return s


def _palm(rt, role):
    palm = f"{role}_palm"
    return palm if palm in rt.hands else rt.hands[0]


def _kind(ctx):
    return getattr(getattr(ctx.rt, "profile", None), "kind", None) or ("humanoid" if getattr(ctx.rt, "profile_id", "") == "g1" else None)


def world_of(ctx, palm=None):
    """The scene as pps/traj.py sees it: objects (centre, radius, half height), surface, markers, regions, hand."""
    tasks, rt = ctx.tasks, ctx.rt
    held = set(v for v in tasks.holding.values() if v)
    objs = {}
    for n, o in tasks.objs.items():
        if n in held:
            continue
        objs[n] = {"pos": [float(x) for x in tasks.mug_center(n)], "r": float(o["r"]), "h": float(o["half_h"]),
                   "kind": tasks.kinds.get(n, ""), "movable": True}
    for n, o in tasks.obstacles.items():
        objs[n] = {"pos": [float(x) for x in o["pos"]], "r": float(o["r"]), "h": float(o["h"]), "kind": tasks.kinds.get(n, ""),
                   "movable": False}
    w = {"objects": objs, "surface": float(tasks_mod.TABLE_TOP), "regions": {k: list(v) for k, v in tasks_mod.REGIONS.items()},
         "markers": {m["id"]: [float(x) for x in m["pos"]] for m in tasks.markers}}
    if palm is not None:
        w["hand"] = [float(x) for x in rt.data.mocap_pos[rt.hands_mid[palm]]]
        h = tasks.holding.get(palm)
        if h in tasks.objs:          # carrying: the object hangs below the hand
            c = tasks.mug_center(h)
            w["carry_drop"] = max(0.0, float(w["hand"][2] - (c[2] - tasks.objs[h]["half_h"])))
            w["carrying"] = h
    return w


def _poses(ctx):
    """Frame origins: the top centre of every object (held ones too: a trajectory may be relative to what I carry)."""
    tasks = ctx.tasks
    out = {}
    for n, o in tasks.objs.items():
        c = tasks.mug_center(n)
        out[n] = [float(c[0]), float(c[1]), float(c[2] + o["half_h"])]
    for n, o in tasks.obstacles.items():
        out[n] = [float(o["pos"][0]), float(o["pos"][1]), float(o["pos"][2] + o["h"])]
    return out


# ---------------------------------------------------------------------------- checks
def _checkers(ctx, palm, tr_world, dense, world):
    rt, tasks = ctx.rt, ctx.tasks
    top = world["surface"]
    tol_default = float(getattr(getattr(rt, "validator", None), "reach_tol", 0.06))
    tols = {k: p["tol"] for k, p in zip(dense["key"], tr_world["points"]) if p.get("tol")}

    def reach(pts, qs, info):
        rt.precheck(pts, palm, quats=qs)
        d = list(getattr(rt, "last_precheck_dists", []) or [0.0] * len(pts))
        na, idx = info["n_approach"], info["idx"]
        out = []
        for j, dist in enumerate(d):
            tol = tols.get(idx[j - na], tol_default) if j >= na else tol_default
            out.append(f"out of my reach by {max(1.0, dist * 100):.0f} cm" if dist >= tol else None)
        return out

    def surface(pts, qs, info):
        na, idx = info["n_approach"], info["idx"]
        x0, x1 = tasks_mod.TABLE_X
        y0, y1 = tasks_mod.TABLE_Y
        out = []
        for j, p in enumerate(pts):
            press = j >= na and np.isfinite(dense["force"][idx[j - na]]) and dense["force"][idx[j - na]] > 0
            lim = top - (0.02 if press else -0.005)
            on = x0 - 0.03 < p[0] < x1 + 0.03 and y0 - 0.03 < p[1] < y1 + 0.03
            out.append(f"{max(1.0, (lim - p[2]) * 100):.0f} cm into the {tasks_mod.SURFACE}" if on and p[2] < lim else None)
        return out

    # objects: the hand must not go into them (except what it grasps along the way); a carried object must clear them
    grasped, drop_planned = set(), 0.06
    for t_ev, kind, val, pi in dense["events"]:
        if kind == "grip" and val >= 0.5:
            p = np.asarray(tr_world["points"][pi]["p"], float)
            for n, o in world["objects"].items():
                if o.get("movable") and graspable(p, o["pos"], o["r"], o["h"]):
                    if not grasped:
                        drop_planned = max(0.0, float(p[2] - (o["pos"][2] - o["h"])))
                    grasped.add(n)
    carry0 = world.get("carrying")
    drop0 = float(world.get("carry_drop", 0.0))

    def objects(pts, qs, info):
        na, idx = info["n_approach"], info["idx"]
        out = []
        for j, p in enumerate(pts):
            g = dense["grip"][idx[j - na]] if j >= na else np.nan
            carrying = (carry0 is not None and (not np.isfinite(g) or g >= 0.5)) or (np.isfinite(g) and g >= 0.5 and grasped)
            drop = drop0 if carry0 else drop_planned
            hit = None
            for n, o in world["objects"].items():
                if n in grasped:
                    continue
                c, r, hh = o["pos"], o["r"], o["h"]
                dh = np.hypot(p[0] - c[0], p[1] - c[1])
                margin = 0.02 if not o.get("movable") else 0.012
                if dh < r + margin and p[2] < c[2] + hh + 0.01:
                    hit = f"my hand would hit the {n.replace('_', ' ')}"
                    break
                if carrying and dh < r + 0.04 and p[2] - drop < c[2] + hh + 0.005:
                    hit = f"what I carry would hit the {n.replace('_', ' ')}"
                    break
            out.append(hit)
        return out

    def zones(pts, qs, info):
        ko = getattr(tasks, "keepout", None)
        if ko is None:
            return [None] * len(pts)
        got = ko.along(pts, {}, palm) or [None] * len(pts)
        return [f"inside {z[0]}" if z else None for z in got]

    extra = [lambda pts, qs, info, fn=fn: fn(ctx, pts, qs, info) for fn in CHECKERS]
    return [reach, surface, objects, zones, *extra]


def _ghost_idx(dense):
    idx = T.check_samples(dense)
    if len(idx) <= MAX_GHOST:
        return idx
    must = set(dense["key"]) | {0, len(dense["t"]) - 1}
    rest = [k for k in idx if k not in must]
    keep = max(0, MAX_GHOST - len(must))
    pick = [rest[int(round(j * (len(rest) - 1) / max(1, keep - 1)))] for j in range(keep)] if keep and rest else []
    return sorted(must | set(pick))


def prepare(ctx, tr, check=True):
    """Normalized trajectory -> everything needed to show, check and run it. Raises T.TrajError."""
    rt = ctx.rt
    s = _st(ctx)
    palm = _palm(rt, tr["role"])
    world = world_of(ctx, palm)
    w_tr = T.to_world(tr, _poses(ctx))
    q0 = s["qhome"].get(palm)
    kind = _kind(ctx)
    dense = T.densify(w_tr, q0=q0, kind=kind)
    idx = _ghost_idx(dense)
    hand = np.asarray(world["hand"], float)
    approach = T.approach_points(hand, dense["p"][0])
    if check:
        res = T.check(dense, _checkers(ctx, palm, w_tr, dense, world), idx=idx, n_points=len(tr["points"]), approach=approach)
    else:
        res = {"ok": [True] * len(tr["points"]), "why": None, "duration": round(dense["duration"], 2), "idx": idx,
               "sample_ok": [True] * len(idx), "approach_ok": [True] * len(approach), "issues": []}
    tr = dict(tr, check={"ok": res["ok"], "why": res["why"], "duration": res["duration"]})
    ok = all(res["sample_ok"]) and all(res["approach_ok"])
    return {"traj": tr, "world_traj": w_tr, "dense": dense, "idx": idx, "res": res, "palm": palm, "world": world,
            "approach": approach, "ok": ok, "kind": kind}


# ---------------------------------------------------------------------------- install into the path follower
def _on_event(ctx, palm):
    def cb(kind, value, pi):
        rt, tasks, s = ctx.rt, ctx.tasks, _st(ctx)
        if kind == "force":
            if hasattr(rt, "set_force"):
                rt.set_force(palm, value)
            s["events"].append(f"force {value:g} N at point {pi + 1}")
            return
        if hasattr(rt, "set_gripper"):                  # P0: a real gripper grasps by contact
            rt.set_gripper(palm, value)
            return
        held = tasks.holding.get(palm)
        if value >= 0.5 and not held:                   # kinematic fallback: close next to an object -> hold it
            ee = rt.data.site_xpos[rt.palm_sid[palm]]
            best = min(((float(np.linalg.norm(tasks.mug_center(n) - ee)), n) for n in tasks.objs
                        if n not in tasks.holding.values()
                        and graspable(ee, tasks.mug_center(n), tasks.objs[n]["r"], tasks.objs[n]["half_h"])), default=None)
            if best:
                tasks.cur = best[1]
                tasks._event("grasp", palm)
                s["events"].append(f"grasped the {best[1].replace('_', ' ')} at point {pi + 1}")
            else:
                s["events"].append(f"closed on nothing at point {pi + 1}")
        elif value < 0.5 and held:
            tasks._event("release", palm)
            s["events"].append(f"let go of the {held.replace('_', ' ')} at point {pi + 1}")
    return cb


def _make_path(ctx, prep, path=None):
    """rt.path for a prepared trajectory (or upgrade an adopted drawn path in place)."""
    dense, idx, res, palm = prep["dense"], prep["idx"], prep["res"], prep["palm"]
    pts = dense["p"][idx]
    ok = list(res["sample_ok"])
    if not all(res["approach_ok"]) and ok:
        ok[0] = False
    N = len(dense["t"])
    di = np.clip(np.searchsorted(np.asarray(idx), np.arange(N), side="right") - 1, 0, len(idx) - 1)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1) if len(pts) > 1 else np.zeros(0)
    side = "right" if palm.startswith("right") else "left"
    timed = {"timed": True, "dense": {"t": dense["t"], "p": dense["p"], "q": dense["q"], "events": dense["events"], "di": di,
                               "didx": np.asarray(idx)},
             "traj_id": prep["traj"]["id"], "approach_speed": T.default_speed(prep["kind"]), "on_event": _on_event(ctx, palm),
             "why": None if prep["ok"] else _why(prep)}
    if path is not None:                                # adopted drawn path: keep its points and verdicts
        path.update(timed, pts0=np.array(path["pts"], float).copy())
        return path
    return {"pts": pts, "ok": ok, "palm": palm, "cum": np.concatenate([[0.0], np.cumsum(seg)]), "i": 0, "hand": side,
            "pts0": pts.copy(), **timed}


def _why(prep):
    r = prep["res"]
    return (r["why"] or "Part of the path is blocked (red).") + " Drag the red points, or tell me how to change it."


def _install(ctx, prep):
    rt, s = ctx.rt, _st(ctx)
    path = _make_path(ctx, prep)
    rt.path, rt.path_msg = path, None
    s["path"], s["prep"] = path, prep
    return path


# ---------------------------------------------------------------------------- plan card
def _summary(prep):
    tr, d = prep["traj"], prep["dense"]
    n = len(tr["points"])
    return f"{n} point{'s' if n != 1 else ''}, {d['duration']:.1f} s"


def _plan(ctx, prep, tid):
    tasks = ctx.tasks
    tr = prep["traj"]
    side = "right" if prep["palm"].startswith("right") else "left"
    hand = tasks.limb(side)
    label = tr.get("label") or {"draw": "your path", "keyframes": "your keyframes", "demo": "what you showed me",
                                "vr": "your path", "skill": "the skill"}.get(tr["source"], "this path")
    w = prep["world_traj"]
    notes = list((tr.get("meta") or {}).get("notes") or [])
    steps = [{"text": f"Move my {hand} to the start", "at": 0.0}]
    dur = max(1e-6, prep["dense"]["duration"])
    for t_ev, kind, val, pi in prep["dense"]["events"]:
        if kind == "grip":
            steps.append({"text": f"{'Close' if val >= 0.5 else 'Open'} the gripper at point {pi + 1}", "at": round(t_ev / dur, 3)})
        elif kind == "force" and val > 0:
            steps.append({"text": f"Press with {val:g} N from point {pi + 1}", "at": round(t_ev / dur, 3)})
    if tr.get("pattern"):
        p = tr["pattern"]
        steps.append({"text": f"{p['kind'].capitalize()} {p['amp'] * 1000:.0f} mm at {p['freq']:g} Hz", "at": 0.1})
    steps.append({"text": f"Finish at point {len(tr['points'])}", "at": 1.0})
    steps.sort(key=lambda x: x["at"])
    speeds = [p["speed"] for p in tr["points"] if p.get("speed")]
    v0 = T.default_speed(prep["kind"])
    cons = [f"{(min(speeds) if speeds else v0) * 100:.0f}" + (f"–{max(speeds) * 100:.0f}" if speeds and max(speeds) > min(speeds) + 1e-6 else "")
            + " cm/s"]
    if any(p.get("yaw") or p.get("tilt") or p.get("q") for p in tr["points"]):
        cons.append("Wrist turns along the way")
    if any(p.get("force") for p in tr["points"]):
        cons.append(f"Presses up to {max(p['force'] or 0 for p in tr['points']):g} N")
    obj = T.frame_object(tr["frame"])
    if obj:
        cons.append(f"Moves with the {obj.replace('_', ' ')}")
    for name in (tr.get("meta") or {}).get("lifted") or []:
        cons.append(f"Clear of the {name.replace('_', ' ')}")
    end = w["points"][-1]["p"]
    plan = {"id": tid, "anchor": tid, "t": time.time(), "action": "follow_path", "run": {"action": "follow_path", "id": tid},
            "ok": prep["ok"], "hand": side, "traj": tr["id"], "steps": steps[:6], "constraints": cons[:6],
            "waypoints": [], "wp_ok": [],                # web/ui/traj.js draws this ghost (handles, wrist, speed)
            "target": {"label": "the end of the path", "pos": _r([end[0], end[1], float(tasks_mod.TABLE_TOP)], 3)},
            "carry": None, "alternative": None,
            "intent": f"Move my {hand} along {label}: {_summary(prep)}."}
    if prep["ok"]:
        plan.update(phase="proposing", verdict="I can do this", message=f"Following {label}.",
                    reasons=[n[:1].upper() + n[1:] + "." for n in notes][:3] or [f"Every point is within my {side} arm's reach."])
        say = f"Here's the path: {label}, {_summary(prep)}. Say go, or drag a point to fix it."
        tasks.set_task(tid, "follow_path", "proposed", say, "waiting for you")
    else:
        why = _why(prep)
        plan.update(phase="blocked", verdict="I can't do this yet", message=why,
                    reasons=[f"{i['why'][:1].upper()}{i['why'][1:]} {i['where']}." for i in prep["res"]["issues"][:3]],
                    alternative={"text": "Drag the red points to fix them, or say 'higher there' / 'go around it'.", "task": None})
        say = why
        tasks.set_task(tid, "follow_path", "failed", say)
    plan["say"] = say
    ctx.store["plan"] = plan
    return plan


def _drop_plan(ctx, s, message=None):
    p = ctx.store.get("plan")
    if p and s.get("plan_id") and p.get("id") == s["plan_id"] and p.get("phase") in ("proposing", "blocked", "needs_you"):
        ctx.store["plan"] = None
        if message:
            ctx.tasks.set_task(p["id"], "follow_path", "failed", message)
    s["plan_id"] = None


# ---------------------------------------------------------------------------- ops
def _reply(s, msg, ok, message, **kw):
    s["reply"] = {"id": msg.get("id"), "op": msg.get("op"), "ok": bool(ok), "message": message, "t": round(time.time(), 2), **kw}
    s["rev"] += 1


def _new_id(s, prefix="t"):
    s["seq"] += 1
    return f"{prefix}{s['seq']}"


def _compile(ctx, s, msg):
    """propose / set payload -> normalized trajectory."""
    rt = ctx.rt
    if msg.get("spec") is not None:
        spec = dict(msg["spec"])
        role = str(spec.get("role") or "auto").lower()
        palm = _palm(rt, role if role in ("left", "right") else "right")
        tr, _ = T.from_keypoints(dict(spec, role=palm.replace("_palm", "")), world_of(ctx, palm))
        if role not in ("left", "right") and len(rt.hands) > 1:      # auto: the hand on the path's side (like tasks.hand_for)
            y = float(np.mean([p["p"][1] for p in T.to_world(tr, _poses(ctx))["points"]]))
            other = "left_palm" if y > 0 else "right_palm"
            if other != palm and other in rt.hands:
                palm = other
                tr, _ = T.from_keypoints(dict(spec, role=palm.replace("_palm", "")), world_of(ctx, palm))
    elif msg.get("demo") is not None:
        from ..human import demo as demo_mod
        d = demo_mod.load(str(msg["demo"]))
        if d is None:
            raise T.TrajError(f"I have no demonstration called '{msg['demo']}'")
        palm = rt.hands[0]
        tr = T.from_demo(d, hand=[float(x) for x in rt.data.mocap_pos[rt.hands_mid[palm]]])
        if T.frame_object(tr["frame"]) and T.frame_object(tr["frame"]) not in _poses(ctx):
            tr = T.to_world(tr, {T.frame_object(tr["frame"]): rt.data.mocap_pos[rt.hands_mid[palm]].tolist()})
    else:
        default_role = "right" if "right_palm" in rt.hands else rt.hands[0].replace("_palm", "")
        tr = T.normalize(msg.get("traj"), default_role=default_role)
    tr = dict(tr, id=_new_id(s), check=None)
    return T.normalize(tr)


def _restore_prev(ctx, s):
    rt = ctx.rt
    prev = s.get("prev")
    s["prev"] = None
    if rt.path is s["path"] and rt.mode != "FOLLOW":
        if prev and prev[0] is not None:
            rt.path, s["path"], s["current"], s["prep"] = prev[0], prev[0], prev[1], prev[2]
        else:
            rt.path, s["path"], s["prep"] = None, None, None


def op_propose(ctx, s, msg):
    tr = _compile(ctx, s, msg)
    prep = prepare(ctx, tr)
    if s["proposed"] is None:                         # a cancel puts the old drawn path back
        s["prev"] = (ctx.rt.path, s["current"], s["prep"])
    _drop_plan(ctx, s)
    _install(ctx, prep)
    s["proposed"], s["current"] = prep["traj"], None
    tid = str(msg.get("task_id") or f"traj-{tr['id']}-{int(time.time() * 1000) % 100000}")
    s["plan_id"] = tid
    plan = _plan(ctx, prep, tid)
    _reply(s, msg, prep["ok"], plan["say"], traj=tr["id"], plan=tid, n=len(tr["points"]), duration=prep["res"]["duration"],
           why=prep["res"]["why"], notes=(tr.get("meta") or {}).get("notes"))


def op_set(ctx, s, msg):
    tr = _compile(ctx, s, msg)
    prep = prepare(ctx, tr)
    _drop_plan(ctx, s, "Replaced by a new path.")
    s["proposed"], s["prev"] = None, None
    _install(ctx, prep)
    s["current"] = prep["traj"]
    text = f"Path ready: {_summary(prep)}." + ("" if prep["ok"] else " " + _why(prep))
    _reply(s, msg, prep["ok"], text, traj=tr["id"], n=len(tr["points"]), duration=prep["res"]["duration"], why=prep["res"]["why"])
    if msg.get("run") and prep["ok"]:
        op_run(ctx, s, {"op": "run", "id": msg.get("id"), "task_id": msg.get("task_id")})
        r = s["reply"]
        r.update(op="set", message=f"{text} {r['message']}", traj=tr["id"], n=len(tr["points"]), duration=prep["res"]["duration"])


def _active(s):
    return ("proposed", s["proposed"]) if s["proposed"] is not None else (("current", s["current"]) if s["current"] is not None else (None, None))


def op_edit(ctx, s, msg):
    which, tr = _active(s)
    if tr is None:
        raise T.TrajError("there is no path to change yet")
    if s["running"]:
        raise T.TrajError("I'm moving along it right now; say stop first")
    prep0 = s["prep"]
    world = world_of(ctx, prep0["palm"]) if prep0 else world_of(ctx)
    poses = _poses(ctx)
    origin = T.frame_origin(tr["frame"], poses)
    surface = world["surface"]
    w_tr = T.to_world(tr, poses)
    changed = []
    if msg.get("insert") is not None:
        ins = msg["insert"] if isinstance(msg["insert"], dict) else {"after": msg["insert"]}
        p = ins.get("p")
        if p is not None and (ins.get("world", msg.get("world", True))):
            p = (np.asarray(T._vec(p, 3, "p"), float) - origin).tolist()
        tr = T.insert(tr, int(ins.get("after", len(tr["points"]) - 1)), p)
        changed.append("added a point")
    if msg.get("remove") is not None:
        ids = msg["remove"] if isinstance(msg["remove"], list) else [msg["remove"]]
        ids = T.select(w_tr, index=ids)
        tr = T.edit(tr, ids, {"remove": True})
        changed.append(f"removed point{'s' if len(ids) > 1 else ''} {', '.join(str(i + 1) for i in ids)}")
    ch = dict(msg.get("changes") or {})
    pt = msg.get("point")
    if isinstance(pt, dict):
        ch.update(pt)
        if pt.get("p") is not None and msg.get("world", True):
            ch["p"] = (np.asarray(T._vec(pt["p"], 3, "p"), float) - origin).tolist()
    if ch:
        ids = T.select(T.to_world(tr, poses), index=msg.get("index"), near=msg.get("near"), world=world) \
            if (msg.get("index") is not None or msg.get("near") is not None) else list(range(len(tr["points"])))
        ch["_default_speed"] = T.default_speed(_kind(ctx))
        tr = T.edit(tr, ids, ch, surface=surface, origin=origin)
        what = [k for k in ch if not k.startswith("_")]
        changed.append(f"changed {', '.join(what)} at point{'s' if len(ids) > 1 else ''} {', '.join(str(i + 1) for i in ids)}")
    if "pattern" in msg:
        tr = T.normalize(dict(tr, pattern=msg["pattern"], check=None))
        changed.append("changed the pattern" if msg["pattern"] else "removed the pattern")
    if msg.get("label") is not None:
        tr = dict(tr, label=str(msg["label"])[:160])
    if isinstance(msg.get("profile"), dict):              # 2D fallback: lift + arch over the drawn heights
        meta = dict(tr.get("meta") or {})
        base = meta.get("profile_base") or []
        if len(base) != len(tr["points"]):                # points were added or removed since: start from here
            base = [p["p"][2] - (meta.get("profile") or {}).get("lift", 0.0) for p in tr["points"]]
        lift = float(np.clip(float(msg["profile"].get("lift", 0.0)), -0.1, 0.4))
        arch = float(np.clip(float(msg["profile"].get("arch", 0.0)), 0.0, 0.4))
        P = np.array([p["p"] for p in tr["points"]], float)
        sl = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))])
        L = max(sl[-1], 1e-6)
        pts = [dict(p, p=[p["p"][0], p["p"][1], float(base[i] + lift + arch * np.sin(np.pi * sl[i] / L))]) for i, p in enumerate(tr["points"])]
        meta.update(profile_base=base, profile={"lift": lift, "arch": arch})
        tr = T.normalize(dict(tr, points=pts, meta=meta, check=None))
        changed.append(f"lifted it {lift * 100:+.0f} cm" + (f" with a {arch * 100:.0f} cm arch" if arch > 0 else ""))
    if not changed and msg.get("label") is None:
        raise T.TrajError("nothing to change")
    prep = prepare(ctx, T.normalize(dict(tr, check=None)))
    _install(ctx, prep)
    s[which] = prep["traj"]
    if which == "proposed" and s.get("plan_id"):
        p = ctx.store.get("plan")
        if p and p.get("id") == s["plan_id"] and p.get("phase") in ("proposing", "blocked", "needs_you"):
            _plan(ctx, prep, s["plan_id"])
    text = ("Okay, " + "; ".join(changed) + ". " if changed else "") + (f"{_summary(prep)}, all clear." if prep["ok"] else _why(prep))
    _reply(s, msg, prep["ok"], text, traj=prep["traj"]["id"], n=len(prep["traj"]["points"]), duration=prep["res"]["duration"],
           why=prep["res"]["why"])


def op_run(ctx, s, msg):
    rt = ctx.rt
    if s["running"]:
        _reply(s, msg, True, "Already moving along it.")
        return
    if s["proposed"] is not None and s.get("plan_id"):
        p = ctx.store.get("plan")
        if p and p.get("id") == s["plan_id"]:
            if p.get("phase") != "proposing" or not p.get("ok"):
                _reply(s, msg, False, (s["prep"] and not s["prep"]["ok"] and _why(s["prep"])) or "That path can't run yet.")
                return
            ext.handle(ctx, {"type": "plan", "op": "confirm", "id": s["plan_id"]})
            _reply(s, msg, True, "Running it.", task=s["plan_id"])
            return
    if s["current"] is None and s["proposed"] is None and s.get("last") is not None:
        prep = prepare(ctx, s["last"])
        _install(ctx, prep)
        s["current"] = prep["traj"]
    if s["path"] is None or rt.path is not s["path"]:
        _reply(s, msg, False, "There is no path to run. Draw one, place keyframes, or tell me where to go.")
        return
    if s["prep"] is not None and not s["prep"]["ok"]:
        _reply(s, msg, False, _why(s["prep"]))
        return
    tid = str(msg.get("task_id") or f"traj-run-{int(time.time() * 1000) % 100000}")
    _drop_plan(ctx, s)
    ctx.dispatch({"type": "task", "id": tid, "action": "follow_path"})
    _reply(s, msg, True, "Running it.", task=tid)


def op_clear(ctx, s, msg):
    rt = ctx.rt
    if s["running"]:
        ctx.dispatch({"type": "task", "id": f"traj-stop-{int(time.time() * 1000) % 100000}", "action": "stop"})
    _drop_plan(ctx, s, "Cleared. Nothing moved." if s["proposed"] is not None else None)
    if rt.path is s["path"] and rt.path is not None:
        rt.path, rt.path_msg = None, None
    s.update(current=None, proposed=None, prep=None, path=None, prev=None, running=False)
    _reply(s, msg, True, "Cleared the path.")


OPS = {"propose": op_propose, "set": op_set, "edit": op_edit, "run": op_run, "clear": op_clear}


@ext.handler("traj")
def on_traj(ctx, msg):
    s = _st(ctx)
    fn = OPS.get(msg.get("op"))
    if fn is None:
        _reply(s, msg, False, f"Unknown trajectory op '{msg.get('op')}'. Use propose, set, edit, run or clear.")
        return
    try:
        fn(ctx, s, msg)
    except T.TrajError as e:
        text = str(e)
        _reply(s, msg, False, text[:1].upper() + text[1:] + ("" if text.endswith(".") else "."))


# ---------------------------------------------------------------------------- per frame
def _adopt(ctx, s):
    """A stroke / 3D-editor path (tasks.set_path_world) -> a trajectory with source "draw", upgraded in place."""
    rt = ctx.rt
    path = rt.path
    tr = T.normalize({"id": _new_id(s), "role": "right" if path["palm"].startswith("right") else "left", "frame": "world",
                      "source": "draw", "label": "your path", "points": [{"p": [float(x) for x in p]} for p in path["pts"]],
                      "meta": {"profile_base": [float(p[2]) for p in path["pts"]]}})
    prep = prepare(ctx, tr, check=False)             # tasks.set_path_world already checked every point (reach + zones)
    prep["idx"] = list(prep["dense"]["key"])
    ok = [bool(o) for o in path["ok"]]
    why = path.get("why") or (None if all(ok) else "Part of that path is out of my reach (red).")
    prep["res"] = dict(prep["res"], ok=ok, why=why, sample_ok=ok, idx=prep["idx"])
    prep["ok"] = all(ok)
    prep["traj"] = dict(prep["traj"], check={"ok": ok, "why": why, "duration": prep["res"]["duration"]})
    keep_why = path.get("why")
    _make_path(ctx, prep, path)
    path["why"] = keep_why
    s["path"], s["prep"], s["current"] = path, prep, prep["traj"]
    if s["proposed"] is not None:
        _drop_plan(ctx, s, "Replaced by your drawing.")
        s["proposed"], s["prev"] = None, None


@ext.tick
def traj_tick(ctx):
    s = _st(ctx)
    rt, tasks = ctx.rt, ctx.tasks
    path = rt.path
    # someone else changed rt.path
    if path is not s["path"]:
        if path is not None and path.get("src") == "draw" and not path.get("timed") and rt.mode != "FOLLOW":
            try:
                _adopt(ctx, s)
            except Exception:
                import traceback
                traceback.print_exc()
                s["path"] = None
        elif s["path"] is not None:
            if s["current"] is not None:
                s["last"] = s["current"]
            if s["proposed"] is not None:
                _drop_plan(ctx, s, "Cleared. Nothing moved." if path is None else None)
            s.update(current=None, proposed=None, prep=None, path=None, prev=None, running=False)
    # running?
    mine = s["path"] is not None and rt.path is s["path"]
    if mine and rt.mode == "FOLLOW":
        if not s["running"]:
            s["running"], s["events"] = True, []
            if s["proposed"] is not None:
                s["current"], s["proposed"], s["prev"] = s["proposed"], None, None
            tr = s["current"] or {}
            label = tr.get("label") or "your path"
            tasks.done_msg = f"Finished {label}."
            if tasks.task and tasks.task.get("status") == "running" and tasks.task.get("action") == "follow_path":
                hand = tasks.limb("right" if s["path"]["palm"].startswith("right") else "left")
                dur = (s["prep"] or {}).get("res", {}).get("duration")
                tasks.task["message"] = f"Following {label} with my {hand}" + (f" ({dur:.1f} s)." if dur else ".")
            s["rev"] += 1
    elif s["running"]:
        s["running"] = False
        s["rev"] += 1
        if s["events"] and tasks.task and tasks.task.get("status") == "done" and tasks.task.get("action") == "follow_path":
            tasks.task["message"] = tasks.task["message"].rstrip(".") + ": " + "; ".join(s["events"][:3]) + "."
    # the proposal's plan card went away (cancel, or another command took over): drop it, put the old path back
    if s["proposed"] is not None and s.get("plan_id"):
        p = ctx.store.get("plan")
        if not p or p.get("id") != s["plan_id"]:
            s["plan_id"] = None
            s["proposed"] = None
            _restore_prev(ctx, s)
            s["rev"] += 1
    # keep-out zones changed: check the path again
    zsig = repr([(z.get("id"), z.get("center"), z.get("size")) for z in (ctx.store.get("zones") or [])])
    if s["zones"] is None:
        s["zones"] = zsig
    elif zsig != s["zones"]:
        s["zones"] = zsig
        which, tr = _active(s)
        # an adopted stroke is re-checked by the zones feature itself (tasks.set_path_world); everything else here
        if tr is not None and mine and not s["running"] and s["path"].get("src") != "draw":
            try:
                op_edit(ctx, s, {"op": "edit", "label": tr.get("label") or ""})
            except T.TrajError:
                pass


# ---------------------------------------------------------------------------- state
def _ghost(ctx, s):
    prep = s["prep"]
    if prep is None:
        return None
    rt = ctx.rt
    d, idx, res = prep["dense"], prep["idx"], prep["res"]
    key_pos = {k: j for j, k in enumerate(idx)}
    hand = rt.data.mocap_pos[rt.hands_mid[prep["palm"]]]
    w = prep["world_traj"]
    qs = d["q"][idx]
    turns = len(qs) > 1 and float(np.max(np.abs(qs @ qs[0]))) > 0 and float(np.min(np.abs(qs @ qs[0]))) < 0.99996   # > ~1 degree
    g = {"id": prep["traj"]["id"], "pts": [_r(p, 3) for p in d["p"][idx]], "ok": [bool(o) for o in res["sample_ok"]],
         "q": [_r(q, 3) for q in qs] if turns else None,       # null: the wrist keeps kq[0] all along
         "t": [round(float(t), 2) for t in d["t"][idx]],
         "key": [key_pos.get(k, 0) for k in d["key"]], "keys": [_r(p["p"], 4) for p in w["points"]],
         "kq": [_r(d["q"][k], 4) for k in d["key"]],
         "grip": [None if not np.isfinite(d["grip"][k]) else round(float(d["grip"][k]), 2) for k in d["key"]],
         "approach": [_r(hand, 3), _r(d["p"][0], 3)], "approach_ok": all(res["approach_ok"]),
         "origin": _r(T.frame_origin(prep["traj"]["frame"], _poses(ctx)), 4) if T.frame_object(prep["traj"]["frame"]) else None,
         "surface": round(float(tasks_mod.TABLE_TOP), 4), "duration": round(float(d["duration"]), 2),
         "speed": T.default_speed(prep["kind"]), "hand": "right" if prep["palm"].startswith("right") else "left",
         "axis": [float(x) for x in getattr(getattr(rt, "profile", None), "ee_axis", (0, 0, 1))]}   # the gripper's approach axis
    return g


def _point_index(s, rt):
    path, prep = s["path"], s["prep"]
    if path is None or prep is None:
        return 0
    di = int(path.get("i", 0))
    keys = [list(prep["idx"]).index(k) if k in prep["idx"] else 0 for k in prep["dense"]["key"]]
    return int(max([j for j, k in enumerate(keys) if k <= di] or [0]))


def _objects_state(ctx):
    """Objects as the authoring UI needs them: keyframes land above an object's top; handles snap their height to it."""
    tasks = ctx.tasks
    out = []
    for n, o in tasks.objs.items():
        c = tasks.mug_center(n)
        out.append({"name": n, "pos": _r(c, 3), "r": round(float(o["r"]), 3), "top": round(float(c[2] + o["half_h"]), 3), "movable": True})
    for n, o in tasks.obstacles.items():
        out.append({"name": n, "pos": _r(o["pos"], 3), "r": round(float(o["r"]), 3), "top": round(float(o["pos"][2] + o["h"]), 3),
                    "movable": False})
    return out


@ext.state
def traj_state(ctx):
    s = _st(ctx)
    which, tr = _active(s)
    prep = s["prep"]
    out = {"current": s["current"], "proposed": s["proposed"], "rev": s["rev"], "reply": s["reply"], "running": s["running"],
           "ok": None if prep is None else bool(prep["ok"]), "why": None if prep is None or prep["ok"] else _why(prep),
           "i": 0, "progress": 0.0, "t": 0.0, "plan_id": s.get("plan_id"), "ghost": None, "info": [], "issues": [],
           "objects": _objects_state(ctx), "surface": round(float(tasks_mod.TABLE_TOP), 4)}
    if prep is not None and tr is not None:
        out["ghost"] = _ghost(ctx, s)
        out["info"] = T.describe(prep["world_traj"], prep["world"])
        out["issues"] = prep["res"]["issues"][:4]
        path = s["path"]
        if path is not None:
            out["i"] = _point_index(s, ctx.rt)
            dur = max(1e-6, prep["dense"]["duration"])
            out["t"] = round(float(path.get("t_now", 0.0)), 2) if s["running"] else 0.0
            out["progress"] = round(min(1.0, out["t"] / dur), 3) if s["running"] else 0.0
    return {KEY: out}
