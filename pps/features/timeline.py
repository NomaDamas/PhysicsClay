"""Teach timeline (UX 2.0 #F): re-show a single step of a taught task.

Reorder / delete / constraint toggles need no server code: the browser sends the edited spec through the existing
{type:"teach", cmd:"set_spec"}. Only "show again" needs help, because Teacher.start() forgets the spec:

    {type:"timeline", op:"reshow", index}    keep the current spec, record a fresh demonstration for step <index>
    {type:"timeline", op:"reshow_cancel"}    stop that recording and put the old spec back

When the recording ends (the usual {type:"teach", cmd:"stop"}, or the user saying "done"), the steps the Teacher
understood replace step <index>; the other steps stay in place and new constraints are merged in.
state.timeline = {reshow: {index, steps:[{object, target}], constraints} | null}
"""
import copy

from .. import ext

GRASP = ("grasp_top", "grasp_side")


def _plain(steps):
    return [{"object": s.get("object"), "target": s.get("target")} for s in steps or []]


@ext.handler("timeline")
def on_timeline(ctx, msg):
    tl, teacher = ctx.store.setdefault("timeline", {"reshow": None}), ctx.teacher
    op = msg.get("op")
    if op == "reshow":
        spec = teacher.spec
        try:
            i = int(msg.get("index"))
        except (TypeError, ValueError):
            return
        if not spec or not 0 <= i < len(spec["steps"]) or teacher.phase in ("recording", "practice"):
            return
        tl["reshow"] = {"index": i, "spec": copy.deepcopy(spec)}
        teacher.start()
        teacher.question = f"Show me step {i + 1} again, and say what you're doing. Say 'done' when finished."
    elif op == "reshow_cancel" and tl["reshow"]:
        saved = tl["reshow"]["spec"]
        tl["reshow"] = None
        teacher.set_spec(saved)


@ext.tick
def merge_reshow(ctx):
    tl, teacher = ctx.store.get("timeline"), ctx.teacher
    if not tl or not tl.get("reshow"):
        return
    if getattr(ctx.demo, "steps", None):  # the demo autopilot took over teaching
        tl["reshow"] = None
        return
    if teacher.phase == "recording":
        return
    r, tl["reshow"] = tl["reshow"], None
    if teacher.phase != "confirm" or not teacher.spec:  # cancelled or replaced by something else: drop it
        return
    old, new = r["spec"], teacher.spec
    i = r["index"]
    steps = _plain(old["steps"])
    steps[i:i + 1] = _plain(new["steps"])[:1]
    cons = [dict(c) for c in old["constraints"]]
    for c in new["constraints"]:
        if c["type"] in GRASP:
            cons = [x for x in cons if x["type"] not in GRASP]
        if not any(x["type"] == c["type"] and x.get("object") == c.get("object") for x in cons):
            cons.append(dict(c))
    teacher.set_spec({"goal": old.get("goal"), "steps": steps, "constraints": cons})
    teacher.question = f"Step {i + 1} updated from what you showed me. " + teacher.question


@ext.state
def timeline_state(ctx):
    r = (ctx.store.get("timeline") or {}).get("reshow")
    return {"timeline": {"reshow": {"index": r["index"], "steps": _plain(r["spec"]["steps"]),
                                    "constraints": r["spec"]["constraints"]} if r else None}}
