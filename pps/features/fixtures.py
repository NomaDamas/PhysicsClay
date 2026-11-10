"""Fixtures and multi-step plans (issue #48, docs/worlds.md, docs/PROTOCOL.md "v9 · Worlds").

* `{type: "fixture", op: "run" | "stop", name?, speed?}`: start / stop a conveyor (the only one when no name).
  `op: "lock" | "unlock", name`: jam a drawer / door (a check proves the robot reports it honestly);
  `op: "set", name, open: 0..1`: put a part at an opening (a setup probe: what is inside doesn't move with it).
* `{type: "task", action: "sequence", id, steps: [task, ...], text?, preview?}`: several tasks in a row ("open the top
  drawer, take the spoon, put it in the sink, close the drawer"). With `preview` it is a plan card first (the first step
  is checked for real; the later ones as they come, since the world changes under them); confirming runs the steps in
  order and stops at the first that fails, saying which and why. `state.sequence` follows it; the last step's result
  ends it as the task with the sequence's id.
* A single `open` / `close` / `pick_from` / `place_in` sent with `preview` becomes a one-step plan card the same way.
"""
import time

import numpy as np

from .. import ext

FIXTURE_ACTIONS = ("open", "close", "pick_from", "place_in")
KNOWN = FIXTURE_ACTIONS + ("pick_place", "reach", "press", "drive", "home", "stop", "reset_scene", "follow_path")
CTX = [None]


def _cap(s):
    return s[:1].upper() + s[1:] if s else s


def _reply(ctx, msg, status, text):
    if msg.get("id") is not None:
        ctx.tasks.set_task(str(msg["id"]), "fixture", status, text)


# ---------------------------------------------------------------------- {type: "fixture"}
@ext.handler("fixture")
def on_fixture(ctx, msg):
    tasks = ctx.tasks
    fx = getattr(tasks, "fx", None)
    op = msg.get("op")
    if fx is None:
        _reply(ctx, msg, "failed", "There are no fixtures in this world.")
        return
    if op in ("run", "stop"):
        name = str(msg.get("name") or "").strip().lower().replace(" ", "_")
        belts = [b for b in fx.belts if not name or b["name"] == name or name in b["name"] or name in ("conveyor", "belt", "line")]
        if not belts:
            _reply(ctx, msg, "failed", "There is no conveyor here." if not fx.belts else f"I don't know the conveyor '{msg.get('name')}'.")
            return
        for b in belts:
            b["running"] = op == "run"
            if msg.get("speed") is not None:
                try:
                    b["speed"] = float(np.clip(float(msg["speed"]), -0.3, 0.3))
                except (TypeError, ValueError):
                    pass
        _reply(ctx, msg, "done", f"{_cap(belts[0]['label'])} is {'running' if op == 'run' else 'stopped'}.")
        return
    p, why = tasks.find_part(msg.get("name") or "")
    if p is None:
        _reply(ctx, msg, "failed", why)
        return
    if op in ("lock", "unlock"):
        fx.lock(p, op == "lock")
        _reply(ctx, msg, "done", f"{_cap(p['label'])} is {'jammed' if op == 'lock' else 'free again'}.")
    elif op == "set":
        try:
            v = float(np.clip(float(msg.get("open", 0.0)), 0.0, 1.0))
        except (TypeError, ValueError):
            _reply(ctx, msg, "failed", "open must be a number from 0 to 1.")
            return
        lo, hi = p["range"]
        fx.set_q(p, lo + v * (hi - lo))
        import mujoco
        mujoco.mj_forward(ctx.rt.model, ctx.rt.data)
        _reply(ctx, msg, "done", f"{_cap(p['label'])} set to {v * 100:.0f}% open.")
    else:
        _reply(ctx, msg, "failed", f"Unknown fixture op '{op}'. Use run, stop, lock, unlock or set.")


# ---------------------------------------------------------------------- multi-step plans
def step_text(tasks, s):
    """One sentence per step (the plan card's list)."""
    a = s.get("action")
    if a in ("open", "close"):
        p, _ = tasks.find_part(s.get("fixture") or s.get("target") or "")
        what = p["label"] if p else (s.get("fixture") or "it")
        amt = s.get("amount")
        try:
            pct = "" if a == "close" or amt in (None, "") else f" to {float(amt) * (100 if float(amt) <= 1 else 1):.0f}%"
        except (TypeError, ValueError):
            pct = ""
        return f"{_cap(a)} {what}{pct}"
    if a == "pick_from":
        c, _ = tasks.find_container(s.get("container") or s.get("target") or "")
        return f"Take {s.get('object') or 'it'} out of {c['label'] if c else s.get('container') or 'there'}"
    if a == "place_in":
        c, _ = tasks.find_container(s.get("container") or s.get("target") or "")
        return f"Put {s.get('object') or 'it'} in {c['label'] if c else s.get('container') or 'there'}"
    if a == "pick_place":
        return f"Move {s.get('object') or 'it'} to {s.get('target') or 'the spot'}"
    if a == "drive":
        return "Drive the base"
    if a == "home":
        return "Go back to my start pose"
    return _cap(str(a or "step").replace("_", " "))


def check_step(tasks, s):
    """Can this step be understood at all (the robot checks its reach when it gets there)? -> why not, or None."""
    a = s.get("action")
    if a not in KNOWN:
        return f"I don't know how to '{a}'."
    caps = getattr(tasks.rt, "capabilities", ())
    if a in FIXTURE_ACTIONS + ("pick_place",) and "pick_place" not in caps:
        return f"The {getattr(tasks.rt, 'label', 'robot')} can't grasp things."
    if a in ("open", "close"):
        p, why = tasks.find_part(s.get("fixture") or s.get("target") or "")
        return None if p is not None else why
    if a in ("pick_from", "place_in"):
        c, why = tasks.find_container(s.get("container") or s.get("target") or "")
        if c is None:
            return why
        if s.get("object") and tasks.obj_name(s["object"]) is None:
            return f"I don't see '{s['object']}'."
    if a == "drive" and "drive_base" not in caps:
        return f"The {getattr(tasks.rt, 'label', 'robot')} has no base to drive."
    return None


def _clean_steps(steps):
    out = []
    for s in list(steps or [])[:12]:
        if isinstance(s, dict) and s.get("action"):
            out.append({k: v for k, v in s.items() if k not in ("type", "id", "preview") and v not in (None, "")})
    return out


def propose(ctx, msg):
    """A plan card for a sequence (or a single fixture action): steps, a real check of the first one, run on confirm."""
    tasks = ctx.tasks
    tid = str(msg.get("id", f"seq-{time.time():.3f}"))
    steps = _clean_steps(msg.get("steps"))
    if not steps:
        tasks.set_task(tid, "sequence", "failed", "A plan needs at least one step.")
        return
    texts = [step_text(tasks, s) for s in steps]
    intent = msg.get("text") or ("; ".join(texts) + ".")
    plan = {"id": tid, "anchor": tid, "t": time.time(), "seq": True, "action": "sequence", "constraints": [], "reasons": [],
            "waypoints": [], "wp_ok": [], "target": None, "carry": None, "alternative": None, "hand": None, "intent": _cap(intent),
            "steps": [{"text": t, "at": round(i / len(steps), 3)} for i, t in enumerate(texts)],
            "run": {"type": "task", "action": "sequence", "id": tid, "steps": steps, "text": msg.get("text")}}
    bad = next(((i, w) for i, w in ((i, check_step(tasks, s)) for i, s in enumerate(steps)) if w), None)
    if bad is None:                           # the first step for real (reach, grasp, what's in the way)
        try:
            from .plan import _dry_run
            first = dict(steps[0], id=tid)
            if first["action"] in FIXTURE_ACTIONS + ("pick_place", "reach"):
                verdict, cap = _dry_run(ctx, first)
                if verdict.get("status") == "failed":
                    bad = (0, verdict.get("message"))
                elif cap.get("path"):
                    from .plan import _decimate
                    pts = np.asarray(cap["path"].get("pts", []), float)
                    if len(pts):
                        plan["waypoints"], plan["wp_ok"] = _decimate(pts, list(cap["path"].get("ok", [])) or [True] * len(pts))
        except Exception as e:                # a plan card must never run anything unconfirmed
            bad = (0, f"I couldn't check the first step ({e}).")
    if bad is None:
        plan.update(ok=True, phase="proposing", verdict="I can do this",
                    reasons=["The first step is within reach; I'll check each next step when I get to it."] if len(steps) > 1 else
                    ["Every waypoint is within reach."],
                    message=plan["intent"])
        say = plan["intent"] + " Say go, or tap Run."
        tasks.set_task(tid, steps[0]["action"] if len(steps) == 1 else "sequence", "proposed", say, "waiting for you")
    else:
        i, why = bad
        plan.update(ok=False, phase="blocked", verdict=f"I can't — step {i + 1}" if len(steps) > 1 else "I can't do this yet",
                    reasons=[why], message=why if len(steps) == 1 else f"Step {i + 1} ({texts[i].lower()}): {why}")
        say = plan["message"]
        tasks.set_task(tid, steps[0]["action"] if len(steps) == 1 else "sequence", "failed", say)
    plan["say"] = say
    ctx.store["plan"] = plan


@ext.pre_task
def fixture_previews(ctx, msg):
    """Previewed fixture actions and sequences become plan cards (plan.py handles the single motions)."""
    CTX[0] = ctx
    a = msg.get("action")
    if a == "sequence":
        if msg.get("preview"):
            propose(ctx, msg)
        else:
            start(ctx, msg)
        return True
    if a in FIXTURE_ACTIONS and msg.get("preview"):
        step = {k: v for k, v in msg.items() if k not in ("type", "id", "preview")}
        propose(ctx, {"id": msg.get("id"), "steps": [step]})
        return True
    return None


@ext.action("sequence")
def run_sequence(msg):
    """The confirmed plan card (plan.py on_plan -> the app's dispatch) or a direct sequence task."""
    if CTX[0] is not None:
        start(CTX[0], msg)


def start(ctx, msg):
    tid = str(msg.get("id", f"seq-{time.time():.3f}"))
    steps = _clean_steps(msg.get("steps"))
    if not steps:
        ctx.tasks.set_task(tid, "sequence", "failed", "A plan needs at least one step.")
        return
    texts = [step_text(ctx.tasks, s) for s in steps]
    ctx.store["sequence"] = {"id": tid, "steps": steps, "texts": texts, "i": -1, "sub": None, "status": "running",
                             "results": [None] * len(steps), "message": "", "t": time.time()}
    p = ctx.store.get("plan")
    if p and p.get("id") == tid:
        p.update(phase="running", progress=0.0, step_i=0)
    ctx.tasks.set_task(tid, "sequence", "running", f"Step 1 of {len(steps)}: {texts[0]}.")
    _next(ctx)


def _next(ctx):
    sq = ctx.store["sequence"]
    sq["i"] += 1
    i = sq["i"]
    if i >= len(sq["steps"]):
        _finish(ctx, "done", " ".join(r for r in sq["results"] if r) or "Done.")
        return
    sub = f"{sq['id']}#{i + 1}"
    sq["sub"] = sub
    msg = dict(sq["steps"][i], type="task", id=sub)
    p = ctx.store.get("plan")
    if p and p.get("id") == sq["id"]:
        p["step_i"], p["progress"] = i, round(i / len(sq["steps"]), 3)
    if hasattr(ctx, "dispatch"):
        ctx.dispatch(msg)


def _finish(ctx, status, message):
    sq = ctx.store.get("sequence")
    if sq is None:
        return
    sq["status"], sq["message"] = status, message
    ctx.tasks.set_task(sq["id"], "sequence", status, message)
    p = ctx.store.get("plan")
    if p and p.get("id") == sq["id"]:
        p.update(phase=status, message=message)
        if status == "done":
            p["progress"], p["step_i"] = 1.0, len(p.get("steps") or [1]) - 1


@ext.tick
def sequence_tick(ctx):
    CTX[0] = ctx
    sq = ctx.store.get("sequence")
    if not sq or sq["status"] != "running":
        return
    t = ctx.tasks.task or {}
    if t.get("id") == sq["sub"]:
        st = t.get("status")
        if st == "done":
            sq["results"][sq["i"]] = t.get("message", "")
            _next(ctx)
        elif st in ("failed", "asking"):
            n = len(sq["steps"])
            _finish(ctx, "failed", (f"Step {sq['i'] + 1} of {n} ({sq['texts'][sq['i']].lower()}) didn't work: " if n > 1 else "")
                    + (t.get("message") or "it failed."))
        elif st == "running":
            p = ctx.store.get("plan")
            rt = ctx.rt
            if p and p.get("id") == sq["id"] and rt.path is not None:
                frac = rt.path["i"] / max(1, len(rt.path["pts"]) - 1)
                p["progress"] = round((sq["i"] + min(1.0, frac)) / len(sq["steps"]), 3)
    elif t.get("id") not in (sq["id"], sq["sub"]) and time.time() - sq["t"] > 0.5 and t.get("status") in ("running", "done", "failed"):
        _finish(ctx, "failed", f"Stopped at step {sq['i'] + 1}: another command took over.")


@ext.state
def sequence_state(ctx):
    sq = ctx.store.get("sequence")
    if not sq:
        return {"sequence": None}
    return {"sequence": {"id": sq["id"], "status": sq["status"], "i": sq["i"], "n": len(sq["steps"]), "message": sq["message"],
                         "steps": [{"text": t, "result": r} for t, r in zip(sq["texts"], sq["results"])]}}
