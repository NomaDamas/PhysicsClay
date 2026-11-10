"""P4 Skills from demonstration on the server (docs/skills.md, issue #32) and the demo packs' live parts (docs/packs.md).

A demonstration recorded in watch `learn` mode becomes a parametric skill (pps/skills): when `learn` stops, the
demonstration is ranked (weave, wipe, stir, press, spot; "which one?" when two fit equally, through the candidates
question), its parameters are fitted, the skill is saved (skills/<name>.json) and, when this scene has something to apply
it to, it is compiled onto that geometry and proposed as a P2 trajectory (ghost + plan card) with one sentence to
confirm: "I'll weld the L seam (22 cm): 5 mm zigzag at 2 Hz, 10 mm/s, 10° push angle. Right?". Corrections ("wider",
"slower", "4 mm") edit the parameters, recompile and re-propose. "Go" runs it.

Messages ({type: "skill", ...}; every op answers in state.skillbook.reply = {id, op, ok, message, ...}):
  op:"extract", demo? (default: the last one), kind?, name?, target?, propose? (default true)
  op:"apply", name, target?, overrides? {param: value}, preview? (default true), task_id?
  op:"edit", name? (default: the skill on screen), changes? {dotted.param: value | {mul} | {add}}, text? ("wider")
  op:"list"  op:"delete", name  op:"bead_clear"  op:"pack_demo" (play the loaded pack's synthetic demonstration)
  source: "agent" on any op: the agent speaks for itself (the robot doesn't say the sentence too)

State: skillbook = {skills: [summary + ok (this robot can)], current: {name, kind, describe, target, plan, phase},
                    reply, rev, beads: [{id, seam, r, pts, live}], seams: [{name, label, points}], pack, spot}

The weld bead: while a weave_seam trajectory runs, the torch tip's actual position (site of the end effector, so the
weave is in it) is appended to a bead whenever it is on the seam (within the weave width + 8 mm, below standoff + 12 mm);
the web draws it as a growing tube, the robot view as capsules. No arc or melt physics.
Packs: when a pack's world is built, its setup runs once (a sentence, standing rules such as the workshop's handover);
in the gym the barbell follows the lifter's hands (kinematic mode only: with P0 physics it is a 20 kg free body).
"""
import math
import time

import numpy as np

from .. import ext
from .. import skills as SK
from .. import tasks as tasks_mod
from ..human import demo as demo_mod

KEY = "skillbook"
BEAD_STEP = 0.0015          # m between bead points
BEAD_MAX = 700
LIFTS = ("squat", "press", "curl")


def _st(ctx):
    s = ctx.store.get(KEY)
    if s is None:
        s = ctx.store[KEY] = {"current": None, "reply": None, "rev": 0, "demo_sig": None, "beads": [], "world": None,
                              "pack_rules": [], "list": None, "list_t": 0.0, "spot": None, "job": None, "bar": None,
                              "seq": 0}
    return s


def _r(p, n=3):
    return [round(float(x), n) for x in p]


def _say(ctx, text, msg=None):
    if (msg or {}).get("source") == "agent":
        return
    b = getattr(ctx, "brain", None)
    if b is not None and hasattr(b, "utter") and text:
        try:
            b.utter(text, "robot")
        except Exception:
            pass


def _reply(s, msg, ok, message, **kw):
    s["reply"] = dict({"id": msg.get("id"), "op": msg.get("op"), "ok": bool(ok), "message": message, "t": round(time.time(), 2)}, **kw)
    s["rev"] += 1


def _tid(s, what):
    s["seq"] += 1
    return f"skill-{what}-{s['seq']}-{int(time.time() * 1000) % 100000}"


def _physics(ctx):
    return getattr(ctx.tasks, "plant", None) is not None


def _caps(ctx):
    return tuple(getattr(ctx.rt, "capabilities", ()) or ())


# ---------------------------------------------------------------------------- the scene as skills see it
def _spec(ctx):
    w = getattr(ctx, "world", None)
    return (w.spec if w is not None else None) or {}


def skill_world(ctx, palm=None):
    from ..world import scene as scene_mod
    tasks, rt = ctx.tasks, ctx.rt
    spec = _spec(ctx)
    so = {o["name"]: o for o in spec.get("objects") or []}
    held = set(v for v in getattr(tasks, "holding", {}).values() if v)

    def entry(n, pos, r, h, movable):
        kind = tasks.kinds.get(n) or (so.get(n) or {}).get("kind") or ""
        shape = (scene_mod.KINDS.get(kind) or (None, None, "cyl"))[2]
        return {"pos": _r(pos, 4), "r": float(r), "h": float(h), "kind": kind, "size": (so.get(n) or {}).get("size"),
                "shape": "box" if shape == "box" else "cyl", "movable": movable}
    objs = {n: entry(n, tasks.mug_center(n), o["r"], o["half_h"], True) for n, o in tasks.objs.items() if n not in held}
    objs.update({n: entry(n, o["pos"], o["r"], o["h"], False) for n, o in tasks.obstacles.items()})
    palm = palm or (rt.hands[0] if "right_palm" not in rt.hands else "right_palm")
    return {"surface": float(tasks_mod.TABLE_TOP), "objects": objs, "seams": spec.get("seams") or {},
            "welded": [b["seam"] for b in _st(ctx)["beads"] if b.get("done")], "caps": list(_caps(ctx)),
            "hand": _r(rt.data.mocap_pos[rt.hands_mid[palm]], 4), "regions": {k: list(v) for k, v in tasks_mod.REGIONS.items()},
            "markers": {m["id"]: _r(m["pos"], 4) for m in getattr(tasks, "markers", [])}}


# ---------------------------------------------------------------------------- apply (compile onto this scene)
def _plan_skill(ctx, tid, sk, out):
    """Make the plan card speak for the skill: the one sentence to confirm, and the skill on the card (web chips)."""
    p = ctx.store.get("plan")
    if not p or p.get("id") != tid:
        return None
    if p.get("phase") == "proposing":
        say = f"{out['say']} Right? Say go, or tell me what to change."
        p["say"], p["message"] = say, out["say"]
        ctx.tasks.set_task(tid, "follow_path", "proposed", say, "waiting for you")
    p["skill"] = {"name": sk["name"], "kind": sk["kind"], "describe": SK.describe(sk), "label": SK.KINDS[sk["kind"]]["label"],
                  "target": out.get("target")}
    return p


def apply(ctx, s, sk, target=None, preview=True, tid=None, msg=None):
    """Compile `sk` on this scene and propose it (preview) or run it. -> (ok, sentence, extra)."""
    msg = msg or {}
    rt = ctx.rt
    palm = "right_palm" if "right_palm" in rt.hands else rt.hands[0]
    out = SK.compile(sk, skill_world(ctx, palm), target)
    tid = tid or _tid(s, "apply")
    if out["kind"] == "trajectory":
        tr = out["traj"]
        if not preview and out.get("press") and _physics(ctx) and hasattr(ctx.tasks, "press"):
            m = {"type": "task", "id": tid, "action": "press", **out["press"]}     # P0: the admittance press itself
            if not ext.intercept_task(ctx, m):
                ctx.dispatch(m)
            s["current"] = {"name": sk["name"], "kind": sk["kind"], "describe": SK.describe(sk), "target": out["target"], "plan": tid,
                            "phase": "running", "preview": False}
            return True, out["say"], {"plan": tid}
        rid = f"{tid}-tj"
        ext.handle(ctx, {"type": "traj", "op": "propose" if preview else "set", "run": not preview, "traj": tr,
                         "task_id": tid, "id": rid})
        r = (ctx.store.get("traj") or {}).get("reply") or {}
        if r.get("id") != rid:
            raise SK.SkillError("trajectories aren't available on this server")
        s["current"] = {"name": sk["name"], "kind": sk["kind"], "describe": SK.describe(sk), "target": out["target"],
                        "plan": tid if preview else r.get("task"), "phase": "proposed" if preview else "running", "preview": preview,
                        "traj": r.get("traj"), "ok": bool(r.get("ok"))}
        if not r.get("ok"):
            why = r.get("message") or "the path is blocked"
            s["current"]["phase"] = "blocked"
            return False, f"{out['say']} But I can't do it yet: {why[:1].lower()}{why[1:]}", {"plan": tid, "why": r.get("why")}
        if preview:
            _plan_skill(ctx, tid, sk, out)
            return True, f"{out['say']} Right?", {"plan": tid, "duration": r.get("duration"), "n": r.get("n")}
        return True, out["say"], {"task": r.get("task")}
    if out["kind"] == "watch":
        m = dict(out["msg"], id=f"{tid}-w", quiet=True)
        ext.handle(ctx, m)
        wr = ((ctx.store.get("watch") or {}).get("reply") or {})
        if not wr.get("ok", True) and wr.get("id") == m["id"]:
            raise SK.SkillError(wr.get("message") or "watching isn't available")
        mf = float(m.get("max_force") or 40.0)
        if _physics(ctx):                        # P0: soften the arm and cap the end-effector force while spotting
            ctx.push({"type": "physics", "op": "set", "params": {"impedance": True, "force_limit": mf}})
        s["spot"] = {"name": sk["name"], "max_force": mf, "impedance": _physics(ctx), "t": time.time()}
        s["current"] = {"name": sk["name"], "kind": sk["kind"], "describe": SK.describe(sk), "target": out["target"], "plan": None,
                        "phase": "running", "preview": False}
        return True, out["say"], {}
    m = dict(out["msg"], id=tid)                  # mirror_motion: play the recorded whole-body motion
    if preview:
        m["preview"] = True
    if not ext.intercept_task(ctx, m):
        ctx.dispatch(m)
    s["current"] = {"name": sk["name"], "kind": sk["kind"], "describe": SK.describe(sk), "target": out["target"], "plan": tid,
                    "phase": "proposed" if preview else "running", "preview": preview}
    return True, out["say"], {"plan": tid}


# ---------------------------------------------------------------------------- extract (a demonstration -> a skill)
KIND_WORDS = {"weave_seam": "a weave along a seam", "wipe": "wiping", "stir": "stirring", "press": "pressing", "spot": "a lift to spot"}


def _ask_kind(ctx, s, msg, ranked, demo):
    """Two kinds fit about equally: ask with the candidates question card ("which one?")."""
    tid = str(msg.get("task_id") or _tid(s, "ask"))
    opts = [{"key": str(i + 1), "name": r["kind"], "label": SK.KINDS[r["kind"]]["label"].lower(), "color": None}
            for i, r in enumerate(ranked[:3])]
    spoken = " or ".join(f"{o['key']}, {KIND_WORDS.get(o['name'], o['label'])}" for o in opts)
    q = f"Was that {spoken}?"
    resume = {k: v for k, v in msg.items() if k not in ("kind",)}
    resume.update(type="skill", op="extract", demo=demo["name"], task_id=tid)
    ctx.store["choices"] = {"id": tid, "question": q, "options": opts, "msg": resume, "field": "kind", "t": time.time(), "markers": {}}
    ctx.tasks.set_task(tid, "skill", "asking", q)
    _say(ctx, q, msg)
    return q


def op_extract(ctx, s, msg):
    name = msg.get("demo") or ((ctx.store.get("watch") or {}).get("last_demo") or {}).get("name")
    demo = demo_mod.load(name) if name else None
    if demo is None:
        _reply(s, msg, False, "I have no demonstration to learn from yet. Say 'watch me' and show me.")
        return
    world = skill_world(ctx)
    kind = msg.get("kind")
    ranked = SK.rank(demo, world) if not kind else []
    if not kind and not ranked:
        _reply(s, msg, False, f"I watched {demo.get('duration', 0):.0f} s of {demo.get('activity') or 'motion'}, but it isn't a skill I "
                              "know how to learn (a weave, a wipe, a stir, a press or a lift).", demo=name)
        return
    if not kind and SK.ambiguous(ranked) and not msg.get("_chosen"):
        q = _ask_kind(ctx, s, msg, ranked, demo)
        _reply(s, msg, True, q, question=q, options=[r["kind"] for r in ranked[:3]], demo=name)
        return
    sk = SK.extract(demo, kind=kind, name=msg.get("name") or demo.get("name"), world=world)
    SK.save(sk)
    s["list"] = None
    learned = f"Learned '{sk['name']}', {SK.KINDS[sk['kind']]['label'].lower()}: {SK.describe(sk)}."
    notes = list(sk.get("notes") or [])
    if msg.get("propose", True) is False:
        _reply(s, msg, True, learned, skill=SK.summary(sk), notes=notes, demo=name)
        return
    try:
        ok, said, extra = apply(ctx, s, sk, msg.get("target"), preview=True, tid=msg.get("task_id"), msg=msg)
    except SK.SkillError as e:
        text = f"{learned} {str(e)[:1].upper()}{str(e)[1:]}."
        _reply(s, msg, True, text, skill=SK.summary(sk), notes=notes, proposed=False, demo=name)
        _say(ctx, text, msg)
        return
    text = said if ok else said
    _reply(s, msg, ok, text, skill=SK.summary(sk), notes=notes, proposed=ok, demo=name, **extra)
    _say(ctx, (f"{notes[0]}. " if notes and "angle" in notes[0] else "") + text, msg)


def op_apply(ctx, s, msg):
    sk = SK.load(msg.get("name") or (s["current"] or {}).get("name") or "")
    if sk is None:
        have = ", ".join(x["name"] for x in _listing(s)) or "none yet"
        _reply(s, msg, False, f"I don't have a skill called '{msg.get('name')}' (I have: {have}).")
        return
    if isinstance(msg.get("overrides"), dict) and msg["overrides"]:
        sk, _ = SK.edit(sk, {k: v for k, v in msg["overrides"].items()})
    ok, said, extra = apply(ctx, s, sk, msg.get("target"), preview=msg.get("preview", True) is not False, tid=msg.get("task_id"), msg=msg)
    _reply(s, msg, ok, said, skill=SK.summary(sk), **extra)
    _say(ctx, said, msg)


def op_edit(ctx, s, msg):
    cur = s["current"] or {}
    name = msg.get("name") or cur.get("name")
    sk = SK.load(name) if name else None
    if sk is None:
        _reply(s, msg, False, "There is no skill on screen to change. Name one, or show me first.")
        return
    if sk.get("example"):                         # examples stay as shipped: the edit becomes the person's own copy
        sk = dict(sk, name=f"my_{sk['name']}")
        sk.pop("example", None)
    if isinstance(msg.get("changes"), dict) and msg["changes"]:
        sk2, words = SK.edit(sk, msg["changes"])
    elif msg.get("text"):
        sk2, words = SK.edit_text(sk, str(msg["text"]))
    else:
        _reply(s, msg, False, "Say what to change: wider, narrower, slower, faster, more push, closer, harder…")
        return
    SK.save(sk2)
    s["list"] = None
    changed = "; ".join(words) if words else "nothing changed (already at the limit)"
    if cur.get("name") in (name, sk2["name"]) and cur.get("phase") in ("proposed", "blocked") and cur.get("preview", True):
        tgt = cur.get("target") or {}
        target = tgt.get("seam") or tgt.get("region") or tgt.get("container") or tgt.get("target") or tgt.get("object")
        ok, said, extra = apply(ctx, s, sk2, target, preview=True, tid=cur.get("plan"), msg=msg)
        text = f"Okay: {changed}. {said}"
        _reply(s, msg, ok, text, skill=SK.summary(sk2), changed=words, **extra)
    else:
        text = f"Okay: {changed}. '{sk2['name']}' is now {SK.describe(sk2)}."
        _reply(s, msg, True, text, skill=SK.summary(sk2), changed=words)
    _say(ctx, text, msg)


def _listing(s):
    if s["list"] is None or time.time() - s["list_t"] > 3.0:
        s["list"], s["list_t"] = SK.listing(), time.time()
    return s["list"]


def op_list(ctx, s, msg):
    caps = _caps(ctx)
    items = [SK.summary(k, all(c in caps for c in SK.needs(k))) for k in _listing(s)]
    _reply(s, msg, True, f"{len(items)} skills.", skills=items)


def op_delete(ctx, s, msg):
    ok = SK.delete(msg.get("name") or "")
    s["list"] = None
    _reply(s, msg, ok, f"Deleted '{msg.get('name')}'." if ok else f"There's no skill '{msg.get('name')}' of yours to delete (examples stay).")


def op_bead_clear(ctx, s, msg):
    s["beads"] = []
    _reply(s, msg, True, "Cleared the weld beads.")


# ---------------------------------------------------------------------------- pack demos (no camera needed)
def op_pack_demo(ctx, s, msg):
    from ..human import synth
    from ..world import packs
    pk = packs.find(_spec(ctx).get("pack") or msg.get("pack") or "")
    d = (pk or {}).get("demo")
    if not d:
        _reply(s, msg, False, "Load a demo pack first (weld cell, gym or workshop).")
        return
    kw = {k: v for k, v in d.items() if k not in ("kind", "seconds", "name", "hold", "mode")}
    if "stall_at" in kw:
        kw["stall_at"] = tuple(kw["stall_at"])
    frames = synth.frames(d["kind"], float(d.get("seconds") or 6.0), **kw)
    hold = float(d.get("hold") or 0.5)
    if pk["id"] == "weld_cell":
        ext.handle(ctx, {"type": "watch", "op": "learn_start", "name": d.get("name") or "weave_demo", "id": f"{msg.get('id')}-l"})
        s["job"] = {"stop": time.time() + len(frames) / 20.0 + 0.6, "op": "learn_stop"}
        text = "Playing a synthetic welder: a 5 mm weave at 2 Hz, 1 cm/s. I'm learning it."
    elif pk["id"] == "gym":
        sk = SK.load("spot_bench")
        apply(ctx, s, sk, None, preview=False, msg=msg)
        text = "Playing a synthetic lifter who gets stuck on the third rep; I'm spotting."
    else:
        text = "Playing a synthetic person holding an open hand out."
    ext.handle(ctx, {"type": "copresence_feed", "frames": frames, "fps": 20, "hold": hold})
    _reply(s, msg, True, text, seconds=round(len(frames) / 20.0, 1))
    _say(ctx, text, msg)


OPS = {"extract": op_extract, "apply": op_apply, "edit": op_edit, "list": op_list, "delete": op_delete,
       "bead_clear": op_bead_clear, "pack_demo": op_pack_demo}


@ext.handler("skill")
def on_skill(ctx, msg):
    s = _st(ctx)
    fn = OPS.get(msg.get("op"))
    if fn is None:
        _reply(s, msg, False, f"Unknown skill op '{msg.get('op')}'. Use extract, apply, edit, list, delete, bead_clear or pack_demo.")
        return
    try:
        fn(ctx, s, msg)
    except SK.SkillError as e:
        t = str(e)
        _reply(s, msg, False, t[:1].upper() + t[1:] + ("" if t.endswith(".") else "."))


# ---------------------------------------------------------------------------- per frame
def _new_world(ctx, s):
    """A world was built: beads and the skill on screen belonged to the old one; run the new pack's setup once."""
    w = getattr(ctx, "world", None)
    if w is None or s["world"] is w:
        return
    first = s["world"] is None
    s["world"] = w
    s["beads"], s["current"], s["bar"] = [], None, None
    wt = ctx.store.get("watch")
    for rid in s["pack_rules"]:                 # the previous pack's standing rules go with it
        if wt is not None:
            ext.handle(ctx, {"type": "watch", "op": "rule_remove", "rule": rid, "id": f"pack-rm-{rid}"})
    s["pack_rules"] = []
    if s.get("spot"):
        _spot_off(ctx, s)
    pack = w.spec.get("pack")
    if not pack or first:
        return
    from ..world import packs
    pk = packs.find(pack)
    if pk is None:
        return
    st = pk.get("setup") or {}
    for r in st.get("rules") or []:
        rid = f"pack-{pack}-{len(s['pack_rules'])}"
        ext.handle(ctx, {"type": "watch", "op": "rule", "text": r.get("text"), "id": rid})
        rep = ((ctx.store.get("watch") or {}).get("reply") or {})
        if rep.get("id") == rid and rep.get("ok") and rep.get("rule"):
            s["pack_rules"].append(rep["rule"]["id"])
    if st.get("watch"):
        ext.handle(ctx, {"type": "watch", "op": "mode", "mode": st["watch"], "quiet": True, "id": f"pack-{pack}-mode"})
    if st.get("say"):
        _say(ctx, st["say"])


def _spot_off(ctx, s):
    if s["spot"] and s["spot"].get("impedance"):
        ctx.push({"type": "physics", "op": "set", "params": {"impedance": False}})
    s["spot"] = None


def _auto_extract(ctx, s):
    """learn just stopped (a new demonstration): extract a skill and propose it."""
    w = ctx.store.get("watch") or {}
    last = w.get("last_demo")
    sig = (last.get("name"), last.get("frames"), last.get("duration")) if last else None
    if sig == s["demo_sig"]:
        return
    s["demo_sig"] = sig
    if last and w.get("mode") != "learn":
        on_skill(ctx, {"type": "skill", "op": "extract", "demo": last["name"], "id": f"auto-{last['name']}"})


def _bead(ctx, s):
    tj = ctx.store.get("traj") or {}
    cur = tj.get("current") or {}
    meta = ((cur.get("meta") or {}).get("skill") or {})
    running = bool(tj.get("running")) and meta.get("kind") == "weave_seam"
    live = next((b for b in s["beads"] if b.get("live")), None)
    if not running:
        if live is not None:
            live["live"] = False
            live["done"] = len(live["pts"]) > 10
            if s["current"] and s["current"].get("phase") == "running":
                s["current"]["phase"] = "done"
            if live["done"] and (ctx.store.get("traj") or {}).get("current"):
                ext.handle(ctx, {"type": "traj", "op": "clear", "id": "skill-weld-done"})   # the bead shows what was done
        return
    if live is None or live.get("traj") != cur.get("id"):
        if live is not None:
            live["live"] = False
        live = {"id": f"bead{len(s['beads']) + 1}", "traj": cur.get("id"), "seam": meta.get("seam"), "pts": [], "live": True,
                "done": False, "r": float(meta.get("bead_r") or 0.003), "poly": np.asarray(meta.get("polyline") or [], float),
                "standoff": float(meta.get("standoff") or 0.015), "amp": float(((meta.get("params") or {}).get("weave") or {}).get("amp") or 0.0)}
        s["beads"].append(live)
        if s["current"]:
            s["current"]["phase"] = "running"
    path = getattr(ctx.rt, "path", None) or {}
    palm = path.get("palm") or ctx.rt.hands[0]
    tip = np.asarray(ctx.rt.data.site_xpos[ctx.rt.palm_sid[palm]], float)
    P = live["poly"]
    if len(P) < 2:
        return
    from ..skills import geom
    d, _ = geom.dist_to_polyline(tip, P)
    zs = float(np.max(P[:, 2]))
    if d > live["amp"] + 0.008 or tip[2] > zs + live["standoff"] + 0.012:
        return
    p = [round(float(tip[0]), 4), round(float(tip[1]), 4), round(zs + 0.6 * live["r"], 4)]   # the bead sits on the plate
    if not live["pts"] or math.dist(live["pts"][-1], p) >= BEAD_STEP:
        live["pts"].append(p)
        if len(live["pts"]) > BEAD_MAX:
            live["pts"] = live["pts"][::2]


def _bar(ctx, s):
    """Gym: is the lifter holding the bar (hands apart at bar width, level)? Kinematic mode: the barbell follows their
    hands while they do. With physics it is a 20 kg free body the (unsimulated) person can't lift, so it stays where it
    is; the hold is still reported (skillbook.bar, with follows: false)."""
    tasks = ctx.tasks
    if "barbell" not in getattr(tasks, "objs", {}):
        return
    o, phys = tasks.objs["barbell"], _physics(ctx)
    J = ((ctx.store.get("watch") or {}).get("model") or None)
    J = getattr(J, "J", None)
    held = False
    if J is not None and "l_wrist" in J and "r_wrist" in J:
        a, b = np.asarray(J["l_wrist"], float), np.asarray(J["r_wrist"], float)
        gap = float(np.linalg.norm(a - b))
        held = 0.3 < gap < 0.95 and abs(a[2] - b[2]) < 0.15
    floor = float(tasks_mod.TABLE_TOP) + float(o["half_h"])
    if held:
        mid = (a + b) / 2
        want = np.array([mid[0], mid[1], max(floor, mid[2])])
        if not phys:
            tasks.set_obj_pose("barbell", want - o["off"])
        s["bar"] = {"held": True, "pos": _r(want), "follows": not phys}
    elif s.get("bar") and s["bar"].get("held"):
        if not phys:
            tasks.set_obj_pose("barbell", o["home"])
        s["bar"] = {"held": False, "pos": _r(tasks.mug_center("barbell")), "follows": not phys}


@ext.tick
def skills_tick(ctx):
    s = _st(ctx)
    _new_world(ctx, s)
    _auto_extract(ctx, s)
    _bead(ctx, s)
    _bar(ctx, s)
    if s["spot"] and (ctx.store.get("watch") or {}).get("mode") != "assist":
        _spot_off(ctx, s)
        if s["current"] and s["current"].get("kind") == "spot":
            s["current"]["phase"] = "done"
    job = s.get("job")
    if job and time.time() >= job["stop"]:
        s["job"] = None
        if (ctx.store.get("watch") or {}).get("mode") == "learn":
            ext.handle(ctx, {"type": "watch", "op": job["op"], "id": "pack-demo-stop"})
    cur = s["current"]
    if cur and cur.get("phase") == "proposed" and cur.get("plan"):
        p = ctx.store.get("plan") or {}
        t = ctx.tasks.task or {}
        if p.get("id") != cur["plan"] and t.get("id") != cur["plan"]:
            cur["phase"] = "dropped"
        elif p.get("id") == cur["plan"] and p.get("phase") == "running":
            cur["phase"] = "running"


@ext.overlay
def paint_beads(ctx, scn):
    import mujoco
    s = ctx.store.get(KEY) or {}
    for b in s.get("beads") or []:
        pts = b.get("pts") or []
        if len(pts) < 2:
            continue
        step = max(1, len(pts) // 140)
        sel = pts[::step] + ([pts[-1]] if (len(pts) - 1) % step else [])
        n = len(sel)
        for i in range(n - 1):
            if scn.ngeom >= scn.maxgeom:
                return
            hot = b.get("live") and i > n - 12
            rgba = np.array([1.0, 0.55, 0.15, 1.0] if hot else [0.52, 0.44, 0.36, 1.0], dtype=np.float32)
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, float(b.get("r") or 0.003), np.asarray(sel[i], float),
                                 np.asarray(sel[i + 1], float))
            scn.ngeom += 1


# ---------------------------------------------------------------------------- state
@ext.state
def skills_state(ctx):
    s = _st(ctx)
    caps = _caps(ctx)
    spec = _spec(ctx)
    kinds = {str(o.get("kind")) for o in spec.get("objects") or []}
    fits = {"weave_seam": bool(spec.get("seams")), "spot": spec.get("pack") == "gym" or "barbell" in kinds,
            "stir": bool(kinds & {"bowl", "pot", "cup", "mug"}), "wipe": bool(kinds & {"plate", "steel_plate", "box", "tray"}),
            "press": False, "mirror_motion": True}
    items = []
    for k in _listing(s):
        if k["kind"] == "mirror_motion" and "imitate_video" not in caps:
            continue                              # whole-body motion only on bodies that can imitate
        it = SK.summary(k, all(c in caps for c in SK.needs(k)))
        it["fits"] = not k.get("example") or fits.get(k["kind"], False)    # examples show where they fit the scene
        items.append(it)
    seams = [{"name": n, "label": v.get("label"), "points": v.get("points")} for n, v in (spec.get("seams") or {}).items()]
    pack = None
    if spec.get("pack"):
        from ..world import packs
        pk = packs.find(spec["pack"])
        if pk:
            pack = {"id": pk["id"], "label": pk.get("label"), "examples": (pk.get("examples") or [])[:4], "physics": pk.get("physics"),
                    "demo": bool(pk.get("demo"))}
    cur = s["current"]
    return {KEY: {"skills": items[:40], "current": cur, "reply": s["reply"], "rev": s["rev"], "seams": seams, "pack": pack,
                  "beads": [{"id": b["id"], "seam": b.get("seam"), "r": b.get("r"), "live": bool(b.get("live")), "pts": b["pts"]}
                            for b in s["beads"] if b["pts"]],
                  "spot": s["spot"], "bar": s.get("bar"), "kinds": {k: v["label"] for k, v in SK.KINDS.items()}}}
