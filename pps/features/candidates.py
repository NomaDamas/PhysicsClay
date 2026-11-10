"""Candidates (UX 2.0 #C): "which one?". When a pick/move names something that matches more than one movable
object ("the block" with a red and a green block, "the thing", or a pointing ray that lands between two objects),
the robot does not guess: it numbers the candidates, asks, and waits. The answer comes back as
{"type": "choose", "id", "key"} (a number, a name/colour, free text such as "the red one", or a point), and the
original intent resumes with that object. Unambiguous requests pass straight through.

State: "choices": {"id", "question", "options": [{"key", "name", "label", "color", "pos", "uv"}]} or None."""
import re
import time

import numpy as np

from .. import ext
from ..tasks import Tasks

KINDS = {"mug": ("mug", "cup", "컵", "머그"), "block": ("block", "cube", "블록", "큐브")}
COLORS = {"red": ("red", "빨간", "빨강"), "green": ("green", "초록", "녹색"), "blue": ("blue", "파란", "파랑")}
GENERIC = ("object", "thing", "item", "something", "물건")
ORDINALS = [  # option index -> words that name it
    ("1", "one", "first", "하나", "첫"),
    ("2", "two", "second", "둘", "두 번째", "두번째"),
    ("3", "three", "third", "셋", "세 번째", "세번째"),
    ("4", "four", "fourth", "넷", "네 번째", "네번째"),
]
CANCEL = ("cancel", "never mind", "nevermind", "forget it", "neither", "none", "취소", "됐어", "아니야")
POINT_RADIUS = 0.10      # a point (marker / ray hit) within this many metres of an object counts as pointing at it
EXPIRE_S = 120.0


def kind_of(name):
    return next((k for k in KINDS if k in name), None)


def color_of(name):
    return next((c for c in COLORS if name.startswith(c + "_")), None)


def _has(t, words):
    return any(w in t for w in words)


def matches(tasks, text):
    """Movable objects the words could mean (all of them for 'the thing'); [] when the words name nothing movable."""
    t = str(text or "").lower()
    names = list(tasks.objs)
    colors = [c for c, w in COLORS.items() if _has(t, w)]
    kinds = [k for k, w in KINDS.items() if _has(t, w)]
    if colors:
        names = [n for n in names if color_of(n) in colors]
    if kinds:
        names = [n for n in names if kind_of(n) in kinds]
    if not colors and not kinds and not _has(t, GENERIC):
        return []
    return names


def _xy(p):
    return np.asarray(p, float)[:2]


def near_point(tasks, p, among=None):
    """Movable objects within POINT_RADIUS of a pointed spot, nearest first."""
    ds = sorted((float(np.linalg.norm(_xy(tasks.mug_center(n)) - _xy(p))), n) for n in (among or tasks.objs))
    return [n for d, n in ds if d < POINT_RADIUS]


def _spoken_list(labels):
    return labels[0] if len(labels) == 1 else ", ".join(labels[:-1]) + " or " + labels[-1]


def ask(ctx, msg, names, why=""):
    tasks = ctx.tasks
    # number them the way the robot sees them: left to right in its view (robot frame +y is its left)
    names = sorted(names, key=lambda n: -float(tasks.mug_center(n)[1]))
    options = []
    for i, n in enumerate(names):
        options.append({"key": str(i + 1), "name": n, "label": n.replace("_", " "), "color": color_of(n)})
    kinds = {kind_of(n) for n in names}
    noun = (kinds.pop() if len(kinds) == 1 else "one") or "one"
    spoken = _spoken_list([f"{o['key']}, the {o['label']}" for o in options])
    q = f"{why}Which {noun} do you mean: {spoken}?"
    tid = str(msg.get("id", "t?"))
    ctx.store["choices"] = {"id": tid, "question": q, "options": options, "msg": dict(msg), "t": time.time(),
                            "markers": {mk["id"]: mk["pos"].copy() for mk in tasks.markers}}
    tasks.set_task(tid, msg.get("action", "pick_place"), "asking", q)
    return True


@ext.pre_task
def disambiguate(ctx, msg):
    if msg.get("action") != "pick_place" or msg.get("_chosen"):
        return None
    tasks = ctx.tasks
    words = matches(tasks, msg.get("object"))
    # pointing at the object: a robot-view uv (raycast) or a world point
    hit = None
    if msg.get("object_pos") is not None:
        hit = np.asarray(msg["object_pos"], float)
    elif msg.get("object_uv") is not None and ctx.cam_params is not None:
        uv = msg["object_uv"]
        hit = tasks.raycast(ctx.cam_params, float(uv[0]), float(uv[1]), ctx.aspect)
    if hit is not None:
        near = near_point(tasks, hit, words or None) or near_point(tasks, hit)   # the point beats a default noun
        if len(near) == 1:
            msg["object"] = near[0]
            return None
        if len(near) > 1:
            return ask(ctx, msg, near, "You're pointing between two things. ")
    if len(words) > 1:
        return ask(ctx, msg, words)
    if len(words) == 1 and words[0] != tasks.obj_name(msg.get("object")):
        msg["object"] = words[0]      # e.g. "the cube" when only one block can be meant
    return None


def _word(t, w):
    if w.isascii():
        return re.search(rf"(?<![a-z]){re.escape(w)}(?![a-z])", t) is not None
    return w in t


def pick_option(options, text):
    """Free text / key -> option dict, or None when it names zero or several. Colour and kind words win over
    ordinals so "the red one" is the red block, not option one."""
    t = str(text if text is not None else "").strip().lower()
    if not t:
        return None
    for o in options:
        if t in (o["key"], o["name"], o["label"]):
            return o
    hits = [o for o in options if any(_word(t, w) for w in COLORS.get(o["color"], ()))]
    if not hits and len({kind_of(o["name"]) for o in options}) > 1:
        hits = [o for o in options if any(_word(t, w) for w in KINDS.get(kind_of(o["name"]), ()))]
    if not hits:
        hits = [options[i] for i, words in enumerate(ORDINALS[: len(options)]) if any(_word(t, w) for w in words)]
    if not hits and any(_word(t, w) for w in ("last", "마지막")):
        hits = [options[-1]]
    if not hits and any(_word(t, w) for w in ("left", "왼")):
        hits = [options[0]]
    if not hits and any(_word(t, w) for w in ("right", "오른")):
        hits = [options[-1]]
    if not hits:                                        # a word of the option's label ("the weave", "wiping")
        hits = [o for o in options if any(len(w) > 3 and (_word(t, w) or _word(t, w.rstrip("e") + "ing"))
                                          for w in str(o["label"]).lower().split())]
    return hits[0] if len(hits) == 1 else None


def resume(ctx, opt):
    ch = ctx.store.pop("choices", None)
    if not ch:
        return
    if ch["msg"].get("type") not in (None, "task"):      # another feature's question (P4: "was that a weave or a wipe?")
        ext.handle(ctx, dict(ch["msg"], **{ch.get("field") or "object": opt["name"]}, _chosen=True))
        return
    m = dict(ch["msg"], object=opt["name"], _chosen=True)
    m.pop("object_uv", None), m.pop("object_pos", None)
    ctx.tasks.set_task(ch["id"], m.get("action", "pick_place"), "planning", f"Okay, the {opt['label']}.")
    if not ext.intercept_task(ctx, m):     # other features (plan preview) still get to see it
        ctx.dispatch(m)


@ext.handler("choose")
def on_choose(ctx, msg):
    ch = ctx.store.get("choices")
    if not ch or (msg.get("id") is not None and str(msg.get("id")) != ch["id"]):
        return
    key = msg.get("key", msg.get("text"))
    if str(key or "").strip().lower() in CANCEL or msg.get("cancel"):
        ctx.store.pop("choices", None)
        ctx.tasks.set_task(ch["id"], ch["msg"].get("action", "pick_place"), "failed", "Okay, I left everything where it is.")
        return
    opt = None
    p = msg.get("pos") if all(o["name"] in ctx.tasks.objs for o in ch["options"]) else None
    if p is None and msg.get("uv") is not None and ctx.cam_params is not None and all(o["name"] in ctx.tasks.objs for o in ch["options"]):
        p = ctx.tasks.raycast(ctx.cam_params, float(msg["uv"][0]), float(msg["uv"][1]), ctx.aspect)
    if p is not None:
        near = near_point(ctx.tasks, p, [o["name"] for o in ch["options"]])
        opt = next((o for o in ch["options"] if near and o["name"] == near[0]), None)
    if opt is None:
        opt = pick_option(ch["options"], key)
    if opt is None:
        n = len(ch["options"])
        ctx.tasks.set_task(ch["id"], ch["msg"].get("action", "pick_place"), "asking",
                           f"Sorry, I didn't get which one. Say a number from 1 to {n}, or tap it. " + ch["question"])
        return
    resume(ctx, opt)


@ext.tick
def watch(ctx):
    ch = ctx.store.get("choices")
    if not ch:
        return
    t = ctx.tasks.task
    if not t or t.get("id") != ch["id"] or t.get("status") != "asking" or time.time() - ch["t"] > EXPIRE_S:
        ctx.store.pop("choices", None)      # superseded (new task, stop, reset) or abandoned
        return
    if any(o["name"] not in ctx.tasks.objs for o in ch["options"]):
        return                              # not a question about objects: pointing doesn't answer it
    # pointing while asked: a new or moved marker (body pointing sets P, a tap adds A-F) next to one candidate
    for mk in ctx.tasks.markers:
        old = ch["markers"].get(mk["id"])
        if old is not None and np.allclose(old, mk["pos"]):
            continue
        near = near_point(ctx.tasks, mk["pos"], [o["name"] for o in ch["options"]])
        if not near:
            ch["markers"][mk["id"]] = mk["pos"].copy()
            continue
        # that point only answered the question: put the markers back so "there" still means the spot from before
        tk = ctx.tasks
        if old is None:
            tk.markers = [m for m in tk.markers if m["id"] != mk["id"]]
        else:
            tk.set_marker(mk["id"], old)
        resume(ctx, next(o for o in ch["options"] if o["name"] == near[0]))
        return


@ext.state
def choices_state(ctx):
    ch = ctx.store.get("choices")
    if not ch:
        return {"choices": None}
    out = []
    for o in ch["options"]:
        if o["name"] not in ctx.tasks.objs:                 # an option that isn't an object (a skill kind): no badge
            out.append(dict(o, pos=None, uv=None))
            continue
        p = ctx.tasks.mug_center(o["name"])
        uv = Tasks.project(ctx.cam_params, p, ctx.aspect) if ctx.cam_params is not None else None
        out.append(dict(o, pos=[round(float(x), 3) for x in p], uv=uv))
    return {"choices": {"id": ch["id"], "question": ctx.tasks.task.get("message", ch["question"]) if ctx.tasks.task else ch["question"],
                        "options": out}}
