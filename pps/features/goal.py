"""Goal (#E): show the end state instead of the steps.

The browser sends where each movable object should end up, either arranged as ghosts in the 3D stage or read from a
photo of the desired layout. The server turns that into an ordered list of pick_place moves and runs them one by one:

* objects already within 3 cm of their target are skipped;
* targets are kept on the table, clear of the fixed obstacles and of each other;
* if a target is still occupied by another object, that object moves first, or is set aside when it is part of a cycle
  (a swap) or has no target of its own;
* a carry whose straight line passes an obstacle goes over it on a raised route;
* the sequence stops at the first failed step with that step's reason.

Messages (see docs/PROTOCOL.md, v4 "Goal"):
  {type:"goal", op:"set", targets:{name:[x, y]}, run?:bool}
  {type:"goal", op:"photo", jpg_b64, view?:"user"|"robot", offline?:bool, run?:bool}
  {type:"goal", op:"run"} | {op:"cancel"} | {op:"clear"}
State: goal = {status, message, targets, sequence, i, source, objects, obstacles, rev, photo}
"""
import base64
import json
import re
import ssl
import threading
import time
import urllib.request
from contextlib import contextmanager

import cv2
import mujoco
import numpy as np

from .. import ext
from ..spatial import API, ER_MODEL, gemini_key
from ..tasks import TABLE_TOP, TABLE_X, TABLE_Y

TOL = 0.03             # an object within 3 cm of its target is in place
GAP = 0.02             # extra clearance between planned spots so tasks.free_spot never moves a goal target
# Part of the table a photo maps onto: the front strip both hands reach (the back of the table is out of reach).
WORK_X, WORK_Y = (0.25, 0.47), (-0.36, 0.36)
ER_TIMEOUT, FLASH_TIMEOUT = 15, 10
MAX_JPG = 1_600_000


def _g(ctx):
    g = ctx.store.get("goal")
    if g is None:
        g = ctx.store["goal"] = {"status": "idle", "message": "", "targets": {}, "sequence": [], "i": 0,
                                 "source": None, "rev": 0, "photo": None, "tid": None, "dispatched": False,
                                 "retried": False, "fixing": False, "pending_photo": None, "run_after_photo": False}
    return g


def _set(g, status, message):
    g["status"], g["message"] = status, message


def _pretty(tasks, name):
    p = tasks.pretty(name)
    return p if p != name else "the " + name.replace("_", " ")


# ---------------------------------------------------------------------- geometry helpers
def _cur(tasks, name):
    return np.asarray(tasks.mug_center(name), float)[:2].copy()


def _on_table(p, r=0.0):
    m = 0.035 + r * 0.5
    return TABLE_X[0] + m < p[0] < TABLE_X[1] - m and TABLE_Y[0] + m < p[1] < TABLE_Y[1] - m


def _clamp(p, r=0.0):
    m = 0.04 + r * 0.5
    return np.array([np.clip(p[0], TABLE_X[0] + m, TABLE_X[1] - m), np.clip(p[1], TABLE_Y[0] + m, TABLE_Y[1] - m)])


def _seg_dist(p, a, b):
    ab = b - a
    t = 0.0 if float(ab @ ab) < 1e-9 else float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _blocker(p, r, others, gap=GAP):
    """others: [(name, xy, r)] -> name of the first one too close to p, else None."""
    for n, c, rr in others:
        if np.linalg.norm(np.asarray(p) - np.asarray(c)) < r + rr + 0.01 + gap:
            return n
    return None


def _free_near(p, r, others, reach_bias=True):
    """Nearest point to p on the table that clears every (name, xy, r) in others."""
    if _on_table(p, r) and _blocker(p, r, others) is None:
        return np.asarray(p, float)
    best = None
    for rad in np.arange(0.03, 0.30, 0.02):
        for ang in np.linspace(0, 2 * np.pi, 24, endpoint=False):
            c = np.array([p[0] + rad * np.cos(ang), p[1] + rad * np.sin(ang)])
            if not _on_table(c, r) or _blocker(c, r, others) is not None:
                continue
            score = rad + (0.6 * max(0.0, c[0] - 0.40) + 0.4 * max(0.0, abs(c[1]) - 0.30) if reach_bias else 0.0)
            if best is None or score < best[0]:
                best = (score, c)
        if best is not None and rad > best[0] + 0.02:
            break
    return None if best is None else best[1]


def _obstacles(tasks):
    return [(n, np.asarray(o["pos"], float)[:2], float(o["r"])) for n, o in tasks.obstacles.items()]


# ---------------------------------------------------------------------- planning
def plan(tasks, targets):
    """targets {name: xy} -> (sequence, notes, fixed targets). Pure: reads positions, moves nothing."""
    objs = list(tasks.objs)
    rad = {n: float(tasks.objs[n]["r"]) for n in objs}
    cur = {n: _cur(tasks, n) for n in objs}
    obst = _obstacles(tasks)
    notes, fixed = [], {}
    # 1. make every target reachable-ish, on the table and clear of obstacles, other targets and objects that stay put
    for n in objs:
        if n not in targets:
            continue
        p = _clamp(np.asarray(targets[n], float)[:2], rad[n])
        others = obst + [(m, q, rad[m]) for m, q in fixed.items()]
        q = _free_near(p, rad[n], others)
        if q is None:
            notes.append(f"There is no free spot for {_pretty(tasks, n)} near its goal; leaving it where it is.")
            continue
        who = _blocker(p, rad[n], others)
        if np.linalg.norm(q - p) > 0.01 and who:
            notes.append(f"Moved {_pretty(tasks, n)}'s spot {np.linalg.norm(q - p) * 100:.0f} cm to clear {_pretty(tasks, who)}.")
        elif np.linalg.norm(q - np.asarray(targets[n], float)[:2]) > 0.01:
            notes.append(f"Kept {_pretty(tasks, n)}'s spot on the table.")
        fixed[n] = q
    # 2. order the moves; free a target before using it
    pending = [n for n in objs if n in fixed and np.linalg.norm(cur[n] - fixed[n]) > TOL]
    skipped = [n for n in objs if n in fixed and n not in pending]
    sim = dict(cur)
    seq = []

    def occupant(n, p):
        return _blocker(p, rad[n], [(m, sim[m], rad[m]) for m in objs if m != n], gap=GAP / 2)

    for _ in range(4 * len(objs) + 4):
        if not pending:
            break
        ready = [n for n in pending if occupant(n, fixed[n]) is None]
        if ready:
            # prefer a move that frees someone else's target
            frees = [n for n in ready if any(m != n and _blocker(fixed[m], rad[m], [(n, sim[n], rad[n])], gap=GAP / 2) for m in pending)]
            n = (frees or ready)[0]
            seq.append({"object": n, "to": fixed[n], "kind": "place"})
            sim[n] = fixed[n]
            pending.remove(n)
            continue
        n0 = pending[0]
        b = occupant(n0, fixed[n0])
        others = obst + [(m, sim[m], rad[m]) for m in objs if m != b] + [(m, q, rad[m]) for m, q in fixed.items() if m != b]
        aside = _free_near(sim[b], rad[b], others)
        if aside is None:
            notes.append(f"I can't make room: {_pretty(tasks, b)} sits on {_pretty(tasks, n0)}'s goal and there is no free spot to set it aside.")
            break
        seq.append({"object": b, "to": aside, "kind": "aside"})
        sim[b] = aside
    for k, s in enumerate(seq):
        s.update(i=k, label=_pretty(tasks, s["object"]), status="pending", err_cm=None,
                 to=[round(float(s["to"][0]), 3), round(float(s["to"][1]), 3)])
    return seq, notes, fixed, skipped


def _describe(tasks, seq, skipped, notes):
    if not seq:
        msg = "Everything is already where you want it."
    else:
        parts = [(f"move {s['label']} out of the way" if s["kind"] == "aside" else f"put {s['label']} in its spot") for s in seq]
        msg = f"{len(seq)} move{'s' if len(seq) != 1 else ''}: " + ", then ".join(parts) + "."
    if skipped:
        msg += " " + " and ".join(_pretty(tasks, n) for n in skipped).capitalize() + (" is" if len(skipped) == 1 else " are") + " already in place."
    if notes:
        msg += " " + " ".join(notes)
    return msg


def _set_goal(ctx, targets, source, run):
    g, tasks = _g(ctx), ctx.tasks
    if g["status"] == "running":
        g["message"] = "I'm still arranging the last goal. Press Cancel first."
        return
    clean = {}
    for n, p in (targets or {}).items():
        key = str(n).strip().lower().replace(" ", "_")
        if key in tasks.objs and isinstance(p, (list, tuple)) and len(p) >= 2:
            try:
                clean[key] = [float(p[0]), float(p[1])]
            except (TypeError, ValueError):
                continue
    if not clean:
        g.update(targets={}, sequence=[], i=0, source=source, rev=g["rev"] + 1)
        _set(g, "failed", "I didn't get a goal spot for any object I can move.")
        return
    seq, notes, fixed, skipped = plan(tasks, clean)
    if not str(source).startswith("photo"):
        g["photo"] = None
    g.update(targets={n: [round(float(q[0]), 3), round(float(q[1]), 3)] for n, q in fixed.items()},
             sequence=seq, i=0, source=source, rev=g["rev"] + 1, tid=None, dispatched=False, retried=False, fixing=False)
    msg = _describe(tasks, seq, skipped, notes)
    if not seq:
        _set(g, "done", msg)
    elif run:
        _set(g, "running", "Making it so. " + msg)
    else:
        _set(g, "ready", msg + " Press Make it so.")


# ---------------------------------------------------------------------- execution
@contextmanager
def _exact(tasks, name, xy):
    """pick_place for an exact object and an exact table spot, without the language resolver (works for any object
    in tasks.objs, including ones added later)."""
    dest = np.array([float(xy[0]), float(xy[1]), TABLE_TOP])
    tasks.obj_name = lambda _t: name
    tasks.resolve = lambda *a, **k: (dest.copy(), "its goal spot")
    try:
        yield
    finally:
        for attr in ("obj_name", "resolve"):
            tasks.__dict__.pop(attr, None)


def _over_route(tasks, name, to):
    """Raised carry for when the straight line passes a fixed obstacle -> hand via points, or None if not needed."""
    o = tasks.objs[name]
    a, b = _cur(tasks, name), np.asarray(to, float)
    hit = [(n, ob) for n, ob in tasks.obstacles.items()
           if _seg_dist(np.asarray(ob["pos"], float)[:2], a, b) < ob["r"] + o["r"] + 0.02]
    if not hit:
        return None
    top = max(float(ob["pos"][2]) + float(ob["h"]) for _, ob in hit)
    hz = top + 0.035 + o["half_h"] + 0.01
    off = np.array([-(o["r"] + 0.015), 0.0])
    return [[*(a + off), hz], [*((a + b) / 2 + off), hz + 0.02], [*(b + off), hz]]


def _dispatch(ctx, g, step, via=None):
    tasks = ctx.tasks
    tid = f"goal-{g['rev']}-{step['i']}" + ("-over" if via else "") + ("-fix" if g["fixing"] else "")
    g["tid"], g["dispatched"] = tid, True
    step["status"] = "running"
    what = f"moving {step['label']} out of the way" if step["kind"] == "aside" else f"putting {step['label']} in its spot"
    g["message"] = (f"Move {step['i'] + 1} of {len(g['sequence'])}: {what}" + (" (over the obstacle)" if via else "")
                    + (" again, to land it exactly" if g["fixing"] else "") + ".")
    with _exact(tasks, step["object"], step["to"]):
        tasks.pick_place(tid, step["object"], "goal", None, None, ctx.aspect, via_pts=via)


def _busy(ctx):
    t = ctx.tasks
    return t.kind is not None or ctx.rt.mode in ("FOLLOW", "PREVIEW", "REC")


@ext.tick
def goal_tick(ctx):
    g = _g(ctx)
    if g["pending_photo"] is not None:                 # photo read in the background thread: plan it here (worker thread)
        res, g["pending_photo"] = g["pending_photo"], None
        g["photo"] = {k: v for k, v in res.items() if k != "targets"}
        if res.get("targets"):
            _set_goal(ctx, res["targets"], "photo:" + res["source"], g.pop("run_after_photo", False))
            g["message"] = res["summary"] + " " + g["message"]
        else:
            _set(g, "failed", res["summary"])
    if g["status"] != "running":
        return
    tasks, seq = ctx.tasks, g["sequence"]
    if g["i"] >= len(seq):
        errs = [s["err_cm"] for s in seq if s["kind"] == "place" and s["err_cm"] is not None]
        _set(g, "done", f"Done: it looks like your goal ({len(seq)} move{'s' if len(seq) != 1 else ''}"
             + (f", every object within {max(errs):.1f} cm" if errs else "") + ").")
        return
    step = seq[g["i"]]
    if not g["dispatched"]:
        if _busy(ctx):
            return
        via = _over_route(tasks, step["object"], step["to"])
        _dispatch(ctx, g, step, via)
        return
    t = tasks.task or {}
    if t.get("id") != g["tid"]:
        step["status"] = "failed"
        _set(g, "stopped", f"Stopped at move {g['i'] + 1} of {len(seq)}: another command took over.")
        return
    if t.get("status") in ("planning", "running"):
        return
    if t.get("status") == "failed":
        msg = t.get("message", "")
        if "would hit" in msg and not g["retried"]:   # carry clipped an obstacle: once more, over it
            g["retried"] = True
            via = _over_route(tasks, step["object"], step["to"]) or [
                [*(np.asarray(step["to"]) - [tasks.objs[step["object"]]["r"] + 0.015, 0]), 1.12]]
            _dispatch(ctx, g, step, via)
            return
        step["status"] = "failed"
        _set(g, "failed", f"Move {g['i'] + 1} of {len(seq)} ({step['label']}) failed: {msg}")
        return
    if t.get("status") == "done":
        err = float(np.linalg.norm(_cur(tasks, step["object"]) - np.asarray(step["to"])))
        step["err_cm"] = round(err * 100, 1)
        if step["kind"] == "place" and err > TOL and not g["fixing"]:
            # placed off the mark (e.g. a long reach across the table): pick it up once more from where it landed,
            # now usually with the nearer hand, and set it down exactly
            g["fixing"], g["dispatched"], g["retried"], g["tid"] = True, False, False, None
            g["message"] = f"{step['label'].capitalize()} landed {err * 100:.0f} cm off; correcting it."
            return
        if step["kind"] == "place" and err > TOL:
            step["status"] = "failed"
            _set(g, "failed", f"{step['label'].capitalize()} ended {err * 100:.0f} cm from its spot (more than {TOL * 100:.0f} cm).")
            return
        step["status"] = "done"
        g["i"] += 1
        g["dispatched"], g["retried"], g["fixing"], g["tid"] = False, False, False, None


# ---------------------------------------------------------------------- photo understanding
def _colour_profile(tasks, name):
    """Colour the offline reader looks for: from the name (red block, white mug...) or else the model's visual colour."""
    words = name.lower().replace("_", " ").split()
    named = {"red": ("hue", 0), "orange": ("hue", 18), "yellow": ("hue", 30), "green": ("hue", 60), "blue": ("hue", 110),
             "purple": ("hue", 140), "pink": ("hue", 165), "white": ("light", None), "grey": ("light", None),
             "gray": ("light", None), "black": ("dark", None)}
    for w in words:
        if w in named:
            return named[w]
    m = tasks.rt.model
    try:
        bid = m.body(name).id
    except KeyError:
        return ("light", None)
    for gid in range(m.ngeom):
        if m.geom_bodyid[gid] != bid or m.geom_group[gid] != 2:
            continue
        rgba = m.mat_rgba[m.geom_matid[gid]] if m.geom_matid[gid] >= 0 else m.geom_rgba[gid]
        hsv = cv2.cvtColor(np.uint8([[np.clip(np.asarray(rgba[:3]) * 255, 0, 255)[::-1]]]), cv2.COLOR_BGR2HSV)[0, 0]
        if hsv[1] < 60:
            return ("light", None) if hsv[2] > 90 else ("dark", None)
        return ("hue", int(hsv[0]))
    return ("light", None)


def _blob_mask(hsv, prof):
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1], hsv[..., 2]
    if prof[0] == "light":
        return ((s < 50) & (v > 150)).astype(np.uint8)
    if prof[0] == "dark":
        return ((v < 55)).astype(np.uint8)
    dh = np.abs(h - prof[1])
    dh = np.minimum(dh, 180 - dh)
    return ((dh <= 9) & (s > 90) & (v > 60)).astype(np.uint8)


def locate_blobs(img, profiles):
    """Offline colour-blob reader. profiles {name: profile} -> ({name: (u, v)}, notes). u, v in 0..1 of the image."""
    h, w = img.shape[:2]
    scale = 480.0 / max(h, w)
    if scale < 1:
        img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    found, used = {}, []
    for name, prof in profiles.items():
        mask = cv2.morphologyEx(_blob_mask(hsv, prof), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
        best = None
        for k in range(1, n):
            x, y, bw, bh, area = stats[k]
            if area < 0.0015 * h * w or area > 0.25 * h * w:          # specks, or the background/table itself
                continue
            c = cents[k]
            if any(np.hypot(c[0] - uc, c[1] - vc) < 6 for uc, vc in used):
                continue
            if best is None or area > best[0]:
                best = (area, c)
        if best is not None:
            used.append(tuple(best[1]))
            found[name] = (float(best[1][0] / w), float(best[1][1] / h))
    return found


def locate_gemini(jpg, names, key):
    """Gemini Robotics-ER on the photo -> ({name: (u, v)}, table box (u0, v0, u1, v1) | None, model). Raises on failure."""
    listing = ", ".join(f'"{n}" ({n.replace("_", " ")})' for n in names)
    prompt = ("This photo shows the layout a person wants on a table. Find the table top and each of these objects: "
              f"{listing}. Answer only JSON: "
              '{"table": [ymin, xmin, ymax, xmax] or null, "objects": [{"label": <one of the quoted names>, "point": [y, x]}]}. '
              "Coordinates are normalized to 0-1000. Leave out objects you cannot see.")
    body = {"contents": [{"parts": [{"inline_data": {"mime_type": "image/jpeg", "data": base64.b64encode(jpg).decode()}},
                                    {"text": prompt}]}],
            "generationConfig": {"temperature": 0.2}}
    err = None
    for model, tmo in ((ER_MODEL, ER_TIMEOUT), ("gemini-3.6-flash", FLASH_TIMEOUT)):
        req = urllib.request.Request(API + model + ":generateContent", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json", "x-goog-api-key": key, "User-Agent": "curl/8.7.1"})
        try:
            import certifi
            with urllib.request.urlopen(req, timeout=tmo, context=ssl.create_default_context(cafile=certifi.where())) as r:
                data = json.loads(r.read())
        except Exception as e:      # 503 busy, 429, timeout, offline: try the next model, then the offline reader
            err = e
            continue
        text = "".join(p.get("text", "") for p in data["candidates"][0]["content"]["parts"])
        m = re.search(r"\{.*\}", text, re.S)
        j = json.loads(m.group(0)) if m else {}
        found = {}
        for it in j.get("objects") or []:
            lab = str(it.get("label", "")).strip().lower().replace(" ", "_")
            name = lab if lab in names else next((n for n in names if n in lab or lab in n), None)
            pt = it.get("point")
            if name and name not in found and isinstance(pt, (list, tuple)) and len(pt) == 2:
                found[name] = (float(pt[1]) / 1000.0, float(pt[0]) / 1000.0)
        tb = j.get("table")
        box = None
        if isinstance(tb, (list, tuple)) and len(tb) == 4:
            y0, x0, y1, x1 = [float(v) / 1000.0 for v in tb]
            if x1 - x0 > 0.15 and y1 - y0 > 0.15:
                box = (x0, y0, x1, y1)
        return found, box, model
    raise err or RuntimeError("no answer")


def uv_to_table(u, v, view="user", box=None):
    """Photo position -> table point. view "user": taken from your side, facing the robot (the robot is at the top of
    the photo); "robot": taken from the robot's side (what the robot sees). Positions are relative to the table box when
    one was found, else to the whole photo, and land on the reachable front strip of the table."""
    if box is not None:
        u = (u - box[0]) / max(1e-6, box[2] - box[0])
        v = (v - box[1]) / max(1e-6, box[3] - box[1])
    u, v = float(np.clip(u, 0, 1)), float(np.clip(v, 0, 1))
    if view == "robot":
        return [WORK_X[1] - v * (WORK_X[1] - WORK_X[0]), WORK_Y[1] - u * (WORK_Y[1] - WORK_Y[0])]
    return [WORK_X[0] + v * (WORK_X[1] - WORK_X[0]), WORK_Y[0] + u * (WORK_Y[1] - WORK_Y[0])]


def read_photo(jpg, names, profiles, view, offline, key):
    """-> {source, targets, found:[{name, u, v}], summary, ms, error?}. Runs off the worker thread."""
    t0 = time.time()
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return {"source": "none", "targets": {}, "found": [], "summary": "I couldn't open that photo.", "ms": 0}
    found, box, source, why = {}, None, "offline", None
    if not offline and key:
        try:
            found, box, model = locate_gemini(jpg, names, key)
            source = "gemini"
            if not found:
                why = "Gemini didn't see any object I can move"
        except Exception as e:
            why = f"Gemini didn't answer ({str(e)[:60]})"
    elif not offline:
        why = "no Gemini key on the server"
    if not found:
        found, box, source = locate_blobs(img, profiles), None, "offline"
    targets = {n: uv_to_table(u, v, view, box) for n, (u, v) in found.items()}
    pretty = lambda n: n.replace("_", " ")
    if targets:
        summary = (f"From your photo ({'Gemini Robotics-ER' if source == 'gemini' else 'offline colour reader'}"
                   f"{'; ' + why if why and source == 'offline' else ''}): I see {', '.join(pretty(n) for n in targets)}.")
        missing = [n for n in names if n not in targets]
        if missing:
            summary += f" I don't see {', '.join(pretty(n) for n in missing)}; it stays where it is."
    else:
        summary = "I couldn't find any object I can move in that photo" + (f" ({why})" if why else "") + "."
    return {"source": source, "targets": targets, "view": view, "summary": summary, "ms": int((time.time() - t0) * 1000),
            "found": [{"name": n, "u": round(u, 3), "v": round(v, 3)} for n, (u, v) in found.items()],
            "box": None if box is None else [round(b, 3) for b in box], "error": why}


# ---------------------------------------------------------------------- messages + state
@ext.handler("goal")
def on_goal(ctx, msg):
    g, op = _g(ctx), msg.get("op")
    if op == "set":
        _set_goal(ctx, msg.get("targets") or {}, msg.get("source") or "arrange", bool(msg.get("run")))
    elif op == "run":
        if g["status"] == "running":
            return
        if g["targets"]:
            _set_goal(ctx, g["targets"], g["source"], True)    # re-plan from where things are now
        else:
            _set(g, "failed", "There is no goal yet. Arrange the ghosts or send a photo first.")
    elif op in ("cancel", "clear"):
        if g["status"] == "running":
            if g["dispatched"] and (ctx.tasks.task or {}).get("id") == g["tid"] and ctx.tasks.task.get("status") in ("planning", "running"):
                ctx.tasks.stop(f"goal-cancel-{time.time():.0f}")
            _set(g, "stopped", "Cancelled. Objects stay where they are now.")
        if op == "clear":
            g.update(targets={}, sequence=[], i=0, source=None, photo=None, rev=g["rev"] + 1)
            _set(g, "idle", "")
    elif op == "photo":
        if g["status"] in ("running", "locating"):
            g["message"] = "I'm busy with the last goal. Press Cancel first." if g["status"] == "running" else g["message"]
            return
        raw = str(msg.get("jpg_b64") or "")
        raw = raw.split(",", 1)[1] if raw.startswith("data:") else raw
        try:
            jpg = base64.b64decode(raw, validate=False)
        except Exception:
            jpg = b""
        if not jpg or len(jpg) > MAX_JPG:
            _set(g, "failed", "That photo is empty or too large (keep it under 1.5 MB).")
            return
        tasks = ctx.tasks
        names = list(tasks.objs)
        profiles = {n: _colour_profile(tasks, n) for n in names}
        view = "robot" if msg.get("view") == "robot" else "user"
        offline = bool(msg.get("offline"))
        key = None if offline else gemini_key()
        g["run_after_photo"] = bool(msg.get("run"))
        _set(g, "locating", "Reading your photo" + (" with Gemini Robotics-ER…" if key else " (offline colour reader)…"))

        def work():
            try:
                res = read_photo(jpg, names, profiles, view, offline, key)
            except Exception as e:
                res = {"source": "none", "targets": {}, "found": [], "summary": f"I couldn't read that photo ({str(e)[:60]}).", "ms": 0}
            g["pending_photo"] = res

        threading.Thread(target=work, daemon=True).start()


def _shape(ctx, name):
    cache = ctx.store.setdefault("goal_shapes", {})
    if name not in cache:
        m = ctx.rt.model
        kind = "cyl"
        try:
            bid = m.body(name).id
            for gid in range(m.ngeom):
                if m.geom_bodyid[gid] == bid and m.geom_group[gid] == 3:
                    kind = "box" if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX else "cyl"
                    break
        except KeyError:
            pass
        cache[name] = kind
    return cache[name]


@ext.state
def goal_state(ctx):
    g, tasks = _g(ctx), ctx.tasks
    objects = [{"name": n, "label": n.replace("_", " "), "pos": [round(float(x), 3) for x in tasks.mug_center(n)],
                "r": round(float(o["r"]), 3), "h": round(2 * float(o["half_h"]), 3), "shape": _shape(ctx, n)}
               for n, o in tasks.objs.items()]
    obstacles = [{"name": n, "pos": [round(float(x), 3) for x in o["pos"][:2]], "r": round(float(o["r"]), 3)}
                 for n, o in tasks.obstacles.items()]
    return {"goal": {"status": g["status"], "message": g["message"], "targets": g["targets"], "source": g["source"],
                     "sequence": g["sequence"], "i": g["i"], "rev": g["rev"], "photo": g["photo"],
                     "objects": objects, "obstacles": obstacles, "tolerance_cm": TOL * 100,
                     "work_area": [list(WORK_X), list(WORK_Y)]}}
