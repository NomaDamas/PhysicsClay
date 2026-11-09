"""Goal (#E) checks: an arranged goal becomes the right ordered sequence and ends with every object within 3 cm of its
target; a photo of the layout is read by the offline colour-blob fallback and mapped onto the table."""
import base64
import time

import cv2
import numpy as np


async def _wait(t, cond, limit):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.3)
        if cond(t.state().get("goal") or {}):
            break
    return t.state().get("goal") or {}


def _objs(g):
    return {o["name"]: o for o in g.get("objects") or []}


async def run(t):
    await t.task("reset_scene", 1)
    await t.pump(0.5)
    g = t.state().get("goal") or {}
    objs = _objs(g)
    t.check("goal: state lists the movable objects", {"mug", "red_block"} <= set(objs), str(sorted(objs)))
    if not {"mug", "red_block"} <= set(objs):
        return
    mug, red = objs["mug"]["pos"][:2], objs["red_block"]["pos"][:2]

    # 1. plan only: an object already within 3 cm is skipped
    await t.send(type="goal", op="set", targets={"mug": [mug[0] + 0.01, mug[1]], "red_block": [0.30, -0.05]})
    g = await _wait(t, lambda g: g.get("status") in ("ready", "failed", "done"), 5)
    seq = [(s["kind"], s["object"]) for s in g.get("sequence") or []]
    t.check("goal: skips objects already in place", g.get("status") == "ready" and [o for _, o in seq] == ["red_block"]
            and "already in place" in g.get("message", ""), f"{seq} {g.get('message', '')[:90]}")
    rb = (g.get("targets") or {}).get("red_block") or [0, 0]
    t.check("goal: target kept clear of the vase", np.hypot(rb[0] - 0.33, rb[1] - 0.04) > 0.045 + 0.026 + 0.01, str(rb))

    # 2. arranged swap: targets occupy each other -> set one aside, then place both, each within 3 cm
    await t.send(type="goal", op="set", targets={"mug": red, "red_block": mug}, run=True, source="arrange")
    g = await _wait(t, lambda g: g.get("status") != "ready" and g.get("sequence"), 5)
    seq = [(s["kind"], s["object"]) for s in g.get("sequence") or []]
    t.check("goal: swap is ordered (set one aside first)", seq == [("aside", "red_block"), ("place", "mug"), ("place", "red_block")], str(seq))
    g = await _wait(t, lambda g: g.get("status") in ("done", "failed", "stopped"), 150)
    objs = _objs(g)
    errs = {n: round(float(np.hypot(objs[n]["pos"][0] - p[0], objs[n]["pos"][1] - p[1])) * 100, 1)
            for n, p in (g.get("targets") or {}).items() if n in objs}
    t.check("goal: arranged goal runs to the end", g.get("status") == "done", g.get("message", "")[:110])
    t.check("goal: every object within 3 cm of its target", errs and all(e <= 3.0 for e in errs.values()), str(errs))
    await t.task("reset_scene", 1)

    # 3. photo: synthetic top view (taken from your side, robot at the top) through the offline colour reader
    img = np.full((480, 640, 3), (60, 105, 160), np.uint8)               # wooden table (BGR)
    cv2.rectangle(img, (110, 280), (150, 320), (70, 80, 225), -1)        # red block at u=0.2, v=0.625
    cv2.circle(img, (544, 120), 30, (235, 235, 235), -1)                 # white mug at u=0.85, v=0.25
    jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
    await t.send(type="goal", op="photo", jpg_b64=base64.b64encode(jpg).decode(), view="user", offline=True)
    g = await _wait(t, lambda g: g.get("status") not in ("locating", "idle", "done") and (g.get("photo") or {}).get("source"), 15)
    ph = g.get("photo") or {}
    tg = g.get("targets") or {}
    t.check("goal: photo read offline (colour blobs)", ph.get("source") == "offline" and {"mug", "red_block"} <= set(tg),
            f"{g.get('status')} {ph.get('summary', '')[:80]}")
    exp = {"red_block": (0.25 + 0.625 * 0.22, -0.36 + 0.2 * 0.72), "mug": (0.25 + 0.25 * 0.22, -0.36 + 0.85 * 0.72)}
    off = {n: round(float(np.hypot(tg[n][0] - x, tg[n][1] - y)) * 100, 1) for n, (x, y) in exp.items() if n in tg}
    t.check("goal: photo positions map onto the table", off and all(v <= 3.0 for v in off.values()), str(off))
    t.check("goal: photo goal becomes a plan", g.get("status") == "ready" and len(g.get("sequence") or []) >= 1,
            str([(s["kind"], s["object"]) for s in g.get("sequence") or []]))
    await t.send(type="goal", op="clear")
    await t.pump(0.3)
