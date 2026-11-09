"""Candidates (#C): an ambiguous request asks "which one?", every way of answering runs the right object, and an
unambiguous request does not ask."""
import time


def _choices(t):
    return t.state().get("choices") or {}


def _key(t, name):
    return next((o["key"] for o in _choices(t).get("options", []) if o["name"] == name), None)


async def _wait(t, fn, limit=6.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.2)
        if fn(t.state()):
            return True
    return False


async def run(t):
    await t.task("reset_scene", 1)
    names = sorted(o["name"] for o in t.state().get("scene_objects", []))
    t.check("candidates: green block in the scene", "green block" in names, str(names))

    # 1. "the block" with a red and a green block -> asks, numbered left to right, nothing moves
    tk = await t.task("pick_place", 2, object="block", target="center")
    ch = _choices(t)
    opts = ch.get("options", [])
    t.check("candidates: ambiguous 'block' asks", tk.get("status") == "asking" and ch.get("id") == tk.get("id"),
            f"{tk.get('status')}: {tk.get('message', '')[:70]}")
    t.check("candidates: numbered red + green options with 3D pos and robot-view uv",
            sorted(o["name"] for o in opts) == ["green_block", "red_block"] and [o["key"] for o in opts] == ["1", "2"]
            and all(len(o.get("pos") or []) == 3 and o.get("uv") for o in opts), str([(o["key"], o["name"], o.get("uv")) for o in opts]))

    # 2. answer in words -> the original intent resumes with that object
    green0 = next((o["pos"] for o in t.state().get("scene_objects", []) if o["name"] == "green block"), None)
    await t.send(type="choose", id=ch.get("id"), key="the green one")
    ok = await _wait(t, lambda s: (s.get("task") or {}).get("id") == tk.get("id") and s["task"]["status"] in ("running", "done", "failed"))
    run1 = t.state().get("task") or {}
    t.check("candidates: 'the green one' runs the green block", ok and run1.get("status") in ("running", "done") and "green block" in run1.get("message", "")
            and not t.state().get("choices"), run1.get("message", "")[:80])
    done = await t.until_done(30)
    green = next((o["pos"] for o in t.state().get("scene_objects", []) if o["name"] == "green block"), None)
    moved = green is not None and green0 is not None and max(abs(a - b) for a, b in zip(green, green0)) > 0.05
    t.check("candidates: green block placed", done.get("status") == "done" and "green block" in done.get("message", "") and moved,
            f"{done.get('message', '')[:60]} {green}")
    await t.task("reset_scene", 1)

    # 3. unambiguous -> no question
    tk = await t.task("pick_place", 2, object="red block", target="center")
    t.check("candidates: 'red block' does not ask", tk.get("status") in ("running", "done") and not t.state().get("choices"),
            f"{tk.get('status')}: {tk.get('message', '')[:60]}")
    await t.task("stop", 1); await t.task("reset_scene", 1)
    tk = await t.task("pick_place", 2, object="mug", target="center")
    t.check("candidates: 'mug' does not ask", tk.get("status") in ("running", "done") and not t.state().get("choices"), tk.get("status", ""))
    await t.task("stop", 1); await t.task("reset_scene", 1)

    # 4. answer by number (chip / badge click sends the key)
    tk = await t.task("pick_place", 2, object="cube", target="center")
    k = _key(t, "red_block")
    await t.send(type="choose", id=tk.get("id"), key=k)
    ok = await _wait(t, lambda s: (s.get("task") or {}).get("status") in ("running", "done", "failed") and (s["task"] or {}).get("id") == tk.get("id"))
    t.check("candidates: choose by number", ok and "red block" in (t.state().get("task") or {}).get("message", ""),
            f"key {k}: {(t.state().get('task') or {}).get('message', '')[:60]}")
    await t.task("stop", 1); await t.task("reset_scene", 1)

    # 5. a pointing ray landing between two objects ("put that in the center") asks; pointing at one answers
    pos = {o["name"]: o["pos"] for o in t.state().get("scene_objects", [])}
    mid = [(a + b) / 2 for a, b in zip(pos["mug"], pos["red block"])]
    tk = await t.task("pick_place", 2, object="that", object_pos=mid, target="center")
    t.check("candidates: pointing between two objects asks", tk.get("status") == "asking" and "pointing" in tk.get("message", ""),
            tk.get("message", "")[:70])
    await t.send(type="choose", id=tk.get("id"), pos=pos["red block"])
    ok = await _wait(t, lambda s: (s.get("task") or {}).get("status") in ("running", "done", "failed"))
    t.check("candidates: choose by pointing at it", ok and "red block" in (t.state().get("task") or {}).get("message", ""),
            (t.state().get("task") or {}).get("message", "")[:60])
    await t.task("stop", 1); await t.task("reset_scene", 1)

    # 6. tapping the object on the robot view while asked answers too, and the tap leaves no stray marker
    tk = await t.task("pick_place", 2, object="block", target="center")
    uv = next((o.get("uv") for o in _choices(t).get("options", []) if o["name"] == "green_block"), None)
    if uv:
        await t.send(type="tap", u=uv[0], v=uv[1] + 0.01)
        ok = await _wait(t, lambda s: (s.get("task") or {}).get("status") in ("running", "done", "failed"))
        msg = (t.state().get("task") or {}).get("message", "")
        t.check("candidates: tap on a candidate answers", ok and "green block" in msg and not t.state().get("markers"),
                f"{msg[:60]} markers={[m['id'] for m in t.state().get('markers', [])]}")
    else:
        t.check("candidates: tap on a candidate answers", False, "no uv for the green block")
    await t.task("stop", 1); await t.task("reset_scene", 1)

    # 7. "the thing" -> all movable objects; cancel leaves everything alone; a new task supersedes the question
    tk = await t.task("pick_place", 2, object="thing", target="center")
    t.check("candidates: 'thing' lists every movable object", len(_choices(t).get("options", [])) == 3, tk.get("message", "")[:80])
    await t.send(type="choose", id=tk.get("id"), key="cancel")
    ok = await _wait(t, lambda s: not s.get("choices") and (s.get("task") or {}).get("status") == "failed")
    t.check("candidates: cancel", ok, (t.state().get("task") or {}).get("message", "")[:60])
    await t.task("pick_place", 2, object="block", target="center")
    await t.task("home", 1)
    ok = await _wait(t, lambda s: not s.get("choices"), 3)
    t.check("candidates: a new task clears the question", ok)
