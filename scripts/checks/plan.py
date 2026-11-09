"""Plan preview (#B): preview never moves the robot, confirm runs it, cancel clears it, a blocked plan explains why."""
import time


def _mug(st):
    return next((o["pos"] for o in st.get("scene_objects", []) if o["name"] == "mug"), [0, 0, 0])


def _still(st, mug0):
    moved = max(abs(a - b) for a, b in zip(_mug(st), mug0))
    return st.get("mode") == "IDLE" and moved < 0.005 and not any((st.get("holding") or {}).values()), f"mode={st.get('mode')} mug moved {moved * 100:.1f} cm"


async def _plan_for(t, tid, limit=8):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.3)
        p = t.state().get("plan")
        if p and p.get("id") == tid:
            return p
    return t.state().get("plan") or {}


async def run(t):
    await t.task("reset_scene", 1)
    await t.pump(0.6)
    t.check("plan: idle state is null", "plan" in t.state() and t.state()["plan"] is None, str(t.state().get("plan"))[:60])
    mug0 = _mug(t.state())

    # 1. propose (reachable) -> nothing moves
    task = await t.task("pick_place", 4, object="mug", target="center", preview=True)
    tid = task.get("id")
    p = await _plan_for(t, tid)
    t.check("plan: preview proposes instead of running", task.get("status") == "proposed" and p.get("phase") == "proposing" and p.get("ok") is True,
            f"{task.get('status')} / {p.get('phase')}: {p.get('verdict')}")
    t.check("plan: intent, target, steps, constraints, waypoints", bool(p.get("intent")) and (p.get("target") or {}).get("pos")
            and len(p.get("steps") or []) >= 4 and p.get("constraints") and len(p.get("waypoints") or []) > 5
            and len(p["waypoints"]) == len(p.get("wp_ok") or []) and (p.get("carry") or {}).get("object") == "mug",
            f"{p.get('intent')} · {len(p.get('steps') or [])} steps · {len(p.get('waypoints') or [])} pts")
    await t.pump(1.5)
    ok, why = _still(t.state(), mug0)
    t.check("plan: preview never moves the robot", ok and (t.state().get("plan") or {}).get("phase") == "proposing", why)

    # 2. confirm -> runs the same task id to the end
    await t.send(type="plan", op="confirm", id=tid)
    t0, running = time.time(), False
    while time.time() - t0 < 8:
        await t.pump(0.3)
        st = t.state()
        if (st.get("task") or {}).get("id") == tid and st["task"]["status"] in ("running", "done"):
            running = True
            break
    t.check("plan: confirm runs the task", running and (t.state().get("plan") or {}).get("phase") in ("running", "done"),
            f"{(t.state().get('task') or {}).get('status')} / {(t.state().get('plan') or {}).get('phase')}")
    fin = await t.until_done(30)
    await t.pump(0.6)
    p = t.state().get("plan") or {}
    t.check("plan: confirmed task finishes, card shows done", fin.get("id") == tid and fin.get("status") == "done" and p.get("phase") == "done",
            f"{fin.get('status')} / {p.get('phase')}: {fin.get('message', '')[:50]}")

    # 3. cancel -> plan cleared, nothing moved
    await t.task("reset_scene", 1)
    await t.pump(0.6)
    mug0 = _mug(t.state())
    task = await t.task("pick_place", 4, object="mug", target="center", preview=True)
    await _plan_for(t, task.get("id"))
    await t.send(type="plan", op="cancel")
    await t.pump(1.0)
    st = t.state()
    ok, why = _still(st, mug0)
    t.check("plan: cancel clears it", st.get("plan") is None and (st.get("task") or {}).get("status") == "failed"
            and "Cancelled" in (st.get("task") or {}).get("message", "") and ok, why)

    # 4. blocked: out of reach -> why + nearest reachable spot; confirm does nothing
    task = await t.task("pick_place", 6, object="mug", target="back", preview=True)
    p = await _plan_for(t, task.get("id"))
    alt = p.get("alternative") or {}
    t.check("plan: a blocked plan explains why", p.get("ok") is False and "can't" in (p.get("verdict") or "")
            and "reach" in " ".join(p.get("reasons") or []) and p.get("phase") == "needs_you" and (alt.get("task") or {}).get("target"),
            f"{p.get('verdict')} · alt {alt.get('label')}")
    t.check("plan: blocked plan shows red waypoints", False in (p.get("wp_ok") or []), f"{(p.get('wp_ok') or []).count(False)} red")
    await t.send(type="plan", op="confirm")
    await t.pump(1.2)
    ok, why = _still(t.state(), mug0)
    t.check("plan: confirming a blocked plan does nothing", ok and (t.state().get("task") or {}).get("status") == "failed", why)

    # 5. blocked by the vase
    task = await t.task("pick_place", 6, object="mug", target="left", preview=True)
    p = await _plan_for(t, task.get("id"))
    t.check("plan: carry through the vase is blocked", p.get("ok") is False and "too low" in (p.get("verdict") or "") and p.get("phase") == "blocked",
            str(p.get("verdict")))

    # 6. a new command supersedes a waiting proposal; tasks without preview are untouched
    task = await t.task("reach", 4, target="right", preview=True)
    p = await _plan_for(t, task.get("id"))
    t.check("plan: reach preview", p.get("phase") == "proposing" and p.get("action") == "reach", str(p.get("verdict")))
    task = await t.task("reach", 3, target="right")
    await t.pump(0.5)
    t.check("plan: task without preview runs directly, stale proposal dropped", task.get("status") in ("running", "done")
            and t.state().get("plan") is None, f"{task.get('status')} / plan {t.state().get('plan') and t.state()['plan'].get('phase')}")
    await t.until_done(15)
