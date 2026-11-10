"""No planning stalls (docs/V1.md P0b "Refusals don't stall the loop", pps/slicing.py).

A plan card's dry run and a goal's set-aside spots plan the way the real run does (a dozen IK traces for a refusal; a
reach test per candidate spot), which used to happen inside one server frame and froze the stream for seconds on a
busy machine. They now run as jobs that pause between IK waypoints. The server reports its frame times in state.perf
(`{type: "perf", op: "reset"}` restarts the maxima); this check plans
  * a plan card for a pick the robot can do and for one it has to refuse,
  * a goal that needs one object set aside first (physics: the set-aside spots are reach-tested),
and checks the longest frame while they plan stays under 0.5 s, and that the plans are the same as before (proposed /
refused, the swap ordered with the set-aside first).
"""
import time

LIMIT_MS = 500.0


async def _frames(t, sec):
    await t.send(type="perf", op="reset")
    await t.pump(sec)
    return dict(t.state().get("perf") or {})


async def run(t):
    st = t.state
    await t.task("reset_scene", 1)
    await t.pump(1.0)
    phys = (st().get("physics") or {}).get("mode") == "dynamic"
    idle = await _frames(t, 2.0)
    t.check("stalls: the server reports its frame times", idle.get("frames", 0) > 0 and "max_ms" in idle,
            f"idle: {idle.get('frames')} frames, longest {idle.get('max_ms')} ms, planning budget {idle.get('budget_ms')} ms")

    # 1. plan cards (dry runs): one it can do, one it has to refuse (tries every way in)
    for target, want in (("center", ("proposed",)), ("back", ("proposed", "failed"))):   # (back: refused with physics)
        await t.task("reset_scene", 1)
        await t.pump(0.5)
        await t.send(type="perf", op="reset")
        t0, seen = time.time(), False
        tid = f"stall-{target}-{t0:.3f}"
        await t.send(type="task", id=tid, action="pick_place", object="mug", target=target, preview=True)
        while time.time() - t0 < 90:
            await t.pump(0.2)
            tk = st().get("task") or {}
            if tk.get("id") == tid:
                seen = seen or tk.get("status") == "planning"
                if tk.get("status") not in ("planning", None):
                    break
        tk, pf = st().get("task") or {}, dict(st().get("perf") or {})
        t.check(f"stalls: plan card dry run (mug to the {target}) keeps every frame under 0.5 s",
                tk.get("id") == tid and tk.get("status") in want and 0 < float(pf.get("max_ms") or 1e9) < LIMIT_MS,
                f"{tk.get('status')} after {time.time() - t0:.1f} s ({'planned across frames' if seen else 'one frame'}): longest "
                f"frame {pf.get('max_ms')} ms (planning {pf.get('plan_max_ms')} ms of a frame at most, idle {idle.get('max_ms')} ms) "
                f"{str(tk.get('message'))[:60]}")
        await t.send(type="plan", op="cancel", id=tid)
        await t.pump(0.3)

    # 2. a goal that needs a set-aside spot (the swap of checks/goal.py), planned only
    await t.task("reset_scene", 1)
    if phys:                                   # (where checks/goal.py starts the red block in physics)
        await t.send(type="physics", id=f"stall-place-{time.time():.3f}", op="place", object="red block", pos=[0.30, -0.42])
    await t.pump(0.8)
    g = st().get("goal") or {}
    objs = {o["name"]: o["pos"][:2] for o in g.get("objects") or []}
    if not {"mug", "red_block"} <= set(objs):
        t.check("stalls: goal set-aside planning keeps every frame under 0.5 s", False, f"objects: {sorted(objs)}")
        return
    await t.send(type="perf", op="reset")
    await t.send(type="goal", op="set", targets={"mug": objs["red_block"], "red_block": objs["mug"]}, source="arrange")
    t0, seen = time.time(), False
    while time.time() - t0 < 90:
        await t.pump(0.2)
        g = st().get("goal") or {}
        seen = seen or g.get("status") == "planning"
        if g.get("status") not in ("planning", "idle"):
            break
    pf = dict(st().get("perf") or {})
    seq = [(s["kind"], s["object"]) for s in g.get("sequence") or []]
    t.check("stalls: goal set-aside planning keeps every frame under 0.5 s",
            g.get("status") == "ready" and seq and seq[0][0] == "aside" and 0 < float(pf.get("max_ms") or 1e9) < LIMIT_MS,
            f"{g.get('status')} after {time.time() - t0:.1f} s ({'planned across frames' if seen else 'one frame'}) {seq}: longest "
            f"frame {pf.get('max_ms')} ms (planning {pf.get('plan_max_ms')} ms of a frame at most, idle {idle.get('max_ms')} ms)")
    await t.send(type="goal", op="clear")
    await t.pump(0.3)
