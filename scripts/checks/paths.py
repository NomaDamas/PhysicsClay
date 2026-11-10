"""Paths respect the objects on the table (v1 P0c). With physics the hand is a real gripper, so a drawn path is checked
against the objects it passes: over a movable one it is lifted (when the arm reaches that high), into a fixed one (the
vase is never flown over, as with a carry) the waypoints go red with the reason, and following the lifted path leaves
the mug standing where it was. The kinematic fallback (PC_PHYSICS=0) keeps the old behaviour: the path is not changed.

Runs inside scripts/selftest.py (async def run(t))."""
import math
import os

# from the table's near right corner, down to hand height straight through the mug, and on past it (the start is away
# from the mug: on the P2 line the move to a stroke's start is traj.py's own straight approach)
THROUGH_MUG = [[0.23, -0.30, 0.95], [0.33, -0.18, 0.82], [0.44, -0.18, 0.95]]
BY_VASE = [[0.28, -0.12, 0.84], [0.30, -0.02, 0.84], [0.30, 0.10, 0.84]]        # hand height, 1.5 cm past the vase


def _obj(t, name):
    return ((t.state().get("physics") or {}).get("objects") or {}).get(name) or {}


def _tilt(q):
    """Degrees between the object's up axis and the world's (q = w, x, y, z)."""
    w, x, y, z = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


async def run(t):
    st = t.state
    await t.task("reset_scene", 1)
    await t.send(type="clear"); await t.pump(0.3)
    msgs = lambda: " ".join(st().get("messages") or [])
    dynamic = ((st().get("physics") or {}).get("mode")) == "dynamic"

    await t.send(type="path3d", points=THROUGH_MUG, hand="right"); await t.pump(1.5)
    p = st().get("path") or {}
    ok = p.get("ok") or []
    if not dynamic or os.environ.get("PC_PHYSICS", "1") == "0":
        t.check("paths: kinematic fallback leaves a drawn path as drawn (no object check)",
                ok and all(ok) and "lifted" not in msgs(), msgs()[:100])
        await t.send(type="clear"); await t.pump(0.3)
        return
    t.check("paths: a path through the mug is lifted over it (all reachable)", ok and all(ok) and "lifted over the mug" in msgs(),
            f"{sum(ok)}/{len(ok)} ok: {msgs()[:110]}")
    # following it, on an arm (the G1's wrist, which carries its rubber hand, turns along a stroke run by the P2 follower
    # and can brush the mug: docs/V1.md P0c limits)
    rev = (st().get("world") or {}).get("rev", 0)
    await t.send(type="world", id=f"paths-world-{rev}", op="set", robot="panda")
    for _ in range(100):
        await t.pump(0.25)
        if (st().get("world") or {}).get("rev", 0) > rev:
            break
    await t.send(type="path3d", points=THROUGH_MUG, hand="right"); await t.pump(1.5)
    lifted = "lifted over the mug" in msgs()
    m0 = _obj(t, "mug")
    r = await t.task("follow_path", 3)
    touched = set()
    for _ in range(240):                           # (sim time, up to ~60 s of wall on a loaded machine)
        await t.pump(0.25)
        for c in (st().get("physics") or {}).get("contacts") or []:
            if "mug" in c["a"] + c["b"] and "table" not in c["a"] + c["b"]:
                touched.add(c["b"] if "mug" in c["a"] else c["a"])
        if (st().get("task") or {}).get("status") not in ("running", "planning"):
            break
    fin = st().get("task") or {}
    await t.pump(0.6)
    m1 = _obj(t, "mug")
    moved = math.hypot(m1["p"][0] - m0["p"][0], m1["p"][1] - m0["p"][1]) * 100 if m0 and m1 else 99
    t.check("paths: following the lifted path leaves the mug standing where it was (Panda)",
            lifted and fin.get("status") == "done" and moved < 1.0 and _tilt(m1.get("q", [1, 0, 0, 0])) < 5,
            f"{fin.get('status')}: {fin.get('message', '')[:50]} mug moved {moved:.1f} cm, tilt {_tilt(m1.get('q', [1, 0, 0, 0])):.0f}°"
            + (f", touched by {sorted(touched)}" if touched else ""))
    rev = (st().get("world") or {}).get("rev", 0)
    await t.send(type="world", id=f"paths-world-{rev}", op="reset")
    for _ in range(100):
        await t.pump(0.25)
        if (st().get("world") or {}).get("rev", 0) > rev:
            break
    await t.task("home", 1.5)

    await t.send(type="path3d", points=BY_VASE, hand="right"); await t.pump(1.5)
    p = st().get("path") or {}
    ok = p.get("ok") or []
    t.check("paths: a path into the vase is red, with the reason", ok and not all(ok) and "would hit the vase" in msgs(),
            f"{sum(ok)}/{len(ok)} ok: {msgs()[:110]}")
    r = await t.task("follow_path", 2)
    t.check("paths: follow_path refuses it, saying what it would hit", r.get("status") == "failed" and "would hit the vase" in r.get("message", ""),
            r.get("message", "")[:100])
    await t.send(type="clear"); await t.pump(0.3)
    await t.task("reset_scene", 1)

    # P2 trajectories (pps/features/traj.py, where present): the gripper's own size counts, not only its TCP point. The
    # TCP passes 2 cm over the mug's top, so the hand-point check alone is happy; the fingers below it are not.
    if not isinstance(st().get("traj"), dict):
        return
    top = next((o["pos"][2] for o in st().get("scene_objects") or [] if o["name"] == "mug"), 0.811) + 0.071
    tr = {"role": "right", "frame": "world", "source": "keyframes", "label": "skim the mug",     # raw points (language
          "points": [{"p": [0.24, -0.18, top + 0.02]}, {"p": [0.42, -0.18, top + 0.02]}]}       # keypoints get lifted)
    await t.send(type="traj", id="p0c-skim", op="propose", traj=tr, task_id="p0c-skim-plan")
    tj = {}
    for _ in range(40):
        await t.pump(0.25)
        tj = st().get("traj") or {}
        if (tj.get("reply") or {}).get("id") == "p0c-skim":
            break
    ghost = tj.get("ghost") or {}
    t.check("paths: a trajectory whose gripper skims the mug is blocked, saying so (P2)",
            ghost.get("ok") and not all(ghost["ok"]) and "would hit the mug" in (tj.get("why") or ""), (tj.get("why") or str(tj.get("reply")))[:100])
    await t.send(type="traj", op="clear"); await t.pump(0.4)
    await t.task("reset_scene", 1)

    # language keypoints are lifted with the gripper's real extent (pps/traj.py hand_extent, lead_in): the default robot's
    # (the G1's) gripper hangs ~10 cm below its TCP as its hand turns up over the table, so the old fixed 5 cm lift over
    # the mug ran it through the mug and knocked it over; now the pass clears it and the mug stays standing
    spec = {"label": "over the mug", "keypoints": [{"xyz": [0.24, -0.19, 0.92]}, {"via": "over", "of": "mug"}, {"xyz": [0.50, -0.19, 0.92]}]}
    m0 = _obj(t, "mug")
    await t.send(type="traj", id="p2-over", op="set", spec=spec, run=True)
    tj, started = {}, False
    for _ in range(80):
        await t.pump(0.25)
        tj = st().get("traj") or {}
        started = started or bool(tj.get("running"))
        if (tj.get("reply") or {}).get("id") == "p2-over" and (started or (tj.get("reply") or {}).get("ok") is False):
            break
    ghost = tj.get("ghost") or {}
    mug = next((o["pos"] for o in st().get("scene_objects") or [] if o["name"] == "mug"), [0.33, -0.188, 0.811])
    over = [q[2] for q in ghost.get("pts") or [] if math.hypot(q[0] - mug[0], q[1] - mug[1]) < 0.065]
    low = (min(over) - (mug[2] + 0.071)) * 100 if over else -99.0
    fin = await t.until_done(45) if started else (st().get("task") or {})
    await t.pump(0.6)
    m1 = _obj(t, "mug")
    moved = math.hypot(m1["p"][0] - m0["p"][0], m1["p"][1] - m0["p"][1]) * 100 if m0 and m1 else 99
    tilt = _tilt(m1.get("q", [1, 0, 0, 0])) if m1 else 99
    t.check("paths: a keypoint pass over the mug lifts the gripper clear of it and leaves it standing (P2, gripper-aware lift)",
            fin.get("status") == "done" and low > 6.0 and moved < 1.0 and tilt < 5,
            f"{fin.get('status')}: {str(fin.get('message') or (tj.get('reply') or {}).get('message'))[:50]}; TCP {low:.1f} cm over the "
            f"mug's top; mug moved {moved:.1f} cm, tilt {tilt:.0f}°")
    await t.send(type="traj", op="clear"); await t.pump(0.4)
    await t.task("reset_scene", 1)
