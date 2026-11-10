"""P0 physics (docs/V1.md): the robot is a real robot. state.physics is published; a dropped object lands and rests; a
pick with no grip force fails as a slip; a very heavy object is refused or dropped honestly; a real pick holds the
object by contact (both pads); impedance mode keeps the end-effector force under its limit while pressing on the table;
`physics set / reset` change the plant at run time; and the kinematic fallback (PC_PHYSICS=0) still builds the old world.

Runs inside scripts/selftest.py (async def run(t)). The server must run with physics on (the default)."""
import os
import re
import time
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
PRESS_SCENE = {"template": "table", "objects": [{"name": "block", "kind": "block", "color": "red", "pos": [0.40, -0.3]}]}


def _phys(t):
    return t.state().get("physics") or {}


async def _until(t, cond, limit, sim=False):
    """Pump until cond(state); limit in wall seconds, or with sim=True in the robot's own (sim) seconds (a loaded machine
    runs the plant slower than real time), capped at 4x the limit of wall time."""
    t0, ran, last = time.time(), 0.0, None
    while time.time() - t0 < (4 * limit if sim else limit):
        await t.pump(0.2)
        if cond(t.state()):
            return True
        if sim:
            now = float((t.state().get("physics") or {}).get("time") or 0.0)
            ran += max(0.0, now - last) if last is not None and now >= last else 0.0
            last = now
            if ran >= limit:
                return False
    return False


async def _physics(t, **kw):
    tid = f"phys-{time.time():.3f}"
    await t.send(type="physics", id=tid, **kw)
    await _until(t, lambda s: (s.get("task") or {}).get("id") == tid, 3.0)
    return t.state().get("task") or {}


async def _world(t, **kw):
    rev = (t.state().get("world") or {}).get("rev", 0)
    await t.send(type="world", id=f"world-{rev}", **kw)
    await _until(t, lambda s: (s.get("world") or {}).get("rev", 0) > rev, 25.0)


async def run(t):
    await t.task("reset_scene", 1)
    await t.pump(0.5)
    p = _phys(t)
    keys = ("mode", "dt", "rtf", "grasp", "ee_force", "contacts", "params")
    if os.environ.get("PC_PHYSICS", "1") == "0":     # the whole suite run on the kinematic fallback
        r = await _physics(t, op="set", params={"mass_scale": 2})
        t.check("physics: kinematic fallback reports it and refuses plant changes", p.get("mode") == "kinematic"
                and all(k in p for k in keys) and r.get("status") == "failed", f"mode={p.get('mode')} {r.get('message', '')[:60]}")
        return
    t.check("physics: state.physics is published (dynamic, 2 ms, real time)",
            all(k in p for k in keys) and p.get("mode") == "dynamic" and abs((p.get("dt") or 0) - 0.002) < 1e-9 and (p.get("rtf") or 0) > 0.5,
            f"mode={p.get('mode')} dt={p.get('dt')} rtf={p.get('rtf')} missing={[k for k in keys if k not in p]}")
    if p.get("mode") != "dynamic":
        return

    # 1. an object dropped from 10 cm lands and rests
    z0 = ((p.get("objects") or {}).get("red_block") or {}).get("p", [0, 0, 0])[2]
    tid, zs = f"phys-drop-{time.time():.3f}", []
    await t.send(type="physics", id=tid, op="drop", object="red block", height=0.10)
    t1 = time.time()
    while time.time() - t1 < 2.5:                # every frame: the fall itself lasts ~0.14 s
        await t.pump(0.02)
        o = (_phys(t).get("objects") or {}).get("red_block") or {}
        if o and (t.state().get("task") or {}).get("id") == tid:
            zs.append((o["p"][2], o["v"]))
    r = t.state().get("task") or {}
    land = zs[-1] if zs else (0, 9)
    t.check("physics: an object dropped from 10 cm lands and rests",
            r.get("status") == "done" and zs and (max(z for z, _ in zs) > z0 + 0.02 or max(v for _, v in zs) > 0.3)
            and abs(land[0] - z0) < 0.01 and land[1] < 0.02,
            f"{r.get('message', '')[:30]} rest z {z0:.3f} -> peak {max([z for z, _ in zs] or [0]):.3f} (max speed "
            f"{max([v for _, v in zs] or [0]):.2f}) -> {land[0]:.3f} m, speed {land[1]:.3f} m/s")

    # 2. a real pick holds by contact: both pads on the mug while it is carried
    await t.task("reset_scene", 1)
    res = await t.task("pick_place", 6, object="mug", target="center")
    seen = {"grasp": False, "pads": False}

    def watch(s):
        ph = s.get("physics") or {}
        if "mug" in (ph.get("grasp") or {}).values():
            seen["grasp"] = True
        if any("pad" in c["a"] + c["b"] and "mug" in c["a"] + c["b"] for c in ph.get("contacts") or []):
            seen["pads"] = True
        return (s.get("task") or {}).get("status") not in ("running", "planning")
    await _until(t, watch, 40, sim=True)
    fin = t.state().get("task") or {}
    t.check("physics: a real pick holds the mug by contact (both pads) and places it", res.get("status") in ("running", "done")
            and fin.get("status") == "done" and seen["grasp"] and seen["pads"], f"{fin.get('status')}: {fin.get('message', '')[:70]} {seen}")

    # 3. no grip force -> the pick fails as a slip
    await t.task("reset_scene", 1)
    r = await _physics(t, op="set", params={"grip_force": 0})
    res = await t.task("pick_place", 6, object="red block", target="center")
    fin = await t.until_done(40) if res.get("status") == "running" else res
    t.check("physics: a pick with gripper force 0 fails as a slip", r.get("status") == "done" and _phys(t)["params"]["grip_force"] == 0
            and fin.get("status") == "failed" and "slip" in fin.get("message", "").lower(), fin.get("message", "")[:90])
    await _physics(t, op="reset")
    await t.task("reset_scene", 1)

    # 4. a heavy object (mass_scale 20) is refused or dropped honestly
    r = await _physics(t, op="set", params={"mass_scale": 20})
    res = await t.task("pick_place", 6, object="mug", target="center")
    fin = await t.until_done(40) if res.get("status") == "running" else res
    msg = fin.get("message", "").lower()
    t.check("physics: a heavy object (mass x20) is refused or dropped honestly",
            fin.get("status") == "failed" and any(k in msg for k in ("heavy", "slip")), fin.get("message", "")[:90])
    r = await _physics(t, op="reset")
    t.check("physics: 'reset' restores the default parameters", r.get("status") == "done" and _phys(t)["params"].get("mass_scale") == 1.0
            and _phys(t)["params"].get("grip_force") == 40.0, str(_phys(t).get("params"))[:120])
    await t.task("reset_scene", 1)

    # 4b. a pick nothing could reach is refused at once (no dozen ways in, no stalled loop): wall time, on purpose
    await t.send(type="physics", id=f"phys-far-{time.time():.3f}", op="place", object="red block", pos=[0.78, 0.45])
    await t.pump(0.5)
    t0 = time.time()
    res = await t.task("pick_place", 10, object="red block", target="center")
    took = time.time() - t0
    t.check("physics: an unreachable pick is refused in under 2 s (wall)", res.get("status") == "failed" and took < 2.0,
            f"{took:.2f} s {res.get('status')}: {res.get('message', '')[:70]}")
    await t.task("reset_scene", 1)

    # 5. impedance mode: pressing on the table stays under the force limit (an arm, an uncluttered table)
    await _world(t, op="set", robot="panda", scene=PRESS_SCENE)
    await _physics(t, op="set", params={"impedance": True, "force_limit": 15})
    res = await t.task("press", 4, target="center", force=30, hold_s=1.0)
    fz = []

    def press(s):
        f = ((s.get("physics") or {}).get("ee_force") or {}).get("right")
        if f:
            fz.append(f[2])
        return (s.get("task") or {}).get("status") not in ("running", "planning")
    await _until(t, press, 30, sim=True)
    fin = t.state().get("task") or {}
    peak = re.search(r"peak ([\d.]+) N", fin.get("message", ""))
    peak = float(peak.group(1)) if peak else max(fz or [0])
    t.check("physics: impedance mode keeps the end-effector force under its limit while pressing",
            fin.get("status") == "done" and max(fz or [0]) > 5 and peak <= 15 * 1.05 and "capped" in fin.get("message", ""),
            f"{fin.get('message', '')[:100]} (sampled max {max(fz or [0]):.1f} N)")
    await _physics(t, op="reset")
    res = await t.task("press", 4, target="center", force=10, hold_s=1.0)
    await _until(t, lambda s: (s.get("task") or {}).get("status") not in ("running", "planning"), 30, sim=True)
    fin = t.state().get("task") or {}
    mean = re.search(r"with ([\d.]+) N", fin.get("message", ""))
    t.check("physics: a pressing force is tracked along the approach axis (10 N asked)",
            fin.get("status") == "done" and mean and abs(float(mean.group(1)) - 10) < 2.5, fin.get("message", "")[:100])
    # 6. a mobile base still drives with physics (its base joints are servoed; the plant follows the planned base)
    await _world(t, op="set", robot="tidybot")
    res = await t.task("drive", 3, dy=0.2)
    fin = await t.until_done(15) if res.get("status") == "running" else res
    t.check("physics: a mobile base drives (TidyBot, servoed base joints)", fin.get("status") == "done" and "Drove" in fin.get("message", "")
            and (_phys(t).get("rtf") or 0) > 0.3, f"{fin.get('message', '')[:60]} rtf={_phys(t).get('rtf')}")
    await _world(t, op="reset")

    # 7. the kinematic fallback still builds the old world (PC_PHYSICS=0 / --kinematic)
    sys.path.insert(0, str(ROOT))
    from pps.world import scene
    w = scene.build(None, None, physics=False)
    mocap = all(w.model.body(n).mocapid[0] >= 0 for n in ("mug", "red_block", "green_block"))
    t.check("physics: kinematic fallback builds the UX 2.0 world (objects ride the hand)", not w.physics and w.ik_model is w.model and mocap,
            f"physics={w.physics} mocap objects={mocap}")
