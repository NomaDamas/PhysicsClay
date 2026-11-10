"""P2b live teleop (docs/vr.md, docs/PROTOCOL.md v8) over the real protocol, default world (G1 at the table):

1. state.teleop is there; {op:"start"} takes the hand;
2. a 30 Hz stream of {role, p, q, t} moves the end effector; rate, lag and the echoed client time are reported;
3. a target out of reach is clamped (and flagged), one below the table stays above it;
4. closing the gripper at the mug grasps it (kinematic grasp without P0), lifting carries it, opening puts it down;
5. stop lets go; a command while a path runs is held ("busy") and does not move the hand;
6. the pure rules (rate limit, P5 safety stop / slow-down / no target inside the person, physical gripper) run as
   tests/test_teleop.py (stand-ins for the runtime, so they hold whether or not P5 is merged in).
Run alone: start a server and call run(t) (see scripts/selftest.py)."""
import asyncio
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _tp(t):
    return t.state().get("teleop") or {}


def _hand(t, role="right"):
    return ((_tp(t).get("hands") or {}).get(role)) or {}


def _dist(a, b):
    return math.dist(a, b) if a and b else 0.0


async def _stream(t, role, p, seconds, grip=None, q=None, hz=30):
    t0 = time.time()
    n = 0
    while time.time() - t0 < seconds:
        msg = {"type": "teleop", "role": role, "p": [round(x, 4) for x in p], "t": round((time.time() - t0) * 1000, 1) + n * 1e-3}
        if grip is not None:
            msg["grip"] = grip
        if q is not None:
            msg["q"] = q
        await t.send(**msg)
        n += 1
        await t.pump(1 / hz)
    return n


async def _until(t, pred, limit=6.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.2)
        if pred(t.state()):
            return True
    return False


async def run(t):
    await t.task("reset_scene", 1)
    await t.pump(0.5)
    tp = _tp(t)
    t.check("teleop: state.teleop present and idle", isinstance(tp, dict) and tp.get("active") is False and "right" in (tp.get("hands") or {}),
            str({k: tp.get(k) for k in ("active", "role", "rate")}))

    await t.send(type="teleop", op="start", role="right", id="tp-start")
    await _until(t, lambda s: ((s.get("teleop") or {}).get("reply") or {}).get("id") == "tp-start", 4)
    tp = _tp(t)
    t.check("teleop: start takes the hand", tp.get("active") and (tp.get("reply") or {}).get("ok") and tp.get("role") == "right",
            (tp.get("reply") or {}).get("message"))

    h0 = _hand(t)
    p0, e0 = h0.get("p"), h0.get("ee")
    goal = [p0[0] + 0.02, p0[1] + 0.06, p0[2] + 0.08]
    n = await _stream(t, "right", goal, 1.6, q=h0.get("q"))
    await t.pump(0.6)
    h1, tp = _hand(t), _tp(t)
    moved = _dist(e0, h1.get("ee"))
    t.check("teleop: a 30 Hz stream moves the end effector", moved > 0.05 and _dist(h1.get("p"), goal) < 0.01,
            f"{n} commands, ee moved {moved * 100:.1f} cm, target off by {_dist(h1.get('p'), goal) * 100:.1f} cm")
    t.check("teleop: rate, lag and the echoed client time are reported",
            tp.get("echo") is not None and isinstance(tp.get("lag_ms"), int) and tp.get("lag_ms") < 2000 and tp.get("held") is None,
            f"rate {tp.get('rate')}/s, lag {tp.get('lag_ms')} ms, echo {tp.get('echo')}")

    # out of reach -> clamped (flagged); below the table -> stays above it
    far = [2.6, p0[1], 1.0]
    await _stream(t, "right", far, 1.2)
    tp, h2 = _tp(t), _hand(t)
    t.check("teleop: a target out of reach is clamped", tp.get("clamped") is True and h2["p"][0] < 1.2,
            f"asked x=2.6, target x={h2['p'][0]:.2f}")
    low = [0.45, p0[1], 0.40]
    await _stream(t, "right", low, 1.5)
    h3 = _hand(t)
    top = (t.state().get("traj") or {}).get("surface") or 0.74
    t.check("teleop: a target under the table stays above it", h3["p"][2] >= top + 0.01, f"z {h3['p'][2]:.3f} (table {top})")

    # grasp the mug, carry it up, put it down
    mug = next((o["pos"] for o in t.state().get("scene_objects", []) if o["name"] == "mug"), None)
    if mug is None:
        t.check("teleop: grasp the mug", False, "no mug in this world")
    else:
        at = [mug[0], mug[1], mug[2] + 0.085]
        await _stream(t, "right", at, 2.6, grip=0.0)
        await _stream(t, "right", at, 0.5, grip=1.0)
        held = _hand(t).get("holding")
        up = [at[0], at[1], at[2] + 0.12]
        await _stream(t, "right", up, 1.6, grip=1.0)
        mug_up = next((o["pos"] for o in t.state().get("scene_objects", []) if o["name"] == "mug"), mug)
        await _stream(t, "right", at, 1.6, grip=1.0)
        await _stream(t, "right", at, 0.4, grip=0.0)
        released = _hand(t).get("holding") is None
        mug_down = next((o["pos"] for o in t.state().get("scene_objects", []) if o["name"] == "mug"), mug)
        t.check("teleop: closing at the mug grasps it, lifting carries it, opening puts it down",
                held == "mug" and mug_up[2] - mug[2] > 0.06 and released and abs(mug_down[2] - mug[2]) < 0.01,
                f"holding={held} lift {100 * (mug_up[2] - mug[2]):.1f} cm, released={released}, back at z {mug_down[2]:.3f}")

    await t.send(type="teleop", op="stop", id="tp-stop")
    await _until(t, lambda s: ((s.get("teleop") or {}).get("reply") or {}).get("id") == "tp-stop", 4)
    t.check("teleop: stop lets go", _tp(t).get("active") is False, (_tp(t).get("reply") or {}).get("message"))

    # busy: while a path runs, teleop commands don't move the hand
    await t.task("home", 2)
    await t.pump(0.8)
    a = _hand(t).get("p")
    pts = [[a[0], a[1], a[2] + 0.04], [a[0] + 0.05, a[1], a[2] + 0.08], [a[0] + 0.05, a[1] + 0.08, a[2] + 0.08]]
    await t.send(type="traj", op="set", run=True, id="tp-busy", traj={"source": "vr", "frame": "world", "points": [{"p": p} for p in pts]})
    ok = await _until(t, lambda s: (s.get("traj") or {}).get("running"), 6)
    if ok:
        before = _hand(t).get("p")
        await t.send(type="teleop", role="right", p=[before[0], before[1] - 0.3, before[2] + 0.3])
        await t.pump(0.3)
        tp = _tp(t)
        t.check("teleop: a command while a path runs is held, not applied", str(tp.get("held") or "").startswith("busy"), tp.get("held"))
    else:
        t.check("teleop: a command while a path runs is held, not applied", False, "the path did not start")
    await t.task("stop", 1)
    await t.send(type="traj", op="clear")
    await t.pump(0.5)

    # the pure rules, incl. P5's safety stop / slow-down / no target inside the person (stand-ins, works without P5)
    r = await asyncio.get_event_loop().run_in_executor(None, lambda: subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(ROOT / "tests" / "test_teleop.py")], cwd=ROOT, capture_output=True, text=True, timeout=120))
    last = (r.stdout.strip().splitlines() or ["?"])[-1]
    t.check("teleop(unit): rate limit, reach, safety stop/slow/body zone, grippers (tests/test_teleop.py)", r.returncode == 0, last)
    if "safety" in t.state():                       # P5 merged: its state is there and teleop reports the same scale
        sc = (t.state().get("safety") or {}).get("speed_scale")
        t.check("teleop: reports P5's safety speed scale", sc is None or abs(float(_tp(t).get("scale", 1)) - float(sc)) < 0.05,
                f"safety {sc} teleop {_tp(t).get('scale')}")
