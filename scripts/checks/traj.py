"""P2 trajectories (docs/V1.md P2, docs/PROTOCOL.md v7) end to end, no LLM:

1. language keypoints -> a checked ghost + plan card; nothing moves; a path over the vase clears it;
2. edit a handle's height (the desktop handles' message) and a voice-style edit ("higher at the end");
3. run -> completes; a gripper close / open along the path picks the mug up and puts it down;
4. a blocked path explains why, and fixing only the bad point makes it runnable;
5. the old messages keep working: a stroke (draw) and a 3D path (path3d) become trajectories with source "draw", and
   follow_path runs them; the 2D height profile lifts a stroke;
6. the agent's make_trajectory / edit_trajectory / run_trajectory tools exist, and calling make_trajectory the way the
   model would (simulated tool call, in a thread like the real agent) proposes the path.
Run alone: start a server and call run(t) (see scripts/selftest.py)."""
import asyncio
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _tj(t):
    return t.state().get("traj") or {}


async def _op(t, rid, limit=8, **msg):
    await t.send(type="traj", id=rid, **msg)
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.2)
        r = _tj(t).get("reply") or {}
        if r.get("id") == rid:
            return r
    return {}


def _obj(st, name):
    return next((o for o in (st.get("traj") or {}).get("objects") or [] if o["name"] == name), None)


def _mug(st):
    return next((o["pos"] for o in st.get("scene_objects", []) if o["name"] == "mug"), [0, 0, 0])


async def _until(t, pred, limit=10):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.25)
        if pred(t.state()):
            return True
    return False


async def run(t):
    await t.task("reset_scene", 1)
    await t.send(type="traj", op="clear")
    await t.pump(0.6)
    st = t.state()
    t.check("traj: state.traj present and idle", isinstance(st.get("traj"), dict) and st["traj"].get("proposed") is None
            and st["traj"].get("current") is None and _obj(st, "vase") is not None, str(st.get("traj"))[:80])
    mug0 = _mug(st)

    # 1. propose from language keypoints: above the mug, then the left of the table (the vase is in between)
    spec = {"label": "over the vase to the left", "keypoints": [{"rel": "above", "of": "mug", "dz": 0.08}, {"region": "left", "dz": 0.08}]}
    r = await _op(t, "tj1", op="propose", spec=spec, task_id="tj-plan-1")
    await t.pump(0.6)
    st, tj = t.state(), _tj(t)
    g, p = tj.get("ghost") or {}, st.get("plan") or {}
    tr = tj.get("proposed") or {}
    t.check("traj: keypoints -> proposed trajectory + plan card", r.get("ok") and tr.get("source") == "language"
            and p.get("id") == "tj-plan-1" and p.get("phase") == "proposing" and (st.get("task") or {}).get("status") == "proposed",
            f"{r.get('message', '')[:70]} | plan {p.get('phase')}")
    t.check("traj: ghost has dense points, handles, orientation, timing", len(g.get("pts") or []) > 10 and len(g.get("keys") or []) == len(tr.get("points") or [])
            and (g.get("q") is None or len(g["q"]) == len(g["pts"])) and len(g.get("kq") or []) == len(g["keys"]) and (g.get("t") or [0])[-1] > 0.5 and all(g.get("ok") or [False]),
            f"{len(g.get('pts') or [])} samples, {len(g.get('keys') or [])} points, {(g.get('t') or [0])[-1]:.1f} s")
    vase = _obj(st, "vase")
    over = [q for q in g.get("pts") or [] if math.hypot(q[0] - vase["pos"][0], q[1] - vase["pos"][1]) < vase["r"] + 0.03]
    low = min((q[2] for q in over), default=None)
    t.check("traj: a keypoint plan over the vase clears it", "vase" in ((tr.get("meta") or {}).get("lifted") or [])
            and over and low >= vase["top"] + 0.04, f"lowest over the vase {low and (low - vase['top']) * 100:.1f} cm above its top"
            if low is not None else "never passes over the vase")
    await t.pump(1.0)
    st = t.state()
    t.check("traj: proposing never moves the robot", st.get("mode") == "IDLE" and max(abs(a - b) for a, b in zip(_mug(st), mug0)) < 0.005,
            f"mode {st.get('mode')}")

    # 2. edit: drag handle 2 up 5 cm (what web/ui/traj.js sends), then "higher at the end" (what the agent sends)
    k0 = list(g["keys"])
    up = [k0[1][0], k0[1][1], round(k0[1][2] + 0.05, 4)]
    r = await _op(t, "tj2", op="edit", index=1, point={"p": up}, world=True)
    await t.pump(0.4)
    k1 = (_tj(t).get("ghost") or {}).get("keys") or []
    t.check("traj: edit a handle's height", r.get("ok") and len(k1) == len(k0) and abs(k1[1][2] - up[2]) < 0.002
            and (t.state().get("plan") or {}).get("phase") == "proposing", f"{r.get('message', '')[:70]}")
    r = await _op(t, "tj3", op="edit", near="end", changes={"dz": 0.03})
    await t.pump(0.4)
    k2 = (_tj(t).get("ghost") or {}).get("keys") or []
    t.check("traj: voice edit 'higher at the end'", r.get("ok") and k2 and abs(k2[-1][2] - (k1[-1][2] + 0.03)) < 0.002, r.get("message", "")[:70])

    # 3. run -> the same task id goes running -> done
    r = await _op(t, "tj4", op="run")
    ran = await _until(t, lambda s: (s.get("task") or {}).get("id") == "tj-plan-1" and s["task"]["status"] in ("running", "done"), 6)
    running = _tj(t).get("running")
    fin = await t.until_done(30)
    await t.pump(0.5)
    tj = _tj(t)
    t.check("traj: run follows it to the end", r.get("ok") and ran and fin.get("id") == "tj-plan-1" and fin.get("status") == "done"
            and tj.get("current") and not tj.get("proposed") and not tj.get("running"),
            f"{fin.get('status')}: {fin.get('message', '')[:60]} (running seen: {running})")

    # 3b. gripper along the path: down to the mug, close, lift over, put it down at the center, open
    await t.task("reset_scene", 1)
    await t.pump(0.4)
    mug0 = _mug(t.state())
    spec = {"label": "move the mug to the center by hand", "keypoints": [
        {"rel": "above", "of": "mug", "dz": 0.06}, {"rel": "at", "of": "mug", "grip": 1}, {"rel": "above", "of": "mug", "dz": 0.10},
        {"xyz": [0.27, -0.07, 1.0]}, {"xyz": [0.27, -0.07, 0.91], "grip": 0}, {"dxyz": [0, 0, 0.06]}]}
    r = await _op(t, "tj5", op="set", spec=spec, run=True)
    started = await _until(t, lambda s: (s.get("traj") or {}).get("running"), 5)
    fin = await t.until_done(40)
    await t.pump(0.5)
    st = t.state()
    moved = math.hypot(_mug(st)[0] - mug0[0], _mug(st)[1] - mug0[1])
    t.check("traj: grip along the path picks and places", r.get("ok") and started and fin.get("status") == "done" and moved > 0.08
            and not any((st.get("holding") or {}).values()), f"{fin.get('message', '')[:80]} (mug moved {moved * 100:.0f} cm)")

    # 4. blocked: a point far behind the table, then fix only that point
    await t.task("reset_scene", 1)
    spec = {"label": "too far", "keypoints": [{"xyz": [0.27, -0.06, 0.86]}, {"xyz": [0.95, -0.2, 0.9]}]}
    r = await _op(t, "tj6", op="propose", spec=spec, task_id="tj-plan-2")
    await t.pump(0.4)
    st, tj = t.state(), _tj(t)
    p = st.get("plan") or {}
    t.check("traj: a blocked path says why", r.get("ok") is False and p.get("phase") == "blocked" and "reach" in (tj.get("why") or "")
            and (tj.get("ghost") or {}).get("ok") and not all(tj["ghost"]["ok"]), (tj.get("why") or "")[:90])
    r = await _op(t, "tj7", op="edit", index=-1, point={"p": [0.45, -0.25, 0.86]}, world=True)
    await t.pump(0.4)
    p = t.state().get("plan") or {}
    t.check("traj: fixing only the bad point makes it runnable", r.get("ok") and p.get("phase") == "proposing" and _tj(t).get("ok"),
            r.get("message", "")[:80])
    await t.send(type="plan", op="cancel", id="tj-plan-2")
    await t.pump(0.6)
    tj = _tj(t)
    t.check("traj: cancel drops the proposal", tj.get("proposed") is None and (t.state().get("plan") is None), str(tj.get("reply", {}).get("message"))[:40])

    # 5. old messages: a stroke and a 3D path become source "draw"; follow_path runs them; the height profile lifts a stroke
    await t.send(type="view", name="robot")
    await t.pump(0.5)
    await t.send(type="draw", points=[[0.4, 0.75], [0.5, 0.72], [0.6, 0.75]])
    await t.pump(1.0)
    tj, st = _tj(t), t.state()
    cur = tj.get("current") or {}
    t.check("traj: a stroke becomes a 'draw' trajectory", st.get("path") and cur.get("source") == "draw"
            and len(cur.get("points") or []) == len(st["path"]["pts"]), f"{len(cur.get('points') or [])} points")
    z0 = [q["p"][2] for q in cur.get("points") or []]
    r = await _op(t, "tj8", op="edit", profile={"lift": 0.03, "arch": 0.02})
    await t.pump(0.4)
    z1 = [q["p"][2] for q in (_tj(t).get("current") or {}).get("points") or []]
    lifted = [b - a for a, b in zip(z0, z1)]
    t.check("traj: the 2D height profile lifts the stroke (+ arch)", r.get("id") == "tj8" and lifted and min(lifted) > 0.025
            and max(lifted) > min(lifted) + 0.01, f"lift {min(lifted or [0]) * 100:.1f}..{max(lifted or [0]) * 100:.1f} cm · {r.get('message', '')[:50]}")
    await t.send(type="path3d", points=[[0.28, -0.30, 1.0], [0.29, -0.20, 1.05], [0.28, -0.10, 1.0]], hand="right")
    await t.pump(1.2)
    cur = _tj(t).get("current") or {}
    t.check("traj: a 3D editor path becomes a 'draw' trajectory", cur.get("source") == "draw" and len(cur.get("points") or []) >= 3,
            f"{len(cur.get('points') or [])} points")
    task = await t.task("follow_path", 2)
    fin = await t.until_done(25)
    t.check("traj: follow_path still runs a drawn path", task.get("status") in ("running", "done") and fin.get("status") == "done",
            f"{fin.get('message', '')[:60]}")
    await t.send(type="clear")
    await t.pump(0.5)
    t.check("traj: clear removes it", _tj(t).get("current") is None and t.state().get("path") is None)
    await t.send(type="view", name="robot")

    # 6. the agent's tools: schema, then a simulated tool call (the model's JSON, executed the way the agent does)
    from pps.agent import Agent, tool_specs
    specs = {s["name"]: s for s in tool_specs({"robot": "g1"})}
    mk = specs.get("make_trajectory") or {}
    kp = (((mk.get("parameters") or {}).get("properties") or {}).get("keypoints") or {}).get("items") or {}
    have = set((kp.get("properties") or {}))
    t.check("traj: agent tools make/edit/run_trajectory exist", {"make_trajectory", "edit_trajectory", "run_trajectory"} <= set(specs)
            and {"of", "rel", "via", "xyz", "dz", "speed", "force", "grip", "yaw", "tilt", "tol", "pattern"} <= have,
            ", ".join(sorted(have))[:90])
    loop = asyncio.get_running_loop()
    agent = Agent(push=lambda m: asyncio.run_coroutine_threadsafe(t.send(**m), loop), state=t.state, server_cfg=lambda: None, poll=0.1)

    async def call(name, args, confirm=True):
        fut = asyncio.ensure_future(asyncio.to_thread(agent.execute, name, args, confirm))
        while not fut.done():
            await t.pump(0.15)
        return fut.result()
    await t.task("reset_scene", 1)
    filled = {"from": "hand", "via": "over", "rel": "above", "region": "left", "xyz": [0, 0, 0], "dxyz": [0, 0, 0], "marker": "",
              "dx": 0, "dy": 0, "grip": 0, "force": 0, "yaw": 0, "pattern": False}      # models fill in every field
    res = await call("make_trajectory", {"description": "go over the vase to the left, slowly", "speed": 0.1, "role": "auto",
                                         "keypoints": [dict(filled, where="object", of="mug", dz=0.08),
                                                       dict(filled, where="pass", of="vase"),
                                                       {"region": "left", "dz": 0.08}]})       # no `where`: guessed
    p = t.state().get("plan") or {}
    pts = ((res.get("trajectory") or {}).get("points")) or []
    t.check("traj: agent make_trajectory proposes it", res.get("ok") and p.get("phase") == "proposing" and agent._proposed
            and any(q.get("near") == "vase" for q in pts), f"{res.get('message', '')[:70]}")
    n0 = len(pts)
    res = await call("edit_trajectory", {"near": "end", "changes": {"dz": 0.04, "speed_factor": 0.5}})
    t.check("traj: agent edit_trajectory ('higher and slower at the end')", res.get("ok")
            and len((res.get("trajectory") or {}).get("points") or []) == n0, res.get("message", "")[:80])
    await t.send(type="plan", op="cancel")
    await t.pump(0.5)
    await t.send(type="traj", op="clear")
    await t.pump(0.3)
