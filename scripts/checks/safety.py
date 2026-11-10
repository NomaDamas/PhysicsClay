"""v1 P5 Safety around people (#33): speed and separation monitoring against a synthetic person (no camera; frames are
played through {"type": "copresence_feed"} exactly like camera landmarks).

  * a person leaning toward the moving arm slows it; reaching into its path stops it and pauses the task with a
    message; stepping back resumes it and the task finishes;
  * no new motion during a stop; the UI can't switch safety off ("disable" and friends change nothing);
  * a task whose target is inside the person's torso is refused;
  * assist mode lets the hands come close (contact only with a force reading, which the kinematic sim doesn't have),
    while the head still stops the robot.
Pure maths (capsules, SSM formula, hysteresis, contact table) is in tests/test_safety.py.

Standalone (starts its own --fake server on PC_SAFETY_PORT, default 8925):
    .venv/bin/python scripts/checks/safety.py
"""
import asyncio
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pps.human import synth  # noqa: E402

LEFT_PATH = [[0.30 + 0.18 * (k % 2), 0.20, 0.95] for k in range(9)]     # the G1's left hand, back and forth
SLOW = dict(lean=30, at=[0.80, 0.20, 1.00])                            # leaning in, hand ~25 cm from the path
INTO = dict(lean=35, at=[0.55, 0.20, 0.97])                            # hand into the path's far end
BENT = dict(lean=55)                                                   # bent over the table: torso above its edge
HANDS = dict(lean=40, at=[0.55, 0.10, 1.05], at_l=[0.55, -0.10, 1.05])  # both hands out over the table (assist)
HEAD = dict(lean=75, at=[0.55, 0.10, 1.05], at_l=[0.55, -0.10, 1.05])   # ...and the head comes down to them


def _sf(t):
    return t.state().get("safety") or {}


async def feed(t, kw=None, hold=60.0):
    frames = synth.frames("lean", 0.3, **kw) if kw is not None else synth.frames("rest", 0.3)
    await t.send(type="copresence_feed", frames=frames, fps=20, hold=hold)


async def wait_for(t, pred, limit=6.0, step=0.15, sim=False):
    """Poll the state until pred holds, for `limit` s. sim=True: when it needs the robot to move, `limit` is seconds of
    the robot's own (sim) time while physics runs, since a loaded machine runs the plant slower than real time (capped
    at 4x the limit of wall time). The person's side (feeds, safety's resume_s) is real time either way."""
    t0, sim_s, last = time.time(), 0.0, None
    while time.time() - t0 < (4 * limit if sim else limit):
        await t.pump(step)
        try:
            if pred(t.state()):
                return True
        except Exception:
            pass
        ph = t.state().get("physics") or {}
        if not sim or ph.get("mode") != "dynamic":
            if time.time() - t0 >= limit:
                break
            continue
        now = float(ph.get("time") or 0.0)
        sim_s += max(0.0, now - last) if last is not None and now >= last else 0.0
        last = now
        if sim_s >= limit:
            break
    return False


async def clear(t):
    await t.send(type="copresence_feed", frames=[])
    await wait_for(t, lambda s: not (s.get("safety") or {}).get("stop") and not (s.get("safety") or {}).get("person"), 6.0)


async def run(t):
    st = t.state()
    if ((st.get("world") or {}).get("robot") or {}).get("id") not in (None, "g1"):
        await t.send(type="world", op="reset", id="safety-reset")
        await wait_for(t, lambda s: ((s.get("world") or {}).get("robot") or {}).get("id") == "g1", 25.0, 0.4)
    await t.send(type="watch", op="mode", mode="off", id="sf-off")
    await t.task("reset_scene", 1)
    await t.pump(0.6)
    sf = _sf(t)
    t.check("safety: state.safety in every frame; the no-camera dancer is not a person (full speed)",
            sf and sf.get("person") is False and sf.get("speed_scale") == 1.0 and sf.get("stop") is False and sf.get("config", {}).get("d_stop"),
            f"keys={sorted(sf)[:9]} person={sf.get('person')} scale={sf.get('speed_scale')}")

    # ---- a moving arm: the left hand goes back and forth over the table
    await t.send(type="path3d", points=LEFT_PATH, hand="left")
    await t.pump(0.4)
    task = await t.task("follow_path", 1)
    path_id = task.get("id")
    t.check("safety: the arm starts its path with nobody near (speed x1)", task.get("status") == "running" and _sf(t).get("speed_scale") == 1.0,
            f"{task.get('status')} {task.get('message')} scale={_sf(t).get('speed_scale')}")

    # paths start with a slow approach to their first point (P2): let the hand get onto the path before anyone leans in
    await wait_for(t, lambda s: ((s.get("path") or {}).get("i") or 0) >= 1, 8.0, sim=True)

    # a person leans in: slower, not stopped
    await feed(t, SLOW)
    ok = await wait_for(t, lambda s: 0.0 < (s.get("safety") or {}).get("speed_scale", 1) < 0.95 and not (s.get("safety") or {}).get("stop"), 4.0)
    sf = _sf(t)
    t.check("safety: a person leaning toward the moving arm slows it (inside d_slow, above d_stop)", ok and sf.get("zone") == "slow"
            and (sf.get("message") or "").startswith("Slowing: you're") and sf.get("person"),
            f"scale={sf.get('speed_scale')} d_min={sf.get('d_min')} msg={sf.get('message')!r} closest={(sf.get('closest') or {}).get('human_part')}")

    # the hand reaches into the path: protective stop, the task pauses with a message
    await feed(t, INTO)
    ok = await wait_for(t, lambda s: (s.get("safety") or {}).get("stop"), 8.0)
    await t.pump(0.4)
    sf, task = _sf(t), t.state().get("task") or {}
    i0 = (t.state().get("path") or {}).get("i")
    t.check("safety: crossing d_stop -> protective stop, speed 0, the task pauses with an honest message",
            ok and sf.get("speed_scale") == 0 and task.get("status") == "running" and task.get("step") == "paused"
            and str(task.get("message", "")).startswith("Paused for safety") and (sf.get("message") or "").startswith("Stopped"),
            f"stop={sf.get('stop')} d_min={sf.get('d_min')} reason={sf.get('reason')} task={task.get('status')}/{task.get('step')}: {task.get('message')!r}")

    # the UI cannot switch safety off
    cfg0 = dict(sf.get("config") or {})
    for m in ({"op": "disable"}, {"op": "off"}, {"op": "set", "params": {"d_stop": 0.0, "d_slow": 0.0}}, {"enabled": False},
              {"op": "config", "d_stop": 0.01}):
        await t.send(type="safety", id=f"sf-{m.get('op', 'x')}", **m)
    await t.pump(1.2)
    sf = _sf(t)
    i1 = (t.state().get("path") or {}).get("i")
    t.check("safety: the robot holds still while stopped", i0 is not None and i1 == i0 and sf.get("stop"), f"path i {i0} -> {i1}")
    t.check("safety: the UI can't disable it ('disable', 'off', 'set', enabled:false change nothing)",
            sf.get("stop") and sf.get("config") == cfg0 and (sf.get("reply") or {}).get("ok") is False,
            f"stop={sf.get('stop')} reply={sf.get('reply')} config same={sf.get('config') == cfg0}")

    # the person steps back: automatic resume (after resume_s), the task finishes
    await feed(t)
    ok = await wait_for(t, lambda s: not (s.get("safety") or {}).get("stop") and (s.get("path") or {}).get("i", -1) != i1, 6.0, sim=True)
    task = t.state().get("task") or {}
    t.check("safety: moving away resumes automatically (clear for resume_s, then a ramp)", ok and task.get("step") != "paused"
            and not str(task.get("message", "")).startswith("Paused"), f"stop={_sf(t).get('stop')} task={task.get('message')!r}")
    done = await t.until_done(30)
    t.check("safety: the paused task then finishes", done.get("id") == path_id and done.get("status") == "done",
            f"{done.get('id')} {done.get('status')} {done.get('message')!r}")
    await t.task("reset_scene", 1)

    # ---- a target inside the person's body zone is refused
    await feed(t, BENT)
    await wait_for(t, lambda s: (s.get("safety") or {}).get("person"), 3.0)
    torso = next((c for c in _sf(t).get("capsules") or [] if c[7] == "torso"), None)
    if torso is None:
        t.check("safety: a task targeting the person's torso is refused", False, "no torso capsule in state.safety")
    else:
        a, b = torso[:3], torso[3:6]
        chest = [a[0] + 0.25 * (b[0] - a[0]), a[1] + 0.25 * (b[1] - a[1]), a[2] + 0.25 * (b[2] - a[2])]
        before = {m["id"] for m in t.state().get("markers") or []}
        await t.send(type="point3d", pos=[chest[0], chest[1], chest[2] - 0.12])    # reach hovers 12 cm above a marker
        await t.pump(0.4)
        mk = next((m["id"] for m in t.state().get("markers") or [] if m["id"] not in before), None)
        r = await t.task("reach", 1, target=f"marker {mk}")
        t.check("safety: a task targeting the person's torso is refused", mk and r.get("status") == "failed" and "body zone" in str(r.get("message", "")),
                f"marker={mk} chest={[round(x, 2) for x in chest]} -> {r.get('status')} {r.get('message')!r}")
        r = await t.task("reach", 1, target="left")
        t.check("safety: ...while a target away from the person is fine", r.get("status") in ("running", "done"), f"{r.get('status')} {r.get('message')!r}")
        await t.task("stop", 1)
    await t.send(type="clear")
    await t.task("reset_scene", 1)

    # ---- assist: the hands may come close (contact needs a force reading), the head still stops the robot
    await feed(t, HANDS)
    # the synthetic hands jump toward the robot when the feed switches (a fast approach: SSM may stop for it), so
    # let the person settle before assist starts
    await wait_for(t, lambda s: (s.get("safety") or {}).get("person") and not (s.get("safety") or {}).get("stop"), 6.0)
    await t.send(type="watch", op="mode", mode="assist", id="sf-assist")
    seen = {"body": False}

    def near_hands(s):
        f = s.get("safety") or {}
        b = f.get("binding") or {}
        if f.get("stop") and f.get("reason") == "body" and b.get("robot_kind") == "ee" and b.get("region") in ("hand", "forearm"):
            seen["body"] = True      # the hands vs the gripper must never be a body stop while cooperating (the robot's
            #                          upper arm passing the hands at speed still may: that pair isn't cooperative)
        c = f.get("closest") or {}
        return f.get("coop") and f.get("d_min") is not None and f["d_min"] < f.get("config", {}).get("d_stop", 0.12) \
            and c.get("region") in ("hand", "forearm")
    ok = await wait_for(t, near_hands, 12.0, sim=True)
    sf = _sf(t)
    t.check("safety: assist lets the robot's hands come closer than d_stop to the person's hands (hand vs gripper is never a body stop)",
            ok and not seen["body"] and (not sf.get("stop") or sf.get("reason") == "no_force") and sf.get("speed_scale", 1) <= sf.get("config", {}).get("coop_scale", 0.3) + 1e-6,
            f"d_min={sf.get('d_min')} coop={sf.get('coop')} stop={sf.get('stop')}/{sf.get('reason')} scale={sf.get('speed_scale')} "
            f"closest={(sf.get('closest') or {}).get('human_part')} binding={sf.get('binding')}")
    if (t.state().get("physics") or {}).get("mode") == "dynamic":     # P0 measures the end-effector force
        fmax = (sf.get("config") or {}).get("f_contact_max", 40.0)
        t.check("safety: assist: contact only with a measured force (P0 physics: the force is read and under the cap, else a latched stop)",
                sf.get("force") is not None and (sf["force"] <= fmax or (sf.get("stop") and sf.get("latched"))),
                f"force={sf.get('force')} cap={fmax} d_min={sf.get('d_min')} stop={sf.get('stop')} latched={sf.get('latched')}")
    else:
        t.check("safety: assist: contact only with a measured force (none in the kinematic sim -> stop at contact, never a press)",
                sf.get("force") is None and (sf.get("d_min", 1) > -0.02 or sf.get("stop")), f"force={sf.get('force')} d_min={sf.get('d_min')} stop={sf.get('stop')}")
    await feed(t, HEAD)
    ok = await wait_for(t, lambda s: (s.get("safety") or {}).get("stop") and (s.get("safety") or {}).get("reason") == "critical", 6.0)
    sf = _sf(t)
    t.check("safety: head and body close -> stop even in assist ('Stopped: too close to your head|body')", ok and "too close to your" in str(sf.get("message")),
            f"stop={sf.get('stop')} reason={sf.get('reason')} msg={sf.get('message')!r} watch={(t.state().get('watch') or {}).get('mode')}")
    await t.send(type="watch", op="mode", mode="off", id="sf-off2")
    await t.send(type="task", id="sf-speed", action="set_speed", value=1.0)     # the speed command can't override it
    await t.pump(0.6)
    r = await t.task("reach", 1, target="center")
    t.check("safety: no new motion during a protective stop", _sf(t).get("stop") and r.get("status") == "failed"
            and "can't move" in str(r.get("message", "")), f"{r.get('status')} {r.get('message')!r}")
    await clear(t)
    await t.task("stop", 1)


# ---------------------------------------------------------------------- standalone
def _main():
    import aiohttp
    port = int(os.environ.get("PC_SAFETY_PORT", "8925"))
    base = f"http://127.0.0.1:{port}"
    results = []

    def check(name, ok, detail=""):
        results.append((name, bool(ok)))
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)

    async def go():
        from types import SimpleNamespace
        s = aiohttp.ClientSession()
        for _ in range(120):
            try:
                async with s.get(base + "/") as r:
                    if r.status == 200:
                        break
            except Exception:
                await asyncio.sleep(0.5)
        ws = await s.ws_connect(base + "/ws")
        st = {}

        async def pump(sec):
            nonlocal st
            t0 = time.time()
            while time.time() - t0 < sec:
                m = await asyncio.wait_for(ws.receive(), 10)
                if m.type == aiohttp.WSMsgType.TEXT:
                    st = json.loads(m.data)
            return st

        async def task(action, wait=1.0, **kw):
            tid = f"{action}-{time.time():.3f}"
            await ws.send_str(json.dumps({"type": "task", "id": tid, "action": action, **kw}))
            t0 = time.time()
            while time.time() - t0 < wait:
                await pump(0.2)
                if st.get("task") and st["task"]["id"] == tid and st["task"]["status"] != "planning":
                    break
            return st.get("task") or {}

        async def until_done(limit=25):
            t0 = time.time()
            while time.time() - t0 < limit:
                await pump(0.3)
                if (st.get("task") or {}).get("status") != "running":
                    break
            return st.get("task") or {}

        t = SimpleNamespace(send=lambda **k: ws.send_str(json.dumps(k)), pump=pump, task=task, until_done=until_done,
                            check=check, state=lambda: st, http=s, base=base, frames=lambda: {})
        try:
            await pump(1.0)
            await run(t)
        finally:
            await ws.close()
            await s.close()

    srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--port", str(port),
                            "--https-port", str(port + 1)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        asyncio.run(go())
    finally:
        srv.terminate()
        try:
            srv.wait(10)
        except subprocess.TimeoutExpired:
            srv.kill()
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    _main()
