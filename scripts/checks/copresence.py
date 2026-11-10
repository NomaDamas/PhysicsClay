"""UX 3 co-presence (#20): the person across the table, pointing in the robot's world frame, deixis, handover.

Deterministic: {"type": "copresence_feed"} plays synthetic landmark frames (pps.features.copresence.synth_pose builds a
person standing across the table who points at a world point or holds out an open hand) instead of the camera.

Standalone (starts its own --fake --mock server on PC_COPRESENCE_PORT, default 8931); extra check names run after it
on the same server (e.g. the neighbours this feature touches):
    .venv/bin/python scripts/checks/copresence.py [candidates point gestures plan]
"""
import asyncio
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pps.features.copresence import synth_pose, to_world  # noqa: E402

TABLE_TOP = 0.74
SPOT = [0.40, -0.32, TABLE_TOP]         # empty, reachable spot (clear of the red block; no carry past the vase)
HAND = [0.80, -0.05, TABLE_TOP + 0.25]  # where the person holds out their hand (beyond the robot's reach)


def _d(a, b, n=2):
    return math.dist(a[:n], b[:n])


def _cp(t):
    return t.state().get("copresence") or {}


def unit(check):
    """The mapping as pure functions: across the table, facing the robot, not mirrored, real proportions."""
    J = to_world(synth_pose(point_at=[0.3, -0.3, TABLE_TOP]))
    check("copresence: avatar stands across the table", abs(J["pelvis"][0] - 1.15) < 1e-6 and 1.5 < J["head"][2] < 1.8
          and J["l_ankle"][2] < 0.1, f"pelvis={J['pelvis'].round(2)} head={J['head'].round(2)}")
    check("copresence: not mirrored (the person's left hand is on the robot's right)", J["l_shoulder"][1] < 0 < J["r_shoulder"][1]
          and J["l_index"][1] < 0, f"l_sh={J['l_shoulder'].round(2)} r_sh={J['r_shoulder'].round(2)}")
    check("copresence: pointing hand reaches toward the robot", J["l_index"][0] < J["l_shoulder"][0] - 0.3, f"{J['l_index'].round(2)}")


async def feed(t, frames, hold=60.0):
    await t.send(type="copresence_feed", frames=frames, fps=20, hold=hold)
    # A new feed disarms the pointing (re-arming takes ARM_HOLD). Wait until a state frame shows that, so the checks
    # below never read the previous pose's frames. On a slow server (CI renders in software) that takes a while.
    await wait_for(t, lambda c: not (c.get("pointing") or {}).get("armed"), 3.0, 0.1)


async def wait_for(t, pred, limit=4.0, step=0.2):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(step)
        if pred(_cp(t)):
            return True
    return False


async def utter(t, text, rid, run=True):
    await t.send(type="copresence", op="utter", text=text, id=rid, run=run)
    await wait_for(t, lambda c: (c.get("reply") or {}).get("id") == rid, 3.0)
    return _cp(t).get("reply") or {}


def objpos(t, name):
    return next((o["pos"] for o in t.state().get("scene_objects") or [] if o["name"] == name.replace("_", " ")), None)


async def run(t):
    unit(t.check)
    await t.task("reset_scene", 1)
    await t.send(type="speed", scale=1.0)
    st = await t.pump(0.8)
    cp = st.get("copresence") or {}
    j = cp.get("joints") or {}
    t.check("copresence: synthetic dancer drives the avatar (joints in state)", cp.get("on") and all(k in j for k in ("head", "l_wrist", "r_wrist", "pelvis", "l_ankle"))
            and abs(j["pelvis"][0] - 1.15) < 0.01, f"keys={sorted(j)[:6]} src={cp.get('source')}")
    t.check("copresence: the synthetic dancer never arms pointing", not (cp.get("pointing") or {}).get("armed"), str(cp.get("pointing")))

    # 1. pointing at the mug -> armed after the hold, hit near the mug, candidate = mug, attention on it
    mug = objpos(t, "mug")
    await feed(t, [synth_pose(point_at=mug)] * 30)
    early = await wait_for(t, lambda c: (c.get("pointing") or {}).get("hit") is not None, 1.0, 0.1)
    armed_early = (_cp(t).get("pointing") or {}).get("armed")
    ok = await wait_for(t, lambda c: (c.get("pointing") or {}).get("armed"), 3.0)
    p = _cp(t).get("pointing") or {}
    att = _cp(t).get("attention") or {}
    t.check("copresence: pointing arms only after the hold (debounce)", early and not armed_early and ok, f"early={early} armed_early={armed_early}")
    t.check("copresence: pointing at the mug -> hit at the mug, candidate = mug", p.get("hit") and _d(p["hit"], mug) < 0.05
            and p.get("target") == "mug" and p.get("candidates") == ["mug"], f"hit={p.get('hit')} mug={mug} cands={p.get('candidates')}")
    t.check("copresence: attention on the pointed object", att.get("kind") == "object" and att.get("name") == "mug", str(att))

    # 2. "pick this up" -> picks up the pointed mug and holds it
    r = await utter(t, "pick this up", "cp-1")
    task = r.get("task") or {}
    t.check("copresence: 'pick this up' resolves to the pointed mug", r.get("ok") and task.get("action") == "pick_place" and task.get("hold")
            and task.get("object") == "mug", json.dumps(r)[:200])
    done = await t.until_done(20)
    hold = t.state().get("holding") or {}
    t.check("copresence: picked up and holding the mug", done.get("status") == "done" and "mug" in hold.values(), f"{done.get('status')} {done.get('message')} {hold}")

    # 3. "put it here" while pointing at an empty spot -> a marker at the hit, the mug goes there
    await feed(t, [synth_pose(point_at=SPOT)] * 30)
    await wait_for(t, lambda c: (c.get("pointing") or {}).get("armed"), 3.0)
    p = _cp(t).get("pointing") or {}
    t.check("copresence: pointing at an empty spot -> exact table hit", p.get("hit") and _d(p["hit"], SPOT) < 0.02 and not p.get("candidates"),
            f"hit={p.get('hit')} cands={p.get('candidates')}")
    r = await utter(t, "put it here", "cp-2")
    mk = next((m for m in t.state().get("markers") or [] if m["id"] == r.get("marker")), None)
    t.check("copresence: 'put it here' -> marker at the pointed spot (+-5 cm)", r.get("ok") and mk and _d(mk["pos"], SPOT) < 0.05
            and (r.get("task") or {}).get("target") == f"marker {r.get('marker')}", f"marker={mk} reply={json.dumps(r)[:160]}")
    done = await t.until_done(25)
    mugp = objpos(t, "mug")
    t.check("copresence: the mug is placed at the spot", done.get("status") == "done" and mugp and _d(mugp, SPOT) < 0.06, f"{done.get('message')} mug={mugp}")

    # 4. "which one?" when the ray passes between two objects
    green = objpos(t, "green_block")
    await t.task("reset_scene", 1)
    mug, green = objpos(t, "mug"), objpos(t, "green_block")
    mid = [(mug[0] + green[0]) / 2, (mug[1] + green[1]) / 2, TABLE_TOP]
    await feed(t, [synth_pose(point_at=mid)] * 30)
    await wait_for(t, lambda c: (c.get("pointing") or {}).get("armed"), 3.0)
    r = await utter(t, "grab that", "cp-3")
    await t.pump(0.6)
    ch = t.state().get("choices") or {}
    t.check("copresence: pointing between two things -> 'which one?'", len(r.get("candidates") or []) == 2
            and sorted(o["name"] for o in ch.get("options") or []) == sorted(["mug", "green_block"]), f"cands={r.get('candidates')} choices={ch.get('question')}")
    if ch:
        await t.send(type="choose", id=ch["id"], key="cancel")
        await t.pump(0.3)

    # 5. "give it to me" with an open hand while holding -> reach toward the hand (clamped), let go when it's close
    await feed(t, [synth_pose(point_at=objpos(t, "mug"))] * 30)
    await wait_for(t, lambda c: (c.get("pointing") or {}).get("armed"), 3.0)
    await utter(t, "pick this up", "cp-4")
    await t.until_done(35)          # the pointing hand is within half a metre of the arm: it works slower (P5 safety)
    await feed(t, [synth_pose(offer=HAND)] * 20)
    await wait_for(t, lambda c: c.get("offer") is not None, 2.0)
    c0 = _cp(t)
    r = await utter(t, "give it to me", "cp-5")
    await t.pump(0.4)
    ho = _cp(t).get("handover") or {}
    hand, tgt = ho.get("hand"), ho.get("target")
    t.check("copresence: open hand held out is seen", c0.get("offer") is not None and (c0.get("hand_open") or {}).get(c0.get("offer")) is True, f"offer={c0.get('offer')} open={c0.get('hand_open')}")
    t.check("copresence: 'give it to me' -> reach target toward the hand, clamped to reach",
            (r.get("task") or {}).get("action") == "handover" and hand and tgt and _d(hand, HAND, 3) < 0.08
            and tgt[0] <= hand[0] + 1e-3 and math.dist(tgt, hand) < 0.45 and (t.state().get("task") or {}).get("status") == "running",
            f"hand={hand} target={tgt} task={(t.state().get('task') or {}).get('message')}")
    done = await t.until_done(20)
    ok = await wait_for(t, lambda c: (c.get("handover") or {}).get("phase") == "offering", 2.0)
    t.check("copresence: holds the object out, waiting for your hand", ok and "mug" in (t.state().get("holding") or {}).values(), f"{done.get('message')} phase={(_cp(t).get('handover') or {}).get('phase')}")
    await feed(t, [synth_pose(offer=tgt)] * 20)     # the person's hand comes to the robot's hand
    ok = await wait_for(t, lambda c: c.get("given") == "mug", 3.0)
    t.check("copresence: lets go when your hand is close -> you hold the mug", ok and "mug" not in (t.state().get("holding") or {}).values(),
            f"given={_cp(t).get('given')} phase={(_cp(t).get('handover') or {}).get('phase')}")
    if ok:   # 5b. the person draws their hand back with it: the jaws open first, the mug comes along, it isn't thrown or dropped
        away = [tgt[0] + 0.15, tgt[1], tgt[2]]
        start = objpos(t, "mug")
        await feed(t, [synth_pose(offer=away)] * 20)
        track, sim, last, t0 = [start] if start else [], 0.0, None, time.time()
        while sim < 4.0 and time.time() - t0 < 40:            # 4 s of the robot's time: open, clear, out, to your hand
            await t.pump(0.2)
            m = objpos(t, "mug")
            if m:
                track.append(m)
            ph = t.state().get("physics") or {}
            if ph.get("mode") != "dynamic":
                sim = time.time() - t0
                continue
            now = float(ph.get("time") or 0.0)
            sim += max(0.0, now - last) if last is not None and now >= last else 0.0
            last = now
        j = _cp(t).get("joints") or {}
        sd = c0.get("offer") or "l"
        hand = [(a + b) / 2 for a, b in zip(j[f"{sd}_wrist"], j[f"{sd}_index"])] if f"{sd}_wrist" in j and f"{sd}_index" in j else away
        end = track[-1] if track else None
        t.check("copresence: you take the mug away: it comes with your hand (not thrown, not dropped)",
                _cp(t).get("given") == "mug" and start and end and math.dist(end, hand) < 0.09 and math.dist(end, away) < math.dist(start, away) - 0.05
                and min(p[2] for p in track) > TABLE_TOP + 0.05 and "mug" not in (t.state().get("holding") or {}).values(),
                f"mug {start} -> {end}, your hand {[round(x, 3) for x in hand]}, lowest z {min((p[2] for p in track), default=0):.3f}, "
                f"given={_cp(t).get('given')}")

    # 6. hand paths in robot-world coordinates
    paths = _cp(t).get("hand_paths") or {}
    t.check("copresence: hand paths in the robot's world frame", len(paths.get("l") or []) + len(paths.get("r") or []) > 3
            and all(p[0] > 0.5 for p in (paths.get("l") or []) + (paths.get("r") or [])), f"n={len(paths.get('l') or [])}/{len(paths.get('r') or [])}")
    await t.send(type="copresence_feed", frames=[])
    await t.task("reset_scene", 1)


# ---------------------------------------------------------------------- standalone runner
def _main():
    import aiohttp
    port = int(os.environ.get("PC_COPRESENCE_PORT", "8931"))
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
                if (st.get("task") or {}).get("status") not in ("running", "planning"):
                    break
            return st.get("task") or {}

        t = SimpleNamespace(send=lambda **k: ws.send_str(json.dumps(k)), pump=pump, task=task, until_done=until_done,
                            check=check, state=lambda: st, http=s, base=base, frames=lambda: {})
        try:
            import importlib.util
            await run(t)
            for name in sys.argv[1:]:
                await task("reset_scene", 1)
                spec = importlib.util.spec_from_file_location(f"checks_{name}", ROOT / "scripts" / "checks" / f"{name}.py")
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                try:
                    await mod.run(t)
                except Exception as e:
                    check(f"checks/{name}.py ran", False, repr(e)[:120])
        finally:
            await ws.close()
            await s.close()

    srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--mock", "--port", str(port),
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
