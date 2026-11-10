"""v1 P3 Watching the person (#31): activity and reps, coach cues, respond rules, mirror on an arm, learn -> demo file,
assist on a stall, and the agent's watch tools. No camera: synthetic people (pps/human/synth.py) are played through
{"type": "copresence_feed"} exactly like camera landmarks.

Standalone (starts its own --fake server on PC_WATCH_PORT, default 8915):
    .venv/bin/python scripts/checks/watch.py
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

from pps.features.copresence import to_world  # noqa: E402
from pps.human import coach, synth  # noqa: E402
from pps.human.demo import DemoRecorder, place, to_trajectory  # noqa: E402
from pps.human.model import HumanModel  # noqa: E402
from pps.human.rules import Predicate, RuleError, compile_do, compile_when, split_rule  # noqa: E402

TABLE_TOP = 0.74
DEMO = "selftest_weave"


def _play(frames, fps=20):
    m, reps, acts = HumanModel(), [], []
    for i, f in enumerate(frames):
        st = m.update(to_world(f), "feed", i / fps)
        reps += [v for k, v in m.events if k == "rep"]
        if not acts or acts[-1] != st["activity"]:
            acts.append(st["activity"])
    return st, reps, acts, m


def unit(check):
    st, reps, acts, _ = _play(synth.frames("squat", reps=5))
    check("watch(unit): 5 synthetic squats -> activity squat, 5 reps", st["activity"] == "squat" and st["reps"] == 5, f"{acts} reps={st['reps']}")
    check("watch(unit): good squats get no form cues", all(not coach.cues(r, coach.reference("squat")) for r in reps),
          str([coach.cues(r, coach.reference("squat")) for r in reps]))
    st, reps, _, _ = _play(synth.frames("squat", reps=4, valgus=1.0, lean=60, knee_min=120))
    cs = [coach.cues(r, coach.reference("squat")) for r in reps]
    check("watch(unit): coach flags knee valgus, depth and back angle", reps and all({"valgus", "depth", "back"} <= set(c) for c in cs), str(cs))
    st, _, acts, _ = _play(synth.frames("weave", 6, amp=0.015, freq=2.0))
    wv = st.get("weave") or {}
    check("watch(unit): weave detected with its width and frequency", st["activity"] == "weave" and abs(wv.get("amp", 0) - 0.015) < 0.004
          and abs(wv.get("freq", 0) - 2.0) < 0.35, f"{acts} {wv}")
    st, _, acts, _ = _play(synth.frames("wave", 4))
    check("watch(unit): wave", st["activity"] == "wave", str(acts))
    st, _, acts, _ = _play(synth.frames("curl", 8))
    check("watch(unit): curl counted", st["activity"] == "curl" and st["reps"] >= 2, f"{acts} reps={st['reps']}")
    st, _, acts, _ = _play(synth.frames("press", 8))
    check("watch(unit): overhead press counted", st["activity"] == "press" and st["reps"] >= 2, f"{acts} reps={st['reps']}")
    st, _, acts, _ = _play(synth.frames("rest", 3))
    check("watch(unit): still", st["activity"] == "still", str(acts))
    st, _, acts, _ = _play(synth.frames("squat", 9, stall_at=(2, 0.4)))
    check("watch(unit): a stall mid-rep is detected", st["stalled"] and st["phase"] == "up", f"{acts} phase={st['phase']}")
    st, _, _, m = _play(synth.frames("offer", 2))
    check("watch(unit): 33 world landmarks, grounded feet, hands", len(st["pose"]) == 33 and all(p is not None for p in st["pose"])
          and min(m.J["l_ankle"][2], m.J["r_ankle"][2]) < 0.08 and "right" in st["hands"] and "left" in st["hands"], f"filled={st['pose_filled']}")
    # rules
    sp = split_rule("When I put my hand out, give me the wrench")
    w, d = compile_when(sp[0]), compile_do(sp[1], ["wrench", "mug"])
    check("watch(unit): 'when I put my hand out, give me the wrench' -> hand_open_near_robot / handover wrench",
          w == "hand_open_near_robot" and d == {"task": {"action": "handover", "object": "wrench"}}, f"{w} {d}")
    check("watch(unit): offline phrases", compile_when("after 5 squats") == "activity:squat and reps>=5"
          and compile_when("I'm holding the torch") == "holding:torch" and compile_when("I get stuck") == "stalled",
          f"{compile_when('after 5 squats')} | {compile_when('I am holding the torch')}")
    h = {"visible": True, "activity": "squat", "reps": 6, "holding": "torch", "hands": {"right": {"open": True, "near_robot": True}}}
    ok = Predicate("activity:squat and reps>=5")(h) and Predicate(expr='holding == "torch" and hands.right.open')(h) and not Predicate("stalled")(h)
    bad = []
    for e in ("__import__('os').system('x')", "().__class__", "h[0]"):
        try:
            Predicate(expr=e)
            bad.append(e)
        except RuleError:
            pass
    check("watch(unit): predicates over state.human; unsafe expressions refused", ok and not bad, f"accepted={bad}")
    # demo -> stub trajectory
    rec = DemoRecorder("u", "panda", {"plate": [0.79, 0.04, 0.76], "mug": [0.40, -0.2, 0.78]}, now=0.0)
    mm = HumanModel()
    for i, f in enumerate(synth.frames("weave", 5)):
        hs = mm.update(to_world(f), "feed", i / 20)
        rec.add(i / 20, hs, [v for k, v in mm.events if k == "rep"])
    demo = rec.finish()
    tr = demo and demo["trajectory"]
    pts = place(tr, [0.45, 0.0, 0.76]) if tr else []
    check("watch(unit): demo -> object-relative trajectory (stub extractor)", demo and demo["activity"] == "weave" and tr["frame"] == "object:plate"
          and len(tr["points"]) > 10 and tr["pattern"] and abs(pts[0][0] - 0.45) < 1e-6, f"{(demo or {}).get('activity')} {tr and tr['frame']} {tr and tr['pattern']}")
    # agent tools
    from pps.agent import tool_specs
    specs = {t["name"]: t for t in tool_specs({})}
    need = ("set_watch_mode", "add_watch_rule", "learn_from_demo", "mirror_person")
    check("watch(unit): agent tools set_watch_mode / add_watch_rule / learn_from_demo / mirror_person",
          all(n in specs for n in need) and set(specs["set_watch_mode"]["parameters"]["properties"]["mode"]["enum"])
          == {"off", "mirror", "learn", "respond", "assist", "coach"}, ",".join(n for n in need if n not in specs))
    from pps.agent import Agent
    sent, world = [], {"watch": {"mode": "off"}, "human": {"visible": True, "activity": "still"}}

    def push(m):                      # answers like the server's watch handler would
        sent.append(m)
        world["watch"] = {"mode": "respond", "reply": {"id": m.get("id"), "op": m.get("op"), "ok": True, "message": "Okay.",
                                                       "rule": {"id": "R1", "when": m.get("when"), "expr": m.get("expr")}}}
    ag = Agent(push=push, state=lambda: dict(world), server_cfg=lambda: None, poll=0.01)
    res = ag.execute("add_watch_rule", {"when": "my hand is shaking", "do": "say slow down", "expr": "speed > 0.6 and activity == 'still'"})
    m = sent[-1] if sent else {}
    check("watch(unit): agent add_watch_rule -> {type: watch, op: rule, when, do, expr} and reads the reply",
          m.get("type") == "watch" and m.get("op") == "rule" and m.get("expr") and res.get("ok") and res.get("rule", {}).get("id") == "R1", f"{m} -> {res}")
    prompt = (ROOT / "pps" / "world" / "prompts" / "base.md").read_text()
    check("watch(unit): base prompt teaches watching ('do it like me', 'watch me')", "do it like me" in prompt.lower() and "watch me" in prompt.lower())


# ---------------------------------------------------------------------- end to end
def _w(t):
    return t.state().get("watch") or {}


def _h(t):
    return t.state().get("human") or {}


async def feed(t, frames, hold=30.0):
    await t.send(type="copresence_feed", frames=frames, fps=20, hold=hold)


async def watch(t, **kw):
    rid = f"wt{time.time():.3f}"
    await t.send(type="watch", id=rid, **kw)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        await t.pump(0.15)
        r = _w(t).get("reply") or {}
        if r.get("id") == rid:
            return r
    return {}


async def wait_for(t, pred, limit=6.0, step=0.2):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(step)
        try:
            if pred(t.state()):
                return True
        except Exception:
            pass
    return False


async def run(t):
    unit(t.check)
    await t.task("reset_scene", 1)
    await t.send(type="speed", scale=1.0)
    await t.pump(0.6)
    h = _h(t)
    t.check("watch: state.human and state.watch in every frame", "visible" in h and _w(t).get("mode") == "off",
            f"human keys={sorted(h)[:8]} watch={_w(t).get('mode')}")

    # ---- activity + reps, live through the server
    await feed(t, synth.frames("squat", reps=5), hold=2.0)
    ok = await wait_for(t, lambda s: (s.get("human") or {}).get("reps") == 5, 16.0)
    h = _h(t)
    t.check("watch: synthetic squats -> human.activity squat, 5 reps", ok and h.get("activity") == "squat",
            f"activity={h.get('activity')} reps={h.get('reps')} phase={h.get('phase')} summary={h.get('summary')!r}")

    # ---- coach: bad form gets cues, reps counted
    r = await watch(t, op="mode", mode="coach")
    await feed(t, synth.frames("squat", reps=4, valgus=1.0, knee_min=120), hold=4.0)
    ok = await wait_for(t, lambda s: ((s.get("watch") or {}).get("coach") or {}).get("reps", 0) >= 3, 14.0)
    c = _w(t).get("coach") or {}
    cues = {k for x in c.get("history") or [] for k in x.get("cues") or []}
    t.check("watch: coach counts reps and says 'knees caving in' / 'deeper'", r.get("ok") and ok and {"valgus", "depth"} <= cues,
            f"reps={c.get('reps')} cues={sorted(cues)} last={c.get('last')}")
    await watch(t, op="mode", mode="off")

    # ---- respond: a voice-free rule hands the mug over once, with cooldown
    await watch(t, op="rules_clear")
    r = await watch(t, op="rule", text="when I put my hand out, give me the mug", cooldown=20)
    rule = r.get("rule") or {}
    t.check("watch: rule 'when I put my hand out, give me the mug' compiled", r.get("ok") and rule.get("when") == "hand_open_near_robot"
            and (rule.get("task") or {}).get("action") == "handover" and _w(t).get("mode") == "respond", f"{r.get('message')} {rule}")
    offer = synth.frames("offer", 0.5, target=[0.80, -0.05, TABLE_TOP + 0.25])
    f0 = _w(t).get("fires", 0)                    # the counter is per server (earlier checks may have fired rules)
    await feed(t, offer, hold=30.0)
    ok = await wait_for(t, lambda s: (s.get("watch") or {}).get("fires", 0) >= f0 + 1, 6.0)
    task = t.state().get("task") or {}
    t.check("watch: open hand held out -> rule fires a handover", ok and task.get("action") == "handover" and str(task.get("id", "")).startswith("watch-R"),
            f"fires={_w(t).get('fires')} task={task.get('id')} {task.get('status')} {task.get('message', '')[:60]}")
    await t.pump(3.0)
    n1 = (_w(t).get("rules") or [{}])[0].get("fired")
    await feed(t, synth.frames("rest", 1.2) + offer, hold=30.0)    # hand down, then out again: inside the cooldown
    await t.pump(3.5)
    n2 = (_w(t).get("rules") or [{}])[0].get("fired")
    t.check("watch: the rule fires once (latched while the hand stays out, cooldown after)", n1 == 1 and n2 == 1, f"fired {n1} -> {n2}")
    await watch(t, op="rules_clear")
    await watch(t, op="mode", mode="off")
    await t.task("stop", 1)
    await t.task("reset_scene", 1)

    # ---- learn: a demonstration becomes a demo file (+ a replayable path)
    r = await watch(t, op="learn_start", name=DEMO)
    await feed(t, synth.frames("weave", 6.0, amp=0.015, freq=2.0), hold=0.5)
    await wait_for(t, lambda s: ((s.get("watch") or {}).get("learn") or {}).get("seconds", 0) >= 5.0, 9.0)
    r = await watch(t, op="learn_stop")
    d = r.get("demo") or {}
    path = Path(d.get("path") or "/nonexistent")
    data = json.loads(path.read_text()) if path.exists() else {}
    pat = data.get("pattern") or {}
    t.check("watch: learn -> recordings/demos/<name>.json with frames, tool, pattern, trajectory",
            r.get("ok") and path.exists() and len(data.get("frames") or []) > 40 and data.get("activity") == "weave"
            and abs(pat.get("freq", 0) - 2.0) < 0.4 and (data.get("trajectory") or {}).get("source") == "demo",
            f"{r.get('message')} pattern={pat} acts={''.join((f.get('activity') or '-')[:2] + ' ' for f in (data.get('frames') or [])[::8])}")
    r = await watch(t, op="replay", name=DEMO, task_id="watch-replay-check")
    await t.pump(1.0)
    pl, task = t.state().get("plan") or {}, t.state().get("task") or {}
    t.check("watch: replay proposes the demonstrated path (plan card)", r.get("ok") and task.get("id") == "watch-replay-check"
            and task.get("action") == "follow_path" and task.get("status") in ("proposed", "failed", "running", "done"),
            f"{r.get('message')} task={task.get('status')} plan={pl.get('phase')}")
    if pl.get("id") == "watch-replay-check":
        await t.send(type="plan", op="cancel", id="watch-replay-check")
    try:
        path.unlink()
    except OSError:
        pass
    await t.send(type="clear")
    await t.task("reset_scene", 1)

    # ---- assist: follow under the hands, help on a stall
    r = await watch(t, op="mode", mode="assist")
    await feed(t, synth.frames("squat", 9.0, stall_at=(2, 0.4), bar=True), hold=6.0)
    ok = await wait_for(t, lambda s: ((s.get("watch") or {}).get("assist") or {}).get("phase") == "helping", 14.0)
    a = _w(t).get("assist") or {}
    t.check("watch: assist follows the person and helps when they stall (force request for impedance control)",
            ok and (a.get("request") or {}).get("force_n", 0) > 0, f"phase={a.get('phase')} lift={a.get('lift')} request={a.get('request')}")
    await watch(t, op="mode", mode="off")
    await t.task("reset_scene", 1)

    # ---- mirror on an arm (Panda): the gripper follows the synthetic wrist, the fist closes it
    await t.send(type="world", op="set", robot="panda", id="watch-panda")
    ok = await wait_for(t, lambda s: ((s.get("world") or {}).get("robot") or {}).get("id") == "panda", 25.0, 0.4)
    if not ok:
        t.check("watch: mirror on the Panda", False, "the world didn't switch to the Panda")
        return
    anc = (t.state().get("copresence") or {}).get("anchor") or [1.15, 0.0]
    base = [anc[0] - 0.45, anc[1] - 0.05, TABLE_TOP + 0.30]
    await feed(t, synth.frames("hand", 0.3, at=base, open=True), hold=30.0)
    await t.pump(0.8)
    r = await watch(t, op="mode", mode="mirror")
    await t.pump(2.0)

    def ee():
        m = _w(t).get("mirror") or {}
        p = (m.get("pairs") or [{}])[0]
        return p.get("ee"), p.get("grip"), p.get("person")
    e1, g1, who = ee()
    moved = [base[0], base[1] + 0.15, base[2] + 0.05]      # the person's hand moves to the robot's left and up
    await feed(t, synth.frames("hand", 0.3, at=moved, open=True), hold=30.0)
    await t.pump(2.5)
    e2, g2, _ = ee()
    dy = (e2[1] - e1[1]) if (e1 and e2) else 0
    dz = (e2[2] - e1[2]) if (e1 and e2) else 0
    t.check("watch: mirror on the Panda moves the gripper with the person's wrist", r.get("ok") and who == "right" and dy > 0.08 and dz > 0.02,
            f"ee {e1} -> {e2} (dy {dy * 100:.1f} cm, dz {dz * 100:.1f} cm)")
    await feed(t, synth.frames("hand", 0.3, at=moved, open=False), hold=30.0)
    await t.pump(1.2)
    _, g3, _ = ee()
    t.check("watch: mirror: an open hand opens the gripper, a fist closes it", g2 == 0 and g3 == 1, f"grip open={g2} fist={g3}")
    await t.task("stop", 1)
    t.check("watch: 'stop' also stops mirroring", _w(t).get("mode") == "off", _w(t).get("mode"))
    await t.send(type="copresence_feed", frames=[])
    await t.send(type="world", op="reset", id="watch-reset")
    await wait_for(t, lambda s: ((s.get("world") or {}).get("robot") or {}).get("id") == "g1", 25.0, 0.4)


# ---------------------------------------------------------------------- standalone
def _main():
    import aiohttp
    port = int(os.environ.get("PC_WATCH_PORT", "8915"))
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
