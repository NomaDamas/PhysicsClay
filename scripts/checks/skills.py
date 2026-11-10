"""v1 P4 Skills from demonstration (#32) + demo packs (#35), over the real protocol with synthetic people (no camera):

* weld cell: the pack loads (UR5e + torch TCP, two seams); a synthetic hand weaves in `learn` mode; when learn stops the
  skill is extracted (width, frequency, travel speed within 15 %) and proposed on a seam with one sentence; "wider"
  edits it and re-proposes; applied to the other (L) seam it runs and leaves a bead that follows the seam with the weave;
* gym: the pack loads (G1, bench, 20 kg barbell); coach counts synthetic squats; the spot skill follows the bar (the
  lifter's hands) and helps when the bar stalls, with a force request for P0 impedance;
* workshop: the pack's standing rule "when I put my hand out, give me the wrench" fires a handover of the wrench;
* the agent's apply_skill / edit_skill_params / learn_skill_from_demo / load_pack tools send the right messages.

Standalone (starts its own --fake server on PC_SKILLS_PORT, default 8921):
    .venv/bin/python scripts/checks/skills.py
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

import numpy as np  # noqa: E402

from pps.features.copresence import to_world  # noqa: E402
from pps.human import periodic, synth  # noqa: E402
from pps.human.activity import ActivityTracker  # noqa: E402
from pps.human.model import ground  # noqa: E402

DEMO = "p4check_weave"
AMP, FREQ, TRAVEL = 0.005, 2.0, 0.02


def unit(check):
    # the weave false positive (PR #40): a still hand's MediaPipe jitter is not a weave; a welder's weave is
    def acts(frames):
        tr, seen = ActivityTracker(), set()
        for i, f in enumerate(frames):
            tr.update(i / 20, ground(to_world(f)), {"speed": 0.1})
            seen.add(tr.activity)
        return seen
    fp = [a for a in (0.01, 0.02, 0.03) for s in range(3) if "weave" in acts(synth.frames("jitter", 8, amp=a, seed=s))]
    ok_w = "weave" in acts(synth.frames("weave", 6, amp=AMP, freq=FREQ, travel=TRAVEL))
    check("skills(unit): a still, jittering hand is never a weave; a travelling 5 mm weave is", not fp and ok_w,
          f"jitter read as weave: {fp}; weld weave seen: {ok_w}")
    t = np.arange(0, 6, 1 / 30)
    a = periodic.analyze(t, np.stack([0.8 + 0.004 * np.sin(2 * np.pi * 2.5 * t), 0.1 - 0.006 * t, 0.98 + 0 * t], 1))
    check("skills(unit): periodic analysis reads width, frequency and travel", abs(a["amp"] - 0.004) < 0.0006
          and abs(a["freq"] - 2.5) < 0.05 and abs(a["speed"] - 0.006) < 0.0005 and periodic.is_weave(a)[0],
          f"amp {a['amp'] * 1000:.2f} mm, {a['freq']:.2f} Hz, {a['speed'] * 1000:.1f} mm/s")
    # agent tools
    from pps.agent import Agent, tool_specs
    st0 = {"world": {"rev": 1, "robot": {"id": "ur5e", "capabilities": ["pick_place", "reach", "follow_path"]}, "robots": []},
           "skillbook": {"skills": [{"name": "my_weave", "kind": "weave_seam", "describe": "5 mm zigzag at 2 Hz"}]}}
    specs = {t_["name"]: t_ for t_ in tool_specs(st0)}
    need = ("learn_skill_from_demo", "list_skills", "apply_skill", "edit_skill_params", "load_pack")
    check("skills(unit): agent tools learn_skill_from_demo / list_skills / apply_skill / edit_skill_params / load_pack",
          all(n in specs for n in need) and specs["load_pack"]["parameters"]["properties"]["name"]["enum"] == ["weld_cell", "gym", "workshop"],
          ",".join(n for n in need if n not in specs))
    sent, world = [], dict(st0)

    def push(m):                       # answers like pps/features/skills.py would
        sent.append(m)
        world["skillbook"] = dict(world["skillbook"], reply={"id": m.get("id"), "op": m.get("op"), "ok": True, "message": "I'll weld the L seam.",
                                                             "plan": "skill-apply-1", "skill": {"name": m.get("name")}})
        world["plan"] = {"id": "skill-apply-1", "phase": "proposing", "intent": "Move my right hand along the L seam"}
    ag = Agent(push=push, state=lambda: dict(world), server_cfg=lambda: None, poll=0.01)
    res = ag.execute("apply_skill", {"name": "my_weave", "target": "seam_l", "overrides": {"weave.amp": 0.006, "travel_speed": ""}})
    m = sent[-1] if sent else {}
    check("skills(unit): agent apply_skill -> {type: skill, op: apply, name, target, overrides, preview} and reads the plan",
          m.get("type") == "skill" and m.get("op") == "apply" and m.get("name") == "my_weave" and m.get("target") == "seam_l"
          and m.get("overrides") == {"weave.amp": 0.006} and m.get("preview") is True and m.get("source") == "agent"
          and res.get("ok") and (res.get("plan") or {}).get("phase") == "proposing", f"{m} -> {res}")
    ag._proposed = None
    ag.execute("edit_skill_params", {"text": "a bit wider", "changes": {"travel_speed": 0.004}})
    m2 = sent[-1]
    ag.execute("learn_skill_from_demo", {"kind": "weave_seam", "target": "the L seam"})
    m3 = sent[-1]
    check("skills(unit): agent edit_skill_params / learn_skill_from_demo messages",
          m2.get("op") == "edit" and m2.get("text") == "a bit wider" and m2.get("changes") == {"travel_speed": 0.004}
          and m3.get("op") == "extract" and m3.get("kind") == "weave_seam" and m3.get("target") == "the L seam", f"{m2} | {m3}")
    prompt = (ROOT / "pps" / "world" / "prompts" / "base.md").read_text().lower()
    check("skills(unit): base prompt teaches skills (confirm in one sentence, corrections edit parameters)",
          "geometry gives precision" in prompt and "edit_skill_params" in prompt and "right?" in prompt)


# ---------------------------------------------------------------------------- end to end
def _sb(t):
    return t.state().get("skillbook") or {}


def _w(t):
    return t.state().get("watch") or {}


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


async def skill(t, **kw):
    rid = f"sk{time.time():.3f}"
    await t.send(type="skill", id=rid, **kw)
    ok = await wait_for(t, lambda s: ((s.get("skillbook") or {}).get("reply") or {}).get("id") == rid, 8.0, 0.15)
    return (_sb(t).get("reply") or {}) if ok else {}


async def watch(t, **kw):
    rid = f"wt{time.time():.3f}"
    await t.send(type="watch", id=rid, **kw)
    await wait_for(t, lambda s: ((s.get("watch") or {}).get("reply") or {}).get("id") == rid, 4.0, 0.15)
    return _w(t).get("reply") or {}


async def load_pack(t, name):
    rev = (t.state().get("world") or {}).get("rev")
    await t.send(type="world", op="pack", name=name, id=f"pack-{name}")
    ok = await wait_for(t, lambda s: (s.get("world") or {}).get("rev") != rev, 30.0, 0.3)
    await t.pump(0.8)
    return ok, t.state().get("world") or {}


async def run(t):
    unit(t.check)
    await t.send(type="speed", scale=1.0)
    # ---------------------------------------------------------------- weld cell
    ok, w = await load_pack(t, "weld cell")
    sc = w.get("scene") or {}
    t.check("skills: weld cell pack loads (UR5e, torch on the arm with its tip as the TCP, straight + L seams)",
            ok and (w.get("robot") or {}).get("id") in ("ur5e", "panda") and ((w.get("robot") or {}).get("tool") or {}).get("kind") == "torch"
            and set(sc.get("seams") or {}) == {"seam_a", "seam_l"} and w.get("pack") == "weld_cell" and (_sb(t).get("pack") or {}).get("id") == "weld_cell",
            f"robot={(w.get('robot') or {}).get('id')} tool={(w.get('robot') or {}).get('tool')} seams={list(sc.get('seams') or {})}")
    await watch(t, op="learn_start", name=DEMO)
    await t.send(type="copresence_feed", frames=synth.frames("weave", 6.0, amp=AMP, freq=FREQ, travel=TRAVEL), fps=20, hold=0.5)
    await wait_for(t, lambda s: ((s.get("watch") or {}).get("learn") or {}).get("seconds", 0) >= 5.6, 10.0)
    await watch(t, op="learn_stop")
    got = await wait_for(t, lambda s: ((s.get("skillbook") or {}).get("reply") or {}).get("demo") == DEMO, 8.0)
    r = _sb(t).get("reply") or {}
    from pps import skills as SK
    sk = SK.load(DEMO) or {"params": {"weave": {}}}
    p = sk["params"]
    e = (abs(p["weave"].get("amp", 0) / AMP - 1), abs(p["weave"].get("freq", 0) / FREQ - 1), abs(p.get("travel_speed", 0) / TRAVEL - 1))
    t.check("skills: learn stop -> weave_seam skill extracted (width, frequency, travel speed within 15%)",
            got and sk.get("kind") == "weave_seam" and max(e) <= 0.15,
            f"amp {p['weave'].get('amp', 0) * 1000:.2f} mm, {p['weave'].get('freq', 0):.2f} Hz, {p.get('travel_speed', 0) * 1000:.1f} mm/s "
            f"(errors {', '.join(f'{x * 100:.0f}%' for x in e)}); measured {sk.get('measured', {}).get('smoothing_gain')} gain")
    pl = t.state().get("plan") or {}
    t.check("skills: ... and proposed on a seam with one sentence to confirm (plan card, nothing moves)",
            r.get("proposed") and pl.get("phase") == "proposing" and (pl.get("skill") or {}).get("name") == DEMO
            and "Right?" in (pl.get("say") or "") and "mm zigzag" in (pl.get("say") or "") and not (t.state().get("traj") or {}).get("running"),
            (pl.get("say") or r.get("message") or "")[:140])
    amp0 = p["weave"].get("amp", 0)
    r = await skill(t, op="edit", text="wider")
    pl2 = t.state().get("plan") or {}
    sk2 = SK.load(DEMO) or {"params": {"weave": {}}}
    t.check("skills: 'wider' edits the weave width, recompiles and re-proposes",
            r.get("ok") and abs(sk2["params"]["weave"].get("amp", 0) - round(amp0 * 1.25, 4)) < 2e-4 and pl2.get("phase") == "proposing"
            and pl2.get("id") == pl.get("id") and (t.state().get("traj") or {}).get("proposed"),
            f"{r.get('message', '')[:120]}")
    if pl2.get("id"):
        await t.send(type="plan", op="cancel", id=pl2["id"])
        await t.pump(0.4)
    r = await skill(t, op="apply", name=DEMO, target="seam_l", preview=False)
    ran = await wait_for(t, lambda s: (s.get("traj") or {}).get("running"), 6.0)
    done = await wait_for(t, lambda s: not (s.get("traj") or {}).get("running"), 40.0, 0.4)
    beads = [b for b in _sb(t).get("beads") or [] if b.get("seam") == "seam_l"]
    pts = np.array(beads[-1]["pts"]) if beads else np.zeros((0, 3))
    from pps.skills import geom
    poly = np.array(((t.state().get("world") or {}).get("scene") or {}).get("seams", {}).get("seam_l", {}).get("points") or [[0, 0, 0], [0, 0, 0.1]])
    dev = [geom.dist_to_polyline(q, poly)[0] for q in pts] if len(pts) else [1.0]
    cover = [geom.dist_to_polyline(poly[k], pts)[0] for k in (0, len(poly) - 1)] if len(pts) > 1 else [1.0]
    task = t.state().get("task") or {}
    amp1 = sk2["params"]["weave"].get("amp", 0.006)
    t.check("skills: applied to the other (L) seam it runs and leaves a bead along the seam, weave included (no arc physics)",
            r.get("ok") and ran and done and task.get("status") == "done" and len(pts) >= 40 and max(dev) <= amp1 + 0.006
            and max(dev) >= 0.6 * amp1 and max(cover) <= 0.03,
            f"{r.get('message', '')[:70]} | bead {len(pts)} pts, max off-seam {max(dev) * 1000:.1f} mm (weave {amp1 * 1000:.1f} mm), "
            f"ends {[round(c * 100, 1) for c in cover]} cm | {task.get('message', '')[:60]}")
    # ---------------------------------------------------------------- gym
    ok, w = await load_pack(t, "gym")
    names = sorted(o["name"] for o in (w.get("scene") or {}).get("objects") or [])
    t.check("skills: gym pack loads (G1, bench, 20 kg barbell, gym floor)", ok and (w.get("robot") or {}).get("id") in ("g1", "apollo")
            and names == ["barbell", "bench"] and (w.get("scene") or {}).get("template") == "gym", f"{(w.get('robot') or {}).get('id')} {names}")
    await watch(t, op="mode", mode="coach")
    await t.send(type="copresence_feed", frames=synth.frames("squat", reps=4), fps=20, hold=2.0)
    ok = await wait_for(t, lambda s: (((s.get("watch") or {}).get("coach") or {}).get("reps") or 0) >= 3, 14.0)
    t.check("skills: gym: the coach counts synthetic squats", ok, f"reps={(_w(t).get('coach') or {}).get('reps')}")
    await watch(t, op="mode", mode="off")
    r = await skill(t, op="apply", name="spot_bench", preview=False)
    await t.send(type="copresence_feed", frames=synth.frames("squat", 9.0, stall_at=(2, 0.4), bar=True), fps=20, hold=6.0)
    held = await wait_for(t, lambda s: ((s.get("skillbook") or {}).get("bar") or {}).get("held"), 6.0)
    ok = await wait_for(t, lambda s: ((s.get("watch") or {}).get("assist") or {}).get("phase") == "helping", 14.0)
    a = _w(t).get("assist") or {}
    t.check("skills: gym: the spot skill follows the bar under the hands and helps when it stalls (force request for P0 impedance)",
            r.get("ok") and _w(t).get("mode") == "assist" and held and ok and (a.get("request") or {}).get("force_n") == 40.0,
            f"{r.get('message', '')[:80]} | bar held={held} phase={a.get('phase')} bar={a.get('bar')} request={a.get('request')}")
    await watch(t, op="mode", mode="off")
    await t.send(type="copresence_feed", frames=[])
    # ---------------------------------------------------------------- workshop
    ok, w = await load_pack(t, "workshop")
    await t.pump(0.6)
    rules = _w(t).get("rules") or []
    t.check("skills: workshop pack loads with the rule 'when I put my hand out, give me the wrench'",
            ok and any(x.get("when") == "hand_open_near_robot" and (x.get("task") or {}).get("object") == "wrench" for x in rules)
            and _w(t).get("mode") == "respond", f"rules={[(x.get('when'), x.get('do')) for x in rules]} mode={_w(t).get('mode')}")
    f0 = _w(t).get("fires", 0)
    await t.send(type="copresence_feed", frames=synth.frames("offer", 0.5, target=[0.80, -0.05, 0.99]), fps=20, hold=30.0)
    ok = await wait_for(t, lambda s: (s.get("watch") or {}).get("fires", 0) >= f0 + 1, 8.0)
    task = t.state().get("task") or {}
    t.check("skills: workshop: an open hand held out fires the handover of the wrench", ok and task.get("action") == "handover"
            and "wrench" in (task.get("message") or ""), f"fires={_w(t).get('fires')} task={task.get('status')} {task.get('message', '')[:70]}")
    await t.task("stop", 1)
    await t.send(type="copresence_feed", frames=[])
    # ---------------------------------------------------------------- back to the default world
    rev = (t.state().get("world") or {}).get("rev")
    await t.send(type="world", op="reset", id="p4-reset")
    await wait_for(t, lambda s: (s.get("world") or {}).get("rev") != rev, 30.0, 0.4)
    await t.pump(0.8)
    t.check("skills: leaving the workshop removes its standing rule; the default world is back",
            not any(x.get("when") == "hand_open_near_robot" for x in _w(t).get("rules") or [])
            and ((t.state().get("world") or {}).get("robot") or {}).get("id") == "g1" and not (t.state().get("world") or {}).get("pack"),
            f"rules={[x.get('when') for x in _w(t).get('rules') or []]}")
    await watch(t, op="mode", mode="off")
    for f in list((ROOT / "skills").glob("p4check_*.json")) + list((ROOT / "recordings" / "demos").glob("p4check_*.json")):
        try:
            f.unlink()
        except OSError:
            pass


# ---------------------------------------------------------------------- standalone
def _main():
    import aiohttp
    port = int(os.environ.get("PC_SKILLS_PORT", "8921"))
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

        t = SimpleNamespace(send=lambda **k: ws.send_str(json.dumps(k)), pump=pump, task=task, check=check, state=lambda: st, http=s,
                            base=base, frames=lambda: {})
        try:
            await pump(1.0)
            await run(t)
        finally:
            await ws.close()
            await s.close()

    srv = None
    if not os.environ.get("PC_SKILLS_EXTERNAL"):
        srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--port", str(port),
                                "--https-port", str(port + 1)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        asyncio.run(go())
    finally:
        if srv is not None:
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
