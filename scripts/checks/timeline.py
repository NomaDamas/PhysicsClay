"""Teach timeline (#F): card edits (reorder / delete / chip toggle via teach set_spec) change what the robot practises,
"show again" re-records one step in place, and a saved skill card plays."""
import time
from pathlib import Path

SKILLS = Path(__file__).resolve().parents[2] / "recordings" / "skills"


async def run(t):
    T = lambda **k: t.send(type="teach", **k)
    tc = lambda: t.state()["teach"]
    steps_of = lambda: [(s["object"], s["target"]) for s in (tc()["spec"] or {}).get("steps", [])]

    async def until(cond, limit=10):
        """Pump until cond() holds (state frames can lag on a loaded machine)."""
        t0 = time.time()
        while time.time() - t0 < limit:
            await t.pump(0.2)
            try:
                if cond():
                    return True
            except (KeyError, TypeError):
                pass
        return False

    async def practise(limit=70):
        """Run practice; return the order in which steps were run (object, target)."""
        order = []
        await T(cmd="practice")
        t0 = time.time()
        while time.time() - t0 < limit:
            await t.pump(0.25)
            spec = tc()["spec"] or {}
            for s in spec.get("steps", []):
                k = (s["object"], s["target"])
                if s["status"] in ("running", "done", "failed") and k not in order:
                    order.append(k)
            if tc()["phase"] != "practice" and time.time() - t0 > 1:
                break
        return order

    # teach two steps
    await T(cmd="start"); await until(lambda: tc()["phase"] == "recording")
    for u in ["pick up the mug and put it in the middle", "then put the red block on the right"]:
        await T(cmd="utterance", text=u); await t.pump(0.2)
    await T(cmd="stop"); await until(lambda: tc()["phase"] == "confirm" and tc()["spec"])
    base = steps_of()
    t.check("timeline: two step cards", base == [("mug", "center"), ("red_block", "right")], str(base))
    if len(base) != 2:
        return
    spec = tc()["spec"]

    # reorder (what the card drag / move buttons send) -> the practised sequence follows the new order
    rev = [{"object": o, "target": g} for o, g in reversed(base)]
    await T(cmd="set_spec", spec={"goal": spec["goal"], "steps": rev, "constraints": spec["constraints"]}); await until(lambda: steps_of() == base[::-1])
    t.check("timeline: reorder -> spec order", steps_of() == base[::-1] and tc()["phase"] == "confirm", str(steps_of()))
    order = await practise()
    t.check("timeline: reorder changes the practised sequence", order == base[::-1] and tc()["phase"] == "done", f"{tc()['phase']}: {order}")

    # delete one card -> only the remaining step is practised
    await T(cmd="set_spec", spec={"goal": spec["goal"], "steps": [{"object": "mug", "target": "center"}], "constraints": []}); await until(lambda: len(steps_of()) == 1 and tc()["phase"] == "confirm")
    order = await practise()
    t.check("timeline: delete changes the practised sequence", order == [("mug", "center")] and tc()["phase"] == "done", f"{tc()['phase']}: {order}")

    # constraint chip toggles on and off through set_spec
    await T(cmd="set_spec", spec={"goal": "g", "steps": [{"object": "mug", "target": "center"}], "constraints": [{"type": "slow"}]}); await until(lambda: tc()["spec"]["constraints"])
    on = [c["type"] for c in tc()["spec"]["constraints"]]
    await T(cmd="set_spec", spec={"goal": "g", "steps": [{"object": "mug", "target": "center"}], "constraints": []})
    await until(lambda: not tc()["spec"]["constraints"])
    t.check("timeline: constraint chip toggles", on == ["slow"] and tc()["spec"]["constraints"] == [], str(on))

    # show again: re-record step 2 only; step 1 and the constraints stay
    await T(cmd="set_spec", spec={"goal": "g", "steps": [{"object": "mug", "target": "center"}, {"object": "red_block", "target": "right"}],
                                  "constraints": [{"type": "avoid", "object": "vase"}]}); await until(lambda: len(steps_of()) == 2)
    await t.send(type="timeline", op="reshow", index=1); await until(lambda: (t.state().get("timeline") or {}).get("reshow"))
    r = (t.state().get("timeline") or {}).get("reshow")
    t.check("timeline: show again records one step", tc()["phase"] == "recording" and r and r["index"] == 1 and len(r["steps"]) == 2, str(r)[:80])
    await T(cmd="utterance", text="put the red block on the left, slowly"); await t.pump(0.2)
    await T(cmd="stop"); await until(lambda: tc()["phase"] == "confirm" and tc()["spec"])
    cons = sorted(c["type"] for c in tc()["spec"]["constraints"])
    t.check("timeline: show again replaces just that step", steps_of() == [("mug", "center"), ("red_block", "left")] and cons == ["avoid", "slow"]
            and not (t.state().get("timeline") or {}).get("reshow"), f"{steps_of()} {cons}")
    await t.send(type="timeline", op="reshow", index=0); await until(lambda: tc()["phase"] == "recording")
    await t.send(type="timeline", op="reshow_cancel"); await until(lambda: tc()["phase"] == "confirm")
    t.check("timeline: cancel show again keeps the old steps", tc()["phase"] == "confirm" and steps_of() == [("mug", "center"), ("red_block", "left")], str(steps_of()))

    # a saved skill card plays (the card sends the existing task play_skill)
    await T(cmd="set_spec", spec={"goal": "g", "steps": [{"object": "mug", "target": "center"}], "constraints": []}); await until(lambda: len(steps_of()) == 1)
    await T(cmd="save", name="timeline_check"); await until(lambda: "timeline_check" in t.state().get("skills", []))
    t.check("timeline: saved task shows as a skill card", "timeline_check" in t.state().get("skills", []))
    task = await t.task("play_skill", 4, name="timeline_check"); await until(lambda: tc()["phase"] == "practice", 4)
    t.check("timeline: skill card plays", tc()["phase"] == "practice" and "timeline_check" in task.get("message", ""), task.get("message", "")[:60])
    await t.until_done(30)
    await T(cmd="cancel"); await t.pump(0.2)
    try:
        (SKILLS / "timeline_check.json").unlink()
    except OSError:
        pass
