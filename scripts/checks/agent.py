"""Brain (UX 3): the LLM agent.

1. Pure unit test of the tool loop with a fake LLM and a fake world (no server, no network): tool call -> push the
   browser message -> wait on state -> function_call_output back to the model -> reply spoken; the plan guard, reset,
   LLM errors, the step limit, and that a key never shows up in state.
2. Real LLM (skipped cleanly when no LLM is configured in .env): "put the mug in the center" -> the agent calls
   pick_place -> a plan is proposed (or the task completes); "what's on the table?" -> observe/context + a reply that
   names objects; reset clears the conversation.
Run alone: .venv/bin/python scripts/checks/agent.py   (unit test only)"""
import asyncio
import json
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


# ---------------------------------------------------------------------------- unit test (fake LLM, fake world)
class FakeWorld:
    """Answers the browser messages the agent sends the way the server would, enough for the loop."""

    def __init__(self):
        self.lock = threading.Lock()
        self.st = {"robot": "g1", "task": None, "plan": None, "markers": [], "zones": [], "regions": ["left", "right", "center"],
                   "scene_objects": [{"name": "mug", "pos": [0.34, 0.1, 0.78]}, {"name": "red block", "pos": [0.3, -0.2, 0.76]}]}
        self.sent = []

    def push(self, m):
        self.sent.append(m)
        with self.lock:
            st = dict(self.st)
            if m.get("type") == "task" and m.get("action") in ("pick_place", "reach"):
                if m.get("preview"):
                    st["task"] = {"id": m["id"], "action": m["action"], "status": "proposed", "message": "Pick up the mug. Say go."}
                    st["plan"] = {"id": m["id"], "phase": "proposing", "ok": True, "intent": "Pick up the mug and put it on the center.",
                                  "verdict": "I can do this", "steps": [{"text": "Reach"}, {"text": "Grasp"}]}
                else:
                    st["task"] = {"id": m["id"], "action": m["action"], "status": "done", "message": "Placed mug on the center."}
            elif m.get("type") == "task":
                st["task"] = {"id": m["id"], "action": m["action"], "status": "done", "message": f"{m['action']} ok"}
            elif m.get("type") == "zone" and m.get("op") == "add":
                st["zones"] = st["zones"] + [{"id": "A", "label": f"the {m.get('near')} keep-out zone"}]
            self.st = st

    def state(self):
        with self.lock:
            return dict(self.st)


def fake_llm(script, seen):
    """script: list of output-item lists, one per call. seen: gets (instructions, input, tools) of every call."""
    def llm(cfg, instructions, items, tools):
        seen.append((instructions, [dict(i) for i in items], tools))
        if not script:
            return [{"type": "message", "content": [{"type": "output_text", "text": "(out of script)"}]}]
        out = script.pop(0)
        if isinstance(out, Exception):
            raise out
        return out
    return llm


def call(name, args, cid):
    return {"type": "function_call", "call_id": cid, "name": name, "arguments": json.dumps(args)}


def say(text):
    return {"type": "message", "role": "assistant", "content": [{"type": "output_text", "text": text}]}


def wait_idle(ag, limit=5.0):
    t0 = time.time()
    while ag.busy and time.time() - t0 < limit:
        time.sleep(0.01)
    return not ag.busy


def unit(check):
    from pps.agent import Agent, LLMError, MAX_STEPS, tool_specs
    cfg = {"base_url": "https://example.invalid/v1", "key": "sk-SECRET-never-shown", "model": "fake-model", "effort": "medium"}

    # tools follow capabilities; world tools exist but say "not available" without state.world
    names = {t["name"] for t in tool_specs({"robot": "g1"})}
    check("agent unit: tools from G1 capabilities", {"observe", "pick_place", "reach", "follow_path", "go_home", "stop", "set_speed", "add_zone",
                                                    "clear_zones", "set_goal_layout", "choose_candidate", "confirm_plan", "cancel_plan",
                                                    "set_robot", "build_scene", "add_object", "remove_object", "start_teaching"} <= names, f"{len(names)} tools")
    arm = {t["name"] for t in tool_specs({"world": {"rev": 1, "robot": {"id": "panda", "capabilities": ["pick_place", "reach"]}, "robots": []}})}
    check("agent unit: an arm gets no humanoid tools", "start_teaching" not in arm and "follow_me" not in arm and "pick_place" in arm, "")

    # 1. one tool call (preview) -> output fed back -> reply spoken
    w, seen = FakeWorld(), []
    ag = Agent(push=w.push, state=w.state, llm=fake_llm([[call("pick_place", {"object": "mug", "target": "center", "marker": "", "xyz": [0, 0, 0]}, "c1")],
                                                          [say("Here's my plan: mug to the center. Say go.")]], seen),
               server_cfg=lambda: cfg, poll=0.01)
    ok = ag.submit("put the mug in the center", confirm=True)
    wait_idle(ag)
    kinds = [e["kind"] for e in ag.events]
    tool = next((e for e in ag.events if e["kind"] == "tool"), {})
    sent = next((m for m in w.sent if m.get("type") == "task"), {})
    fco = [i for i in seen[1][1] if i.get("type") == "function_call_output"] if len(seen) > 1 else []
    check("agent unit: tool loop (call -> browser message -> result -> reply)",
          ok and kinds == ["user", "thinking", "tool", "reply"] and tool.get("ok") is True and sent.get("preview") is True
          and sent.get("object") == "mug" and sent.get("target") == "center" and not any(m.get("type") == "point3d" for m in w.sent) and fco and json.loads(fco[0]["output"])["status"] == "proposed" and ag.drain() == ["Here's my plan: mug to the center. Say go."],
          f"{kinds} sent={sent.get('action')} preview={sent.get('preview')}")
    check("agent unit: the system prompt carries the world context", "Right now" in seen[0][0] and "red block" in seen[0][0], "")

    # 2. a second motion call in the same turn is refused while the plan waits; memory carries to the next turn
    w2, seen2 = FakeWorld(), []
    ag2 = Agent(push=w2.push, state=w2.state, llm=fake_llm([[call("pick_place", {"object": "mug", "target": "left"}, "a")],
                                                            [call("reach", {"target": "right"}, "b")], [say("Plan ready, say go.")],
                                                            [say("You asked me to move the mug.")]], seen2),
                server_cfg=lambda: cfg, poll=0.01)
    ag2.submit("move the mug left then reach right")
    wait_idle(ag2)
    second = [e for e in ag2.events if e["kind"] == "tool"][1]
    ag2.submit("what did I ask?")
    wait_idle(ag2)
    hist = seen2[-1][1]
    check("agent unit: one plan per turn while it waits for go", second.get("ok") is False and "already waiting" in second.get("text", ""), second.get("text", "")[:60])
    check("agent unit: conversation memory", any(i.get("role") == "user" and "move the mug" in str(i.get("content")) for i in hist)
          and any(i.get("type") == "function_call" for i in hist), f"{len(hist)} items")

    # 3. confirm off -> runs directly; zone tool; LLM error; step limit; reset; key never in state
    w3, seen3 = FakeWorld(), []
    ag3 = Agent(push=w3.push, state=w3.state, llm=fake_llm([[call("add_zone", {"near": "vase"}, "z"), call("pick_place", {"object": "mug", "target": "right"}, "p")],
                                                            [say("Done.")], LLMError("HTTP 500")] + [[call("observe", {}, f"o{k}")] for k in range(MAX_STEPS)], seen3),
                server_cfg=lambda: cfg, poll=0.01)
    ag3.submit("stay away from the vase and put the mug right", confirm=False)
    wait_idle(ag3)
    direct = next(m for m in w3.sent if m.get("type") == "task")
    zone_ev = next(e for e in ag3.events if e.get("tool") == "add_zone")
    check("agent unit: confirm off runs directly; zone tool", "preview" not in direct and zone_ev.get("ok") is True
          and any(m.get("type") == "zone" and m.get("near") == "vase" for m in w3.sent), zone_ev.get("text", ""))
    ag3.drain()
    ag3.submit("again")
    wait_idle(ag3)
    check("agent unit: an LLM error is reported, not fatal", ag3.events[-1]["kind"] == "error" and not ag3.busy and ag3.drain(), ag3.events[-1]["text"][:60])
    ag3.submit("loop forever")
    wait_idle(ag3)
    n_obs = sum(1 for e in ag3.events if e.get("tool") == "observe")
    check("agent unit: step limit", n_obs == MAX_STEPS and ag3.events[-1]["kind"] == "reply", f"{n_obs} observe calls")
    blob = json.dumps([ag.state(), ag2.state(), ag3.state()])
    check("agent unit: the key never reaches state", "SECRET" not in blob, "")
    ag3.reset()
    check("agent unit: reset clears", len(ag3.events) == 1 and ag3.history == [] and not ag3.busy, str(list(ag3.events))[:60])

    # BYOK: the server key is never used; visitor endpoints must be public https (no SSRF into this machine)
    import os
    from pps.agent import public_url_problem, server_llm, visitor_llm
    old = os.environ.get("PC_BYOK")
    os.environ["PC_BYOK"] = "1"
    try:
        no_server_key = server_llm() is None
    finally:
        if old is None:
            os.environ.pop("PC_BYOK", None)
        else:
            os.environ["PC_BYOK"] = old
    bad = [public_url_problem(u) for u in ("http://example.com/v1", "https://127.0.0.1/v1", "https://localhost:8777/v1", "https://10.0.0.5/v1")]
    vc, _ = visitor_llm({"base_url": "https://1.1.1.1/v1", "key": "sk-x", "model": "m", "effort": "bogus"})
    check("agent unit: BYOK never uses the server key; visitor URL must be public https", no_server_key and all(bad)
          and vc and vc["effort"] is None and vc["key"] == "sk-x", str(bad[:2]))

    # 4. no LLM -> clear error, nothing runs; outcome of a confirmed plan is reported later
    ag4 = Agent(push=w.push, state=w.state, llm=fake_llm([], []), server_cfg=lambda: None, poll=0.01)
    check("agent unit: no LLM configured -> error event", ag4.submit("hi") is False and ag4.events[-1]["kind"] == "error" and not ag4.state()["available"], "")
    pid = sent.get("id")
    ag.watch({"id": pid, "action": "pick_place", "status": "done", "message": "Placed mug on the center."})
    check("agent unit: a confirmed plan's result is reported and remembered", ag.events[-1]["kind"] == "result" and ag.drain() == ["Placed mug on the center."]
          and ag.notes, str(ag.events[-1])[:80])


# ---------------------------------------------------------------------------- real LLM over the protocol
async def _turn(t, text, limit=100):
    await t.send(type="agent", text=text, confirm=True)
    t0, started = time.time(), False
    while time.time() - t0 < limit:
        await t.pump(0.3)
        a = t.state().get("agent") or {}
        if a.get("busy"):
            started = True
        elif started or any(e.get("kind") == "error" for e in a.get("events") or []):
            break
    return t.state().get("agent") or {}


async def run(t):
    unit(t.check)
    from pps.agent import server_llm
    if server_llm() is None:
        t.check("agent: real LLM checks skipped (no LLM_BASE_URL / LLM_API_KEY)", True, "skipped")
        return
    await t.task("reset_scene", 1)
    await t.send(type="agent", op="reset")
    await t.pump(0.6)
    a = t.state().get("agent") or {}
    t.check("agent: state.agent available with the server model", a.get("available") is True and a.get("model") and not a.get("busy"), str(a.get("model")))

    t0 = time.time()
    a = await _turn(t, "put the mug in the center")
    st = t.state()
    evs = a.get("events") or []
    tools = [e for e in evs if e.get("kind") == "tool"]
    reply = next((e["text"] for e in reversed(evs) if e.get("kind") == "reply"), "")
    plan, task = st.get("plan") or {}, st.get("task") or {}
    ok = any(e.get("tool") == "pick_place" and e.get("ok") for e in tools) and (plan.get("phase") == "proposing" or task.get("status") in ("running", "done"))
    t.check("agent: 'put the mug in the center' -> pick_place -> plan proposed", ok,
            f"{time.time() - t0:.0f}s tools={[e.get('tool') for e in tools]} plan={plan.get('phase')} reply={reply[:70]!r}")
    print("      transcript:", json.dumps([{k: e.get(k) for k in ("kind", "tool", "args", "ok", "text") if e.get(k) is not None} for e in evs], ensure_ascii=False)[:1500], flush=True)
    t.check("agent: reply spoken through state.say", bool(reply) and (st.get("say") or {}).get("text") == reply, str((st.get("say") or {}).get("text"))[:60])
    if plan.get("phase") == "proposing":
        await t.send(type="plan", op="cancel", id=plan.get("id"))
        await t.pump(0.8)

    t0 = time.time()
    a = await _turn(t, "what's on the table?")
    evs = a.get("events") or []
    last_user = max([k for k, e in enumerate(evs) if e.get("kind") == "user"] or [0])
    evs = evs[last_user:]                     # this turn only
    reply = next((e["text"] for e in reversed(evs) if e.get("kind") == "reply"), "")
    low = reply.lower()
    t.check("agent: \"what's on the table?\" -> reply names the objects", any(w in low for w in ("mug", "block", "vase")) and not any(
        e.get("tool") in ("pick_place", "reach") for e in evs), f"{time.time() - t0:.0f}s {reply[:90]!r}")
    print("      transcript:", json.dumps([{k: e.get(k) for k in ("kind", "tool", "ok", "text") if e.get(k) is not None} for e in evs], ensure_ascii=False)[:900], flush=True)

    await t.send(type="agent", op="reset")
    await t.pump(0.6)
    a = t.state().get("agent") or {}
    t.check("agent: reset clears the conversation", len(a.get("events") or []) <= 1 and not a.get("busy"), f"{len(a.get('events') or [])} events")


if __name__ == "__main__":
    res = []
    unit(lambda name, ok, detail="": (res.append(bool(ok)), print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")))
    print(f"\n{sum(res)}/{len(res)} passed")
    sys.exit(0 if all(res) else 1)
