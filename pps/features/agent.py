"""Brain (UX 3): wires pps/agent.py into the server.

{type:"agent", text, llm?, confirm?} starts a turn (background thread), {type:"agent", op:"reset"} clears the
conversation. The agent reads the latest published state frame (refreshed here, in the worker thread, every frame) and
acts by pushing browser-style messages (ctx.push = shared.push_cmd). Its replies go out through the existing robot line
(ctx.brain.utter -> state.say) so the feed and TTS treat them like any robot speech. State key: `agent`."""
import json

from .. import ext
from ..agent import Agent

KEY = "agent"


class _Snapshot:
    """Latest state for the agent thread. The worker stores the published JSON string (an immutable reference, so the
    swap is atomic); the agent thread parses it on demand, at most once per frame."""

    def __init__(self):
        self.raw, self._parsed, self._parsed_of = None, {}, None
        self.fallback = {}

    def get(self):
        raw = self.raw
        if raw is None:
            return dict(self.fallback)
        if raw is not self._parsed_of:
            try:
                self._parsed = json.loads(raw)
            except ValueError:
                self._parsed = {}
            self._parsed_of = raw
        return self._parsed


def _get(ctx):
    a = ctx.store.get(KEY)
    if a is None:
        snap = _Snapshot()
        a = Agent(push=ctx.push, state=snap.get)
        a.snap = snap
        ctx.store[KEY] = a
    return a


@ext.handler("agent")
def on_agent(ctx, msg):
    a = _get(ctx)
    if msg.get("op") == "reset":
        a.reset()
        return
    a.submit(msg.get("text"), llm=msg.get("llm"), confirm=msg.get("confirm", True) is not False)


@ext.tick
def agent_tick(ctx):
    a = _get(ctx)
    # a real LLM brain is in charge: the scripted mock must not narrate over it (app.py reads this flag)
    ctx.store["llm_brain"] = bool(a.available()) or bool(getattr(a, "busy", False))
    shared = getattr(ctx.push, "__self__", None)            # ctx.push is shared.push_cmd: read what browsers get
    raw = getattr(shared, "state_en", None) or getattr(shared, "state", None)   # English: pps/i18n.py localizes
    if isinstance(raw, str) and raw not in ("", "{}"):
        a.snap.raw = raw
    else:                                                   # no published frame yet: a minimal snapshot from ctx
        t = ctx.tasks
        a.snap.fallback = {"task": dict(t.task) if t.task else None, "robot": "g1",
                           "markers": [{"id": m["id"], "pos": [float(x) for x in m["pos"]]} for m in t.markers],
                           "scene_objects": [{"name": n.replace("_", " "), "pos": [float(x) for x in t.mug_center(n)]} for n in t.objs]}
    a.watch(ctx.tasks.task, speak=not ctx.brain.enabled)      # the mock brain already narrates task results
    for line in a.drain():
        ctx.brain.utter(line, "robot")


@ext.state
def agent_state(ctx):
    return {KEY: _get(ctx).state()}
