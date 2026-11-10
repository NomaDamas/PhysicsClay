"""The scripted demo and the robot line (state.say): a demo line never covers a real one, the demo stops when the person
or the agent speaks, and nothing of it keeps running afterwards (pps/demo.py, pps/mock.py, pps/features/agent.py)."""

from types import SimpleNamespace

from pps.demo import Demo
from pps.mock import MockBrain


def _demo(brain=None):
    brain = brain or MockBrain()
    tasks = SimpleNamespace(task=None, home=lambda *a, **k: None)
    return Demo(tasks, SimpleNamespace(phase="idle"), brain, SimpleNamespace(path_msg=None)), brain


def _run(demo, steps):
    demo.steps, demo.i, demo.name, demo.t_wait, demo.cond = steps, 0, "test", 0.0, None


def test_demo_lines_are_marked_and_dont_count_as_real():
    demo, brain = _demo()
    _run(demo, [demo.cap("① Video → Robot")])
    demo.tick()
    assert brain.say["text"] == "① Video → Robot" and brain.say["kind"] == "caption"
    assert not brain.real_fresh()


def test_a_demo_line_never_covers_a_fresh_real_line():
    demo, brain = _demo()
    _run(demo, [demo.cap("first caption"), demo.cap("second caption")])
    demo.tick()
    brain.utter("I'll put the mug beside the vase.", "robot")  # e.g. the agent's reply
    demo.t_wait = 0.0  # (its own spacing is over)
    demo.tick()
    assert brain.say["text"] == "I'll put the mug beside the vase."  # the demo waited
    assert demo.steps and demo.i == 1
    brain.real_t -= 10.0  # the real line has been up long enough
    demo.tick()
    assert brain.say["text"] == "second caption"


def test_demo_turns_the_mock_brain_on_and_restores_it():
    demo, brain = _demo()
    assert brain.enabled is False
    demo.start("keyfeatures")
    assert demo.steps and brain.enabled is True
    demo.stop()
    assert not demo.steps and demo.state() is None and brain.enabled is False
    brain.enabled = True  # on before the demo: stays on
    demo.start("loop")
    demo.stop()
    assert brain.enabled is True and demo.loop_name is None


def test_a_finished_demo_restores_the_mock_brain():
    demo, brain = _demo()
    demo.start("keyfeatures")
    _run(demo, [demo.cap("last line")])  # the script's end
    demo.tick()
    demo.tick()
    assert not demo.steps and brain.enabled is False


class _FakeAgent:
    def __init__(self, lines=()):
        self.lines, self.submitted, self.busy = list(lines), [], False
        self.snap = SimpleNamespace(raw=None, fallback={})

    def available(self):
        return True

    def submit(self, text, llm=None, confirm=True):
        self.submitted.append(text)
        return True

    def reset(self):
        pass

    def watch(self, task, speak=True):
        self.speak = speak

    def drain(self):
        out, self.lines = self.lines, []
        return out


def _ctx(agent, demo, brain):
    shared = SimpleNamespace(state='{"say": null}', push_cmd=lambda c: None)
    return SimpleNamespace(
        store={"agent": agent},
        demo=demo,
        brain=brain,
        push=shared.push_cmd,
        tasks=SimpleNamespace(task=None, markers=[], objs={}),
    )


def test_speaking_to_the_agent_stops_a_running_demo():
    from pps.features import agent as fa

    demo, brain = _demo()
    demo.start("keyfeatures")
    ag = _FakeAgent()
    fa.on_agent(_ctx(ag, demo, brain), {"type": "agent", "text": "put the mug in the center"})
    assert ag.submitted == ["put the mug in the center"] and not demo.steps and brain.enabled is False


def test_the_agent_reply_is_said_and_stops_the_demo():
    from pps.features import agent as fa

    demo, brain = _demo()
    demo.start("keyfeatures")
    demo.tick()  # the demo said its first lines
    ag = _FakeAgent(["The vase occupies the center, so I'll put the mug beside it."])
    fa.agent_tick(_ctx(ag, demo, brain))
    assert brain.say == {
        "id": brain._sid,
        "text": "The vase occupies the center, so I'll put the mug beside it.",
        "kind": "robot",
    }
    assert not demo.steps and brain.real_fresh()
    demo.tick()  # nothing of the demo comes back over it
    assert brain.say["text"].startswith("The vase occupies")


def test_the_agent_says_its_own_results_when_the_mock_brain_cant():
    from pps.features import agent as fa

    brain = MockBrain()
    brain.enabled = True  # mock on, but an LLM brain is in charge: the
    ag = _FakeAgent()  # mock doesn't narrate (app.py), so the agent does
    fa.agent_tick(_ctx(ag, None, brain))
    assert ag.speak is True
