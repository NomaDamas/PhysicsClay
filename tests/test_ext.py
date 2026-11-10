"""pps/ext.py: the feature registry (handlers, pre-task hooks, state, ticks, overlays) and its fault isolation."""

import sys
from pathlib import Path

import pytest

import pps.features
from pps import ext


@pytest.fixture
def reg(monkeypatch):
    """Fresh, empty registries so tests don't see (or leak into) the real features."""
    for name in ("HANDLERS",):
        monkeypatch.setattr(ext, name, {})
    for name in ("PRE_TASK", "STATE", "TICK", "OVERLAY"):
        monkeypatch.setattr(ext, name, [])
    return ext


def test_make_ctx_defaults():
    ctx = ext.make_ctx(rt="RT")
    assert ctx.rt == "RT" and ctx.store == {} and ctx.view == "robot"
    assert ctx.cam_params is None and ctx.frame is None
    assert ext.make_ctx().store is not ext.make_ctx().store


def test_handler_dispatch(reg):
    seen = []

    @reg.handler("zone")
    def on_zone(ctx, msg):
        seen.append(msg["op"])

    assert on_zone.__name__ == "on_zone"  # decorators return the function
    ctx = reg.make_ctx()
    assert reg.handle(ctx, {"type": "zone", "op": "add"}) is True
    assert reg.handle(ctx, {"type": "unknown"}) is False
    assert reg.handle(ctx, {}) is False
    assert seen == ["add"]


def test_handler_errors_are_contained(reg, capsys):
    @reg.handler("boom")
    def boom(ctx, msg):
        raise RuntimeError("feature bug")

    assert reg.handle(reg.make_ctx(), {"type": "boom"}) is True  # handled, not raised
    assert "feature bug" in capsys.readouterr().err


def test_pre_task_consumes_only_on_true(reg):
    calls = []
    reg.pre_task(lambda ctx, msg: calls.append("a"))  # None -> pass through
    reg.pre_task(lambda ctx, msg: calls.append("b") or 1)  # truthy but not True -> pass through
    ctx = reg.make_ctx()
    assert reg.intercept_task(ctx, {"type": "task"}) is False
    reg.pre_task(lambda ctx, msg: True)
    assert reg.intercept_task(ctx, {"type": "task"}) is True


def test_pre_task_error_does_not_consume(reg):
    def bad(ctx, msg):
        raise ValueError

    reg.pre_task(bad)
    assert reg.intercept_task(reg.make_ctx(), {}) is False


def test_extra_state_merges_dicts_and_ignores_the_rest(reg):
    reg.state(lambda ctx: {"zones": [1]})
    reg.state(lambda ctx: None)
    reg.state(lambda ctx: ["not", "a", "dict"])
    reg.state(lambda ctx: 1 / 0)
    reg.state(lambda ctx: {"goal": {"ok": True}, "zones": [2]})  # later features win on key clashes
    assert reg.extra_state(reg.make_ctx()) == {"zones": [2], "goal": {"ok": True}}


def test_ticks_and_overlays_run_in_order_and_survive_errors(reg):
    order = []
    reg.tick(lambda ctx: order.append("t1"))
    reg.tick(lambda ctx: 1 / 0)
    reg.tick(lambda ctx: order.append("t2"))
    reg.overlay(lambda ctx, scn: order.append(("o", scn)))
    ctx = reg.make_ctx()
    reg.run_ticks(ctx)
    reg.paint_overlays(ctx, "scene")
    assert order == ["t1", "t2", ("o", "scene")]


def test_load_imports_every_feature():
    # real registries: a feature registers when its module is first imported, whichever test imported it
    ext.load()
    mods = {m.rsplit(".", 1)[-1] for m in sys.modules if m.startswith("pps.features.")}
    shipped = {p.stem for p in Path(pps.features.__file__).parent.glob("*.py")}
    assert {s for s in shipped if not s.startswith("_")} <= mods
    assert "_example" not in mods
    assert {"zone", "copresence"} <= set(ext.HANDLERS)  # message types of the shipped features
    assert ext.STATE and ext.TICK
