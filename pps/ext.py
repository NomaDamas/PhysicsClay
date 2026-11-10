"""Extension points so features live in their own files (pps/features/*.py) instead of growing app.py.

A feature module registers itself at import time:

    from .. import ext

    @ext.handler("zone")                 # browser message {"type": "zone", ...}
    def on_zone(ctx, msg): ...

    @ext.pre_task                        # sees every {"type": "task"} first; return True to consume it
    def preview_first(ctx, msg): ...

    @ext.guard                           # a pre_task that runs before all the others (safety: nobody consumes a
    def people_first(ctx, msg): ...      # task before it is checked)

    @ext.state                           # merged into every state frame the browser receives
    def zones_state(ctx): return {"zones": [...]}

    @ext.tick                            # once per worker frame, after physics
    def step(ctx): ...

    @ext.overlay                         # draw into the robot-eye render: fn(ctx, mjvScene) adds geoms (scn.ngeom)
    def paint(ctx, scn): ...

ctx (one per server) carries: rt, tasks, teacher, brain, demo, spatial, cam_params, aspect, frame, view, store (a dict
features may use for their own state), and dispatch(msg) to run a task message through the normal path.
Every pps/features/*.py module is imported automatically by load()."""
import importlib
import pkgutil
import traceback
from types import SimpleNamespace

HANDLERS, PRE_TASK, STATE, TICK, OVERLAY = {}, [], [], [], []


def handler(kind):
    def deco(fn):
        HANDLERS[kind] = fn
        return fn
    return deco


def pre_task(fn):
    PRE_TASK.append(fn)
    return fn


def guard(fn):
    PRE_TASK.insert(0, fn)
    return fn


def state(fn):
    STATE.append(fn)
    return fn


def tick(fn):
    TICK.append(fn)
    return fn


def overlay(fn):
    OVERLAY.append(fn)
    return fn


def make_ctx(**kw):
    return SimpleNamespace(store={}, cam_params=None, frame=None, view="robot", **kw)


def load():
    from . import features
    for m in pkgutil.iter_modules(features.__path__):
        if not m.name.startswith("_"):
            importlib.import_module(f"{features.__name__}.{m.name}")


def _safe(fn, *a):
    try:
        return fn(*a)
    except Exception:  # a broken feature must never kill the robot loop
        traceback.print_exc()
        return None


def handle(ctx, msg):
    fn = HANDLERS.get(msg.get("type"))
    if fn is None:
        return False
    _safe(fn, ctx, msg)
    return True


def intercept_task(ctx, msg):
    return any(_safe(fn, ctx, msg) is True for fn in PRE_TASK)


def run_ticks(ctx):
    for fn in TICK:
        _safe(fn, ctx)


def paint_overlays(ctx, scn):
    """Called by the worker right after the runtime paints its own overlays, for each renderer's scene."""
    for fn in OVERLAY:
        _safe(fn, ctx, scn)


def extra_state(ctx):
    out = {}
    for fn in STATE:
        d = _safe(fn, ctx)
        if isinstance(d, dict):
            out.update(d)
    return out
