"""Datasets (v1 P1): record every task as a LeRobot episode (pps/data/, docs/datasets.md).

Messages
  {type: "dataset", op: "start", name?, auto: true, fps?: 30, cameras?: ["front", "wrist"], size?: 256}
  {type: "dataset", op: "stop"}                       ends recording (an episode still running is dropped)
  {type: "dataset", op: "discard_last"}               removes the last finished episode from disk
  {type: "dataset", op: "randomize", lighting?: 0..1, camera?: metres, seed?}   per-episode render randomization
                                                      (scripts/generate.py; lighting and camera jitter of the
                                                      dataset cameras only, the live view is untouched)
  {type: "dataset", op: "meta", data: {...}}         JSON added to the following episodes' records ("variation")
Every op answers in state.dataset.reply = {id, op, ok, message}.

State
  dataset = {recording, name, auto, fps, size, cameras, episodes, succeeded, refused, frames, current: {task, frames,
             seconds} | null, last: {i, robot, task, success, outcome, seconds, frames} | null, list: [...last 30],
             root, datasets: [LeRobot roots written so far], pending, cost_ms: {snapshot, render}, randomize, error,
             message, reply}

Episodes are tasks that start moving; the words come from the task message's `utterance` / `text` (the browser and
the agent attach what you said) or are generated from the task's arguments. See pps/data/recorder.py.
"""
from .. import ext
from ..data.recorder import DatasetRecorder

KEY = "dataset"


def _rec(ctx):
    r = ctx.store.get(KEY)
    if r is None:
        r = ctx.store[KEY] = DatasetRecorder()
        r.reply = None
    return r


@ext.handler("dataset")
def on_dataset(ctx, msg):
    r = _rec(ctx)
    op = str(msg.get("op") or "")
    try:
        if op == "start":
            ok, text = r.start(ctx, msg.get("name"), auto=msg.get("auto", True) is not False, fps=msg.get("fps"),
                               cameras=msg.get("cameras"), size=msg.get("size"))
        elif op == "stop":
            ok, text = r.stop(ctx)
        elif op == "discard_last":
            ok, text = r.discard_last()
        elif op == "meta":
            ok, text = r.set_meta(msg.get("data"))
        elif op == "randomize":
            ok, text = r.set_randomization({k: msg.get(k) for k in ("lighting", "camera") if msg.get(k) is not None},
                                           seed=msg.get("seed"))
        else:
            ok, text = False, f"Unknown dataset op '{op}'. Use start, stop, discard_last, randomize or meta."
    except Exception as e:                  # never kill the loop; say what went wrong
        ok, text = False, f"Dataset error: {e}"
    r.reply = {"id": msg.get("id"), "op": op, "ok": bool(ok), "message": text}
    if ok and op in ("start", "stop", "discard_last"):
        r.message = text


@ext.guard          # first in line (features that consume a task, e.g. "which one?", would hide it); never consumes
def dataset_words(ctx, msg):
    """See every task message first: remember who asked for what, in which words."""
    _rec(ctx).note_task(msg)
    return False


@ext.tick
def dataset_tick(ctx):
    _rec(ctx).tick(ctx)


@ext.state
def dataset_state(ctx):
    r = ctx.store.get(KEY)
    if r is None:
        return {KEY: {"recording": False, "episodes": 0, "frames": 0, "last": None, "list": [], "reply": None}}
    st = r.state()
    st["reply"] = r.reply
    return {KEY: st}
