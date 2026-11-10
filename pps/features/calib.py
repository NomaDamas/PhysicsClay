"""P6 calibration in the server (docs/calibration.md): probe the robot, fit the simulator to what it did, apply the fit.

    {type: "calib", op: "probe", probes?: ["swing", "press", "lift", "slide"]}   run the probe routine (a task with status)
    {type: "calib", op: "fit", budget?: "quick"|"normal"|"thorough"}            fit the parameters to the last probe logs
    {type: "calib", op: "apply"}                                                 the fitted parameters -> the plant (physics set)
    {type: "calib", op: "reset"}                                                 the plant back to its defaults, fit dropped
    {type: "calib", op: "run", budget?}                                          probe + fit + apply ("recalibrate here")
    {type: "calib", op: "stop"}                                                  stop probing / fitting

state.calib = {status: idle|probing|probed|fitting|fitted|applied|failed, message, summary, probes: [{name, label, status}],
               progress: {stage, gen, gens}, params: {...}, defaults: {...}, applied: bool,
               fit: {err_before, err_after, seconds, evals, budget, logs: [...]}, per_param: [{name, value, default, ...}]}

The robot is driven through a RobotDriver (pps/bridge SimDriver on the plant): the probe runner takes over the
runtime's plant step while it runs. The fit replays the logs in a headless copy of the world in up to 2 worker
processes (a background thread waits on them, so the robot loop keeps its frame rate). A fitted profile is saved under
recordings/calib/ (one file per robot and scene: the per-site profile).
"""
import json
import threading
import time
from pathlib import Path

from .. import ext
from ..bridge import SimDriver
from ..calib import params as P
from ..calib import probes as PR
from ..calib import sysid

PROFILE_DIR = Path(__file__).resolve().parents[2] / "recordings" / "calib"


def _st(ctx):
    c = ctx.store.get("calib")
    if c is None:
        c = ctx.store["calib"] = {"status": "idle", "message": "", "summary": "", "probes": [], "progress": None, "params": None,
                                  "fit": None, "per_param": [], "applied": False, "logs": [], "runner": None, "job": None,
                                  "tid": None, "auto": False, "rt": None, "world": None}
    return c


def _task(ctx, c, status, message, step=""):
    ctx.tasks.set_task(c.get("tid") or "calib", "calib", status, message, step)


def _fail(ctx, c, msg):
    c["status"], c["message"], c["auto"] = "failed", msg, False
    _task(ctx, c, "failed", msg)


@ext.handler("calib")
def on_calib(ctx, msg):
    c = _st(ctx)
    op = str(msg.get("op") or "probe")
    c["tid"] = str(msg.get("id") or f"calib-{op}")
    rt = ctx.rt
    if op == "stop":
        stop(ctx, c, "Calibration stopped.")
        return
    if op == "reset":
        stop(ctx, c, None)
        if rt.plant is not None:
            rt.plant.reset_params()
        c.update(status="idle", params=None, fit=None, per_param=[], applied=False, progress=None, summary="",
                 message="Physics back to the defaults.")
        _task(ctx, c, "done", "Calibration reset: physics back to the defaults.")
        return
    if rt.plant is None:
        _fail(ctx, c, "Calibration needs physics: the server runs in kinematic mode (restart without --kinematic).")
        return
    if op in ("probe", "run"):
        if c["status"] in ("probing", "fitting"):
            _fail(ctx, c, f"I'm already {c['status']}; say stop first.")
            return
        c["auto"] = op == "run"
        c["budget"] = str(msg.get("budget") or "quick")
        start_probe(ctx, c, msg.get("probes"))
    elif op == "fit":
        if c["status"] in ("probing", "fitting"):
            _fail(ctx, c, f"I'm {c['status']} right now.")
            return
        start_fit(ctx, c, str(msg.get("budget") or "quick"))
    elif op == "apply":
        apply(ctx, c)
    else:
        _fail(ctx, c, f"Unknown calib op '{op}'. Use probe, fit, apply, reset, run or stop.")


# ------------------------------------------------------------------ probing
def start_probe(ctx, c, names=None):
    rt, tasks = ctx.rt, ctx.tasks
    if tasks.kind or rt.mode in ("FOLLOW", "PRESS", "PREVIEW", "REC"):
        tasks.stop("calib-stop")
    names = [n for n in (names or PR.ORDER) if n in PR.PROBES]
    if not names:
        _fail(ctx, c, "No probes to run (swing, press, lift, slide).")
        return
    drv = SimDriver(rt.plant, base_pose=rt.base_pose, name="sim")
    meta = {"robot": ctx.world.profile.id, "scene": ctx.world.spec, "params_at_probe": dict(rt.plant.params)}

    def changed(r):
        c["probes"] = [{"name": n, "label": PR.PROBES[n].label,
                        "status": "done" if n in r.done_probes else "skipped" if any(n == s for s, _ in r.skipped)
                        else "running" if r.probe is not None and r.probe.name == n else "waiting"} for n in names]
    runner = PR.ProbeRunner(rt, tasks, drv, PR.default_probes(names), meta=meta, on_change=changed)
    changed(runner)
    runner.install()
    c.update(status="probing", runner=runner, rt=rt, world=ctx.world, logs=[], progress=None, applied=False,
             message=f"Probing: {PR.PROBES[names[0]].label} (1/{len(names)})", summary="")
    _task(ctx, c, "running", f"Calibrating: I'll run {len(names)} short probes ({', '.join(names)}). Keep the table clear.", "probe")


def _probe_tick(ctx, c):
    r = c["runner"]
    if ctx.rt is not c["rt"]:                            # the world was rebuilt under the probe: drop it
        c.update(runner=None, status="failed", message="The world changed while probing.")
        return
    if not r.finished:
        p = r.progress()
        if r.probe is not None:
            c["message"] = f"Probing: {r.probe.label} ({p['index']}/{p['total']})"
            if (ctx.tasks.task or {}).get("id") == c.get("tid"):
                ctx.tasks.task["step"] = r.probe.name
        return
    c["runner"] = None
    c["logs"] = list(r.logs)
    skipped = "; ".join(f"{n}: {why}" for n, why in r.skipped)
    if r.status != "done" or not r.logs:
        _fail(ctx, c, f"Probing stopped: {r.message or 'no logs'}." + (f" Skipped {skipped}." if skipped else ""))
        return
    secs = sum(lg.duration for lg in r.logs)
    c["status"] = "probed"
    c["message"] = f"Probed: {', '.join(lg.probe for lg in r.logs)} ({secs:.0f} s)." + (f" Skipped {skipped}." if skipped else "")
    if c.get("auto"):
        start_fit(ctx, c, c.get("budget", "quick"))
    else:
        _task(ctx, c, "done", c["message"] + " Say 'fit' to calibrate the simulator to it.")


def stop(ctx, c, msg):
    r = c.get("runner")
    if r is not None and not r.finished:
        r.abort("stopped")
    c["runner"] = None
    job = c.get("job")
    if job is not None:
        job["cancel"] = True
        c["job"] = None
    c["auto"] = False
    if msg:
        c.update(status="idle" if not c.get("logs") else "probed", message=msg)
        _task(ctx, c, "done", msg)


# ------------------------------------------------------------------ fitting (background thread + worker processes)
def start_fit(ctx, c, budget="quick"):
    logs = c.get("logs") or []
    if not logs:
        _fail(ctx, c, "Nothing to fit yet: run the probes first (op: probe).")
        return
    w = c.get("world") or ctx.world
    job = {"cancel": False, "result": None, "error": None, "progress": None, "t0": time.time(), "budget": budget}

    def prog(p):
        job["progress"] = p
        if job["cancel"]:
            raise KeyboardInterrupt

    def work():
        try:
            job["result"] = sysid.fit(logs, w.profile.id, w.spec, budget=budget, on_progress=prog)
        except KeyboardInterrupt:
            job["error"] = "stopped"
        except Exception as e:  # report, never kill the server
            job["error"] = f"{type(e).__name__}: {e}"
    c.update(status="fitting", job=job, progress=None, message=f"Fitting the simulator to {len(logs)} probe logs ({budget}).")
    _task(ctx, c, "running", c["message"], "fit")
    threading.Thread(target=work, name="calib-fit", daemon=True).start()


def _fit_tick(ctx, c):
    job = c["job"]
    if job["progress"]:
        p = job["progress"]
        c["progress"] = {"stage": p["stage"], "stage_index": p["stage_index"], "stages": p["stages"], "gen": p["gen"], "gens": p["gens"]}
        c["message"] = f"Fitting: {p['stage']} ({p['stage_index'] + 1}/{p['stages']}), generation {p['gen']}/{p['gens']}"
    if job["result"] is None and job["error"] is None:
        return
    c["job"] = None
    if job["error"]:
        if job["error"] != "stopped":
            _fail(ctx, c, f"The fit failed: {job['error']}")
        return
    r = job["result"]
    c["params"] = r["params"]
    c["per_param"] = r["per_param"]
    c["fit"] = {k: r[k] for k in ("err_before", "err_after", "seconds", "evals", "budget", "logs", "stages")}
    c["status"] = "fitted"
    c["summary"] = summarize(r)
    c["message"] = c["summary"]
    _save_profile(c)
    if c.get("auto"):
        apply(ctx, c)
    else:
        _task(ctx, c, "done", c["summary"] + " Say 'apply' to use it.")


def summarize(r):
    """One sentence: what changed, and how much better the simulator now follows the robot."""
    parts = []
    for p in r["per_param"]:
        n, v, d = p["name"], p["value"], p["default"]
        if n == "latency_ms":
            parts.append(f"latency {v:.0f} ms")
            continue
        word = {"friction_scale": "friction", "mass_scale": "mass", "damping_scale": "damping", "kp_scale": "stiffness",
                "armature_scale": "rotor inertia", "frictionloss_scale": "joint friction"}.get(n, n)
        note = ""
        if n == "friction_scale" and v < 0.8:
            note = " (slippery)"
        elif n == "friction_scale" and v > 1.25:
            note = " (grippy)"
        parts.append(f"{word} ×{v:.2f}{note}" if abs(v - d) > 0.03 else f"{word} as modelled")
    q0 = [lg["before"].get("q_deg") or 0 for lg in r["logs"]]
    q1 = [lg["after"].get("q_deg") or 0 for lg in r["logs"]]
    gain = f"; joint tracking error {max(q0):.2f}° → {max(q1):.2f}°" if q0 and q1 else ""
    return "Calibrated: " + ", ".join(parts) + gain + "."


def apply(ctx, c):
    if not c.get("params"):
        _fail(ctx, c, "Nothing fitted yet: run the probes and the fit first.")
        return
    w = c.get("world")
    if w is not None and w.profile.id != ctx.world.profile.id:
        _fail(ctx, c, f"That fit is for the {w.profile.label}; probe this robot first.")
        return
    got = P.apply(ctx.rt.plant, c["params"])
    c["applied"], c["status"], c["auto"] = True, "applied", False
    shown = ", ".join(f"{k} {got[k]:.3g}" for k in c["params"])
    c["message"] = f"Applied to the simulator: {shown}."
    _task(ctx, c, "done", (c.get("summary") or "Calibrated.") + " Applied.")


def _save_profile(c):
    """recordings/calib/<robot>.json: the latest fitted profile per robot (a site keeps its own copy of this file)."""
    try:
        w = c.get("world")
        PROFILE_DIR.mkdir(parents=True, exist_ok=True)
        rid = w.profile.id if w is not None else "robot"
        with open(PROFILE_DIR / f"{rid}.json", "w") as f:
            json.dump({"robot": rid, "scene": w.spec if w is not None else None, "params": c["params"], "fit": c["fit"],
                       "time": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)
    except OSError:
        pass


# ------------------------------------------------------------------ loop hooks
@ext.tick
def calib_tick(ctx):
    c = ctx.store.get("calib")
    if not c:
        return
    if c.get("rt") is not None and ctx.rt is not c["rt"] and c.get("runner") is None and c.get("job") is None:
        c.update(status="idle", message="", summary="", probes=[], progress=None, params=None, fit=None, per_param=[],
                 applied=False, logs=[], rt=None, world=None)   # a new world: its own calibration
    if c.get("runner") is not None:
        _probe_tick(ctx, c)
    if c.get("job") is not None:
        _fit_tick(ctx, c)


@ext.pre_task
def busy_probing(ctx, msg):
    """While the probes run the arm is not free: 'stop' ends the probing, other tasks wait."""
    c = ctx.store.get("calib")
    if not c or c.get("runner") is None:
        return False
    if msg.get("action") in ("stop", "home", "reset_scene"):
        stop(ctx, c, "Calibration stopped.")
        return False
    ctx.tasks.set_task(str(msg.get("id", "t?")), str(msg.get("action")), "failed",
                       "I'm calibrating (probing the table): say stop first, or wait until it's done.")
    return True


@ext.state
def calib_state(ctx):
    c = ctx.store.get("calib")
    if not c:
        return {"calib": {"status": "idle", "message": "", "probes": [], "params": None, "fit": None, "per_param": [], "applied": False,
                          "defaults": {n: P.PARAMS[n].default for n in P.CORE}}}
    return {"calib": {"status": c["status"], "message": c["message"], "summary": c.get("summary", ""), "probes": c.get("probes", []),
                      "progress": c.get("progress"), "params": c.get("params"), "fit": c.get("fit"), "per_param": c.get("per_param", []),
                      "applied": c.get("applied", False), "logs": [{"probe": lg.probe, "seconds": round(lg.duration, 1)} for lg in c.get("logs") or []],
                      "defaults": {n: P.PARAMS[n].default for n in P.CORE}}}
