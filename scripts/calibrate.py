"""Real-to-sim calibration from the command line (docs/calibration.md).

  .venv/bin/python scripts/calibrate.py --demo [--seeds 5] [--robot panda] [--budget normal] [--out recordings/calib/demo.json]
      Proof without hardware. For each seed: a "hidden real" robot is the simulator with secret parameters (drawn with
      pps.calib.randomize) and noisy sensors; the probe routine runs on it and records logs; a nominal simulator is
      calibrated from those logs only; then a held-out task (a pick & place the probes never did) is run on the hidden
      robot and replayed in the nominal and in the calibrated simulator. Prints recovered vs true per parameter and the
      held-out tracking error before / after, plus medians over the seeds and the runtime.

  .venv/bin/python scripts/calibrate.py --probe --robot panda [--params '{"friction_scale": 0.6}'] --save logs/
      Run the probe routine on a simulated robot (optionally with given parameters) and save the logs (.npz).

  .venv/bin/python scripts/calibrate.py --fit logs/*.npz [--budget normal]
      Fit the parameters to saved logs (robot and scene come from the logs' meta). With --lerobot episode.parquet
      --robot <id> --scene scene.json, fits a P1 dataset episode (joint terms only unless it has forces / objects).
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

DEMO_SCENE = {"template": "table", "objects": [
    {"name": "red_block", "kind": "block", "color": "red", "pos": [0.42, -0.15], "movable": True},
    {"name": "mug", "kind": "mug", "color": "red", "pos": [0.40, 0.18], "movable": True}]}
# the hidden robot's parameters are drawn from these ranges (inside the search ranges, so a fit can reach them)
DEMO_RANGES = {"friction_scale": (0.5, 1.6), "mass_scale": (0.5, 2.0), "damping_scale": (0.6, 1.7), "kp_scale": (0.6, 1.5),
               "latency_ms": (0.0, 60.0)}
NOISE = {"q": 5e-4, "ee_force": 0.1, "obj_p": 0.0015}       # encoder (rad), wrist F/T (N), object tracking (m)
TOL = {"friction_scale": 0.15, "mass_scale": 0.15, "kp_scale": 0.15, "damping_scale": 0.15, "latency_ms": 10.0}


def heldout_task(rt, tasks, drv, meta, dt=1 / 120, limit=40.0):
    """A pick & place the probes never did (the mug to the centre of the table), recorded on the robot."""
    from pps.calib.probes import TaskRecorder
    rec = TaskRecorder(rt, drv, {**meta, "probe": "heldout", "label": "Held-out pick & place"}).install()
    t = 0.0
    for _ in range(int(0.3 / dt)):                       # a short rest first (the replay starts from it)
        rt.step(dt)
        tasks.tick()
    tasks.pick_place("heldout", "mug", "center", None, None, 4 / 3)
    while t < limit:
        rt.step(dt)
        tasks.tick()
        t += dt
        if (tasks.task or {}).get("status") != "running":
            break
    for _ in range(int(0.5 / dt)):
        rt.step(dt)
        tasks.tick()
    log = rec.uninstall()
    return log, dict(tasks.task or {})


def run_seed(seed, robot, budget, workers, log=print, save_dir=None):
    from pps.calib import params as P, probes as PR, sysid
    from pps.calib.randomize import Randomizer
    t0 = time.time()
    secret = Randomizer(list(DEMO_RANGES), robot=robot, seed=seed, ranges=DEMO_RANGES).sample()
    w, rt, tasks, drv = PR.headless_robot(robot, DEMO_SCENE, params=secret, noise=NOISE, seed=seed)
    meta = {"robot": robot, "scene": w.spec}
    runner = PR.ProbeRunner(rt, tasks, drv, PR.default_probes(), meta=meta)
    logs = PR.run_headless(runner)
    t_probe = time.time() - t0
    hold, task = heldout_task(rt, tasks, drv, meta)
    if save_dir:                                         # keep the logs: the numbers can be re-checked without rerunning
        os.makedirs(save_dir, exist_ok=True)
        for lg in logs + [hold]:
            lg.save(os.path.join(save_dir, f"seed{seed}_{lg.probe}.npz"))
    log(f"  seed {seed}: secret {json.dumps({k: round(v, 3) for k, v in secret.items()})}; probes {[lg.probe for lg in logs]} "
        f"{sum(lg.duration for lg in logs):.0f} s sim in {t_probe:.0f} s; held-out task: {task.get('status')} ({task.get('message', '')[:60]})")
    t1 = time.time()
    fit = sysid.fit(logs, robot, w.spec, budget=budget, workers=workers, seed=seed)
    t_fit = time.time() - t1
    sim = sysid.PlantSim(robot, w.spec)
    nominal = {n: P.PARAMS[n].default for n in fit["params"]}
    track = {"nominal": sysid.tracking(sim, hold, nominal), "calibrated": sysid.tracking(sim, hold, fit["params"]),
             "true": sysid.tracking(sim, hold, {k: secret[k] for k in fit["params"]})}
    for k, v in track.items():
        if v.get("diverged_s") is not None:
            log(f"    note: the {k} replay of the held-out task diverged at {v['diverged_s']} s (MuJoCo instability); "
                f"errors are over the part before it")
    rows = []
    for n, v in fit["params"].items():
        tv = float(secret[n])
        err = abs(v - tv) if n == "latency_ms" else abs(v - tv) / tv
        rows.append({"name": n, "true": round(tv, 4), "fit": round(v, 4), "err": round(err, 4),
                     "ok": bool(err <= TOL.get(n, 0.15))})
    return {"seed": seed, "secret": secret, "fit": fit, "params": rows, "heldout": track, "heldout_task": task.get("status"),
            "seconds": {"probe": round(t_probe, 1), "fit": round(t_fit, 1), "total": round(time.time() - t0, 1)},
            "probe_sim_s": round(sum(lg.duration for lg in logs), 1)}


def fmt_err(n, e):
    return f"{e:.1f} ms" if n == "latency_ms" else f"{100 * e:.1f} %"


def report(results, robot, budget):
    names = [r["name"] for r in results[0]["params"]]
    out = [f"### Hidden-real calibration: {robot}, {len(results)} seeds, budget '{budget}'", "",
           "Per seed (true → fitted):", "",
           "| seed | " + " | ".join(names) + " | held-out joints (°) nominal → calibrated | hand path (mm) | object path (mm) |",
           "|---|" + "---|" * len(names) + "---|---|---|"]
    for r in results:
        cells = [f"{p['true']:.3g} → {p['fit']:.3g}" + ("" if p["ok"] else " ✗") for p in r["params"]]
        h = r["heldout"]
        div = [k for k in ("nominal", "calibrated") if h[k].get("diverged_s") is not None]
        cells[-1] += "".join(f" ({k} replay diverged at {h[k]['diverged_s']} s)" for k in div)
        out.append(f"| {r['seed']} | " + " | ".join(cells) +
                   f" | {h['nominal']['q_deg']:.2f} → {h['calibrated']['q_deg']:.2f} | {h['nominal']['tcp_mm']:.1f} → {h['calibrated']['tcp_mm']:.1f} "
                   f"| {h['nominal'].get('p_mm') or 0:.1f} → {h['calibrated'].get('p_mm') or 0:.1f} |")
    out += ["", "Median over seeds:", "", "| parameter | median error | worst | within tolerance |", "|---|---|---|---|"]
    for n in names:
        errs = [next(p for p in r["params"] if p["name"] == n)["err"] for r in results]
        oks = [next(p for p in r["params"] if p["name"] == n)["ok"] for r in results]
        tol = f"{TOL.get(n, 0.15):g} ms" if n == "latency_ms" else f"{100 * TOL.get(n, 0.15):.0f} %"
        out.append(f"| {n} | {fmt_err(n, float(np.median(errs)))} | {fmt_err(n, max(errs))} | {sum(oks)}/{len(oks)} (±{tol}) |")
    med = lambda key, which, k: float(np.median([r["heldout"][which].get(k) or 0.0 for r in results]))
    out += ["", "Held-out pick & place on the hidden robot, replayed open loop (median over seeds):", "",
            "| simulator | joints RMS (°) | hand path RMS (mm) | object path RMS (mm) | object final (mm) | wrist force RMS (N) |",
            "|---|---|---|---|---|---|"]
    for which, label in (("nominal", "nominal (before)"), ("calibrated", "calibrated (after)"), ("true", "true parameters (noise floor)")):
        out.append(f"| {label} | {med(0, which, 'q_deg'):.3f} | {med(0, which, 'tcp_mm'):.2f} | {med(0, which, 'p_mm'):.2f} | "
                   f"{med(0, which, 'final_obj_mm'):.2f} | {med(0, which, 'f_N'):.3f} |")
    secs = [r["seconds"] for r in results]
    out += ["", f"Runtime per seed (median): probes {np.median([s['probe'] for s in secs]):.0f} s "
                f"({np.median([r['probe_sim_s'] for r in results]):.0f} s of robot time), fit {np.median([s['fit'] for s in secs]):.0f} s "
                f"({int(np.median([r['fit']['evals'] for r in results]))} rollouts, 2 processes), total {np.median([s['total'] for s in secs]):.0f} s.",
            f"Held-out task outcome on the hidden robot: {', '.join(str(r['heldout_task']) for r in results)}."]
    return "\n".join(out)


def demo(a):
    seeds = list(range(a.seed0, a.seed0 + a.seeds))
    results = []
    t0 = time.time()
    print(f"Hidden-real demo: {a.robot}, seeds {seeds}, budget {a.budget}", flush=True)
    for s in seeds:
        r = run_seed(s, a.robot, a.budget, a.workers, log=lambda m: print(m, flush=True), save_dir=a.save if a.save_logs else None)
        results.append(r)
        print("    " + ", ".join(f"{p['name']} {p['true']:.3g}→{p['fit']:.3g} ({fmt_err(p['name'], p['err'])})" for p in r["params"])
              + f"; held-out joints {r['heldout']['nominal']['q_deg']:.2f}°→{r['heldout']['calibrated']['q_deg']:.2f}°"
              f" ({r['seconds']['total']:.0f} s)", flush=True)
    text = report(results, a.robot, a.budget)
    print("\n" + text + f"\n\nTotal {time.time() - t0:.0f} s.")
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump({"robot": a.robot, "budget": a.budget, "noise": NOISE, "ranges": DEMO_RANGES, "results": results,
                       "report": text}, f, indent=1, default=float)
        print(f"Saved {a.out}")
    fails = sum(1 for r in results for p in r["params"] if not p["ok"] and p["name"] in ("friction_scale", "mass_scale", "kp_scale", "latency_ms"))
    return 0 if fails <= len(results) else 1


def rereport(a):
    """Re-render the tables of a saved --demo run (no simulation)."""
    data = json.load(open(a.report))
    print(report(data["results"], data["robot"], data["budget"]))
    return 0


def probe(a):
    from pps.calib import probes as PR
    params = json.loads(a.params) if a.params else None
    scene = json.load(open(a.scene)) if a.scene else DEMO_SCENE
    w, rt, tasks, drv = PR.headless_robot(a.robot, scene, params=params, noise=NOISE if a.noise else None, seed=a.seed0)
    logs = PR.run_headless(PR.ProbeRunner(rt, tasks, drv, PR.default_probes(), meta={"robot": a.robot, "scene": w.spec}))
    os.makedirs(a.save, exist_ok=True)
    for lg in logs:
        p = os.path.join(a.save, f"{lg.probe}.npz")
        lg.save(p)
        print(f"{p}: {len(lg)} rows, {lg.duration:.1f} s")
    return 0


def fit(a):
    from pps.calib import sysid
    from pps.calib.log import Log, from_lerobot
    if a.lerobot:
        from pps.world import scene as S
        scene = json.load(open(a.scene)) if a.scene else DEMO_SCENE
        from pps.bridge.sim import joint_labels
        w = S.build(a.robot, scene, physics=True)
        labels = joint_labels(w.model, w.ik_model.nq)
        home = w.home_q if w.home_q is not None else w.ik_model.qpos0
        logs = [from_lerobot(p, labels, q_home=home, meta={"robot": a.robot, "scene": w.spec}) for p in a.lerobot]
    else:
        logs = [Log.load(p) for p in a.fit]
    robot, scene = logs[0].meta.get("robot", a.robot), logs[0].meta.get("scene", DEMO_SCENE)
    r = sysid.fit(logs, robot, scene, budget=a.budget, workers=a.workers,
                  on_progress=lambda p: print(f"  {p['stage']} {p['gen']}/{p['gens']}: J={p['best']:.4g}", flush=True) if p["gen"] % 5 == 0 else None)
    print(json.dumps({k: r[k] for k in ("params", "err_before", "err_after", "evals", "seconds", "stages")}, indent=1))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="hidden-real proof (no hardware)")
    ap.add_argument("--probe", action="store_true", help="run the probe routine on a simulated robot and save its logs")
    ap.add_argument("--fit", nargs="*", default=None, help="fit saved logs (.npz)")
    ap.add_argument("--lerobot", nargs="*", default=None, help="fit P1 LeRobot episode parquet files")
    ap.add_argument("--robot", default="panda")
    ap.add_argument("--scene", default=None, help="scene spec JSON (default: a table with a block and a mug)")
    ap.add_argument("--params", default=None, help="plant parameters for --probe, JSON")
    ap.add_argument("--noise", action="store_true", help="--probe: noisy sensors like the demo's hidden robot")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--budget", default="normal", choices=["quick", "normal", "thorough"])
    ap.add_argument("--workers", type=int, default=2, help="rollout processes (at most 2)")
    ap.add_argument("--save", default="recordings/calib/logs", help="where --probe (and --demo --save-logs) write logs")
    ap.add_argument("--save-logs", action="store_true", help="--demo: also save every seed's probe and held-out logs")
    ap.add_argument("--report", default=None, help="re-render the tables of a saved --demo JSON")
    ap.add_argument("--out", default="recordings/calib/demo.json")
    a = ap.parse_args()
    if a.report:
        return rereport(a)
    if a.demo:
        return demo(a)
    if a.probe:
        return probe(a)
    if a.fit is not None or a.lerobot:
        return fit(a)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
