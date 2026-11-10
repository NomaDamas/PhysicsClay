"""System identification (docs/calibration.md): fit the plant parameters so the simulator reproduces recorded rollouts.

A rollout log (pps/calib/log.py) holds the commands the robot got (q_des, gripper) and what it did (joints, objects,
wrist force). `PlantSim` replays the commands open loop in a *headless* copy of the world (no server, no IK) with a
candidate parameter vector, and `objective` scores the replay against the log:

    J(θ) = Σ_logs [ w_q · mean ‖q_sim − q‖² / σ_q²  +  w_p · mean ‖p_sim − p‖² / σ_p²  +  w_f · mean ‖f̄_sim − f̄‖² / σ_f² ]

with joint positions q (robot qpos), object positions p, and the wrist force f smoothed over 0.1 s (f̄) on both sides;
σ_q = 0.01 rad, σ_p = 0.01 m, σ_f = 1 N put the channels on comparable scales and w = 1 by default. Missing channels
(NaN in the log: no force sensor, no object tracking) drop out.

`fit` minimizes J with CMA-ES (`cma` when installed, otherwise the small CMA-ES below) in a normalized space (log space
for scale parameters), in stages that follow what each probe excites (dynamics from swing + press, mass from lift,
friction from push), then polishes everything jointly. Rollouts run in up to 2 worker processes.
"""
import math
import multiprocessing as mp
import os
import tempfile
import time
import warnings

import mujoco
import numpy as np

from .. import physics as phys
from . import params as P
from .log import Log

SIGMA = {"q": 0.01, "p": 0.01, "f": 1.0}
WEIGHTS = {"q": 1.0, "p": 1.0, "f": 1.0}
F_SMOOTH = 0.1                                    # s, moving average on the wrist force (contact chatter)
STAGES = [                                        # (label, parameters, probes whose logs score them)
    ("dynamics", ("kp_scale", "damping_scale", "latency_ms", "armature_scale", "frictionloss_scale"), ("swing", "press")),
    ("mass", ("mass_scale",), ("lift",)),
    ("friction", ("friction_scale",), ("slide",)),
]
BUDGETS = {"quick": {"gens": 10, "polish": 0}, "normal": {"gens": 18, "polish": 0}, "thorough": {"gens": 30, "polish": 8}}
MAX_WORKERS = 2


# ------------------------------------------------------------------ headless replay
def make_plant(world):
    """The plant of a world built with physics (what pps.world.runtime.make_runtime creates), without the IK runtime."""
    p = world.profile
    grips = (p.physics or {}).get("grip") or {}
    ee = {r: (f"{r}_tcp" if r in grips else s) for r, s in p.ee.items()}
    return phys.Plant(world.model, world.ik_model, p, world.kinds, world.robot_bodies, ee)


class PlantSim:
    """A headless copy of a world (robot + scene) that replays logged commands with a given parameter vector."""

    def __init__(self, robot, scene_spec):
        from ..world import scene
        self.robot, self.scene = robot, scene_spec
        self.world = scene.build(robot, scene_spec, physics=True)
        self.pl = make_plant(self.world)
        self.tcp = {r: self.pl.site_ids[r] for r in self.pl.site_ids}

    def rollout(self, log, params, tcp=False):
        """Replay `log` with `params`. -> {"q": (N, nq), "p": (N, O, 3), "f": (N, R, 3), "tcp": (N, R, 3)?, "ok": bool}"""
        pl, d = self.pl, self.pl.d
        P.apply(pl, {**P.defaults(P.PARAMS), **params})
        N = len(log)
        roles = list(log.meta["roles"]) if "roles" in log.meta else list(pl.grip)
        objs = list(log.meta["objects"]) if "objects" in log.meta else list(pl.objects)   # what the log tracked
        k0 = _rest_rows(log)
        with warnings.catch_warnings():                  # all-NaN columns (joints a dataset doesn't record)
            warnings.simplefilter("ignore", RuntimeWarning)
            q0 = np.nanmean(log.q[:k0], axis=0)
        q0 = np.where(np.isfinite(q0), q0, log.q_des[0])  # joints a dataset doesn't record: start at their command
        pl.reset(q0)
        for j, n in enumerate(objs):                     # objects where the log saw them at the start, then let them settle
            if n in pl.objects and not np.isnan(log.obj_p[0, j, 0]):
                p0 = np.nanmean(log.obj_p[:k0, j], axis=0)
                pl.set_object(n, p0, log.obj_q[0, j] / np.linalg.norm(log.obj_q[0, j]))
        for r_i, r in enumerate(roles):
            g = pl.grip.get(r)
            if g is not None:
                g["cmd"] = "close" if log.grip[0, r_i] >= 0.5 else "open"
                g["ctrl"] = g["close"] if g["cmd"] == "close" else g["open"]
        pl.q_cmd = np.array(log.q_des[0], float)
        pl.lag.clear()
        pl.acc = 0.0
        pl.settle(0.15)
        out_q = np.zeros((N, pl.nq_r))
        out_p = np.full((N, len(objs), 3), np.nan)
        out_f = np.zeros((N, len(roles), 3))
        out_t = np.zeros((N, len(roles), 3)) if tcp else None
        oidx = [(j, pl.objects[n]["qadr"]) for j, n in enumerate(objs) if n in pl.objects]
        base_ok = not np.all(np.isnan(log.base))
        ok = True
        for k in range(N):
            out_q[k] = d.qpos[:pl.nq_r]
            for j, a in oidx:
                out_p[k, j] = d.qpos[a:a + 3]
            for r_i, r in enumerate(roles):
                if r in pl.ee_force_raw:
                    out_f[k, r_i] = pl.ee_force_raw[r]
                if tcp and r in self.tcp:
                    out_t[k, r_i] = d.site_xpos[self.tcp[r]]
            if k == N - 1:
                break
            for r_i, r in enumerate(roles):
                if r in pl.grip:
                    pl.grip_cmd(r, "close" if log.grip[k, r_i] >= 0.5 else "open")
            bp = (log.base[k, :3], log.base[k, 3:]) if base_ok and not np.isnan(log.base[k, 0]) else None
            n = int(round(float(log.t[k + 1] - log.t[k]) / phys.DT))      # exactly the substeps the robot ran (no float drift)
            pl.acc = 0.0
            pl.step(n * phys.DT + 1e-9, log.q_des[k], bp)
            if not np.isfinite(d.qpos[0]) or np.abs(d.qvel).max() > 1e3:
                ok = False
                out_q[k + 1:] = np.nan
                break
        res = {"q": out_q, "p": out_p, "f": out_f, "ok": ok}
        if tcp:
            res["tcp"] = out_t
        return res

    def tcp_of(self, log):
        """Where the hand was in a log (forward kinematics of the measured joints). -> (N, R, 3)"""
        pl = self.pl
        m = pl.m
        d = mujoco.MjData(m)
        roles = log.meta.get("roles") or list(pl.grip)
        out = np.zeros((len(log), len(roles), 3))
        for k in range(len(log)):
            d.qpos[:pl.nq_r] = log.q[k]
            mujoco.mj_kinematics(m, d)
            for r_i, r in enumerate(roles):
                if r in self.tcp:
                    out[k, r_i] = d.site_xpos[self.tcp[r]]
        return out


def _rest_rows(log, sec=0.2):
    """Rows at the start of a log while the robot is still at rest (averaged for the replay's initial state)."""
    t = log.t - log.t[0]
    return max(1, int(np.searchsorted(t, sec)))


def _smooth(x, t, win=None):
    """Moving average over `win` seconds (default F_SMOOTH) along axis 0 (NaN-free input)."""
    win = F_SMOOTH if win is None else win
    if len(t) < 3:
        return x
    dt = float(np.median(np.diff(t)))
    k = max(1, int(round(win / max(dt, 1e-4))))
    if k <= 1:
        return x
    c = np.cumsum(np.concatenate([np.zeros((1,) + x.shape[1:]), x]), axis=0)
    out = np.empty_like(x)
    n = len(x)
    lo = np.clip(np.arange(n) - k // 2, 0, n)
    hi = np.clip(np.arange(n) + k - k // 2, 0, n)
    out[:] = (c[hi] - c[lo]) / (hi - lo).reshape((-1,) + (1,) * (x.ndim - 1))
    return out


def _segments(log):
    """Row -> segment id: the probe's own segments when recorded, else 0.5 s windows."""
    if log.seg is not None and np.any(log.seg >= 0):
        return np.asarray(log.seg)
    return np.floor((log.t - log.t[0]) / 0.5).astype(int)


def _segment_force_error(log, f_sim, good):
    """Wrist force compared as its mean over each probe segment (hold, slide, press...), weighted by the segment's
    length: contact forces chatter (stick-slip) differently in two equally good simulations, their averages don't."""
    seg = _segments(log)
    tot, w = 0.0, 0
    for s in np.unique(seg[good]):
        rows = good & (seg == s)
        n = int(rows.sum())
        if n < 3:
            continue
        d = f_sim[rows].mean(axis=0) - log.ee_force[rows].mean(axis=0)
        tot += n * float(np.sum(d ** 2))
        w += n
    return tot / w if w else None


def errors(log, sim):
    """Per-channel mean squared errors of a replay against its log (raw units). -> {"q": .., "p": .., "f": ..} (absent: None)"""
    out = {"q": None, "p": None, "f": None}
    if not sim.get("ok", True):
        return {"q": 1e3, "p": 1e3, "f": 1e3}
    eq = sim["q"] - log.q
    ok = np.isfinite(eq)
    out["q"] = float(np.sum(np.where(ok, eq, 0.0) ** 2) / ok.any(axis=1).sum()) if ok.any() else None
    if log.obj_p.size and not np.all(np.isnan(log.obj_p)):
        ep = sim["p"] - log.obj_p
        m = np.isfinite(ep).all(axis=2)
        out["p"] = float(np.mean(np.sum(ep[m] ** 2, axis=-1))) if m.any() else None
    if log.ee_force.size and not np.all(np.isnan(log.ee_force)):
        good = np.isfinite(log.ee_force).all(axis=(1, 2)) & np.isfinite(sim["f"]).all(axis=(1, 2))
        if good.sum() > 3:
            out["f"] = _segment_force_error(log, sim["f"], good)
    return out


def score(errs, weights=None):
    w = {**WEIGHTS, **(weights or {})}
    return float(sum(w[c] * v / SIGMA[c] ** 2 for c, v in errs.items() if v is not None))


def objective(logs, sim, params, weights=None):
    return sum(score(errors(lg, sim.rollout(lg, params)), weights) for lg in logs)


# ------------------------------------------------------------------ CMA-ES (small, self-contained)
class CMA:
    """(mu/mu_w, lambda)-CMA-ES (Hansen 2016, "The CMA Evolution Strategy: A Tutorial") on [0, 1]^n: out-of-box points
    are evaluated at the nearest point of the box plus a quadratic penalty."""

    def __init__(self, x0, sigma0, popsize=None, seed=0):
        n = self.n = len(x0)
        self.rng = np.random.default_rng(seed)
        self.lam = popsize or 4 + int(3 * math.log(n))
        self.mu = self.lam // 2
        w = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.w = w / w.sum()
        self.mueff = 1.0 / np.sum(self.w ** 2)
        self.cc = (4 + self.mueff / n) / (n + 4 + 2 * self.mueff / n)
        self.cs = (self.mueff + 2) / (n + self.mueff + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff) / ((n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0, math.sqrt((self.mueff - 1) / (n + 1)) - 1) + self.cs
        self.chin = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
        self.m = np.array(x0, float)
        self.sigma = float(sigma0)
        self.C = np.eye(n)
        self.pc, self.ps = np.zeros(n), np.zeros(n)
        self.B, self.D = np.eye(n), np.ones(n)
        self.gen = 0
        self.best = (np.inf, self.m.copy())

    def ask(self):
        z = self.rng.standard_normal((self.lam, self.n))
        self._y = z @ (self.B * self.D).T
        return self.m + self.sigma * self._y

    @staticmethod
    def penalized(x):
        xc = np.clip(x, 0.0, 1.0)
        return xc, float(np.sum((x - xc) ** 2)) * 1e3

    def tell(self, xs, fs):
        n = self.n
        idx = np.argsort(fs)
        if fs[idx[0]] < self.best[0]:
            self.best = (float(fs[idx[0]]), np.clip(xs[idx[0]], 0, 1).copy())
        y = self._y[idx[:self.mu]]
        yw = self.w @ y
        self.m = self.m + self.sigma * yw
        Cinvsqrt = self.B @ np.diag(1 / self.D) @ self.B.T
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * Cinvsqrt @ yw
        hsig = np.linalg.norm(self.ps) / math.sqrt(1 - (1 - self.cs) ** (2 * (self.gen + 1))) / self.chin < 1.4 + 2 / (n + 1)
        self.pc = (1 - self.cc) * self.pc + hsig * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * yw
        self.C = ((1 - self.c1 - self.cmu) * self.C + self.c1 * (np.outer(self.pc, self.pc) + (not hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * (y.T * self.w) @ y)
        self.sigma *= math.exp((self.cs / self.damps) * (np.linalg.norm(self.ps) / self.chin - 1))
        self.sigma = float(min(self.sigma, 0.5))
        self.C = (self.C + self.C.T) / 2
        D2, self.B = np.linalg.eigh(self.C)
        self.D = np.sqrt(np.maximum(D2, 1e-20))
        self.gen += 1

    def spread(self):
        """Per-coordinate standard deviation of the search distribution (normalized units)."""
        return self.sigma * np.sqrt(np.diag(self.C))


def _minimize(f_batch, x0, sigma0, gens, popsize=None, seed=0, on_gen=None, tol=2e-3):
    """CMA-ES on [0,1]^n. f_batch(list of x) -> list of J. Uses `cma` (pycma) when installed."""
    try:
        if len(x0) < 2:                                  # pycma does not do one dimension
            raise ImportError
        import cma  # noqa: F401  (optional)
        es = cma.CMAEvolutionStrategy(list(x0), sigma0, {"bounds": [0, 1], "popsize": popsize or 4 + int(3 * math.log(len(x0))),
                                                         "seed": seed + 1, "verbose": -9, "maxiter": gens})
        while not es.stop():
            xs = es.ask()
            es.tell(xs, f_batch([np.array(x) for x in xs]))
            if on_gen:
                on_gen(es.countiter, es.result.fbest)
        return np.array(es.result.xbest), float(es.result.fbest), np.array(es.result.stds), es.countiter
    except ImportError:
        pass
    es = CMA(x0, sigma0, popsize, seed)
    for g in range(gens):
        xs = es.ask()
        clipped = [CMA.penalized(x) for x in xs]
        fs = np.array(f_batch([c for c, _ in clipped])) + np.array([pen for _, pen in clipped])
        es.tell(xs, fs)
        if on_gen:
            on_gen(g + 1, es.best[0])
        if np.max(es.spread()) < tol:
            break
    return es.best[1], es.best[0], es.spread(), es.gen


# ------------------------------------------------------------------ evaluation in worker processes
_W = {}


def _worker_init(robot, scene_spec, log_paths):
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    _W["sim"] = PlantSim(robot, scene_spec)
    _W["logs"] = [Log.load(p) for p in log_paths]


def _worker_eval(job):
    names, fixed, x, idx, weights = job
    specs = [P.PARAMS[n] for n in names]
    prm = {**fixed, **P.from_unit(x, specs)}
    return sum(score(errors(_W["logs"][i], _W["sim"].rollout(_W["logs"][i], prm)), weights) for i in idx)


def _worker_errors(job):
    i, prm = job
    return errors(_W["logs"][i], _W["sim"].rollout(_W["logs"][i], prm))


class Evaluator:
    """Scores parameter vectors on a set of logs: in-process (workers=0) or in a pool of up to 2 processes."""

    def __init__(self, robot, scene_spec, logs, workers=MAX_WORKERS):
        self.logs, self.workers = list(logs), max(0, min(int(workers), MAX_WORKERS))
        self.robot, self.scene = robot, scene_spec
        self.evals = 0
        self.pool, self.tmp = None, None
        if self.workers:
            self.tmp = tempfile.mkdtemp(prefix="pc_calib_")
            paths = [lg.save(os.path.join(self.tmp, f"log{i}.npz")) for i, lg in enumerate(self.logs)]
            self.pool = mp.get_context("spawn").Pool(self.workers, initializer=_worker_init, initargs=(robot, scene_spec, paths))
        self.sim = PlantSim(robot, scene_spec)            # local copy: which parameters act, reports

    def errors(self, jobs):
        """[(log index, params)] -> per-channel errors of each replay."""
        self.evals += len(jobs)
        if self.pool is not None:
            return self.pool.map(_worker_errors, jobs)
        return [errors(self.logs[i], self.sim.rollout(self.logs[i], prm)) for i, prm in jobs]

    def batch(self, names, fixed, xs, idx, weights=None):
        self.evals += len(xs)
        jobs = [(list(names), dict(fixed), np.asarray(x, float), list(idx), weights) for x in xs]
        if self.pool is not None:
            return self.pool.map(_worker_eval, jobs)
        out = []
        for j in jobs:
            specs = [P.PARAMS[n] for n in j[0]]
            prm = {**j[1], **P.from_unit(j[2], specs)}
            out.append(sum(score(errors(self.logs[i], self.sim.rollout(self.logs[i], prm)), weights) for i in j[3]))
        return out

    def close(self):
        if self.pool is not None:
            self.pool.terminate()
            self.pool.join()
            self.pool = None
        if self.tmp:
            for f in os.listdir(self.tmp):
                os.remove(os.path.join(self.tmp, f))
            os.rmdir(self.tmp)
            self.tmp = None

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


# ------------------------------------------------------------------ fitting
def fit(logs, robot, scene_spec, names=None, budget="normal", workers=MAX_WORKERS, seed=0, weights=None, on_progress=None,
        stages=None, start=None):
    """Fit `names` (default: every core parameter that acts on this plant) to the logs. -> dict with params, per-param
    report, objective before / after, per-channel errors, stages, evaluations and runtime."""
    t0 = time.time()
    logs = [lg for lg in logs if len(lg) > 5]
    if not logs:
        raise ValueError("no probe logs to fit")
    bud = BUDGETS.get(budget, BUDGETS["normal"]) if isinstance(budget, str) else dict(budget)
    with Evaluator(robot, scene_spec, logs, workers) as ev:
        if names is None:
            names = P.fit_set(ev.sim.pl)
        names = [n for n in names if n in P.PARAMS]
        cur = {n: P.PARAMS[n].default for n in names}
        cur.update({k: v for k, v in (start or {}).items() if k in cur})
        probes = [lg.probe for lg in logs]
        all_idx = list(range(len(logs)))
        j0 = float(np.sum(ev.batch([], cur, [np.zeros(0)], all_idx, weights)))
        report, spread = [], {}
        plan = []
        for label, pnames, pprobes in (stages or STAGES):
            sel = [n for n in pnames if n in names]
            idx = [i for i, p in enumerate(probes) if p in pprobes]
            if sel and idx:
                plan.append((label, sel, idx, 0.3, bud["gens"]))
        leftover = [n for n in names if not any(n in s for _, s, _, _, _ in plan)]
        if leftover:                                     # parameters no probe targets: fit them on everything
            plan.append(("other", leftover, all_idx, 0.3, bud["gens"]))
        if bud.get("polish") and len(names) > 1:
            plan.append(("polish", list(names), all_idx, 0.06, bud["polish"]))
        for si, (label, sel, idx, sig, gens) in enumerate(plan):
            if label != "polish":                        # more dimensions, more generations
                gens = int(round(gens * (1 + 0.5 * (len(sel) - 1))))
            specs = [P.PARAMS[n] for n in sel]
            fixed = {k: v for k, v in cur.items() if k not in sel}
            x0 = P.to_unit(cur, specs)

            def fb(xs, sel=sel, fixed=fixed, idx=idx):
                return ev.batch(sel, fixed, xs, idx, weights)

            def on_gen(g, fbest, label=label, si=si, gens=gens):
                if on_progress:
                    on_progress({"stage": label, "stage_index": si, "stages": len(plan), "gen": g, "gens": gens, "best": fbest})
            pop = 6 if len(sel) == 1 else None
            xb, fbest, sd, ng = _minimize(fb, x0, sig, gens, pop, seed + si, on_gen)
            cur.update(P.from_unit(xb, specs))
            for s, u, x in zip(specs, sd, xb):
                lo_v, hi_v = s.from_unit(max(0.0, x - u)), s.from_unit(min(1.0, x + u))
                spread[s.name] = (hi_v - lo_v) / 2
        dflt = {n: P.PARAMS[n].default for n in names}
        errs = ev.errors([(i, dflt) for i in all_idx] + [(i, dict(cur)) for i in all_idx])
        per_log = [{"probe": lg.probe, "before": _rms(errs[i]), "after": _rms(errs[len(logs) + i])} for i, lg in enumerate(logs)]
        j1 = float(sum(score(errs[len(logs) + i], weights) for i in all_idx))
        for n in names:
            s = P.PARAMS[n]
            report.append({"name": n, "value": round(cur[n], 4), "default": s.default, "lo": s.lo, "hi": s.hi, "unit": s.unit,
                           "what": s.what, "spread": round(float(spread.get(n, float("nan"))), 4)})
        return {"params": {n: round(float(v), 4) for n, v in cur.items()}, "per_param": report, "err_before": round(j0, 4),
                "err_after": round(j1, 4), "logs": per_log, "evals": ev.evals, "stages": [p[0] for p in plan],
                "seconds": round(time.time() - t0, 1), "budget": budget if isinstance(budget, str) else "custom", "probes": probes}


def _rms(e):
    """Mean squared errors -> RMS in readable units: joints in degrees, objects in mm, force in N."""
    return {"q_deg": None if e["q"] is None else round(math.degrees(math.sqrt(e["q"])), 3),
            "p_mm": None if e["p"] is None else round(1000 * math.sqrt(e["p"]), 2),
            "f_N": None if e["f"] is None else round(math.sqrt(e["f"]), 3)}


def tracking(sim, log, params):
    """How well the simulator with `params` reproduces a log (a held-out task): RMS joint error (deg), hand path (mm),
    object path (mm), final object position (mm) and wrist force (N, segment means). A replay that diverges (MuJoCo
    instability) is scored over the part before it and says when (`diverged_s`)."""
    r = sim.rollout(log, params, tcp=True)
    n = len(log)
    if not r["ok"]:
        bad = np.where(~np.isfinite(r["q"]).all(axis=1))[0]
        n = max(2, int(bad[0])) if len(bad) else n
    lg = log.slice(0, n)
    rr = {"q": r["q"][:n], "p": r["p"][:n], "f": r["f"][:n], "ok": True}
    out = _rms(errors(lg, rr))
    real_tcp = sim.tcp_of(lg)
    out["tcp_mm"] = round(1000 * float(np.sqrt(np.nanmean(np.sum((r["tcp"][:n] - real_tcp) ** 2, axis=-1)))), 2)
    out["diverged_s"] = None if n == len(log) else round(float(log.t[n] - log.t[0]), 2)
    if n == len(log) and log.obj_p.size and not np.all(np.isnan(log.obj_p)):
        k = min(len(log), max(1, int(0.2 / max(float(np.median(np.diff(log.t))), 1e-3))))
        fin_real = np.nanmean(log.obj_p[-k:], axis=0)
        out["final_obj_mm"] = round(1000 * float(np.nanmax(np.linalg.norm(r["p"][-1] - fin_real, axis=-1))), 2)
    return out
