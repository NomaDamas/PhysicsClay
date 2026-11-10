"""Rollout logs: what system identification fits (docs/calibration.md, "Log format").

A log is one probe (or one task) on one robot: per control tick k, the state measured at time t[k] and the command
sent right after reading it. Saved as `.npz` with these keys (N ticks, nq robot qpos entries, nv robot dofs,
R gripper roles, O movable objects):

    t         (N,)        seconds (robot clock); the command of row k is held until t[k+1]
    q_des     (N, nq)     commanded joint positions (the action; P1's `action` without the gripper)
    grip      (N, R)      gripper command per role, 0 open .. 1 close
    q         (N, nq)     measured joint positions (P1's `observation.state` without the gripper)
    dq        (N, nv)     measured joint velocities (NaN when not measured)
    ee_force  (N, R, 3)   wrist force on the hand, world frame, N (NaN when there is no force sensor)
    obj_p     (N, O, 3)   object positions, m (NaN when not observed)
    obj_q     (N, O, 4)   object orientations, wxyz (NaN when not observed)
    base      (N, 7)      planned pose of a welded floating base (legged bodies), else NaN
    seg       (N,)        index of the probe segment (approach, hold, slide...) the row belongs to; -1 unknown
    meta      JSON string {"version": 1, "robot", "scene", "probe", "roles", "objects", "joints", "source", ...}

`from_lerobot(path)` reads a P1 LeRobot v2.1 episode parquet (`observation.state`, `action`, `timestamp`, optional
`observation.ee_force` and `observation.objects`) into the same structure (needs pyarrow).
"""
import json
from dataclasses import dataclass, field

import numpy as np

ARRAYS = ("t", "q_des", "grip", "q", "dq", "ee_force", "obj_p", "obj_q", "base", "seg")


@dataclass
class Log:
    t: np.ndarray
    q_des: np.ndarray
    grip: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    ee_force: np.ndarray
    obj_p: np.ndarray
    obj_q: np.ndarray
    base: np.ndarray
    seg: np.ndarray = None
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.seg is None:
            self.seg = np.full(len(self.t), -1, int)

    def __len__(self):
        return len(self.t)

    @property
    def probe(self):
        return self.meta.get("probe", "log")

    @property
    def duration(self):
        return float(self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    def save(self, path):
        np.savez_compressed(path, **{k: getattr(self, k) for k in ARRAYS}, meta=np.array(json.dumps(self.meta)))
        return path

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as z:
            arrs = {k: np.array(z[k]) for k in ARRAYS if k in z.files}
            meta = json.loads(str(z["meta"]))
        return cls(**arrs, meta=meta)

    def slice(self, i0, i1):
        return Log(**{k: getattr(self, k)[i0:i1].copy() for k in ARRAYS}, meta=dict(self.meta))


class Recorder:
    """Collects rows while a probe runs (pps/calib/probes.py) and turns them into a Log."""

    def __init__(self, meta):
        self.meta = dict(meta)
        self.rows = []

    def add(self, state, q_des, grip, base=None, seg=-1):
        roles, objs = self.meta["roles"], self.meta["objects"]
        f = state.get("ee_force") or {}
        o = state.get("objects") or {}
        nan3, nan4 = np.full(3, np.nan), np.full(4, np.nan)
        self.rows.append((float(state["t"]), np.array(q_des, float), np.array([float(grip.get(r, 0.0)) for r in roles]),
                          np.array(state["q"], float), np.array(state.get("dq") if state.get("dq") is not None else [], float),
                          np.array([f.get(r, nan3) if f.get(r) is not None else nan3 for r in roles], float).reshape(len(roles), 3),
                          np.array([o[n]["p"] if n in o else nan3 for n in objs], float).reshape(len(objs), 3),
                          np.array([o[n]["q"] if n in o else nan4 for n in objs], float).reshape(len(objs), 4),
                          np.full(7, np.nan) if base is None else np.concatenate([base[0], base[1]]).astype(float), int(seg)))

    def log(self):
        if not self.rows:
            return None
        cols = list(zip(*self.rows))
        nv = max(len(c) for c in cols[4])
        dq = np.array([c if len(c) == nv else np.full(nv, np.nan) for c in cols[4]], float)
        return Log(t=np.array(cols[0]), q_des=np.array(cols[1]), grip=np.array(cols[2]), q=np.array(cols[3]), dq=dq,
                   ee_force=np.array(cols[5]), obj_p=np.array(cols[6]), obj_q=np.array(cols[7]), base=np.array(cols[8]),
                   seg=np.array(cols[9], int), meta=dict(self.meta))


def from_lerobot(path, nq, roles=("right",), objects=(), meta=None):
    """A P1 LeRobot episode (parquet) -> Log. `observation.state` / `action` are [robot qpos (nq), gripper per role];
    object poses are read from `observation.objects` ([O*7] per row: p, q per object) when the dataset has them.
    Without object poses or a force column only the joint terms of the fit are informative (kp, damping, latency)."""
    import pyarrow.parquet as pq                      # optional dependency (P1 datasets)
    tab = pq.read_table(path).to_pydict()
    state = np.array(tab["observation.state"], float)
    action = np.array(tab["action"], float)
    t = np.array(tab.get("timestamp") or np.arange(len(state)) / 30.0, float)
    R, O, N = len(roles), len(objects), len(state)
    grip = action[:, nq:nq + R] if action.shape[1] >= nq + R else np.zeros((N, R))
    f = np.full((N, R, 3), np.nan)
    if "observation.ee_force" in tab:
        ef = np.array(tab["observation.ee_force"], float).reshape(N, -1)
        f[:, :, :] = ef[:, :R * 3].reshape(N, R, 3)
    p, q = np.full((N, O, 3), np.nan), np.full((N, O, 4), np.nan)
    if O and "observation.objects" in tab:
        ob = np.array(tab["observation.objects"], float).reshape(N, O, 7)
        p, q = ob[:, :, :3], ob[:, :, 3:]
    m = {"version": 1, "probe": "dataset", "source": str(path), "roles": list(roles), "objects": list(objects), **(meta or {})}
    return Log(t=t - t[0], q_des=action[:, :nq], grip=grip, q=state[:, :nq], dq=np.full((N, 0), np.nan), ee_force=f,
               obj_p=p, obj_q=q, base=np.full((N, 7), np.nan), meta=m)
