"""Dataset recorder: one task = one LeRobot episode, sampled at a fixed rate, with offscreen cameras.

Threads. The worker (server loop) calls tick() once per frame after physics. tick() decides when episodes start and
end and, on a fixed wall-clock grid (default 30 Hz, independent of the server's frame rate), takes a *snapshot*: the
low-dim frame values (joint state, action, gripper, person pose, force) plus the copies of qpos / mocap that rendering
needs. Snapshots go to one dataset thread that owns a private copy of the model, its own MjData and its own offscreen
renderer (OpenGL context), renders the cameras, streams them to ffmpeg and writes the episode files. So the server
keeps its real-time loop: the worker only pays for a few array copies per sample.

Each grid point takes the worker frame nearest to it in time (at most half a period off). When the server's loop runs
slower than the dataset rate, a grid point with no frame of its own repeats the last snapshot (counted per episode as
`repeated_frames`): timestamps stay exactly k / fps, as LeRobot requires.

Episodes. With auto recording on, every task that starts moving (status `running` with the task layer active) becomes
an episode; it ends when the task reports done (success) or failed, when another task takes over (failed,
"interrupted"), when the world is rebuilt, or after max_seconds. Failed episodes are kept and labelled
(`next.success` false, `success: false` and `outcome` in meta/episodes.jsonl). Tasks refused before moving (out of reach,
blocked) produce no frames and are only counted (`refused`). Stopping the recording drops an episode in progress.
Language: the words that asked for the task (`utterance` / `text` on the task message, or the agent's user turn), else a
sentence made from the task's arguments.

Kinematic action: without P0's q_des, a frame's action is the joint position the IK reaches at the next sample (the
command for this step), so the joint part of action[t] == state[t + 1]; the last frame repeats its own state.
"""
import copy
import os
import queue
import re
import sys
import threading
import time
import traceback
from pathlib import Path

import mujoco
import numpy as np

from .adapter import RobotAdapter
from .lerobot import LeRobotWriter

ROOT = Path(__file__).resolve().parents[2]
DATASETS = Path(os.environ.get("PC_DATASETS_DIR") or ROOT / "datasets")
NOT_EPISODES = ("follow_me", "video", "world", "demo", "teach")
MOTION = ("pick_place", "reach", "follow_path", "play_skill", "drive", "handover", "traj")
MAX_LIST = 30


def _background():
    """The dataset thread is background work: a lower scheduling priority, so on a machine with few cores its offscreen
    rendering (all CPU without a GPU: OSMesa) waits for the server's real-time loop instead of slowing it down. Linux:
    nice +10 for this thread only (and the renderer threads it starts, which inherit it); macOS: the utility QoS class."""
    try:
        if sys.platform.startswith("linux"):
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 10)   # a thread id: this thread only
        elif sys.platform == "darwin":
            import ctypes
            ctypes.CDLL(None).pthread_set_qos_class_self_np(0x11, 0)       # QOS_CLASS_UTILITY
    except Exception:
        pass


def slug(s):
    return re.sub(r"_+", "_", re.sub(r"[^a-z0-9_\-]", "_", str(s or "").strip().lower())).strip("_-")[:48]


def _where(tgt, prep=None):
    """'center' -> 'in the center of the table', 'next to the bowl' stays, 'A' -> 'on marker A'."""
    t = tgt.strip()
    if not t:
        return ""
    if re.match(r"^(left|right|front|back)$", t):
        return f"{prep or 'on'} the {t} of the table" if prep else f"on the {t} side of the table"
    if t in ("center", "centre", "middle"):
        return f"{prep or 'in'} the center of the table" if prep else "in the center of the table"
    m = re.match(r"^(left|right) of (the )?(.+)$", t)
    if m:
        return f"to the {m.group(1)} of the {m.group(3)}"
    if re.match(r"^[a-fp]$", t, re.I):
        return f"{prep or 'on'} marker {t.upper()}"
    if re.match(r"^(on|in|into|onto|to|next to|beside|near|behind|in front of|left of|right of|over|under|by)\b", t):
        return t
    return f"{prep or 'at'} {t if t.startswith('the ') else 'the ' + t}"


def describe(msg):
    """A sentence for a task message when nobody said anything (generator tasks, buttons)."""
    a = str(msg.get("action") or "task")
    obj = str(msg.get("object") or "").replace("_", " ").strip()
    tgt = str(msg.get("target") or "").replace("_", " ").strip()
    if a == "pick_place":
        what = f"the {obj}" if obj and not obj.startswith(("the ", "a ")) else (obj or "the object")
        if tgt in ("path", "my path", "along my path", "the path", "route"):
            return f"carry {what} along the drawn route"
        return f"put {what} {_where(tgt)}".strip()
    if a == "reach":
        return f"reach {_where(tgt, 'to')}" if tgt else "reach out"
    if a == "follow_path":
        return "follow the drawn path"
    if a == "play_skill":
        return f"do {str(msg.get('name') or 'the skill').replace('_', ' ')}"
    if a == "drive":
        return "drive the base"
    if a == "handover":
        return f"hand the {obj or 'object'} to the person"
    return a.replace("_", " ")


def sample_randomization(spec, rng, nlight):
    """{"lighting": strength 0..1, "camera": metres} -> concrete values for DatasetCameras.set_randomization."""
    out = {}
    s = float(spec.get("lighting") or 0.0)
    if s > 0:
        dirs = []
        for _ in range(nlight):
            az, el = rng.uniform(0, 2 * np.pi), np.radians(rng.uniform(35, 85))
            dirs.append([float(np.cos(az) * np.cos(el)), float(np.sin(az) * np.cos(el)), float(-np.sin(el))])
        out["light"] = {"dir": dirs, "scale": float(rng.uniform(1 - 0.5 * s, 1 + 0.4 * s)),
                        "ambient": float(rng.uniform(1 - 0.5 * s, 1 + 0.5 * s))}
    c = float(spec.get("camera") or 0.0)
    if c > 0:
        deg = 40.0 * c                                     # 0.05 m of jitter <-> about 2 degrees
        out["camera"] = {k: {"dp": [float(x) for x in rng.uniform(-c, c, 3) * (0.4 if k == "wrist" else 1.0)],
                             "rot": [float(x) for x in rng.uniform(-deg, deg, 2) * (0.5 if k == "wrist" else 1.0)]}
                         for k in ("front", "wrist", "robot")}
    return out


class _Job:
    __slots__ = ("kind", "ep", "values", "snap", "repeat", "model", "rt", "surface", "extra", "success", "done")

    def __init__(self, kind, **kw):
        self.kind = kind
        for k in self.__slots__[1:]:
            setattr(self, k, kw.get(k))


class DatasetRecorder:
    def __init__(self, root=None, fps=30, size=256, cameras=("front", "wrist"), shadows=False, human=True,
                 max_seconds=120.0):
        self.root = Path(root) if root else DATASETS
        self.fps, self.size, self.cams, self.shadows, self.human = int(fps), int(size), tuple(cameras), shadows, human
        self.max_seconds = float(max_seconds)
        self.recording, self.auto, self.name = False, True, None
        self.writers = {}                 # (name, robot, features) -> LeRobotWriter (dataset thread only)
        self.adapter, self.rt_id = None, None
        self.ep = None                    # worker-side episode bookkeeping
        self.pending = None               # the previous sample (kinematic action lookahead)
        self.next_t = 0.0
        self.lang = {}                    # task id -> (text, source)
        self.recorded_ids = []
        self.episodes = []                # finished episodes (summaries, newest last), this session
        self.refused = 0
        self.rand_spec, self.rand, self.rng = {}, {}, np.random.default_rng()
        self.error, self.message = None, None
        self.q = queue.Queue()
        self.lock = threading.Lock()
        self.cost = {"snapshot_ms": 0.0, "render_ms": 0.0, "n": 0}
        self.thread = None
        self.model_copy = None            # (rt id, private MjModel copy)

    # ---------------------------------------------------------------- control (worker thread)
    def start(self, ctx, name=None, auto=True, fps=None, cameras=None, size=None):
        if self.recording:
            return False, f"Already recording '{self.name}'."
        name = slug(name) or time.strftime("pc_%Y%m%d_%H%M%S")
        if fps:
            self.fps = int(np.clip(int(fps), 5, 60))
        if cameras:
            self.cams = tuple(c for c in cameras if c in ("front", "wrist", "robot")) or self.cams
        if size:
            self.size = int(np.clip(int(size), 64, 512))
        # auto: every task is an episode (the only mode so far; auto=false is accepted and pauses capturing)
        self.name, self.auto, self.recording = name, bool(auto), True
        self.error, self.episodes, self.refused, self.recorded_ids = None, [], 0, []
        self.ep, self.pending = None, None
        self._ensure_thread()
        self.message = f"Recording every task into datasets/{name}." if self.auto else f"Recording datasets/{name}, paused (auto off)."
        return True, self.message

    def stop(self, ctx):
        if not self.recording:
            return False, "Not recording."
        dropped = self.ep is not None
        if self.ep is not None:
            self._abort_episode()
        self.recording = False
        self.q.put(_Job("close", extra=self.name))
        n = len(self.episodes)
        self.message = f"Saved {n} episode{'s' if n != 1 else ''} to datasets/{self.name}." + (
            " The task still running was not kept." if dropped else "")
        return True, self.message

    def discard_last(self):
        """Drop the last finished episode (files and metadata). Runs on the dataset thread after everything queued
        before it, so the worker never waits; the result lands in state.dataset.message."""
        with self.lock:
            has = any(e.get("handle") is not None for e in self.episodes)
        if not has:
            return False, "There is no episode to discard."
        self.q.put(_Job("discard"))
        self._ensure_thread()
        return True, "Discarding the last episode."

    def set_randomization(self, spec, seed=None):
        """{"lighting": 0..1, "camera": metres, "seed": int} -> applied to the dataset cameras from the next episode."""
        self.rand_spec = {k: v for k, v in (spec or {}).items() if k in ("lighting", "camera")}
        if seed is not None:
            self.rng = np.random.default_rng(int(seed))
        return True, "Randomization: " + (", ".join(f"{k} {v}" for k, v in self.rand_spec.items()) or "off")

    def set_meta(self, data):
        """Extra JSON attached to every following episode's meta/episodes.jsonl record (the generator's variation)."""
        import json
        try:
            txt = json.dumps(data or {}, default=str)
        except (TypeError, ValueError):
            return False, "meta must be JSON."
        if len(txt) > 4000:
            return False, "meta is too large (4 kB max)."
        self.extra_meta = json.loads(txt) if data else {}
        return True, "Episode metadata set." if data else "Episode metadata cleared."

    def note_task(self, msg):
        """pre_task hook: remember the words that asked for a task (before any feature consumes the message)."""
        tid = str(msg.get("id") or "")
        if not tid:
            return
        text = next((str(msg[k]).strip() for k in ("utterance", "text", "say", "language") if str(msg.get(k) or "").strip()), "")
        src = "user" if text else None
        self.lang[tid] = (text, src, {k: v for k, v in msg.items() if k not in ("type", "id", "preview", "utterance", "text")})
        while len(self.lang) > 200:
            self.lang.pop(next(iter(self.lang)))

    # ---------------------------------------------------------------- worker tick
    def tick(self, ctx):
        if not self.recording:
            return
        now = time.time()
        rt, tasks = ctx.rt, ctx.tasks
        if id(rt) != self.rt_id:                         # a new world (robot or scene): new adapter
            if self.ep is not None:
                self._end_episode(ctx, "failed", "world_changed", "The world was rebuilt during the task.")
            self.adapter = RobotAdapter(ctx)
            self.rt_id = id(rt)
        cur = tasks.task
        if self.ep is not None:
            self._check_end(ctx, cur, now)
        if self.ep is None and self.auto and cur and cur.get("status") == "running" and tasks.kind \
                and cur.get("id") not in self.recorded_ids and cur.get("action") not in NOT_EPISODES:
            self._start_episode(ctx, cur, now)
        elif cur and cur.get("status") == "failed" and cur.get("id") not in self.recorded_ids \
                and (self.ep is None or cur.get("id") != self.ep["id"]):
            if cur.get("action") in MOTION:
                self.refused += 1                        # refused before moving: no episode
            self.recorded_ids.append(cur.get("id"))
            del self.recorded_ids[:-100]
        if self.ep is None:
            return
        half = 0.5 / self.fps                             # a frame takes the grid point nearest to it in time
        if now < self.next_t - half:
            return
        n = 0
        while self.next_t <= now + half:
            n += 1
            self.next_t += 1.0 / self.fps
        if n > self.fps:                                  # a long stall (e.g. a world build): don't fill seconds of copies
            self.ep["stalls"] += 1
            n = 1
            self.next_t = now + 1.0 / self.fps
        self._sample(ctx, n)

    def _sample(self, ctx, repeat):
        t0 = time.perf_counter()
        ad, tasks, d = self.adapter, ctx.tasks, ctx.rt.data
        vals = {"observation.state": ad.state(tasks)}
        act, from_q_des = ad.action(tasks)
        vals["action"] = act
        if self.human:
            pose = np.zeros(99, np.float32)
            h = ((ctx.store.get("watch") or {}).get("human") or {})
            P = h.get("pose") if h.get("visible") and h.get("src") != "synthetic" else None   # never the no-camera dancer
            if P is not None:
                try:
                    a = np.asarray(P, np.float32).reshape(-1)
                    if a.shape[0] == 99:
                        pose = a
                except (TypeError, ValueError):
                    pass
            vals["observation.human.pose"] = pose
        if self.ep["force"]:
            f = ad.ee_force(ctx)
            vals["observation.ee_force"] = f if f is not None else np.zeros(3 * len(ad.roles), np.float32)
        snap = (d.qpos.copy(), d.mocap_pos.copy(), d.mocap_quat.copy())
        if self.pending is not None:
            pv = self.pending
            if not pv["from_q_des"]:                      # kinematic: the command for that step is where the IK got to now
                nj = len(ad.joints)
                pv["values"]["action"][:nj] = vals["observation.state"][:nj]
            self.q.put(_Job("frame", ep=self.ep["handle"], values=pv["values"], snap=pv["snap"], repeat=pv["repeat"]))
        self.pending = {"values": vals, "snap": snap, "repeat": repeat, "from_q_des": from_q_des}
        self.ep["frames"] += repeat
        self.ep["repeated"] += repeat - 1
        self.ep["distinct"] += 1
        dt = (time.perf_counter() - t0) * 1000
        self.cost["snapshot_ms"] = 0.9 * self.cost["snapshot_ms"] + 0.1 * dt if self.cost["n"] else dt

    def _flush_pending(self):
        if self.pending is not None and self.ep is not None:
            pv = self.pending
            self.q.put(_Job("frame", ep=self.ep["handle"], values=pv["values"], snap=pv["snap"], repeat=pv["repeat"]))
        self.pending = None

    def _check_end(self, ctx, cur, now):
        ep, tasks = self.ep, ctx.tasks
        mine = cur is not None and cur.get("id") == ep["id"]
        if mine and cur.get("status") in ("done", "failed"):
            return self._end_episode(ctx, cur["status"], cur["status"], cur.get("message"))
        ref = ep["ref"]
        if ref.get("status") in ("done", "failed"):         # e.g. "stop" marks the running task failed in place
            return self._end_episode(ctx, ref["status"], ref["status"], ref.get("message"))
        if cur is not None and not mine and cur.get("status") == "running" and tasks.kind:
            return self._end_episode(ctx, "failed", "interrupted", "Another task took over before it finished.")
        if not tasks.kind:                                   # the motion is over but nobody reported it (superseded status)
            st = ref.get("status")
            return self._end_episode(ctx, st if st in ("done", "failed") else "done", "finished", ref.get("message") or "Finished.")
        if now - ep["t0"] > self.max_seconds:
            return self._end_episode(ctx, "failed", "timeout", f"Longer than {self.max_seconds:.0f} s.")

    def _start_episode(self, ctx, cur, now):
        tid = cur["id"]
        text, src, args = self.lang.get(tid, ("", None, {}))
        if not text:
            agent = ctx.store.get("agent")
            if tid.startswith("ag") and agent is not None:      # launched by the agent: the user's turn that asked
                try:
                    ev = [e for e in list(agent.events) if e.get("kind") == "user"]
                    if ev:
                        text, src = ev[-1].get("text", ""), "user"
                except Exception:
                    pass
        if not text:
            text, src = describe(dict(args, action=cur.get("action"))), "template"
        world = getattr(ctx, "world", None)
        spec = getattr(world, "spec", {}) or {}
        ad = self.adapter
        robot = ad.robot_id
        if self.model_copy is None or self.model_copy[0] != self.rt_id:
            self.model_copy = (self.rt_id, copy.deepcopy(ctx.rt.model))
        rand = sample_randomization(self.rand_spec, self.rng, self.model_copy[1].nlight) if self.rand_spec else {}
        small = lambda v: len(repr(v)) <= 400                                    # noqa: E731  (no point lists)
        meta = {"robot": robot, "action": cur.get("action"), "args": {k: v for k, v in args.items() if k != "action" and small(v)},
                "language_source": src, "scene": spec.get("template"),
                "objects": [{k: o.get(k) for k in ("name", "kind", "color", "size", "pos", "movable")} for o in spec.get("objects", [])],
                "randomization": {"spec": dict(self.rand_spec), "sampled": rand} if rand else None}
        if getattr(self, "extra_meta", None):
            meta["variation"] = self.extra_meta
        handle = {"name": self.name, "robot": robot, "task": text, "meta": meta, "features": self._features(ad), "ep": None}
        self.q.put(_Job("begin", ep=handle, model=self.model_copy[1], rt=ctx.rt,
                        surface=getattr(world, "surface", None), extra=rand))
        self.ep = {"id": tid, "ref": cur, "handle": handle, "t0": now, "frames": 0, "repeated": 0, "distinct": 0,
                   "stalls": 0, "task": text, "robot": robot, "force": ad.has_force}
        self.recorded_ids.append(tid)
        del self.recorded_ids[:-100]
        self.next_t = now
        self.pending = None
        self._sample(ctx, 1)
        self.next_t = now + 1.0 / self.fps

    def _features(self, ad):
        f = {"observation.state": {"dtype": "float32", "shape": [ad.dim], "names": list(ad.names)},
             "action": {"dtype": "float32", "shape": [ad.dim], "names": list(ad.names)}}
        for c in self.cams:
            f[f"observation.images.{c}"] = {"dtype": "video", "shape": [self.size, self.size, 3]}
        if self.human:
            from ..human.model import POSE33 as names33
            f["observation.human.pose"] = {"dtype": "float32", "shape": [99],
                                           "names": [f"{n}:{a}" for n in names33 for a in ("x", "y", "z")]}
        if ad.has_force:
            f["observation.ee_force"] = {"dtype": "float32", "shape": [3 * len(ad.roles)], "names": ad.force_names()}
        f["next.done"] = {"dtype": "bool", "shape": [1], "names": None}
        f["next.success"] = {"dtype": "bool", "shape": [1], "names": None}
        return f

    def _end_episode(self, ctx, status, outcome, message):
        ep = self.ep
        self._flush_pending()
        dur = time.time() - ep["t0"]
        extra = {"outcome": outcome, "message": str(message or "")[:300], "duration_s": round(dur, 3),
                 "repeated_frames": ep["repeated"], "distinct_states": ep["distinct"]}
        self.q.put(_Job("end", ep=ep["handle"], success=(status == "done"), extra=extra))
        with self.lock:
            self.episodes.append({"robot": ep["robot"], "task": ep["task"], "success": status == "done", "outcome": outcome,
                                  "frames": ep["frames"], "seconds": round(ep["frames"] / self.fps, 2), "index": None,
                                  "handle": ep["handle"], "message": str(message or "")[:160]})
            del self.episodes[:-500]
        self.ep = None

    def _abort_episode(self):
        self.pending = None
        self.q.put(_Job("abort", ep=self.ep["handle"]))
        self.ep = None

    # ---------------------------------------------------------------- dataset thread
    def _ensure_thread(self):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self._run, daemon=True, name="dataset")
            self.thread.start()

    def _writer(self, handle):
        """One LeRobot dataset per (recording name, robot, feature set): datasets/<name>/<robot>/."""
        sig = repr(sorted((k, v["dtype"], tuple(v["shape"])) for k, v in handle["features"].items()))
        key = (handle["name"], handle["robot"], sig)
        w = self.writers.get(key)
        if w is None:
            base = self.root / handle["name"]
            root = base / handle["robot"]
            if any(k[:2] == key[:2] for k in self.writers):  # same robot, other features (e.g. a force sensor appeared)
                root = base / f"{handle['robot']}_{len(self.writers)}"
            try:
                w = LeRobotWriter(root, self.fps, handle["features"], robot_type=handle["robot"])
            except ValueError:                           # an older dataset of that name with other features
                root = base / f"{handle['robot']}_{time.strftime('%H%M%S')}"
                w = LeRobotWriter(root, self.fps, handle["features"], robot_type=handle["robot"])
            self.writers[key] = w
        return w

    def _run(self):
        _background()
        cams, cams_model, d2 = None, None, None
        while True:
            job = self.q.get()
            try:
                if job.kind == "begin":
                    h = job.ep
                    if cams_model is not job.model:
                        if cams is not None:
                            cams.close()
                        cams = None
                        cams_model = job.model
                        if self.cams:
                            from .cameras import DatasetCameras
                            cams = DatasetCameras(job.model, job.rt, job.surface, self.size, self.shadows,
                                                  [c for c in self.cams])
                        d2 = mujoco.MjData(job.model)
                    if cams is not None:
                        cams.set_randomization(job.extra)
                    w = self._writer(h)
                    h["ep"] = w.episode(h["task"], h["meta"])
                    h["writer"] = w
                elif job.kind == "frame":
                    h = job.ep
                    if h.get("ep") is None:
                        continue
                    imgs = {}
                    if cams is not None:
                        t0 = time.perf_counter()
                        q, mp, mq = job.snap
                        d2.qpos[:] = q
                        d2.mocap_pos[:] = mp
                        d2.mocap_quat[:] = mq
                        mujoco.mj_kinematics(cams_model, d2)
                        mujoco.mj_camlight(cams_model, d2)
                        out = cams.render(d2)
                        imgs = {f"observation.images.{k}": v for k, v in out.items()}
                        dt = (time.perf_counter() - t0) * 1000
                        self.cost["render_ms"] = 0.9 * self.cost["render_ms"] + 0.1 * dt if self.cost["n"] else dt
                        self.cost["n"] += 1
                    for _ in range(max(1, int(job.repeat or 1))):
                        h["ep"].add(job.values, imgs)
                elif job.kind == "end":
                    h = job.ep
                    if h.get("ep") is None:
                        continue
                    rec = h["writer"].finalize(h["ep"], success=job.success, extra=job.extra)
                    with self.lock:
                        for e in reversed(self.episodes):
                            if e.get("handle") is h:
                                e["index"] = rec["episode_index"] if rec else None
                                e["root"] = str(h["writer"].root)
                                break
                    self.last_root = str(h["writer"].root)
                    with self.lock:
                        self.roots = sorted({*getattr(self, "roots", []), self.last_root})
                elif job.kind == "abort":
                    h = job.ep
                    if h.get("ep") is not None:
                        h["ep"].abort()
                elif job.kind == "discard":
                    with self.lock:
                        last = next((e for e in reversed(self.episodes) if e.get("index") is not None), None)
                    w = last["handle"].get("writer") if last else None
                    rec = w.discard_last() if w is not None else None
                    with self.lock:
                        if rec is not None and last is not None:
                            self.episodes.remove(last)
                            self.message = f"Discarded episode {rec['episode_index']} ({rec['tasks'][0]})."
                        else:
                            self.message = "There is no episode to discard."
                    self.discards = getattr(self, "discards", 0) + 1
                elif job.kind == "close":
                    for k in [k for k in self.writers if k[0] == job.extra]:
                        self.writers[k].close()
            except Exception as e:
                traceback.print_exc()
                self.error = f"{type(e).__name__}: {e}"[:200]

    # ---------------------------------------------------------------- state
    def busy(self):
        return self.q.qsize()

    def state(self):
        with self.lock:
            eps = [e for e in self.episodes]
        last = eps[-1] if eps else None
        pub = lambda e: {"i": e.get("index"), "robot": e["robot"], "task": e["task"], "success": e["success"],  # noqa: E731
                         "outcome": e["outcome"], "seconds": e["seconds"], "frames": e["frames"]}
        cur = None
        if self.ep is not None:
            cur = {"task": self.ep["task"], "frames": self.ep["frames"], "seconds": round(self.ep["frames"] / self.fps, 1)}
        ok = sum(1 for e in eps if e["success"])
        return {"recording": self.recording, "name": self.name, "auto": self.auto, "fps": self.fps, "size": self.size,
                "cameras": list(self.cams), "episodes": len(eps), "succeeded": ok, "refused": self.refused,
                "frames": sum(e["frames"] for e in eps) + (self.ep["frames"] if self.ep else 0),
                "current": cur, "last": pub(last) if last else None, "list": [pub(e) for e in eps[-MAX_LIST:]],
                "root": str(self.root / self.name) if self.name else None, "pending": self.busy(),
                "datasets": [r for r in getattr(self, "roots", []) if self.name and f"/{self.name}/" in r + "/"],
                "cost_ms": {"snapshot": round(self.cost["snapshot_ms"], 2), "render": round(self.cost["render_ms"], 1)},
                "randomize": self.rand_spec or None, "error": self.error, "message": self.message}
