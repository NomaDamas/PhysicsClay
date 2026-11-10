"""LeRobot v2.1 dataset writer (Hugging Face LeRobot format), without importing lerobot.

Layout under `root` (one dataset = one robot body; state and action sizes are per embodiment, as in LeRobot / OXE):

    meta/info.json             codebase_version, robot_type, fps, features, totals, chunks_size, data_path, video_path, splits
    meta/tasks.jsonl           {"task_index": 0, "task": "put the red cup next to the bowl"}
    meta/episodes.jsonl        {"episode_index": 0, "tasks": ["..."], "length": 212, "success": true, ...extra metadata}
    meta/episodes_stats.jsonl  {"episode_index": 0, "stats": {feature: {"min", "max", "mean", "std", "count"}}}   (v2.1)
    meta/stats.json            the same stats aggregated over all episodes (what v2.0 readers load)
    data/chunk-000/episode_000000.parquet
    videos/chunk-000/<video key>/episode_000000.mp4   (h264, yuv420p, one file per camera per episode)

Every frame row has the user features (vectors as fixed-size float lists, scalars as plain columns) plus timestamp
(= frame_index / fps exactly, which LeRobot checks), frame_index, episode_index, index (global) and task_index. When the
features include `next.done` / `next.success`, the writer sets them: false on every frame, true on the last one
(`next.success` only when the episode succeeded). Video features are not stored in the parquet; their frames go to the
mp4 (streamed to ffmpeg while recording, so memory stays flat). Image stats are per channel in [0, 1], shape [3, 1, 1].

Usage (one thread records, finalize may run on another, one at a time):

    w = LeRobotWriter(root, fps=30, features={"observation.state": {"dtype": "float32", "shape": [8], "names": [...]},
                                              "observation.images.front": {"dtype": "video", "shape": [256, 256, 3]}})
    ep = w.episode("pick up the cup", meta={"robot": "panda"})
    ep.add({"observation.state": q, ...}, {"observation.images.front": rgb})
    rec = w.finalize(ep, success=True)      # or ep.abort()
    w.discard_last()
"""
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time
import uuid
from pathlib import Path

import numpy as np

CODEBASE_VERSION = "v2.1"
CHUNKS_SIZE = 1000
DATA_PATH = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
VIDEO_PATH = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
DEFAULT_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": [1], "names": None},
    "frame_index": {"dtype": "int64", "shape": [1], "names": None},
    "episode_index": {"dtype": "int64", "shape": [1], "names": None},
    "index": {"dtype": "int64", "shape": [1], "names": None},
    "task_index": {"dtype": "int64", "shape": [1], "names": None},
}
DTYPES = {"float32": np.float32, "float64": np.float64, "int64": np.int64, "int32": np.int32, "bool": np.bool_}
IMAGE_STATS_EVERY = 3        # image stats from every 3rd frame, 4x downsampled (LeRobot also samples)


def ffmpeg_exe():
    """The ffmpeg binary: imageio-ffmpeg's bundled one (a setup.sh dependency), else the one on PATH."""
    try:
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
        if exe and os.path.exists(exe):
            return exe
    except Exception:
        pass
    exe = shutil.which("ffmpeg")
    if not exe:
        raise RuntimeError("ffmpeg not found: install imageio-ffmpeg (scripts/setup.sh) or ffmpeg")
    return exe


# ------------------------------------------------------------------ video
class VideoEncoder:
    """RGB frames -> h264 mp4 through an ffmpeg pipe. write() only queues (the caller's loop stays real time); a
    background thread feeds ffmpeg and accumulates per-channel image stats."""

    def __init__(self, path, width, height, fps, crf=23, gop=2, preset="veryfast"):
        self.path, self.w, self.h = Path(path), int(width), int(height)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y",
               "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{self.w}x{self.h}", "-r", str(fps), "-i", "-",
               "-c:v", "libx264", "-preset", preset, "-crf", str(crf), "-g", str(gop), "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(self.path)]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.q = queue.Queue(maxsize=240)
        self.frames = 0
        self.error = None
        self.st = {"n": 0, "min": np.full(3, np.inf), "max": np.full(3, -np.inf), "sum": np.zeros(3), "sq": np.zeros(3), "px": 0}
        self.thread = threading.Thread(target=self._run, daemon=True, name=f"enc-{self.path.stem}")
        self.thread.start()

    def write(self, rgb):
        if self.error:
            return
        a = np.ascontiguousarray(rgb, dtype=np.uint8)
        if a.shape != (self.h, self.w, 3):
            raise ValueError(f"frame shape {a.shape} != {(self.h, self.w, 3)}")
        self.q.put(a)
        self.frames += 1

    def _run(self):
        k = 0
        while True:
            a = self.q.get()
            if a is None:
                break
            if k % IMAGE_STATS_EVERY == 0:
                s = a[::4, ::4].reshape(-1, 3).astype(np.float64) / 255.0
                st = self.st
                st["n"] += 1
                st["px"] += s.shape[0]
                st["min"] = np.minimum(st["min"], s.min(0))
                st["max"] = np.maximum(st["max"], s.max(0))
                st["sum"] += s.sum(0)
                st["sq"] += (s * s).sum(0)
            k += 1
            if self.error:
                continue
            try:
                self.proc.stdin.write(a.tobytes())
            except (BrokenPipeError, OSError) as e:
                self.error = f"ffmpeg: {e}"

    def close(self, timeout=60):
        """Flush and wait for ffmpeg. -> True when the mp4 is complete."""
        self.q.put(None)
        self.thread.join(timeout)
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.error = self.error or "ffmpeg timed out"
        if self.proc.returncode not in (0, None) and not self.error:
            err = (self.proc.stderr.read() or b"").decode(errors="replace")[-300:]
            self.error = f"ffmpeg exit {self.proc.returncode}: {err}"
        return self.error is None and self.path.exists()

    def abort(self):
        self.q.put(None)
        try:
            self.proc.kill()
        except OSError:
            pass
        self.thread.join(5)
        try:
            self.path.unlink()
        except OSError:
            pass

    def stats(self):
        st = self.st
        if not st["px"]:
            z = [[[0.0]]] * 3
            return {"min": z, "max": z, "mean": z, "std": z, "count": [0]}
        mean = st["sum"] / st["px"]
        std = np.sqrt(np.maximum(st["sq"] / st["px"] - mean ** 2, 0.0))
        c = lambda v: [[[float(x)]] for x in v]          # noqa: E731  (3, 1, 1) like LeRobot
        return {"min": c(st["min"]), "max": c(st["max"]), "mean": c(mean), "std": c(std), "count": [int(st["n"])]}


# ------------------------------------------------------------------ stats
def array_stats(a):
    """(N, D) -> {min, max, mean, std, count} with D-long lists (LeRobot per-episode stats)."""
    a = np.asarray(a, np.float64)
    if a.ndim == 1:
        a = a[:, None]
    return {"min": a.min(0).tolist(), "max": a.max(0).tolist(), "mean": a.mean(0).tolist(), "std": a.std(0).tolist(),
            "count": [int(a.shape[0])]}


def aggregate_stats(stats_list):
    """Pool per-episode stats into dataset stats (weighted mean, pooled variance), like lerobot's aggregate_stats."""
    out = {}
    keys = {k for s in stats_list for k in s}
    for k in sorted(keys):
        parts = [s[k] for s in stats_list if k in s and s[k]["count"][0] > 0]
        if not parts:
            continue
        n = np.array([p["count"][0] for p in parts], np.float64)
        means = [np.asarray(p["mean"], np.float64) for p in parts]
        stds = [np.asarray(p["std"], np.float64) for p in parts]
        w = n / n.sum()
        mean = sum(wi * m for wi, m in zip(w, means))
        var = sum(wi * (s ** 2 + (m - mean) ** 2) for wi, s, m in zip(w, stds, means))
        out[k] = {"min": np.min([np.asarray(p["min"]) for p in parts], 0).tolist(),
                  "max": np.max([np.asarray(p["max"]) for p in parts], 0).tolist(),
                  "mean": mean.tolist(), "std": np.sqrt(np.maximum(var, 0)).tolist(), "count": [int(n.sum())]}
    return out


# ------------------------------------------------------------------ files
def _write_json(path, obj):
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(obj, indent=4, ensure_ascii=False))
    os.replace(tmp, path)


def _write_jsonl(path, rows):
    tmp = Path(str(path) + ".tmp")
    tmp.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rows))
    os.replace(tmp, path)


def read_jsonl(path):
    p = Path(path)
    if not p.exists():
        return []
    return [json.loads(line) for line in p.read_text().splitlines() if line.strip()]


def _hf_feature(spec):
    """Hugging Face `datasets` feature JSON for the parquet schema metadata (so `datasets` restores Sequence/Value)."""
    dt = spec["dtype"]
    shape = list(spec["shape"])
    if shape == [1]:
        return {"dtype": dt, "_type": "Value"}
    return {"feature": {"dtype": dt, "_type": "Value"}, "length": int(shape[0]), "_type": "Sequence"}


class Episode:
    """One episode being recorded: low-dim rows in memory, camera frames streamed to ffmpeg."""

    def __init__(self, writer, task, meta):
        self.w = writer
        self.task = str(task or "").strip() or "do the task"
        self.meta = dict(meta or {})
        self.uid = uuid.uuid4().hex[:10]
        self.rows = {k: [] for k in writer.low_keys}
        self.n = 0
        self.t0 = time.time()
        self.enc = {}
        for k in writer.video_keys:
            h, wd = writer.features[k]["shape"][:2]
            self.enc[k] = VideoEncoder(writer.tmp_dir / f"{self.uid}_{k}.mp4", wd, h, writer.fps, crf=writer.crf, gop=writer.gop)
        self.closed = False

    def add(self, values, images=None):
        """values: {feature: array-like} for every low-dim user feature (next.done / next.success are filled in by the
        writer and may be omitted); images: {video key: uint8 (H, W, 3)}."""
        for k in self.w.low_keys:
            if k in ("next.done", "next.success") and k not in values:
                v = False
            else:
                v = values[k]
            self.rows[k].append(np.asarray(v, dtype=DTYPES[self.w.features[k]["dtype"]]).reshape(-1))
        for k in self.w.video_keys:
            self.enc[k].write(images[k])
        self.n += 1

    def abort(self):
        if self.closed:
            return
        self.closed = True
        for e in self.enc.values():
            e.abort()


class LeRobotWriter:
    def __init__(self, root, fps, features, robot_type=None, crf=23, gop=2):
        self.root = Path(root)
        self.fps = int(fps)
        self.robot_type = robot_type
        self.crf, self.gop = crf, gop
        feats = {}
        for k, v in features.items():
            if k in DEFAULT_FEATURES:
                continue
            spec = {"dtype": v["dtype"], "shape": [int(x) for x in v["shape"]], "names": v.get("names")}
            if spec["dtype"] == "video":
                h, w = spec["shape"][:2]
                spec["shape"] = [h, w, 3]
                spec["names"] = ["height", "width", "channels"]
                spec["info"] = {"video.height": h, "video.width": w, "video.codec": "h264", "video.pix_fmt": "yuv420p",
                                "video.is_depth_map": False, "video.fps": self.fps, "video.channels": 3, "has_audio": False}
            elif spec["dtype"] not in DTYPES:
                raise ValueError(f"unsupported dtype {spec['dtype']} for {k}")
            elif len(spec["shape"]) != 1:
                raise ValueError(f"{k}: only 1-D features are supported (flatten it), got shape {spec['shape']}")
            feats[k] = spec
        self.features = {**feats, **DEFAULT_FEATURES}
        self.video_keys = [k for k, v in feats.items() if v["dtype"] == "video"]
        self.low_keys = [k for k, v in feats.items() if v["dtype"] != "video"]
        self.meta_dir = self.root / "meta"
        self.tmp_dir = self.root / ".tmp"
        self.lock = threading.Lock()
        self.episodes, self.ep_stats, self.tasks = [], [], {}
        self.total_frames = 0
        info_p = self.meta_dir / "info.json"
        if info_p.exists():                               # resume an existing dataset (same features only)
            info = json.loads(info_p.read_text())
            if not self._same_features(info.get("features", {})) or int(info.get("fps", 0)) != self.fps:
                raise ValueError(f"{self.root} already holds a dataset with other features or fps; choose another name")
            self.episodes = read_jsonl(self.meta_dir / "episodes.jsonl")
            self.ep_stats = read_jsonl(self.meta_dir / "episodes_stats.jsonl")
            self.tasks = {r["task"]: int(r["task_index"]) for r in read_jsonl(self.meta_dir / "tasks.jsonl")}
            self.total_frames = int(info.get("total_frames", 0))
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        if self.tmp_dir.exists():
            shutil.rmtree(self.tmp_dir, ignore_errors=True)   # leftovers of a crashed recording
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
        self._write_meta()

    def _same_features(self, other):
        def canon(fs):
            return {k: (v.get("dtype"), list(v.get("shape") or []), v.get("names")) for k, v in fs.items()}
        return canon(other) == canon(self.features)

    # ---------------------------------------------------------------- episodes
    @property
    def total_episodes(self):
        return len(self.episodes)

    def episode(self, task, meta=None):
        return Episode(self, task, meta)

    def finalize(self, ep, success=None, extra=None):
        """Close the episode's videos and write its parquet + metadata. -> the episodes.jsonl record (or None when the
        episode had no frames or a video failed)."""
        import pyarrow as pa
        import pyarrow.parquet as pq
        if ep.closed:
            return None
        ep.closed = True
        if ep.n == 0:
            for e in ep.enc.values():
                e.abort()
            return None
        oks = {k: e.close() for k, e in ep.enc.items()}
        if not all(oks.values()):
            bad = {k: ep.enc[k].error for k, ok in oks.items() if not ok}
            for e in ep.enc.values():
                try:
                    e.path.unlink()
                except OSError:
                    pass
            raise RuntimeError(f"video encoding failed: {bad}")
        with self.lock:
            idx = len(self.episodes)
            chunk = idx // CHUNKS_SIZE
            if ep.task not in self.tasks:
                self.tasks[ep.task] = len(self.tasks)
            tix = self.tasks[ep.task]
            n = ep.n
            cols, stats = {}, {}
            for k in self.low_keys:
                a = np.stack(ep.rows[k])
                if k == "next.done":
                    a[:] = False
                    a[-1] = True
                elif k == "next.success":
                    a[:] = False
                    a[-1] = bool(success)
                cols[k] = a
            cols["timestamp"] = (np.arange(n) / self.fps).astype(np.float32)[:, None]
            cols["frame_index"] = np.arange(n, dtype=np.int64)[:, None]
            cols["episode_index"] = np.full((n, 1), idx, np.int64)
            cols["index"] = (self.total_frames + np.arange(n, dtype=np.int64))[:, None]
            cols["task_index"] = np.full((n, 1), tix, np.int64)
            arrays, fields = [], []
            for k in self.low_keys + list(DEFAULT_FEATURES):
                spec, a = self.features[k], cols[k]
                stats[k] = array_stats(a.astype(np.float64))
                pt = {"float32": pa.float32(), "float64": pa.float64(), "int64": pa.int64(), "int32": pa.int32(),
                      "bool": pa.bool_()}[spec["dtype"]]
                if spec["shape"] == [1]:
                    arr = pa.array(a[:, 0], type=pt)
                else:
                    arr = pa.FixedSizeListArray.from_arrays(pa.array(a.reshape(-1), type=pt), int(spec["shape"][0]))
                arrays.append(arr)
                fields.append(pa.field(k, arr.type))
            hf = {"info": {"features": {k: _hf_feature(self.features[k]) for k in self.low_keys + list(DEFAULT_FEATURES)}}}
            schema = pa.schema(fields, metadata={"huggingface": json.dumps(hf)})
            table = pa.Table.from_arrays(arrays, schema=schema)
            dp = self.root / DATA_PATH.format(episode_chunk=chunk, episode_index=idx)
            dp.parent.mkdir(parents=True, exist_ok=True)
            pq.write_table(table, dp)
            for k, e in ep.enc.items():
                vp = self.root / VIDEO_PATH.format(episode_chunk=chunk, video_key=k, episode_index=idx)
                vp.parent.mkdir(parents=True, exist_ok=True)
                os.replace(e.path, vp)
                stats[k] = e.stats()
            rec = {"episode_index": idx, "tasks": [ep.task], "length": n}
            if success is not None:
                rec["success"] = bool(success)
            rec.update(ep.meta)
            rec.update(extra or {})
            self.episodes.append(rec)
            self.ep_stats.append({"episode_index": idx, "stats": stats})
            self.total_frames += n
            self._write_meta()
            return rec

    def discard_last(self):
        """Remove the last episode (files + metadata). -> its record, or None when the dataset is empty."""
        with self.lock:
            if not self.episodes:
                return None
            rec = self.episodes.pop()
            idx = rec["episode_index"]
            self.ep_stats = [s for s in self.ep_stats if s["episode_index"] != idx]
            chunk = idx // CHUNKS_SIZE
            try:
                (self.root / DATA_PATH.format(episode_chunk=chunk, episode_index=idx)).unlink()
            except OSError:
                pass
            for k in self.video_keys:
                try:
                    (self.root / VIDEO_PATH.format(episode_chunk=chunk, video_key=k, episode_index=idx)).unlink()
                except OSError:
                    pass
            self.total_frames -= int(rec["length"])
            used = {t for r in self.episodes for t in r.get("tasks", [])}
            for t in list(self.tasks):                     # drop trailing tasks nothing uses any more (indices stay dense)
                if t not in used and self.tasks[t] == len(self.tasks) - 1:
                    del self.tasks[t]
            self._write_meta()
            return rec

    # ---------------------------------------------------------------- meta
    def info(self):
        n = len(self.episodes)
        return {
            "codebase_version": CODEBASE_VERSION,
            "robot_type": self.robot_type,
            "total_episodes": n,
            "total_frames": int(self.total_frames),
            "total_tasks": len(self.tasks),
            "total_videos": n * len(self.video_keys),
            "total_chunks": max(1, math.ceil(n / CHUNKS_SIZE)),
            "chunks_size": CHUNKS_SIZE,
            "fps": self.fps,
            "splits": {"train": f"0:{n}"},
            "data_path": DATA_PATH,
            "video_path": VIDEO_PATH if self.video_keys else None,
            "features": self.features,
        }

    def _write_meta(self):
        _write_jsonl(self.meta_dir / "episodes.jsonl", self.episodes)
        _write_jsonl(self.meta_dir / "episodes_stats.jsonl", self.ep_stats)
        _write_jsonl(self.meta_dir / "tasks.jsonl", [{"task_index": i, "task": t} for t, i in sorted(self.tasks.items(), key=lambda x: x[1])])
        _write_json(self.meta_dir / "stats.json", aggregate_stats([s["stats"] for s in self.ep_stats]))
        _write_json(self.meta_dir / "info.json", self.info())

    def close(self):
        shutil.rmtree(self.tmp_dir, ignore_errors=True)
