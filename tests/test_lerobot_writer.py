"""Unit tests for pps/data/lerobot.py (v1 P1): write a tiny LeRobot v2.1 dataset to a temp dir and read it back with
pyarrow + ffmpeg only (no lerobot import, no server, no MuJoCo).

    .venv/bin/python -m pytest tests/test_lerobot_writer.py -q
"""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

pq = pytest.importorskip("pyarrow.parquet")
from pps.data import lerobot as L  # noqa: E402

FPS, N_J, SIZE = 10, 3, 32
NAMES = ["joint:a", "joint:b", "gripper:right"]
FEATURES = {
    "observation.state": {"dtype": "float32", "shape": [N_J], "names": NAMES},
    "action": {"dtype": "float32", "shape": [N_J], "names": NAMES},
    "observation.images.front": {"dtype": "video", "shape": [SIZE, SIZE, 3]},
    "next.done": {"dtype": "bool", "shape": [1], "names": None},
    "next.success": {"dtype": "bool", "shape": [1], "names": None},
}


def frames(ep, n, k):
    for i in range(n):
        q = np.array([i, -i, k], np.float32) + 0.5
        img = np.zeros((SIZE, SIZE, 3), np.uint8)
        img[..., 0] = 40 * k + i % 7  # per-episode colour, so image stats differ
        img[: SIZE // 2, :, 1] = 200
        ep.add({"observation.state": q, "action": q + 1}, {"observation.images.front": img})


def decode_frames(path):
    out = subprocess.run(
        [L.ffmpeg_exe(), "-v", "error", "-i", str(path), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
        capture_output=True,
        check=True,
    ).stdout
    return len(out) // (SIZE * SIZE * 3)


@pytest.fixture()
def ds(tmp_path):
    w = L.LeRobotWriter(tmp_path / "toy" / "panda", FPS, FEATURES, robot_type="panda")
    e0 = w.episode("put the red cup next to the bowl", {"robot": "panda", "scene": "table"})
    frames(e0, 12, 0)
    r0 = w.finalize(e0, success=True, extra={"outcome": "done"})
    e1 = w.episode("put the red cup next to the bowl")
    frames(e1, 7, 1)
    r1 = w.finalize(e1, success=False, extra={"outcome": "failed"})
    e2 = w.episode("reach to the left")
    frames(e2, 5, 2)
    r2 = w.finalize(e2, success=True)
    return w, (r0, r1, r2)


def test_info_json(ds):
    w, _ = ds
    info = json.loads((w.root / "meta" / "info.json").read_text())
    assert info["codebase_version"] == "v2.1"
    assert info["fps"] == FPS and info["robot_type"] == "panda"
    assert info["total_episodes"] == 3 and info["total_frames"] == 24 and info["total_tasks"] == 2
    assert info["total_videos"] == 3 and info["total_chunks"] == 1 and info["chunks_size"] == 1000
    assert info["splits"] == {"train": "0:3"}
    assert info["data_path"] == "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
    assert info["video_path"] == "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
    f = info["features"]
    assert f["observation.state"] == {"dtype": "float32", "shape": [N_J], "names": NAMES}
    v = f["observation.images.front"]
    assert v["dtype"] == "video" and v["shape"] == [SIZE, SIZE, 3] and v["names"] == ["height", "width", "channels"]
    assert (
        v["info"]["video.codec"] == "h264" and v["info"]["video.pix_fmt"] == "yuv420p" and v["info"]["video.fps"] == FPS
    )
    for k in ("timestamp", "frame_index", "episode_index", "index", "task_index"):
        assert k in f and f[k]["shape"] == [1]


def test_parquet_rows(ds):
    w, (r0, r1, r2) = ds
    assert (r0["episode_index"], r1["episode_index"], r2["episode_index"]) == (0, 1, 2)
    start = 0
    for rec, n in ((r0, 12), (r1, 7), (r2, 5)):
        p = w.root / L.DATA_PATH.format(episode_chunk=0, episode_index=rec["episode_index"])
        t = pq.read_table(p)
        assert t.num_rows == n == rec["length"]
        cols = t.column_names
        assert "observation.images.front" not in cols  # videos live in mp4 files
        st = np.array(t.column("observation.state").to_pylist(), np.float32)
        assert st.shape == (n, N_J)
        np.testing.assert_allclose(np.array(t.column("action").to_pylist()), st + 1)
        ts = np.array(t.column("timestamp").to_pylist())
        np.testing.assert_allclose(ts, np.arange(n) / FPS, atol=1e-6)  # LeRobot checks timestamps == frame_index / fps
        assert t.column("frame_index").to_pylist() == list(range(n))
        assert set(t.column("episode_index").to_pylist()) == {rec["episode_index"]}
        assert t.column("index").to_pylist() == list(range(start, start + n))
        done = t.column("next.done").to_pylist()
        succ = t.column("next.success").to_pylist()
        assert done == [False] * (n - 1) + [True]
        assert succ == [False] * (n - 1) + [bool(rec["success"])]
        hf = json.loads(t.schema.metadata[b"huggingface"])
        assert hf["info"]["features"]["observation.state"]["length"] == N_J
        start += n
    t1 = pq.read_table(w.root / L.DATA_PATH.format(episode_chunk=0, episode_index=1))
    t2 = pq.read_table(w.root / L.DATA_PATH.format(episode_chunk=0, episode_index=2))
    assert set(t1.column("task_index").to_pylist()) == {0} and set(t2.column("task_index").to_pylist()) == {1}


def test_jsonl_and_videos(ds):
    w, _ = ds
    eps = L.read_jsonl(w.root / "meta" / "episodes.jsonl")
    assert [e["episode_index"] for e in eps] == [0, 1, 2]
    assert [e["length"] for e in eps] == [12, 7, 5]
    assert [e["success"] for e in eps] == [True, False, True]
    assert eps[0]["tasks"] == ["put the red cup next to the bowl"] and eps[0]["scene"] == "table"
    tasks = L.read_jsonl(w.root / "meta" / "tasks.jsonl")
    assert tasks == [
        {"task_index": 0, "task": "put the red cup next to the bowl"},
        {"task_index": 1, "task": "reach to the left"},
    ]
    for e in eps:
        vp = w.root / L.VIDEO_PATH.format(
            episode_chunk=0, video_key="observation.images.front", episode_index=e["episode_index"]
        )
        assert vp.exists() and decode_frames(vp) == e["length"]
    assert not any((w.root / ".tmp").glob("*.mp4"))


def test_stats(ds):
    w, _ = ds
    es = L.read_jsonl(w.root / "meta" / "episodes_stats.jsonl")
    assert [s["episode_index"] for s in es] == [0, 1, 2]
    s0 = es[0]["stats"]
    assert s0["observation.state"]["min"] == [0.5, -10.5, 0.5] and s0["observation.state"]["max"] == [11.5, 0.5, 0.5]
    assert s0["observation.state"]["count"] == [12]
    img = s0["observation.images.front"]
    assert np.array(img["mean"]).shape == (3, 1, 1) and 0.0 <= img["min"][1][0][0] <= img["max"][1][0][0] <= 1.0
    assert abs(img["mean"][1][0][0] - 100 / 255) < 0.05  # half the green channel at 200
    agg = json.loads((w.root / "meta" / "stats.json").read_text())
    allq = np.concatenate([np.array([[i, -i, k] for i in range(n)], float) + 0.5 for n, k in ((12, 0), (7, 1), (5, 2))])
    np.testing.assert_allclose(agg["observation.state"]["mean"], allq.mean(0), atol=1e-5)
    np.testing.assert_allclose(agg["observation.state"]["std"], allq.std(0), atol=1e-5)
    assert agg["observation.state"]["min"] == allq.min(0).tolist() and agg["observation.state"]["count"] == [24]


def test_discard_last_and_resume(ds):
    w, _ = ds
    rec = w.discard_last()
    assert rec["episode_index"] == 2
    info = json.loads((w.root / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 2 and info["total_frames"] == 19 and info["total_tasks"] == 1
    assert not (w.root / L.DATA_PATH.format(episode_chunk=0, episode_index=2)).exists()
    assert len(L.read_jsonl(w.root / "meta" / "episodes_stats.jsonl")) == 2
    w2 = L.LeRobotWriter(w.root, FPS, FEATURES, robot_type="panda")  # reopen: appends after the last episode
    e = w2.episode("reach to the right")
    frames(e, 4, 3)
    r = w2.finalize(e, success=True)
    assert r["episode_index"] == 2
    t = pq.read_table(w.root / L.DATA_PATH.format(episode_chunk=0, episode_index=2))
    assert t.column("index").to_pylist() == list(range(19, 23))
    with pytest.raises(ValueError):
        L.LeRobotWriter(w.root, FPS, {**FEATURES, "action": {"dtype": "float32", "shape": [4], "names": None}})


def test_empty_episode_is_dropped(tmp_path):
    w = L.LeRobotWriter(tmp_path / "e", FPS, FEATURES)
    assert w.finalize(w.episode("nothing")) is None
    e = w.episode("aborted")
    frames(e, 3, 0)
    e.abort()
    assert w.total_episodes == 0 and not any((w.root / ".tmp").glob("*.mp4"))
