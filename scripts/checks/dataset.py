"""v1 P1 Datasets (#28): record tasks as LeRobot v2.1 episodes over the real protocol.

  * {type: "dataset", op: "start", auto: true} -> state.dataset.recording;
  * one task that succeeds (with the words that asked for it) and one that fails (stopped half way) become two
    episodes, labelled; a task refused before moving becomes no episode (counted as refused);
  * files on disk: meta/info.json (features, totals), episodes.jsonl (success, language), tasks.jsonl, stats,
    parquet rows == episode length ~ fps x duration, timestamps == frame_index / fps, kinematic action == next state,
    both camera videos decode with ffmpeg to the same number of frames;
  * discard_last removes the last episode; the real-time loop keeps its frame rate while recording;
  * when `lerobot` happens to be installed, LeRobotDataset loads it (never installed for this check).
The dataset goes to datasets/<selftest name>/ and is deleted at the end. Pure writer tests: tests/test_lerobot_writer.py.

Standalone (starts its own --fake server on PC_DATASET_PORT, default 8906):
    .venv/bin/python scripts/checks/dataset.py
"""
import asyncio
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _ds(t):
    return t.state().get("dataset") or {}


async def wait_for(t, pred, limit=10.0, step=0.2):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(step)
        try:
            if pred(t.state()):
                return True
        except Exception:
            pass
    return False


async def run(t):
    import numpy as np

    from pps.data import lerobot as L

    st = t.state()
    if ((st.get("world") or {}).get("robot") or {}).get("id") not in (None, "g1"):
        await t.send(type="world", op="reset", id="ds-reset")
        await wait_for(t, lambda s: ((s.get("world") or {}).get("robot") or {}).get("id") == "g1", 25.0, 0.4)
    await t.task("reset_scene", 1)
    await t.pump(0.6)

    async def fps_while_running(tid):
        out, t0 = [], time.time()
        while time.time() - t0 < 30:
            await t.pump(0.3)
            tk = t.state().get("task") or {}
            if tk.get("id") == tid and tk.get("status") == "running":
                out.append(float(t.state().get("fps") or 0))
            elif out:
                break
        return out

    # baseline: the same task without recording (the server's frame rate while the robot moves)
    await t.send(type="task", id="ds-base", action="pick_place", object="mug", target="center")
    fps_base = await fps_while_running("ds-base")
    await t.task("reset_scene", 1)
    await t.pump(0.6)
    fps_idle = float(np.median(fps_base)) if fps_base else 0.0
    name = f"selftest_ds_{int(time.time())}"
    root = Path(os.environ.get("PC_DATASETS_DIR") or ROOT / "datasets") / name
    try:
        await t.send(type="dataset", op="start", name=name, auto=True, id="ds1")
        ok = await wait_for(t, lambda s: (s.get("dataset") or {}).get("recording") is True, 5)
        d = _ds(t)
        t.check("dataset: start auto recording", ok and d.get("name") == name and (d.get("reply") or {}).get("ok"),
                str(d.get("reply"))[:90])

        # 1) a task that succeeds, asked for in words
        await t.send(type="task", id="ds-ok", action="pick_place", object="mug", target="center",
                     utterance="put the mug in the middle of the table")
        fps_rec = await fps_while_running("ds-ok")
        task_ok = t.state().get("task") or {}
        await wait_for(t, lambda s: (s.get("dataset") or {}).get("episodes") == 1, 5)
        d = _ds(t)
        last = d.get("last") or {}
        t.check("dataset: a finished task becomes a successful episode",
                task_ok.get("status") == "done" and last.get("success") is True and last.get("task") == "put the mug in the middle of the table",
                f"{task_ok.get('status')} · {last}")

        # 2) refused before moving: no episode
        await t.task("pick_place", 2, object="mug", target="back")
        await t.pump(0.6)
        d = _ds(t)
        t.check("dataset: a task refused before moving is counted, not recorded", d.get("episodes") == 1 and d.get("refused", 0) >= 1,
                f"episodes={d.get('episodes')} refused={d.get('refused')}")

        # 3) a task that fails half way (stopped by the user)
        await t.task("reset_scene", 1)
        await t.send(type="task", id="ds-fail", action="pick_place", object="red block", target="center")
        await wait_for(t, lambda s: (s.get("task") or {}).get("id") == "ds-fail" and s["task"]["status"] == "running", 4)
        await t.pump(1.6)
        await t.task("stop", 1)
        ok = await wait_for(t, lambda s: (s.get("dataset") or {}).get("episodes") == 2, 6)
        d = _ds(t)
        last = d.get("last") or {}
        t.check("dataset: a stopped task is kept as a failed episode", ok and last.get("success") is False and last.get("outcome") == "failed"
                and last.get("task") == "put the red block in the center of the table", str(last)[:140])

        # wait for the dataset thread to write everything, then stop. It renders the cameras of every row (two 256 x 256
        # views): ~170 ms a row with OSMesa on a 2-core runner, so it falls behind a 30 fps episode and catches up
        # afterwards. Wait as long as its backlog keeps shrinking (20 s without progress = stuck), not a fixed 20 s.
        def written(s):
            ds = s.get("dataset") or {}
            return ds.get("pending") == 0 and all(e.get("i") is not None for e in ds.get("list") or [])
        left = None
        while not await wait_for(t, written, 20):
            now_left = _ds(t).get("pending")
            if left is not None and (now_left is None or now_left >= left):
                break
            left = now_left
        cost = (_ds(t).get("cost_ms") or {})
        await t.send(type="dataset", op="stop", id="ds2")
        ok = await wait_for(t, lambda s: (s.get("dataset") or {}).get("recording") is False, 5)
        await t.pump(1.0)
        rec_fps = float(np.median(fps_rec)) if fps_rec else 0.0
        # real time while recording: the loop keeps 80% of its frame rate, or at least the dataset's rate (every row a
        # fresh frame) when it idles faster than that (a loop with nothing to draw idles at MAX_FPS: spare frames
        # above the recording rate aren't real time lost)
        need = min(0.8 * fps_idle, float(d.get("fps") or 30))
        t.check("dataset: stop; the server stays real time while recording",
                ok and rec_fps >= need, f"loop {fps_idle:.0f} fps without, {rec_fps:.0f} with recording (same task; needs {need:.0f}); "
                f"snapshot {cost.get('snapshot')} ms (server loop), render {cost.get('render')} ms (dataset thread)")

        # ---- files on disk
        g1 = root / "g1"
        info = json.loads((g1 / "meta" / "info.json").read_text())
        f = info["features"]
        nst = f["observation.state"]["shape"][0]
        dyn = (t.state().get("physics") or {}).get("mode") == "dynamic"       # P0: ee_force is recorded, action is q_des
        t.check("dataset: meta/info.json (LeRobot v2.1)",
                info["codebase_version"] == "v2.1" and info["total_episodes"] == 2 and info["fps"] == 30 and info["robot_type"] == "g1"
                and len(f["observation.state"]["names"]) == nst and f["action"]["shape"] == [nst]
                and f["observation.images.front"]["dtype"] == "video" and f["observation.images.wrist"]["shape"] == [256, 256, 3]
                and f["observation.human.pose"]["shape"] == [99] and "next.success" in f and ("observation.ee_force" in f) == dyn,
                f"state {nst} ({f['observation.state']['names'][0]} … {f['observation.state']['names'][-1]}), "
                f"{info['total_frames']} frames, {info['total_tasks']} tasks")
        eps = L.read_jsonl(g1 / "meta" / "episodes.jsonl")
        tasks = [r["task"] for r in L.read_jsonl(g1 / "meta" / "tasks.jsonl")]
        t.check("dataset: episodes.jsonl labels success and language",
                [e["success"] for e in eps] == [True, False] and eps[0]["tasks"] == ["put the mug in the middle of the table"]
                and eps[0]["language_source"] == "user" and eps[1]["language_source"] == "template" and len(tasks) == 2,
                str([(e["episode_index"], e["length"], e["success"], e["outcome"]) for e in eps]))
        import pyarrow.parquet as pq
        good, detail = True, []
        for e in eps:
            tb = pq.read_table(g1 / L.DATA_PATH.format(episode_chunk=0, episode_index=e["episode_index"]))
            n = tb.num_rows
            expect = e["duration_s"] * 30
            ts = np.array(tb.column("timestamp").to_pylist())
            stt = np.array(tb.column("observation.state").to_pylist())
            act = np.array(tb.column("action").to_pylist())
            nj = sum(1 for x in f["observation.state"]["names"] if x.startswith("joint:"))
            # kinematic action = the next distinct state (a repeated frame shares its values with the one before)
            nxt = [next((stt[j] for j in range(i + 1, n) if not np.array_equal(stt[j], stt[i])), stt[i]) for i in range(n)]
            if dyn:     # P0: the action is q_des, what the plant's servos track: close to the next state, not a copy of it
                err = np.abs(act[:, :nj] - np.array(nxt)[:, :nj])
                look = float(np.median(err)) < 0.05 and float(err.max()) < 0.6 and not np.allclose(act[:, :nj], stt[:, :nj], atol=1e-5)
            else:
                look = np.allclose(act[:, :nj], np.array(nxt)[:, :nj], atol=1e-5)
            moved = float(np.abs(stt[:, :nj] - stt[0, :nj]).max()) > 0.05   # (a pick & place ends where it started: at home)
            conds = {"rows": n == e["length"], "fps x duration": abs(n - expect) <= 0.1 * expect + 3,
                     "timestamps": np.allclose(ts, np.arange(n) / 30, atol=1e-4), "action": look, "moved": moved,
                     "done": tb.column("next.done").to_pylist()[-1] is True,
                     "no person (the no-camera dancer is not recorded)": not np.array(tb.column("observation.human.pose").to_pylist()).any()}
            good &= all(conds.values())
            if not all(conds.values()):
                detail.append("bad: " + ",".join(k for k, v in conds.items() if not v))
            detail.append(f"ep{e['episode_index']}: {n} rows / {e['duration_s']:.2f} s, {e['repeated_frames']} repeated")
        t.check("dataset: parquet rows ~ fps x duration, exact timestamps, action = next state", good, "; ".join(detail))
        vids = True
        for e in eps:
            for cam in ("front", "wrist"):
                vp = g1 / L.VIDEO_PATH.format(episode_chunk=0, video_key=f"observation.images.{cam}", episode_index=e["episode_index"])
                out = subprocess.run([L.ffmpeg_exe(), "-v", "error", "-i", str(vp), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                                     capture_output=True).stdout if vp.exists() else b""
                frames = len(out) // (256 * 256 * 3)
                img = np.frombuffer(out[:256 * 256 * 3], np.uint8) if out else np.zeros(1)
                vids &= frames == e["length"] and float(img.std()) > 5.0
        size = sum(p.stat().st_size for p in g1.rglob("*.mp4")) / 1e6
        t.check("dataset: front + wrist videos decode (h264) with one frame per row", vids, f"{size:.1f} MB of video")
        es = L.read_jsonl(g1 / "meta" / "episodes_stats.jsonl")
        agg = json.loads((g1 / "meta" / "stats.json").read_text())
        t.check("dataset: per-episode and aggregated stats", len(es) == 2 and "observation.state" in agg
                and np.array(agg["observation.images.front"]["mean"]).shape == (3, 1, 1) and agg["observation.state"]["count"][0] == info["total_frames"])

        # ---- discard the last episode
        await t.send(type="dataset", op="discard_last", id="ds3")
        await wait_for(t, lambda s: (s.get("dataset") or {}).get("episodes") == 1, 8)
        await t.pump(0.5)
        info2 = json.loads((g1 / "meta" / "info.json").read_text())
        gone = not (g1 / L.DATA_PATH.format(episode_chunk=0, episode_index=1)).exists() and not (
            g1 / L.VIDEO_PATH.format(episode_chunk=0, video_key="observation.images.front", episode_index=1)).exists()
        t.check("dataset: discard_last removes the last episode", info2["total_episodes"] == 1 and gone
                and info2["total_frames"] == eps[0]["length"], _ds(t).get("message", ""))

        # ---- optional: LeRobot itself (only if it is already installed; it is never installed for this)
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
        except Exception:
            LeRobotDataset = None
        if LeRobotDataset is not None:
            try:
                ds = LeRobotDataset(f"local/{name}", root=g1)
                item = ds[0]
                t.check("dataset: LeRobotDataset loads it", len(ds) == info2["total_frames"] and "observation.images.front" in item,
                        f"{len(ds)} frames")
            except Exception as e:
                t.check("dataset: LeRobotDataset loads it", False, repr(e)[:120])
    finally:
        await t.send(type="dataset", op="stop", id="ds-cleanup")
        await t.pump(0.5)
        shutil.rmtree(root, ignore_errors=True)


# ---------------------------------------------------------------------- standalone
def _main():
    import aiohttp
    port = int(os.environ.get("PC_DATASET_PORT", "8906"))
    base = f"http://127.0.0.1:{port}"
    results = []

    def check(name, ok, detail=""):
        results.append((name, bool(ok)))
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)

    async def go():
        from types import SimpleNamespace
        s = aiohttp.ClientSession()
        for _ in range(120):
            try:
                async with s.get(base + "/") as r:
                    if r.status == 200:
                        break
            except Exception:
                await asyncio.sleep(0.5)
        ws = await s.ws_connect(base + "/ws")
        st = {}

        async def pump(sec):
            nonlocal st
            t0 = time.time()
            while time.time() - t0 < sec:
                m = await asyncio.wait_for(ws.receive(), 10)
                if m.type == aiohttp.WSMsgType.TEXT:
                    st = json.loads(m.data)
            return st

        async def task(action, wait=1.0, **kw):
            tid = f"{action}-{time.time():.3f}"
            await ws.send_str(json.dumps({"type": "task", "id": tid, "action": action, **kw}))
            t0 = time.time()
            while time.time() - t0 < wait:
                await pump(0.2)
                if st.get("task") and st["task"]["id"] == tid and st["task"]["status"] != "planning":
                    break
            return st.get("task") or {}

        async def until_done(limit=25):
            t0 = time.time()
            while time.time() - t0 < limit:
                await pump(0.3)
                if (st.get("task") or {}).get("status") != "running":
                    break
            return st.get("task") or {}

        t = SimpleNamespace(send=lambda **k: ws.send_str(json.dumps(k)), pump=pump, task=task, until_done=until_done,
                            check=check, state=lambda: st, http=s, base=base, frames=lambda: {})
        try:
            await pump(1.5)
            await run(t)
        finally:
            await ws.close()
            await s.close()

    srv = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "pps.app", "--fake", "--port", str(port),
                            "--https-port", str(port + 1)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        asyncio.run(go())
    finally:
        srv.terminate()
        try:
            srv.wait(10)
        except subprocess.TimeoutExpired:
            srv.kill()
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    _main()
