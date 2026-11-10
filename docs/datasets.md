# Datasets: every task becomes a training episode

PhysicsClay records what the robot does as **robot-learning data** in the
[LeRobot](https://github.com/huggingface/lerobot) dataset format (v2.1). You can record live while you instruct the
robot (say it, point, draw, show), or replay one intent headlessly as many randomized episodes across robots and
scenes (`scripts/generate.py`). See `docs/ROADMAP_DATA.md` for why this matters.

Code: `pps/data/` (writer, adapter, cameras, recorder), `pps/features/dataset.py` (protocol), `web/ui/dataset.js`
(record control), `scripts/generate.py`. Tests: `tests/test_lerobot_writer.py`, `scripts/checks/dataset.py`.

## Record from the app

1. Click **Record** in the top bar, give the dataset a name and press **Start**. The dot turns red: `REC · 0 ep`.
2. Ask the robot for things as usual. Each task that starts moving is one episode, from the moment it starts to the
   moment it reports done or failed. The panel lists the episodes with ✓ / ✕ and their length.
3. **Discard last** deletes the last episode from disk (a bad take). **Stop** ends recording. An episode still
   running when you stop is dropped, not saved half-done.

What counts:

* **Success**: the task reported `done`. **Failure**: it reported `failed` (stopped by you, slipped once P0 physics is
  in, …), another task took over (`interrupted`), the world was rebuilt, or it ran longer than 120 s (`timeout`).
  Failed episodes are kept and labelled; they are useful negatives.
* **Not an episode**: a task refused before moving (out of reach, carried through the vase, inside a keep-out zone,
  a person too close). It has no motion; the panel counts it as "refused before moving".
* **Language**: the words that asked for the task. The browser attaches your last line (typed or spoken, within 30 s)
  to the task as `utterance`; tasks the agent launches use the user's turn. Tasks with no words (buttons, the
  generator without `language`) get a sentence made from their arguments, e.g. "put the red cup next to the bowl".
  `meta/episodes.jsonl` says which (`language_source`: `user` | `template`).

The same over the protocol (any client):

```json
{"type": "dataset", "op": "start", "name": "kitchen_session", "auto": true}
{"type": "dataset", "op": "stop"}
{"type": "dataset", "op": "discard_last"}
{"type": "dataset", "op": "randomize", "lighting": 0.6, "camera": 0.02, "seed": 7}
{"type": "dataset", "op": "meta", "data": {"anything": "added to the next episodes' records"}}
```

`state.dataset = {recording, name, episodes, succeeded, refused, frames, current, last, list, datasets, cost_ms,
message, reply, …}` (full list in `docs/PROTOCOL.md`, "v8 · Datasets").

## Layout

One LeRobot dataset per robot body (state and action sizes belong to an embodiment), under
`datasets/<name>/<robot>/`. Scenes and variations of the same robot share a dataset and are told apart by
per-episode metadata.

```
datasets/kitchen_session/panda/
  meta/info.json              codebase_version "v2.1", robot_type, fps, features, totals, chunks_size, paths, splits
  meta/tasks.jsonl            {"task_index": 0, "task": "put the red cup next to the bowl"}
  meta/episodes.jsonl         {"episode_index": 0, "tasks": [...], "length": 154, "success": true, "outcome": "done",
                               "robot", "scene", "objects", "action", "args", "language_source", "duration_s",
                               "repeated_frames", "randomization", "variation", "message"}
  meta/episodes_stats.jsonl   per-episode min / max / mean / std / count of every feature (v2.1)
  meta/stats.json             the same, aggregated over the dataset (what v2.0 readers use)
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.front/episode_000000.mp4     h264, yuv420p, 256x256, 30 fps
  videos/chunk-000/observation.images.wrist/episode_000000.mp4
```

### Features (one row per frame, 30 Hz by default)

| key | dtype, shape | what |
|---|---|---|
| `observation.state` | float32 [J + E] | joint positions of the robot's actuated joints (`joint:<name>`), then one gripper value per end effector (`gripper:right`, `gripper:left`; 0 open, 1 closed). Frozen joints (fingers, head, wheels: the profile's `freeze`) and a legged body's floating base are left out. Panda: 7 + 1, UR5e 6 + 1, G1 29 + 2. |
| `action` | float32 [J + E] | same names. The joint target for this step: P0's `q_des` when the plant is simulated with dynamics; in today's kinematic mode the IK solution reached at the next frame (so `action[t] == state[t+1]` for the joints). Gripper: the grasp command. |
| `observation.images.front` | video 256x256x3 | a fixed third-person camera across the work surface, facing the robot (in front of the backsplash / under the shelf board on those templates) |
| `observation.images.wrist` | video 256x256x3 | a camera on the first end effector (right hand / the only gripper), beside the wrist, looking past the fingers |
| `observation.human.pose` | float32 [99] | the person's 33 MediaPipe landmarks in the robot's world frame (`nose:x` … ), zeros when nobody is in view |
| `observation.ee_force` | float32 [3E] | end-effector force (`right:fx` …). Only present when P0 physics measures it when recording starts. |
| `next.done` | bool [1] | true on the last frame |
| `next.success` | bool [1] | true on the last frame of a successful episode |
| `timestamp`, `frame_index`, `episode_index`, `index`, `task_index` | | LeRobot bookkeeping; `timestamp == frame_index / fps` exactly |

A third camera, the scene's own `robot_view` (the robot's head / overhead view the UI shows), can be added with
`{"op": "start", "cameras": ["front", "wrist", "robot"]}`; `size` (64–512) and `fps` (5–60) are options too.

Sample `meta/info.json` (Panda, abridged):

```json
{"codebase_version": "v2.1", "robot_type": "panda", "fps": 30, "total_episodes": 4, "total_frames": 641,
 "total_tasks": 1, "total_videos": 8, "total_chunks": 1, "chunks_size": 1000, "splits": {"train": "0:4"},
 "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
 "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
 "features": {
  "observation.state": {"dtype": "float32", "shape": [8], "names": ["joint:joint1", "…", "joint:joint7", "gripper:right"]},
  "action": {"dtype": "float32", "shape": [8], "names": ["joint:joint1", "…", "gripper:right"]},
  "observation.images.front": {"dtype": "video", "shape": [256, 256, 3], "names": ["height", "width", "channels"],
    "info": {"video.height": 256, "video.width": 256, "video.codec": "h264", "video.pix_fmt": "yuv420p",
             "video.is_depth_map": false, "video.fps": 30, "video.channels": 3, "has_audio": false}},
  "observation.images.wrist": {"…": "same as front"},
  "observation.human.pose": {"dtype": "float32", "shape": [99], "names": ["nose:x", "nose:y", "nose:z", "…"]},
  "next.done": {"dtype": "bool", "shape": [1], "names": null},
  "next.success": {"dtype": "bool", "shape": [1], "names": null},
  "timestamp": {"dtype": "float32", "shape": [1], "names": null},
  "frame_index": {"dtype": "int64", "shape": [1], "names": null},
  "episode_index": {"dtype": "int64", "shape": [1], "names": null},
  "index": {"dtype": "int64", "shape": [1], "names": null},
  "task_index": {"dtype": "int64", "shape": [1], "names": null}}}
```

## Generate many episodes from one intent

```bash
# one spoken-style request, two arms, two scenes, 20 randomized episodes each
.venv/bin/python scripts/generate.py \
    --task '{"action": "pick_place", "object": "red_cup", "target": "next to the bowl",
             "language": ["put the red cup next to the bowl", "place the red cup beside the bowl"]}' \
    --robots panda,ur5e --scenes table,kitchen --n 20 --randomize objects,lighting,camera --name cup_bowl
```

```
  [1] panda/table #0: ok     5.1 s  put the red cup next to the bowl
  …
robot      scene     tries episodes  success   rate refused errors
panda      table        20       20       20   100%       0      0
…
```

* **Seeds.** `--seed file.json` or `--task '<json>'`: a task (`action`, `object`, `target`, …, optional `language`
  list of phrasings and `scene.objects`) or a trajectory (`points` with `frame: "world" | "surface" |
  "object:<name>"`, run as a drawn path). Without a `scene`, the objects are the ones the words name (the red cup,
  the bowl); colours named in the words are kept so the language stays true.
* **Randomization** (`--randomize`): `objects` (positions over the reachable band of the surface, sizes ±15 %,
  colours of everything the words don't pin down, 0–2 distractors of other kinds), `lighting` (light directions and
  intensity of the dataset cameras, `--lighting` 0–1), `camera` (position / angle jitter, `--camera-jitter` metres),
  `physics` (sends `{type: "physics", op: "set", params: {friction_scale, mass_scale, damping_scale, kp_scale}}` when
  the server has P0 physics; skipped otherwise). The sampled values are stored per episode (`randomization`,
  `variation`).
* **How it runs.** It starts `python -m pps.app --fake` on `--port` (default 8907; offline, no camera, no LLM keys
  used) or talks to a running server (`--server http://127.0.0.1:8905`), rebuilds the world for each episode and runs
  the task over the normal protocol, so every check the robot makes applies. Episodes run in real time (the server
  loop is real time): about 5–7 s each plus 1–2 s to build the world.
* **Success rate** = successful episodes / attempts; refusals and failed world builds count as attempts.

## Load it

Without LeRobot (pyarrow + ffmpeg, what the tests do):

```python
import json, pyarrow.parquet as pq
root = "datasets/cup_bowl/panda"
info = json.load(open(f"{root}/meta/info.json"))
t = pq.read_table(f"{root}/data/chunk-000/episode_000000.parquet").to_pandas()
t[["timestamp", "observation.state", "action", "next.success"]].head()
```

With LeRobot (not a PhysicsClay dependency; it is large):

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset      # lerobot 0.3.x reads v2.1 directly
ds = LeRobotDataset("local/cup_bowl_panda", root="datasets/cup_bowl/panda")
frame = ds[0]   # observation.state, action, observation.images.front (3x256x256 float), task, …
```

LeRobot 0.4 and later use the v3.0 layout; convert a v2.1 dataset with the converter that ships with LeRobot
(`lerobot/datasets/v30/convert_dataset_v21_to_v30.py`) first.

## Train a policy (ACT or Diffusion Policy)

With lerobot 0.3.x (v2.1 datasets; a GPU helps, CPU works for a smoke test):

```bash
python -m lerobot.scripts.train \
  --dataset.repo_id=local/cup_bowl_panda --dataset.root=datasets/cup_bowl/panda \
  --policy.type=act --policy.push_to_hub=false --policy.device=cuda \
  --output_dir=outputs/train/act_cup_bowl --steps=50000 --batch_size=8

# Diffusion Policy: same, with --policy.type=diffusion (it random-crops the 256x256 images)
```

With lerobot ≥ 0.4 the entry point is `lerobot-train` with the same flags, on the converted dataset. Use one robot's
dataset per policy. The policy reads `observation.state`, the camera keys and `action`; `observation.human.pose` and
`observation.ee_force` are extra observations most policies ignore unless you configure them. These commands are
LeRobot's; PhysicsClay's checks verify the files with pyarrow and ffmpeg (and with `LeRobotDataset` when lerobot happens
to be installed), not by training.

To evaluate a trained policy you need it to drive the simulator (a `RobotDriver`-style loop that sends `q_des`); that
is P6's bridge interface and is not part of this workstream.

## What is honest about this data

* **Simulation only.** Rendered with MuJoCo's OpenGL renderer (no shadows by default, primitive objects, a wooden
  texture): there is a large visual gap to real cameras. Randomizing lighting and cameras narrows it a little.
* **Kinematic until P0 lands.** Today the robot is posed by IK and a grasped object is attached to the hand, so a
  success means the plan was feasible and executed, not that a real grip would hold. With P0 the recorder reads `q_des`,
  real finger joints and contact forces automatically, and "slipped" becomes a labelled failure.
* **Actions are joint targets**, not torques: `q_des` (P0) or the next IK state (now). Real robots need their own
  controllers and calibration (P6).
* **Success is the task layer's verdict** (`done` / `failed`), not a separate perception-based verifier.
* **People in the data are poses, not bodies.** `observation.human.pose` comes from the camera (or VR); it is zeros
  without one, and the synthetic no-camera dancer is never recorded as a person.
* **Language** is what the person said, or a sentence generated from the task when nobody spoke. Paraphrases come only
  from the seed's `language` list (no LLM paraphrasing yet).
* **Frame rate.** Samples are on an exact 30 Hz grid; when the server loop runs slower, a grid point repeats the last
  state (`repeated_frames` per episode, typically 10–20 % at a 25–30 Hz server loop).

## Cost

Measured on an Apple-silicon laptop (G1 and Panda worlds, two 256x256 cameras, shadows off):

* server loop: **~0.06 ms** per sample (a few array copies); its frame rate is unchanged (25–32 fps with and without
  recording);
* dataset thread (its own model copy, MjData and OpenGL context): **10–18 ms** per sample to render both cameras, so
  30 Hz fits with room to spare; ffmpeg encodes in its own processes;
* disk: about **0.3 MB per camera per 5 s episode** (h264, crf 23) plus a few kB of parquet; 100 two-camera episodes
  ≈ 60 MB.
