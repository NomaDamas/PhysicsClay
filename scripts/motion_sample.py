"""Before / after sample: a human video on the left, the simulated Unitree G1 doing the same motion on the right.

The motion is extracted with MediaPipe Pose (whole body: arms, torso twist, squat, foot lifts), retargeted to the G1,
solved with IK under joint limits / self-collision, and checked for balance; the result is also saved as a skill.
Usage: .venv/bin/python scripts/motion_sample.py <video> [--name NAME] [--seconds 45]
Output: recordings/samples/<name>_before_after.mp4 (+ recordings/skills/<name>.json)
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pps import robot_runtime  # noqa: E402
from pps.perceive import Pose  # noqa: E402

SKELETON = [(11, 13), (13, 15), (12, 14), (14, 16), (11, 12), (11, 23), (12, 24), (23, 24),
            (23, 25), (25, 27), (24, 26), (26, 28), (0, 11), (0, 12)]
H_OUT = 720
ONE_LINER = "New Generation of interactive prompt for Physical AI"


def stage_scene(seated=False):
    """The demo scene with the table and objects moved off stage, so the whole body is visible."""
    src = (ROOT / "assets" / "unitree_g1" / "scene_pc.xml").read_text()
    for a, b in (('pos="0.52 0 0.720"', 'pos="6.52 0 0.720"'), ('pos="0.25 -0.45', 'pos="6.25 -0.45'), ('pos="0.25 0.45', 'pos="6.25 0.45'),
                 ('pos="0.79 -0.45', 'pos="6.79 -0.45'), ('pos="0.79 0.45', 'pos="6.79 0.45'),
                 ('<body name="mug" pos="0.33', '<body name="mug" pos="6.33'), ('<body name="vase" pos="0.33', '<body name="vase" pos="6.33'),
                 ('<body name="red_block" pos="0.29', '<body name="red_block" pos="6.29'), ('<body name="blue_block" pos="0.44', '<body name="blue_block" pos="6.44')):
        src = src.replace(a, b)
    if seated:   # a chair under the pelvis
        src = src.replace('<body name="com_target"', '''<body name="chair" pos="-0.13 0 0.20">
      <geom type="box" size="0.17 0.2 0.20" rgba="0.36 0.38 0.44 1" contype="0" conaffinity="0" group="2"/>
      <geom type="box" pos="-0.16 0 0.42" size="0.02 0.2 0.24" rgba="0.36 0.38 0.44 1" contype="0" conaffinity="0" group="2"/>
    </body>
    <body name="com_target"''', 1)
    out = ROOT / "assets" / "unitree_g1" / ("scene_stage_seated.xml" if seated else "scene_stage.xml")
    out.write_text(src)
    return out


GEOM_FIELDS = ("type", "dataid", "objtype", "objid", "category", "matid", "texcoord", "segid", "size", "pos", "mat",
               "rgba", "emission", "specular", "shininess", "reflectance", "transparent")


def duplicate_robot(scn, m, root, offset):
    """Add a second copy of the robot's visual geoms to the scene, shifted by offset (a dancing partner)."""
    n0 = scn.ngeom
    for i in range(n0):
        g = scn.geoms[i]
        if g.objtype != mujoco.mjtObj.mjOBJ_GEOM or g.objid < 0 or m.body_rootid[m.geom_bodyid[g.objid]] != root:
            continue
        if scn.ngeom >= scn.maxgeom:
            break
        h = scn.geoms[scn.ngeom]
        for f in GEOM_FIELDS:
            try:
                setattr(h, f, getattr(g, f))
            except Exception:
                pass
        h.pos[:] = np.asarray(g.pos) + offset
        scn.ngeom += 1


def label(img, text, sub=None):
    cv2.rectangle(img, (16, 16), (16 + 14 * len(text) + 28, 62), (253, 251, 251), -1)
    cv2.putText(img, text, (30, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (31, 29, 29), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(img, sub, (30, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 90), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--name", default=None)
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--seated", action="store_true", help="robot sits on a chair (upper-body imitation)")
    ap.add_argument("--duet", action="store_true", help="render two robots dancing side by side")
    args = ap.parse_args()
    name = args.name or "".join(ch for ch in Path(args.video).stem.lower().replace(" ", "_") if ch.isalnum() or ch in "_-")[:28] or "sample"

    robot_runtime.XML = stage_scene(args.seated)
    rt = robot_runtime.Runtime(mirror=False)          # same side as the video: the robot faces you like the person does
    m, d = rt.model, rt.data
    if args.seated:
        # seated base pose: centre of mass low and back over the chair, feet forward; legs ignore the video
        rt.body_home["com"] = rt.body_home["com"] + np.array([-0.10, 0.0, -0.30])
        for f in rt.body_home["feet"]:
            rt.body_home["feet"][f] = rt.body_home["feet"][f] + np.array([0.10, 0.0, 0.0])
        _orig = rt.body_rt.__call__

        class _Seated:
            def __call__(self, lm, mirror):
                b = _orig(lm, mirror)
                if b is None:
                    return None
                return {"squat": 0.0, "lift": {"l": np.zeros(3), "r": np.zeros(3)}, "yaw": b["yaw"] * 0.5}

            def reset(self):
                pass
        rt.body_rt = _Seated()
        rt.apply_body(None)
        for _ in range(400):
            rt.step(1 / 120)
    renderer = mujoco.Renderer(m, H_OUT, 640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, cam)
    cam.azimuth, cam.elevation, cam.distance = 180, -6, 3.0
    cam.lookat[:] = (0.0, 0.0, 0.72)
    if args.seated:
        cam.azimuth, cam.elevation, cam.distance = 155, -8, 2.6
        cam.lookat[:] = (0.0, 0.0, 0.55)
    if args.duet:
        cam.distance = 3.6
        cam.lookat[:] = (0.0, 0.45, 0.72)
    robot_root = m.body("pelvis").id

    pose = Pose(str(ROOT / "models" / "pose_landmarker_full.task"))
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(round(fps / 30.0)))            # write at <= 30 fps
    out_fps = fps / step
    max_frames = int(args.seconds * fps)
    outdir = ROOT / "recordings" / "samples"
    outdir.mkdir(parents=True, exist_ok=True)
    out_path = outdir / f"{name}_before_after.mp4"
    writer = imageio.get_writer(str(out_path), fps=out_fps, codec="libx264", quality=7, macro_block_size=1)

    rt.engaged, rt.mode = True, "REC"
    rt.recorder.start()
    feas_hist, n, t0 = [], 0, time.time()
    while n < max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        if (n - 1) % step:
            continue
        pts, lm = pose(frame)
        if lm:
            rt.last_lm, rt.new_pose = lm, True
        for _ in range(6):
            rt.step(1.0 / (out_fps * 6))
            rt.new_pose = False
        feas_hist.append(not rt.last_result["flags"])
        # ---- left: the human video with the extracted skeleton
        h, w = frame.shape[:2]
        left = cv2.resize(frame, (int(w * H_OUT / h) // 2 * 2, H_OUT))
        if pts:
            sx = left.shape[1] / w
            sy = H_OUT / h
            for a, b in SKELETON:
                pa, pb = pts[a], pts[b]
                cv2.line(left, (int(pa[0] * sx), int(pa[1] * sy)), (int(pb[0] * sx), int(pb[1] * sy)), (95, 208, 138), 3, cv2.LINE_AA)
        label(left, "Before: human video", "motion extracted by MediaPipe Pose")
        # ---- right: the simulated robot
        rt.paint_model()
        renderer.update_scene(d, cam)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        if args.duet:
            duplicate_robot(renderer.scene, m, robot_root, np.array([0.0, 0.9, 0.0]))
        right = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        feas = 100.0 * np.mean(feas_hist[-60:]) if feas_hist else 100.0
        msgs = rt.last_result["messages"]
        label(right, "After: Unitree G1 (simulated)", f"feasible {feas:.0f}%" + (f" | {msgs[0][:38]}" if msgs else ""))
        canvas = np.hstack([left, right])
        bar = np.full((48, canvas.shape[1], 3), (253, 251, 251), np.uint8)
        cv2.putText(bar, "PhysicsClay  |  " + ONE_LINER, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (31, 29, 29), 2, cv2.LINE_AA)
        writer.append_data(cv2.cvtColor(np.vstack([canvas, bar]), cv2.COLOR_BGR2RGB))
    writer.close()
    cap.release()

    frames = [{"t": f["t"], "targets": f["targets"], "body": f.get("body")} for f in rt.recorder.frames if f.get("targets")]
    skill_dir = ROOT / "recordings" / "skills"
    skill_dir.mkdir(parents=True, exist_ok=True)
    if frames:
        with open(skill_dir / f"{name}.json", "w") as f:
            json.dump({"name": name, "robot": "unitree_g1", "created": time.time(), "frames": frames,
                       "report": rt.recorder.report(), "source_video": Path(args.video).name}, f)
    print(json.dumps({"out": str(out_path), "frames_written": len(feas_hist), "seconds": round(len(feas_hist) / out_fps, 1),
                      "feasible_pct": round(100.0 * float(np.mean(feas_hist)), 1) if feas_hist else None,
                      "skill": name if frames else None, "elapsed_s": round(time.time() - t0, 1)}))


if __name__ == "__main__":
    main()
