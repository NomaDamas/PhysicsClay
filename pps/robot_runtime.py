"""Robot runtime: retarget -> IK (mink) -> validate -> read-back.  Run with mjpython on macOS.

Modes:  MIRROR  live follow (when clutch engaged)        REC  live follow + record IR
        PREVIEW replay recorded targets through IK at speed_scale
Keys arrive from the perceive process: R (rec start/stop), P (preview), A (approve/save),
M (mirror toggle), H (home).  Text commands: slower, faster, mirror on|off, hold, go, reset.
When the clutch is disengaged the ghost targets can be dragged in the viewer (double-click a
target, then Ctrl + right-drag) and the robot follows -> "Shape it".
"""
import argparse
import time
from pathlib import Path

import mink
import mujoco
import numpy as np
from loop_rate_limiters import RateLimiter

from .fake import fake_landmarks
from .ir import IntentRecorder
from .protocol import PERCEIVE_TO_ROBOT, ROBOT_TO_PERCEIVE, make_receiver, make_sender
from .retarget import Retargeter
from .validate import ARM_JOINTS, Validator

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "vendor" / "mink" / "examples" / "unitree_g1" / "scene.xml"

COLLISION_PAIRS = [
    (["left_hand_collision", "right_hand_collision", "left_wrist_collision", "right_wrist_collision"],
     ["torso_collision", "pelvis_collision", "head_collision"]),
    (["left_hand_collision", "left_wrist_collision"], ["left_thigh_collision"]),
    (["right_hand_collision", "right_wrist_collision"], ["right_thigh_collision"]),
    (["left_hand_collision"], ["right_hand_collision"]),
]
FEET = ["right_foot", "left_foot"]
HANDS = ["right_palm", "left_palm"]


class Runtime:
    def __init__(self, mirror=True):
        self.model = mujoco.MjModel.from_xml_path(XML.as_posix())
        self.configuration = mink.Configuration(self.model)
        self.data = self.configuration.data
        m, d = self.model, self.data

        self.pelvis_task = mink.FrameTask("pelvis", "body", position_cost=0.0, orientation_cost=1.0, lm_damping=1.0)
        self.torso_task = mink.FrameTask("torso_link", "body", position_cost=0.0, orientation_cost=1.0, lm_damping=1.0)
        self.posture_task = mink.PostureTask(m, cost=1e-1)
        self.com_task = mink.ComTask(cost=10.0)
        self.feet_tasks = [mink.FrameTask(f, "site", position_cost=10.0, orientation_cost=1.0, lm_damping=1.0) for f in FEET]
        self.hand_tasks = [mink.FrameTask(h, "site", position_cost=4.0, orientation_cost=0.05, lm_damping=1.0) for h in HANDS]
        self.tasks = [self.pelvis_task, self.torso_task, self.posture_task, self.com_task, *self.feet_tasks, *self.hand_tasks]
        self.limits = [
            mink.ConfigurationLimit(m),
            mink.CollisionAvoidanceLimit(model=m, geom_pairs=COLLISION_PAIRS,
                                         minimum_distance_from_collisions=0.01, collision_detection_distance=0.12),
        ]
        self.com_mid = m.body("com_target").mocapid[0]
        self.feet_mid = [m.body(f"{f}_target").mocapid[0] for f in FEET]
        self.hands_mid = {h: m.body(f"{h}_target").mocapid[0] for h in HANDS}
        self.palm_sid = {h: m.site(h).id for h in HANDS}
        self.home()

        # Robot geometry for retargeting (fixed at the home pose).
        sh = {"left_palm": d.xpos[m.body("left_shoulder_pitch_link").id].copy(),
              "right_palm": d.xpos[m.body("right_shoulder_pitch_link").id].copy()}
        arm_len = np.mean([
            np.linalg.norm(d.xpos[m.body(f"{s}_elbow_link").id] - sh[f"{s}_palm"])
            + np.linalg.norm(d.site_xpos[self.palm_sid[f"{s}_palm"]] - d.xpos[m.body(f"{s}_elbow_link").id])
            for s in ("left", "right")])
        self.retargeter = Retargeter(sh, arm_len, mirror=mirror)
        self.validator = Validator(m, d)
        self.recorder = IntentRecorder("unitree_g1", [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)], mirror)

        self.mode = "MIRROR"
        self.engaged = False
        self.hold = False
        self.speed_scale = 1.0
        self.last_lm = None
        self.last_grip = {"l": 0, "r": 0}
        self.new_pose = False
        self.saved = None
        self.preview_t = 0.0
        self.preview_idx = 0
        self.last_result = {"flags": {}, "messages": [], "feasibility": 1.0, "speed_scale": 1.0, "residuals": {}}

        self.default_rgba = m.geom_rgba.copy()
        self.joint_body = {n: m.jnt_bodyid[m.joint(n).id] for n in ARM_JOINTS}
        self.body_geoms = {b: [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_group[g] == 2] for b in range(m.nbody)}
        self.target_geoms = {h: [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(f"{h}_target").id] for h in HANDS}

    # ------------------------------------------------------------------ setup
    def home(self):
        m, d = self.model, self.data
        self.configuration.update_from_keyframe("stand")
        self.posture_task.set_target_from_configuration(self.configuration)
        self.pelvis_task.set_target_from_configuration(self.configuration)
        self.torso_task.set_target_from_configuration(self.configuration)
        for h, f in zip(HANDS, FEET):
            mink.move_mocap_to_frame(m, d, f"{f}_target", f, "site")
            mink.move_mocap_to_frame(m, d, f"{h}_target", h, "site")
        d.mocap_pos[self.com_mid] = d.subtree_com[1]

    # --------------------------------------------------------------- messages
    def handle_message(self, msg):
        self.engaged = bool(msg.get("engaged", False)) and not self.hold
        if msg.get("lm"):
            self.last_lm = msg["lm"]
            self.new_pose = True
        if msg.get("grip"):
            self.last_grip = msg["grip"]
        key = msg.get("key")
        if key:
            self.handle_key(key.upper())
        cmd = msg.get("cmd")
        if cmd:
            self.apply_cmd(cmd.strip().lower().lstrip("/"))

    def handle_key(self, key):
        if key == "R":
            if self.mode == "REC":
                self.mode = "MIRROR"
            else:
                self.recorder.start()
                self.mode = "REC"
        elif key == "P":
            if self.recorder.frames:
                self.mode = "PREVIEW"
                self.preview_t = 0.0
                self.preview_idx = 0
                self.speed_scale = min(self.speed_scale, self.recorder.min_speed_scale)
        elif key == "A":
            if self.recorder.frames:
                self.saved = self.recorder.save((ROOT / "recordings").as_posix())
                self.mode = "MIRROR"
        elif key == "M":
            self.retargeter.mirror = not self.retargeter.mirror
            self.retargeter.reset()
        elif key == "H":
            self.home()
            self.retargeter.reset()
            self.mode = "MIRROR"

    def apply_cmd(self, cmd):
        if "slow" in cmd:
            self.speed_scale = max(0.1, self.speed_scale * 0.75)
        elif "fast" in cmd:
            self.speed_scale = min(1.0, self.speed_scale * 1.33)
        elif "mirror" in cmd:
            self.retargeter.mirror = "off" not in cmd
            self.retargeter.reset()
        elif cmd in ("hold", "stop", "pause"):
            self.hold = True
        elif cmd in ("go", "resume", "follow"):
            self.hold = False
        elif cmd in ("reset", "home"):
            self.handle_key("H")
        elif cmd in ("rec", "record"):
            self.handle_key("R")
        elif cmd in ("preview", "play"):
            self.handle_key("P")
        elif cmd in ("approve", "save", "export"):
            self.handle_key("A")

    # ------------------------------------------------------------------- step
    def update_targets(self, dt):
        d = self.data
        if self.mode in ("MIRROR", "REC"):
            if self.engaged and self.last_lm is not None:
                for palm, tgt in self.retargeter.targets(self.last_lm).items():
                    d.mocap_pos[self.hands_mid[palm]] = tgt
            # else: leave mocap where it is -> user may drag the ghosts in the viewer.
        elif self.mode == "PREVIEW":
            frames = self.recorder.frames
            self.preview_t += dt * self.speed_scale
            while self.preview_idx + 1 < len(frames) and frames[self.preview_idx + 1]["t"] <= self.preview_t:
                self.preview_idx += 1
            for palm, tgt in frames[self.preview_idx]["targets"].items():
                d.mocap_pos[self.hands_mid[palm]] = np.asarray(tgt)
            if self.preview_t >= self.recorder.duration:
                self.mode = "MIRROR"

    def step(self, dt):
        d = self.data
        self.update_targets(dt)
        self.com_task.set_target(d.mocap_pos[self.com_mid])
        for task, mid in zip(self.feet_tasks, self.feet_mid):
            task.set_target(mink.SE3.from_mocap_id(d, mid))
        for task, h in zip(self.hand_tasks, HANDS):
            task.set_target(mink.SE3.from_mocap_id(d, self.hands_mid[h]))
        vel = mink.solve_ik(self.configuration, self.tasks, dt, "daqp", damping=1e-1, limits=self.limits)
        self.configuration.integrate_inplace(vel, dt)
        mujoco.mj_camlight(self.model, d)
        targets = {h: d.mocap_pos[self.hands_mid[h]].copy() for h in HANDS}
        res = self.validator.evaluate(targets, vel)
        if self.mode == "REC" and self.new_pose:
            self.recorder.add(self.last_lm, targets, self.last_grip, d.qpos, res["flags"], res["residuals"], res["speed_scale"])
        self.new_pose = False
        self.last_result = res
        return res

    def readback(self, ik_hz):
        r = self.last_result
        return {"mode": self.mode, "engaged": self.engaged, "hold": self.hold, "mirror": self.retargeter.mirror,
                "feasibility": round(r["feasibility"], 3), "messages": r["messages"], "flags": r["flags"],
                "speed_scale": round(self.speed_scale, 2), "rec_frames": len(self.recorder.frames),
                "rec_duration": round(self.recorder.duration, 2), "preview_t": round(self.preview_t, 2),
                "saved": self.saved, "ik_hz": round(ik_hz, 1)}

    # ------------------------------------------------------------------ paint
    def paint(self, viewer):
        m, d, res = self.model, self.data, self.last_result
        m.geom_rgba[:] = self.default_rgba
        for h in HANDS:
            bad = res["flags"].get(h) == "unreachable"
            m.geom_rgba[self.target_geoms[h]] = (1.0, 0.25, 0.25, 0.55) if bad else (0.3, 0.9, 0.4, 0.35)
        for name, kind in res["flags"].items():
            if kind == "limit":
                m.geom_rgba[self.body_geoms[self.joint_body[name]]] = (1.0, 0.2, 0.2, 1.0)
            elif kind == "collision":
                try:
                    bid = m.geom_bodyid[m.geom(name).id]
                except KeyError:
                    continue
                m.geom_rgba[self.body_geoms[bid]] = (1.0, 0.6, 0.1, 1.0)
        scn = viewer.user_scn
        scn.ngeom = 0
        for h in HANDS:
            if res["flags"].get(h) != "unreachable" or scn.ngeom >= scn.maxgeom:
                continue
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9),
                                np.array([1.0, 0.2, 0.2, 0.9], dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.006,
                                 d.site_xpos[self.palm_sid[h]], d.mocap_pos[self.hands_mid[h]])
            scn.ngeom += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="no viewer (testing)")
    ap.add_argument("--iters", type=int, default=600)
    ap.add_argument("--fake-input", action="store_true", help="synthetic landmarks instead of UDP")
    ap.add_argument("--no-mirror", action="store_true")
    ap.add_argument("--hz", type=float, default=120.0)
    args = ap.parse_args()

    rt = Runtime(mirror=not args.no_mirror)
    recv = make_receiver(PERCEIVE_TO_ROBOT)
    send = make_sender(ROBOT_TO_PERCEIVE)
    rate = RateLimiter(frequency=args.hz, warn=False)
    t_start = time.time()
    last_send, n_steps, hz_t0, ik_hz = 0.0, 0, time.time(), 0.0

    def tick(viewer):
        nonlocal last_send, n_steps, hz_t0, ik_hz
        msgs = recv()
        if args.fake_input:
            t = time.time() - t_start
            msgs.append({"engaged": True, "lm": fake_landmarks(t), "grip": {"l": 0, "r": 0}})
        for msg in msgs:
            rt.handle_message(msg)
        rt.step(rate.dt)
        n_steps += 1
        if time.time() - hz_t0 >= 1.0:
            ik_hz, n_steps, hz_t0 = n_steps / (time.time() - hz_t0), 0, time.time()
        if viewer is not None:
            rt.paint(viewer)
            viewer.sync()
        now = time.time()
        if now - last_send >= 1 / 30:
            send(rt.readback(ik_hz))
            last_send = now
        rate.sleep()

    if args.headless:
        for i in range(args.iters):
            tick(None)
            if i % 120 == 0:
                r = rt.last_result
                print(f"[{i:4d}] mode={rt.mode} feas={r['feasibility']:.2f} ik_hz={ik_hz:.0f} "
                      f"res={ {k: round(v, 3) for k, v in r['residuals'].items()} } flags={r['flags']} msgs={r['messages']}")
        return

    import mujoco.viewer  # noqa: E402  (needs mjpython on macOS)
    with mujoco.viewer.launch_passive(model=rt.model, data=rt.data, show_left_ui=False, show_right_ui=False) as viewer:
        mujoco.mjv_defaultFreeCamera(rt.model, viewer.cam)
        viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 180, -12, 2.4
        viewer.cam.lookat[:] = (0.0, 0.0, 0.8)
        while viewer.is_running():
            tick(viewer)


if __name__ == "__main__":
    main()
