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
XML = ROOT / "assets" / "unitree_g1" / "scene_pc.xml"

COLLISION_PAIRS = [
    (["left_hand_collision", "right_hand_collision", "left_wrist_collision", "right_wrist_collision"],
     ["torso_collision", "pelvis_collision", "head_collision"]),
    (["left_hand_collision", "left_wrist_collision"], ["left_thigh_collision"]),
    (["right_hand_collision", "right_wrist_collision"], ["right_thigh_collision"]),
    (["left_hand_collision"], ["right_hand_collision"]),
    (["left_hand_collision", "right_hand_collision", "left_wrist_collision", "right_wrist_collision",
      "left_elbow_yaw_collision", "right_elbow_yaw_collision"], ["table", "mug_collision"]),
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
        self.last_context = {}
        self.new_pose = False
        self.saved = None
        self.preview_t = 0.0
        self.preview_idx = 0
        self.path = None          # {"pts": (N,3) robot frame, "ok": [bool], "palm": str, "cum": (N,), "i": int}
        self.follow_s = 0.0
        self.follow_speed = 0.3   # m/s along the drawn path
        self.path_msg = None
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
        self.last_context = {"holding": msg.get("holding") or {}, "scene": msg.get("scene") or [], "source": msg.get("source")}
        if msg.get("path"):
            self.set_path(msg["path"])
        key = msg.get("key")
        if key:
            self.handle_key(key.upper())
        cmd = msg.get("cmd")
        if cmd:
            self.apply_cmd(cmd.strip().lower().lstrip("/"))

    # ------------------------------------------------------------- spatial path
    def set_path(self, path):
        """A trajectory drawn by the user (human-world points) -> robot frame + per-waypoint reachability."""
        if self.last_lm is None:
            self.path_msg = "Stand in view first: the path is placed in your body plane"
            return
        pts, palm = [], None
        for pt in path.get("points", []):
            palm, rp = self.retargeter.map_point(pt, path.get("hand", "r"), self.last_lm)
            if rp is not None:
                pts.append(rp)
        if len(pts) < 2 or palm is None:
            return
        pts = np.asarray(pts)
        ok = self.precheck(pts, palm)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.path = {"pts": pts, "ok": ok, "palm": palm, "cum": cum, "i": 0}
        bad = int(sum(1 for o in ok if not o))
        side = "Left" if palm.startswith("left") else "Right"
        self.path_msg = (f"{side} hand path: {len(pts)} waypoints, all reachable. Press F to follow."
                         if bad == 0 else f"{side} hand path: {bad}/{len(pts)} waypoints out of reach (red). Redraw or press F.")

    def precheck(self, pts, palm, iters=10, dt=0.03):
        """Trace the path with IK on a scratch configuration; a waypoint is ok if the hand gets within reach_tol."""
        cfg = mink.Configuration(self.model)
        cfg.update(self.configuration.q.copy())
        task = self.hand_tasks[HANDS.index(palm)]
        saved = task.transform_target_to_world
        mid = self.hands_mid[palm]
        quat = self.data.mocap_quat[mid].copy()
        ok = []
        for p in pts:
            task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(quat), p))
            for _ in range(iters):
                v = mink.solve_ik(cfg, self.tasks, dt, "daqp", damping=1e-1, limits=self.limits)
                cfg.integrate_inplace(v, dt)
            ok.append(bool(np.linalg.norm(cfg.data.site_xpos[self.palm_sid[palm]] - p) < self.validator.reach_tol))
        if saved is not None:
            task.set_target(saved)
        return ok

    def handle_key(self, key):
        if key == "F":
            if self.path is not None:
                self.mode, self.follow_s, self.path["i"] = "FOLLOW", 0.0, 0
                self.recorder.start()
            return
        if key == "C":
            self.path, self.path_msg = None, None
            if self.mode == "FOLLOW":
                self.mode = "MIRROR"
            return
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
        elif self.mode == "FOLLOW" and self.path is not None:
            pts, cum = self.path["pts"], self.path["cum"]
            self.follow_s += dt * self.follow_speed * self.speed_scale
            i = int(np.searchsorted(cum, self.follow_s, side="right")) - 1
            i = max(0, min(i, len(pts) - 2))
            seg = cum[i + 1] - cum[i]
            a = 0.0 if seg < 1e-9 else min(1.0, (self.follow_s - cum[i]) / seg)
            d.mocap_pos[self.hands_mid[self.path["palm"]]] = (1 - a) * pts[i] + a * pts[i + 1]
            self.path["i"] = i if a < 1.0 else i + 1
            if self.follow_s >= cum[-1]:
                self.path["i"] = len(pts) - 1
                self.mode = "MIRROR"
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
        if self.mode == "FOLLOW":
            self.recorder.add(self.last_lm, targets, self.last_grip, d.qpos, res["flags"], res["residuals"], res["speed_scale"], {"source": "drawn_path"})
        if self.mode == "REC" and self.new_pose:
            self.recorder.add(self.last_lm, targets, self.last_grip, d.qpos, res["flags"], res["residuals"], res["speed_scale"], self.last_context)
        self.new_pose = False
        self.last_result = res
        return res

    def readback(self, ik_hz):
        r = self.last_result
        return {"mode": self.mode, "engaged": self.engaged, "hold": self.hold, "mirror": self.retargeter.mirror,
                "feasibility": round(r["feasibility"], 3), "messages": r["messages"], "flags": r["flags"],
                "speed_scale": round(self.speed_scale, 2), "rec_frames": len(self.recorder.frames),
                "rec_duration": round(self.recorder.duration, 2), "preview_t": round(self.preview_t, 2),
                "saved": self.saved, "ik_hz": round(ik_hz, 1),
                "path_n": 0 if self.path is None else len(self.path["pts"]),
                "path_ok": [] if self.path is None else self.path["ok"],
                "path_i": -1 if self.path is None else self.path["i"],
                "path_msg": self.path_msg}

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

        def add_capsule(a, b, width, rgba):
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9),
                                np.array(rgba, dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
            scn.ngeom += 1

        def add_sphere(p, r, rgba):
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([r, 0, 0]), np.asarray(p, dtype=np.float64),
                                np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
            scn.ngeom += 1

        for h in HANDS:
            if res["flags"].get(h) == "unreachable":
                add_capsule(d.site_xpos[self.palm_sid[h]], d.mocap_pos[self.hands_mid[h]], 0.006, (1.0, 0.2, 0.2, 0.9))
        if self.path is not None:
            pts, ok, cur = self.path["pts"], self.path["ok"], self.path["i"]
            for k in range(len(pts)):
                good = ok[k]
                rgba = (0.3, 0.9, 0.4, 0.9) if good else (1.0, 0.25, 0.25, 0.9)
                if k == cur and self.mode == "FOLLOW":
                    rgba = (1.0, 0.85, 0.2, 1.0)
                add_sphere(pts[k], 0.014 if k != cur else 0.02, rgba)
                if k + 1 < len(pts):
                    seg_rgba = (0.3, 0.9, 0.4, 0.7) if (ok[k] and ok[k + 1]) else (1.0, 0.25, 0.25, 0.7)
                    add_capsule(pts[k], pts[k + 1], 0.004, seg_rgba)


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
    n_steps_total = [0]

    def tick(viewer):
        nonlocal last_send, n_steps, hz_t0, ik_hz
        msgs = recv()
        if args.fake_input:
            t = time.time() - t_start
            msgs.append({"engaged": True, "lm": fake_landmarks(t), "grip": {"l": 0, "r": 0}})
            if n_steps_total[0] == 150:   # user draws a path: a line in front of the right shoulder, ending too far
                xs = np.linspace(-0.15, 0.95, 16)
                pts = [[-0.18 + x * 0.3, -0.45 - 0.25 * np.sin(x * 3.0), -0.35 - x * 0.6] for x in xs]
                msgs.append({"engaged": False, "path": {"points": pts, "hand": "r"}})
            if n_steps_total[0] == 200:
                msgs.append({"engaged": False, "key": "F"})
            n_steps_total[0] += 1
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
                if rt.path is not None:
                    print(f"       path ok={''.join('o' if o else 'x' for o in rt.path['ok'])} i={rt.path['i']} msg={rt.path_msg}")
        return

    import mujoco.viewer  # noqa: E402  (needs mjpython on macOS)
    with mujoco.viewer.launch_passive(model=rt.model, data=rt.data, show_left_ui=False, show_right_ui=False) as viewer:
        mujoco.mjv_defaultFreeCamera(rt.model, viewer.cam)
        viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 150, -15, 2.6
        viewer.cam.lookat[:] = (0.0, 0.0, 0.8)
        while viewer.is_running():
            tick(viewer)


if __name__ == "__main__":
    main()
