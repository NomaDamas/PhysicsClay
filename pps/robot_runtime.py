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
from mink.exceptions import NoSolutionFound

from .fake import fake_landmarks
from .ir import IntentRecorder
from .protocol import PERCEIVE_TO_ROBOT, ROBOT_TO_PERCEIVE, make_receiver, make_sender
from .body import BodyRetargeter
from .retarget import Retargeter
from .slicing import point as pause_point
from .traj import follow_step
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
      "left_elbow_yaw_collision", "right_elbow_yaw_collision"], ["table"]),
]
FOLLOW_LAG, FOLLOW_WAIT = 0.03, 2.0      # physics: m the hand may trail the moving target, s it waits for it to catch up
BODY_GEOMS = ["torso_collision","pelvis_collision", "head_collision", "left_hip_collision", "right_hip_collision",
              "left_thigh_collision", "right_thigh_collision"]
FEET = ["right_foot", "left_foot"]


def slerp(q0, q1, a):
    """Unit quaternions (wxyz): spherical interpolation, shortest way."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    c = float(np.dot(q0, q1))
    if c < 0:
        q1, c = -q1, -c
    if c > 0.9995:
        q = q0 + a * (q1 - q0)
    else:
        th = np.arccos(c)
        q = (np.sin((1 - a) * th) * q0 + np.sin(a * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)
HANDS = ["right_palm", "left_palm"]


class Runtime:
    """The Unitree G1 runtime (humanoid: mirroring, whole-body retargeting, balance). Other bodies: pps/world/runtime.py.
    model/home_q: a world built by pps.world.scene (None = the UX 2.0 scene_pc.xml with its 'stand' keyframe)."""
    profile_id = "g1"
    capabilities = ("pick_place", "reach", "follow_path", "imitate_video", "follow_me", "teach_by_showing", "balance")
    safety_scale = 1.0   # P5 (pps/features/safety.py) sets it every frame: 1 free, <1 slowing near a person, 0 stop

    def __init__(self, mirror=True, model=None, home_q=None, ik_model=None, plant=None, ee_site=None):
        """Physics (pps/physics.py): `model` is the plant's model (rendered), `ik_model` the planner's, `plant` steps it.
        Without a plant the IK configuration's data *is* the robot (kinematic mode)."""
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(XML.as_posix())
        self.home_q = None if home_q is None else np.asarray(home_q, float).copy()
        self.hands = list(HANDS)
        self.ee_site = {h: (ee_site or {}).get(h, h) for h in HANDS}   # IK frame per hand (the gripper TCP with physics)
        self.configuration = mink.Configuration(ik_model if ik_model is not None else self.model)
        self.ik_data = self.configuration.data
        self.plant = plant
        self.data = plant.d if plant is not None else self.ik_data
        m, d = self.model, self.data

        self.pelvis_task = mink.FrameTask("pelvis", "body", position_cost=0.0, orientation_cost=1.0, lm_damping=1.0)
        self.torso_task = mink.FrameTask("torso_link", "body", position_cost=0.0, orientation_cost=1.0, lm_damping=1.0)
        self.posture_task = mink.PostureTask(self.configuration.model, cost=1e-1)
        self.com_task = mink.ComTask(cost=10.0)
        self.feet_tasks = [mink.FrameTask(f, "site", position_cost=10.0, orientation_cost=1.0, lm_damping=1.0) for f in FEET]
        # with physics the palm grippers must be held level to grasp: the hand orientation counts (0.05: mirroring only)
        self.hand_tasks = [mink.FrameTask(self.ee_site[h], "site", position_cost=4.0, orientation_cost=0.05,
                                          lm_damping=1.0) for h in HANDS]
        self.tasks = [self.pelvis_task, self.torso_task, self.posture_task, self.com_task, *self.feet_tasks, *self.hand_tasks]
        mi = self.configuration.model
        self.limits = [
            mink.ConfigurationLimit(mi),
            mink.CollisionAvoidanceLimit(model=mi, geom_pairs=COLLISION_PAIRS,
                                         minimum_distance_from_collisions=0.01, collision_detection_distance=0.12),
        ]
        if plant is not None:     # the grippers on the palms must not sweep through the body (with room for tracking lag)
            from .physics import velocity_limit
            self.limits.append(velocity_limit(mi, plant.robot_bodies, ("pc_",)))
            grips = [[f"{s}_palm_col", f"{s}_pad_a", f"{s}_pad_b"] for s in ("right", "left")]
            arms = [f"{s}_{k}_collision" for s in ("right", "left") for k in ("wrist", "hand", "elbow_yaw")]
            upper = [f"{s}_{k}_collision" for s in ("right", "left") for k in ("shoulder_yaw", "elbow_yaw")]
            self.limits += [
                mink.CollisionAvoidanceLimit(model=mi, geom_pairs=[(grips[0] + grips[1], BODY_GEOMS), (grips[0], grips[1]),
                                                                   (arms, ["left_hip_collision", "right_hip_collision"])],
                                             minimum_distance_from_collisions=0.04, collision_detection_distance=0.15),
                mink.CollisionAvoidanceLimit(model=mi, geom_pairs=[(upper, ["torso_collision"])],      # arms across the chest
                                             minimum_distance_from_collisions=0.01, collision_detection_distance=0.08),
                mink.CollisionAvoidanceLimit(model=mi, geom_pairs=[(grips[0] + grips[1], ["table"])],     # pads reach low objects
                                             minimum_distance_from_collisions=0.002, collision_detection_distance=0.025)]
        self.com_mid = m.body("com_target").mocapid[0]
        self.feet_mid = [m.body(f"{f}_target").mocapid[0] for f in FEET]
        self.hands_mid = {h: m.body(f"{h}_target").mocapid[0] for h in HANDS}
        self.palm_sid = {h: m.site(self.ee_site[h]).id for h in HANDS}
        self.root_bid = m.body("pelvis").id
        self.robot_roots = {int(m.body_rootid[self.root_bid])}
        self.home()
        if plant is not None:
            plant.reset(self.configuration.q)
            plant.settle()

        # Robot geometry for retargeting (fixed at the home pose; the palms, not the gripper TCP).
        di = self.ik_data
        sh = {"left_palm": di.xpos[m.body("left_shoulder_pitch_link").id].copy(),
              "right_palm": di.xpos[m.body("right_shoulder_pitch_link").id].copy()}
        arm_len = np.mean([
            np.linalg.norm(di.xpos[m.body(f"{s}_elbow_link").id] - sh[f"{s}_palm"])
            + np.linalg.norm(di.site_xpos[m.site(f"{s}_palm").id] - di.xpos[m.body(f"{s}_elbow_link").id])
            for s in ("left", "right")])
        self.retargeter = Retargeter(sh, arm_len, mirror=mirror)
        # how far the end effector (physics: the gripper's TCP) sits past the palm the arm length is measured to
        # (teleop / skills widen their reach by it; 0 in kinematic mode)
        self.tcp_reach = {h: float(np.linalg.norm(di.site_xpos[self.palm_sid[h]] - di.site_xpos[m.site(h).id])) for h in HANDS}
        self.body_rt = BodyRetargeter()
        self.body_home = {"com": d.mocap_pos[self.com_mid].copy(),
                          "feet": {f: d.mocap_pos[mid].copy() for f, mid in zip(FEET, self.feet_mid)},
                          "torso": self.torso_task.transform_target_to_world.copy()}
        self.body_now = None
        self.validator = Validator(self.configuration.model, self.ik_data, root_body=self.root_bid, palm_sites=dict(self.palm_sid))
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
        self.force_req, self.grip_manual = {}, {}     # physics: set_force (palm -> admittance), set_gripper (role -> cmd)
        self.follow_speed = 0.3   # m/s along the drawn path
        self.path_msg = None
        self.last_result = {"flags": {}, "messages": [], "feasibility": 1.0, "speed_scale": 1.0, "residuals": {}}

        self.default_rgba = m.geom_rgba.copy()
        self.joint_body = {n: m.jnt_bodyid[m.joint(n).id] for n in ARM_JOINTS}
        self.body_geoms = {b: [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_group[g] == 2] for b in range(m.nbody)}
        self.target_geoms = {h: [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(f"{h}_target").id] for h in HANDS}

    # ------------------------------------------------------------------ setup
    def home(self):
        d, di = self.data, self.ik_data
        if self.home_q is not None:
            self.configuration.update(self.home_q.copy())
        else:
            self.configuration.update_from_keyframe("stand")
        self.posture_task.set_target_from_configuration(self.configuration)
        self.pelvis_task.set_target_from_configuration(self.configuration)
        self.torso_task.set_target_from_configuration(self.configuration)
        for h, f in zip(HANDS, FEET):                    # targets from the planned pose (the plant follows it)
            self.mocap_to_site(f"{f}_target", f)
            self.mocap_to_site(f"{h}_target", self.ee_site[h])
        d.mocap_pos[self.com_mid] = di.subtree_com[self.root_bid]
        if hasattr(self, "body_home"):
            self.torso_task.set_target(self.body_home["torso"])
            self.body_now = None
            self.body_rt.reset()

    def orient(self, on):
        """Physics: grasps need the hand's orientation (jaws level, approach axis); everything else keeps the body's own
        weight on it (the G1's is tiny: mirroring and reaching move the hand freely)."""
        if self.plant is None:
            return
        base = getattr(self, "_oc_base", None)
        if base is None:
            base = self._oc_base = [float(np.max(t.orientation_cost)) for t in self.hand_tasks]
        for t, b in zip(self.hand_tasks, base):   # arms hold the jaws square to the object (a few degrees off is a missed grasp)
            t.set_orientation_cost((0.6 if b < 0.2 else max(b, 2.0)) if on else b)

    def solve(self, cfg, dt):
        """IK velocity. If the limits can't all hold at once (a joint starting outside its range, a contact the plant made),
        fall back to the joint limits only rather than stop."""
        try:
            return mink.solve_ik(cfg, self.tasks, dt, "daqp", damping=1e-1, limits=self.limits)
        except NoSolutionFound:
            try:
                return mink.solve_ik(cfg, self.tasks, dt, "daqp", damping=1e-1, limits=self.limits[:1])
            except NoSolutionFound:
                return np.zeros(cfg.model.nv)

    def mocap_to_site(self, mocap_body, site):
        """Put a target ghost (plant data) on a site of the planned pose (planner data)."""
        m, di = self.model, self.ik_data
        mid, sid = m.body(mocap_body).mocapid[0], m.site(site).id
        self.data.mocap_pos[mid] = di.site_xpos[sid]
        mujoco.mju_mat2Quat(self.data.mocap_quat[mid], di.site_xmat[sid])

    @property
    def q_des(self):
        """The planner's joint positions: what the plant tracks (P1 records it as the action). None in kinematic mode: the
        IK pose *is* the robot there, and P1 records the next state as the action instead."""
        return self.configuration.q if self.plant is not None else None

    def base_pose(self):
        """Planned pose of the floating base (the physics harness follows it), or None."""
        b = getattr(self, "root_bid", None)
        if b is None:
            return None
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, self.ik_data.xmat[b])
        return self.ik_data.xpos[b].copy(), q

    def plant_step(self, dt):
        """Advance the plant (physics) toward q_des. Kinematic mode: the IK pose is the robot, nothing to do."""
        if self.plant is not None:
            self.plant.hold_fingers = False
            self.plant.step(dt, self.configuration.q, self.base_pose())

    def safety_hold(self, dt):
        """P5 protective stop with physics: the plan stops where the robot *is* (q_des = the measured joints, taken once
        when the stop starts), and the plant keeps running (gravity, contacts) holding that pose. The gripper keeps its
        command (nothing held is dropped) and the fingers don't move. The hand targets are left alone, so a paused task
        goes on from where it was. Kinematic mode: the IK pose is the robot and it simply isn't advanced."""
        if self.plant is None:
            return
        if not getattr(self, "_safety_held", False):
            self.configuration.update(self.plant.measured_q(self.configuration.model))
            self._safety_held = True
        self.plant.hold_fingers = True
        self.plant.step(dt, self.configuration.q, self.base_pose())

    def sync_fixtures(self):
        """The fixtures' moving parts (drawers, doors) are mocap bodies in the planner's model: copy their plant poses."""
        fx = getattr(self, "fixtures", None)
        if fx is None or self.plant is None:
            return
        mi, di, d = self.configuration.model, self.ik_data, self.data
        for p in fx.parts:
            mid = mi.body_mocapid[p["bid"]]
            if mid >= 0 and p["kind"] not in ("slat", "roller"):
                di.mocap_pos[mid] = d.xpos[p["bid"]]
                di.mocap_quat[mid] = d.xquat[p["bid"]]

    def set_obstacles(self, exclude=(), near=None, radius=0.35, fixtures=False, static=False, skip_geoms=(), static_arm=False,
                      overhead=None):
        """Physics: the planner keeps the hand clear of the objects on the table (synced from the plant), except `exclude`
        (the one being picked). Objects are mocap bodies in the planner's model, so their poses are copied over.
        Generated worlds (issue #48): `fixtures` adds the walls of drawers / doors / bins nearby (their floors and the
        conveyor never count; a fixture part named in `exclude` is the one being operated); `near` (points) keeps only
        what is within `radius` of them, so a cluttered room doesn't cost the IK a thousand distance checks. `static`
        (with `near`) also adds the static furniture there (a counter's edge, a cabinet's sides: world geoms `fx_*` and
        the work surface), and the whole arm keeps off the furniture there too: a hand reaching under a counter for a
        drawer's knob must not put its wrist through the counter's edge or its forearm on the knob of the drawer above
        (the plant would stop it there). `skip_geoms`: geom names that never count (a handle the hand is coming for).
        `static_arm` (with `near`): the static furniture there for the arm links only, not the gripper (a pick reaching
        into a machine: the forearm came to rest on the CNC's side wall, the hand 15 cm over the part). `overhead` (a
        height, with `near`): static furniture there whose underside is above it counts for the gripper too (a machine's
        roof over the part in its vise: the Panda's hand came to rest on it), never what things stand on or beside.
        `overhead` may be (height, point): only what is over that point (the pick, not where it is set down)."""
        if self.plant is None:
            return
        mi, pl = self.configuration.model, self.plant
        fx = getattr(self, "fixtures", None)
        skip = {pl.objects[n]["bid"] for n in exclude if n in pl.objects}
        if fx is not None:
            skip |= {fx.by_name[n]["bid"] for n in exclude if n in fx.by_name}
        for o in pl.objects.values():
            b = o["bid"]
            mid = mi.body_mocapid[b]
            if mid >= 0:
                self.ik_data.mocap_pos[mid] = self.data.xpos[b]
                self.ik_data.mocap_quat[mid] = self.data.xquat[b]
        self.sync_fixtures()
        mujoco.mj_kinematics(mi, self.ik_data)
        targets = {b for b in range(mi.nbody) if (mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith("_target")}
        fx_bodies = {p["bid"] for p in fx.parts} if fx is not None else set()
        pts = None if near is None else np.asarray(near, float).reshape(-1, 3)

        def keep(g):
            b = mi.geom_bodyid[g]
            if b in fx_bodies:
                if not fixtures or "_floor" in (mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_GEOM, g) or "_floor"):
                    return False
            if pts is not None:
                dist = np.min(np.linalg.norm(pts - self.ik_data.geom_xpos[g], axis=1)) - mi.geom_rbound[g]
                return dist < radius
            return True
        geoms = [g for g in range(mi.ngeom) if mi.geom_contype[g] and mi.geom_bodyid[g] not in pl.robot_bodies
                 and mi.geom_bodyid[g] not in skip and mi.geom_bodyid[g] not in targets and mi.geom_bodyid[g] != 0 and keep(g)]
        if skip_geoms:
            skipped = {mi.geom(n).id for n in skip_geoms if n}
            geoms = [g for g in geoms if g not in skipped]
        if overhead is not None and not static and pts is not None:
            named = lambda g: mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            over_p = None
            if isinstance(overhead, (tuple, list)):
                overhead, over_p = float(overhead[0]), np.asarray(overhead[1], float)
            near_p = (lambda g: True) if over_p is None else (
                lambda g: float(np.linalg.norm(self.ik_data.geom_xpos[g] - over_p)) - mi.geom_rbound[g] < radius)
            for g in range(mi.ngeom):
                if mi.geom_bodyid[g] == 0 and mi.geom_contype[g] and mi.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and named(g).startswith("fx_"):
                    R = self.ik_data.geom_xmat[g].reshape(3, 3)
                    if self.ik_data.geom_xpos[g][2] - float((np.abs(R) @ mi.geom_size[g])[2]) > overhead and keep(g) and near_p(g):
                        geoms.append(g)
        if static and pts is not None:
            named = lambda g: mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            geoms += [g for g in range(mi.ngeom) if mi.geom_bodyid[g] == 0 and mi.geom_contype[g]
                      and mi.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and (named(g).startswith("fx_") or named(g) == "table") and keep(g)]
        for lim in (getattr(self, "_obstacle_limit", None), getattr(self, "_arm_obstacle_limit", None)):
            if lim in self.limits:
                self.limits.remove(lim)
        self._obstacle_limit = self._arm_obstacle_limit = None
        hand, arm, base = self._obstacle_links()
        if geoms and hand:
            self._obstacle_limit = mink.CollisionAvoidanceLimit(model=mi, geom_pairs=[(hand, geoms)], minimum_distance_from_collisions=0.012,
                                                                collision_detection_distance=0.08)
            self.limits.append(self._obstacle_limit)
        # the rest of the arm (forearm, upper arm) keeps off the objects and obstacles too, a little closer; not off a
        # generated world's furniture (fx_ bodies: counters, cabinets, drawer walls), which the arm reaches in front of and
        # into (opening a drawer, a shelf): the gripper's own limit keeps the hand clear of those
        fxb = {b for b in range(mi.nbody) if (mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith("fx_")}
        arm_geoms = geoms if static else [g for g in geoms if mi.geom_bodyid[g] not in fxb and mi.geom_bodyid[g] != 0]
        if static_arm and not static and pts is not None:
            named = lambda g: mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            arm_geoms = arm_geoms + [g for g in range(mi.ngeom) if mi.geom_bodyid[g] == 0 and mi.geom_contype[g]
                                     and mi.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and named(g).startswith("fx_") and keep(g)]
        pairs = [(arm, arm_geoms)] if arm_geoms and arm else []
        if base and (fixtures or static) and fx is not None:
            # a mobile base keeps off the furniture around it (the IK drives it to reach: it drove into the cabinets
            # going home after opening a drawer, and pushed the drawer shut again)
            bp = np.array([self.ik_data.xpos[mi.geom_bodyid[g]] for g in base]).mean(axis=0)
            named = lambda g: mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            furn = [g for g in range(mi.ngeom) if mi.geom_contype[g] and mi.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
                    and ((mi.geom_bodyid[g] == 0 and named(g).startswith("fx_")) or (mi.geom_bodyid[g] in fx_bodies and "_floor" not in named(g)))
                    and float(np.linalg.norm(self.ik_data.geom_xpos[g][:2] - bp[:2])) - mi.geom_rbound[g] < 0.6]
            if furn:
                pairs.append((base, furn))
        if pairs:
            self._arm_obstacle_limit = mink.CollisionAvoidanceLimit(model=mi, geom_pairs=pairs, minimum_distance_from_collisions=0.008,
                                                                    collision_detection_distance=0.06)
            self.limits.append(self._arm_obstacle_limit)

    def _obstacle_links(self):
        """Physics: collision geoms that keep clear of the objects: (the gripper and the last links of each arm, the rest
        of each arm up to where it joins the body: the torso, the legged body, a mobile base; a mobile base's chassis,
        which keeps off the furniture). Cached per model."""
        cache = getattr(self, "_obstacle_links_cache", None)
        if cache is not None:
            return cache
        mi, pl = self.configuration.model, self.plant
        hand = [g for r, gr in pl.grip.items() for side in gr["pads"] for g in side]
        near, upper = set(), set()
        prof = getattr(self, "profile", None)
        base_j = set(getattr(prof, "base_joints", ()) or ())
        root = getattr(prof, "root", None) or ("pelvis" if prof is None else None)
        name = lambda b: mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_BODY, b) or ""

        def stop(b):                              # the arm ends where it meets the body
            if b <= 0 or b not in pl.robot_bodies or name(b) == root or "torso" in name(b) or name(b) in ("body", "pelvis"):
                return True
            for k in range(mi.body_jntnum[b]):
                j = mi.body_jntadr[b] + k
                if mi.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE or (mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_JOINT, j) or "") in base_j:
                    return True
            return False
        for r, bodies in pl.ee_bodies.items():
            near |= bodies
            b = mi.site_bodyid[pl.site_ids[r]]
            for k in range(12):
                b = mi.body_parentid[b]
                if stop(b):
                    break
                (near if k < 3 else upper).add(b)
        col = lambda g: bool(mi.geom_contype[g] or mi.geom_conaffinity[g])
        hand += [g for g in range(mi.ngeom) if mi.geom_bodyid[g] in near and col(g) and g not in hand]
        arm = [g for g in range(mi.ngeom) if mi.geom_bodyid[g] in upper - near and col(g)]
        base = []                                 # a mobile base: the rest of the robot (its chassis)
        if prof is not None and getattr(prof, "base", "") == "mobile":
            base = [g for g in range(mi.ngeom) if mi.geom_bodyid[g] in pl.robot_bodies and mi.geom_bodyid[g] not in near | upper
                    and col(g) and g not in hand]
        self._obstacle_links_cache = (hand, arm, base)
        return hand, arm, base

    def hold_here(self):
        """Stop where the robot is: the planner takes the measured pose so q_des doesn't keep pulling the arm."""
        if self.plant is None:
            return
        self.configuration.update(self.plant.measured_q(self.configuration.model))
        for h in self.hands:
            self.mocap_to_site(f"{h}_target", self.ee_site[h])

    # ------------------------------------------------- physics adapters (P1 datasets, P2, P3 mirror, P4, P5 safety, VR)
    @staticmethod
    def _role(palm):
        return palm[:-5] if palm.endswith("_palm") else palm

    def ee_force(self):
        """End-effector contact force {role: [fx, fy, fz]} (N, world frame, smoothed): the wrist-sensor-like reading P5
        needs before it allows cooperative contact, and P1 records. None in kinematic mode: nothing measures force."""
        if self.plant is None:
            return None
        return {r: v.copy() for r, v in self.plant.ee_force.items()}

    def gripper_state(self):
        """How closed each gripper is, measured from its jaws: {role: 0 (open) .. 1 (closed)} (P1 observation.state). A jaw
        stopped by an object reads in between. None in kinematic mode (P1 then uses the task layer's grasp)."""
        if self.plant is None:
            return None
        m, d, out = self.plant.m, self.plant.d, {}
        for r, g in self.plant.grip.items():
            vals = []
            for a in g["acts"]:
                g0, b0, b1 = float(m.actuator_gainprm[a, 0]), float(m.actuator_biasprm[a, 0]), float(m.actuator_biasprm[a, 1])
                if b1 >= 0:                                   # not a position servo: no length <-> command map
                    continue
                lo, hi = (-(g0 * float(c) + b0) / b1 for c in (g["open"], g["close"]))   # servo rest length at open / closed
                if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:              # a closed jaw stops at its joint limit
                    j = m.actuator_trnid[a, 0]
                    if m.jnt_limited[j]:
                        rng = sorted(m.jnt_range[j] * m.actuator_gear[a, 0])
                        lo, hi = float(np.clip(lo, *rng)), float(np.clip(hi, *rng))
                if abs(hi - lo) > 1e-9:
                    vals.append((float(d.actuator_length[a]) - lo) / (hi - lo))
            out[r] = float(np.clip(np.mean(vals), 0.0, 1.0)) if vals else (1.0 if g["cmd"] == "close" else 0.0)
        return out

    @property
    def gripper_cmd(self):
        """The gripper command {role: 0 open | 1 closed} (P1 action). None in kinematic mode."""
        if self.plant is None:
            return None
        return {r: (1.0 if g["cmd"] == "close" else 0.0) for r, g in self.plant.grip.items()}

    @property
    def set_gripper(self):
        """Physics only: rt.set_gripper(palm, 0..1) opens / closes the real gripper (>= 0.5 closes); the object is held by
        friction and the task layer learns what both jaws hold (Tasks._manual_grip). P2 trajectory grip events, P3
        mirroring and VR teleop call it when `hasattr(rt, "set_gripper")`; in kinematic mode the attribute is absent and
        they use the task layer's kinematic grasp instead."""
        if self.plant is None:
            raise AttributeError("set_gripper: kinematic mode has no physical gripper")
        return self._set_gripper

    def _set_gripper(self, palm, value):
        role = self._role(palm)
        if role not in self.plant.grip:
            return False
        cmd = "close" if float(value) >= 0.5 else "open"
        self.plant.grip_cmd(role, cmd)
        self.grip_manual[role] = cmd
        return True

    @property
    def set_force(self):
        """Physics only: rt.set_force(palm, N) presses with that force along the approach (down) while a path is followed
        (P2 point `force`, P4 wipe / press): an admittance offset on the hand target, the loop of the press task. 0 / None
        ends it. Absent in kinematic mode (the force is only recorded there)."""
        if self.plant is None:
            raise AttributeError("set_force: kinematic mode measures no force")
        return self._set_force

    def _set_force(self, palm, value):
        try:
            v = 0.0 if value is None else float(value)
        except (TypeError, ValueError):
            v = 0.0
        if not np.isfinite(v) or v <= 0.0:
            self.force_req.pop(palm, None)
        else:
            self.force_req.setdefault(palm, {"dz": 0.0, "peak": 0.0})["want"] = min(v, 200.0)
        return True

    def force_tick(self, dt):
        """Admittance on the followed path (see Tasks._press_tick): lower the hand target until the surface pushes back
        with the wanted force, back off near 0.9 x the impedance limit; within 3 cm of the path (a path drawn into the
        surface lifts off it). Ends with the path."""
        if not self.force_req:
            return
        if self.plant is None or self.mode != "FOLLOW" or self.path is None:
            self.force_req.clear()
            return
        pl = self.plant
        lim = pl.params["force_limit"] if pl.params["impedance"] else float("inf")
        for palm, fr in self.force_req.items():
            f = float(pl.ee_force_raw.get(self._role(palm), np.zeros(3))[2])       # what the surface pushes up with
            fr["peak"] = max(fr["peak"], f)
            want = min(fr["want"], 0.8 * lim)
            v = -0.03 if f > 0.9 * lim else float(np.clip(0.002 * (want - f), -0.03, 0.01))
            fr["dz"] = float(np.clip(fr["dz"] + v * dt, -0.03, 0.03))
            self.data.mocap_pos[self.hands_mid[palm]] -= np.array([0.0, 0.0, fr["dz"]])

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
        self.path = {"pts": pts, "ok": ok, "palm": palm, "cum": cum, "i": 0, "hand": path.get("hand", "r")}
        bad = int(sum(1 for o in ok if not o))
        side = "Left" if palm.startswith("left") else "Right"
        self.path_msg = (f"{side} hand path: {len(pts)} waypoints, all reachable. Press F to follow."
                         if bad == 0 else f"{side} hand path: {bad}/{len(pts)} waypoints out of reach (red). Redraw or press F.")

    def _trace(self, cfg, task, target, n, dt):
        """n IK steps of a scratch configuration toward target. A planning job may hand the server loop a frame between
        them (pps/slicing.py; the loop's own step sets the hand targets meanwhile, so the target is set again after)."""
        pause_point()
        task.set_target(target)
        for i in range(n):
            if i and i % 2 == 0:
                pause_point()
                task.set_target(target)
            cfg.integrate_inplace(self.solve(cfg, dt), dt)

    def precheck(self, pts, palm, iters=10, dt=0.03, quats=None):
        """Trace the path with IK on a scratch configuration; a waypoint is ok if the hand gets within reach_tol.
        quats: an orientation per waypoint (wxyz: grasps, P2 trajectories); default: the hand target's current one."""
        cfg = mink.Configuration(self.configuration.model)
        cfg.data.mocap_pos[:], cfg.data.mocap_quat[:] = self.ik_data.mocap_pos, self.ik_data.mocap_quat   # objects (physics)
        cfg.update(self.plant.measured_q(cfg.model) if self.plant is not None else self.configuration.q.copy())   # physics: from
        # where the arm really is (it may trail the plan)
        task = self.hand_tasks[self.hands.index(palm)]
        saved = task.transform_target_to_world
        mid = self.hands_mid[palm]
        quat = self.data.mocap_quat[mid].copy()
        ok, dists, poses = [], [], []
        if quats is not None:
            quats = [np.asarray(q, float) for q in quats]
        start = cfg.data.site_xpos[self.palm_sid[palm]].copy()     # far first waypoint: lead in from the hand (unscored),
        n_lead = int(np.linalg.norm(np.asarray(pts[0]) - start) / 0.035)   # the IK moves joints no faster than the robot
        for j in range(1, n_lead):
            self._trace(cfg, task, mink.SE3.from_rotation_and_translation(mink.SO3(quat if quats is None else quats[0]),
                                                                          start + (np.asarray(pts[0]) - start) * j / n_lead), iters, dt)
        for k, p in enumerate(pts):
            n = iters
            if quats is not None and k > 0:       # the hand turns slower than it moves: more time where it turns a lot
                ang = 2 * np.arccos(min(1.0, abs(float(np.dot(quats[k - 1], quats[k])))))
                n = iters * (1 + int(ang / 0.25))
            self._trace(cfg, task, mink.SE3.from_rotation_and_translation(mink.SO3(quat if quats is None else quats[k]), p), n, dt)
            dist = float(np.linalg.norm(cfg.data.site_xpos[self.palm_sid[palm]] - p))
            dists.append(dist)
            ok.append(dist < self.validator.reach_tol)
            poses.append(cfg.q.copy())
        if saved is not None:
            task.set_target(saved)
        self.last_precheck_dists = dists
        self.last_precheck_q = poses           # the planner's posture at each waypoint (P2: the hand's real extent there)
        return ok

    # ------------------------------------------------------------- whole body
    def apply_body(self, b):
        """b = {"squat": 0..0.45, "lift": {"l": xyz, "r": xyz} (human side), "yaw": rad} or None (home)."""
        d, home = self.data, self.body_home
        com = home["com"].copy()
        feet = {f: p.copy() for f, p in home["feet"].items()}
        yaw = 0.0
        if b is not None:
            com[2] -= 0.42 * float(b["squat"])
            for hs, off in b["lift"].items():
                off = np.asarray(off, float)
                if np.linalg.norm(off) < 1e-3:
                    continue
                foot = ("left_foot" if hs == "r" else "right_foot") if self.retargeter.mirror else ("right_foot" if hs == "r" else "left_foot")
                feet[foot] = feet[foot] + off
                stance = "right_foot" if foot == "left_foot" else "left_foot"
                w = float(np.clip(off[2] / 0.05, 0.0, 1.0))
                com[:2] = (1 - w) * com[:2] + w * home["feet"][stance][:2]
            yaw = float(b["yaw"])
        d.mocap_pos[self.com_mid] = com
        for f, mid in zip(FEET, self.feet_mid):
            d.mocap_pos[mid] = feet[f]
        t0 = home["torso"]
        self.torso_task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_z_radians(yaw) @ t0.rotation(), t0.translation()))
        self.body_now = None if b is None else {"com": com.tolist(), "left_foot": feet["left_foot"].tolist(),
                                                "right_foot": feet["right_foot"].tolist(), "yaw": yaw}

    def apply_body_frame(self, bf):
        if not bf:
            return self.apply_body(None)
        d = self.data
        d.mocap_pos[self.com_mid] = bf["com"]
        for f, mid in zip(FEET, self.feet_mid):
            d.mocap_pos[mid] = bf[f]
        t0 = self.body_home["torso"]
        self.torso_task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_z_radians(bf["yaw"]) @ t0.rotation(), t0.translation()))
        self.body_now = bf

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
                if self.new_pose:
                    self.apply_body(self.body_rt(self.last_lm, self.retargeter.mirror))
            elif self.body_now is not None:
                self.apply_body(None)
            # else: leave mocap where it is -> user may drag the ghosts in the viewer.
        elif self.mode == "FOLLOW" and self.path is not None and self.path.get("timed"):
            follow_step(self, dt)       # P2 trajectory: timed samples with orientation and gripper (pps/traj.py)
        elif self.mode == "FOLLOW" and self.path is not None:
            pts, cum = self.path["pts"], self.path["cum"]
            lag = 0.0
            if self.plant is not None:                    # physics: the plan doesn't run away from the arm (it waits while
                mid = self.hands_mid[self.path["palm"]]   # the hand is more than FOLLOW_LAG behind, up to FOLLOW_WAIT s)
                lag = float(np.linalg.norm(d.site_xpos[self.palm_sid[self.path["palm"]]] - d.mocap_pos[mid]))
                self.lag_t = (getattr(self, "lag_t", 0.0) + dt) if lag > FOLLOW_LAG else 0.0
            if not getattr(self, "follow_hold", False) and (lag <= FOLLOW_LAG or self.lag_t > FOLLOW_WAIT):
                # the last few cm into a grasp (x0.35); a drawer / door being moved (its own factor, a third element)
                slow = min([s[2] if len(s) > 2 else 0.35 for s in self.path.get("slow", ()) if s[0] <= self.path["i"] < s[1]] or [1.0])
                sf = self.path.get("sf")                  # physics: segments where the hand turns a lot go slower
                k = 1.0 if sf is None else float(sf[min(self.path["i"], len(sf) - 1)])
                self.follow_s += dt * self.follow_speed * self.speed_scale * slow * k
            i = int(np.searchsorted(cum, self.follow_s, side="right")) - 1
            i = max(0, min(i, len(pts) - 2))
            seg = cum[i + 1] - cum[i]
            a = 0.0 if seg < 1e-9 else min(1.0, (self.follow_s - cum[i]) / seg)
            d.mocap_pos[self.hands_mid[self.path["palm"]]] = (1 - a) * pts[i] + a * pts[i + 1]
            if self.path.get("q") is not None:          # an orientation per waypoint (grasps): slerp between them
                d.mocap_quat[self.hands_mid[self.path["palm"]]] = slerp(self.path["q"][i], self.path["q"][i + 1], a)
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
                if palm in self.hands_mid:
                    d.mocap_pos[self.hands_mid[palm]] = np.asarray(tgt)
            self.apply_body_frame(frames[self.preview_idx].get("body"))
            if self.preview_t >= self.recorder.duration:
                self.mode = "MIRROR"
                self.apply_body(None)

    def step(self, dt):
        d = self.data
        s = float(self.safety_scale)
        if s <= 0.0:                      # protective stop: targets, IK and every motion source hold still
            self.safety_hold(dt)          # physics: the plant keeps running and holds the measured pose
            return self.last_result
        self._safety_held = False
        self.update_targets(dt * s)       # paths, skills, previews advance at the safe speed
        self.force_tick(dt * s)           # physics: a path point's press force (set_force)
        self.com_task.set_target(d.mocap_pos[self.com_mid])
        for task, mid in zip(self.feet_tasks, self.feet_mid):
            task.set_target(mink.SE3.from_mocap_id(d, mid))
        for task, h in zip(self.hand_tasks, self.hands):
            task.set_target(mink.SE3.from_mocap_id(d, self.hands_mid[h]))
        vel = self.solve(self.configuration, dt) * s
        self.configuration.integrate_inplace(vel, dt)   # s < 1: joints (and live mirroring) move slower too
        self.plant_step(dt)                              # the plant runs in real time and tracks the (slower) plan
        mujoco.mj_camlight(self.model, d)
        targets = {h: d.mocap_pos[self.hands_mid[h]].copy() for h in self.hands}
        res = self.validator.evaluate(targets, vel)
        if self.mode == "FOLLOW":
            self.recorder.add(self.last_lm, targets, self.last_grip, d.qpos[:getattr(self, "nq_r", d.qpos.shape[0])], res["flags"], res["residuals"], res["speed_scale"], {"source": "drawn_path"})
        if self.mode == "REC" and self.new_pose:
            self.recorder.add(self.last_lm, targets, self.last_grip, d.qpos[:getattr(self, "nq_r", d.qpos.shape[0])], res["flags"], res["residuals"], res["speed_scale"], self.last_context)
            self.recorder.frames[-1]["body"] = self.body_now
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
        self.paint_model()
        self.paint_overlays(viewer.user_scn, reset=True)

    def paint_model(self):
        """Colour the real model: ghost targets by reachability, links at limits red, collision risks orange."""
        m, res = self.model, self.last_result
        m.geom_rgba[:] = self.default_rgba
        for h in self.hands:
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
                if bid in self.body_geoms and self.body_geoms[bid]:
                    m.geom_rgba[self.body_geoms[bid]] = (1.0, 0.6, 0.1, 1.0)

    def paint_overlays(self, scn, reset):
        """Add path spheres/segments and unreachable connectors to an mjvScene (viewer.user_scn or a Renderer scene)."""
        d, res = self.data, self.last_result
        if reset:
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

        for h in self.hands:
            if res["flags"].get(h) == "unreachable":
                add_capsule(d.site_xpos[self.palm_sid[h]], d.mocap_pos[self.hands_mid[h]], 0.006, (1.0, 0.2, 0.2, 0.9))
        if self.path is not None:
            pts, ok, cur = self.path["pts"], self.path["ok"], self.path["i"]
            for k in range(len(pts)):
                rgba = (0.3, 0.9, 0.4, 0.9) if ok[k] else (1.0, 0.25, 0.25, 0.9)
                if k == cur and self.mode == "FOLLOW":
                    rgba = (1.0, 0.85, 0.2, 1.0)
                add_sphere(pts[k], 0.008 if k != cur else 0.016, rgba)
                if k + 1 < len(pts):
                    seg_rgba = (0.3, 0.9, 0.4, 0.7) if (ok[k] and ok[k + 1]) else (1.0, 0.25, 0.25, 0.7)
                    add_capsule(pts[k], pts[k + 1], 0.003, seg_rgba)


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
