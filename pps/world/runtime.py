"""Runtime for any robot body (arms, two-arm rigs, mobile manipulators, legged bodies) built by pps.world.scene.

Same interface as the G1 Runtime (pps/robot_runtime.py), so tasks / app / features work unchanged:
end-effector FrameTasks (keyed "right_palm" / "left_palm" by role) + PostureTask + ConfigurationLimit
(+ arm-vs-table collision avoidance), legged bodies add feet / CoM / root tasks, mobile bases include their base
joints in the IK with a damping cost so the arm moves first and the base drives only when it has to.
Mirroring a person (follow me), whole-body retargeting and skill recording are G1 features: here they are no-ops.
"""
import mink
import mujoco
import numpy as np

from .. import physics as phys
from ..ir import IntentRecorder
from ..robot_runtime import Runtime
from .validate import BodyValidator


class NullRetargeter:
    """Stands in for the human->robot retargeter on bodies that don't mirror a person."""

    def __init__(self, shoulders):
        self.mirror = True
        self.robot_shoulders = shoulders

    def reset(self):
        pass

    def targets(self, lm):
        return {}

    def map_point(self, pt, hand, lm):
        return None, None


def _down_quat(xmat, axis):
    """Orientation (wxyz) that turns the site's approach axis straight down, keeping its heading."""
    R = np.asarray(xmat, float).reshape(3, 3)
    a = R @ np.asarray(axis, float)
    a /= np.linalg.norm(a)
    down = np.array([0.0, 0.0, -1.0])
    v, c = np.cross(a, down), float(np.dot(a, down))
    if np.linalg.norm(v) < 1e-9:
        Ra = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Ra = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, (Ra @ R).flatten())
    return q


def _frame_pose(m, d, name, typ):
    if typ == "site":
        i = m.site(name).id
        return d.site_xpos[i].copy(), d.site_xmat[i].copy()
    if typ == "geom":
        i = m.geom(name).id
        return d.geom_xpos[i].copy(), d.geom_xmat[i].copy()
    i = m.body(name).id
    return d.xpos[i].copy(), d.xmat[i].copy()


def _direction(v):
    x, y = float(v[0]), float(v[1])
    if abs(x) >= abs(y):
        return "forward" if x > 0 else "back"
    return "to the left" if y > 0 else "to the right"


class BodyRuntime(Runtime):
    def __init__(self, world, mirror=True, plant=None):   # noqa: C901  (deliberately does not call Runtime.__init__)
        p, m = world.profile, world.model
        mi = world.ik_model if world.ik_model is not None else m      # the IK planner's model (physics) or the same model
        self.profile, self.profile_id, self.capabilities = p, p.id, tuple(p.capabilities)
        self.model = m
        self.home_q = None if world.home_q is None else np.asarray(world.home_q, float).copy()
        self.configuration = mink.Configuration(mi)
        self.ik_data = self.configuration.data
        self.plant = plant
        self.data = d = plant.d if plant is not None else self.ik_data
        self.hands = [f"{r}_palm" for r in p.ee]
        grips = (p.physics or {}).get("grip") or {} if plant is not None else {}
        self.ee_site = {f"{r}_palm": (f"{r}_tcp" if r in grips else s) for r, s in p.ee.items()}
        self.robot_roots = world.robot_roots
        robot_bodies = world.robot_bodies

        # ---- joints: frozen (fingers, heads, wheels), base (mobile), free (legged)
        frozen_dofs, base_dofs, free_dofs, arm_joints = [], [], [], []
        for j in range(mi.njnt):
            name = mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
            if mi.jnt_bodyid[j] not in robot_bodies:
                continue
            dof, jt = mi.jnt_dofadr[j], mi.jnt_type[j]
            if jt == mujoco.mjtJoint.mjJNT_FREE:
                free_dofs += list(range(dof, dof + 6))
            elif name in p.base_joints:
                base_dofs.append(dof)
            elif any(f in name for f in p.freeze) or name.startswith("pc_"):
                frozen_dofs.append(dof)                  # fingers (incl. the built grippers' pads) belong to the gripper
            else:
                arm_joints.append(name)
        cost = np.full(mi.nv, 5e-2 if p.base == "legged" else 1e-2)
        cost[frozen_dofs] = 50.0
        cost[base_dofs] = 0.0
        self.posture_task = mink.PostureTask(mi, cost=cost)
        pc, oc = p.hand_cost
        self.hand_tasks = [mink.FrameTask(self.ee_site[h], "site", position_cost=pc, orientation_cost=oc, lm_damping=1.0)
                           for h in self.hands]
        self.tasks = [self.posture_task, *self.hand_tasks]
        self.feet = [f for f, _ in p.feet]
        self.feet_tasks, self.com_task, self.root_task, self.torso_task = [], None, None, None
        if p.base == "legged":
            quad = len(self.feet) == 4
            self.root_task = mink.FrameTask(p.root, "body", position_cost=1.0 if quad else 0.0, orientation_cost=1.0, lm_damping=1.0)
            self.feet_tasks = [mink.FrameTask(f, typ, position_cost=10.0, orientation_cost=0.0 if quad else 1.0, lm_damping=1.0)
                               for f, typ in p.feet]
            self.tasks += [self.root_task, *self.feet_tasks]
            if p.torso:
                self.torso_task = mink.FrameTask(p.torso, "body", position_cost=0.0, orientation_cost=1.0, lm_damping=1.0)
                self.tasks.append(self.torso_task)
            if not quad:
                self.com_task = mink.ComTask(cost=10.0)
                self.tasks.append(self.com_task)
        if base_dofs and p.base_cost > 0:
            dc = np.zeros(mi.nv)
            dc[base_dofs] = p.base_cost
            self.tasks.append(mink.DampingTask(mi, dc))
        # explicit driving (drive_base): a posture task on the base joints only, switched on while a drive runs
        self.base_dofs = base_dofs
        self.base_joints = [mi.joint(n).id for n in p.base_joints if n in [mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_JOINT, j)
                                                                              for j in range(mi.njnt)]]
        self.drive_goal = None
        self.base_task = None
        if self.base_joints:
            self.base_task = mink.PostureTask(mi, cost=np.zeros(mi.nv))
            self.tasks.append(self.base_task)
        self.limits = [mink.ConfigurationLimit(mi)]
        if plant is not None:                            # the planner moves no faster than the plant's servos follow
            self.limits.append(phys.velocity_limit(mi, robot_bodies, p.freeze))
        pairs = self._collision_pairs(robot_bodies) if p.base != "legged" else []
        if pairs:
            self.limits.append(mink.CollisionAvoidanceLimit(model=mi, geom_pairs=pairs, minimum_distance_from_collisions=0.005,
                                                            collision_detection_distance=0.06))
        self.hands_mid = {h: m.body(f"{h}_target").mocapid[0] for h in self.hands}
        self.palm_sid = {h: m.site(self.ee_site[h]).id for h in self.hands}
        self.feet_mid = [m.body(f"{f}_target").mocapid[0] for f in self.feet]
        self.com_mid = m.body("com_target").mocapid[0] if self.com_task is not None else None
        self.root_bid = m.body(p.root).id if p.root else None
        self.home()
        if plant is not None:
            plant.reset(self.configuration.q)
            plant.settle()

        # arm "shoulders": the first moving body of each arm chain (used to suggest a nearer spot)
        shoulders, di = {}, self.ik_data
        for h in self.hands:
            b = m.site_bodyid[self.palm_sid[h]]
            chain = []
            while b != 0 and b in robot_bodies:
                chain.append(b)
                b = m.body_parentid[b]
            arm = [c for c in chain if m.body_jntnum[c] > 0 and not any(
                m.jnt_type[m.body_jntadr[c] + k] == mujoco.mjtJoint.mjJNT_FREE or
                (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.body_jntadr[c] + k) or "") in p.base_joints
                or (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.body_jntadr[c] + k) or "").startswith("pc_")
                for k in range(m.body_jntnum[c]))]
            shoulders[h] = di.xpos[arm[-1] if arm else chain[-1]].copy()
        self.retargeter = NullRetargeter(shoulders)
        self.body_rt = None
        self.body_home = {"com": None if self.com_mid is None else d.mocap_pos[self.com_mid].copy(),
                          "feet": {f: d.mocap_pos[mid].copy() for f, mid in zip(self.feet, self.feet_mid)}, "torso": None}
        self.body_now = None
        self.validator = BodyValidator(mi, self.ik_data, self.palm_sid, arm_joints, pairs,
                                       feet=[(f, t) for f, t in p.feet] if p.base == "legged" else [],
                                       root_body=self.root_bid or 1, reach_tol=p.reach_tol)
        names = [mujoco.mj_id2name(mi, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(mi.njnt)]
        self.recorder = IntentRecorder(p.id, names, mirror)

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
        self.path = None
        self.follow_s = 0.0
        self.follow_speed = 0.3
        self.force_req, self.grip_manual = {}, {}     # physics: set_force (palm -> admittance), set_gripper (role -> cmd)
        self.path_msg = None
        self.last_result = {"flags": {}, "messages": [], "feasibility": 1.0, "speed_scale": 1.0, "residuals": {}}

        self.default_rgba = m.geom_rgba.copy()
        visual = [g for g in range(m.ngeom) if m.geom_group[g] in (0, 1, 2) and m.geom_bodyid[g] in robot_bodies]
        self.joint_body = {n: m.jnt_bodyid[m.joint(n).id] for n in arm_joints}
        self.body_geoms = {b: [g for g in visual if m.geom_bodyid[g] == b] for b in range(m.nbody)}
        self.target_geoms = {h: [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(f"{h}_target").id] for h in self.hands}

    def _collision_pairs(self, robot_bodies):
        """Collision geoms on the last links of each arm vs the work surface (cheap: a handful of pairs)."""
        m = self.model
        try:
            table = m.geom("table").id
        except KeyError:
            return []
        out = []
        for h in self.hands:
            b, links = m.site_bodyid[m.site(self.ee_site[h]).id], []
            while b != 0 and b in robot_bodies and len(links) < 4:
                links.append(b)
                b = m.body_parentid[b]
            geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in links and m.geom_group[g] == 3
                     and (m.geom_contype[g] or m.geom_conaffinity[g])]
            if geoms:
                out.append((geoms, [table]))
        return out

    # ------------------------------------------------------------------ setup
    def home(self):
        m, d, p, di = self.model, self.data, self.profile, self.ik_data      # targets come from the planned pose
        mi = self.configuration.model
        if self.home_q is not None:
            self.configuration.update(self.home_q.copy())
        else:
            self.configuration.update(mi.qpos0.copy())
        self.posture_task.set_target_from_configuration(self.configuration)
        if self.root_task is not None:
            self.root_task.set_target_from_configuration(self.configuration)
        if self.torso_task is not None:
            self.torso_task.set_target_from_configuration(self.configuration)
        for f, (fname, typ) in zip(self.feet, p.feet):
            pos, mat = _frame_pose(mi, di, fname, typ)
            mid = m.body(f"{f}_target").mocapid[0]
            d.mocap_pos[mid] = pos
            mujoco.mju_mat2Quat(d.mocap_quat[mid], mat)
        if self.com_mid is not None:
            d.mocap_pos[self.com_mid] = di.subtree_com[1]
        if getattr(self, "base_task", None) is not None:
            self.drive_goal = None
            self.base_task.set_cost(np.zeros(mi.nv))
            self.base_task.set_target_from_configuration(self.configuration)
            self.base_home = di.xpos[mi.jnt_bodyid[self.base_joints[0]]].copy()
        for h in self.hands:
            sid = self.palm_sid[h]
            mid = self.hands_mid[h]
            d.mocap_pos[mid] = di.site_xpos[sid]
            if p.ee_down:
                d.mocap_quat[mid] = _down_quat(di.site_xmat[sid], p.ee_axis)
            else:
                mujoco.mju_mat2Quat(d.mocap_quat[mid], di.site_xmat[sid])
        self.body_now = None

    # whole-body mirroring is a G1 feature
    def update_targets(self, dt):
        if self.mode in ("MIRROR", "REC"):
            return                      # nothing to mirror: targets stay where the task layer (or a drag) left them
        super().update_targets(dt)

    def apply_body(self, b):
        self.body_now = None

    def apply_body_frame(self, bf):
        self.body_now = None

    def handle_key(self, key):
        if key in ("R", "M"):          # recording a mirrored motion / mirror toggle: G1 only
            return
        super().handle_key(key)

    # ------------------------------------------------------------------ drive
    DRIVE_SPEED, TURN_SPEED, DRIVE_MAX = 0.25, 0.8, 0.6

    def start_drive(self, dx=0.0, dy=0.0, dyaw_deg=0.0):
        """Drive the base by (dx, dy) metres in the world and dyaw degrees. -> (ok, message). The arm targets ride along."""
        m, d = self.configuration.model, self.ik_data         # the planned base (the plant follows it)
        if not self.base_joints:
            return False, "I have no base to drive."
        want = np.array([float(dx or 0.0), float(dy or 0.0), 0.0])
        n = np.linalg.norm(want)
        if n > self.DRIVE_MAX:
            want *= self.DRIVE_MAX / n
        bb = m.jnt_bodyid[self.base_joints[0]]
        note = ""
        slides = [d.xaxis[j] for j in self.base_joints if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE]
        if abs(want[0]) > 0.005 and not any(abs(ax[0]) > 0.3 for ax in slides):
            want[0], note = 0.0, " My base only drives along the table, not towards it."
        room_x = self.base_home[0] + 0.05 - d.xpos[bb][0]      # never into the table
        if want[0] > room_x and want[0] > 0.005:
            want[0], note = max(0.0, room_x), " I stop before the table."
        lo, hi = self.base_home[1] - 0.6 - d.xpos[bb][1], self.base_home[1] + 0.6 - d.xpos[bb][1]
        if not lo <= want[1] <= hi:
            want[1], note = float(np.clip(want[1], lo, hi)), note + " I stay along the table."
        yaw = np.radians(float(np.clip(float(dyaw_deg or 0.0), -90, 90)))
        goal, got, turned = {}, np.zeros(3), 0.0
        for j in self.base_joints:
            adr = m.jnt_qposadr[j]
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE:
                ax = d.xaxis[j].copy()
                step = float(np.dot(want, ax))
                goal[adr] = d.qpos[adr] + step
                got += step * ax
            elif m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                goal[adr] = d.qpos[adr] + yaw
                turned = yaw
        missing = float(np.linalg.norm(want - got))
        if np.linalg.norm(got) < 0.005 and abs(turned) < 1e-3:
            return False, (note.strip() or "My base can't move that way" + (" (it only drives along its own axis)." if missing > 0.005 else "."))
        self.drive_goal = goal
        cost = np.zeros(m.nv)
        cost[self.base_dofs] = 20.0
        self.base_task.set_cost(cost)
        parts = []
        if np.linalg.norm(got) >= 0.005:
            parts.append(f"driving {np.linalg.norm(got) * 100:.0f} cm " + _direction(got))
        if abs(turned) > 1e-3:
            parts.append(f"turning {abs(np.degrees(turned)):.0f}° {'left' if turned > 0 else 'right'}")
        msg = " and ".join(parts)
        msg = msg[:1].upper() + msg[1:] + "." + note
        if missing > 0.01:
            msg += f" My base only drives along its own axis, so {missing * 100:.0f} cm of that is left out."
        return True, msg

    def _drive_tick(self, dt):
        m = self.configuration.model
        if self.drive_goal is None:
            return
        q = self.configuration.q.copy()
        tgt = q.copy()
        done = True
        for j in self.base_joints:
            adr = m.jnt_qposadr[j]
            g = self.drive_goal[adr]
            v = self.DRIVE_SPEED if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE else self.TURN_SPEED
            err = g - q[adr]
            tgt[adr] = q[adr] + float(np.clip(err, -v * dt, v * dt))
            done &= abs(err) < 0.004
        if done:
            self.drive_goal = None
            self.base_task.set_cost(np.zeros(m.nv))
            return
        self.base_task.set_target(tgt)

    @property
    def driving(self):
        return self.drive_goal is not None

    # ------------------------------------------------------------------- step
    def step(self, dt):
        d, di = self.data, self.ik_data
        s = float(self.safety_scale)                     # P5 safety: 0 = protective stop (see Runtime.step)
        if s <= 0.0:
            self.safety_hold(dt)                         # physics: the plant holds the measured pose
            return self.last_result
        self._safety_held = False
        self.update_targets(dt * s)
        self.force_tick(dt * s)                          # physics: a path point's press force (set_force)
        ride = None
        if self.drive_goal is not None:                  # the hand targets ride along with the (planned) base
            bb = self.model.jnt_bodyid[self.base_joints[0]]
            ride = (bb, di.xpos[bb].copy(), di.xmat[bb].reshape(3, 3).copy())
            self._drive_tick(dt * s)
        if self.com_task is not None:
            self.com_task.set_target(d.mocap_pos[self.com_mid])
        for task, mid in zip(self.feet_tasks, self.feet_mid):
            task.set_target(mink.SE3.from_mocap_id(d, mid))
        for task, h in zip(self.hand_tasks, self.hands):
            task.set_target(mink.SE3.from_mocap_id(d, self.hands_mid[h]))
        vel = self.solve(self.configuration, dt) * s
        self.configuration.integrate_inplace(vel, dt)
        self.plant_step(dt)
        if ride is not None:
            bb, p0, R0 = ride
            p1, R1 = di.xpos[bb], di.xmat[bb].reshape(3, 3)
            dR, dq = R1 @ R0.T, np.zeros(4)
            mujoco.mju_mat2Quat(dq, dR.flatten())
            for h in self.hands:
                mid = self.hands_mid[h]
                d.mocap_pos[mid] = p1 + dR @ (d.mocap_pos[mid] - p0)
                q = np.zeros(4)
                mujoco.mju_mulQuat(q, dq, d.mocap_quat[mid].copy())
                d.mocap_quat[mid] = q
        mujoco.mj_camlight(self.model, d)
        targets = {h: d.mocap_pos[self.hands_mid[h]].copy() for h in self.hands}
        res = self.validator.evaluate(targets, vel)
        if self.mode == "FOLLOW":
            self.recorder.add(self.last_lm, targets, self.last_grip, d.qpos[:getattr(self, "nq_r", d.qpos.shape[0])], res["flags"], res["residuals"], res["speed_scale"],
                              {"source": "drawn_path"})
        self.new_pose = False
        self.last_result = res
        return res


def make_runtime(world, mirror=True):
    """World (pps.world.scene.build) -> runtime. The G1 keeps its own runtime (mirroring, whole-body retargeting).
    A world built with physics gets a plant (pps.physics.Plant) that the runtime steps after every IK step."""
    p = world.profile
    plant, ee = None, None
    if world.physics:
        grips = (p.physics or {}).get("grip") or {}
        ee = {r: (f"{r}_tcp" if r in grips else s) for r, s in p.ee.items()}
        plant = phys.Plant(world.model, world.ik_model, p, world.kinds, world.robot_bodies, ee)
    if p.id == "g1":
        rt = Runtime(mirror=mirror, model=world.model, home_q=world.home_q, ik_model=world.ik_model, plant=plant,
                     ee_site=None if ee is None else {f"{r}_palm": s for r, s in ee.items()})
    else:
        rt = BodyRuntime(world, mirror=mirror, plant=plant)
    rt.profile = p
    rt.label = p.label
    rt.nq_r = rt.configuration.model.nq
    return rt
