"""What one dataset frame is for the current robot body: a tiny adapter between the runtime and the recorder.

It reads through attributes that exist today (kinematic IK runtime) and picks up P0 physics automatically when present:

    observation.state   the robot's joint positions (every 1-dof joint of the robot except frozen ones: fingers, heads,
                        wheels, as listed in the profile's `freeze`; the floating base of a legged body is left out),
                        then one gripper value per end effector (0 = open, 1 = closed).
                        gripper: P0 `rt.gripper_state()` -> {role: 0..1} (or a dict attribute `rt.gripper`), else the
                        task layer's grasp state (holding something = 1).
    action              P0: `rt.q_des` (desired joint positions, an nq-long array in qpos order, or {joint name: value})
                        for the same joints, plus the gripper command (`rt.gripper_cmd` {role: 0..1}, else the grasp
                        state). Kinematic mode (no q_des): the IK solution the robot reaches at the *next* sample, i.e.
                        the joint target for this step (the recorder fills it in one frame later; see recorder.py).
    observation.ee_force  P0 `rt.ee_force()` (or attribute) / ctx.store["physics"]["ee_force"] -> {role: [fx, fy, fz]};
                        the feature exists only when a force reading is available when recording starts.

Names are stable and human-readable (`joint:<name>`, `gripper:<role>`) and go to meta/info.json `names`.
"""
import mujoco
import numpy as np

HINGE_SLIDE = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))


def _call(v):
    try:
        return v() if callable(v) else v
    except Exception:
        return None


class RobotAdapter:
    def __init__(self, ctx):
        rt, world = ctx.rt, getattr(ctx, "world", None)
        m = rt.model
        self.rt, self.model = rt, m
        prof = getattr(rt, "profile", None) or getattr(world, "profile", None)
        self.robot_id = getattr(prof, "id", getattr(rt, "profile_id", "robot"))
        freeze = tuple(getattr(prof, "freeze", ()) or ())
        bodies = set(getattr(world, "robot_bodies", None) or ())
        if not bodies:                                   # fallback: everything under the robot roots
            roots = set(getattr(rt, "robot_roots", ()) or ())
            bodies = {b for b in range(1, m.nbody) if int(m.body_rootid[b]) in roots}
        self.joints, self.qadr = [], []
        for j in range(m.njnt):
            if int(m.jnt_type[j]) not in HINGE_SLIDE or int(m.jnt_bodyid[j]) not in bodies:
                continue
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint{j}"
            if any(f in name for f in freeze):
                continue
            self.joints.append(name)
            self.qadr.append(int(m.jnt_qposadr[j]))
        self.qadr = np.array(self.qadr, int)
        self.hands = list(getattr(rt, "hands", []))
        self.roles = [("left" if h.startswith("left") else "right") for h in self.hands]
        self.names = [f"joint:{n}" for n in self.joints] + [f"gripper:{r}" for r in self.roles]
        self.has_force = self.ee_force(ctx) is not None

    # ---------------------------------------------------------------- reads
    @property
    def dim(self):
        return len(self.names)

    def _grip_from_tasks(self, tasks):
        hold = getattr(tasks, "holding", {}) or {}
        return [1.0 if hold.get(h) else 0.0 for h in self.hands]

    def gripper(self, tasks):
        g = _call(getattr(self.rt, "gripper_state", None))
        if not isinstance(g, dict):
            g = getattr(self.rt, "gripper", None)
        if isinstance(g, dict) and g:
            return [float(np.clip(float(g.get(r, g.get(h, 0.0)) or 0.0), 0.0, 1.0)) for r, h in zip(self.roles, self.hands)]
        return self._grip_from_tasks(tasks)

    def gripper_cmd(self, tasks):
        g = getattr(self.rt, "gripper_cmd", None)
        g = _call(g) if callable(g) else g
        if isinstance(g, dict) and g:
            return [float(np.clip(float(g.get(r, g.get(h, 0.0)) or 0.0), 0.0, 1.0)) for r, h in zip(self.roles, self.hands)]
        return self._grip_from_tasks(tasks)

    def joint_q(self):
        return np.asarray(self.rt.data.qpos, np.float64)[self.qadr]

    def state(self, tasks):
        return np.concatenate([self.joint_q(), self.gripper(tasks)]).astype(np.float32)

    def q_des(self):
        """P0's desired joint positions for our joints, or None in kinematic mode."""
        qd = getattr(self.rt, "q_des", None)
        if qd is None:
            return None
        if isinstance(qd, dict):
            try:
                return np.array([float(qd[n]) for n in self.joints])
            except (KeyError, TypeError, ValueError):
                return None
        qd = np.asarray(qd, np.float64).reshape(-1)
        if qd.shape[0] == self.model.nq:
            return qd[self.qadr]
        if qd.shape[0] == len(self.joints):
            return qd
        return None

    def action(self, tasks):
        """-> (action vector, from_q_des). Without q_des the joint part is the current IK solution; the recorder
        replaces it with the next sample's (see the module docstring)."""
        qd = self.q_des()
        joints = qd if qd is not None else self.joint_q()
        return np.concatenate([joints, self.gripper_cmd(tasks)]).astype(np.float32), qd is not None

    def ee_force(self, ctx):
        f = _call(getattr(self.rt, "ee_force", None))
        if not isinstance(f, dict):
            ph = ctx.store.get("physics") if hasattr(ctx, "store") else None
            f = ph.get("ee_force") if isinstance(ph, dict) else None
        if not isinstance(f, dict) or not f:
            return None
        out = []
        for r, h in zip(self.roles, self.hands):
            v = f.get(r, f.get(h))
            try:
                out += [float(x) for x in np.asarray(v, float).reshape(-1)[:3]] if v is not None else [0.0, 0.0, 0.0]
            except (TypeError, ValueError):
                out += [0.0, 0.0, 0.0]
        return np.array(out, np.float32)

    def force_names(self):
        return [f"{r}:{a}" for r in self.roles for a in ("fx", "fy", "fz")]
