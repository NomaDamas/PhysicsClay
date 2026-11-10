"""SimDriver: the RobotDriver protocol (pps/bridge/__init__.py) on top of the MuJoCo plant (pps/physics.py Plant).

The plant is the robot: `send_q_des` sets the setpoint its position servos track, `advance(dt)` steps it (a simulated
robot doesn't run on its own clock), `read_state` reads the joints, the gripper opening, the wrist force (contact force
on the hand) and the object poses. Optional sensor noise (`noise={"q": rad, "dq": rad/s, "ee_force": N, "obj_p": m}`)
makes it behave like measured data: the "hidden real" robot of `scripts/calibrate.py --demo` is a SimDriver with
secret parameters and noisy sensors.
"""
import mujoco
import numpy as np

from .. import physics as phys


def joint_labels(model, nq):
    """One label per qpos entry of the robot prefix (hinge / slide: the joint name; free / ball: name[i])."""
    out = [None] * nq
    for j in range(model.njnt):
        a = int(model.jnt_qposadr[j])
        if a >= nq:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint{j}"
        n = {int(mujoco.mjtJoint.mjJNT_FREE): 7, int(mujoco.mjtJoint.mjJNT_BALL): 4}.get(int(model.jnt_type[j]), 1)
        for i in range(n):
            if a + i < nq:
                out[a + i] = name if n == 1 else f"{name}[{i}]"
    return [x or f"q{i}" for i, x in enumerate(out)]


class SimDriver:
    def __init__(self, plant, base_pose=None, noise=None, seed=0, name="sim"):
        self.plant = plant
        self.base_pose = base_pose                       # callable -> (pos, quat) for a welded base (legged), or None
        self.noise = {k: float(v) for k, v in (noise or {}).items() if v}
        self.rng = np.random.default_rng(seed)
        self.q_des = np.array(plant.d.qpos[:plant.nq_r], float)
        self.t_cmd = float(plant.d.time)
        roles = list(plant.grip) or list(plant.site_ids)
        self.capabilities = {"name": name, "joints": joint_labels(plant.m, plant.nq_r), "roles": roles,
                             "grippers": list(plant.grip), "ee_force": True, "objects": True, "realtime": False,
                             "rate_hz": 1.0 / phys.DT, "nq": plant.nq_r, "nv": plant.nv_r}

    # ------------------------------------------------------------------ protocol
    def read_state(self):
        pl, d = self.plant, self.plant.d
        q = np.array(d.qpos[:pl.nq_r], float)
        dq = np.array(d.qvel[:pl.nv_r], float)
        f = {r: np.array(v, float) for r, v in pl.ee_force_raw.items()}
        objs = {n: {"p": np.array(d.qpos[o["qadr"]:o["qadr"] + 3], float), "q": np.array(d.qpos[o["qadr"] + 3:o["qadr"] + 7], float)}
                for n, o in pl.objects.items()}
        nz = self.noise
        if nz:
            r = self.rng
            if nz.get("q"):
                q = q + r.normal(0, nz["q"], q.shape)
            if nz.get("dq"):
                dq = dq + r.normal(0, nz["dq"], dq.shape)
            if nz.get("ee_force"):
                f = {k: v + r.normal(0, nz["ee_force"], 3) for k, v in f.items()}
            if nz.get("obj_p"):
                for o in objs.values():
                    o["p"] = o["p"] + r.normal(0, nz["obj_p"], 3)
        return {"q": q, "dq": dq, "gripper": self.gripper_opening(), "ee_force": f, "t": float(d.time), "objects": objs}

    def send_q_des(self, q, t=None):
        q = np.asarray(q, float)
        if q.shape != self.q_des.shape:
            raise ValueError(f"q_des has {q.shape[0]} entries, the robot has {self.q_des.shape[0]}")
        self.q_des = q.copy()
        self.t_cmd = float(self.plant.d.time if t is None else t)

    def send_gripper(self, role, value):
        if role in self.plant.grip:
            self.plant.grip_cmd(role, "close" if float(value) >= 0.5 else "open")

    def stop(self):
        self.q_des = np.array(self.plant.d.qpos[:self.plant.nq_r], float)

    # ------------------------------------------------------------------ simulation only
    def advance(self, dt):
        """Step the plant by dt seconds toward the last setpoint (the robot's servos, latency and contacts)."""
        return self.plant.step(dt, self.q_des, self.base_pose() if self.base_pose else None)

    def commanded_grip(self):
        """{role: 0 open | 1 close}: the gripper command in force (also when a task commands the plant directly)."""
        return {r: 1.0 if g["cmd"] == "close" else 0.0 for r, g in self.plant.grip.items()}

    def gripper_opening(self):
        """{role: 0 closed .. 1 open} from the gripper actuator's length between its closed and open commands."""
        out, d = {}, self.plant.d
        for r, g in self.plant.grip.items():
            a = g["acts"][0]
            o, c = float(g["open"]), float(g["close"])
            out[r] = float(np.clip((float(d.actuator_length[a]) - c) / (o - c), 0.0, 1.0)) if abs(o - c) > 1e-9 else 0.0
        return out
