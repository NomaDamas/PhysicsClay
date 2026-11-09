"""Feasibility validator: turns the IK result into robot read-back signals.

Four signals, all computed from the physics model (never from an LLM):
  unreachable  -> palm site did not reach its target (IK residual)
  limit        -> a joint sits at its range limit
  collision    -> two collision geoms are closer than the margin
  too_fast     -> required joint velocity exceeds the teleop velocity cap
"""
import collections

import mujoco
import numpy as np

ARM_JOINTS = [
    f"{s}_{j}_joint"
    for s in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
] + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]

COLLISION_CHECKS = [
    ("left_hand_collision", "torso_collision"), ("right_hand_collision", "torso_collision"),
    ("left_hand_collision", "head_collision"), ("right_hand_collision", "head_collision"),
    ("left_hand_collision", "pelvis_collision"), ("right_hand_collision", "pelvis_collision"),
    ("left_hand_collision", "right_hand_collision"),
    ("left_hand_collision", "left_thigh_collision"), ("right_hand_collision", "right_thigh_collision"),
    ("left_hand_collision", "table"), ("right_hand_collision", "table"),
]
PENETRATION_ONLY = {"left_elbow_yaw_collision", "right_elbow_yaw_collision"}


def pretty(name):
    return name.replace("_joint", "").replace("_collision", "").replace("_", " ").replace("mug", "the mug").replace("table", "the table")


class Validator:
    def __init__(self, model, data, reach_tol=0.06, limit_margin=0.035, vmax=4.0, coll_dist=0.015, hist=60):
        self.model, self.data = model, data
        self.reach_tol, self.limit_margin, self.vmax, self.coll_dist = reach_tol, limit_margin, vmax, coll_dist
        self.hist = collections.deque(maxlen=hist)
        self.joint_ids = [model.joint(n).id for n in ARM_JOINTS]
        self.joint_qpos = [model.jnt_qposadr[j] for j in self.joint_ids]
        self.joint_dof = [model.jnt_dofadr[j] for j in self.joint_ids]
        self.pairs = []
        for a, b in COLLISION_CHECKS:
            try:
                self.pairs.append((model.geom(a).id, model.geom(b).id, a, b))
            except KeyError:
                pass
        self.palm_sites = {"left_palm": model.site("left_palm").id, "right_palm": model.site("right_palm").id}
        self._vel_ema = 0.0
        self._fromto = np.zeros(6)

    def evaluate(self, targets, vel):
        m, d = self.model, self.data
        flags, msgs, residuals = {}, [], {}
        for palm, tgt in targets.items():
            dist = float(np.linalg.norm(d.site_xpos[self.palm_sites[palm]] - tgt))
            residuals[palm] = dist
            if dist > self.reach_tol:
                flags[palm] = "unreachable"
                side = "Left" if palm.startswith("left") else "Right"
                msgs.append(f"{side} hand unreachable by {dist * 100:.0f} cm")
        for jid, qa, name in zip(self.joint_ids, self.joint_qpos, ARM_JOINTS):
            lo, hi = m.jnt_range[jid]
            q = d.qpos[qa]
            if min(q - lo, hi - q) < self.limit_margin:
                flags[name] = "limit"
                msgs.append(f"{pretty(name).capitalize()} at joint limit")
        for g1, g2, n1, n2 in self.pairs:
            dist = mujoco.mj_geomDistance(m, d, g1, g2, 0.1, self._fromto)
            margin = 0.0 if (n1 in PENETRATION_ONLY or n2 in PENETRATION_ONLY) else self.coll_dist
            if dist < margin:
                flags[n1] = "collision"
                flags[n2] = "collision"
                msgs.append(f"{pretty(n1).capitalize()} would hit {pretty(n2)}")
        vmax_obs = float(np.max(np.abs(vel[self.joint_dof]))) if vel is not None else 0.0
        self._vel_ema = 0.8 * self._vel_ema + 0.2 * vmax_obs
        speed_scale = min(1.0, self.vmax / self._vel_ema) if self._vel_ema > 1e-6 else 1.0
        if speed_scale < 0.9:
            flags["speed"] = "too_fast"
            msgs.append(f"Too fast for joints: replay at {speed_scale:.1f}x")
        self.hist.append(not flags)
        feasibility = sum(self.hist) / len(self.hist)
        return {"flags": flags, "messages": msgs[:4], "feasibility": feasibility,
                "speed_scale": speed_scale, "residuals": residuals}
