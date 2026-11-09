"""Feasibility validator for any body (the G1 keeps pps/validate.py). Same four signals, from the physics model:
unreachable (end-effector residual), limit (arm joint at its range), collision (arm vs the work surface),
too_fast (joint velocity); plus balance for legged bodies only (centre of mass over the support feet)."""
import collections

import mujoco
import numpy as np


def _frame_pos(m, d, name, typ):
    if typ == "site":
        return d.site_xpos[m.site(name).id]
    if typ == "geom":
        return d.geom_xpos[m.geom(name).id]
    return d.xpos[m.body(name).id]


def pretty(name):
    return name.replace("_joint", "").replace("_collision", "").replace("/", " ").replace("_", " ")


class BodyValidator:
    def __init__(self, model, data, palm_sites, arm_joints, coll_pairs=(), feet=(), root_body=1,
                 reach_tol=0.06, limit_margin=0.02, vmax=4.0, coll_dist=0.004, hist=60):
        self.model, self.data = model, data
        self.reach_tol, self.limit_margin, self.vmax, self.coll_dist = reach_tol, limit_margin, vmax, coll_dist
        self.hist = collections.deque(maxlen=hist)
        self.palm_sites = dict(palm_sites)
        m = model
        self.joints = []
        for n in arm_joints:
            j = m.joint(n).id
            if m.jnt_limited[j] and m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                self.joints.append((j, m.jnt_qposadr[j], n))
        self.dofs = [m.jnt_dofadr[m.joint(n).id] for n in arm_joints]
        self.pairs = [(g, t) for geoms, tables in coll_pairs for g in geoms for t in tables][:24]
        self.feet = list(feet)
        self.root_body = root_body
        self.ground_z = None
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
                side = "Left" if palm.startswith("left") else ("Right" if len(self.palm_sites) > 1 else "The")
                msgs.append(f"{side} {'hand' if side != 'The' else 'gripper'} unreachable by {dist * 100:.0f} cm")
        for jid, qa, name in self.joints:
            lo, hi = m.jnt_range[jid]
            q = d.qpos[qa]
            if min(q - lo, hi - q) < self.limit_margin * max(1.0, (hi - lo) / 6):
                flags[name] = "limit"
                msgs.append(f"{pretty(name).capitalize()} at joint limit")
        hit = False
        for g, t in self.pairs:
            if hit:
                break
            if mujoco.mj_geomDistance(m, d, g, t, 0.05, self._fromto) < self.coll_dist:
                n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"
                flags[n] = "collision"
                flags["table"] = "collision"
                msgs.append("The arm would hit the table")
                hit = True
        if self.feet:
            self._balance(flags, msgs)
        vmax_obs = float(np.max(np.abs(vel[self.dofs]))) if (vel is not None and self.dofs) else 0.0
        self._vel_ema = 0.8 * self._vel_ema + 0.2 * vmax_obs
        speed_scale = min(1.0, self.vmax / self._vel_ema) if self._vel_ema > 1e-6 else 1.0
        if speed_scale < 0.9:
            flags["speed"] = "too_fast"
            msgs.append(f"Too fast for joints: replay at {speed_scale:.1f}x")
        self.hist.append(not flags)
        feasibility = sum(self.hist) / len(self.hist)
        return {"flags": flags, "messages": msgs[:4], "feasibility": feasibility, "speed_scale": speed_scale,
                "residuals": residuals}

    def _balance(self, flags, msgs):
        """Kinematic check: the centre of mass stays over the support polygon (bounding box of the planted feet)."""
        m, d = self.model, self.data
        pts = np.array([_frame_pos(m, d, f, t) for f, t in self.feet])
        if self.ground_z is None:
            self.ground_z = float(pts[:, 2].min())
        down = pts[pts[:, 2] < self.ground_z + 0.03]
        com = d.subtree_com[self.root_body][:2]
        if len(down) == 0:
            flags["balance"] = "balance"
            msgs.append("Would tip over: no foot is on the ground")
            return
        lo, hi = down[:, :2].min(0) - np.array([0.07, 0.035]), down[:, :2].max(0) + np.array([0.10, 0.035])
        gap = float(np.linalg.norm(np.maximum(np.maximum(lo - com, com - hi), 0.0)))
        if gap > 0.01:
            flags["balance"] = "balance"
            msgs.append(f"Would tip over: centre of mass {gap * 100:.0f} cm outside the feet")
