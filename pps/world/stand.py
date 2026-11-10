"""A standing keyframe for a legged model that ships without one (the Unitree H2), computed from the model itself.

From the model's zero pose (legs straight, feet flat on the floor, arms in the model's own ready pose) the floating base
is lowered by `drop` metres, level, and each leg's joints (the chain from the foot up to the base) are solved so that
the foot is exactly where it was, flat (damped least squares on the foot's 6-D pose error, joint ranges respected).
drop 0.05 gives the H2 about the knee bend of the G1's 'stand' keyframe (~0.65 rad). Profiles ask for it with
`home_key="stand"` + `stand={"drop": ...}`; a model that has a keyframe of that name keeps its own. Cached per model.
"""
from functools import lru_cache

import numpy as np


def _rot_err(R, Rt):
    """Rotation error (world frame, rad) that turns R into Rt (small-angle)."""
    return 0.5 * sum(np.cross(R[:, i], Rt[:, i]) for i in range(3))


@lru_cache(maxsize=8)
def _solve(xml, feet, root, drop, arm_q):
    import mujoco
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    q = m.qpos0.copy()
    for name, v in arm_q:                         # the profile's arm pose (joint name -> rad), if any
        q[m.jnt_qposadr[m.joint(name).id]] = v
    d.qpos[:] = q
    mujoco.mj_kinematics(m, d)
    rb = m.body(root).id
    fj = m.body_jntadr[rb]
    if m.jnt_type[fj] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError(f"{root} has no free joint")
    legs = []
    for f, typ in feet:
        b = m.body(f).id if typ == "body" else (m.site(f).bodyid if typ == "site" else m.geom(f).bodyid)
        bid = int(np.asarray(b).ravel()[0])
        kids = [c for c in range(m.nbody) if m.body_parentid[c] == bid]
        while len(kids) == 1:                     # the sole: down to the last link (the H2's 'foot' is above its ankle pitch)
            bid = kids[0]
            kids = [c for c in range(m.nbody) if m.body_parentid[c] == bid]
        jnts, c = [], bid
        while c != rb and c != 0:                 # the leg: every hinge between the foot and the base
            jnts += [j for j in range(m.body_jntadr[c], m.body_jntadr[c] + m.body_jntnum[c]) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
            c = m.body_parentid[c]
        legs.append((bid, d.xpos[bid].copy(), d.xmat[bid].reshape(3, 3).copy(), jnts))
        for j in jnts:                            # a straight leg is singular (it can't get shorter): start the knee bent
            if "knee" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or ""):
                q[m.jnt_qposadr[j]] = float(np.clip(0.3, *m.jnt_range[j])) if m.jnt_limited[j] else 0.3
    q[m.jnt_qposadr[fj] + 2] -= drop              # the base, lowered and level
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    err = 1.0
    for _ in range(200):
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        err = 0.0
        for bid, p_t, R_t, jnts in legs:
            e = np.concatenate([p_t - d.xpos[bid], _rot_err(d.xmat[bid].reshape(3, 3), R_t)])
            err = max(err, float(np.linalg.norm(e[:3])), 0.1 * float(np.linalg.norm(e[3:])))
            mujoco.mj_jacBody(m, d, jp, jr, bid)
            cols = [m.jnt_dofadr[j] for j in jnts]
            J = np.vstack([jp[:, cols], jr[:, cols]])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            for j, v in zip(jnts, dq):
                a = m.jnt_qposadr[j]
                q[a] = float(np.clip(q[a] + v, *m.jnt_range[j])) if m.jnt_limited[j] else q[a] + v
        if err < 1e-5:
            break
    if err > 0.002:
        raise ValueError(f"no standing pose for {xml}: a foot is off by {err * 100:.1f} cm")
    return tuple(q.tolist())


def stand_qpos(xml, feet, root, drop=0.05, arms=None):
    """-> qpos (the model's own layout, free base first) of a standing pose: the base `drop` m lower than in the zero pose,
    feet flat where they were; `arms`: {joint: rad} set first (any other joint keeps its zero pose)."""
    return np.array(_solve(str(xml), tuple(tuple(f) for f in feet), root, float(drop), tuple(sorted((arms or {}).items()))))
