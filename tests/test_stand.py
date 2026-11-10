"""The Unitree H2 (v1 final): a standing keyframe computed from its model (pps/world/stand.py), palm grippers, and the
collision boxes physics adds around its links (pps.physics.add_hulls). Headless, no server."""

import mujoco
import numpy as np
import pytest

from pps.world import profiles, scene
from pps.world.stand import stand_qpos

H2 = profiles.PROFILES["h2"]
XML = profiles.ROOT / H2.model_xml
pytestmark = pytest.mark.skipif(not XML.is_file(), reason="vendor/mink not fetched (run scripts/setup.sh)")


def _soles(m, d):
    return {s: (d.site_xpos[m.site(s).id].copy(), d.site_xmat[m.site(s).id].reshape(3, 3)[:, 2].copy())
            for s in ("left_forefoot", "left_heel", "right_forefoot", "right_heel")}


def test_stand_keeps_the_feet_flat_where_they_were_and_bends_the_knees():
    m = mujoco.MjModel.from_xml_path(XML.as_posix())
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    zero = _soles(m, d)
    q = stand_qpos(XML, H2.feet, H2.root, **H2.stand)
    assert q.shape == (m.nq,)
    d.qpos[:] = q
    mujoco.mj_kinematics(m, d)
    for s, (p, z) in _soles(m, d).items():
        assert np.linalg.norm(p - zero[s][0]) < 0.002, s          # the foot didn't move ...
        assert z[2] > 0.999, s                                    # ... and stays flat
    assert abs((q[2] - m.qpos0[2]) + H2.stand["drop"]) < 1e-9     # the base came down, level
    np.testing.assert_allclose(q[3:7], m.qpos0[3:7])
    for side in ("left", "right"):
        knee = q[m.jnt_qposadr[m.joint(f"{side}_knee_joint").id]]
        assert 0.4 < knee < 1.0                                   # slightly bent, like the G1's stand
    for j in range(m.njnt):                                       # within the joint ranges
        if m.jnt_limited[j]:
            v = q[m.jnt_qposadr[j]]
            assert m.jnt_range[j][0] - 1e-9 <= v <= m.jnt_range[j][1] + 1e-9


@pytest.fixture(scope="module")
def h2_world():
    return scene.build("h2", None, physics=True)


def test_h2_world_starts_standing_with_palm_grippers_and_link_hulls(h2_world):
    w = h2_world
    m, mi = w.model, w.ik_model
    q = w.home_q
    d = mujoco.MjData(mi)
    d.qpos[:] = q
    mujoco.mj_kinematics(mi, d)
    for side in ("left", "right"):
        assert q[mi.jnt_qposadr[mi.joint(f"{side}_knee_joint").id]] > 0.4
        assert abs(d.site_xpos[mi.site(f"{side}_forefoot").id][2]) < 0.003     # feet on the floor
        for k in ("pad_a", "pad_b", "palm_col"):
            assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_{k}") >= 0
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{side}_tcp") >= 0
    hulls = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("pc_hull_")]
    assert len(hulls) >= 10
    assert all(m.geom_contype[g] == 0 and m.geom_conaffinity[g] & 1 for g in hulls)     # touch the world, not the robot
    names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) for g in hulls}
    assert {"torso_link", "pelvis", "right_elbow_link", "left_wrist_pitch_link"} <= names
    hidden = [g for g in range(m.ngeom) if "index" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "")]
    assert hidden and all(m.geom_group[g] > 2 for g in hidden)    # the undriven fingers aren't drawn
    assert all(not (m.geom_contype[g] or m.geom_conaffinity[g]) for g in hidden)


def test_h2_hulls_dont_touch_the_robot_or_table_at_home(h2_world):
    w = h2_world
    m = w.model
    d = mujoco.MjData(m)
    d.qpos[:len(w.home_q)] = w.home_q
    mujoco.mj_forward(m, d)
    robot = w.robot_bodies
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        assert not (b1 in robot and b2 in robot), "self contact at home"
        if b1 in robot or b2 in robot:      # only the feet stand on something
            g = c.geom1 if b1 in robot else c.geom2
            assert m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and "foot" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") \
                or "heel" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)


def test_other_bodies_get_no_hulls():
    w = scene.build("g1", None, physics=True)
    assert not any((mujoco.mj_id2name(w.model, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("pc_hull_") for g in range(w.model.ngeom))
