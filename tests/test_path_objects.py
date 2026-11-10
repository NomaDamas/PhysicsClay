"""pps/tasks.py path_object_hits / lift_path: a drawn path or trajectory checked against the objects on the table, with
the gripper's real extent below its TCP (v1 P0c). Pure geometry, no MuJoCo."""

import numpy as np
import pytest

from pps import tasks

TOP = 0.74
MUG = (np.array([0.33, -0.18, TOP + 0.071]), 0.065, 0.071, "the mug")      # centre, radius, half height, name
VASE = (np.array([0.33, 0.04, TOP + 0.12]), 0.045, 0.12, "the vase")
BLOCK = (np.array([0.29, -0.34, TOP + 0.026]), 0.037, 0.026, "the red block")


def line(a, b, n=15):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return np.array([a + (b - a) * k / (n - 1) for k in range(n)])


def test_a_path_through_the_mug_at_hand_height_hits_it_where_it_passes():
    pts = line([0.20, -0.18, 0.82], [0.46, -0.18, 0.82])
    hits = tasks.path_object_hits(pts, [MUG, VASE], tasks.hand_disc(0.02, 0.03))
    assert hits[0] is None and hits[-1] is None           # far from it: fine
    assert any(hits) and {h for h in hits if h} == {"my hand would hit the mug"}
    inside = [i for i, p in enumerate(pts) if abs(p[0] - 0.33) < 0.065]
    assert all(hits[i] for i in inside)


def test_the_gripper_below_the_tcp_counts():
    """A TCP 2 cm over the mug's top clears it as a point, but not with fingertips reaching 4 cm below the TCP."""
    z = MUG[0][2] + MUG[2] + 0.02
    pts = line([0.25, -0.18, z], [0.41, -0.18, z])
    assert not any(tasks.path_object_hits(pts, [MUG], tasks.hand_disc(0.0, 0.0)))
    assert any(tasks.path_object_hits(pts, [MUG], tasks.hand_disc(0.04, 0.03)))


def test_the_footprint_counts_beside_an_object():
    pts = line([0.20, -0.18 + 0.065 + 0.02, 0.80], [0.46, -0.18 + 0.065 + 0.02, 0.80])   # passes 2 cm beside the mug
    assert not any(tasks.path_object_hits(pts, [MUG], tasks.hand_disc(0.02, 0.0)))
    assert any(tasks.path_object_hits(pts, [MUG], tasks.hand_disc(0.02, 0.03)))


def test_what_the_hand_carries_is_checked_too():
    """The hand passes 6 cm over the block, but the mug it carries hangs 10 cm below the TCP."""
    z = BLOCK[0][2] + BLOCK[2] + 0.06
    pts = line([0.20, -0.34, z], [0.40, -0.34, z])
    assert not any(tasks.path_object_hits(pts, [BLOCK], tasks.hand_disc(0.02, 0.03)))
    hits = tasks.path_object_hits(pts, [BLOCK], tasks.hand_disc(0.02, 0.03), carry=("the mug", 0.065, 0.071, 0.10))
    assert any(hits) and {h for h in hits if h} == {"the mug would hit the red block"}


def test_lift_path_goes_over_and_clears_it():
    pts = line([0.03, -0.18, 0.82], [0.63, -0.18, 0.82], n=61)
    up, names = tasks.lift_path(pts, [MUG], tasks.hand_disc(0.03, 0.03))
    assert names == ["the mug"]
    assert not any(tasks.path_object_hits(up, [MUG], tasks.hand_disc(0.03, 0.03)))
    assert np.allclose(up[:, :2], pts[:, :2])               # only heights change
    assert up[0, 2] == pytest.approx(0.82) and up[-1, 2] == pytest.approx(0.82)   # far ends untouched
    slope = np.abs(np.diff(up[:, 2])) / np.linalg.norm(np.diff(up[:, :2], axis=0), axis=1)
    assert slope.max() <= 1.0 + 1e-6                        # ramps no steeper than 45°


def test_lift_path_leaves_a_clear_path_alone():
    pts = line([0.25, 0.20, 0.95], [0.35, 0.25, 0.95])
    up, names = tasks.lift_path(pts, [MUG, BLOCK], tasks.hand_disc(0.03, 0.03))   # (callers never pass the vase)
    assert names == [] and np.allclose(up, pts)


def test_lift_path_only_over_the_bodies_it_is_given():
    """The caller passes only movable objects (the vase is never flown over): a path into it is still a hit after."""
    pts = line([0.28, -0.12, 0.84], [0.30, 0.10, 0.84])
    up, names = tasks.lift_path(pts, [MUG], tasks.hand_disc(0.03, 0.03))
    assert "the vase" not in names
    assert any(h == "my hand would hit the vase" for h in tasks.path_object_hits(up, [MUG, VASE], tasks.hand_disc(0.03, 0.03)))
