"""pps/features/copresence.py, pure parts: utterance classification (deixis) and the person-in-the-robot-world
geometry (landmarks <-> world, pointing rays, open hands, offers)."""

import numpy as np
import pytest

from pps.features import copresence as c


# ---------------------------------------------------------------------------------------------- classify (deixis)
@pytest.mark.parametrize(
    "text, kind",
    [
        ("put this there", "pick_place"),
        ("Put THAT over there.", "pick_place"),
        ("이거 저기 놔", "pick_place"),
        ("pick this up", "pick"),
        ("grab that", "pick"),
        ("put it there", "place"),
        ("there", "place"),
        ("여기", "place"),
        ("take this to the left", "pick_region"),
        ("give me that", "handover"),
        ("hand it over", "handover"),
        ("이거 줘", "handover"),
        ("what time is it", None),
        ("there is a cat on the left of the table and a dog", None),
        ("", None),
        (None, None),
    ],
)
def test_classify(text, kind):
    assert c.classify(text) == kind


def test_korean_place_verbs_with_jwo_are_not_handovers():
    assert c.classify("이거 저기 놔줘") == "pick_place"
    assert c.classify("놔줘") != "handover"


@pytest.mark.parametrize(
    "text, region",
    [
        ("put it on the left", "left"),
        ("to the centre", "center"),
        ("the middle one", "center"),
        ("오른쪽에", "right"),
        ("nowhere", None),
        ("leftover", None),
    ],
)
def test_region_words(text, region):
    assert c._region(text) == region


def test_word_match_is_whole_word():
    assert c._has("put this here", ("this",))
    assert not c._has("thistle", ("this",))
    assert c._has("이거 줘", ("이거",))


# ---------------------------------------------------------------------------------------------- geometry
def test_unit_handles_zero_vectors():
    assert np.allclose(c._unit(np.array([3.0, 0, 4.0])), [0.6, 0, 0.8])
    assert np.allclose(c._unit(np.zeros(3)), 0)


def test_to_world_needs_both_shoulders():
    assert c.to_world(None) is None
    assert c.to_world({"l_shoulder": [0, 0, 0]}) is None


def test_to_world_from_world_round_trip():
    lm = c.synth_pose(point_at=[0.45, -0.2, c.TABLE_TOP])
    J = c.to_world(lm)
    back = c.from_world(J)
    for k, v in lm.items():
        assert back[k] == pytest.approx(v, abs=1e-3), k


def test_resting_person_stands_across_the_table():
    J = c.to_world(c.synth_pose())
    assert J["pelvis"] == pytest.approx([c.PERSON[0], c.PERSON[1], c.HIP_Z])
    for k in ("neck", "head", "l_knee", "r_ankle"):
        assert k in J
    assert J["head"][2] > J["neck"][2] > J["pelvis"][2] > J["l_ankle"][2]
    # not mirrored: the person's left shoulder is on the robot's right (-y)
    assert J["l_shoulder"][1] < J["r_shoulder"][1]
    assert c.arm_ray(J, "l") is None and c.arm_ray(J, "r") is None
    assert c.offered_hand(J) is None


def test_hip_height_defaults_and_uses_the_ankles():
    assert c.hip_height({}) == c.HIP_Z
    assert c.hip_height(c.synth_pose()) == pytest.approx(c.HIP_Z, abs=0.01)
    tall = {"l_hip": [0.1, 0, 0], "r_hip": [-0.1, 0, 0], "l_ankle": [0.1, 0.95, 0], "r_ankle": [-0.1, 0.95, 0]}
    assert c.hip_height(tall) == pytest.approx(0.95 + c.ANKLE_Z)


@pytest.mark.parametrize("target", [[0.45, -0.2], [0.35, 0.25], [0.60, 0.0]])
def test_pointing_ray_hits_the_table_where_the_person_points(target):
    tgt = [target[0], target[1], c.TABLE_TOP]
    J = c.to_world(c.synth_pose(point_at=tgt))
    side = "l" if target[1] < 0 else "r"
    ray = c.arm_ray(J, side)
    assert ray is not None and ray["side"] == side
    assert ray["hit"] == pytest.approx(tgt, abs=0.01)
    assert ray["extension"] >= c.EXTEND and ray["forward"] >= c.FORWARD
    assert not c.grip_open(J, side)  # a pointing hand is not an open hand
    assert c.arm_ray(J, "r" if side == "l" else "l") is None


def test_offered_open_hand_is_an_offer_not_a_point():
    J = c.to_world(c.synth_pose(offer=[0.7, 0.1, 1.0]))
    assert c.offered_hand(J) == "r"
    assert c.grip_open(J, "r") is True
    assert c.arm_ray(J, "r") is None
    assert c.hand_pos(J, "r") == pytest.approx((J["r_wrist"] + J["r_index"]) / 2)


def test_grip_open_unknown_without_landmarks():
    assert c.grip_open({}, "l") is None


def test_on_table():
    assert c.on_table([0.4, 0.0])
    assert not c.on_table([1.0, 0.0])
    assert c.on_table([c.TABLE_X[1] + 0.01, 0.0], margin=0.02)
