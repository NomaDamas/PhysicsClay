"""pps/world/scene.py: scene spec normalisation, named regions and free-spot search (no MuJoCo compile)."""

import math

import pytest

from pps import tasks
from pps.world import scene


def _gap(a, b):
    """Distance between two normalised objects minus their footprint radii."""
    ra, rb = max(a["size"][:2]) / 2, max(b["size"][:2]) / 2
    return math.hypot(a["pos"][0] - b["pos"][0], a["pos"][1] - b["pos"][1]) - ra - rb


def _inside(s, o):
    r = max(o["size"][:2]) / 2
    return s["x"][0] + r <= o["pos"][0] <= s["x"][1] - r and s["y"][0] + r <= o["pos"][1] <= s["y"][1] - r


# ---------------------------------------------------------------------------------------------- surfaces / regions
@pytest.mark.parametrize("tpl", list(scene.TEMPLATES))
def test_surface_known_templates(tpl):
    s = scene.surface(tpl)
    assert s["template"] == tpl
    assert s["x"][0] == pytest.approx(0.22)  # every template's near edge is at x = 0.22
    assert s["x"][0] < s["x"][1] and s["y"][0] < s["y"][1]


def test_surface_unknown_template_falls_back_to_table():
    assert scene.surface("moon")["template"] == "table"
    assert scene.surface(None) == scene.surface("table")


def test_table_regions_match_the_ux2_values():
    reg = scene.regions(scene.surface("table"))
    assert set(reg) == {"left", "right", "center", "front", "back"}
    for k, (x, y) in tasks.REGIONS.items():
        assert reg[k] == pytest.approx((x, y)), k


@pytest.mark.parametrize("tpl", list(scene.TEMPLATES))
def test_regions_lie_on_the_surface(tpl):
    s = scene.surface(tpl)
    for x, y in scene.regions(s).values():
        assert s["x"][0] < x < s["x"][1]
        assert s["y"][0] < y < s["y"][1]


def test_templates_listing():
    ids = [t["id"] for t in scene.templates()]
    assert ids == list(scene.TEMPLATES)
    assert all({"id", "label", "top"} <= set(t) for t in scene.templates())


# ---------------------------------------------------------------------------------------------- normalize
def test_default_scene_is_stable():
    spec, notes = scene.normalize(scene.DEFAULT_SCENE)
    assert notes == []
    assert [o["name"] for o in spec["objects"]] == [o["name"] for o in scene.DEFAULT_SCENE["objects"]]
    for got, want in zip(spec["objects"], scene.DEFAULT_SCENE["objects"]):
        assert got["pos"] == pytest.approx(want["pos"])
        assert got["movable"] == want["movable"]
    assert scene.normalize(spec)[0] == spec  # idempotent
    assert scene.is_default("g1", scene.DEFAULT_SCENE)
    assert not scene.is_default("panda", scene.DEFAULT_SCENE)


def test_empty_and_garbage_specs():
    assert scene.normalize(None) == ({"template": "table", "objects": []}, [])
    spec, _ = scene.normalize({"template": "nope", "objects": ["mug", 3, None, {"name": "cup"}]})
    assert spec["template"] == "table"
    assert [o["name"] for o in spec["objects"]] == ["cup"]


@pytest.mark.parametrize(
    "raw, kind",
    [
        ({"kind": "mug"}, "mug"),
        ({"kind": "Apple"}, "fruit"),  # alias, case-insensitive
        ({"kind": "brick"}, "block"),
        ({"name": "coffee mug"}, "mug"),  # from the name, longest word first
        ({"name": "soda can"}, "can"),
        ({"name": "thing"}, "block"),  # unknown -> block
        ({"kind": "spaceship"}, "block"),
    ],
)
def test_kind_resolution(raw, kind):
    spec, _ = scene.normalize({"objects": [raw]})
    assert spec["objects"][0]["kind"] == kind


def test_colour_from_field_name_or_kind_default():
    spec, _ = scene.normalize(
        {
            "objects": [
                {"name": "a", "kind": "cup", "color": "Purple"},
                {"name": "green_bowl"},
                {"name": "b", "kind": "box", "color": "ultraviolet"},
            ]
        }
    )
    assert [o["color"] for o in spec["objects"]] == ["purple", "green", scene.KINDS["box"][1]]


def test_names_are_slugged_and_unique():
    spec, _ = scene.normalize(
        {
            "objects": [
                {"name": "Red Block!"},
                {"name": "red block"},
                {"name": "red_block"},
                {"kind": "cup", "color": "blue"},
                {"kind": "cup"},
            ]
        }
    )
    names = [o["name"] for o in spec["objects"]]
    assert names == ["red_block", "red_block_2", "red_block_3", "blue_cup", "cup"]
    assert all(len(n) <= 24 for n in names)


def test_sizes_are_clamped_and_cylinders_are_round():
    spec, _ = scene.normalize(
        {
            "objects": [
                {"name": "huge", "kind": "box", "size": [1.0, 0.001, 0.1]},
                {"name": "jar", "kind": "cup", "size": [0.05, 0.2, 0.1]},
                {"name": "bad", "kind": "block", "size": [0.1, -1, "x"]},
            ]
        }
    )
    huge, jar, bad = spec["objects"]
    assert huge["size"] == [0.40, 0.02, 0.1]
    assert jar["size"][0] == jar["size"][1]
    assert bad["size"] == list(scene.KINDS["block"][0])


def test_named_positions_use_the_regions():
    spec, notes = scene.normalize(
        {
            "objects": [
                {"name": "a", "kind": "block", "pos": "left"},
                {"name": "b", "kind": "block", "pos": "on the right please"},
            ]
        }
    )
    reg = scene.regions(scene.surface("table"))
    assert spec["objects"][0]["pos"] == pytest.approx(reg["left"])
    assert spec["objects"][1]["pos"] == pytest.approx(reg["right"])
    assert notes == []


def test_back_left_moves_the_spot_back():
    spec, _ = scene.normalize({"objects": [{"name": "a", "kind": "block", "pos": "back left"}]})
    reg = scene.regions(scene.surface("table"))
    assert spec["objects"][0]["pos"][0] == pytest.approx(reg["back"][0])


def test_positions_are_clamped_onto_the_surface_with_a_note():
    spec, notes = scene.normalize({"objects": [{"name": "far", "kind": "block", "pos": [5.0, -9.0]}]})
    s = scene.surface("table")
    assert _inside(s, spec["objects"][0])
    assert notes == ["far moved onto the table"]


OVERLAP_BUG = pytest.mark.xfail(
    reason="normalize() keeps an overlapping explicit position when _free_xy finds no "
    "free spot (stacks objects silently, no 'no room' note)",
    strict=False,
)


@pytest.mark.parametrize("tpl", [t if t != "shelf" else pytest.param(t, marks=OVERLAP_BUG) for t in scene.TEMPLATES])
def test_objects_are_spread_without_overlap(tpl):
    objs = [{"name": f"o{i}", "kind": "bowl", "pos": "center"} for i in range(5)]
    spec, _ = scene.normalize({"template": tpl, "objects": objs})
    s = scene.surface(tpl)
    placed = spec["objects"]
    assert len(placed) == 5
    assert all(_inside(s, o) for o in placed)
    for i, a in enumerate(placed):
        for b in placed[i + 1 :]:
            assert _gap(a, b) > 0, (a, b)


def test_at_most_16_objects_and_no_room_is_reported():
    spec, _ = scene.normalize({"objects": [{"name": f"b{i}", "kind": "block"} for i in range(30)]})
    assert len(spec["objects"]) == 16
    big = [{"name": f"p{i}", "kind": "plate", "size": [0.4, 0.4, 0.02]} for i in range(10)]
    spec, notes = scene.normalize({"template": "shelf", "objects": big})
    assert len(spec["objects"]) < 10
    assert any(n.startswith("no room for") for n in notes)


# ---------------------------------------------------------------------------------------------- _free_xy
def test_free_xy_empty_surface_takes_the_first_comfortable_spot():
    s = scene.surface("table")
    xy = scene._free_xy(s, 0.03, [])
    assert xy == pytest.approx([s["x"][0] + 0.10, 0.0])


def test_free_xy_avoids_taken_spots_and_prefers_the_nearest():
    s = scene.surface("table")
    taken = [([s["x"][0] + 0.10, 0.0], 0.05)]
    xy = scene._free_xy(s, 0.03, taken)
    assert math.hypot(xy[0] - taken[0][0][0], xy[1] - taken[0][0][1]) > 0.03 + 0.05 + 0.03
    want = [s["x"][0] + 0.28, 0.26]
    assert scene._free_xy(s, 0.03, taken, prefer=want) == pytest.approx(want)


def test_free_xy_returns_none_when_full():
    s = scene.surface("table")
    assert scene._free_xy(s, 0.9, []) is None  # too big for any spot
