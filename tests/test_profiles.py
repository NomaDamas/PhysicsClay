"""pps/world/profiles.py: every robot profile is complete (prompts, known capabilities, catalog in sync)."""

import json

import pytest

from pps.world import profiles

ROOT = profiles.ROOT
PROMPTS = ROOT / "pps" / "world" / "prompts"
KINDS = {"humanoid", "arm", "bimanual", "mobile_manipulator", "quadruped_arm"}
BASES = {"legged", "fixed", "mobile"}
# capabilities the server, the agent (pps/agent.py) and the web UI (web/ui/robots.js) know how to use
KNOWN_CAPS = set(profiles.HUMANOID_CAPS) | set(profiles.ARM_CAPS) | {"bimanual", "drive_base"}
# dock tools web/ui/robots.js can show
KNOWN_TOOLS = {"point", "zone", "goal", "show", "path", "drive", "handover", "arm"}
IDS = list(profiles.PROFILES)


def test_default_robot_exists():
    assert profiles.DEFAULT_ROBOT in profiles.PROFILES


@pytest.mark.parametrize("rid", IDS)
def test_profile_is_complete(rid):
    p = profiles.PROFILES[rid]
    assert p.id == rid
    assert p.kind in KINDS
    assert p.base in BASES
    assert p.mount.get("on") in ("floor", "surface")
    assert p.ee and "right" in p.ee, "single arms use the 'right' role so task code works unchanged"
    assert set(p.capabilities) <= KNOWN_CAPS, set(p.capabilities) - KNOWN_CAPS
    assert len(set(p.capabilities)) == len(p.capabilities)
    if not p.experimental:
        assert "pick_place" in p.capabilities
    if p.kind == "bimanual":
        assert "bimanual" in p.capabilities and "left" in p.ee
    if p.base == "mobile":
        assert "drive_base" in p.capabilities and p.base_joints
    if p.base == "legged":
        assert p.feet and p.root


@pytest.mark.parametrize("rid", IDS)
def test_profile_has_prompts(rid):
    p = profiles.PROFILES[rid]
    md, js = PROMPTS / f"{rid}.md", PROMPTS / f"{rid}.json"
    assert md.is_file() and md.read_text().strip()
    assert p.prompt == f"pps/world/prompts/{rid}.md"
    ui = json.loads(js.read_text())
    assert {"label", "kind", "tagline", "examples", "tools"} <= set(ui)
    assert ui["kind"] == p.kind
    assert ui["examples"] and all(isinstance(e, str) and e for e in ui["examples"])
    assert set(ui["tools"]) <= KNOWN_TOOLS, set(ui["tools"]) - KNOWN_TOOLS


def test_no_orphan_prompts():
    ids = set(IDS)
    for f in PROMPTS.glob("*.json"):
        assert f.stem in ids, f"{f.name} has no profile"
    for f in PROMPTS.glob("*.md"):
        assert f.stem in ids | {"base"}, f"{f.name} has no profile"


def test_web_catalog_is_in_sync():
    import importlib.util

    spec = importlib.util.spec_from_file_location("sync_robot_prompts", ROOT / "scripts" / "sync_robot_prompts.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    want = mod.render(mod.build())
    assert mod.OUT.read_text() == want, "run: python scripts/sync_robot_prompts.py"
    assert set(json.loads(want)) == set(IDS)


@pytest.mark.parametrize(
    "alias, rid",
    list(profiles.ALIASES.items())
    + [
        (None, profiles.DEFAULT_ROBOT),
        ("", profiles.DEFAULT_ROBOT),
        ("UR5E", "ur5e"),
        ("Unitree-G1", "g1"),
        ("stretch 3", "stretch3"),
    ],
)
def test_get_resolves_aliases(alias, rid):
    assert profiles.get(alias).id == rid


def test_get_unknown_is_none():
    assert profiles.get("r2d2") is None


def test_public_views_hide_runtime_details():
    for p in profiles.PROFILES.values():
        pub = p.public()
        assert {"id", "label", "kind", "base", "ee", "capabilities"} <= set(pub)
        assert "model_xml" not in pub and "freeze" not in pub
        assert set(p.summary()) <= set(pub) | {"experimental"}
    assert [s["id"] for s in profiles.listing()] == IDS


@pytest.mark.parametrize("rid", IDS)
def test_model_files_exist(rid):
    p = profiles.PROFILES[rid]
    path = ROOT / p.model_xml
    if "vendor/" in p.model_xml and not (ROOT / "vendor" / "mink").exists():
        pytest.skip("vendor/mink not fetched (run scripts/setup.sh)")
    assert path.is_file(), p.model_xml
    if p.home_xml:
        assert (ROOT / p.home_xml).is_file()
