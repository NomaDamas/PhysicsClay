"""pps/recon: photos -> simulated objects and spaces. Pure and fast: synthetic depth from a tiny ray caster, a synthetic
ArUco photo, stubbed HTTP for the cloud providers, a stub TripoSR model, a temp object library. No downloads."""

import json
import math

import cv2
import numpy as np
import pytest
from conftest import models_missing

trimesh = pytest.importorskip("trimesh")

from pps.recon import fit, library, physical, scale, sim  # noqa: E402
from pps.recon.base import ReconError  # noqa: E402
from pps.recon.providers import cloud, primitive, triposr  # noqa: E402


# ---------------------------------------------------------------------------------------------- a tiny ray caster
def camera(w=320, h=240, hfov=60.0, pos=(0.0, -0.42, 0.36), look=(0.0, 0.0, 0.04)):
    """Pinhole camera above a table (world z up, table top z = 0) -> (K, R world->camera rows, centre)."""
    K = fit.intrinsics(w, h, hfov=hfov)
    C = np.asarray(pos, float)
    f = np.asarray(look, float) - C
    f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0])
    r /= np.linalg.norm(r)
    d = np.cross(f, r)
    return K, np.stack([r, d, f]), C


def render(shape, dims, w=320, h=240, yaw=0.0, color=(40, 50, 220), **cam):
    """-> (BGR image, metric depth (m), object mask, K). shape: 'box' dims (sx, sy, sz) | 'cyl' dims (r, height)."""
    K, R, C = camera(w, h, **cam)
    vs, us = np.mgrid[0:h, 0:w].astype(float)
    dc = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones_like(us)], -1)
    dw = dc @ R  # world directions (t = camera depth)
    t_tab = np.where(dw[..., 2] < -1e-9, -C[2] / np.where(dw[..., 2] < -1e-9, dw[..., 2], -1), np.inf)
    t_obj = np.full((h, w), np.inf)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.0]])  # world -> object
    o, dl = Rz @ C, dw @ Rz.T
    if shape == "box":
        lo, hi = np.array([-dims[0] / 2, -dims[1] / 2, 0.0]), np.array([dims[0] / 2, dims[1] / 2, dims[2]])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (lo - o) / dl, (hi - o) / dl
        tn, tf = np.nanmax(np.minimum(t1, t2), -1), np.nanmin(np.maximum(t1, t2), -1)
        t_obj = np.where((tn <= tf) & (tn > 0), tn, np.inf)
    else:
        r, H = dims
        a = dl[..., 0] ** 2 + dl[..., 1] ** 2
        b = 2 * (o[0] * dl[..., 0] + o[1] * dl[..., 1])
        cc = o[0] ** 2 + o[1] ** 2 - r * r
        disc = b * b - 4 * a * cc
        ts = np.where(disc >= 0, (-b - np.sqrt(np.clip(disc, 0, None))) / (2 * a), np.inf)
        z = o[2] + ts * dl[..., 2]
        t_side = np.where((ts > 0) & (z >= 0) & (z <= H), ts, np.inf)
        tc = (H - o[2]) / np.where(np.abs(dl[..., 2]) > 1e-9, dl[..., 2], 1e-9)
        px, py = o[0] + tc * dl[..., 0], o[1] + tc * dl[..., 1]
        t_cap = np.where((tc > 0) & (px**2 + py**2 <= r * r), tc, np.inf)
        t_obj = np.minimum(t_side, t_cap)
    hit = t_obj < t_tab
    depth = np.where(hit, t_obj, t_tab)
    img = np.full((h, w, 3), (120, 150, 170), np.uint8)
    img[hit] = color
    img[~np.isfinite(depth)] = (235, 235, 235)
    return img, np.where(np.isfinite(depth), depth, np.nan), hit, K


# ---------------------------------------------------------------------------------------------- primitive fits
def test_box_recovered_from_synthetic_depth():
    dims = (0.10, 0.06, 0.08)
    img, depth, mask, K = render("box", dims, yaw=0.35)
    r = primitive.Primitive().reconstruct(img, mask, {"kind": "box"}, depth=depth, K=K)
    assert r.shape == "box" and r.metric
    got = np.asarray(r.mesh.extents)
    assert sorted(got[:2]) == pytest.approx(sorted(dims[:2]), rel=0.10)
    assert got[2] == pytest.approx(dims[2], rel=0.10)
    assert r.mesh.is_watertight


def test_cylinder_recovered_from_synthetic_depth():
    rad, H = 0.035, 0.12
    img, depth, mask, K = render("cyl", (rad, H))
    r = primitive.Primitive().reconstruct(img, mask, {}, depth=depth, K=K)
    assert r.shape in ("cylinder", "lathe")
    ex = np.asarray(r.mesh.extents)
    assert ex[0] == pytest.approx(2 * rad, rel=0.10) and ex[1] == pytest.approx(2 * rad, rel=0.10)
    assert ex[2] == pytest.approx(H, rel=0.10)
    assert r.colors is not None and len(r.colors) == len(r.mesh.vertices)
    red = r.colors[:, 0].astype(int) - r.colors[:, 2].astype(int)  # RGBA, painted from the (red) photo
    assert np.median(red) > 80


def test_shape_choice_without_hints():
    img, depth, mask, K = render("box", (0.12, 0.12, 0.05), yaw=0.2)
    assert primitive.Primitive().reconstruct(img, mask, {}, depth=depth, K=K).shape == "box"


def test_outline_only_gives_a_closed_mesh_in_pixels():
    img, _, mask, _ = render("cyl", (0.04, 0.10))
    r = primitive.Primitive().reconstruct(img, mask, {"kind": "cup"})
    assert r.shape == "lathe" and not r.metric and r.px_per_unit == 1.0
    assert r.mesh.is_watertight and r.mesh.extents[2] > 20  # pixel units


def test_no_outline_is_a_clear_error():
    with pytest.raises(ReconError):
        primitive.Primitive().reconstruct(np.zeros((50, 50, 3), np.uint8), np.zeros((50, 50), bool), {})


# ---------------------------------------------------------------------------------------------- metric scale
@pytest.mark.parametrize(
    "text, dim, m",
    [
        ("it's 9 cm tall", "height", 0.09),
        ("about 20cm long", "longest", 0.20),
        ("높이 9센티", "height", 0.09),
        ("3 inches wide", "width", 0.0762),
        ("120 mm", "longest", 0.12),
        ("diameter 7.5 cm", "width", 0.075),
    ],
)
def test_parse_size(text, dim, m):
    p = scale.parse_size(text)
    assert p["dim"] == dim and p["m"] == pytest.approx(m, rel=1e-3)


def test_parse_size_ignores_non_sizes():
    assert scale.parse_size("a red mug") is None and scale.parse_size("") is None and scale.parse_size("900 m") is None


def _marker_photo(side_px=100, warp=False):
    img = np.full((480, 640, 3), 200, np.uint8)
    mk = cv2.cvtColor(scale.marker_image(0, side_px, border=0), cv2.COLOR_GRAY2BGR)
    img[300 : 300 + side_px, 80 : 80 + side_px] = mk
    cv2.rectangle(img, (360, 160), (440, 400), (40, 60, 200), -1)  # the object, 80 px wide
    if warp:  # a slanted view of the table
        src = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])
        dst = np.float32([[40, 30], [600, 10], [630, 470], [10, 450]])
        img = cv2.warpPerspective(img, cv2.getPerspectiveTransform(src, dst), (640, 480), borderValue=(200, 200, 200))
    return img


def test_aruco_scale_from_a_synthetic_photo():
    c = scale.aruco(_marker_photo(100))
    assert c is not None and c["source"] == "aruco"
    assert c["m_per_px"] == pytest.approx(0.05 / 100, rel=0.03)
    f = scale.factor(c, extents=[80, 50, 120], px_per_unit=1.0)  # 80 px across, in pixel units -> 4 cm
    assert 80 * f == pytest.approx(0.04, rel=0.04)
    assert scale.aruco(_marker_photo(100, warp=True)) is not None  # still found at an angle
    assert scale.aruco(np.full((240, 320, 3), 200, np.uint8)) is None


def test_scale_priority_and_choice():
    img = _marker_photo(100)
    cands = scale.candidates(img, said="10 cm tall", vlm={"height_cm": 12}, label="mug")
    assert [c["source"] for c in cands][:3] == ["you", "aruco", "vlm"]
    f, used = scale.choose(cands, [50, 50, 200], px_per_unit=1.0)
    assert used["source"] == "you" and 200 * f == pytest.approx(0.10)
    f, used = scale.choose(scale.candidates(img, label="mug"), [80, 80, 200], px_per_unit=1.0)
    assert used["source"] == "aruco"
    f, used = scale.choose(scale.candidates(None, label="mug"), [1, 1, 2])
    assert used["source"] == "prior" and 2 * f == pytest.approx(0.10)
    f, used = scale.choose(scale.candidates(None, label="mug", metric=True), [0.06, 0.06, 0.1], metric=True)
    assert f == 1.0 and used["source"] == "depth"  # metric depth keeps its own scale


# ---------------------------------------------------------------------------------------------- physics-ready
def test_watertight_repair_fills_a_hole():
    box = trimesh.creation.box(extents=[0.1, 0.06, 0.04])
    holed = trimesh.Trimesh(box.vertices, box.faces[:-1], process=False)
    assert not holed.is_watertight
    fixed, tight, notes = physical.repair(holed)
    assert tight and fixed.is_watertight and fixed.volume == pytest.approx(0.1 * 0.06 * 0.04, rel=1e-6)
    assert notes


def test_inertia_from_a_known_mesh():
    a, b, c, m = 0.10, 0.06, 0.04, 0.5
    p = physical.mass_props(trimesh.creation.box(extents=[a, b, c]), "wood", mass=m)
    diag = np.diag(p["inertia"])
    assert diag == pytest.approx(
        [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12], rel=1e-6
    )
    solid = physical.mass_props(trimesh.creation.box(extents=[a, b, c]), "wood")
    assert solid["mass"] == pytest.approx(a * b * c * 600, rel=1e-6) and solid["friction"] == 0.55
    cup = physical.mass_props(fit.cylinder_mesh(0.04, 0.10), "ceramic", hollow=True)
    assert 0.05 < cup["mass"] < 0.6  # a ceramic cup's walls, not a solid cylinder (1.2 kg)
    w, q = physical.principal(p["inertia"])
    assert sorted(w) == pytest.approx(sorted(diag)) and np.linalg.norm(q) == pytest.approx(1.0)


def _cup_mesh():
    """A cup with a cavity (concave)."""
    prof = np.array([[0.0, 0.0], [0.04, 0.0], [0.04, 0.10], [0.034, 0.10], [0.034, 0.008], [0.0, 0.008]])
    m = trimesh.creation.revolve(prof, sections=24)
    m.merge_vertices()
    if m.volume < 0:
        m.invert()
    return m


def test_convex_decomposition_and_its_fallback(monkeypatch):
    cup = _cup_mesh()
    assert cup.is_watertight and physical.convexity(cup) < 0.5
    hulls, method = physical.decompose(trimesh.creation.box(extents=[0.1, 0.1, 0.1]))
    assert method == "hull" and len(hulls) == 1
    monkeypatch.setattr(physical, "_has", lambda mod: False)  # no CoACD installed: k-means hull clusters
    hulls, method = physical.decompose(cup)
    assert method == "clusters" and 2 <= len(hulls) <= physical.MAX_HULLS
    assert all(h.is_convex for h in hulls)
    assert sum(h.volume for h in hulls) < cup.convex_hull.volume  # tighter than one hull around the cavity


def test_decimate_keeps_colours():
    m = fit.densify(fit.cylinder_mesh(0.03, 0.1), target_faces=6000)
    cols = np.tile(np.array([10, 200, 30, 255], np.uint8), (len(m.vertices), 1))
    out, c = physical.decimate(m, max_faces=400, colors=cols)
    assert len(out.faces) <= 400 and c is not None and len(c) == len(out.vertices)
    assert np.all(c[:, 1] > 150)


# ---------------------------------------------------------------------------------------------- library + scenes
@pytest.fixture
def lib(tmp_path, monkeypatch):
    monkeypatch.setenv("PC_LIBRARY", str(tmp_path))
    return tmp_path


def _saved(kind="bottle", label="bottle"):
    m = fit.densify(fit.lathe_mesh([0.01, 0.05, 0.09, 0.11], [0.03, 0.032, 0.022, 0.012], 0.12))
    cols = np.tile(np.array([200, 40, 40, 255], np.uint8), (len(m.vertices), 1))
    prep = physical.prepare(m, cols, "plastic", False)
    return library.save(
        prep,
        {
            "label": label,
            "name": label,
            "kind": kind,
            "provider": "primitive",
            "license": "Apache-2.0",
            "scale_source": "you",
        },
        b"\xff\xd8jpeg",
    )


def test_library_save_load_list_delete(lib):
    meta = _saved()
    got = library.load(meta["id"])
    assert got["kind"] == "bottle" and got["provider"] == "primitive" and got["scale_source"] == "you"
    assert got["size"] == pytest.approx([0.064, 0.064, 0.12], abs=0.006) and got["mass"] > 0
    assert len(library.colors(meta["id"])) == 3 * got["vertices"]
    assert [o["id"] for o in library.listing()] == [meta["id"]]
    assert (lib / "objects" / meta["id"] / "collision_0.obj").exists()
    assert library.load("../etc") is None and not library.valid_id("../x")
    assert library.delete(meta["id"]) and library.load(meta["id"]) is None


def test_scene_spec_with_a_library_object(lib):
    from pps.world import scene

    meta = _saved()
    spec, notes = scene.normalize({"objects": [{"name": "my bottle", "mesh": meta["id"], "pos": [0.4, 0.0]}]})
    o = spec["objects"][0]
    assert (
        notes == []
        and o["mesh"] == meta["id"]
        and o["scale"] == 1.0
        and o["kind"] == "bottle"
        and o["name"] == "my_bottle"
    )
    assert scene.normalize(spec)[0] == spec  # idempotent
    big, _ = scene.normalize({"objects": [{"name": "b", "mesh": meta["id"], "size": [0.128, 0.128, 0.24]}]})
    assert big["objects"][0]["scale"] == pytest.approx(2.0, rel=0.05)  # a size sets the scale (generators)
    gone, notes = scene.normalize({"objects": [{"name": "ghost", "mesh": "nope_123456"}]})
    assert gone["objects"] == [] and "missing" in notes[0]


def test_imported_object_rests_on_the_table_under_gravity(lib):
    import mujoco

    from pps.world import scene

    meta = _saved()
    spec = {"template": "table", "objects": [{"name": "my_bottle", "mesh": meta["id"], "pos": [0.40, 0.0]}]}
    try:
        w = scene.build("panda", spec, physics=True)
    except ValueError as e:
        models_missing(f"robot model not on disk: {e}")
    m, d = w.model, mujoco.MjData(w.model)
    nq, nv = w.ik_model.nq, w.ik_model.nv
    b = m.body("obj_my_bottle").id
    assert (
        m.geom_contype[m.geom("obj_my_bottle_collision").id] == 0
        and m.geom_contype[m.geom("obj_my_bottle_hull0").id] == 3
    )
    assert m.body_mass[b] == pytest.approx(meta["mass"], rel=5e-3)  # (the spec keeps grams)
    d.qpos[:nq] = w.home_q[:nq]
    mujoco.mj_forward(m, d)
    for _ in range(600):  # 1.2 s, the arm held at home
        d.qpos[:nq] = w.home_q[:nq]
        d.qvel[:nv] = 0
        mujoco.mj_step(m, d)
    z = d.xpos[b][2]
    assert z == pytest.approx(w.surface["top"] + meta["size"][2] / 2, abs=0.006)
    assert np.linalg.norm(d.qvel[nv : nv + 3]) < 0.02
    kin = scene.build("panda", spec, physics=False)  # kinematic: the bounding primitive is the object
    assert kin.model.geom_contype[kin.model.geom("obj_my_bottle_collision").id] == 1
    assert sim.MESHES.get("rc_my_bottle_vis") == meta["id"]


def test_generator_draws_library_objects(lib):
    import importlib.util
    from pathlib import Path

    from pps.world import scene

    spec_ = importlib.util.spec_from_file_location(
        "gen", Path(__file__).resolve().parents[1] / "scripts" / "generate.py"
    )
    gen = importlib.util.module_from_spec(spec_)
    spec_.loader.exec_module(gen)
    meta = _saved()
    pool = gen.library_objects()
    assert [m["id"] for m in pool] == [meta["id"]]
    rng = np.random.default_rng(3)
    seen = []
    for _ in range(6):
        sp = gen.randomize_scene(
            "table", [{"name": "red_cup", "kind": "cup", "color": "red", "_named": True}], rng, True, pool, True
        )
        n, _ = scene.normalize(sp)
        seen += [o for o in n["objects"] if o.get("mesh")]
    assert seen and all(o["mesh"] == meta["id"] and 0.9 <= o["scale"] <= 1.1 for o in seen)


# ---------------------------------------------------------------------------------------------- providers (no network)
def _glb_box():
    box = trimesh.creation.box(extents=[0.2, 0.4, 0.1])  # y up in glTF: 0.4 tall, 0.1 deep
    box.visual = trimesh.visual.ColorVisuals(box, vertex_colors=np.tile([30, 200, 60, 255], (len(box.vertices), 1)))
    return trimesh.Scene(box).export(file_type="glb")


def test_fal_trellis_adapter_with_stubbed_http(monkeypatch):
    calls, glb = [], _glb_box()

    def fake(method, url, headers=None, body=None, timeout=60):
        calls.append((method, url, (headers or {}).get("Authorization"), body))
        if method == "POST":
            assert body["image_url"].startswith("data:image/jpeg;base64,")
            return 200, json.dumps(
                {
                    "request_id": "r1",
                    "status_url": "https://queue.fal.run/fal-ai/trellis/requests/r1/status",
                    "response_url": "https://queue.fal.run/fal-ai/trellis/requests/r1",
                }
            ).encode()
        if url.endswith("/status"):
            n = sum(1 for c in calls if c[1].endswith("/status"))
            return 200, json.dumps({"status": "IN_PROGRESS" if n < 2 else "COMPLETED"}).encode()
        if url.endswith("/requests/r1"):
            return 200, json.dumps({"model_mesh": {"url": "https://cdn.example/m.glb"}}).encode()
        return 200, glb

    monkeypatch.setattr(cloud, "fetch", fake)
    img, _, mask, _ = render("box", (0.1, 0.06, 0.08))
    r = cloud.FalTrellis().reconstruct(img, mask, {}, keys={"fal": "SECRET"}, poll=0)
    assert (
        calls[0][0] == "POST" and calls[0][1] == "https://queue.fal.run/fal-ai/trellis" and calls[0][2] == "Key SECRET"
    )
    assert r.provider == "fal_trellis" and "MIT" in r.license
    assert np.asarray(r.mesh.extents) == pytest.approx([0.2, 0.1, 0.4], abs=1e-6)  # turned z up
    assert r.colors is not None and np.all(r.colors[:, 1] > 150)


def test_cloud_errors_never_show_the_key(monkeypatch):
    monkeypatch.setattr(cloud, "fetch", lambda *a, **k: (401, b'{"detail": "bad key SECRET"}'))
    img, _, mask, _ = render("box", (0.1, 0.06, 0.08))
    with pytest.raises(ReconError) as e:
        cloud.FalTrellis().reconstruct(img, mask, {}, keys={"fal": "SECRET"}, poll=0)
    assert "rejected" in str(e.value) and "SECRET" not in str(e.value)
    monkeypatch.setattr(cloud, "fetch", lambda *a, **k: (402, b'{"message": "no credits"}'))
    with pytest.raises(ReconError) as e:
        cloud.Meshy().reconstruct(img, mask, {}, keys={"meshy": "SECRET2"}, poll=0)
    assert "credits" in str(e.value) and "SECRET2" not in str(e.value)


def test_meshy_adapter_with_stubbed_http(monkeypatch):
    glb, seen = _glb_box(), []

    def fake(method, url, headers=None, body=None, timeout=60):
        seen.append((method, url))
        if method == "POST":
            assert headers["Authorization"] == "Bearer K" and body["should_texture"] is True
            return 202, b'{"result": "t9"}'
        if url.endswith("/t9"):
            n = sum(1 for _, u in seen if u.endswith("/t9"))
            return 200, json.dumps(
                {
                    "status": "SUCCEEDED" if n > 1 else "IN_PROGRESS",
                    "progress": 50,
                    "model_urls": {"glb": "https://assets.meshy.ai/x.glb"},
                }
            ).encode()
        return 200, glb

    monkeypatch.setattr(cloud, "fetch", fake)
    img, _, mask, _ = render("box", (0.1, 0.06, 0.08))
    r = cloud.Meshy().reconstruct(img, mask, {}, keys={"meshy": "K"}, poll=0)
    assert r.provider == "meshy" and "CC BY 4.0" in r.license and len(r.mesh.faces) == 12


def test_triposr_adapter_with_a_stub_model(monkeypatch):
    class Stub:
        def __call__(self, images, device):
            self.image = np.asarray(images[0])
            return ["codes"]

        def extract_mesh(self, codes, has_vertex_color, resolution=256):
            m = trimesh.creation.box(extents=[0.3, 0.5, 0.8])  # raw TripoSR frame: x to the camera, y across, z up
            m.visual = trimesh.visual.ColorVisuals(m, vertex_colors=np.tile([220, 40, 40, 255], (len(m.vertices), 1)))
            return [m]

    stub = Stub()
    monkeypatch.setattr(triposr, "_load", lambda: (stub, "cpu"))
    img, _, mask, _ = render("cyl", (0.04, 0.12))
    r = triposr.TripoSR().reconstruct(img, mask, {})
    assert stub.image.shape == (512, 512, 3) and tuple(stub.image[0, 0]) == (128, 128, 128)  # cut out on grey
    assert np.asarray(r.mesh.extents) == pytest.approx([0.5, 0.3, 0.8])  # x across the photo
    assert r.colors is not None and r.license.startswith("MIT") and r.px_per_unit > 0
    ok, why = triposr.TripoSR().available()
    assert ok or why  # never downloads by itself


def test_provider_listing_names_licenses():
    from pps.recon.providers import listing

    ps = {p["id"]: p for p in listing({})}
    assert set(ps) == {"primitive", "triposr", "fal_trellis", "meshy"}
    assert ps["primitive"]["ok"] and ps["primitive"]["offline"]
    assert all(p["license"] for p in ps.values())


# ---------------------------------------------------------------------------------------------- scan my table
def test_layout_to_spec_keeps_relative_positions():
    from pps.recon import space

    layout = {
        "surface": {"kind": "desk"},
        "objects": [
            {"name": "cup", "kind": "cup", "color": "red", "size_m": [0.08, 0.08, 0.1], "xy": [-0.2, 0.30]},
            {"name": "box", "kind": "box", "color": "blue", "size_m": [0.1, 0.15, 0.06], "xy": [0.1, 0.45]},
        ],
    }
    spec, notes = space.to_spec(layout)
    a, b = spec["objects"]
    assert spec["template"] == "table" and notes == []
    assert a["pos"][1] > b["pos"][1]  # photo left -> robot's left (+y)
    assert b["pos"][0] > a["pos"][0]  # further from the camera -> further from the robot (+x)
    assert b["pos"][0] - a["pos"][0] == pytest.approx(0.15) and a["pos"][1] - b["pos"][1] == pytest.approx(0.30)
    assert space.template_for("kitchen counter") == "kitchen"


def test_blobs_find_objects_on_a_plain_table():
    from pps.recon import space

    img = np.full((360, 480, 3), (30, 40, 50), np.uint8)
    cv2.rectangle(img, (40, 120), (440, 350), (90, 130, 170), -1)  # the table top
    for x, y, c in ((120, 220, (40, 40, 220)), (240, 260, (60, 190, 60)), (350, 200, (200, 90, 40))):
        cv2.rectangle(img, (x - 18, y - 30), (x + 18, y + 20), c, -1)
    found, hull = space.blobs(img)
    assert len(found) == 3 and hull[300, 240]
    assert {space.color_name(img, b["mask"]) for b in found} == {"red", "green", "blue"}


def test_assumed_plane_geometry():
    from pps.recon import space

    K = fit.intrinsics(640, 480)
    pl = space.assumed_plane(K)
    p = pl.hit(320, 400)
    assert p is not None and abs(pl.n @ p + pl.d) < 1e-9
    top = fit.project((p + pl.n * 0.1)[None], K)[0, 1]
    assert pl.height(p, top) == pytest.approx(0.1, rel=1e-3)
