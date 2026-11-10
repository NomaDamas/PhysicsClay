"""Real photos -> simulated objects and spaces (#50, pps/recon, docs/RECON.md).

The "real photo" is a render of a simulated object (MuJoCo, robot hidden) with its metric depth, as a depth camera or
a phone's AR depth would give it: upload -> the object appears in state.world with its mesh, /scene.json streams the
mesh with vertex colours -> a Panda picks it with physics. Then "scan my table" on a rendered table with three things
-> roughly three objects -> Use this layout builds that scene. Without a GL context for the render the checks SKIP."""
import asyncio
import json
import math
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CUP = {"name": "red_cup", "kind": "cup", "color": "red", "size": [0.06, 0.06, 0.10], "pos": [0.42, 0.0]}
TABLE = [{"name": "red_cup", "kind": "cup", "color": "red", "pos": [0.36, 0.16]},
         {"name": "blue_box", "kind": "box", "color": "blue", "pos": [0.46, -0.02]},
         {"name": "green_can", "kind": "can", "color": "green", "pos": [0.34, -0.20]}]


def render(objects, lookat, distance, azimuth, elevation, w=640, h=480):
    """A photo + metric depth (16-bit PNG, mm) of objects on the table, robot hidden -> (jpg bytes, png bytes, K)."""
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    import cv2
    import mujoco
    import numpy as np

    from pps.world import scene
    wd = scene.build("panda", {"template": "table", "objects": objects}, physics=False)
    m = wd.model
    for g in range(m.ngeom):                              # the photo is of the table, not of the robot
        if m.geom_bodyid[g] in wd.robot_bodies:
            m.geom_group[g] = 5
    d = mujoco.MjData(m)
    if wd.home_q is not None:
        d.qpos[:len(wd.home_q)] = wd.home_q
    mujoco.mj_forward(m, d)
    opt = mujoco.MjvOption()
    opt.geomgroup[1] = 0                                  # IK target ghosts
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, distance, azimuth, elevation
    prev = mujoco.get_mju_user_warning()
    mujoco.set_mju_user_warning(lambda msg: None)        # macOS GL: "ARB_clip_control unavailable" would land in MUJOCO_LOG.TXT
    r = mujoco.Renderer(m, h, w)
    try:
        r.update_scene(d, cam, scene_option=opt)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(d, cam, scene_option=opt)
        depth = r.render().copy()
    finally:
        r.close()
        mujoco.set_mju_user_warning(prev)
    f = (h / 2) / math.tan(math.radians(float(m.vis.global_.fovy)) / 2)
    jpg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()
    png = cv2.imencode(".png", np.clip(depth * 1000, 0, 65535).astype(np.uint16))[1].tobytes()
    return jpg, png, [f, f, w / 2, h / 2]


async def upload(t, path, jpg, png, meta):
    import aiohttp
    fd = aiohttp.FormData()
    fd.add_field("photo", jpg, filename="photo.jpg", content_type="image/jpeg")
    fd.add_field("depth", png, filename="depth.png", content_type="image/png")
    fd.add_field("meta", json.dumps(meta))
    async with t.http.post(t.base + path, data=fd) as r:
        return await r.json()


async def job_done(t, jid, limit=120):
    for _ in range(int(limit / 0.5)):
        await t.pump(0.5)
        j = next((x for x in (t.state().get("recon") or {}).get("jobs") or [] if x.get("id") == jid), None)
        if j and j.get("status") in ("ready", "failed", "added"):
            async with t.http.get(f"{t.base}/recon/job/{jid}") as r:
                return await r.json()
    return None


async def world_change(t, rev, limit=40):
    for _ in range(int(limit / 0.25)):
        await t.pump(0.25)
        w = t.state().get("world") or {}
        if w.get("rev", 0) > rev:
            return w
    return None


async def run(t):
    st = t.state
    try:
        loop = asyncio.get_running_loop()
        photo = await loop.run_in_executor(None, lambda: render([CUP], [0.42, 0.0, 0.79], 0.40, 200, -32))
    except Exception as e:                                # no GL here (CI without a display): nothing to photograph
        print(f"SKIP  recon: can't render a test photo ({str(e)[:60]})", flush=True)
        return
    rev = (st().get("world") or {}).get("rev", 0)
    await t.send(type="world", op="set", robot="panda", scene={"template": "table", "objects": []}, id="recon-world")
    w = await world_change(t, rev)
    t.check("recon: Panda at an empty table", bool(w) and w["robot"]["id"] == "panda" and not w["scene"]["objects"])

    # ---- an object from a photo (+ metric depth): recognized, measured, made physical
    jpg, png, K = photo
    res = await upload(t, "/recon/import", jpg, png, {"provider": "primitive", "K": K, "hints": {"label": "cup", "vlm": False}, "tag": "check"})
    j = await job_done(t, res.get("job")) if res.get("ok") else None
    s = (j or {}).get("summary") or {}
    dims = s.get("size_cm") or [0, 0, 0]
    err = max(abs(dims[0] - 6) / 6, abs(dims[1] - 6) / 6, abs(dims[2] - 10) / 10) if j else 9
    t.check("recon: photo -> cylinder of the right size (metric depth)", bool(j) and j["status"] == "ready" and s.get("shape") in ("cylinder", "lathe")
            and err < 0.12, f"{s.get('shape')} {dims} cm (true 6 x 6 x 10, worst {err * 100:.0f} %), {s.get('mass_g')} g, "
            f"{s.get('hulls')} hull(s), scale from {(s.get('scale') or {}).get('label')}, {s.get('seconds')} s")
    async with t.http.get(f"{t.base}/recon/job/{res.get('job')}/mesh.bin") as r:
        b = await r.read()
    nv, nf = struct.unpack("<II", b[:8]) if len(b) >= 8 else (0, 0)
    t.check("recon: 3D preview mesh with colours", r.status == 200 and len(b) == 8 + nv * 12 + nf * 12 + nv * 3 and nf > 50, f"{nv} verts")

    rev = (st().get("world") or {}).get("rev", 0)
    await t.send(type="recon", op="add", job=res.get("job"), id="recon-add")
    w = await world_change(t, rev)
    objs = (w or {}).get("scene", {}).get("objects") or []
    o = next((x for x in objs if x.get("mesh")), None)
    t.check("recon: Add to scene -> the object is in state.world with its mesh", bool(o) and o["kind"] == "cup",
            f"{o and o['name']} ({o and o['mesh']}), {len(objs)} objects")
    async with t.http.get(t.base + "/scene.json") as r:
        sj = await r.json()
    col = next((v for v in (sj.get("meshes") or {}).values() if v.get("colors")), None)
    ok_c = False
    if col:
        async with t.http.get(t.base + col["url"]) as r:
            mb = await r.read()
        async with t.http.get(t.base + col["colors"]) as r:
            cb = await r.read()
        ok_c = len(cb) == 3 * struct.unpack("<II", mb[:8])[0]
    t.check("recon: the 3D stage gets the mesh and its vertex colours", ok_c, col and col.get("colors"))

    if o:
        name = o["name"]

        def pos():
            return next((x["pos"] for x in st().get("scene_objects") or [] if x["name"] == name.replace("_", " ")), None)
        p0 = pos()
        r_ = await t.task("pick_place", 8, object=name.replace("_", " "), target="left")
        if r_.get("status") == "running":
            r_ = await t.until_done(90)
        p1 = pos()
        moved = math.dist(p0[:2], p1[:2]) if p0 and p1 else 0
        held = (st().get("physics") or {}).get("mode")
        t.check("recon: a Panda picks and places the imported object with physics", r_.get("status") == "done" and moved > 0.10,
                f"{r_.get('status')}: {str(r_.get('message'))[:70]} (moved {moved:.3f} m, physics {held})")

    # ---- scan my table
    try:
        jpg, png, K = await loop.run_in_executor(None, lambda: render(TABLE, [0.42, 0.0, 0.74], 0.95, 0, -48))
    except Exception as e:
        t.check("recon: render the table photo", False, str(e)[:80])
        return
    res = await upload(t, "/recon/scan", jpg, png, {"K": K, "hints": {"vlm": False}, "tag": "check"})
    j = await job_done(t, res.get("job")) if res.get("ok") else None
    s = (j or {}).get("summary") or {}
    lay = s.get("layout") or {}
    n = s.get("count") or 0
    t.check("recon: scan my table -> roughly the right objects", bool(j) and 2 <= n <= 4,
            f"{n} found (3 there): {', '.join(s.get('objects_brief') or [])}; scale from {(lay.get('scale') or {}).get('source')}")
    rev = (st().get("world") or {}).get("rev", 0)
    await t.send(type="recon", op="use_layout", job=res.get("job"), shapes="primitive", id="recon-use")
    w = await world_change(t, rev)
    got = len((w or {}).get("scene", {}).get("objects") or [])
    t.check("recon: Use this layout builds the scene", bool(w) and got == n and got >= 2, f"{got} objects on the {(w or {}).get('scene', {}).get('template')}")
    rev = (st().get("world") or {}).get("rev", 0)
    await t.send(type="world", op="reset", id="recon-reset")
    await world_change(t, rev)
    if o:                                                 # the check's object leaves the person's library as it was
        await t.send(type="recon", op="delete_library", id=o["mesh"])
        await t.pump(0.3)
