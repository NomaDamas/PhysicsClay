"""Complex worlds (issue #48, docs/worlds.md): generated homes and industrial sites with real joints and clutter.

* kitchen_full + Panda: one plan card for "open the top drawer, take the spoon, put it in the sink, close the drawer",
  confirmed once; the drawer really opens (>= 60 %), the spoon ends up in the sink, the drawer is shut again, and the
  moving drawer is streamed in geom_poses (the 3D view renders it live). A jammed drawer is reported honestly.
* warehouse + Panda: pick a box out of the tote (bin picking) and put it in the empty tote.
* assembly_line + Panda: a part rides the conveyor to the photo-eye (the belt stops for it) and the arm picks it into
  the finished-parts bin.
* lab + Panda: a test tube goes from the yellow source rack into a free slot of the tube rack.
Seeds are fixed so the layouts are the ones these steps were tuned on; every seed builds (tests/test_worlds.py).
"""
import time

SEEDS = {"kitchen_full": 1, "warehouse": 1, "assembly_line": 1, "lab": 1}


async def run(t):
    st = t.state

    async def world(sec=60.0, **kw):
        rev = (st().get("world") or {}).get("rev", 0)
        await t.send(type="world", id=f"world-{rev}-{time.time():.0f}", **kw)
        t0 = time.time()
        while time.time() - t0 < sec:
            await t.pump(0.25)
            w = st().get("world") or {}
            if w.get("rev", 0) > rev:
                return w
            tk = st().get("task") or {}
            if tk.get("action") == "world" and tk.get("status") == "failed":
                return None
        return None

    def fixture(name):
        return next((f for f in (st().get("world") or {}).get("fixtures") or [] if f.get("name") == name), {})

    def where(obj):
        o = next((o for o in st().get("scene_objects") or [] if o.get("name", "").replace(" ", "_") == obj), None)
        return (o or {}).get("in")

    async def task(action, limit=90, **kw):
        res = await t.task(action, 8, **kw)
        if res.get("status") == "running":
            res = await t.until_done(limit)
        return res

    # ---------------------------------------------------------------- kitchen: one plan card, four steps
    w = await world(robot="panda", scene={"template": "kitchen_full", "seed": SEEDS["kitchen_full"]})
    fx = {f["name"] for f in (w or {}).get("fixtures") or []}
    cont = {c["name"]: c for c in (w or {}).get("containers") or []}
    t.check("worlds: kitchen_full builds with drawers, doors, a sink and clutter",
            bool(w) and {"drawer_1", "drawer_2", "fridge_door"} <= fx and "spoon" in (cont.get("drawer_1") or {}).get("objects", [])
            and "sink" in cont and len(st().get("scene_objects") or []) >= 12, f"{len(fx)} parts, {len(st().get('scene_objects') or [])} objects")
    async with t.http.get(t.base + "/scene.json") as resp:
        sj = await resp.json()
    drawer_geoms = [str(g["id"]) for g in sj.get("geoms") or [] if str(g.get("name", "")).startswith("fx_drawer_stack_drawer_1") and g.get("dynamic")]
    pose0 = (st().get("geom_poses") or {}).get(drawer_geoms[0]) if drawer_geoms else None
    steps = [{"action": "open", "fixture": "the top drawer", "amount": 0.8}, {"action": "pick_from", "container": "drawer_1", "object": "spoon"},
             {"action": "place_in", "object": "spoon", "container": "sink"}, {"action": "close", "fixture": "drawer_1"}]
    res = await t.task("sequence", 30, steps=steps, preview=True, text="Open the top drawer, take the spoon, put it in the sink, close the drawer.")
    p = st().get("plan") or {}
    t.check("worlds: a multi-step request is one plan card", res.get("status") == "proposed" and p.get("phase") == "proposing"
            and len(p.get("steps") or []) == 4, f"{res.get('status')}: {str(res.get('message'))[:70]} ({len(p.get('steps') or [])} steps)")
    best, moved, seq = 0.0, False, None
    if res.get("status") == "proposed":
        await t.send(type="plan", op="confirm", id=res.get("id"))
        t0 = time.time()
        while time.time() - t0 < 400:
            await t.pump(0.3)
            best = max(best, float(fixture("drawer_1").get("open") or 0.0))
            pose = (st().get("geom_poses") or {}).get(drawer_geoms[0]) if drawer_geoms else None
            moved = moved or bool(pose and pose0 and abs(pose[0] - pose0[0]) > 0.1)
            seq = st().get("sequence") or {}
            if seq.get("status") in ("done", "failed"):
                break
    seq = seq or {}
    t.check("worlds: kitchen plan runs: open the top drawer, take the spoon, put it in the sink, close the drawer",
            seq.get("status") == "done", f"{seq.get('status')}: {str(seq.get('message'))[:110]}")
    t.check("worlds: the drawer really opened (>= 60 %) and the 3D view saw it move", best >= 0.6 and moved, f"max {best:.2f}, streamed {moved}")
    t.check("worlds: the spoon is in the sink and the drawer is shut", where("spoon") == "sink" and float(fixture("drawer_1").get("open") or 1) < 0.1,
            f"spoon in {where('spoon')}, drawer_1 {fixture('drawer_1').get('open')}")
    res = await task("open", 120, fixture="the second drawer", amount=0.8)
    opened = float(fixture("drawer_2").get("open") or 0.0)
    res2 = await task("close", 120, fixture="drawer_2") if res.get("status") == "done" else {}
    t.check("worlds: the second drawer (its knob right under the top drawer's) opens from the front and closes again",
            res.get("status") == "done" and opened >= 0.6 and res2.get("status") == "done" and float(fixture("drawer_2").get("open") or 1) < 0.1,
            f"{res.get('message', '')[:60]} ({opened:.2f}); {res2.get('message', '')[:50]}")
    await t.send(type="fixture", op="lock", name="drawer_1", id="lock-1")
    await t.pump(0.5)
    res = await task("open", 60, fixture="drawer_1", amount=0.8)
    t.check("worlds: a jammed drawer is reported honestly (how far it got)", res.get("status") == "failed" and "stuck" in res.get("message", "")
            and float(fixture("drawer_1").get("open") or 0) < 0.3, res.get("message", "")[:100])
    await t.send(type="fixture", op="unlock", name="drawer_1", id="lock-2")
    await t.task("reset_scene", 1)

    # ---------------------------------------------------------------- warehouse: bin picking
    w = await world(robot="panda", scene={"template": "warehouse", "seed": SEEDS["warehouse"]})
    res = await task("pick_from", 60, container="tote", object="box")
    held = (st().get("holding") or {}).get("right")
    t.check("worlds: warehouse: pick a box out of the tote", bool(w) and res.get("status") == "done" and held == "box",
            f"{res.get('status')}: {res.get('message', '')[:80]}")
    res = await task("place_in", 60, object="box", container="the empty tote")
    t.check("worlds: warehouse: ... and put it in the empty tote", res.get("status") == "done" and where("box") == "tote_out",
            f"{res.get('status')}: {res.get('message', '')[:80]}")
    await t.task("reset_scene", 1)

    # ---------------------------------------------------------------- assembly line: a part rides the conveyor
    w = await world(robot="panda", scene={"template": "assembly_line", "seed": SEEDS["assembly_line"]})
    belt = fixture("conveyor")
    t0, part = time.time(), None
    while time.time() - t0 < 60:
        await t.pump(0.5)
        part = fixture("conveyor").get("holding")
        if part:
            break
    t.check("worlds: assembly line: a part rides the belt to the photo-eye and the belt stops for it",
            bool(w) and belt.get("running") and part is not None and fixture("conveyor").get("holding") == part, f"part at the eye: {part}")
    if part:
        res = await task("place_in", 60, object=part, container="out_bin")
        t.check("worlds: assembly line: the arm picks it off the belt into the bin", res.get("status") == "done" and where(part) == "out_bin",
                f"{res.get('status')}: {res.get('message', '')[:80]}")
    await t.task("reset_scene", 1)

    # ---------------------------------------------------------------- lab: a tube into the rack
    w = await world(robot="panda", scene={"template": "lab", "seed": SEEDS["lab"]})
    tube = next((o.get("name", "").replace(" ", "_") for o in st().get("scene_objects") or [] if str(o.get("in", "")).startswith("source_rack")), None)
    res = await task("place_in", 60, object=tube or "tube_1", container="the tube rack")
    slot = where(tube) or ""
    t.check("worlds: lab: a tube goes into a free slot of the tube rack", bool(w) and res.get("status") == "done"
            and slot.startswith("rack_"), f"{tube} -> {slot}: {res.get('message', '')[:70]}")
    await world(op="reset")
