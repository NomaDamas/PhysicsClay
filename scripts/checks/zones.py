"""Keep-out zones (#D): a zone blocks a path through it, a route around passes, pick_place detours, remove and clear work."""

THROUGH = [[0.28, -0.12, 0.84], [0.30, -0.02, 0.84], [0.30, 0.10, 0.84]]       # hand height: crosses zone A
OVER = [[0.28, -0.12, 1.14], [0.30, -0.02, 1.14], [0.30, 0.10, 1.14]]          # same line, above the 30 cm zone
CARRY = [[0.28, -0.12, 1.04], [0.29, -0.02, 1.14], [0.29, 0.10, 1.14], [0.29, 0.20, 1.02]]   # the core "over the vase" route


async def run(t):
    st = t.state

    async def until(pred, sec=4.0):   # state frames can lag on a loaded machine
        for _ in range(int(sec / 0.25)):
            if pred(st().get("zones") or []):
                return True
            await t.pump(0.25)
        return pred(st().get("zones") or [])
    msg = lambda r: r.get("message", "")
    await t.send(type="zone", op="clear"); await until(lambda z: z == [])
    await t.send(type="zone", op="add", rect=[0.24, -0.08, 0.38, 0.02], source="paint"); await until(lambda z: len(z) == 1)
    z = st().get("zones") or []
    t.check("zone: paint a rectangle", len(z) == 1 and z[0]["id"] == "A" and z[0]["shape"] == "rect" and z[0]["label"] == "keep-out zone A", str(z)[:120])

    await t.send(type="path3d", points=THROUGH, hand="right"); await t.pump(1.5)
    p, m = st().get("path") or {}, " ".join(st().get("messages") or [])
    t.check("zone: blocks a path through it (waypoints red, zone named)", p and not all(p.get("ok") or [True]) and "keep-out zone A" in m, m[:90])
    r = await t.task("follow_path", 3)
    t.check("zone: follow_path refuses, naming the zone", r.get("status") == "failed" and "keep-out zone A" in msg(r), msg(r)[:90])
    await t.send(type="path3d", points=OVER, hand="right"); await t.pump(1.5)
    p = st().get("path") or {}
    t.check("zone: a route around it passes", p and p.get("ok") and all(p["ok"]), f"{sum(p.get('ok') or [])}/{len(p.get('ok') or [])} ok")
    await t.send(type="zone", op="add", rect=[0.22, -0.06, 0.36, 0.24], height=0.5, source="paint"); await t.pump(1.5)
    p = st().get("path") or {}
    t.check("zone: a new (taller) zone re-checks the drawn route", p and not all(p.get("ok") or [True]), f"{sum(p.get('ok') or [])}/{len(p.get('ok') or [])} ok")

    await t.send(type="path3d", points=CARRY, hand="right"); await t.pump(1.2)
    r = await t.task("pick_place", 6, object="mug", target="along my path")
    t.check("zone: carrying along a route through it is refused", r.get("status") == "failed" and "keep-out zone" in msg(r), msg(r)[:90])
    await t.send(type="zone", op="remove", id="B"); await t.send(type="clear"); await t.pump(0.4)
    await t.task("reset_scene", 1.5)

    r = await t.task("pick_place", 12, object="mug", target="center")
    ok = r.get("status") in ("running", "done") and "keep-out zone A" in msg(r)
    r = await t.until_done(45) if ok else r
    t.check("zone: pick_place detours around/over it", ok and r.get("status") == "done", msg(r)[:100])
    await t.task("reset_scene", 1.5)

    await t.send(type="zone", op="add", near="vase", source="voice"); await until(lambda z: len(z) == 2)
    z = st().get("zones") or []
    say = (st().get("say") or {}).get("text", "")
    t.check("zone: 'stay away from the vase' adds a zone around it", len(z) == 2 and z[1]["shape"] == "circle" and z[1]["near"] == "the vase" and "stay away" in say, say[:80])
    await t.send(type="zone", op="remove", id="A"); await until(lambda z: len(z) == 1)
    z = st().get("zones") or []
    t.check("zone: remove", [x["id"] for x in z] == ["B"], str([x["id"] for x in z]))
    r = await t.task("pick_place", 12, object="mug", target="left")   # without zones this is refused (the vase)
    ok = r.get("status") in ("running", "done") and "vase keep-out zone" in msg(r)
    r = await t.until_done(45) if ok else r
    t.check("zone: mug -> left routes around the vase zone", ok and r.get("status") == "done", msg(r)[:100])

    await t.send(type="zone", op="clear")
    t.check("zone: clear", await until(lambda z: z == []) and st().get("zones") == [], str(st().get("zones")))
    await t.task("reset_scene", 1.5)
    r = await t.task("pick_place", 3, object="mug", target="left")
    t.check("zone: no zones -> unchanged behaviour (vase refusal)", r.get("status") == "failed" and "vase" in msg(r), msg(r)[:80])
