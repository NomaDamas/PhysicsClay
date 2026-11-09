"""UX 3 world (#17): every non-experimental robot body switches in and picks & places the mug (status done, the mug moved);
a kitchen counter with three objects is built on the fly and one is picked; objects are added / removed; humanoid-only
features are refused on an arm; the world resets to the G1 at the default table (the UX 2.0 checks run after this)."""
import struct

KITCHEN = {"template": "kitchen", "objects": [{"name": "bowl", "kind": "bowl", "color": "white"}, {"name": "red cup", "kind": "cup", "color": "red"},
                                              {"name": "banana", "kind": "fruit", "color": "yellow"}]}


async def run(t):
    st = t.state

    async def world(sec=20.0, **kw):
        rev = (st().get("world") or {}).get("rev", 0)
        await t.send(type="world", id=f"world-{rev}", **kw)
        for _ in range(int(sec / 0.25)):
            await t.pump(0.25)
            w = st().get("world") or {}
            if w.get("rev", 0) > rev:
                return w
            tk = st().get("task") or {}
            if tk.get("action") == "world" and tk.get("status") == "failed":
                return None
        return None

    def pos(name):
        return next((o["pos"] for o in st().get("scene_objects") or [] if o["name"] == name), None)

    def moved(a, b):
        return a is not None and b is not None and ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    w0 = st().get("world") or {}
    robots = [r for r in w0.get("robots") or [] if not r.get("experimental")]
    t.check("world: state.world lists robot bodies and templates",
            w0.get("robot", {}).get("id") == "g1" and len(robots) >= 9 and {"table", "kitchen", "shelf", "floor"} <= {x["id"] for x in w0.get("templates") or []},
            f"{len(robots)} robots, rev {w0.get('rev')}")

    for r in robots:
        if r["id"] == "g1":
            continue
        w = await world(op="set", robot=r["id"])
        ok = bool(w) and w["robot"]["id"] == r["id"] and st().get("robot") == r["id"]
        p0 = pos("mug")
        res = await t.task("pick_place", 6, object="mug", target="center")
        if res.get("status") == "running":
            res = await t.until_done(40)
        d = moved(p0, pos("mug"))
        t.check(f"world: {r['id']} switches in and picks & places the mug", ok and res.get("status") == "done" and d and d > 0.05,
                f"{res.get('status')}: {res.get('message', '')[:70]} (moved {round(d or 0, 3)} m)")

    w = await world(op="set", robot="panda", scene=KITCHEN)
    names = sorted(o["name"] for o in st().get("scene_objects") or [])
    t.check("world: kitchen scene built on the fly", bool(w) and w["scene"]["template"] == "kitchen" and names == ["banana", "bowl", "red cup"],
            str(names))
    async with t.http.get(t.base + "/scene.json") as resp:
        sj = await resp.json()
    mesh = next(iter(sj.get("meshes") or {}), None)
    ok_mesh = False
    if mesh:
        async with t.http.get(t.base + sj["meshes"][mesh]["url"]) as resp:
            b = await resp.read()
            nv, nf = struct.unpack("<II", b[:8]) if len(b) >= 8 else (0, 0)
            ok_mesh = resp.status == 200 and len(b) == 8 + nv * 12 + nf * 12
    t.check("world: /scene.json follows the world (kitchen counter, panda meshes)",
            abs(sj.get("surface", {}).get("z", 0) - 0.90) < 1e-6 and ok_mesh and len(sj.get("geoms") or []) > 20, f"{len(sj.get('geoms') or [])} geoms")
    p0 = pos("red cup")
    res = await t.task("pick_place", 6, object="red cup", target="center")
    if res.get("status") == "running":
        res = await t.until_done(40)
    d = moved(p0, pos("red cup"))
    t.check("world: pick the red cup on the kitchen counter", res.get("status") == "done" and d and d > 0.05,
            f"{res.get('status')}: {res.get('message', '')[:70]}")
    w = await world(op="add_object", object={"kind": "bottle", "color": "green"})
    t.check("world: add an object", bool(w) and "green bottle" in [o["name"] for o in st().get("scene_objects") or []],
            str([o["name"] for o in st().get("scene_objects") or []]))
    w = await world(op="remove_object", name="bowl")
    t.check("world: remove an object", bool(w) and "bowl" not in [o["name"] for o in st().get("scene_objects") or []],
            str([o["name"] for o in st().get("scene_objects") or []]))
    res = await t.task("follow_me", 2, enabled=True)
    t.check("world: humanoid-only feature refused on an arm", res.get("status") == "failed" and "humanoid" in res.get("message", ""),
            res.get("message", "")[:80])

    w = await world(op="reset")
    names = sorted(o["name"] for o in st().get("scene_objects") or [])
    t.check("world: reset -> the G1 at the default table", bool(w) and st().get("robot") == "g1" and w["robot"]["id"] == "g1"
            and names == ["blue block", "green block", "mug", "red block", "vase"], f"{st().get('robot')} {names}")
