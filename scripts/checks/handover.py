"""Two-arm handover (#37): on ALOHA, `{type: "task", action: "handover", object, to, target}` makes one arm pick the
object, both arms meet in the middle, the receiving gripper closes and holds it, the giving gripper opens, and the
receiver puts it down. The object passes from one hand to the other without being dropped (with physics: held by
contact the whole way, the contact grasp moves from the right jaws to the left jaws). A one-armed robot refuses.

Runs inside scripts/selftest.py (async def run(t))."""
import time

BAR = {"name": "bar", "kind": "box", "color": "yellow", "size": [0.03, 0.24, 0.03], "pos": [0.33, -0.22]}
SCENE = {"template": "table", "objects": [BAR, {"name": "cup", "kind": "cup", "color": "blue", "pos": [0.30, 0.25]}]}


async def _until(t, cond, limit):
    t0 = time.time()
    while time.time() - t0 < limit:
        await t.pump(0.2)
        if cond(t.state()):
            return True
    return False


async def _world(t, **kw):
    rev = (t.state().get("world") or {}).get("rev", 0)
    await t.send(type="world", id=f"world-ho-{rev}", **kw)
    await _until(t, lambda s: (s.get("world") or {}).get("rev", 0) > rev, 25.0)


def _bar(s):
    o = next((o for o in s.get("scene_objects") or [] if o["name"] == "bar"), None)
    return o["pos"] if o else None


async def run(t):
    await _world(t, op="set", robot="aloha", scene=SCENE)
    st = t.state()
    t.check("handover: ALOHA lists handover between its arms", "bimanual" in ((st.get("world") or {}).get("robot") or {}).get("capabilities", []),
            str(((st.get("world") or {}).get("robot") or {}).get("capabilities")))
    z0 = (_bar(st) or [0, 0, 0])[2]
    tid = f"ho-{time.time():.3f}"
    await t.send(type="task", id=tid, action="handover", object="bar", to="left", target="left")
    holders, low, phys = [], 0.0, (st.get("physics") or {}).get("mode") == "dynamic"
    grasps = []

    def watch(s):
        nonlocal low
        task = s.get("task") or {}
        h = tuple(sorted(k for k, v in (s.get("holding") or {}).items() if v == "bar"))
        if h and (not holders or holders[-1] != h):
            holders.append(h)
        g = tuple(sorted(k for k, v in ((s.get("physics") or {}).get("grasp") or {}).items() if v == "bar"))
        if g and (not grasps or grasps[-1] != g):
            grasps.append(g)
        p = _bar(s)
        if p and holders and not h and p[2] > z0 + 0.03 and task.get("id") == tid and task.get("status") == "running":
            low = min(low, -1.0)                  # off the table and in no hand: dropped
        return task.get("id") == tid and task.get("status") in ("done", "failed")
    await _until(t, watch, 5.0)
    fin = await t.until_done(90)
    watch(t.state())
    end = _bar(t.state()) or [0, 0, 0]
    ok = fin.get("id") == tid and fin.get("status") == "done"
    t.check("handover: right arm picks, both meet, left arm takes it and puts it down", ok and "left hand" in fin.get("message", "")
            and end[1] > 0.1, f"{fin.get('status')}: {fin.get('message', '')[:110]} bar={end}")
    passed = ("right",) in holders and ("left",) in holders and holders.index(("right",)) < holders.index(("left",))
    t.check("handover: the bar goes from the right hand to the left hand (both hold it in between)", passed and ("left", "right") in holders,
            f"holders={holders}")
    if phys:
        t.check("handover: held by contact all the way, no drop", ("right",) in grasps and ("left",) in grasps
                and grasps.index(("right",)) < grasps.index(("left",)) and low == 0.0 and end[2] < z0 + 0.02,
                f"contact grasps={grasps} {'dropped' if low else 'never in the air unheld'}, end z {end[2]:.3f}")
    await _world(t, op="set", robot="panda")
    r = await t.task("handover", 3, object="bar", to="left")
    t.check("handover: a one-armed robot refuses", r.get("status") == "failed" and "one arm" in r.get("message", ""), r.get("message", "")[:90])
    await _world(t, op="reset")
