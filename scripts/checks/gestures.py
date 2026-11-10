"""#G gestures + nudge pad. Synthetic landmarks: the pure classifier first, then each gesture end to end over the protocol
(test-only {"type": "gesture_feed"} plays frames through the server's detector), then nudge/speed messages."""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NAMES = ("stop", "slower", "come_here", "go_on")


def _arm(sx, el, wr, index, thumb, pinky):
    s = "l" if sx > 0 else "r"
    return {f"{s}_elbow": el, f"{s}_wrist": wr, f"{s}_index": index, f"{s}_thumb": thumb, f"{s}_pinky": pinky}


def _add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def rest_arm(sx):
    sh = [sx * 0.18, -0.45, 0.0]
    el = _add(sh, [sx * 0.04, 0.25, 0.0]); wr = _add(el, [0.0, 0.25, 0.0])
    return _arm(sx, el, wr, _add(wr, [0, 0.08, 0]), _add(wr, [-sx * 0.02, 0.06, -0.02]), _add(wr, [sx * 0.02, 0.07, 0]))


def body(**arms):
    lm = {"nose": [0.0, -0.65, -0.05], "l_shoulder": [0.18, -0.45, 0.0], "r_shoulder": [-0.18, -0.45, 0.0],
          "l_hip": [0.10, 0.0, 0.0], "r_hip": [-0.10, 0.0, 0.0]}
    lm.update(rest_arm(1.0)); lm.update(rest_arm(-1.0))
    for v in arms.values():
        lm.update(v)
    return lm


def pose(name, t=0.0):
    if name == "rest":
        return body()
    if name == "stop":
        return body(l=_arm(1, [0.25, -0.65, 0], [0.20, -0.85, 0], [0.20, -0.95, 0], [0.16, -0.90, 0], [0.22, -0.93, 0]),
                    r=_arm(-1, [-0.25, -0.65, 0], [-0.20, -0.85, 0], [-0.20, -0.95, 0], [-0.16, -0.90, 0], [-0.22, -0.93, 0]))
    if name == "slower":
        arms = {}
        for sx in (1.0, -1.0):
            wr = [sx * 0.25, -0.15, -0.30]
            arms[str(sx)] = _arm(sx, [sx * 0.23, -0.22, -0.10], wr, _add(wr, [sx * 0.02, 0.0, -0.08]),
                                 _add(wr, [-sx * 0.04, 0.0, -0.05]), _add(wr, [sx * 0.05, 0.01, -0.06]))
        return body(**arms)
    if name == "go_on":          # right thumb up at chest height, left hand down
        wr = [-0.15, -0.20, -0.25]
        return body(r=_arm(-1, [-0.20, -0.25, -0.05], wr, _add(wr, [0.02, -0.01, -0.03]), _add(wr, [0, -0.07, 0]),
                           _add(wr, [0, 0.02, -0.02])))
    if name == "palm_forward":   # right open palm pushed toward the camera, fingers up
        wr = [-0.18, -0.42, -0.45]
        return body(r=_arm(-1, [-0.18, -0.43, -0.22], wr, _add(wr, [0, -0.12, -0.02]), _add(wr, [0.05, -0.07, -0.02]),
                           _add(wr, [-0.04, -0.10, -0.02])))
    if name == "come_here":      # right hand in front at chest height, fingers curling toward the body (1.2 Hz)
        wr = [-0.12, -0.25, -0.30]
        k = 0.07 * math.sin(2 * math.pi * 1.2 * t)
        return body(r=_arm(-1, [-0.20, -0.25, -0.08], wr, _add(wr, [0, -0.03, -0.08 + k]), _add(wr, [0.02, -0.02, -0.04]),
                           _add(wr, [-0.03, -0.02, -0.06 + k])))
    raise KeyError(name)


def frames(name, seconds, fps=20):
    return [pose(name, i / fps) for i in range(int(seconds * fps))]


def unit(check):
    """The classifier and debounce as pure functions: each gesture fires exactly once; rest and the --fake motion never."""
    sys.path.insert(0, str(ROOT))
    from pps.features.gestures import Detector, classify_static
    from pps.fake import fake_landmarks
    for name, want in (("rest", None), ("stop", "stop"), ("slower", "slower"), ("go_on", "go_on"), ("palm_forward", "go_on")):
        check(f"gestures: classify {name}", classify_static(pose(name)) == want, str(classify_static(pose(name))))
    for name, want in (("stop", "stop"), ("slower", "slower"), ("go_on", "go_on"), ("palm_forward", "go_on"), ("come_here", "come_here")):
        det, fired = Detector(), []
        seq = frames(name, 3.0) + frames("rest", 0.5)
        for i, lm in enumerate(seq):
            f = det.update(lm, 100.0 + i / 20)[1]
            if f:
                fired.append(f)
        check(f"gestures: {name} held 3 s fires once", fired == [want], str(fired))
    det, fired = Detector(), []
    for i in range(20 * 40):                      # 40 s of the synthetic --fake person: moving, so nothing fires
        f = det.update(fake_landmarks(i / 20), 100.0 + i / 20)[1]
        if f:
            fired.append(f)
    check("gestures: synthetic motion never fires", not fired, str(fired[:4]))


async def feed(t, name, seconds):
    await t.send(type="gesture_feed", frames=frames(name, seconds) + frames("rest", 0.6), fps=20)
    seen, t0 = [], None
    for _ in range(int((seconds + 1.6) / 0.25)):
        st = await t.pump(0.25)
        last = (st.get("gestures") or {}).get("last")
        if last and (not seen or seen[-1] != last["id"]):
            seen.append(last["id"])
            t0 = last
    return seen, t0


async def run(t):
    unit(t.check)
    await t.send(type="mock", enabled=False)      # with the mock brain on, it owns "stop"; test ours
    st = await t.pump(0.5)
    t.check("gestures: state present", isinstance(st.get("gestures"), dict) and isinstance(st.get("nudge"), dict))
    base = ((st.get("gestures") or {}).get("last") or {}).get("id", 0)

    # slower: speed drops once
    await t.send(type="speed", scale=1.0)
    await t.pump(0.4)
    seen, last = await feed(t, "slower", 2.0)
    sp = t.state().get("speed_scale")
    t.check("gestures: slower fires once -> speed down", len([i for i in seen if i > base]) == 1 and last["name"] == "slower" and sp is not None and sp < 1.0,
            f"seen={seen} speed={sp}")
    base = max([base] + seen)

    # go on: with a drawn path waiting, it runs it
    await t.send(type="path3d", points=[[0.32, -0.18, 0.84], [0.36, -0.12, 0.86], [0.40, -0.06, 0.86]], hand="right")
    await t.pump(0.6)
    path = t.state().get("path") or {}
    ready = bool(path.get("ok")) and all(path["ok"])
    seen, last = await feed(t, "go_on", 2.0)
    st = t.state()
    t.check("gestures: go on fires once -> runs the waiting path", len([i for i in seen if i > base]) == 1 and last["name"] == "go_on"
            and (not ready or last.get("effect") == "Following the path"),
            f"seen={seen} path_ok={ready} effect={last and last.get('effect')} task={(st.get('task') or {}).get('action')}")
    base = max([base] + seen)
    await t.until_done(15)

    # come here: reaches toward the person (wait out the 2 s cross-gesture cooldown first: the path may finish fast)
    await t.pump(2.2)
    seen, last = await feed(t, "come_here", 2.2)
    st = t.state()
    t.check("gestures: come here fires once -> reaches toward you", len([i for i in seen if i > base]) == 1 and last["name"] == "come_here"
            and (st.get("task") or {}).get("id") == "g-come", f"seen={seen} effect={last and last.get('effect')} task={(st.get('task') or {}).get('message')}")
    base = max([base] + seen)
    await t.until_done(15)

    # stop (mock brain off): stops
    seen, last = await feed(t, "stop", 1.5)
    st = t.state()
    t.check("gestures: stop fires once -> stops", len([i for i in seen if i > base]) == 1 and last["name"] == "stop"
            and (st.get("task") or {}).get("action") == "stop", f"seen={seen}")
    await t.task("home", 1.5)

    # nudge a running pick-and-place: the place target moves 2 cm
    await t.send(type="speed", scale=0.3)
    tk = await t.task("pick_place", 3, object="mug", target="center")
    await t.pump(0.4)
    n0 = t.state().get("nudge") or {}
    # with physics, away from the vase: the G1's 18 cm wide side gripper doesn't fit 2 cm closer to it at that spot (the
    # nudge says so); the kinematic hand was a point
    dy = -0.02 if ((t.state().get("physics") or {}).get("mode")) == "dynamic" else 0.02
    await t.send(type="nudge", dx=0.0, dy=dy, dz=0.0)
    await t.pump(0.6)
    n1 = t.state().get("nudge") or {}
    t.check("nudge: active while a task runs", tk.get("status") == "running" and n0.get("active") and n0.get("kind") == "running", str(n0))
    p0, p1 = n0.get("pos"), n1.get("pos")
    t.check("nudge: moves the active target 2 cm", (n1.get("last") or {}).get("ok") and p0 and p1 and abs(p1[1] - p0[1] - dy) < 1e-3
            and abs(p1[0] - p0[0]) < 1e-3, f"{p0} -> {p1} {n1.get('offset')} {(n1.get('last') or {}).get('message')}")
    await t.send(type="speed", scale=1.0)
    done = await t.until_done(30)
    objs = {o["name"]: o["pos"] for o in t.state().get("scene_objects") or []}
    t.check("nudge: the mug lands where the nudged target is", done.get("status") == "done" and "mug" in objs and p1 and abs(objs["mug"][1] - p1[1]) < 0.012,
            f"{done.get('status')} mug={objs.get('mug')} target={p1}")

    # nudge a marker; speed message
    await t.task("reset_scene", 1)
    await t.send(type="tap", u=0.5, v=0.6)
    await t.pump(0.6)
    mk = (t.state().get("markers") or [None])[-1]
    await t.send(type="nudge", dx=0.02, dy=0, dz=0)
    await t.pump(0.6)
    mk2 = (t.state().get("markers") or [None])[-1]
    t.check("nudge: moves the latest marker 2 cm", mk and mk2 and abs(mk2["pos"][0] - mk["pos"][0] - 0.02) < 1e-3, f"{mk and mk['pos']} -> {mk2 and mk2['pos']}")
    await t.send(type="speed", scale=0.4)
    await t.pump(0.5)
    t.check("speed: sets speed_scale", abs((t.state().get("speed_scale") or 0) - 0.4) < 1e-3, str(t.state().get("speed_scale")))
    await t.send(type="speed", scale=1.0)
    await t.pump(0.3)
