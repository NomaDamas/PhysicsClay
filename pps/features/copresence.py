"""Co-presence (UX 3, issue #20): the person is in the robot's world, across the table.

The person's pose (MediaPipe world landmarks, or the synthetic dancer without a camera) drives a mannequin standing
across the table, facing the robot. Their pointing, hands and words are read in the robot's own world frame:

* Avatar: landmarks -> robot world. Hip centre at PERSON (x 1.15, facing -x), hips at the person's leg length
  (from the ankles when visible, else HIP_Z). Not mirrored: the person's right hand is on the robot's left, as in
  real life. MediaPipe (x right, y down, z away from the camera) -> world (x = ax + z, y = ay - x, z = hip_z - y).
* Shared-frame pointing: a ray from the shoulder through the index fingertip (wrist when the fingertip is missing),
  intersected with the table top -> `hit`. Movable objects within RAY_R of the ray are candidates. The pointing is
  "armed" only when the arm is extended toward the table and held still for ARM_HOLD s; while armed it also moves
  marker P (so "there" and the candidates question resolve to it, like a tap).
* Deixis: `resolve_deixis(ctx, text)` resolves this/that/here/there (이거/저거/여기/저기) against the armed pointing at
  speech time (history HISTORY_S s; "put this there" uses the first and the last pointing of a TWO_POINT_S window)
  and returns the task to run. Browser: {type:"copresence", op:"utter", text, id} -> state.copresence.reply
  {id, ok, kind, object, candidates, spot, marker, message, task}; with run:true the server also runs the task
  (the Brain agent uses that path; it talks to the world only through messages).
* Pick and hold: {type:"task", action:"pick_place", hold:true, object, object_pos?} picks the object up and keeps it
  in the hand ("pick this up"). "Which one?" (candidates.py) still asks first when the pointing is ambiguous.
* Handover: {type:"task", action:"handover", object?, hand?} or {type:"copresence", op:"handover"}: carry the held
  (or named / pointed) object toward the person's offered hand, clamped to what the arm can reach, hold it there
  and let go once their hand comes within RELEASE_DIST. The object then stays in their hand.
* Attention: what the robot attends to (the pointed object or spot while pointing, the person when they speak or
  wave, the object in hand while working); with the "Shared space" view on, the robot-eye camera widens and turns
  slightly toward it.
* Hand paths: the person's wrist paths of the last PATH_S s in robot-world coordinates (`hand_path(ctx, side)`),
  not mirrored, for Teach-by-Showing.

Messages: {type:"copresence", op: "on"|"off"|"view"(on: bool)|"heard"|"utter"|"handover"|"demo"(phase?)|"place"(x, y)}.
"demo" plays a synthetic person (no camera needed): the whole 24 s sequence, or with phase point_mug | point_spot |
offer | take | rest one pose held for 30 s.
Test-only: {type:"copresence_feed", frames:[lm, ...], fps:20, hold: seconds} plays landmark frames (MediaPipe world
coordinates) instead of the camera; the last frame stays for `hold` s. `synth_pose()` builds such frames.

State: copresence = {on, view, source, joints: {name: [x,y,z]}, pointing: {side, from, dir, hit, armed, target,
candidates} | None, hand_open: {l, r}, offer: side | None, attention: {kind, name?, pos} | None,
handover: {phase, object, target, hand} | None, hand_paths: {l, r}, reply, recent_point}.
"""
import collections
import math
import re
import time

import mujoco
import numpy as np

from .. import ext
from ..tasks import TABLE_TOP, TABLE_X, TABLE_Y, densify

PERSON = (1.15, 0.0)     # hip centre on the floor, across the table from the robot (which stands near x 0 facing +x)
HIP_Z = 0.93             # hip height when the legs aren't visible
ANKLE_Z = 0.07
RAY_R = 0.06             # m: objects this close to the pointing ray are candidates
ARM_HOLD = 0.5           # s an extended arm must be held before the pointing is armed
ARM_STILL = 0.06         # m the fingertip may drift while arming
OPEN_ON = 0.45           # pinky (index) to wrist / forearm length for a hand to read open (see grip_open)
OPEN_OFF = 0.40          # ...and to stay open once it is (hysteresis against real-camera flicker)
EXTEND = 0.80            # straightness of an extended arm (shoulder-tip distance / arm length)
FORWARD = 0.18           # m the fingertip must be in front of the shoulder (toward the robot)
HISTORY_S = 1.5          # s of pointing history a "this"/"there" looks back over
TWO_POINT_S = 4.0        # s window for "put this there" (object first, then the spot)
RELEASE_DIST = 0.12      # m: the robot lets go once the person's hand has come this close to the object it offers ...
GRAB_DIST = 0.07         # m: ... (or at the object) and stayed GRAB_HOLD_S, after presenting it for PRESENT_S
GRAB_HOLD_S, PRESENT_S = 0.4, 0.8
HANDOVER_GAP = 0.10      # m: the robot offers the object this far in front of your hand; you close the gap (P5 safety:
                         #    the robot never moves into you)
OFFER_TIMEOUT = 15.0     # s the robot holds an offered object out before it gives up (and keeps holding it)
PATH_S = 4.0
SMOOTH = 0.5             # EMA factor for joints (calm avatar)
SHARED_FOVY = 90.0       # robot-eye field of view in the Shared space view (sees the table and the person)
SHARED_TILT = 18.0       # deg the robot eye pitches up in the Shared space view
LOOK_MAX = 14.0          # deg the attention may turn the eye away from that base
GREY = (0.80, 0.81, 0.84, 1.0)
ORANGE = (0.96, 0.57, 0.27, 0.95)
INDIGO = (0.36, 0.47, 0.87, 0.9)

BONES = (("l_shoulder", "l_elbow", 0.045), ("l_elbow", "l_wrist", 0.038), ("r_shoulder", "r_elbow", 0.045),
         ("r_elbow", "r_wrist", 0.038), ("l_hip", "l_knee", 0.065), ("l_knee", "l_ankle", 0.05),
         ("r_hip", "r_knee", 0.065), ("r_knee", "r_ankle", 0.05), ("l_shoulder", "r_shoulder", 0.06),
         ("l_hip", "r_hip", 0.075), ("neck", "pelvis", 0.13))

OBJ_DEIX = ("this", "that", "these", "those", "이거", "저거", "그거", "이것", "저것", "그것", "요거")
PLACE_DEIX = ("here", "there", "여기", "저기", "거기", "이쪽", "저쪽")
PICK_V = ("pick", "grab", "take", "lift", "get", "hold", "집어", "들어", "잡아", "가져")
PLACE_V = ("put", "place", "set", "move", "drop", "bring", "leave", "놔", "놓", "둬", "올려", "옮겨", "내려")
HANDOVER = re.compile(r"\b(give|hand|pass|bring)\b[^.?!]*\bme\b|\bhand (it|this|that) over\b|건네|넘겨|나한테|저한테|내게|제게")
HANDOVER_KO = re.compile(r"(^|\s)(줘|줘요|주세요|줄래)")      # 줘 on its own ("이거 줘"), not "놔줘" / "집어 줘"
REGION_WORDS = (("left", "left"), ("right", "right"), ("center", "center"), ("centre", "center"), ("middle", "center"),
                ("front", "front"), ("back", "back"), ("왼", "left"), ("오른", "right"), ("가운데", "center"))


# ---------------------------------------------------------------------- geometry (pure)
def _v(p):
    return np.asarray(p, float)


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v * 0.0


def hip_height(lm):
    """Leg length from the ankles (MediaPipe y is down) when they're visible and the person stands, else HIP_Z."""
    hips = [_v(lm[k]) for k in ("l_hip", "r_hip") if k in lm]
    ank = [_v(lm[k]) for k in ("l_ankle", "r_ankle") if k in lm]
    if not hips or not ank:
        return HIP_Z
    drop = max(a[1] for a in ank) - float(np.mean([h[1] for h in hips]))
    return float(np.clip(drop + ANKLE_Z, 0.8, 1.05)) if drop > 0.6 else HIP_Z


def to_world(lm, anchor=PERSON, hip_z=None):
    """MediaPipe pose-world landmarks -> {name: np.array xyz} in the robot world, standing across the table facing
    the robot (-x). Proper rotation, so not mirrored. Adds pelvis, neck and head; fills in missing legs standing."""
    if not lm or "l_shoulder" not in lm or "r_shoulder" not in lm:
        return None
    P = {k: _v(v) for k, v in lm.items() if isinstance(v, (list, tuple)) and len(v) == 3}
    hips = [P[k] for k in ("l_hip", "r_hip") if k in P]
    sh_mid = (P["l_shoulder"] + P["r_shoulder"]) / 2
    o = np.mean(hips, axis=0) if hips else sh_mid + np.array([0.0, 0.48, 0.0])
    sw = float(np.linalg.norm(P["l_shoulder"] - P["r_shoulder"]))
    s = 1.0 if 0.2 < sw < 0.6 else 0.38 / max(sw, 1e-3)       # MediaPipe world is metric; only fix absurd scales
    hz = hip_height(lm) if hip_z is None else hip_z

    def w(p):
        q = (p - o) * s
        return np.array([anchor[0] + q[2], anchor[1] - q[0], hz - q[1]])

    J = {k: w(p) for k, p in P.items()}
    pelvis = np.array([anchor[0], anchor[1], hz])
    J["pelvis"] = pelvis
    across = J["l_shoulder"] - J["r_shoulder"]                 # the person's right -> left
    facing = _unit(np.cross(across, [0.0, 0.0, 1.0]))          # toward the robot when they face it
    if not np.any(facing):
        facing = np.array([-1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    J["neck"] = (J["l_shoulder"] + J["r_shoulder"]) / 2 + up * 0.05
    J["head"] = J["nose"] - facing * 0.07 + up * 0.04 if "nose" in J else J["neck"] + up * 0.2
    side = _unit(np.array([across[0], across[1], 0.0])) if np.any(across[:2]) else np.array([0.0, -1.0, 0.0])
    for sd, sg in (("l", 1.0), ("r", -1.0)):
        if f"{sd}_hip" not in J:
            J[f"{sd}_hip"] = pelvis + sg * side * 0.10
        hp = J[f"{sd}_hip"]
        if f"{sd}_knee" not in J:
            J[f"{sd}_knee"] = np.array([hp[0], hp[1], (hp[2] + ANKLE_Z) / 2 + 0.02])
        if f"{sd}_ankle" not in J:
            J[f"{sd}_ankle"] = np.array([hp[0], hp[1], ANKLE_Z])
    return J


def from_world(J, anchor=PERSON, hip_z=HIP_Z):
    """Inverse of to_world (for synthetic frames): {name: world xyz} -> MediaPipe pose-world landmarks."""
    out = {}
    for k, p in J.items():
        p = _v(p)
        out[k] = [round(float(-(p[1] - anchor[1])), 4), round(float(hip_z - p[2]), 4), round(float(p[0] - anchor[0]), 4)]
    return out


def synth_pose(point_at=None, offer=None, side=None, anchor=PERSON, hip_z=HIP_Z):
    """A person standing across the table (MediaPipe world landmarks): at rest, pointing at a world point with a
    straight arm, or holding out an open hand toward a world point. Deterministic (checks, the demo)."""
    ax, ay = anchor
    J = {"nose": [ax - 0.10, ay, hip_z + 0.69],
         "l_hip": [ax, ay - 0.10, hip_z], "r_hip": [ax, ay + 0.10, hip_z],
         "l_knee": [ax - 0.02, ay - 0.10, (hip_z + ANKLE_Z) / 2 + 0.02], "r_knee": [ax - 0.02, ay + 0.10, (hip_z + ANKLE_Z) / 2 + 0.02],
         "l_ankle": [ax, ay - 0.10, ANKLE_Z], "r_ankle": [ax, ay + 0.10, ANKLE_Z]}
    target = point_at if point_at is not None else offer
    if side is None:
        side = "l" if target is not None and target[1] < ay else "r"   # the person's left hand is on the robot's right (-y)
    lean = np.zeros(3)
    if offer is not None:                  # lean over the table toward a hand-over point out of arm's reach
        sh = np.array([ax, ay + (-0.18 if side == "l" else 0.18), hip_z + 0.47])
        k = float(np.clip(np.linalg.norm(_v(offer) - sh) - 0.55, 0.0, 0.35))
        lean = np.array([-k, 0.0, -0.3 * k])
        J["nose"] = (_v(J["nose"]) + lean).tolist()
    for sd, sy in (("l", -1.0), ("r", 1.0)):
        sh = np.array([ax, ay + sy * 0.18, hip_z + 0.47]) + lean
        J[f"{sd}_shoulder"] = sh.tolist()
        if target is not None and sd == side:
            d = _unit(_v(target) - sh)
            el, wr = sh + 0.28 * d, sh + 0.55 * d
            if offer is not None:          # open hand: fingers spread away from the wrist
                reach = min(0.55, float(np.linalg.norm(_v(offer) - sh)))
                wr = sh + reach * d
                el = sh + reach * 0.5 * d + np.array([0.0, 0.0, -0.04])
                tip = wr + 0.14 * d
                J[f"{sd}_index"], J[f"{sd}_pinky"] = tip.tolist(), (tip + np.array([0.0, sy * 0.04, 0.0])).tolist()
                J[f"{sd}_thumb"] = (wr + 0.06 * d + np.array([0.0, -sy * 0.05, 0.0])).tolist()
            else:                          # pointing: index finger along the arm, others curled
                J[f"{sd}_index"] = (sh + 0.65 * d).tolist()
                J[f"{sd}_pinky"] = (wr + 0.05 * d + np.array([0.0, sy * 0.02, -0.01])).tolist()
                J[f"{sd}_thumb"] = (wr + 0.05 * d + np.array([0.0, -sy * 0.02, 0.01])).tolist()
        else:                              # resting arm, hanging by the side
            el, wr = sh + np.array([0.0, sy * 0.02, -0.28]), sh + np.array([-0.02, sy * 0.03, -0.54])
            J[f"{sd}_index"] = (wr + np.array([0.0, 0.0, -0.09])).tolist()
            J[f"{sd}_pinky"] = (wr + np.array([0.0, sy * 0.02, -0.08])).tolist()
            J[f"{sd}_thumb"] = (wr + np.array([-0.02, -sy * 0.01, -0.06])).tolist()
        J[f"{sd}_elbow"], J[f"{sd}_wrist"] = el.tolist(), wr.tolist()
    return from_world(J, anchor, hip_z)


def on_table(p, margin=0.0):
    return TABLE_X[0] - margin <= p[0] <= TABLE_X[1] + margin and TABLE_Y[0] - margin <= p[1] <= TABLE_Y[1] + margin


def arm_ray(J, sd):
    """-> pointing candidate for one arm {side, from, dir, hit, extension, forward} or None when the arm is not
    extended toward the robot's side of the room."""
    need = (f"{sd}_shoulder", f"{sd}_elbow", f"{sd}_wrist")
    if J is None or any(k not in J for k in need):
        return None
    sh, el, wr = (J[k] for k in need)
    tip = J.get(f"{sd}_index", wr)
    length = float(np.linalg.norm(el - sh) + np.linalg.norm(wr - el) + np.linalg.norm(tip - wr))
    if length < 0.2:
        return None
    ext_ = float(np.linalg.norm(tip - sh)) / length
    across = J["l_shoulder"] - J["r_shoulder"]
    facing = _unit(np.cross(across, [0.0, 0.0, 1.0]))
    fwd = float(np.dot(tip - sh, facing))
    if ext_ < EXTEND or fwd < FORWARD or grip_open(J, sd):   # an open hand held out is an offer, not a point
        return None
    d = _unit(tip - sh)
    hit = None
    if d[2] < -0.03:
        t = (TABLE_TOP - tip[2]) / d[2]
        if t > 0:
            h = tip + t * d
            if on_table(h, 0.02):
                hit = np.array([h[0], h[1], TABLE_TOP])
    return {"side": sd, "from": tip, "dir": d, "hit": hit, "extension": ext_, "forward": fwd}


def ray_candidates(tasks, ray, skip=()):
    """Movable objects whose side is within RAY_R of the pointing ray (or of its table hit), nearest first:
    [(gap to the object's side, name, the ray goes through it)]."""
    out = []
    tip, d, hit = ray["from"], ray["dir"], ray["hit"]
    for name, o in getattr(tasks, "objs", {}).items():
        if name in skip:
            continue
        c = tasks.mug_center(name)
        v = c - tip
        s = float(np.dot(v, d))
        if s <= 0:
            continue
        dist = float(np.linalg.norm(v - s * d))
        if hit is not None:
            dist = min(dist, float(np.linalg.norm(c[:2] - hit[:2])))
        r = float(o.get("r", 0.03))
        if dist - r < RAY_R:
            out.append((max(dist - r, 0.0), name, dist < r))
    return sorted(out)


def grip_open(J, sd, was=None):
    """Open hand: the pinky (or, without it, the index) landmark far from the wrist relative to the forearm. A
    pointing hand has the index out but the pinky curled, so it does not count as open.

    Real camera (#37): MediaPipe Pose puts the pinky / index points at the knuckles, so on real footage the pinky
    ratio sits at 0.15-0.39 for closed, relaxed and even open hands alike, with single-frame spikes to 0.42+; at the
    old 0.40 threshold `hand_open` flickered (37 flips in 6 s of a palm wave, true on a barbell grip). Hence a
    stricter OPEN_ON with hysteresis: a hand that was open (`was`) stays open down to OPEN_OFF. Synthetic people are
    unaffected (open hands 0.49-0.57, others <= 0.37). The reliable open/closed on a real camera comes from MediaPipe
    Hands in the browser (pps/human/model.py, which watch.py feeds back into `hand_open`)."""
    try:
        wr, el = J[f"{sd}_wrist"], J[f"{sd}_elbow"]
    except KeyError:
        return None
    fore = float(np.linalg.norm(wr - el))
    if fore < 1e-3:
        return None
    if f"{sd}_pinky" in J:
        return bool(np.linalg.norm(J[f"{sd}_pinky"] - wr) / fore >= (OPEN_OFF if was else OPEN_ON))
    if f"{sd}_index" in J:
        return bool(np.linalg.norm(J[f"{sd}_index"] - wr) / fore >= (OPEN_OFF if was else OPEN_ON))
    return None


def offered_hand(J):
    """The hand held out toward the robot (wrist well in front of the shoulder; open hands first), or None."""
    if J is None:
        return None
    across = J["l_shoulder"] - J["r_shoulder"]
    facing = _unit(np.cross(across, [0.0, 0.0, 1.0]))
    best = None
    for sd in ("l", "r"):
        if f"{sd}_wrist" not in J:
            continue
        f = float(np.dot(J[f"{sd}_wrist"] - J[f"{sd}_shoulder"], facing))
        score = (bool(grip_open(J, sd)), f)            # an open hand wins over a pointing one
        if f > 0.15 and (best is None or score > best[0]):
            best = (score, sd)
    return None if best is None else best[1]


def hand_pos(J, sd):
    wr = J[f"{sd}_wrist"]
    tip = J.get(f"{sd}_index")
    return wr if tip is None else (wr + tip) / 2


# ---------------------------------------------------------------------- state
def _st(ctx):
    s = ctx.store.get("copresence")
    if s is None:
        s = ctx.store["copresence"] = {
            "on": True, "view": False, "J": None, "src": None, "feed": None, "point": None, "arm": None,
            "hist": collections.deque(maxlen=240), "spoke_t": 0.0, "wave": collections.deque(maxlen=60),
            "paths": {"l": collections.deque(maxlen=200), "r": collections.deque(maxlen=200)},
            "handover": None, "given": None, "reply": None, "last_obj": None, "cam": {}, "anchor": PERSON,
            "attention": None, "hand_open": {}, "offer": None}
    return s


def _r(p, n=3):
    return None if p is None else [round(float(x), n) for x in p]


def _pretty(tasks, name):
    try:
        return tasks.pretty(name)
    except Exception:
        return "the " + str(name).replace("_", " ")


def _held(tasks):
    return next(((h, o) for h, o in getattr(tasks, "holding", {}).items() if o), (None, None))


def _say(ctx, text):
    if getattr(ctx, "brain", None) is not None and hasattr(ctx.brain, "utter"):
        try:
            ctx.brain.utter(text, "vision")
        except Exception:
            pass


# ---------------------------------------------------------------------- per frame
@ext.handler("copresence_feed")
def on_feed(ctx, msg):
    st = _st(ctx)
    frames = [f for f in (msg.get("frames") or []) if isinstance(f, dict)]
    st["feed"] = {"frames": frames, "fps": float(msg.get("fps") or 20), "t0": time.time(),
                  "hold": float(msg.get("hold") or 0.0)} if frames else None
    st["arm"], st["J"] = None, None


def _landmarks(ctx, st, now):
    feed = st["feed"]
    if feed is not None:
        i = int((now - feed["t0"]) * feed["fps"])
        n = len(feed["frames"])
        if i < n:
            return feed["frames"][i], "feed"
        if now - feed["t0"] - n / feed["fps"] < feed["hold"]:
            return feed["frames"][-1], "feed"
        st["feed"] = None
    return getattr(ctx, "lm", None), getattr(ctx, "source", None)


def _pick_palm(rt, y):
    """The robot's hand on the side of a point (y > 0 = robot's left); single-arm bodies only have 'right_palm'."""
    hands = list(getattr(rt, "hands", None) or getattr(rt, "palm_sid", {}).keys() or ["right_palm"])
    want = "right_palm" if y <= 0 else "left_palm"
    return want if want in hands else hands[0]


@ext.tick
def copresence_tick(ctx):
    st, now, tasks = _st(ctx), time.time(), ctx.tasks
    if st.get("anchor_auto", True):     # stand across whatever surface the current world has (table, counter, shelf…)
        st["anchor"] = (min(1.6, TABLE_X[1] + 0.33), 0.5 * (TABLE_Y[0] + TABLE_Y[1]))
    lm, src = _landmarks(ctx, st, now)
    st["src"] = src
    J = to_world(lm, st["anchor"]) if (st["on"] and lm) else None
    if J is not None and st["J"] is not None:
        prev = st["J"]
        J = {k: (SMOOTH * v + (1 - SMOOTH) * prev[k]) if k in prev else v for k, v in J.items()}
    st["J"] = J
    _pointing(ctx, st, J, src, now)
    _track_hands(st, J, now)
    _handover_tick(ctx, st, J, now)
    st["attention"] = _attention(ctx, st, J, now)
    _look(ctx, st)


def _pointing(ctx, st, J, src, now):
    tasks = ctx.tasks
    rays = [r for r in (arm_ray(J, "l"), arm_ray(J, "r")) if r is not None] if J is not None else []
    ray = max(rays, key=lambda r: r["forward"]) if rays else None
    can_arm = src not in ("synthetic", "video", None)   # the synthetic dancer and a replayed video are not pointing at you
    arm = st["arm"]
    if ray is None or not can_arm:
        st["arm"] = None
    elif arm is None or arm["side"] != ray["side"]:
        st["arm"] = {"side": ray["side"], "t0": now, "tip0": ray["from"].copy(), "armed": False}
    elif not arm["armed"]:
        if float(np.linalg.norm(ray["from"] - arm["tip0"])) > ARM_STILL:
            arm["t0"], arm["tip0"] = now, ray["from"].copy()
        elif now - arm["t0"] >= ARM_HOLD:
            arm["armed"] = True
    armed = bool(st["arm"] and st["arm"]["armed"])
    if ray is not None:
        cands = ray_candidates(tasks, ray, skip=(st["given"]["object"],) if st["given"] else ())
        through = [n for _, n, th in cands if th]
        ray["candidates"] = through if len(through) == 1 else [n for _, n, _ in cands]   # straight at one: no doubt
        ray["target"] = cands[0][1] if cands else None
        ray["armed"] = armed
        if cands and cands[0][2]:          # the ray goes through the object: the spot meant is where it stands
            c = tasks.mug_center(cands[0][1])
            ray["hit"] = np.array([c[0], c[1], TABLE_TOP])
    st["point"] = ray
    if armed and ray is not None and (ray["hit"] is not None or ray["target"]):
        st["hist"].append((now, {"hit": None if ray["hit"] is None else ray["hit"].copy(), "target": ray["target"],
                                 "candidates": list(ray["candidates"]), "side": ray["side"]}))
        if ray["target"]:
            st["last_obj"] = ray["target"]
        if ray["hit"] is not None:     # keep marker P on the armed spot: "there" and the "which one?" question use it
            p = next((mk for mk in tasks.markers if mk["id"] == "P"), None)
            if p is None or float(np.linalg.norm(p["pos"][:2] - ray["hit"][:2])) > 0.05:
                tasks.set_marker("P", ray["hit"].copy())
    while st["hist"] and now - st["hist"][0][0] > TWO_POINT_S + 1.0:
        st["hist"].popleft()


def _track_hands(st, J, now):
    prev = st.get("pose_open") or {}         # the pose's own reading (watch.py may overwrite hand_open from Hands)
    st["hand_open"], st["offer"] = {}, None
    if J is None:
        st["pose_open"] = {}
        return
    for sd in ("l", "r"):
        st["hand_open"][sd] = grip_open(J, sd, was=prev.get(sd))
        if f"{sd}_wrist" in J:
            pth = st["paths"][sd]
            pth.append((now, J[f"{sd}_wrist"].copy()))
            while pth and now - pth[0][0] > PATH_S:
                pth.popleft()
    st["pose_open"] = dict(st["hand_open"])
    st["offer"] = offered_hand(J)
    # wave: a hand above the shoulder swinging sideways (>= 2 direction changes in 1.5 s)
    up = [sd for sd in ("l", "r") if f"{sd}_wrist" in J and J[f"{sd}_wrist"][2] > J[f"{sd}_shoulder"][2] + 0.08]
    wv = st["wave"]
    if up:
        wv.append((now, float(J[f"{up[0]}_wrist"][1])))
    else:
        wv.clear()
    win = [y for t, y in wv if now - t < 1.5]
    if len(win) >= 8 and max(win) - min(win) > 0.08:
        dy = np.diff(win)
        sg = np.sign(dy[np.abs(dy) > 0.004])
        if np.count_nonzero(np.diff(sg)) >= 2:
            st["spoke_t"] = now


def hand_path(ctx, side="r", seconds=PATH_S):
    """The person's wrist path over the last `seconds`, robot-world coordinates (not mirrored): [(t, [x,y,z])]."""
    st, now = _st(ctx), time.time()
    return [(t, _r(p)) for t, p in st["paths"].get(side, ()) if now - t <= seconds]


def _attention(ctx, st, J, now):
    tasks = ctx.tasks
    ray = st["point"]
    if ray is not None and ray.get("armed"):
        if ray["target"]:
            return {"kind": "object", "name": ray["target"], "pos": _r(tasks.mug_center(ray["target"]))}
        if ray["hit"] is not None:
            return {"kind": "spot", "name": None, "pos": _r(ray["hit"])}
    ho = st["handover"]
    if ho and ho["phase"] == "offering" and J is not None:
        sd = st["offer"] or ho.get("side") or "r"
        if f"{sd}_wrist" in J:
            return {"kind": "person", "name": "your hand", "pos": _r(hand_pos(J, sd))}
    if J is not None and now - st["spoke_t"] < 2.5:
        return {"kind": "person", "name": "you", "pos": _r(J["head"])}
    t = tasks.task or {}
    if t.get("status") == "running" and t.get("action") in ("pick_place", "handover") and getattr(tasks, "cur", None) in getattr(tasks, "objs", {}):
        return {"kind": "object", "name": tasks.cur, "pos": _r(tasks.mug_center(tasks.cur))}
    if J is not None:
        return {"kind": "person", "name": "you", "pos": _r(J["head"])}
    return None


# ---------------------------------------------------------------------- robot eye: shared view + attention
def _quat_from_forward(f):
    z = -_unit(f)
    x = np.cross(f, [0.0, 0.0, 1.0])
    x = _unit(x) if np.linalg.norm(x) > 1e-6 else np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.column_stack([x, y, z]).flatten())
    return q


def _rotate_toward(a, b, max_deg):
    a, b = _unit(a), _unit(b)
    ang = math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))
    if ang <= max_deg or ang < 1e-6:
        return b
    k = math.radians(max_deg) / math.radians(ang)
    return _unit((1 - k) * a + k * b)


def _look(ctx, st):
    """Shared space view: the robot eye widens and pitches up to see the table and the person, and turns slightly
    toward what it attends to. Changes the model camera, so taps/projections (computed from it) stay consistent.
    Restores the original camera when the view is off."""
    m, d = ctx.rt.model, ctx.rt.data
    try:
        cid = m.camera("robot_view").id
    except KeyError:
        return
    if int(m.cam_bodyid[cid]) != 0:
        return
    key = id(m)
    cam = st["cam"].get(key)
    if cam is None:
        if not st["view"]:
            return
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, m.cam_quat[cid])
        f0 = -mat.reshape(3, 3)[:, 2]
        cam = st["cam"][key] = {"quat": m.cam_quat[cid].copy(), "fovy": float(m.cam_fovy[cid]), "f0": f0, "f": f0.copy(),
                                "fov": float(m.cam_fovy[cid])}
    f0 = cam["f0"]
    if st["view"]:
        horiz = _unit(np.array([f0[0], f0[1], 0.0]))
        pitch = math.degrees(math.asin(float(np.clip(f0[2], -1, 1)))) + SHARED_TILT
        base = _unit(horiz * math.cos(math.radians(pitch)) + np.array([0, 0, 1.0]) * math.sin(math.radians(pitch)))
        att = st["attention"]
        want = base
        if att and att.get("pos") is not None:
            to = _unit(_v(att["pos"]) - d.cam_xpos[cid])
            want = _rotate_toward(base, _unit(base + 0.5 * (to - base)), LOOK_MAX)
        want_fov = SHARED_FOVY
    else:
        want, want_fov = f0, cam["fovy"]
    cam["f"] = _unit(cam["f"] + 0.15 * (want - cam["f"]))
    cam["fov"] += 0.15 * (want_fov - cam["fov"])
    if not st["view"] and float(np.linalg.norm(cam["f"] - f0)) < 2e-3 and abs(cam["fov"] - cam["fovy"]) < 0.05:
        m.cam_quat[cid], m.cam_fovy[cid] = cam["quat"], cam["fovy"]
        st["cam"].pop(key, None)
    else:
        m.cam_quat[cid] = _quat_from_forward(cam["f"])
        m.cam_fovy[cid] = cam["fov"]
    mujoco.mj_camlight(m, d)


# ---------------------------------------------------------------------- deixis
def _has(t, words):
    for w in words:
        if w.isascii():
            if re.search(rf"(?<![a-z']){re.escape(w)}(?![a-z])", t):
                return True
        elif w in t:
            return True
    return False


def classify(text):
    """Utterance -> 'handover' | 'pick_place' | 'place' | 'pick' | 'pick_region' | None (no deixis we handle)."""
    t = str(text or "").lower().strip()
    if not t:
        return None
    if HANDOVER.search(t) or (HANDOVER_KO.search(t) and not _has(t, PLACE_V + PICK_V)):
        return "handover"
    obj, place = _has(t, OBJ_DEIX), _has(t, PLACE_DEIX)
    if obj and place:
        return "pick_place"
    if place and (_has(t, PLACE_V) or _has(t, ("it", "them")) or len(t.split()) <= 3):
        return "place"
    if obj and _region(t) and _has(t, PLACE_V + ("take",)):
        return "pick_region"
    if obj and (_has(t, PICK_V) or re.search(r"\bup\b", t)):
        return "pick"
    return None


def _region(t):
    return next((r for w, r in REGION_WORDS if (re.search(rf"\b{w}\b", t) if w.isascii() else w in t)), None)


def _recent(st, now, window=HISTORY_S):
    return [s for t, s in st["hist"] if now - t <= window]


def _segments(st, now):
    """Distinct pointing targets over the TWO_POINT_S window, oldest first: [{target, hit, candidates}]."""
    segs = []
    for t, s in st["hist"]:
        if now - t > TWO_POINT_S:
            continue
        key = s["target"] or (None if s["hit"] is None else tuple(np.round(s["hit"][:2] / 0.08)))
        if not segs or segs[-1][0] != key:
            segs.append((key, s))
        else:
            segs[-1] = (key, s)
    return [s for _, s in segs]


def _named(tasks, text):
    """An object the words name ("the mug"), via candidates.matches when it's loaded."""
    try:
        from .candidates import matches
        names = matches(tasks, text)
    except Exception:
        names = []
    return names


def resolve_deixis(ctx, text, now=None):
    """Resolve this/that/here/there in `text` against the person's pointing at speech time.

    Returns {ok, kind, object, candidates, spot, marker, message, task}. `task` is the task message to run (without
    type/id), e.g. {"action": "pick_place", "hold": True, "object": "mug", "object_pos": [...]} for "pick this up",
    {"action": "pick_place", "object": "mug", "target": "marker C"} for "put it there" (a marker is added at the
    pointed spot), {"action": "handover", "object": ..., "hand": [...]} for "give it to me". kind None: no deixis
    this module handles (let the normal parser have it). For the Brain agent: call it in the worker (it reads and
    adds markers), or send {type:"copresence", op:"utter", text, run:true} from the agent thread."""
    st, tasks = _st(ctx), ctx.tasks
    now = time.time() if now is None else now
    kind = classify(text)
    out = {"ok": False, "kind": kind, "object": None, "candidates": [], "spot": None, "marker": None, "message": "", "task": None}
    if kind is None:
        return out
    t = str(text).lower()
    words = _named(tasks, t)
    recent = _recent(st, now)
    cur = recent[-1] if recent else None
    ray = st["point"]
    if cur is None and ray is not None and ray.get("armed"):
        cur = {"hit": ray["hit"], "target": ray["target"], "candidates": ray["candidates"], "side": ray["side"]}
    held_palm, held = _held(tasks)

    def pointed_object(sample):
        cands = list(sample["candidates"]) if sample else []
        if words:
            cands = [c for c in cands if c in words] or cands
        return cands

    if kind == "handover":
        J = st["J"]
        if J is None:
            out["message"] = "I can't see you. Stand where my camera sees you, then hold out your hand."
            return out
        obj = held
        if obj is None:
            cands = pointed_object(cur) if _has(t, OBJ_DEIX) else []
            obj = (cands[0] if len(cands) == 1 else None) or (words[0] if len(words) == 1 else None)
            if obj is None and len(cands) > 1:
                out.update(candidates=cands, message="Which one should I give you?")
                obj = None
        if obj is None and not out["candidates"]:
            out["message"] = "I'm not holding anything. Point at something and say 'give me this'."
            return out
        sd = st["offer"]
        hand = hand_pos(J, sd) if sd else None
        out.update(ok=True, object=obj, spot=_r(hand))
        task = {"action": "handover", "object": obj or "this", "hand": _r(hand)}
        if out["candidates"]:
            task["candidates"] = out["candidates"]
        out["task"] = task
        out["message"] = (f"Bringing {_pretty(tasks, obj)} to your hand." if obj else out["message"])
        return out

    if kind in ("pick", "pick_region"):
        if cur is None:
            out["message"] = "I didn't see you point. Hold your arm out toward it for a moment, then say it again."
            return out
        cands = pointed_object(cur)
        if not cands:
            out["message"] = "You're pointing at an empty spot; I don't see anything to pick up there."
            out["spot"] = _r(cur["hit"])
            return out
        task = {"action": "pick_place", "object": cands[0] if len(cands) == 1 else "this"}
        if len(cands) == 1:
            task["object_pos"] = _r(tasks.mug_center(cands[0]))
        else:
            task["candidates"] = cands
        if kind == "pick":
            task["hold"] = True
        else:
            task["target"] = _region(t)
        out.update(ok=True, object=cands[0] if len(cands) == 1 else None, candidates=cands, task=task,
                   message=(f"{_pretty(tasks, cands[0])[0].upper()}{_pretty(tasks, cands[0])[1:]}, the one you pointed at."
                            if len(cands) == 1 else "You're pointing at more than one thing."))
        st["last_obj"] = out["object"] or st["last_obj"]
        return out

    # place / pick_place: the spot is the latest pointing; "this" in "put this there" is the pointing before it
    segs = _segments(st, now)
    spot_s = next((s for s in reversed(segs) if s["hit"] is not None), None) if (cur is not None) else None
    if cur is not None and cur["hit"] is not None:
        spot_s = cur
    if spot_s is None:
        out["message"] = "I didn't see where you pointed. Point at the spot on the table for a moment, then say it again."
        return out
    obj, cands = None, []
    if kind == "pick_place":
        first = next((s for s in segs if s is not spot_s and s["candidates"]), None)
        cands = pointed_object(first) if first is not None else []
        if len(cands) == 1:
            obj = cands[0]
    if obj is None and not cands:
        obj = held or (words[0] if len(words) == 1 else None) or st["last_obj"] or getattr(tasks, "cur", None)
    if obj is None and not cands:
        out["message"] = "What should I put there? Point at it first, or name it."
        return out
    mid = tasks.add_marker(np.asarray(spot_s["hit"], float).copy())
    task = {"action": "pick_place", "object": obj or "this", "target": f"marker {mid}"}
    if cands and obj is None:
        task["candidates"] = cands
    elif obj is not None and not held and obj in getattr(tasks, "objs", {}):
        task["object_pos"] = _r(tasks.mug_center(obj))
    out.update(ok=True, object=obj, candidates=cands, spot=_r(spot_s["hit"]), marker=mid, task=task,
               message=f"Putting {_pretty(tasks, obj) if obj else 'it'} on the spot you pointed at (marker {mid}).")
    return out


# ---------------------------------------------------------------------- tasks: pick and hold, handover
@ext.pre_task
def copresence_tasks(ctx, msg):
    a = msg.get("action")
    if a == "pick_place" and msg.get("hold"):
        return _pick_hold(ctx, msg)
    if a == "handover" and not (msg.get("to") or msg.get("between_arms")):   # to the person (between a robot's two
        return _handover(ctx, msg)                                            # hands: tasks.handover_arms)
    return None


def _ask(ctx, msg, names, why):
    try:
        from . import candidates
        return candidates.ask(ctx, msg, names, why)
    except Exception:
        return None


def _pick_hold(ctx, msg):
    tasks, tid = ctx.tasks, str(msg.get("id", "cp-pick"))
    cands = [c for c in (msg.get("candidates") or []) if c in tasks.objs]
    if len(cands) > 1 and not msg.get("_chosen"):
        m = {k: v for k, v in msg.items() if k != "candidates"}
        return _ask(ctx, m, cands, "You're pointing at more than one thing. ") or True
    name = msg.get("object") if msg.get("object") in tasks.objs else tasks.obj_name(msg.get("object"))
    if name is None or name not in tasks.objs:
        tasks.set_task(tid, "pick_place", "failed", f"I don't see '{msg.get('object')}' to pick up.")
        return True
    palm, held = _held(tasks)
    if held == name:
        tasks.set_task(tid, "pick_place", "done", f"I'm already holding {_pretty(tasks, name)}.")
        return True
    if held:
        tasks.set_task(tid, "pick_place", "failed", f"I'm still holding {_pretty(tasks, held)}.")
        return True
    tasks.cur = name
    c = tasks.mug_center(name)
    palm = _pick_palm(ctx.rt, c[1])
    cur = ctx.rt.data.site_xpos[ctx.rt.palm_sid[palm]].copy()
    g_kps, quats, gi = tasks.grasp_waypoints(name, palm, cur)
    kps, ev = [cur] + g_kps, {gi: "grasp"}
    if getattr(tasks, "keepout", None):
        kps, ev, _ = tasks.keepout.route(kps, ev, palm)
    what = _pretty(tasks, name)
    tasks.done_msg = f"I'm holding {what}. Say 'put it there' while you point, or 'give it to me'."
    tasks._launch(tid, "pick_place", kps, palm, ev, "task", f"Picking up {what}, the one you pointed at.", quats=quats)
    _st(ctx)["last_obj"] = name
    return True


def _clamp_reach(ctx, palm, chain, hand):
    """Trace chain + [hand] with the reach precheck; -> (the farthest reachable point on the last leg, reached the
    hand?) or (None, False) when even the chain (e.g. the grasp) is out of reach."""
    rt, tasks = ctx.rt, ctx.tasks
    lo = TABLE_TOP + 0.10 + float(getattr(tasks, "mug_half_h", 0.05))
    goal = np.array([hand[0], hand[1], float(np.clip(hand[2], lo, 1.25))])
    pts = [np.asarray(chain[0], float)]
    for kp in list(chain[1:]) + [goal]:
        pts.extend(densify([pts[-1], np.asarray(kp, float)], 0.03)[1:])
    start = len(pts) - len(densify([chain[-1], goal], 0.03))
    ok = rt.precheck(np.array(pts), palm)
    if not all(ok[: start + 1]):
        return None, False
    j = start
    while j + 1 < len(pts) and ok[j + 1]:
        j += 1
    if j == start and len(chain) == 1:
        return None, False
    return pts[j], j == len(pts) - 1


def _handover(ctx, msg):
    tasks, rt, st = ctx.tasks, ctx.rt, _st(ctx)
    tid = str(msg.get("id", "cp-handover"))
    J = st["J"]
    cands = [c for c in (msg.get("candidates") or []) if c in tasks.objs]
    if len(cands) > 1 and not msg.get("_chosen"):
        m = {k: v for k, v in msg.items() if k != "candidates"}
        return _ask(ctx, m, cands, "Which one should I give you? ") or True
    palm, held = _held(tasks)
    name = held
    if name is None:
        o = msg.get("object")
        name = o if o in tasks.objs else (tasks.obj_name(o) if o and o != "this" else None)
    if name is None or name not in tasks.objs:
        tasks.set_task(tid, "handover", "failed", "I'm not holding anything. Point at something and say 'give me this'.")
        return True
    if held and held != name:
        tasks.set_task(tid, "handover", "failed", f"I'm holding {_pretty(tasks, held)}; I'll hand you that first.")
        return True
    hand = msg.get("hand")
    side = st["offer"]
    if hand is None and J is not None:
        hand = hand_pos(J, side) if side else None
    if hand is None:                 # no hand held out: in front of the person's chest, over the table edge
        ax, ay = st["anchor"]
        hand = [ax - 0.35, ay, TABLE_TOP + 0.25]
    hand = np.asarray(hand, float)
    if palm is None:
        c = tasks.mug_center(name)
        palm = _pick_palm(ctx.rt, c[1])
    cur = rt.data.site_xpos[rt.palm_sid[palm]].copy()
    kps, ev, quats = [cur], {}, None
    if held is None:
        tasks.cur = name
        g_kps, quats, gi = tasks.grasp_waypoints(name, palm, cur)
        kps += g_kps
        ev[gi] = "grasp"
    toward = np.array([cur[0] - hand[0], cur[1] - hand[1], 0.0])
    n = float(np.linalg.norm(toward))
    goal = hand + (toward / n) * HANDOVER_GAP if n > HANDOVER_GAP else hand
    target, exact = _clamp_reach(ctx, palm, kps, goal)
    if target is None:
        tasks.set_task(tid, "handover", "failed", "I can't reach toward your hand from here.")
        return True
    kps.append(target)
    if quats is not None:
        quats = list(quats) + [quats[-1]]
    what = _pretty(tasks, name)
    tasks.done_msg = (f"Here you go: take {what}. I'll let go when your hand is close." if exact else
                      f"That's as far as I reach: take {what} from my hand. I'll let go when your hand is close.")
    ok = tasks._launch(tid, "handover", kps, palm, ev, "task", f"Bringing {what} to your hand.", quats=quats)
    if ok:
        st["handover"] = {"phase": "carrying", "object": name, "palm": palm, "target": target, "hand": hand,
                          "side": side, "tid": tid, "t": time.time(), "exact": exact}
        st["given"] = None
    return True


def _handover_tick(ctx, st, J, now):
    tasks, rt = ctx.tasks, ctx.rt
    ho = st["handover"]
    if ho is not None:
        palm = ho["palm"]
        if ho["phase"] == "carrying":
            t = tasks.task or {}
            if t.get("id") != ho["tid"] or t.get("status") == "failed":
                st["handover"] = None              # stopped, superseded or failed
            elif t.get("status") == "done":
                ho["phase"], ho["t"] = "offering", now
        elif ho["phase"] == "offering":
            if tasks.holding.get(palm) != ho["object"]:
                st["handover"] = None
            elif now - ho["t"] > OFFER_TIMEOUT:
                st["handover"] = None
                _say(ctx, f"I'll keep holding {_pretty(tasks, ho['object'])} for now.")
            elif J is not None:
                # present it at the offered spot first, then let go once the person's hand has come to it and stays a
                # moment (a hand that was already near when the arm stopped, e.g. 10 cm in front of it for safety, is not
                # yet taking it): the person isn't simulated, so "it's in their hand" is their hand at the object
                op = tasks.mug_center(ho["object"]) if ho["object"] in tasks.objs else rt.data.site_xpos[rt.palm_sid[palm]]
                near = [(float(np.linalg.norm(hand_pos(J, sd) - op)), sd) for sd in ("l", "r") if f"{sd}_wrist" in J]
                near = min(near) if near else None
                if near is not None and ho.get("d0") is None:
                    ho["d0"] = near[0]
                came = near is not None and (near[0] < GRAB_DIST or (near[0] < RELEASE_DIST and ho["d0"] - near[0] > 0.04))
                ho["near_t"] = (ho.get("near_t") or now) if came else None
                if came and now - ho["t"] >= PRESENT_S and now - ho["near_t"] >= GRAB_HOLD_S:
                    obj = ho["object"]
                    tasks.holding[palm] = None
                    tasks.grasp_off = None
                    if getattr(tasks, "plant", None) is not None:     # physics: the gripper opens, the person takes it
                        tasks.held.pop(palm, None)
                        tasks.plant.grip_cmd(tasks.role(palm), "open")
                    st["given"] = {"object": obj, "side": near[1],
                                   "off": tasks.obj_pos(obj) - hand_pos(J, near[1]), "last": None}
                    ho["phase"], ho["t"] = "given", now
                    _say(ctx, "Here you go.")
                    if (tasks.task or {}).get("id") == ho["tid"]:
                        tasks.task["message"] = f"Handed {_pretty(tasks, obj)} to you."
        elif ho["phase"] == "given" and now - ho["t"] > 3.0:
            st["handover"] = None
    g = st["given"]
    if g is not None:
        obj = g["object"]
        o = tasks.objs.get(obj)
        mpos = None if o is None else tasks.obj_pos(obj)
        moved = g["last"] is not None and mpos is not None and float(np.linalg.norm(mpos - g["last"])) > (0.05 if getattr(tasks, "plant", None) is not None else 0.01)
        if o is None or obj in tasks.holding.values() or moved:
            st["given"] = None                    # the robot took it back, or the scene was reset
        elif J is not None and f"{g['side']}_wrist" in J:
            p = hand_pos(J, g["side"]) + 0.5 * g["off"]
            tasks.set_obj_pose(obj, p)               # the person holds it (not simulated): it follows the hand
            g["last"] = p.copy()


# ---------------------------------------------------------------------- messages
@ext.handler("copresence")
def on_copresence(ctx, msg):
    st, op = _st(ctx), msg.get("op")
    if op in ("on", "off"):
        st["on"] = op == "on"
        if not st["on"]:
            st["J"], st["point"], st["arm"] = None, None, None
    elif op == "view":
        st["view"] = bool(msg.get("on", True))
    elif op == "place":
        try:
            st["anchor"] = (float(np.clip(float(msg.get("x", PERSON[0])), 0.95, 1.6)), float(np.clip(float(msg.get("y", 0.0)), -0.6, 0.6)))
            st["anchor_auto"] = False
        except (TypeError, ValueError):
            pass
    elif op == "heard":
        st["spoke_t"] = time.time()
    elif op == "utter":
        st["spoke_t"] = time.time()
        r = resolve_deixis(ctx, msg.get("text", ""))
        r = dict(r, id=msg.get("id"), text=str(msg.get("text", ""))[:200])
        if msg.get("run") and r["task"]:
            m = dict(r["task"], type="task", id=str(msg.get("id") or f"cp-{int(time.time() * 1000) % 100000}"))
            if msg.get("preview"):
                m["preview"] = True
            if not ext.intercept_task(ctx, m):
                ctx.dispatch(m)
            r["ran"] = m["id"]
        st["reply"] = r
    elif op == "handover":
        _handover(ctx, dict(msg, action="handover", id=msg.get("id") or "cp-handover"))
    elif op == "demo":
        ph = msg.get("phase")
        if ph:                     # hold one pose for 30 s: point_mug | point_spot | offer | take | rest
            on_feed(ctx, {"frames": [demo_pose(ctx, ph)], "fps": 20, "hold": float(msg.get("hold") or 30.0)})
        else:
            on_feed(ctx, {"frames": demo_frames(ctx), "fps": 20, "hold": 0.5})


def demo_pose(ctx, phase):
    """One synthetic pose: pointing at the mug or at a free spot, an open hand held out, or reaching for the object
    the robot offers ("take")."""
    tasks, st = ctx.tasks, _st(ctx)
    if phase == "point_mug" and "mug" in getattr(tasks, "objs", {}):
        return synth_pose(point_at=tasks.mug_center("mug"))
    if phase == "point_spot":
        return synth_pose(point_at=[0.44, -0.30, TABLE_TOP])
    if phase == "offer":
        return synth_pose(offer=[0.75, -0.05, TABLE_TOP + 0.25])
    if phase == "take" and st["handover"] is not None:
        rt = ctx.rt
        return synth_pose(offer=rt.data.site_xpos[rt.palm_sid[st["handover"]["palm"]]].copy())
    return synth_pose()


def demo_frames(ctx, fps=20):
    """A canned person for headless demos (24 s): rest, point at the mug (1-7 s), point at a free spot (8-14 s), hold
    out an open hand (15-23 s)."""
    tasks = ctx.tasks
    mug = tasks.mug_center("mug") if "mug" in getattr(tasks, "objs", {}) else np.array([0.4, -0.15, 0.8])
    seq = [(1.0, synth_pose()), (6.0, synth_pose(point_at=mug)), (1.0, synth_pose()),
           (6.0, synth_pose(point_at=[0.44, -0.30, TABLE_TOP])), (1.0, synth_pose()),
           (8.0, synth_pose(offer=[0.75, -0.05, TABLE_TOP + 0.25])), (1.0, synth_pose())]
    out = []
    for sec, f in seq:
        out += [f] * int(sec * fps)
    return out


@ext.state
def copresence_state(ctx):
    st, now = _st(ctx), time.time()
    J, ray = st["J"], st["point"]
    pointing = None
    if ray is not None:
        pointing = {"side": ray["side"], "from": _r(ray["from"]), "dir": _r(ray["dir"]), "hit": _r(ray["hit"]),
                    "armed": bool(ray.get("armed")), "target": ray.get("target"), "candidates": ray.get("candidates", [])}
    ho = st["handover"]
    paths = {sd: [_r(p) for t, p in list(st["paths"][sd])[::3]] for sd in ("l", "r")}
    return {"copresence": {
        "on": st["on"], "view": st["view"], "source": st["src"], "anchor": list(st["anchor"]), "visible": _person_visible(st),
        "joints": None if J is None else {k: _r(v) for k, v in J.items()},
        "pointing": pointing, "recent_point": bool(_recent(st, now)) or bool(pointing and pointing["armed"]),
        "hand_open": st["hand_open"], "offer": st["offer"], "attention": st["attention"],
        "handover": None if ho is None else {"phase": ho["phase"], "object": ho["object"], "target": _r(ho["target"]),
                                             "hand": _r(ho["hand"])},
        "given": None if st["given"] is None else st["given"]["object"],
        "hand_paths": paths, "reply": st["reply"],
    }}


# ---------------------------------------------------------------------- robot-eye drawing
def _capsule(scn, a, b, width, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(a, float), np.asarray(b, float))
    scn.ngeom += 1


def _geom(scn, kind, size, pos, rgba, mat=None):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, kind, np.asarray(size, float), np.asarray(pos, float),
                        np.eye(3).flatten() if mat is None else np.asarray(mat, float).flatten(), np.array(rgba, np.float32))
    scn.ngeom += 1


def _ring(scn, c, r, rgba, width=0.004, n=20):
    pts = [np.array([c[0] + r * math.cos(2 * math.pi * k / n), c[1] + r * math.sin(2 * math.pi * k / n), c[2]]) for k in range(n + 1)]
    for a, b in zip(pts, pts[1:]):
        _capsule(scn, a, b, width, rgba)


def _person_visible(st):
    """Draw the person only when there is one: a real camera / phone, a played demo person, or the Shared space view.
    The synthetic no-camera dancer is not a person across the table."""
    return bool(st.get("view")) or st.get("src") in ("feed", "webcam", "phone")


@ext.overlay
def paint_copresence(ctx, scn):
    st = _st(ctx)
    J = st["J"]
    if J is None or not _person_visible(st):
        return
    for a, b, w in BONES:
        if a in J and b in J:
            _capsule(scn, J[a], J[b], w, GREY)
    for k in ("l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_knee", "r_knee"):
        if k in J:
            _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.05 if "shoulder" in k else 0.045, 0, 0], J[k], GREY)
    for sd in ("l", "r"):
        if f"{sd}_wrist" in J:
            _capsule(scn, J[f"{sd}_wrist"], hand_pos(J, sd) + 0.4 * (hand_pos(J, sd) - J[f"{sd}_wrist"]), 0.032, GREY)
        if f"{sd}_ankle" in J:
            a = J[f"{sd}_ankle"]
            _capsule(scn, a, a + np.array([-0.12, 0, -0.03]), 0.04, GREY)
    _capsule(scn, J["neck"], J["head"] + np.array([0, 0, -0.06]), 0.05, GREY)
    _geom(scn, mujoco.mjtGeom.mjGEOM_ELLIPSOID, [0.085, 0.08, 0.11], J["head"], GREY)
    ray = st["point"]
    if ray is not None:
        armed = bool(ray.get("armed"))
        col = ORANGE if armed else (ORANGE[0], ORANGE[1], ORANGE[2], 0.35)
        end = ray["hit"] if ray["hit"] is not None else ray["from"] + ray["dir"] * 0.6
        _capsule(scn, ray["from"], end, 0.005 if armed else 0.003, col)
        if ray["hit"] is not None:
            h = ray["hit"] + np.array([0, 0, 0.003])
            _ring(scn, h, 0.035, col, 0.004)
            if armed:
                _geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.012, 0.0015, 0], h, col)
    att = st["attention"]
    if att and att["kind"] == "object" and att.get("name") in getattr(ctx.tasks, "objs", {}):
        o = ctx.tasks.objs[att["name"]]
        c = ctx.tasks.mug_center(att["name"])
        _ring(scn, [c[0], c[1], TABLE_TOP + 0.004], float(o.get("r", 0.04)) + 0.03, INDIGO, 0.004, 28)
