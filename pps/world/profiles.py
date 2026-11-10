"""Robot profiles: every body PhysicsClay can drive (docs/WORLD.md §1).

World frame = robot frame: x forward (towards the work surface), y left, z up. The work surface's near edge is at
x = 0.22 for every template, so a body is placed by its `mount`:
  on="floor"    the robot stands/rolls on the floor at (x, y)            (humanoids, mobile bases, Spot)
  on="surface"  the robot is bolted at (x, y) at the surface height,     (fixed arms; pedestal=True draws the stand)
yaw rotates the body about z (degrees).  End-effector sites are keyed by role; a single arm uses "right" so the task
code (pick_place / reach / follow_path / markers / zones / plan preview) works unchanged.
"""
from dataclasses import asdict, dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EX = "vendor/mink/examples/"

HUMANOID_CAPS = ("pick_place", "reach", "follow_path", "imitate_video", "follow_me", "teach_by_showing", "balance")
ARM_CAPS = ("pick_place", "reach", "follow_path")


@dataclass
class RobotProfile:
    id: str
    label: str
    kind: str                  # humanoid | arm | bimanual | mobile_manipulator | quadruped_arm
    model_xml: str             # relative to the repo root
    mount: dict                # {"x", "y", "yaw" (deg), "on": "floor"|"surface", "pedestal": bool}
    ee: dict                   # role -> site, e.g. {"right": "gripper"}
    home_key: str | None       # keyframe in model_xml (or home_xml) used as the start pose
    base: str                  # legged | fixed | mobile
    capabilities: tuple
    camera: dict | None = None  # robot_view camera override {"pos": [..], "xyaxes": [..], "fovy": deg}
    prompt: str = ""           # pps/world/prompts/<id>.md (owned by the Robot UX workstream)
    experimental: bool = False
    note: str = ""
    # --- runtime details (not part of the public state)
    home_xml: str | None = None        # take the home keyframe from this file instead (G1: scene_pc.xml)
    stand: dict | None = None          # legged model without a home keyframe: compute one (pps/world/stand.py): {"drop": m}
    feet: tuple = ()                   # legged: foot frames, (name, "site"|"geom"|"body")
    root: str | None = None            # legged / free base: the floating body (orientation task)
    torso: str | None = None           # humanoid torso body (orientation task)
    base_joints: tuple = ()            # mobile: base joints the IK may move
    base_cost: float = 0.0             # mobile: damping on base joints (higher = arm first, then drive)
    planar_base: tuple = ()            # replace the model's free joint by these joints: ("x"|"y"|"yaw", ...)
    hand_cost: tuple = (4.0, 0.3)      # FrameTask (position_cost, orientation_cost) for the end effectors
    ee_down: bool = True               # aim the end-effector's approach axis down (top grasps)
    ee_axis: tuple = (0, 0, 1)         # the end-effector site's approach axis (local frame)
    grasp: str = "top"                 # default pick_place grasp: "top" | "side"
    freeze: tuple = ()                 # joint name substrings the IK must not move (fingers, head, wheels)
    reach_tol: float = 0.06
    safety: dict = field(default_factory=dict)   # P5 overrides of pps.safety.Config (server config; clamped to floors)
    tool: dict | None = None           # a tool on the end effector (set by the scene: demo packs mount a welding torch;
                                       # ee then names the tool's tip site, the TCP)
    # --- physics (pps/physics.py, docs/V1.md P0)
    physics_base: str = "fixed"        # legged bodies: "fixed" = the base is welded (a harness), legs held by their servos
    physics: dict = field(default_factory=dict)   # {"grip": {role: gripper}, "payload": kg, "grasp": "top"|"side"}

    def public(self):
        d = {"id": self.id, "label": self.label, "kind": self.kind, "base": self.base, "ee": list(self.ee),
             "capabilities": list(self.capabilities)}
        if self.tool:
            d["tool"] = {k: self.tool[k] for k in ("kind", "label", "tcp") if k in self.tool}
        if self.experimental:
            d["experimental"] = True
        if self.note:
            d["note"] = self.note
        return d

    def summary(self):
        d = {"id": self.id, "label": self.label, "kind": self.kind, "capabilities": list(self.capabilities)}
        if self.experimental:
            d["experimental"] = True
        return d

    def has(self, cap):
        return cap in self.capabilities

    def asdict(self):
        return asdict(self)


LEGGED_NOTE = ("Physics: manipulation runs with the base welded in place and the legs held by their servos; dynamic walking "
               "needs a learned locomotion policy and is out of scope.")


# bodies whose contact pick & place isn't reliable yet with physics on are listed as experimental (physics only), with
# why: physics={"experimental": "<reason>"}. None today: xArm7, iiwa14 and Apollo left the list in v1 P0b; Stretch 3,
# TidyBot and Spot in v1 P0c; the H2 (a computed stand, palm grippers) in v1's final pass (>= 9/10 on both tasks in
# scripts/bench_grasp.py).


def _built(width, approach=(0, 0, 1), axis=(0, 1, 0), length=0.06, base=0.0):
    """A simple parallel-jaw gripper built at the end-effector site (pps.physics.add_grippers)."""
    return {"build": True, "width": width, "approach": approach, "axis": axis, "len": length, "base": base}


def _native(act, open_, close, a, b, width, tcp=(0, 0, 0), approach=(0, 0, 1), axis=(0, 1, 0), boost=1.0, pads=None,
            nocollide=(), nocollide_bodies=(), jaw=None, up=None, pad_h=None, parallel=None):
    """The model's own gripper: actuator, open/closed commands, the bodies of each side's pads, TCP (site frame).
    pads: {"a": [{"body", "pos", "size", "quat"}], "b": [...]} flat pads added to the finger bodies (their frames);
    nocollide: geom names on the pad bodies whose contact is replaced by those pads; nocollide_bodies: bodies whose geoms
    don't touch things (hulls that don't match the real finger); parallel: spring-loaded parallel pads driven by the
    gripper's joint (pps.physics._parallel_pads)."""
    return {"act": act, "open": open_, "close": close, "a": a, "b": b, "width": width, "tcp": tcp, "approach": approach,
            "axis": axis, "boost": boost, "pads": pads or {}, "nocollide": tuple(nocollide), "nocollide_bodies": tuple(nocollide_bodies),
            "jaw": jaw, **({"up": up} if up is not None else {}), **({"pad_h": pad_h} if pad_h is not None else {}),
            **({"parallel": parallel} if parallel is not None else {})}


def _sg3_grip():
    """Hello Robot's SG3 (Stretch 3). The real fingers are flexible strips with suction-cup-like rubber tips that swivel
    flat against what they hold; the model's rigid hinged fingers close the tips along an arc, tilted up to 30°, so a
    squeezed block or mug squirted out. Physics models the tips as two parallel, spring-loaded pads: each pad's carrier
    follows where the real fingertip's inner face is across the jaws for the gripper's slide joint s (a cubic fitted to
    the model's own finger kinematics: 1.5 cm at s = 0, 8.7 cm open), the pad sits on a slide joint behind it with a spring
    (the finger's flex: 5 kN/m, damped, up to 1.2 cm) and the force comes from the gripper's own force-limited servo
    through that coupling (boost: 2 x dx/ds, so each pad squeezes with about grip_force). The finger meshes rotate as
    before (they're the visuals); the rubber tips' hulls are replaced by the pads (4 x 4 cm faces)."""
    return dict(_native("gripper", 0.04, -0.02, ("pc_right_pad_a",), ("pc_right_pad_b",), 0.15, approach=(0, 0, 1), axis=(1, 0, 0),
                        boost=3.7, up=(0, 1, 0), pad_h=0.02, nocollide_bodies=("rubber_tip_left", "rubber_tip_right"),
                        parallel={"joint": "joint_gripper_slide", "x": (0.01502, 1.8295, 0.3927, -30.31), "z": 0.0,
                                  "half": (0.02, 0.02), "t": 0.008, "k": 5000.0, "c": 20.0, "flex": 0.012}),
                room=0.04)   # pad (0.8 cm) + the finger strip outside it (~1.5 cm) + 1.5 cm to open: a spot needs that beside it


def _xarm_grip():
    """uFactory xArm gripper: the model collides with each finger's convex hull, a single contact point on a thin handle
    (a spoon lying in a drawer), about which a held long thing pivots and creeps out of the jaws. The real finger has a
    flat rubber tip (3.2 x 0.8 cm, from the finger mesh: its inner face at 2.6 cm from the finger's axis, the last 8 mm);
    that tip is the pad (box: a line contact across a handle, which holds it square), the hulls don't touch things."""
    pads = {"a": [{"body": "left_finger", "pos": (0.0, -0.0235, 0.057), "size": (0.015, 0.0025, 0.0045)}],
            "b": [{"body": "right_finger", "pos": (0.0, 0.0235, 0.057), "size": (0.015, 0.0025, 0.0045)}]}
    return dict(_native("gripper", 0, 255, ("left_finger",), ("right_finger",), 0.085, tcp=(0, 0, -0.03), boost=0.62, pads=pads),
                nocollide_mesh=True)


def _aloha_grip(r):
    """ViperX 300 fingers (ALOHA): the model's finger capsule sits ~1 cm inside the finger mesh near the tips, so a flat
    3.6 x 1.4 cm pad on each fingertip's inner face (from the mesh) does the grasping; open = 0.041 (6.8 cm at the pads)."""
    fl, fr = f"{r}/left_finger_link", f"{r}/right_finger_link"
    pads = {"a": [{"body": fl, "pos": (0.01721, -0.07182, 0.0221), "quat": (0.5004, 0.5, -0.4996, -0.5), "size": (0.018, 0.004, 0.007)}],
            "b": [{"body": fr, "pos": (0.01726, 0.07182, 0.0221), "quat": (-0.5, 0.5004, 0.5, -0.4996), "size": (0.018, 0.004, 0.007)}]}
    hulls = [f"{r}_left_finger_coll", f"{r}_right_finger_coll"] + [f"{r}/{s}_g{i}" for s in ("left", "right") for i in range(3)]
    return _native(f"{r}/gripper", 0.041, 0.002, (fl,), (fr,), 0.068, tcp=(0.005, 0, 0), approach=(1, 0, 0), boost=1.0,
                   pads=pads, nocollide=hulls)


PROFILES: dict[str, RobotProfile] = {p.id: p for p in [
    RobotProfile(
        "g1", "Unitree G1 humanoid", "humanoid", "assets/unitree_g1/g1.xml",
        {"x": 0.0, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "right_palm", "left": "left_palm"}, "stand", "legged",
        HUMANOID_CAPS, home_xml="assets/unitree_g1/scene_pc.xml",
        feet=(("right_foot", "site"), ("left_foot", "site")), root="pelvis", torso="torso_link", grasp="side",
        physics={"payload": 2.0, "grasp": "side", "home": {"right_shoulder_roll_joint": -0.35, "left_shoulder_roll_joint": 0.35},
                 "grip": {r: _built(0.15, approach=(1, 0, 0), length=0.09, base=0.07) for r in ("right", "left")}},
        prompt="pps/world/prompts/g1.md"),
    RobotProfile(
        "panda", "Franka Emika Panda arm", "arm", EX + "franka_emika_panda/mjx_panda.xml",
        {"x": 0.02, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "gripper"}, "home", "fixed",
        ARM_CAPS, freeze=("finger",), prompt="pps/world/prompts/panda.md",
        physics={"payload": 3.0, "grip": {"right": _native("actuator8", 0.04, 0.0, ("left_finger",), ("right_finger",), 0.08, boost=2.0)}}),
    RobotProfile(
        "ur5e", "Universal Robots UR5e arm", "arm", EX + "universal_robots_ur5e/ur5e.xml",
        {"x": 0.02, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "attachment_site"}, "home", "fixed",
        ARM_CAPS, prompt="pps/world/prompts/ur5e.md", physics={"payload": 5.0, "grip": {"right": _built(0.14)}}),
    RobotProfile(
        "xarm7", "uFactory xArm7", "arm", EX + "ufactory_xarm7/xarm7.xml",
        {"x": 0.02, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "link_tcp"}, "home", "fixed",
        ARM_CAPS, freeze=("driver", "finger", "knuckle"), prompt="pps/world/prompts/xarm7.md",
        physics={"payload": 3.5, "grip": {"right": _xarm_grip()}}),
    RobotProfile(
        "iiwa14", "KUKA LBR iiwa 14", "arm", EX + "kuka_iiwa_14/iiwa14.xml",
        {"x": -0.10, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "attachment_site"}, "home", "fixed",
        ARM_CAPS, prompt="pps/world/prompts/iiwa14.md", physics={"payload": 14.0, "grip": {"right": _built(0.14)}}),
    RobotProfile(
        "aloha", "ALOHA two-arm rig", "bimanual", EX + "aloha/aloha.xml",
        {"x": 0.40, "y": 0.0, "yaw": -90, "on": "surface"}, {"right": "right/gripper", "left": "left/gripper"},
        "neutral_pose", "fixed", ("pick_place", "reach", "follow_path", "bimanual"), ee_axis=(1, 0, 0),
        freeze=("finger",), prompt="pps/world/prompts/aloha.md",
        physics={"payload": 0.75, "grip": {r: _aloha_grip(r) for r in ("right", "left")}}),
    RobotProfile(
        "apollo", "Apptronik Apollo humanoid", "humanoid", EX + "apptronik_apollo/apollo.xml",
        {"x": 0.0, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "right_palm", "left": "left_palm"}, "stand", "legged",
        ("pick_place", "reach", "follow_path", "balance"), feet=(("right_foot", "site"), ("left_foot", "site")),
        root="base_link", grasp="side", ee_down=False, hand_cost=(5.0, 0.05), freeze=("neck",), camera={"pos": [0.24, 0.0, 1.80]},
        note="Whole-body IK with balance. Following you, video imitation and teach-by-showing are tuned for the G1 only.",
        physics={"payload": 10.0, "grasp": "side",
                 "exclude": [("base_link", "torso_link"), ("torso_link", "r_shoulder_fe_link"), ("torso_link", "l_shoulder_fe_link")],
                 "home": {"r_shoulder_aa": -0.35, "l_shoulder_aa": 0.35},      # grippers clear of the hips
                 # stiffer arm / torso servos than the model's (its hand lagged 6-9 cm behind the plan and swept into things)
                 "stiffer": [(("torso", "shoulder", "elbow"), 3.0), (("wrist",), 5.0)],
                 "grip": {r: dict(_built(0.15, approach=(0, 0, -1), length=0.09, base=0.04), hide_hand=True) for r in ("right", "left")}},
        prompt="pps/world/prompts/apollo.md"),
    RobotProfile(
        "stretch3", "Hello Robot Stretch 3", "mobile_manipulator", EX + "hello_robot_stretch_3/stretch_33.xml",
        {"x": -0.22, "y": 0.0, "yaw": 90, "on": "floor"}, {"right": "link_grasp_center"}, "home", "mobile",
        ("pick_place", "reach", "follow_path", "drive_base"), planar_base=("x",), base_joints=("base_x",), base_cost=0.5,
        ee_down=False, hand_cost=(4.0, 0.0), freeze=("wheel", "head", "rubber", "gripper"), prompt="pps/world/prompts/stretch3.md",
        physics={"payload": 1.5, "grasp": "side",
                 "stiffer": [(("wrist",), 10.0)],         # the model's wrist servos (kp 10-50) let the fingers droop onto the table
                 "home": {"joint_lift": 0.75},             # the model's home has the fingers 2 cm inside the table top
                 "grip": {"right": _sg3_grip()}}),
    RobotProfile(
        "tidybot", "Stanford TidyBot (Kinova on a mobile base)", "mobile_manipulator", EX + "stanford_tidybot/tidybot.xml",
        {"x": -0.30, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "pinch_site"}, "home", "mobile",
        ("pick_place", "reach", "follow_path", "drive_base"), base_joints=("joint_x", "joint_y", "joint_th"), base_cost=1.0,
        freeze=("driver", "coupler", "spring", "follower"), prompt="pps/world/prompts/tidybot.md",
        # transit_over: on the way in the hand goes over the other objects (the Kinova trails its plan by 3-5 cm in transit
        # and cut the corner into the mug next to a block)
        physics={"payload": 4.0, "transit_over": True,
                 "grip": {"right": _native("fingers_actuator", 0, 255, ("left_pad", "left_follower"), ("right_pad", "right_follower"), 0.085, axis=(1, 0, 0), boost=0.115)}}),
    RobotProfile(
        "spot", "Boston Dynamics Spot with arm", "quadruped_arm", EX + "boston_dynamics_spot/spot_arm.xml",
        {"x": -0.35, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "EE"}, "home", "legged",
        ("pick_place", "reach", "follow_path", "balance"),
        feet=(("FL", "site"), ("FR", "site"), ("HL", "site"), ("HR", "site")), root="body", ee_axis=(1, 0, 0),
        freeze=("f1x",), prompt="pps/world/prompts/spot.md",
        physics={"payload": 5.0, "grip": {"right": dict(
            _native("arm_f1x", -0.9, 0.0, ("arm_link_fngr",), ("arm_link_wr1",), 0.10, tcp=(0.03, 0, -0.02), approach=(1, 0, 0),
                    axis=(0, 0, 1), boost=0.06, jaw=-0.013),
            wide_open=-1.25, deep=0.05)}}),   # the mug: the finger comes down outside its far wall, holds 5 cm down
    RobotProfile(
        "h2", "Unitree H2 humanoid", "humanoid", EX + "unitree_h2/h2_sharpa.xml",
        {"x": -0.05, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "right_wrist_site", "left": "left_wrist_site"}, "stand",
        "legged", ("pick_place", "reach", "follow_path", "balance"),
        stand={"drop": 0.05},       # the model has no keyframe: a stand computed from it (pps/world/stand.py), knees ~0.69 rad
        feet=(("left_ankle_roll_link", "body"), ("right_ankle_roll_link", "body")), root="pelvis", grasp="side",
        ee_down=False, hand_cost=(5.0, 0.05), camera={"pos": [0.16, 0.0, 1.86]},
        freeze=("thumb", "index", "middle", "ring", "pinky", "head"),
        physics={"payload": 3.0, "grasp": "side",
                 # the model collides with its hands and feet only: boxes around the body's and arms' meshes (add_hulls)
                 "hulls": ("pelvis", "torso", "hip_yaw", "shoulder", "elbow", "wrist_roll", "wrist_pitch"),
                 # its joints have no actuators in the model; the servos physics adds are the G1-sized defaults (kp 150,
                 # 30 N m): 3x stiffer for its heavier arms (the hand trailed the plan by 1.5 cm setting the mug down; the
                 # effort caps stay, well inside the model's 60-130 N m shoulder / elbow limits)
                 "stiffer": [(("shoulder", "elbow", "waist"), 3.0), (("wrist",), 3.0)],
                 # arms down beside the body, grippers 9 cm out from the hips (the model's own ready pose holds them 20 cm
                 # over the work surface at chest height: into a bench-top machine's enclosure)
                 "home": {"right_shoulder_pitch_joint": 0.3, "right_shoulder_roll_joint": -0.3, "right_elbow_joint": 0.8,
                          "left_shoulder_pitch_joint": 0.3, "left_shoulder_roll_joint": 0.3, "left_elbow_joint": 0.8},
                 # the Sharpa hands' 22 joints aren't driven: a two-pad gripper on each palm, jaws across the palm (thumb
                 # up), the fingers neither collide nor are drawn
                 "grip": {r: dict(_built(0.15, approach=(0, 0, 1), axis=(1, 0, 0), length=0.09, base=0.06), hide_hand=True,
                                  up=(-1, 0, 0), hide_fingers=("thumb", "index", "middle", "ring", "pinky")) for r in ("right", "left")}},
        note="Whole-body IK with balance; the model ships without a standing pose, so one is computed from it (knees slightly "
             "bent). Its dexterous hands aren't driven: with physics a two-pad gripper on each palm does the grasping.",
        prompt="pps/world/prompts/h2.md"),
]}

DEFAULT_ROBOT = "g1"
ALIASES = {"unitree_g1": "g1", "franka": "panda", "franka_panda": "panda", "ur5": "ur5e", "xarm": "xarm7",
           "kuka": "iiwa14", "iiwa": "iiwa14", "stretch": "stretch3", "stretch_3": "stretch3", "tidy_bot": "tidybot",
           "spot_arm": "spot", "unitree_h2": "h2", "apptronik_apollo": "apollo"}


def get(rid):
    k = str(rid or DEFAULT_ROBOT).strip().lower().replace("-", "_").replace(" ", "_")
    k = ALIASES.get(k, k)
    return PROFILES.get(k)


def listing(physics=False):
    """Robot cards. With physics, a body whose contact grasp isn't reliable yet is listed as experimental (WORLD.md rule)."""
    out = []
    for p in PROFILES.values():
        s = p.summary()
        if physics and (p.physics or {}).get("experimental"):
            s["experimental"] = True
        out.append(s)
    return out


def physics_public(p, physics):
    """state.world.robot with the physics notes: legged bases are welded; grippers that don't grasp reliably yet."""
    d = p.public()
    if physics and p.base == "legged":
        d["note"] = (d.get("note", "") + " " + LEGGED_NOTE).strip()
    if physics and (p.physics or {}).get("experimental"):
        d["experimental"] = True
        d["note"] = (d.get("note", "") + " Physics: " + p.physics["experimental"]).strip()
    return d
