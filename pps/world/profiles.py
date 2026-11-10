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


PROFILES: dict[str, RobotProfile] = {p.id: p for p in [
    RobotProfile(
        "g1", "Unitree G1 humanoid", "humanoid", "assets/unitree_g1/g1.xml",
        {"x": 0.0, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "right_palm", "left": "left_palm"}, "stand", "legged",
        HUMANOID_CAPS, home_xml="assets/unitree_g1/scene_pc.xml",
        feet=(("right_foot", "site"), ("left_foot", "site")), root="pelvis", torso="torso_link", grasp="side",
        prompt="pps/world/prompts/g1.md"),
    RobotProfile(
        "panda", "Franka Emika Panda arm", "arm", EX + "franka_emika_panda/mjx_panda.xml",
        {"x": 0.02, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "gripper"}, "home", "fixed",
        ARM_CAPS, freeze=("finger",), prompt="pps/world/prompts/panda.md"),
    RobotProfile(
        "ur5e", "Universal Robots UR5e arm", "arm", EX + "universal_robots_ur5e/ur5e.xml",
        {"x": 0.02, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "attachment_site"}, "home", "fixed",
        ARM_CAPS, prompt="pps/world/prompts/ur5e.md"),
    RobotProfile(
        "xarm7", "uFactory xArm7", "arm", EX + "ufactory_xarm7/xarm7.xml",
        {"x": 0.02, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "link_tcp"}, "home", "fixed",
        ARM_CAPS, freeze=("driver", "finger", "knuckle"), prompt="pps/world/prompts/xarm7.md"),
    RobotProfile(
        "iiwa14", "KUKA LBR iiwa 14", "arm", EX + "kuka_iiwa_14/iiwa14.xml",
        {"x": -0.10, "y": 0.0, "yaw": 0, "on": "surface", "pedestal": True}, {"right": "attachment_site"}, "home", "fixed",
        ARM_CAPS, prompt="pps/world/prompts/iiwa14.md"),
    RobotProfile(
        "aloha", "ALOHA two-arm rig", "bimanual", EX + "aloha/aloha.xml",
        {"x": 0.40, "y": 0.0, "yaw": -90, "on": "surface"}, {"right": "right/gripper", "left": "left/gripper"},
        "neutral_pose", "fixed", ("pick_place", "reach", "follow_path", "bimanual"), ee_axis=(1, 0, 0),
        freeze=("finger",), prompt="pps/world/prompts/aloha.md"),
    RobotProfile(
        "apollo", "Apptronik Apollo humanoid", "humanoid", EX + "apptronik_apollo/apollo.xml",
        {"x": 0.0, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "right_palm", "left": "left_palm"}, "stand", "legged",
        ("pick_place", "reach", "follow_path", "balance"), feet=(("right_foot", "site"), ("left_foot", "site")),
        root="base_link", grasp="side", ee_down=False, hand_cost=(5.0, 0.05), freeze=("neck",), camera={"pos": [0.24, 0.0, 1.80]},
        note="Whole-body IK with balance. Following you, video imitation and teach-by-showing are tuned for the G1 only.",
        prompt="pps/world/prompts/apollo.md"),
    RobotProfile(
        "stretch3", "Hello Robot Stretch 3", "mobile_manipulator", EX + "hello_robot_stretch_3/stretch_33.xml",
        {"x": -0.22, "y": 0.0, "yaw": 90, "on": "floor"}, {"right": "link_grasp_center"}, "home", "mobile",
        ("pick_place", "reach", "follow_path", "drive_base"), planar_base=("x",), base_joints=("base_x",), base_cost=0.5,
        ee_down=False, hand_cost=(4.0, 0.0), freeze=("wheel", "head", "rubber", "gripper"), prompt="pps/world/prompts/stretch3.md"),
    RobotProfile(
        "tidybot", "Stanford TidyBot (Kinova on a mobile base)", "mobile_manipulator", EX + "stanford_tidybot/tidybot.xml",
        {"x": -0.30, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "pinch_site"}, "home", "mobile",
        ("pick_place", "reach", "follow_path", "drive_base"), base_joints=("joint_x", "joint_y", "joint_th"), base_cost=1.0,
        freeze=("driver", "coupler", "spring", "follower"), prompt="pps/world/prompts/tidybot.md"),
    RobotProfile(
        "spot", "Boston Dynamics Spot with arm", "quadruped_arm", EX + "boston_dynamics_spot/spot_arm.xml",
        {"x": -0.35, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "EE"}, "home", "legged",
        ("pick_place", "reach", "follow_path", "balance"),
        feet=(("FL", "site"), ("FR", "site"), ("HL", "site"), ("HR", "site")), root="body", ee_axis=(1, 0, 0),
        freeze=("f1x",), prompt="pps/world/prompts/spot.md"),
    RobotProfile(
        "h2", "Unitree H2 humanoid (experimental)", "humanoid", EX + "unitree_h2/h2_sharpa.xml",
        {"x": -0.05, "y": 0.0, "yaw": 0, "on": "floor"}, {"right": "right_wrist_site", "left": "left_wrist_site"}, None,
        "legged", ("reach", "follow_path", "balance"),
        feet=(("left_ankle_roll_link", "body"), ("right_ankle_roll_link", "body")), root="pelvis", grasp="side",
        ee_down=False, hand_cost=(5.0, 0.0), camera={"pos": [0.16, 0.0, 1.86]},
        freeze=("thumb", "index", "middle", "ring", "pinky", "head"), experimental=True,
        note="The model ships without a standing keyframe and its dexterous hands are not driven: it starts from the "
             "zero pose, reaches with whole-body IK and keeps its feet planted, but pick & place is not supported.",
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


def listing():
    return [p.summary() for p in PROFILES.values()]
