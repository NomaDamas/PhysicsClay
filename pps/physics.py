"""P0 physics (docs/V1.md): the robot is treated as a real robot.

IK (mink) stays the *planner*: it solves on its own copy of the model (objects as mocap, no dynamics) and gives the
desired joint positions q_des. The *plant* is an MjData stepped with mj_step at 2 ms, `n_sub` substeps per server
tick, that tracks q_des with position actuators (the model's own, or ones added here for joints that have none).
Objects are free bodies with mass, inertia and friction; grippers are finger joints driven by actuators, and a grasp
holds only by contact. Legged bodies are welded at the pelvis / body (a harness that follows the planned base pose)
with the legs held by their actuators: dynamic walking needs a learned policy and is out of scope.

`PC_PHYSICS=0` (or `python -m pps.app --kinematic`) keeps the old kinematic mode: IK poses the robot and a picked
object rides the hand (mocap). Robot joints come first in qpos (objects are attached after the robot), so the
planner's q is a prefix of the plant's qpos.
"""
import math
import os
import time
from collections import deque

import mujoco
import numpy as np

DT = 0.002
LEGACY_KINDS = {"mug": "mug", "red_block": "block", "green_block": "block"}     # the UX 2.0 scene's movable bodies
DEFAULTS = {"friction_scale": 1.0, "mass_scale": 1.0, "damping_scale": 1.0, "kp_scale": 1.0, "latency_ms": 0.0,
            "gravity": -9.81, "grip_force": 40.0, "impedance": False, "force_limit": 20.0}
LIMITS = {"friction_scale": (0.05, 5.0), "mass_scale": (0.05, 50.0), "damping_scale": (0.0, 10.0), "kp_scale": (0.05, 5.0),
          "latency_ms": (0.0, 500.0), "gravity": (-30.0, 0.0), "grip_force": (0.0, 200.0), "force_limit": (1.0, 200.0)}
# kind -> (mass kg at the default size, sliding friction)
OBJ = {"mug": (0.35, 0.9), "block": (0.08, 0.8), "box": (0.25, 0.7), "bowl": (0.30, 0.9), "cup": (0.12, 0.9),
       "can": (0.35, 0.8), "bottle": (0.45, 0.8), "plate": (0.35, 0.8), "fruit": (0.18, 0.9), "vase": (0.60, 0.9)}
MUG = {"R": 0.043, "t": 0.008, "z0": -0.068, "z1": 0.072, "cy": 0.004}   # the scanned mug: outer radius, wall, bottom/top, centre y
PAD_FRICTION = (1.2, 0.02, 0.0001)
# finger pads: stiff contact (2 timesteps) that wins the mix against the object's softer default (solmix), so a held object
# neither creeps nor turns in the jaws; with impratio 1 (a larger ratio makes light objects wobble in the grasp at 2 ms)
PAD_CONTACT = {"condim": 4, "solref": [0.004, 1.0], "solimp": [0.95, 0.99, 0.001, 0.5, 2.0], "solmix": 100.0}
SURFACE_FRICTION = (0.6, 0.005, 0.0001)
# position servos added for joints without an actuator: name pattern -> (kp, kv, effort)
ADDED_GAINS = [(("hip", "knee", "ankle"), (400.0, 20.0, 120.0)), (("waist", "torso"), (300.0, 15.0, 88.0)),
               (("wrist",), (40.0, 2.0, 8.0)), (("base",), (4000.0, 600.0, 400.0)), ((), (150.0, 8.0, 30.0))]
IMPEDANCE = {"kp": 0.2, "effort": 0.4}          # arm stiffness and effort cap in impedance mode (fractions)
ARMATURE_MIN = 0.02                             # kg m^2: least reflected rotor inertia of a servoed robot hinge
V_MAX = 2.0                                     # rad/s (m/s for slides): q_des is rate-limited, as in a real controller
A_MAX = 10.0                                    # rad/s^2 (m/s^2 for slides): ... and acceleration-limited
SETPOINT_TAU = 0.03                             # s: how fast the setpoint closes a gap to the plan (on top of its velocity)


def velocity_limit(ik_model, robot_bodies, frozen=()):
    """mink limit: the planner moves joints no faster than the plant's servos can follow (V_MAX). Fingers excluded."""
    import mink
    v = {}
    for j in range(ik_model.njnt):
        name = mujoco.mj_id2name(ik_model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if not name or name.startswith("pc_") or any(k in name for k in frozen):
            continue
        if ik_model.jnt_bodyid[j] in robot_bodies and int(ik_model.jnt_type[j]) in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            v[name] = V_MAX * (0.5 if ik_model.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE else 1.0)
    return mink.VelocityLimit(ik_model, v)


def enabled():
    return os.environ.get("PC_PHYSICS", "1").strip().lower() not in ("0", "false", "off", "no")


def _yaw_of(mat):
    R = np.asarray(mat).reshape(3, 3)
    return math.atan2(R[1, 0], R[0, 0])


def _rot_to_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).flatten())
    return q


# ------------------------------------------------------------------ building the robot side (planner and plant)
def joint_layout(spec):
    """[(name, type)] in qpos order: keyframes written for this layout are remapped by name after joints are added."""
    return [(j.name, j.type) for j in spec.joints]


def remap_q(layout, q_old, model):
    """A qpos written for `layout` -> the same pose in `model` (joints added since keep their qpos0)."""
    q, i = model.qpos0.copy(), 0
    for k, (name, typ) in enumerate(layout):
        n = {mujoco.mjtJoint.mjJNT_FREE: 7, mujoco.mjtJoint.mjJNT_BALL: 4}.get(typ, 1)
        try:
            j = model.joint(name).id if name else k
        except KeyError:
            i += n
            continue
        a = model.jnt_qposadr[j]
        q[a:a + n] = q_old[i:i + n]
        i += n
    return q


def home_overrides(profile, model, q):
    """Physics-only changes to the start pose (e.g. G1 arms a little out so the palm grippers clear the hips)."""
    q = np.array(q, float).copy()
    for name, v in ((profile.physics or {}).get("home") or {}).items():
        try:
            q[model.jnt_qposadr[model.joint(name).id]] = v
        except KeyError:
            pass
    return q


def robot_spec_bodies(spec):
    """Spec bodies that belong to the robot (everything but the world, objects, fixtures and IK target ghosts)."""
    out = []
    for b in spec.bodies:
        n = b.name or ""
        if b is spec.worldbody or n.startswith(("obj_", "obs_")) or n.endswith("_target") or n.startswith("pc_harness"):
            continue
        if n in LEGACY_KINDS or n in ("vase", "blue_block"):
            continue
        root = b
        while root.parent is not None and root.parent is not spec.worldbody:
            root = root.parent
        rn = root.name or ""
        if rn.startswith(("obj_", "obs_")) or rn.endswith("_target") or rn in LEGACY_KINDS or rn in ("vase", "blue_block"):
            continue
        out.append(b)
    return out


def _frame(approach, closing):
    """Rotation whose z is the approach axis and y the closing axis (both in the site frame)."""
    z = np.asarray(approach, float)
    z /= np.linalg.norm(z)
    y = np.asarray(closing, float)
    y = y - np.dot(y, z) * z
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


def add_grippers(spec, profile):
    """Robot-side additions for every role whose `grip` says build: a simple parallel-jaw gripper at the end-effector site
    (UR5e, iiwa: no gripper in the model; humanoids: a two-pad gripper on the palm). Adds a `<role>_tcp` site between
    the pads for every role (built or native), which the IK then targets."""
    grips = (profile.physics or {}).get("grip") or {}
    for role, g in grips.items():
        site = spec.site(profile.ee[role])
        body = site.parent
        sp, sq = np.array(site.pos, float), np.array(site.quat, float)
        Rs = np.zeros(9)
        mujoco.mju_quat2Mat(Rs, sq / np.linalg.norm(sq))
        Rs = Rs.reshape(3, 3)
        if not g.get("build"):
            tcp = sp + Rs @ np.asarray(g.get("tcp", (0, 0, 0)), float)
            body.add_site(name=f"{role}_tcp", pos=tcp.tolist(), quat=sq.tolist(), size=[0.006], rgba=[1, 0.6, 0.1, 0.0], group=5)
            _native_pads(spec, role, g)
            continue
        w, L, base = float(g["width"]), float(g.get("len", 0.07)), float(g.get("base", 0.0))
        Rg = Rs @ _frame(g.get("approach", profile.ee_axis), g.get("axis", (0, 1, 0)))
        gb = body.add_body(name=f"pc_{role}_gripper", pos=sp.tolist(), quat=_rot_to_quat(Rg).tolist())
        dark, pad_rgba = [0.18, 0.19, 0.21, 1], [0.32, 0.33, 0.36, 1]
        palm = [0.022, w / 2 + 0.014, 0.014]
        gb.add_geom(name=f"{role}_palm_vis", type=mujoco.mjtGeom.mjGEOM_BOX, size=palm, pos=[0, 0, base + palm[2]],
                    rgba=dark, contype=0, conaffinity=0, group=2, density=0)
        gb.add_geom(name=f"{role}_palm_col", type=mujoco.mjtGeom.mjGEOM_BOX, size=palm, pos=[0, 0, base + palm[2]],
                    rgba=[0, 0, 0, 0], group=3, mass=0.25, contype=1, conaffinity=1)
        pt = 0.007
        for side, sgn in (("a", 1.0), ("b", -1.0)):
            fb = gb.add_body(name=f"pc_{role}_finger_{side}", pos=[0, sgn * (w / 2 + pt), base + 2 * palm[2]])
            fb.add_joint(name=f"pc_{role}_finger_{side}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, -sgn, 0],
                         range=[0, w / 2], limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=5.0, armature=0.1)   # + transmission inertia
            size = [0.018, pt, L / 2]
            fb.add_geom(name=f"{role}_pad_{side}_vis", type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=[0, 0, L / 2], rgba=pad_rgba,
                        contype=0, conaffinity=0, group=2, density=0)
            fb.add_geom(name=f"{role}_pad_{side}", type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=[0, 0, L / 2], rgba=[0, 0, 0, 0],
                        group=3, mass=0.08, contype=1, conaffinity=1, friction=list(PAD_FRICTION), **PAD_CONTACT)
        gb.add_site(name=f"{role}_tcp", pos=[0, 0, base + 2 * palm[2] + L - 0.022], size=[0.006], rgba=[1, 0.6, 0.1, 0.0], group=5)
        for side in ("a", "b"):
            spec.add_exclude(bodyname1=f"pc_{role}_finger_{side}", bodyname2=body.name)
        spec.add_exclude(bodyname1=f"pc_{role}_finger_a", bodyname2=f"pc_{role}_finger_b")


def _native_pads(spec, role, g):
    """A model's own fingers, made to grasp: flat pads where the fingers really touch things (g["pads"]: boxes in the finger
    bodies' frames, from the finger meshes) when the model's collision hulls don't match the finger's inner face, those
    hulls switched off for contact (g["nocollide"]: geom names), and the pad contact (friction, stiff, torsional) on every
    geom the fingers grasp with. Planner and plant both get them (geom ids stay aligned)."""
    off = set(g.get("nocollide", ()))
    for side in ("a", "b"):
        for i, pd in enumerate((g.get("pads") or {}).get(side, ())):
            spec.body(pd["body"]).add_geom(name=f"{role}_pad_{side}{i}", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(pd["size"]),
                                           pos=list(pd["pos"]), quat=list(pd.get("quat", (1, 0, 0, 0))), rgba=[0, 0, 0, 0],
                                           group=3, density=0, contype=1, conaffinity=1, friction=list(PAD_FRICTION), **PAD_CONTACT)
    for side in ("a", "b"):
        for bn in g[side]:
            for geom in spec.body(bn).geoms:
                if geom.name in off:
                    geom.contype, geom.conaffinity = 0, 0
                elif (geom.contype or geom.conaffinity) and not geom.name.startswith(f"{role}_pad_"):
                    geom.friction = list(PAD_FRICTION)
                    for k, v in PAD_CONTACT.items():
                        setattr(geom, k, v)
    for bn in g.get("nocollide_bodies", ()):
        for geom in spec.body(bn).geoms:
            geom.contype, geom.conaffinity = 0, 0


# ------------------------------------------------------------------ plant-only changes
def _box_inertia(m, s):
    a, b, c = (2 * x for x in s)
    return [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12]


def _cyl_inertia(m, r, h):
    return [m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2]


def plant_spec(spec, profile, kinds):
    """Mutate the (already compiled once for the planner) spec into the plant: free objects, materials, contact bits,
    harness for legged bodies, servos for joints without actuators, gravity compensation, options."""
    o = spec.option
    o.timestep, o.integrator = DT, mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    o.cone, o.impratio = mujoco.mjtCone.mjCONE_ELLIPTIC, 1.0
    for k in list(spec.keys):                             # keyframes: the plant has more qpos (free objects)
        spec.delete(k)
    legged = profile.base == "legged"
    robot = robot_spec_bodies(spec)
    robot_names = {b.name for b in robot}
    for g in spec.geoms:
        p = g.parent
        if p is spec.worldbody:
            if g.name == "floor":
                g.contype, g.conaffinity, g.friction = 2, 2, list(SURFACE_FRICTION)
            elif g.contype or g.conaffinity:
                g.friction = list(SURFACE_FRICTION)
        elif legged and p.name in robot_names and (g.contype or g.conaffinity):
            g.conaffinity |= 2                            # feet stand on the floor
    for b in robot:
        b.gravcomp = 1.0                                  # the robot's controller compensates its own weight
    acted = {a.target for a in spec.actuators if a.trntype == mujoco.mjtTrn.mjTRN_JOINT}
    tendons = {t.name: t for t in spec.tendons}
    for a in spec.actuators:                              # joints a tendon servo drives (Stretch's telescoping arm) need no
        if a.trntype == mujoco.mjtTrn.mjTRN_TENDON and a.target in tendons:   # servo of their own
            for w in tendons[a.target].path:
                if w.type == mujoco.mjtWrap.mjWRAP_JOINT:
                    acted.add(getattr(w.target, "name", w.target))
    for j in spec.joints:
        if j.parent.name not in robot_names or j.type == mujoco.mjtJoint.mjJNT_FREE:
            continue
        hinge = j.type == mujoco.mjtJoint.mjJNT_HINGE
        if j.name in acted:
            j.actgravcomp = True                          # gravity compensation counts against the actuator's limit
            if hinge and j.armature < ARMATURE_MIN:       # geared joints have reflected rotor inertia; without it a light
                j.armature = ARMATURE_MIN                 # wrist rattles against a stiff grasp at 2 ms
            continue
        if j.name.startswith("pc_") or any(k in j.name for k in profile.freeze):
            continue                                      # finger linkages, rubber tips: passive (driven by the gripper)
        if j.armature < (ARMATURE_MIN if hinge else 0.01):
            j.armature = ARMATURE_MIN if hinge else 0.01  # reflected rotor inertia (models without it are too light)
        gains = list((profile.physics or {}).get("gains", ())) + ADDED_GAINS     # a profile's own servo gains first
        kp, kv, eff = next(v for pats, v in gains if not pats or any(k in j.name for k in pats))
        a = spec.add_actuator(name=f"pc_servo_{j.name}", target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=kp, kv=kv)
        a.forcelimited, a.forcerange = True, [-eff, eff]
        j.actgravcomp = True
    for pats, f in (profile.physics or {}).get("stiffer", ()):  # the model's own servos, stiffer (its gains were tuned for
        for a in spec.actuators:                                 # kinematic demos; the torque limits stay the model's)
            if a.trntype == mujoco.mjtTrn.mjTRN_JOINT and any(k in a.target for k in pats) and a.biasprm[1] < 0:
                g, b = np.array(a.gainprm, float), np.array(a.biasprm, float)
                g[0], b[1], b[2] = g[0] * f, b[1] * f, b[2] * math.sqrt(f)
                a.gainprm, a.biasprm = g, b
    for b1, b2 in (profile.physics or {}).get("exclude", ()):   # hulls of links that are chained through massless joints
        spec.add_exclude(bodyname1=b1, bodyname2=b2)           # (Apollo's torso and pelvis overlap whenever it bends)
    for role, g in ((profile.physics or {}).get("grip") or {}).items():
        if g.get("hide_hand"):                            # a rigid dexterous hand (Apollo): the pads do the grasping
            for geom in spec.site(profile.ee[role]).parent.find_all(mujoco.mjtObj.mjOBJ_GEOM):
                if not (geom.parent.name or "").startswith("pc_"):
                    geom.contype, geom.conaffinity = 0, 0
        if g.get("build"):                                # the jaws are coupled (rack and pinion): they move symmetrically
            spec.add_equality(name=f"pc_{role}_jaws", type=mujoco.mjtEq.mjEQ_JOINT, name1=f"pc_{role}_finger_a",
                              name2=f"pc_{role}_finger_b", objtype=mujoco.mjtObj.mjOBJ_JOINT, solref=[0.005, 1.0])
            for side in ("a", "b"):
                a = spec.add_actuator(name=f"pc_{role}_finger_{side}", target=f"pc_{role}_finger_{side}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
                a.set_to_position(kp=6000.0, kv=60.0)
                a.ctrllimited, a.ctrlrange = True, [0.0, float(g["width"]) / 2 + 0.03]
                a.forcelimited, a.forcerange = True, [-DEFAULTS["grip_force"], DEFAULTS["grip_force"]]
    if legged and profile.root:                          # weld the base to a harness that follows the planned base pose
        h = spec.worldbody.add_body(name="pc_harness", mocap=True)
        h.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.01, 0, 0], contype=0, conaffinity=0, group=5, rgba=[0, 0, 0, 0])
        e = spec.add_equality(name="pc_harness", type=mujoco.mjtEq.mjEQ_WELD, name1=profile.root, name2="pc_harness",
                              objtype=mujoco.mjtObj.mjOBJ_BODY)
        data = np.zeros(len(e.data))
        data[6], data[10] = 1.0, 1.0                     # relpose = identity (body pose == harness pose), torquescale 1
        e.data = data
        e.solref = [0.01, 1.0]
    for b in spec.bodies:                                 # movable objects: mocap -> free bodies with mass and friction
        n = b.name or ""
        if not b.mocap or not (n.startswith("obj_") or n in LEGACY_KINDS):
            continue
        name = n[4:] if n.startswith("obj_") else n
        kind = kinds.get(name) or LEGACY_KINDS.get(name, "block")
        col = spec.geom(f"{n}_collision")
        b.mocap = False
        b.add_freejoint(name=f"{n}_free")
        mass, mu = OBJ.get(kind, (0.2, 0.8))
        sz, gp = np.array(col.size, float), np.array(col.pos, float)
        cyl = col.type == mujoco.mjtGeom.mjGEOM_CYLINDER
        if kind == "mug":
            s = sz[0] / 0.065
            b.explicitinertial, b.mass = True, mass * s ** 3
            b.ipos = [0, MUG["cy"] * s, -0.012 * s]
            b.inertia = _cyl_inertia(b.mass, MUG["R"] * s, (MUG["z1"] - MUG["z0"]) * s)
            col.contype, col.conaffinity = 0, 0           # the bounding cylinder stays for the task layer; the walls collide
            _hollow_mug(b, n, s, mu)
            continue
        from .world.scene import KINDS                    # late: scene imports this module
        dsz = np.array(KINDS.get(kind, KINDS["block"])[0], float) / 2
        vol = float(sz[0] ** 2 * sz[1] / (dsz[0] ** 2 * dsz[2])) if cyl else float(np.prod(sz) / np.prod(dsz))
        b.explicitinertial, b.mass, b.ipos = True, float(mass * np.clip(vol, 0.2, 8.0)), gp.tolist()
        b.inertia = _cyl_inertia(b.mass, sz[0], 2 * sz[1]) if cyl else _box_inertia(b.mass, sz)
        col.contype, col.conaffinity, col.condim = 3, 3, 4
        col.friction = [mu, 0.01, 0.0001]


def _hollow_mug(b, n, s, mu):
    """The mug collides as what it is: a bottom, a wall of 12 segments and a handle, so a gripper can hold it by the rim."""
    R, t, z0, z1, cy = MUG["R"] * s, MUG["t"] * s, MUG["z0"] * s, MUG["z1"] * s, MUG["cy"] * s
    kw = dict(contype=3, conaffinity=3, condim=4, friction=[mu, 0.01, 0.0001], group=3, rgba=[0, 0, 0, 0])
    b.add_geom(name=f"{n}_bottom", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[R, 0.005 * s, 0], pos=[0, cy, z0 + 0.005 * s], **kw)
    hz, k = (z1 - z0) / 2, 12
    half_w = R * math.tan(math.pi / k) + 0.001
    for i in range(k):
        a = 2 * math.pi * i / k
        b.add_geom(name=f"{n}_wall{i}", type=mujoco.mjtGeom.mjGEOM_BOX, size=[t / 2, half_w, hz],
                   pos=[(R - t / 2) * math.cos(a), cy + (R - t / 2) * math.sin(a), z0 + hz], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)], **kw)
    b.add_geom(name=f"{n}_handle", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.006 * s, 0.016 * s, 0.035 * s],
               pos=[0, cy - R - 0.014 * s, (z0 + z1) / 2 + 0.004 * s], **kw)


# ------------------------------------------------------------------ the plant
class Plant:
    """MjData stepped with mj_step, tracking q_des. One per world; the runtime owns it (rt.plant)."""

    def __init__(self, model, ik_model, profile, kinds, robot_bodies, ee_sites):
        self.m, self.d = model, mujoco.MjData(model)
        m = model
        self.profile, self.kinds = profile, dict(kinds)
        self.nq_r = ik_model.nq                          # robot qpos prefix (planner)
        self.nv_r = ik_model.nv
        self.robot_bodies = set(robot_bodies)
        self.ee_sites = dict(ee_sites)                   # role -> site name (tcp)
        self.params = dict(DEFAULTS)
        self.base = {"gain": m.actuator_gainprm.copy(), "bias": m.actuator_biasprm.copy(), "frange": m.actuator_forcerange.copy(),
                     "flim": m.actuator_forcelimited.copy(), "fric": m.geom_friction.copy(), "mass": m.body_mass.copy(),
                     "inertia": m.body_inertia.copy(), "damp": m.dof_damping.copy()}
        # actuators: joint position servos track q_des; velocity servos hold 0; gripper actuators get gripper commands
        grips = (profile.physics or {}).get("grip") or {}
        self.grip = {}
        grip_acts = set()
        for role, g in grips.items():
            acts = [m.actuator(f"pc_{role}_finger_{s}").id for s in ("a", "b")] if g.get("build") else [m.actuator(g["act"]).id]
            grip_acts |= set(acts)
            pads = self._pads(role, g)
            built = bool(g.get("build"))
            self.grip[role] = {"acts": acts, "open": g.get("open", 0.0), "close": g.get("close"), "build": built,
                               "pads": pads, "cmd": "open", "width": float(g.get("width", 0.08)), "boost": float(g.get("boost", 1.0)),
                               # in the TCP site's frame: the closing axis, the approach axis, TCP -> fingertip distance
                               "axis": np.array((0, 1, 0) if built else g.get("axis", (0, 1, 0)), float),
                               "approach": np.array((0, 0, 1) if built else g.get("approach", (0, 0, 1)), float),
                               "tip": 0.022 if built else float(g.get("tip", 0.015)),
                               # one fixed jaw (side b): where its face is along the closing axis from the TCP (m)
                               "jaw": None if g.get("jaw") is None else float(g["jaw"]),
                               # side grasps: the hand's top side (TCP frame) and how far it hangs below the TCP (m)
                               "up": None if g.get("up") is None else np.array(g["up"], float), "pad_h": float(g.get("pad_h", 0.018))}
            if g.get("build"):
                self.grip[role]["close"] = float(g["width"]) / 2 + 0.03
        self.track, self.hold0, self.track_tendon = [], [], []
        for a in range(m.nu):
            if a in grip_acts:
                continue
            if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
                j = m.actuator_trnid[a, 0]
                if m.jnt_qposadr[j] >= self.nq_r or m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    continue
                pos_servo = m.actuator_biasprm[a, 1] < 0 and m.actuator_gaintype[a] == 0
                if pos_servo:
                    self.track.append((a, int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j])))
                else:
                    self.hold0.append(a)
            elif m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_TENDON and m.actuator_biasprm[a, 1] < 0:
                t = m.actuator_trnid[a, 0]               # a position servo on a fixed tendon of robot joints (Stretch's
                parts = []                               # telescoping arm): it tracks the tendon length of the plan
                for k in range(m.tendon_adr[t], m.tendon_adr[t] + m.tendon_num[t]):
                    if m.wrap_type[k] == mujoco.mjtWrap.mjWRAP_JOINT:
                        j = m.wrap_objid[k]
                        parts.append((int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]), float(m.wrap_prm[k])))
                if parts and all(qa < self.nq_r for qa, _, _ in parts):
                    self.track_tendon.append((a, parts))
        self.arm_acts = self._arm_acts()
        self.harness = None
        try:
            self.harness = (m.body("pc_harness").mocapid[0], m.body(profile.root).id)
        except KeyError:
            pass
        self.site_ids = {r: m.site(s).id for r, s in self.ee_sites.items()}
        self.ee_bodies = {r: self._subtree(m.site_bodyid[sid]) for r, sid in self.site_ids.items()}
        self._measure_tips(ik_model)
        self.objects = {}
        for b in range(1, m.nbody):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if m.body_jntnum[b] == 1 and m.jnt_type[m.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE and (n.startswith("obj_") or n in LEGACY_KINDS):
                name = n[4:] if n.startswith("obj_") else n
                j = m.body_jntadr[b]
                self.objects[name] = {"bid": b, "qadr": int(m.jnt_qposadr[j]), "dadr": int(m.jnt_dofadr[j]),
                                      "kind": self.kinds.get(name) or LEGACY_KINDS.get(name, "block"),
                                      "geoms": [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and (m.geom_contype[g] or m.geom_conaffinity[g])]}
        self.geom_obj = {g: n for n, o in self.objects.items() for g in o["geoms"]}
        self.q_cmd = None                                 # rate-limited setpoint the servos see
        self.v_cmd, self.q_tgt = None, None               # its velocity, the plan it follows
        slide = np.zeros(self.nq_r, bool)
        for j in range(m.njnt):
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE and m.jnt_qposadr[j] < self.nq_r:
                slide[m.jnt_qposadr[j]] = True
        self.v_max = np.where(slide, 0.5 * V_MAX, V_MAX)
        self.a_max = np.where(slide, 0.5 * A_MAX, A_MAX)
        self.q_prev = None
        self.lag = deque()
        self.acc = 0.0
        self.sub_cap = 60
        self.rtf, self._sim_w, self._wall_w, self._t_w = 1.0, 0.0, 0.0, time.time()
        self.step_cost = 0.0
        self.ee_force = {r: np.zeros(3) for r in self.site_ids}        # wrist-sensor-like: contact force on the hand (N)
        self.ee_force_raw = {r: np.zeros(3) for r in self.site_ids}
        self._f6 = np.zeros(6)
        self.set_params({})

    # ---------------------------------------------------------------- structure
    def _subtree(self, b0):
        m = self.m
        out = {b0}
        for b in range(b0 + 1, m.nbody):
            if m.body_parentid[b] in out:
                out.add(b)
        return out

    def _measure_tips(self, ik_model):
        """How far the gripper reaches past its TCP along the approach axis (fingertips, open), from its collision geoms."""
        m = self.m
        d = mujoco.MjData(m)
        d.qpos[:] = m.qpos0
        mujoco.mj_kinematics(m, d)
        for r, g in self.grip.items():
            sid = self.site_ids[r]
            R, p = d.site_xmat[sid].reshape(3, 3), d.site_xpos[sid]
            ap, ax = R @ g["approach"], R @ g["axis"]
            third = np.cross(ap, ax)
            reach, span, depth = 0.0, 0.0, 0.0
            for k in range(m.ngeom):
                if m.geom_bodyid[k] not in self.ee_bodies[r] or not (m.geom_contype[k] or m.geom_conaffinity[k]):
                    continue
                c, h = m.geom_aabb[k, :3], m.geom_aabb[k, 3:]
                Rg = d.geom_xmat[k].reshape(3, 3)
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            corner = d.geom_xpos[k] + Rg @ (c + h * np.array([sx, sy, sz])) - p
                            reach = max(reach, float(np.dot(corner, ap)))
                            if np.dot(corner, ap) > -0.06:     # the hand's footprint near the work (not the wrist above)
                                span, depth = max(span, abs(float(np.dot(corner, ax)))), max(depth, abs(float(np.dot(corner, third))))
            g["tip"] = max(g["tip"], reach)
            g["span"], g["depth"] = span, depth             # half extents across / along the jaws (open), for placing

    def _pads(self, role, g):
        m = self.m
        if g.get("build"):
            return [[m.geom(f"{role}_pad_a").id], [m.geom(f"{role}_pad_b").id]]
        out = []
        for pats in (g["a"], g["b"]):
            ids = [k for k in range(m.ngeom) if (m.geom_contype[k] or m.geom_conaffinity[k]) and
                   any((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[k]) or "") == p for p in pats)]
            out.append(ids)
        return out

    def _arm_acts(self):
        """Actuators that move the arms (impedance mode softens these): joints between the base and each end effector."""
        m = self.m
        out = set()
        for s in self.ee_sites.values():
            b = m.site_bodyid[m.site(s).id]
            while b > 0:
                for k in range(m.body_jntnum[b]):
                    j = m.body_jntadr[b] + k
                    for a, qa, _ in self.track:
                        if m.actuator_trnid[a, 0] == j:
                            out.add(a)
                b = m.body_parentid[b]
        return out

    # ---------------------------------------------------------------- parameters
    def set_params(self, p):
        """Plant parameters at run time (P6 randomization / calibration). Unknown keys are ignored, values clamped."""
        for k, v in (p or {}).items():
            if k not in DEFAULTS:
                continue
            if k == "impedance":
                self.params[k] = bool(v)
            else:
                lo, hi = LIMITS[k]
                self.params[k] = float(np.clip(float(v), lo, hi))
        m, b, P = self.m, self.base, self.params
        m.geom_friction[:] = b["fric"]
        m.geom_friction[:, 0] *= P["friction_scale"]
        objs = [o["bid"] for o in self.objects.values()]
        m.body_mass[:] = b["mass"]
        m.body_inertia[:] = b["inertia"]
        m.body_mass[objs] *= P["mass_scale"]
        m.body_inertia[objs] *= P["mass_scale"]
        m.dof_damping[:] = b["damp"] * P["damping_scale"]
        m.opt.gravity[:] = (0.0, 0.0, P["gravity"])
        m.actuator_gainprm[:] = b["gain"]
        m.actuator_biasprm[:] = b["bias"]
        m.actuator_forcerange[:] = b["frange"]
        m.actuator_forcelimited[:] = b["flim"]
        for a, _, _ in self.track:
            k = P["kp_scale"] * (IMPEDANCE["kp"] if P["impedance"] and a in self.arm_acts else 1.0)
            m.actuator_gainprm[a, 0] *= k
            m.actuator_biasprm[a, 1] *= k
            m.actuator_biasprm[a, 2] *= math.sqrt(k)
            if P["impedance"] and a in self.arm_acts:
                fr = b["frange"][a] if b["flim"][a] else np.array([-80.0, 80.0])
                m.actuator_forcelimited[a] = 1
                m.actuator_forcerange[a] = fr * IMPEDANCE["effort"]
        for role, g in self.grip.items():
            F = P["grip_force"] * g["boost"]
            for a in g["acts"]:                           # native joint-servo fingers: stiff enough to saturate at F
                if not g["build"] and b["bias"][a, 1] < 0 and m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
                    kp = max(b["gain"][a, 0], 3000.0)
                    m.actuator_gainprm[a, 0], m.actuator_biasprm[a, 1] = kp, -kp
                m.actuator_forcelimited[a] = 1
                m.actuator_forcerange[a] = (-F, F)
        return self.params

    def reset_params(self):
        self.params = dict(DEFAULTS)
        return self.set_params({})

    # ---------------------------------------------------------------- state
    def reset(self, q_robot, objects_home=True):
        """Hard reset (world built / scene reset): robot at q_robot, objects at their initial poses, at rest."""
        m, d = self.m, self.d
        if objects_home:
            mp, mq = d.mocap_pos.copy(), d.mocap_quat.copy()     # the IK target ghosts live here: keep them
            mujoco.mj_resetData(m, d)
            d.mocap_pos[:], d.mocap_quat[:] = mp, mq
        d.qpos[:self.nq_r] = q_robot
        d.qvel[:] = 0
        d.act[:] = 0
        self.q_cmd = np.array(q_robot, float).copy()
        self.v_cmd, self.q_tgt = np.zeros(self.nq_r), self.q_cmd.copy()
        self.q_prev = self.q_cmd.copy()
        self.lag.clear()
        self._apply_ctrl(self.q_cmd, np.zeros(self.nq_r))
        mujoco.mj_forward(m, d)
        if self.harness is not None:                     # the harness starts where the base is
            mid, bid = self.harness
            d.mocap_pos[mid] = d.xpos[bid]
            mujoco.mju_mat2Quat(d.mocap_quat[mid], d.xmat[bid])
            mujoco.mj_forward(m, d)

    def snap_robot(self, q_robot):
        """Teleport only the robot (keeps objects where they are)."""
        d = self.d
        d.qpos[:self.nq_r] = q_robot
        d.qvel[:self.nv_r] = 0
        self.q_cmd = np.array(q_robot, float).copy()
        self.v_cmd, self.q_tgt = np.zeros(self.nq_r), self.q_cmd.copy()
        self.q_prev = self.q_cmd.copy()
        self.lag.clear()
        mujoco.mj_forward(self.m, d)

    def settle(self, sec=0.4):
        for _ in range(int(sec / DT)):
            self._apply_ctrl(self.q_cmd, np.zeros(self.nq_r))
            mujoco.mj_step(self.m, self.d)

    # ---------------------------------------------------------------- control
    def _apply_ctrl(self, q, qd):
        m, d = self.m, self.d
        for a, qa, da in self.track:
            # setpoint + velocity feed-forward: kp (q* - q) + kv (qd* - qd) + b qd*, so neither the servo's damping nor the
            # joint's own (b, friction-like) make it lag behind a moving plan
            kp, kv = m.actuator_gainprm[a, 0], -m.actuator_biasprm[a, 2] + m.dof_damping[da]
            v = q[qa] + (kv / kp * qd[qa] if kp > 0 else 0.0)        # hinge/slide: qpos and velocity share the index
            lo, hi = m.actuator_ctrlrange[a] if m.actuator_ctrllimited[a] else (-1e9, 1e9)
            d.ctrl[a] = min(max(v, lo), hi)
        for a in self.hold0:
            d.ctrl[a] = 0.0
        for a, parts in self.track_tendon:
            kp, kv = m.actuator_gainprm[a, 0], -m.actuator_biasprm[a, 2] + sum(m.dof_damping[da] * c * c for _, da, c in parts)
            L = sum(c * q[qa] for qa, _, c in parts)
            Ld = sum(c * qd[qa] for qa, _, c in parts)
            v = L + (kv / kp * Ld if kp > 0 else 0.0)
            lo, hi = m.actuator_ctrlrange[a] if m.actuator_ctrllimited[a] else (-1e9, 1e9)
            d.ctrl[a] = min(max(v, lo), hi)
        for g in self.grip.values():                      # fingers take ~0.8 s to travel: no slamming into things
            v = g["open"] if g["cmd"] == "open" else g["close"]
            rate = abs(g["close"] - g["open"]) / 0.8 * DT
            g["ctrl"] = v if g.get("ctrl") is None else g["ctrl"] + float(np.clip(v - g["ctrl"], -rate, rate))
            for a in g["acts"]:
                d.ctrl[a] = g["ctrl"]

    def _harness(self, base_pose):
        if self.harness is None or base_pose is None:
            return
        mid, _ = self.harness
        self.d.mocap_pos[mid] = base_pose[0]
        self.d.mocap_quat[mid] = base_pose[1]

    def step(self, dt, q_des, base_pose=None):
        """Advance the plant by dt (wall seconds of this tick) toward q_des. Returns the number of substeps run."""
        m, d = self.m, self.d
        t0 = time.time()
        lat = self.params["latency_ms"] / 1000.0
        self.lag.append((d.time + dt, np.array(q_des, float)))
        target = self.lag[0][1]
        while len(self.lag) > 1 and self.lag[1][0] <= d.time + dt - lat:
            self.lag.popleft()
            target = self.lag[0][1]
        if lat <= 0:
            target = self.lag[-1][1]
            self.lag.clear()
            self.lag.append((d.time, target))
        self.acc += dt
        n = int(self.acc / DT)
        self.acc -= n * DT
        if n > self.sub_cap:                             # can't keep up: run fewer substeps (sim slows, rtf < 1)
            n, self.acc = self.sub_cap, 0.0
        if self.q_cmd is None:
            self.q_cmd = target.copy()
        if self.v_cmd is None or self.q_tgt is None:
            self.v_cmd, self.q_tgt = np.zeros_like(target), target.copy()
        self._harness(base_pose)
        if n:                                            # the setpoint follows the plan with velocity and acceleration limits
            v_plan = (target - self.q_tgt) / (n * DT)    # (a real controller never jumps: a planner that changes its mind
            self.q_tgt = target.copy()                   # quickly would otherwise jerk the arm, and what it holds)
            for s in range(n):
                err = target - self.q_cmd
                v_want = np.clip(v_plan + err / SETPOINT_TAU, -self.v_max, self.v_max)
                v_want = np.sign(v_want) * np.minimum(np.abs(v_want), np.sqrt(2 * self.a_max * np.abs(err)) + np.abs(v_plan))
                self.v_cmd = self.v_cmd + np.clip(v_want - self.v_cmd, -self.a_max * DT, self.a_max * DT)
                self.q_cmd = self.q_cmd + self.v_cmd * DT
                self._apply_ctrl(self.q_cmd, self.v_cmd)
                mujoco.mj_step(m, d)
        cost = time.time() - t0
        self.step_cost = 0.9 * self.step_cost + 0.1 * (cost / max(n, 1))
        self._sim_w += n * DT
        self._wall_w += dt
        now = time.time()
        if now - self._t_w >= 1.0:
            self.rtf = self._sim_w / max(self._wall_w, 1e-6) if self._wall_w > 0 else 1.0
            self._sim_w, self._wall_w, self._t_w = 0.0, 0.0, now
        self._measure()
        return n

    def budget(self, frame_budget):
        """Cap substeps so stepping uses at most frame_budget seconds per tick (called by the server loop)."""
        if self.step_cost > 0:
            self.sub_cap = int(np.clip(frame_budget / self.step_cost, 4, 60))

    # ---------------------------------------------------------------- sensing
    def _contact_force(self, i):
        """World-frame force that geom1 of contact i exerts on geom2."""
        mujoco.mj_contactForce(self.m, self.d, i, self._f6)
        return self.d.contact[i].frame.reshape(3, 3).T @ self._f6[:3]

    def _measure(self):
        m, d = self.m, self.d
        f = {r: np.zeros(3) for r in self.site_ids}
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            for r, bodies in self.ee_bodies.items():
                in1, in2 = b1 in bodies, b2 in bodies
                if in1 == in2:
                    continue
                other = b2 if in1 else b1
                if other in self.robot_bodies:
                    continue
                fc = self._contact_force(i)
                f[r] += fc if in2 else -fc                # force on the hand
        self.ee_force_raw = f
        for r in f:
            self.ee_force[r] = 0.8 * self.ee_force[r] + 0.2 * f[r]

    def touching(self, role, name):
        """(side a touches, side b touches) the object `name`."""
        g = self.grip.get(role)
        o = self.objects.get(name)
        if g is None or o is None:
            return False, False
        d, geoms = self.d, set(o["geoms"])
        a, b = set(g["pads"][0]), set(g["pads"][1])
        ta = tb = False
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if g1 in geoms or g2 in geoms:
                other = g2 if g1 in geoms else g1
                ta |= other in a
                tb |= other in b
        return ta, tb

    def object_contacts(self, name):
        """Geoms touching the object, other than the given role's gripper (table, other objects, ...)."""
        o = self.objects.get(name)
        if o is None:
            return []
        geoms, d, out = set(o["geoms"]), self.d, []
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in geoms and c.geom2 not in geoms:
                out.append(c.geom2)
            elif c.geom2 in geoms and c.geom1 not in geoms:
                out.append(c.geom1)
        return out

    def grip_load(self, role, name):
        """Upward force (N) the hand exerts on the object: its weight once it hangs free in the gripper."""
        o = self.objects.get(name)
        if o is None or role not in self.ee_bodies:
            return 0.0
        m, d, geoms, hand = self.m, self.d, set(o["geoms"]), self.ee_bodies[role]
        fz = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom2 in geoms and m.geom_bodyid[c.geom1] in hand:
                fz += self._contact_force(i)[2]
            elif c.geom1 in geoms and m.geom_bodyid[c.geom2] in hand:
                fz -= self._contact_force(i)[2]
        return fz

    def support(self, name, role=None):
        """Upward force (N) on the object from everything but the given hand: the surface (or whatever it rests on)."""
        o = self.objects.get(name)
        if o is None:
            return 0.0
        m, d, geoms = self.m, self.d, set(o["geoms"])
        hand = self.ee_bodies.get(role, set())
        fz = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom2 in geoms and c.geom1 not in geoms and m.geom_bodyid[c.geom1] not in hand:
                fz += self._contact_force(i)[2]
            elif c.geom1 in geoms and c.geom2 not in geoms and m.geom_bodyid[c.geom2] not in hand:
                fz -= self._contact_force(i)[2]
        return fz

    def weight(self, name):
        o = self.objects.get(name)
        return 0.0 if o is None else float(self.m.body_mass[o["bid"]]) * abs(float(self.m.opt.gravity[2]))

    def grip_cmd(self, role, cmd):
        if role in self.grip:
            self.grip[role]["cmd"] = cmd

    def grasped(self):
        """role -> object held by contact on both sides (or None)."""
        out = {}
        for r in self.grip:
            out[r] = next((n for n in self.objects if all(self.touching(r, n))), None)
        return out

    def obj_speed(self, name):
        o = self.objects[name]
        return float(np.linalg.norm(self.d.qvel[o["dadr"]:o["dadr"] + 3]))

    def set_object(self, name, pos, quat=None):
        o = self.objects[name]
        d = self.d
        d.qpos[o["qadr"]:o["qadr"] + 3] = pos
        if quat is not None:
            d.qpos[o["qadr"] + 3:o["qadr"] + 7] = quat
        d.qvel[o["dadr"]:o["dadr"] + 6] = 0
        mujoco.mj_kinematics(self.m, d)

    def _label(self, g):
        m = self.m
        if g in self.geom_obj:
            return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, self.objects[self.geom_obj[g]]["bid"])
        b = m.geom_bodyid[g]
        for r, gr in self.grip.items():
            if g in gr["pads"][0] or g in gr["pads"][1]:
                return f"{r}_pad"
        for r, bodies in self.ee_bodies.items():
            if b in bodies:
                return f"{r}_hand"
        if b in self.robot_bodies:
            return "robot"
        return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "world"

    def state(self, holding=None):
        m, d = self.m, self.d
        cons = []
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 not in self.geom_obj and c.geom2 not in self.geom_obj:
                b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                if not any((b1 in bs) != (b2 in bs) for bs in self.ee_bodies.values()):
                    continue                              # only contacts with objects or the hands
            mujoco.mj_contactForce(m, d, i, self._f6)
            cons.append({"a": self._label(c.geom1), "b": self._label(c.geom2), "f": round(float(self._f6[0]), 2)})
        cons.sort(key=lambda c: -c["f"])
        objs = {}
        for n, o in self.objects.items():
            q = d.qpos[o["qadr"]:o["qadr"] + 7]
            objs[n] = {"p": [round(float(x), 4) for x in q[:3]], "q": [round(float(x), 4) for x in q[3:]], "v": round(self.obj_speed(n), 3)}
        return {"mode": "dynamic", "dt": DT, "rtf": round(self.rtf, 2), "substeps": self.sub_cap, "time": round(float(d.time), 2),
                "grasp": self.grasped(), "ee_force": {r: [round(float(x), 2) for x in v] for r, v in self.ee_force.items()},
                "contacts": cons[:12], "objects": objs, "params": dict(self.params)}


def kinematic_state(holding):
    return {"mode": "kinematic", "dt": None, "rtf": 1.0, "grasp": holding, "ee_force": {}, "contacts": [], "params": {}}
