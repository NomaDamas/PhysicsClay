"""The calibration parameter vector (docs/calibration.md): what system identification fits and domain randomization varies.

Every parameter is a P0 plant parameter (pps/physics.py DEFAULTS), so a fitted or sampled vector is applied to a live
model without rebuilding the world, the same way the `{type: "physics", op: "set", params}` message does it:

| name               | what it scales                                                        | default | search range | space |
|--------------------|-----------------------------------------------------------------------|---------|--------------|-------|
| friction_scale     | sliding friction of every geom (object-surface, finger pads)          | 1       | 0.4 .. 2.0   | log   |
| mass_scale         | mass and inertia of the movable objects                               | 1       | 0.4 .. 2.5   | log   |
| damping_scale      | viscous damping of every joint                                        | 1       | 0.4 .. 2.5   | log   |
| kp_scale           | position-servo stiffness (the servo's kv scales with its square root) | 1       | 0.5 .. 2.0   | log   |
| latency_ms         | delay between a command and the servos seeing it                      | 0       | 0 .. 80      | lin   |
| armature_scale     | rotor inertia reflected at the robot's joints (optional)              | 1       | 0.5 .. 2.0   | log   |
| frictionloss_scale | Coulomb friction of the robot's joints (optional)                     | 1       | 0.2 .. 3.0   | log   |

A scale parameter only exists for a robot when the model has something to scale (`active(plant)`): Panda joints have
damping and armature but no frictionloss, the UR5e's have no damping, a scene without movable objects has no mass or
object friction to fit. Per-robot ranges can be narrowed in ROBOTS.
"""
import math
from dataclasses import dataclass, replace

import numpy as np

from .. import physics as phys


@dataclass(frozen=True)
class Spec:
    name: str
    default: float
    lo: float
    hi: float
    log: bool
    unit: str
    what: str

    def clip(self, v):
        return float(np.clip(float(v), self.lo, self.hi))

    def to_unit(self, v):
        v = self.clip(v)
        if self.log:
            return (math.log(v) - math.log(self.lo)) / (math.log(self.hi) - math.log(self.lo))
        return (v - self.lo) / (self.hi - self.lo)

    def from_unit(self, u):
        u = float(np.clip(u, 0.0, 1.0))
        if self.log:
            return float(math.exp(math.log(self.lo) + u * (math.log(self.hi) - math.log(self.lo))))
        return float(self.lo + u * (self.hi - self.lo))


PARAMS = {s.name: s for s in [
    Spec("friction_scale", 1.0, 0.4, 2.0, True, "×", "sliding friction (object-surface, finger pads)"),
    Spec("mass_scale", 1.0, 0.4, 2.5, True, "×", "object mass and inertia"),
    Spec("damping_scale", 1.0, 0.4, 2.5, True, "×", "joint viscous damping"),
    Spec("kp_scale", 1.0, 0.5, 2.0, True, "×", "servo stiffness (kv ∝ √kp)"),
    Spec("latency_ms", 0.0, 0.0, 80.0, False, "ms", "command delay"),
    Spec("armature_scale", 1.0, 0.5, 2.0, True, "×", "joint rotor inertia"),
    Spec("frictionloss_scale", 1.0, 0.2, 3.0, True, "×", "joint Coulomb friction"),
]}
CORE = ("friction_scale", "mass_scale", "damping_scale", "kp_scale", "latency_ms")
OPTIONAL = ("armature_scale", "frictionloss_scale")
# per-robot overrides of the search / randomization range: {robot id: {name: (lo, hi)}}
ROBOTS = {
    "g1": {"kp_scale": (0.6, 1.6)},
    "h2": {"kp_scale": (0.6, 1.6)},
}
assert all(n in phys.DEFAULTS for n in PARAMS), "every calibration parameter must be a plant parameter"


def spec(name, robot=None):
    s = PARAMS[name]
    lohi = (ROBOTS.get(robot or "") or {}).get(name)
    return replace(s, lo=lohi[0], hi=lohi[1]) if lohi else s


def space(names=None, robot=None):
    """-> [Spec] for `names` (default: the core five) with the robot's ranges."""
    return [spec(n, robot) for n in (names or CORE)]


def defaults(names=None):
    return {n: PARAMS[n].default for n in (names or CORE)}


def active(plant):
    """Parameters that change something on this plant (a scale of zeros does nothing)."""
    m, nv = plant.m, plant.nv_r
    out = ["kp_scale", "latency_ms"] if plant.track else ["latency_ms"]
    if plant.objects:
        out += ["friction_scale", "mass_scale"]
    if np.any(m.dof_damping[:nv] > 0) or np.any(plant.base["damp"][:nv] > 0):
        out.append("damping_scale")
    if np.any(plant.base["arm"][:nv] > 0):
        out.append("armature_scale")
    if np.any(plant.base["floss"][:nv] > 0):
        out.append("frictionloss_scale")
    return [n for n in PARAMS if n in out]


def fit_set(plant, optional=False):
    """The parameters system identification fits by default on this plant."""
    act = active(plant)
    return [n for n in CORE + (OPTIONAL if optional else ()) if n in act]


def to_unit(params, specs):
    return np.array([s.to_unit(params.get(s.name, s.default)) for s in specs])


def from_unit(x, specs):
    return {s.name: s.from_unit(u) for s, u in zip(specs, x)}


# ------------------------------------------------------------------ applying
def apply(target, params):
    """Apply a parameter vector to a live plant (pps.physics.Plant) or a ModelParams. Missing keys keep their value.
    -> the plant's full parameter dict."""
    p = {k: float(v) for k, v in (params or {}).items() if k in phys.DEFAULTS and k != "impedance"}
    return target.set_params(p)


def snapshot(target):
    return dict(target.params)


def restore(target, snap):
    """Back to a snapshot (snapshot(plant) taken before apply)."""
    return target.set_params(dict(snap))


def physics_message(params, tid=None):
    """The same change as a browser / P1 generator message: {type: physics, op: set, params}."""
    msg = {"type": "physics", "op": "set", "params": {k: round(float(v), 6) for k, v in params.items() if k in phys.DEFAULTS}}
    if tid:
        msg["id"] = tid
    return msg


class ModelParams:
    """The same parameters on a bare MjModel (no Plant): for tools and tests that build their own model.
    objects: body ids of the movable objects; servos: position-actuator ids; robot_dofs: dof ids of the robot joints."""

    def __init__(self, model, objects=(), servos=(), robot_dofs=None):
        m = self.m = model
        self.objects, self.servos = list(objects), list(servos)
        self.dofs = list(range(m.nv)) if robot_dofs is None else list(robot_dofs)
        self.base = {"fric": m.geom_friction.copy(), "mass": m.body_mass.copy(), "inertia": m.body_inertia.copy(),
                     "damp": m.dof_damping.copy(), "arm": m.dof_armature.copy(), "floss": m.dof_frictionloss.copy(),
                     "gain": m.actuator_gainprm.copy(), "bias": m.actuator_biasprm.copy()}
        self.params = {n: phys.DEFAULTS[n] for n in PARAMS}

    def set_params(self, p):
        for k, v in (p or {}).items():
            if k in PARAMS:
                lo, hi = phys.LIMITS[k]
                self.params[k] = float(np.clip(float(v), lo, hi))
        m, b, P = self.m, self.base, self.params
        m.geom_friction[:] = b["fric"]
        m.geom_friction[:, 0] *= P["friction_scale"]
        m.body_mass[:], m.body_inertia[:] = b["mass"], b["inertia"]
        m.body_mass[self.objects] *= P["mass_scale"]
        m.body_inertia[self.objects] *= P["mass_scale"]
        m.dof_damping[:] = b["damp"] * P["damping_scale"]
        m.dof_armature[:], m.dof_frictionloss[:] = b["arm"], b["floss"]
        m.dof_armature[self.dofs] *= P["armature_scale"]
        m.dof_frictionloss[self.dofs] *= P["frictionloss_scale"]
        m.actuator_gainprm[:], m.actuator_biasprm[:] = b["gain"], b["bias"]
        k = P["kp_scale"]
        for a in self.servos:
            m.actuator_gainprm[a, 0] *= k
            m.actuator_biasprm[a, 1] *= k
            m.actuator_biasprm[a, 2] *= math.sqrt(k)
        return dict(self.params)

    def reset_params(self):
        return self.set_params({n: PARAMS[n].default for n in PARAMS})
