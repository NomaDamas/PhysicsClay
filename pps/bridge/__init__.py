"""Real-robot bridge (docs/V1.md P6, docs/calibration.md): one small interface between PhysicsClay's planner and a robot.

Everything that commands a robot for calibration (pps/calib/probes.py) goes through a `RobotDriver`, so the same probe
routine and system identification run on the simulated plant today and on hardware later. No hardware is claimed:
`SimDriver` wraps the MuJoCo plant (pps/physics.py); `LeRobotDriver` and `ROS2Driver` are documented stubs that raise
NotImplementedError and say what an implementation needs.

The protocol (duck-typed; `isinstance(x, RobotDriver)` works because it is a runtime-checkable Protocol):

    read_state() -> {"q": [nq] joint positions (rad / m) in the planner's joint order,
                     "dq": [nv] joint velocities,
                     "gripper": {role: opening 0 (closed) .. 1 (open)},
                     "ee_force": {role: [fx, fy, fz] N, world frame, force *on* the hand} | None (no F/T sensor),
                     "t": seconds on the robot's clock}
    send_q_des(q, t)          joint position setpoint for time t (the robot's own servos track it)
    send_gripper(role, value) 0 = open .. 1 = close
    stop()                    hold the current position (protective stop: q_des := measured q)
    capabilities              {"name", "joints": [names], "roles": [..], "ee_force": bool, "objects": bool,
                               "realtime": bool, "rate_hz": float}

A real-time driver (`capabilities["realtime"]`) runs on the robot's clock: the caller sends commands at `rate_hz` and
time passes by itself. A simulated driver is stepped by its caller with `advance(dt)` (not part of the protocol).
Object poses (`read_state()["objects"]`, `{name: {"p": [x,y,z], "q": [w,x,y,z]}}`) are optional: on hardware they come
from perception (markers, a depth camera, Gemini ER), in sim from the plant.
"""
from typing import Protocol, runtime_checkable

from .sim import SimDriver
from .stubs import LeRobotDriver, ROS2Driver

__all__ = ["RobotDriver", "SimDriver", "LeRobotDriver", "ROS2Driver", "conforms"]


@runtime_checkable
class RobotDriver(Protocol):
    capabilities: dict

    def read_state(self) -> dict: ...

    def send_q_des(self, q, t: float) -> None: ...

    def send_gripper(self, role: str, value: float) -> None: ...

    def stop(self) -> None: ...


STATE_KEYS = ("q", "dq", "gripper", "ee_force", "t")
CAP_KEYS = ("name", "joints", "roles", "ee_force", "realtime", "rate_hz")


def conforms(driver):
    """-> list of problems (empty when the driver implements the protocol and its state has the documented keys)."""
    bad = []
    if not isinstance(driver, RobotDriver):
        bad.append("does not implement read_state / send_q_des / send_gripper / stop / capabilities")
        return bad
    caps = driver.capabilities if isinstance(driver.capabilities, dict) else {}
    bad += [f"capabilities lacks '{k}'" for k in CAP_KEYS if k not in caps]
    try:
        s = driver.read_state()
    except NotImplementedError as e:
        return bad + [f"read_state: {e}"]
    bad += [f"state lacks '{k}'" for k in STATE_KEYS if k not in s]
    if "q" in s and caps.get("joints") is not None and len(s["q"]) != len(caps["joints"]):
        bad.append(f"q has {len(s['q'])} entries for {len(caps['joints'])} joints")
    return bad
