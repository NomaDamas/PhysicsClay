"""Hardware drivers: documented stubs. PhysicsClay does not drive hardware yet and claims no hardware results.

Each stub states what an implementation needs. To add one, implement the four methods and `capabilities` of the
RobotDriver protocol (pps/bridge/__init__.py), keep the joint order of the robot profile's planner model (the
`joints` capability, checked by `pps.bridge.conforms`), and run `scripts/calibrate.py --driver <yours>` once per site.
"""


class _Stub:
    NEEDS = ""

    def __init__(self, **config):
        self.config = dict(config)
        self.capabilities = {"name": self.NAME, "joints": list(config.get("joints") or []), "roles": list(config.get("roles") or ["right"]),
                             "ee_force": bool(config.get("ee_force", False)), "objects": False, "realtime": True,
                             "rate_hz": float(config.get("rate_hz", 100.0))}

    def _no(self, what):
        raise NotImplementedError(f"{self.NAME}.{what} is not implemented: PhysicsClay has no hardware driver yet. {self.NEEDS}")

    def read_state(self):
        self._no("read_state")

    def send_q_des(self, q, t):
        self._no("send_q_des")

    def send_gripper(self, role, value):
        self._no("send_gripper")

    def stop(self):
        self._no("stop")


class LeRobotDriver(_Stub):
    """LeRobot hardware (SO-100 / Koch / ALOHA-class arms with Feetech or Dynamixel servos)."""
    NAME = "lerobot"
    NEEDS = ("Needs: `pip install lerobot` and a robot config (`lerobot.common.robot_devices`), a calibrated arm on a "
             "serial port, a map from PhysicsClay's planner joints (profile model) to the motor ids, and unit "
             "conversion (servo ticks <-> rad). read_state: present_position / present_velocity (+ present_current as a "
             "torque proxy, no wrist force); send_q_des: goal_position at the control rate; stop: torque hold at the "
             "measured position. The arm needs its own profile in pps/world/profiles.py with matching joints.")


class ROS2Driver(_Stub):
    """A ROS 2 robot (ros2_control: a JointTrajectoryController or forward position controller)."""
    NAME = "ros2"
    NEEDS = ("Needs: rclpy, a running ros2_control stack, the topic names (/joint_states in, "
             "/<controller>/commands or a FollowJointTrajectory action out, an optional WrenchStamped F/T topic and a "
             "gripper action), the joint-name map to the planner model, and a clock bridge (t is the ROS time). "
             "stop() should call the controller's hold / a protective stop, never just stop publishing.")
