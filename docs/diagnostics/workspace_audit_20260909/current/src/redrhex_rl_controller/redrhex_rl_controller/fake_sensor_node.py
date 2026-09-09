"""Fake RedRhex sensors for ROS2 graph and ONNX smoke tests."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError

from . import redrhex_contract as C


def training_frame_quaternion_xyzw() -> tuple[float, float, float, float]:
    """Quaternion for the IsaacLab reset body frame (+90 degrees about X)."""

    value = math.sqrt(0.5)
    return value, 0.0, 0.0, value


class RedRhexFakeSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("redrhex_fake_sensor_node")
        self.declare_parameter("rate_hz", 125.0)
        self.declare_parameter("publish_cmd_vel", True)
        self.declare_parameter("cmd_vx", 0.0)
        self.declare_parameter("cmd_vy", 0.0)
        self.declare_parameter("cmd_wz", 0.0)
        self.declare_parameter("publish_abad_joints", False)
        self.declare_parameter("publish_damper_joints", False)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.publish_cmd_vel = bool(self.get_parameter("publish_cmd_vel").value)
        self.publish_abad_joints = bool(self.get_parameter("publish_abad_joints").value)
        self.publish_damper_joints = bool(self.get_parameter("publish_damper_joints").value)
        self.phase = 0.0

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.heartbeat_pub = self.create_publisher(Bool, "/redrhex/lowlevel_heartbeat", 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        self.phase = (self.phase + 2.0 * math.pi * C.BASE_GAIT_FREQUENCY_HZ / self.rate_hz) % (2.0 * math.pi)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        (
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
        ) = training_frame_quaternion_xyzw()
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        js = JointState()
        js.header.stamp = now
        js.name = list(C.MAIN_DRIVE_JOINT_NAMES)
        main_pos = [p + 0.05 * math.sin(self.phase) for p in C.INIT_MAIN_DRIVE_POS]
        js.position = list(main_pos)
        js.velocity = [0.05 * math.cos(self.phase)] * 6
        if self.publish_abad_joints:
            js.name += C.ABAD_JOINT_NAMES
            js.position += list(C.INIT_ABAD_POS)
            js.velocity += [0.0] * 6
        if self.publish_damper_joints:
            js.name += C.DAMPER_JOINT_NAMES
            js.position += list(C.INIT_DAMPER_POS)
            js.velocity += [0.0] * 6
        self.joint_pub.publish(js)

        if self.publish_cmd_vel:
            cmd = Twist()
            cmd.linear.x = float(self.get_parameter("cmd_vx").value)
            cmd.linear.y = float(self.get_parameter("cmd_vy").value)
            cmd.angular.z = float(self.get_parameter("cmd_wz").value)
            self.cmd_pub.publish(cmd)

        hb = Bool()
        hb.data = True
        self.heartbeat_pub.publish(hb)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RedRhexFakeSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
