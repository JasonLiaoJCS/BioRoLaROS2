"""Fail-closed manual command helper.

The former motion modes were retired because they were not a complete,
calibration-backed Main-drive mapping workflow.  Only joint listing and an
all-disabled stop stream remain callable.
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from redrhex_msgs.msg import RedRhexMotorCommand

from . import redrhex_contract as C

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError


FULL_SIM2REAL_MANUAL = "/home/jetson/rinbo_ros_ws/docs/redrhex_sim2real_sbrio.md"
RETIRED_MOTION_MODES = {
    "init-stand",
    "single-abad",
    "single-main-velocity",
    "all-abad",
    "all-main-velocity",
}


def retired_motion_message() -> str:
    return (
        "Manual motor enable/motion modes are retired and no ROS publisher was created. "
        "The safe Main-drive mapping CLI is not implemented yet; do not use this helper "
        f"to calibrate or move hardware. Follow {FULL_SIM2REAL_MANUAL}."
    )


def _base_command(enable: bool, mode: int) -> RedRhexMotorCommand:
    msg = RedRhexMotorCommand()
    msg.header.frame_id = "redrhex_base"
    msg.joint_names = C.MAIN_DRIVE_JOINT_NAMES + C.ABAD_JOINT_NAMES
    msg.target_position_rad = list(C.INIT_MAIN_DRIVE_POS) + list(C.INIT_ABAD_POS)
    msg.target_velocity_rad_s = [0.0] * 12
    msg.kp = [0.0] * 6 + [20.0] * 6
    msg.kd = [1.0] * 6 + [1.0] * 6
    msg.effort_limit_nm = [20.0] * 6 + [3.0] * 6
    msg.enable = bool(enable)
    msg.mode = int(mode)
    return msg


class MotorCommandTool(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("redrhex_motor_command_tool")
        self.args = args
        self.lowlevel_heartbeat = False
        self.pub = self.create_publisher(RedRhexMotorCommand, args.topic, 10)
        self.create_subscription(Bool, args.heartbeat_topic, self._on_heartbeat, 10)

    def _on_heartbeat(self, msg: Bool) -> None:
        self.lowlevel_heartbeat = bool(msg.data)

    def build_command(self) -> RedRhexMotorCommand:
        msg = _base_command(enable=self.args.enable, mode=self.args.mode_id)
        if self.args.mode == "disable":
            msg.enable = False
            msg.kp = [0.0] * 12
            msg.kd = [0.0] * 12
            msg.target_velocity_rad_s = [0.0] * 12
        elif self.args.mode == "init-stand":
            pass
        elif self.args.mode == "single-abad":
            idx = self.args.index
            if idx < 0 or idx >= 6:
                raise ValueError("--index must be 0..5 for single-abad")
            msg.target_position_rad[6 + idx] = float(self.args.position)
            msg.kp[6 + idx] = float(self.args.kp)
            msg.kd[6 + idx] = float(self.args.kd)
            msg.effort_limit_nm[6 + idx] = float(self.args.effort_limit)
        elif self.args.mode == "single-main-velocity":
            idx = self.args.index
            if idx < 0 or idx >= 6:
                raise ValueError("--index must be 0..5 for single-main-velocity")
            msg.kp[idx] = 0.0
            msg.kd[idx] = float(self.args.kd)
            msg.target_velocity_rad_s[idx] = float(self.args.velocity)
            msg.effort_limit_nm[idx] = float(self.args.effort_limit)
        elif self.args.mode == "all-abad":
            pos = float(self.args.position)
            for i in range(6):
                msg.target_position_rad[6 + i] = pos
                msg.kp[6 + i] = float(self.args.kp)
                msg.kd[6 + i] = float(self.args.kd)
                msg.effort_limit_nm[6 + i] = float(self.args.effort_limit)
        elif self.args.mode == "all-main-velocity":
            vel = float(self.args.velocity)
            for i in range(6):
                msg.kp[i] = 0.0
                msg.kd[i] = float(self.args.kd)
                msg.target_velocity_rad_s[i] = vel
                msg.effort_limit_nm[i] = float(self.args.effort_limit)
        else:
            raise ValueError(f"Unsupported mode {self.args.mode}")
        return msg

    @staticmethod
    def summarize_command(msg: RedRhexMotorCommand) -> dict:
        rows = []
        for idx, name in enumerate(msg.joint_names):
            rows.append(
                {
                    "index": idx,
                    "joint": name,
                    "pos_rad": round(float(msg.target_position_rad[idx]), 5),
                    "vel_rad_s": round(float(msg.target_velocity_rad_s[idx]), 5),
                    "kp": round(float(msg.kp[idx]), 5),
                    "kd": round(float(msg.kd[idx]), 5),
                    "effort_nm": round(float(msg.effort_limit_nm[idx]), 5),
                }
            )
        return {
            "enable": bool(msg.enable),
            "mode": int(msg.mode),
            "joint_count": len(msg.joint_names),
            "joints": rows,
        }

    def run(self) -> None:
        msg = self.build_command()
        if self.args.dry_run:
            print(json.dumps(self.summarize_command(msg), indent=2))
            return
        self.wait_until_ready(msg.enable)

        period = 1.0 / max(float(self.args.rate_hz), 1.0)
        end_time = time.monotonic() + max(float(self.args.duration), period)
        self.get_logger().warn(
            f"Publishing manual command mode={self.args.mode} enable={msg.enable} "
            f"duration={self.args.duration:.2f}s. Keep E-stop in hand."
        )
        while rclpy.ok() and time.monotonic() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def wait_until_ready(self, enabled: bool) -> None:
        self._wait_for_motor_command_subscriber()
        if enabled and not self.args.skip_heartbeat_check:
            deadline = time.monotonic() + max(float(self.args.heartbeat_timeout_s), 0.0)
            while rclpy.ok() and time.monotonic() < deadline:
                if self.lowlevel_heartbeat:
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            raise RuntimeError(
                f"Refusing enabled command because {self.args.heartbeat_topic} did not become true. "
                "Start/fix redrhex_lowlevel_bridge first."
            )

    def _wait_for_motor_command_subscriber(self) -> None:
        deadline = time.monotonic() + max(float(self.args.wait_for_subscriber_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pub.get_subscription_count() == 0 and not self.args.allow_no_subscriber:
            raise RuntimeError(
                f"No subscriber on {self.args.topic}. Start redrhex_lowlevel_bridge before manual motor tests."
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fail-closed RedRHex helper. Motion/enable modes are retired; "
            "only list-joints and an all-disabled stop stream remain."
        )
    )
    parser.add_argument(
        "mode",
        choices=[
            "list-joints",
            "disable",
            "init-stand",
            "single-abad",
            "single-main-velocity",
            "all-abad",
            "all-main-velocity",
        ],
    )
    parser.add_argument(
        "--enable",
        action="store_true",
        help="Retired: any use exits nonzero before creating a ROS publisher.",
    )
    parser.add_argument(
        "--confirm-risk",
        action="store_true",
        help="Required together with --enable. Confirms you have E-stop and the robot is safe to move.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the command JSON and do not publish.")
    parser.add_argument("--topic", default="/redrhex/motor_commands")
    parser.add_argument("--heartbeat-topic", default="/redrhex/lowlevel_heartbeat")
    parser.add_argument("--wait-for-subscriber-s", type=float, default=2.0)
    parser.add_argument(
        "--allow-no-subscriber",
        action="store_true",
        help="Publish even if no low-level bridge subscriber is visible. Not recommended on hardware.",
    )
    parser.add_argument(
        "--skip-heartbeat-check",
        action="store_true",
        help="Allow enabled commands without /redrhex/lowlevel_heartbeat=true. Not recommended on hardware.",
    )
    parser.add_argument("--heartbeat-timeout-s", type=float, default=2.0)
    parser.add_argument("--index", type=int, default=0, help="Leg/joint index 0..5 in policy order.")
    parser.add_argument("--position", type=float, default=0.0, help="Target ABAD position in rad.")
    parser.add_argument("--velocity", type=float, default=0.3, help="Target main-drive velocity in rad/s.")
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--effort-limit", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--mode-id", type=int, default=2)
    return parser


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.mode == "list-joints":
        for idx, name in enumerate(C.MAIN_DRIVE_JOINT_NAMES + C.ABAD_JOINT_NAMES):
            print(f"{idx:02d}: {name}")
        print("Damper joints are simulated spring legs only on real hardware; no motor command is sent.")
        return
    if args.enable or args.mode in RETIRED_MOTION_MODES:
        raise SystemExit(retired_motion_message())
    if args.duration > 10.0:
        raise SystemExit("Refusing duration > 10 s in manual tool.")

    rclpy.init()
    node = MotorCommandTool(args)
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError) as exc:
        raise SystemExit(
            "Disabled command stream was interrupted before verified completion; "
            "motor output state is UNKNOWN. Use the physical E-stop or power cut "
            "and verify fresh output-disabled feedback."
        ) from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
