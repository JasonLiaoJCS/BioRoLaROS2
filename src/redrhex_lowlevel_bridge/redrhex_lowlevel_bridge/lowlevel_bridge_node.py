"""ROS2 node bridging RedRhexMotorCommand to a replaceable low-level backend."""

from __future__ import annotations

import math
import os
import re
import signal
import threading
from collections.abc import Mapping
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from redrhex_msgs.msg import RedRhexMotorCommand, RedRhexMotorState

from .mock_bridge import MockLowLevelBridge
from .rinbo_ros_backend import RinboRosBackend
from .serial_bridge import SerialLowLevelBridge
from .sbrio_udp_bridge import SbrioUdpBridge

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError


MAIN_JOINT_NAMES_POLICY_ORDER = [
    "Revolute_15",
    "Revolute_7",
    "Revolute_12",
    "Revolute_18",
    "Revolute_23",
    "Revolute_24",
]

ABAD_JOINT_NAMES_POLICY_ORDER = [
    "Revolute_14",
    "Revolute_6",
    "Revolute_11",
    "Revolute_17",
    "Revolute_22",
    "Revolute_21",
]


# This capability is intentionally not a ROS parameter.  The canonical policy
# launch injects it only into the low-level child process after all artifact and
# safety validation has passed.  A raw ``ros2 run ... --ros-args -p ...`` or a
# parameter YAML therefore cannot mint active-hardware authority.
_CANONICAL_PARENT_PID_ENV = "REDRHEX_CANONICAL_POLICY_PARENT_PID"
_CANONICAL_PARENT_START_ENV = "REDRHEX_CANONICAL_POLICY_PARENT_START_TICKS"
_CANONICAL_NONCE_ENV = "REDRHEX_CANONICAL_POLICY_LAUNCH_NONCE"
_CANONICAL_NONCE_PATTERN = re.compile(r"[0-9a-f]{64}")


def _linux_process_start_ticks(pid: int) -> str:
    """Return Linux /proc start ticks without being confused by spaces in comm."""

    stat = Path(f"/proc/{int(pid)}/stat").read_text(encoding="utf-8")
    close_paren = stat.rfind(")")
    if close_paren < 0:
        raise ValueError(f"malformed /proc/{pid}/stat")
    # The suffix begins at field 3 (state); starttime is field 22.
    suffix_fields = stat[close_paren + 2 :].split()
    if len(suffix_fields) <= 19:
        raise ValueError(f"incomplete /proc/{pid}/stat")
    return suffix_fields[19]


def _canonical_policy_launch_provenance_reason(
    environ: Mapping[str, str] | None = None,
    *,
    process_parent_pid: int | None = None,
    parent_start_ticks: str | None = None,
    parent_cmdline_tokens: list[str] | None = None,
) -> str | None:
    """Validate the launch-only, non-ROS active-hardware capability.

    This is an operational provenance boundary against accidental/raw ROS CLI
    activation, not a privilege boundary against an attacker who can execute
    arbitrary code as the same Unix user.
    """

    values = os.environ if environ is None else environ
    expected_pid_text = str(values.get(_CANONICAL_PARENT_PID_ENV, "")).strip()
    expected_start = str(values.get(_CANONICAL_PARENT_START_ENV, "")).strip()
    nonce = str(values.get(_CANONICAL_NONCE_ENV, "")).strip()
    if not expected_pid_text or not expected_start or not nonce:
        return "canonical policy launch capability is absent"
    try:
        expected_pid = int(expected_pid_text)
    except ValueError:
        return "canonical policy launch parent PID is invalid"
    actual_parent_pid = (
        os.getppid() if process_parent_pid is None else int(process_parent_pid)
    )
    if expected_pid <= 1 or expected_pid != actual_parent_pid:
        return (
            "canonical policy launch parent PID mismatch: "
            f"expected={expected_pid}, actual={actual_parent_pid}"
        )
    if _CANONICAL_NONCE_PATTERN.fullmatch(nonce) is None:
        return "canonical policy launch nonce is malformed"
    try:
        actual_start = (
            _linux_process_start_ticks(actual_parent_pid)
            if parent_start_ticks is None
            else str(parent_start_ticks)
        )
        if parent_cmdline_tokens is None:
            cmdline_bytes = Path(
                f"/proc/{actual_parent_pid}/cmdline"
            ).read_bytes()
            tokens = [
                token.decode("utf-8", errors="replace")
                for token in cmdline_bytes.split(b"\0")
                if token
            ]
        else:
            tokens = [str(token) for token in parent_cmdline_tokens]
    except (OSError, ValueError) as exc:
        return f"cannot inspect canonical policy launch parent: {exc}"
    if actual_start != expected_start:
        return "canonical policy launch parent start time mismatch"
    try:
        launch_index = tokens.index("launch")
    except ValueError:
        return "parent process is not ros2 launch"
    expected_tail = [
        "redrhex_rl_controller",
        "redrhex_policy_bringup.launch.py",
    ]
    if tokens[launch_index + 1 : launch_index + 3] != expected_tail:
        return (
            "parent is not the canonical redrhex policy launch: "
            + " ".join(tokens)
        )
    return None


_ACTIVE_RINBO_PARAMETER_NAMES = (
    "feedback_rate_hz",
    "hardware.max_disabled_legs",
    "rinbo.allow_enable",
    "rinbo.canonical_policy_launch_authorized",
    "rinbo.command_topic",
    "rinbo.state_topic",
    "rinbo.power_state_topic",
    "rinbo.joint_state_topic",
    "rinbo.preview_topic",
    "rinbo.publish_preview",
    "rinbo.publish_when_disabled",
    "rinbo.publish_shutdown_disable",
    "rinbo.shutdown_disable_repeats",
    "rinbo.shutdown_disable_period_s",
    "rinbo.disabled_handshake_repeats",
    "rinbo.require_downstream_output_ack",
    "rinbo.downstream_output_ack_topic",
    "rinbo.downstream_output_ack_timeout_s",
    "rinbo.require_state",
    "rinbo.block_if_duplicate_command_publishers",
    "rinbo.state_timeout_s",
    "rinbo.main_pwm_per_rad_s",
    "rinbo.main_max_pwm",
    "rinbo.main_pwm_slew_rate_per_s",
    "rinbo.main_velocity_max_dt_s",
    "rinbo.main_velocity_clip_rad_s",
    "rinbo.max_main_target_velocity_rad_s",
    "rinbo.disabled_servo_control_mode",
    "rinbo.servo_control_mode",
    "rinbo.max_abad_target_position_rad",
    "rinbo.require_exact_command_contract",
    "rinbo.require_single_telemetry_publisher",
    "rinbo.expected_telemetry_publisher_node",
    "rinbo.require_monotonic_telemetry_stamp",
    "rinbo.require_abad_command_calibration",
    "rinbo.abad_command_calibrated",
    "rinbo.require_main_drive_calibration",
    "rinbo.main_drive_calibrated",
    "rinbo.require_power_state",
    "rinbo.require_power_relay",
    "rinbo.power_state_timeout_s",
    "rinbo.command_timeout_s",
    "rinbo.recovery_healthy_samples",
    "rinbo.power_bus_voltage_channel",
    "rinbo.leg_current_channels_rinbo_order",
    "rinbo.min_bus_voltage",
    "rinbo.max_bus_voltage",
    "rinbo.max_current_a",
    "rinbo.current_trip_samples",
    "rinbo.voltage_trip_samples",
    "rinbo.block_if_duplicate_upstream_publishers",
    "rinbo.expected_upstream_command_publisher_node",
    "rinbo.upstream_command_max_age_s",
)


def _validate_active_rinbo_safety_contract(
    params: Mapping[str, object],
    *,
    canonical_launch_provenance: bool = False,
) -> None:
    """Reject a writable Rinbo bridge whose core guards were weakened.

    Site YAML remains the source of calibration and mapping values, but a ROS
    parameter file must never be able to turn the hardware writer into a less
    protected mode. This validation runs before the backend creates its ROS
    command publisher.
    """

    violations: list[str] = []
    common_required_true = (
        "rinbo.publish_shutdown_disable",
        "rinbo.require_downstream_output_ack",
    )
    for name in common_required_true:
        if not bool(params[name]):
            violations.append(f"{name} must be true")
    common_exact_values = {
        "hardware.max_disabled_legs": 1,
        "rinbo.command_topic": "/motor/command",
        "rinbo.state_topic": "/motor/state",
        "rinbo.power_state_topic": "/power/state",
        "rinbo.joint_state_topic": "/joint_states",
        "rinbo.preview_topic": "/redrhex/rinbo_motor_command_preview",
        "rinbo.publish_preview": True,
        "rinbo.publish_when_disabled": False,
        "rinbo.disabled_servo_control_mode": 0,
        "rinbo.downstream_output_ack_topic": "/rinbo/motor_output_enabled",
    }
    for name, expected in common_exact_values.items():
        if params[name] != expected:
            violations.append(f"{name} must be exactly {expected!r}")
    for name, minimum in (
        ("rinbo.shutdown_disable_repeats", 8.0),
        ("rinbo.disabled_handshake_repeats", 8.0),
    ):
        try:
            value = float(params[name])
        except (TypeError, ValueError):
            violations.append(f"{name} must be numeric")
            continue
        if not math.isfinite(value) or value < minimum:
            violations.append(f"{name} must be >= {minimum:g}")
    try:
        shutdown_period_s = float(params["rinbo.shutdown_disable_period_s"])
    except (TypeError, ValueError):
        violations.append("rinbo.shutdown_disable_period_s must be numeric")
    else:
        if (
            not math.isfinite(shutdown_period_s)
            or shutdown_period_s <= 0.0
            or shutdown_period_s > 0.02
        ):
            violations.append(
                "rinbo.shutdown_disable_period_s must be in (0, 0.02]"
            )

    if bool(params["rinbo.canonical_policy_launch_authorized"]):
        violations.append(
            "rinbo.canonical_policy_launch_authorized is retired and must remain false"
        )

    if not bool(params["rinbo.allow_enable"]):
        if violations:
            raise ValueError(
                "Unsafe read-only Rinbo configuration: " + "; ".join(violations)
            )
        return

    required_true = (
        "rinbo.require_state",
        "rinbo.block_if_duplicate_command_publishers",
        "rinbo.require_exact_command_contract",
        "rinbo.require_single_telemetry_publisher",
        "rinbo.require_monotonic_telemetry_stamp",
        "rinbo.require_abad_command_calibration",
        "rinbo.abad_command_calibrated",
        "rinbo.require_main_drive_calibration",
        "rinbo.main_drive_calibrated",
        "rinbo.require_power_state",
        "rinbo.require_power_relay",
        "rinbo.block_if_duplicate_upstream_publishers",
    )
    for name in required_true:
        if not bool(params[name]):
            violations.append(f"{name} must be true")
    if not canonical_launch_provenance:
        violations.append(
            "active Rinbo output requires canonical non-ROS launch provenance"
        )

    exact_values = {
        "rinbo.main_velocity_clip_rad_s": 20.0,
        "rinbo.servo_control_mode": 2,
        "rinbo.expected_telemetry_publisher_node": "rinbo_ros2_bridge",
        "rinbo.expected_upstream_command_publisher_node": "/redrhex_rl_controller",
        "rinbo.power_bus_voltage_channel": 7,
    }
    for name, expected in exact_values.items():
        if params[name] != expected:
            violations.append(f"{name} must be exactly {expected!r}")
    if list(params["rinbo.leg_current_channels_rinbo_order"]) != [1, 2, 3, 4, 5, 6]:
        violations.append(
            "rinbo.leg_current_channels_rinbo_order must be exactly [1,2,3,4,5,6]"
        )

    upper_bounds = {
        "rinbo.downstream_output_ack_timeout_s": 0.10,
        "rinbo.state_timeout_s": 0.25,
        "rinbo.main_pwm_per_rad_s": 40.0,
        "rinbo.main_max_pwm": 80.0,
        "rinbo.main_pwm_slew_rate_per_s": 250.0,
        "rinbo.main_velocity_max_dt_s": 0.20,
        "rinbo.max_main_target_velocity_rad_s": 1.0,
        "rinbo.max_abad_target_position_rad": 0.18,
        "rinbo.power_state_timeout_s": 0.35,
        "rinbo.command_timeout_s": 0.10,
        "rinbo.max_bus_voltage": 42.0,
        "rinbo.max_current_a": 10.0,
        "rinbo.current_trip_samples": 100.0,
        "rinbo.voltage_trip_samples": 3.0,
        "rinbo.upstream_command_max_age_s": 0.10,
    }
    for name, maximum in upper_bounds.items():
        try:
            value = float(params[name])
        except (TypeError, ValueError):
            violations.append(f"{name} must be numeric")
            continue
        if not math.isfinite(value) or value <= 0.0 or value > maximum:
            violations.append(f"{name} must be in (0, {maximum:g}]")

    lower_bounds = {
        "feedback_rate_hz": 50.0,
        "rinbo.recovery_healthy_samples": 5.0,
        "rinbo.min_bus_voltage": 18.0,
    }
    for name, minimum in lower_bounds.items():
        try:
            value = float(params[name])
        except (TypeError, ValueError):
            violations.append(f"{name} must be numeric")
            continue
        if not math.isfinite(value) or value < minimum:
            violations.append(f"{name} must be >= {minimum:g}")

    if violations:
        raise ValueError(
            "Unsafe active Rinbo configuration: " + "; ".join(violations)
        )


def _reject_unarbitrated_backend_enable(backend: str, allow_enable: bool) -> None:
    if allow_enable:
        raise ValueError(
            f"{backend}.allow_enable=true is retired because it bypasses the "
            "final Rinbo motor/power arbiter; use backend=biorola_ros"
        )


class LowLevelBridgeNode(Node):
    def __init__(self, *, context: Context | None = None) -> None:
        super().__init__("redrhex_lowlevel_bridge", context=context)
        self._bridge_shutdown_complete = False
        self.declare_parameter("backend", "mock")
        mask_descriptor = ParameterDescriptor(
            description=(
                "Physical disabled-leg mask. Startup-only; update the site YAML "
                "while output is off, then restart the stack."
            ),
            read_only=True,
        )
        self.declare_parameter(
            "hardware.disabled_legs",
            Parameter.Type.STRING_ARRAY,
            descriptor=mask_descriptor,
        )
        self.declare_parameter(
            "hardware.max_disabled_legs", 1, descriptor=mask_descriptor
        )
        self.declare_parameter("mock.print_every_n", 50)
        self.declare_parameter("serial.port", "/dev/ttyUSB0")
        self.declare_parameter("serial.baudrate", 921600)
        self.declare_parameter("serial.timeout_s", 0.005)
        self.declare_parameter("serial.allow_enable", False)
        self.declare_parameter("sbrio.remote_host", "192.168.0.2")
        self.declare_parameter("sbrio.command_port", 15000)
        self.declare_parameter("sbrio.bind_host", "0.0.0.0")
        self.declare_parameter("sbrio.feedback_port", 15001)
        self.declare_parameter("sbrio.timeout_s", 0.002)
        self.declare_parameter("sbrio.heartbeat_timeout_s", 0.25)
        self.declare_parameter("sbrio.allow_enable", False)
        self.declare_parameter("sbrio.require_feedback", False)
        self.declare_parameter("rinbo.command_topic", "/motor/command")
        self.declare_parameter("rinbo.state_topic", "/motor/state")
        self.declare_parameter("rinbo.power_state_topic", "/power/state")
        self.declare_parameter("rinbo.joint_state_topic", "/joint_states")
        self.declare_parameter("rinbo.preview_topic", "/redrhex/rinbo_motor_command_preview")
        self.declare_parameter("rinbo.publish_preview", True)
        self.declare_parameter("rinbo.allow_enable", False)
        self.declare_parameter("rinbo.canonical_policy_launch_authorized", False)
        self.declare_parameter("rinbo.publish_when_disabled", False)
        self.declare_parameter("rinbo.disabled_servo_control_mode", 0)
        self.declare_parameter("rinbo.publish_shutdown_disable", True)
        self.declare_parameter("rinbo.shutdown_disable_repeats", 5)
        self.declare_parameter("rinbo.shutdown_disable_period_s", 0.02)
        self.declare_parameter("rinbo.disabled_handshake_repeats", 5)
        self.declare_parameter("rinbo.require_downstream_output_ack", True)
        self.declare_parameter(
            "rinbo.downstream_output_ack_topic", "/rinbo/motor_output_enabled"
        )
        self.declare_parameter("rinbo.downstream_output_ack_timeout_s", 0.10)
        self.declare_parameter("rinbo.require_state", True)
        self.declare_parameter("rinbo.block_if_duplicate_command_publishers", True)
        self.declare_parameter("rinbo.state_timeout_s", 0.25)
        self.declare_parameter("rinbo.main_position_counts_per_rev", 54984.83)
        self.declare_parameter("rinbo.main_pwm_per_rad_s", 40.0)
        self.declare_parameter("rinbo.main_max_pwm", 80.0)
        self.declare_parameter("rinbo.main_pwm_slew_rate_per_s", 250.0)
        self.declare_parameter("rinbo.main_encoder_zero_counts_rinbo_order", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("rinbo.main_encoder_sign_rinbo_order", [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("rinbo.main_velocity_sign_policy_order", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("rinbo.main_direction_positive_rinbo_order", [True, True, True, False, False, False])
        self.declare_parameter("rinbo.main_velocity_filter_alpha", 0.35)
        self.declare_parameter("rinbo.main_velocity_max_dt_s", 0.20)
        self.declare_parameter("rinbo.main_velocity_clip_rad_s", 80.0)
        self.declare_parameter("rinbo.max_main_target_velocity_rad_s", 30.0)
        self.declare_parameter("rinbo.abad_encoder_zero_rinbo_order", [740, 2565, 3283, 1944, 2071, 989])
        self.declare_parameter("rinbo.abad_encoder_counts_per_rad", 1000.0)
        self.declare_parameter(
            "rinbo.abad_encoder_counts_per_rad_rinbo_order",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("rinbo.abad_encoder_min", 0)
        self.declare_parameter("rinbo.abad_encoder_max", 65535)
        self.declare_parameter("rinbo.abad_sign_rinbo_order", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("rinbo.servo_control_mode", 2)
        self.declare_parameter("rinbo.max_abad_target_position_rad", 0.7)
        self.declare_parameter("rinbo.require_exact_command_contract", True)
        self.declare_parameter("rinbo.require_single_telemetry_publisher", True)
        self.declare_parameter(
            "rinbo.expected_telemetry_publisher_node", "rinbo_ros2_bridge"
        )
        self.declare_parameter("rinbo.require_monotonic_telemetry_stamp", True)
        self.declare_parameter("rinbo.publish_abad_joint_feedback", False)
        self.declare_parameter("rinbo.abad_feedback_calibrated", False)
        self.declare_parameter("rinbo.require_abad_command_calibration", False)
        self.declare_parameter("rinbo.abad_command_calibrated", False)
        self.declare_parameter("rinbo.require_main_drive_calibration", True)
        self.declare_parameter("rinbo.main_drive_calibrated", False)
        self.declare_parameter("rinbo.require_power_state", True)
        self.declare_parameter("rinbo.require_power_relay", True)
        self.declare_parameter("rinbo.power_state_timeout_s", 0.35)
        self.declare_parameter("rinbo.command_timeout_s", 0.10)
        self.declare_parameter("rinbo.recovery_healthy_samples", 5)
        self.declare_parameter("rinbo.power_bus_voltage_channel", 7)
        self.declare_parameter("rinbo.leg_current_channels_rinbo_order", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("rinbo.min_bus_voltage", 18.0)
        self.declare_parameter("rinbo.max_bus_voltage", 30.0)
        self.declare_parameter("rinbo.max_current_a", 3.0)
        self.declare_parameter("rinbo.max_bus_current_a", 30.0)
        self.declare_parameter("rinbo.stop_on_bus_current_limit", False)
        self.declare_parameter("rinbo.voltage_trip_samples", 3)
        self.declare_parameter("rinbo.current_trip_samples", 2)
        self.declare_parameter("rinbo.block_if_duplicate_upstream_publishers", True)
        self.declare_parameter(
            "rinbo.expected_upstream_command_publisher_node",
            "/redrhex_rl_controller",
        )
        self.declare_parameter("rinbo.upstream_command_max_age_s", 0.10)
        self.declare_parameter("feedback_rate_hz", 50.0)

        backend = str(self.get_parameter("backend").value)
        if backend == "mock":
            self.bridge = MockLowLevelBridge(int(self.get_parameter("mock.print_every_n").value))
        elif backend == "serial":
            _reject_unarbitrated_backend_enable(
                "serial", bool(self.get_parameter("serial.allow_enable").value)
            )
            self.bridge = SerialLowLevelBridge(
                str(self.get_parameter("serial.port").value),
                int(self.get_parameter("serial.baudrate").value),
                float(self.get_parameter("serial.timeout_s").value),
                bool(self.get_parameter("serial.allow_enable").value),
            )
        elif backend == "sbrio_udp":
            _reject_unarbitrated_backend_enable(
                "sbrio", bool(self.get_parameter("sbrio.allow_enable").value)
            )
            self.bridge = SbrioUdpBridge(
                str(self.get_parameter("sbrio.remote_host").value),
                int(self.get_parameter("sbrio.command_port").value),
                str(self.get_parameter("sbrio.bind_host").value),
                int(self.get_parameter("sbrio.feedback_port").value),
                float(self.get_parameter("sbrio.timeout_s").value),
                float(self.get_parameter("sbrio.heartbeat_timeout_s").value),
                bool(self.get_parameter("sbrio.allow_enable").value),
                bool(self.get_parameter("sbrio.require_feedback").value),
            )
        elif backend in ("rinbo_ros", "biorola_ros"):
            active_rinbo_params = {
                name: self.get_parameter(name).value
                for name in _ACTIVE_RINBO_PARAMETER_NAMES
            }
            provenance_reason = _canonical_policy_launch_provenance_reason()
            _validate_active_rinbo_safety_contract(
                active_rinbo_params,
                canonical_launch_provenance=(provenance_reason is None),
            )
            if bool(active_rinbo_params["rinbo.allow_enable"]):
                # Do not leave a reusable capability in this process's
                # environment for any subprocess it might later create.
                os.environ.pop(_CANONICAL_PARENT_PID_ENV, None)
                os.environ.pop(_CANONICAL_PARENT_START_ENV, None)
                os.environ.pop(_CANONICAL_NONCE_ENV, None)
            if bool(active_rinbo_params["rinbo.allow_enable"]):
                upstream_topic = self.resolve_topic_name(
                    "/redrhex/motor_commands"
                )
                if upstream_topic != "/redrhex/motor_commands":
                    raise ValueError(
                        "Active Rinbo hardware forbids remapping "
                        f"/redrhex/motor_commands to {upstream_topic!r}"
                    )
            if int(self.get_parameter("hardware.max_disabled_legs").value) != 1:
                raise ValueError(
                    "Rinbo hardware bringup supports at most one predeclared disabled leg"
                )
            require_downstream_output_ack = bool(
                self.get_parameter("rinbo.require_downstream_output_ack").value
            )
            if not require_downstream_output_ack:
                raise ValueError(
                    "Rinbo hardware backend requires final-arbiter output acknowledgement; "
                    "rinbo.require_downstream_output_ack cannot be disabled"
                )
            publish_abad_feedback = bool(
                self.get_parameter("rinbo.publish_abad_joint_feedback").value
            )
            if publish_abad_feedback and not bool(
                self.get_parameter("rinbo.abad_feedback_calibrated").value
            ):
                raise ValueError(
                    "rinbo.publish_abad_joint_feedback=true requires "
                    "rinbo.abad_feedback_calibrated=true; zero/counts-per-radian must be measured first"
                )
            try:
                disabled_legs = list(self.get_parameter("hardware.disabled_legs").value)
            except ParameterUninitializedException:
                disabled_legs = []
            try:
                abad_encoder_counts_per_rad_rinbo_order = list(
                    self.get_parameter(
                        "rinbo.abad_encoder_counts_per_rad_rinbo_order"
                    ).value
                )
            except ParameterUninitializedException:
                abad_encoder_counts_per_rad_rinbo_order = []
            self.bridge = RinboRosBackend(
                self,
                str(self.get_parameter("rinbo.command_topic").value),
                str(self.get_parameter("rinbo.state_topic").value),
                str(self.get_parameter("rinbo.power_state_topic").value),
                str(self.get_parameter("rinbo.joint_state_topic").value),
                str(self.get_parameter("rinbo.preview_topic").value),
                bool(self.get_parameter("rinbo.publish_preview").value),
                bool(self.get_parameter("rinbo.allow_enable").value),
                bool(self.get_parameter("rinbo.publish_when_disabled").value),
                int(self.get_parameter("rinbo.disabled_servo_control_mode").value),
                bool(self.get_parameter("rinbo.publish_shutdown_disable").value),
                int(self.get_parameter("rinbo.shutdown_disable_repeats").value),
                float(self.get_parameter("rinbo.shutdown_disable_period_s").value),
                bool(self.get_parameter("rinbo.require_state").value),
                bool(self.get_parameter("rinbo.block_if_duplicate_command_publishers").value),
                float(self.get_parameter("rinbo.state_timeout_s").value),
                float(self.get_parameter("rinbo.main_position_counts_per_rev").value),
                float(self.get_parameter("rinbo.main_pwm_per_rad_s").value),
                float(self.get_parameter("rinbo.main_max_pwm").value),
                float(self.get_parameter("rinbo.main_pwm_slew_rate_per_s").value),
                list(self.get_parameter("rinbo.main_encoder_zero_counts_rinbo_order").value),
                list(self.get_parameter("rinbo.main_encoder_sign_rinbo_order").value),
                list(self.get_parameter("rinbo.main_velocity_sign_policy_order").value),
                list(self.get_parameter("rinbo.main_direction_positive_rinbo_order").value),
                float(self.get_parameter("rinbo.main_velocity_filter_alpha").value),
                float(self.get_parameter("rinbo.main_velocity_max_dt_s").value),
                float(self.get_parameter("rinbo.main_velocity_clip_rad_s").value),
                list(self.get_parameter("rinbo.abad_encoder_zero_rinbo_order").value),
                float(self.get_parameter("rinbo.abad_encoder_counts_per_rad").value),
                int(self.get_parameter("rinbo.abad_encoder_min").value),
                int(self.get_parameter("rinbo.abad_encoder_max").value),
                list(self.get_parameter("rinbo.abad_sign_rinbo_order").value),
                int(self.get_parameter("rinbo.servo_control_mode").value),
                bool(self.get_parameter("rinbo.require_power_state").value),
                float(self.get_parameter("rinbo.power_state_timeout_s").value),
                int(self.get_parameter("rinbo.power_bus_voltage_channel").value),
                float(self.get_parameter("rinbo.min_bus_voltage").value),
                float(self.get_parameter("rinbo.max_current_a").value),
                int(self.get_parameter("rinbo.current_trip_samples").value),
                MAIN_JOINT_NAMES_POLICY_ORDER,
                disabled_legs=disabled_legs,
                max_disabled_legs=int(self.get_parameter("hardware.max_disabled_legs").value),
                leg_current_channels_rinbo_order=list(
                    self.get_parameter("rinbo.leg_current_channels_rinbo_order").value
                ),
                max_bus_voltage=float(self.get_parameter("rinbo.max_bus_voltage").value),
                max_bus_current_a=float(self.get_parameter("rinbo.max_bus_current_a").value),
                stop_on_bus_current_limit=bool(
                    self.get_parameter("rinbo.stop_on_bus_current_limit").value
                ),
                voltage_trip_samples=int(self.get_parameter("rinbo.voltage_trip_samples").value),
                require_power_relay=bool(self.get_parameter("rinbo.require_power_relay").value),
                command_timeout_s=float(self.get_parameter("rinbo.command_timeout_s").value),
                recovery_healthy_samples=int(
                    self.get_parameter("rinbo.recovery_healthy_samples").value
                ),
                publish_abad_joint_feedback=publish_abad_feedback,
                abad_joint_names_policy_order=ABAD_JOINT_NAMES_POLICY_ORDER,
                max_main_target_velocity_rad_s=float(
                    self.get_parameter("rinbo.max_main_target_velocity_rad_s").value
                ),
                max_abad_target_position_rad=float(
                    self.get_parameter("rinbo.max_abad_target_position_rad").value
                ),
                require_exact_command_contract=bool(
                    self.get_parameter("rinbo.require_exact_command_contract").value
                ),
                require_single_telemetry_publisher=bool(
                    self.get_parameter(
                        "rinbo.require_single_telemetry_publisher"
                    ).value
                ),
                expected_telemetry_publisher_node=str(
                    self.get_parameter(
                        "rinbo.expected_telemetry_publisher_node"
                    ).value
                ),
                require_monotonic_telemetry_stamp=bool(
                    self.get_parameter(
                        "rinbo.require_monotonic_telemetry_stamp"
                    ).value
                ),
                require_abad_command_calibration=bool(
                    self.get_parameter(
                        "rinbo.require_abad_command_calibration"
                    ).value
                ),
                abad_command_calibrated=bool(
                    self.get_parameter("rinbo.abad_command_calibrated").value
                ),
                disabled_handshake_repeats=int(
                    self.get_parameter("rinbo.disabled_handshake_repeats").value
                ),
                require_downstream_output_ack=require_downstream_output_ack,
                downstream_output_ack_topic=str(
                    self.get_parameter("rinbo.downstream_output_ack_topic").value
                ),
                downstream_output_ack_timeout_s=float(
                    self.get_parameter("rinbo.downstream_output_ack_timeout_s").value
                ),
                abad_encoder_counts_per_rad_rinbo_order=(
                    abad_encoder_counts_per_rad_rinbo_order
                ),
                require_main_drive_calibration=bool(
                    self.get_parameter("rinbo.require_main_drive_calibration").value
                ),
                main_drive_calibrated=bool(
                    self.get_parameter("rinbo.main_drive_calibrated").value
                ),
            )
        else:
            raise ValueError(
                f"Unknown low-level backend '{backend}'. Expected mock, serial, sbrio_udp, rinbo_ros, or biorola_ros."
            )
        self.backend = backend
        self.block_if_duplicate_upstream_publishers = bool(
            self.get_parameter("rinbo.block_if_duplicate_upstream_publishers").value
        )
        self.expected_upstream_command_publisher_node = str(
            self.get_parameter(
                "rinbo.expected_upstream_command_publisher_node"
            ).value
        ).strip()
        self.require_expected_upstream_command_publisher = bool(
            backend in ("rinbo_ros", "biorola_ros")
            and getattr(self.bridge, "allow_enable", False)
        )
        self.upstream_command_max_age_s = float(
            self.get_parameter("rinbo.upstream_command_max_age_s").value
        )
        if (
            not math.isfinite(self.upstream_command_max_age_s)
            or self.upstream_command_max_age_s <= 0.0
        ):
            raise ValueError("rinbo.upstream_command_max_age_s must be positive and finite")
        self.last_upstream_command_stamp_ns: int | None = None
        self.last_upstream_guard_reason = ""
        self.upstream_publisher_diagnostic = "not checked"
        self.bridge.connect()

        self.create_subscription(RedRhexMotorCommand, "/redrhex/motor_commands", self._on_motor_command, 10)
        self.feedback_pub = self.create_publisher(RedRhexMotorState, "/motor_feedback", 10)
        self.heartbeat_pub = self.create_publisher(Bool, "/redrhex/lowlevel_heartbeat", 10)
        self.output_enabled_pub = self.create_publisher(
            Bool, "/redrhex/lowlevel_output_enabled", 10
        )
        self.disabled_legs_pub = self.create_publisher(
            String, "/redrhex/lowlevel_disabled_legs", 10
        )
        self.diag_pub = self.create_publisher(DiagnosticArray, "/redrhex/lowlevel_diagnostics", 10)

        rate = float(self.get_parameter("feedback_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(f"Low-level bridge started with backend={backend}")

    def _on_motor_command(self, msg: RedRhexMotorCommand) -> None:
        if msg.enable or self.require_expected_upstream_command_publisher:
            guard_reason = self._upstream_command_guard_reason(msg)
            if guard_reason:
                self.last_upstream_guard_reason = guard_reason
                self.get_logger().error(
                    f"Rejecting untrusted/stale upstream command: {guard_reason}"
                )
                if hasattr(self.bridge, "external_safety_stop"):
                    self.bridge.external_safety_stop(guard_reason)
                else:
                    self._send_generic_disable()
                return
        try:
            self.bridge.send_motor_command(msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to send motor command: {exc}")

    def _upstream_command_guard_reason(self, msg: RedRhexMotorCommand) -> str | None:
        if (
            self.block_if_duplicate_upstream_publishers
            or self.require_expected_upstream_command_publisher
        ):
            try:
                infos = self.get_publishers_info_by_topic("/redrhex/motor_commands")
            except Exception as exc:
                self.upstream_publisher_diagnostic = f"graph query failed: {exc}"
                return self.upstream_publisher_diagnostic
            names = [
                f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
                for info in infos
            ]
            self.upstream_publisher_diagnostic = (
                f"count={len(infos)} names={','.join(names) if names else 'none'}"
            )
            if len(infos) != 1:
                return (
                    "expected exactly one publisher on /redrhex/motor_commands, "
                    f"got {len(infos)}: {names}"
                )
            if (
                self.require_expected_upstream_command_publisher
                and names[0] != self.expected_upstream_command_publisher_node
            ):
                return (
                    "upstream publisher must be the canonical controller "
                    f"{self.expected_upstream_command_publisher_node}, got {names[0]}"
                )
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now_ns = int(self.get_clock().now().nanoseconds)
        max_age_ns = int(self.upstream_command_max_age_s * 1_000_000_000)
        if stamp_ns <= 0:
            return "upstream command has a zero/invalid header stamp"
        if (
            self.last_upstream_command_stamp_ns is not None
            and stamp_ns <= self.last_upstream_command_stamp_ns
        ):
            return (
                "upstream command stamp is duplicate/out-of-order: "
                f"current={stamp_ns}, previous={self.last_upstream_command_stamp_ns}"
            )
        if abs(now_ns - stamp_ns) > max_age_ns:
            return (
                f"upstream command age exceeds {self.upstream_command_max_age_s:.3f}s: "
                f"delta={(now_ns - stamp_ns) * 1.0e-9:.3f}s"
            )
        self.last_upstream_command_stamp_ns = stamp_ns
        return None

    def _send_generic_disable(self) -> None:
        stop = RedRhexMotorCommand()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "redrhex_upstream_guard_stop"
        stop.joint_names = MAIN_JOINT_NAMES_POLICY_ORDER + ABAD_JOINT_NAMES_POLICY_ORDER
        stop.target_position_rad = [0.0] * 12
        stop.target_velocity_rad_s = [0.0] * 12
        stop.kp = [0.0] * 12
        stop.kd = [0.0] * 12
        stop.effort_limit_nm = [0.0] * 12
        stop.enable = False
        stop.mode = 0
        try:
            self.bridge.send_motor_command(stop)
        except Exception as exc:
            self.get_logger().error(f"Failed to send upstream guard disable: {exc}")

    def _tick(self) -> None:
        if hasattr(self.bridge, "safety_watchdog"):
            self.bridge.safety_watchdog()
        alive = self.bridge.is_alive()
        hb = Bool()
        hb.data = bool(alive)
        self.heartbeat_pub.publish(hb)
        output_status = Bool()
        if hasattr(self.bridge, "actual_output_enabled"):
            output_status.data = bool(self.bridge.actual_output_enabled())
        else:
            output_status.data = bool(
                getattr(self.bridge, "last_command_was_enabled", False)
            )
        self.output_enabled_pub.publish(output_status)
        disabled_legs_status = String()
        disabled_legs_status.data = ",".join(
            str(name) for name in getattr(self.bridge, "disabled_legs", [])
        )
        self.disabled_legs_pub.publish(disabled_legs_status)

        state = self.bridge.read_motor_state()
        if state is not None:
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = "redrhex_base"
            self.feedback_pub.publish(state)

        status = DiagnosticStatus()
        status.name = "redrhex_lowlevel_bridge"
        status.hardware_id = self.backend
        status.level = DiagnosticStatus.OK if alive else DiagnosticStatus.ERROR
        status.message = "alive" if alive else "not alive"
        disabled_legs = list(getattr(self.bridge, "disabled_legs", []))
        if alive and disabled_legs:
            status.level = DiagnosticStatus.WARN
            status.message = f"DEGRADED MODE: disabled legs {','.join(disabled_legs)}"
        status.values = [KeyValue(key="backend", value=self.backend)]
        status.values.append(
            KeyValue(key="lowlevel_output_enabled", value=str(output_status.data))
        )
        status.values.append(
            KeyValue(
                key="upstream_motor_command_publishers",
                value=self.upstream_publisher_diagnostic,
            )
        )
        status.values.append(
            KeyValue(
                key="upstream_guard_reason",
                value=self.last_upstream_guard_reason or "none",
            )
        )
        if hasattr(self.bridge, "diagnostic_values"):
            status.values.extend(
                KeyValue(key=key, value=value) for key, value in self.bridge.diagnostic_values().items()
            )
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)

    def destroy_node(self) -> bool:
        if not self._bridge_shutdown_complete:
            try:
                self.bridge.shutdown()
            except Exception as exc:
                # Shutdown is a best-effort safety path and must never turn
                # Ctrl-C into an uncaught RCLError traceback. RinboRosBackend
                # has its own fresh-context fallback before reaching here.
                try:
                    self.get_logger().error(
                        f"Low-level bridge shutdown did not complete cleanly: {exc}"
                    )
                except Exception:
                    pass
            finally:
                self._bridge_shutdown_complete = True
        return super().destroy_node()


def main(args=None) -> None:
    # Keep ROS's default SIGINT handler from invalidating the context before
    # the all-disabled shutdown train is sent.  The private context also keeps
    # an unrelated/default rclpy.shutdown() call from tearing this writer down.
    context = Context()
    rclpy.init(
        args=args,
        context=context,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    stop_requested = threading.Event()
    previous_handlers: dict[int, object] = {}

    def _request_stop(_signum, _frame) -> None:
        stop_requested.set()

    if threading.current_thread() is threading.main_thread():
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, _request_stop)

    node: LowLevelBridgeNode | None = None
    executor: SingleThreadedExecutor | None = None
    try:
        node = LowLevelBridgeNode(context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while context.ok() and not stop_requested.is_set():
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=0.5)
            except Exception:
                pass
        if context.ok():
            context.shutdown()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
