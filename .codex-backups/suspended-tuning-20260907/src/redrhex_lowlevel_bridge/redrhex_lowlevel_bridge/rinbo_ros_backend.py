"""Adapter backend for JasonLiaoJCS/BioRoLaROS2 rinbo_msgs.

This backend is for the existing BioRoLaROS2/RhexROS2 stack:
  /motor/command  rinbo_msgs/msg/MotorCmdStamped
  /motor/state    rinbo_msgs/msg/MotorStateStamped

It also publishes /joint_states from the six main-drive encoders so the RL
controller can build IsaacLab-compatible observations.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from sensor_msgs.msg import JointState

from redrhex_msgs.msg import RedRhexMotorState

from .bridge_base import LowLevelBridgeBase


@dataclass(frozen=True)
class RinboLegMapping:
    rinbo_field: str
    policy_index: int
    is_left: bool


class RinboRosBackend(LowLevelBridgeBase):
    """ROS adapter for the existing BioRoLaROS2/RhexROS2 bridge.

    The rinbo bridge expects PWM-like main-drive commands in LegCmd.voltage /
    LegCmd.direction. ABAD commands are sent as ServoCmd.position_encoder.
    """

    POLICY_TO_RINBO_LEGS = [
        RinboLegMapping("r1", 0, False),
        RinboLegMapping("r2", 1, False),
        RinboLegMapping("r3", 2, False),
        RinboLegMapping("l1", 3, True),
        RinboLegMapping("l2", 4, True),
        RinboLegMapping("l3", 5, True),
    ]
    RINBO_LEG_ORDER = ["l1", "l2", "l3", "r1", "r2", "r3"]
    RINBO_PHYSICAL_LEG_NAMES = ["L1", "L2", "L3", "R1", "R2", "R3"]
    RINBO_SERVO_ORDER = ["sl1", "sl2", "sl3", "sr1", "sr2", "sr3"]
    # Policy ABAD order is RF, RM, RR, LF, LM, LR.
    POLICY_ABAD_INDEX_BY_RINBO_SERVO = [3, 4, 5, 0, 1, 2]

    def __init__(
        self,
        node,
        command_topic: str,
        state_topic: str,
        power_state_topic: str,
        joint_state_topic: str,
        preview_topic: str,
        publish_preview: bool,
        allow_enable: bool,
        publish_when_disabled: bool,
        disabled_servo_control_mode: int,
        publish_shutdown_disable: bool,
        shutdown_disable_repeats: int,
        shutdown_disable_period_s: float,
        require_state: bool,
        block_if_duplicate_command_publishers: bool,
        state_timeout_s: float,
        main_position_counts_per_rev: float,
        main_pwm_per_rad_s: float,
        main_max_pwm: float,
        main_pwm_slew_rate_per_s: float,
        main_encoder_zero_counts_rinbo_order: list[float],
        main_encoder_sign_rinbo_order: list[float],
        main_velocity_sign_policy_order: list[float],
        main_direction_positive_rinbo_order: list[bool],
        main_velocity_filter_alpha: float,
        main_velocity_max_dt_s: float,
        main_velocity_clip_rad_s: float,
        abad_encoder_zero_rinbo_order: list[int],
        abad_encoder_counts_per_rad: float,
        abad_encoder_min: int,
        abad_encoder_max: int,
        abad_sign_rinbo_order: list[float],
        servo_control_mode: int,
        require_power_state: bool,
        power_state_timeout_s: float,
        power_bus_voltage_channel: int,
        min_bus_voltage: float,
        max_current_a: float,
        current_trip_samples: int,
        main_joint_names_policy_order: list[str],
        disabled_legs: list[str] | None = None,
        max_disabled_legs: int = 1,
        leg_current_channels_rinbo_order: list[int] | None = None,
        max_bus_voltage: float = 30.0,
        max_bus_current_a: float = 30.0,
        stop_on_bus_current_limit: bool = False,
        voltage_trip_samples: int = 3,
        require_power_relay: bool = True,
        command_timeout_s: float = 0.10,
        recovery_healthy_samples: int = 5,
        publish_abad_joint_feedback: bool = False,
        abad_joint_names_policy_order: list[str] | None = None,
        max_main_target_velocity_rad_s: float = 30.0,
        max_abad_target_position_rad: float = 0.7,
        require_exact_command_contract: bool = False,
        require_single_telemetry_publisher: bool = False,
        expected_telemetry_publisher_node: str = "rinbo_ros2_bridge",
        require_monotonic_telemetry_stamp: bool = False,
        require_abad_command_calibration: bool = False,
        abad_command_calibrated: bool = False,
        disabled_handshake_repeats: int = 5,
        require_downstream_output_ack: bool = False,
        downstream_output_ack_topic: str = "/rinbo/motor_output_enabled",
        downstream_output_ack_timeout_s: float = 0.10,
        abad_encoder_counts_per_rad_rinbo_order: list[float] | None = None,
        require_main_drive_calibration: bool = False,
        main_drive_calibrated: bool = False,
    ) -> None:
        self.node = node
        self.command_topic = command_topic
        self.state_topic = state_topic
        self.power_state_topic = power_state_topic
        self.joint_state_topic = joint_state_topic
        self.preview_topic = preview_topic
        if self.command_topic.rstrip("/") == self.preview_topic.rstrip("/"):
            raise ValueError(
                "rinbo.preview_topic must not resolve to rinbo.command_topic"
            )
        self.publish_preview = bool(publish_preview)
        self.allow_enable = bool(allow_enable)
        self.publish_when_disabled = bool(publish_when_disabled)
        self.disabled_servo_control_mode = int(disabled_servo_control_mode)
        self.publish_shutdown_disable = bool(publish_shutdown_disable)
        self.shutdown_disable_repeats = int(shutdown_disable_repeats)
        self.shutdown_disable_period_s = float(shutdown_disable_period_s)
        self.require_state = bool(require_state)
        self.block_if_duplicate_command_publishers = bool(block_if_duplicate_command_publishers)
        self.state_timeout_s = float(state_timeout_s)
        self.main_position_counts_per_rev = float(main_position_counts_per_rev)
        if not math.isfinite(self.main_position_counts_per_rev) or self.main_position_counts_per_rev <= 0.0:
            raise ValueError("main_position_counts_per_rev must be finite and positive")
        self.main_rad_per_count = 2.0 * math.pi / self.main_position_counts_per_rev
        if not math.isfinite(self.main_rad_per_count):
            raise ValueError("main_position_counts_per_rev produces a non-finite radians-per-count scale")
        self.main_pwm_per_rad_s = float(main_pwm_per_rad_s)
        self.main_max_pwm = float(main_max_pwm)
        self.main_pwm_slew_rate_per_s = float(main_pwm_slew_rate_per_s)
        self.main_encoder_zero_counts_rinbo_order = [float(x) for x in main_encoder_zero_counts_rinbo_order]
        self.main_encoder_sign_rinbo_order = [float(x) for x in main_encoder_sign_rinbo_order]
        self.main_velocity_sign_policy_order = [float(x) for x in main_velocity_sign_policy_order]
        self.main_direction_positive_rinbo_order = [bool(x) for x in main_direction_positive_rinbo_order]
        self.main_velocity_filter_alpha = float(main_velocity_filter_alpha)
        self.main_velocity_max_dt_s = float(main_velocity_max_dt_s)
        self.main_velocity_clip_rad_s = float(main_velocity_clip_rad_s)
        self.abad_encoder_zero_rinbo_order = [int(x) for x in abad_encoder_zero_rinbo_order]
        self.abad_encoder_counts_per_rad = float(abad_encoder_counts_per_rad)
        per_servo_abad_scales = [
            float(x) for x in (abad_encoder_counts_per_rad_rinbo_order or [])
        ]
        if per_servo_abad_scales:
            self.abad_encoder_counts_per_rad_rinbo_order = per_servo_abad_scales
            self.abad_encoder_scale_source = "per-servo"
        else:
            self.abad_encoder_counts_per_rad_rinbo_order = [
                self.abad_encoder_counts_per_rad
            ] * 6
            self.abad_encoder_scale_source = "legacy-scalar"
        self.abad_encoder_min = int(abad_encoder_min)
        self.abad_encoder_max = int(abad_encoder_max)
        self.abad_sign_rinbo_order = [float(x) for x in abad_sign_rinbo_order]
        self.servo_control_mode = int(servo_control_mode)
        self.require_power_state = bool(require_power_state)
        self.power_state_timeout_s = float(power_state_timeout_s)
        self.power_bus_voltage_channel = int(power_bus_voltage_channel)
        self.min_bus_voltage = float(min_bus_voltage)
        self.max_current_a = float(max_current_a)
        self.current_trip_samples = int(current_trip_samples)
        self.main_joint_names_policy_order = list(main_joint_names_policy_order)
        self.max_disabled_legs = int(max_disabled_legs)
        self.disabled_legs = self._normalize_disabled_legs(disabled_legs or [])
        self.disabled_leg_fields = {
            self.RINBO_LEG_ORDER[self.RINBO_PHYSICAL_LEG_NAMES.index(name)] for name in self.disabled_legs
        }
        self.leg_current_channels_rinbo_order = list(
            leg_current_channels_rinbo_order if leg_current_channels_rinbo_order is not None else [1, 2, 3, 4, 5, 6]
        )
        self.max_bus_voltage = float(max_bus_voltage)
        self.max_bus_current_a = float(max_bus_current_a)
        self.stop_on_bus_current_limit = bool(stop_on_bus_current_limit)
        self.voltage_trip_samples = int(voltage_trip_samples)
        self.require_power_relay = bool(require_power_relay)
        self.command_timeout_s = float(command_timeout_s)
        self.recovery_healthy_samples = int(recovery_healthy_samples)
        self.publish_abad_joint_feedback = bool(publish_abad_joint_feedback)
        self.abad_joint_names_policy_order = list(abad_joint_names_policy_order or [])
        self.max_main_target_velocity_rad_s = float(max_main_target_velocity_rad_s)
        self.max_abad_target_position_rad = float(max_abad_target_position_rad)
        self.require_exact_command_contract = bool(require_exact_command_contract)
        self.require_single_telemetry_publisher = bool(
            require_single_telemetry_publisher
        )
        self.expected_telemetry_publisher_node = str(
            expected_telemetry_publisher_node
        ).strip().lstrip("/")
        if (
            self.require_single_telemetry_publisher
            and not self.expected_telemetry_publisher_node
        ):
            raise ValueError("expected_telemetry_publisher_node cannot be empty")
        self.require_monotonic_telemetry_stamp = bool(
            require_monotonic_telemetry_stamp
        )
        self.require_abad_command_calibration = bool(
            require_abad_command_calibration
        )
        self.abad_command_calibrated = bool(abad_command_calibrated)
        self.require_main_drive_calibration = bool(require_main_drive_calibration)
        self.main_drive_calibrated = bool(main_drive_calibrated)
        self.disabled_handshake_repeats = int(disabled_handshake_repeats)
        self.require_downstream_output_ack = bool(require_downstream_output_ack)
        self.downstream_output_ack_topic = str(downstream_output_ack_topic).strip()
        self.downstream_output_ack_timeout_s = float(
            downstream_output_ack_timeout_s
        )
        self._rinbo_leg_index_by_field = {field: idx for idx, field in enumerate(self.RINBO_LEG_ORDER)}

        if len(self.main_encoder_zero_counts_rinbo_order) != 6:
            raise ValueError("main_encoder_zero_counts_rinbo_order must have length 6")
        if len(self.main_encoder_sign_rinbo_order) != 6:
            raise ValueError("main_encoder_sign_rinbo_order must have length 6")
        if len(self.main_velocity_sign_policy_order) != 6:
            raise ValueError("main_velocity_sign_policy_order must have length 6")
        if len(self.main_direction_positive_rinbo_order) != 6:
            raise ValueError("main_direction_positive_rinbo_order must have length 6")
        if len(self.abad_encoder_zero_rinbo_order) != 6:
            raise ValueError("abad_encoder_zero_rinbo_order must have length 6")
        if len(self.abad_encoder_counts_per_rad_rinbo_order) != 6:
            raise ValueError(
                "abad_encoder_counts_per_rad_rinbo_order must be empty or have length 6"
            )
        if len(self.abad_sign_rinbo_order) != 6:
            raise ValueError("abad_sign_rinbo_order must have length 6")
        if len(self.main_joint_names_policy_order) != 6:
            raise ValueError("main_joint_names_policy_order must have length 6")
        if self.publish_abad_joint_feedback or self.require_exact_command_contract:
            if len(self.abad_joint_names_policy_order) != 6:
                raise ValueError(
                    "abad_joint_names_policy_order must have length 6 for the command/feedback contract"
                )
            if len(set(self.abad_joint_names_policy_order)) != 6:
                raise ValueError("abad_joint_names_policy_order must contain unique names")
            if set(self.abad_joint_names_policy_order).intersection(self.main_joint_names_policy_order):
                raise ValueError("ABAD and main joint feedback names must not overlap")
        if self.max_disabled_legs < 1 or self.max_disabled_legs > 5:
            raise ValueError("max_disabled_legs must be in [1, 5]")
        if len(self.disabled_legs) > self.max_disabled_legs:
            raise ValueError(
                f"hardware.disabled_legs has {len(self.disabled_legs)} entries, "
                f"exceeding hardware.max_disabled_legs={self.max_disabled_legs}"
            )
        if len(self.leg_current_channels_rinbo_order) != 6:
            raise ValueError("leg_current_channels_rinbo_order must have length 6")
        if any(channel < 0 or channel > 7 for channel in self.leg_current_channels_rinbo_order):
            raise ValueError("leg_current_channels_rinbo_order entries must be in [0, 7]")
        if len(set(self.leg_current_channels_rinbo_order)) != 6:
            raise ValueError("leg_current_channels_rinbo_order entries must be unique")
        if self.power_bus_voltage_channel in self.leg_current_channels_rinbo_order:
            raise ValueError("leg_current_channels_rinbo_order must not include the bus channel")
        if self.main_position_counts_per_rev <= 0.0:
            raise ValueError("main_position_counts_per_rev must be positive")
        if self.main_pwm_per_rad_s <= 0.0 or not 0.0 < self.main_max_pwm <= 80.0:
            raise ValueError(
                "main PWM conversion must be positive and main_max_pwm must be in (0, 80]"
            )
        if self.main_pwm_slew_rate_per_s <= 0.0:
            raise ValueError("main_pwm_slew_rate_per_s must be positive")
        if self.abad_encoder_counts_per_rad <= 0.0:
            raise ValueError("abad_encoder_counts_per_rad must be positive")
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in self.abad_encoder_counts_per_rad_rinbo_order
        ):
            raise ValueError(
                "abad_encoder_counts_per_rad_rinbo_order must contain only finite positive values"
            )
        if self.abad_encoder_min >= self.abad_encoder_max:
            raise ValueError("abad_encoder_min must be smaller than abad_encoder_max")
        uint32_max = (1 << 32) - 1
        if not 0 <= self.abad_encoder_min <= uint32_max or not 0 <= self.abad_encoder_max <= uint32_max:
            raise ValueError("ABAD encoder min/max must fit rinbo_msgs uint32")
        if not 0 <= self.disabled_servo_control_mode <= uint32_max:
            raise ValueError("disabled_servo_control_mode must fit rinbo_msgs uint32")
        if not 0 <= self.servo_control_mode <= uint32_max:
            raise ValueError("servo_control_mode must fit rinbo_msgs uint32")
        if not 0.0 <= self.main_velocity_filter_alpha <= 1.0:
            raise ValueError("main_velocity_filter_alpha must be in [0, 1]")
        if self.main_velocity_max_dt_s <= 0.0:
            raise ValueError("main_velocity_max_dt_s must be positive")
        if self.main_velocity_clip_rad_s <= 0.0:
            raise ValueError("main_velocity_clip_rad_s must be positive")
        if self.shutdown_disable_repeats < 0:
            raise ValueError("shutdown_disable_repeats must be non-negative")
        if self.disabled_handshake_repeats < 0:
            raise ValueError("disabled_handshake_repeats must be non-negative")
        if self.require_downstream_output_ack and not self.downstream_output_ack_topic:
            raise ValueError("downstream_output_ack_topic cannot be empty")
        if (
            not math.isfinite(self.downstream_output_ack_timeout_s)
            or self.downstream_output_ack_timeout_s <= 0.0
        ):
            raise ValueError(
                "downstream_output_ack_timeout_s must be positive and finite"
            )
        if self.shutdown_disable_period_s < 0.0:
            raise ValueError("shutdown_disable_period_s must be non-negative")
        if self.power_state_timeout_s <= 0.0:
            raise ValueError("power_state_timeout_s must be positive")
        if self.power_bus_voltage_channel < 0 or self.power_bus_voltage_channel > 7:
            raise ValueError("power_bus_voltage_channel must be 0..7")
        if self.min_bus_voltage <= 0.0:
            raise ValueError("min_bus_voltage must be positive")
        if self.max_bus_voltage <= self.min_bus_voltage:
            raise ValueError("max_bus_voltage must be greater than min_bus_voltage")
        if self.max_current_a <= 0.0:
            raise ValueError("max_current_a must be positive")
        if self.max_bus_current_a <= 0.0:
            raise ValueError("max_bus_current_a must be positive")
        if self.current_trip_samples <= 0:
            raise ValueError("current_trip_samples must be positive")
        if self.voltage_trip_samples <= 0:
            raise ValueError("voltage_trip_samples must be positive")
        if self.command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        if self.recovery_healthy_samples <= 0:
            raise ValueError("recovery_healthy_samples must be positive")
        if self.max_main_target_velocity_rad_s <= 0.0:
            raise ValueError("max_main_target_velocity_rad_s must be positive")
        if self.max_abad_target_position_rad <= 0.0:
            raise ValueError("max_abad_target_position_rad must be positive")
        if self.require_power_relay and not self.require_power_state:
            raise ValueError("require_power_relay=true requires require_power_state=true")
        finite_scalars = {
            "state_timeout_s": self.state_timeout_s,
            "main_position_counts_per_rev": self.main_position_counts_per_rev,
            "main_pwm_per_rad_s": self.main_pwm_per_rad_s,
            "main_max_pwm": self.main_max_pwm,
            "main_pwm_slew_rate_per_s": self.main_pwm_slew_rate_per_s,
            "main_velocity_filter_alpha": self.main_velocity_filter_alpha,
            "main_velocity_max_dt_s": self.main_velocity_max_dt_s,
            "main_velocity_clip_rad_s": self.main_velocity_clip_rad_s,
            "abad_encoder_counts_per_rad": self.abad_encoder_counts_per_rad,
            "shutdown_disable_period_s": self.shutdown_disable_period_s,
            "power_state_timeout_s": self.power_state_timeout_s,
            "min_bus_voltage": self.min_bus_voltage,
            "max_bus_voltage": self.max_bus_voltage,
            "max_current_a": self.max_current_a,
            "max_bus_current_a": self.max_bus_current_a,
            "command_timeout_s": self.command_timeout_s,
            "max_main_target_velocity_rad_s": self.max_main_target_velocity_rad_s,
            "max_abad_target_position_rad": self.max_abad_target_position_rad,
        }
        for name, value in finite_scalars.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        for name, values in (
            ("main_encoder_zero_counts_rinbo_order", self.main_encoder_zero_counts_rinbo_order),
            ("main_encoder_sign_rinbo_order", self.main_encoder_sign_rinbo_order),
            ("main_velocity_sign_policy_order", self.main_velocity_sign_policy_order),
            ("abad_sign_rinbo_order", self.abad_sign_rinbo_order),
        ):
            if any(not math.isfinite(value) for value in values):
                raise ValueError(f"{name} must contain only finite values")
        if self.state_timeout_s <= 0.0:
            raise ValueError("state_timeout_s must be positive")
        for name, values in (
            ("main_encoder_sign_rinbo_order", self.main_encoder_sign_rinbo_order),
            ("main_velocity_sign_policy_order", self.main_velocity_sign_policy_order),
            ("abad_sign_rinbo_order", self.abad_sign_rinbo_order),
        ):
            if any(value not in (-1.0, 1.0) for value in values):
                raise ValueError(f"{name} must contain only -1.0 or 1.0")

        self.connected = False
        self.sequence = 0
        self.last_state_time: float | None = None
        self.latest_motor_state: RedRhexMotorState | None = None
        self.latest_positions_policy = [0.0] * 6
        self.latest_velocities_policy = [0.0] * 6
        self._prev_positions_policy: list[float] | None = None
        self._prev_state_time: float | None = None
        self.latest_raw_positions_rinbo = [0.0] * 6
        self.latest_servo_positions_rinbo = list(self.abad_encoder_zero_rinbo_order)
        self.latest_abad_positions_policy = [0.0] * 6
        self.latest_abad_velocities_policy = [0.0] * 6
        self._prev_abad_positions_policy: list[float] | None = None
        self._prev_abad_state_time: float | None = None
        self.latest_power_voltages = [0.0] * 8
        self.latest_power_currents = [0.0] * 8
        self.last_power_state_time: float | None = None
        self.current_trip_count = 0
        self.bus_current_trip_count = 0
        self.undervoltage_trip_count = 0
        self.overvoltage_trip_count = 0
        self.power_state_valid = False
        self.power_relay_on = False
        self.power_healthy_count = 0
        self.last_command_was_enabled = False
        self.last_enabled_command_time: float | None = None
        self.last_preview_pwm_rinbo_order = [0.0] * 6
        self.last_raw_pwm_rinbo_order = [0.0] * 6
        self.last_pwm_rinbo_order = [0.0] * 6
        self._slewed_pwm_rinbo_order = [0.0] * 6
        self._last_pwm_publish_time: float | None = None
        self.last_abad_encoder_targets_rinbo_order = list(self.abad_encoder_zero_rinbo_order)
        self.last_actual_publish_state = "never"
        self.last_block_reason = ""
        self._last_warned_block_reason = ""
        self.safety_latched = False
        self.safety_latch_reason = ""
        self._safety_disable_repeats_remaining = 0
        self._disabled_handshake_packets_remaining = self.disabled_handshake_repeats
        self.last_rinbo_state_sequence: int | None = None
        self.last_power_state_sequence: int | None = None
        self.last_rinbo_state_stamp_ns: int | None = None
        self.last_power_state_stamp_ns: int | None = None
        self.telemetry_source_ok = {
            self.state_topic: not self.require_single_telemetry_publisher,
            self.power_state_topic: not self.require_single_telemetry_publisher,
            self.downstream_output_ack_topic: not self.require_single_telemetry_publisher,
        }
        self.telemetry_source_diagnostic = {
            self.state_topic: "not checked",
            self.power_state_topic: "not checked",
            self.downstream_output_ack_topic: "not checked",
        }
        self.downstream_output_enabled = False
        self.last_downstream_output_ack_time: float | None = None
        self.downstream_enable_epoch_time: float | None = None
        self._last_shutdown_disable_publish_count = 0
        self.shutdown_disable_status = "not requested"

    @classmethod
    def _normalize_disabled_legs(cls, values: list[str]) -> list[str]:
        normalized: list[str] = []
        for value in values:
            name = str(value).strip().upper()
            if name not in cls.RINBO_PHYSICAL_LEG_NAMES:
                raise ValueError(
                    f"Unknown hardware.disabled_legs entry {value!r}; expected L1,L2,L3,R1,R2,R3"
                )
            if name in normalized:
                raise ValueError(f"Duplicate hardware.disabled_legs entry {name!r}")
            normalized.append(name)
        return normalized

    @staticmethod
    def _sequence_is_newer(sequence: int, previous: int) -> bool:
        delta = (int(sequence) - int(previous)) & 0xFFFFFFFF
        return 0 < delta < 0x80000000

    def _accept_telemetry_sequence(self, msg, attribute: str, label: str) -> bool:
        if self.require_single_telemetry_publisher and not self.telemetry_source_ok.get(
            label, False
        ):
            reason = self.telemetry_source_diagnostic.get(
                label, f"unverified telemetry publisher on {label}"
            )
            if self.last_command_was_enabled:
                self.external_safety_stop(reason)
            else:
                self.last_block_reason = reason
                self._warn_once(reason)
            return False
        header = getattr(msg, "header", None)
        if header is None or not hasattr(header, "seq"):
            reason = f"{label} is missing the sbRIO header sequence"
            if self.last_command_was_enabled:
                self.external_safety_stop(reason)
            else:
                self.last_block_reason = reason
                self._warn_once(reason)
            return False
        sequence = int(header.seq) & 0xFFFFFFFF
        previous = getattr(self, attribute)
        if previous is not None and not self._sequence_is_newer(sequence, previous):
            reason = (
                f"{label} duplicate/out-of-order sequence: current={sequence}, "
                f"previous={previous}"
            )
            if self.last_command_was_enabled:
                self.external_safety_stop(reason)
            else:
                self.last_block_reason = reason
                self._warn_once(reason)
            return False
        if self.require_monotonic_telemetry_stamp:
            stamp = getattr(header, "stamp", None)
            stamp_ns = (
                0
                if stamp is None
                else int(getattr(stamp, "sec", 0)) * 1_000_000_000
                + int(getattr(stamp, "nanosec", 0))
            )
            stamp_attribute = (
                "last_rinbo_state_stamp_ns"
                if attribute == "last_rinbo_state_sequence"
                else "last_power_state_stamp_ns"
            )
            previous_stamp = getattr(self, stamp_attribute)
            if stamp_ns <= 0 or (
                previous_stamp is not None and stamp_ns <= previous_stamp
            ):
                reason = (
                    f"{label} zero/duplicate/out-of-order source stamp: "
                    f"current={stamp_ns}, previous={previous_stamp}"
                )
                if self.last_command_was_enabled:
                    self.external_safety_stop(reason)
                else:
                    self.last_block_reason = reason
                    self._warn_once(reason)
                return False
            setattr(self, stamp_attribute, stamp_ns)
        setattr(self, attribute, sequence)
        return True

    def connect(self) -> None:
        try:
            from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped, PowerStateStamped
            from std_msgs.msg import Bool
        except Exception as exc:  # pragma: no cover - requires external BioRoLaROS2 overlay
            raise RuntimeError(
                "rinbo_msgs is required for backend='biorola_ros'/'rinbo_ros'. Build/source BioRoLaROS2 first."
            ) from exc

        expected_resolved_topics = {
            self.command_topic: "/motor/command",
            self.state_topic: "/motor/state",
            self.power_state_topic: "/power/state",
            self.joint_state_topic: "/joint_states",
            self.preview_topic: "/redrhex/rinbo_motor_command_preview",
            self.downstream_output_ack_topic: "/rinbo/motor_output_enabled",
        }
        resolved_violations = []
        for configured, expected in expected_resolved_topics.items():
            resolved = self.node.resolve_topic_name(configured)
            if resolved != expected:
                resolved_violations.append(
                    f"{configured!r} resolves to {resolved!r}, expected {expected!r}"
                )
        if resolved_violations:
            raise RuntimeError(
                "Rinbo hardware topic remapping is forbidden: "
                + "; ".join(resolved_violations)
            )

        self.MotorCmdStamped = MotorCmdStamped
        self.MotorStateStamped = MotorStateStamped
        self.PowerStateStamped = PowerStateStamped
        self.cmd_pub = self.node.create_publisher(MotorCmdStamped, self.command_topic, 10)
        self.preview_pub = self.node.create_publisher(MotorCmdStamped, self.preview_topic, 10)
        self.joint_pub = self.node.create_publisher(JointState, self.joint_state_topic, 10)
        self.state_sub = self.node.create_subscription(MotorStateStamped, self.state_topic, self._on_rinbo_state, 10)
        self.power_state_sub = self.node.create_subscription(
            PowerStateStamped, self.power_state_topic, self._on_power_state, 10
        )
        self.downstream_output_ack_sub = self.node.create_subscription(
            Bool,
            self.downstream_output_ack_topic,
            self._on_downstream_output_ack,
            10,
        )
        self.connected = True
        self._disabled_handshake_packets_remaining = self.disabled_handshake_repeats
        self.node.get_logger().info(
            f"Rinbo ROS backend connected: command={self.command_topic}, "
            f"state={self.state_topic}, power_state={self.power_state_topic}"
        )
        if self.disabled_legs:
            self.node.get_logger().warn(
                "DEGRADED MODE: hardware.disabled_legs="
                f"{self.disabled_legs}; final Rinbo main-drive output is forced disabled"
            )

    def send_motor_command(self, cmd) -> None:
        if not self.connected:
            raise RuntimeError("Rinbo ROS backend is not connected")
        try:
            self._validate_command(cmd)
            # Preview uses the physical MotorCmdStamped type for observability,
            # so it must remain non-actionable even if a ROS remap or unsafe
            # YAML accidentally routes the preview publisher to /motor/command.
            preview_msg = self._make_motor_cmd_msg(
                cmd, enabled=False, preview=True
            )
        except (ArithmeticError, TypeError, ValueError) as exc:
            self._block_enabled_command(
                f"invalid motor command: {exc}",
                "blocked_invalid_command",
                latch=True,
            )
            return
        enabled = bool(cmd.enable)
        if not enabled and self.last_command_was_enabled:
            # A final arbiter can be latched even when this process does not
            # know its reason.  A bounded train of fresh, all-disabled packets
            # gives it an explicit startup/stop rearm handshake without
            # continuously publishing commands during an output-blocked dry run.
            self._disabled_handshake_packets_remaining = max(
                self._disabled_handshake_packets_remaining,
                self.disabled_handshake_repeats,
            )
        if not enabled:
            self._try_clear_safety_latch()

        if self.publish_preview:
            self.preview_pub.publish(preview_msg)

        if enabled and self.safety_latched:
            self._block_enabled_command(
                self.safety_latch_reason or "low-level safety fault latched",
                "blocked_safety_latched",
                latch=False,
            )
            return
        if enabled and not self.allow_enable:
            self._block_enabled_command(
                "rinbo.allow_enable is false", "blocked_allow_enable", latch=False
            )
            return
        if (
            enabled
            and self.require_main_drive_calibration
            and not self.main_drive_calibrated
        ):
            healthy_legs = [
                name for name in self.RINBO_PHYSICAL_LEG_NAMES
                if name not in self.disabled_legs
            ]
            self._block_enabled_command(
                "main-drive zero/sign mapping is not calibrated for healthy legs "
                + ",".join(healthy_legs),
                "blocked_main_drive_uncalibrated",
                latch=False,
            )
            return
        if (
            enabled
            and self.require_abad_command_calibration
            and not self.abad_command_calibrated
        ):
            self._block_enabled_command(
                "ABAD command calibration is not acknowledged",
                "blocked_abad_command_uncalibrated",
                latch=False,
            )
            return
        if enabled and self.require_state and not self.is_alive():
            self._block_enabled_command(
                "no recent /motor/state", "blocked_no_recent_state", latch=True
            )
            return
        if enabled:
            power_ok, power_reason = self._power_ok_for_enable()
            if not power_ok:
                self._block_enabled_command(
                    power_reason,
                    "blocked_power_safety",
                    latch=self._power_fault_requires_latch(),
                )
                return
        if enabled and self.block_if_duplicate_command_publishers:
            duplicate_count, endpoint_names = self._command_publisher_count()
            if duplicate_count != 1:
                self._block_enabled_command(
                    f"expected exactly one publisher on {self.command_topic}, got "
                    f"{duplicate_count}: {endpoint_names}",
                    "blocked_duplicate_publishers",
                    latch=True,
                )
                return
        if enabled and self.cmd_pub.get_subscription_count() == 0:
            self._block_enabled_command(
                f"no subscriber on {self.command_topic}",
                "blocked_no_command_subscriber",
                latch=True,
            )
            return

        # During dry-run, avoid publishing disabled preview packets because
        # BioRoLaROS2 servo commands have no per-servo enable. If motors were
        # previously enabled, still send one disabled packet to release legs.
        if (
            not enabled
            and not self.publish_when_disabled
            and not self.last_command_was_enabled
            and self._disabled_handshake_packets_remaining <= 0
        ):
            self.last_command_was_enabled = False
            self.last_enabled_command_time = None
            self.last_pwm_rinbo_order = [0.0] * 6
            self.last_actual_publish_state = "preview_only_disabled"
            return

        try:
            msg = self._make_motor_cmd_msg(cmd, enabled=enabled, preview=False, apply_slew=True)
        except (ArithmeticError, TypeError, ValueError) as exc:
            self._block_enabled_command(
                f"invalid motor command conversion: {exc}",
                "blocked_invalid_command",
                latch=True,
            )
            return
        was_enabled = self.last_command_was_enabled
        self.cmd_pub.publish(msg)
        if not enabled and self._disabled_handshake_packets_remaining > 0:
            self._disabled_handshake_packets_remaining -= 1
        self.last_command_was_enabled = enabled
        publish_time = time.monotonic()
        self.last_enabled_command_time = publish_time if enabled else None
        if enabled and not was_enabled:
            self.downstream_enable_epoch_time = publish_time
            self.downstream_output_enabled = False
        elif not enabled:
            self.downstream_enable_epoch_time = None
        self.last_actual_publish_state = "published_enabled" if enabled else "published_disabled"

    def _make_motor_cmd_msg(self, cmd, enabled: bool, preview: bool, apply_slew: bool = False):
        if preview:
            enabled = False
        msg = self.MotorCmdStamped()
        now = self.node.get_clock().now().to_msg()
        if preview:
            seq = self.sequence
        else:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            seq = self.sequence
        msg.header.seq = seq
        msg.header.stamp = now
        msg.header.frame_id = "redrhex_preview" if preview else "redrhex_base"
        msg.servo_control_mode = self.servo_control_mode if enabled else self.disabled_servo_control_mode

        self._disable_all_legs(msg)
        self._set_main_drive_pwm(msg, cmd, enabled, preview=preview, apply_slew=apply_slew)
        if enabled:
            self._set_abad_servo_targets(msg, cmd)
        else:
            self._set_abad_neutral_targets(msg)
        self._enforce_disabled_legs(msg)
        return msg

    def _validate_command(self, cmd) -> None:
        expected_joint_names = (
            self.main_joint_names_policy_order + self.abad_joint_names_policy_order
        )
        if self.require_exact_command_contract:
            if list(getattr(cmd, "joint_names", [])) != expected_joint_names:
                raise ValueError(
                    "joint_names must exactly match policy main+ABAD order: "
                    f"{expected_joint_names}"
                )
            for field in (
                "target_position_rad",
                "target_velocity_rad_s",
                "kp",
                "kd",
                "effort_limit_nm",
            ):
                values = list(getattr(cmd, field, []))
                if len(values) != len(expected_joint_names):
                    raise ValueError(
                        f"{field} must have exactly {len(expected_joint_names)} values"
                    )
        else:
            if len(cmd.target_velocity_rad_s) < 6:
                raise ValueError("target_velocity_rad_s must contain at least 6 main-drive values")
            if len(cmd.target_position_rad) < 12:
                raise ValueError("target_position_rad must contain 6 main-drive + 6 ABAD values")
        for field in (
            "target_position_rad",
            "target_velocity_rad_s",
            "kp",
            "kd",
            "effort_limit_nm",
        ):
            for index, value in enumerate(getattr(cmd, field, [])):
                numeric = float(value)
                if not math.isfinite(numeric):
                    raise ValueError(f"non-finite {field}[{index}]={value!r}")
        for index in range(6):
            target_velocity = float(cmd.target_velocity_rad_s[index])
            if abs(target_velocity) > self.max_main_target_velocity_rad_s:
                raise ValueError(
                    f"main target velocity[{index}]={target_velocity:.6g} exceeds "
                    f"{self.max_main_target_velocity_rad_s:.6g} rad/s"
                )
            abad_position = float(cmd.target_position_rad[6 + index])
            if abs(abad_position) > self.max_abad_target_position_rad:
                raise ValueError(
                    f"ABAD target position[{index}]={abad_position:.6g} exceeds "
                    f"{self.max_abad_target_position_rad:.6g} rad"
                )
            scaled_pwm = float(cmd.target_velocity_rad_s[index]) * self.main_pwm_per_rad_s
            if not math.isfinite(scaled_pwm):
                raise ValueError(f"non-finite derived main PWM at policy index {index}")
            command_idx = 6 + index
            servo_idx = self.POLICY_ABAD_INDEX_BY_RINBO_SERVO.index(index)
            scaled_abad = (
                float(cmd.target_position_rad[command_idx])
                * self.abad_encoder_counts_per_rad_rinbo_order[servo_idx]
            )
            if not math.isfinite(scaled_abad):
                raise ValueError(f"non-finite derived ABAD target at policy index {index}")

    def read_motor_state(self):
        return self.latest_motor_state

    def is_alive(self) -> bool:
        return self._state_feedback_alive() and not self.safety_latched

    def actual_output_enabled(self) -> bool:
        if not self.last_command_was_enabled:
            return False
        if not self.require_downstream_output_ack:
            return True
        if (
            not self.telemetry_source_ok.get(self.downstream_output_ack_topic, False)
            or not self.downstream_output_enabled
            or self.last_downstream_output_ack_time is None
            or self.downstream_enable_epoch_time is None
            or self.last_downstream_output_ack_time < self.downstream_enable_epoch_time
        ):
            return False
        return (
            time.monotonic() - self.last_downstream_output_ack_time
            <= self.downstream_output_ack_timeout_s
        )

    def _state_feedback_alive(self) -> bool:
        if not self.connected:
            return False
        if not self.require_state:
            return True
        if self.last_state_time is None:
            return False
        return time.monotonic() - self.last_state_time <= self.state_timeout_s

    def shutdown(self) -> None:
        if self.connected and self.publish_shutdown_disable:
            self._last_shutdown_disable_publish_count = 0
            primary_error: Exception | None = None
            try:
                context = getattr(self.node, "context", None)
                if context is not None and not context.ok():
                    raise RuntimeError("primary rclpy context is already invalid")
                self._publish_shutdown_disable()
            except Exception as exc:
                primary_error = exc

            if (
                self._last_shutdown_disable_publish_count
                < self.shutdown_disable_repeats
            ):
                try:
                    self._release_primary_command_publisher()
                    fallback_count = (
                        self._publish_shutdown_disable_with_fresh_context()
                    )
                except Exception as fallback_exc:
                    self.shutdown_disable_status = (
                        "FAILED: primary="
                        f"{primary_error or 'incomplete train'}; "
                        f"fresh-context fallback={fallback_exc}"
                    )
                else:
                    self.shutdown_disable_status = (
                        "fresh-context fallback published "
                        f"{fallback_count}/{self.shutdown_disable_repeats} packets"
                    )
            else:
                self.shutdown_disable_status = (
                    "primary context published "
                    f"{self._last_shutdown_disable_publish_count}/"
                    f"{self.shutdown_disable_repeats} packets"
                )
        self.connected = False

    def _release_primary_command_publisher(self) -> None:
        """Remove the stale writer before creating the one-shot fallback."""

        publisher = getattr(self, "cmd_pub", None)
        if publisher is None:
            return
        try:
            self.node.destroy_publisher(publisher)
        except Exception:
            # A fully invalid context may already have removed the endpoint.
            pass
        try:
            del self.cmd_pub
        except AttributeError:
            pass

    def diagnostic_values(self) -> dict[str, str]:
        state_age = "none" if self.last_state_time is None else f"{time.monotonic() - self.last_state_time:.4f}"
        power_age = (
            "none" if self.last_power_state_time is None else f"{time.monotonic() - self.last_power_state_time:.4f}"
        )
        publisher_count, publisher_names = self._command_publisher_count()
        bus_voltage = self.latest_power_voltages[self.power_bus_voltage_channel]
        max_current = self._max_leg_current()
        bus_current = abs(self.latest_power_currents[self.power_bus_voltage_channel])
        return {
            "rinbo_command_topic": self.command_topic,
            "rinbo_state_topic": self.state_topic,
            "rinbo_power_state_topic": self.power_state_topic,
            "rinbo_preview_topic": self.preview_topic,
            "rinbo_publish_preview": str(self.publish_preview),
            "rinbo_require_state": str(self.require_state),
            "rinbo_require_power_state": str(self.require_power_state),
            "rinbo_allow_enable": str(self.allow_enable),
            "rinbo_publish_when_disabled": str(self.publish_when_disabled),
            "rinbo_disabled_handshake_remaining": str(
                self._disabled_handshake_packets_remaining
            ),
            "rinbo_require_downstream_output_ack": str(
                self.require_downstream_output_ack
            ),
            "rinbo_require_main_drive_calibration": str(
                self.require_main_drive_calibration
            ),
            "rinbo_main_drive_calibrated": str(self.main_drive_calibrated),
            "rinbo_downstream_output_ack_topic": self.downstream_output_ack_topic,
            "rinbo_downstream_output_enabled": str(self.downstream_output_enabled),
            "rinbo_actual_output_enabled": str(self.actual_output_enabled()),
            "rinbo_block_if_duplicate_command_publishers": str(self.block_if_duplicate_command_publishers),
            "rinbo_command_subscribers": str(self.cmd_pub.get_subscription_count() if hasattr(self, "cmd_pub") else 0),
            "rinbo_command_publishers": f"{publisher_count}: {publisher_names}",
            "rinbo_main_velocity_filter_alpha": f"{self.main_velocity_filter_alpha:.3f}",
            "rinbo_main_velocity_clip_rad_s": f"{self.main_velocity_clip_rad_s:.3f}",
            "rinbo_main_pwm_slew_rate_per_s": f"{self.main_pwm_slew_rate_per_s:.3f}",
            "rinbo_abad_encoder_scale_source": self.abad_encoder_scale_source,
            "rinbo_abad_encoder_counts_per_rad_legacy_scalar": f"{self.abad_encoder_counts_per_rad:.6g}",
            "rinbo_abad_encoder_counts_per_rad_sl1_sl2_sl3_sr1_sr2_sr3": ",".join(
                f"{value:.6g}"
                for value in self.abad_encoder_counts_per_rad_rinbo_order
            ),
            "rinbo_last_state_age_s": state_age,
            "rinbo_last_power_state_age_s": power_age,
            "rinbo_bus_voltage_channel": str(self.power_bus_voltage_channel),
            "rinbo_bus_voltage_v": f"{bus_voltage:.3f}",
            "rinbo_min_bus_voltage_v": f"{self.min_bus_voltage:.3f}",
            "rinbo_max_bus_voltage_v": f"{self.max_bus_voltage:.3f}",
            "rinbo_max_current_a": f"{max_current:.3f}",
            "rinbo_current_limit_a": f"{self.max_current_a:.3f}",
            "rinbo_current_trip_count": str(self.current_trip_count),
            "rinbo_bus_current_a": f"{bus_current:.3f}",
            "rinbo_bus_current_limit_a": f"{self.max_bus_current_a:.3f}",
            "rinbo_power_relay_on": str(self.power_relay_on),
            "rinbo_power_healthy_count": str(self.power_healthy_count),
            "rinbo_disabled_legs": ",".join(self.disabled_legs) if self.disabled_legs else "none",
            "rinbo_safety_latched": str(self.safety_latched),
            "rinbo_safety_latch_reason": self.safety_latch_reason,
            "rinbo_last_command_enabled": str(self.last_command_was_enabled),
            "rinbo_shutdown_disable_status": self.shutdown_disable_status,
            "rinbo_actual_publish_state": self.last_actual_publish_state,
            "rinbo_last_block_reason": self.last_block_reason,
            "rinbo_last_raw_pwm_l1_l2_l3_r1_r2_r3": ",".join(f"{x:.2f}" for x in self.last_raw_pwm_rinbo_order),
            "rinbo_last_preview_pwm_l1_l2_l3_r1_r2_r3": ",".join(f"{x:.2f}" for x in self.last_preview_pwm_rinbo_order),
            "rinbo_last_pwm_l1_l2_l3_r1_r2_r3": ",".join(f"{x:.2f}" for x in self.last_pwm_rinbo_order),
            "rinbo_last_abad_sl1_sl2_sl3_sr1_sr2_sr3": ",".join(str(x) for x in self.last_abad_encoder_targets_rinbo_order),
            "rinbo_servo_state_sl1_sl2_sl3_sr1_sr2_sr3": ",".join(str(x) for x in self.latest_servo_positions_rinbo),
            "rinbo_main_vel_policy_order_rad_s": ",".join(f"{x:.3f}" for x in self.latest_velocities_policy),
            "rinbo_publish_abad_joint_feedback": str(self.publish_abad_joint_feedback),
            "rinbo_require_abad_command_calibration": str(
                self.require_abad_command_calibration
            ),
            "rinbo_abad_command_calibrated": str(self.abad_command_calibrated),
            "rinbo_require_single_telemetry_publisher": str(
                self.require_single_telemetry_publisher
            ),
            "rinbo_expected_telemetry_publisher_node": self.expected_telemetry_publisher_node,
            "rinbo_motor_state_source": self.telemetry_source_diagnostic[
                self.state_topic
            ],
            "rinbo_power_state_source": self.telemetry_source_diagnostic[
                self.power_state_topic
            ],
            "rinbo_last_motor_state_sequence": (
                "none" if self.last_rinbo_state_sequence is None else str(self.last_rinbo_state_sequence)
            ),
            "rinbo_last_power_state_sequence": (
                "none" if self.last_power_state_sequence is None else str(self.last_power_state_sequence)
            ),
            "rinbo_last_motor_state_stamp_ns": (
                "none"
                if self.last_rinbo_state_stamp_ns is None
                else str(self.last_rinbo_state_stamp_ns)
            ),
            "rinbo_last_power_state_stamp_ns": (
                "none"
                if self.last_power_state_stamp_ns is None
                else str(self.last_power_state_stamp_ns)
            ),
            "rinbo_abad_pos_policy_order_rad": ",".join(
                f"{x:.4f}" for x in self.latest_abad_positions_policy
            ),
            "rinbo_abad_vel_policy_order_rad_s": ",".join(
                f"{x:.4f}" for x in self.latest_abad_velocities_policy
            ),
        }

    def _warn_once(self, reason: str) -> None:
        if reason != self._last_warned_block_reason:
            self.node.get_logger().warn(f"Blocking enabled BioRoLaROS2 command: {reason}")
            self._last_warned_block_reason = reason

    def _latch_safety(self, reason: str) -> None:
        newly_latched = not self.safety_latched
        if newly_latched:
            self.node.get_logger().warn(f"Latching Rinbo low-level safety fault: {reason}")
            # Recovery dwell must consist entirely of samples received after
            # this fault; pre-fault healthy telemetry cannot clear the latch.
            self.power_healthy_count = 0
        self.safety_latched = True
        if not self.safety_latch_reason:
            self.safety_latch_reason = reason
        self._safety_disable_repeats_remaining = max(
            self._safety_disable_repeats_remaining,
            max(1, self.shutdown_disable_repeats),
        )

    def external_safety_stop(self, reason: str) -> None:
        """Latch and actively disable after an upstream contract violation."""

        self._block_enabled_command(
            str(reason), "blocked_upstream_command_contract", latch=True
        )

    def _block_enabled_command(self, reason: str, publish_state: str, latch: bool) -> None:
        if latch:
            self._latch_safety(reason)
        self.last_block_reason = reason
        self._warn_once(reason)
        if self.last_command_was_enabled or latch:
            self._publish_safety_disable(reason)
        else:
            self.last_command_was_enabled = False
            self.last_enabled_command_time = None
        self.last_actual_publish_state = publish_state

    def _try_clear_safety_latch(self) -> None:
        if not self.safety_latched:
            return
        if self.require_state and not self._state_feedback_alive():
            return
        power_ok, _ = self._power_ok_for_enable(ignore_latch=True)
        if not power_ok:
            return
        if self.require_power_state and self.power_healthy_count < self.recovery_healthy_samples:
            return
        if self.block_if_duplicate_command_publishers:
            publisher_count, _ = self._command_publisher_count()
            if publisher_count != 1:
                return
        if hasattr(self, "cmd_pub") and self.cmd_pub.get_subscription_count() == 0:
            return
        old_reason = self.safety_latch_reason
        self.safety_latched = False
        self.safety_latch_reason = ""
        self._safety_disable_repeats_remaining = 0
        self._last_warned_block_reason = ""
        self.node.get_logger().warn(
            f"Cleared Rinbo safety latch after explicit disabled command and healthy telemetry: {old_reason}"
        )

    def safety_watchdog(self) -> None:
        if not self.connected:
            return
        if self.require_single_telemetry_publisher:
            self._update_telemetry_source_guard()
        if self.safety_latched and self._safety_disable_repeats_remaining > 0:
            self._publish_safety_disable(self.safety_latch_reason or "low-level safety fault latched")
        if self.require_power_state and (
            self.last_power_state_time is None
            or time.monotonic() - self.last_power_state_time > self.power_state_timeout_s
        ):
            self.power_healthy_count = 0
        if not self.last_command_was_enabled:
            return
        now = time.monotonic()
        if (
            self.require_downstream_output_ack
            and self.downstream_enable_epoch_time is not None
            and now - self.downstream_enable_epoch_time
            > self.downstream_output_ack_timeout_s
            and not self.actual_output_enabled()
        ):
            self._block_enabled_command(
                "final Rinbo arbiter did not acknowledge enabled output",
                "blocked_downstream_output_ack",
                latch=True,
            )
            return
        if self.last_enabled_command_time is None or now - self.last_enabled_command_time > self.command_timeout_s:
            age = 0.0 if self.last_enabled_command_time is None else now - self.last_enabled_command_time
            self._block_enabled_command(
                f"enabled command stream stale: {age:.3f}s",
                "blocked_command_stale",
                latch=True,
            )
            return
        if self.require_state and not self.is_alive():
            self._block_enabled_command(
                "no recent /motor/state", "blocked_no_recent_state", latch=True
            )
            return
        power_ok, power_reason = self._power_ok_for_enable()
        if not power_ok:
            self._block_enabled_command(
                power_reason,
                "blocked_power_safety",
                latch=self._power_fault_requires_latch(),
            )

    def _update_telemetry_source_guard(self) -> None:
        topics = [self.state_topic, self.power_state_topic]
        if self.require_downstream_output_ack:
            topics.append(self.downstream_output_ack_topic)
        for topic in topics:
            try:
                infos = self.node.get_publishers_info_by_topic(topic)
            except Exception as exc:
                ok = False
                diagnostic = f"{topic} graph query failed: {exc}"
            else:
                endpoints = [
                    f"{getattr(info, 'node_namespace', '').rstrip('/')}/"
                    f"{getattr(info, 'node_name', '')}".replace("//", "/")
                    for info in infos
                ]
                node_names = [
                    str(getattr(info, "node_name", "")).lstrip("/") for info in infos
                ]
                ok = (
                    len(infos) == 1
                    and node_names[0] == self.expected_telemetry_publisher_node
                )
                diagnostic = (
                    f"{topic} expected one {self.expected_telemetry_publisher_node}, "
                    f"got {len(infos)}: {endpoints}"
                )
            self.telemetry_source_ok[topic] = ok
            self.telemetry_source_diagnostic[topic] = diagnostic
            if not ok and self.last_command_was_enabled:
                self.external_safety_stop(diagnostic)

    def _on_downstream_output_ack(self, msg) -> None:
        if self.require_single_telemetry_publisher and not self.telemetry_source_ok.get(
            self.downstream_output_ack_topic, False
        ):
            self.downstream_output_enabled = False
            return
        self.downstream_output_enabled = bool(getattr(msg, "data", False))
        self.last_downstream_output_ack_time = time.monotonic()

    def _command_publisher_count(self) -> tuple[int, str]:
        try:
            infos = self.node.get_publishers_info_by_topic(self.command_topic)
        except Exception as exc:
            return -1, f"graph query failed: {exc}"
        names = []
        for info in infos:
            node_name = getattr(info, "node_name", "")
            node_namespace = getattr(info, "node_namespace", "")
            names.append(f"{node_namespace.rstrip('/')}/{node_name}".replace("//", "/") or "<unknown>")
        return len(infos), ",".join(names) if names else "none"

    def _publish_shutdown_disable(self) -> int:
        if not hasattr(self, "cmd_pub") or not hasattr(self, "MotorCmdStamped"):
            return 0
        msg = self.MotorCmdStamped()
        msg.header.frame_id = "redrhex_shutdown_disable"
        msg.servo_control_mode = self.disabled_servo_control_mode
        self._disable_all_legs(msg)
        self._set_abad_neutral_targets(msg)
        for _ in range(self.shutdown_disable_repeats):
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            msg.header.seq = self.sequence
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.cmd_pub.publish(msg)
            self._last_shutdown_disable_publish_count += 1
            if self.shutdown_disable_period_s > 0.0:
                time.sleep(self.shutdown_disable_period_s)
        return self._last_shutdown_disable_publish_count

    def _publish_shutdown_disable_with_fresh_context(self) -> int:
        """Recover the disabled train after the primary ROS context is invalid.

        The fallback publisher is created lazily, so it can never appear as a
        duplicate writer while active output is running.  It waits until the
        stale writer has disappeared from the graph before publishing.
        """

        if not hasattr(self, "MotorCmdStamped"):
            raise RuntimeError("MotorCmdStamped type was never connected")

        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions

        fallback_context = Context()
        fallback_node = None
        try:
            rclpy.init(
                context=fallback_context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            fallback_node = Node(
                "redrhex_lowlevel_bridge_shutdown_fallback",
                context=fallback_context,
                enable_rosout=False,
                start_parameter_services=False,
            )
            publisher = fallback_node.create_publisher(
                self.MotorCmdStamped, self.command_topic, 10
            )

            graph_deadline = time.monotonic() + 1.5
            graph_ready = False
            last_publisher_count = -1
            last_subscription_count = -1
            while time.monotonic() < graph_deadline:
                publisher_infos = fallback_node.get_publishers_info_by_topic(
                    self.command_topic
                )
                last_publisher_count = len(publisher_infos)
                last_subscription_count = publisher.get_subscription_count()
                if (
                    last_publisher_count == 1
                    and last_subscription_count > 0
                ):
                    graph_ready = True
                    break
                time.sleep(0.02)
            if not graph_ready:
                raise RuntimeError(
                    "did not become the sole discovered /motor/command writer "
                    "with a subscriber within 1.5s: "
                    f"publishers={last_publisher_count}, "
                    f"subscribers={last_subscription_count}"
                )

            msg = self.MotorCmdStamped()
            msg.header.frame_id = "redrhex_shutdown_disable_fallback"
            msg.servo_control_mode = self.disabled_servo_control_mode
            self._disable_all_legs(msg)
            self._set_abad_neutral_targets(msg)
            published = 0
            for _ in range(self.shutdown_disable_repeats):
                self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                msg.header.seq = self.sequence
                msg.header.stamp = fallback_node.get_clock().now().to_msg()
                publisher.publish(msg)
                published += 1
                if self.shutdown_disable_period_s > 0.0:
                    time.sleep(self.shutdown_disable_period_s)
            # Give the middleware a bounded opportunity to enqueue/transmit
            # the final sample before destroying this one-shot writer.
            time.sleep(max(0.02, self.shutdown_disable_period_s))
            return published
        finally:
            if fallback_node is not None:
                try:
                    fallback_node.destroy_node()
                except Exception:
                    pass
            if fallback_context.ok():
                fallback_context.shutdown()

    def _publish_safety_disable(self, reason: str) -> None:
        if not hasattr(self, "cmd_pub") or not hasattr(self, "MotorCmdStamped"):
            self.last_command_was_enabled = False
            self.last_enabled_command_time = None
            return
        msg = self.MotorCmdStamped()
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        msg.header.seq = self.sequence
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "redrhex_power_safety_disable"
        msg.servo_control_mode = self.disabled_servo_control_mode
        self._disable_all_legs(msg)
        self._set_abad_neutral_targets(msg)
        self.cmd_pub.publish(msg)
        if self._safety_disable_repeats_remaining > 0:
            self._safety_disable_repeats_remaining -= 1
        self._slewed_pwm_rinbo_order = [0.0] * 6
        self.last_pwm_rinbo_order = [0.0] * 6
        self.last_command_was_enabled = False
        self.last_enabled_command_time = None
        self.last_actual_publish_state = "published_power_safety_disable"
        self.last_block_reason = reason

    def _disable_all_legs(self, msg) -> None:
        for field in self.RINBO_LEG_ORDER:
            leg = getattr(msg, field)
            leg.enable = False
            leg.direction = False
            leg.voltage = 0.0
            leg.state = 0
            leg.reset_position = False

    def _set_main_drive_pwm(self, msg, cmd, enabled: bool, preview: bool, apply_slew: bool) -> None:
        raw_pwm_rinbo_order = [0.0] * 6
        for mapping in self.POLICY_TO_RINBO_LEGS:
            rinbo_idx = self._rinbo_leg_index_by_field[mapping.rinbo_field]
            if mapping.rinbo_field in self.disabled_leg_fields:
                raw_pwm_rinbo_order[rinbo_idx] = 0.0
                continue
            target_velocity = (
                float(cmd.target_velocity_rad_s[mapping.policy_index])
                * self.main_velocity_sign_policy_order[mapping.policy_index]
            )
            pwm = max(-self.main_max_pwm, min(self.main_max_pwm, target_velocity * self.main_pwm_per_rad_s))
            raw_pwm_rinbo_order[rinbo_idx] = float(pwm)

        pwm_rinbo_order = raw_pwm_rinbo_order
        if enabled and apply_slew:
            pwm_rinbo_order = self._apply_pwm_slew(raw_pwm_rinbo_order)
        elif not enabled and not preview:
            self._slewed_pwm_rinbo_order = [0.0] * 6
            self._last_pwm_publish_time = time.monotonic()

        if preview:
            self.last_preview_pwm_rinbo_order = list(pwm_rinbo_order)
        else:
            self.last_raw_pwm_rinbo_order = list(raw_pwm_rinbo_order)
            self.last_pwm_rinbo_order = list(pwm_rinbo_order)

        for mapping in self.POLICY_TO_RINBO_LEGS:
            leg = getattr(msg, mapping.rinbo_field)
            rinbo_idx = self._rinbo_leg_index_by_field[mapping.rinbo_field]
            pwm = pwm_rinbo_order[rinbo_idx]
            if enabled and mapping.rinbo_field not in self.disabled_leg_fields:
                leg.enable = True
                leg.state = 1
                leg.reset_position = False
                direction_positive = self.main_direction_positive_rinbo_order[rinbo_idx]
                leg.direction = direction_positive if pwm >= 0.0 else not direction_positive
                leg.voltage = abs(float(pwm))

    def _apply_pwm_slew(self, raw_pwm_rinbo_order: list[float]) -> list[float]:
        now = time.monotonic()
        if self._last_pwm_publish_time is None:
            dt = 0.02
        else:
            dt = max(0.0, min(0.10, now - self._last_pwm_publish_time))
        max_delta = self.main_pwm_slew_rate_per_s * dt
        slewed = [
            0.0
            if self.RINBO_LEG_ORDER[i] in self.disabled_leg_fields
            else max(
                self._slewed_pwm_rinbo_order[i] - max_delta,
                min(
                    self._slewed_pwm_rinbo_order[i] + max_delta,
                    float(raw_pwm_rinbo_order[i]),
                ),
            )
            for i in range(6)
        ]
        self._slewed_pwm_rinbo_order = slewed
        self._last_pwm_publish_time = now
        return slewed

    def _set_abad_servo_targets(self, msg, cmd) -> None:
        targets: list[int] = []
        abad_targets_policy = list(cmd.target_position_rad[6:12])
        for servo_idx, field in enumerate(self.RINBO_SERVO_ORDER):
            policy_idx = self.POLICY_ABAD_INDEX_BY_RINBO_SERVO[servo_idx]
            if self.RINBO_PHYSICAL_LEG_NAMES[servo_idx] in self.disabled_legs:
                target = int(
                    max(
                        self.abad_encoder_min,
                        min(
                            self.abad_encoder_max,
                            self.abad_encoder_zero_rinbo_order[servo_idx],
                        ),
                    )
                )
                getattr(msg, field).position_encoder = target
                targets.append(target)
                continue
            target_rad = float(abad_targets_policy[policy_idx])
            raw = (
                self.abad_encoder_zero_rinbo_order[servo_idx]
                + self.abad_sign_rinbo_order[servo_idx]
                * target_rad
                * self.abad_encoder_counts_per_rad_rinbo_order[servo_idx]
            )
            if not math.isfinite(raw):
                raise ValueError(f"non-finite derived ABAD target for {field}")
            clamped = max(float(self.abad_encoder_min), min(float(self.abad_encoder_max), raw))
            target = int(round(clamped))
            getattr(msg, field).position_encoder = target
            targets.append(target)
        self.last_abad_encoder_targets_rinbo_order = targets

    def _enforce_disabled_legs(self, msg) -> None:
        for field in self.disabled_leg_fields:
            leg = getattr(msg, field)
            leg.enable = False
            leg.direction = False
            leg.voltage = 0.0
            leg.state = 0
            leg.reset_position = False

    def _set_abad_neutral_targets(self, msg) -> None:
        targets: list[int] = []
        for servo_idx, field in enumerate(self.RINBO_SERVO_ORDER):
            target = int(
                max(
                    self.abad_encoder_min,
                    min(self.abad_encoder_max, self.abad_encoder_zero_rinbo_order[servo_idx]),
                )
            )
            getattr(msg, field).position_encoder = target
            targets.append(target)
        self.last_abad_encoder_targets_rinbo_order = targets

    def _on_rinbo_state(self, msg) -> None:
        if not self._accept_telemetry_sequence(
            msg, "last_rinbo_state_sequence", "/motor/state"
        ):
            return
        state_time = time.monotonic()
        rinbo_positions = [
            float(msg.l1.position),
            float(msg.l2.position),
            float(msg.l3.position),
            float(msg.r1.position),
            float(msg.r2.position),
            float(msg.r3.position),
        ]

        invalid_healthy_legs = [
            self.RINBO_PHYSICAL_LEG_NAMES[index]
            for index, position in enumerate(rinbo_positions)
            if self.RINBO_PHYSICAL_LEG_NAMES[index] not in self.disabled_legs
            and not math.isfinite(position)
        ]
        if invalid_healthy_legs:
            reason = (
                "non-finite /motor/state position on active leg(s): "
                + ",".join(invalid_healthy_legs)
            )
            if self.connected and hasattr(self, "cmd_pub"):
                self._block_enabled_command(reason, "blocked_invalid_motor_state", latch=True)
            else:
                self._latch_safety(reason)
                self.last_block_reason = reason
            return

        # A declared, physically isolated leg has no trustworthy encoder.
        # Always substitute its calibrated neutral instead of allowing finite
        # electrical garbage to enter the fixed-size policy observation.
        for index, position in enumerate(rinbo_positions):
            if self.RINBO_PHYSICAL_LEG_NAMES[index] in self.disabled_legs:
                rinbo_positions[index] = self.main_encoder_zero_counts_rinbo_order[index]

        servo_positions_rinbo = [
            int(getattr(msg.sl1, "position_encoder", 0)),
            int(getattr(msg.sl2, "position_encoder", 0)),
            int(getattr(msg.sl3, "position_encoder", 0)),
            int(getattr(msg.sr1, "position_encoder", 0)),
            int(getattr(msg.sr2, "position_encoder", 0)),
            int(getattr(msg.sr3, "position_encoder", 0)),
        ]
        invalid_healthy_servos = [
            self.RINBO_PHYSICAL_LEG_NAMES[index]
            for index, position in enumerate(servo_positions_rinbo)
            if self.RINBO_PHYSICAL_LEG_NAMES[index] not in self.disabled_legs
            and not self.abad_encoder_min <= position <= self.abad_encoder_max
        ]
        if invalid_healthy_servos:
            reason = (
                "out-of-range /motor/state servo encoder on active leg(s): "
                + ",".join(invalid_healthy_servos)
            )
            if self.connected and hasattr(self, "cmd_pub"):
                self._block_enabled_command(reason, "blocked_invalid_motor_state", latch=True)
            else:
                self._latch_safety(reason)
                self.last_block_reason = reason
            return
        for index in range(6):
            if self.RINBO_PHYSICAL_LEG_NAMES[index] in self.disabled_legs:
                servo_positions_rinbo[index] = int(
                    max(
                        self.abad_encoder_min,
                        min(
                            self.abad_encoder_max,
                            self.abad_encoder_zero_rinbo_order[index],
                        ),
                    )
                )
        abad_positions_policy = [0.0] * 6
        for servo_idx, raw_position in enumerate(servo_positions_rinbo):
            policy_idx = self.POLICY_ABAD_INDEX_BY_RINBO_SERVO[servo_idx]
            abad_positions_policy[policy_idx] = (
                (float(raw_position) - float(self.abad_encoder_zero_rinbo_order[servo_idx]))
                * self.abad_sign_rinbo_order[servo_idx]
                / self.abad_encoder_counts_per_rad_rinbo_order[servo_idx]
            )
        if not all(math.isfinite(value) for value in abad_positions_policy):
            reason = "non-finite derived ABAD policy state from /motor/state"
            if self.connected and hasattr(self, "cmd_pub"):
                self._block_enabled_command(reason, "blocked_invalid_motor_state", latch=True)
            else:
                self._latch_safety(reason)
                self.last_block_reason = reason
            return

        rinbo_rad = [
            (rinbo_positions[i] - self.main_encoder_zero_counts_rinbo_order[i])
            * self.main_encoder_sign_rinbo_order[i]
            * self.main_rad_per_count
            for i in range(6)
        ]
        invalid_derived_healthy_legs = [
            self.RINBO_PHYSICAL_LEG_NAMES[index]
            for index, position in enumerate(rinbo_rad)
            if self.RINBO_PHYSICAL_LEG_NAMES[index] not in self.disabled_legs
            and not math.isfinite(position)
        ]
        if invalid_derived_healthy_legs:
            reason = (
                "non-finite derived /motor/state position on active leg(s): "
                + ",".join(invalid_derived_healthy_legs)
            )
            if self.connected and hasattr(self, "cmd_pub"):
                self._block_enabled_command(reason, "blocked_invalid_motor_state", latch=True)
            else:
                self._latch_safety(reason)
                self.last_block_reason = reason
            return

        # Policy order: RF, RM, RR, LF, LM, LR.
        policy_positions = [0.0] * 6
        for mapping in self.POLICY_TO_RINBO_LEGS:
            rinbo_idx = self._rinbo_leg_index_by_field[mapping.rinbo_field]
            derived = rinbo_rad[rinbo_idx]
            policy_positions[mapping.policy_index] = (
                self.latest_positions_policy[mapping.policy_index]
                if mapping.rinbo_field in self.disabled_leg_fields and not math.isfinite(derived)
                else derived
            )
        policy_velocities = self._estimate_policy_velocities(policy_positions, state_time)
        abad_velocities_policy = self._estimate_abad_policy_velocities(
            abad_positions_policy, state_time
        )
        for mapping in self.POLICY_TO_RINBO_LEGS:
            if mapping.rinbo_field in self.disabled_leg_fields:
                policy_velocities[mapping.policy_index] = 0.0
                abad_velocities_policy[mapping.policy_index] = 0.0
        if not all(
            math.isfinite(value)
            for value in policy_positions + policy_velocities + abad_velocities_policy
        ):
            reason = "non-finite derived policy state from /motor/state"
            if self.connected and hasattr(self, "cmd_pub"):
                self._block_enabled_command(reason, "blocked_invalid_motor_state", latch=True)
            else:
                self._latch_safety(reason)
                self.last_block_reason = reason
            return

        self.last_state_time = state_time
        self.latest_raw_positions_rinbo = rinbo_positions
        self.latest_servo_positions_rinbo = servo_positions_rinbo
        self.latest_positions_policy = policy_positions
        self.latest_velocities_policy = policy_velocities
        self.latest_abad_positions_policy = abad_positions_policy
        self.latest_abad_velocities_policy = abad_velocities_policy

        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.header.frame_id = "redrhex_base"
        joint_names = list(self.main_joint_names_policy_order)
        joint_positions = [float(x) for x in policy_positions]
        joint_velocities = [float(x) for x in policy_velocities]
        if self.publish_abad_joint_feedback:
            joint_names.extend(self.abad_joint_names_policy_order)
            joint_positions.extend(float(x) for x in abad_positions_policy)
            joint_velocities.extend(float(x) for x in abad_velocities_policy)
        js.name = joint_names
        js.position = joint_positions
        js.velocity = joint_velocities
        self.joint_pub.publish(js)

        state = RedRhexMotorState()
        state.header.stamp = js.header.stamp
        state.header.frame_id = "redrhex_base"
        state.joint_names = list(self.main_joint_names_policy_order)
        state.position_rad = [float(x) for x in policy_positions]
        state.velocity_rad_s = [float(x) for x in policy_velocities]
        state.effort_nm = [0.0] * 6
        state.current_a = self._motor_currents_policy_order()
        state.temperature_c = []
        state.fault = [False] * 6
        self.latest_motor_state = state

    def _on_power_state(self, msg) -> None:
        if not self._accept_telemetry_sequence(
            msg, "last_power_state_sequence", "/power/state"
        ):
            return
        sample_time = time.monotonic()
        previous_power_state_time = self.last_power_state_time
        self.last_power_state_time = sample_time
        self.latest_power_voltages = [float(getattr(msg, f"v_{idx}", 0.0)) for idx in range(8)]
        self.latest_power_currents = [float(getattr(msg, f"i_{idx}", 0.0)) for idx in range(8)]
        self.power_relay_on = bool(getattr(msg, "power", False))
        relevant_values = [self.latest_power_voltages[self.power_bus_voltage_channel]]
        relevant_values.extend(
            self.latest_power_currents[channel]
            for index, channel in enumerate(self.leg_current_channels_rinbo_order)
            if self.RINBO_PHYSICAL_LEG_NAMES[index] not in self.disabled_legs
        )
        if self.stop_on_bus_current_limit:
            relevant_values.append(self.latest_power_currents[self.power_bus_voltage_channel])
        self.power_state_valid = all(math.isfinite(value) for value in relevant_values)
        bus_voltage = self.latest_power_voltages[self.power_bus_voltage_channel]
        bus_current = abs(self.latest_power_currents[self.power_bus_voltage_channel])
        max_current = self._max_leg_current()
        if max_current > self.max_current_a:
            self.current_trip_count += 1
        else:
            self.current_trip_count = 0
        if bus_current > self.max_bus_current_a:
            self.bus_current_trip_count += 1
        else:
            self.bus_current_trip_count = 0
        if bus_voltage < self.min_bus_voltage:
            self.undervoltage_trip_count += 1
        else:
            self.undervoltage_trip_count = 0
        if bus_voltage > self.max_bus_voltage:
            self.overvoltage_trip_count += 1
        else:
            self.overvoltage_trip_count = 0
        instant_power_safe = (
            self.power_state_valid
            and (not self.require_power_relay or self.power_relay_on)
            and self.min_bus_voltage <= bus_voltage <= self.max_bus_voltage
            and max_current <= self.max_current_a
            and (not self.stop_on_bus_current_limit or bus_current <= self.max_bus_current_a)
        )
        if instant_power_safe:
            sample_gap_too_long = (
                previous_power_state_time is None
                or sample_time - previous_power_state_time > self.power_state_timeout_s
            )
            self.power_healthy_count = 1 if sample_gap_too_long else self.power_healthy_count + 1
        else:
            self.power_healthy_count = 0
        if self.latest_motor_state is not None:
            self.latest_motor_state.current_a = self._motor_currents_policy_order()
        if self.connected and self.last_command_was_enabled:
            power_ok, reason = self._power_ok_for_enable()
            if not power_ok:
                self._block_enabled_command(
                    reason,
                    "blocked_power_safety",
                    latch=self._power_fault_requires_latch(),
                )

    def _motor_currents_policy_order(self) -> list[float]:
        if len(self.latest_power_currents) < 8:
            return []
        policy_currents = [0.0] * 6
        for mapping in self.POLICY_TO_RINBO_LEGS:
            rinbo_index = self._rinbo_leg_index_by_field[mapping.rinbo_field]
            if mapping.rinbo_field not in self.disabled_leg_fields:
                channel = self.leg_current_channels_rinbo_order[rinbo_index]
                policy_currents[mapping.policy_index] = float(
                    self.latest_power_currents[channel]
                )
        return policy_currents

    def _max_leg_current(self) -> float:
        return max(
            (
                abs(self.latest_power_currents[channel])
                for index, channel in enumerate(self.leg_current_channels_rinbo_order)
                if self.RINBO_PHYSICAL_LEG_NAMES[index] not in self.disabled_legs
            ),
            default=0.0,
        )

    def _power_fault_requires_latch(self) -> bool:
        if self.last_power_state_time is None:
            return False
        if not self.power_state_valid:
            return True
        if time.monotonic() - self.last_power_state_time > self.power_state_timeout_s:
            return True
        if self.require_power_relay and not self.power_relay_on:
            return self.last_command_was_enabled
        return (
            self.undervoltage_trip_count >= self.voltage_trip_samples
            or self.overvoltage_trip_count >= self.voltage_trip_samples
            or self.current_trip_count >= self.current_trip_samples
            or (
                self.stop_on_bus_current_limit
                and self.bus_current_trip_count >= self.current_trip_samples
            )
        )

    def _power_ok_for_enable(self, ignore_latch: bool = False) -> tuple[bool, str]:
        if self.safety_latched and not ignore_latch:
            return False, self.safety_latch_reason or "low-level safety fault latched"
        if not self.require_power_state:
            return True, ""
        if self.last_power_state_time is None:
            return False, f"waiting for {self.power_state_topic}"
        age = time.monotonic() - self.last_power_state_time
        if age > self.power_state_timeout_s:
            return False, f"{self.power_state_topic} stale: {age:.3f}s"
        if not self.power_state_valid:
            return False, f"invalid NaN/Inf in {self.power_state_topic}"
        if self.require_power_relay and not self.power_relay_on:
            return False, "power relay is off"
        bus_voltage = self.latest_power_voltages[self.power_bus_voltage_channel]
        if bus_voltage < self.min_bus_voltage:
            return (
                False,
                f"bus voltage low on ch{self.power_bus_voltage_channel}: "
                f"{bus_voltage:.2f}V < {self.min_bus_voltage:.2f}V "
                f"({self.undervoltage_trip_count}/{self.voltage_trip_samples} samples)",
            )
        if bus_voltage > self.max_bus_voltage:
            return (
                False,
                f"bus voltage high on ch{self.power_bus_voltage_channel}: "
                f"{bus_voltage:.2f}V > {self.max_bus_voltage:.2f}V "
                f"({self.overvoltage_trip_count}/{self.voltage_trip_samples} samples)",
            )
        max_current = self._max_leg_current()
        if max_current > self.max_current_a:
            return (
                False,
                f"current high for {self.current_trip_count} samples: "
                f"{max_current:.2f}A > {self.max_current_a:.2f}A",
            )
        if self.stop_on_bus_current_limit:
            bus_current = abs(self.latest_power_currents[self.power_bus_voltage_channel])
            if bus_current > self.max_bus_current_a:
                return (
                    False,
                    f"bus current high for {self.bus_current_trip_count} samples: "
                    f"{bus_current:.2f}A > {self.max_bus_current_a:.2f}A",
                )
        return True, ""

    def _estimate_policy_velocities(self, policy_positions: list[float], state_time: float) -> list[float]:
        if self._prev_positions_policy is None or self._prev_state_time is None:
            self._prev_positions_policy = list(policy_positions)
            self._prev_state_time = state_time
            return list(self.latest_velocities_policy)

        prev_positions = list(self._prev_positions_policy)
        dt = state_time - self._prev_state_time
        self._prev_positions_policy = list(policy_positions)
        self._prev_state_time = state_time
        if dt <= 1.0e-6 or dt > self.main_velocity_max_dt_s:
            return [0.0] * 6

        raw_vel = [
            max(
                -self.main_velocity_clip_rad_s,
                min(self.main_velocity_clip_rad_s, (float(policy_positions[i]) - float(prev_positions[i])) / dt),
            )
            for i in range(6)
        ]
        alpha = self.main_velocity_filter_alpha
        return [
            alpha * raw_vel[i] + (1.0 - alpha) * self.latest_velocities_policy[i]
            for i in range(6)
        ]

    def _estimate_abad_policy_velocities(
        self, policy_positions: list[float], state_time: float
    ) -> list[float]:
        if self._prev_abad_positions_policy is None or self._prev_abad_state_time is None:
            self._prev_abad_positions_policy = list(policy_positions)
            self._prev_abad_state_time = state_time
            return list(self.latest_abad_velocities_policy)

        previous = list(self._prev_abad_positions_policy)
        dt = state_time - self._prev_abad_state_time
        self._prev_abad_positions_policy = list(policy_positions)
        self._prev_abad_state_time = state_time
        if dt <= 1.0e-6 or dt > self.main_velocity_max_dt_s:
            return [0.0] * 6

        raw_velocity = [
            max(
                -self.main_velocity_clip_rad_s,
                min(
                    self.main_velocity_clip_rad_s,
                    (float(policy_positions[index]) - float(previous[index])) / dt,
                ),
            )
            for index in range(6)
        ]
        alpha = self.main_velocity_filter_alpha
        return [
            alpha * raw_velocity[index]
            + (1.0 - alpha) * self.latest_abad_velocities_policy[index]
            for index in range(6)
        ]
