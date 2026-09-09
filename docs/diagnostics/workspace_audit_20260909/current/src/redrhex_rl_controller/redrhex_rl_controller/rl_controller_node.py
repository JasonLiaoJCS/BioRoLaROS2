"""ROS2 node wiring RedRhex sensors -> ONNX policy -> motor commands."""

from __future__ import annotations

import math
import re
import time
import traceback

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray, String

from redrhex_msgs.msg import RedRhexMotorCommand, RedRhexMotorState

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError

from . import redrhex_contract as C
from .action_decoder import ActionDecoder, DecodedMotorCommand
from .degraded_mode import normalize_disabled_legs, policy_indices_for_disabled_legs
from .golden_policy import (
    GOLDEN_SCHEMA,
    controller_config_sha256 as compute_controller_config_sha256,
    deployment_source_sha256,
    decoder_config_sha256 as compute_decoder_config_sha256,
    decoder_source_sha256,
    observation_source_sha256,
)
from .observation_builder import ObservationBuilder
from .policy_onnx_runner import PolicyONNXRunner
from .policy_validation import check_reference_action, reference_policy_input
from .safety_filter import SafetyFilter, SafetyState
from .state_machine import RedRhexState, RedRhexStateMachine, StateMachineInputs


def _declare_get(
    node: Node,
    name: str,
    default,
    *,
    descriptor: ParameterDescriptor | None = None,
):
    if not node.has_parameter(name):
        node.declare_parameter(name, default, descriptor=descriptor)
    return node.get_parameter(name).value


def _declare_string_array(
    node: Node,
    name: str,
    *,
    descriptor: ParameterDescriptor | None = None,
) -> list[str]:
    if not node.has_parameter(name):
        node.declare_parameter(
            name, Parameter.Type.STRING_ARRAY, descriptor=descriptor
        )
    try:
        values = node.get_parameter(name).value
    except ParameterUninitializedException:
        # ROS 2 cannot infer the element type of an empty YAML sequence.  An
        # uninitialized STRING_ARRAY therefore means the intended strict-mode
        # default, not a startup failure.
        return []
    return [str(value) for value in values]


class RedRhexRLControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("redrhex_rl_controller")

        mask_descriptor = ParameterDescriptor(
            description=(
                "Physical disabled-leg mask. Startup-only; update the site YAML "
                "while output is off, then restart the stack."
            ),
            read_only=True,
        )
        self.max_disabled_legs = int(
            _declare_get(
                self,
                "hardware.max_disabled_legs",
                1,
                descriptor=mask_descriptor,
            )
        )
        self.disabled_legs = normalize_disabled_legs(
            _declare_string_array(
                self, "hardware.disabled_legs", descriptor=mask_descriptor
            ),
            self.max_disabled_legs,
        )
        self.disabled_policy_indices = policy_indices_for_disabled_legs(self.disabled_legs)

        self.onnx_path = str(_declare_get(self, "policy.onnx_path", "/home/jetson/redrhex_models/policy.onnx"))
        self.expected_obs_dim = int(_declare_get(self, "policy.expected_obs_dim", C.OBS_DIM_SINGLE))
        self.expected_action_dim = int(_declare_get(self, "policy.expected_action_dim", C.ACTION_DIM))
        self.policy_hz_param = float(_declare_get(self, "policy.policy_hz", 0.0))
        self.policy_hz = self.policy_hz_param if self.policy_hz_param > 0.0 else C.POLICY_HZ
        self.use_cuda = bool(_declare_get(self, "policy.use_cuda", False))
        self.use_tensorrt = bool(_declare_get(self, "policy.use_tensorrt", False))
        self.allow_history_dim = bool(_declare_get(self, "policy.allow_history_dim", True))
        self.observation_contract_id = str(
            _declare_get(self, "policy.observation_contract", C.OBSERVATION_CONTRACT_ID)
        )
        self.action_contract_id = str(
            _declare_get(self, "policy.action_contract", C.ACTION_DECODER_CONTRACT_ID)
        )
        self.require_contract_metadata = bool(
            _declare_get(self, "policy.require_contract_metadata", False)
        )
        self.expected_policy_sha256 = str(
            _declare_get(self, "policy.expected_sha256", "")
        ).strip().lower()
        self.startup_action_abs_limit = float(
            _declare_get(self, "policy.startup_action_abs_limit", 1.5)
        )
        self.expected_bridge_config_sha256 = str(
            _declare_get(self, "policy.expected_bridge_config_sha256", "")
        ).strip().lower()
        self.max_inference_p99_ms = float(
            _declare_get(self, "policy.max_inference_p99_ms", 6.0)
        )
        self.inference_benchmark_warmup_runs = int(
            _declare_get(self, "policy.inference_benchmark_warmup_runs", 20)
        )
        self.inference_benchmark_runs = int(
            _declare_get(self, "policy.inference_benchmark_runs", 200)
        )

        self.enable_policy_param = bool(_declare_get(self, "state_machine.enable_policy_on_start", False))
        self.enable_motor_output_param = bool(_declare_get(self, "state_machine.enable_motor_output_on_start", False))
        self.init_stand_duration_s = float(_declare_get(self, "state_machine.init_stand_duration_s", 2.0))
        self.warmup_duration_s = float(_declare_get(self, "state_machine.warmup_duration_s", 1.0))
        self.init_stand_timeout_s = float(_declare_get(self, "state_machine.init_stand_timeout_s", 12.0))
        self.init_stand_position_tolerance_rad = float(
            _declare_get(self, "state_machine.init_stand_position_tolerance_rad", 0.12)
        )
        self.init_stand_velocity_tolerance_rad_s = float(
            _declare_get(self, "state_machine.init_stand_velocity_tolerance_rad_s", 0.25)
        )
        self.init_stand_stable_time_s = float(
            _declare_get(self, "state_machine.init_stand_stable_time_s", 0.50)
        )
        self.output_enable_ack_timeout_s = float(
            _declare_get(self, "state_machine.output_enable_ack_timeout_s", 0.15)
        )
        self.policy_run_max_duration_s = float(
            _declare_get(self, "state_machine.policy_run_max_duration_s", 0.0)
        )

        self.sensor_timeout_s = float(_declare_get(self, "safety.sensor_timeout_s", 0.10))
        self.cmd_timeout_s = float(_declare_get(self, "safety.cmd_timeout_s", 0.25))
        self.require_motor_feedback = bool(_declare_get(self, "safety.require_motor_feedback", False))
        self.require_lowlevel_heartbeat = bool(_declare_get(self, "safety.require_lowlevel_heartbeat", False))

        command_limits = {
            "vx_min": float(_declare_get(self, "commands.vx_min", C.COMMAND_LIMITS["vx_min"])),
            "vx_max": float(_declare_get(self, "commands.vx_max", C.COMMAND_LIMITS["vx_max"])),
            "vy_min": float(_declare_get(self, "commands.vy_min", C.COMMAND_LIMITS["vy_min"])),
            "vy_max": float(_declare_get(self, "commands.vy_max", C.COMMAND_LIMITS["vy_max"])),
            "wz_min": float(_declare_get(self, "commands.wz_min", C.COMMAND_LIMITS["wz_min"])),
            "wz_max": float(_declare_get(self, "commands.wz_max", C.COMMAND_LIMITS["wz_max"])),
        }
        self.command_profile = str(
            _declare_get(self, "commands.profile", "external_cmd_vel")
        ).strip()
        self.fixed_forward_vx = float(
            _declare_get(self, "commands.fixed_forward_vx", 0.22)
        )

        builder_cfg = {
            "sensor_profile": str(_declare_get(self, "observation.sensor_profile", "full_state")),
            "encoder_only_rig_acknowledged": bool(
                _declare_get(self, "observation.encoder_only_rig_acknowledged", False)
            ),
            "full_feedback_rig_acknowledged": bool(
                _declare_get(self, "observation.full_feedback_rig_acknowledged", False)
            ),
            "imu_alignment_calibrated": bool(
                _declare_get(self, "observation.imu_alignment_calibrated", False)
            ),
            "expected_imu_frame_id": str(
                _declare_get(self, "observation.expected_imu_frame_id", "")
            ).strip(),
            "expected_imu_publisher_node": str(
                _declare_get(self, "observation.expected_imu_publisher_node", "")
            ).strip(),
            "imu_policy_to_sensor_quaternion_xyzw": list(
                _declare_get(
                    self,
                    "observation.imu_policy_to_sensor_quaternion_xyzw",
                    [0.0, 0.0, 0.0, 1.0],
                )
            ),
            "imu_upright_quaternion_xyzw": list(
                _declare_get(
                    self,
                    "observation.imu_upright_quaternion_xyzw",
                    [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
                )
            ),
            "rig_projected_gravity": list(
                _declare_get(
                    self,
                    "observation.rig_projected_gravity",
                    C.REFERENCE_PROJECTED_GRAVITY,
                )
            ),
            "expected_obs_dim": C.OBS_DIM_SINGLE,
            "policy_input_dim": self.expected_obs_dim,
            "policy_history_length": int(_declare_get(self, "observation.policy_history_length", C.POLICY_HISTORY_LENGTH)),
            "base_lin_vel_source": str(_declare_get(self, "observation.base_lin_vel_source", "zero")),
            "odom_twist_in_body_frame": bool(_declare_get(self, "observation.odom_twist_in_body_frame", True)),
            "abad_feedback_source": str(_declare_get(self, "observation.abad_feedback_source", "commanded")),
            "estimate_missing_joint_velocity": bool(_declare_get(self, "observation.estimate_missing_joint_velocity", True)),
            "observation_clip": float(_declare_get(self, "observation.observation_clip", 100.0)),
            "require_imu_source_stamp": bool(
                _declare_get(self, "observation.require_imu_source_stamp", False)
            ),
            "max_imu_source_age_s": float(
                _declare_get(self, "observation.max_imu_source_age_s", 0.10)
            ),
            "max_imu_future_skew_s": float(
                _declare_get(self, "observation.max_imu_future_skew_s", 0.02)
            ),
            "disabled_leg_indices": self.disabled_policy_indices,
            "disabled_leg_observation_mode": str(
                _declare_get(
                    self, "observation.disabled_leg_observation_mode", "passthrough"
                )
            ),
            "command_limits": command_limits,
            "command_profile": self.command_profile,
            "fixed_forward_vx": self.fixed_forward_vx,
        }
        self.expected_imu_publisher_node = str(
            builder_cfg["expected_imu_publisher_node"]
        ).strip().lstrip("/")
        decoder_cfg = {
            "action_clip": float(_declare_get(self, "safety.action_clip", 1.0)),
            "main_drive_vel_limit_rad_s": float(_declare_get(self, "safety.main_drive_vel_limit_rad_s", 30.0)),
            "abad_pos_limit": float(_declare_get(self, "safety.abad_pos_limit_rad", C.STAGE_ABAD_POS_LIMIT)),
            "main_drive_slew_rate_rad_s2": float(_declare_get(self, "safety.main_drive_slew_rate_rad_s2", 120.0)),
            "abad_slew_rate_rad_s": float(_declare_get(self, "safety.abad_slew_rate_rad_s", 6.0)),
            "include_damper_command": bool(_declare_get(self, "action.include_damper_command", False)),
            "main_drive_init_control_mode": str(
                _declare_get(self, "action.main_drive_init_control_mode", "velocity_to_pose")
            ),
            "init_stand_main_drive_position_gain": float(
                _declare_get(self, "action.init_stand_main_drive_position_gain", 3.0)
            ),
            "init_stand_max_main_drive_vel_rad_s": float(
                _declare_get(self, "action.init_stand_max_main_drive_vel_rad_s", 1.5)
            ),
            "stand_main_drive_kp": list(_declare_get(self, "action.stand_main_drive_kp", [12.0] * 6)),
            "stand_main_drive_kd": list(_declare_get(self, "action.stand_main_drive_kd", [1.0] * 6)),
            "main_drive_sign": list(_declare_get(self, "action.main_drive_sign", [1.0] * 6)),
            "abad_sign": list(_declare_get(self, "action.abad_sign", [1.0] * 6)),
            "damper_sign": list(_declare_get(self, "action.damper_sign", [1.0] * 6)),
            "main_drive_zero_offset_rad": list(_declare_get(self, "action.main_drive_zero_offset_rad", [0.0] * 6)),
            "abad_zero_offset_rad": list(_declare_get(self, "action.abad_zero_offset_rad", [0.0] * 6)),
            "damper_zero_offset_rad": list(_declare_get(self, "action.damper_zero_offset_rad", [0.0] * 6)),
            "main_drive_kp": list(_declare_get(self, "action.main_drive_kp", [0.0] * 6)),
            "main_drive_kd": list(_declare_get(self, "action.main_drive_kd", [50.0] * 6)),
            "abad_kp": list(_declare_get(self, "action.abad_kp", [40.0] * 6)),
            "abad_kd": list(_declare_get(self, "action.abad_kd", [4.0] * 6)),
            "play_forward_compat_enable": bool(
                _declare_get(self, "action.play_forward_compat_enable", False)
            ),
            "play_forward_compat_bias_scale": float(
                _declare_get(self, "action.play_forward_compat_bias_scale", 1.0)
            ),
            "play_forward_compat_residual_scale": float(
                _declare_get(self, "action.play_forward_compat_residual_scale", 0.04)
            ),
            "play_forward_compat_residual_clip": float(
                _declare_get(self, "action.play_forward_compat_residual_clip", 0.30)
            ),
            "disabled_leg_indices": self.disabled_policy_indices,
        }
        self.hardware_mapping_calibrated = bool(
            _declare_get(self, "action.hardware_mapping_calibrated", False)
        )
        # The observation contract is the single source of truth for whether
        # IMU freshness and tilt protection are required.  Keeping this
        # derived avoids a launch/config typo silently disabling either guard.
        self.observation_builder = ObservationBuilder(builder_cfg)
        safety_cfg = {
            "sensor_timeout_s": self.sensor_timeout_s,
            "cmd_timeout_s": self.cmd_timeout_s,
            "motor_feedback_timeout_s": float(_declare_get(self, "safety.motor_feedback_timeout_s", 0.25)),
            "heartbeat_timeout_s": float(_declare_get(self, "safety.heartbeat_timeout_s", 0.10)),
            "max_abs_roll_rad": float(_declare_get(self, "safety.max_abs_roll_rad", 0.7)),
            "max_abs_pitch_rad": float(_declare_get(self, "safety.max_abs_pitch_rad", 0.7)),
            "action_clip": float(_declare_get(self, "safety.action_clip", 1.0)),
            "max_raw_action_abs": float(
                _declare_get(self, "safety.max_raw_action_abs", 1.5)
            ),
            "main_drive_vel_limit_rad_s": float(_declare_get(self, "safety.main_drive_vel_limit_rad_s", 30.0)),
            "abad_pos_limit_rad": float(_declare_get(self, "safety.abad_pos_limit_rad", 0.7)),
            "max_motor_temperature_c": float(_declare_get(self, "safety.max_motor_temperature_c", 70.0)),
            "max_motor_current_a": float(_declare_get(self, "safety.max_motor_current_a", 20.0)),
            "max_measured_main_drive_velocity_rad_s": float(
                _declare_get(self, "safety.max_measured_main_drive_velocity_rad_s", 80.0)
            ),
            "max_control_loop_dt_s": float(_declare_get(self, "safety.max_control_loop_dt_s", 0.03)),
            "max_inference_duration_s": float(
                _declare_get(self, "safety.max_inference_duration_s", 0.008)
            ),
            "motor_current_trip_samples": int(_declare_get(self, "safety.motor_current_trip_samples", 5)),
            "control_loop_trip_samples": int(_declare_get(self, "safety.control_loop_trip_samples", 3)),
            "inference_trip_samples": int(
                _declare_get(self, "safety.inference_trip_samples", 3)
            ),
            "disabled_leg_indices": self.disabled_policy_indices,
            "require_motor_feedback": self.require_motor_feedback,
            "require_lowlevel_heartbeat": self.require_lowlevel_heartbeat,
            "require_imu": self.observation_builder.requires_imu,
            "enable_tilt_guard": self.observation_builder.requires_imu,
            "command_limits": command_limits,
        }

        self.action_decoder = ActionDecoder(decoder_cfg)
        self.decoder_config_sha256 = compute_decoder_config_sha256(decoder_cfg)
        self.decoder_source_sha256 = decoder_source_sha256()
        self.observation_source_sha256 = observation_source_sha256()
        self.deployment_source_sha256 = deployment_source_sha256()
        self.safety_filter = SafetyFilter(safety_cfg)
        if not math.isfinite(self.startup_action_abs_limit) or self.startup_action_abs_limit <= 0.0:
            raise ValueError("policy.startup_action_abs_limit must be positive and finite")
        if self.expected_policy_sha256 and (
            len(self.expected_policy_sha256) != 64
            or any(ch not in "0123456789abcdef" for ch in self.expected_policy_sha256)
        ):
            raise ValueError("policy.expected_sha256 must be empty or a 64-character lowercase hex digest")
        if self.expected_bridge_config_sha256 and re.fullmatch(
            r"[0-9a-f]{64}", self.expected_bridge_config_sha256
        ) is None:
            raise ValueError(
                "policy.expected_bridge_config_sha256 must be empty or 64 lowercase hex characters"
            )
        if (
            not math.isfinite(self.max_inference_p99_ms)
            or self.max_inference_p99_ms <= 0.0
            or self.inference_benchmark_warmup_runs < 0
            or self.inference_benchmark_runs < 100
        ):
            raise ValueError("policy inference benchmark configuration is invalid")
        stand_limits = {
            "state_machine.init_stand_duration_s": self.init_stand_duration_s,
            "state_machine.warmup_duration_s": self.warmup_duration_s,
            "state_machine.init_stand_timeout_s": self.init_stand_timeout_s,
            "state_machine.init_stand_position_tolerance_rad": self.init_stand_position_tolerance_rad,
            "state_machine.init_stand_velocity_tolerance_rad_s": self.init_stand_velocity_tolerance_rad_s,
            "state_machine.init_stand_stable_time_s": self.init_stand_stable_time_s,
            "state_machine.output_enable_ack_timeout_s": self.output_enable_ack_timeout_s,
        }
        for name, value in stand_limits.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if (
            not math.isfinite(self.policy_run_max_duration_s)
            or self.policy_run_max_duration_s < 0.0
        ):
            raise ValueError(
                "state_machine.policy_run_max_duration_s must be finite and >= 0"
            )
        if self.command_profile == "fixed_forward" and not (
            0.0 < self.policy_run_max_duration_s <= 300.0
        ):
            raise ValueError(
                "fixed_forward state_machine.policy_run_max_duration_s "
                "must be in (0, 300]"
            )
        if self._is_strict_rig_profile():
            violations = self._strict_rig_safety_violations(command_limits, decoder_cfg)
            if violations:
                raise ValueError(
                    f"unsafe {self.observation_builder.sensor_profile} configuration: "
                    + "; ".join(violations)
                )
        self.abad_feedback_source = self.observation_builder.abad_feedback_source
        self.required_joint_names = list(self.observation_builder.required_joint_names)
        self.include_damper_command = self.action_decoder.include_damper_command
        self.state_machine = RedRhexStateMachine(
            require_motor_feedback=self.require_motor_feedback,
            require_lowlevel_heartbeat=self.require_lowlevel_heartbeat,
        )
        self.controller_config_sha256 = compute_controller_config_sha256(
            self._effective_controller_parameter_tree()
        )

        self.policy_runner: PolicyONNXRunner | None = None
        self.policy_loaded = False
        self.policy_sha256 = "unavailable"
        self.startup_reference_action_max = math.nan
        self.startup_inference_benchmark: dict[str, float | int] = {}
        try:
            self.policy_runner = PolicyONNXRunner(
                self.onnx_path,
                expected_obs_dim=self.expected_obs_dim,
                expected_action_dim=self.expected_action_dim,
                use_cuda=self.use_cuda,
                use_tensorrt=self.use_tensorrt,
                allow_history_dim=self.allow_history_dim,
                expected_observation_contract=self.observation_contract_id,
                expected_action_contract=self.action_contract_id,
                expected_input_layout="auto",
                require_contract_metadata=self.require_contract_metadata,
            )
            if not self.policy_runner.is_sensor_v2 and self.policy_runner.obs_dim is not None:
                self.observation_builder.set_policy_input_dim(self.policy_runner.obs_dim)
            self.policy_sha256 = self.policy_runner.sha256
            if self._is_strict_rig_profile() and not self.expected_policy_sha256:
                raise ValueError(
                    f"{self.observation_builder.sensor_profile} requires policy.expected_sha256 "
                    "to pin the verified artifact; "
                    "the node will remain observation-only until it is configured"
                )
            if self.expected_policy_sha256 and self.policy_sha256 != self.expected_policy_sha256:
                raise ValueError(
                    f"policy SHA256 {self.policy_sha256} != configured {self.expected_policy_sha256}"
                )
            if self.require_contract_metadata and not self.policy_runner.is_sensor_v2:
                policy_metadata = self.policy_runner.io_info.metadata
                required_evidence = {
                    C.ONNX_NORMALIZER_KEY: C.NORMALIZER_EMBEDDED,
                    C.ONNX_DECODER_SOURCE_SHA256_KEY: self.decoder_source_sha256,
                    C.ONNX_DECODER_CONFIG_SHA256_KEY: self.decoder_config_sha256,
                    C.ONNX_GOLDEN_SCHEMA_KEY: GOLDEN_SCHEMA,
                    C.ONNX_CONTROLLER_CONFIG_SHA256_KEY: self.controller_config_sha256,
                    C.ONNX_OBSERVATION_SOURCE_SHA256_KEY: self.observation_source_sha256,
                    C.ONNX_DEPLOYMENT_SOURCE_SHA256_KEY: self.deployment_source_sha256,
                    C.ONNX_BRIDGE_CONFIG_SHA256_KEY: self.expected_bridge_config_sha256,
                }
                for key, expected_value in required_evidence.items():
                    actual_value = policy_metadata.get(key)
                    if actual_value != expected_value:
                        raise ValueError(
                            f"policy metadata {key}={actual_value!r}, expected {expected_value!r}"
                        )
                training_git_sha = policy_metadata.get(
                    C.ONNX_TRAINING_GIT_SHA_KEY, ""
                ).strip().lower()
                if re.fullmatch(r"[0-9a-f]{7,64}", training_git_sha) is None:
                    raise ValueError(
                        f"policy metadata {C.ONNX_TRAINING_GIT_SHA_KEY} must be a 7-64 hex commit"
                    )
                for source_key in (
                    C.ONNX_TRAINING_ENV_SOURCE_SHA256_KEY,
                    C.ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY,
                ):
                    if re.fullmatch(
                        r"[0-9a-f]{64}", policy_metadata.get(source_key, "")
                    ) is None:
                        raise ValueError(
                            f"policy metadata {source_key} must be a 64-character SHA256"
                        )
                try:
                    training_action_clip = float(
                        policy_metadata[C.ONNX_TRAINING_ACTION_CLIP_KEY]
                    )
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"policy metadata is missing numeric {C.ONNX_TRAINING_ACTION_CLIP_KEY}"
                    ) from exc
                if not math.isclose(
                    training_action_clip,
                    C.TRAINING_ACTION_CLIP,
                    rel_tol=0.0,
                    abs_tol=1.0e-12,
                ):
                    raise ValueError(
                        f"policy training action clip {training_action_clip} != "
                        f"{C.TRAINING_ACTION_CLIP}"
                    )
            if self.policy_runner.is_sensor_v2:
                nominal_main = np.asarray(C.INIT_MAIN_DRIVE_POS, dtype=np.float32)
                reference_frame = np.concatenate(
                    [
                        np.zeros(3, dtype=np.float32),
                        np.asarray(C.REFERENCE_PROJECTED_GRAVITY, dtype=np.float32),
                        np.sin(nominal_main),
                        np.cos(nominal_main),
                        np.zeros(18, dtype=np.float32),
                    ]
                )
                reference_input = np.repeat(reference_frame[None, :], 60, axis=0)
                reference_command = self.observation_builder.sensor_v2_command()
                reference_action = self.policy_runner.run(reference_input, reference_command)
            else:
                reference_input = reference_policy_input(
                    self.policy_runner.obs_dim or self.expected_obs_dim,
                    self.observation_builder.history_length,
                )
                reference_action = self.policy_runner.run(reference_input)
            reference_ok, self.startup_reference_action_max = check_reference_action(
                reference_action, self.startup_action_abs_limit
            )
            if not reference_ok:
                raise ValueError(
                    "reference observation produced implausible policy action: "
                    f"max_abs={self.startup_reference_action_max:.3f} > "
                    f"policy.startup_action_abs_limit={self.startup_action_abs_limit:.3f}; "
                    "the ONNX normalizer/action contract is incompatible"
                )
            if self.policy_runner.is_sensor_v2:
                self.startup_inference_benchmark = self.policy_runner.benchmark_sensor_v2(
                    [(reference_input, reference_command)],
                    warmup_runs=self.inference_benchmark_warmup_runs,
                    runs=self.inference_benchmark_runs,
                )
            else:
                self.startup_inference_benchmark = self.policy_runner.benchmark(
                    [
                        np.zeros(self.policy_runner.obs_dim or self.expected_obs_dim, dtype=np.float32),
                        reference_input,
                    ],
                    warmup_runs=self.inference_benchmark_warmup_runs,
                    runs=self.inference_benchmark_runs,
                )
            if (
                float(self.startup_inference_benchmark["p99_ms"])
                > self.max_inference_p99_ms
            ):
                raise ValueError(
                    "CPU inference p99 exceeds startup gate: "
                    f"{self.startup_inference_benchmark['p99_ms']:.3f} ms > "
                    f"{self.max_inference_p99_ms:.3f} ms"
                )
            self.policy_loaded = True
            self.get_logger().info(f"Loaded ONNX policy: {self.policy_runner.io_info}")
        except Exception as exc:
            self.get_logger().error(f"Failed to load ONNX policy: {exc}")

        self.estop = False
        self.enable_policy = self.enable_policy_param
        self.enable_motor_output = self.enable_motor_output_param
        self.recover_requested = False
        self.last_motor_feedback_time: float | None = None
        self.last_lowlevel_heartbeat_time: float | None = None
        self.last_lowlevel_output_status_time: float | None = None
        self.lowlevel_output_enabled = False
        self.last_lowlevel_disabled_legs_time: float | None = None
        self.lowlevel_disabled_legs: list[str] | None = None
        self.motor_temperatures: list[float] = []
        self.motor_currents: list[float] = []
        self.motor_faults: list[bool] = []
        self.motor_velocities: list[float] = []
        self.motor_feedback_sequence = 0
        self.control_sequence = 0
        self.inference_sequence = 0
        self.last_inference_duration_s: float | None = None
        self.battery_state: BatteryState | None = None
        self.last_loop_time: float | None = None
        self.last_diag_values: dict[str, str] = {}
        self.state_enter_time = self._now_s()
        self.init_stand_output_start_time: float | None = None
        self.init_stand_stable_since: float | None = None
        self.init_stand_verified = False
        self.motor_enable_request_time: float | None = None
        self.last_stand_max_error_rad = math.inf
        self.last_stand_max_speed_rad_s = math.inf
        self.last_publisher_check_time: float | None = None
        self.publisher_guard_reasons: list[str] = []
        self.publisher_guard_diagnostic = "not checked"

        self.create_subscription(
            Imu, "/imu/data", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/estop", self._on_estop, 10)
        self.create_subscription(Bool, "/redrhex/enable_policy", self._on_enable_policy, 10)
        self.create_subscription(Bool, "/redrhex/enable_motors", self._on_enable_motors, 10)
        self.create_subscription(Bool, "/redrhex/recover", self._on_recover, 10)
        self.create_subscription(Bool, "/redrhex/lowlevel_heartbeat", self._on_lowlevel_heartbeat, 10)
        self.create_subscription(
            Bool,
            "/redrhex/lowlevel_output_enabled",
            self._on_lowlevel_output_enabled,
            10,
        )
        self.create_subscription(
            String,
            "/redrhex/lowlevel_disabled_legs",
            self._on_lowlevel_disabled_legs,
            10,
        )
        self.create_subscription(RedRhexMotorState, "/motor_feedback", self._on_motor_feedback, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery_state, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.obs_pub = self.create_publisher(Float32MultiArray, "/redrhex/observation", 10)
        self.policy_input_pub = self.create_publisher(Float32MultiArray, "/redrhex/policy_input", 10)
        self.imputation_mask_pub = self.create_publisher(
            Float32MultiArray, "/redrhex/observation_imputation_mask", 10
        )
        self.raw_action_pub = self.create_publisher(Float32MultiArray, "/redrhex/policy_action_raw", 10)
        self.safe_action_pub = self.create_publisher(Float32MultiArray, "/redrhex/policy_action_safe", 10)
        self.motor_cmd_pub = self.create_publisher(RedRhexMotorCommand, "/redrhex/motor_commands", 10)
        self.state_pub = self.create_publisher(String, "/redrhex/state_machine_state", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/redrhex/diagnostics", 10)

        self.timer = self.create_timer(1.0 / self.policy_hz, self._control_tick)
        self.get_logger().info(f"RedRhex RL controller started at {self.policy_hz:.1f} Hz.")
        if self.observation_builder.is_encoder_only_rig:
            self.get_logger().warn(
                "ENCODER_ONLY_RIG: base linear/angular velocity are imputed, projected gravity is fixed, "
                "and IMU tilt/fall protection is unavailable. Use only while rigidly suspended."
            )
        elif self.observation_builder.is_full_feedback_rig:
            self.get_logger().warn(
                "FULL_FEEDBACK_RIG: main-drive encoders, ABAD encoders, and aligned IMU are required; "
                "base linear velocity remains imputed. Use only while rigidly suspended."
            )
        if self.command_profile == "fixed_forward":
            self.get_logger().warn(
                "FIXED_FORWARD command profile: POLICY_RUN will command "
                f"[vx,vy,wz]=[{self.fixed_forward_vx:.3f},0,0] m/s; /cmd_vel is ignored. "
                f"This rig trial expires after {self.policy_run_max_duration_s:.1f} s. "
                "Disable policy/motors or assert E-stop to return the command to zero."
            )
        if self.disabled_legs:
            self.get_logger().warn(
                "DEGRADED MODE: hardware.disabled_legs="
                f"{self.disabled_legs} (policy indices {self.disabled_policy_indices}); "
                "fault/current checks ignore only those isolated legs. Rinbo motor "
                "temperature telemetry is unavailable, so the configured temperature guard is inactive"
            )

    def _is_strict_rig_profile(self) -> bool:
        return bool(
            self.observation_builder.is_encoder_only_rig
            or self.observation_builder.is_full_feedback_rig
        )

    def _strict_rig_safety_violations(
        self, command_limits: dict[str, float], decoder_cfg: dict
    ) -> list[str]:
        """Return fail-closed deployment violations shared by suspended rig modes."""

        violations: list[str] = []
        if self.max_disabled_legs != 1:
            violations.append("hardware.max_disabled_legs must be exactly 1")
        if not self.require_contract_metadata:
            violations.append("policy.require_contract_metadata must be true")
        if self.use_cuda or self.use_tensorrt:
            violations.append("strict rig startup benchmark requires CPU inference")
        if bool(self.get_parameter("use_sim_time").value):
            violations.append("use_sim_time must be false on real hardware")
        if not self.expected_bridge_config_sha256:
            violations.append("policy.expected_bridge_config_sha256 must pin the bridge semantics")
        if not self.hardware_mapping_calibrated:
            violations.append(
                "action.hardware_mapping_calibrated must be true only after main/ABAD sign and zero measurements"
            )
        if self.observation_builder.history_length != C.POLICY_HISTORY_LENGTH:
            violations.append(
                f"observation.policy_history_length must be {C.POLICY_HISTORY_LENGTH}"
            )
        if self.max_inference_p99_ms > 6.0:
            violations.append("policy.max_inference_p99_ms must be <= 6.0")
        if self.inference_benchmark_runs < 100:
            violations.append("policy.inference_benchmark_runs must be >= 100")
        if not math.isclose(
            self.policy_hz, C.POLICY_HZ, rel_tol=0.0, abs_tol=1.0e-9
        ):
            violations.append(f"policy.policy_hz must resolve to {C.POLICY_HZ:g} Hz")
        if self.output_enable_ack_timeout_s > 0.15:
            violations.append("state_machine.output_enable_ack_timeout_s must be <= 0.15")
        if not 0.0 < self.policy_run_max_duration_s <= 300.0:
            violations.append(
                "state_machine.policy_run_max_duration_s must be in (0, 300]"
            )
        if self.init_stand_timeout_s > 60.0:
            violations.append("state_machine.init_stand_timeout_s must be <= 60.0")
        if self.init_stand_duration_s < 2.0:
            violations.append("state_machine.init_stand_duration_s must be >= 2.0")
        if self.init_stand_stable_time_s < 0.50:
            violations.append("state_machine.init_stand_stable_time_s must be >= 0.50")
        if self.warmup_duration_s < 1.0:
            violations.append("state_machine.warmup_duration_s must be >= 1.0")
        if self.init_stand_position_tolerance_rad > 0.12:
            violations.append(
                "state_machine.init_stand_position_tolerance_rad must be <= 0.12"
            )
        if self.init_stand_velocity_tolerance_rad_s > 0.25:
            violations.append(
                "state_machine.init_stand_velocity_tolerance_rad_s must be <= 0.25"
            )
        if self.safety_filter.heartbeat_timeout_s > 0.15:
            violations.append("safety.heartbeat_timeout_s must be <= 0.15")
        if self.sensor_timeout_s > 0.10:
            violations.append("safety.sensor_timeout_s must be <= 0.10")
        if self.cmd_timeout_s > 0.25:
            violations.append("safety.cmd_timeout_s must be <= 0.25")
        if self.safety_filter.motor_feedback_timeout_s > 0.25:
            violations.append("safety.motor_feedback_timeout_s must be <= 0.25")
        if not self.require_motor_feedback:
            violations.append("safety.require_motor_feedback must be true")
        if not self.require_lowlevel_heartbeat:
            violations.append("safety.require_lowlevel_heartbeat must be true")
        if self.enable_policy_param or self.enable_motor_output_param:
            violations.append("policy and motor output must both start disabled")
        if self.safety_filter.max_abs_roll_rad > 0.45:
            violations.append("safety.max_abs_roll_rad must be <= 0.45")
        if self.safety_filter.max_abs_pitch_rad > 0.45:
            violations.append("safety.max_abs_pitch_rad must be <= 0.45")
        if self.safety_filter.action_clip > 0.35:
            violations.append("safety.action_clip must be <= 0.35")
        if self.safety_filter.max_raw_action_abs > 1.5:
            violations.append("safety.max_raw_action_abs must be <= 1.5")
        if self.safety_filter.main_drive_vel_limit_rad_s > 1.0:
            violations.append("safety.main_drive_vel_limit_rad_s must be <= 1.0")
        if self.safety_filter.abad_pos_limit_rad > 0.18:
            violations.append("safety.abad_pos_limit_rad must be <= 0.18")
        if self.safety_filter.max_measured_main_drive_velocity_rad_s > 4.0:
            violations.append(
                "safety.max_measured_main_drive_velocity_rad_s must be <= 4.0"
            )
        if self.safety_filter.max_motor_temperature_c > 55.0:
            violations.append("safety.max_motor_temperature_c must be <= 55.0")
        if self.safety_filter.max_motor_current_a > 10.0:
            violations.append("safety.max_motor_current_a must be <= 10.0")
        if self.safety_filter.max_control_loop_dt_s > 0.03:
            violations.append("safety.max_control_loop_dt_s must be <= 0.03")
        if self.safety_filter.max_inference_duration_s > 0.016:
            violations.append("safety.max_inference_duration_s must be <= 0.016")
        if self.safety_filter.motor_current_trip_samples > 100:
            violations.append("safety.motor_current_trip_samples must be <= 100")
        if self.safety_filter.control_loop_trip_samples > 3:
            violations.append("safety.control_loop_trip_samples must be <= 3")
        if self.safety_filter.inference_trip_samples > 3:
            violations.append("safety.inference_trip_samples must be <= 3")
        if decoder_cfg["main_drive_slew_rate_rad_s2"] > 12.0:
            violations.append("safety.main_drive_slew_rate_rad_s2 must be <= 12.0")
        if decoder_cfg["abad_slew_rate_rad_s"] > 1.0:
            violations.append("safety.abad_slew_rate_rad_s must be <= 1.0")
        if decoder_cfg["init_stand_max_main_drive_vel_rad_s"] > 0.35:
            violations.append(
                "action.init_stand_max_main_drive_vel_rad_s must be <= 0.35"
            )
        if self.startup_action_abs_limit > 1.5:
            violations.append("policy.startup_action_abs_limit must be <= 1.5")

        command_bounds = {
            "commands.vx_min": (command_limits["vx_min"], 0.0, 0.22),
            "commands.vx_max": (command_limits["vx_max"], 0.22, 0.22),
            "commands.vy_min": (command_limits["vy_min"], -0.14, 0.14),
            "commands.vy_max": (command_limits["vy_max"], -0.14, 0.14),
            "commands.wz_min": (command_limits["wz_min"], -0.17, 0.17),
            "commands.wz_max": (command_limits["wz_max"], -0.17, 0.17),
        }
        for name, (value, minimum, maximum) in command_bounds.items():
            if value < minimum or value > maximum:
                violations.append(f"{name} must be in [{minimum}, {maximum}]")

        if self.observation_builder.is_encoder_only_rig:
            if not np.allclose(
                self.observation_builder.rig_projected_gravity,
                np.asarray(C.REFERENCE_PROJECTED_GRAVITY, dtype=np.float64),
                rtol=0.0,
                atol=1.0e-9,
            ):
                violations.append(
                    "observation.rig_projected_gravity must match the training frame [0,-1,0]"
                )
        else:
            if not self.observation_builder.full_feedback_rig_acknowledged:
                violations.append(
                    "observation.full_feedback_rig_acknowledged must be true"
                )
            if not self.observation_builder.imu_alignment_calibrated:
                violations.append("observation.imu_alignment_calibrated must be true")
            if self.observation_builder.base_lin_vel_source != "zero":
                violations.append("observation.base_lin_vel_source must be 'zero'")
            if self.observation_builder.abad_feedback_source != "joint_states":
                violations.append(
                    "observation.abad_feedback_source must be 'joint_states'"
                )
            if not str(self.observation_builder.expected_imu_frame_id).strip():
                violations.append("observation.expected_imu_frame_id must be non-empty")
            if not self.expected_imu_publisher_node:
                violations.append(
                    "observation.expected_imu_publisher_node must be non-empty"
                )
            if not self.observation_builder.requires_imu:
                violations.append("full_feedback_rig must require IMU safety guards")
            if not self.observation_builder.require_imu_source_stamp:
                violations.append("full_feedback_rig requires IMU source stamps")
            if self.command_profile != "fixed_forward":
                violations.append("commands.profile must be 'fixed_forward'")

        if self.command_profile == "fixed_forward" and not (
            math.isclose(self.fixed_forward_vx, 0.22, rel_tol=0.0, abs_tol=1.0e-12)
            and command_limits["vx_min"]
            <= self.fixed_forward_vx
            <= command_limits["vx_max"]
            and command_limits["vy_min"] <= 0.0 <= command_limits["vy_max"]
            and command_limits["wz_min"] <= 0.0 <= command_limits["wz_max"]
        ):
            violations.append(
                "commands.fixed_forward_vx must be exactly 0.22 m/s and inside command limits"
            )
        compat_expected = {
            "play_forward_compat_bias_scale": 1.0,
            "play_forward_compat_residual_scale": 0.04,
            "play_forward_compat_residual_clip": 0.30,
        }
        if not decoder_cfg["play_forward_compat_enable"]:
            violations.append("action.play_forward_compat_enable must be true")
        for name, expected in compat_expected.items():
            if not math.isclose(
                float(decoder_cfg[name]), expected, rel_tol=0.0, abs_tol=1.0e-12
            ):
                violations.append(f"action.{name} must be exactly {expected:g}")
        return violations

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _effective_controller_parameter_tree(self) -> dict:
        """Materialize the effective ROS parameters used by the artifact hash."""

        tree: dict = {}
        for name, parameter in self.get_parameters_by_prefix("").items():
            if name.split(".", 1)[0] not in {
                "hardware",
                "policy",
                "observation",
                "state_machine",
                "safety",
                "action",
                "commands",
            }:
                continue
            value = parameter.value
            if value is None:
                continue
            current = tree
            parts = name.split(".")
            for part in parts[:-1]:
                current = current.setdefault(part, {})
            current[parts[-1]] = value
        tree.setdefault("hardware", {})["disabled_legs"] = list(self.disabled_legs)
        return tree

    def _on_imu(self, msg: Imu) -> None:
        self.observation_builder.update_imu(
            msg, self._now_s(), receipt_monotonic_s=time.monotonic()
        )

    def _on_joint_states(self, msg: JointState) -> None:
        self.observation_builder.update_joint_state(msg, self._now_s())

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.observation_builder.update_cmd_vel(msg, self._now_s())

    def _on_odom(self, msg: Odometry) -> None:
        self.observation_builder.update_odom(msg, self._now_s())

    def _drop_enable_latches(self, reason: str) -> None:
        if self.enable_policy or self.enable_motor_output:
            self.get_logger().warn(f"Dropping enable latches: {reason}")
        self.enable_policy = False
        self.enable_motor_output = False
        self.motor_enable_request_time = None
        self.init_stand_output_start_time = None
        self.init_stand_stable_since = None
        self.init_stand_verified = False
        self.observation_builder.set_policy_running(False)

    def _policy_enable_allowed(self) -> bool:
        return (
            (not self.estop)
            and self.enable_motor_output
            and self._lowlevel_output_ack_ready(self._now_s())
            and self.init_stand_verified
            and self.state_machine.state
            in (
                RedRhexState.POLICY_READY,
                RedRhexState.POLICY_RUN,
            )
        )

    def _lowlevel_output_ack_ready(self, now_s: float) -> bool:
        """Require a fresh positive acknowledgement from this enable attempt."""
        return (
            self.enable_motor_output
            and self._lowlevel_mask_ready(now_s)
            and self._sensor_publisher_contract_ready(now_s)
            and self.motor_enable_request_time is not None
            and self.last_lowlevel_output_status_time is not None
            and self.last_lowlevel_output_status_time >= self.motor_enable_request_time
            and now_s - self.last_lowlevel_output_status_time
            <= self.safety_filter.heartbeat_timeout_s
            and self.lowlevel_output_enabled
        )

    def _motor_output_enable_allowed(self) -> bool:
        # Once INIT_STAND has completed, any output interruption invalidates
        # the stand/stability evidence.  Re-enabling is therefore only legal
        # after returning to INIT_STAND.
        now_s = self._now_s()
        self._update_sensor_publisher_guard(now_s)
        return (
            (not self.estop)
            and self.state_machine.state == RedRhexState.INIT_STAND
            and self._lowlevel_mask_ready(now_s)
            and self._sensor_publisher_contract_ready(now_s)
        )

    def _sensor_publisher_contract_ready(self, now_s: float) -> bool:
        if not self._is_strict_rig_profile():
            return True
        return (
            self.last_publisher_check_time is not None
            and now_s - self.last_publisher_check_time <= 0.15
            and not self.publisher_guard_reasons
        )

    def _lowlevel_mask_ready(self, now_s: float) -> bool:
        return (
            self.last_lowlevel_disabled_legs_time is not None
            and now_s - self.last_lowlevel_disabled_legs_time
            <= self.safety_filter.heartbeat_timeout_s
            and self.lowlevel_disabled_legs is not None
            and set(self.lowlevel_disabled_legs) == set(self.disabled_legs)
            and len(self.lowlevel_disabled_legs) == len(self.disabled_legs)
        )

    def _restart_init_stand(self, reason: str) -> None:
        self.enable_policy = False
        self.state_machine.transition(RedRhexState.INIT_STAND, reason)
        now_s = self._now_s()
        self.state_enter_time = now_s
        self.motor_enable_request_time = now_s if self.enable_motor_output else None
        self.init_stand_output_start_time = None
        self.init_stand_stable_since = None
        self.init_stand_verified = False
        self.observation_builder.reset(gait_phase=0.0)
        self.action_decoder.reset(gait_phase=0.0)

    def _end_policy_run(self, reason: str) -> None:
        """Hard-disable output and require a new INIT_STAND/re-arm cycle."""

        self._drop_enable_latches(reason)
        self._restart_init_stand(reason)

    def _policy_run_lease_expired(self, now_s: float) -> bool:
        return (
            self.state_machine.state == RedRhexState.POLICY_RUN
            and self.policy_run_max_duration_s > 0.0
            and now_s - self.state_enter_time >= self.policy_run_max_duration_s
        )

    def _on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)
        if self.estop:
            self._drop_enable_latches("E-stop asserted")

    def _on_enable_policy(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if not requested:
            if self.state_machine.state == RedRhexState.POLICY_RUN:
                self._end_policy_run(
                    "policy disabled; motor output disabled and init stand must be re-armed"
                )
            else:
                self.enable_policy = False
            return
        if not self._policy_enable_allowed():
            self.enable_policy = False
            self.get_logger().warn(
                f"Rejecting policy enable while state={self.state_machine.state.value}, estop={self.estop}"
            )
            return
        self.enable_policy = True

    def _on_enable_motors(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if not requested:
            self.enable_motor_output = False
            self.motor_enable_request_time = None
            if self.state_machine.state in (
                RedRhexState.WARMUP,
                RedRhexState.POLICY_READY,
                RedRhexState.POLICY_RUN,
            ):
                self._restart_init_stand("motor output disabled; re-run init stand")
            else:
                self.init_stand_output_start_time = None
                self.init_stand_stable_since = None
            return
        # Treat enable as an edge-triggered request.  A latched/repeating
        # publisher must not extend the first-ack deadline forever, and a
        # repeated True in WARMUP/RUN must not turn a valid latch back off.
        if self.enable_motor_output:
            return
        if not self._motor_output_enable_allowed():
            self.enable_motor_output = False
            self.get_logger().warn(
                f"Rejecting motor output enable while state={self.state_machine.state.value}, estop={self.estop}"
            )
            return
        self.enable_motor_output = True
        self.motor_enable_request_time = self._now_s()

    def _on_recover(self, msg: Bool) -> None:
        self.recover_requested = bool(msg.data)

    def _on_lowlevel_heartbeat(self, msg: Bool) -> None:
        if msg.data:
            self.last_lowlevel_heartbeat_time = self._now_s()

    def _on_lowlevel_output_enabled(self, msg: Bool) -> None:
        self.last_lowlevel_output_status_time = self._now_s()
        self.lowlevel_output_enabled = bool(msg.data)

    def _on_lowlevel_disabled_legs(self, msg: String) -> None:
        self.last_lowlevel_disabled_legs_time = self._now_s()
        try:
            values = [item.strip() for item in str(msg.data).split(",") if item.strip()]
            self.lowlevel_disabled_legs = normalize_disabled_legs(
                values, self.max_disabled_legs
            )
        except ValueError as exc:
            self.lowlevel_disabled_legs = None
            self.get_logger().error(f"Invalid low-level disabled-leg contract: {exc}")

    def _on_motor_feedback(self, msg: RedRhexMotorState) -> None:
        self.last_motor_feedback_time = self._now_s()
        self.motor_feedback_sequence += 1
        self.motor_temperatures = [float(x) for x in msg.temperature_c]
        self.motor_currents = [float(x) for x in msg.current_a]
        self.motor_faults = [bool(x) for x in msg.fault]
        self.motor_velocities = [float(x) for x in msg.velocity_rad_s]

    def _on_battery_state(self, msg: BatteryState) -> None:
        self.battery_state = msg

    def _evaluate_init_stand_gate(
        self, now_s: float, feedback_ready: bool
    ) -> tuple[bool, str | None]:
        current = self.observation_builder.get_main_drive_positions()
        velocity = np.asarray(
            [
                self.observation_builder.joint_vel.get(name, 0.0)
                for name in self.observation_builder.main_drive_joint_names
            ],
            dtype=np.float64,
        )
        target = np.asarray(self.action_decoder.init_main_drive_pos, dtype=np.float64)
        active = [index for index in range(6) if index not in self.disabled_policy_indices]
        if not active or not np.isfinite(current[active]).all() or not np.isfinite(velocity[active]).all():
            self.init_stand_stable_since = None
            self.last_stand_max_error_rad = math.inf
            self.last_stand_max_speed_rad_s = math.inf
            return False, None

        wrapped_error = np.arctan2(np.sin(target - current), np.cos(target - current))
        self.last_stand_max_error_rad = float(np.max(np.abs(wrapped_error[active])))
        self.last_stand_max_speed_rad_s = float(np.max(np.abs(velocity[active])))
        within_tolerance = (
            self.last_stand_max_error_rad <= self.init_stand_position_tolerance_rad
            and self.last_stand_max_speed_rad_s <= self.init_stand_velocity_tolerance_rad_s
        )

        if self.enable_motor_output and feedback_ready:
            if self.init_stand_output_start_time is None:
                self.init_stand_output_start_time = now_s
            if within_tolerance:
                if self.init_stand_stable_since is None:
                    self.init_stand_stable_since = now_s
            else:
                self.init_stand_stable_since = None
        else:
            self.init_stand_stable_since = None

        stable_time = (
            0.0
            if self.init_stand_stable_since is None
            else max(0.0, now_s - self.init_stand_stable_since)
        )
        # Pre-arm waiting does not count.  The minimum dwell starts only after
        # this enable attempt has a fresh low-level output acknowledgement and
        # usable feedback.
        minimum_time_met = (
            self.init_stand_output_start_time is not None
            and now_s - self.init_stand_output_start_time
            >= self.init_stand_duration_s
        )
        ready = (
            self.enable_motor_output
            and feedback_ready
            and minimum_time_met
            and stable_time >= self.init_stand_stable_time_s
        )
        timeout_reason = None
        if (
            self.init_stand_output_start_time is not None
            and not ready
            and now_s - self.init_stand_output_start_time > self.init_stand_timeout_s
        ):
            timeout_reason = (
                "init stand timeout: "
                f"max_error={self.last_stand_max_error_rad:.3f} rad, "
                f"max_speed={self.last_stand_max_speed_rad_s:.3f} rad/s"
            )
        return ready, timeout_reason

    def _update_sensor_publisher_guard(self, now_s: float) -> None:
        if not self._is_strict_rig_profile():
            self.publisher_guard_reasons = []
            self.publisher_guard_diagnostic = "not required"
            return
        if (
            self.last_publisher_check_time is not None
            and now_s - self.last_publisher_check_time < 0.10
        ):
            return
        self.last_publisher_check_time = now_s
        guarded_topics = [
            ("/joint_states", "redrhex_lowlevel_bridge"),
            ("/motor_feedback", "redrhex_lowlevel_bridge"),
            ("/redrhex/lowlevel_heartbeat", "redrhex_lowlevel_bridge"),
            ("/redrhex/lowlevel_output_enabled", "redrhex_lowlevel_bridge"),
            ("/redrhex/lowlevel_disabled_legs", "redrhex_lowlevel_bridge"),
        ]
        if self.observation_builder.is_full_feedback_rig:
            guarded_topics.append(("/imu/data", self.expected_imu_publisher_node))
        reasons: list[str] = []
        diagnostics: list[str] = []
        for topic, expected_node in guarded_topics:
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception as exc:
                diagnostics.append(f"{topic}=graph-error:{exc}")
                reasons.append(f"cannot verify the unique publisher for {topic}: {exc}")
                continue
            names = [
                f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
                for info in infos
            ]
            diagnostics.append(f"{topic}={len(infos)}:{','.join(names) if names else 'none'}")
            node_names = [str(info.node_name).lstrip("/") for info in infos]
            if len(infos) != 1 or node_names[0] != expected_node.lstrip("/"):
                reasons.append(
                    f"{topic} must have exactly one publisher named {expected_node}, "
                    f"got {names}"
                )
        self.publisher_guard_reasons = reasons
        self.publisher_guard_diagnostic = " | ".join(diagnostics)

    def _control_tick(self) -> None:
        self.control_sequence += 1
        now_s = self._now_s()
        monotonic_now_s = time.monotonic()
        dt = C.CONTROL_DT if self.last_loop_time is None else max(0.0, now_s - self.last_loop_time)
        self.last_loop_time = now_s

        obs_status = self.observation_builder.status(
            now_s,
            self.sensor_timeout_s,
            self.cmd_timeout_s,
            monotonic_now_s=monotonic_now_s,
        )
        # Use projected-gravity tilt, not raw/relative quaternion RPY.  The
        # policy's upright gravity is -Y, and yaw must never trip the tilt gate.
        roll, pitch = self.observation_builder.get_relative_tilt_roll_pitch()
        imu_age = (
            None
            if self.observation_builder.imu_receipt_monotonic_s is None
            else monotonic_now_s
            - self.observation_builder.imu_receipt_monotonic_s
        )
        joint_age = None if self.observation_builder.joint_time is None else now_s - self.observation_builder.joint_time
        motor_age = None if self.last_motor_feedback_time is None else now_s - self.last_motor_feedback_time
        heartbeat_age = None if self.last_lowlevel_heartbeat_time is None else now_s - self.last_lowlevel_heartbeat_time
        output_status_age = (
            None
            if self.last_lowlevel_output_status_time is None
            else now_s - self.last_lowlevel_output_status_time
        )
        lowlevel_mask_age = (
            None
            if self.last_lowlevel_disabled_legs_time is None
            else now_s - self.last_lowlevel_disabled_legs_time
        )
        self.last_diag_values = {
            "imu_age_s": self._fmt_optional(imu_age),
            "joint_state_age_s": self._fmt_optional(joint_age),
            "motor_feedback_age_s": self._fmt_optional(motor_age),
            "heartbeat_age_s": self._fmt_optional(heartbeat_age),
            "lowlevel_output_status_age_s": self._fmt_optional(output_status_age),
            "lowlevel_output_enabled": str(self.lowlevel_output_enabled),
            "lowlevel_disabled_legs_age_s": self._fmt_optional(lowlevel_mask_age),
            "controller_disabled_legs": ",".join(self.disabled_legs) or "none",
            "lowlevel_disabled_legs": (
                "invalid_or_missing"
                if self.lowlevel_disabled_legs is None
                else (",".join(self.lowlevel_disabled_legs) or "none")
            ),
            "control_loop_dt_s": f"{dt:.4f}",
            "inference_duration_s": self._fmt_optional(
                self.last_inference_duration_s
            ),
            "roll_rad": f"{roll:.4f}",
            "pitch_rad": f"{pitch:.4f}",
            "cmd_vel": ",".join(f"{x:.3f}" for x in self.observation_builder.cmd_vel),
            "command_profile": self.command_profile,
            "fixed_forward_vx_m_s": f"{self.fixed_forward_vx:.3f}",
            "policy_run_max_duration_s": f"{self.policy_run_max_duration_s:.3f}",
            "motor_temperature_telemetry": (
                "available" if self.motor_temperatures else "unavailable"
            ),
            "motor_temperature_guard": (
                "active" if self.motor_temperatures else "inactive"
            ),
        }
        self._update_sensor_publisher_guard(now_s)
        self.last_diag_values["sensor_publishers"] = self.publisher_guard_diagnostic

        pre_safety = SafetyState(
            estop=self.estop,
            imu_age_s=imu_age,
            joint_state_age_s=joint_age,
            motor_feedback_age_s=motor_age,
            heartbeat_age_s=heartbeat_age,
            roll_rad=roll,
            pitch_rad=pitch,
            command=self.observation_builder.cmd_vel.copy(),
            motor_temperatures_c=self.motor_temperatures,
            motor_currents_a=self.motor_currents,
            motor_faults=self.motor_faults,
            motor_velocities_rad_s=self.motor_velocities,
            control_loop_dt_s=dt,
            inference_duration_s=None,
            inference_sequence=self.inference_sequence,
            motor_feedback_sequence=self.motor_feedback_sequence,
            control_sequence=self.control_sequence,
        )
        safety_result = self.safety_filter.check(pre_safety)
        if not self._lowlevel_mask_ready(now_s):
            safety_result.reasons.append(
                "low-level disabled-leg mask is stale or does not match the controller"
            )
            safety_result.ok = False
        if self.publisher_guard_reasons:
            safety_result.reasons.extend(self.publisher_guard_reasons)
            safety_result.ok = False
        output_ack_ready = self._lowlevel_output_ack_ready(now_s)
        output_ack_required = self.enable_motor_output and (
            self.state_machine.state
            in (
                RedRhexState.WARMUP,
                RedRhexState.POLICY_READY,
                RedRhexState.POLICY_RUN,
            )
            or self.init_stand_output_start_time is not None
        )
        first_ack_timed_out = (
            self.enable_motor_output
            and self.motor_enable_request_time is not None
            and not output_ack_ready
            and now_s - self.motor_enable_request_time > self.output_enable_ack_timeout_s
        )
        if first_ack_timed_out:
            safety_result.reasons.append(
                "low-level output enable acknowledgement did not arrive before handshake deadline"
            )
            safety_result.ok = False
        if output_ack_required and not output_ack_ready:
            safety_result.reasons.append("low-level output enable acknowledgement lost")
            safety_result.ok = False
        ignore_waiting_timeouts = self.state_machine.state in (RedRhexState.BOOT, RedRhexState.SENSOR_CHECK)
        if not safety_result.ok and not ignore_waiting_timeouts:
            self._drop_enable_latches("; ".join(safety_result.reasons[:3]))
        sm_safety_ok = safety_result.ok or ignore_waiting_timeouts
        if not obs_status.ok and not ignore_waiting_timeouts:
            self._drop_enable_latches("; ".join(obs_status.reasons[:3]))
            sm_safety_ok = False
        if self.estop:
            sm_safety_ok = False

        elapsed_in_state = now_s - self.state_enter_time
        policy_run_timeout_reason: str | None = None
        if self._policy_run_lease_expired(now_s):
            policy_run_timeout_reason = (
                "policy run lease expired after "
                f"{self.policy_run_max_duration_s:.3f} s; motor output disabled"
            )
            self._end_policy_run(policy_run_timeout_reason)
            safety_result.reasons.append(policy_run_timeout_reason)
            elapsed_in_state = 0.0
        motor_ready = motor_age is not None and motor_age <= self.safety_filter.motor_feedback_timeout_s
        bridge_ready = heartbeat_age is not None and heartbeat_age <= self.safety_filter.heartbeat_timeout_s
        stand_feedback_ready = (
            obs_status.ok
            and (motor_ready or not self.require_motor_feedback)
            and (bridge_ready or not self.require_lowlevel_heartbeat)
            and output_ack_ready
        )
        stand_ready, stand_timeout_reason = self._evaluate_init_stand_gate(
            now_s, stand_feedback_ready
        )
        if self.state_machine.state in (
            RedRhexState.INIT_STAND,
            RedRhexState.WARMUP,
            RedRhexState.POLICY_READY,
        ):
            self.init_stand_verified = stand_ready
        if self.state_machine.state == RedRhexState.POLICY_READY and not stand_ready:
            self._restart_init_stand(
                "stand evidence lost before policy enable; re-run init stand"
            )
        stand_stable_time = (
            0.0
            if self.init_stand_stable_since is None
            else max(0.0, now_s - self.init_stand_stable_since)
        )
        self.last_diag_values.update(
            {
                "joint_state_sequence": str(self.observation_builder.joint_sequence),
                "init_stand_max_error_rad": self._fmt_optional(
                    None if not math.isfinite(self.last_stand_max_error_rad) else self.last_stand_max_error_rad
                ),
                "init_stand_max_speed_rad_s": self._fmt_optional(
                    None if not math.isfinite(self.last_stand_max_speed_rad_s) else self.last_stand_max_speed_rad_s
                ),
                "init_stand_stable_time_s": f"{stand_stable_time:.3f}",
                "init_stand_ready": str(stand_ready),
            }
        )
        if stand_timeout_reason is not None and self.state_machine.state in (
            RedRhexState.INIT_STAND,
            RedRhexState.WARMUP,
        ):
            sm_safety_ok = False
        inputs = StateMachineInputs(
            policy_loaded=self.policy_loaded,
            sensors_ready=obs_status.ok,
            motor_feedback_ready=motor_ready,
            lowlevel_alive=bridge_ready,
            estop=self.estop,
            safety_ok=sm_safety_ok,
            fall_detected=self.safety_filter.enable_tilt_guard
            and (
                abs(roll) > self.safety_filter.max_abs_roll_rad
                or abs(pitch) > self.safety_filter.max_abs_pitch_rad
            ),
            init_stand_done=stand_ready,
            warmup_done=stand_ready and elapsed_in_state >= self.warmup_duration_s,
            enable_policy=self.enable_policy,
            recover_requested=self.recover_requested,
            reasons=safety_result.reasons
            + obs_status.reasons
            + ([stand_timeout_reason] if stand_timeout_reason else []),
        )
        old_state = self.state_machine.state
        state = self.state_machine.update(inputs)
        if state != old_state:
            self.state_enter_time = now_s
            self.get_logger().info(f"State transition: {old_state.value} -> {state.value}: {self.state_machine.last_transition_reason}")
            if state == RedRhexState.INIT_STAND:
                self.observation_builder.reset(gait_phase=0.0)
                self.action_decoder.reset(gait_phase=0.0)
                self.init_stand_output_start_time = None
                self.init_stand_stable_since = None
                if self.enable_motor_output and self.motor_enable_request_time is None:
                    self.motor_enable_request_time = now_s
            if state in (RedRhexState.PROTECTIVE_STOP, RedRhexState.FALL_DETECTED, RedRhexState.RECOVER):
                self._drop_enable_latches(self.state_machine.last_transition_reason)

        # The fixed deployment command is deliberately state-aware: it first
        # appears in the exact observation/action tick that enters POLICY_RUN,
        # and is zeroed by every non-running or protective state.
        self.observation_builder.set_policy_running(
            self.state_machine.state == RedRhexState.POLICY_RUN
        )
        pre_safety.command = self.observation_builder.cmd_vel.copy()
        self.last_diag_values["cmd_vel"] = ",".join(
            f"{x:.3f}" for x in self.observation_builder.cmd_vel
        )
        self.last_diag_values["fixed_forward_active"] = str(
            self.command_profile == "fixed_forward"
            and self.state_machine.state == RedRhexState.POLICY_RUN
        )

        decoded: DecodedMotorCommand | None = None
        observation_for_pub: np.ndarray | None = None
        single_observation_for_pub: np.ndarray | None = None
        raw_action: np.ndarray | None = None
        reasons = list(safety_result.reasons) + list(obs_status.reasons)
        if stand_timeout_reason:
            reasons.append(stand_timeout_reason)

        try:
            if obs_status.ok:
                single_observation_for_pub = self.observation_builder.build_single(
                    now_s, update_phase=False
                )
            if state in (RedRhexState.BOOT, RedRhexState.SENSOR_CHECK, RedRhexState.MOTOR_IDLE):
                decoded = self.action_decoder.disabled_command()
            elif state in (RedRhexState.INIT_STAND, RedRhexState.WARMUP, RedRhexState.POLICY_READY):
                decoded = self.action_decoder.init_stand_command(
                    current_main_pos=self.observation_builder.get_main_drive_positions(),
                    enable=True,
                )
            elif state == RedRhexState.POLICY_RUN:
                if self.policy_runner is None:
                    raise RuntimeError("policy runner is not loaded")
                inference_start_ns = time.perf_counter_ns()
                if self.policy_runner.is_sensor_v2:
                    single_obs = self.observation_builder.build_single(now_s)
                    single_observation_for_pub = single_obs
                    sensor_history = self.observation_builder.build_sensor_v2_input(now_s)
                    observation_for_pub = sensor_history.reshape(-1)
                    raw_action = self.policy_runner.run(
                        sensor_history, self.observation_builder.sensor_v2_command()
                    )
                else:
                    observation_for_pub = self.observation_builder.build_policy_input(now_s)
                    single_obs = observation_for_pub[: C.OBS_DIM_SINGLE]
                    single_observation_for_pub = single_obs
                    raw_action = self.policy_runner.run(observation_for_pub)
                self.last_inference_duration_s = (
                    time.perf_counter_ns() - inference_start_ns
                ) * 1.0e-9
                self.inference_sequence += 1
                pre_safety.inference_duration_s = self.last_inference_duration_s
                pre_safety.inference_sequence = self.inference_sequence
                gravity_start, gravity_stop = C.OBSERVATION_SLICES[
                    "projected_gravity"
                ]
                projected_gravity = single_obs[gravity_start:gravity_stop]
                decoded = self.action_decoder.decode(
                    raw_action,
                    self.observation_builder.get_main_drive_positions(),
                    self.observation_builder.get_abad_positions(),
                    self.observation_builder.cmd_vel.copy(),
                    projected_gravity,
                    dt,
                    self.observation_builder.gait_phase,
                )
                post_safety = self.safety_filter.check(pre_safety, single_obs, raw_action, decoded)
                if not post_safety.ok:
                    reasons.extend(post_safety.reasons)
                    self._drop_enable_latches("; ".join(post_safety.reasons[:3]))
                    self.state_machine.transition(RedRhexState.PROTECTIVE_STOP, "; ".join(post_safety.reasons))
                    decoded = self.action_decoder.protective_stop_command(
                        self.observation_builder.get_main_drive_positions(),
                        self.observation_builder.get_abad_positions(),
                    )
                else:
                    self.observation_builder.update_last_actions(raw_action)
                    self.raw_action_pub.publish(Float32MultiArray(data=[float(x) for x in raw_action]))
                    self.safe_action_pub.publish(Float32MultiArray(data=[float(x) for x in decoded.safe_action]))
            else:
                decoded = self.action_decoder.protective_stop_command(
                    self.observation_builder.get_main_drive_positions(),
                    self.observation_builder.get_abad_positions(),
                )
        except Exception as exc:
            reasons.append(str(exc))
            self.get_logger().error(f"Control tick failed: {exc}\n{traceback.format_exc()}")
            self._drop_enable_latches(str(exc))
            self.state_machine.transition(RedRhexState.PROTECTIVE_STOP, str(exc))
            decoded = self.action_decoder.protective_stop_command(
                self.observation_builder.get_main_drive_positions(),
                self.observation_builder.get_abad_positions(),
            )

        actual_motor_enable = self._publish_motor_command(decoded)
        if actual_motor_enable:
            self.observation_builder.update_commanded_abad_position(decoded.target_abad_position, now_s)
        if single_observation_for_pub is not None:
            self.obs_pub.publish(
                Float32MultiArray(data=[float(x) for x in single_observation_for_pub])
            )
            self.imputation_mask_pub.publish(
                Float32MultiArray(
                    data=[
                        1.0 if index in self.observation_builder.imputed_observation_indices() else 0.0
                        for index in range(C.OBS_DIM_SINGLE)
                    ]
                )
            )
        if observation_for_pub is not None:
            self.policy_input_pub.publish(
                Float32MultiArray(data=[float(x) for x in observation_for_pub])
            )
        self._publish_state_and_diagnostics(reasons, safety_result.ok and obs_status.ok)
        self.recover_requested = False

    def _publish_motor_command(self, decoded: DecodedMotorCommand) -> bool:
        msg = RedRhexMotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "redrhex_base"
        msg.joint_names = decoded.joint_names
        msg.target_position_rad = decoded.target_position_rad
        msg.target_velocity_rad_s = decoded.target_velocity_rad_s
        msg.kp = decoded.kp
        msg.kd = decoded.kd
        msg.effort_limit_nm = decoded.effort_limit_nm
        msg.enable = bool(decoded.enable and self.enable_motor_output and not self.estop)
        msg.mode = decoded.mode
        self.motor_cmd_pub.publish(msg)
        return bool(msg.enable)

    @staticmethod
    def _fmt_optional(value: float | None) -> str:
        return "none" if value is None else f"{value:.4f}"

    def _publish_state_and_diagnostics(self, reasons: list[str], ok: bool) -> None:
        state_msg = String()
        state_msg.data = self.state_machine.state.value
        self.state_pub.publish(state_msg)

        status = DiagnosticStatus()
        status.name = "redrhex_rl_controller"
        status.hardware_id = "redrhex"
        status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.WARN
        if (self.disabled_legs or self._is_strict_rig_profile()) and (
            status.level == DiagnosticStatus.OK
        ):
            status.level = DiagnosticStatus.WARN
        if self.state_machine.state in (RedRhexState.PROTECTIVE_STOP, RedRhexState.FALL_DETECTED):
            status.level = DiagnosticStatus.ERROR
        if reasons:
            status.message = "; ".join(reasons[:6])
        elif self.observation_builder.is_encoder_only_rig:
            status.message = (
                "ENCODER_ONLY_RIG: base state is imputed; no IMU tilt/fall protection; suspended rig only"
            )
        elif self.observation_builder.is_full_feedback_rig:
            status.message = (
                "FULL_FEEDBACK_RIG: base linear velocity is imputed; suspended rig only"
            )
        elif self.disabled_legs:
            status.message = f"DEGRADED MODE: disabled legs {','.join(self.disabled_legs)}"
        else:
            status.message = "OK"
        status.values = [
            KeyValue(key="state", value=self.state_machine.state.value),
            KeyValue(key="last_transition_reason", value=self.state_machine.last_transition_reason),
            KeyValue(key="policy_loaded", value=str(self.policy_loaded)),
            KeyValue(key="sensor_profile", value=self.observation_builder.sensor_profile),
            KeyValue(key="requires_imu", value=str(self.observation_builder.requires_imu)),
            KeyValue(
                key="expected_imu_publisher_node",
                value=self.expected_imu_publisher_node or "none",
            ),
            KeyValue(
                key="imputed_observation_indices",
                value=",".join(str(x) for x in self.observation_builder.imputed_observation_indices()),
            ),
            KeyValue(
                key="observation_provenance",
                value=",".join(self.observation_builder.observation_provenance()),
            ),
            KeyValue(
                key="disabled_leg_observation_mode",
                value=self.observation_builder.disabled_leg_observation_mode,
            ),
            KeyValue(key="policy_enabled", value=str(self.enable_policy)),
            KeyValue(key="motor_output_enabled", value=str(self.enable_motor_output and not self.estop)),
            KeyValue(key="estop", value=str(self.estop)),
            KeyValue(key="abad_feedback_source", value=self.abad_feedback_source),
            KeyValue(key="required_joint_names", value=",".join(self.required_joint_names)),
            KeyValue(key="damper_command_enabled", value=str(self.include_damper_command)),
            KeyValue(key="disabled_legs", value=",".join(self.disabled_legs) if self.disabled_legs else "none"),
            KeyValue(key="onnx_path", value=self.onnx_path),
            KeyValue(key="onnx_sha256", value=self.policy_sha256),
            KeyValue(key="observation_contract", value=self.observation_contract_id),
            KeyValue(key="action_contract", value=self.action_contract_id),
            KeyValue(key="decoder_source_sha256", value=self.decoder_source_sha256),
            KeyValue(key="decoder_config_sha256", value=self.decoder_config_sha256),
            KeyValue(
                key="observation_source_sha256",
                value=self.observation_source_sha256,
            ),
            KeyValue(
                key="deployment_source_sha256",
                value=self.deployment_source_sha256,
            ),
            KeyValue(
                key="expected_bridge_config_sha256",
                value=self.expected_bridge_config_sha256 or "missing",
            ),
            KeyValue(
                key="controller_config_sha256",
                value=self.controller_config_sha256,
            ),
            KeyValue(
                key="startup_reference_action_max",
                value=self._fmt_optional(
                    None
                    if not math.isfinite(self.startup_reference_action_max)
                    else self.startup_reference_action_max
                ),
            ),
            KeyValue(
                key="startup_inference_p99_ms",
                value=self._fmt_optional(
                    float(self.startup_inference_benchmark["p99_ms"])
                    if self.startup_inference_benchmark
                    else None
                ),
            ),
        ]
        status.values.extend(KeyValue(key=key, value=value) for key, value in self.last_diag_values.items())
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RedRhexRLControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
