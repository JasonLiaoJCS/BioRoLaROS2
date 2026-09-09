"""Preflight checks before RedRhex real-robot bringup."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

from . import redrhex_contract as C
from .action_decoder import ActionDecoder
from .degraded_mode import normalize_disabled_legs, policy_indices_for_disabled_legs
from .golden_policy import (
    GOLDEN_SCHEMA,
    bridge_config_sha256,
    controller_config_sha256,
    controller_params_with_disabled_legs,
    decoder_config_from_ros_params,
    decoder_config_sha256,
    decoder_source_sha256,
    deployment_source_sha256,
    observation_source_sha256,
    require_complete_bridge_params,
    require_complete_controller_params,
)
from .observation_builder import ObservationBuilder
from .policy_onnx_runner import PolicyONNXRunner
from .policy_validation import check_reference_action, reference_policy_input
from .safety_filter import SafetyFilter


def _append_check(result: dict[str, object], name: str, ok: bool, **fields: object) -> None:
    check = {"name": name, "ok": bool(ok)}
    check.update(fields)
    result["checks"].append(check)


def _load_ros_params(config_path: str | None, result: dict[str, object]) -> dict:
    if not config_path:
        result["warnings"].append("No --config provided; YAML parameter validation skipped.")
        return {}
    path = Path(config_path).expanduser()
    if not path.exists():
        _append_check(result, "config_exists", False, path=str(path))
        return {}
    _append_check(result, "config_exists", True, path=str(path))
    try:
        import yaml
    except Exception as exc:
        result["warnings"].append(f"PyYAML unavailable, YAML validation skipped: {exc}")
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    params = data.get("redrhex_rl_controller", {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        _append_check(result, "config_schema", False, error="missing redrhex_rl_controller.ros__parameters")
        return {}
    _append_check(result, "config_schema", True)
    return params


def _load_bridge_params(
    config_path: str | None,
    result: dict[str, object],
    *,
    require_complete: bool = False,
) -> dict:
    if not config_path:
        result["warnings"].append(
            "No --bridge-config provided; controller/bridge cross-check was skipped."
        )
        return {}
    path = Path(config_path).expanduser()
    if not path.exists():
        _append_check(result, "bridge_config_exists", False, path=str(path))
        return {}
    _append_check(result, "bridge_config_exists", True, path=str(path))
    try:
        import yaml
    except Exception as exc:
        result["warnings"].append(f"PyYAML unavailable, bridge YAML validation skipped: {exc}")
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    params = data.get("redrhex_lowlevel_bridge", {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        _append_check(
            result,
            "bridge_config_schema",
            False,
            error="missing redrhex_lowlevel_bridge.ros__parameters",
        )
        return {}
    _append_check(result, "bridge_config_schema", True)
    if require_complete:
        try:
            require_complete_bridge_params(params)
            _append_check(result, "complete_active_bridge_deployment_profile", True)
        except Exception as exc:
            _append_check(
                result,
                "complete_active_bridge_deployment_profile",
                False,
                error=str(exc),
            )
    return params


def _nested(params: dict, *keys: str, default=None):
    cur = params
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _command_limits_from_params(params: dict) -> dict:
    return {
        "vx_min": float(_nested(params, "commands", "vx_min", default=C.COMMAND_LIMITS["vx_min"])),
        "vx_max": float(_nested(params, "commands", "vx_max", default=C.COMMAND_LIMITS["vx_max"])),
        "vy_min": float(_nested(params, "commands", "vy_min", default=C.COMMAND_LIMITS["vy_min"])),
        "vy_max": float(_nested(params, "commands", "vy_max", default=C.COMMAND_LIMITS["vy_max"])),
        "wz_min": float(_nested(params, "commands", "wz_min", default=C.COMMAND_LIMITS["wz_min"])),
        "wz_max": float(_nested(params, "commands", "wz_max", default=C.COMMAND_LIMITS["wz_max"])),
    }


def _validate_deployment_config(params: dict, policy_input_dim: int, result: dict[str, object]) -> None:
    if not params:
        return
    try:
        require_complete_controller_params(params)
        _append_check(result, "complete_controller_deployment_profile", True)
    except Exception as exc:
        _append_check(
            result,
            "complete_controller_deployment_profile",
            False,
            error=str(exc),
        )
    command_limits = _command_limits_from_params(params)
    command_profile = str(
        _nested(params, "commands", "profile", default="external_cmd_vel")
    )
    fixed_forward_vx = float(
        _nested(params, "commands", "fixed_forward_vx", default=0.22)
    )
    policy_run_max_duration = float(
        _nested(
            params,
            "state_machine",
            "policy_run_max_duration_s",
            default=0.0,
        )
    )
    _append_check(
        result,
        "fixed_forward_run_lease",
        command_profile != "fixed_forward"
        or (
            math.isfinite(policy_run_max_duration)
            and 0.0 < policy_run_max_duration <= 10.0
        ),
        command_profile=command_profile,
        policy_run_max_duration_s=policy_run_max_duration,
    )
    disabled_legs: list[str] = []
    disabled_indices: list[int] = []
    try:
        disabled_legs = normalize_disabled_legs(
            list(_nested(params, "hardware", "disabled_legs", default=[])),
            int(_nested(params, "hardware", "max_disabled_legs", default=1)),
        )
        disabled_indices = policy_indices_for_disabled_legs(disabled_legs)
        _append_check(result, "disabled_legs_config", True, disabled_legs=disabled_legs)
    except Exception as exc:
        _append_check(result, "disabled_legs_config", False, error=str(exc))
    sensor_profile = str(
        _nested(params, "observation", "sensor_profile", default="full_state")
    )
    builder: ObservationBuilder | None = None
    try:
        builder = ObservationBuilder(
            {
                "sensor_profile": sensor_profile,
                "encoder_only_rig_acknowledged": bool(
                    _nested(
                        params,
                        "observation",
                        "encoder_only_rig_acknowledged",
                        default=False,
                    )
                ),
                "full_feedback_rig_acknowledged": bool(
                    _nested(
                        params,
                        "observation",
                        "full_feedback_rig_acknowledged",
                        default=False,
                    )
                ),
                "imu_alignment_calibrated": bool(
                    _nested(
                        params,
                        "observation",
                        "imu_alignment_calibrated",
                        default=False,
                    )
                ),
                "expected_imu_frame_id": str(
                    _nested(
                        params,
                        "observation",
                        "expected_imu_frame_id",
                        default="",
                    )
                ).strip(),
                "expected_imu_publisher_node": str(
                    _nested(
                        params,
                        "observation",
                        "expected_imu_publisher_node",
                        default="",
                    )
                ).strip(),
                "imu_policy_to_sensor_quaternion_xyzw": list(
                    _nested(
                        params,
                        "observation",
                        "imu_policy_to_sensor_quaternion_xyzw",
                        default=[0.0, 0.0, 0.0, 1.0],
                    )
                ),
                "imu_upright_quaternion_xyzw": list(
                    _nested(
                        params,
                        "observation",
                        "imu_upright_quaternion_xyzw",
                        default=[math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
                    )
                ),
                "rig_projected_gravity": list(
                    _nested(
                        params,
                        "observation",
                        "rig_projected_gravity",
                        default=C.REFERENCE_PROJECTED_GRAVITY,
                    )
                ),
                "expected_obs_dim": C.OBS_DIM_SINGLE,
                "policy_input_dim": int(policy_input_dim),
                "policy_history_length": int(
                    _nested(params, "observation", "policy_history_length", default=C.POLICY_HISTORY_LENGTH)
                ),
                "base_lin_vel_source": str(_nested(params, "observation", "base_lin_vel_source", default="zero")),
                "odom_twist_in_body_frame": bool(
                    _nested(params, "observation", "odom_twist_in_body_frame", default=True)
                ),
                "abad_feedback_source": str(_nested(params, "observation", "abad_feedback_source", default="commanded")),
                "estimate_missing_joint_velocity": bool(
                    _nested(params, "observation", "estimate_missing_joint_velocity", default=True)
                ),
                "observation_clip": float(
                    _nested(params, "observation", "observation_clip", default=100.0)
                ),
                "require_imu_source_stamp": bool(
                    _nested(params, "observation", "require_imu_source_stamp", default=False)
                ),
                "max_imu_source_age_s": float(
                    _nested(params, "observation", "max_imu_source_age_s", default=0.10)
                ),
                "max_imu_future_skew_s": float(
                    _nested(params, "observation", "max_imu_future_skew_s", default=0.02)
                ),
                "disabled_leg_indices": disabled_indices,
                "disabled_leg_observation_mode": str(
                    _nested(
                        params,
                        "observation",
                        "disabled_leg_observation_mode",
                        default="passthrough",
                    )
                ),
                "command_limits": command_limits,
                "command_profile": command_profile,
                "fixed_forward_vx": fixed_forward_vx,
            }
        )
        _append_check(
            result,
            "observation_builder_config",
            True,
            sensor_profile=builder.sensor_profile,
            imputed_indices=builder.imputed_observation_indices(),
            provenance=builder.observation_provenance(),
        )
    except Exception as exc:
        _append_check(result, "observation_builder_config", False, error=str(exc))

    try:
        ActionDecoder(
            {
                "action_clip": float(_nested(params, "safety", "action_clip", default=1.0)),
                "max_raw_action_abs": float(
                    _nested(params, "safety", "max_raw_action_abs", default=1.5)
                ),
                "main_drive_vel_limit_rad_s": float(
                    _nested(params, "safety", "main_drive_vel_limit_rad_s", default=30.0)
                ),
                "abad_pos_limit": float(_nested(params, "safety", "abad_pos_limit_rad", default=C.STAGE_ABAD_POS_LIMIT)),
                "main_drive_slew_rate_rad_s2": float(
                    _nested(params, "safety", "main_drive_slew_rate_rad_s2", default=120.0)
                ),
                "abad_slew_rate_rad_s": float(_nested(params, "safety", "abad_slew_rate_rad_s", default=6.0)),
                "include_damper_command": bool(_nested(params, "action", "include_damper_command", default=False)),
                "main_drive_init_control_mode": str(
                    _nested(params, "action", "main_drive_init_control_mode", default="velocity_to_pose")
                ),
                "init_stand_main_drive_position_gain": float(
                    _nested(params, "action", "init_stand_main_drive_position_gain", default=3.0)
                ),
                "init_stand_max_main_drive_vel_rad_s": float(
                    _nested(params, "action", "init_stand_max_main_drive_vel_rad_s", default=1.5)
                ),
                "main_drive_sign": list(_nested(params, "action", "main_drive_sign", default=[1.0] * 6)),
                "abad_sign": list(_nested(params, "action", "abad_sign", default=[1.0] * 6)),
                "damper_sign": list(_nested(params, "action", "damper_sign", default=[1.0] * 6)),
                "main_drive_zero_offset_rad": list(
                    _nested(params, "action", "main_drive_zero_offset_rad", default=[0.0] * 6)
                ),
                "abad_zero_offset_rad": list(_nested(params, "action", "abad_zero_offset_rad", default=[0.0] * 6)),
                "damper_zero_offset_rad": list(
                    _nested(params, "action", "damper_zero_offset_rad", default=[0.0] * 6)
                ),
                "main_drive_kp": list(_nested(params, "action", "main_drive_kp", default=[0.0] * 6)),
                "main_drive_kd": list(_nested(params, "action", "main_drive_kd", default=[50.0] * 6)),
                "abad_kp": list(_nested(params, "action", "abad_kp", default=[40.0] * 6)),
                "abad_kd": list(_nested(params, "action", "abad_kd", default=[4.0] * 6)),
                "stand_main_drive_kp": list(
                    _nested(params, "action", "stand_main_drive_kp", default=[12.0] * 6)
                ),
                "stand_main_drive_kd": list(_nested(params, "action", "stand_main_drive_kd", default=[1.0] * 6)),
                "play_forward_compat_enable": bool(
                    _nested(params, "action", "play_forward_compat_enable", default=False)
                ),
                "play_forward_compat_bias_scale": float(
                    _nested(params, "action", "play_forward_compat_bias_scale", default=1.0)
                ),
                "play_forward_compat_residual_scale": float(
                    _nested(params, "action", "play_forward_compat_residual_scale", default=0.04)
                ),
                "play_forward_compat_residual_clip": float(
                    _nested(params, "action", "play_forward_compat_residual_clip", default=0.30)
                ),
                "disabled_leg_indices": disabled_indices,
            }
        )
        _append_check(result, "action_decoder_config", True)
    except Exception as exc:
        _append_check(result, "action_decoder_config", False, error=str(exc))

    try:
        SafetyFilter(
            {
                "sensor_timeout_s": float(_nested(params, "safety", "sensor_timeout_s", default=0.10)),
                "cmd_timeout_s": float(_nested(params, "safety", "cmd_timeout_s", default=0.25)),
                "motor_feedback_timeout_s": float(
                    _nested(params, "safety", "motor_feedback_timeout_s", default=0.25)
                ),
                "heartbeat_timeout_s": float(_nested(params, "safety", "heartbeat_timeout_s", default=0.10)),
                "max_abs_roll_rad": float(_nested(params, "safety", "max_abs_roll_rad", default=0.7)),
                "max_abs_pitch_rad": float(_nested(params, "safety", "max_abs_pitch_rad", default=0.7)),
                "action_clip": float(_nested(params, "safety", "action_clip", default=1.0)),
                "main_drive_vel_limit_rad_s": float(
                    _nested(params, "safety", "main_drive_vel_limit_rad_s", default=30.0)
                ),
                "abad_pos_limit_rad": float(_nested(params, "safety", "abad_pos_limit_rad", default=0.7)),
                "max_motor_temperature_c": float(
                    _nested(params, "safety", "max_motor_temperature_c", default=70.0)
                ),
                "max_motor_current_a": float(_nested(params, "safety", "max_motor_current_a", default=20.0)),
                "max_measured_main_drive_velocity_rad_s": float(
                    _nested(
                        params,
                        "safety",
                        "max_measured_main_drive_velocity_rad_s",
                        default=80.0,
                    )
                ),
                "max_control_loop_dt_s": float(_nested(params, "safety", "max_control_loop_dt_s", default=0.03)),
                "max_inference_duration_s": float(
                    _nested(params, "safety", "max_inference_duration_s", default=0.008)
                ),
                "motor_current_trip_samples": int(
                    _nested(params, "safety", "motor_current_trip_samples", default=5)
                ),
                "control_loop_trip_samples": int(
                    _nested(params, "safety", "control_loop_trip_samples", default=3)
                ),
                "inference_trip_samples": int(
                    _nested(params, "safety", "inference_trip_samples", default=3)
                ),
                "disabled_leg_indices": disabled_indices,
                "require_motor_feedback": bool(
                    _nested(params, "safety", "require_motor_feedback", default=False)
                ),
                "require_lowlevel_heartbeat": bool(
                    _nested(params, "safety", "require_lowlevel_heartbeat", default=False)
                ),
                "require_imu": (
                    builder.requires_imu
                    if builder is not None
                    else sensor_profile in ("full_state", "full_feedback_rig")
                ),
                "enable_tilt_guard": (
                    builder.requires_imu
                    if builder is not None
                    else sensor_profile in ("full_state", "full_feedback_rig")
                ),
                "command_limits": command_limits,
            }
        )
        _append_check(result, "safety_filter_config", True)
    except Exception as exc:
        _append_check(result, "safety_filter_config", False, error=str(exc))

    if disabled_legs:
        result["warnings"].append(
            f"Degraded mode is configured for {disabled_legs}; keep those legs physically isolated and test suspended."
        )

    if bool(_nested(params, "state_machine", "enable_policy_on_start", default=False)):
        result["warnings"].append("enable_policy_on_start is true; keep it false for real-robot bringup.")
    if bool(_nested(params, "state_machine", "enable_motor_output_on_start", default=False)):
        result["warnings"].append("enable_motor_output_on_start is true; keep it false for real-robot bringup.")
    if bool(_nested(params, "action", "include_damper_command", default=False)):
        result["warnings"].append("include_damper_command is true, but real RedRhex dampers are not motors.")
    if str(_nested(params, "observation", "abad_feedback_source", default="commanded")) != "commanded":
        result["warnings"].append(
            "ABAD joint_states feedback is selected; bridge calibration acknowledgement will be checked."
        )
    if str(_nested(params, "observation", "base_lin_vel_source", default="zero")) == "zero":
        result["warnings"].append(
            "base_lin_vel_source is zero. This is only safe for bench tests; use odom/leg odometry before serious locomotion."
        )
    if sensor_profile in ("encoder_only_rig", "full_feedback_rig"):
        is_encoder_only_rig = sensor_profile == "encoder_only_rig"
        is_full_feedback_rig = sensor_profile == "full_feedback_rig"
        action_clip = float(_nested(params, "safety", "action_clip", default=1.0))
        max_raw_action = float(
            _nested(params, "safety", "max_raw_action_abs", default=1.5)
        )
        main_limit = float(
            _nested(params, "safety", "main_drive_vel_limit_rad_s", default=30.0)
        )
        abad_limit = float(_nested(params, "safety", "abad_pos_limit_rad", default=0.7))
        measured_limit = float(
            _nested(
                params,
                "safety",
                "max_measured_main_drive_velocity_rad_s",
                default=80.0,
            )
        )
        configured_policy_hz = float(
            _nested(params, "policy", "policy_hz", default=0.0)
        )
        resolved_policy_hz = (
            configured_policy_hz if configured_policy_hz > 0.0 else C.POLICY_HZ
        )
        output_ack_timeout = float(
            _nested(
                params,
                "state_machine",
                "output_enable_ack_timeout_s",
                default=0.15,
            )
        )
        heartbeat_timeout = float(
            _nested(params, "safety", "heartbeat_timeout_s", default=0.10)
        )
        sensor_timeout = float(
            _nested(params, "safety", "sensor_timeout_s", default=0.10)
        )
        cmd_timeout = float(
            _nested(params, "safety", "cmd_timeout_s", default=0.25)
        )
        motor_feedback_timeout = float(
            _nested(params, "safety", "motor_feedback_timeout_s", default=0.25)
        )
        max_temperature = float(
            _nested(params, "safety", "max_motor_temperature_c", default=70.0)
        )
        max_abs_roll = float(
            _nested(params, "safety", "max_abs_roll_rad", default=0.7)
        )
        max_abs_pitch = float(
            _nested(params, "safety", "max_abs_pitch_rad", default=0.7)
        )
        max_current = float(
            _nested(params, "safety", "max_motor_current_a", default=20.0)
        )
        max_control_dt = float(
            _nested(params, "safety", "max_control_loop_dt_s", default=0.03)
        )
        max_inference_duration = float(
            _nested(params, "safety", "max_inference_duration_s", default=0.008)
        )
        current_trip_samples = int(
            _nested(params, "safety", "motor_current_trip_samples", default=5)
        )
        loop_trip_samples = int(
            _nested(params, "safety", "control_loop_trip_samples", default=3)
        )
        inference_trip_samples = int(
            _nested(params, "safety", "inference_trip_samples", default=3)
        )
        main_slew = float(
            _nested(params, "safety", "main_drive_slew_rate_rad_s2", default=120.0)
        )
        abad_slew = float(
            _nested(params, "safety", "abad_slew_rate_rad_s", default=6.0)
        )
        init_stand_speed = float(
            _nested(
                params,
                "action",
                "init_stand_max_main_drive_vel_rad_s",
                default=1.5,
            )
        )
        startup_action_limit = float(
            _nested(params, "policy", "startup_action_abs_limit", default=1.5)
        )
        max_inference_p99_ms = float(
            _nested(params, "policy", "max_inference_p99_ms", default=6.0)
        )
        inference_benchmark_runs = int(
            _nested(params, "policy", "inference_benchmark_runs", default=200)
        )
        init_duration = float(
            _nested(params, "state_machine", "init_stand_duration_s", default=2.0)
        )
        init_timeout = float(
            _nested(params, "state_machine", "init_stand_timeout_s", default=12.0)
        )
        stable_duration = float(
            _nested(params, "state_machine", "init_stand_stable_time_s", default=0.50)
        )
        warmup_duration = float(
            _nested(params, "state_machine", "warmup_duration_s", default=1.0)
        )
        stand_position_tolerance = float(
            _nested(
                params,
                "state_machine",
                "init_stand_position_tolerance_rad",
                default=0.12,
            )
        )
        stand_velocity_tolerance = float(
            _nested(
                params,
                "state_machine",
                "init_stand_velocity_tolerance_rad_s",
                default=0.25,
            )
        )
        rig_gravity = np.asarray(
            _nested(
                params,
                "observation",
                "rig_projected_gravity",
                default=C.REFERENCE_PROJECTED_GRAVITY,
            ),
            dtype=np.float64,
        )
        profile_acknowledged = bool(
            _nested(
                params,
                "observation",
                (
                    "encoder_only_rig_acknowledged"
                    if is_encoder_only_rig
                    else "full_feedback_rig_acknowledged"
                ),
                default=False,
            )
        )
        rig_gravity_matches = (
            not is_encoder_only_rig
            or (
                rig_gravity.shape == (3,)
                and np.allclose(
                    rig_gravity,
                    np.asarray(C.REFERENCE_PROJECTED_GRAVITY, dtype=np.float64),
                    rtol=0.0,
                    atol=1.0e-9,
                )
            )
        )
        expected_imu_frame_id = str(
            _nested(
                params,
                "observation",
                "expected_imu_frame_id",
                default="",
            )
        ).strip()
        expected_imu_publisher_node = str(
            _nested(
                params,
                "observation",
                "expected_imu_publisher_node",
                default="",
            )
        ).strip()
        full_feedback_contract = (
            not is_full_feedback_rig
            or (
                bool(
                    _nested(
                        params,
                        "observation",
                        "imu_alignment_calibrated",
                        default=False,
                    )
                )
                and str(
                    _nested(
                        params,
                        "observation",
                        "base_lin_vel_source",
                        default="zero",
                    )
                )
                == "zero"
                and str(
                    _nested(
                        params,
                        "observation",
                        "abad_feedback_source",
                        default="commanded",
                    )
                )
                == "joint_states"
                and bool(expected_imu_frame_id)
                and bool(expected_imu_publisher_node)
                and bool(
                    _nested(
                        params,
                        "observation",
                        "require_imu_source_stamp",
                        default=False,
                    )
                )
                and builder is not None
                and builder.requires_imu
                and command_profile == "fixed_forward"
            )
        )
        invariants = (
            profile_acknowledged
            and int(
                _nested(params, "hardware", "max_disabled_legs", default=1)
            ) == 1
            and bool(
                _nested(params, "policy", "require_contract_metadata", default=False)
            )
            and not bool(_nested(params, "policy", "use_cuda", default=False))
            and not bool(_nested(params, "policy", "use_tensorrt", default=False))
            and re.fullmatch(
                r"[0-9a-f]{64}",
                str(
                    _nested(
                        params,
                        "policy",
                        "expected_bridge_config_sha256",
                        default="",
                    )
                ).strip(),
            )
            is not None
            and bool(
                _nested(params, "action", "hardware_mapping_calibrated", default=False)
            )
            and not bool(_nested(params, "use_sim_time", default=False))
            and int(
                _nested(
                    params,
                    "observation",
                    "policy_history_length",
                    default=C.POLICY_HISTORY_LENGTH,
                )
            )
            == C.POLICY_HISTORY_LENGTH
            and (
                not disabled_indices
                or str(
                    _nested(
                        params,
                        "observation",
                        "disabled_leg_observation_mode",
                        default="passthrough",
                    )
                )
                == "nominal"
            )
            and bool(_nested(params, "safety", "require_motor_feedback", default=False))
            and bool(_nested(params, "safety", "require_lowlevel_heartbeat", default=False))
            and math.isclose(
                resolved_policy_hz, C.POLICY_HZ, rel_tol=0.0, abs_tol=1.0e-9
            )
            and 0.0 < output_ack_timeout <= 0.15
            and 0.0 < policy_run_max_duration <= 300.0
            and 0.0 < heartbeat_timeout <= 0.15
            and 0.0 < sensor_timeout <= 0.10
            and 0.0 < cmd_timeout <= 0.25
            and 0.0 < motor_feedback_timeout <= 0.25
            and init_duration >= 2.0
            and 0.0 < init_timeout <= 60.0
            and stable_duration >= 0.50
            and warmup_duration >= 1.0
            and 0.0 < stand_position_tolerance <= 0.12
            and 0.0 < stand_velocity_tolerance <= 0.25
            and rig_gravity_matches
            and full_feedback_contract
            and not bool(
                _nested(params, "state_machine", "enable_policy_on_start", default=False)
            )
            and not bool(
                _nested(
                    params,
                    "state_machine",
                    "enable_motor_output_on_start",
                    default=False,
                )
            )
            and 0.0 < max_abs_roll <= 0.45
            and 0.0 < max_abs_pitch <= 0.45
            and action_clip <= 0.35
            and max_raw_action <= 1.5
            and main_limit <= 1.0
            and abad_limit <= 0.18
            and measured_limit <= 4.0
            and max_current <= 10.0
            and max_control_dt <= 0.03
            and max_inference_duration <= 0.016
            and 0 < current_trip_samples <= 100
            and 0 < loop_trip_samples <= 3
            and 0 < inference_trip_samples <= 3
            and main_slew <= 12.0
            and abad_slew <= 1.0
            and init_stand_speed <= 0.35
            and startup_action_limit <= 1.5
            and 0.0 < max_inference_p99_ms <= 6.0
            and inference_benchmark_runs >= 100
            and 0.0 <= command_limits["vx_min"] <= 0.22
            and math.isclose(
                command_limits["vx_max"], 0.22, rel_tol=0.0, abs_tol=1.0e-12
            )
            and -0.14 <= command_limits["vy_min"] <= 0.14
            and -0.14 <= command_limits["vy_max"] <= 0.14
            and -0.17 <= command_limits["wz_min"] <= 0.17
            and -0.17 <= command_limits["wz_max"] <= 0.17
            and command_profile == "fixed_forward"
            and math.isclose(
                fixed_forward_vx, 0.22, rel_tol=0.0, abs_tol=1.0e-12
            )
            and command_limits["vx_min"] <= fixed_forward_vx <= command_limits["vx_max"]
            and command_limits["vy_min"] <= 0.0 <= command_limits["vy_max"]
            and command_limits["wz_min"] <= 0.0 <= command_limits["wz_max"]
            and bool(
                _nested(params, "action", "play_forward_compat_enable", default=False)
            )
            and math.isclose(
                float(
                    _nested(
                        params,
                        "action",
                        "play_forward_compat_bias_scale",
                        default=1.0,
                    )
                ),
                1.0,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
            and math.isclose(
                float(
                    _nested(
                        params,
                        "action",
                        "play_forward_compat_residual_scale",
                        default=0.04,
                    )
                ),
                0.04,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
            and math.isclose(
                float(
                    _nested(
                        params,
                        "action",
                        "play_forward_compat_residual_clip",
                        default=0.30,
                    )
                ),
                0.30,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
        )
        _append_check(
            result,
            f"{sensor_profile}_invariants",
            invariants,
            profile_acknowledged=profile_acknowledged,
            imu_alignment_calibrated=bool(
                _nested(
                    params,
                    "observation",
                    "imu_alignment_calibrated",
                    default=False,
                )
            ),
            expected_imu_frame_id=expected_imu_frame_id or "missing",
            expected_imu_publisher_node=expected_imu_publisher_node or "missing",
            require_imu_source_stamp=bool(
                _nested(
                    params,
                    "observation",
                    "require_imu_source_stamp",
                    default=False,
                )
            ),
            hardware_mapping_calibrated=bool(
                _nested(params, "action", "hardware_mapping_calibrated", default=False)
            ),
            expected_bridge_config_sha256=str(
                _nested(
                    params,
                    "policy",
                    "expected_bridge_config_sha256",
                    default="",
                )
            )
            or "missing",
            base_lin_vel_source=str(
                _nested(
                    params,
                    "observation",
                    "base_lin_vel_source",
                    default="zero",
                )
            ),
            abad_feedback_source=str(
                _nested(
                    params,
                    "observation",
                    "abad_feedback_source",
                    default="commanded",
                )
            ),
            require_contract_metadata=bool(
                _nested(params, "policy", "require_contract_metadata", default=False)
            ),
            max_disabled_legs=int(
                _nested(params, "hardware", "max_disabled_legs", default=1)
            ),
            policy_hz=resolved_policy_hz,
            output_enable_ack_timeout_s=output_ack_timeout,
            policy_run_max_duration_s=policy_run_max_duration,
            heartbeat_timeout_s=heartbeat_timeout,
            sensor_timeout_s=sensor_timeout,
            cmd_timeout_s=cmd_timeout,
            motor_feedback_timeout_s=motor_feedback_timeout,
            rig_projected_gravity=rig_gravity.tolist(),
            init_stand_duration_s=init_duration,
            init_stand_timeout_s=init_timeout,
            init_stand_stable_time_s=stable_duration,
            warmup_duration_s=warmup_duration,
            init_stand_position_tolerance_rad=stand_position_tolerance,
            init_stand_velocity_tolerance_rad_s=stand_velocity_tolerance,
            action_clip=action_clip,
            max_raw_action_abs=max_raw_action,
            main_limit_rad_s=main_limit,
            abad_limit_rad=abad_limit,
            measured_speed_limit_rad_s=measured_limit,
            max_motor_temperature_c=max_temperature,
            motor_temperature_telemetry="unavailable_on_rinbo_backend",
            motor_temperature_guard="inactive_without_telemetry",
            max_motor_current_a=max_current,
            max_control_loop_dt_s=max_control_dt,
            max_inference_duration_s=max_inference_duration,
            motor_current_trip_samples=current_trip_samples,
            control_loop_trip_samples=loop_trip_samples,
            inference_trip_samples=inference_trip_samples,
            main_drive_slew_rate_rad_s2=main_slew,
            abad_slew_rate_rad_s=abad_slew,
            init_stand_max_main_drive_vel_rad_s=init_stand_speed,
            startup_action_abs_limit=startup_action_limit,
            max_inference_p99_ms=max_inference_p99_ms,
            inference_benchmark_runs=inference_benchmark_runs,
            command_limits=command_limits,
            command_profile=command_profile,
            fixed_forward_vx_m_s=fixed_forward_vx,
            play_forward_compat_enable=bool(
                _nested(params, "action", "play_forward_compat_enable", default=False)
            ),
        )
        if command_profile == "fixed_forward":
            result["warnings"].append(
                "FIXED_FORWARD is active: POLICY_RUN ignores /cmd_vel and uses "
                f"[vx,vy,wz]=[{fixed_forward_vx:.3f},0,0]; all non-run states use zero."
            )
        else:
            result["warnings"].append(
                "external_cmd_vel is selected: without a fresh /cmd_vel message the command remains zero."
            )
        if is_encoder_only_rig:
            result["warnings"].append(
                "ENCODER_ONLY_RIG has no measured base motion, attitude, tilt, or fall protection; suspended rig only."
            )
        else:
            result["warnings"].append(
                "FULL_FEEDBACK_RIG measures joint and IMU state, but base linear velocity is still zero-imputed; suspended rig only."
            )


def _validate_bridge_config(
    controller_params: dict, bridge_params: dict, result: dict[str, object]
) -> None:
    if not bridge_params:
        return
    try:
        controller_disabled_legs = normalize_disabled_legs(
            list(_nested(controller_params, "hardware", "disabled_legs", default=[]) or []),
            int(_nested(controller_params, "hardware", "max_disabled_legs", default=1)),
        )
        bridge_disabled_legs = normalize_disabled_legs(
            list(_nested(bridge_params, "hardware", "disabled_legs", default=[]) or []),
            int(_nested(bridge_params, "hardware", "max_disabled_legs", default=1)),
        )
        mask_contract_ok = set(controller_disabled_legs) == set(bridge_disabled_legs)
    except (TypeError, ValueError) as exc:
        controller_disabled_legs = []
        bridge_disabled_legs = []
        mask_contract_ok = False
        result["warnings"].append(f"invalid controller/bridge disabled-leg mask: {exc}")
    _append_check(
        result,
        "controller_bridge_disabled_leg_contract",
        mask_contract_ok,
        controller_disabled_legs=controller_disabled_legs,
        bridge_disabled_legs=bridge_disabled_legs,
    )
    backend = str(_nested(bridge_params, "backend", default=""))
    if backend in ("rinbo_ros", "biorola_ros"):
        result["warnings"].append(
            "Rinbo motor_temperature_telemetry=unavailable; the configured 55 C "
            "controller guard is inactive and must not be treated as protection."
        )
    disabled_handshake_repeats = int(
        _nested(
            bridge_params,
            "rinbo",
            "disabled_handshake_repeats",
            default=0,
        )
    )
    downstream_ack_timeout_s = float(
        _nested(
            bridge_params,
            "rinbo",
            "downstream_output_ack_timeout_s",
            default=float("inf"),
        )
    )
    safety_ok = (
        backend in ("rinbo_ros", "biorola_ros")
        and bool(_nested(bridge_params, "rinbo", "require_state", default=False))
        and bool(_nested(bridge_params, "rinbo", "require_power_state", default=False))
        and bool(_nested(bridge_params, "rinbo", "require_power_relay", default=False))
        and not bool(_nested(bridge_params, "rinbo", "publish_when_disabled", default=True))
        and not bool(_nested(bridge_params, "rinbo", "allow_enable", default=True))
        and disabled_handshake_repeats >= 5
        and bool(
            _nested(
                bridge_params,
                "rinbo",
                "require_downstream_output_ack",
                default=False,
            )
        )
        and bool(
            str(
                _nested(
                    bridge_params,
                    "rinbo",
                    "downstream_output_ack_topic",
                    default="",
                )
            ).strip()
        )
        and math.isfinite(downstream_ack_timeout_s)
        and 0.0 < downstream_ack_timeout_s <= 0.10
        and bool(
            _nested(
                bridge_params,
                "rinbo",
                "block_if_duplicate_upstream_publishers",
                default=False,
            )
        )
        and bool(
            _nested(
                bridge_params,
                "rinbo",
                "require_exact_command_contract",
                default=False,
            )
        )
        and bool(
            _nested(
                bridge_params,
                "rinbo",
                "require_single_telemetry_publisher",
                default=False,
            )
        )
        and bool(
            _nested(
                bridge_params,
                "rinbo",
                "require_monotonic_telemetry_stamp",
                default=False,
            )
        )
        and bool(
            str(
                _nested(
                    bridge_params,
                    "rinbo",
                    "expected_telemetry_publisher_node",
                    default="",
                )
            ).strip()
        )
    )
    _append_check(
        result,
        "bridge_bench_safe_invariants",
        safety_ok,
        backend=backend,
        allow_enable=_nested(bridge_params, "rinbo", "allow_enable", default=None),
        require_state=_nested(bridge_params, "rinbo", "require_state", default=None),
        require_power_state=_nested(
            bridge_params, "rinbo", "require_power_state", default=None
        ),
        require_power_relay=_nested(
            bridge_params, "rinbo", "require_power_relay", default=None
        ),
        block_if_duplicate_upstream_publishers=_nested(
            bridge_params,
            "rinbo",
            "block_if_duplicate_upstream_publishers",
            default=None,
        ),
        require_exact_command_contract=_nested(
            bridge_params, "rinbo", "require_exact_command_contract", default=None
        ),
        require_single_telemetry_publisher=_nested(
            bridge_params,
            "rinbo",
            "require_single_telemetry_publisher",
            default=None,
        ),
        require_monotonic_telemetry_stamp=_nested(
            bridge_params,
            "rinbo",
            "require_monotonic_telemetry_stamp",
            default=None,
        ),
        disabled_handshake_repeats=disabled_handshake_repeats,
        require_downstream_output_ack=_nested(
            bridge_params,
            "rinbo",
            "require_downstream_output_ack",
            default=None,
        ),
        downstream_output_ack_timeout_s=downstream_ack_timeout_s,
    )

    require_abad_calibration = bool(
        _nested(
            bridge_params,
            "rinbo",
            "require_abad_command_calibration",
            default=False,
        )
    )
    abad_command_calibrated = bool(
        _nested(
            bridge_params,
            "rinbo",
            "abad_command_calibrated",
            default=False,
        )
    )
    _append_check(
        result,
        "bridge_abad_command_calibration",
        (not require_abad_calibration) or abad_command_calibrated,
        required=require_abad_calibration,
        acknowledged=abad_command_calibrated,
    )

    controller_main_limit = float(
        _nested(
            controller_params,
            "safety",
            "main_drive_vel_limit_rad_s",
            default=30.0,
        )
    )
    controller_abad_limit = float(
        _nested(controller_params, "safety", "abad_pos_limit_rad", default=0.7)
    )
    bridge_main_limit = float(
        _nested(
            bridge_params,
            "rinbo",
            "max_main_target_velocity_rad_s",
            default=float("inf"),
        )
    )
    bridge_abad_limit = float(
        _nested(
            bridge_params,
            "rinbo",
            "max_abad_target_position_rad",
            default=float("inf"),
        )
    )
    upstream_max_age = float(
        _nested(
            bridge_params,
            "rinbo",
            "upstream_command_max_age_s",
            default=float("inf"),
        )
    )
    _append_check(
        result,
        "bridge_final_semantic_limits",
        math.isfinite(bridge_main_limit)
        and math.isfinite(bridge_abad_limit)
        and bridge_main_limit <= controller_main_limit
        and bridge_abad_limit <= controller_abad_limit
        and math.isfinite(upstream_max_age)
        and 0.0 < upstream_max_age <= 0.10,
        controller_main_limit_rad_s=controller_main_limit,
        bridge_main_limit_rad_s=bridge_main_limit,
        controller_abad_limit_rad=controller_abad_limit,
        bridge_abad_limit_rad=bridge_abad_limit,
        upstream_command_max_age_s=upstream_max_age,
    )

    abad_source = str(
        _nested(controller_params, "observation", "abad_feedback_source", default="commanded")
    )
    publish_abad = bool(
        _nested(bridge_params, "rinbo", "publish_abad_joint_feedback", default=False)
    )
    abad_calibrated = bool(
        _nested(bridge_params, "rinbo", "abad_feedback_calibrated", default=False)
    )
    abad_match = abad_source != "joint_states" or (publish_abad and abad_calibrated)
    _append_check(
        result,
        "controller_bridge_abad_feedback_contract",
        abad_match,
        controller_source=abad_source,
        bridge_publishes_abad=publish_abad,
        bridge_calibration_acknowledged=abad_calibrated,
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Check ONNX and RedRhex deployment assumptions.")
    parser.add_argument("--onnx", default=None)
    parser.add_argument("--config", default=None, help="Path to redrhex_policy.yaml for parameter validation.")
    parser.add_argument(
        "--disabled-legs",
        default=None,
        help="Optional comma-separated runtime hardware mask, for example L1",
    )
    parser.add_argument(
        "--bridge-config",
        default=None,
        help="Path to lowlevel_bridge YAML for cross-layer validation.",
    )
    parser.add_argument("--expected-obs-dim", type=int, default=C.OBS_DIM_SINGLE)
    parser.add_argument("--expected-action-dim", type=int, default=C.ACTION_DIM)
    parser.add_argument("--use-cuda", action="store_true")
    parser.add_argument("--use-tensorrt", action="store_true")
    args = parser.parse_args(argv)

    result: dict[str, object] = {
        "python": sys.executable,
        "numpy_version": np.__version__,
        "onnx_path": None,
        "repo_policy_hz": C.POLICY_HZ,
        "repo_control_dt_s": C.CONTROL_DT,
        "single_obs_dim": C.OBS_DIM_SINGLE,
        "history_obs_dim": C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH,
        "action_dim": C.ACTION_DIM,
        "checks": [],
        "warnings": [],
        "next_steps": [],
    }
    params = _load_ros_params(args.config, result)
    disabled_leg_override: list[str] | None = None
    if args.disabled_legs is not None:
        disabled_leg_override = [
            item.strip() for item in args.disabled_legs.split(",") if item.strip()
        ]
    if params:
        params = controller_params_with_disabled_legs(
            params,
            disabled_leg_override,
        )
    strict_bridge_profile = str(
        _nested(params, "observation", "sensor_profile", default="")
    ) in ("encoder_only_rig", "full_feedback_rig")
    bridge_params = _load_bridge_params(
        args.bridge_config,
        result,
        require_complete=strict_bridge_profile,
    )
    if disabled_leg_override is not None and bridge_params:
        bridge_maximum = int(
            _nested(bridge_params, "hardware", "max_disabled_legs", default=1)
        )
        bridge_params.setdefault("hardware", {})["disabled_legs"] = normalize_disabled_legs(
            disabled_leg_override, bridge_maximum
        )
    onnx_path = args.onnx or _nested(params, "policy", "onnx_path", default="/home/jetson/redrhex_models/policy.onnx")
    result["onnx_path"] = str(Path(onnx_path).expanduser())

    all_names = C.ALL_CONTROLLED_JOINT_NAMES
    _append_check(
        result,
        "redrhex_contract_joint_names",
        len(C.MAIN_DRIVE_JOINT_NAMES) == 6
        and len(C.ABAD_JOINT_NAMES) == 6
        and len(C.DAMPER_JOINT_NAMES) == 6
        and len(set(all_names)) == len(all_names),
        controlled_joint_count=len(all_names),
    )
    _append_check(
        result,
        "redrhex_contract_dimensions",
        C.OBS_DIM_SINGLE == args.expected_obs_dim and C.ACTION_DIM == args.expected_action_dim,
        obs_dim=C.OBS_DIM_SINGLE,
        action_dim=C.ACTION_DIM,
    )

    path = Path(onnx_path).expanduser()
    if not path.exists():
        _append_check(result, "onnx_exists", False)
        result["next_steps"].append(
            "Place the verified packaged ONNX at "
            f"{path}, or update policy.onnx_path in the site controller YAML; "
            "do not substitute an unverified source ONNX."
        )
        print(json.dumps(result, indent=2))
        return 2
    _append_check(result, "onnx_exists", True, size_bytes=path.stat().st_size)

    try:
        runner = PolicyONNXRunner(
            str(path),
            expected_obs_dim=args.expected_obs_dim,
            expected_action_dim=args.expected_action_dim,
            use_cuda=args.use_cuda,
            use_tensorrt=args.use_tensorrt,
            allow_history_dim=True,
            # Preflight reports semantic metadata as a separate check so an
            # incompatible model still receives the numerical action-range test.
            expected_observation_contract=None,
            expected_action_contract=None,
            require_contract_metadata=False,
        )
        info = runner.io_info
        result["onnx_io"] = {
            "input_name": info.input_name,
            "input_shape": info.input_shape,
            "input_type": info.input_type,
            "output_name": info.output_name,
            "output_shape": info.output_shape,
            "output_type": info.output_type,
            "providers": info.providers,
            "obs_dim": info.obs_dim,
            "action_dim": info.action_dim,
            "sha256": info.sha256,
            "metadata": info.metadata,
        }
        expected_obs_contract = str(
            _nested(
                params,
                "policy",
                "observation_contract",
                default=C.OBSERVATION_CONTRACT_ID,
            )
        )
        expected_action_contract = str(
            _nested(
                params,
                "policy",
                "action_contract",
                default=C.ACTION_DECODER_CONTRACT_ID,
            )
        )
        require_metadata = bool(
            _nested(params, "policy", "require_contract_metadata", default=False)
        )
        actual_obs_contract = info.metadata.get(C.ONNX_OBSERVATION_CONTRACT_KEY)
        actual_action_contract = info.metadata.get(C.ONNX_ACTION_CONTRACT_KEY)
        expected_input_layout = (
            C.POLICY_INPUT_LAYOUT_HISTORY
            if info.obs_dim == C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH
            else C.POLICY_INPUT_LAYOUT_SINGLE
        )
        actual_input_layout = info.metadata.get(C.ONNX_INPUT_LAYOUT_KEY)
        metadata_matches = (
            (actual_obs_contract in (None, expected_obs_contract))
            and (actual_action_contract in (None, expected_action_contract))
            and (actual_input_layout in (None, expected_input_layout))
            and (
                not require_metadata
                or (
                    actual_obs_contract == expected_obs_contract
                    and actual_action_contract == expected_action_contract
                    and actual_input_layout == expected_input_layout
                )
            )
        )
        _append_check(
            result,
            "onnx_semantic_contract_metadata",
            metadata_matches,
            required=require_metadata,
            expected_observation_contract=expected_obs_contract,
            actual_observation_contract=actual_obs_contract or "missing",
            expected_action_contract=expected_action_contract,
            actual_action_contract=actual_action_contract or "missing",
            expected_input_layout=expected_input_layout,
            actual_input_layout=actual_input_layout or "missing",
        )
        if require_metadata and params:
            resolved_decoder_config = decoder_config_from_ros_params(params)
            expected_decoder_source_sha = decoder_source_sha256()
            expected_decoder_config_sha = decoder_config_sha256(
                resolved_decoder_config
            )
            expected_observation_source_sha = observation_source_sha256()
            expected_deployment_source_sha = deployment_source_sha256()
            expected_bridge_sha = ""
            bridge_config_hash_error = ""
            if bridge_params:
                try:
                    expected_bridge_sha = bridge_config_sha256(bridge_params)
                except (TypeError, ValueError) as exc:
                    # Completeness is reported as its own preflight check.  Do
                    # not turn a useful fail-closed report into a generic ONNX
                    # load exception merely because the bridge cannot be
                    # artifact-hashed.
                    bridge_config_hash_error = str(exc)
            try:
                actual_training_clip = float(
                    info.metadata.get(C.ONNX_TRAINING_ACTION_CLIP_KEY, "nan")
                )
            except (TypeError, ValueError):
                actual_training_clip = float("nan")
            evidence_matches = (
                info.metadata.get(C.ONNX_NORMALIZER_KEY) == C.NORMALIZER_EMBEDDED
                and math.isclose(
                    actual_training_clip,
                    C.TRAINING_ACTION_CLIP,
                    rel_tol=0.0,
                    abs_tol=1.0e-12,
                )
                and info.metadata.get(C.ONNX_DECODER_SOURCE_SHA256_KEY)
                == expected_decoder_source_sha
                and info.metadata.get(C.ONNX_DECODER_CONFIG_SHA256_KEY)
                == expected_decoder_config_sha
                and info.metadata.get(C.ONNX_GOLDEN_SCHEMA_KEY) == GOLDEN_SCHEMA
                and re.fullmatch(
                    r"[0-9a-f]{7,64}",
                    info.metadata.get(C.ONNX_TRAINING_GIT_SHA_KEY, ""),
                )
                is not None
                and re.fullmatch(
                    r"[0-9a-f]{64}",
                    info.metadata.get(C.ONNX_TRAINING_ENV_SOURCE_SHA256_KEY, ""),
                )
                is not None
                and re.fullmatch(
                    r"[0-9a-f]{64}",
                    info.metadata.get(
                        C.ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY, ""
                    ),
                )
                is not None
                and info.metadata.get(C.ONNX_CONTROLLER_CONFIG_SHA256_KEY)
                == controller_config_sha256(params)
                and info.metadata.get(C.ONNX_OBSERVATION_SOURCE_SHA256_KEY)
                == expected_observation_source_sha
                and info.metadata.get(C.ONNX_DEPLOYMENT_SOURCE_SHA256_KEY)
                == expected_deployment_source_sha
                and bool(expected_bridge_sha)
                and info.metadata.get(C.ONNX_BRIDGE_CONFIG_SHA256_KEY)
                == expected_bridge_sha
                and str(
                    _nested(
                        params,
                        "policy",
                        "expected_bridge_config_sha256",
                        default="",
                    )
                ).strip()
                == expected_bridge_sha
            )
            _append_check(
                result,
                "onnx_golden_decoder_evidence",
                evidence_matches,
                expected_normalizer=C.NORMALIZER_EMBEDDED,
                actual_normalizer=info.metadata.get(C.ONNX_NORMALIZER_KEY, "missing"),
                expected_training_action_clip=C.TRAINING_ACTION_CLIP,
                actual_training_action_clip=(
                    actual_training_clip
                    if math.isfinite(actual_training_clip)
                    else "missing"
                ),
                expected_decoder_source_sha256=expected_decoder_source_sha,
                actual_decoder_source_sha256=info.metadata.get(
                    C.ONNX_DECODER_SOURCE_SHA256_KEY, "missing"
                ),
                expected_decoder_config_sha256=expected_decoder_config_sha,
                actual_decoder_config_sha256=info.metadata.get(
                    C.ONNX_DECODER_CONFIG_SHA256_KEY, "missing"
                ),
                expected_golden_schema=GOLDEN_SCHEMA,
                actual_golden_schema=info.metadata.get(
                    C.ONNX_GOLDEN_SCHEMA_KEY, "missing"
                ),
                training_git_sha=info.metadata.get(
                    C.ONNX_TRAINING_GIT_SHA_KEY, "missing"
                ),
                training_env_source_sha256=info.metadata.get(
                    C.ONNX_TRAINING_ENV_SOURCE_SHA256_KEY, "missing"
                ),
                training_env_config_source_sha256=info.metadata.get(
                    C.ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY, "missing"
                ),
                expected_controller_config_sha256=controller_config_sha256(params),
                actual_controller_config_sha256=info.metadata.get(
                    C.ONNX_CONTROLLER_CONFIG_SHA256_KEY, "missing"
                ),
                expected_observation_source_sha256=expected_observation_source_sha,
                actual_observation_source_sha256=info.metadata.get(
                    C.ONNX_OBSERVATION_SOURCE_SHA256_KEY, "missing"
                ),
                expected_deployment_source_sha256=expected_deployment_source_sha,
                actual_deployment_source_sha256=info.metadata.get(
                    C.ONNX_DEPLOYMENT_SOURCE_SHA256_KEY, "missing"
                ),
                expected_bridge_config_sha256=expected_bridge_sha or "missing bridge config",
                bridge_config_hash_error=bridge_config_hash_error or "none",
                actual_bridge_config_sha256=info.metadata.get(
                    C.ONNX_BRIDGE_CONFIG_SHA256_KEY, "missing"
                ),
            )
        obs_dim = info.obs_dim or args.expected_obs_dim
        action = runner.run(np.zeros(obs_dim, dtype=np.float32))
        _append_check(
            result,
            "zero_observation_inference",
            bool(np.isfinite(action).all() and action.shape == (args.expected_action_dim,)),
            action_min=float(np.min(action)),
            action_max=float(np.max(action)),
        )
        reference_input = reference_policy_input(
            obs_dim,
            int(
                _nested(
                    params,
                    "observation",
                    "policy_history_length",
                    default=C.POLICY_HISTORY_LENGTH,
                )
            ),
        )
        reference_action = runner.run(reference_input)
        startup_limit = float(
            _nested(params, "policy", "startup_action_abs_limit", default=1.5)
        )
        reference_ok, reference_max = check_reference_action(reference_action, startup_limit)
        _append_check(
            result,
            "reference_observation_action_range",
            reference_ok,
            action_min=float(np.min(reference_action)),
            action_max=float(np.max(reference_action)),
            max_abs_action=reference_max,
            configured_limit=startup_limit,
        )
        benchmark_warmup = int(
            _nested(params, "policy", "inference_benchmark_warmup_runs", default=20)
        )
        benchmark_runs = int(
            _nested(params, "policy", "inference_benchmark_runs", default=200)
        )
        benchmark_limit_ms = float(
            _nested(params, "policy", "max_inference_p99_ms", default=6.0)
        )
        benchmark = runner.benchmark(
            [np.zeros(obs_dim, dtype=np.float32), reference_input],
            warmup_runs=benchmark_warmup,
            runs=benchmark_runs,
        )
        _append_check(
            result,
            "cpu_inference_p99_latency",
            bool(benchmark["p99_ms"] <= benchmark_limit_ms),
            configured_limit_ms=benchmark_limit_ms,
            providers=info.providers,
            **benchmark,
        )
        expected_sha256 = str(
            _nested(params, "policy", "expected_sha256", default="")
        ).strip().lower()
        sensor_profile = str(
            _nested(params, "observation", "sensor_profile", default="full_state")
        )
        strict_rig_profile = sensor_profile in (
            "encoder_only_rig",
            "full_feedback_rig",
        )
        sha_is_pinned = bool(expected_sha256)
        _append_check(
            result,
            "policy_sha256",
            (sha_is_pinned or not strict_rig_profile)
            and (not expected_sha256 or expected_sha256 == info.sha256),
            actual=info.sha256,
            expected=expected_sha256 or "not pinned",
            required=strict_rig_profile,
        )
        if not info.metadata:
            result["warnings"].append(
                "ONNX has no semantic contract metadata; do not enable hardware until export/decode golden parity is recorded."
            )
        if obs_dim == C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH:
            result["warnings"].append("ONNX expects 280-D policy+history input; keep policy_history_length=5.")
        elif obs_dim != C.OBS_DIM_SINGLE:
            result["warnings"].append(f"Unexpected obs dim {obs_dim}; verify export and YAML.")
    except Exception as exc:
        _append_check(result, "onnx_load_and_run", False, error=str(exc))
        print(json.dumps(result, indent=2))
        return 3

    _validate_deployment_config(params, obs_dim, result)
    _validate_bridge_config(params, bridge_params, result)
    sensor_profile = str(
        _nested(params, "observation", "sensor_profile", default="full_state")
    )
    if sensor_profile in ("encoder_only_rig", "full_feedback_rig"):
        first_step = (
            "Run the real Rinbo bridge with allow_enable=false and inspect all required sensor topics; "
            f"fake sensors are forbidden in {sensor_profile}."
        )
    else:
        first_step = "Launch mock mode with use_fake_sensors:=true and start_bridge:=false."
    result["next_steps"] = [
        first_step,
        "Do not enable policy on hardware until INIT_STAND, single ABAD, and single main-drive tests pass.",
        "Keep /redrhex/enable_motors false until the robot is suspended, current-limited, and E-stop is ready.",
    ]
    if any(not bool(check.get("ok")) for check in result["checks"]):
        print(json.dumps(result, indent=2))
        return 4
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
