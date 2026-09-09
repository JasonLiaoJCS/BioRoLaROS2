import math
import os
import re
import secrets
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from redrhex_rl_controller.golden_policy import (
    bridge_config_sha256,
    bridge_params_with_disabled_legs,
    load_bridge_ros_params,
    load_controller_ros_params,
)
from redrhex_rl_controller.degraded_mode import normalize_disabled_legs


_ACTIVE_FORBIDDEN_OVERRIDE_NAMES = (
    "max_disabled_legs",
    "onnx_path",
    "policy_hz",
    "use_cuda",
    "use_tensorrt",
    "enable_policy_on_start",
    "enable_motor_output_on_start",
    "sensor_profile",
    "encoder_only_rig_acknowledged",
    "base_lin_vel_source",
    "abad_feedback_source",
    "require_lowlevel_heartbeat",
    "require_motor_feedback",
    "fake_rate_hz",
    "fake_cmd_vx",
    "fake_cmd_vy",
    "fake_cmd_wz",
)

_CANONICAL_PARENT_PID_ENV = "REDRHEX_CANONICAL_POLICY_PARENT_PID"
_CANONICAL_PARENT_START_ENV = "REDRHEX_CANONICAL_POLICY_PARENT_START_TICKS"
_CANONICAL_NONCE_ENV = "REDRHEX_CANONICAL_POLICY_LAUNCH_NONCE"


def _linux_process_start_ticks(pid: int) -> str:
    stat = Path(f"/proc/{int(pid)}/stat").read_text(encoding="utf-8")
    close_paren = stat.rfind(")")
    if close_paren < 0:
        raise RuntimeError(f"malformed /proc/{pid}/stat")
    suffix_fields = stat[close_paren + 2 :].split()
    if len(suffix_fields) <= 19:
        raise RuntimeError(f"incomplete /proc/{pid}/stat")
    return suffix_fields[19]


def _canonical_bridge_environment(active: bool) -> dict[str, str]:
    """Mint a one-process, non-ROS capability after active launch validation."""

    if not active:
        return {}
    parent_pid = os.getpid()
    return {
        _CANONICAL_PARENT_PID_ENV: str(parent_pid),
        _CANONICAL_PARENT_START_ENV: _linux_process_start_ticks(parent_pid),
        _CANONICAL_NONCE_ENV: secrets.token_hex(32),
    }


def _bool_text(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _maybe_add(params: dict, name: str, value: str, value_type):
    text = value.strip()
    if text == "":
        return
    if value_type is bool:
        params[name] = _bool_text(text)
    elif value_type is float:
        params[name] = float(text)
    elif value_type is int:
        params[name] = int(text)
    else:
        params[name] = text


def _csv_list(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _profile_config(package_name: str, prefix: str, profile: str) -> str:
    if profile not in (
        "bench_safe",
        "encoder_only_rig",
        "full_feedback_rig",
        "locomotion_tune",
    ):
        raise RuntimeError(
            "safety_profile must be 'bench_safe', 'encoder_only_rig', "
            "'full_feedback_rig', 'locomotion_tune', or use explicit config files."
        )
    return os.path.join(get_package_share_directory(package_name), "config", f"{prefix}_{profile}.yaml")


def _validate_active_bridge_enable(
    *,
    requested: bool,
    controller_params: dict,
    bridge_params: dict,
    requested_sensor_profile: str,
    start_bridge: bool,
    use_fake_sensors: bool,
    disabled_legs: list[str] | None = None,
    launch_overrides: dict[str, str] | None = None,
) -> str:
    """Fail closed before an active Rinbo hardware launch can create nodes."""

    configured_sensor_profile = str(
        controller_params.get("observation", {}).get("sensor_profile", "")
    ).strip()
    effective_sensor_profile = (
        requested_sensor_profile.strip() or configured_sensor_profile
    )
    if not requested:
        return effective_sensor_profile

    strict_profiles = {"encoder_only_rig", "full_feedback_rig"}
    violations: list[str] = []
    if configured_sensor_profile not in strict_profiles:
        violations.append(
            "selected controller YAML observation.sensor_profile must be a strict rig"
        )
    if effective_sensor_profile not in strict_profiles:
        violations.append(
            "effective observation.sensor_profile must be encoder_only_rig or full_feedback_rig"
        )
    if not start_bridge:
        violations.append("start_bridge must be true")
    if use_fake_sensors:
        violations.append("use_fake_sensors must be false")
    if str(bridge_params.get("backend", "")).strip() != "biorola_ros":
        violations.append("selected bridge YAML backend must be biorola_ros")

    nonempty_overrides = sorted(
        name
        for name, value in (launch_overrides or {}).items()
        if str(value).strip()
    )
    if nonempty_overrides:
        violations.append(
            "active hardware forbids launch overrides: "
            + ", ".join(nonempty_overrides)
        )

    controller_hardware = controller_params.get("hardware", {})
    bridge_hardware = bridge_params.get("hardware", {})
    try:
        controller_max_disabled = int(
            controller_hardware.get("max_disabled_legs", -1)
        )
        controller_mask = normalize_disabled_legs(
            list(controller_hardware.get("disabled_legs", []) or []), 1
        )
    except (TypeError, ValueError) as exc:
        controller_max_disabled = -1
        controller_mask = []
        violations.append(f"invalid controller YAML disabled-leg contract: {exc}")
    try:
        bridge_max_disabled = int(bridge_hardware.get("max_disabled_legs", -1))
        bridge_mask = normalize_disabled_legs(
            list(bridge_hardware.get("disabled_legs", []) or []), 1
        )
    except (TypeError, ValueError) as exc:
        bridge_max_disabled = -1
        bridge_mask = []
        violations.append(f"invalid bridge YAML disabled-leg contract: {exc}")
    try:
        requested_mask = normalize_disabled_legs(list(disabled_legs or []), 1)
    except (TypeError, ValueError) as exc:
        requested_mask = []
        violations.append(f"invalid launch disabled_legs: {exc}")
    effective_launch_mask = requested_mask if disabled_legs else controller_mask
    if controller_max_disabled != 1 or bridge_max_disabled != 1:
        violations.append(
            "controller and bridge hardware.max_disabled_legs must both equal 1"
        )
    if not (
        controller_mask == bridge_mask == effective_launch_mask
        and len(effective_launch_mask) <= 1
    ):
        violations.append(
            "controller YAML, bridge YAML, and effective launch disabled_legs "
            "must be identical and contain at most one leg"
        )

    state_machine = controller_params.get("state_machine", {})
    safety = controller_params.get("safety", {})
    policy = controller_params.get("policy", {})
    action = controller_params.get("action", {})
    if bool(state_machine.get("enable_policy_on_start", True)) or bool(
        state_machine.get("enable_motor_output_on_start", True)
    ):
        violations.append("controller policy and motor output must start disabled")
    if not bool(safety.get("require_lowlevel_heartbeat", False)) or not bool(
        safety.get("require_motor_feedback", False)
    ):
        violations.append(
            "controller heartbeat and motor-feedback requirements must both be true"
        )
    try:
        lease_s = float(state_machine.get("policy_run_max_duration_s", math.nan))
    except (TypeError, ValueError):
        lease_s = math.nan
    if not math.isclose(lease_s, 3.0, rel_tol=0.0, abs_tol=1.0e-12):
        violations.append("state_machine.policy_run_max_duration_s must equal 3.0")
    for key, label in (
        ("expected_sha256", "policy.expected_sha256"),
        ("expected_bridge_config_sha256", "policy.expected_bridge_config_sha256"),
    ):
        if re.fullmatch(r"[0-9a-f]{64}", str(policy.get(key, "")).strip()) is None:
            violations.append(f"{label} must be a 64-character lowercase SHA256")
    if not bool(action.get("hardware_mapping_calibrated", False)):
        violations.append("action.hardware_mapping_calibrated must be true")

    rinbo = bridge_params.get("rinbo", {})
    required_true = (
        "publish_shutdown_disable",
        "require_downstream_output_ack",
        "require_state",
        "block_if_duplicate_command_publishers",
        "require_main_drive_calibration",
        "main_drive_calibrated",
        "require_exact_command_contract",
        "require_single_telemetry_publisher",
        "require_monotonic_telemetry_stamp",
        "require_abad_command_calibration",
        "abad_command_calibrated",
        "require_power_state",
        "require_power_relay",
        "block_if_duplicate_upstream_publishers",
    )
    disabled_guards = [name for name in required_true if not bool(rinbo.get(name, False))]
    if disabled_guards:
        violations.append(
            "selected bridge YAML must enable safety/calibration gates: "
            + ", ".join(disabled_guards)
        )
    if bool(rinbo.get("allow_enable", True)):
        violations.append(
            "selected bridge YAML rinbo.allow_enable must remain false; use only the operational launch latch"
        )
    if bool(rinbo.get("canonical_policy_launch_authorized", False)):
        violations.append(
            "selected bridge YAML must not self-authorize canonical policy launch"
        )
    if bool(rinbo.get("publish_when_disabled", True)):
        violations.append("selected bridge YAML rinbo.publish_when_disabled must be false")
    exact_bridge_values = {
        "command_topic": "/motor/command",
        "state_topic": "/motor/state",
        "power_state_topic": "/power/state",
        "joint_state_topic": "/joint_states",
        "preview_topic": "/redrhex/rinbo_motor_command_preview",
        "publish_preview": True,
        "disabled_servo_control_mode": 0,
        "servo_control_mode": 2,
        "downstream_output_ack_topic": "/rinbo/motor_output_enabled",
        "expected_telemetry_publisher_node": "rinbo_ros2_bridge",
        "expected_upstream_command_publisher_node": "/redrhex_rl_controller",
        "power_bus_voltage_channel": 7,
    }
    for name, expected in exact_bridge_values.items():
        value = rinbo.get(name, expected)
        if value != expected:
            violations.append(
                f"bridge rinbo.{name} must be exactly {expected!r}"
            )
    if list(
        rinbo.get("leg_current_channels_rinbo_order", [1, 2, 3, 4, 5, 6])
    ) != [1, 2, 3, 4, 5, 6]:
        violations.append(
            "bridge rinbo.leg_current_channels_rinbo_order must be exactly [1,2,3,4,5,6]"
        )

    bounded_bridge_values = (
        ("state_timeout_s", 0.25),
        ("downstream_output_ack_timeout_s", 0.10),
        ("max_main_target_velocity_rad_s", 1.0),
        ("max_abad_target_position_rad", 0.18),
        ("power_state_timeout_s", 0.35),
        ("command_timeout_s", 0.10),
        ("upstream_command_max_age_s", 0.10),
        ("max_bus_voltage", 30.0),
        ("max_current_a", 3.0),
        ("main_max_pwm", 80.0),
        ("main_pwm_slew_rate_per_s", 250.0),
    )
    for name, upper_bound in bounded_bridge_values:
        try:
            value = float(rinbo.get(name, math.nan))
        except (TypeError, ValueError):
            value = math.nan
        if not math.isfinite(value) or value <= 0.0 or value > upper_bound:
            violations.append(f"bridge rinbo.{name} must be in (0, {upper_bound:g}]")
    try:
        min_bus_voltage = float(rinbo.get("min_bus_voltage", math.nan))
        max_bus_voltage = float(rinbo.get("max_bus_voltage", math.nan))
    except (TypeError, ValueError):
        min_bus_voltage = math.nan
        max_bus_voltage = math.nan
    if (
        not math.isfinite(min_bus_voltage)
        or min_bus_voltage < 18.0
        or not min_bus_voltage < max_bus_voltage
    ):
        violations.append(
            "bridge rinbo.min_bus_voltage must be >= 18.0 and below max_bus_voltage"
        )
    if bool(rinbo.get("publish_abad_joint_feedback", False)) and not bool(
        rinbo.get("abad_feedback_calibrated", False)
    ):
        violations.append(
            "bridge ABAD joint feedback requires abad_feedback_calibrated=true"
        )
    for name in (
        "downstream_output_ack_topic",
        "expected_telemetry_publisher_node",
    ):
        if not str(rinbo.get(name, "")).strip():
            violations.append(f"bridge rinbo.{name} must be non-empty")
    for name, minimum in (
        ("disabled_handshake_repeats", 8),
        ("shutdown_disable_repeats", 8),
        ("recovery_healthy_samples", 5),
    ):
        try:
            value = int(rinbo.get(name, 0))
        except (TypeError, ValueError):
            value = 0
        if value < minimum:
            violations.append(f"bridge rinbo.{name} must be >= {minimum}")
    for name in ("current_trip_samples", "voltage_trip_samples"):
        try:
            value = int(rinbo.get(name, 0))
        except (TypeError, ValueError):
            value = 0
        if value <= 0 or value > 3:
            violations.append(f"bridge rinbo.{name} must be in [1, 3]")
    if violations:
        raise RuntimeError(
            "bridge_rinbo_allow_enable:=true is restricted to an active strict "
            "hardware rig: " + "; ".join(violations)
        )
    return effective_sensor_profile


def _launch_setup(context, *args, **kwargs):
    safety_profile = LaunchConfiguration("safety_profile").perform(context).strip()
    config = LaunchConfiguration("config").perform(context).strip()
    bridge_config = LaunchConfiguration("bridge_config").perform(context).strip()
    if safety_profile == "custom":
        if not config or not bridge_config:
            raise RuntimeError("safety_profile:=custom requires config:=... and bridge_config:=...")
    else:
        if not config:
            config = _profile_config("redrhex_rl_controller", "redrhex_policy", safety_profile)
        if not bridge_config:
            bridge_config = _profile_config("redrhex_lowlevel_bridge", "lowlevel_bridge", safety_profile)
    # Decisions below use the selected files, never the safety_profile label.
    controller_file_params = load_controller_ros_params(config)
    bridge_file_params = load_bridge_ros_params(bridge_config)
    use_fake_sensors = LaunchConfiguration("use_fake_sensors")
    start_bridge = LaunchConfiguration("start_bridge")
    use_fake_sensors_value = _bool_text(use_fake_sensors.perform(context))
    start_bridge_value = _bool_text(start_bridge.perform(context))
    if use_fake_sensors_value and start_bridge_value:
        raise RuntimeError(
            "use_fake_sensors:=true cannot be combined with start_bridge:=true: fake /joint_states, "
            "/imu/data, and heartbeat would overwrite or mask real hardware feedback"
        )
    requested_sensor_profile = LaunchConfiguration("sensor_profile").perform(context).strip()
    disabled_legs = _csv_list(LaunchConfiguration("disabled_legs").perform(context))
    active_launch_overrides = {
        name: LaunchConfiguration(name).perform(context).strip()
        for name in _ACTIVE_FORBIDDEN_OVERRIDE_NAMES
    }
    fake_publish_abad_joints = LaunchConfiguration("fake_publish_abad_joints")
    fake_publish_damper_joints = LaunchConfiguration("fake_publish_damper_joints")
    if _bool_text(fake_publish_abad_joints.perform(context)):
        active_launch_overrides["fake_publish_abad_joints"] = "true"
    if _bool_text(fake_publish_damper_joints.perform(context)):
        active_launch_overrides["fake_publish_damper_joints"] = "true"
    bridge_allow_enable_text = LaunchConfiguration(
        "bridge_rinbo_allow_enable"
    ).perform(context)
    bridge_allow_enable_requested = _bool_text(bridge_allow_enable_text)
    strict_sensor_profile = _validate_active_bridge_enable(
        requested=bridge_allow_enable_requested,
        controller_params=controller_file_params,
        bridge_params=bridge_file_params,
        requested_sensor_profile=requested_sensor_profile,
        start_bridge=start_bridge_value,
        use_fake_sensors=use_fake_sensors_value,
        disabled_legs=disabled_legs,
        launch_overrides=active_launch_overrides,
    )
    if strict_sensor_profile in ("encoder_only_rig", "full_feedback_rig") and use_fake_sensors_value:
        raise RuntimeError(
            f"{strict_sensor_profile} requires real hardware feedback; fake sensors are forbidden"
        )
    controller_overrides = {}
    if disabled_legs:
        controller_overrides["hardware.disabled_legs"] = disabled_legs

    # Close the cross-package gap without trusting a launch-time calibration
    # override: hash the bridge YAML that will actually be launched. The only
    # remaining user-facing bridge override is allow_enable, deliberately
    # excluded from the canonical semantic hash; disabled_legs is included
    # explicitly. Active provenance is carried separately as a process-only
    # capability, never as a ROS parameter.
    expected_bridge_hash = str(
        controller_file_params.get("policy", {}).get(
            "expected_bridge_config_sha256", ""
        )
    ).strip()
    if expected_bridge_hash:
        effective_bridge_params = bridge_params_with_disabled_legs(
            bridge_file_params,
            disabled_legs or None,
        )
        actual_bridge_hash = bridge_config_sha256(effective_bridge_params)
        if actual_bridge_hash != expected_bridge_hash:
            raise RuntimeError(
                "selected bridge_config semantic SHA256 does not match controller "
                "policy.expected_bridge_config_sha256: "
                f"{actual_bridge_hash} != {expected_bridge_hash}"
            )
    _maybe_add(
        controller_overrides,
        "hardware.max_disabled_legs",
        LaunchConfiguration("max_disabled_legs").perform(context),
        int,
    )
    _maybe_add(controller_overrides, "policy.onnx_path", LaunchConfiguration("onnx_path").perform(context), str)
    _maybe_add(controller_overrides, "policy.policy_hz", LaunchConfiguration("policy_hz").perform(context), float)
    _maybe_add(controller_overrides, "policy.use_cuda", LaunchConfiguration("use_cuda").perform(context), bool)
    _maybe_add(controller_overrides, "policy.use_tensorrt", LaunchConfiguration("use_tensorrt").perform(context), bool)
    _maybe_add(
        controller_overrides,
        "state_machine.enable_policy_on_start",
        LaunchConfiguration("enable_policy_on_start").perform(context),
        bool,
    )
    _maybe_add(
        controller_overrides,
        "state_machine.enable_motor_output_on_start",
        LaunchConfiguration("enable_motor_output_on_start").perform(context),
        bool,
    )
    _maybe_add(
        controller_overrides,
        "observation.sensor_profile",
        LaunchConfiguration("sensor_profile").perform(context),
        str,
    )
    _maybe_add(
        controller_overrides,
        "observation.encoder_only_rig_acknowledged",
        LaunchConfiguration("encoder_only_rig_acknowledged").perform(context),
        bool,
    )
    _maybe_add(
        controller_overrides,
        "observation.base_lin_vel_source",
        LaunchConfiguration("base_lin_vel_source").perform(context),
        str,
    )
    _maybe_add(
        controller_overrides,
        "observation.abad_feedback_source",
        LaunchConfiguration("abad_feedback_source").perform(context),
        str,
    )
    _maybe_add(
        controller_overrides,
        "safety.require_lowlevel_heartbeat",
        LaunchConfiguration("require_lowlevel_heartbeat").perform(context),
        bool,
    )
    _maybe_add(
        controller_overrides,
        "safety.require_motor_feedback",
        LaunchConfiguration("require_motor_feedback").perform(context),
        bool,
    )

    controller_parameters = [config]
    if controller_overrides:
        controller_parameters.append(controller_overrides)

    bridge_overrides = {}
    if disabled_legs:
        bridge_overrides["hardware.disabled_legs"] = disabled_legs
    _maybe_add(
        bridge_overrides,
        "rinbo.allow_enable",
        bridge_allow_enable_text,
        bool,
    )
    bridge_parameters = [bridge_config]
    if bridge_overrides:
        bridge_parameters.append(bridge_overrides)

    fake_params = {
        "publish_abad_joints": fake_publish_abad_joints,
        "publish_damper_joints": fake_publish_damper_joints,
    }
    _maybe_add(fake_params, "rate_hz", LaunchConfiguration("fake_rate_hz").perform(context), float)
    _maybe_add(fake_params, "cmd_vx", LaunchConfiguration("fake_cmd_vx").perform(context), float)
    _maybe_add(fake_params, "cmd_vy", LaunchConfiguration("fake_cmd_vy").perform(context), float)
    _maybe_add(fake_params, "cmd_wz", LaunchConfiguration("fake_cmd_wz").perform(context), float)

    return [
        Node(
            package="redrhex_rl_controller",
            executable="rl_controller_node",
            name="redrhex_rl_controller",
            output="screen",
            parameters=controller_parameters,
        ),
        Node(
            package="redrhex_lowlevel_bridge",
            executable="lowlevel_bridge_node",
            name="redrhex_lowlevel_bridge",
            output="screen",
            parameters=bridge_parameters,
            additional_env=_canonical_bridge_environment(
                bridge_allow_enable_requested
            ),
            condition=IfCondition(start_bridge),
        ),
        Node(
            package="redrhex_rl_controller",
            executable="fake_sensor_node",
            name="redrhex_fake_sensor_node",
            output="screen",
            parameters=[fake_params],
            condition=IfCondition(use_fake_sensors),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "safety_profile",
            default_value="bench_safe",
            description=(
                "bench_safe, encoder_only_rig, full_feedback_rig, "
                "locomotion_tune, or custom."
            ),
        ),
        DeclareLaunchArgument("config", default_value="", description="Optional explicit controller YAML. Overrides safety_profile."),
        DeclareLaunchArgument("bridge_config", default_value="", description="Optional explicit bridge YAML. Overrides safety_profile."),
        DeclareLaunchArgument("use_fake_sensors", default_value="false"),
        DeclareLaunchArgument("fake_publish_abad_joints", default_value="false"),
        DeclareLaunchArgument("fake_publish_damper_joints", default_value="false"),
        DeclareLaunchArgument("fake_rate_hz", default_value=""),
        DeclareLaunchArgument("fake_cmd_vx", default_value=""),
        DeclareLaunchArgument("fake_cmd_vy", default_value=""),
        DeclareLaunchArgument("fake_cmd_wz", default_value=""),
        DeclareLaunchArgument("start_bridge", default_value="true"),
        DeclareLaunchArgument("disabled_legs", default_value="", description="Comma-separated physical legs, for example L1. Passed to controller and bridge."),
        DeclareLaunchArgument("max_disabled_legs", default_value="", description="Optional guard; default profiles allow one disabled leg."),
        DeclareLaunchArgument("onnx_path", default_value="", description="Optional override for policy.onnx_path."),
        DeclareLaunchArgument("policy_hz", default_value="", description="Optional override for policy.policy_hz."),
        DeclareLaunchArgument("use_cuda", default_value="", description="Optional bool override for policy.use_cuda."),
        DeclareLaunchArgument("use_tensorrt", default_value="", description="Optional bool override for policy.use_tensorrt."),
        DeclareLaunchArgument("enable_policy_on_start", default_value="", description="Keep false on hardware."),
        DeclareLaunchArgument("enable_motor_output_on_start", default_value="", description="Keep false on hardware."),
        DeclareLaunchArgument("base_lin_vel_source", default_value="", description="zero or odom."),
        DeclareLaunchArgument(
            "sensor_profile",
            default_value="",
            description="full_state, encoder_only_rig, or full_feedback_rig.",
        ),
        DeclareLaunchArgument(
            "encoder_only_rig_acknowledged",
            default_value="",
            description="Explicit acknowledgement required by encoder_only_rig.",
        ),
        DeclareLaunchArgument("abad_feedback_source", default_value="", description="commanded or joint_states."),
        DeclareLaunchArgument("require_lowlevel_heartbeat", default_value="", description="Optional bool safety override."),
        DeclareLaunchArgument("require_motor_feedback", default_value="", description="Optional bool safety override."),
        DeclareLaunchArgument("bridge_rinbo_allow_enable", default_value="false", description="Optional bool override for bridge rinbo.allow_enable."),
        OpaqueFunction(function=_launch_setup),
    ])
