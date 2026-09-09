from __future__ import annotations

from types import SimpleNamespace

import pytest

from redrhex_lowlevel_bridge.lowlevel_bridge_node import (
    _ACTIVE_RINBO_PARAMETER_NAMES,
    _CANONICAL_NONCE_ENV,
    _CANONICAL_PARENT_PID_ENV,
    _CANONICAL_PARENT_START_ENV,
    LowLevelBridgeNode,
    _canonical_policy_launch_provenance_reason,
    _reject_unarbitrated_backend_enable,
    _validate_active_rinbo_safety_contract,
)


def _safe_params() -> dict[str, object]:
    values: dict[str, object] = {
        "feedback_rate_hz": 50.0,
        "hardware.max_disabled_legs": 1,
        "rinbo.allow_enable": True,
        "rinbo.canonical_policy_launch_authorized": False,
        "rinbo.command_topic": "/motor/command",
        "rinbo.state_topic": "/motor/state",
        "rinbo.power_state_topic": "/power/state",
        "rinbo.joint_state_topic": "/joint_states",
        "rinbo.preview_topic": "/redrhex/rinbo_motor_command_preview",
        "rinbo.publish_preview": True,
        "rinbo.publish_when_disabled": False,
        "rinbo.publish_shutdown_disable": True,
        "rinbo.shutdown_disable_repeats": 8,
        "rinbo.shutdown_disable_period_s": 0.02,
        "rinbo.disabled_handshake_repeats": 8,
        "rinbo.require_downstream_output_ack": True,
        "rinbo.downstream_output_ack_topic": "/rinbo/motor_output_enabled",
        "rinbo.downstream_output_ack_timeout_s": 0.10,
        "rinbo.require_state": True,
        "rinbo.block_if_duplicate_command_publishers": True,
        "rinbo.state_timeout_s": 0.25,
        "rinbo.main_pwm_per_rad_s": 40.0,
        "rinbo.main_max_pwm": 80.0,
        "rinbo.main_pwm_slew_rate_per_s": 250.0,
        "rinbo.main_velocity_max_dt_s": 0.20,
        "rinbo.main_velocity_clip_rad_s": 20.0,
        "rinbo.max_main_target_velocity_rad_s": 1.0,
        "rinbo.disabled_servo_control_mode": 0,
        "rinbo.servo_control_mode": 2,
        "rinbo.max_abad_target_position_rad": 0.18,
        "rinbo.require_exact_command_contract": True,
        "rinbo.require_single_telemetry_publisher": True,
        "rinbo.expected_telemetry_publisher_node": "rinbo_ros2_bridge",
        "rinbo.require_monotonic_telemetry_stamp": True,
        "rinbo.require_abad_command_calibration": True,
        "rinbo.abad_command_calibrated": True,
        "rinbo.require_main_drive_calibration": True,
        "rinbo.main_drive_calibrated": True,
        "rinbo.require_power_state": True,
        "rinbo.require_power_relay": True,
        "rinbo.power_state_timeout_s": 0.35,
        "rinbo.command_timeout_s": 0.10,
        "rinbo.recovery_healthy_samples": 5,
        "rinbo.power_bus_voltage_channel": 7,
        "rinbo.leg_current_channels_rinbo_order": [1, 2, 3, 4, 5, 6],
        "rinbo.min_bus_voltage": 18.0,
        "rinbo.max_bus_voltage": 30.0,
        "rinbo.max_current_a": 3.0,
        "rinbo.current_trip_samples": 3,
        "rinbo.voltage_trip_samples": 3,
        "rinbo.block_if_duplicate_upstream_publishers": True,
        "rinbo.expected_upstream_command_publisher_node": "/redrhex_rl_controller",
        "rinbo.upstream_command_max_age_s": 0.10,
    }
    assert set(values) == set(_ACTIVE_RINBO_PARAMETER_NAMES)
    return values


def test_active_rinbo_safety_contract_accepts_checked_in_limits() -> None:
    _validate_active_rinbo_safety_contract(
        _safe_params(), canonical_launch_provenance=True
    )


def test_active_rinbo_contract_rejects_ros_parameter_only_authorization() -> None:
    params = _safe_params()
    params["rinbo.canonical_policy_launch_authorized"] = True

    with pytest.raises(ValueError, match="retired"):
        _validate_active_rinbo_safety_contract(
            params, canonical_launch_provenance=False
        )


def test_active_rinbo_contract_requires_non_ros_launch_provenance() -> None:
    with pytest.raises(ValueError, match="non-ROS launch provenance"):
        _validate_active_rinbo_safety_contract(_safe_params())


def test_canonical_launch_provenance_requires_live_exact_parent() -> None:
    environment = {
        _CANONICAL_PARENT_PID_ENV: "4321",
        _CANONICAL_PARENT_START_ENV: "987654",
        _CANONICAL_NONCE_ENV: "a" * 64,
    }
    canonical_cmdline = [
        "/usr/bin/python3",
        "/opt/ros/humble/bin/ros2",
        "launch",
        "redrhex_rl_controller",
        "redrhex_policy_bringup.launch.py",
        "bridge_rinbo_allow_enable:=true",
    ]

    assert (
        _canonical_policy_launch_provenance_reason(
            environment,
            process_parent_pid=4321,
            parent_start_ticks="987654",
            parent_cmdline_tokens=canonical_cmdline,
        )
        is None
    )
    assert "PID mismatch" in str(
        _canonical_policy_launch_provenance_reason(
            environment,
            process_parent_pid=4322,
            parent_start_ticks="987654",
            parent_cmdline_tokens=canonical_cmdline,
        )
    )
    assert "not ros2 launch" in str(
        _canonical_policy_launch_provenance_reason(
            environment,
            process_parent_pid=4321,
            parent_start_ticks="987654",
            parent_cmdline_tokens=["ros2", "run", "redrhex_lowlevel_bridge"],
        )
    )


def test_read_only_rinbo_keeps_calibration_workflow_available() -> None:
    params = _safe_params()
    params["rinbo.allow_enable"] = False
    params["rinbo.require_state"] = False
    params["rinbo.main_drive_calibrated"] = False

    _validate_active_rinbo_safety_contract(params)


@pytest.mark.parametrize(
    ("name", "unsafe_value"),
    [
        ("rinbo.command_topic", "/fake/motor"),
        ("rinbo.preview_topic", "/motor/command"),
        ("rinbo.publish_when_disabled", True),
        ("rinbo.disabled_servo_control_mode", 2),
        ("rinbo.publish_shutdown_disable", False),
        ("rinbo.shutdown_disable_repeats", 7),
        ("rinbo.disabled_handshake_repeats", 7),
    ],
)
def test_read_only_rinbo_still_rejects_actionable_disabled_path(
    name: str, unsafe_value: object
) -> None:
    params = _safe_params()
    params["rinbo.allow_enable"] = False
    params["rinbo.main_drive_calibrated"] = False
    params["rinbo.abad_command_calibrated"] = False
    params[name] = unsafe_value

    with pytest.raises(ValueError, match="Unsafe read-only Rinbo"):
        _validate_active_rinbo_safety_contract(params)


@pytest.mark.parametrize("backend", ["serial", "sbrio"])
def test_unarbitrated_hardware_backend_enable_is_retired(backend: str) -> None:
    with pytest.raises(ValueError, match="final Rinbo motor/power arbiter"):
        _reject_unarbitrated_backend_enable(backend, True)

    _reject_unarbitrated_backend_enable(backend, False)


def test_lowlevel_disabled_leg_parameters_are_startup_only() -> None:
    import rclpy
    from rclpy.parameter import Parameter

    node = None
    rclpy.init()
    try:
        node = LowLevelBridgeNode()
        assert node.describe_parameter("hardware.disabled_legs").read_only
        assert node.describe_parameter("hardware.max_disabled_legs").read_only
        results = node.set_parameters(
            [
                Parameter("hardware.disabled_legs", value=["R2"]),
                Parameter("hardware.max_disabled_legs", value=2),
            ]
        )
        assert [result.successful for result in results] == [False, False]
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _upstream_guard_node(publisher_name: str = "redrhex_rl_controller"):
    node = object.__new__(LowLevelBridgeNode)
    node.block_if_duplicate_upstream_publishers = True
    node.require_expected_upstream_command_publisher = True
    node.expected_upstream_command_publisher_node = "/redrhex_rl_controller"
    node.upstream_command_max_age_s = 0.10
    node.last_upstream_command_stamp_ns = None
    node.upstream_publisher_diagnostic = "not checked"
    node.get_publishers_info_by_topic = lambda _topic: [
        SimpleNamespace(node_namespace="/", node_name=publisher_name)
    ]
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)
    )
    return node


def _upstream_command(*, enable: bool, stamp_ns: int = 9_950_000_000):
    return SimpleNamespace(
        enable=enable,
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        ),
    )


def test_disabled_rearm_requires_fresh_canonical_upstream_source() -> None:
    node = _upstream_guard_node()
    disabled = _upstream_command(enable=False)

    assert node._upstream_command_guard_reason(disabled) is None
    assert "duplicate" in str(node._upstream_command_guard_reason(disabled))

    untrusted = _upstream_guard_node("fake_controller")
    assert "canonical controller" in str(
        untrusted._upstream_command_guard_reason(disabled)
    )


def test_untrusted_disabled_command_latches_safety_instead_of_rearming() -> None:
    node = _upstream_guard_node("fake_controller")
    stops: list[str] = []
    forwarded: list[object] = []
    node.bridge = SimpleNamespace(
        external_safety_stop=lambda reason: stops.append(reason),
        send_motor_command=lambda msg: forwarded.append(msg),
    )
    node.last_upstream_guard_reason = ""
    node.get_logger = lambda: SimpleNamespace(error=lambda _message: None)

    node._on_motor_command(_upstream_command(enable=False))

    assert stops and "canonical controller" in stops[0]
    assert forwarded == []


@pytest.mark.parametrize(
    ("name", "unsafe_value"),
    [
        ("hardware.max_disabled_legs", 2),
        ("rinbo.canonical_policy_launch_authorized", True),
        ("rinbo.command_topic", "/fake/motor_command"),
        ("rinbo.state_topic", "/fake/motor_state"),
        ("rinbo.power_state_topic", "/fake/power_state"),
        ("rinbo.preview_topic", "/motor/command"),
        ("rinbo.publish_preview", False),
        ("rinbo.publish_when_disabled", True),
        ("rinbo.publish_shutdown_disable", False),
        ("rinbo.require_state", False),
        ("rinbo.block_if_duplicate_command_publishers", False),
        ("rinbo.require_exact_command_contract", False),
        ("rinbo.require_single_telemetry_publisher", False),
        ("rinbo.require_monotonic_telemetry_stamp", False),
        ("rinbo.require_power_state", False),
        ("rinbo.require_power_relay", False),
        ("rinbo.block_if_duplicate_upstream_publishers", False),
        ("rinbo.expected_upstream_command_publisher_node", "/fake_controller"),
        ("rinbo.require_main_drive_calibration", False),
        ("rinbo.main_drive_calibrated", False),
        ("rinbo.require_abad_command_calibration", False),
        ("rinbo.abad_command_calibrated", False),
        ("rinbo.shutdown_disable_repeats", 7),
        ("rinbo.disabled_handshake_repeats", 7),
        ("rinbo.downstream_output_ack_timeout_s", 0.101),
        ("rinbo.state_timeout_s", 0.251),
        ("rinbo.main_pwm_per_rad_s", 40.1),
        ("rinbo.main_max_pwm", 80.1),
        ("rinbo.main_pwm_slew_rate_per_s", 250.1),
        ("rinbo.main_velocity_clip_rad_s", 80.0),
        ("rinbo.max_main_target_velocity_rad_s", 1.01),
        ("rinbo.max_abad_target_position_rad", 0.181),
        ("rinbo.power_state_timeout_s", 0.351),
        ("rinbo.command_timeout_s", 0.101),
        ("rinbo.min_bus_voltage", 17.9),
        ("rinbo.max_bus_voltage", 42.1),
        ("rinbo.max_current_a", 10.01),
        ("rinbo.current_trip_samples", 101),
        ("rinbo.voltage_trip_samples", 4),
        ("rinbo.upstream_command_max_age_s", 0.101),
        ("rinbo.power_bus_voltage_channel", 0),
        ("rinbo.leg_current_channels_rinbo_order", [0, 1, 2, 3, 4, 5]),
    ],
)
def test_active_rinbo_safety_contract_rejects_weakened_parameter(
    name: str, unsafe_value: object
) -> None:
    params = _safe_params()
    params[name] = unsafe_value

    with pytest.raises(ValueError, match=name.replace(".", r"\.")):
        _validate_active_rinbo_safety_contract(
            params, canonical_launch_provenance=True
        )
