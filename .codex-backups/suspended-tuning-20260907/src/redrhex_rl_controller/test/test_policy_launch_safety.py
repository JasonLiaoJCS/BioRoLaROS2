from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "redrhex_policy_bringup.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "redrhex_policy_launch_safety", LAUNCH_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _controller(profile: str, disabled_legs: list[str] | None = None) -> dict:
    return {
        "hardware": {
            "disabled_legs": list(disabled_legs or []),
            "max_disabled_legs": 1,
        },
        "policy": {
            "expected_sha256": "a" * 64,
            "expected_bridge_config_sha256": "b" * 64,
        },
        "observation": {"sensor_profile": profile},
        "state_machine": {
            "enable_policy_on_start": False,
            "enable_motor_output_on_start": False,
            "policy_run_max_duration_s": 3.0,
        },
        "safety": {
            "require_lowlevel_heartbeat": True,
            "require_motor_feedback": True,
        },
        "action": {"hardware_mapping_calibrated": True},
    }


def _bridge(
    backend: str = "biorola_ros", disabled_legs: list[str] | None = None
) -> dict:
    return {
        "backend": backend,
        "hardware": {
            "disabled_legs": list(disabled_legs or []),
            "max_disabled_legs": 1,
        },
        "rinbo": {
            "allow_enable": False,
            "publish_when_disabled": False,
            "publish_shutdown_disable": True,
            "shutdown_disable_repeats": 8,
            "disabled_handshake_repeats": 8,
            "require_downstream_output_ack": True,
            "downstream_output_ack_topic": "/rinbo/motor_output_enabled",
            "downstream_output_ack_timeout_s": 0.10,
            "require_state": True,
            "block_if_duplicate_command_publishers": True,
            "state_timeout_s": 0.25,
            "max_main_target_velocity_rad_s": 1.0,
            "require_main_drive_calibration": True,
            "main_drive_calibrated": True,
            "max_abad_target_position_rad": 0.18,
            "require_exact_command_contract": True,
            "require_single_telemetry_publisher": True,
            "expected_telemetry_publisher_node": "rinbo_ros2_bridge",
            "require_monotonic_telemetry_stamp": True,
            "require_abad_command_calibration": True,
            "abad_command_calibrated": True,
            "require_power_state": True,
            "require_power_relay": True,
            "power_state_timeout_s": 0.35,
            "command_timeout_s": 0.10,
            "block_if_duplicate_upstream_publishers": True,
            "upstream_command_max_age_s": 0.10,
            "recovery_healthy_samples": 5,
            "min_bus_voltage": 18.0,
            "max_bus_voltage": 30.0,
            "max_current_a": 3.0,
            "current_trip_samples": 3,
            "voltage_trip_samples": 3,
            "main_max_pwm": 80.0,
            "main_pwm_slew_rate_per_s": 250.0,
        },
    }


def test_canonical_bridge_capability_is_process_only_and_active_only(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(module.os, "getpid", lambda: 4321)
    monkeypatch.setattr(module, "_linux_process_start_ticks", lambda _pid: "99")
    monkeypatch.setattr(module.secrets, "token_hex", lambda _count: "a" * 64)

    assert module._canonical_bridge_environment(False) == {}
    assert module._canonical_bridge_environment(True) == {
        "REDRHEX_CANONICAL_POLICY_PARENT_PID": "4321",
        "REDRHEX_CANONICAL_POLICY_PARENT_START_TICKS": "99",
        "REDRHEX_CANONICAL_POLICY_LAUNCH_NONCE": "a" * 64,
    }


@pytest.mark.parametrize("profile", ["encoder_only_rig", "full_feedback_rig"])
def test_active_bridge_enable_accepts_only_real_strict_rigs(profile: str) -> None:
    module = _load_launch_module()

    effective = module._validate_active_bridge_enable(
        requested=True,
        controller_params=_controller(profile),
        bridge_params=_bridge(),
        requested_sensor_profile="",
        start_bridge=True,
        use_fake_sensors=False,
    )

    assert effective == profile


@pytest.mark.parametrize(
    ("controller_profile", "override_profile", "start_bridge", "fake", "backend", "reason"),
    [
        (
            "full_state",
            "encoder_only_rig",
            True,
            False,
            "biorola_ros",
            "selected controller YAML",
        ),
        (
            "encoder_only_rig",
            "full_state",
            True,
            False,
            "biorola_ros",
            "effective observation.sensor_profile",
        ),
        (
            "encoder_only_rig",
            "",
            False,
            False,
            "biorola_ros",
            "start_bridge must be true",
        ),
        (
            "encoder_only_rig",
            "",
            True,
            True,
            "biorola_ros",
            "use_fake_sensors must be false",
        ),
        (
            "encoder_only_rig",
            "",
            True,
            False,
            "mock",
            "backend must be biorola_ros",
        ),
    ],
)
def test_active_bridge_enable_rejects_nonhardware_or_spoofed_launches(
    controller_profile: str,
    override_profile: str,
    start_bridge: bool,
    fake: bool,
    backend: str,
    reason: str,
) -> None:
    module = _load_launch_module()

    with pytest.raises(RuntimeError, match=reason):
        module._validate_active_bridge_enable(
            requested=True,
            controller_params=_controller(controller_profile),
            bridge_params=_bridge(backend),
            requested_sensor_profile=override_profile,
            start_bridge=start_bridge,
            use_fake_sensors=fake,
        )


def test_inactive_bridge_latch_does_not_restrict_read_only_launch() -> None:
    module = _load_launch_module()

    effective = module._validate_active_bridge_enable(
        requested=False,
        controller_params=_controller("full_state"),
        bridge_params=_bridge("mock"),
        requested_sensor_profile="",
        start_bridge=False,
        use_fake_sensors=True,
    )

    assert effective == "full_state"


@pytest.mark.parametrize(
    "name",
    [
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
        "fake_publish_abad_joints",
        "fake_publish_damper_joints",
    ],
)
def test_active_bridge_enable_rejects_every_semantic_launch_override(
    name: str,
) -> None:
    module = _load_launch_module()

    with pytest.raises(RuntimeError, match="forbids launch overrides"):
        module._validate_active_bridge_enable(
            requested=True,
            controller_params=_controller("encoder_only_rig"),
            bridge_params=_bridge(),
            requested_sensor_profile=("encoder_only_rig" if name == "sensor_profile" else ""),
            start_bridge=True,
            use_fake_sensors=False,
            launch_overrides={name: "true"},
        )


@pytest.mark.parametrize(
    ("controller_mask", "bridge_mask", "launch_mask"),
    [
        (["L1"], ["R3"], ["L1"]),
        (["L1"], ["L1"], ["R3"]),
        ([], [], ["L1"]),
    ],
)
def test_active_bridge_enable_requires_identical_artifact_and_launch_masks(
    controller_mask: list[str], bridge_mask: list[str], launch_mask: list[str]
) -> None:
    module = _load_launch_module()

    with pytest.raises(RuntimeError, match="must be identical"):
        module._validate_active_bridge_enable(
            requested=True,
            controller_params=_controller("encoder_only_rig", controller_mask),
            bridge_params=_bridge(disabled_legs=bridge_mask),
            requested_sensor_profile="",
            start_bridge=True,
            use_fake_sensors=False,
            disabled_legs=launch_mask,
        )


def test_active_bridge_enable_accepts_same_single_disabled_leg_everywhere() -> None:
    module = _load_launch_module()

    effective = module._validate_active_bridge_enable(
        requested=True,
        controller_params=_controller("full_feedback_rig", ["L1"]),
        bridge_params=_bridge(disabled_legs=["L1"]),
        requested_sensor_profile="",
        start_bridge=True,
        use_fake_sensors=False,
        disabled_legs=["L1"],
    )

    assert effective == "full_feedback_rig"


@pytest.mark.parametrize("side", ["controller", "bridge"])
def test_active_bridge_enable_requires_both_max_disabled_legs_exactly_one(
    side: str,
) -> None:
    module = _load_launch_module()
    controller = _controller("encoder_only_rig")
    bridge = _bridge()
    target = controller if side == "controller" else bridge
    target["hardware"]["max_disabled_legs"] = 2

    with pytest.raises(RuntimeError, match="must both equal 1"):
        module._validate_active_bridge_enable(
            requested=True,
            controller_params=controller,
            bridge_params=bridge,
            requested_sensor_profile="",
            start_bridge=True,
            use_fake_sensors=False,
        )


@pytest.mark.parametrize(
    ("section", "key", "value", "reason"),
    [
        ("state_machine", "enable_policy_on_start", True, "must start disabled"),
        (
            "state_machine",
            "policy_run_max_duration_s",
            4.0,
            "must equal 3.0",
        ),
        (
            "safety",
            "require_lowlevel_heartbeat",
            False,
            "requirements must both be true",
        ),
        ("action", "hardware_mapping_calibrated", False, "must be true"),
    ],
)
def test_active_bridge_enable_rejects_weakened_controller_yaml(
    section: str, key: str, value: object, reason: str
) -> None:
    module = _load_launch_module()
    controller = _controller("encoder_only_rig")
    controller[section][key] = value

    with pytest.raises(RuntimeError, match=reason):
        module._validate_active_bridge_enable(
            requested=True,
            controller_params=controller,
            bridge_params=_bridge(),
            requested_sensor_profile="",
            start_bridge=True,
            use_fake_sensors=False,
        )


@pytest.mark.parametrize(
    ("key", "value", "reason"),
    [
        ("require_state", False, "safety/calibration gates"),
        ("abad_command_calibrated", False, "safety/calibration gates"),
        ("max_current_a", 3.1, "max_current_a"),
        ("min_bus_voltage", 17.9, "min_bus_voltage"),
        ("command_timeout_s", 0.101, "command_timeout_s"),
        ("main_max_pwm", 80.1, "main_max_pwm"),
        ("current_trip_samples", 4, "current_trip_samples"),
    ],
)
def test_active_bridge_enable_rejects_weakened_bridge_yaml(
    key: str, value: object, reason: str
) -> None:
    module = _load_launch_module()
    bridge = _bridge()
    bridge["rinbo"][key] = value

    with pytest.raises(RuntimeError, match=reason):
        module._validate_active_bridge_enable(
            requested=True,
            controller_params=_controller("encoder_only_rig"),
            bridge_params=bridge,
            requested_sensor_profile="",
            start_bridge=True,
            use_fake_sensors=False,
        )
