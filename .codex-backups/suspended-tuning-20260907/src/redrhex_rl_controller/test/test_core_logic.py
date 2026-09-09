from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from sensor_msgs.msg import JointState

from redrhex_rl_controller import redrhex_contract as C
from redrhex_rl_controller.action_decoder import ActionDecoder
from redrhex_rl_controller.degraded_mode import normalize_disabled_legs, policy_indices_for_disabled_legs
from redrhex_rl_controller.golden_policy import (
    GOLDEN_SCHEMA,
    GOLDEN_PRODUCER,
    bridge_config_sha256,
    controller_config_sha256,
    controller_params_with_disabled_legs,
    decoder_config_sha256,
    decoder_config_from_ros_params,
    decoder_source_sha256,
    deployment_source_sha256,
    load_bridge_ros_params,
    load_controller_ros_params,
    observation_source_sha256,
    require_complete_bridge_params,
    require_complete_controller_params,
    required_active_bridge_parameter_names,
    sha256_file,
)
from redrhex_rl_controller.golden_recorder import IsaacLabGoldenRecorder
from redrhex_rl_controller.observation_builder import ObservationBuilder
from redrhex_rl_controller.policy_onnx_runner import PolicyONNXRunner
from redrhex_rl_controller.preflight_check import (
    _load_bridge_params,
    _validate_bridge_config,
)
from redrhex_rl_controller.rl_controller_node import RedRhexRLControllerNode
from redrhex_rl_controller.safety_filter import SafetyFilter, SafetyState
from redrhex_rl_controller.state_machine import RedRhexState, RedRhexStateMachine


def _imu_msg() -> SimpleNamespace:
    return SimpleNamespace(
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        angular_velocity=SimpleNamespace(x=0.1, y=-0.2, z=0.3),
    )


def _cmd_vel_msg(vx: float = 0.1, vy: float = 0.0, wz: float = 0.0) -> SimpleNamespace:
    return SimpleNamespace(
        linear=SimpleNamespace(x=vx, y=vy, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=wz),
    )


def _joint_state_msg() -> SimpleNamespace:
    return SimpleNamespace(
        name=list(C.MAIN_DRIVE_JOINT_NAMES),
        position=[0.01 * i for i in range(6)],
        velocity=[0.1 * i for i in range(6)],
    )


def _active_bridge_profile(name: str = "encoder_only_rig") -> tuple[Path, dict]:
    path = (
        Path(__file__).resolve().parents[2]
        / "redrhex_lowlevel_bridge"
        / "config"
        / f"lowlevel_bridge_{name}.yaml"
    )
    return path, load_bridge_ros_params(path)


def _fixed_forward_policy_run_node(
    *, now_s: float = 20.0, state_enter_time: float = 10.0, lease_s: float = 3.0
) -> RedRhexRLControllerNode:
    builder = ObservationBuilder(
        {
            "command_profile": "fixed_forward",
            "fixed_forward_vx": 0.11,
        }
    )
    builder.set_policy_running(True)
    state_machine = RedRhexStateMachine()
    state_machine.transition(RedRhexState.POLICY_RUN, "test fixture")

    node = object.__new__(RedRhexRLControllerNode)
    node.enable_policy = True
    node.enable_motor_output = True
    node.motor_enable_request_time = state_enter_time
    node.init_stand_output_start_time = state_enter_time
    node.init_stand_stable_since = state_enter_time
    node.init_stand_verified = True
    node.estop = False
    node.state_enter_time = state_enter_time
    node.policy_run_max_duration_s = lease_s
    node.state_machine = state_machine
    node.observation_builder = builder
    node.action_decoder = ActionDecoder()
    node._now_s = lambda: now_s
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda _message: None,
        info=lambda _message: None,
    )
    return node


def _assert_policy_run_ended(node: RedRhexRLControllerNode) -> None:
    assert node.state_machine.state == RedRhexState.INIT_STAND
    assert not node.enable_policy
    assert not node.enable_motor_output
    assert node.motor_enable_request_time is None
    assert node.init_stand_output_start_time is None
    assert node.init_stand_stable_since is None
    assert not node.init_stand_verified
    np.testing.assert_array_equal(node.observation_builder.cmd_vel, np.zeros(3))


def test_observation_builder_single_and_history_shapes() -> None:
    builder = ObservationBuilder({"policy_input_dim": C.OBS_DIM_SINGLE})
    builder.update_imu(_imu_msg(), now_s=1.0)
    builder.update_joint_state(_joint_state_msg(), now_s=1.0)
    builder.update_cmd_vel(_cmd_vel_msg(), now_s=1.0)

    status = builder.status(now_s=1.01, sensor_timeout_s=0.1, cmd_timeout_s=0.25)
    assert status.ok, status.reasons

    obs = builder.build_policy_input(now_s=1.01)
    assert obs.shape == (C.OBS_DIM_SINGLE,)
    assert np.isfinite(obs).all()

    history_builder = ObservationBuilder(
        {"policy_input_dim": C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH}
    )
    history_builder.update_imu(_imu_msg(), now_s=1.0)
    history_builder.update_joint_state(_joint_state_msg(), now_s=1.0)
    history_builder.update_cmd_vel(_cmd_vel_msg(), now_s=1.0)
    history = history_builder.build_policy_input(now_s=1.01)
    assert history.shape == (C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH,)
    assert np.isfinite(history).all()


def test_strict_history_length_and_onnx_width_fail_at_startup_binding() -> None:
    with pytest.raises(ValueError, match="policy_history_length must be 5"):
        ObservationBuilder(
            {
                "sensor_profile": "encoder_only_rig",
                "encoder_only_rig_acknowledged": True,
                "policy_history_length": 4,
            }
        )
    builder = ObservationBuilder()
    with pytest.raises(ValueError, match="incompatible with history length"):
        builder.set_policy_input_dim(4 * C.OBS_DIM_SINGLE)


def test_encoder_only_rig_requires_ack_and_builds_from_joint_state_without_imu() -> None:
    with pytest.raises(ValueError, match="encoder_only_rig_acknowledged"):
        ObservationBuilder({"sensor_profile": "encoder_only_rig"})

    builder = ObservationBuilder(
        {
            "sensor_profile": "encoder_only_rig",
            "encoder_only_rig_acknowledged": True,
            "policy_input_dim": C.OBS_DIM_SINGLE,
        }
    )
    joint_state = JointState()
    joint_state.name = list(C.MAIN_DRIVE_JOINT_NAMES)
    joint_state.position = [0.01 * index for index in range(6)]
    joint_state.velocity = [0.1 * index for index in range(6)]
    builder.update_joint_state(joint_state, now_s=1.0)

    status = builder.status(now_s=1.01, sensor_timeout_s=0.1, cmd_timeout_s=0.25)
    assert status.ok, status.reasons

    observation = builder.build_policy_input(now_s=1.01)
    np.testing.assert_allclose(observation[0:3], np.zeros(3))
    np.testing.assert_allclose(observation[3:6], np.zeros(3))
    np.testing.assert_allclose(observation[6:9], [0.0, -1.0, 0.0])

    provenance = builder.observation_provenance()
    assert len(provenance) == C.OBS_DIM_SINGLE
    assert provenance == (
        ["imputed_zero"] * 6
        + ["rig_constant"] * 3
        + ["encoder"] * 12
        + ["encoder_velocity"] * 6
        + ["commanded_estimate"] * 6
        + ["commanded_estimate_velocity"] * 6
        + ["command"] * 3
        + ["internal_phase"] * 2
        + ["previous_policy_action"] * 12
    )


def test_disabled_l1_observation_is_nominal_even_without_readback() -> None:
    builder = ObservationBuilder(
        {
            "sensor_profile": "encoder_only_rig",
            "encoder_only_rig_acknowledged": True,
            "disabled_leg_indices": [3],
            "disabled_leg_observation_mode": "nominal",
        }
    )
    msg = JointState()
    msg.name = [
        name for index, name in enumerate(C.MAIN_DRIVE_JOINT_NAMES) if index != 3
    ]
    msg.position = [0.1] * 5
    msg.velocity = [0.2] * 5
    builder.commanded_abad_pos[3] = 99.0
    builder.commanded_abad_vel[3] = 99.0
    builder.update_joint_state(msg, now_s=1.0)
    assert builder.status(1.01, 0.1, 0.25).ok
    observation = builder.build_single(1.01, update_phase=False)
    assert observation[9 + 3] == pytest.approx(
        np.sin(C.INIT_MAIN_DRIVE_POS[3])
    )
    assert observation[15 + 3] == pytest.approx(
        np.cos(C.INIT_MAIN_DRIVE_POS[3])
    )
    assert observation[21 + 3] == 0.0
    assert observation[27 + 3] == 0.0
    assert observation[33 + 3] == 0.0
    for offset in (9, 15, 21, 27, 33):
        assert builder.observation_provenance()[offset + 3] == "disabled_leg_imputed"


def test_encoder_only_fixed_forward_command_is_active_only_during_policy_run() -> None:
    fixed_vx = 0.11
    builder = ObservationBuilder(
        {
            "sensor_profile": "encoder_only_rig",
            "encoder_only_rig_acknowledged": True,
            "command_profile": "fixed_forward",
            "fixed_forward_vx": fixed_vx,
            "command_limits": {
                "vx_min": 0.0,
                "vx_max": 0.12,
                "vy_min": -0.14,
                "vy_max": 0.14,
                "wz_min": -0.17,
                "wz_max": 0.17,
            },
        }
    )
    builder.update_joint_state(_joint_state_msg(), now_s=1.0)

    # Autonomous motion must not leak into INIT_STAND/WARMUP/POLICY_READY.
    np.testing.assert_array_equal(builder.cmd_vel, np.zeros(3))

    builder.set_policy_running(True)
    np.testing.assert_allclose(builder.cmd_vel, [fixed_vx, 0.0, 0.0])

    # A stray external publisher cannot steer or stop the fixed-forward profile.
    builder.update_cmd_vel(_cmd_vel_msg(vx=0.0, vy=0.14, wz=-0.17), now_s=1.01)
    np.testing.assert_allclose(builder.cmd_vel, [fixed_vx, 0.0, 0.0])

    # Fixed commands do not depend on /cmd_vel freshness.
    status = builder.status(now_s=10.0, sensor_timeout_s=20.0, cmd_timeout_s=0.25)
    assert status.ok, status.reasons
    np.testing.assert_allclose(builder.cmd_vel, [fixed_vx, 0.0, 0.0])
    observation = builder.build_single(now_s=10.0, update_phase=False)
    start, stop = C.OBSERVATION_SLICES["velocity_command"]
    np.testing.assert_allclose(observation[start:stop], [fixed_vx, 0.0, 0.0])

    # Every exit from POLICY_RUN and every reset must synchronously remove motion.
    builder.set_policy_running(False)
    np.testing.assert_array_equal(builder.cmd_vel, np.zeros(3))
    builder.set_policy_running(True)
    np.testing.assert_allclose(builder.cmd_vel, [fixed_vx, 0.0, 0.0])
    builder.reset()
    np.testing.assert_array_equal(builder.cmd_vel, np.zeros(3))


def test_external_cmd_vel_profile_preserves_topic_clipping_and_timeout() -> None:
    builder = ObservationBuilder(
        {
            "command_limits": {
                "vx_min": 0.0,
                "vx_max": 0.12,
                "vy_min": -0.14,
                "vy_max": 0.14,
                "wz_min": -0.17,
                "wz_max": 0.17,
            },
        }
    )
    assert builder.command_profile == "external_cmd_vel"

    builder.update_cmd_vel(_cmd_vel_msg(vx=0.5, vy=-0.5, wz=0.5), now_s=1.0)
    np.testing.assert_allclose(builder.cmd_vel, [0.12, -0.14, 0.17])
    builder.status(now_s=1.24, sensor_timeout_s=0.1, cmd_timeout_s=0.25)
    np.testing.assert_allclose(builder.cmd_vel, [0.12, -0.14, 0.17])

    builder.status(now_s=1.251, sensor_timeout_s=0.1, cmd_timeout_s=0.25)
    np.testing.assert_array_equal(builder.cmd_vel, np.zeros(3))


@pytest.mark.parametrize(
    ("overrides", "error"),
    [
        ({"command_profile": "autonomous"}, "command_profile"),
        (
            {"command_profile": "fixed_forward", "fixed_forward_vx": float("nan")},
            "fixed_forward_vx",
        ),
        (
            {"command_profile": "fixed_forward", "fixed_forward_vx": float("inf")},
            "fixed_forward_vx",
        ),
        (
            {"command_profile": "fixed_forward", "fixed_forward_vx": 0.0},
            "fixed_forward_vx",
        ),
        (
            {"command_profile": "fixed_forward", "fixed_forward_vx": 0.10},
            "fixed_forward_vx",
        ),
        (
            {
                "command_profile": "fixed_forward",
                "fixed_forward_vx": 0.121,
                "command_limits": {"vx_min": 0.0, "vx_max": 0.12},
            },
            "fixed_forward_vx",
        ),
        (
            {
                "command_profile": "fixed_forward",
                "fixed_forward_vx": 0.11,
                "command_limits": {"vy_min": 0.01, "vy_max": 0.14},
            },
            "fixed_forward requires zero",
        ),
        (
            {
                "command_profile": "fixed_forward",
                "fixed_forward_vx": 0.11,
                "command_limits": {"wz_min": -0.17, "wz_max": -0.01},
            },
            "fixed_forward requires zero",
        ),
        (
            {
                "command_profile": "fixed_forward",
                "fixed_forward_vx": 0.11,
                "command_limits": {"vx_min": 0.115, "vx_max": 0.12},
            },
            "fixed_forward_vx",
        ),
    ],
    ids=[
        "unknown-profile",
        "nan",
        "infinity",
        "zero",
        "stage5-threshold",
        "above-upper-limit",
        "lateral-zero-outside-limits",
        "yaw-zero-outside-limits",
        "below-lower-limit",
    ],
)
def test_command_profile_rejects_invalid_config(
    overrides: dict[str, object], error: str
) -> None:
    with pytest.raises(ValueError, match=error):
        ObservationBuilder(overrides)


def test_checked_in_profiles_enable_fixed_forward_only_for_suspended_rigs() -> None:
    config_dir = Path(__file__).resolve().parents[1] / "config"
    expected_profiles = {
        "redrhex_policy.yaml": ("external_cmd_vel", 0.0),
        "redrhex_policy_bench_safe.yaml": ("external_cmd_vel", 0.0),
        "redrhex_policy_encoder_only_rig.yaml": ("fixed_forward", 3.0),
        "redrhex_policy_full_feedback_rig.yaml": ("fixed_forward", 3.0),
        "redrhex_policy_locomotion_tune.yaml": ("external_cmd_vel", 0.0),
    }

    for filename, (expected_profile, expected_lease_s) in expected_profiles.items():
        document = yaml.safe_load((config_dir / filename).read_text(encoding="utf-8"))
        params = document["redrhex_rl_controller"]["ros__parameters"]
        commands = params["commands"]
        assert commands["profile"] == expected_profile
        lease_s = float(params["state_machine"]["policy_run_max_duration_s"])
        assert lease_s == expected_lease_s
        if expected_profile == "fixed_forward":
            fixed_vx = float(commands["fixed_forward_vx"])
            assert np.isfinite(fixed_vx)
            assert 0.10 < fixed_vx <= float(commands["vx_max"])
            assert np.isfinite(lease_s)
            assert 0.0 < lease_s <= 10.0


def test_controller_stop_path_clears_fixed_forward_command() -> None:
    builder = ObservationBuilder(
        {
            "command_profile": "fixed_forward",
            "fixed_forward_vx": 0.11,
        }
    )
    builder.set_policy_running(True)

    node = object.__new__(RedRhexRLControllerNode)
    node.enable_policy = True
    node.enable_motor_output = True
    node.motor_enable_request_time = 1.0
    node.init_stand_output_start_time = 1.0
    node.init_stand_stable_since = 1.0
    node.init_stand_verified = True
    node.observation_builder = builder
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

    RedRhexRLControllerNode._drop_enable_latches(node, "test protective stop")

    assert not node.enable_policy
    assert not node.enable_motor_output
    assert node.motor_enable_request_time is None
    assert node.init_stand_output_start_time is None
    assert node.init_stand_stable_since is None
    assert not node.init_stand_verified
    np.testing.assert_array_equal(builder.cmd_vel, np.zeros(3))


def test_policy_disable_during_run_clears_both_latches_and_requires_rearm() -> None:
    node = _fixed_forward_policy_run_node()

    RedRhexRLControllerNode._on_enable_policy(node, SimpleNamespace(data=False))

    _assert_policy_run_ended(node)
    assert "policy disabled" in node.state_machine.last_transition_reason

    # Re-enabling policy without a new motor/INIT_STAND cycle must fail closed.
    RedRhexRLControllerNode._on_enable_policy(node, SimpleNamespace(data=True))
    assert not node.enable_policy
    assert not node.enable_motor_output
    assert node.state_machine.state == RedRhexState.INIT_STAND


def test_policy_run_lease_boundary_and_expiry_path_require_rearm() -> None:
    node = _fixed_forward_policy_run_node(
        now_s=13.0, state_enter_time=10.0, lease_s=3.0
    )

    assert not RedRhexRLControllerNode._policy_run_lease_expired(node, 12.999999)
    assert RedRhexRLControllerNode._policy_run_lease_expired(node, 13.0)

    RedRhexRLControllerNode._end_policy_run(
        node, "policy run lease expired; test requires re-arm"
    )

    _assert_policy_run_ended(node)
    assert "lease expired" in node.state_machine.last_transition_reason

    unlimited = _fixed_forward_policy_run_node(lease_s=0.0)
    assert not RedRhexRLControllerNode._policy_run_lease_expired(unlimited, 1.0e9)
    unlimited.state_machine.transition(RedRhexState.INIT_STAND, "not running")
    unlimited.policy_run_max_duration_s = 3.0
    assert not RedRhexRLControllerNode._policy_run_lease_expired(unlimited, 1.0e9)


@pytest.mark.parametrize(
    ("lease_s", "error"),
    [
        (float("nan"), "finite and >= 0"),
        (float("inf"), "finite and >= 0"),
        (-1.0, "finite and >= 0"),
        (0.0, "policy_run_max_duration_s.*\\(0, 10\\]"),
        (4.0, "policy_run_max_duration_s must be exactly 3.0"),
        (10.001, "policy_run_max_duration_s.*\\(0, 10\\]"),
    ],
    ids=[
        "nan",
        "infinity",
        "negative",
        "zero-fixed-lease",
        "noncanonical-rig-lease",
        "over-ten-seconds",
    ],
)
def test_encoder_only_node_rejects_invalid_policy_run_lease(
    tmp_path: Path, lease_s: float, error: str
) -> None:
    import rclpy

    canonical = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "redrhex_policy_encoder_only_rig.yaml"
    )
    document = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    document["redrhex_rl_controller"]["ros__parameters"]["state_machine"][
        "policy_run_max_duration_s"
    ] = lease_s
    config = tmp_path / "invalid_policy_run_lease.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")

    node = None
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    try:
        with pytest.raises(ValueError, match=error):
            node = RedRhexRLControllerNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize(
    "field", ["max_abs_roll_rad", "max_abs_pitch_rad"]
)
def test_encoder_only_node_rejects_relaxed_tilt_limit(
    tmp_path: Path, field: str
) -> None:
    import rclpy

    canonical = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "redrhex_policy_encoder_only_rig.yaml"
    )
    document = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    document["redrhex_rl_controller"]["ros__parameters"]["safety"][field] = 0.451
    config = tmp_path / f"relaxed_{field}.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")

    node = None
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    try:
        with pytest.raises(ValueError, match=rf"safety\.{field} must be <= 0.45"):
            node = RedRhexRLControllerNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_full_state_external_profile_accepts_zero_policy_run_lease(tmp_path: Path) -> None:
    import rclpy

    canonical = (
        Path(__file__).resolve().parents[1] / "config" / "redrhex_policy.yaml"
    )
    config = tmp_path / "external_unlimited_policy_run.yaml"
    config.write_text(canonical.read_text(encoding="utf-8"), encoding="utf-8")

    node = None
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    try:
        node = RedRhexRLControllerNode()
        assert node.command_profile == "external_cmd_vel"
        assert node.policy_run_max_duration_s == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_controller_disabled_leg_parameters_are_startup_only(tmp_path: Path) -> None:
    import rclpy
    from rclpy.parameter import Parameter

    canonical = (
        Path(__file__).resolve().parents[1] / "config" / "redrhex_policy.yaml"
    )
    config = tmp_path / "startup_only_mask.yaml"
    config.write_text(canonical.read_text(encoding="utf-8"), encoding="utf-8")

    node = None
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    try:
        node = RedRhexRLControllerNode()
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


@pytest.mark.parametrize("lease_s", [0.0, 10.001], ids=["zero", "over-ten-seconds"])
def test_fixed_forward_requires_bounded_lease_even_with_full_state(
    tmp_path: Path, lease_s: float
) -> None:
    import rclpy

    canonical = (
        Path(__file__).resolve().parents[1] / "config" / "redrhex_policy.yaml"
    )
    document = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    params = document["redrhex_rl_controller"]["ros__parameters"]
    params["commands"]["profile"] = "fixed_forward"
    params["state_machine"]["policy_run_max_duration_s"] = lease_s
    config = tmp_path / "unbounded_full_state_fixed_forward.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")

    node = None
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    try:
        with pytest.raises(ValueError, match="policy_run_max_duration_s"):
            node = RedRhexRLControllerNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_last_action_observation_matches_simulator_two_inference_lag() -> None:
    builder = ObservationBuilder(
        {
            "sensor_profile": "encoder_only_rig",
            "encoder_only_rig_acknowledged": True,
        }
    )
    raw = np.linspace(-1.5, 1.5, C.ACTION_DIM, dtype=np.float32)
    builder.update_last_actions(raw)
    np.testing.assert_allclose(builder.last_actions, 0.0)
    next_raw = -raw
    builder.update_last_actions(next_raw)
    np.testing.assert_allclose(
        builder.last_actions,
        np.clip(raw, -C.TRAINING_ACTION_CLIP, C.TRAINING_ACTION_CLIP),
    )
    # Rig attenuation may be 0.35, but indices 44:56 follow the training
    # clamp, not the hardware-only attenuation.
    assert float(np.max(np.abs(builder.last_actions))) == C.TRAINING_ACTION_CLIP
    imputed_indices = builder.imputed_observation_indices()
    assert len(imputed_indices) == 21
    assert imputed_indices == list(range(0, 9)) + list(range(27, 39))


def test_fake_sensor_training_quaternion_projects_reference_gravity() -> None:
    from redrhex_rl_controller.fake_sensor_node import training_frame_quaternion_xyzw

    quaternion = np.asarray(training_frame_quaternion_xyzw(), dtype=np.float64)
    assert quaternion.shape == (4,)
    assert np.isfinite(quaternion).all()
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)

    builder = ObservationBuilder()
    builder.update_imu(
        SimpleNamespace(
            orientation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
            angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        now_s=1.0,
    )
    builder.update_joint_state(_joint_state_msg(), now_s=1.0)

    observation = builder.build_single(now_s=1.01, update_phase=False)
    np.testing.assert_allclose(
        observation[6:9], C.REFERENCE_PROJECTED_GRAVITY, atol=1.0e-6
    )


def test_action_decoder_outputs_safe_command_shapes() -> None:
    decoder = ActionDecoder()
    disabled = decoder.disabled_command()
    assert len(disabled.joint_names) == 12
    assert not disabled.enable

    init = decoder.init_stand_command(current_main_pos=np.zeros(6))
    assert len(init.target_position_rad) == 12
    assert len(init.target_velocity_rad_s) == 12
    assert init.enable

    decoded = decoder.decode(
        np.zeros(C.ACTION_DIM, dtype=np.float32),
        main_drive_pos=np.zeros(6),
        abad_pos=np.zeros(6),
        command=np.array([0.2, 0.0, 0.0]),
        projected_gravity=np.array([0.0, -1.0, 0.0]),
        dt=C.CONTROL_DT,
    )
    assert decoded.enable
    assert decoded.safe_action.shape == (C.ACTION_DIM,)
    assert decoded.target_main_drive_velocity.shape == (6,)
    assert decoded.target_abad_position.shape == (6,)

    with pytest.raises(ValueError):
        decoder.decode(np.zeros(11), np.zeros(6), np.zeros(6), np.zeros(3), np.zeros(3), C.CONTROL_DT)


def test_play_forward_compat_matches_simulator_equation() -> None:
    decoder = ActionDecoder(
        {
            "action_warmup_steps": 0,
            "main_drive_slew_rate_rad_s2": 1.0e6,
            "play_forward_compat_enable": True,
            "play_forward_compat_bias_scale": 1.0,
            "play_forward_compat_residual_scale": 0.04,
            "play_forward_compat_residual_clip": 0.30,
        }
    )
    main_pos = np.zeros(6)
    action = np.full(C.ACTION_DIM, 0.8, dtype=np.float32)
    decoded = decoder.decode(
        action,
        main_pos,
        np.zeros(6),
        np.asarray([0.22, 0.0, 0.0]),
        np.asarray(C.REFERENCE_PROJECTED_GRAVITY),
        C.CONTROL_DT,
        gait_phase=0.0,
    )
    desired_phase = np.remainder(decoder.leg_phase_offsets, 2.0 * np.pi)
    forward_profile = np.where(
        decoder._in_stance_phase(desired_phase),
        decoder.stance_velocity,
        decoder.swing_velocity,
    ) + np.clip(
        -decoder.forward_phase_lock_gain
        * decoder._phase_error(np.zeros(6), desired_phase),
        -2.0,
        2.0,
    )
    vx_norm = 0.22 / 0.45
    expected = (
        forward_profile * decoder.direction_multiplier * vx_norm
        + 0.30 * decoder.drive_vel_scale * 0.04
    )
    max_vel = max(decoder.swing_velocity * 1.5, decoder.drive_vel_scale * 1.5)
    np.testing.assert_allclose(
        decoded.target_main_drive_velocity,
        np.clip(expected, -max_vel, max_vel),
        rtol=0.0,
        atol=1.0e-9,
    )


@pytest.mark.parametrize(
    ("physical_leg", "policy_index"),
    [("R1", 0), ("R2", 1), ("R3", 2), ("L1", 3), ("L2", 4), ("L3", 5)],
)
def test_degraded_mode_maps_every_physical_leg_to_policy_index(
    physical_leg: str, policy_index: int
) -> None:
    disabled_legs = normalize_disabled_legs([f" {physical_leg.lower()} "])

    assert disabled_legs == [physical_leg]
    assert policy_indices_for_disabled_legs(disabled_legs) == [policy_index]


def test_degraded_mode_accepts_empty_and_rejects_bad_names() -> None:
    assert normalize_disabled_legs([]) == []
    assert policy_indices_for_disabled_legs([]) == []

    with pytest.raises(ValueError, match="Unknown hardware.disabled_legs"):
        normalize_disabled_legs(["X1"])
    with pytest.raises(ValueError, match="Duplicate hardware.disabled_legs"):
        normalize_disabled_legs(["L1", "l1"], max_disabled_legs=2)


@pytest.mark.parametrize(
    ("physical_leg", "disabled_index"),
    [("R1", 0), ("R2", 1), ("R3", 2), ("L1", 3), ("L2", 4), ("L3", 5)],
)
def test_action_decoder_masks_every_disabled_leg_in_init_stand_and_policy_commands(
    physical_leg: str, disabled_index: int
) -> None:
    assert policy_indices_for_disabled_legs([physical_leg]) == [disabled_index]
    decoder = ActionDecoder(
        {
            "disabled_leg_indices": [disabled_index],
            "action_warmup_steps": 0,
            "main_drive_slew_rate_rad_s2": 1000.0,
            "abad_slew_rate_rad_s": 1000.0,
        }
    )

    init = decoder.init_stand_command(current_main_pos=np.zeros(6))
    abad_output_index = 6 + disabled_index
    assert init.target_main_drive_velocity[disabled_index] == 0.0
    assert init.target_velocity_rad_s[disabled_index] == 0.0
    assert init.kp[disabled_index] == 0.0
    assert init.kd[disabled_index] == 0.0
    assert init.effort_limit_nm[disabled_index] == 0.0
    assert init.kp[abad_output_index] == 0.0
    assert init.kd[abad_output_index] == 0.0
    assert init.effort_limit_nm[abad_output_index] == 0.0
    assert any(
        abs(value) > 0.0
        for index, value in enumerate(init.target_main_drive_velocity)
        if index != disabled_index
    )

    decoded = decoder.decode(
        np.ones(C.ACTION_DIM, dtype=np.float32),
        main_drive_pos=np.zeros(6),
        abad_pos=np.zeros(6),
        command=np.zeros(3),
        projected_gravity=np.array([0.0, -1.0, 0.0]),
        dt=C.CONTROL_DT,
    )

    assert decoded.target_main_drive_velocity[disabled_index] == 0.0
    assert decoded.target_abad_position[disabled_index] == 0.0
    assert decoded.target_velocity_rad_s[disabled_index] == 0.0
    assert decoded.target_position_rad[abad_output_index] == 0.0
    assert decoded.safe_action[disabled_index] == 0.0
    assert decoded.safe_action[abad_output_index] == 0.0
    assert decoded.kp[disabled_index] == 0.0
    assert decoded.kd[disabled_index] == 0.0
    assert decoded.effort_limit_nm[disabled_index] == 0.0
    assert decoded.kp[abad_output_index] == 0.0
    assert decoded.kd[abad_output_index] == 0.0
    assert decoded.effort_limit_nm[abad_output_index] == 0.0
    assert any(
        abs(value) > 0.0
        for index, value in enumerate(decoded.target_main_drive_velocity)
        if index != disabled_index
    )
    assert any(
        abs(value) > 0.0
        for index, value in enumerate(decoded.target_abad_position)
        if index != disabled_index
    )


def test_safety_filter_reports_bad_inputs_without_throwing() -> None:
    safety = SafetyFilter()
    ok_state = SafetyState(
        imu_age_s=0.01,
        joint_state_age_s=0.01,
        command=np.zeros(3),
        control_loop_dt_s=C.CONTROL_DT,
    )
    assert safety.check(ok_state).ok

    bad_command = SafetyState(
        imu_age_s=0.01,
        joint_state_age_s=0.01,
        command=np.array([0.0, 0.0]),
        control_loop_dt_s=C.CONTROL_DT,
    )
    result = safety.check(bad_command)
    assert not result.ok
    assert any("velocity command shape" in reason for reason in result.reasons)

    bad_action = safety.check(ok_state, raw_action=np.zeros(11))
    assert not bad_action.ok
    assert any("policy action shape" in reason for reason in bad_action.reasons)


def test_safety_filter_clips_at_decoder_but_only_stops_implausible_raw_action() -> None:
    safety = SafetyFilter({"action_clip": 0.35, "max_raw_action_abs": 1.5})
    state = SafetyState(
        imu_age_s=0.01,
        joint_state_age_s=0.01,
        command=np.zeros(3),
        control_loop_dt_s=C.CONTROL_DT,
    )

    # A normal actor output may exceed the conservative bench attenuation;
    # ActionDecoder clips it, so this is not itself a policy fault.
    assert safety.check(state, raw_action=np.full(C.ACTION_DIM, 0.8)).ok

    at_threshold = np.zeros(C.ACTION_DIM)
    at_threshold[3] = -1.5
    assert safety.check(state, raw_action=at_threshold).ok

    just_over_threshold = np.zeros(C.ACTION_DIM)
    just_over_threshold[3] = -1.501
    over_result = safety.check(state, raw_action=just_over_threshold)
    assert not over_result.ok
    assert "raw policy action magnitude implausibly large" in over_result.reasons

    implausible = safety.check(state, raw_action=np.full(C.ACTION_DIM, 2.0))
    assert not implausible.ok
    assert "raw policy action magnitude implausibly large" in implausible.reasons


def test_observation_builder_rejects_per_joint_stale_or_nonfinite_updates() -> None:
    builder = ObservationBuilder(
        {
            "sensor_profile": "encoder_only_rig",
            "encoder_only_rig_acknowledged": True,
        }
    )
    builder.update_joint_state(_joint_state_msg(), now_s=1.0)
    assert builder.status(1.01, 0.1, 0.25).ok

    partial = _joint_state_msg()
    partial.name = partial.name[:-1]
    partial.position = partial.position[:-1]
    partial.velocity = partial.velocity[:-1]
    builder.update_joint_state(partial, now_s=1.2)
    status = builder.status(1.21, 0.1, 0.25)
    assert not status.ok
    assert any("stale joints" in reason for reason in status.reasons)

    bad = _joint_state_msg()
    bad.position[0] = float("nan")
    builder.update_joint_state(bad, now_s=1.22)
    status = builder.status(1.23, 0.1, 0.25)
    assert not status.ok
    assert any("invalid joints" in reason for reason in status.reasons)


def test_safety_filter_encoder_only_mode_accepts_no_imu_and_rejects_tilt_guard() -> None:
    safety = SafetyFilter({"require_imu": False, "enable_tilt_guard": False})
    state = SafetyState(
        imu_age_s=None,
        joint_state_age_s=0.01,
        command=np.zeros(3),
        control_loop_dt_s=C.CONTROL_DT,
        roll_rad=10.0,
        pitch_rad=-10.0,
    )

    result = safety.check(state)
    assert result.ok, result.reasons

    with pytest.raises(ValueError, match="enable_tilt_guard=true requires require_imu=true"):
        SafetyFilter({"require_imu": False, "enable_tilt_guard": True})


def test_safety_filter_ignores_disabled_l1_fault_and_current_only() -> None:
    safety = SafetyFilter(
        {
            "disabled_leg_indices": [3],
            "max_motor_current_a": 3.0,
            "motor_current_trip_samples": 2,
        }
    )
    state = SafetyState(
        imu_age_s=0.01,
        joint_state_age_s=0.01,
        command=np.zeros(3),
        control_loop_dt_s=C.CONTROL_DT,
        control_sequence=1,
        motor_feedback_sequence=1,
        motor_currents_a=[0.0, 0.0, 0.0, 4.5, 0.0, 0.0],
        motor_faults=[False, False, False, True, False, False],
    )

    first = safety.check(state)
    assert first.ok, first.reasons

    # Rechecking the same feedback message must not consume another debounce sample.
    duplicate = safety.check(state)
    assert duplicate.ok, duplicate.reasons

    state.motor_feedback_sequence = 2
    state.control_sequence = 2
    second = safety.check(state)
    assert second.ok, second.reasons

    state.motor_currents_a[3] = float("nan")
    state.motor_feedback_sequence = 3
    state.control_sequence = 3
    disabled_nan = safety.check(state)
    assert disabled_nan.ok, disabled_nan.reasons

    state.motor_currents_a[0] = float("nan")
    state.motor_feedback_sequence = 4
    state.control_sequence = 4
    healthy_nan = safety.check(state)
    assert not healthy_nan.ok
    assert "healthy motor current NaN/Inf" in healthy_nan.reasons
    assert "motor fault flag" not in second.reasons

    active_fault_safety = SafetyFilter({"disabled_leg_indices": [3]})
    active_fault = SafetyState(
        imu_age_s=0.01,
        joint_state_age_s=0.01,
        command=np.zeros(3),
        control_loop_dt_s=C.CONTROL_DT,
        motor_faults=[True, False, False, True, False, False],
    )
    active_result = active_fault_safety.check(active_fault)
    assert not active_result.ok
    assert "healthy motor fault flag" in active_result.reasons


def test_disabled_feedback_nan_is_ignored_but_shapes_and_healthy_values_fail() -> None:
    safety = SafetyFilter({"disabled_leg_indices": [3]})
    state = SafetyState(
        imu_age_s=0.01,
        joint_state_age_s=0.01,
        command=np.zeros(3),
        motor_temperatures_c=[30.0, 30.0, 30.0, float("nan"), 30.0, 30.0],
        motor_velocities_rad_s=[0.0, 0.0, 0.0, float("nan"), 0.0, 0.0],
        motor_faults=[False, False, False, True, False, False],
    )
    assert safety.check(state).ok
    state.motor_temperatures_c[0] = float("nan")
    assert "healthy motor temperature NaN/Inf" in safety.check(state).reasons
    state.motor_temperatures_c = [30.0] * 6
    state.motor_velocities_rad_s[0] = float("nan")
    assert "healthy motor velocity feedback NaN/Inf" in safety.check(state).reasons
    state.motor_velocities_rad_s = [0.0] * 6
    state.motor_faults = [False] * 5
    assert "motor fault feedback shape (5,) != (6,)" in safety.check(state).reasons


def test_policy_onnx_runner_accepts_expected_io(tmp_path) -> None:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    onnx_path = tmp_path / "policy.onnx"
    output = numpy_helper.from_array(np.zeros((1, C.ACTION_DIM), dtype=np.float32), name="zeros")
    graph = helper.make_graph(
        [helper.make_node("Constant", inputs=[], outputs=["action"], value=output)],
        "dummy_redrhex_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, C.OBS_DIM_SINGLE])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, C.ACTION_DIM])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, onnx_path)

    runner = PolicyONNXRunner(str(onnx_path))
    action = runner.run(np.zeros(C.OBS_DIM_SINGLE, dtype=np.float32))
    assert action.shape == (C.ACTION_DIM,)
    assert np.allclose(action, 0.0)
    benchmark = runner.benchmark(
        [np.zeros(C.OBS_DIM_SINGLE, dtype=np.float32)],
        warmup_runs=2,
        runs=100,
    )
    assert benchmark["samples"] == 100
    assert 0.0 <= benchmark["p50_ms"] <= benchmark["p99_ms"] <= benchmark["max_ms"]


def test_policy_onnx_runner_requires_semantic_contract_and_history_layout(tmp_path) -> None:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    onnx_path = tmp_path / "verified_policy.onnx"
    output = numpy_helper.from_array(
        np.zeros((1, C.ACTION_DIM), dtype=np.float32), name="verified_zeros"
    )
    graph = helper.make_graph(
        [helper.make_node("Constant", inputs=[], outputs=["action"], value=output)],
        "verified_redrhex_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, C.OBS_DIM_SINGLE])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, C.ACTION_DIM])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    for key, value in (
        (C.ONNX_OBSERVATION_CONTRACT_KEY, C.OBSERVATION_CONTRACT_ID),
        (C.ONNX_ACTION_CONTRACT_KEY, C.ACTION_DECODER_CONTRACT_ID),
        (C.ONNX_INPUT_LAYOUT_KEY, C.POLICY_INPUT_LAYOUT_SINGLE),
    ):
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.save(model, onnx_path)

    PolicyONNXRunner(
        str(onnx_path),
        expected_observation_contract=C.OBSERVATION_CONTRACT_ID,
        expected_action_contract=C.ACTION_DECODER_CONTRACT_ID,
        expected_input_layout="auto",
        require_contract_metadata=True,
    )

    with pytest.raises(ValueError, match="redrhex.action_contract"):
        PolicyONNXRunner(
            str(onnx_path),
            expected_observation_contract=C.OBSERVATION_CONTRACT_ID,
            expected_action_contract="wrong_decoder",
            expected_input_layout="auto",
            require_contract_metadata=True,
        )

    history_path = tmp_path / "verified_history_policy.onnx"
    history_output = numpy_helper.from_array(
        np.zeros((1, C.ACTION_DIM), dtype=np.float32), name="history_zeros"
    )
    history_graph = helper.make_graph(
        [
            helper.make_node(
                "Constant", inputs=[], outputs=["history_action"], value=history_output
            )
        ],
        "verified_redrhex_history_policy",
        [
            helper.make_tensor_value_info(
                "history_obs",
                TensorProto.FLOAT,
                [1, C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH],
            )
        ],
        [
            helper.make_tensor_value_info(
                "history_action", TensorProto.FLOAT, [1, C.ACTION_DIM]
            )
        ],
    )
    history_model = helper.make_model(
        history_graph, opset_imports=[helper.make_opsetid("", 17)]
    )
    history_model.ir_version = 8
    for key, value in (
        (C.ONNX_OBSERVATION_CONTRACT_KEY, C.OBSERVATION_CONTRACT_ID),
        (C.ONNX_ACTION_CONTRACT_KEY, C.ACTION_DECODER_CONTRACT_ID),
        (C.ONNX_INPUT_LAYOUT_KEY, C.POLICY_INPUT_LAYOUT_HISTORY),
    ):
        entry = history_model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.save(history_model, history_path)

    history_runner = PolicyONNXRunner(
        str(history_path),
        expected_observation_contract=C.OBSERVATION_CONTRACT_ID,
        expected_action_contract=C.ACTION_DECODER_CONTRACT_ID,
        expected_input_layout="auto",
        require_contract_metadata=True,
    )
    assert history_runner.obs_dim == C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH
    assert (
        history_runner.io_info.metadata[C.ONNX_INPUT_LAYOUT_KEY]
        == C.POLICY_INPUT_LAYOUT_HISTORY
    )

    for entry in history_model.metadata_props:
        if entry.key == C.ONNX_INPUT_LAYOUT_KEY:
            entry.value = C.POLICY_INPUT_LAYOUT_SINGLE
    bad_history_path = tmp_path / "bad_history_layout_policy.onnx"
    onnx.save(history_model, bad_history_path)
    with pytest.raises(ValueError, match="redrhex.policy_input_layout"):
        PolicyONNXRunner(
            str(bad_history_path),
            expected_observation_contract=C.OBSERVATION_CONTRACT_ID,
            expected_action_contract=C.ACTION_DECODER_CONTRACT_ID,
            expected_input_layout="auto",
            require_contract_metadata=True,
        )


@pytest.mark.parametrize(
    ("input_types", "output_types"),
    [
        (("float32", "float32"), ("float32",)),
        (("float32",), ("float32", "float32")),
        (("float64",), ("float32",)),
        (("float32",), ("float64",)),
    ],
    ids=["multiple-inputs", "multiple-outputs", "float64-input", "float64-output"],
)
def test_policy_onnx_runner_rejects_ambiguous_or_non_float32_io(
    tmp_path, input_types: tuple[str, ...], output_types: tuple[str, ...]
) -> None:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    tensor_type = {
        "float32": TensorProto.FLOAT,
        "float64": TensorProto.DOUBLE,
    }
    numpy_type = {
        "float32": np.float32,
        "float64": np.float64,
    }
    graph_inputs = [
        helper.make_tensor_value_info(
            f"obs_{index}", tensor_type[dtype], [1, C.OBS_DIM_SINGLE]
        )
        for index, dtype in enumerate(input_types)
    ]
    graph_outputs = []
    nodes = []
    for index, dtype in enumerate(output_types):
        output_name = f"action_{index}"
        constant = numpy_helper.from_array(
            np.zeros((1, C.ACTION_DIM), dtype=numpy_type[dtype]),
            name=f"zeros_{index}",
        )
        nodes.append(
            helper.make_node("Constant", inputs=[], outputs=[output_name], value=constant)
        )
        graph_outputs.append(
            helper.make_tensor_value_info(
                output_name, tensor_type[dtype], [1, C.ACTION_DIM]
            )
        )

    graph = helper.make_graph(
        nodes,
        "invalid_redrhex_policy_io",
        graph_inputs,
        graph_outputs,
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx_path = tmp_path / "invalid_policy_io.onnx"
    onnx.save(model, onnx_path)

    with pytest.raises(ValueError):
        PolicyONNXRunner(str(onnx_path))


def test_golden_bundle_cli_recomputes_and_hash_binds_every_artifact(tmp_path) -> None:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    torch = pytest.importorskip("torch")
    source_onnx = tmp_path / "source.onnx"
    training_sha = "a" * 40
    training_env_source = tmp_path / "redrhex_env.py"
    training_env_config_source = tmp_path / "redrhex_env_cfg.py"
    training_env_source.write_text("# captured training env\n", encoding="utf-8")
    training_env_config_source.write_text("# captured training config\n", encoding="utf-8")
    constant = numpy_helper.from_array(
        np.zeros((1, C.ACTION_DIM), dtype=np.float32), name="zero_action"
    )
    graph = helper.make_graph(
        [helper.make_node("Constant", inputs=[], outputs=["action"], value=constant)],
        "golden_bundle_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, C.OBS_DIM_SINGLE])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, C.ACTION_DIM])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    for key, value in (
        (C.ONNX_OBSERVATION_CONTRACT_KEY, C.OBSERVATION_CONTRACT_ID),
        (C.ONNX_ACTION_CONTRACT_KEY, C.ACTION_DECODER_CONTRACT_ID),
        (C.ONNX_INPUT_LAYOUT_KEY, C.POLICY_INPUT_LAYOUT_SINGLE),
        (C.ONNX_NORMALIZER_KEY, C.NORMALIZER_EMBEDDED),
        (C.ONNX_TRAINING_ACTION_CLIP_KEY, str(C.TRAINING_ACTION_CLIP)),
        (C.ONNX_TRAINING_GIT_SHA_KEY, training_sha),
        (C.ONNX_TRAINING_ENV_SOURCE_SHA256_KEY, sha256_file(training_env_source)),
        (
            C.ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY,
            sha256_file(training_env_config_source),
        ),
    ):
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.save(model, source_onnx)

    class _ZeroPolicy(torch.nn.Module):
        def forward(self, observation):
            return torch.zeros(
                (observation.shape[0], C.ACTION_DIM),
                dtype=observation.dtype,
                device=observation.device,
            )

    torchscript = tmp_path / "policy.pt"
    traced = torch.jit.trace(
        _ZeroPolicy(), torch.zeros((16, C.OBS_DIM_SINGLE), dtype=torch.float32)
    )
    traced.save(str(torchscript))

    controller_config = tmp_path / "controller.yaml"
    canonical_profile = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "redrhex_policy_encoder_only_rig.yaml"
    )
    bridge_config = tmp_path / "bridge.yaml"
    _, bridge_params = _active_bridge_profile()
    bridge_params["hardware"]["disabled_legs"] = ["L1"]
    bridge_config.write_text(
        yaml.safe_dump(
            {"redrhex_lowlevel_bridge": {"ros__parameters": bridge_params}}
        ),
        encoding="utf-8",
    )
    controller_document = yaml.safe_load(canonical_profile.read_text(encoding="utf-8"))
    controller_params = controller_document["redrhex_rl_controller"]["ros__parameters"]
    controller_params["hardware"]["disabled_legs"] = ["L1"]
    controller_params["policy"]["expected_bridge_config_sha256"] = bridge_config_sha256(
        bridge_params
    )
    controller_config.write_text(yaml.safe_dump(controller_document), encoding="utf-8")
    resolved_controller_params = controller_params_with_disabled_legs(
        load_controller_ros_params(controller_config), ["L1"]
    )
    decoder_config = decoder_config_from_ros_params(resolved_controller_params)
    decoder = ActionDecoder(decoder_config)
    sample_count = 256
    policy_input = np.zeros((sample_count, C.OBS_DIM_SINGLE), dtype=np.float32)
    base_lin_vel = np.zeros((sample_count, 3), dtype=np.float64)
    base_ang_vel = np.zeros((sample_count, 3), dtype=np.float64)
    main_pos = np.zeros((sample_count, 6), dtype=np.float64)
    main_pos[:, 3] = C.INIT_MAIN_DRIVE_POS[3]
    main_vel = np.zeros((sample_count, 6), dtype=np.float64)
    abad_pos = np.zeros((sample_count, 6), dtype=np.float64)
    abad_vel = np.zeros((sample_count, 6), dtype=np.float64)
    command = np.tile(np.asarray([0.22, 0.0, 0.0]), (sample_count, 1))
    gravity = np.tile(np.asarray(C.REFERENCE_PROJECTED_GRAVITY), (sample_count, 1))
    dt = np.full(sample_count, C.CONTROL_DT, dtype=np.float64)
    gait_phase = np.tile(
        np.linspace(0.0, 2.0 * np.pi, 128, endpoint=False, dtype=np.float64), 2
    )
    episode_start = np.zeros(sample_count, dtype=bool)
    episode_start[0] = True
    episode_start[128] = True
    actions = np.zeros((sample_count, C.ACTION_DIM), dtype=np.float32)
    single_observation = np.concatenate(
        [
            base_lin_vel,
            base_ang_vel,
            gravity,
            np.sin(main_pos),
            np.cos(main_pos),
            main_vel / C.BASE_GAIT_ANGULAR_VEL,
            abad_pos / C.ABAD_POS_SCALE,
            abad_vel,
            command,
            np.column_stack((np.sin(gait_phase), np.cos(gait_phase))),
            np.zeros((sample_count, C.ACTION_DIM), dtype=np.float64),
        ],
        axis=1,
    ).astype(np.float32)
    policy_input[:] = single_observation
    decoded_rows = []
    for index in range(sample_count):
        if episode_start[index]:
            decoder.reset()
        decoded_rows.append(
            decoder.decode(
                actions[index],
                main_pos[index],
                abad_pos[index],
                command[index],
                gravity[index],
                dt[index],
                gait_phase[index],
            )
        )
    vectors = tmp_path / "golden.npz"
    np.savez(
        vectors,
        policy_input=policy_input,
        single_observation=single_observation,
        base_lin_vel=base_lin_vel,
        base_ang_vel=base_ang_vel,
        main_drive_pos=main_pos,
        main_drive_vel=main_vel,
        abad_pos=abad_pos,
        abad_vel=abad_vel,
        command=command,
        projected_gravity=gravity,
        dt=dt,
        gait_phase=gait_phase,
        episode_start=episode_start,
        expected_policy_action=actions,
        expected_training_clipped_action=actions,
        expected_safe_action=np.stack([row.safe_action for row in decoded_rows]),
        expected_target_main_drive_velocity=np.stack(
            [row.target_main_drive_velocity for row in decoded_rows]
        ),
        expected_target_abad_position=np.stack(
            [row.target_abad_position for row in decoded_rows]
        ),
        expected_target_position_rad=np.stack(
            [row.target_position_rad for row in decoded_rows]
        ),
        expected_target_velocity_rad_s=np.stack(
            [row.target_velocity_rad_s for row in decoded_rows]
        ),
        expected_kp=np.stack([row.kp for row in decoded_rows]),
        expected_kd=np.stack([row.kd for row in decoded_rows]),
        expected_effort_limit_nm=np.stack(
            [row.effort_limit_nm for row in decoded_rows]
        ),
        expected_enable=np.asarray([row.enable for row in decoded_rows]),
        expected_mode=np.asarray([row.mode for row in decoded_rows], dtype=np.uint8),
        joint_names=np.asarray(decoded_rows[0].joint_names),
        producer=np.asarray(GOLDEN_PRODUCER),
        training_git_sha=np.asarray(training_sha),
        training_env_source_sha256=np.asarray(sha256_file(training_env_source)),
        training_env_config_source_sha256=np.asarray(
            sha256_file(training_env_config_source)
        ),
        training_play_source_sha256=np.asarray(""),
        deployment_disabled_legs_csv=np.asarray("L1"),
        deployment_command_profile=np.asarray("fixed_forward"),
        deployment_fixed_forward_vx=np.asarray(0.22),
    )

    package_root = Path(__file__).resolve().parents[1]
    compare_script = package_root / "scripts" / "compare_onnx_with_torch.py"
    package_script = package_root / "scripts" / "package_verified_policy.py"
    report = tmp_path / "golden.json"
    packaged = tmp_path / "packaged.onnx"
    environment = dict(os.environ)
    source_root = str(package_root)
    environment["PYTHONPATH"] = (
        source_root
        if not environment.get("PYTHONPATH")
        else source_root + os.pathsep + environment["PYTHONPATH"]
    )
    subprocess.run(
        [
            sys.executable,
            str(compare_script),
            "--onnx",
            str(source_onnx),
            "--torchscript",
            str(torchscript),
            "--golden-vectors",
            str(vectors),
            "--controller-config",
            str(controller_config),
            "--bridge-config",
            str(bridge_config),
            "--disabled-legs",
            "L1",
            "--training-git-sha",
            training_sha,
            "--training-env-source",
            str(training_env_source),
            "--training-env-config-source",
            str(training_env_config_source),
            "--report-json",
            str(report),
        ],
        check=True,
        env=environment,
        capture_output=True,
        text=True,
    )
    report_data = json.loads(report.read_text(encoding="utf-8"))
    assert report_data["schema"] == GOLDEN_SCHEMA
    assert report_data["passed"] is True
    assert report_data["artifacts"] == {
        "onnx_sha256": sha256_file(source_onnx),
        "torchscript_sha256": sha256_file(torchscript),
        "vectors_sha256": sha256_file(vectors),
        "controller_yaml_sha256": sha256_file(controller_config),
        "controller_config_sha256": controller_config_sha256(
            resolved_controller_params
        ),
        "decoder_config_sha256": decoder_config_sha256(decoder_config),
        "decoder_source_sha256": decoder_source_sha256(),
        "observation_source_sha256": observation_source_sha256(),
        "deployment_source_sha256": deployment_source_sha256(),
        "bridge_yaml_sha256": sha256_file(bridge_config),
        "bridge_config_sha256": bridge_config_sha256(bridge_params),
        "training_git_sha": training_sha,
        "training_env_source_sha256": sha256_file(training_env_source),
        "training_env_config_source_sha256": sha256_file(
            training_env_config_source
        ),
    }
    assert report_data["torch_onnx"]["passed"] is True
    assert report_data["policy_expected"]["passed"] is True
    assert report_data["ros_decoder"]["passed"] is True

    subprocess.run(
        [
            sys.executable,
            str(package_script),
            str(source_onnx),
            str(packaged),
            "--torchscript",
            str(torchscript),
            "--golden-vectors",
            str(vectors),
            "--controller-config",
            str(controller_config),
            "--bridge-config",
            str(bridge_config),
            "--disabled-legs",
            "L1",
            "--training-git-sha",
            training_sha,
            "--training-env-source",
            str(training_env_source),
            "--training-env-config-source",
            str(training_env_config_source),
            "--golden-report",
            str(report),
        ],
        check=True,
        env=environment,
        capture_output=True,
        text=True,
    )
    packaged_model = onnx.load(packaged)
    packaged_metadata = {
        item.key: item.value for item in packaged_model.metadata_props
    }
    assert packaged_metadata[C.ONNX_GOLDEN_SCHEMA_KEY] == GOLDEN_SCHEMA
    assert (
        packaged_metadata[C.ONNX_DECODER_CONFIG_SHA256_KEY]
        == decoder_config_sha256(decoder_config)
    )
    assert (
        packaged_metadata[C.ONNX_DECODER_SOURCE_SHA256_KEY]
        == decoder_source_sha256()
    )
    assert packaged_metadata["redrhex.source_onnx_sha256"] == sha256_file(source_onnx)
    assert packaged_metadata["redrhex.torchscript_sha256"] == sha256_file(torchscript)
    assert packaged_metadata["redrhex.golden_vectors_sha256"] == sha256_file(vectors)
    assert packaged_metadata[C.ONNX_CONTROLLER_CONFIG_SHA256_KEY] == controller_config_sha256(
        resolved_controller_params
    )
    assert packaged_metadata["redrhex.controller_yaml_sha256"] == sha256_file(
        controller_config
    )
    assert packaged_metadata["redrhex.golden_report_sha256"] == sha256_file(report)

    # Changing any bound artifact after report generation must be detected.
    with vectors.open("ab") as stream:
        stream.write(b"tamper")
    rejected = subprocess.run(
        [
            sys.executable,
            str(package_script),
            str(source_onnx),
            str(tmp_path / "must_not_exist.onnx"),
            "--torchscript",
            str(torchscript),
            "--golden-vectors",
            str(vectors),
            "--controller-config",
            str(controller_config),
            "--bridge-config",
            str(bridge_config),
            "--disabled-legs",
            "L1",
            "--training-git-sha",
            training_sha,
            "--training-env-source",
            str(training_env_source),
            "--training-env-config-source",
            str(training_env_config_source),
            "--golden-report",
            str(report),
        ],
        check=False,
        env=environment,
        capture_output=True,
        text=True,
    )
    assert rejected.returncode != 0
    assert not (tmp_path / "must_not_exist.onnx").exists()


def test_controller_and_decoder_hash_bind_mask_but_ignore_artifact_pointer() -> None:
    base = {
        "hardware": {"disabled_legs": [], "max_disabled_legs": 1},
        "policy": {"onnx_path": "/old.onnx", "expected_sha256": "a" * 64},
        "safety": {"action_clip": 0.35},
    }
    moved = json.loads(json.dumps(base))
    moved["policy"]["onnx_path"] = "/new.onnx"
    moved["policy"]["expected_sha256"] = "b" * 64
    assert controller_config_sha256(base) == controller_config_sha256(moved)

    l1 = controller_params_with_disabled_legs(base, ["L1"])
    assert controller_config_sha256(l1) != controller_config_sha256(base)
    assert decoder_config_sha256(decoder_config_from_ros_params(l1)) != decoder_config_sha256(
        decoder_config_from_ros_params(base)
    )

    lowercase = {
        "hardware": {"disabled_legs": ["l1"], "max_disabled_legs": 1}
    }
    assert controller_params_with_disabled_legs(lowercase, None)["hardware"][
        "disabled_legs"
    ] == ["L1"]


def test_bridge_hash_excludes_only_operational_launch_latches() -> None:
    _, base = _active_bridge_profile("full_feedback_rig")
    armed = json.loads(json.dumps(base))
    armed["rinbo"]["allow_enable"] = True
    armed["rinbo"]["canonical_policy_launch_authorized"] = True
    assert bridge_config_sha256(base) == bridge_config_sha256(armed)
    changed_mapping = json.loads(json.dumps(base))
    changed_mapping["rinbo"]["main_encoder_zero_counts_rinbo_order"][0] = 1.0
    assert bridge_config_sha256(base) != bridge_config_sha256(changed_mapping)


def test_active_bridge_hash_rejects_sparse_runtime_default_profile() -> None:
    _, params = _active_bridge_profile()
    del params["rinbo"]["expected_upstream_command_publisher_node"]

    with pytest.raises(
        ValueError,
        match="missing: rinbo.expected_upstream_command_publisher_node",
    ):
        require_complete_bridge_params(params)
    with pytest.raises(ValueError, match="complete deployment profile"):
        bridge_config_sha256(params)


def test_active_bridge_hash_rejects_every_single_required_field_deletion() -> None:
    _, complete = _active_bridge_profile("full_feedback_rig")
    for dotted_name in sorted(required_active_bridge_parameter_names()):
        sparse = json.loads(json.dumps(complete))
        keys = dotted_name.split(".")
        parent = sparse
        for key in keys[:-1]:
            parent = parent[key]
        del parent[keys[-1]]

        with pytest.raises(ValueError, match="complete deployment profile"):
            bridge_config_sha256(sparse)


def test_active_bridge_hash_schema_tracks_lowlevel_rinbo_declarations() -> None:
    source = (
        Path(__file__).resolve().parents[2]
        / "redrhex_lowlevel_bridge"
        / "redrhex_lowlevel_bridge"
        / "lowlevel_bridge_node.py"
    )
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    declared: set[str] = set()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
        ):
            declared.add(node.args[0].value)
    expected = {
        "backend",
        "feedback_rate_hz",
        "hardware.disabled_legs",
        "hardware.max_disabled_legs",
        *(name for name in declared if name.startswith("rinbo.")),
    }
    expected.remove("rinbo.canonical_policy_launch_authorized")

    assert required_active_bridge_parameter_names() == expected


@pytest.mark.parametrize("profile", ["encoder_only_rig", "full_feedback_rig"])
def test_active_bridge_profiles_are_complete_and_match_controller_pin(
    profile: str,
) -> None:
    _, bridge_params = _active_bridge_profile(profile)
    require_complete_bridge_params(bridge_params)
    actual = bridge_config_sha256(bridge_params)
    controller_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / f"redrhex_policy_{profile}.yaml"
    )
    controller_params = load_controller_ros_params(controller_path)
    assert controller_params["policy"]["expected_bridge_config_sha256"] == actual


def test_controller_config_hash_binds_command_profile_and_fixed_speed() -> None:
    external = {
        "commands": {
            "profile": "external_cmd_vel",
            "fixed_forward_vx": 0.11,
        }
    }
    fixed = json.loads(json.dumps(external))
    fixed["commands"]["profile"] = "fixed_forward"
    faster = json.loads(json.dumps(fixed))
    faster["commands"]["fixed_forward_vx"] = 0.12

    assert controller_config_sha256(external) != controller_config_sha256(fixed)
    assert controller_config_sha256(fixed) != controller_config_sha256(faster)


def test_controller_hash_binds_every_semantic_launch_override() -> None:
    profile = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "redrhex_policy_encoder_only_rig.yaml"
    )
    base = load_controller_ros_params(profile)
    base_hash = controller_config_sha256(base)
    semantic_overrides = {
        ("hardware", "disabled_legs"): ["L1"],
        ("hardware", "max_disabled_legs"): 2,
        ("policy", "policy_hz"): 124.0,
        ("policy", "use_cuda"): True,
        ("policy", "use_tensorrt"): True,
        ("state_machine", "enable_policy_on_start"): True,
        ("state_machine", "enable_motor_output_on_start"): True,
        ("observation", "sensor_profile"): "full_state",
        ("observation", "encoder_only_rig_acknowledged"): False,
        ("observation", "base_lin_vel_source"): "odom",
        ("observation", "abad_feedback_source"): "joint_states",
        ("safety", "require_lowlevel_heartbeat"): False,
        ("safety", "require_motor_feedback"): False,
    }
    for path, value in semantic_overrides.items():
        changed = json.loads(json.dumps(base))
        changed[path[0]][path[1]] = value
        assert controller_config_sha256(changed) != base_hash, ".".join(path)

    # Relocating the exact hash-pinned artifact is operational, not semantic.
    moved = json.loads(json.dumps(base))
    moved["policy"]["onnx_path"] = "/different/location/policy.onnx"
    assert controller_config_sha256(moved) == base_hash


def test_artifact_tooling_rejects_sparse_controller_profile() -> None:
    with pytest.raises(ValueError, match="complete deployment profile"):
        require_complete_controller_params(
            {"safety": {"action_clip": 0.35}, "hardware": {"disabled_legs": []}}
        )


def test_isaaclab_golden_recorder_rejects_incomplete_live_step(tmp_path: Path) -> None:
    source_onnx = tmp_path / "policy.onnx"
    env_source = tmp_path / "redrhex_env.py"
    cfg_source = tmp_path / "redrhex_env_cfg.py"
    source_onnx.write_bytes(b"exporter placeholder for field-validation test")
    env_source.write_text("# env\n", encoding="utf-8")
    cfg_source.write_text("# cfg\n", encoding="utf-8")
    recorder = IsaacLabGoldenRecorder(
        output_npz=tmp_path / "golden.npz",
        source_onnx=source_onnx,
        joint_names=C.MAIN_DRIVE_JOINT_NAMES + C.ABAD_JOINT_NAMES,
        training_git_sha="a" * 40,
        training_env_source=env_source,
        training_env_config_source=cfg_source,
        normalizer_embedded=True,
        policy_input_dim=C.OBS_DIM_SINGLE,
        disabled_legs=["L1"],
        command_profile="fixed_forward",
        fixed_forward_vx=0.22,
    )
    with pytest.raises(ValueError, match="golden sample fields missing"):
        recorder.record_step(
            policy_input=np.zeros(C.OBS_DIM_SINGLE, dtype=np.float32)
        )
    assert not (tmp_path / "golden.npz").exists()


def test_golden_recorder_metadata_conflict_preserves_source_and_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import onnx
    from onnx import helper

    source_onnx = tmp_path / "policy.onnx"
    model = helper.make_model(helper.make_graph([], "metadata_conflict", [], []))
    entry = model.metadata_props.add()
    entry.key = C.ONNX_OBSERVATION_CONTRACT_KEY
    entry.value = "conflicting_contract"
    onnx.save(model, source_onnx)
    original_source = source_onnx.read_bytes()

    env_source = tmp_path / "redrhex_env.py"
    cfg_source = tmp_path / "redrhex_env_cfg.py"
    env_source.write_text("# env\n", encoding="utf-8")
    cfg_source.write_text("# cfg\n", encoding="utf-8")
    output_npz = tmp_path / "golden.npz"
    recorder = IsaacLabGoldenRecorder(
        output_npz=output_npz,
        source_onnx=source_onnx,
        joint_names=C.MAIN_DRIVE_JOINT_NAMES + C.ABAD_JOINT_NAMES,
        training_git_sha="a" * 40,
        training_env_source=env_source,
        training_env_config_source=cfg_source,
        normalizer_embedded=True,
        policy_input_dim=C.OBS_DIM_SINGLE,
        disabled_legs=["L1"],
        command_profile="fixed_forward",
        fixed_forward_vx=0.22,
    )
    recorder.record_step(
        **{field: np.asarray(0.0) for field in recorder._rows}
    )
    monkeypatch.setattr(
        "redrhex_rl_controller.golden_recorder.load_golden_vectors",
        lambda _path: {},
    )

    with pytest.raises(ValueError, match="conflicting_contract"):
        recorder.finalize()

    assert source_onnx.read_bytes() == original_source
    assert not output_npz.exists()
    assert not list(tmp_path.glob(".*.tmp.onnx"))
    assert not list(tmp_path.glob(".*.tmp.npz"))


def test_preflight_rejects_controller_bridge_disabled_leg_mismatch() -> None:
    controller = {
        "hardware": {"disabled_legs": ["L1"], "max_disabled_legs": 1},
        "safety": {"main_drive_vel_limit_rad_s": 1.0, "abad_pos_limit_rad": 0.18},
    }
    bridge = {
        "hardware": {"disabled_legs": [], "max_disabled_legs": 1},
        "backend": "biorola_ros",
        "rinbo": {},
    }
    result = {"checks": [], "warnings": []}

    _validate_bridge_config(controller, bridge, result)

    mask_check = next(
        check
        for check in result["checks"]
        if check["name"] == "controller_bridge_disabled_leg_contract"
    )
    assert mask_check["ok"] is False
    assert mask_check["controller_disabled_legs"] == ["L1"]
    assert mask_check["bridge_disabled_legs"] == []


def test_preflight_reports_incomplete_active_bridge_profile(tmp_path: Path) -> None:
    _, params = _active_bridge_profile()
    del params["rinbo"]["command_topic"]
    path = tmp_path / "sparse_bridge.yaml"
    path.write_text(
        yaml.safe_dump(
            {"redrhex_lowlevel_bridge": {"ros__parameters": params}}
        ),
        encoding="utf-8",
    )
    result = {"checks": [], "warnings": []}

    loaded = _load_bridge_params(str(path), result, require_complete=True)

    assert loaded == params
    completeness = next(
        check
        for check in result["checks"]
        if check["name"] == "complete_active_bridge_deployment_profile"
    )
    assert completeness["ok"] is False
    assert "rinbo.command_topic" in completeness["error"]
