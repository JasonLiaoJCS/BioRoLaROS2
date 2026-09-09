from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import Imu

from redrhex_rl_controller import redrhex_contract as C
from redrhex_rl_controller.observation_builder import ObservationBuilder


SQRT_HALF = math.sqrt(0.5)


def _imu(
    quaternion_xyzw: list[float],
    angular_velocity: list[float] | None = None,
    *,
    frame_id: str = "",
    stamp_s: float = 0.0,
) -> SimpleNamespace:
    angular_velocity = angular_velocity or [0.0, 0.0, 0.0]
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(
                sec=int(stamp_s),
                nanosec=int(round((stamp_s - int(stamp_s)) * 1.0e9)),
            ),
        ),
        orientation=SimpleNamespace(
            x=quaternion_xyzw[0],
            y=quaternion_xyzw[1],
            z=quaternion_xyzw[2],
            w=quaternion_xyzw[3],
        ),
        angular_velocity=SimpleNamespace(
            x=angular_velocity[0],
            y=angular_velocity[1],
            z=angular_velocity[2],
        ),
    )


def _main_joint_state() -> SimpleNamespace:
    return SimpleNamespace(
        name=list(C.MAIN_DRIVE_JOINT_NAMES),
        position=[0.0] * 6,
        velocity=[0.0] * 6,
    )


def _full_feedback_config(**overrides: object) -> dict[str, object]:
    config: dict[str, object] = {
        "sensor_profile": "full_feedback_rig",
        "full_feedback_rig_acknowledged": True,
        "imu_alignment_calibrated": True,
        "base_lin_vel_source": "zero",
        "abad_feedback_source": "joint_states",
        "require_imu_source_stamp": True,
    }
    config.update(overrides)
    return config


def test_training_upright_is_zero_relative_tilt_and_preserves_projected_gravity() -> None:
    builder = ObservationBuilder()
    builder.update_imu(_imu([SQRT_HALF, 0.0, 0.0, SQRT_HALF]), now_s=1.0)
    builder.update_joint_state(_main_joint_state(), now_s=1.0)

    roll, pitch, yaw = builder.get_roll_pitch_yaw()
    np.testing.assert_allclose([roll, pitch, yaw], np.zeros(3), atol=1.0e-7)

    observation = builder.build_single(now_s=1.01, update_phase=False)
    np.testing.assert_allclose(
        observation[6:9], C.REFERENCE_PROJECTED_GRAVITY, atol=1.0e-6
    )


def test_mount_rotation_composes_sensor_orientation_into_policy_orientation() -> None:
    # Level sensor orientation is identity, while the policy frame is mounted
    # +90 degrees about X inside the sensor frame.
    builder = ObservationBuilder(
        {
            "imu_policy_to_sensor_quaternion_xyzw": [
                SQRT_HALF,
                0.0,
                0.0,
                SQRT_HALF,
            ]
        }
    )
    builder.update_imu(_imu([0.0, 0.0, 0.0, 1.0]), now_s=1.0)

    np.testing.assert_allclose(
        builder.imu_quat_xyzw,
        [SQRT_HALF, 0.0, 0.0, SQRT_HALF],
        atol=1.0e-7,
    )
    np.testing.assert_allclose(builder.get_roll_pitch_yaw(), np.zeros(3), atol=1.0e-7)


def test_mount_rotation_maps_sensor_angular_velocity_to_policy_frame() -> None:
    # q_S_P=+90 deg about Z; inverse rotation maps sensor +X to policy -Y.
    builder = ObservationBuilder(
        {
            "imu_policy_to_sensor_quaternion_xyzw": [
                0.0,
                0.0,
                SQRT_HALF,
                SQRT_HALF,
            ]
        }
    )
    builder.update_imu(
        _imu([0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        now_s=1.0,
    )

    np.testing.assert_allclose(builder.imu_ang_vel, [0.0, -1.0, 0.0], atol=1.0e-7)


def test_tilt_guard_is_invariant_to_world_yaw_in_training_frame() -> None:
    builder = ObservationBuilder()
    yaw = 0.8
    # q_W_P = q_W_yaw * q_W_P_upright.
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
    q_upright = np.array([SQRT_HALF, 0.0, 0.0, SQRT_HALF])
    x1, y1, z1, w1 = q_yaw
    x2, y2, z2, w2 = q_upright
    q = [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]
    builder.update_imu(_imu(q), now_s=1.0)

    np.testing.assert_allclose(
        builder.get_relative_tilt_roll_pitch(), np.zeros(2), atol=1.0e-7
    )


@pytest.mark.parametrize("axis", ["x", "z"])
def test_tilt_guard_detects_both_axes_around_training_gravity(axis: str) -> None:
    builder = ObservationBuilder()
    angle = 0.3
    half_sin = math.sin(angle / 2.0)
    local_tilt = (
        np.array([half_sin, 0.0, 0.0, math.cos(angle / 2.0)])
        if axis == "x"
        else np.array([0.0, 0.0, half_sin, math.cos(angle / 2.0)])
    )
    q_upright = np.array([SQRT_HALF, 0.0, 0.0, SQRT_HALF])
    x1, y1, z1, w1 = q_upright
    x2, y2, z2, w2 = local_tilt
    q = [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]
    builder.update_imu(_imu(q), now_s=1.0)

    tilt_x, tilt_z = builder.get_relative_tilt_roll_pitch()
    measured = tilt_x if axis == "x" else tilt_z
    assert abs(measured) == pytest.approx(angle, abs=1.0e-7)


def test_expected_imu_frame_mismatch_is_reported_by_status() -> None:
    builder = ObservationBuilder({"expected_imu_frame_id": "imu_link"})
    builder.update_imu(
        _imu([SQRT_HALF, 0.0, 0.0, SQRT_HALF], frame_id="wrong_link"),
        now_s=1.0,
    )
    builder.update_joint_state(_main_joint_state(), now_s=1.0)

    status = builder.status(now_s=1.01, sensor_timeout_s=0.1, cmd_timeout_s=0.25)
    assert not status.ok
    assert status.reasons == [
        "IMU frame_id mismatch: expected 'imu_link', got 'wrong_link'"
    ]


def test_imu_epoch_source_stamp_uses_ros_clock_and_must_be_monotonic() -> None:
    epoch = 1_800_000_000.0
    builder = ObservationBuilder(
        {
            "require_imu_source_stamp": True,
            "max_imu_source_age_s": 0.10,
            "max_imu_future_skew_s": 0.02,
        }
    )
    builder.update_imu(
        _imu([SQRT_HALF, 0.0, 0.0, SQRT_HALF], stamp_s=epoch - 0.01),
        now_s=epoch,
        receipt_monotonic_s=123.0,
    )
    assert builder.imu_valid_reason is None
    assert builder.imu_receipt_monotonic_s == 123.0
    builder.update_imu(
        _imu([SQRT_HALF, 0.0, 0.0, SQRT_HALF], stamp_s=epoch - 0.01),
        now_s=epoch + 0.01,
    )
    assert builder.imu_valid_reason == "IMU source header stamp is not strictly monotonic"


@pytest.mark.parametrize(
    ("stamp_offset", "reason"),
    [
        (-0.11, "IMU source header stamp is stale"),
        (0.03, "IMU source header stamp is too far in the future"),
    ],
)
def test_imu_source_stamp_age_and_future_skew_fail_closed(
    stamp_offset: float, reason: str
) -> None:
    epoch = 1_800_000_000.0
    builder = ObservationBuilder({"require_imu_source_stamp": True})
    builder.update_imu(
        _imu(
            [SQRT_HALF, 0.0, 0.0, SQRT_HALF],
            stamp_s=epoch + stamp_offset,
        ),
        now_s=epoch,
    )
    assert builder.imu_valid_reason == reason


@pytest.mark.parametrize(
    ("field", "reason"),
    [
        ("orientation_covariance", "IMU orientation is marked unavailable"),
        (
            "angular_velocity_covariance",
            "IMU angular velocity is marked unavailable",
        ),
    ],
)
def test_imu_covariance_unavailable_markers_fail_closed(
    field: str, reason: str
) -> None:
    builder = ObservationBuilder()
    msg = _imu([SQRT_HALF, 0.0, 0.0, SQRT_HALF])
    setattr(msg, field, [-1.0] + [0.0] * 8)
    builder.update_imu(msg, now_s=1.0)
    builder.update_joint_state(_main_joint_state(), now_s=1.0)

    status = builder.status(now_s=1.01, sensor_timeout_s=0.1, cmd_timeout_s=0.25)
    assert not status.ok
    assert status.reasons == [reason]


def test_real_ros_imu_fixed_covariance_arrays_do_not_use_ambiguous_truth() -> None:
    msg = Imu()
    msg.orientation.x = SQRT_HALF
    msg.orientation.w = SQRT_HALF
    msg.orientation_covariance = [0.0] * 9
    msg.angular_velocity_covariance = [0.0] * 9
    # Humble's generated fixed-array fields expose numpy arrays; this is the
    # concrete message type used by subscriptions and fake_sensor_node.
    assert isinstance(msg.orientation_covariance, np.ndarray)

    builder = ObservationBuilder()
    builder.update_imu(msg, now_s=1.0, receipt_monotonic_s=10.0)

    assert builder.imu_valid_reason is None
    np.testing.assert_allclose(
        builder.imu_quat_xyzw,
        [SQRT_HALF, 0.0, 0.0, SQRT_HALF],
        atol=1.0e-7,
    )


def test_full_feedback_profile_requires_explicit_ack_and_calibrated_real_inputs() -> None:
    with pytest.raises(ValueError, match="full_feedback_rig_acknowledged"):
        ObservationBuilder(_full_feedback_config(full_feedback_rig_acknowledged=False))
    with pytest.raises(ValueError, match="imu_alignment_calibrated"):
        ObservationBuilder(_full_feedback_config(imu_alignment_calibrated=False))
    with pytest.raises(ValueError, match="base_lin_vel_source='zero'"):
        ObservationBuilder(_full_feedback_config(base_lin_vel_source="odom"))
    with pytest.raises(ValueError, match="abad_feedback_source='joint_states'"):
        ObservationBuilder(_full_feedback_config(abad_feedback_source="commanded"))
    with pytest.raises(ValueError, match=r"projected gravity \[0,-1,0\]"):
        ObservationBuilder(
            _full_feedback_config(
                imu_upright_quaternion_xyzw=[0.0, 0.0, 0.0, 1.0]
            )
        )

    builder = ObservationBuilder(_full_feedback_config())
    assert builder.requires_imu
    assert builder.is_full_feedback_rig
    assert not builder.is_encoder_only_rig
    assert builder.required_joint_names == (
        list(C.MAIN_DRIVE_JOINT_NAMES) + list(C.ABAD_JOINT_NAMES)
    )


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("imu_policy_to_sensor_quaternion_xyzw", [0.0, 0.0, 1.0], "four values"),
        ("imu_policy_to_sensor_quaternion_xyzw", [0.0, 0.0, 0.0, 0.0], "norm is zero"),
        ("imu_upright_quaternion_xyzw", [0.0, 0.0, 0.0, 2.0], "near-unit"),
        ("imu_upright_quaternion_xyzw", [0.0, 0.0, math.nan, 1.0], "finite"),
    ],
)
def test_imu_configuration_quaternions_are_validated(
    key: str, value: list[float], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        ObservationBuilder({key: value})


def test_near_unit_configuration_quaternion_is_normalized() -> None:
    builder = ObservationBuilder(
        {"imu_policy_to_sensor_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0005]}
    )
    np.testing.assert_allclose(
        builder.imu_policy_to_sensor_quaternion_xyzw,
        [0.0, 0.0, 0.0, 1.0],
        atol=1.0e-12,
    )
