"""Build real-robot observations matching RedRhex IsaacLab policy inputs."""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from . import redrhex_contract as C


@dataclass
class ObservationStatus:
    ok: bool
    reasons: list[str] = field(default_factory=list)


class SensorV2HistoryBuffer:
    """60 Hz, oldest-to-newest history with zero-order hold between samples."""

    def __init__(self, history_length: int = 60, frame_dim: int = 36, rate_hz: float = 60.0) -> None:
        if history_length <= 0 or frame_dim <= 0 or not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("sensor-v2 history dimensions and rate must be positive")
        self.history_length = int(history_length)
        self.frame_dim = int(frame_dim)
        self.period_s = 1.0 / float(rate_hz)
        self._frames: deque[np.ndarray] = deque(maxlen=self.history_length)
        self._next_sample_s: float | None = None

    def reset(self) -> None:
        self._frames.clear()
        self._next_sample_s = None

    def update(self, frame: np.ndarray, now_s: float) -> np.ndarray:
        value = np.asarray(frame, dtype=np.float32)
        if value.shape != (self.frame_dim,) or not np.isfinite(value).all():
            raise ValueError(f"sensor-v2 frame must be finite shape ({self.frame_dim},), got {value.shape}")
        if not math.isfinite(now_s):
            raise ValueError("sensor-v2 sample time must be finite")
        if not self._frames:
            self._frames.extend(value.copy() for _ in range(self.history_length))
            self._next_sample_s = now_s + self.period_s
        elif self._next_sample_s is not None and now_s + 1.0e-9 >= self._next_sample_s:
            missed = int((now_s - self._next_sample_s) // self.period_s) + 1
            for _ in range(min(missed, self.history_length)):
                self._frames.append(value.copy())
            self._next_sample_s += missed * self.period_s
        return np.stack(self._frames, axis=0).astype(np.float32)


def _stamp_to_float(msg: object | None, fallback: float | None = None) -> float:
    if msg is None:
        return time.monotonic() if fallback is None else fallback
    sec = getattr(getattr(msg, "header", None), "stamp", None)
    if sec is None:
        return time.monotonic() if fallback is None else fallback
    stamp_s = float(getattr(sec, "sec", 0)) + 1.0e-9 * float(getattr(sec, "nanosec", 0))
    return stamp_s if stamp_s > 0.0 else (time.monotonic() if fallback is None else fallback)


def _source_stamp_to_float(msg: object | None) -> float | None:
    """Return a real message source stamp; never disguise zero as receipt time."""

    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    stamp_s = float(getattr(stamp, "sec", 0)) + 1.0e-9 * float(
        getattr(stamp, "nanosec", 0)
    )
    return stamp_s if math.isfinite(stamp_s) and stamp_s > 0.0 else None


def _normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1.0e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / norm


def _validated_unit_quat_xyzw(value: object, name: str) -> np.ndarray:
    """Validate and normalize a configuration quaternion in ROS xyzw order."""
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError(f"{name} must contain four values in xyzw order")
    if not np.isfinite(q).all():
        raise ValueError(f"{name} must contain only finite values")
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-9:
        raise ValueError(f"{name} quaternion norm is zero")
    # Normalizing small representation/rounding error is useful, but accepting
    # an arbitrary scale would hide a malformed calibration value.
    if abs(norm - 1.0) > 1.0e-3:
        raise ValueError(f"{name} must be a near-unit quaternion, got norm={norm:.9g}")
    return q / norm


def _quat_multiply_xyzw(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product for ROS xyzw quaternions (left * right)."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def _quat_inverse_xyzw(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quat_xyzw(q_xyzw)
    return np.array([-x, -y, -z, w], dtype=np.float64)


def _quat_inverse_rotate_xyzw(q_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quat_xyzw(q_xyzw)
    # Rotation matrix body->world for ROS xyzw quaternion, then transpose.
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)
    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return np.array(
        [
            r00 * v[0] + r10 * v[1] + r20 * v[2],
            r01 * v[0] + r11 * v[1] + r21 * v[2],
            r02 * v[0] + r12 * v[1] + r22 * v[2],
        ],
        dtype=np.float64,
    )


def _quat_to_roll_pitch_yaw(q_xyzw: np.ndarray) -> tuple[float, float, float]:
    x, y, z, w = _normalize_quat_xyzw(q_xyzw)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _wrap_angle_diff(new: float, old: float) -> float:
    return math.atan2(math.sin(new - old), math.cos(new - old))


class ObservationBuilder:
    """Stateful observation builder.

    Single-frame observation is the 56-D vector from RedrhexEnv._get_observations().
    If the exported ONNX expects 280-D input, build_policy_input() returns
    [current_obs, previous_obs_1, ..., previous_obs_4], matching the RSL-RL
    "policy"+"history" group order used by this repo's newer configs.
    """

    def __init__(self, config: dict | None = None) -> None:
        self.cfg = config or {}
        self.sensor_profile = str(self.cfg.get("sensor_profile", "full_state"))
        self.encoder_only_rig_acknowledged = bool(
            self.cfg.get("encoder_only_rig_acknowledged", False)
        )
        self.full_feedback_rig_acknowledged = bool(
            self.cfg.get("full_feedback_rig_acknowledged", False)
        )
        self.imu_alignment_calibrated = bool(
            self.cfg.get("imu_alignment_calibrated", False)
        )
        self.expected_imu_frame_id = str(
            self.cfg.get("expected_imu_frame_id", "")
        ).strip()
        self.imu_policy_to_sensor_quaternion_xyzw = _validated_unit_quat_xyzw(
            self.cfg.get(
                "imu_policy_to_sensor_quaternion_xyzw",
                [0.0, 0.0, 0.0, 1.0],
            ),
            "imu_policy_to_sensor_quaternion_xyzw",
        )
        sqrt_half = math.sqrt(0.5)
        self.imu_upright_quaternion_xyzw = _validated_unit_quat_xyzw(
            self.cfg.get(
                "imu_upright_quaternion_xyzw",
                [sqrt_half, 0.0, 0.0, sqrt_half],
            ),
            "imu_upright_quaternion_xyzw",
        )
        self.rig_projected_gravity = np.asarray(
            self.cfg.get("rig_projected_gravity", C.REFERENCE_PROJECTED_GRAVITY),
            dtype=np.float64,
        )
        self.expected_obs_dim = int(self.cfg.get("expected_obs_dim", C.OBS_DIM_SINGLE))
        self.policy_input_dim = int(self.cfg.get("policy_input_dim", self.expected_obs_dim))
        self.history_length = int(self.cfg.get("policy_history_length", C.POLICY_HISTORY_LENGTH))
        self.base_gait_angular_vel = float(self.cfg.get("base_gait_angular_vel", C.BASE_GAIT_ANGULAR_VEL))
        self.abad_pos_scale = float(self.cfg.get("abad_pos_scale", C.ABAD_POS_SCALE))
        self.base_gait_frequency_hz = float(self.cfg.get("base_gait_frequency_hz", C.BASE_GAIT_FREQUENCY_HZ))
        self.base_lin_vel_source = str(self.cfg.get("base_lin_vel_source", "zero"))
        self.odom_twist_in_body_frame = bool(self.cfg.get("odom_twist_in_body_frame", True))
        self.abad_feedback_source = str(self.cfg.get("abad_feedback_source", "commanded"))
        self.estimate_missing_joint_velocity = bool(self.cfg.get("estimate_missing_joint_velocity", True))
        self.observation_clip = float(self.cfg.get("observation_clip", 100.0))
        self.command_limits = dict(C.COMMAND_LIMITS)
        self.command_limits.update(self.cfg.get("command_limits", {}))
        self.command_profile = str(
            self.cfg.get("command_profile", "external_cmd_vel")
        ).strip()
        self.fixed_forward_vx = float(
            self.cfg.get("fixed_forward_vx", 0.22)
        )
        self.require_imu_source_stamp = bool(
            self.cfg.get("require_imu_source_stamp", False)
        )
        self.max_imu_source_age_s = float(
            self.cfg.get("max_imu_source_age_s", 0.10)
        )
        self.max_imu_future_skew_s = float(
            self.cfg.get("max_imu_future_skew_s", 0.02)
        )
        self.disabled_leg_indices = sorted(
            {int(index) for index in self.cfg.get("disabled_leg_indices", [])}
        )
        self.disabled_leg_observation_mode = str(
            self.cfg.get("disabled_leg_observation_mode", "passthrough")
        ).strip()

        self.main_drive_joint_names = list(self.cfg.get("main_drive_joint_names", C.MAIN_DRIVE_JOINT_NAMES))
        self.abad_joint_names = list(self.cfg.get("abad_joint_names", C.ABAD_JOINT_NAMES))
        self.required_joint_names = list(self.main_drive_joint_names)
        if self.abad_feedback_source == "joint_states":
            self.required_joint_names += self.abad_joint_names
        if self.disabled_leg_observation_mode == "nominal":
            disabled_names = {
                names[index]
                for names in (self.main_drive_joint_names, self.abad_joint_names)
                for index in self.disabled_leg_indices
            }
            self.required_joint_names = [
                name for name in self.required_joint_names if name not in disabled_names
            ]
        self._validate_config()

        self.last_actions = np.zeros(C.ACTION_DIM, dtype=np.float32)
        self._pending_last_actions = np.zeros(C.ACTION_DIM, dtype=np.float32)
        self.gait_phase = 0.0
        self._history: deque[np.ndarray] = deque(maxlen=max(1, self.history_length))
        self._last_build_time: float | None = None
        self._sensor_v2_history = SensorV2HistoryBuffer()

        self.imu_quat_xyzw: np.ndarray | None = None
        self.imu_ang_vel = np.zeros(3, dtype=np.float64)
        self.imu_time: float | None = None
        self.imu_receipt_monotonic_s: float | None = None
        self.imu_source_stamp_s: float | None = None
        self.imu_valid_reason: str | None = None
        self.joint_pos: dict[str, float] = {}
        self.joint_vel: dict[str, float] = {}
        self.joint_last_seen: dict[str, float] = {}
        self.invalid_joint_names: set[str] = set()
        self.joint_time: float | None = None
        self.joint_sequence = 0
        self.cmd_vel = np.zeros(3, dtype=np.float64)
        self.cmd_time: float | None = None
        self._policy_running = False
        self.odom_lin_vel = np.zeros(3, dtype=np.float64)
        self.odom_time: float | None = None
        self.commanded_abad_pos = np.asarray(C.INIT_ABAD_POS, dtype=np.float64)
        self.commanded_abad_vel = np.zeros(6, dtype=np.float64)
        self.commanded_abad_time: float | None = None
        self.last_single_observation: np.ndarray | None = None

    def _validate_config(self) -> None:
        if self.sensor_profile not in (
            "full_state",
            "encoder_only_rig",
            "full_feedback_rig",
        ):
            raise ValueError(
                "sensor_profile must be 'full_state', 'encoder_only_rig', "
                "or 'full_feedback_rig'"
            )
        if self.sensor_profile == "encoder_only_rig" and not self.encoder_only_rig_acknowledged:
            raise ValueError(
                "encoder_only_rig requires encoder_only_rig_acknowledged=true; "
                "this mode has no measured base velocity, angular velocity, tilt, or fall detection"
            )
        if self.sensor_profile == "full_feedback_rig":
            if not self.full_feedback_rig_acknowledged:
                raise ValueError(
                    "full_feedback_rig requires full_feedback_rig_acknowledged=true"
                )
            if not self.imu_alignment_calibrated:
                raise ValueError(
                    "full_feedback_rig requires imu_alignment_calibrated=true"
                )
            if self.base_lin_vel_source != "zero":
                raise ValueError("full_feedback_rig requires base_lin_vel_source='zero'")
            if self.abad_feedback_source != "joint_states":
                raise ValueError(
                    "full_feedback_rig requires abad_feedback_source='joint_states'"
                )
            upright_gravity = _quat_inverse_rotate_xyzw(
                self.imu_upright_quaternion_xyzw,
                np.array([0.0, 0.0, -1.0], dtype=np.float64),
            )
            if not np.allclose(
                upright_gravity,
                np.asarray(C.REFERENCE_PROJECTED_GRAVITY, dtype=np.float64),
                rtol=0.0,
                atol=1.0e-3,
            ):
                raise ValueError(
                    "full_feedback_rig imu_upright_quaternion_xyzw must produce "
                    "the training projected gravity [0,-1,0]"
                )
        if self.rig_projected_gravity.shape != (3,) or not np.isfinite(self.rig_projected_gravity).all():
            raise ValueError("rig_projected_gravity must contain three finite values")
        gravity_norm = float(np.linalg.norm(self.rig_projected_gravity))
        if abs(gravity_norm - 1.0) > 1.0e-3:
            raise ValueError("rig_projected_gravity must be a unit vector")
        if self.sensor_profile == "encoder_only_rig" and self.base_lin_vel_source != "zero":
            raise ValueError("encoder_only_rig requires base_lin_vel_source='zero'")
        if self.expected_obs_dim != C.OBS_DIM_SINGLE:
            raise ValueError(f"expected_obs_dim must be {C.OBS_DIM_SINGLE}, got {self.expected_obs_dim}")
        if self.history_length <= 0:
            raise ValueError("policy_history_length must be positive")
        if self.history_length != C.POLICY_HISTORY_LENGTH:
            raise ValueError(
                f"policy_history_length must be {C.POLICY_HISTORY_LENGTH}"
            )
        if self.policy_input_dim not in (C.OBS_DIM_SINGLE, C.OBS_DIM_SINGLE * self.history_length):
            raise ValueError(
                f"policy_input_dim must be {C.OBS_DIM_SINGLE} or {C.OBS_DIM_SINGLE * self.history_length}, "
                f"got {self.policy_input_dim}"
            )
        if self.base_lin_vel_source not in ("zero", "odom"):
            raise ValueError("base_lin_vel_source must be 'zero' or 'odom'")
        if self.abad_feedback_source not in ("commanded", "joint_states"):
            raise ValueError("abad_feedback_source must be 'commanded' or 'joint_states'")
        if not np.isfinite(self.observation_clip) or self.observation_clip <= 0.0:
            raise ValueError("observation_clip must be positive and finite")
        if self.disabled_leg_observation_mode not in ("passthrough", "nominal"):
            raise ValueError(
                "disabled_leg_observation_mode must be 'passthrough' or 'nominal'"
            )
        if any(index < 0 or index >= 6 for index in self.disabled_leg_indices):
            raise ValueError("disabled_leg_indices entries must be in [0, 5]")
        if (
            self.sensor_profile in ("encoder_only_rig", "full_feedback_rig")
            and self.disabled_leg_indices
            and self.disabled_leg_observation_mode != "nominal"
        ):
            raise ValueError(
                "strict rig disabled legs require disabled_leg_observation_mode='nominal'"
            )
        if (
            not math.isfinite(self.max_imu_source_age_s)
            or self.max_imu_source_age_s <= 0.0
            or not math.isfinite(self.max_imu_future_skew_s)
            or self.max_imu_future_skew_s < 0.0
        ):
            raise ValueError("IMU source stamp age/skew limits are invalid")
        if self.sensor_profile == "full_feedback_rig" and not self.require_imu_source_stamp:
            raise ValueError("full_feedback_rig requires require_imu_source_stamp=true")

        for name, joint_names in (
            ("main_drive_joint_names", self.main_drive_joint_names),
            ("abad_joint_names", self.abad_joint_names),
        ):
            if len(joint_names) != 6:
                raise ValueError(f"{name} must contain 6 joints, got {len(joint_names)}")
            if len(set(joint_names)) != len(joint_names):
                raise ValueError(f"{name} contains duplicate names: {joint_names}")
        if set(self.main_drive_joint_names).intersection(self.abad_joint_names):
            raise ValueError("main_drive_joint_names and abad_joint_names overlap")

        for key in ("vx", "vy", "wz"):
            lo = self.command_limits[f"{key}_min"]
            hi = self.command_limits[f"{key}_max"]
            if not np.isfinite([lo, hi]).all() or lo > hi:
                raise ValueError(f"invalid command limit for {key}: min={lo}, max={hi}")

        if self.command_profile not in ("external_cmd_vel", "fixed_forward"):
            raise ValueError(
                "command_profile must be 'external_cmd_vel' or 'fixed_forward'"
            )
        if not math.isfinite(self.fixed_forward_vx):
            raise ValueError("fixed_forward_vx must be finite")
        if self.command_profile == "fixed_forward":
            # Stage-5 resolves forward gait only above 0.10 m/s.  Accepting a
            # smaller fixed value would silently reproduce the original bug:
            # the policy would run, but the decoder would remain in neutral.
            if self.fixed_forward_vx <= 0.10:
                raise ValueError(
                    "fixed_forward_vx must be > 0.10 m/s to enter the stage-5 forward branch"
                )
            if not (
                self.command_limits["vx_min"]
                <= self.fixed_forward_vx
                <= self.command_limits["vx_max"]
            ):
                raise ValueError(
                    "fixed_forward_vx must be inside commands.vx_min..vx_max"
                )
            for axis in ("vy", "wz"):
                if not (
                    self.command_limits[f"{axis}_min"]
                    <= 0.0
                    <= self.command_limits[f"{axis}_max"]
                ):
                    raise ValueError(
                        f"fixed_forward requires zero to be inside the {axis} command limits"
                    )

    def reset(self, gait_phase: float = 0.0) -> None:
        self.gait_phase = float(gait_phase) % (2.0 * math.pi)
        self.last_actions[:] = 0.0
        self._pending_last_actions[:] = 0.0
        self.commanded_abad_pos = np.asarray(C.INIT_ABAD_POS, dtype=np.float64)
        self.commanded_abad_vel[:] = 0.0
        self.commanded_abad_time = None
        self._history.clear()
        self._last_build_time = None
        self._sensor_v2_history.reset()
        self.last_single_observation = None
        self.set_policy_running(False)

    def set_policy_input_dim(self, policy_input_dim: int) -> None:
        """Bind ONNX input shape to the configured history contract at startup."""

        resolved = int(policy_input_dim)
        allowed = (C.OBS_DIM_SINGLE, C.OBS_DIM_SINGLE * self.history_length)
        if resolved not in allowed:
            raise ValueError(
                f"ONNX policy input dim {resolved} is incompatible with history length "
                f"{self.history_length}; expected {allowed[0]} or {allowed[1]}"
            )
        if (
            resolved != C.OBS_DIM_SINGLE
            and self.history_length != C.POLICY_HISTORY_LENGTH
        ):
            raise ValueError(
                f"history ONNX requires exactly {C.POLICY_HISTORY_LENGTH} frames"
            )
        self.policy_input_dim = resolved

    def update_imu(
        self,
        msg: object,
        now_s: float | None = None,
        receipt_monotonic_s: float | None = None,
    ) -> None:
        orientation = getattr(msg, "orientation")
        angular_velocity = getattr(msg, "angular_velocity")
        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "")).strip()
        quat = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64
        )
        angular = np.array(
            [angular_velocity.x, angular_velocity.y, angular_velocity.z], dtype=np.float64
        )
        # ROS2 fixed-size numeric fields may be numpy arrays. Never use their
        # truth value: numpy deliberately rejects ambiguous array booleans.
        raw_orientation_covariance = getattr(msg, "orientation_covariance", None)
        orientation_covariance = (
            []
            if raw_orientation_covariance is None
            else list(raw_orientation_covariance)
        )
        raw_angular_velocity_covariance = getattr(
            msg, "angular_velocity_covariance", None
        )
        angular_velocity_covariance = (
            []
            if raw_angular_velocity_covariance is None
            else list(raw_angular_velocity_covariance)
        )
        receipt_monotonic = (
            time.monotonic()
            if receipt_monotonic_s is None
            else float(receipt_monotonic_s)
        )
        # ``now_s`` is the ROS/system clock and is the only clock compared to
        # the ROS header stamp. Receipt freshness uses monotonic time below.
        source_now_s = None if now_s is None else float(now_s)
        receipt_s = receipt_monotonic if source_now_s is None else source_now_s
        source_stamp_s = _source_stamp_to_float(msg)
        stamp_error: str | None = None
        if self.require_imu_source_stamp and source_stamp_s is None:
            stamp_error = "IMU source header stamp is missing or zero"
        elif source_stamp_s is not None:
            if (
                self.imu_source_stamp_s is not None
                and source_stamp_s <= self.imu_source_stamp_s
            ):
                stamp_error = "IMU source header stamp is not strictly monotonic"
            elif (
                source_now_s is not None
                and source_now_s - source_stamp_s > self.max_imu_source_age_s
            ):
                stamp_error = "IMU source header stamp is stale"
            elif (
                source_now_s is not None
                and source_stamp_s - source_now_s > self.max_imu_future_skew_s
            ):
                stamp_error = "IMU source header stamp is too far in the future"
        if stamp_error is not None:
            self.imu_valid_reason = stamp_error
        elif self.expected_imu_frame_id and frame_id != self.expected_imu_frame_id:
            self.imu_valid_reason = (
                "IMU frame_id mismatch: expected "
                f"'{self.expected_imu_frame_id}', got '{frame_id}'"
            )
        elif orientation_covariance and float(orientation_covariance[0]) < 0.0:
            self.imu_valid_reason = "IMU orientation is marked unavailable"
        elif angular_velocity_covariance and float(angular_velocity_covariance[0]) < 0.0:
            self.imu_valid_reason = "IMU angular velocity is marked unavailable"
        elif not np.isfinite(quat).all() or not np.isfinite(angular).all():
            self.imu_valid_reason = "IMU contains NaN or Inf"
        elif float(np.linalg.norm(quat)) < 1.0e-6:
            self.imu_valid_reason = "IMU quaternion norm is zero"
        elif abs(float(np.linalg.norm(quat)) - 1.0) > 0.10:
            self.imu_valid_reason = "IMU quaternion is not near unit length"
        else:
            self.imu_valid_reason = None
            # ROS IMU orientation is q_W_S.  The calibrated mounting rotation
            # q_S_P maps policy-frame vectors into the sensor frame, therefore
            # q_W_P = q_W_S * q_S_P.
            self.imu_quat_xyzw = _normalize_quat_xyzw(
                _quat_multiply_xyzw(
                    _normalize_quat_xyzw(quat),
                    self.imu_policy_to_sensor_quaternion_xyzw,
                )
            )
            # angular_velocity is expressed in S; inverse(q_S_P) maps it to P.
            self.imu_ang_vel = _quat_inverse_rotate_xyzw(
                self.imu_policy_to_sensor_quaternion_xyzw,
                angular,
            )
            if source_stamp_s is not None:
                self.imu_source_stamp_s = source_stamp_s
        self.imu_time = receipt_s
        self.imu_receipt_monotonic_s = receipt_monotonic

    def update_joint_state(self, msg: object, now_s: float | None = None) -> None:
        names: Iterable[str] = getattr(msg, "name", [])
        positions = list(getattr(msg, "position", []))
        velocities = list(getattr(msg, "velocity", []))
        stamp_s = now_s if now_s is not None else _stamp_to_float(msg)
        prev_time = self.joint_time
        dt = None if prev_time is None else max(0.0, stamp_s - prev_time)
        names = [str(name) for name in names]
        duplicate_names = {name for name in names if names.count(name) > 1}
        self.invalid_joint_names.update(duplicate_names)
        for idx, name in enumerate(names):
            if idx < len(positions):
                old_pos = self.joint_pos.get(name)
                new_pos = float(positions[idx])
                if math.isfinite(new_pos) and name not in duplicate_names:
                    self.joint_pos[name] = new_pos
                    self.joint_last_seen[name] = stamp_s
                    self.invalid_joint_names.discard(name)
                    if (
                        idx >= len(velocities)
                        and self.estimate_missing_joint_velocity
                        and name in self.required_joint_names
                        and old_pos is not None
                        and dt is not None
                        and dt > 1.0e-6
                    ):
                        self.joint_vel[name] = _wrap_angle_diff(new_pos, old_pos) / dt
                else:
                    self.invalid_joint_names.add(name)
            if idx < len(velocities):
                velocity = float(velocities[idx])
                if math.isfinite(velocity) and name not in duplicate_names:
                    self.joint_vel[name] = velocity
                else:
                    self.invalid_joint_names.add(name)
        self.joint_time = stamp_s
        self.joint_sequence += 1

    def update_cmd_vel(self, msg: object, now_s: float | None = None) -> None:
        if self.command_profile == "fixed_forward":
            # A fixed deployment command is part of the artifact/config hash.
            # Ignore an accidental keyboard/teleop publisher so it cannot
            # silently change the command semantics on hardware.
            return
        linear = getattr(msg, "linear")
        angular = getattr(msg, "angular")
        cmd = np.array([linear.x, linear.y, angular.z], dtype=np.float64)
        cmd[0] = np.clip(cmd[0], self.command_limits["vx_min"], self.command_limits["vx_max"])
        cmd[1] = np.clip(cmd[1], self.command_limits["vy_min"], self.command_limits["vy_max"])
        cmd[2] = np.clip(cmd[2], self.command_limits["wz_min"], self.command_limits["wz_max"])
        self.cmd_vel = cmd
        self.cmd_time = now_s if now_s is not None else _stamp_to_float(msg)

    def set_policy_running(self, running: bool) -> None:
        """Activate a fixed deployment command only while policy output runs.

        INIT_STAND, WARMUP, POLICY_READY and every stop state see a zero fixed
        command.  External /cmd_vel behavior stays backward compatible.
        """

        self._policy_running = bool(running)
        if self.command_profile != "fixed_forward":
            return
        self.cmd_vel[:] = (
            [self.fixed_forward_vx, 0.0, 0.0]
            if self._policy_running
            else [0.0, 0.0, 0.0]
        )
        self.cmd_time = None

    def update_odom(self, msg: object, now_s: float | None = None) -> None:
        twist = getattr(getattr(msg, "twist"), "twist")
        lin = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float64)
        if not self.odom_twist_in_body_frame and self.imu_quat_xyzw is not None:
            lin = _quat_inverse_rotate_xyzw(self.imu_quat_xyzw, lin)
        self.odom_lin_vel = lin
        self.odom_time = now_s if now_s is not None else _stamp_to_float(msg)

    def update_last_actions(self, action: np.ndarray) -> None:
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (C.ACTION_DIM,):
            raise ValueError(f"last action shape must be ({C.ACTION_DIM},), got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("last action contains NaN or Inf")
        # DirectRLEnv calls _pre_physics_step(action_t), where RedrhexEnv first
        # copies actions_{t-1} to last_actions and then stores action_t.  The
        # following obs_{t+1} therefore contains action_{t-1}: a two-inference
        # lag.  Keep one pending slot so hardware matches that exact ordering.
        clipped = np.clip(
            action, -C.TRAINING_ACTION_CLIP, C.TRAINING_ACTION_CLIP
        ).astype(np.float32)
        self.last_actions = self._pending_last_actions.copy()
        self._pending_last_actions = clipped

    def update_commanded_abad_position(self, target_position: np.ndarray, now_s: float) -> None:
        target = np.asarray(target_position, dtype=np.float64).reshape(-1)
        if target.shape != (6,):
            raise ValueError(f"commanded ABAD position must have shape (6,), got {target.shape}")
        if not np.isfinite(target).all():
            raise ValueError("commanded ABAD position contains NaN or Inf")
        if self.commanded_abad_time is not None:
            dt = max(0.0, now_s - self.commanded_abad_time)
            if dt > 1.0e-6:
                self.commanded_abad_vel = (target - self.commanded_abad_pos) / dt
        self.commanded_abad_pos = target.copy()
        self.commanded_abad_time = now_s

    def status(
        self,
        now_s: float,
        sensor_timeout_s: float,
        cmd_timeout_s: float,
        monotonic_now_s: float | None = None,
    ) -> ObservationStatus:
        reasons: list[str] = []
        if self.requires_imu:
            if self.imu_time is None:
                reasons.append("waiting for /imu/data")
            elif (
                monotonic_now_s is not None
                and self.imu_receipt_monotonic_s is not None
                and monotonic_now_s - self.imu_receipt_monotonic_s
                > sensor_timeout_s
            ) or (
                monotonic_now_s is None and now_s - self.imu_time > sensor_timeout_s
            ):
                reasons.append("IMU timeout")
            elif self.imu_valid_reason:
                reasons.append(self.imu_valid_reason)
        if self.joint_time is None:
            reasons.append("waiting for /joint_states")
        elif now_s - self.joint_time > sensor_timeout_s:
            reasons.append("joint_states timeout")
        missing = [name for name in self.required_joint_names if name not in self.joint_pos]
        if missing:
            reasons.append(f"missing joints: {missing}")
        stale = [
            name
            for name in self.required_joint_names
            if name in self.joint_last_seen and now_s - self.joint_last_seen[name] > sensor_timeout_s
        ]
        if stale:
            reasons.append(f"stale joints: {stale}")
        invalid = sorted(set(self.required_joint_names).intersection(self.invalid_joint_names))
        if invalid:
            reasons.append(f"invalid joints: {invalid}")
        if self.base_lin_vel_source == "odom":
            if self.odom_time is None:
                reasons.append("waiting for /odom")
            elif now_s - self.odom_time > sensor_timeout_s:
                reasons.append("odom timeout")
        if (
            self.command_profile == "external_cmd_vel"
            and self.cmd_time is not None
            and now_s - self.cmd_time > cmd_timeout_s
        ):
            self.cmd_vel[:] = 0.0
        return ObservationStatus(ok=len(reasons) == 0, reasons=reasons)

    def build_single(self, now_s: float, update_phase: bool = True) -> np.ndarray:
        if self.requires_imu and self.imu_quat_xyzw is None:
            raise RuntimeError("Cannot build observation before IMU is received.")
        missing = [name for name in self.required_joint_names if name not in self.joint_pos]
        if missing:
            raise RuntimeError(f"Cannot build observation; missing joint_states for {missing}.")

        if update_phase:
            if self._last_build_time is None:
                dt = C.CONTROL_DT
            else:
                dt = max(0.0, min(0.05, now_s - self._last_build_time))
            self.gait_phase = (self.gait_phase + 2.0 * math.pi * self.base_gait_frequency_hz * dt) % (2.0 * math.pi)
            self._last_build_time = now_s

        if self.sensor_profile == "encoder_only_rig":
            base_lin_vel = np.zeros(3, dtype=np.float64)
            base_ang_vel = np.zeros(3, dtype=np.float64)
            projected_gravity = self.rig_projected_gravity
        elif self.base_lin_vel_source == "odom" and self.odom_time is not None:
            base_lin_vel = self.odom_lin_vel
            base_ang_vel = self.imu_ang_vel
            projected_gravity = _quat_inverse_rotate_xyzw(
                self.imu_quat_xyzw, np.array([0.0, 0.0, -1.0])
            )
        else:
            base_lin_vel = np.zeros(3, dtype=np.float64)
            base_ang_vel = self.imu_ang_vel
            projected_gravity = _quat_inverse_rotate_xyzw(
                self.imu_quat_xyzw, np.array([0.0, 0.0, -1.0])
            )
        main_pos = np.array(
            [
                self.joint_pos.get(
                    name,
                    C.INIT_MAIN_DRIVE_POS[index]
                    if self.disabled_leg_observation_mode == "nominal"
                    and index in self.disabled_leg_indices
                    else 0.0,
                )
                for index, name in enumerate(self.main_drive_joint_names)
            ],
            dtype=np.float64,
        )
        main_vel = np.array([self.joint_vel.get(name, 0.0) for name in self.main_drive_joint_names], dtype=np.float64)
        if self.abad_feedback_source == "joint_states":
            abad_pos = np.array(
                [
                    self.joint_pos.get(
                        name,
                        C.INIT_ABAD_POS[index]
                        if self.disabled_leg_observation_mode == "nominal"
                        and index in self.disabled_leg_indices
                        else 0.0,
                    )
                    for index, name in enumerate(self.abad_joint_names)
                ],
                dtype=np.float64,
            )
            abad_vel = np.array([self.joint_vel.get(name, 0.0) for name in self.abad_joint_names], dtype=np.float64)
        else:
            abad_pos = self.commanded_abad_pos.copy()
            abad_vel = self.commanded_abad_vel.copy()
        if self.disabled_leg_observation_mode == "nominal":
            for index in self.disabled_leg_indices:
                main_pos[index] = float(C.INIT_MAIN_DRIVE_POS[index])
                main_vel[index] = 0.0
                abad_pos[index] = float(C.INIT_ABAD_POS[index])
                abad_vel[index] = 0.0

        obs = np.concatenate(
            [
                base_lin_vel,
                base_ang_vel,
                projected_gravity,
                np.sin(main_pos),
                np.cos(main_pos),
                main_vel / self.base_gait_angular_vel,
                abad_pos / self.abad_pos_scale,
                abad_vel,
                self.cmd_vel,
                np.array([math.sin(self.gait_phase), math.cos(self.gait_phase)], dtype=np.float64),
                self.last_actions.astype(np.float64),
            ]
        )

        if obs.shape != (C.OBS_DIM_SINGLE,):
            raise RuntimeError(f"Observation dim {obs.shape[0]} != {C.OBS_DIM_SINGLE}.")
        if not np.isfinite(obs).all():
            raise RuntimeError("Observation contains NaN or Inf.")
        # IsaacLab applies the same clamp to the concatenated policy
        # observation. Keep the hardware inference contract byte-for-byte close.
        obs = np.clip(obs, -self.observation_clip, self.observation_clip).astype(np.float32)
        self.last_single_observation = obs.copy()
        return obs

    def build_policy_input(self, now_s: float) -> np.ndarray:
        obs = self.build_single(now_s)
        self._history.appendleft(obs.copy())
        if self.policy_input_dim == C.OBS_DIM_SINGLE:
            return obs
        expected_history_dim = C.OBS_DIM_SINGLE * self.history_length
        if self.policy_input_dim != expected_history_dim:
            raise RuntimeError(
                f"Unsupported policy_input_dim={self.policy_input_dim}; expected 56 or {expected_history_dim}."
            )
        frames = list(self._history)
        while len(frames) < self.history_length:
            frames.append(np.zeros(C.OBS_DIM_SINGLE, dtype=np.float32))
        stacked = np.concatenate(frames, axis=0).astype(np.float32)
        if stacked.shape != (expected_history_dim,):
            raise RuntimeError(f"Stacked observation dim {stacked.shape[0]} != {expected_history_dim}.")
        return stacked

    def build_sensor_v2_frame(self, now_s: float) -> np.ndarray:
        """Build the raw 36-D sensor frame specified by student-observation.v2."""
        # Reuse the validated/aligned sensor extraction in build_single, then
        # replace legacy-normalized joint fields with the required raw units.
        legacy = self.build_single(now_s, update_phase=False)
        main_pos = self.get_main_drive_positions().astype(np.float64)
        main_vel = np.array(
            [self.joint_vel.get(name, 0.0) for name in self.main_drive_joint_names],
            dtype=np.float64,
        )
        abad_pos = self.get_abad_positions().astype(np.float64)
        if self.abad_feedback_source == "joint_states":
            abad_vel = np.array(
                [self.joint_vel.get(name, 0.0) for name in self.abad_joint_names],
                dtype=np.float64,
            )
        else:
            abad_vel = self.commanded_abad_vel.copy()
        if self.disabled_leg_observation_mode == "nominal":
            for index in self.disabled_leg_indices:
                main_pos[index] = C.INIT_MAIN_DRIVE_POS[index]
                main_vel[index] = 0.0
                abad_pos[index] = C.INIT_ABAD_POS[index]
                abad_vel[index] = 0.0
        frame = np.concatenate(
            [legacy[3:6], legacy[6:9], np.sin(main_pos), np.cos(main_pos), main_vel, abad_pos, abad_vel]
        ).astype(np.float32)
        if frame.shape != (36,) or not np.isfinite(frame).all():
            raise RuntimeError("sensor-v2 frame must be a finite 36-D vector")
        return frame

    def build_sensor_v2_input(self, now_s: float) -> np.ndarray:
        """Return [60,36] history sampled at 60 Hz, oldest frame first."""
        return self._sensor_v2_history.update(self.build_sensor_v2_frame(now_s), now_s)

    def sensor_v2_command(self) -> np.ndarray:
        """Fixed forward command used by the suspended-rig policy test."""
        return np.array([self.fixed_forward_vx, 0.0, 0.0], dtype=np.float32)

    def get_roll_pitch_yaw(self) -> tuple[float, float, float]:
        if not self.requires_imu or self.imu_quat_xyzw is None:
            return 0.0, 0.0, 0.0
        relative = _normalize_quat_xyzw(
            _quat_multiply_xyzw(
                _quat_inverse_xyzw(self.imu_upright_quaternion_xyzw),
                self.imu_quat_xyzw,
            )
        )
        return _quat_to_roll_pitch_yaw(relative)

    def get_relative_tilt_roll_pitch(self) -> tuple[float, float]:
        """Return yaw-invariant tilt components around the policy X/Z axes.

        The training frame is unusual: upright projected gravity is -Y, not
        the usual ROS -Z.  Raw quaternion RPY therefore calls the nominal
        +90-degree X alignment a fall, and a relative quaternion can call a
        harmless world-yaw rotation a pitch.  Gravity itself is yaw-invariant,
        so use its X/Z departure from the training -Y direction for the two
        tilt guards.
        """

        if not self.requires_imu or self.imu_quat_xyzw is None:
            return 0.0, 0.0
        projected_gravity = _quat_inverse_rotate_xyzw(
            self.imu_quat_xyzw,
            np.array([0.0, 0.0, -1.0], dtype=np.float64),
        )
        projected_gravity = projected_gravity / max(
            float(np.linalg.norm(projected_gravity)), 1.0e-12
        )
        upright_component = -float(projected_gravity[1])
        tilt_about_x = math.atan2(
            -float(projected_gravity[2]), upright_component
        )
        tilt_about_z = math.atan2(
            float(projected_gravity[0]), upright_component
        )
        return tilt_about_x, tilt_about_z

    @property
    def requires_imu(self) -> bool:
        return self.sensor_profile in ("full_state", "full_feedback_rig")

    @property
    def is_encoder_only_rig(self) -> bool:
        return self.sensor_profile == "encoder_only_rig"

    @property
    def is_full_feedback_rig(self) -> bool:
        return self.sensor_profile == "full_feedback_rig"

    def observation_provenance(self) -> list[str]:
        """Return one source label for every single-frame observation element."""
        if self.is_encoder_only_rig:
            base_sources = ["imputed_zero"] * 3 + ["imputed_zero"] * 3 + ["rig_constant"] * 3
        else:
            lin_source = "odometry" if self.base_lin_vel_source == "odom" else "imputed_zero"
            base_sources = [lin_source] * 3 + ["imu"] * 3 + ["imu"] * 3
        abad_source = "encoder" if self.abad_feedback_source == "joint_states" else "commanded_estimate"
        sources = (
            base_sources
            + ["encoder"] * 6
            + ["encoder"] * 6
            + ["encoder_velocity"] * 6
            + [abad_source] * 6
            + [f"{abad_source}_velocity"] * 6
            + ["command"] * 3
            + ["internal_phase"] * 2
            + ["previous_policy_action"] * 12
        )
        if self.disabled_leg_observation_mode == "nominal":
            for leg_index in self.disabled_leg_indices:
                for offset in (
                    9,
                    15,
                    21,
                    27,
                    33,
                ):
                    sources[offset + leg_index] = "disabled_leg_imputed"
        if len(sources) != C.OBS_DIM_SINGLE:
            raise RuntimeError(f"observation provenance dim {len(sources)} != {C.OBS_DIM_SINGLE}")
        return sources

    def imputed_observation_indices(self) -> list[int]:
        return [
            index
            for index, source in enumerate(self.observation_provenance())
            if source in (
                "imputed_zero",
                "rig_constant",
                "commanded_estimate",
                "commanded_estimate_velocity",
                "disabled_leg_imputed",
            )
        ]

    def get_main_drive_positions(self) -> np.ndarray:
        values = np.array(
            [self.joint_pos.get(name, 0.0) for name in self.main_drive_joint_names],
            dtype=np.float64,
        )
        if self.disabled_leg_observation_mode == "nominal":
            values[self.disabled_leg_indices] = np.asarray(C.INIT_MAIN_DRIVE_POS)[
                self.disabled_leg_indices
            ]
        return values

    def get_abad_positions(self) -> np.ndarray:
        if self.abad_feedback_source == "commanded":
            values = self.commanded_abad_pos.copy()
        else:
            values = np.array(
                [self.joint_pos.get(name, 0.0) for name in self.abad_joint_names],
                dtype=np.float64,
            )
        if self.disabled_leg_observation_mode == "nominal":
            values[self.disabled_leg_indices] = np.asarray(C.INIT_ABAD_POS)[
                self.disabled_leg_indices
            ]
        return values
