"""Safety checks for RedRhex real-robot policy deployment."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import redrhex_contract as C
from .action_decoder import DecodedMotorCommand


@dataclass
class SafetyState:
    estop: bool = False
    imu_age_s: float | None = None
    joint_state_age_s: float | None = None
    motor_feedback_age_s: float | None = None
    heartbeat_age_s: float | None = None
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    command: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    motor_temperatures_c: list[float] = field(default_factory=list)
    motor_currents_a: list[float] = field(default_factory=list)
    motor_faults: list[bool] = field(default_factory=list)
    motor_velocities_rad_s: list[float] = field(default_factory=list)
    control_loop_dt_s: float | None = None
    inference_duration_s: float | None = None
    inference_sequence: int | None = None
    motor_feedback_sequence: int | None = None
    control_sequence: int | None = None


@dataclass
class SafetyResult:
    ok: bool
    reasons: list[str]


class SafetyFilter:
    def __init__(self, config: dict | None = None) -> None:
        self.cfg = config or {}
        self.sensor_timeout_s = float(self.cfg.get("sensor_timeout_s", 0.10))
        self.cmd_timeout_s = float(self.cfg.get("cmd_timeout_s", 0.25))
        self.motor_feedback_timeout_s = float(self.cfg.get("motor_feedback_timeout_s", 0.25))
        self.heartbeat_timeout_s = float(self.cfg.get("heartbeat_timeout_s", 0.10))
        self.max_abs_roll_rad = float(self.cfg.get("max_abs_roll_rad", 0.7))
        self.max_abs_pitch_rad = float(self.cfg.get("max_abs_pitch_rad", 0.7))
        self.action_clip = float(self.cfg.get("action_clip", 1.0))
        self.max_raw_action_abs = float(self.cfg.get("max_raw_action_abs", 1.5))
        self.main_drive_vel_limit_rad_s = float(self.cfg.get("main_drive_vel_limit_rad_s", 30.0))
        self.abad_pos_limit_rad = float(self.cfg.get("abad_pos_limit_rad", 0.7))
        self.max_motor_temperature_c = float(self.cfg.get("max_motor_temperature_c", 70.0))
        self.max_motor_current_a = float(self.cfg.get("max_motor_current_a", 20.0))
        self.max_measured_main_drive_velocity_rad_s = float(
            self.cfg.get("max_measured_main_drive_velocity_rad_s", 80.0)
        )
        self.max_control_loop_dt_s = float(self.cfg.get("max_control_loop_dt_s", 0.03))
        self.max_inference_duration_s = float(
            self.cfg.get("max_inference_duration_s", 0.008)
        )
        self.motor_current_trip_samples = int(self.cfg.get("motor_current_trip_samples", 5))
        self.control_loop_trip_samples = int(self.cfg.get("control_loop_trip_samples", 3))
        self.inference_trip_samples = int(self.cfg.get("inference_trip_samples", 3))
        self.disabled_leg_indices = sorted({int(x) for x in self.cfg.get("disabled_leg_indices", [])})
        self.require_motor_feedback = bool(self.cfg.get("require_motor_feedback", False))
        self.require_lowlevel_heartbeat = bool(self.cfg.get("require_lowlevel_heartbeat", False))
        self.require_imu = bool(self.cfg.get("require_imu", True))
        self.enable_tilt_guard = bool(self.cfg.get("enable_tilt_guard", True))
        self.command_limits = dict(C.COMMAND_LIMITS)
        self.command_limits.update(self.cfg.get("command_limits", {}))
        self._current_trip_count = 0
        self._control_loop_trip_count = 0
        self._inference_trip_count = 0
        self._last_motor_feedback_sequence: int | None = None
        self._last_control_sequence: int | None = None
        self._last_inference_sequence: int | None = None
        self._validate_config()

    def _validate_config(self) -> None:
        positive_fields = {
            "sensor_timeout_s": self.sensor_timeout_s,
            "cmd_timeout_s": self.cmd_timeout_s,
            "motor_feedback_timeout_s": self.motor_feedback_timeout_s,
            "heartbeat_timeout_s": self.heartbeat_timeout_s,
            "max_abs_roll_rad": self.max_abs_roll_rad,
            "max_abs_pitch_rad": self.max_abs_pitch_rad,
            "action_clip": self.action_clip,
            "max_raw_action_abs": self.max_raw_action_abs,
            "main_drive_vel_limit_rad_s": self.main_drive_vel_limit_rad_s,
            "abad_pos_limit_rad": self.abad_pos_limit_rad,
            "max_motor_temperature_c": self.max_motor_temperature_c,
            "max_motor_current_a": self.max_motor_current_a,
            "max_measured_main_drive_velocity_rad_s": self.max_measured_main_drive_velocity_rad_s,
            "max_control_loop_dt_s": self.max_control_loop_dt_s,
            "max_inference_duration_s": self.max_inference_duration_s,
        }
        for name, value in positive_fields.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite, got {value}")
        if (
            self.motor_current_trip_samples <= 0
            or self.control_loop_trip_samples <= 0
            or self.inference_trip_samples <= 0
        ):
            raise ValueError("safety trip sample counts must be positive")
        if self.enable_tilt_guard and not self.require_imu:
            raise ValueError("enable_tilt_guard=true requires require_imu=true")
        if any(index < 0 or index >= 6 for index in self.disabled_leg_indices):
            raise ValueError("disabled_leg_indices entries must be in [0, 5]")
        for key in ("vx", "vy", "wz"):
            lo = self.command_limits[f"{key}_min"]
            hi = self.command_limits[f"{key}_max"]
            if not np.isfinite([lo, hi]).all() or lo > hi:
                raise ValueError(f"invalid command limit for {key}: min={lo}, max={hi}")

    @staticmethod
    def _as_vector(values, expected_len: int, name: str, reasons: list[str]) -> np.ndarray | None:
        try:
            arr = np.asarray(values, dtype=np.float64).reshape(-1)
        except Exception as exc:
            reasons.append(f"{name} invalid: {exc}")
            return None
        if arr.shape != (expected_len,):
            reasons.append(f"{name} shape {arr.shape} != ({expected_len},)")
            return None
        if not np.isfinite(arr).all():
            reasons.append(f"{name} NaN/Inf")
            return None
        return arr

    def check(
        self,
        state: SafetyState,
        observation: np.ndarray | None = None,
        raw_action: np.ndarray | None = None,
        command: DecodedMotorCommand | None = None,
    ) -> SafetyResult:
        reasons: list[str] = []

        if state.estop:
            reasons.append("E-stop active")
        if self.require_imu and (state.imu_age_s is None or state.imu_age_s > self.sensor_timeout_s):
            reasons.append("IMU timeout")
        if state.joint_state_age_s is None or state.joint_state_age_s > self.sensor_timeout_s:
            reasons.append("joint_states timeout")
        if self.require_motor_feedback and (
            state.motor_feedback_age_s is None or state.motor_feedback_age_s > self.motor_feedback_timeout_s
        ):
            reasons.append("motor_feedback timeout")
        if self.require_lowlevel_heartbeat and (
            state.heartbeat_age_s is None or state.heartbeat_age_s > self.heartbeat_timeout_s
        ):
            reasons.append("low-level heartbeat timeout")
        if self.enable_tilt_guard:
            if abs(state.roll_rad) > self.max_abs_roll_rad:
                reasons.append(f"roll too large: {state.roll_rad:.3f} rad")
            if abs(state.pitch_rad) > self.max_abs_pitch_rad:
                reasons.append(f"pitch too large: {state.pitch_rad:.3f} rad")
        new_control_sample = (
            state.control_sequence is None or state.control_sequence != self._last_control_sequence
        )
        if new_control_sample:
            self._last_control_sequence = state.control_sequence
            self._control_loop_trip_count = (
                self._control_loop_trip_count + 1
                if state.control_loop_dt_s is not None and state.control_loop_dt_s > self.max_control_loop_dt_s
                else 0
            )
        if self._control_loop_trip_count >= self.control_loop_trip_samples:
            loop_dt = 0.0 if state.control_loop_dt_s is None else state.control_loop_dt_s
            reasons.append(f"control loop deadline miss: {loop_dt:.4f} s")
        inference_s = state.inference_duration_s
        new_inference_sample = (
            state.inference_sequence is None
            or state.inference_sequence != self._last_inference_sequence
        )
        if inference_s is not None and new_inference_sample:
            self._last_inference_sequence = state.inference_sequence
            if not np.isfinite(inference_s) or inference_s < 0.0:
                reasons.append("inference duration NaN/Inf or negative")
            else:
                self._inference_trip_count = (
                    self._inference_trip_count + 1
                    if inference_s > self.max_inference_duration_s
                    else 0
                )
        if self._inference_trip_count >= self.inference_trip_samples:
            reasons.append(
                f"policy inference deadline miss: {float(inference_s or 0.0):.4f} s"
            )

        cmd = self._as_vector(state.command, 3, "velocity command", reasons)
        if cmd is not None:
            if (
                cmd[0] < self.command_limits["vx_min"]
                or cmd[0] > self.command_limits["vx_max"]
                or cmd[1] < self.command_limits["vy_min"]
                or cmd[1] > self.command_limits["vy_max"]
                or cmd[2] < self.command_limits["wz_min"]
                or cmd[2] > self.command_limits["wz_max"]
            ):
                reasons.append(f"velocity command outside training range: {cmd.tolist()}")

        if observation is not None and not np.isfinite(observation).all():
            reasons.append("observation NaN/Inf")
        if raw_action is not None:
            raw_action = self._as_vector(raw_action, C.ACTION_DIM, "policy action", reasons)
            if raw_action is not None and np.max(np.abs(raw_action)) > self.max_raw_action_abs + 1.0e-4:
                reasons.append("raw policy action magnitude implausibly large")

        if command is not None:
            main_vel = self._as_vector(command.target_main_drive_velocity, 6, "main drive velocity target", reasons)
            abad_pos = self._as_vector(command.target_abad_position, 6, "ABAD position target", reasons)
            if main_vel is not None and np.max(np.abs(main_vel)) > self.main_drive_vel_limit_rad_s + 1.0e-6:
                reasons.append("main drive velocity target exceeds limit")
            if abad_pos is not None and np.max(np.abs(abad_pos)) > self.abad_pos_limit_rad + 1.0e-6:
                reasons.append("ABAD position target exceeds limit")

        if state.motor_temperatures_c:
            temperatures = np.asarray(
                state.motor_temperatures_c, dtype=np.float64
            ).reshape(-1)
            if temperatures.shape != (6,):
                reasons.append(
                    f"motor temperature feedback shape {temperatures.shape} != (6,)"
                )
            else:
                healthy_temperatures = np.delete(
                    temperatures, self.disabled_leg_indices
                )
                if not np.isfinite(healthy_temperatures).all():
                    reasons.append("healthy motor temperature NaN/Inf")
                elif (
                    healthy_temperatures.size
                    and float(np.max(healthy_temperatures))
                    > self.max_motor_temperature_c
                ):
                    reasons.append("healthy motor temperature too high")
        new_feedback_sample = (
            state.motor_feedback_sequence is None
            or state.motor_feedback_sequence != self._last_motor_feedback_sequence
        )
        currents = np.asarray(state.motor_currents_a, dtype=np.float64).reshape(-1)
        active_currents: np.ndarray | None = None
        if currents.size > 0:
            if currents.shape != (6,):
                reasons.append(f"motor current feedback shape {currents.shape} != (6,)")
            else:
                healthy_mask = np.ones(6, dtype=bool)
                healthy_mask[self.disabled_leg_indices] = False
                # A physically isolated, explicitly disabled leg may report a
                # stale value, NaN, or over-current sentinel.  Never let that
                # bypass extend to a healthy leg.
                active_currents = currents[healthy_mask]
                if not np.isfinite(active_currents).all():
                    reasons.append("healthy motor current NaN/Inf")
        if new_feedback_sample:
            self._last_motor_feedback_sequence = state.motor_feedback_sequence
            over_current = (
                active_currents is not None
                and active_currents.size > 0
                and np.isfinite(active_currents).all()
                and float(np.max(np.abs(active_currents))) > self.max_motor_current_a
            )
            self._current_trip_count = self._current_trip_count + 1 if over_current else 0
        if active_currents is not None and self._current_trip_count >= self.motor_current_trip_samples:
            reasons.append("motor current too high")
        faults = np.asarray(state.motor_faults).reshape(-1)
        if faults.size > 0:
            if faults.shape != (6,):
                reasons.append(f"motor fault feedback shape {faults.shape} != (6,)")
            elif any(
                bool(value)
                for index, value in enumerate(faults)
                if index not in self.disabled_leg_indices
            ):
                reasons.append("healthy motor fault flag")

        velocities = np.asarray(state.motor_velocities_rad_s, dtype=np.float64).reshape(-1)
        if velocities.size > 0:
            if velocities.shape != (6,):
                reasons.append(f"motor velocity feedback shape {velocities.shape} != (6,)")
            else:
                active_velocities = np.delete(velocities, self.disabled_leg_indices)
                if not np.isfinite(active_velocities).all():
                    reasons.append("healthy motor velocity feedback NaN/Inf")
                elif (
                    active_velocities.size
                    and float(np.max(np.abs(active_velocities)))
                    > self.max_measured_main_drive_velocity_rad_s
                ):
                    reasons.append("healthy measured main drive velocity too high")

        return SafetyResult(ok=len(reasons) == 0, reasons=reasons)
