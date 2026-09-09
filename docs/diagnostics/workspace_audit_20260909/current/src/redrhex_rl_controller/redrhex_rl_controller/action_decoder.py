"""Decode RedRhex policy actions into motor command targets."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import redrhex_contract as C


@dataclass
class DecodedMotorCommand:
    joint_names: list[str]
    target_position_rad: list[float]
    target_velocity_rad_s: list[float]
    kp: list[float]
    kd: list[float]
    effort_limit_nm: list[float]
    enable: bool
    mode: int
    safe_action: np.ndarray
    target_main_drive_velocity: np.ndarray
    target_abad_position: np.ndarray


class ActionDecoder:
    """Replicates the deploy-critical action processing in RedrhexEnv._apply_action().

    Domain randomization and simulator-only torque proxies are intentionally not
    applied on the real robot. Mode gating, warmup, command bias, limits, and
    lateral/diag/yaw procedural parts are preserved for sim-to-real consistency.
    """

    MODE_DISABLED = 0
    MODE_MIXED_POSITION_VELOCITY = 1
    MODE_INIT_STAND = 2
    MODE_PROTECTIVE_STOP = 255

    def __init__(self, config: dict | None = None) -> None:
        self.cfg = config or {}
        self.main_drive_joint_names = list(self.cfg.get("main_drive_joint_names", C.MAIN_DRIVE_JOINT_NAMES))
        self.abad_joint_names = list(self.cfg.get("abad_joint_names", C.ABAD_JOINT_NAMES))
        self.damper_joint_names = list(self.cfg.get("damper_joint_names", C.DAMPER_JOINT_NAMES))
        self.include_damper_command = bool(self.cfg.get("include_damper_command", False))
        self.main_drive_init_control_mode = str(self.cfg.get("main_drive_init_control_mode", "velocity_to_pose"))
        self.disabled_leg_indices = sorted({int(x) for x in self.cfg.get("disabled_leg_indices", [])})
        self.healthy_leg_mask = np.ones(6, dtype=bool)
        for index in self.disabled_leg_indices:
            if index < 0 or index >= 6:
                raise ValueError(f"disabled_leg_indices entries must be in [0, 5], got {index}")
            self.healthy_leg_mask[index] = False

        self.direction_multiplier = np.asarray(
            self.cfg.get("leg_direction_multiplier", C.LEG_DIRECTION_MULTIPLIER), dtype=np.float64
        )
        self.leg_phase_offsets = np.zeros(6, dtype=np.float64)
        self.leg_phase_offsets[C.TRIPOD_A_LEG_INDICES] = 0.0
        self.leg_phase_offsets[C.TRIPOD_B_LEG_INDICES] = C.TRIPOD_PHASE_OFFSET

        self.main_sign = np.asarray(self.cfg.get("main_drive_sign", [1.0] * 6), dtype=np.float64)
        self.abad_sign = np.asarray(self.cfg.get("abad_sign", [1.0] * 6), dtype=np.float64)
        self.damper_sign = np.asarray(self.cfg.get("damper_sign", [1.0] * 6), dtype=np.float64)
        self.main_zero_offset = np.asarray(self.cfg.get("main_drive_zero_offset_rad", [0.0] * 6), dtype=np.float64)
        self.abad_zero_offset = np.asarray(self.cfg.get("abad_zero_offset_rad", [0.0] * 6), dtype=np.float64)
        self.damper_zero_offset = np.asarray(self.cfg.get("damper_zero_offset_rad", [0.0] * 6), dtype=np.float64)

        self.init_main_drive_pos = np.asarray(self.cfg.get("init_main_drive_pos", C.INIT_MAIN_DRIVE_POS), dtype=np.float64)
        self.init_abad_pos = np.asarray(self.cfg.get("init_abad_pos", C.INIT_ABAD_POS), dtype=np.float64)
        self.init_damper_pos = np.asarray(self.cfg.get("init_damper_pos", C.INIT_DAMPER_POS), dtype=np.float64)

        self.main_kp = np.asarray(self.cfg.get("main_drive_kp", [0.0] * 6), dtype=np.float64)
        self.main_kd = np.asarray(self.cfg.get("main_drive_kd", [50.0] * 6), dtype=np.float64)
        self.stand_main_kp = np.asarray(self.cfg.get("stand_main_drive_kp", [12.0] * 6), dtype=np.float64)
        self.stand_main_kd = np.asarray(self.cfg.get("stand_main_drive_kd", [1.0] * 6), dtype=np.float64)
        self.init_stand_main_drive_position_gain = float(
            self.cfg.get("init_stand_main_drive_position_gain", 3.0)
        )
        self.init_stand_max_main_drive_vel = float(self.cfg.get("init_stand_max_main_drive_vel_rad_s", 1.5))
        self.abad_kp = np.asarray(self.cfg.get("abad_kp", [40.0] * 6), dtype=np.float64)
        self.abad_kd = np.asarray(self.cfg.get("abad_kd", [4.0] * 6), dtype=np.float64)
        self.damper_kp = np.asarray(self.cfg.get("damper_kp", [200.0] * 6), dtype=np.float64)
        self.damper_kd = np.asarray(self.cfg.get("damper_kd", [20.0] * 6), dtype=np.float64)
        self.main_effort_limit = np.asarray(self.cfg.get("main_drive_effort_limit_nm", [100.0] * 6), dtype=np.float64)
        self.abad_effort_limit = np.asarray(self.cfg.get("abad_effort_limit_nm", [8.0] * 6), dtype=np.float64)
        self.damper_effort_limit = np.asarray(self.cfg.get("damper_effort_limit_nm", [50.0] * 6), dtype=np.float64)

        self.action_clip = float(self.cfg.get("action_clip", 1.0))
        self.base_gait_angular_vel = float(self.cfg.get("base_gait_angular_vel", C.BASE_GAIT_ANGULAR_VEL))
        self.base_gait_frequency_hz = float(self.cfg.get("base_gait_frequency_hz", C.BASE_GAIT_FREQUENCY_HZ))
        self.stance_velocity = float(self.cfg.get("stance_velocity", C.STANCE_VELOCITY))
        self.swing_velocity = float(self.cfg.get("swing_velocity", C.SWING_VELOCITY))
        self.stance_phase_start = float(self.cfg.get("stance_phase_start", C.STANCE_PHASE_START))
        self.stance_phase_end = float(self.cfg.get("stance_phase_end", C.STANCE_PHASE_END))

        self.drive_vel_scale = float(self.cfg.get("drive_vel_scale", C.STAGE_DRIVE_VEL_SCALE))
        self.base_residual_scale = float(
            self.cfg.get("main_drive_residual_scale", C.STAGE_MAIN_DRIVE_RESIDUAL_SCALE)
        )
        self.forward_residual_scale = float(
            self.cfg.get("forward_policy_drive_residual_scale", C.STAGE_FORWARD_POLICY_DRIVE_RESIDUAL_SCALE)
        )
        self.diag_residual_scale = float(
            self.cfg.get("diag_policy_drive_residual_scale", C.STAGE_DIAG_POLICY_DRIVE_RESIDUAL_SCALE)
        )
        self.yaw_residual_scale = float(
            self.cfg.get("yaw_policy_drive_residual_scale", C.STAGE_YAW_POLICY_DRIVE_RESIDUAL_SCALE)
        )
        self.forward_bias_scale = float(self.cfg.get("forward_bias_scale", C.STAGE_FORWARD_BIAS_SCALE))
        self.yaw_drive_bias_scale = float(self.cfg.get("yaw_drive_bias_scale", C.STAGE_YAW_DRIVE_BIAS_SCALE))
        self.yaw_body_pattern_sign = float(self.cfg.get("yaw_body_pattern_sign", 1.0))
        self.yaw_stability_tilt_limit = float(self.cfg.get("yaw_stability_tilt_limit", 0.38))
        self.yaw_safe_min_scale = float(self.cfg.get("yaw_safe_min_scale", C.STAGE_YAW_SAFE_MIN_SCALE))
        self.yaw_hard_brake_tilt = float(self.cfg.get("yaw_hard_brake_tilt", C.STAGE_YAW_HARD_BRAKE_TILT))
        self.yaw_hard_brake_scale = float(self.cfg.get("yaw_hard_brake_scale", C.STAGE_YAW_HARD_BRAKE_SCALE))
        self.forward_phase_lock_gain = float(self.cfg.get("forward_phase_lock_gain", 1.2))
        self.forward_residual_cap_ratio = float(
            self.cfg.get("forward_residual_cap_ratio", C.STAGE_FORWARD_RESIDUAL_CAP_RATIO)
        )
        # Exact play/eval compatibility branch used by the simulator.  These
        # values are deployment-contract inputs and are therefore hashed.
        self.play_forward_compat_enable = bool(
            self.cfg.get("play_forward_compat_enable", False)
        )
        self.play_forward_compat_bias_scale = float(
            self.cfg.get("play_forward_compat_bias_scale", 1.0)
        )
        self.play_forward_compat_residual_scale = float(
            self.cfg.get("play_forward_compat_residual_scale", 0.04)
        )
        self.play_forward_compat_residual_clip = float(
            self.cfg.get("play_forward_compat_residual_clip", 0.30)
        )

        self.abad_pos_scale = float(self.cfg.get("abad_pos_scale", C.ABAD_POS_SCALE))
        self.abad_pos_limit = float(self.cfg.get("abad_pos_limit", C.STAGE_ABAD_POS_LIMIT))
        self.lateral_gait_frequency_hz = float(self.cfg.get("lateral_gait_frequency_hz", 0.5))
        self.require_stand_before_lateral = bool(self.cfg.get("require_stand_before_lateral", True))
        self.lock_main_drive_in_lateral = bool(self.cfg.get("lock_main_drive_in_lateral", True))
        self.lateral_soft_lock_enable = bool(self.cfg.get("lateral_soft_lock_enable", True))
        self.lateral_soft_lock_velocity = float(
            self.cfg.get("lateral_soft_lock_velocity", C.STAGE_LATERAL_SOFT_LOCK_VELOCITY)
        )
        self.lateral_policy_drive_residual_scale = float(
            self.cfg.get("lateral_policy_drive_residual_scale", C.STAGE_LATERAL_POLICY_DRIVE_RESIDUAL_SCALE)
        )
        self.lateral_stand_pos_tol = float(self.cfg.get("lateral_stand_pos_tol", 0.12))
        self.lateral_contact_pose_tol = float(self.cfg.get("lateral_contact_pose_tol", 0.18))
        self.lateral_min_contact_count = float(self.cfg.get("lateral_min_contact_count", 6.0))
        self.lateral_go_to_stand_timeout_s = float(self.cfg.get("lateral_go_to_stand_timeout_s", 1.5))
        self.lateral_timeout_cooldown_steps = int(self.cfg.get("lateral_timeout_cooldown_steps", 80))
        self.lateral_abad_base_amplitude = float(
            self.cfg.get("lateral_abad_base_amplitude", C.STAGE_LATERAL_ABAD_BASE_AMPLITUDE)
        )
        self.lateral_abad_max_amplitude = float(
            self.cfg.get("lateral_abad_max_amplitude", C.STAGE_LATERAL_ABAD_MAX_AMPLITUDE)
        )
        self.lateral_abad_policy_blend = float(
            self.cfg.get("lateral_abad_policy_blend", C.STAGE_LATERAL_ABAD_POLICY_BLEND)
        )
        self.diag_abad_bias_scale = float(self.cfg.get("diag_abad_bias_scale", C.STAGE_DIAG_ABAD_BIAS_SCALE))
        self.diag_abad_policy_blend = float(
            self.cfg.get("diag_abad_policy_blend", C.STAGE_DIAG_ABAD_POLICY_BLEND)
        )
        self.yaw_abad_action_scale = float(self.cfg.get("yaw_abad_action_scale", C.STAGE_YAW_ABAD_ACTION_SCALE))
        self.yaw_abad_stance_bias = float(self.cfg.get("yaw_abad_stance_bias", C.STAGE_YAW_ABAD_STANCE_BIAS))
        self.yaw_abad_policy_blend = float(self.cfg.get("yaw_abad_policy_blend", C.STAGE_YAW_ABAD_POLICY_BLEND))

        self.action_warmup_steps = int(self.cfg.get("action_warmup_steps", C.STAGE_ACTION_WARMUP_STEPS))
        self.main_drive_vel_limit = float(self.cfg.get("main_drive_vel_limit_rad_s", 30.0))
        self.main_drive_slew_rate = float(self.cfg.get("main_drive_slew_rate_rad_s2", 120.0))
        self.abad_slew_rate = float(self.cfg.get("abad_slew_rate_rad_s", 6.0))

        self.step_count = 0
        self.gait_phase = 0.0
        self.lateral_gait_phase = 0.0
        self.lateral_fsm_state = 0
        self.lateral_state_time = 0.0
        self.lateral_timeout_cooldown = 0
        self.prev_target_drive_vel = np.zeros(6, dtype=np.float64)
        self.prev_target_abad_pos = np.zeros(6, dtype=np.float64)
        self._validate_config()

    def reset(self, gait_phase: float = 0.0) -> None:
        self.step_count = 0
        self.gait_phase = float(gait_phase) % (2.0 * math.pi)
        self.lateral_gait_phase = 0.0
        self.lateral_fsm_state = 0
        self.lateral_state_time = 0.0
        self.lateral_timeout_cooldown = 0
        self.prev_target_drive_vel[:] = 0.0
        self.prev_target_abad_pos[:] = 0.0

    def _validate_config(self) -> None:
        for name, joint_names in (
            ("main_drive_joint_names", self.main_drive_joint_names),
            ("abad_joint_names", self.abad_joint_names),
            ("damper_joint_names", self.damper_joint_names),
        ):
            if len(joint_names) != 6:
                raise ValueError(f"{name} must contain 6 joints, got {len(joint_names)}")
            if len(set(joint_names)) != len(joint_names):
                raise ValueError(f"{name} contains duplicate names: {joint_names}")

        vectors = {
            "leg_direction_multiplier": self.direction_multiplier,
            "main_drive_sign": self.main_sign,
            "abad_sign": self.abad_sign,
            "damper_sign": self.damper_sign,
            "main_drive_zero_offset_rad": self.main_zero_offset,
            "abad_zero_offset_rad": self.abad_zero_offset,
            "damper_zero_offset_rad": self.damper_zero_offset,
            "init_main_drive_pos": self.init_main_drive_pos,
            "init_abad_pos": self.init_abad_pos,
            "init_damper_pos": self.init_damper_pos,
            "main_drive_kp": self.main_kp,
            "main_drive_kd": self.main_kd,
            "stand_main_drive_kp": self.stand_main_kp,
            "stand_main_drive_kd": self.stand_main_kd,
            "abad_kp": self.abad_kp,
            "abad_kd": self.abad_kd,
            "damper_kp": self.damper_kp,
            "damper_kd": self.damper_kd,
            "main_drive_effort_limit_nm": self.main_effort_limit,
            "abad_effort_limit_nm": self.abad_effort_limit,
            "damper_effort_limit_nm": self.damper_effort_limit,
        }
        for name, values in vectors.items():
            if values.shape != (6,):
                raise ValueError(f"{name} must have length 6, got shape {values.shape}")
            if not np.isfinite(values).all():
                raise ValueError(f"{name} contains NaN or Inf")

        for name, values in (("main_drive_sign", self.main_sign), ("abad_sign", self.abad_sign), ("damper_sign", self.damper_sign)):
            if not np.all(np.isin(values, [-1.0, 1.0])):
                raise ValueError(f"{name} must contain only -1.0 or 1.0")

        if self.action_clip <= 0.0:
            raise ValueError("action_clip must be positive")
        if self.main_drive_vel_limit <= 0.0:
            raise ValueError("main_drive_vel_limit_rad_s must be positive")
        if self.abad_pos_limit <= 0.0:
            raise ValueError("abad_pos_limit must be positive")
        if self.main_drive_slew_rate <= 0.0 or self.abad_slew_rate <= 0.0:
            raise ValueError("slew-rate limits must be positive")
        if self.main_drive_init_control_mode not in ("velocity_to_pose", "position"):
            raise ValueError("main_drive_init_control_mode must be 'velocity_to_pose' or 'position'")
        if self.init_stand_main_drive_position_gain <= 0.0 or self.init_stand_max_main_drive_vel <= 0.0:
            raise ValueError("INIT_STAND main-drive velocity-to-pose gains must be positive")
        compat_values = (
            self.play_forward_compat_bias_scale,
            self.play_forward_compat_residual_scale,
            self.play_forward_compat_residual_clip,
        )
        if not np.isfinite(compat_values).all():
            raise ValueError("play_forward_compat parameters must be finite")
        if self.play_forward_compat_bias_scale < 0.0:
            raise ValueError("play_forward_compat_bias_scale must be >= 0")
        if self.play_forward_compat_residual_scale < 0.0:
            raise ValueError("play_forward_compat_residual_scale must be >= 0")
        if self.play_forward_compat_residual_clip <= 0.0:
            raise ValueError("play_forward_compat_residual_clip must be > 0")

    @staticmethod
    def _resolve_command_modes(command: np.ndarray) -> tuple[bool, bool, bool, bool, int]:
        cmd_vx, cmd_vy, cmd_wz = command
        lin_zero = 0.08
        yaw_zero = 0.10
        mode_fwd = cmd_vx > 0.10 and abs(cmd_vy) < lin_zero and abs(cmd_wz) < yaw_zero
        mode_lat = abs(cmd_vx) < lin_zero and abs(cmd_vy) > 0.12 and abs(cmd_wz) < yaw_zero
        mode_diag = cmd_vx > 0.10 and abs(cmd_vy) > 0.10 and abs(cmd_wz) < yaw_zero
        mode_yaw = abs(cmd_vx) < lin_zero and abs(cmd_vy) < lin_zero and abs(cmd_wz) > 0.15
        if mode_fwd:
            return mode_fwd, mode_lat, mode_diag, mode_yaw, 0
        if mode_lat:
            return mode_fwd, mode_lat, mode_diag, mode_yaw, 1
        if mode_diag:
            return mode_fwd, mode_lat, mode_diag, mode_yaw, 2
        if mode_yaw:
            return mode_fwd, mode_lat, mode_diag, mode_yaw, 3
        return mode_fwd, mode_lat, mode_diag, mode_yaw, 4

    def _in_stance_phase(self, phase: np.ndarray) -> np.ndarray:
        if self.stance_phase_start < 0.0:
            start = self.stance_phase_start + 2.0 * math.pi
            return np.logical_or(phase >= start, phase < self.stance_phase_end)
        return np.logical_and(phase >= self.stance_phase_start, phase < self.stance_phase_end)

    @staticmethod
    def _phase_error(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        return np.arctan2(np.sin(a - b), np.cos(a - b))

    def _warmup_scale(self) -> float:
        if self.action_warmup_steps <= 0:
            return 1.0
        return float(np.clip((self.step_count + 1.0) / float(self.action_warmup_steps), 0.0, 1.0))

    def init_stand_command(self, current_main_pos: np.ndarray | None = None, enable: bool = True) -> DecodedMotorCommand:
        target_main_vel = np.zeros(6, dtype=np.float64)
        if self.main_drive_init_control_mode == "velocity_to_pose" and current_main_pos is not None:
            current_main_pos = np.asarray(current_main_pos, dtype=np.float64).reshape(6)
            pos_error = np.arctan2(
                np.sin(self.init_main_drive_pos - current_main_pos),
                np.cos(self.init_main_drive_pos - current_main_pos),
            )
            target_main_vel = np.clip(
                self.init_stand_main_drive_position_gain * pos_error,
                -self.init_stand_max_main_drive_vel,
                self.init_stand_max_main_drive_vel,
            )
            target_main_vel[~self.healthy_leg_mask] = 0.0
            main_pos = self._apply_position_hooks(current_main_pos, self.main_sign, self.main_zero_offset)
            main_kp = self.main_kp
            main_kd = self.stand_main_kd
        else:
            main_pos = self._apply_position_hooks(self.init_main_drive_pos, self.main_sign, self.main_zero_offset)
            main_kp = self.stand_main_kp
            main_kd = self.stand_main_kd
        abad_pos = self._apply_position_hooks(self.init_abad_pos, self.abad_sign, self.abad_zero_offset)
        damper_pos = self._apply_position_hooks(self.init_damper_pos, self.damper_sign, self.damper_zero_offset)
        return self._pack_command(
            main_position=main_pos,
            main_velocity=self._apply_velocity_hooks(target_main_vel, self.main_sign),
            abad_position=abad_pos,
            damper_position=damper_pos,
            enable=enable,
            mode=self.MODE_INIT_STAND,
            safe_action=np.zeros(C.ACTION_DIM, dtype=np.float32),
            target_main_drive_velocity=target_main_vel,
            target_abad_position=self.init_abad_pos.copy(),
            main_kp=main_kp,
            main_kd=main_kd,
        )

    def disabled_command(self) -> DecodedMotorCommand:
        return self._pack_command(
            main_position=np.zeros(6),
            main_velocity=np.zeros(6),
            abad_position=np.zeros(6),
            damper_position=self._apply_position_hooks(self.init_damper_pos, self.damper_sign, self.damper_zero_offset),
            enable=False,
            mode=self.MODE_DISABLED,
            safe_action=np.zeros(C.ACTION_DIM, dtype=np.float32),
            target_main_drive_velocity=np.zeros(6),
            target_abad_position=np.zeros(6),
        )

    def protective_stop_command(self, current_main_pos: np.ndarray, current_abad_pos: np.ndarray) -> DecodedMotorCommand:
        main_pos = self._apply_position_hooks(current_main_pos, self.main_sign, self.main_zero_offset)
        abad_pos = self._apply_position_hooks(current_abad_pos, self.abad_sign, self.abad_zero_offset)
        return self._pack_command(
            main_position=main_pos,
            main_velocity=np.zeros(6),
            abad_position=abad_pos,
            damper_position=self._apply_position_hooks(self.init_damper_pos, self.damper_sign, self.damper_zero_offset),
            enable=False,
            mode=self.MODE_PROTECTIVE_STOP,
            safe_action=np.zeros(C.ACTION_DIM, dtype=np.float32),
            target_main_drive_velocity=np.zeros(6),
            target_abad_position=current_abad_pos.copy(),
        )

    def decode(
        self,
        action: np.ndarray,
        main_drive_pos: np.ndarray,
        abad_pos: np.ndarray,
        command: np.ndarray,
        projected_gravity: np.ndarray,
        dt: float,
        gait_phase: float | None = None,
    ) -> DecodedMotorCommand:
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (C.ACTION_DIM,):
            raise ValueError(f"action must be ({C.ACTION_DIM},), got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("action contains NaN or Inf")

        action = np.clip(action, -self.action_clip, self.action_clip)
        raw_drive_actions = action[:6].astype(np.float64)
        raw_abad_actions = action[6:12].astype(np.float64)
        command = np.asarray(command, dtype=np.float64).reshape(3)

        if gait_phase is None:
            self.gait_phase = (self.gait_phase + 2.0 * math.pi * self.base_gait_frequency_hz * dt) % (2.0 * math.pi)
        else:
            self.gait_phase = float(gait_phase) % (2.0 * math.pi)

        mode_fwd, mode_lat, mode_diag, mode_yaw, _ = self._resolve_command_modes(command)
        masked_drive_actions = raw_drive_actions.copy()
        masked_abad_actions = raw_abad_actions.copy()
        if mode_lat:
            masked_drive_actions[:] = 0.0
        if mode_fwd:
            masked_abad_actions[:] = 0.0

        main_drive_pos = np.asarray(main_drive_pos, dtype=np.float64).reshape(6)
        effective_pos = main_drive_pos * self.direction_multiplier
        leg_phase = np.remainder(effective_pos, 2.0 * math.pi)
        desired_phase = np.remainder(self.gait_phase + self.leg_phase_offsets, 2.0 * math.pi)
        desired_in_stance = self._in_stance_phase(desired_phase)
        forward_base_velocity = np.where(desired_in_stance, self.stance_velocity, self.swing_velocity)
        phase_correction = np.clip(-self.forward_phase_lock_gain * self._phase_error(leg_phase, desired_phase), -2.0, 2.0)
        forward_profile = forward_base_velocity + phase_correction if (mode_fwd or mode_diag) else np.zeros(6)

        cmd_vx, cmd_vy, cmd_wz = command
        vx_norm = float(np.clip(cmd_vx / max(float(self.cfg.get("drive_bias_vx_ref", 0.45)), 1.0e-3), -1.0, 1.0))
        wz_norm = float(np.clip(cmd_wz / max(float(self.cfg.get("drive_bias_wz_ref", 1.0)), 1.0e-3), -1.0, 1.0))
        forward_bias_joint = forward_profile * self.direction_multiplier * vx_norm * self.forward_bias_scale

        yaw_body_pattern = np.array([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]) * self.yaw_body_pattern_sign
        yaw_phase_gain = np.where(desired_in_stance, C.STANCE_VELOCITY_RATIO, C.SWING_VELOCITY_RATIO)
        yaw_bias_body = yaw_body_pattern * (wz_norm * self.yaw_drive_bias_scale) * yaw_phase_gain
        yaw_bias_joint = yaw_bias_body * self.direction_multiplier if mode_yaw else np.zeros(6)

        reference_projected_gravity = np.asarray(self.cfg.get("reference_projected_gravity", [0.0, -1.0, 0.0]), dtype=np.float64)
        pg = np.asarray(projected_gravity, dtype=np.float64).reshape(3)
        denom = max(np.linalg.norm(pg) * np.linalg.norm(reference_projected_gravity), 1.0e-6)
        gravity_alignment = float(np.clip(np.dot(pg, reference_projected_gravity) / denom, -1.0, 1.0))
        roll_pitch_rms = math.acos(gravity_alignment)
        yaw_safe_scale = 1.0
        if mode_yaw:
            yaw_safe_scale = float(np.clip(1.0 - roll_pitch_rms / max(self.yaw_stability_tilt_limit, 1.0e-3), self.yaw_safe_min_scale, 1.0))
            yaw_bias_joint *= yaw_safe_scale

        residual_scale = self.base_residual_scale
        if mode_fwd:
            residual_scale = self.forward_residual_scale
        elif mode_diag:
            residual_scale = self.diag_residual_scale
        elif mode_yaw:
            residual_scale = self.yaw_residual_scale
        drive_residual = masked_drive_actions * self.drive_vel_scale * residual_scale
        if mode_yaw:
            drive_residual *= yaw_safe_scale
        if mode_fwd:
            cap = np.maximum(np.abs(forward_bias_joint) * self.forward_residual_cap_ratio, 0.08)
            drive_residual = np.clip(drive_residual, -cap, cap)

        warmup = self._warmup_scale()
        target_drive_vel = (forward_bias_joint + yaw_bias_joint + drive_residual) * warmup
        # Keep this block in the same location and use the same equations as
        # RedrhexEnv._apply_action(): after normal residual construction and
        # before the common velocity clamp.
        pure_forward = mode_fwd and not mode_lat and not mode_diag and not mode_yaw
        if self.play_forward_compat_enable and pure_forward:
            compat_drive_bias = (
                forward_profile
                * self.direction_multiplier
                * vx_norm
                * self.play_forward_compat_bias_scale
            )
            compat_drive_residual = (
                np.clip(
                    masked_drive_actions,
                    -self.play_forward_compat_residual_clip,
                    self.play_forward_compat_residual_clip,
                )
                * self.drive_vel_scale
                * self.play_forward_compat_residual_scale
            )
            target_drive_vel = (compat_drive_bias + compat_drive_residual) * warmup
        max_vel = max(self.swing_velocity * 1.5, self.drive_vel_scale * 1.5)
        target_drive_vel = np.clip(target_drive_vel, -max_vel, max_vel)

        yaw_hard_brake = mode_yaw and roll_pitch_rms > self.yaw_hard_brake_tilt
        if yaw_hard_brake:
            target_drive_vel *= self.yaw_hard_brake_scale

        final_drive_vel = target_drive_vel.copy()
        ready_lateral = False
        if mode_lat:
            if self.lateral_timeout_cooldown > 0:
                self.lateral_timeout_cooldown -= 1
            self.lateral_state_time += dt
            pos_error = np.arctan2(np.sin(main_drive_pos - self.init_main_drive_pos), np.cos(main_drive_pos - self.init_main_drive_pos))
            healthy_error = np.abs(pos_error[self.healthy_leg_mask])
            pose_error = float(np.max(healthy_error))
            contact_count = float(np.sum(healthy_error < self.lateral_contact_pose_tol))
            required_contact_count = min(self.lateral_min_contact_count, float(np.sum(self.healthy_leg_mask)))
            if self.lateral_fsm_state == 0 and self.require_stand_before_lateral and self.lateral_timeout_cooldown == 0:
                self.lateral_fsm_state = 1
                self.lateral_state_time = 0.0
            if not self.require_stand_before_lateral:
                self.lateral_fsm_state = 2

            if self.lateral_fsm_state == 1:
                if pose_error < self.lateral_stand_pos_tol and contact_count >= required_contact_count:
                    self.lateral_fsm_state = 2
                    self.lateral_state_time = 0.0
                elif self.lateral_state_time > self.lateral_go_to_stand_timeout_s:
                    self.lateral_fsm_state = 0
                    self.lateral_state_time = 0.0
                    self.lateral_timeout_cooldown = self.lateral_timeout_cooldown_steps
                else:
                    final_drive_vel = np.clip(-3.0 * pos_error, -2.0, 2.0)

            ready_lateral = self.lateral_fsm_state == 2
            if ready_lateral:
                self.lateral_gait_phase = (self.lateral_gait_phase + 2.0 * math.pi * self.lateral_gait_frequency_hz * dt) % (2.0 * math.pi)
                if self.lock_main_drive_in_lateral:
                    if self.lateral_soft_lock_enable:
                        clearance_wave = np.sin(self.lateral_gait_phase + self.leg_phase_offsets)
                        soft_drive = self.lateral_soft_lock_velocity * clearance_wave * self.direction_multiplier
                        soft_drive += (
                            self.lateral_policy_drive_residual_scale
                            * self.lateral_soft_lock_velocity
                            * raw_drive_actions
                            * self.direction_multiplier
                        )
                        final_drive_vel = np.clip(
                            soft_drive,
                            -1.8 * self.lateral_soft_lock_velocity,
                            1.8 * self.lateral_soft_lock_velocity,
                        )
                    else:
                        final_drive_vel[:] = 0.0
        else:
            self.lateral_gait_phase = 0.0
            self.lateral_fsm_state = 0
            self.lateral_state_time = 0.0
            self.lateral_timeout_cooldown = 0

        base_abad_pos = masked_abad_actions * self.abad_pos_scale
        target_abad_pos = base_abad_pos.copy()
        if mode_fwd:
            target_abad_pos[:] = 0.0
        if mode_lat and self.lateral_fsm_state == 1:
            target_abad_pos[:] = 0.0
        if mode_lat and ready_lateral:
            lateral_dir = math.copysign(1.0, cmd_vy) if abs(cmd_vy) > 1.0e-6 else 0.0
            phase_sin = math.sin(self.lateral_gait_phase)
            vy_ratio = float(np.clip(abs(cmd_vy) / 0.12, 0.0, 1.5))
            amp = self.lateral_abad_base_amplitude + (self.lateral_abad_max_amplitude - self.lateral_abad_base_amplitude) * min(vy_ratio, 1.0)
            right = -lateral_dir * phase_sin * amp
            left = lateral_dir * phase_sin * amp
            lateral_abad = np.array([right, right, right, left, left, left], dtype=np.float64)
            blend = float(np.clip(self.lateral_abad_policy_blend, 0.0, 1.0))
            target_abad_pos = (1.0 - blend) * lateral_abad + blend * base_abad_pos
        if mode_diag:
            diag_dir = math.copysign(1.0, cmd_vy) if abs(cmd_vy) > 1.0e-6 else 0.0
            vy_ratio = float(np.clip(abs(cmd_vy) / 0.12, 0.0, 1.5))
            amp = self.diag_abad_bias_scale * min(vy_ratio, 1.0)
            diag_bias = np.array([-diag_dir * amp] * 3 + [diag_dir * amp] * 3, dtype=np.float64)
            blend = float(np.clip(self.diag_abad_policy_blend, 0.0, 1.0))
            target_abad_pos = blend * base_abad_pos + (1.0 - blend) * diag_bias
        if mode_yaw:
            target_abad_pos *= float(np.clip(self.yaw_abad_action_scale, 0.0, 1.0))
            if self.yaw_abad_stance_bias > 1.0e-6:
                stance_target = np.array([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]) * self.yaw_abad_stance_bias
                blend = float(np.clip(self.yaw_abad_policy_blend, 0.0, 1.0))
                target_abad_pos = blend * target_abad_pos + (1.0 - blend) * stance_target
        if yaw_hard_brake:
            target_abad_pos *= self.yaw_hard_brake_scale

        target_abad_pos *= warmup
        target_abad_pos = np.clip(target_abad_pos, -self.abad_pos_limit, self.abad_pos_limit)

        max_dv = self.main_drive_slew_rate * max(dt, 0.0)
        final_drive_vel = np.clip(final_drive_vel, self.prev_target_drive_vel - max_dv, self.prev_target_drive_vel + max_dv)
        max_dp = self.abad_slew_rate * max(dt, 0.0)
        target_abad_pos = np.clip(target_abad_pos, self.prev_target_abad_pos - max_dp, self.prev_target_abad_pos + max_dp)

        final_drive_vel = np.clip(final_drive_vel, -self.main_drive_vel_limit, self.main_drive_vel_limit)
        final_drive_vel[~self.healthy_leg_mask] = 0.0
        target_abad_pos[~self.healthy_leg_mask] = 0.0
        self.prev_target_drive_vel = final_drive_vel.copy()
        self.prev_target_abad_pos = target_abad_pos.copy()
        self.step_count += 1

        main_cmd_pos = self._apply_position_hooks(main_drive_pos, self.main_sign, self.main_zero_offset)
        main_cmd_vel = self._apply_velocity_hooks(final_drive_vel, self.main_sign)
        abad_cmd_pos = self._apply_position_hooks(target_abad_pos, self.abad_sign, self.abad_zero_offset)
        damper_cmd_pos = self._apply_position_hooks(self.init_damper_pos, self.damper_sign, self.damper_zero_offset)

        return self._pack_command(
            main_position=main_cmd_pos,
            main_velocity=main_cmd_vel,
            abad_position=abad_cmd_pos,
            damper_position=damper_cmd_pos,
            enable=True,
            mode=self.MODE_MIXED_POSITION_VELOCITY,
            safe_action=action.astype(np.float32),
            target_main_drive_velocity=final_drive_vel,
            target_abad_position=target_abad_pos,
        )

    @staticmethod
    def _apply_position_hooks(values: np.ndarray, sign: np.ndarray, offset: np.ndarray) -> np.ndarray:
        return sign * np.asarray(values, dtype=np.float64) + offset

    @staticmethod
    def _apply_velocity_hooks(values: np.ndarray, sign: np.ndarray) -> np.ndarray:
        return sign * np.asarray(values, dtype=np.float64)

    def _pack_command(
        self,
        main_position: np.ndarray,
        main_velocity: np.ndarray,
        abad_position: np.ndarray,
        damper_position: np.ndarray,
        enable: bool,
        mode: int,
        safe_action: np.ndarray,
        target_main_drive_velocity: np.ndarray,
        target_abad_position: np.ndarray,
        main_kp: np.ndarray | None = None,
        main_kd: np.ndarray | None = None,
        abad_kp: np.ndarray | None = None,
        abad_kd: np.ndarray | None = None,
    ) -> DecodedMotorCommand:
        main_position = np.asarray(main_position, dtype=np.float64).copy()
        main_velocity = np.asarray(main_velocity, dtype=np.float64).copy()
        abad_position = np.asarray(abad_position, dtype=np.float64).copy()
        damper_position = np.asarray(damper_position, dtype=np.float64).copy()
        main_kp_values = np.asarray(self.main_kp if main_kp is None else main_kp, dtype=np.float64).copy()
        main_kd_values = np.asarray(self.main_kd if main_kd is None else main_kd, dtype=np.float64).copy()
        abad_kp_values = np.asarray(self.abad_kp if abad_kp is None else abad_kp, dtype=np.float64).copy()
        abad_kd_values = np.asarray(self.abad_kd if abad_kd is None else abad_kd, dtype=np.float64).copy()
        main_effort = self.main_effort_limit.copy()
        abad_effort = self.abad_effort_limit.copy()
        damper_kp_values = self.damper_kp.copy()
        damper_kd_values = self.damper_kd.copy()
        damper_effort = self.damper_effort_limit.copy()
        safe_action = np.asarray(safe_action, dtype=np.float32).copy()
        target_main_drive_velocity = np.asarray(target_main_drive_velocity, dtype=np.float64).copy()
        target_abad_position = np.asarray(target_abad_position, dtype=np.float64).copy()
        for index in self.disabled_leg_indices:
            main_velocity[index] = 0.0
            main_kp_values[index] = 0.0
            main_kd_values[index] = 0.0
            abad_kp_values[index] = 0.0
            abad_kd_values[index] = 0.0
            main_effort[index] = 0.0
            abad_effort[index] = 0.0
            damper_effort[index] = 0.0
            damper_kp_values[index] = 0.0
            damper_kd_values[index] = 0.0
            target_main_drive_velocity[index] = 0.0
            target_abad_position[index] = 0.0
            if safe_action.shape == (C.ACTION_DIM,):
                safe_action[index] = 0.0
                safe_action[6 + index] = 0.0

        joint_names = self.main_drive_joint_names + self.abad_joint_names
        pos = list(main_position) + list(abad_position)
        vel = list(main_velocity) + [0.0] * 6
        kp = list(main_kp_values) + list(abad_kp_values)
        kd = list(main_kd_values) + list(abad_kd_values)
        effort = list(main_effort) + list(abad_effort)
        if self.include_damper_command:
            joint_names += self.damper_joint_names
            pos += list(damper_position)
            vel += [0.0] * 6
            kp += list(damper_kp_values)
            kd += list(damper_kd_values)
            effort += list(damper_effort)
        return DecodedMotorCommand(
            joint_names=joint_names,
            target_position_rad=[float(x) for x in pos],
            target_velocity_rad_s=[float(x) for x in vel],
            kp=[float(x) for x in kp],
            kd=[float(x) for x in kd],
            effort_limit_nm=[float(x) for x in effort],
            enable=bool(enable),
            mode=int(mode),
            safe_action=safe_action.copy(),
            target_main_drive_velocity=target_main_drive_velocity.copy(),
            target_abad_position=target_abad_position.copy(),
        )
