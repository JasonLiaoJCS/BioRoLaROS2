"""Constants mirrored from the IsaacLab RedRhex task.

Source of truth inspected in:
source/RedRhex/RedRhex/tasks/direct/redrhex/redrhex_env.py
source/RedRhex/RedRhex/tasks/direct/redrhex/redrhex_env_cfg.py
"""

from __future__ import annotations

import math


OBS_DIM_SINGLE = 56
ACTION_DIM = 12
# RedrhexEnv._pre_physics_step() clamps the policy output before storing it in
# ``self.actions``. DirectRLEnv observes only after applying the new action, so
# indices 44:56 contain the action from two policy inferences earlier.
TRAINING_ACTION_CLIP = 1.0
POLICY_HISTORY_LENGTH = 5

# These identifiers describe semantics, not only tensor shapes.  A 56 -> 12
# network trained with a different action decoder is not interchangeable.
OBSERVATION_CONTRACT_ID = "redrhex_obs56_v2_action_lag2"
ACTION_DECODER_CONTRACT_ID = "redrhex_stage5_residual_v1"
ONNX_OBSERVATION_CONTRACT_KEY = "redrhex.observation_contract"
ONNX_ACTION_CONTRACT_KEY = "redrhex.action_contract"
ONNX_INPUT_LAYOUT_KEY = "redrhex.policy_input_layout"
ONNX_NORMALIZER_KEY = "redrhex.normalizer"
ONNX_TRAINING_ACTION_CLIP_KEY = "redrhex.training_action_clip"
ONNX_DECODER_SOURCE_SHA256_KEY = "redrhex.action_decoder_source_sha256"
ONNX_DECODER_CONFIG_SHA256_KEY = "redrhex.decoder_config_sha256"
ONNX_GOLDEN_SCHEMA_KEY = "redrhex.golden_schema"
ONNX_TRAINING_GIT_SHA_KEY = "redrhex.training_git_sha"
ONNX_CONTROLLER_CONFIG_SHA256_KEY = "redrhex.controller_config_sha256"
ONNX_OBSERVATION_SOURCE_SHA256_KEY = "redrhex.observation_source_sha256"
ONNX_DEPLOYMENT_SOURCE_SHA256_KEY = "redrhex.deployment_source_sha256"
ONNX_BRIDGE_CONFIG_SHA256_KEY = "redrhex.bridge_config_sha256"
ONNX_TRAINING_ENV_SOURCE_SHA256_KEY = "redrhex.training_env_source_sha256"
ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY = (
    "redrhex.training_env_config_source_sha256"
)
ONNX_TRAINING_PLAY_SOURCE_SHA256_KEY = "redrhex.training_play_source_sha256"
# Optional exporter status keys.  Absence remains backwards compatible, but
# when present their values are validated against an explicit approval list.
ONNX_ARTIFACT_STATUS_KEY = "redrhex_artifact_status"
ONNX_QUALITY_STATUS_KEY = "redrhex_quality_status"
POLICY_INPUT_LAYOUT_SINGLE = "single_56"
POLICY_INPUT_LAYOUT_HISTORY = "current_to_oldest_5x56"
NORMALIZER_EMBEDDED = "embedded"

SIM_DT = 1.0 / 250.0
DECIMATION = 2
CONTROL_DT = SIM_DT * DECIMATION
POLICY_HZ = 1.0 / CONTROL_DT

# The IsaacLab asset starts with a +90 deg rotation about X
# (wxyz = [0.7071068, 0.7071068, 0, 0]).  World gravity expressed in that
# policy body frame is therefore [0, -1, 0], not the ROS-identity [0, 0, -1].
# Encoder-only rig deployment must use this exact training-frame reference.
REFERENCE_PROJECTED_GRAVITY = [0.0, -1.0, 0.0]

BASE_GAIT_FREQUENCY_HZ = 1.0
BASE_GAIT_ANGULAR_VEL = 2.0 * math.pi * BASE_GAIT_FREQUENCY_HZ
STANCE_PHASE_START = -math.pi / 6.0
STANCE_PHASE_END = math.pi / 6.0
STANCE_VELOCITY_RATIO = 0.15
SWING_VELOCITY_RATIO = 1.5
STANCE_VELOCITY = BASE_GAIT_ANGULAR_VEL * STANCE_VELOCITY_RATIO
SWING_VELOCITY = BASE_GAIT_ANGULAR_VEL * SWING_VELOCITY_RATIO
TRIPOD_PHASE_OFFSET = math.pi

MAIN_DRIVE_JOINT_NAMES = [
    "Revolute_15",
    "Revolute_7",
    "Revolute_12",
    "Revolute_18",
    "Revolute_23",
    "Revolute_24",
]

ABAD_JOINT_NAMES = [
    "Revolute_14",
    "Revolute_6",
    "Revolute_11",
    "Revolute_17",
    "Revolute_22",
    "Revolute_21",
]

DAMPER_JOINT_NAMES = [
    "Revolute_5",
    "Revolute_8",
    "Revolute_13",
    "Revolute_25",
    "Revolute_26",
    "Revolute_27",
]

ALL_CONTROLLED_JOINT_NAMES = MAIN_DRIVE_JOINT_NAMES + ABAD_JOINT_NAMES + DAMPER_JOINT_NAMES

TRIPOD_A_LEG_INDICES = [0, 3, 5]
TRIPOD_B_LEG_INDICES = [1, 2, 4]
LEG_DIRECTION_MULTIPLIER = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]

INIT_MAIN_DRIVE_POS = [
    45.0 * math.pi / 180.0,
    45.0 * math.pi / 180.0,
    45.0 * math.pi / 180.0,
    -45.0 * math.pi / 180.0,
    -45.0 * math.pi / 180.0,
    -45.0 * math.pi / 180.0,
]
INIT_ABAD_POS = [0.0] * 6
INIT_DAMPER_POS = [
    45.0 * math.pi / 180.0,
    45.0 * math.pi / 180.0,
    -45.0 * math.pi / 180.0,
    45.0 * math.pi / 180.0,
    45.0 * math.pi / 180.0,
    45.0 * math.pi / 180.0,
]

MAIN_DRIVE_VEL_SCALE = 8.0
ABAD_POS_SCALE = 0.61096

# Stage-5 deployment defaults from the inspected training config.
STAGE_ID = 5
STAGE_DRIVE_VEL_SCALE = 6.8
STAGE_MAIN_DRIVE_RESIDUAL_SCALE = 0.20
STAGE_FORWARD_BIAS_SCALE = 0.90
STAGE_YAW_DRIVE_BIAS_SCALE = 1.50
STAGE_YAW_SAFE_MIN_SCALE = 0.18
STAGE_YAW_HARD_BRAKE_TILT = 0.46
STAGE_YAW_HARD_BRAKE_SCALE = 0.24
STAGE_LATERAL_SOFT_LOCK_VELOCITY = 2.3
STAGE_LATERAL_POLICY_DRIVE_RESIDUAL_SCALE = 0.30
STAGE_LATERAL_ABAD_BASE_AMPLITUDE = 0.58
STAGE_LATERAL_ABAD_MAX_AMPLITUDE = 0.86
STAGE_LATERAL_ABAD_POLICY_BLEND = 0.18
STAGE_DIAG_ABAD_BIAS_SCALE = 0.32
STAGE_DIAG_ABAD_POLICY_BLEND = 0.58
STAGE_YAW_ABAD_ACTION_SCALE = 0.46
STAGE_YAW_ABAD_STANCE_BIAS = 0.12
STAGE_YAW_ABAD_POLICY_BLEND = 0.56
STAGE_ABAD_POS_LIMIT = 0.62
STAGE_ACTION_WARMUP_STEPS = 120
STAGE_FORWARD_POLICY_DRIVE_RESIDUAL_SCALE = 0.06
STAGE_DIAG_POLICY_DRIVE_RESIDUAL_SCALE = 0.20
STAGE_YAW_POLICY_DRIVE_RESIDUAL_SCALE = 0.18
STAGE_FORWARD_RESIDUAL_CAP_RATIO = 0.22

COMMAND_LIMITS = {
    "vx_min": 0.0,
    "vx_max": 0.56,
    "vy_min": -0.60,
    "vy_max": 0.60,
    "wz_min": -0.70,
    "wz_max": 0.70,
}

OBSERVATION_SLICES = {
    "base_lin_vel": (0, 3),
    "base_ang_vel": (3, 6),
    "projected_gravity": (6, 9),
    "main_drive_pos_sin": (9, 15),
    "main_drive_pos_cos": (15, 21),
    "main_drive_vel_scaled": (21, 27),
    "abad_pos_scaled": (27, 33),
    "abad_vel": (33, 39),
    "velocity_command": (39, 42),
    "gait_phase_sin_cos": (42, 44),
    "last_actions": (44, 56),
}
