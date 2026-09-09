"""Machine-verifiable RedRhex policy and stateful decoder golden replay."""

from __future__ import annotations

import copy
import ast
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np

from . import action_decoder as action_decoder_module
from . import redrhex_contract as C
from .action_decoder import ActionDecoder
from .degraded_mode import normalize_disabled_legs, policy_indices_for_disabled_legs
from .policy_onnx_runner import PolicyONNXRunner
from .policy_validation import deployment_metadata_rejection_reasons


GOLDEN_SCHEMA = "redrhex_golden_v2"
MIN_GOLDEN_EPISODE_SAMPLES = C.STAGE_ACTION_WARMUP_STEPS + 1
MIN_GOLDEN_EPISODES = 2
MIN_GOLDEN_SAMPLES = MIN_GOLDEN_EPISODES * MIN_GOLDEN_EPISODE_SAMPLES

VECTOR_FIELDS = (
    "policy_input",
    "single_observation",
    "base_lin_vel",
    "base_ang_vel",
    "main_drive_pos",
    "main_drive_vel",
    "abad_pos",
    "abad_vel",
    "command",
    "projected_gravity",
    "dt",
    "gait_phase",
    "episode_start",
    "expected_policy_action",
    "expected_training_clipped_action",
    "expected_safe_action",
    "expected_target_main_drive_velocity",
    "expected_target_abad_position",
    "expected_target_position_rad",
    "expected_target_velocity_rad_s",
    "expected_kp",
    "expected_kd",
    "expected_effort_limit_nm",
    "expected_enable",
    "expected_mode",
    "joint_names",
)

GOLDEN_PRODUCER = "redrhex_isaaclab_play_recorder_v1"
PROVENANCE_FIELDS = (
    "producer",
    "training_git_sha",
    "training_env_source_sha256",
    "training_env_config_source_sha256",
    "training_play_source_sha256",
    "deployment_disabled_legs_csv",
    "deployment_command_profile",
    "deployment_fixed_forward_vx",
)


# Every parameter which can change the behaviour of the final BioRoLa/Rinbo
# writer must be present in an artifact-bound deployment YAML.  The canonical
# launch authorization bit is intentionally absent: it is an operational latch
# created by the validated launch path, not a site-configurable semantic value.
#
# Keep this list in sync with the Rinbo parameter declarations in
# redrhex_lowlevel_bridge/lowlevel_bridge_node.py.  The cross-package regression
# test fails if the bridge gains another runtime parameter without extending
# this schema.
_ACTIVE_BRIDGE_BACKENDS = frozenset(("biorola_ros", "rinbo_ros"))
_ACTIVE_BRIDGE_REQUIRED_PARAMETER_NAMES = frozenset(
    {
        "backend",
        "feedback_rate_hz",
        "hardware.disabled_legs",
        "hardware.max_disabled_legs",
        "rinbo.abad_command_calibrated",
        "rinbo.abad_encoder_counts_per_rad",
        "rinbo.abad_encoder_counts_per_rad_rinbo_order",
        "rinbo.abad_encoder_max",
        "rinbo.abad_encoder_min",
        "rinbo.abad_encoder_zero_rinbo_order",
        "rinbo.abad_feedback_calibrated",
        "rinbo.abad_sign_rinbo_order",
        "rinbo.allow_enable",
        "rinbo.block_if_duplicate_command_publishers",
        "rinbo.block_if_duplicate_upstream_publishers",
        "rinbo.command_timeout_s",
        "rinbo.command_topic",
        "rinbo.current_trip_samples",
        "rinbo.disabled_handshake_repeats",
        "rinbo.disabled_servo_control_mode",
        "rinbo.downstream_output_ack_timeout_s",
        "rinbo.downstream_output_ack_topic",
        "rinbo.expected_telemetry_publisher_node",
        "rinbo.expected_upstream_command_publisher_node",
        "rinbo.joint_state_topic",
        "rinbo.leg_current_channels_rinbo_order",
        "rinbo.main_direction_positive_rinbo_order",
        "rinbo.main_drive_calibrated",
        "rinbo.main_encoder_sign_rinbo_order",
        "rinbo.main_encoder_zero_counts_rinbo_order",
        "rinbo.main_max_pwm",
        "rinbo.main_position_counts_per_rev",
        "rinbo.main_pwm_per_rad_s",
        "rinbo.main_pwm_slew_rate_per_s",
        "rinbo.main_velocity_clip_rad_s",
        "rinbo.main_velocity_filter_alpha",
        "rinbo.main_velocity_max_dt_s",
        "rinbo.main_velocity_sign_policy_order",
        "rinbo.max_abad_target_position_rad",
        "rinbo.max_bus_current_a",
        "rinbo.max_bus_voltage",
        "rinbo.max_current_a",
        "rinbo.max_main_target_velocity_rad_s",
        "rinbo.min_bus_voltage",
        "rinbo.power_bus_voltage_channel",
        "rinbo.power_state_timeout_s",
        "rinbo.power_state_topic",
        "rinbo.preview_topic",
        "rinbo.publish_abad_joint_feedback",
        "rinbo.publish_preview",
        "rinbo.publish_shutdown_disable",
        "rinbo.publish_when_disabled",
        "rinbo.recovery_healthy_samples",
        "rinbo.require_abad_command_calibration",
        "rinbo.require_downstream_output_ack",
        "rinbo.require_exact_command_contract",
        "rinbo.require_main_drive_calibration",
        "rinbo.require_monotonic_telemetry_stamp",
        "rinbo.require_power_relay",
        "rinbo.require_power_state",
        "rinbo.require_single_telemetry_publisher",
        "rinbo.require_state",
        "rinbo.servo_control_mode",
        "rinbo.shutdown_disable_period_s",
        "rinbo.shutdown_disable_repeats",
        "rinbo.state_timeout_s",
        "rinbo.state_topic",
        "rinbo.stop_on_bus_current_limit",
        "rinbo.upstream_command_max_age_s",
        "rinbo.voltage_trip_samples",
    }
)


def sha256_file(path: str | Path) -> str:
    source = Path(path)
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _nested(params: dict[str, Any], *keys: str, default: Any = None) -> Any:
    current: Any = params
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def load_controller_ros_params(path: str | Path) -> dict[str, Any]:
    try:
        import yaml
    except Exception as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("PyYAML is required to load the decoder config") from exc
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    params = document.get("redrhex_rl_controller", {}).get("ros__parameters")
    if not isinstance(params, dict):
        raise ValueError(
            "decoder config must contain redrhex_rl_controller.ros__parameters"
        )
    require_complete_controller_params(params)
    return params


def load_bridge_ros_params(path: str | Path) -> dict[str, Any]:
    """Load low-level bridge parameters for an immutable semantic hash."""

    try:
        import yaml
    except Exception as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("PyYAML is required to load the bridge config") from exc
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    params = document.get("redrhex_lowlevel_bridge", {}).get("ros__parameters")
    if not isinstance(params, dict):
        raise ValueError(
            "bridge config must contain redrhex_lowlevel_bridge.ros__parameters"
        )
    return params


def required_controller_parameter_names() -> set[str]:
    """Read the node declarations so artifact tooling cannot hash a sparse YAML."""

    source = Path(__file__).with_name("rl_controller_node.py")
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id not in {"_declare_get", "_declare_string_array"}:
            continue
        if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant):
            value = node.args[1].value
            if isinstance(value, str):
                names.add(value)
    if not names:
        raise RuntimeError("failed to discover controller ROS parameter declarations")
    return names


def required_active_bridge_parameter_names() -> set[str]:
    """Return the explicit parameter schema for an artifact-bound Rinbo writer."""

    return set(_ACTIVE_BRIDGE_REQUIRED_PARAMETER_NAMES)


def _leaf_parameter_names(params: dict[str, Any]) -> set[str]:
    present: set[str] = set()

    def visit(value: Any, prefix: str = "") -> None:
        if not isinstance(value, dict):
            if prefix:
                present.add(prefix)
            return
        for key, item in value.items():
            child = f"{prefix}.{key}" if prefix else str(key)
            visit(item, child)

    visit(params)
    return present


def require_complete_controller_params(params: dict[str, Any]) -> None:
    """Require every runtime-declared deployment parameter to be explicit."""

    missing = sorted(
        required_controller_parameter_names() - _leaf_parameter_names(params)
    )
    if missing:
        raise ValueError(
            "controller config must be a complete deployment profile; missing: "
            + ", ".join(missing)
        )


def require_complete_bridge_params(params: dict[str, Any]) -> None:
    """Reject an active Rinbo profile which would inherit runtime defaults.

    Non-Rinbo configurations are not policy artifact inputs.  A Rinbo/BioRoLa
    configuration, however, is allowed into the semantic hash only when every
    hardware-writer parameter is explicitly represented by the YAML mapping.
    """

    if "backend" not in params:
        raise ValueError(
            "active bridge config must be a complete deployment profile; missing: backend"
        )
    backend = str(params["backend"]).strip()
    if backend not in _ACTIVE_BRIDGE_BACKENDS:
        return
    missing = sorted(
        required_active_bridge_parameter_names() - _leaf_parameter_names(params)
    )
    if missing:
        raise ValueError(
            "active bridge config must be a complete deployment profile; missing: "
            + ", ".join(missing)
        )


def controller_params_with_disabled_legs(
    params: dict[str, Any], disabled_legs: list[str] | None
) -> dict[str, Any]:
    """Apply the same named hardware mask used by launch/runtime."""

    resolved = copy.deepcopy(params)
    hardware = resolved.setdefault("hardware", {})
    if not isinstance(hardware, dict):
        raise ValueError("controller hardware config must be a mapping")
    maximum = int(hardware.get("max_disabled_legs", 1))
    selected = (
        list(hardware.get("disabled_legs", []) or [])
        if disabled_legs is None
        else disabled_legs
    )
    hardware["disabled_legs"] = normalize_disabled_legs(selected, maximum)
    return resolved


def bridge_params_with_disabled_legs(
    params: dict[str, Any], disabled_legs: list[str] | None
) -> dict[str, Any]:
    resolved = copy.deepcopy(params)
    hardware = resolved.setdefault("hardware", {})
    if not isinstance(hardware, dict):
        raise ValueError("bridge hardware config must be a mapping")
    maximum = int(hardware.get("max_disabled_legs", 1))
    selected = (
        list(hardware.get("disabled_legs", []) or [])
        if disabled_legs is None
        else disabled_legs
    )
    hardware["disabled_legs"] = normalize_disabled_legs(selected, maximum)
    return resolved


def decoder_config_from_ros_params(
    params: dict[str, Any], disabled_leg_indices: list[int] | None = None
) -> dict[str, Any]:
    """Resolve the same ActionDecoder inputs used by rl_controller_node."""

    if disabled_leg_indices is None:
        disabled_legs = normalize_disabled_legs(
            list(_nested(params, "hardware", "disabled_legs", default=[]) or []),
            int(_nested(params, "hardware", "max_disabled_legs", default=1)),
        )
        disabled_leg_indices = policy_indices_for_disabled_legs(disabled_legs)

    return {
        "action_clip": float(_nested(params, "safety", "action_clip", default=1.0)),
        "main_drive_vel_limit_rad_s": float(
            _nested(params, "safety", "main_drive_vel_limit_rad_s", default=30.0)
        ),
        "abad_pos_limit": float(
            _nested(params, "safety", "abad_pos_limit_rad", default=C.STAGE_ABAD_POS_LIMIT)
        ),
        "main_drive_slew_rate_rad_s2": float(
            _nested(params, "safety", "main_drive_slew_rate_rad_s2", default=120.0)
        ),
        "abad_slew_rate_rad_s": float(
            _nested(params, "safety", "abad_slew_rate_rad_s", default=6.0)
        ),
        "include_damper_command": bool(
            _nested(params, "action", "include_damper_command", default=False)
        ),
        "main_drive_init_control_mode": str(
            _nested(
                params,
                "action",
                "main_drive_init_control_mode",
                default="velocity_to_pose",
            )
        ),
        "init_stand_main_drive_position_gain": float(
            _nested(
                params,
                "action",
                "init_stand_main_drive_position_gain",
                default=3.0,
            )
        ),
        "init_stand_max_main_drive_vel_rad_s": float(
            _nested(
                params,
                "action",
                "init_stand_max_main_drive_vel_rad_s",
                default=1.5,
            )
        ),
        "stand_main_drive_kp": list(
            _nested(params, "action", "stand_main_drive_kp", default=[12.0] * 6)
        ),
        "stand_main_drive_kd": list(
            _nested(params, "action", "stand_main_drive_kd", default=[1.0] * 6)
        ),
        "main_drive_sign": list(
            _nested(params, "action", "main_drive_sign", default=[1.0] * 6)
        ),
        "abad_sign": list(
            _nested(params, "action", "abad_sign", default=[1.0] * 6)
        ),
        "damper_sign": list(
            _nested(params, "action", "damper_sign", default=[1.0] * 6)
        ),
        "main_drive_zero_offset_rad": list(
            _nested(params, "action", "main_drive_zero_offset_rad", default=[0.0] * 6)
        ),
        "abad_zero_offset_rad": list(
            _nested(params, "action", "abad_zero_offset_rad", default=[0.0] * 6)
        ),
        "damper_zero_offset_rad": list(
            _nested(params, "action", "damper_zero_offset_rad", default=[0.0] * 6)
        ),
        "main_drive_kp": list(
            _nested(params, "action", "main_drive_kp", default=[0.0] * 6)
        ),
        "main_drive_kd": list(
            _nested(params, "action", "main_drive_kd", default=[50.0] * 6)
        ),
        "abad_kp": list(_nested(params, "action", "abad_kp", default=[40.0] * 6)),
        "abad_kd": list(_nested(params, "action", "abad_kd", default=[4.0] * 6)),
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
        "disabled_leg_indices": list(disabled_leg_indices or []),
    }


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_jsonable(item) for item in value.tolist()]
    if isinstance(value, (np.floating, float)):
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ValueError("decoder config contains NaN or Inf")
        return numeric
    if isinstance(value, (np.integer, int)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, str) or value is None:
        return value
    raise TypeError(f"unsupported decoder config value {type(value).__name__}")


def canonical_decoder_config(decoder_config: dict[str, Any]) -> dict[str, Any]:
    # The mask is part of the decoder contract: lateral contact/FSM logic uses
    # the healthy-leg set before the final output mask is applied.
    return _jsonable(decoder_config)


def decoder_config_sha256(decoder_config: dict[str, Any]) -> str:
    payload = json.dumps(
        canonical_decoder_config(decoder_config),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def decoder_source_sha256() -> str:
    digest = hashlib.sha256()
    for label, source in (
        ("action_decoder.py", Path(action_decoder_module.__file__).resolve()),
        ("redrhex_contract.py", Path(C.__file__).resolve()),
    ):
        digest.update(label.encode("utf-8"))
        digest.update(b"\0")
        digest.update(source.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _source_bundle_sha256(labels: tuple[str, ...]) -> str:
    digest = hashlib.sha256()
    package_dir = Path(__file__).resolve().parent
    for label in labels:
        source = package_dir / label
        digest.update(label.encode("utf-8"))
        digest.update(b"\0")
        digest.update(source.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def observation_source_sha256() -> str:
    return _source_bundle_sha256(
        ("observation_builder.py", "policy_validation.py", "redrhex_contract.py")
    )


def deployment_source_sha256() -> str:
    return _source_bundle_sha256(
        (
            "action_decoder.py",
            "degraded_mode.py",
            "observation_builder.py",
            "policy_onnx_runner.py",
            "redrhex_contract.py",
            "rl_controller_node.py",
            "safety_filter.py",
            "state_machine.py",
        )
    )


def canonical_controller_config(params: dict[str, Any]) -> dict[str, Any]:
    """Canonical deployment semantics, excluding circular artifact pointers."""

    canonical = _jsonable(params)
    policy = canonical.get("policy") if isinstance(canonical, dict) else None
    if isinstance(policy, dict):
        policy.pop("onnx_path", None)
        policy.pop("expected_sha256", None)
    return canonical


def controller_config_sha256(params: dict[str, Any]) -> str:
    payload = json.dumps(
        canonical_controller_config(params),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def canonical_bridge_config(params: dict[str, Any]) -> dict[str, Any]:
    """Canonical bridge semantics excluding operational launch latches.

    Checked-in rig profiles deliberately keep ``allow_enable=false`` and the
    launch command may override it only after preflight.  Mapping, units,
    limits, calibration acknowledgements, and disabled-leg semantics remain
    bound because every other parameter is retained.  The canonical-launch
    authorization bit is likewise generated by launch and is never sourced
    from or bound into a deployment YAML.
    """

    canonical = _jsonable(params)
    for backend_key in ("serial", "sbrio", "rinbo"):
        backend = canonical.get(backend_key) if isinstance(canonical, dict) else None
        if isinstance(backend, dict):
            backend.pop("allow_enable", None)
            if backend_key == "rinbo":
                backend.pop("canonical_policy_launch_authorized", None)
    return canonical


def bridge_config_sha256(params: dict[str, Any]) -> str:
    require_complete_bridge_params(params)
    payload = json.dumps(
        canonical_bridge_config(params), sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def load_golden_vectors(path: str | Path) -> dict[str, np.ndarray]:
    vector_path = Path(path).expanduser().resolve()
    with np.load(vector_path, allow_pickle=False) as archive:
        missing = [
            field
            for field in VECTOR_FIELDS + PROVENANCE_FIELDS
            if field not in archive.files
        ]
        if missing:
            raise ValueError(f"golden NPZ is missing fields: {missing}")
        vectors = {
            field: np.asarray(archive[field])
            for field in VECTOR_FIELDS + PROVENANCE_FIELDS
        }

    if str(vectors["producer"].reshape(-1)[0]) != GOLDEN_PRODUCER:
        raise ValueError("golden vectors were not emitted by the IsaacLab play recorder")
    if re.fullmatch(
        r"[0-9a-f]{7,64}", str(vectors["training_git_sha"].reshape(-1)[0])
    ) is None:
        raise ValueError("golden training_git_sha is invalid")
    for field in (
        "training_env_source_sha256",
        "training_env_config_source_sha256",
        "training_play_source_sha256",
    ):
        value = str(vectors[field].reshape(-1)[0])
        if value and re.fullmatch(r"[0-9a-f]{64}", value) is None:
            raise ValueError(f"golden {field} is not a SHA256")
    deployment_profile = str(
        vectors["deployment_command_profile"].reshape(-1)[0]
    )
    if deployment_profile not in ("fixed_forward", "external_cmd_vel"):
        raise ValueError("golden deployment_command_profile is invalid")
    deployment_vx = float(vectors["deployment_fixed_forward_vx"].reshape(-1)[0])
    if not math.isfinite(deployment_vx):
        raise ValueError("golden deployment_fixed_forward_vx is invalid")

    policy_input = np.asarray(vectors["policy_input"], dtype=np.float32)
    if policy_input.ndim != 2 or policy_input.shape[1] not in (
        C.OBS_DIM_SINGLE,
        C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH,
    ):
        raise ValueError(
            "policy_input must have shape [N,56] or [N,280], got "
            f"{policy_input.shape}"
        )
    sample_count = policy_input.shape[0]
    if sample_count < MIN_GOLDEN_SAMPLES:
        raise ValueError(
            f"golden trajectory needs at least {MIN_GOLDEN_SAMPLES} ordered samples"
        )
    vectors["policy_input"] = policy_input

    shapes = {
        "single_observation": (sample_count, C.OBS_DIM_SINGLE),
        "base_lin_vel": (sample_count, 3),
        "base_ang_vel": (sample_count, 3),
        "main_drive_pos": (sample_count, 6),
        "main_drive_vel": (sample_count, 6),
        "abad_pos": (sample_count, 6),
        "abad_vel": (sample_count, 6),
        "command": (sample_count, 3),
        "projected_gravity": (sample_count, 3),
        "dt": (sample_count,),
        "gait_phase": (sample_count,),
        "episode_start": (sample_count,),
        "expected_policy_action": (sample_count, C.ACTION_DIM),
        "expected_training_clipped_action": (sample_count, C.ACTION_DIM),
        "expected_safe_action": (sample_count, C.ACTION_DIM),
        "expected_target_main_drive_velocity": (sample_count, 6),
        "expected_target_abad_position": (sample_count, 6),
        "expected_enable": (sample_count,),
        "expected_mode": (sample_count,),
    }
    for field, expected_shape in shapes.items():
        if vectors[field].shape != expected_shape:
            raise ValueError(
                f"golden field {field} has shape {vectors[field].shape}, "
                f"expected {expected_shape}"
            )

    joint_names = np.asarray(vectors["joint_names"])
    if joint_names.ndim != 1 or joint_names.size not in (12, 18):
        raise ValueError("joint_names must be a one-dimensional 12- or 18-name array")
    if len(set(str(name) for name in joint_names.tolist())) != joint_names.size:
        raise ValueError("joint_names must be unique")
    command_width = int(joint_names.size)
    for field in (
        "expected_target_position_rad",
        "expected_target_velocity_rad_s",
        "expected_kp",
        "expected_kd",
        "expected_effort_limit_nm",
    ):
        expected_shape = (sample_count, command_width)
        if vectors[field].shape != expected_shape:
            raise ValueError(
                f"golden field {field} has shape {vectors[field].shape}, "
                f"expected {expected_shape}"
            )

    numeric_fields = [field for field in VECTOR_FIELDS if field not in (
        "episode_start", "expected_enable", "expected_mode", "joint_names"
    )]
    for field in numeric_fields:
        if not np.isfinite(np.asarray(vectors[field], dtype=np.float64)).all():
            raise ValueError(f"golden field {field} contains NaN or Inf")
    dt = np.asarray(vectors["dt"], dtype=np.float64)
    if np.any(dt <= 0.0):
        raise ValueError("golden dt must be positive")
    starts = np.asarray(vectors["episode_start"], dtype=bool)
    if not bool(starts[0]) or not bool(np.any(starts)):
        raise ValueError("golden trajectory must mark the first sample as episode_start")
    episode_indices = np.flatnonzero(starts)
    episode_lengths = np.diff(np.append(episode_indices, sample_count))
    if episode_indices.size < MIN_GOLDEN_EPISODES:
        raise ValueError(
            f"golden trajectory needs at least {MIN_GOLDEN_EPISODES} episodes to verify reset behavior"
        )
    if np.any(episode_lengths < MIN_GOLDEN_EPISODE_SAMPLES):
        raise ValueError(
            "every golden episode must contain at least "
            f"{MIN_GOLDEN_EPISODE_SAMPLES} samples to cross the decoder warmup"
        )
    previous_action = np.zeros((sample_count, C.ACTION_DIM), dtype=np.float64)
    clipped_action = np.asarray(
        vectors["expected_training_clipped_action"], dtype=np.float64
    )
    episode_start_index = 0
    for index in range(sample_count):
        if starts[index]:
            episode_start_index = index
        if index - episode_start_index >= 2:
            previous_action[index] = clipped_action[index - 2]
    reconstructed_single = np.concatenate(
        [
            np.asarray(vectors["base_lin_vel"], dtype=np.float64),
            np.asarray(vectors["base_ang_vel"], dtype=np.float64),
            np.asarray(vectors["projected_gravity"], dtype=np.float64),
            np.sin(np.asarray(vectors["main_drive_pos"], dtype=np.float64)),
            np.cos(np.asarray(vectors["main_drive_pos"], dtype=np.float64)),
            np.asarray(vectors["main_drive_vel"], dtype=np.float64)
            / C.BASE_GAIT_ANGULAR_VEL,
            np.asarray(vectors["abad_pos"], dtype=np.float64) / C.ABAD_POS_SCALE,
            np.asarray(vectors["abad_vel"], dtype=np.float64),
            np.asarray(vectors["command"], dtype=np.float64),
            np.column_stack(
                (
                    np.sin(np.asarray(vectors["gait_phase"], dtype=np.float64)),
                    np.cos(np.asarray(vectors["gait_phase"], dtype=np.float64)),
                )
            ),
            previous_action,
        ],
        axis=1,
    ).astype(np.float32)
    single_observation = np.asarray(vectors["single_observation"], dtype=np.float32)
    if not np.allclose(
        single_observation, reconstructed_single, rtol=0.0, atol=1.0e-5
    ):
        raise ValueError(
            "single_observation does not match the raw 56-D training feature contract"
        )
    if policy_input.shape[1] == C.OBS_DIM_SINGLE:
        expected_policy_input = single_observation
    else:
        history_rows: list[np.ndarray] = []
        history: list[np.ndarray] = []
        for index, current in enumerate(single_observation):
            if starts[index]:
                history = []
            history.insert(0, current)
            history = history[: C.POLICY_HISTORY_LENGTH]
            frames = history + [
                np.zeros(C.OBS_DIM_SINGLE, dtype=np.float32)
                for _ in range(C.POLICY_HISTORY_LENGTH - len(history))
            ]
            history_rows.append(np.concatenate(frames))
        expected_policy_input = np.stack(history_rows).astype(np.float32)
    if not np.allclose(policy_input, expected_policy_input, rtol=0.0, atol=1.0e-5):
        raise ValueError(
            "policy_input does not match single_56/current-to-oldest history layout"
        )
    phase = np.mod(np.asarray(vectors["gait_phase"], dtype=np.float64), 2.0 * math.pi)
    phase_bins = set(np.floor(phase / (0.5 * math.pi)).astype(int).tolist())
    if phase_bins != {0, 1, 2, 3}:
        raise ValueError("golden trajectory must cover all four gait-phase quadrants")
    if not bool(
        np.any(
            np.abs(
                np.asarray(
                    vectors["expected_target_main_drive_velocity"], dtype=np.float64
                )
            )
            > 1.0e-6
        )
    ):
        raise ValueError("golden trajectory has no non-trivial main-drive target")
    gravity_norm = np.linalg.norm(
        np.asarray(vectors["projected_gravity"], dtype=np.float64), axis=1
    )
    if not np.allclose(gravity_norm, 1.0, atol=1.0e-3, rtol=0.0):
        raise ValueError("every projected_gravity sample must be a unit vector")
    return vectors


def require_trajectory_matches_controller(
    vectors: dict[str, np.ndarray], params: dict[str, Any]
) -> None:
    """Cross-check raw golden fields against the selected runtime profile."""

    commands = np.asarray(vectors["command"], dtype=np.float64)
    limits = (
        (
            float(_nested(params, "commands", "vx_min", default=C.COMMAND_LIMITS["vx_min"])),
            float(_nested(params, "commands", "vx_max", default=C.COMMAND_LIMITS["vx_max"])),
        ),
        (
            float(_nested(params, "commands", "vy_min", default=C.COMMAND_LIMITS["vy_min"])),
            float(_nested(params, "commands", "vy_max", default=C.COMMAND_LIMITS["vy_max"])),
        ),
        (
            float(_nested(params, "commands", "wz_min", default=C.COMMAND_LIMITS["wz_min"])),
            float(_nested(params, "commands", "wz_max", default=C.COMMAND_LIMITS["wz_max"])),
        ),
    )
    for axis, (minimum, maximum) in enumerate(limits):
        if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum > maximum:
            raise ValueError("controller command limits are invalid")
        if np.any(commands[:, axis] < minimum - 1.0e-9) or np.any(
            commands[:, axis] > maximum + 1.0e-9
        ):
            raise ValueError(
                f"golden command axis {axis} exceeds controller range [{minimum}, {maximum}]"
            )
    sensor_profile = str(
        _nested(params, "observation", "sensor_profile", default="full_state")
    )
    if sensor_profile == "encoder_only_rig":
        if not np.allclose(vectors["base_lin_vel"], 0.0, rtol=0.0, atol=1.0e-9):
            raise ValueError("encoder_only_rig golden base_lin_vel must be zero")
        if not np.allclose(vectors["base_ang_vel"], 0.0, rtol=0.0, atol=1.0e-9):
            raise ValueError("encoder_only_rig golden base_ang_vel must be zero")
        expected_gravity = np.asarray(C.REFERENCE_PROJECTED_GRAVITY, dtype=np.float64)
        if not np.allclose(
            vectors["projected_gravity"], expected_gravity, rtol=0.0, atol=1.0e-9
        ):
            raise ValueError(
                "encoder_only_rig golden projected_gravity must be [0,-1,0]"
            )
    command_profile = str(
        _nested(params, "commands", "profile", default="external_cmd_vel")
    )
    if command_profile == "fixed_forward":
        if not np.allclose(vectors["base_lin_vel"], 0.0, rtol=0.0, atol=1.0e-9):
            raise ValueError("suspended fixed-forward golden base_lin_vel must be zero")
        fixed_command = np.array(
            [
                float(_nested(params, "commands", "fixed_forward_vx", default=0.22)),
                0.0,
                0.0,
            ],
            dtype=np.float64,
        )
        starts = np.flatnonzero(np.asarray(vectors["episode_start"], dtype=bool))
        ends = np.append(starts[1:], commands.shape[0])
        every_episode_is_fixed = all(
            np.allclose(commands[start:end], fixed_command, rtol=0.0, atol=1.0e-9)
            for start, end in zip(starts, ends)
        )
        if not every_episode_is_fixed:
            raise ValueError(
                "every fixed-forward golden deployment episode must use exactly [vx,0,0]"
            )
        if not np.allclose(
            vectors["expected_target_abad_position"], 0.0, rtol=0.0, atol=1.0e-9
        ):
            raise ValueError("fixed-forward golden ABAD targets must remain zero")
    else:
        resolved_modes = [ActionDecoder._resolve_command_modes(row)[4] for row in commands]
        for mode, label in (
            (0, "forward"),
            (1, "lateral"),
            (2, "diagonal"),
            (3, "yaw"),
        ):
            if mode not in resolved_modes:
                raise ValueError(
                    f"golden trajectory does not cross the decoder's {label} mode threshold"
                )
        longest_lateral_run = 0
        current_lateral_run = 0
        for mode in resolved_modes:
            current_lateral_run = current_lateral_run + 1 if mode == 1 else 0
            longest_lateral_run = max(longest_lateral_run, current_lateral_run)
        if longest_lateral_run < 16:
            raise ValueError(
                "golden trajectory needs at least 16 consecutive lateral-mode samples"
            )
        if not np.any(
            np.abs(
                np.asarray(vectors["expected_target_abad_position"], dtype=np.float64)
            )
            > 1.0e-6
        ):
            raise ValueError("external-command golden trajectory has no ABAD target")

    disabled_legs = normalize_disabled_legs(
        list(_nested(params, "hardware", "disabled_legs", default=[]) or []),
        int(_nested(params, "hardware", "max_disabled_legs", default=1)),
    )
    disabled_indices = policy_indices_for_disabled_legs(disabled_legs)
    recorded_disabled_legs = str(
        np.asarray(vectors["deployment_disabled_legs_csv"]).reshape(-1)[0]
    )
    if recorded_disabled_legs != ",".join(disabled_legs):
        raise ValueError(
            "golden disabled-leg provenance does not match controller profile"
        )
    if str(
        np.asarray(vectors["deployment_command_profile"]).reshape(-1)[0]
    ) != command_profile:
        raise ValueError("golden command-profile provenance does not match controller")
    if not math.isclose(
        float(np.asarray(vectors["deployment_fixed_forward_vx"]).reshape(-1)[0]),
        float(_nested(params, "commands", "fixed_forward_vx", default=0.22)),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("golden fixed-forward speed provenance does not match controller")
    disabled_mode = str(
        _nested(
            params,
            "observation",
            "disabled_leg_observation_mode",
            default="passthrough",
        )
    )
    if disabled_indices and disabled_mode == "nominal":
        main_pos = np.asarray(vectors["main_drive_pos"], dtype=np.float64)
        main_vel = np.asarray(vectors["main_drive_vel"], dtype=np.float64)
        abad_pos = np.asarray(vectors["abad_pos"], dtype=np.float64)
        abad_vel = np.asarray(vectors["abad_vel"], dtype=np.float64)
        for leg_index in disabled_indices:
            if not np.allclose(
                main_pos[:, leg_index], C.INIT_MAIN_DRIVE_POS[leg_index], atol=1.0e-9
            ):
                raise ValueError("disabled-leg golden main position is not nominal")
            if not np.allclose(main_vel[:, leg_index], 0.0, atol=1.0e-9):
                raise ValueError("disabled-leg golden main velocity is not zero")
            if not np.allclose(
                abad_pos[:, leg_index], C.INIT_ABAD_POS[leg_index], atol=1.0e-9
            ) or not np.allclose(abad_vel[:, leg_index], 0.0, atol=1.0e-9):
                raise ValueError("disabled-leg golden ABAD observation is not nominal")


def require_golden_provenance(
    vectors: dict[str, np.ndarray],
    *,
    training_git_sha: str,
    training_env_source: str | Path,
    training_env_config_source: str | Path,
    training_play_source: str | Path | None = None,
) -> None:
    expected = {
        "training_git_sha": str(training_git_sha).strip().lower(),
        "training_env_source_sha256": sha256_file(training_env_source),
        "training_env_config_source_sha256": sha256_file(
            training_env_config_source
        ),
        "training_play_source_sha256": (
            sha256_file(training_play_source) if training_play_source else ""
        ),
    }
    for field, expected_value in expected.items():
        actual = str(np.asarray(vectors[field]).reshape(-1)[0]).strip().lower()
        if actual != expected_value:
            raise ValueError(
                f"golden provenance {field}={actual!r}, expected {expected_value!r}"
            )


def _max_abs(actual: np.ndarray, expected: np.ndarray) -> float:
    return float(np.max(np.abs(np.asarray(actual) - np.asarray(expected))))


def _max_relative(actual: np.ndarray, expected: np.ndarray) -> float:
    actual_array = np.asarray(actual, dtype=np.float64)
    expected_array = np.asarray(expected, dtype=np.float64)
    denominator = np.maximum(np.abs(expected_array), 1.0e-12)
    return float(np.max(np.abs(actual_array - expected_array) / denominator))


def replay_onnx_and_decoder(
    onnx_path: str | Path,
    vectors_path: str | Path,
    decoder_config: dict[str, Any],
    *,
    policy_rtol: float = 1.0e-4,
    policy_atol: float = 1.0e-4,
    decoder_atol: float = 1.0e-5,
) -> dict[str, Any]:
    """Replay an ordered trajectory and return recomputed parity metrics."""

    if policy_rtol < 0.0 or policy_atol < 0.0 or decoder_atol < 0.0:
        raise ValueError("parity tolerances must be non-negative")
    vectors = load_golden_vectors(vectors_path)
    policy_input = vectors["policy_input"]
    runner = PolicyONNXRunner(
        str(onnx_path),
        expected_obs_dim=C.OBS_DIM_SINGLE,
        expected_action_dim=C.ACTION_DIM,
        allow_history_dim=True,
        expected_observation_contract=C.OBSERVATION_CONTRACT_ID,
        expected_action_contract=C.ACTION_DECODER_CONTRACT_ID,
        expected_input_layout="auto",
        require_contract_metadata=True,
    )
    if runner.obs_dim != policy_input.shape[1]:
        raise ValueError(
            f"golden input width {policy_input.shape[1]} != ONNX input {runner.obs_dim}"
        )

    decoder = ActionDecoder(dict(decoder_config))
    expected_joint_names = [str(name) for name in vectors["joint_names"].tolist()]
    actual_policy_actions: list[np.ndarray] = []
    actual_fields: dict[str, list[np.ndarray]] = {
        "safe_action": [],
        "target_main_drive_velocity": [],
        "target_abad_position": [],
        "target_position_rad": [],
        "target_velocity_rad_s": [],
        "kp": [],
        "kd": [],
        "effort_limit_nm": [],
    }
    actual_enable: list[bool] = []
    actual_mode: list[int] = []
    episode_start = np.asarray(vectors["episode_start"], dtype=bool)
    for index, policy_vector in enumerate(policy_input):
        if episode_start[index]:
            decoder.reset(gait_phase=0.0)
        action = runner.run(policy_vector)
        decoded = decoder.decode(
            action,
            np.asarray(vectors["main_drive_pos"][index], dtype=np.float64),
            np.asarray(vectors["abad_pos"][index], dtype=np.float64),
            np.asarray(vectors["command"][index], dtype=np.float64),
            np.asarray(vectors["projected_gravity"][index], dtype=np.float64),
            float(vectors["dt"][index]),
            float(vectors["gait_phase"][index]),
        )
        if decoded.joint_names != expected_joint_names:
            raise ValueError(
                f"decoder joint order mismatch at sample {index}: {decoded.joint_names}"
            )
        actual_policy_actions.append(action)
        actual_fields["safe_action"].append(np.asarray(decoded.safe_action))
        actual_fields["target_main_drive_velocity"].append(
            np.asarray(decoded.target_main_drive_velocity)
        )
        actual_fields["target_abad_position"].append(
            np.asarray(decoded.target_abad_position)
        )
        actual_fields["target_position_rad"].append(
            np.asarray(decoded.target_position_rad)
        )
        actual_fields["target_velocity_rad_s"].append(
            np.asarray(decoded.target_velocity_rad_s)
        )
        actual_fields["kp"].append(np.asarray(decoded.kp))
        actual_fields["kd"].append(np.asarray(decoded.kd))
        actual_fields["effort_limit_nm"].append(np.asarray(decoded.effort_limit_nm))
        actual_enable.append(bool(decoded.enable))
        actual_mode.append(int(decoded.mode))

    actual_action = np.stack(actual_policy_actions).astype(np.float32)
    expected_action = np.asarray(vectors["expected_policy_action"], dtype=np.float32)
    policy_ok = bool(
        np.allclose(actual_action, expected_action, rtol=policy_rtol, atol=policy_atol)
    )
    clipped_actual = np.clip(
        actual_action, -C.TRAINING_ACTION_CLIP, C.TRAINING_ACTION_CLIP
    )
    clipped_expected = np.asarray(
        vectors["expected_training_clipped_action"], dtype=np.float32
    )
    training_clip_error = _max_abs(clipped_actual, clipped_expected)
    training_clip_ok = bool(
        np.allclose(clipped_actual, clipped_expected, rtol=0.0, atol=decoder_atol)
    )

    expected_by_actual_field = {
        "safe_action": "expected_safe_action",
        "target_main_drive_velocity": "expected_target_main_drive_velocity",
        "target_abad_position": "expected_target_abad_position",
        "target_position_rad": "expected_target_position_rad",
        "target_velocity_rad_s": "expected_target_velocity_rad_s",
        "kp": "expected_kp",
        "kd": "expected_kd",
        "effort_limit_nm": "expected_effort_limit_nm",
    }
    decoder_metrics: dict[str, float] = {}
    decoder_ok = training_clip_ok
    for actual_name, expected_name in expected_by_actual_field.items():
        actual_values = np.stack(actual_fields[actual_name])
        expected_values = np.asarray(vectors[expected_name])
        error = _max_abs(actual_values, expected_values)
        decoder_metrics[actual_name] = error
        decoder_ok = decoder_ok and bool(
            np.allclose(actual_values, expected_values, rtol=0.0, atol=decoder_atol)
        )
    enable_ok = np.array_equal(
        np.asarray(actual_enable, dtype=bool),
        np.asarray(vectors["expected_enable"], dtype=bool),
    )
    mode_ok = np.array_equal(
        np.asarray(actual_mode, dtype=np.uint8),
        np.asarray(vectors["expected_mode"], dtype=np.uint8),
    )
    decoder_ok = decoder_ok and enable_ok and mode_ok

    return {
        "sample_count": int(policy_input.shape[0]),
        "episode_count": int(np.count_nonzero(episode_start)),
        "policy_input_dim": int(policy_input.shape[1]),
        "onnx_sha256": runner.sha256,
        "onnx_metadata": dict(runner.io_info.metadata),
        "onnx_action": actual_action,
        "policy_expected_parity": {
            "passed": policy_ok,
            "rtol": float(policy_rtol),
            "atol": float(policy_atol),
            "max_abs_error": _max_abs(actual_action, expected_action),
            "max_rel_error": _max_relative(actual_action, expected_action),
        },
        "ros_decoder": {
            "passed": bool(decoder_ok),
            "atol": float(decoder_atol),
            "training_clip_max_abs_error": training_clip_error,
            "fields": decoder_metrics,
            "enable_match": bool(enable_ok),
            "mode_match": bool(mode_ok),
        },
        "max_abs_raw_action": float(np.max(np.abs(actual_action))),
    }


def require_replay_passed(result: dict[str, Any]) -> None:
    if not bool(result["policy_expected_parity"]["passed"]):
        raise ValueError(
            "ONNX actions do not match golden expected_policy_action: "
            f"max_abs={result['policy_expected_parity']['max_abs_error']:.8g}"
        )
    if not bool(result["ros_decoder"]["passed"]):
        fields = result["ros_decoder"]["fields"]
        worst_name = max(fields, key=fields.get)
        raise ValueError(
            "ROS ActionDecoder trajectory parity failed: "
            f"worst={worst_name} max_abs={fields[worst_name]:.8g}"
        )


def require_source_export_metadata(
    result: dict[str, Any], expected_training_git_sha: str | None = None
) -> None:
    """Require metadata to originate in the training exporter, before packaging."""

    metadata = dict(result.get("onnx_metadata", {}))
    metadata_rejections = deployment_metadata_rejection_reasons(metadata)
    if metadata_rejections:
        raise ValueError(
            "source ONNX metadata explicitly blocks deployment: "
            + "; ".join(metadata_rejections)
        )
    normalizer = metadata.get(C.ONNX_NORMALIZER_KEY)
    if normalizer != C.NORMALIZER_EMBEDDED:
        raise ValueError(
            f"source ONNX must declare {C.ONNX_NORMALIZER_KEY}={C.NORMALIZER_EMBEDDED!r}"
        )
    action_clip_text = metadata.get(C.ONNX_TRAINING_ACTION_CLIP_KEY)
    try:
        action_clip = float(action_clip_text)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"source ONNX must declare numeric {C.ONNX_TRAINING_ACTION_CLIP_KEY}"
        ) from exc
    if not math.isclose(
        action_clip, C.TRAINING_ACTION_CLIP, rel_tol=0.0, abs_tol=1.0e-12
    ):
        raise ValueError(
            f"source ONNX training action clip {action_clip} != {C.TRAINING_ACTION_CLIP}"
        )
    training_git_sha = str(metadata.get(C.ONNX_TRAINING_GIT_SHA_KEY, "")).strip().lower()
    if re.fullmatch(r"[0-9a-f]{7,64}", training_git_sha) is None:
        raise ValueError(
            f"source ONNX must declare {C.ONNX_TRAINING_GIT_SHA_KEY} as a 7-64 hex commit"
        )
    if expected_training_git_sha is not None and training_git_sha != str(
        expected_training_git_sha
    ).strip().lower():
        raise ValueError(
            f"source ONNX training git SHA {training_git_sha} != "
            f"requested {expected_training_git_sha}"
        )
