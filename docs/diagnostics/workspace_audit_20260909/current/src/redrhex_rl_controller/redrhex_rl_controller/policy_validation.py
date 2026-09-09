"""Reference vectors used to reject incompatible deployment policies."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping

import numpy as np

from . import redrhex_contract as C


_EXPLICIT_DEPLOYMENT_BLOCKERS = (
    "diagnostic_only",
    "not_deployable",
    "non_deployable",
    "quality_rejected",
    "deployment_rejected",
    "do_not_deploy",
)

_APPROVED_STATUS_VALUES = {
    C.ONNX_ARTIFACT_STATUS_KEY: frozenset({"deployable", "deployment_approved"}),
    C.ONNX_QUALITY_STATUS_KEY: frozenset({"approved", "quality_approved", "passed"}),
}


def _normalized_metadata_token(value: object) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(value).strip().lower()).strip("_")


def deployment_metadata_rejection_reasons(
    metadata: Mapping[str, object] | None,
) -> list[str]:
    """Return explicit reasons why ONNX metadata forbids deployment.

    Export pipelines may attach richer metadata than the RedRhex contract.
    Known status keys are allowlisted when present, and strong negative labels
    remain fail-closed even when shapes and required contract keys look valid.
    """

    reasons: list[str] = []
    for raw_key, raw_value in (metadata or {}).items():
        key = _normalized_metadata_token(raw_key)
        value = _normalized_metadata_token(raw_value)
        approved_values = _APPROVED_STATUS_VALUES.get(key)
        if approved_values is not None:
            if value not in approved_values:
                reasons.append(f"{raw_key}={raw_value}")
            continue
        if any(
            marker in key or marker in value
            for marker in _EXPLICIT_DEPLOYMENT_BLOCKERS
        ):
            reasons.append(f"{raw_key}={raw_value}")
            continue
        if key.endswith("deployable") and value in {"0", "false", "no"}:
            reasons.append(f"{raw_key}={raw_value}")
            continue
        if key.endswith(("deployment_approved", "quality_approved")) and value in {
            "0",
            "false",
            "no",
            "rejected",
        }:
            reasons.append(f"{raw_key}={raw_value}")
    return reasons


def reference_single_observation() -> np.ndarray:
    """Return a finite, training-frame reset observation.

    This is intentionally not an all-zero vector: projected gravity, main
    joint cosine, and gait phase cosine have meaningful non-zero reset values.
    """

    main_pos = np.asarray(C.INIT_MAIN_DRIVE_POS, dtype=np.float32)
    obs = np.concatenate(
        [
            np.zeros(3, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
            np.asarray(C.REFERENCE_PROJECTED_GRAVITY, dtype=np.float32),
            np.sin(main_pos),
            np.cos(main_pos),
            np.zeros(6, dtype=np.float32),
            np.asarray(C.INIT_ABAD_POS, dtype=np.float32) / float(C.ABAD_POS_SCALE),
            np.zeros(6, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
            np.asarray([math.sin(0.0), math.cos(0.0)], dtype=np.float32),
            np.zeros(C.ACTION_DIM, dtype=np.float32),
        ]
    ).astype(np.float32)
    if obs.shape != (C.OBS_DIM_SINGLE,) or not np.isfinite(obs).all():
        raise RuntimeError("internal reference observation contract is invalid")
    return obs


def reference_policy_input(input_dim: int, history_length: int = C.POLICY_HISTORY_LENGTH) -> np.ndarray:
    """Build the reference 56-D input or first-tick 280-D history input."""

    single = reference_single_observation()
    if input_dim == C.OBS_DIM_SINGLE:
        return single
    history_dim = C.OBS_DIM_SINGLE * int(history_length)
    if input_dim != history_dim:
        raise ValueError(f"unsupported policy input dim {input_dim}; expected 56 or {history_dim}")
    # ObservationBuilder initializes history with the current frame followed
    # by zero padding, so preflight must test that exact ordering.
    return np.concatenate(
        [single, np.zeros(history_dim - C.OBS_DIM_SINGLE, dtype=np.float32)]
    ).astype(np.float32)


def check_reference_action(action: np.ndarray, max_abs_action: float) -> tuple[bool, float]:
    action = np.asarray(action, dtype=np.float32).reshape(-1)
    if action.shape != (C.ACTION_DIM,) or not np.isfinite(action).all():
        return False, math.inf
    if not math.isfinite(max_abs_action) or max_abs_action <= 0.0:
        raise ValueError("max_abs_action must be positive and finite")
    observed = float(np.max(np.abs(action)))
    return observed <= max_abs_action, observed
