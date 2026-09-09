#!/usr/bin/env python3
"""Verify Torch/ONNX and stateful ROS decoder parity on golden trajectories."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

from redrhex_rl_controller import redrhex_contract as C
from redrhex_rl_controller.golden_policy import (
    GOLDEN_SCHEMA,
    bridge_config_sha256,
    bridge_params_with_disabled_legs,
    controller_config_sha256,
    controller_params_with_disabled_legs,
    decoder_config_from_ros_params,
    decoder_config_sha256,
    decoder_source_sha256,
    deployment_source_sha256,
    load_bridge_ros_params,
    load_controller_ros_params,
    load_golden_vectors,
    observation_source_sha256,
    replay_onnx_and_decoder,
    require_replay_passed,
    require_golden_provenance,
    require_source_export_metadata,
    require_trajectory_matches_controller,
    sha256_file,
)


def _run_torchscript(path: str | Path, observations: np.ndarray) -> np.ndarray:
    try:
        import torch
    except Exception as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("Torch is required for golden policy verification") from exc
    module = torch.jit.load(str(path), map_location="cpu")
    module.eval()
    with torch.no_grad():
        output = module(torch.from_numpy(observations.astype(np.float32)))
    if not hasattr(output, "detach"):
        raise ValueError("TorchScript policy must return one tensor")
    action = output.detach().cpu().numpy().astype(np.float32)
    if action.shape != (observations.shape[0], C.ACTION_DIM):
        raise ValueError(
            f"TorchScript action shape {action.shape} != "
            f"({observations.shape[0]}, {C.ACTION_DIM})"
        )
    if not np.isfinite(action).all():
        raise ValueError("TorchScript action contains NaN or Inf")
    return action


def _max_abs(actual: np.ndarray, expected: np.ndarray) -> float:
    return float(np.max(np.abs(np.asarray(actual) - np.asarray(expected))))


def _max_relative(actual: np.ndarray, expected: np.ndarray) -> float:
    actual_array = np.asarray(actual, dtype=np.float64)
    expected_array = np.asarray(expected, dtype=np.float64)
    denominator = np.maximum(np.abs(expected_array), 1.0e-12)
    return float(np.max(np.abs(actual_array - expected_array) / denominator))


def _validate_git_sha(value: str) -> str:
    sha = value.strip().lower()
    if re.fullmatch(r"[0-9a-f]{7,64}", sha) is None:
        raise ValueError("--training-git-sha must be 7-64 hexadecimal characters")
    return sha


def _parse_disabled_legs(value: str | None) -> list[str] | None:
    if value is None:
        return None
    return [item.strip() for item in value.split(",") if item.strip()]


def _simple_compare(args: argparse.Namespace) -> int:
    try:
        import onnxruntime as ort
    except Exception as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("onnxruntime is required") from exc
    session = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    if len(session.get_inputs()) != 1 or len(session.get_outputs()) != 1:
        raise ValueError("policy must expose exactly one ONNX input and one output")
    input_info = session.get_inputs()[0]
    output_info = session.get_outputs()[0]
    obs_dim = input_info.shape[-1] if isinstance(input_info.shape[-1], int) else args.obs_dim
    if args.obs_npy:
        observations = np.load(args.obs_npy, allow_pickle=False).astype(np.float32)
        if observations.ndim == 1:
            observations = observations.reshape(1, -1)
    else:
        observations = np.zeros((1, obs_dim), dtype=np.float32)
    if observations.ndim != 2 or observations.shape[1] != obs_dim:
        raise ValueError(
            f"observation shape {observations.shape} does not match ONNX width {obs_dim}"
        )
    onnx_action = np.asarray(
        session.run([output_info.name], {input_info.name: observations})[0],
        dtype=np.float32,
    )
    torch_action = _run_torchscript(args.torchscript, observations)
    max_abs = _max_abs(onnx_action, torch_action)
    print(f"onnx action shape:  {onnx_action.shape}")
    print(f"torch action shape: {torch_action.shape}")
    print(f"max abs diff: {max_abs:.8f}")
    if not np.allclose(onnx_action, torch_action, rtol=args.rtol, atol=args.atol):
        raise ValueError(
            "Torch/ONNX mismatch is too large; check normalizer, export path, and input order"
        )
    print("ONNX/TorchScript tensor consistency OK (decoder parity not checked)")
    return 0


def _golden_compare(args: argparse.Namespace) -> int:
    if not all(
        (
            args.controller_config,
            args.bridge_config,
            args.report_json,
            args.training_git_sha,
            args.training_env_source,
            args.training_env_config_source,
        )
    ):
        raise ValueError(
            "golden verification requires controller/bridge configs, report, training git SHA, "
            "and both training environment source files"
        )
    report_path = Path(args.report_json).expanduser().resolve()
    if report_path.exists():
        raise ValueError(f"report already exists; choose an immutable new path: {report_path}")
    training_git_sha = _validate_git_sha(args.training_git_sha)
    params = controller_params_with_disabled_legs(
        load_controller_ros_params(args.controller_config),
        _parse_disabled_legs(args.disabled_legs),
    )
    disabled_legs = _parse_disabled_legs(args.disabled_legs)
    bridge_params = bridge_params_with_disabled_legs(
        load_bridge_ros_params(args.bridge_config), disabled_legs
    )
    semantic_bridge_sha = bridge_config_sha256(bridge_params)
    configured_bridge_sha = str(
        params.get("policy", {}).get("expected_bridge_config_sha256", "")
    ).strip()
    if configured_bridge_sha != semantic_bridge_sha:
        raise ValueError(
            "controller policy.expected_bridge_config_sha256 does not match --bridge-config: "
            f"{configured_bridge_sha or 'missing'} != {semantic_bridge_sha}"
        )
    training_sources = {
        "training_env_source_sha256": sha256_file(args.training_env_source),
        "training_env_config_source_sha256": sha256_file(
            args.training_env_config_source
        ),
    }
    if args.training_play_source:
        training_sources["training_play_source_sha256"] = sha256_file(
            args.training_play_source
        )
    decoder_config = decoder_config_from_ros_params(params)
    replay = replay_onnx_and_decoder(
        args.onnx,
        args.golden_vectors,
        decoder_config,
        policy_rtol=args.rtol,
        policy_atol=args.atol,
        decoder_atol=args.decoder_atol,
    )
    require_source_export_metadata(replay, training_git_sha)
    require_replay_passed(replay)
    max_raw_action_abs = float(
        params.get("safety", {}).get("max_raw_action_abs", 1.5)
    )
    if not math.isfinite(max_raw_action_abs) or max_raw_action_abs <= 0.0:
        raise ValueError("safety.max_raw_action_abs must be positive and finite")
    if float(replay["max_abs_raw_action"]) > max_raw_action_abs:
        raise ValueError(
            "golden trajectory raw action exceeds deployment hard-stop limit: "
            f"{replay['max_abs_raw_action']:.6g} > {max_raw_action_abs:.6g}"
        )
    vectors = load_golden_vectors(args.golden_vectors)
    require_golden_provenance(
        vectors,
        training_git_sha=training_git_sha,
        training_env_source=args.training_env_source,
        training_env_config_source=args.training_env_config_source,
        training_play_source=args.training_play_source or None,
    )
    require_trajectory_matches_controller(vectors, params)
    torch_action = _run_torchscript(args.torchscript, vectors["policy_input"])
    onnx_action = np.asarray(replay["onnx_action"], dtype=np.float32)
    expected_action = np.asarray(vectors["expected_policy_action"], dtype=np.float32)
    torch_onnx_ok = bool(
        np.allclose(torch_action, onnx_action, rtol=args.rtol, atol=args.atol)
    )
    torch_expected_ok = bool(
        np.allclose(torch_action, expected_action, rtol=args.rtol, atol=args.atol)
    )
    if not torch_onnx_ok or not torch_expected_ok:
        raise ValueError(
            "TorchScript, ONNX, and golden expected actions are not the same artifact contract"
        )
    metadata = dict(replay["onnx_metadata"])
    input_layout = metadata[C.ONNX_INPUT_LAYOUT_KEY]
    report = {
        "schema": GOLDEN_SCHEMA,
        "contracts": {
            "observation": C.OBSERVATION_CONTRACT_ID,
            "input_layout": input_layout,
            "history_length": (
                C.POLICY_HISTORY_LENGTH
                if replay["policy_input_dim"] == C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH
                else 1
            ),
            "policy_input_dim": replay["policy_input_dim"],
            "action_decoder": C.ACTION_DECODER_CONTRACT_ID,
            "action_dim": C.ACTION_DIM,
            "training_action_clip": C.TRAINING_ACTION_CLIP,
            "normalizer": C.NORMALIZER_EMBEDDED,
            "disabled_legs": list(
                params.get("hardware", {}).get("disabled_legs", [])
            ),
        },
        "artifacts": {
            "onnx_sha256": sha256_file(args.onnx),
            "torchscript_sha256": sha256_file(args.torchscript),
            "vectors_sha256": sha256_file(args.golden_vectors),
            "controller_yaml_sha256": sha256_file(args.controller_config),
            "controller_config_sha256": controller_config_sha256(params),
            "decoder_config_sha256": decoder_config_sha256(decoder_config),
            "decoder_source_sha256": decoder_source_sha256(),
            "observation_source_sha256": observation_source_sha256(),
            "deployment_source_sha256": deployment_source_sha256(),
            "bridge_yaml_sha256": sha256_file(args.bridge_config),
            "bridge_config_sha256": semantic_bridge_sha,
            "training_git_sha": training_git_sha,
            **training_sources,
        },
        "vectors": {
            "sample_count": replay["sample_count"],
            "episode_count": replay["episode_count"],
            "ordered_trajectory": True,
        },
        "torch_onnx": {
            "passed": True,
            "rtol": float(args.rtol),
            "atol": float(args.atol),
            "max_abs_error": _max_abs(torch_action, onnx_action),
            "max_rel_error": _max_relative(torch_action, onnx_action),
            "torch_expected_max_abs_error": _max_abs(torch_action, expected_action),
        },
        "policy_expected": replay["policy_expected_parity"],
        "ros_decoder": replay["ros_decoder"],
        "range": {
            "max_abs_raw_action": replay["max_abs_raw_action"],
            "configured_max_raw_action_abs": max_raw_action_abs,
        },
        "passed": True,
    }
    # JSON forbids NaN here so the report can be independently audited.
    serialized = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(serialized + "\n", encoding="utf-8")
    print(serialized)
    print(f"golden report: {report_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare TorchScript and ONNX. With --golden-vectors, also replay "
            "the ordered ROS ActionDecoder trajectory and create a hash-bound JSON report."
        )
    )
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--torchscript", required=True)
    parser.add_argument("--obs-npy", default="", help="Legacy tensor-only smoke-test input")
    parser.add_argument("--golden-vectors", default="", help="Ordered redrhex_golden_v2 NPZ")
    parser.add_argument("--controller-config", default="", help="Exact deployment controller YAML")
    parser.add_argument("--bridge-config", default="", help="Exact low-level bridge YAML")
    parser.add_argument(
        "--disabled-legs",
        default=None,
        help="Optional comma-separated runtime hardware mask, for example L1",
    )
    parser.add_argument("--training-git-sha", default="")
    parser.add_argument("--training-env-source", default="")
    parser.add_argument("--training-env-config-source", default="")
    parser.add_argument("--training-play-source", default="")
    parser.add_argument("--report-json", default="")
    parser.add_argument("--obs-dim", type=int, default=C.OBS_DIM_SINGLE)
    parser.add_argument("--rtol", type=float, default=1.0e-4)
    parser.add_argument("--atol", type=float, default=1.0e-4)
    parser.add_argument("--decoder-atol", type=float, default=1.0e-5)
    args = parser.parse_args()
    for name in ("rtol", "atol", "decoder_atol"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and non-negative")
    if args.golden_vectors:
        return _golden_compare(args)
    if args.controller_config or args.report_json or args.training_git_sha:
        raise ValueError("golden report options require --golden-vectors")
    return _simple_compare(args)


if __name__ == "__main__":
    raise SystemExit(main())
