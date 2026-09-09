#!/usr/bin/env python3
"""Package a hash-bound RedRhex policy only after recomputing golden parity."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import onnx

from redrhex_rl_controller import redrhex_contract as C
from redrhex_rl_controller.golden_policy import (
    GOLDEN_SCHEMA,
    MIN_GOLDEN_EPISODES,
    MIN_GOLDEN_SAMPLES,
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
from redrhex_rl_controller.policy_onnx_runner import PolicyONNXRunner
from redrhex_rl_controller.policy_validation import (
    check_reference_action,
    deployment_metadata_rejection_reasons,
    reference_policy_input,
)


def _run_torchscript(path: Path, observations: np.ndarray) -> np.ndarray:
    try:
        import torch
    except Exception as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("Torch is required to re-verify the golden bundle") from exc
    module = torch.jit.load(str(path), map_location="cpu")
    module.eval()
    with torch.no_grad():
        output = module(torch.from_numpy(observations.astype(np.float32)))
    if not hasattr(output, "detach"):
        raise ValueError("TorchScript policy must return one tensor")
    action = output.detach().cpu().numpy().astype(np.float32)
    if action.shape != (observations.shape[0], C.ACTION_DIM):
        raise ValueError(f"unexpected TorchScript action shape {action.shape}")
    if not np.isfinite(action).all():
        raise ValueError("TorchScript action contains NaN or Inf")
    return action


def _require_hex(value: object, name: str, min_length: int = 64, max_length: int = 64) -> str:
    text = str(value).strip().lower()
    if re.fullmatch(rf"[0-9a-f]{{{min_length},{max_length}}}", text) is None:
        raise ValueError(f"{name} must be {min_length}-{max_length} hexadecimal characters")
    return text


def _require_equal(actual: object, expected: object, name: str) -> None:
    if actual != expected:
        raise ValueError(f"golden report {name}={actual!r}, expected {expected!r}")


def _validate_report(
    report: dict,
    *,
    source: Path,
    torchscript: Path,
    vectors: Path,
    controller_config: Path,
    bridge_config: Path,
    bridge_params: dict,
    training_env_source: Path,
    training_env_config_source: Path,
    training_play_source: Path | None,
    decoder_config: dict,
    controller_params: dict,
    training_git_sha: str,
    max_raw_action_abs: float,
) -> tuple[float, float, float]:
    _require_equal(report.get("schema"), GOLDEN_SCHEMA, "schema")
    _require_equal(report.get("passed"), True, "passed")
    contracts = report.get("contracts")
    artifacts = report.get("artifacts")
    vector_info = report.get("vectors")
    if not isinstance(contracts, dict) or not isinstance(artifacts, dict) or not isinstance(vector_info, dict):
        raise ValueError("golden report is missing contracts/artifacts/vectors objects")
    _require_equal(contracts.get("observation"), C.OBSERVATION_CONTRACT_ID, "contracts.observation")
    _require_equal(contracts.get("action_decoder"), C.ACTION_DECODER_CONTRACT_ID, "contracts.action_decoder")
    _require_equal(contracts.get("action_dim"), C.ACTION_DIM, "contracts.action_dim")
    _require_equal(contracts.get("training_action_clip"), C.TRAINING_ACTION_CLIP, "contracts.training_action_clip")
    _require_equal(contracts.get("normalizer"), C.NORMALIZER_EMBEDDED, "contracts.normalizer")
    _require_equal(
        contracts.get("disabled_legs"),
        list(controller_params.get("hardware", {}).get("disabled_legs", [])),
        "contracts.disabled_legs",
    )

    expected_hashes = {
        "onnx_sha256": sha256_file(source),
        "torchscript_sha256": sha256_file(torchscript),
        "vectors_sha256": sha256_file(vectors),
        "controller_yaml_sha256": sha256_file(controller_config),
        "controller_config_sha256": controller_config_sha256(controller_params),
        "decoder_config_sha256": decoder_config_sha256(decoder_config),
        "decoder_source_sha256": decoder_source_sha256(),
        "observation_source_sha256": observation_source_sha256(),
        "deployment_source_sha256": deployment_source_sha256(),
        "bridge_yaml_sha256": sha256_file(bridge_config),
        "bridge_config_sha256": bridge_config_sha256(bridge_params),
        "training_env_source_sha256": sha256_file(training_env_source),
        "training_env_config_source_sha256": sha256_file(training_env_config_source),
    }
    if training_play_source is not None:
        expected_hashes["training_play_source_sha256"] = sha256_file(
            training_play_source
        )
    for name, expected in expected_hashes.items():
        actual = _require_hex(artifacts.get(name), f"artifacts.{name}")
        _require_equal(actual, expected, f"artifacts.{name}")
    _require_equal(
        _require_hex(artifacts.get("training_git_sha"), "artifacts.training_git_sha", 7, 64),
        training_git_sha,
        "artifacts.training_git_sha",
    )
    _require_equal(vector_info.get("ordered_trajectory"), True, "vectors.ordered_trajectory")
    if (
        int(vector_info.get("sample_count", 0)) < MIN_GOLDEN_SAMPLES
        or int(vector_info.get("episode_count", 0)) < MIN_GOLDEN_EPISODES
    ):
        raise ValueError("golden report does not contain a sufficiently long ordered trajectory")

    torch_onnx = report.get("torch_onnx")
    policy_expected = report.get("policy_expected")
    ros_decoder = report.get("ros_decoder")
    if not all(isinstance(item, dict) for item in (torch_onnx, policy_expected, ros_decoder)):
        raise ValueError("golden report is missing parity result objects")
    for name, item in (
        ("torch_onnx", torch_onnx),
        ("policy_expected", policy_expected),
        ("ros_decoder", ros_decoder),
    ):
        _require_equal(item.get("passed"), True, f"{name}.passed")
    range_info = report.get("range")
    if not isinstance(range_info, dict):
        raise ValueError("golden report is missing range object")
    report_max_raw = float(range_info.get("max_abs_raw_action", math.inf))
    report_configured_limit = float(
        range_info.get("configured_max_raw_action_abs", math.nan)
    )
    if (
        not math.isfinite(report_max_raw)
        or report_max_raw > max_raw_action_abs
        or not math.isclose(
            report_configured_limit,
            max_raw_action_abs,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
    ):
        raise ValueError(
            "golden report raw-action range exceeds or does not match the deployment limit"
        )
    rtol = float(torch_onnx.get("rtol"))
    atol = float(torch_onnx.get("atol"))
    decoder_atol = float(ros_decoder.get("atol"))
    for name, value, maximum in (
        ("torch_onnx.rtol", rtol, 1.0e-4),
        ("torch_onnx.atol", atol, 1.0e-4),
        ("ros_decoder.atol", decoder_atol, 1.0e-5),
    ):
        if not math.isfinite(value) or value < 0.0 or value > maximum:
            raise ValueError(f"golden report {name}={value} exceeds allowed {maximum}")
    return rtol, atol, decoder_atol


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Recompute Torch/ONNX/ROS decoder parity, then copy an already "
            "contract-tagged source ONNX into an immutable hardware package."
        )
    )
    parser.add_argument("input_onnx")
    parser.add_argument("output_onnx")
    parser.add_argument("--torchscript", required=True)
    parser.add_argument("--golden-vectors", required=True)
    parser.add_argument("--controller-config", required=True)
    parser.add_argument("--bridge-config", required=True)
    parser.add_argument(
        "--disabled-legs",
        default=None,
        help="Optional comma-separated runtime hardware mask, for example L1",
    )
    parser.add_argument("--training-git-sha", required=True)
    parser.add_argument("--training-env-source", required=True)
    parser.add_argument("--training-env-config-source", required=True)
    parser.add_argument("--training-play-source", default="")
    parser.add_argument("--golden-report", required=True)
    parser.add_argument("--max-reference-action", type=float, default=1.5)
    args = parser.parse_args()

    source = Path(args.input_onnx).expanduser().resolve()
    destination = Path(args.output_onnx).expanduser().resolve()
    torchscript = Path(args.torchscript).expanduser().resolve()
    vectors = Path(args.golden_vectors).expanduser().resolve()
    controller_config = Path(args.controller_config).expanduser().resolve()
    bridge_config = Path(args.bridge_config).expanduser().resolve()
    training_env_source = Path(args.training_env_source).expanduser().resolve()
    training_env_config_source = Path(
        args.training_env_config_source
    ).expanduser().resolve()
    training_play_source = (
        Path(args.training_play_source).expanduser().resolve()
        if args.training_play_source
        else None
    )
    report_path = Path(args.golden_report).expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"input ONNX does not exist: {source}")
    metadata_model = onnx.load(str(source), load_external_data=False)
    source_metadata = {item.key: item.value for item in metadata_model.metadata_props}
    metadata_rejections = deployment_metadata_rejection_reasons(source_metadata)
    if metadata_rejections:
        raise SystemExit(
            "source ONNX metadata explicitly blocks deployment: "
            + "; ".join(metadata_rejections)
        )
    if source_metadata.get("bundle_schema") == "redrhex.sensor-policy-bundle.v2":
        raise SystemExit(
            "experimental sensor-v2 contract recognized, but this packager only emits "
            "obs56/obs280 runtime artifacts; validate sensor-v2 offline with "
            "check_onnx_io.py --contract experimental-sensor-v2 and do not write it "
            "to the active policy YAML"
        )
    for name, path in (
        ("TorchScript", torchscript),
        ("golden vectors", vectors),
        ("controller config", controller_config),
        ("bridge config", bridge_config),
        ("training env source", training_env_source),
        ("training env config source", training_env_config_source),
        ("golden report", report_path),
    ):
        if not path.is_file():
            raise SystemExit(f"{name} does not exist: {path}")
    if training_play_source is not None and not training_play_source.is_file():
        raise SystemExit(f"training play source does not exist: {training_play_source}")
    if destination.exists():
        raise SystemExit(f"output already exists; choose a new immutable path: {destination}")
    if destination == source:
        raise SystemExit("input and output must be different files")
    source_sha256 = sha256_file(source)
    training_git_sha = _require_hex(args.training_git_sha, "training_git_sha", 7, 64)
    if not math.isfinite(args.max_reference_action) or args.max_reference_action <= 0.0:
        raise SystemExit("--max-reference-action must be positive and finite")

    report = json.loads(report_path.read_text(encoding="utf-8"))
    if not isinstance(report, dict):
        raise SystemExit("golden report root must be an object")
    disabled_legs = (
        None
        if args.disabled_legs is None
        else [item.strip() for item in args.disabled_legs.split(",") if item.strip()]
    )
    params = controller_params_with_disabled_legs(
        load_controller_ros_params(controller_config), disabled_legs
    )
    bridge_params = bridge_params_with_disabled_legs(
        load_bridge_ros_params(bridge_config), disabled_legs
    )
    semantic_bridge_sha = bridge_config_sha256(bridge_params)
    configured_bridge_sha = str(
        params.get("policy", {}).get("expected_bridge_config_sha256", "")
    ).strip()
    if configured_bridge_sha != semantic_bridge_sha:
        raise SystemExit(
            "controller expected bridge hash does not match --bridge-config: "
            f"{configured_bridge_sha or 'missing'} != {semantic_bridge_sha}"
        )
    decoder_config = decoder_config_from_ros_params(params)
    max_raw_action_abs = float(
        params.get("safety", {}).get("max_raw_action_abs", 1.5)
    )
    if not math.isfinite(max_raw_action_abs) or max_raw_action_abs <= 0.0:
        raise SystemExit("safety.max_raw_action_abs must be positive and finite")
    rtol, atol, decoder_atol = _validate_report(
        report,
        source=source,
        torchscript=torchscript,
        vectors=vectors,
        controller_config=controller_config,
        bridge_config=bridge_config,
        bridge_params=bridge_params,
        training_env_source=training_env_source,
        training_env_config_source=training_env_config_source,
        training_play_source=training_play_source,
        decoder_config=decoder_config,
        controller_params=params,
        training_git_sha=training_git_sha,
        max_raw_action_abs=max_raw_action_abs,
    )

    replay = replay_onnx_and_decoder(
        source,
        vectors,
        decoder_config,
        policy_rtol=rtol,
        policy_atol=atol,
        decoder_atol=decoder_atol,
    )
    require_source_export_metadata(replay, training_git_sha)
    require_replay_passed(replay)
    if float(replay["max_abs_raw_action"]) > max_raw_action_abs:
        raise SystemExit(
            "recomputed golden trajectory raw action exceeds deployment hard-stop limit: "
            f"{replay['max_abs_raw_action']:.6g} > {max_raw_action_abs:.6g}"
        )
    vector_data = load_golden_vectors(vectors)
    require_golden_provenance(
        vector_data,
        training_git_sha=training_git_sha,
        training_env_source=training_env_source,
        training_env_config_source=training_env_config_source,
        training_play_source=training_play_source,
    )
    require_trajectory_matches_controller(vector_data, params)
    torch_action = _run_torchscript(torchscript, vector_data["policy_input"])
    onnx_action = np.asarray(replay["onnx_action"], dtype=np.float32)
    expected_action = np.asarray(vector_data["expected_policy_action"], dtype=np.float32)
    if not np.allclose(torch_action, onnx_action, rtol=rtol, atol=atol):
        raise SystemExit("recomputed Torch/ONNX parity failed")
    if not np.allclose(torch_action, expected_action, rtol=rtol, atol=atol):
        raise SystemExit("recomputed Torch/golden-action parity failed")

    runner = PolicyONNXRunner(str(source))
    input_dim = runner.obs_dim or C.OBS_DIM_SINGLE
    reference_action = runner.run(reference_policy_input(input_dim))
    action_ok, max_abs_action = check_reference_action(
        reference_action, args.max_reference_action
    )
    if not action_ok:
        raise SystemExit(
            f"reference action max abs {max_abs_action:.6f} exceeds "
            f"{args.max_reference_action:.6f}"
        )

    if sha256_file(source) != source_sha256:
        raise SystemExit("source ONNX changed during verification; start again")
    # Re-read both the model and metadata at the final save boundary.  This
    # preserves external tensor data and rechecks deployment status before save.
    model = onnx.load(str(source))
    metadata = {item.key: item.value for item in model.metadata_props}
    metadata_rejections = deployment_metadata_rejection_reasons(metadata)
    if metadata_rejections:
        raise SystemExit(
            "source ONNX metadata explicitly blocks deployment: "
            + "; ".join(metadata_rejections)
        )
    # The semantic contract keys must already be emitted by the training
    # exporter and were exercised above.  This packager only adds evidence;
    # it never upgrades an untagged shape-only model into a stage-5 model.
    metadata.update(
        {
            C.ONNX_DECODER_SOURCE_SHA256_KEY: decoder_source_sha256(),
            C.ONNX_DECODER_CONFIG_SHA256_KEY: decoder_config_sha256(decoder_config),
            C.ONNX_GOLDEN_SCHEMA_KEY: GOLDEN_SCHEMA,
            C.ONNX_TRAINING_GIT_SHA_KEY: training_git_sha,
            "redrhex.source_onnx_sha256": source_sha256,
            "redrhex.torchscript_sha256": sha256_file(torchscript),
            "redrhex.golden_vectors_sha256": sha256_file(vectors),
            "redrhex.controller_yaml_sha256": sha256_file(controller_config),
            C.ONNX_CONTROLLER_CONFIG_SHA256_KEY: controller_config_sha256(params),
            C.ONNX_OBSERVATION_SOURCE_SHA256_KEY: observation_source_sha256(),
            C.ONNX_DEPLOYMENT_SOURCE_SHA256_KEY: deployment_source_sha256(),
            C.ONNX_BRIDGE_CONFIG_SHA256_KEY: semantic_bridge_sha,
            C.ONNX_TRAINING_ENV_SOURCE_SHA256_KEY: sha256_file(training_env_source),
            C.ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY: sha256_file(
                training_env_config_source
            ),
            "redrhex.golden_report_sha256": sha256_file(report_path),
        }
    )
    if training_play_source is not None:
        metadata[C.ONNX_TRAINING_PLAY_SOURCE_SHA256_KEY] = sha256_file(
            training_play_source
        )
    del model.metadata_props[:]
    for key, value in sorted(metadata.items()):
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = str(value)
    destination.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(destination))
    print(f"packaged: {destination}")
    print(f"source sha256: {source_sha256}")
    print(f"packaged sha256: {sha256_file(destination)}")
    print(f"golden report sha256: {sha256_file(report_path)}")
    print(f"reference max abs action: {max_abs_action:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
