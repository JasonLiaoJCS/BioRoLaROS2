#!/usr/bin/env python3
"""Inspect and smoke-test a RedRhex policy.onnx."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np

from redrhex_rl_controller.policy_validation import (
    check_reference_action,
    deployment_metadata_rejection_reasons,
    reference_policy_input,
)


_OBS56_CONTRACT = "obs56"
_SENSOR_V2_CONTRACT = "experimental-sensor-v2"
_SENSOR_V2_SCHEMA = "redrhex.sensor-policy-bundle.v2"
_SENSOR_V2_CONTRACT_ID = "redrhex.student-observation.v2"
_SENSOR_V2_FIXED_COMMAND = np.asarray([[0.22, 0.0, 0.0]], dtype=np.float32)
_SHA256_RE = re.compile(r"[0-9a-f]{64}")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_expected_sha256(path: Path, expected: str) -> str:
    expected = str(expected).strip().lower()
    if _SHA256_RE.fullmatch(expected) is None:
        raise SystemExit("--expected-sha256 must be exactly 64 hexadecimal characters")
    actual = _sha256_file(path)
    if actual != expected:
        raise SystemExit(f"ONNX SHA256 mismatch: actual {actual}, expected {expected}")
    print(f"onnx sha256: {actual}")
    return actual


def _canonical_json_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _require_mapping(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise SystemExit(f"sensor-v2 sidecar {name} must be an object")
    return value


def _require_equal(actual: object, expected: object, name: str) -> None:
    if actual != expected:
        raise SystemExit(
            f"sensor-v2 {name} mismatch: actual {actual!r}, expected {expected!r}"
        )


def _require_finite_tensor(value: np.ndarray, name: str, shape: tuple[int, ...]) -> None:
    if value.shape != shape:
        raise SystemExit(f"sensor-v2 {name} shape {value.shape} != {shape}")
    if not np.isfinite(value).all():
        raise SystemExit(f"sensor-v2 {name} contains NaN/Inf")


def _validate_sensor_v2_sidecar(
    sidecar_path: Path, metadata: dict[str, str]
) -> dict:
    try:
        sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot read sensor-v2 sidecar {sidecar_path}: {exc}") from exc
    sidecar = _require_mapping(sidecar, "root")
    sidecar_metadata = _require_mapping(sidecar.get("metadata"), "metadata")
    _require_equal(sidecar_metadata, metadata, "sidecar/ONNX metadata")

    required_metadata = {
        "bundle_schema": _SENSOR_V2_SCHEMA,
        "bundle_version": "2",
        "contract_id": _SENSOR_V2_CONTRACT_ID,
        "checkpoint_kind": "student_ppo_v2",
        "stage": "minimal_direct_ppo",
    }
    for key, expected in required_metadata.items():
        _require_equal(metadata.get(key), expected, f"metadata.{key}")
    for key in (
        "action_contract_sha256",
        "architecture_sha256",
        "calibration_sha256",
        "canonical_config_sha256",
        "checkpoint_sha256",
        "config_sha256",
        "contract_sha256",
        "feature_layout_sha256",
        "training_calibration_sha256",
    ):
        value = str(metadata.get(key, "")).strip().lower()
        if _SHA256_RE.fullmatch(value) is None:
            raise SystemExit(f"sensor-v2 metadata.{key} must be a SHA256")

    expected_io = {
        "inputs": {"sensor_history": [1, 60, 36], "command": [1, 3]},
        "outputs": {"actions": [1, 12], "base_velocity_estimate": [1, 3]},
    }
    _require_equal(sidecar.get("io"), expected_io, "sidecar.io")

    hash_bindings = (
        ("contract", "contract_sha256", False),
        ("action_contract", "action_contract_sha256", False),
        ("feature_layout", "feature_layout_sha256", False),
        ("calibration", "calibration_sha256", True),
        ("training_calibration", "training_calibration_sha256", True),
    )
    for object_name, metadata_key, has_embedded_sha in hash_bindings:
        value = dict(_require_mapping(sidecar.get(object_name), object_name))
        if has_embedded_sha:
            embedded = value.pop("sha256", None)
            _require_equal(embedded, metadata[metadata_key], f"{object_name}.sha256")
        _require_equal(
            _canonical_json_sha256(value),
            metadata[metadata_key],
            f"{object_name} canonical SHA256",
        )

    contract = _require_mapping(sidecar.get("contract"), "contract")
    _require_equal(contract.get("contract_id"), _SENSOR_V2_CONTRACT_ID, "contract_id")
    _require_equal(contract.get("history_length"), 60, "contract.history_length")
    _require_equal(contract.get("history_order"), "oldest_to_newest", "contract.history_order")
    _require_equal(contract.get("sensor_frame_dim"), 36, "contract.sensor_frame_dim")
    command_contract = _require_mapping(contract.get("command"), "contract.command")
    _require_equal(command_contract.get("dimension"), 3, "contract.command.dimension")
    _require_equal(
        command_contract.get("ordering"), ["vx", "vy", "wz"], "contract.command.ordering"
    )
    _require_equal(command_contract.get("external_input"), True, "contract.command.external_input")

    action_contract = _require_mapping(sidecar.get("action_contract"), "action_contract")
    _require_equal(action_contract.get("action_dim"), 12, "action_contract.action_dim")
    gate = _require_mapping(
        action_contract.get("strict_forward_command_gate"),
        "action_contract.strict_forward_command_gate",
    )
    vx_threshold = float(gate.get("active_when_vx_greater_than_m_s", math.inf))
    max_vy = float(gate.get("max_abs_vy_m_s", math.nan))
    max_wz = float(gate.get("max_abs_wz_rad_s", math.nan))
    if not (0.22 > vx_threshold and max_vy >= 0.0 and max_wz >= 0.0):
        raise SystemExit("sensor-v2 sidecar does not permit fixed command [0.22,0,0]")

    parity = _require_mapping(sidecar.get("torch_onnx_parity"), "torch_onnx_parity")
    _require_equal(parity.get("status"), "passed", "torch_onnx_parity.status")
    if int(parity.get("sample_count", 0)) < 1:
        raise SystemExit("sensor-v2 sidecar has no Torch/ONNX parity samples")
    return sidecar


def _check_sensor_v2(
    sess: object,
    onnx_path: Path,
    metadata: dict[str, str],
    expected_sha256: str,
    sidecar_path: str,
    max_reference_action: float,
) -> int:
    if not expected_sha256:
        raise SystemExit("experimental sensor-v2 mode requires --expected-sha256")
    if not sidecar_path:
        raise SystemExit("experimental sensor-v2 mode requires --sidecar")
    _require_expected_sha256(onnx_path, expected_sha256)
    _validate_sensor_v2_sidecar(Path(sidecar_path).expanduser().resolve(), metadata)

    inputs = {item.name: item for item in sess.get_inputs()}
    outputs = {item.name: item for item in sess.get_outputs()}
    if set(inputs) != {"sensor_history", "command"} or set(outputs) != {
        "actions",
        "base_velocity_estimate",
    }:
        raise SystemExit(
            "sensor-v2 ONNX must have inputs sensor_history/command and "
            "outputs actions/base_velocity_estimate"
        )
    expected_shapes = {
        "sensor_history": [1, 60, 36],
        "command": [1, 3],
        "actions": [1, 12],
        "base_velocity_estimate": [1, 3],
    }
    for name, item in {**inputs, **outputs}.items():
        if item.type != "tensor(float)":
            raise SystemExit(f"sensor-v2 {name} must be float32, got {item.type}")
        if list(item.shape) != expected_shapes[name]:
            raise SystemExit(
                f"sensor-v2 {name} shape {list(item.shape)} != {expected_shapes[name]}"
            )

    feeds = {
        "sensor_history": np.zeros((1, 60, 36), dtype=np.float32),
        "command": _SENSOR_V2_FIXED_COMMAND.copy(),
    }
    actions, velocity = sess.run(["actions", "base_velocity_estimate"], feeds)
    actions = np.asarray(actions, dtype=np.float32)
    velocity = np.asarray(velocity, dtype=np.float32)
    _require_finite_tensor(actions, "actions", (1, 12))
    _require_finite_tensor(velocity, "base_velocity_estimate", (1, 3))
    max_action = float(np.max(np.abs(actions)))
    if max_action > max_reference_action:
        raise SystemExit(
            f"sensor-v2 fixed-command action max abs {max_action:.6f} exceeds "
            f"{max_reference_action:.6f}"
        )
    print("contract: experimental sensor-v2 (offline validation only)")
    print("fixed command: [0.22, 0.0, 0.0]")
    print(f"fixed-command action max abs: {max_action:.6f}")
    print(f"base-velocity estimate: {velocity.reshape(-1).tolist()}")
    print("EXPERIMENTAL SENSOR-V2 OFFLINE CHECK OK")
    print("NOT DEPLOYABLE BY THE CURRENT OBS56/OBS280 RUNTIME OR PACKAGER")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("onnx_path")
    parser.add_argument("--expected-obs-dim", type=int, default=56)
    parser.add_argument("--expected-action-dim", type=int, default=12)
    parser.add_argument("--max-reference-action", type=float, default=1.5)
    parser.add_argument(
        "--contract",
        choices=(_OBS56_CONTRACT, _SENSOR_V2_CONTRACT),
        default=_OBS56_CONTRACT,
    )
    parser.add_argument("--expected-sha256", default="")
    parser.add_argument("--sidecar", default="")
    args = parser.parse_args()

    try:
        import onnxruntime as ort
    except Exception as exc:
        raise SystemExit("onnxruntime is not installed. Try: pip install onnxruntime") from exc

    sess = ort.InferenceSession(args.onnx_path, providers=["CPUExecutionProvider"])
    model_meta = sess.get_modelmeta()
    metadata = dict(getattr(model_meta, "custom_metadata_map", {}) or {})
    metadata_rejections = deployment_metadata_rejection_reasons(metadata)
    if metadata_rejections:
        raise SystemExit(
            "ONNX metadata explicitly blocks deployment: "
            + "; ".join(metadata_rejections)
        )
    onnx_path = Path(args.onnx_path).expanduser().resolve()
    if args.contract == _SENSOR_V2_CONTRACT:
        return _check_sensor_v2(
            sess,
            onnx_path,
            metadata,
            args.expected_sha256,
            args.sidecar,
            args.max_reference_action,
        )
    if metadata.get("bundle_schema") == _SENSOR_V2_SCHEMA:
        raise SystemExit(
            "sensor-v2 contract detected; it is not obs56/obs280. Re-run with "
            "--contract experimental-sensor-v2, --expected-sha256, and --sidecar "
            "for offline-only validation."
        )
    if args.expected_sha256:
        _require_expected_sha256(onnx_path, args.expected_sha256)
    if len(sess.get_inputs()) != 1 or len(sess.get_outputs()) != 1:
        raise SystemExit("ONNX must have exactly one input and one output")
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]
    if inp.type != "tensor(float)" or out.type != "tensor(float)":
        raise SystemExit("ONNX input and output must both be float32 tensors")
    print(f"providers: {sess.get_providers()}")
    print(f"input name:  {inp.name}")
    print(f"input shape: {inp.shape}")
    print(f"input type:  {inp.type}")
    print(f"output name:  {out.name}")
    print(f"output shape: {out.shape}")
    print(f"output type:  {out.type}")

    obs_dim = inp.shape[-1] if isinstance(inp.shape[-1], int) else args.expected_obs_dim
    if obs_dim not in (args.expected_obs_dim, args.expected_obs_dim * 5):
        raise SystemExit(
            f"Unexpected ONNX input dim {obs_dim}. Expected {args.expected_obs_dim} "
            f"or {args.expected_obs_dim * 5} for policy+history."
        )

    obs = np.zeros((1, obs_dim), dtype=np.float32)
    action = np.asarray(sess.run([out.name], {inp.name: obs})[0], dtype=np.float32)
    print(f"zero-observation action shape: {action.shape}")
    print(f"zero-observation action finite: {np.isfinite(action).all()}")
    print(f"zero-observation action min/max: {float(np.min(action)):.6f} / {float(np.max(action)):.6f}")
    if action.reshape(-1).shape[0] != args.expected_action_dim:
        raise SystemExit(f"Unexpected action dim {action.reshape(-1).shape[0]}")
    if not np.isfinite(action).all():
        raise SystemExit("ONNX output contains NaN/Inf")
    reference = reference_policy_input(obs_dim)
    reference_action = np.asarray(
        sess.run([out.name], {inp.name: reference.reshape(1, -1)})[0], dtype=np.float32
    ).reshape(-1)
    reference_ok, reference_max = check_reference_action(
        reference_action, args.max_reference_action
    )
    print(f"reference-observation action max abs: {reference_max:.6f}")
    if not reference_ok:
        raise SystemExit(
            "Reference observation produced implausible action: "
            f"{reference_max:.6f} > {args.max_reference_action:.6f}"
        )
    print("ONNX I/O check OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
