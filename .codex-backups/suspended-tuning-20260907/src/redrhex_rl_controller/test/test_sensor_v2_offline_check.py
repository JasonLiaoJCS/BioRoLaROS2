from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _script_environment(package_root: Path) -> dict[str, str]:
    environment = dict(os.environ)
    source_root = str(package_root)
    environment["PYTHONPATH"] = (
        source_root
        if not environment.get("PYTHONPATH")
        else source_root + os.pathsep + environment["PYTHONPATH"]
    )
    return environment


def _write_sensor_v2_fixture(tmp_path: Path) -> tuple[Path, Path, str]:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    contract = {
        "contract_id": "redrhex.student-observation.v2",
        "history_length": 60,
        "history_order": "oldest_to_newest",
        "sensor_frame_dim": 36,
        "command": {
            "dimension": 3,
            "ordering": ["vx", "vy", "wz"],
            "external_input": True,
        },
    }
    action_contract = {
        "action_dim": 12,
        "strict_forward_command_gate": {
            "active_when_vx_greater_than_m_s": 0.1,
            "max_abs_vy_m_s": 0.08,
            "max_abs_wz_rad_s": 0.1,
        },
    }
    feature_layout = {"features": []}
    calibration_payload = {"hardware_ready": True, "readiness_blockers": []}
    training_calibration_payload = {
        "hardware_ready": False,
        "readiness_blockers": ["synthetic_fixture"],
    }
    metadata = {
        "action_contract_sha256": _canonical_sha256(action_contract),
        "architecture_sha256": "1" * 64,
        "bundle_schema": "redrhex.sensor-policy-bundle.v2",
        "bundle_version": "2",
        "calibration_sha256": _canonical_sha256(calibration_payload),
        "canonical_config_sha256": "2" * 64,
        "checkpoint_kind": "student_ppo_v2",
        "checkpoint_sha256": "3" * 64,
        "config_sha256": "4" * 64,
        "contact_supervision": "disabled",
        "contract_id": "redrhex.student-observation.v2",
        "contract_sha256": _canonical_sha256(contract),
        "feature_layout_sha256": _canonical_sha256(feature_layout),
        "stage": "minimal_direct_ppo",
        "training_calibration_sha256": _canonical_sha256(
            training_calibration_payload
        ),
        "training_seed": "42",
    }
    calibration = dict(calibration_payload, sha256=metadata["calibration_sha256"])
    training_calibration = dict(
        training_calibration_payload,
        sha256=metadata["training_calibration_sha256"],
    )

    action = numpy_helper.from_array(
        np.zeros((1, 12), dtype=np.float32), name="constant_action"
    )
    velocity = numpy_helper.from_array(
        np.zeros((1, 3), dtype=np.float32), name="constant_velocity"
    )
    graph = helper.make_graph(
        [
            helper.make_node("Constant", [], ["actions"], value=action),
            helper.make_node(
                "Constant", [], ["base_velocity_estimate"], value=velocity
            ),
        ],
        "experimental_sensor_v2_fixture",
        [
            helper.make_tensor_value_info(
                "sensor_history", TensorProto.FLOAT, [1, 60, 36]
            ),
            helper.make_tensor_value_info("command", TensorProto.FLOAT, [1, 3]),
        ],
        [
            helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 12]),
            helper.make_tensor_value_info(
                "base_velocity_estimate", TensorProto.FLOAT, [1, 3]
            ),
        ],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 18)])
    model.ir_version = 8
    for key, value in metadata.items():
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.checker.check_model(model)
    onnx_path = tmp_path / "policy.onnx"
    onnx.save(model, onnx_path)

    sidecar = {
        "action_contract": action_contract,
        "calibration": calibration,
        "contract": contract,
        "feature_layout": feature_layout,
        "io": {
            "inputs": {"sensor_history": [1, 60, 36], "command": [1, 3]},
            "outputs": {
                "actions": [1, 12],
                "base_velocity_estimate": [1, 3],
            },
        },
        "metadata": metadata,
        "torch_onnx_parity": {"status": "passed", "sample_count": 1},
        "training_calibration": training_calibration,
    }
    sidecar_path = tmp_path / "policy.onnx.json"
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    expected_sha = hashlib.sha256(onnx_path.read_bytes()).hexdigest()
    return onnx_path, sidecar_path, expected_sha


def _write_obs56_fixture(tmp_path: Path) -> Path:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    action = numpy_helper.from_array(
        np.zeros((1, 12), dtype=np.float32), name="constant_action"
    )
    graph = helper.make_graph(
        [helper.make_node("Constant", [], ["actions"], value=action)],
        "obs56_fixture",
        [helper.make_tensor_value_info("observation", TensorProto.FLOAT, [1, 56])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 12])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 18)])
    model.ir_version = 8
    path = tmp_path / "obs56.onnx"
    onnx.save(model, path)
    return path


def test_experimental_sensor_v2_mode_checks_contract_and_inference(tmp_path: Path) -> None:
    pytest.importorskip("onnxruntime")
    onnx_path, sidecar_path, expected_sha = _write_sensor_v2_fixture(tmp_path)
    package_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [
            sys.executable,
            str(package_root / "scripts" / "check_onnx_io.py"),
            str(onnx_path),
            "--contract",
            "experimental-sensor-v2",
            "--expected-sha256",
            expected_sha,
            "--sidecar",
            str(sidecar_path),
        ],
        check=False,
        env=_script_environment(package_root),
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "fixed command: [0.22, 0.0, 0.0]" in result.stdout
    assert "EXPERIMENTAL SENSOR-V2 OFFLINE CHECK OK" in result.stdout
    assert "NOT DEPLOYABLE BY THE CURRENT OBS56/OBS280 RUNTIME" in result.stdout


def test_sensor_v2_is_recognized_but_rejected_in_default_mode(tmp_path: Path) -> None:
    pytest.importorskip("onnxruntime")
    onnx_path, _, _ = _write_sensor_v2_fixture(tmp_path)
    package_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, str(package_root / "scripts" / "check_onnx_io.py"), str(onnx_path)],
        check=False,
        env=_script_environment(package_root),
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "sensor-v2 contract detected" in (result.stdout + result.stderr)


def test_experimental_sensor_v2_mode_rejects_wrong_sha(tmp_path: Path) -> None:
    pytest.importorskip("onnxruntime")
    onnx_path, sidecar_path, _ = _write_sensor_v2_fixture(tmp_path)
    package_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [
            sys.executable,
            str(package_root / "scripts" / "check_onnx_io.py"),
            str(onnx_path),
            "--contract",
            "experimental-sensor-v2",
            "--expected-sha256",
            "0" * 64,
            "--sidecar",
            str(sidecar_path),
        ],
        check=False,
        env=_script_environment(package_root),
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "ONNX SHA256 mismatch" in (result.stdout + result.stderr)


def test_experimental_sensor_v2_mode_rejects_sidecar_metadata_mismatch(
    tmp_path: Path,
) -> None:
    pytest.importorskip("onnxruntime")
    onnx_path, sidecar_path, expected_sha = _write_sensor_v2_fixture(tmp_path)
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    sidecar["metadata"]["training_seed"] = "99"
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    package_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [
            sys.executable,
            str(package_root / "scripts" / "check_onnx_io.py"),
            str(onnx_path),
            "--contract",
            "experimental-sensor-v2",
            "--expected-sha256",
            expected_sha,
            "--sidecar",
            str(sidecar_path),
        ],
        check=False,
        env=_script_environment(package_root),
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "sidecar/ONNX metadata" in (result.stdout + result.stderr)


def test_obs56_default_mode_is_unchanged(tmp_path: Path) -> None:
    pytest.importorskip("onnxruntime")
    onnx_path = _write_obs56_fixture(tmp_path)
    package_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, str(package_root / "scripts" / "check_onnx_io.py"), str(onnx_path)],
        check=False,
        env=_script_environment(package_root),
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "ONNX I/O check OK" in result.stdout


def test_obs56_packager_recognizes_sensor_v2_as_offline_only(tmp_path: Path) -> None:
    onnx_path, _, _ = _write_sensor_v2_fixture(tmp_path)
    package_root = Path(__file__).resolve().parents[1]
    dummy_paths = {
        name: tmp_path / name
        for name in (
            "policy.pt",
            "golden.npz",
            "controller.yaml",
            "bridge.yaml",
            "training_env.py",
            "training_env_config.py",
            "report.json",
        )
    }
    for path in dummy_paths.values():
        path.write_text("{}", encoding="utf-8")
    destination = tmp_path / "must_not_exist.onnx"
    result = subprocess.run(
        [
            sys.executable,
            str(package_root / "scripts" / "package_verified_policy.py"),
            str(onnx_path),
            str(destination),
            "--torchscript",
            str(dummy_paths["policy.pt"]),
            "--golden-vectors",
            str(dummy_paths["golden.npz"]),
            "--controller-config",
            str(dummy_paths["controller.yaml"]),
            "--bridge-config",
            str(dummy_paths["bridge.yaml"]),
            "--training-git-sha",
            "a" * 40,
            "--training-env-source",
            str(dummy_paths["training_env.py"]),
            "--training-env-config-source",
            str(dummy_paths["training_env_config.py"]),
            "--golden-report",
            str(dummy_paths["report.json"]),
        ],
        check=False,
        env=_script_environment(package_root),
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "experimental sensor-v2 contract recognized" in (result.stdout + result.stderr)
    assert not destination.exists()
