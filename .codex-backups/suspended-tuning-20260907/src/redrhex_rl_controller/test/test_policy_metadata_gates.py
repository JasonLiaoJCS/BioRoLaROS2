from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import pytest

from redrhex_rl_controller import redrhex_contract as C
from redrhex_rl_controller.policy_onnx_runner import PolicyONNXRunner


def _write_zero_policy(path: Path, metadata: dict[str, str]) -> None:
    output = numpy_helper.from_array(
        np.zeros((1, C.ACTION_DIM), dtype=np.float32), name="zero_action"
    )
    graph = helper.make_graph(
        [helper.make_node("Constant", inputs=[], outputs=["action"], value=output)],
        "metadata_gate_policy",
        [
            helper.make_tensor_value_info(
                "observation", TensorProto.FLOAT, [1, C.OBS_DIM_SINGLE]
            )
        ],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, C.ACTION_DIM])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    for key, value in metadata.items():
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.checker.check_model(model)
    onnx.save(model, path)


def _script_environment(package_root: Path) -> dict[str, str]:
    environment = dict(os.environ)
    source_root = str(package_root)
    environment["PYTHONPATH"] = (
        source_root
        if not environment.get("PYTHONPATH")
        else source_root + os.pathsep + environment["PYTHONPATH"]
    )
    return environment


def test_runtime_runner_rejects_quality_rejected_metadata(tmp_path: Path) -> None:
    policy = tmp_path / "quality_rejected.onnx"
    _write_zero_policy(policy, {C.ONNX_QUALITY_STATUS_KEY: "quality_rejected"})

    with pytest.raises(ValueError, match="quality_rejected"):
        PolicyONNXRunner(str(policy))


def test_check_onnx_io_rejects_before_printing_success(tmp_path: Path) -> None:
    policy = tmp_path / "diagnostic.onnx"
    _write_zero_policy(
        policy,
        {C.ONNX_ARTIFACT_STATUS_KEY: "diagnostic_only_not_deployable"},
    )
    package_root = Path(__file__).resolve().parents[1]
    completed = subprocess.run(
        [
            sys.executable,
            str(package_root / "scripts" / "check_onnx_io.py"),
            str(policy),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=_script_environment(package_root),
    )

    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "diagnostic_only_not_deployable" in output
    assert "ONNX I/O check OK" not in output


def test_packager_rejects_metadata_before_reading_report(tmp_path: Path) -> None:
    policy = tmp_path / "diagnostic.onnx"
    _write_zero_policy(
        policy,
        {C.ONNX_ARTIFACT_STATUS_KEY: "diagnostic_only_not_deployable"},
    )
    required_files = {
        "torchscript": tmp_path / "policy.pt",
        "vectors": tmp_path / "golden.npz",
        "controller": tmp_path / "controller.yaml",
        "bridge": tmp_path / "bridge.yaml",
        "training_env": tmp_path / "training_env.py",
        "training_config": tmp_path / "training_env_config.py",
        "training_play": tmp_path / "training_play.py",
        "report": tmp_path / "invalid_report.json",
    }
    for path in required_files.values():
        path.write_text("not parsed before metadata rejection", encoding="utf-8")

    package_root = Path(__file__).resolve().parents[1]
    destination = tmp_path / "must_not_exist.onnx"
    completed = subprocess.run(
        [
            sys.executable,
            str(package_root / "scripts" / "package_verified_policy.py"),
            str(policy),
            str(destination),
            "--torchscript",
            str(required_files["torchscript"]),
            "--golden-vectors",
            str(required_files["vectors"]),
            "--controller-config",
            str(required_files["controller"]),
            "--bridge-config",
            str(required_files["bridge"]),
            "--training-git-sha",
            "a" * 40,
            "--training-env-source",
            str(required_files["training_env"]),
            "--training-env-config-source",
            str(required_files["training_config"]),
            "--training-play-source",
            str(required_files["training_play"]),
            "--golden-report",
            str(required_files["report"]),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=_script_environment(package_root),
    )

    output = completed.stdout + completed.stderr
    assert completed.returncode != 0
    assert "diagnostic_only_not_deployable" in output
    assert not destination.exists()
