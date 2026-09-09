"""ONNX Runtime wrapper for RedRhex policy inference."""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .policy_validation import deployment_metadata_rejection_reasons


@dataclass(frozen=True)
class ONNXIOInfo:
    input_name: str
    input_shape: list[Any]
    input_type: str
    output_name: str
    output_shape: list[Any]
    output_type: str
    obs_dim: int | None
    action_dim: int | None
    providers: list[str]
    sha256: str
    metadata: dict[str, str]
    interface_kind: str = "legacy_obs"


class PolicyONNXRunner:
    """Small, defensive ONNX Runtime runner.

    The exported RSL-RL ONNX in this repo includes the actor observation
    normalizer when one exists because scripts/rsl_rl/play.py calls
    export_policy_as_onnx(policy_nn, normalizer=normalizer, ...).
    """

    def __init__(
        self,
        onnx_path: str,
        expected_obs_dim: int = 56,
        expected_action_dim: int = 12,
        use_cuda: bool = False,
        use_tensorrt: bool = False,
        allow_history_dim: bool = True,
        expected_observation_contract: str | None = None,
        expected_action_contract: str | None = None,
        expected_input_layout: str | None = None,
        require_contract_metadata: bool = False,
    ) -> None:
        try:
            import onnxruntime as ort
        except Exception as exc:  # pragma: no cover - depends on Jetson install
            raise RuntimeError(
                "onnxruntime is required. Install on Jetson with onnxruntime or onnxruntime-gpu."
            ) from exc

        self._ort = ort
        self.onnx_path = str(Path(onnx_path).expanduser())
        if not Path(self.onnx_path).exists():
            raise FileNotFoundError(f"policy.onnx not found: {self.onnx_path}")

        available = ort.get_available_providers()
        providers: list[str] = []
        if use_tensorrt and "TensorrtExecutionProvider" not in available:
            raise RuntimeError(
                "TensorRT execution was requested, but TensorrtExecutionProvider "
                f"is unavailable; installed providers are {available}"
            )
        if use_cuda and "CUDAExecutionProvider" not in available:
            raise RuntimeError(
                "CUDA execution was requested, but CUDAExecutionProvider is "
                f"unavailable; installed providers are {available}"
            )
        if use_tensorrt:
            providers.append("TensorrtExecutionProvider")
        if use_cuda:
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")

        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 1
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = ort.InferenceSession(self.onnx_path, sess_options=session_options, providers=providers)
        active_providers = self.session.get_providers()
        if use_tensorrt and "TensorrtExecutionProvider" not in active_providers:
            raise RuntimeError(
                "TensorRT provider was requested but ONNX Runtime did not activate it; "
                f"active providers are {active_providers}"
            )
        if use_cuda and "CUDAExecutionProvider" not in active_providers:
            raise RuntimeError(
                "CUDA provider was requested but ONNX Runtime did not activate it; "
                f"active providers are {active_providers}"
            )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        self.sha256 = hashlib.sha256(Path(self.onnx_path).read_bytes()).hexdigest()
        model_meta = self.session.get_modelmeta()
        self.metadata = dict(getattr(model_meta, "custom_metadata_map", {}) or {})
        self.interface_kind = "legacy_obs"
        sensor_v2 = (
            self.metadata.get("contract_id") == "redrhex.student-observation.v2"
            and [(item.name, list(item.shape), item.type) for item in inputs]
            == [
                ("sensor_history", [1, 60, 36], "tensor(float)"),
                ("command", [1, 3], "tensor(float)"),
            ]
            and [(item.name, list(item.shape), item.type) for item in outputs]
            == [
                ("actions", [1, 12], "tensor(float)"),
                ("base_velocity_estimate", [1, 3], "tensor(float)"),
            ]
        )
        if sensor_v2:
            self.interface_kind = "sensor_v2"
            self.sensor_history_input = inputs[0]
            self.command_input = inputs[1]
            self.input = self.sensor_history_input
            self.output = outputs[0]
        elif len(inputs) != 1 or len(outputs) != 1:
            raise ValueError(
                "ONNX must be legacy one-input/one-output or the exact "
                "redrhex.student-observation.v2 interface; got "
                f"{len(inputs)} input(s) and {len(outputs)} output(s)."
            )
        else:
            self.input = inputs[0]
            self.output = outputs[0]
        if self.input.type != "tensor(float)" or self.output.type != "tensor(float)":
            raise ValueError(
                "ONNX input/output must both be float32 tensors; "
                f"got input={self.input.type}, output={self.output.type}."
            )
        self.input_name = self.input.name
        self.output_name = self.output.name
        self.expected_obs_dim = int(expected_obs_dim)
        self.expected_action_dim = int(expected_action_dim)
        self.allow_history_dim = bool(allow_history_dim)
        metadata_rejections = deployment_metadata_rejection_reasons(self.metadata)
        if metadata_rejections:
            raise ValueError(
                "ONNX metadata explicitly blocks deployment: "
                + "; ".join(metadata_rejections)
            )
        self.expected_observation_contract = expected_observation_contract
        self.expected_action_contract = expected_action_contract
        self.expected_input_layout = expected_input_layout
        self.require_contract_metadata = bool(require_contract_metadata)

        self.obs_dim = None if sensor_v2 else self._last_static_dim(self.input.shape)
        self.action_dim = self._last_static_dim(self.output.shape)

        allowed_obs_dims = {self.expected_obs_dim}
        if self.allow_history_dim:
            allowed_obs_dims.add(self.expected_obs_dim * 5)
        self.allowed_obs_dims = allowed_obs_dims
        if not sensor_v2 and self.obs_dim is not None and self.obs_dim not in allowed_obs_dims:
            raise ValueError(
                f"ONNX input dim {self.obs_dim} is not compatible with expected "
                f"{sorted(allowed_obs_dims)}. Inspect policy export/obs history."
            )
        if self.action_dim is not None and self.action_dim != self.expected_action_dim:
            raise ValueError(
                f"ONNX output dim {self.action_dim} != expected action dim {self.expected_action_dim}."
            )
        if not sensor_v2:
            self._validate_contract_metadata()

    @staticmethod
    def _last_static_dim(shape: list[Any]) -> int | None:
        if not shape:
            return None
        dim = shape[-1]
        if isinstance(dim, int) and dim > 0:
            return dim
        return None

    @property
    def io_info(self) -> ONNXIOInfo:
        return ONNXIOInfo(
            input_name=self.input_name,
            input_shape=list(self.input.shape),
            input_type=self.input.type,
            output_name=self.output_name,
            output_shape=list(self.output.shape),
            output_type=self.output.type,
            obs_dim=self.obs_dim,
            action_dim=self.action_dim,
            providers=list(self.session.get_providers()),
            sha256=self.sha256,
            metadata=dict(self.metadata),
            interface_kind=self.interface_kind,
        )

    @property
    def is_sensor_v2(self) -> bool:
        return self.interface_kind == "sensor_v2"

    def _validate_contract_metadata(self) -> None:
        from . import redrhex_contract as C

        expected = {
            C.ONNX_OBSERVATION_CONTRACT_KEY: self.expected_observation_contract,
            C.ONNX_ACTION_CONTRACT_KEY: self.expected_action_contract,
        }
        input_layout = self.expected_input_layout
        if input_layout == "auto":
            input_layout = (
                C.POLICY_INPUT_LAYOUT_HISTORY
                if self.obs_dim == C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH
                else C.POLICY_INPUT_LAYOUT_SINGLE
            )
        expected[C.ONNX_INPUT_LAYOUT_KEY] = input_layout
        for key, value in expected.items():
            if not value:
                continue
            actual = self.metadata.get(key)
            if actual is None:
                if self.require_contract_metadata:
                    raise ValueError(
                        f"ONNX is missing required metadata {key}={value!r}; tensor shape alone "
                        "cannot prove policy/deployment compatibility."
                    )
                continue
            if actual != value:
                raise ValueError(f"ONNX metadata {key}={actual!r}, expected {value!r}.")

    def run(self, observation: np.ndarray, command: np.ndarray | None = None) -> np.ndarray:
        if self.is_sensor_v2:
            if command is None:
                raise ValueError("sensor-v2 policy requires command [1,3] or [3].")
            history = np.asarray(observation, dtype=np.float32)
            if history.shape == (60, 36):
                history = history.reshape(1, 60, 36)
            cmd = np.asarray(command, dtype=np.float32)
            if cmd.shape == (3,):
                cmd = cmd.reshape(1, 3)
            if history.shape != (1, 60, 36):
                raise ValueError(f"sensor_history must have shape [1,60,36] or [60,36], got {history.shape}.")
            if cmd.shape != (1, 3):
                raise ValueError(f"command must have shape [1,3] or [3], got {cmd.shape}.")
            if not np.isfinite(history).all() or not np.isfinite(cmd).all():
                raise ValueError("sensor-v2 input contains NaN or Inf.")
            outputs = self.session.run(
                [self.output_name],
                {"sensor_history": history, "command": cmd},
            )
            action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
            if action.shape != (self.expected_action_dim,):
                raise ValueError(f"Policy action shape {action.shape} != ({self.expected_action_dim},).")
            if not np.isfinite(action).all():
                raise ValueError("Policy action contains NaN or Inf.")
            return action
        obs = np.asarray(observation, dtype=np.float32)
        if obs.ndim == 1:
            obs = obs.reshape(1, -1)
        if obs.ndim != 2 or obs.shape[0] != 1:
            raise ValueError(f"Observation must have shape [1, N] or [N], got {obs.shape}.")
        if self.obs_dim is not None and obs.shape[1] != self.obs_dim:
            raise ValueError(f"Observation dim {obs.shape[1]} does not match ONNX input dim {self.obs_dim}.")
        if self.obs_dim is None and obs.shape[1] not in self.allowed_obs_dims:
            raise ValueError(
                f"Observation dim {obs.shape[1]} is not one of allowed dims {sorted(self.allowed_obs_dims)}."
            )
        if not np.isfinite(obs).all():
            raise ValueError("Observation contains NaN or Inf.")

        outputs = self.session.run([self.output_name], {self.input_name: obs})
        action = np.asarray(outputs[0], dtype=np.float32)
        if action.ndim == 2 and action.shape[0] == 1:
            action = action.reshape(-1)
        if action.shape != (self.expected_action_dim,):
            raise ValueError(f"Policy action shape {action.shape} != ({self.expected_action_dim},).")
        if not np.isfinite(action).all():
            raise ValueError("Policy action contains NaN or Inf.")
        return action

    def benchmark_sensor_v2(
        self,
        samples: list[tuple[np.ndarray, np.ndarray]],
        *,
        warmup_runs: int = 20,
        runs: int = 200,
    ) -> dict[str, float | int]:
        if not self.is_sensor_v2:
            raise ValueError("benchmark_sensor_v2 requires a sensor-v2 policy")
        if warmup_runs < 0 or runs < 100 or not samples:
            raise ValueError("benchmark requires samples, warmup_runs >= 0 and runs >= 100")
        for index in range(warmup_runs):
            self.run(*samples[index % len(samples)])
        durations_ms = np.empty(runs, dtype=np.float64)
        for index in range(runs):
            start_ns = time.perf_counter_ns()
            self.run(*samples[index % len(samples)])
            durations_ms[index] = (time.perf_counter_ns() - start_ns) * 1.0e-6
        return self._latency_summary(durations_ms, warmup_runs)

    @staticmethod
    def _latency_summary(durations_ms: np.ndarray, warmup_runs: int) -> dict[str, float | int]:
        return {
            "samples": int(durations_ms.size), "warmup_runs": int(warmup_runs),
            "mean_ms": float(np.mean(durations_ms)), "p50_ms": float(np.percentile(durations_ms, 50)),
            "p95_ms": float(np.percentile(durations_ms, 95)), "p99_ms": float(np.percentile(durations_ms, 99)),
            "max_ms": float(np.max(durations_ms)),
        }

    def benchmark(
        self,
        observations: list[np.ndarray] | tuple[np.ndarray, ...],
        *,
        warmup_runs: int = 20,
        runs: int = 200,
    ) -> dict[str, float | int]:
        """Measure repeated wall-clock CPU inference latency, including p99."""

        if warmup_runs < 0 or runs < 100:
            raise ValueError("benchmark requires warmup_runs >= 0 and runs >= 100")
        inputs = [np.asarray(value, dtype=np.float32) for value in observations]
        if not inputs:
            raise ValueError("benchmark requires at least one observation")
        for index in range(warmup_runs):
            self.run(inputs[index % len(inputs)])
        durations_ms = np.empty(runs, dtype=np.float64)
        for index in range(runs):
            start_ns = time.perf_counter_ns()
            self.run(inputs[index % len(inputs)])
            durations_ms[index] = (time.perf_counter_ns() - start_ns) * 1.0e-6
        return {
            "samples": int(runs),
            "warmup_runs": int(warmup_runs),
            "mean_ms": float(np.mean(durations_ms)),
            "p50_ms": float(np.percentile(durations_ms, 50)),
            "p95_ms": float(np.percentile(durations_ms, 95)),
            "p99_ms": float(np.percentile(durations_ms, 99)),
            "max_ms": float(np.max(durations_ms)),
        }
