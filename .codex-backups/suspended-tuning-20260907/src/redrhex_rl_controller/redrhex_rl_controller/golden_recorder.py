"""IsaacLab/play.py helper for recording real simulator golden trajectories.

This module intentionally does not synthesize observations or decoder targets.
The simulator caller must pass every value captured from its live step.
"""

from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path

import numpy as np

from . import redrhex_contract as C
from .golden_policy import (
    GOLDEN_PRODUCER,
    PROVENANCE_FIELDS,
    VECTOR_FIELDS,
    load_golden_vectors,
    sha256_file,
)
from .degraded_mode import normalize_disabled_legs


class IsaacLabGoldenRecorder:
    """Accumulate live IsaacLab play samples and create a validated NPZ."""

    def __init__(
        self,
        *,
        output_npz: str | Path,
        source_onnx: str | Path,
        joint_names: list[str],
        training_git_sha: str,
        training_env_source: str | Path,
        training_env_config_source: str | Path,
        training_play_source: str | Path | None = None,
        normalizer_embedded: bool,
        policy_input_dim: int,
        disabled_legs: list[str],
        command_profile: str,
        fixed_forward_vx: float,
    ) -> None:
        self.output_npz = Path(output_npz).expanduser().resolve()
        self.source_onnx = Path(source_onnx).expanduser().resolve()
        self.joint_names = np.asarray(joint_names, dtype=np.str_)
        self.training_git_sha = str(training_git_sha).strip().lower()
        if re.fullmatch(r"[0-9a-f]{7,64}", self.training_git_sha) is None:
            raise ValueError("training_git_sha must be 7-64 hexadecimal characters")
        if not normalizer_embedded:
            raise ValueError("hardware export requires the observation normalizer embedded")
        if policy_input_dim not in (
            C.OBS_DIM_SINGLE,
            C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH,
        ):
            raise ValueError("policy_input_dim must be 56 or 280")
        self.policy_input_dim = int(policy_input_dim)
        self.disabled_legs = normalize_disabled_legs(disabled_legs, 5)
        self.command_profile = str(command_profile)
        if self.command_profile not in ("fixed_forward", "external_cmd_vel"):
            raise ValueError("command_profile is invalid")
        self.fixed_forward_vx = float(fixed_forward_vx)
        if not np.isfinite(self.fixed_forward_vx):
            raise ValueError("fixed_forward_vx must be finite")
        self.training_env_source = Path(training_env_source).expanduser().resolve()
        self.training_env_config_source = Path(
            training_env_config_source
        ).expanduser().resolve()
        self.training_play_source = (
            Path(training_play_source).expanduser().resolve()
            if training_play_source is not None
            else None
        )
        for path in (
            self.source_onnx,
            self.training_env_source,
            self.training_env_config_source,
        ):
            if not path.is_file():
                raise FileNotFoundError(path)
        if self.training_play_source is not None and not self.training_play_source.is_file():
            raise FileNotFoundError(self.training_play_source)
        self._rows: dict[str, list[np.ndarray]] = {
            field: [] for field in VECTOR_FIELDS if field != "joint_names"
        }

    def record_step(self, **sample) -> None:
        """Record one live simulator step; no field is reconstructed here."""

        missing = sorted(set(self._rows) - set(sample))
        extra = sorted(set(sample) - set(self._rows))
        if missing or extra:
            raise ValueError(f"golden sample fields missing={missing}, extra={extra}")
        for field in self._rows:
            value = np.asarray(sample[field])
            if value.dtype.kind in "fc" and not np.isfinite(value).all():
                raise ValueError(f"sample {field} contains NaN/Inf")
            self._rows[field].append(value.copy())

    def finalize(self) -> Path:
        if self.output_npz.exists():
            raise FileExistsError(
                f"golden output already exists; choose an immutable new path: {self.output_npz}"
            )
        if not self._rows["policy_input"]:
            raise ValueError("no live simulator steps were recorded")
        archive = {
            field: np.stack(rows) for field, rows in self._rows.items()
        }
        archive["joint_names"] = self.joint_names
        archive.update(
            {
                "producer": np.asarray(GOLDEN_PRODUCER),
                "training_git_sha": np.asarray(self.training_git_sha),
                "training_env_source_sha256": np.asarray(
                    sha256_file(self.training_env_source)
                ),
                "training_env_config_source_sha256": np.asarray(
                    sha256_file(self.training_env_config_source)
                ),
                "training_play_source_sha256": np.asarray(
                    sha256_file(self.training_play_source)
                    if self.training_play_source is not None
                    else ""
                ),
                "deployment_disabled_legs_csv": np.asarray(
                    ",".join(self.disabled_legs)
                ),
                "deployment_command_profile": np.asarray(self.command_profile),
                "deployment_fixed_forward_vx": np.asarray(self.fixed_forward_vx),
            }
        )
        if set(PROVENANCE_FIELDS) - set(archive):
            raise RuntimeError("internal golden provenance error")
        self.output_npz.parent.mkdir(parents=True, exist_ok=True)
        temporary_fd, temporary_name = tempfile.mkstemp(
            dir=self.output_npz.parent,
            prefix=f".{self.output_npz.name}.",
            suffix=".tmp.npz",
        )
        os.close(temporary_fd)
        temporary_npz = Path(temporary_name)
        try:
            np.savez(temporary_npz, **archive)
            load_golden_vectors(temporary_npz)
            self._tag_training_onnx()
            # Keep an output path immutable from the caller's perspective:
            # the complete, validated archive appears in one filesystem step.
            if self.output_npz.exists():
                raise FileExistsError(
                    "golden output appeared while recording; choose a new path: "
                    f"{self.output_npz}"
                )
            os.replace(temporary_npz, self.output_npz)
        except Exception:
            temporary_npz.unlink(missing_ok=True)
            raise
        return self.output_npz

    def _tag_training_onnx(self) -> None:
        """Attach only training-origin metadata to the exporter-created ONNX."""

        import onnx

        model = onnx.load(str(self.source_onnx))
        metadata = {entry.key: entry.value for entry in model.metadata_props}
        input_layout = (
            C.POLICY_INPUT_LAYOUT_HISTORY
            if self.policy_input_dim == C.OBS_DIM_SINGLE * C.POLICY_HISTORY_LENGTH
            else C.POLICY_INPUT_LAYOUT_SINGLE
        )
        training_metadata = {
            C.ONNX_OBSERVATION_CONTRACT_KEY: C.OBSERVATION_CONTRACT_ID,
            C.ONNX_ACTION_CONTRACT_KEY: C.ACTION_DECODER_CONTRACT_ID,
            C.ONNX_INPUT_LAYOUT_KEY: input_layout,
            C.ONNX_NORMALIZER_KEY: C.NORMALIZER_EMBEDDED,
            C.ONNX_TRAINING_ACTION_CLIP_KEY: str(C.TRAINING_ACTION_CLIP),
            C.ONNX_TRAINING_GIT_SHA_KEY: self.training_git_sha,
            C.ONNX_TRAINING_ENV_SOURCE_SHA256_KEY: sha256_file(
                self.training_env_source
            ),
            C.ONNX_TRAINING_ENV_CONFIG_SOURCE_SHA256_KEY: sha256_file(
                self.training_env_config_source
            ),
        }
        if self.training_play_source is not None:
            training_metadata[C.ONNX_TRAINING_PLAY_SOURCE_SHA256_KEY] = sha256_file(
                self.training_play_source
            )
        for key, value in training_metadata.items():
            existing = metadata.get(key)
            if existing is not None and existing != value:
                raise ValueError(
                    f"source ONNX metadata {key}={existing!r}, expected {value!r}"
                )
            metadata[key] = value
        del model.metadata_props[:]
        for key, value in sorted(metadata.items()):
            entry = model.metadata_props.add()
            entry.key = key
            entry.value = value
        temporary_fd, temporary_name = tempfile.mkstemp(
            dir=self.source_onnx.parent,
            prefix=f".{self.source_onnx.name}.",
            suffix=".tmp.onnx",
        )
        os.close(temporary_fd)
        temporary_onnx = Path(temporary_name)
        try:
            onnx.save(model, str(temporary_onnx))
            # Preserve the exporter's access mode instead of publishing the
            # restrictive mode selected by mkstemp.
            temporary_onnx.chmod(self.source_onnx.stat().st_mode & 0o777)
            os.replace(temporary_onnx, self.source_onnx)
        except Exception:
            temporary_onnx.unlink(missing_ok=True)
            raise
