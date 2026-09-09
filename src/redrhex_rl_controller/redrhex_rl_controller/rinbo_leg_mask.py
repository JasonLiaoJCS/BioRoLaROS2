"""Safely prepare and inspect the one-leg degraded-mode configuration.

The mask is deliberately a startup-only hardware contract.  This tool edits
the selected site-local YAML files while all mask consumers are stopped; it
never tries to hot-change a running ROS parameter or power the robot.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
from typing import Any, Callable, Iterator

import yaml

from . import redrhex_contract as C
from .degraded_mode import normalize_disabled_legs
from .golden_policy import (
    bridge_config_sha256,
    controller_config_sha256,
    decoder_config_from_ros_params,
    decoder_config_sha256,
)
from .policy_validation import deployment_metadata_rejection_reasons


LEG_NAMES = ("L1", "L2", "L3", "R1", "R2", "R3")
LEG_INFO = {
    "L1": {
        "position": "左前",
        "rinbo_index": 0,
        "power_channel": 1,
        "policy_index": 3,
        "main": "Revolute_18",
        "servo": "SL1",
    },
    "L2": {
        "position": "左中",
        "rinbo_index": 1,
        "power_channel": 2,
        "policy_index": 4,
        "main": "Revolute_23",
        "servo": "SL2",
    },
    "L3": {
        "position": "左后",
        "rinbo_index": 2,
        "power_channel": 3,
        "policy_index": 5,
        "main": "Revolute_24",
        "servo": "SL3",
    },
    "R1": {
        "position": "右前",
        "rinbo_index": 3,
        "power_channel": 4,
        "policy_index": 0,
        "main": "Revolute_15",
        "servo": "SR1",
    },
    "R2": {
        "position": "右中",
        "rinbo_index": 4,
        "power_channel": 5,
        "policy_index": 1,
        "main": "Revolute_7",
        "servo": "SR2",
    },
    "R3": {
        "position": "右后",
        "rinbo_index": 5,
        "power_channel": 6,
        "policy_index": 2,
        "main": "Revolute_12",
        "servo": "SR3",
    },
}
FSM_NODES = ("rinbo_cali", "rinbo_standing", "rinbo_tripod_rslip")
MASK_CONSUMER_NODE_BASENAMES = frozenset(
    FSM_NODES + ("redrhex_rl_controller", "redrhex_lowlevel_bridge")
)
MANAGED_MARKER = "# rinbo_leg_mask managed active mask:"
DEFAULT_CONFIG_FILENAMES = {
    "fsm": "rinbo_fsm_disabled_leg.yaml",
    "controller": "redrhex_policy_full_feedback_rig.yaml",
    "bridge": "lowlevel_bridge_full_feedback_rig.yaml",
}


class LegMaskError(RuntimeError):
    """User-facing configuration or safety error."""


def _mask_text(mask: list[str]) -> str:
    return mask[0] if mask else "none"


def _default_site_dir() -> Path:
    configured = os.environ.get("REDRHEX_SITE_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path.home() / "redrhex_site"


def _default_config_path(environment_name: str, filename: str) -> str:
    configured = os.environ.get(environment_name, "").strip()
    if configured:
        return configured
    return str(_default_site_dir() / filename)


def _canonical_target(value: str | None) -> list[str]:
    text = str(value or "").strip().upper()
    if text in ("", "NONE", "CLEAR", "[]"):
        return []
    return normalize_disabled_legs([text], 1)


def _path_value(document: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = document
    for key in path:
        if not isinstance(current, dict) or key not in current:
            raise LegMaskError(f"YAML is missing {'.'.join(path)}")
        current = current[key]
    return current


def _load_yaml_text(text: str, label: str) -> dict[str, Any]:
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise LegMaskError(f"{label} is not valid YAML: {exc}") from exc
    if not isinstance(document, dict):
        raise LegMaskError(f"{label} must contain a YAML mapping")
    return document


def _read_yaml(path: Path, label: str) -> tuple[str, dict[str, Any]]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise LegMaskError(f"cannot read {label} {path}: {exc}") from exc
    return text, _load_yaml_text(text, label)


_KEY_LINE = re.compile(
    r"^(?P<indent> *)(?P<key>[A-Za-z0-9_.-]+):(?P<tail>[^\r\n]*)(?P<newline>\r?\n?)$"
)


def _replace_yaml_scalar(
    text: str, path: tuple[str, ...], rendered_value: str
) -> str:
    """Replace one existing simple YAML value without discarding comments."""

    stack: list[tuple[int, str]] = []
    output: list[str] = []
    matches = 0
    for line in text.splitlines(keepends=True):
        match = _KEY_LINE.match(line)
        if match is None:
            output.append(line)
            continue
        indent = len(match.group("indent"))
        while stack and stack[-1][0] >= indent:
            stack.pop()
        key = match.group("key")
        current_path = tuple(item[1] for item in stack) + (key,)
        if current_path == path:
            tail = match.group("tail")
            inline_comment = ""
            comment_match = re.search(r"\s+#", tail)
            if comment_match is not None:
                inline_comment = tail[comment_match.start():]
            output.append(
                f"{match.group('indent')}{key}: {rendered_value}"
                f"{inline_comment}{match.group('newline')}"
            )
            matches += 1
        else:
            output.append(line)
        stack.append((indent, key))
    if matches != 1:
        raise LegMaskError(
            f"expected exactly one YAML key {'.'.join(path)}, found {matches}"
        )
    return "".join(output)


def _with_marker(text: str, mask: list[str]) -> str:
    marker = (
        f"{MANAGED_MARKER} {_mask_text(mask)}; startup-only, restart required"
    )
    lines = text.splitlines(keepends=True)
    for index, line in enumerate(lines[:8]):
        if line.startswith(MANAGED_MARKER):
            newline = "\n" if line.endswith("\n") else ""
            lines[index] = marker + newline
            return "".join(lines)
    return marker + "\n" + text


def _generalize_mask_comments(text: str) -> str:
    """Remove stale L1-only wording from older site-local templates."""

    replacements = {
        "# Current robot: L1 is physically isolated and must never receive output.": (
            "# The selected disabled leg must be physically isolated and must never "
            "receive output."
        ),
        "# L1 readback may be absent/stale because it is physically isolated.": (
            "# Disabled-leg readback may be absent/stale because it is physically "
            "isolated."
        ),
        "# have repeatable zero/sign/counts-per-radian measurements. L1 stays isolated.": (
            "# have repeatable zero/sign/counts-per-radian measurements. The selected "
            "leg stays isolated."
        ),
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def _render_mask(mask: list[str]) -> str:
    return "[]" if not mask else f'["{mask[0]}"]'


def _normalized_mask(value: Any, label: str) -> list[str]:
    if value is None:
        values: list[Any] = []
    elif isinstance(value, list):
        values = value
    else:
        raise LegMaskError(f"{label} must be a YAML list")
    try:
        return normalize_disabled_legs(values, 1)
    except (TypeError, ValueError) as exc:
        raise LegMaskError(f"invalid {label}: {exc}") from exc


def _fsm_masks(document: dict[str, Any]) -> dict[str, list[str]]:
    masks: dict[str, list[str]] = {}
    for node in FSM_NODES:
        maximum = int(
            _path_value(
                document,
                (node, "ros__parameters", "hardware", "max_disabled_legs"),
            )
        )
        if maximum != 1:
            raise LegMaskError(
                f"{node}.hardware.max_disabled_legs must be exactly 1"
            )
        masks[node] = _normalized_mask(
            _path_value(
                document,
                (node, "ros__parameters", "hardware", "disabled_legs"),
            ),
            f"{node}.hardware.disabled_legs",
        )
    return masks


def _named_params(
    document: dict[str, Any], node_name: str, label: str
) -> dict[str, Any]:
    params = _path_value(document, (node_name, "ros__parameters"))
    if not isinstance(params, dict):
        raise LegMaskError(f"{label} ros__parameters must be a mapping")
    maximum = int(_path_value(params, ("hardware", "max_disabled_legs")))
    if maximum != 1:
        raise LegMaskError(f"{label} hardware.max_disabled_legs must be exactly 1")
    return params


def _params_mask(params: dict[str, Any], label: str) -> list[str]:
    return _normalized_mask(
        _path_value(params, ("hardware", "disabled_legs")),
        f"{label} hardware.disabled_legs",
    )


def _update_fsm_text(text: str, mask: list[str]) -> str:
    document = _load_yaml_text(text, "FSM config")
    _fsm_masks(document)
    updated = text
    for node in FSM_NODES:
        updated = _replace_yaml_scalar(
            updated,
            (node, "ros__parameters", "hardware", "disabled_legs"),
            _render_mask(mask),
        )
    updated = _with_marker(updated, mask)
    if any(value != mask for value in _fsm_masks(_load_yaml_text(updated, "FSM config")).values()):
        raise LegMaskError("internal error: updated FSM masks do not match")
    return updated


def _update_policy_texts(
    controller_text: str,
    bridge_text: str,
    mask: list[str],
    *,
    invalidate_policy: bool,
) -> tuple[str, str, str]:
    controller_text = _generalize_mask_comments(controller_text)
    bridge_text = _generalize_mask_comments(bridge_text)
    controller_document = _load_yaml_text(controller_text, "controller config")
    bridge_document = _load_yaml_text(bridge_text, "bridge config")
    controller_params = _named_params(
        controller_document, "redrhex_rl_controller", "controller"
    )
    bridge_params = _named_params(
        bridge_document, "redrhex_lowlevel_bridge", "bridge"
    )
    _params_mask(controller_params, "controller")
    _params_mask(bridge_params, "bridge")

    updated_bridge = _replace_yaml_scalar(
        bridge_text,
        (
            "redrhex_lowlevel_bridge",
            "ros__parameters",
            "hardware",
            "disabled_legs",
        ),
        _render_mask(mask),
    )
    if invalidate_policy:
        for parameter_name in (
            "main_drive_calibrated",
            "abad_feedback_calibrated",
            "abad_command_calibrated",
        ):
            updated_bridge = _replace_yaml_scalar(
                updated_bridge,
                (
                    "redrhex_lowlevel_bridge",
                    "ros__parameters",
                    "rinbo",
                    parameter_name,
                ),
                "false",
            )
    updated_bridge = _with_marker(updated_bridge, mask)
    updated_bridge_document = _load_yaml_text(updated_bridge, "bridge config")
    updated_bridge_params = _named_params(
        updated_bridge_document, "redrhex_lowlevel_bridge", "bridge"
    )
    semantic_bridge_hash = bridge_config_sha256(updated_bridge_params)

    updated_controller = _replace_yaml_scalar(
        controller_text,
        (
            "redrhex_rl_controller",
            "ros__parameters",
            "hardware",
            "disabled_legs",
        ),
        _render_mask(mask),
    )
    if mask:
        updated_controller = _replace_yaml_scalar(
            updated_controller,
            (
                "redrhex_rl_controller",
                "ros__parameters",
                "observation",
                "disabled_leg_observation_mode",
            ),
            '"nominal"',
        )
    updated_controller = _replace_yaml_scalar(
        updated_controller,
        (
            "redrhex_rl_controller",
            "ros__parameters",
            "policy",
            "expected_bridge_config_sha256",
        ),
        json.dumps(semantic_bridge_hash),
    )
    if invalidate_policy:
        updated_controller = _replace_yaml_scalar(
            updated_controller,
            (
                "redrhex_rl_controller",
                "ros__parameters",
                "policy",
                "expected_sha256",
            ),
            '""',
        )
        updated_controller = _replace_yaml_scalar(
            updated_controller,
            (
                "redrhex_rl_controller",
                "ros__parameters",
                "action",
                "hardware_mapping_calibrated",
            ),
            "false",
        )
    updated_controller = _with_marker(updated_controller, mask)

    updated_controller_document = _load_yaml_text(
        updated_controller, "controller config"
    )
    updated_controller_params = _named_params(
        updated_controller_document, "redrhex_rl_controller", "controller"
    )
    if _params_mask(updated_controller_params, "controller") != mask:
        raise LegMaskError("internal error: updated controller mask does not match")
    if _params_mask(updated_bridge_params, "bridge") != mask:
        raise LegMaskError("internal error: updated bridge mask does not match")
    pinned_hash = str(
        _path_value(
            updated_controller_params,
            ("policy", "expected_bridge_config_sha256"),
        )
    )
    if pinned_hash != semantic_bridge_hash:
        raise LegMaskError("internal error: bridge hash pin was not updated")
    return updated_controller, updated_bridge, semantic_bridge_hash


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _inspect_policy(
    controller_params: dict[str, Any],
    bridge_params: dict[str, Any] | None,
    onnx_override: str,
) -> dict[str, Any]:
    policy_params = _path_value(controller_params, ("policy",))
    if not isinstance(policy_params, dict):
        raise LegMaskError("controller policy section must be a mapping")
    configured_path = str(policy_params.get("onnx_path", "")).strip()
    selected_path = onnx_override.strip() or configured_path
    expected_file_sha = str(policy_params.get("expected_sha256", "")).strip().lower()
    result: dict[str, Any] = {
        "path": selected_path,
        "expected_sha256": expected_file_sha,
        "exists": False,
        "sha256_ok": False,
        "metadata_ok": False,
        "mask_binding_ok": False,
        "issues": [],
    }
    if not selected_path:
        result["issues"].append("policy.onnx_path is empty")
        return result
    path = Path(selected_path).expanduser()
    result["path"] = str(path)
    if not path.is_file():
        result["issues"].append("ONNX file does not exist")
        return result
    result["exists"] = True
    actual_sha = _sha256_file(path)
    result["actual_sha256"] = actual_sha
    result["sha256_ok"] = bool(expected_file_sha) and actual_sha == expected_file_sha
    if not expected_file_sha:
        result["issues"].append("policy.expected_sha256 is empty")
    elif actual_sha != expected_file_sha:
        result["issues"].append("ONNX file SHA256 does not match controller YAML")

    try:
        import onnx

        model = onnx.load(str(path), load_external_data=False)
        metadata = {entry.key: entry.value for entry in model.metadata_props}
    except Exception as exc:  # pragma: no cover - depends on artifact/dependency
        result["issues"].append(f"cannot read ONNX metadata: {exc}")
        return result

    rejections = deployment_metadata_rejection_reasons(metadata)
    if rejections:
        result["issues"].extend(rejections)
    if metadata.get("bundle_schema") == "redrhex.sensor-policy-bundle.v2":
        rejections.append(
            "experimental sensor-v2 is not deployable by the current hardware packager/runtime"
        )
        result["issues"].append(rejections[-1])
    expected_metadata = {
        C.ONNX_OBSERVATION_CONTRACT_KEY: str(
            policy_params.get("observation_contract", "")
        ),
        C.ONNX_ACTION_CONTRACT_KEY: str(policy_params.get("action_contract", "")),
        C.ONNX_CONTROLLER_CONFIG_SHA256_KEY: controller_config_sha256(
            controller_params
        ),
        C.ONNX_DECODER_CONFIG_SHA256_KEY: decoder_config_sha256(
            decoder_config_from_ros_params(controller_params)
        ),
    }
    if bridge_params is not None:
        expected_metadata[C.ONNX_BRIDGE_CONFIG_SHA256_KEY] = bridge_config_sha256(
            bridge_params
        )
    mismatches: dict[str, dict[str, str | None]] = {}
    for key, expected in expected_metadata.items():
        actual = metadata.get(key)
        if actual != expected:
            mismatches[key] = {"actual": actual, "expected": expected}
    result["metadata_mismatches"] = mismatches
    if mismatches:
        result["issues"].append(
            "ONNX metadata is bound to a different mask/configuration"
        )
    result["metadata_ok"] = not rejections and not mismatches
    result["mask_binding_ok"] = result["sha256_ok"] and result["metadata_ok"]
    return result


def inspect_configs(
    *,
    fsm_path: Path | None,
    controller_path: Path | None,
    bridge_path: Path | None,
    onnx_override: str = "",
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "mask_ok": True,
        "effective_mask": None,
        "configs": {},
        "issues": [],
        "warnings": [],
    }
    observed_masks: list[tuple[str, list[str]]] = []
    controller_params: dict[str, Any] | None = None
    bridge_params: dict[str, Any] | None = None

    if fsm_path is not None:
        _, document = _read_yaml(fsm_path, "FSM config")
        masks = _fsm_masks(document)
        report["configs"]["fsm"] = {
            "path": str(fsm_path),
            "masks": {name: _mask_text(mask) for name, mask in masks.items()},
        }
        observed_masks.extend((f"FSM/{name}", mask) for name, mask in masks.items())
    if controller_path is not None:
        _, document = _read_yaml(controller_path, "controller config")
        controller_params = _named_params(
            document, "redrhex_rl_controller", "controller"
        )
        mask = _params_mask(controller_params, "controller")
        report["configs"]["controller"] = {
            "path": str(controller_path),
            "mask": _mask_text(mask),
            "disabled_leg_observation_mode": str(
                _path_value(
                    controller_params,
                    ("observation", "disabled_leg_observation_mode"),
                )
            ),
        }
        observed_masks.append(("controller", mask))
    if bridge_path is not None:
        _, document = _read_yaml(bridge_path, "bridge config")
        bridge_params = _named_params(
            document, "redrhex_lowlevel_bridge", "bridge"
        )
        mask = _params_mask(bridge_params, "bridge")
        report["configs"]["bridge"] = {
            "path": str(bridge_path),
            "mask": _mask_text(mask),
        }
        observed_masks.append(("bridge", mask))

    if not observed_masks:
        raise LegMaskError(
            "no configuration paths supplied; set REDRHEX_FSM_CFG, "
            "REDRHEX_SITE_CTRL, and REDRHEX_SITE_BRIDGE"
        )
    first_mask = observed_masks[0][1]
    mismatched = [name for name, mask in observed_masks if mask != first_mask]
    if mismatched:
        report["mask_ok"] = False
        report["issues"].append(
            "disabled-leg mismatch: "
            + ", ".join(f"{name}={_mask_text(mask)}" for name, mask in observed_masks)
        )
    else:
        report["effective_mask"] = _mask_text(first_mask)
        if first_mask:
            report["disabled_leg_mapping"] = dict(LEG_INFO[first_mask[0]])

    if controller_params is not None and bridge_params is not None:
        actual_bridge_hash = bridge_config_sha256(bridge_params)
        pinned_bridge_hash = str(
            _path_value(
                controller_params, ("policy", "expected_bridge_config_sha256")
            )
        )
        hash_ok = actual_bridge_hash == pinned_bridge_hash
        report["bridge_hash"] = {
            "actual": actual_bridge_hash,
            "controller_pin": pinned_bridge_hash,
            "ok": hash_ok,
        }
        if not hash_ok:
            report["mask_ok"] = False
            report["issues"].append(
                "controller policy.expected_bridge_config_sha256 does not match bridge YAML"
            )
    elif controller_params is not None or bridge_params is not None:
        report["warnings"].append(
            "controller/bridge pair is incomplete; policy contract was not checked"
        )

    if controller_params is not None:
        report["policy"] = _inspect_policy(
            controller_params, bridge_params, onnx_override
        )
    return report


def _run_command(command: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise LegMaskError(f"failed to run {' '.join(command)}: {exc}") from exc


def _running_mask_consumers() -> list[str]:
    completed = _run_command(["ros2", "node", "list"], 4.0)
    if completed.returncode != 0:
        raise LegMaskError(
            "cannot inspect the ROS graph; stop all writers, then use --offline "
            "only if the graph is intentionally unavailable"
        )
    running: list[str] = []
    for raw_line in completed.stdout.splitlines():
        name = raw_line.strip()
        if not name.startswith("/"):
            continue
        if name.rsplit("/", 1)[-1] in MASK_CONSUMER_NODE_BASENAMES:
            running.append(name)
    return sorted(set(running))


def _visible_command_publishers() -> list[str]:
    listed = _run_command(["ros2", "topic", "list"], 3.0)
    if listed.returncode != 0:
        raise LegMaskError(
            "cannot inspect ROS topics; use --offline only after manually "
            "confirming every command writer is stopped and relay power is off"
        )
    visible_topics = {line.strip() for line in listed.stdout.splitlines()}
    active: list[str] = []
    for topic in ("/motor/command", "/redrhex/motor_commands"):
        if topic not in visible_topics:
            continue
        completed = _run_command(["ros2", "topic", "info", topic, "-v"], 3.0)
        if completed.returncode != 0:
            raise LegMaskError(f"cannot inspect publishers on {topic}")
        match = re.search(r"Publisher count:\s*(\d+)", completed.stdout)
        if match is None:
            raise LegMaskError(f"publisher count is unavailable for {topic}")
        if int(match.group(1)) > 0:
            active.append(f"{topic} ({match.group(1)} publisher(s))")
    return active


def _live_power_is_on() -> bool | None:
    try:
        completed = _run_command(
            ["ros2", "topic", "echo", "/power/state", "--once", "--field", "power"],
            2.5,
        )
    except LegMaskError:
        return None
    if completed.returncode != 0:
        return None
    values = re.findall(r"\b(true|false)\b", completed.stdout.lower())
    if not values:
        return None
    return values[-1] == "true"


def _live_masks() -> dict[str, Any]:
    completed = _run_command(["ros2", "node", "list"], 4.0)
    if completed.returncode != 0:
        return {"available": False, "error": completed.stderr.strip()}
    nodes = sorted(
        {
            line.strip()
            for line in completed.stdout.splitlines()
            if line.strip().startswith("/")
            and line.strip().rsplit("/", 1)[-1]
            in MASK_CONSUMER_NODE_BASENAMES
        }
    )
    masks: dict[str, str] = {}
    errors: dict[str, str] = {}
    for node in nodes:
        value = _run_command(
            ["ros2", "param", "get", node, "hardware.disabled_legs"], 3.0
        )
        if value.returncode != 0:
            errors[node] = (value.stderr or value.stdout).strip()
            continue
        names = re.findall(r"\b[LR][123]\b", value.stdout.upper())
        masks[node] = ",".join(names) if names else "none"
    power = _live_power_is_on()
    return {
        "available": True,
        "masks": masks,
        "errors": errors,
        "power": "unknown" if power is None else ("on" if power else "off"),
    }


def _state_base() -> Path:
    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    return (
        Path(state_root).expanduser()
        if state_root
        else Path.home() / ".local" / "state"
    ) / "rinbo_leg_mask"


def _backup_root() -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    return _state_base() / "backups" / timestamp


@contextmanager
def _configuration_lock() -> Iterator[None]:
    """Serialize mask writers from safety inspection through post-write verify."""

    state_dir = _state_base()
    lock_path = state_dir / "update.lock"
    lock_file = None
    try:
        state_dir.mkdir(parents=True, exist_ok=True)
        lock_file = lock_path.open("a+", encoding="utf-8")
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        if lock_file is not None:
            lock_file.close()
        raise LegMaskError(f"cannot lock disabled-leg configuration: {exc}") from exc
    try:
        yield
    finally:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            lock_file.close()


def _fsm_template_path() -> Path:
    candidates: list[Path] = []
    configured = os.environ.get("REDRHEX_FSM_TEMPLATE", "").strip()
    if configured:
        configured_path = Path(configured).expanduser()
        if not configured_path.is_file():
            raise LegMaskError(
                f"REDRHEX_FSM_TEMPLATE does not name a file: {configured_path}"
            )
        return configured_path.resolve()
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("rinbo_fsm"))
            / "config"
            / "disabled_leg_template.yaml"
        )
    except Exception:
        pass
    candidates.append(
        Path(__file__).resolve().parents[2]
        / "rinbo_fsm"
        / "config"
        / "disabled_leg_template.yaml"
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise LegMaskError(
        "cannot find rinbo_fsm/config/disabled_leg_template.yaml; rebuild "
        "rinbo_fsm or set REDRHEX_FSM_TEMPLATE"
    )


def _package_config_directories() -> set[Path]:
    source_root = Path(__file__).resolve().parents[2]
    directories = {
        source_root / "rinbo_fsm" / "config",
        source_root / "redrhex_rl_controller" / "config",
        source_root / "redrhex_lowlevel_bridge" / "config",
    }
    try:
        from ament_index_python.packages import get_package_share_directory

        for package_name in (
            "rinbo_fsm",
            "redrhex_rl_controller",
            "redrhex_lowlevel_bridge",
        ):
            directories.add(
                Path(get_package_share_directory(package_name)) / "config"
            )
    except Exception:
        pass
    return {directory.resolve() for directory in directories if directory.exists()}


def _reject_package_templates(paths: tuple[Path | None, ...]) -> None:
    template_dirs = _package_config_directories()
    configured_fsm_template = os.environ.get("REDRHEX_FSM_TEMPLATE", "").strip()
    configured_template_path = (
        Path(configured_fsm_template).expanduser().resolve()
        if configured_fsm_template
        else None
    )
    for path in paths:
        if path is None:
            continue
        if configured_template_path is not None and path == configured_template_path:
            raise LegMaskError(
                f"refusing to modify FSM template {path}; copy it to a site-local file"
            )
        if any(path == directory or path.is_relative_to(directory) for directory in template_dirs):
            raise LegMaskError(
                f"refusing to modify package template {path}; use a site-local file "
                f"under {_default_site_dir()}"
            )


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = path.stat().st_mode & 0o777 if path.is_file() else 0o644
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_name, mode)
        os.replace(temporary_name, path)
        try:
            directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except (AttributeError, OSError):
            # Some non-POSIX filesystems do not support directory fsync.
            pass
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def _write_transaction(
    updates: dict[Path, str],
    *,
    expected_originals: dict[Path, str | None],
    validate: Callable[[], None],
) -> Path:
    def current_text(path: Path) -> str | None:
        if not os.path.lexists(path):
            return None
        if not path.is_file():
            raise LegMaskError(f"configuration target is not a file: {path}")
        try:
            return path.read_text(encoding="utf-8")
        except OSError as exc:
            raise LegMaskError(f"cannot read configuration {path}: {exc}") from exc

    originals = {path: expected_originals[path] for path in updates}
    for path, expected in originals.items():
        if current_text(path) != expected:
            raise LegMaskError(
                f"configuration changed while preparing the update: {path}; "
                "nothing was overwritten"
            )
    backup_root = _backup_root()
    try:
        backup_root.mkdir(parents=True, exist_ok=False)
        for index, path in enumerate(updates):
            original = originals[path]
            if original is not None:
                _atomic_write(backup_root / f"{index}_{path.name}", original)
    except OSError as exc:
        raise LegMaskError(f"cannot create configuration backup: {exc}") from exc
    written: list[Path] = []
    try:
        for path, text in updates.items():
            if current_text(path) != originals[path]:
                raise LegMaskError(
                    f"configuration changed before commit: {path}; aborting update"
                )
            _atomic_write(path, text)
            written.append(path)
        validate()
    except BaseException as original_error:
        rollback_errors: list[str] = []
        for path in reversed(written):
            try:
                original = originals[path]
                current = current_text(path)
                if original is None:
                    if current == updates[path]:
                        path.unlink(missing_ok=True)
                    elif current is not None:
                        raise LegMaskError(
                            "new file changed externally; refusing to remove it"
                        )
                else:
                    if current == updates[path]:
                        _atomic_write(path, original)
                    elif current != original:
                        raise LegMaskError(
                            "updated file changed externally; refusing to overwrite it"
                        )
            except BaseException as rollback_error:  # pragma: no cover - catastrophic I/O
                rollback_errors.append(f"{path}: {rollback_error}")
        if rollback_errors:
            raise LegMaskError(
                f"configuration update failed ({original_error}); rollback also "
                f"failed: {'; '.join(rollback_errors)}; backups: {backup_root}"
            ) from original_error
        raise
    return backup_root


def _config_paths(
    args: argparse.Namespace, *, require_all: bool, allow_missing_fsm: bool = False
) -> tuple[Path | None, Path | None, Path | None]:
    values = (
        str(args.fsm_config or "").strip(),
        str(args.controller_config or "").strip(),
        str(args.bridge_config or "").strip(),
    )
    fsm_only = bool(getattr(args, "allow_partial", False))
    if fsm_only:
        values = (values[0], "", "")
    if require_all and not getattr(args, "allow_partial", False) and not all(values):
        raise LegMaskError(
            "mask update requires all three site-local files. Export "
            "REDRHEX_FSM_CFG, REDRHEX_SITE_CTRL, REDRHEX_SITE_BRIDGE, or use "
            "--allow-partial intentionally."
        )
    paths = tuple(
        Path(value).expanduser().resolve() if value else None for value in values
    )
    selected_paths = [path for path in paths if path is not None]
    if len(set(selected_paths)) != len(selected_paths):
        raise LegMaskError("FSM, controller, and bridge must use distinct files")
    for index, path in enumerate(paths):
        if path is None:
            continue
        if path.is_file():
            continue
        if index == 0 and allow_missing_fsm and not path.exists():
            continue
        if not path.is_file():
            raise LegMaskError(f"configuration file does not exist: {path}")
    return paths  # type: ignore[return-value]


def _prepare_update(args: argparse.Namespace, mask: list[str]) -> int:
    with _configuration_lock():
        return _prepare_update_locked(args, mask)


def _prepare_update_locked(args: argparse.Namespace, mask: list[str]) -> int:
    if not args.confirm_power_off:
        raise LegMaskError(
            "refusing to change the startup mask without --confirm-power-off"
        )
    initialize_fsm = bool(getattr(args, "initialize_missing_fsm", False))
    fsm_path, controller_path, bridge_path = _config_paths(
        args, require_all=True, allow_missing_fsm=initialize_fsm
    )
    _reject_package_templates((fsm_path, controller_path, bridge_path))
    if (controller_path is None) != (bridge_path is None):
        raise LegMaskError(
            "controller and bridge configs must be updated as a pair"
        )

    if not args.offline:
        running = _running_mask_consumers()
        if running:
            raise LegMaskError(
                "stop every mask-consuming node before changing configuration: "
                + ", ".join(running)
            )
        command_publishers = _visible_command_publishers()
        if command_publishers:
            raise LegMaskError(
                "stop every motor-command publisher before changing configuration: "
                + ", ".join(command_publishers)
            )
        live_power = _live_power_is_on()
        if live_power is True:
            raise LegMaskError("/power/state reports relay power=true; turn it off first")
        if live_power is None:
            raise LegMaskError(
                "cannot prove relay power is off from /power/state; use --offline "
                "only after manually confirming relay power is off"
            )

    existing_fsm_path = (
        fsm_path if fsm_path is not None and fsm_path.is_file() else None
    )
    current = inspect_configs(
        fsm_path=existing_fsm_path,
        controller_path=controller_path,
        bridge_path=bridge_path,
    )
    policy_changed = False
    controller_config = current["configs"].get("controller")
    bridge_config = current["configs"].get("bridge")
    if controller_config is not None and bridge_config is not None:
        target_text = _mask_text(mask)
        policy_changed = (
            controller_config["mask"] != target_text
            or bridge_config["mask"] != target_text
            or not current.get("bridge_hash", {}).get("ok", False)
            or (
                bool(mask)
                and controller_config["disabled_leg_observation_mode"] != "nominal"
            )
        )

    updates: dict[Path, str] = {}
    originals: dict[Path, str | None] = {}
    if fsm_path is not None:
        if fsm_path.is_file():
            fsm_text, _ = _read_yaml(fsm_path, "FSM config")
            originals[fsm_path] = fsm_text
        elif initialize_fsm:
            template_path = _fsm_template_path()
            fsm_text, template_document = _read_yaml(template_path, "FSM template")
            _fsm_masks(template_document)
            originals[fsm_path] = None
        else:  # guarded by _config_paths
            raise LegMaskError(f"configuration file does not exist: {fsm_path}")
        updates[fsm_path] = _update_fsm_text(fsm_text, mask)
    semantic_bridge_hash = ""
    if controller_path is not None and bridge_path is not None:
        controller_text, _ = _read_yaml(controller_path, "controller config")
        bridge_text, _ = _read_yaml(bridge_path, "bridge config")
        originals[controller_path] = controller_text
        originals[bridge_path] = bridge_text
        updated_controller, updated_bridge, semantic_bridge_hash = _update_policy_texts(
            controller_text,
            bridge_text,
            mask,
            invalidate_policy=policy_changed,
        )
        updates[controller_path] = updated_controller
        updates[bridge_path] = updated_bridge

    changed_updates = {
        path: text for path, text in updates.items() if originals.get(path) != text
    }

    if args.dry_run:
        print(f"DRY RUN: would set disabled leg to {_mask_text(mask)}")
        for path in changed_updates:
            action = "create" if originals.get(path) is None else "update"
            print(f"  would {action}: {path}")
        if not changed_updates:
            print("  configuration is already at the requested mask")
        if semantic_bridge_hash:
            print(f"  bridge semantic SHA256: {semantic_bridge_hash}")
        if policy_changed and controller_path is not None:
            print(
                "  policy SHA and leg-dependent calibration acknowledgements would "
                "be cleared; recalibration/repackage is required"
            )
        return 0

    def validate_update() -> None:
        report = inspect_configs(
            fsm_path=fsm_path,
            controller_path=controller_path,
            bridge_path=bridge_path,
        )
        expected = _mask_text(mask)
        if not report["mask_ok"] or report.get("effective_mask") != expected:
            issues = "; ".join(report.get("issues", [])) or (
                f"effective mask is {report.get('effective_mask')}, expected {expected}"
            )
            raise LegMaskError(f"post-write mask verification failed: {issues}")

    if not changed_updates:
        validate_update()
        print(f"startup mask 已经是: {_mask_text(mask)}（无需写入）")
        return 0

    backup = _write_transaction(
        changed_updates,
        expected_originals={path: originals[path] for path in changed_updates},
        validate=validate_update,
    )
    print(f"已将 startup mask 设为: {_mask_text(mask)}")
    for path in changed_updates:
        action = "已建立" if originals.get(path) is None else "已更新"
        print(f"  {action} {path}")
    print(f"  备份目录 {backup}")
    if semantic_bridge_hash:
        print(f"  bridge semantic SHA256: {semantic_bridge_hash}")
    if policy_changed and controller_path is not None:
        print(
            "  旧 policy 与腿相关校正确认已失效：policy.expected_sha256、"
            "Main/ABAD/hardware mapping acknowledgements 已复位；必须重新校正并用"
            "这个 mask 执行 compare/package/preflight。"
        )
    print("配置只会在下次启动时生效；本工具没有启动节点，也没有上电。")
    return 0


def _print_report(report: dict[str, Any]) -> None:
    print("配置中的 startup mask：")
    for name, config in report["configs"].items():
        if name == "fsm":
            print(f"  FSM: {config['path']}")
            for node, mask in config["masks"].items():
                print(f"    {node}: {mask}")
        else:
            print(f"  {name}: {config['mask']}  ({config['path']})")
    print(
        "一致性: "
        + (f"OK ({report['effective_mask']})" if report["mask_ok"] else "MISMATCH")
    )
    mapping = report.get("disabled_leg_mapping")
    if mapping is not None:
        print(
            "屏蔽腿映射: "
            f"{mapping['position']}, Rinbo index {mapping['rinbo_index']}, "
            f"power ch {mapping['power_channel']}, policy index {mapping['policy_index']}, "
            f"Main {mapping['main']}, Servo {mapping['servo']}（Servo 必须实体隔离）"
        )
    if "bridge_hash" in report:
        status = "OK" if report["bridge_hash"]["ok"] else "MISMATCH"
        print(f"Controller ↔ Bridge semantic hash: {status}")
    policy = report.get("policy")
    if policy is not None:
        if policy["mask_binding_ok"]:
            print(
                f"ONNX mask/config binding: MATCH ({policy['path']}); "
                "仍须 package/preflight 全部通过"
            )
        else:
            print(
                "ONNX mask/config binding: NOT MATCHED "
                f"({policy['path'] or 'not configured'})"
            )
            for issue in policy["issues"]:
                print(f"  - {issue}")
    live = report.get("live")
    if live is not None:
        if not live.get("available"):
            print(f"Live ROS graph: unavailable ({live.get('error', '')})")
        else:
            print(f"Live relay: {live['power']}")
            if not live["masks"]:
                print("Live mask consumers: none")
            for node, mask in live["masks"].items():
                print(f"  {node}: {mask}")
    for issue in report["issues"]:
        print(f"ERROR: {issue}")
    for warning in report["warnings"]:
        print(f"WARNING: {warning}")


def _status(args: argparse.Namespace, *, verify: bool) -> int:
    fsm_path, controller_path, bridge_path = _config_paths(
        args, require_all=verify and args.scope == "policy"
    )
    report = inspect_configs(
        fsm_path=fsm_path,
        controller_path=controller_path,
        bridge_path=bridge_path,
        onnx_override=args.onnx,
    )
    if args.live:
        report["live"] = _live_masks()
        configured = report.get("effective_mask")
        if configured is not None and report["live"].get("available"):
            live_mismatches = {
                node: mask
                for node, mask in report["live"].get("masks", {}).items()
                if mask != configured
            }
            if live_mismatches:
                report["mask_ok"] = False
                report["issues"].append(
                    "live masks differ from config: "
                    + ", ".join(f"{node}={mask}" for node, mask in live_mismatches.items())
                )
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_report(report)
    if not report["mask_ok"]:
        return 1
    if verify and args.scope == "policy":
        policy = report.get("policy")
        if policy is None or not policy["mask_binding_ok"]:
            return 1
    return 0


def _commands(args: argparse.Namespace) -> int:
    fsm_path, controller_path, bridge_path = _config_paths(args, require_all=False)
    report = inspect_configs(
        fsm_path=fsm_path,
        controller_path=controller_path,
        bridge_path=bridge_path,
    )
    if not report["mask_ok"]:
        _print_report(report)
        raise LegMaskError("refusing to print run commands for mismatched masks")
    mask = report["effective_mask"]
    launch_mask = "" if mask == "none" else str(mask)
    quote = shlex.quote
    if launch_mask:
        print(f"export REDRHEX_BAD_LEG={quote(launch_mask)}")
        print(
            "# 注意：power_tool 的 --disabled-leg 只调整 current gate，不会设置控制 mask"
        )
        print(
            "ros2 run redrhex_lowlevel_bridge rinbo_power_tool status "
            f"--disabled-leg {quote(launch_mask)}"
        )
    else:
        print("unset REDRHEX_BAD_LEG  # strict six-leg mode")
    if fsm_path is not None:
        config = quote(str(fsm_path))
        print("# 一次只运行一个；每次退出后先确认 output=false")
        for executable in ("rinbo_cali", "rinbo_standing", "rinbo_tripod"):
            print(
                f"ros2 run rinbo_fsm {executable} --ros-args --params-file {config}"
            )
        print("timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once")
    if controller_path is not None and bridge_path is not None:
        print("\n# Policy：先重新封装并让 verify --scope policy 通过")
        print(
            "ros2 run redrhex_rl_controller preflight_check "
            f"--config {quote(str(controller_path))} "
            f"--bridge-config {quote(str(bridge_path))} "
            f"--disabled-legs {quote(launch_mask)}"
        )
        print("# 只读启动（不会允许硬件 output）")
        print(
            "ros2 launch redrhex_rl_controller redrhex_policy_bringup.launch.py "
            "safety_profile:=custom "
            f"config:={quote(str(controller_path))} "
            f"bridge_config:={quote(str(bridge_path))} "
            f"disabled_legs:={quote(launch_mask)} "
            "bridge_rinbo_allow_enable:=false start_bridge:=true use_fake_sensors:=false"
        )
    return 0


def _auto_current_mask(args: argparse.Namespace) -> str:
    paths: list[Path | None] = []
    for value in (
        args.fsm_config,
        args.controller_config,
        args.bridge_config,
    ):
        text = str(value or "").strip()
        path = Path(text).expanduser().resolve() if text else None
        paths.append(path if path is not None and path.is_file() else None)
    if not any(paths):
        return "unknown（尚未找到现场配置）"
    try:
        report = inspect_configs(
            fsm_path=paths[0],
            controller_path=paths[1],
            bridge_path=paths[2],
        )
    except (LegMaskError, OSError, ValueError) as exc:
        return f"unknown（{exc}）"
    if not report["mask_ok"]:
        return "MISMATCH（三份配置不一致）"
    return str(report["effective_mask"])


def _print_auto_menu(current: str) -> None:
    print(f"当前 startup mask: {current}")
    print("请选择要屏蔽的实体腿：")
    for index, leg in enumerate(LEG_NAMES, start=1):
        info = LEG_INFO[leg]
        print(
            f"  {index}. {leg}  {info['position']}  "
            f"Main={info['main']}  Servo={info['servo']}"
        )
    print("  7. none  解除屏蔽，恢复严格六腿模式")
    print("  q. 取消（不修改任何配置）")


def _select_auto_target(current: str) -> list[str] | None:
    _print_auto_menu(current)
    try:
        answer = input("选择 [1-7/q]: ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\n已取消；没有修改任何配置。")
        return None
    if answer.lower() in ("q", "quit", "cancel"):
        print("已取消；没有修改任何配置。")
        return None
    if not answer:
        raise LegMaskError("空白选择不会解除屏蔽；请明确输入 7 或 none")
    if answer in tuple(str(index) for index in range(1, 7)):
        return [LEG_NAMES[int(answer) - 1]]
    if answer.lower() in ("7", "none", "clear"):
        return []
    try:
        return _canonical_target(answer)
    except (TypeError, ValueError) as exc:
        raise LegMaskError(
            f"无效选择 {answer!r}；只能选择 1-7、L1-L3、R1-R3 或 q"
        ) from exc


def _print_auto_target(current: str, mask: list[str]) -> None:
    target = _mask_text(mask)
    print(f"准备切换 startup mask: {current} -> {target}")
    if mask:
        info = LEG_INFO[mask[0]]
        print(
            f"  {mask[0]} = {info['position']}, Main {info['main']}, "
            f"Servo {info['servo']}"
        )
        print(
            f"  注意：软件会屏蔽 Main；{info['servo']} 仍必须由操作者实体隔离，"
            "其余五腿必须确认无已知损坏，才可进入后续架空／低功率校正。"
        )
    else:
        print("  注意：none 只允许在六腿都已修复、校正并验证后使用。")


def _print_auto_paths(args: argparse.Namespace) -> None:
    print("  将统一检查／更新：")
    for label, value in (
        ("FSM", args.fsm_config),
        ("Controller", args.controller_config),
        ("Bridge", args.bridge_config),
    ):
        path = Path(str(value)).expanduser().resolve()
        suffix = "（FSM 缺失时自动建立）" if label == "FSM" and not path.exists() else ""
        print(f"    {label}: {path}{suffix}")


def _confirm_auto(mask: list[str]) -> bool:
    if mask:
        prompt = (
            "确认所有控制节点已停止、relay 已断电、目标腿已实体隔离，"
            "且其余五腿无已知损坏、可安全进入架空／低功率校正？ "
            "[y/N]: "
        )
    else:
        prompt = (
            "确认 relay 已断电，且六腿都已修复、重新校正并完成低功率验证？ "
            "[y/N]: "
        )
    try:
        answer = input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\n已取消；没有修改任何配置。")
        return False
    if answer not in ("y", "yes", "是"):
        print("已取消；没有修改任何配置。")
        return False
    return True


def _auto(args: argparse.Namespace) -> int:
    current = _auto_current_mask(args)
    if args.target is None:
        selected = _select_auto_target(current)
        if selected is None:
            return 0
        mask = selected
    else:
        if not str(args.target).strip():
            raise LegMaskError("target 不可为空；解除屏蔽必须明确输入 none")
        mask = _canonical_target(args.target)

    _print_auto_target(current, mask)
    _print_auto_paths(args)
    if not args.yes and not _confirm_auto(mask):
        return 0

    # --yes is an explicit operator certification, not a power-control action.
    # Online graph/publisher/relay checks still run unless --offline was supplied.
    args.confirm_power_off = True
    args.confirm_all_six_verified = not mask
    args.dry_run = False
    args.initialize_missing_fsm = True
    result = _prepare_update(args, mask)
    if result != 0:
        return result

    expected = _mask_text(mask)
    print(f"SUCCESS：三份启动配置已一致，当前屏蔽腿 = {expected}")
    if mask:
        print(f"当前 terminal 如需旧流程变量：export REDRHEX_BAD_LEG={mask[0]}")
        print(
            "ONNX policy 必须针对这个 mask 重新校正、比较、封装并通过 preflight。"
        )
    else:
        print("当前 terminal 如需旧流程变量：unset REDRHEX_BAD_LEG")
        print("已恢复严格六腿配置；启动前仍须完成六腿 preflight。")
    print(
        "可执行 `ros2 run redrhex_rl_controller rinbo_leg_mask commands` "
        "取得下一阶段命令。"
    )
    return 0


def _add_config_arguments(
    parser: argparse.ArgumentParser, *, include_allow_partial: bool = True
) -> None:
    parser.add_argument(
        "--fsm-config",
        default=_default_config_path(
            "REDRHEX_FSM_CFG", DEFAULT_CONFIG_FILENAMES["fsm"]
        ),
    )
    parser.add_argument(
        "--controller-config",
        default=_default_config_path(
            "REDRHEX_SITE_CTRL", DEFAULT_CONFIG_FILENAMES["controller"]
        ),
    )
    parser.add_argument(
        "--bridge-config",
        default=_default_config_path(
            "REDRHEX_SITE_BRIDGE", DEFAULT_CONFIG_FILENAMES["bridge"]
        ),
    )
    if include_allow_partial:
        parser.add_argument(
            "--allow-partial",
            action="store_true",
            help=(
                "Intentionally operate on FSM only; controller/bridge are not "
                "read or written."
            ),
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Prepare/check one startup-only disabled-leg mask across Rinbo FSM, "
            "RL controller, bridge, and packaged ONNX contracts."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    auto = subparsers.add_parser(
        "auto",
        help="Interactive or one-command safe mask/unmask workflow (recommended)",
    )
    _add_config_arguments(auto, include_allow_partial=False)
    auto.add_argument(
        "target",
        nargs="?",
        metavar="{L1,L2,L3,R1,R2,R3,none}",
        help="Leg to mask, or none to restore strict six-leg mode; omit for a menu.",
    )
    auto.add_argument(
        "--yes",
        action="store_true",
        help=(
            "Skip the prompt and certify relay-off/physical isolation; with none, "
            "certify all six legs are repaired/calibrated/verified; with a leg, "
            "certify the other five have no known damage and are safe to calibrate."
        ),
    )
    auto.add_argument(
        "--offline",
        action="store_true",
        help=(
            "Skip live ROS checks only after manually stopping all writers and "
            "confirming relay power is off."
        ),
    )

    status = subparsers.add_parser("status", help="Show configured and optional live masks")
    _add_config_arguments(status)
    status.add_argument("--onnx", default="")
    status.add_argument("--live", action="store_true")
    status.add_argument("--json", action="store_true")

    verify = subparsers.add_parser("verify", help="Fail if the selected contract is inconsistent")
    _add_config_arguments(verify)
    verify.add_argument("--scope", choices=("fsm", "policy"), default="fsm")
    verify.add_argument("--onnx", default="")
    verify.add_argument("--live", action="store_true")
    verify.add_argument("--json", action="store_true")

    set_parser = subparsers.add_parser("set", help="Set one disabled leg in site YAML files")
    _add_config_arguments(set_parser)
    set_parser.add_argument("leg", choices=LEG_NAMES, type=str.upper)
    set_parser.add_argument("--confirm-power-off", action="store_true")
    set_parser.add_argument("--offline", action="store_true")
    set_parser.add_argument("--dry-run", action="store_true")

    clear = subparsers.add_parser("clear", help="Return to strict six-leg mode")
    _add_config_arguments(clear)
    clear.add_argument("--confirm-power-off", action="store_true")
    clear.add_argument("--confirm-all-six-verified", action="store_true")
    clear.add_argument("--offline", action="store_true")
    clear.add_argument("--dry-run", action="store_true")

    commands = subparsers.add_parser("commands", help="Print commands using the selected configs")
    _add_config_arguments(commands)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "auto":
            return _auto(args)
        if args.command == "status":
            return _status(args, verify=False)
        if args.command == "verify":
            return _status(args, verify=True)
        if args.command == "set":
            return _prepare_update(args, _canonical_target(args.leg))
        if args.command == "clear":
            if not args.confirm_all_six_verified:
                raise LegMaskError(
                    "clear requires --confirm-all-six-verified after repair, "
                    "calibration, and validation"
                )
            return _prepare_update(args, [])
        if args.command == "commands":
            return _commands(args)
        parser.error(f"unsupported command {args.command}")
    except (LegMaskError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
