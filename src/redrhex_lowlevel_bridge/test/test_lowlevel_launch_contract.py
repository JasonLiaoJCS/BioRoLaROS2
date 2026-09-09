from __future__ import annotations

import ast
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "lowlevel_bridge.launch.py"
)
ALLOWED_ARGUMENTS = {
    "safety_profile",
    "config",
    "disabled_legs",
    "max_disabled_legs",
}
ALLOWED_RUNTIME_OVERRIDES = {
    "hardware.disabled_legs",
    "hardware.max_disabled_legs",
}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "redrhex_lowlevel_bridge_launch_contract", LAUNCH_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _runtime_override_keys(source: str) -> set[str]:
    keys: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if not isinstance(target, ast.Subscript):
                    continue
                if not isinstance(target.value, ast.Name):
                    continue
                if target.value.id != "override_params":
                    continue
                if isinstance(target.slice, ast.Constant) and isinstance(
                    target.slice.value, str
                ):
                    keys.add(target.slice.value)
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "_maybe_add" or len(node.args) < 2:
            continue
        if not isinstance(node.args[0], ast.Name):
            continue
        if node.args[0].id != "override_params":
            continue
        if isinstance(node.args[1], ast.Constant) and isinstance(
            node.args[1].value, str
        ):
            keys.add(node.args[1].value)
    return keys


def test_standalone_launch_show_args_exposes_only_operational_contract() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()
    declared = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert declared == ALLOWED_ARGUMENTS


def test_standalone_launch_cannot_override_semantic_safety_or_backend() -> None:
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert _runtime_override_keys(source) == ALLOWED_RUNTIME_OVERRIDES
    assert 'LaunchConfiguration("backend")' not in source


def test_standalone_launch_offers_full_feedback_profile() -> None:
    module = _load_launch_module()
    module.get_package_share_directory = lambda _package: "/tmp/redrhex_share"

    selected = module._profile_config("full_feedback_rig")

    assert selected.endswith("/config/lowlevel_bridge_full_feedback_rig.yaml")
