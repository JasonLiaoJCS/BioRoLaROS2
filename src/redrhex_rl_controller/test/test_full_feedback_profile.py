from __future__ import annotations

import copy
from pathlib import Path

import yaml

from redrhex_rl_controller.preflight_check import _validate_deployment_config


def _profile_params() -> dict:
    profile = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "redrhex_policy_full_feedback_rig.yaml"
    )
    document = yaml.safe_load(profile.read_text(encoding="utf-8"))
    return document["redrhex_rl_controller"]["ros__parameters"]


def _checks_for(params: dict) -> dict[str, dict]:
    result: dict[str, object] = {"checks": [], "warnings": [], "next_steps": []}
    _validate_deployment_config(params, 56, result)
    return {check["name"]: check for check in result["checks"]}


def _calibrated_params() -> dict:
    params = copy.deepcopy(_profile_params())
    observation = params["observation"]
    observation["imu_alignment_calibrated"] = True
    observation["expected_imu_frame_id"] = "imu_link"
    observation["expected_imu_publisher_node"] = "imu_driver"
    params["action"]["hardware_mapping_calibrated"] = True
    return params


def test_checked_in_full_feedback_profile_fails_closed_until_imu_is_calibrated() -> None:
    checks = _checks_for(_profile_params())

    assert checks["complete_controller_deployment_profile"]["ok"]
    assert not checks["observation_builder_config"]["ok"]
    assert not checks["full_feedback_rig_invariants"]["ok"]


def test_full_feedback_profile_requires_fixed_forward_and_real_imu_contract() -> None:
    params = _calibrated_params()
    checks = _checks_for(params)

    assert checks["observation_builder_config"]["ok"]
    assert checks["full_feedback_rig_invariants"]["ok"]

    params["commands"]["profile"] = "external_cmd_vel"
    checks = _checks_for(params)
    assert not checks["full_feedback_rig_invariants"]["ok"]
