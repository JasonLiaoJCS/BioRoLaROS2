from __future__ import annotations

from pathlib import Path
import subprocess

import pytest
import yaml

import redrhex_rl_controller.rinbo_leg_mask as leg_mask_module
from redrhex_rl_controller.golden_policy import bridge_config_sha256
from redrhex_rl_controller.rinbo_leg_mask import (
    FSM_NODES,
    LEG_NAMES,
    LegMaskError,
    _canonical_target,
    _fsm_masks,
    _load_yaml_text,
    _named_params,
    _params_mask,
    _replace_yaml_scalar,
    _update_fsm_text,
    _update_policy_texts,
    inspect_configs,
    main,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
FSM_TEMPLATE = WORKSPACE_SRC / "rinbo_fsm" / "config" / "disabled_leg_template.yaml"
CONTROLLER_TEMPLATE = PACKAGE_ROOT / "config" / "redrhex_policy_full_feedback_rig.yaml"
BRIDGE_TEMPLATE = (
    WORKSPACE_SRC
    / "redrhex_lowlevel_bridge"
    / "config"
    / "lowlevel_bridge_full_feedback_rig.yaml"
)


def _write_site_configs(
    tmp_path: Path,
    *,
    mask: list[str] | None = None,
    create_fsm: bool = True,
) -> tuple[Path, Path, Path]:
    """Create an isolated, internally consistent set of site YAMLs."""

    selected_mask = list(mask or [])
    site_dir = tmp_path / "site"
    site_dir.mkdir()
    fsm = site_dir / "rinbo_fsm_disabled_leg.yaml"
    controller = site_dir / "redrhex_policy.yaml"
    bridge = site_dir / "lowlevel_bridge.yaml"

    if create_fsm:
        fsm.write_text(
            _update_fsm_text(
                FSM_TEMPLATE.read_text(encoding="utf-8"), selected_mask
            ),
            encoding="utf-8",
        )
    controller_text, bridge_text, _ = _update_policy_texts(
        CONTROLLER_TEMPLATE.read_text(encoding="utf-8"),
        BRIDGE_TEMPLATE.read_text(encoding="utf-8"),
        selected_mask,
        invalidate_policy=True,
    )
    controller.write_text(controller_text, encoding="utf-8")
    bridge.write_text(bridge_text, encoding="utf-8")
    return fsm, controller, bridge


def _auto_args(
    target: str | None,
    fsm: Path,
    controller: Path,
    bridge: Path,
    *,
    yes: bool = False,
    offline: bool = True,
) -> list[str]:
    args = ["auto"]
    if target is not None:
        args.append(target)
    if yes:
        args.append("--yes")
    if offline:
        args.append("--offline")
    args.extend(
        [
            "--fsm-config",
            str(fsm),
            "--controller-config",
            str(controller),
            "--bridge-config",
            str(bridge),
        ]
    )
    return args


@pytest.mark.parametrize("leg", LEG_NAMES)
def test_every_named_leg_updates_all_fsm_and_policy_sections(leg: str) -> None:
    fsm_text = _update_fsm_text(FSM_TEMPLATE.read_text(encoding="utf-8"), [leg])
    controller_text, bridge_text, semantic_hash = _update_policy_texts(
        CONTROLLER_TEMPLATE.read_text(encoding="utf-8"),
        BRIDGE_TEMPLATE.read_text(encoding="utf-8"),
        [leg],
        invalidate_policy=True,
    )

    assert set(_fsm_masks(_load_yaml_text(fsm_text, "FSM")).keys()) == set(FSM_NODES)
    assert all(
        mask == [leg]
        for mask in _fsm_masks(_load_yaml_text(fsm_text, "FSM")).values()
    )
    controller = _named_params(
        _load_yaml_text(controller_text, "controller"),
        "redrhex_rl_controller",
        "controller",
    )
    bridge = _named_params(
        _load_yaml_text(bridge_text, "bridge"),
        "redrhex_lowlevel_bridge",
        "bridge",
    )
    assert _params_mask(controller, "controller") == [leg]
    assert _params_mask(bridge, "bridge") == [leg]
    assert controller["policy"]["expected_bridge_config_sha256"] == semantic_hash
    assert semantic_hash == bridge_config_sha256(bridge)
    assert controller["policy"]["expected_sha256"] == ""


def test_policy_sha_is_preserved_when_mask_did_not_change() -> None:
    controller_text = _replace_yaml_scalar(
        CONTROLLER_TEMPLATE.read_text(encoding="utf-8"),
        (
            "redrhex_rl_controller",
            "ros__parameters",
            "policy",
            "expected_sha256",
        ),
        f'"{"a" * 64}"',
    )

    updated, _, _ = _update_policy_texts(
        controller_text,
        BRIDGE_TEMPLATE.read_text(encoding="utf-8"),
        ["L1"],
        invalidate_policy=False,
    )
    params = _named_params(
        _load_yaml_text(updated, "controller"),
        "redrhex_rl_controller",
        "controller",
    )
    assert params["policy"]["expected_sha256"] == "a" * 64


def test_mask_change_resets_leg_dependent_policy_calibration_gates() -> None:
    controller_text = CONTROLLER_TEMPLATE.read_text(encoding="utf-8").replace(
        "hardware_mapping_calibrated: false", "hardware_mapping_calibrated: true"
    )
    bridge_text = BRIDGE_TEMPLATE.read_text(encoding="utf-8")
    for name in (
        "main_drive_calibrated",
        "abad_feedback_calibrated",
        "abad_command_calibrated",
    ):
        bridge_text = bridge_text.replace(f"{name}: false", f"{name}: true")

    updated_controller, updated_bridge, _ = _update_policy_texts(
        controller_text, bridge_text, ["R2"], invalidate_policy=True
    )
    controller = _named_params(
        _load_yaml_text(updated_controller, "controller"),
        "redrhex_rl_controller",
        "controller",
    )
    bridge = _named_params(
        _load_yaml_text(updated_bridge, "bridge"),
        "redrhex_lowlevel_bridge",
        "bridge",
    )

    assert controller["action"]["hardware_mapping_calibrated"] is False
    assert bridge["rinbo"]["main_drive_calibrated"] is False
    assert bridge["rinbo"]["abad_feedback_calibrated"] is False
    assert bridge["rinbo"]["abad_command_calibrated"] is False


def test_disabled_leg_forces_nominal_observation_mode() -> None:
    controller_text = CONTROLLER_TEMPLATE.read_text(encoding="utf-8").replace(
        'disabled_leg_observation_mode: "nominal"',
        'disabled_leg_observation_mode: "passthrough"',
    )

    updated_controller, _, _ = _update_policy_texts(
        controller_text,
        BRIDGE_TEMPLATE.read_text(encoding="utf-8"),
        ["R1"],
        invalidate_policy=True,
    )
    controller = _named_params(
        _load_yaml_text(updated_controller, "controller"),
        "redrhex_rl_controller",
        "controller",
    )

    assert controller["observation"]["disabled_leg_observation_mode"] == "nominal"


def test_inspection_detects_one_mismatched_fsm_section(tmp_path: Path) -> None:
    fsm = tmp_path / "fsm.yaml"
    text = _update_fsm_text(FSM_TEMPLATE.read_text(encoding="utf-8"), ["L2"])
    text = _replace_yaml_scalar(
        text,
        (
            "rinbo_tripod_rslip",
            "ros__parameters",
            "hardware",
            "disabled_legs",
        ),
        '["R3"]',
    )
    fsm.write_text(text, encoding="utf-8")

    report = inspect_configs(
        fsm_path=fsm, controller_path=None, bridge_path=None
    )

    assert not report["mask_ok"]
    assert "disabled-leg mismatch" in report["issues"][0]


def test_verify_fsm_checks_supplied_peer_configs_unless_partial_is_explicit(
    tmp_path: Path
) -> None:
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    controller.write_text(
        _replace_yaml_scalar(
            controller.read_text(encoding="utf-8"),
            (
                "redrhex_rl_controller",
                "ros__parameters",
                "hardware",
                "disabled_legs",
            ),
            '["R2"]',
        ),
        encoding="utf-8",
    )
    arguments = [
        "verify",
        "--scope",
        "fsm",
        "--fsm-config",
        str(fsm),
        "--controller-config",
        str(controller),
        "--bridge-config",
        str(bridge),
    ]

    assert main(arguments) != 0
    assert main([*arguments, "--allow-partial"]) == 0


@pytest.mark.parametrize(
    ("value", "expected"),
    [("l1", ["L1"]), (" R3 ", ["R3"]), ("none", []), ("[]", [])],
)
def test_cli_target_spelling_is_canonical(value: str, expected: list[str]) -> None:
    assert _canonical_target(value) == expected


def test_clear_requires_explicit_six_leg_verification() -> None:
    result = main(["clear", "--confirm-power-off", "--offline", "--allow-partial"])

    assert result == 2


def test_set_allow_partial_updates_an_fsm_only_site_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm = tmp_path / "fsm-site.yaml"
    fsm.write_text(FSM_TEMPLATE.read_text(encoding="utf-8"), encoding="utf-8")

    result = main(
        [
            "set",
            "R3",
            "--fsm-config",
            str(fsm),
            "--allow-partial",
            "--confirm-power-off",
            "--offline",
        ]
    )

    assert result == 0
    masks = _fsm_masks(_load_yaml_text(fsm.read_text(encoding="utf-8"), "FSM"))
    assert set(masks) == set(FSM_NODES)
    assert all(mask == ["R3"] for mask in masks.values())


def test_auto_named_leg_yes_updates_and_verifies_every_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm, controller, bridge = _write_site_configs(tmp_path)
    real_inspect = leg_mask_module.inspect_configs
    inspect_calls = 0

    def recording_inspect(*args: object, **kwargs: object) -> dict[str, object]:
        nonlocal inspect_calls
        inspect_calls += 1
        return real_inspect(*args, **kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(leg_mask_module, "inspect_configs", recording_inspect)
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt="": (_ for _ in ()).throw(
            AssertionError("--yes must make auto non-interactive")
        ),
    )

    result = main(_auto_args("R2", fsm, controller, bridge, yes=True))

    assert result == 0
    assert inspect_calls >= 2  # pre-change inspection plus post-write verification
    report = real_inspect(
        fsm_path=fsm, controller_path=controller, bridge_path=bridge
    )
    assert report["mask_ok"]
    assert report["effective_mask"] == "R2"


def test_auto_none_yes_is_the_explicit_six_leg_confirmation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L3"])
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt="": (_ for _ in ()).throw(
            AssertionError("auto none --yes must be non-interactive")
        ),
    )

    result = main(_auto_args("none", fsm, controller, bridge, yes=True))

    assert result == 0
    report = inspect_configs(
        fsm_path=fsm, controller_path=controller, bridge_path=bridge
    )
    assert report["mask_ok"]
    assert report["effective_mask"] == "none"


def test_auto_without_target_can_cancel_from_interactive_menu(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    originals = {
        path: path.read_text(encoding="utf-8")
        for path in (fsm, controller, bridge)
    }
    prompts: list[str] = []

    def cancel_at_menu(prompt: str = "") -> str:
        prompts.append(prompt)
        return "q"

    monkeypatch.setattr("builtins.input", cancel_at_menu)

    result = main(_auto_args(None, fsm, controller, bridge))

    assert result == 0
    assert prompts
    assert all(path.read_text(encoding="utf-8") == text for path, text in originals.items())


def test_auto_explicit_target_without_yes_prompts_and_cancel_does_not_write(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    originals = {
        path: path.read_text(encoding="utf-8")
        for path in (fsm, controller, bridge)
    }
    prompt_count = 0

    def decline(_prompt: str = "") -> str:
        nonlocal prompt_count
        prompt_count += 1
        return "n"

    monkeypatch.setattr("builtins.input", decline)

    result = main(_auto_args("R3", fsm, controller, bridge))

    assert result == 0
    assert prompt_count >= 1
    assert all(path.read_text(encoding="utf-8") == text for path, text in originals.items())


def test_auto_creates_missing_fsm_site_config_from_template(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm, controller, bridge = _write_site_configs(tmp_path, create_fsm=False)
    assert not fsm.exists()
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt="": (_ for _ in ()).throw(
            AssertionError("--yes must make auto non-interactive")
        ),
    )

    result = main(_auto_args("L2", fsm, controller, bridge, yes=True))

    assert result == 0
    assert fsm.is_file()
    report = inspect_configs(
        fsm_path=fsm, controller_path=controller, bridge_path=bridge
    )
    assert report["mask_ok"]
    assert report["effective_mask"] == "L2"
    assert set(report["configs"]["fsm"]["masks"]) == set(FSM_NODES)


def test_auto_does_not_fabricate_a_missing_controller_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    controller.unlink()
    fsm_before = fsm.read_text(encoding="utf-8")
    bridge_before = bridge.read_text(encoding="utf-8")

    result = main(_auto_args("R1", fsm, controller, bridge, yes=True))

    assert result != 0
    assert not controller.exists()
    assert fsm.read_text(encoding="utf-8") == fsm_before
    assert bridge.read_text(encoding="utf-8") == bridge_before


def test_auto_refuses_to_use_the_fsm_package_template_as_its_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    _, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    originals = {
        path: path.read_bytes() for path in (FSM_TEMPLATE, controller, bridge)
    }

    result = main(
        _auto_args("R2", FSM_TEMPLATE, controller, bridge, yes=True)
    )

    assert result != 0
    assert all(path.read_bytes() == content for path, content in originals.items())


def test_auto_still_refuses_to_write_while_a_mask_consumer_is_running(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    originals = {
        path: path.read_text(encoding="utf-8")
        for path in (fsm, controller, bridge)
    }
    monkeypatch.setattr(
        leg_mask_module, "_running_mask_consumers", lambda: ["/rinbo_standing"]
    )

    result = main(
        _auto_args("R1", fsm, controller, bridge, yes=True, offline=False)
    )

    assert result != 0
    assert all(path.read_text(encoding="utf-8") == text for path, text in originals.items())


def test_auto_online_refuses_when_relay_power_cannot_be_proven_off(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    originals = {
        path: path.read_text(encoding="utf-8")
        for path in (fsm, controller, bridge)
    }
    monkeypatch.setattr(leg_mask_module, "_running_mask_consumers", lambda: [])
    monkeypatch.setattr(leg_mask_module, "_visible_command_publishers", lambda: [])
    monkeypatch.setattr(leg_mask_module, "_live_power_is_on", lambda: None)

    result = main(
        _auto_args("R2", fsm, controller, bridge, yes=True, offline=False)
    )

    assert result != 0
    assert all(
        path.read_text(encoding="utf-8") == text
        for path, text in originals.items()
    )


@pytest.mark.parametrize("failure_stage", ["list", "info"])
def test_publisher_inspection_errors_fail_closed(
    failure_stage: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fake_run(
        command: list[str], _timeout_s: float
    ) -> subprocess.CompletedProcess[str]:
        if command == ["ros2", "topic", "list"]:
            return subprocess.CompletedProcess(
                command,
                1 if failure_stage == "list" else 0,
                stdout="/motor/command\n",
                stderr="graph unavailable" if failure_stage == "list" else "",
            )
        assert command == ["ros2", "topic", "info", "/motor/command", "-v"]
        return subprocess.CompletedProcess(
            command, 1, stdout="", stderr="topic info unavailable"
        )

    monkeypatch.setattr(leg_mask_module, "_run_command", fake_run)

    with pytest.raises(LegMaskError, match="cannot inspect"):
        leg_mask_module._visible_command_publishers()


def test_auto_returns_nonzero_when_post_write_inspection_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm, controller, bridge = _write_site_configs(tmp_path)
    originals = {
        path: path.read_text(encoding="utf-8")
        for path in (fsm, controller, bridge)
    }
    real_inspect = leg_mask_module.inspect_configs

    def reject_updated_mask(*args: object, **kwargs: object) -> dict[str, object]:
        report = real_inspect(*args, **kwargs)  # type: ignore[arg-type]
        if report.get("effective_mask") == "R3":
            report["mask_ok"] = False
            report["issues"].append("injected post-write mismatch")
        return report

    monkeypatch.setattr(leg_mask_module, "inspect_configs", reject_updated_mask)

    result = main(_auto_args("R3", fsm, controller, bridge, yes=True))

    assert result != 0
    assert all(
        path.read_text(encoding="utf-8") == text
        for path, text in originals.items()
    )


def test_transaction_refuses_a_file_changed_after_prepare_before_commit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    target = tmp_path / "fsm-site.yaml"
    target.write_text("prepared original\n", encoding="utf-8")
    real_atomic_write = leg_mask_module._atomic_write
    external_text = "external concurrent edit\n"
    injected_change = False

    def change_target_while_writing_backup(path: Path, text: str) -> None:
        nonlocal injected_change
        if path != target and not injected_change:
            target.write_text(external_text, encoding="utf-8")
            injected_change = True
        real_atomic_write(path, text)

    monkeypatch.setattr(
        leg_mask_module, "_atomic_write", change_target_while_writing_backup
    )

    with pytest.raises(LegMaskError, match="changed before commit"):
        leg_mask_module._write_transaction(
            {target: "tool update\n"},
            expected_originals={target: "prepared original\n"},
            validate=lambda: None,
        )

    assert injected_change
    assert target.read_text(encoding="utf-8") == external_text


def test_auto_default_config_path_precedence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    site_dir = tmp_path / "site-defaults"
    monkeypatch.setenv("REDRHEX_SITE_DIR", str(site_dir))
    for name in ("REDRHEX_FSM_CFG", "REDRHEX_SITE_CTRL", "REDRHEX_SITE_BRIDGE"):
        monkeypatch.delenv(name, raising=False)

    site_args = leg_mask_module.build_parser().parse_args(["auto", "L1"])

    assert Path(site_args.fsm_config) == site_dir / "rinbo_fsm_disabled_leg.yaml"
    assert Path(site_args.controller_config) == (
        site_dir / "redrhex_policy_full_feedback_rig.yaml"
    )
    assert Path(site_args.bridge_config) == (
        site_dir / "lowlevel_bridge_full_feedback_rig.yaml"
    )

    environment_paths = {
        "REDRHEX_FSM_CFG": tmp_path / "env-fsm.yaml",
        "REDRHEX_SITE_CTRL": tmp_path / "env-controller.yaml",
        "REDRHEX_SITE_BRIDGE": tmp_path / "env-bridge.yaml",
    }
    for name, path in environment_paths.items():
        monkeypatch.setenv(name, str(path))
    environment_args = leg_mask_module.build_parser().parse_args(["auto", "L2"])

    assert Path(environment_args.fsm_config) == environment_paths["REDRHEX_FSM_CFG"]
    assert Path(environment_args.controller_config) == environment_paths[
        "REDRHEX_SITE_CTRL"
    ]
    assert Path(environment_args.bridge_config) == environment_paths[
        "REDRHEX_SITE_BRIDGE"
    ]

    cli_paths = (
        tmp_path / "cli-fsm.yaml",
        tmp_path / "cli-controller.yaml",
        tmp_path / "cli-bridge.yaml",
    )
    cli_args = leg_mask_module.build_parser().parse_args(
        [
            "auto",
            "R1",
            "--fsm-config",
            str(cli_paths[0]),
            "--controller-config",
            str(cli_paths[1]),
            "--bridge-config",
            str(cli_paths[2]),
        ]
    )

    assert Path(cli_args.fsm_config) == cli_paths[0]
    assert Path(cli_args.controller_config) == cli_paths[1]
    assert Path(cli_args.bridge_config) == cli_paths[2]


def test_auto_rejects_reusing_one_file_for_two_config_roles(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    fsm, controller, bridge = _write_site_configs(tmp_path, mask=["L1"])
    originals = {
        path: path.read_text(encoding="utf-8")
        for path in (fsm, controller, bridge)
    }

    result = main(_auto_args("R1", fsm, controller, controller, yes=True))

    assert result != 0
    assert all(
        path.read_text(encoding="utf-8") == text
        for path, text in originals.items()
    )


def test_yaml_rewriter_preserves_comments() -> None:
    source = (
        "root:\n  hardware:\n    # keep this safety note\n"
        "    disabled_legs: [L1]  # physical isolation\n"
    )

    updated = _replace_yaml_scalar(
        source, ("root", "hardware", "disabled_legs"), '["R2"]'
    )

    assert "# keep this safety note" in updated
    assert 'disabled_legs: ["R2"]  # physical isolation' in updated
    assert yaml.safe_load(updated)["root"]["hardware"]["disabled_legs"] == ["R2"]


def test_fsm_rejects_max_disabled_legs_other_than_one() -> None:
    source = FSM_TEMPLATE.read_text(encoding="utf-8").replace(
        "max_disabled_legs: 1", "max_disabled_legs: 2", 1
    )

    with pytest.raises(LegMaskError, match="must be exactly 1"):
        _update_fsm_text(source, ["L1"])
