import inspect
import math

import pytest

from redrhex_lowlevel_bridge.rinbo_bringup_check import (
    CheckResult,
    RinboBringupCheck,
    _exact_publisher_result,
    _fresh_boolean_result,
    _fresh_message_result,
    _fresh_power_chain_result,
    _power_state_required,
    _result_exit_code,
    _stream_stats,
    _validate_args,
    build_parser,
)


def test_stream_stats_reports_rate_and_max_gap() -> None:
    count, rate_hz, max_gap_s = _stream_stats([1.0, 1.02, 1.04, 1.10])

    assert count == 4
    assert rate_hz == pytest.approx(30.0)
    assert max_gap_s == pytest.approx(0.06)


def test_stream_stats_requires_two_messages() -> None:
    count, rate_hz, max_gap_s = _stream_stats([1.0])

    assert count == 1
    assert rate_hz == 0.0
    assert math.isinf(max_gap_s)


@pytest.mark.parametrize("count", [0, 2])
def test_required_publisher_cardinality_is_fail_closed(count: int) -> None:
    result = _exact_publisher_result("/imu/data", count, "none")

    assert result.level == "ERROR"
    assert f"got {count}" in result.detail


def test_required_publisher_accepts_exactly_one() -> None:
    result = _exact_publisher_result(
        "/rinbo/motor_output_enabled", 1, "/rinbo_ros_bridge"
    )

    assert result.level == "OK"


def test_handoff_boolean_requires_fresh_exact_value() -> None:
    ok = _fresh_boolean_result(
        "/rinbo/motor_output_enabled disabled",
        seen=True,
        value=False,
        expected=False,
        receipt_monotonic_s=9.95,
        now_monotonic_s=10.0,
        timeout_s=0.20,
    )
    enabled = _fresh_boolean_result(
        "/rinbo/motor_output_enabled disabled",
        seen=True,
        value=True,
        expected=False,
        receipt_monotonic_s=9.95,
        now_monotonic_s=10.0,
        timeout_s=0.20,
    )
    stale = _fresh_boolean_result(
        "power relay on",
        seen=True,
        value=True,
        expected=True,
        receipt_monotonic_s=9.0,
        now_monotonic_s=10.0,
        timeout_s=0.35,
    )

    assert ok.level == "OK"
    assert enabled.level == "ERROR"
    assert "expected false" in enabled.detail
    assert stale.level == "ERROR"
    assert "stale" in stale.detail


def test_fresh_message_rejects_missing_and_future_receipts() -> None:
    missing = _fresh_message_result(
        "/imu/data",
        seen=False,
        receipt_monotonic_s=None,
        now_monotonic_s=10.0,
        timeout_s=0.10,
    )
    future = _fresh_message_result(
        "/imu/data",
        seen=True,
        receipt_monotonic_s=10.1,
        now_monotonic_s=10.0,
        timeout_s=0.10,
    )

    assert missing.level == "ERROR"
    assert future.level == "ERROR"


def test_power_chain_accepts_one_fresh_all_on_state() -> None:
    result = _fresh_power_chain_result(
        seen=True,
        digital=True,
        signal=True,
        power=True,
        receipt_monotonic_s=9.80,
        now_monotonic_s=10.0,
        timeout_s=0.35,
    )

    assert result.level == "OK"
    assert "digital=true, signal=true, power=true" in result.detail
    assert "fresh message" in result.detail


@pytest.mark.parametrize("off_relay", ["digital", "signal", "power"])
def test_power_chain_rejects_each_false_relay(off_relay: str) -> None:
    states = {"digital": True, "signal": True, "power": True}
    states[off_relay] = False

    result = _fresh_power_chain_result(
        seen=True,
        **states,
        receipt_monotonic_s=9.80,
        now_monotonic_s=10.0,
        timeout_s=0.35,
    )

    assert result.level == "ERROR"
    assert f"{off_relay}=false" in result.detail
    assert f"required true: {off_relay}" in result.detail


def test_power_chain_rejects_stale_all_on_state() -> None:
    result = _fresh_power_chain_result(
        seen=True,
        digital=True,
        signal=True,
        power=True,
        receipt_monotonic_s=9.60,
        now_monotonic_s=10.0,
        timeout_s=0.35,
    )

    assert result.level == "ERROR"
    assert "stale" in result.detail


def test_policy_handoff_cli_flags_form_one_read_only_check() -> None:
    args = build_parser().parse_args(
        [
            "--strict",
            "--max-motor-command-publishers",
            "0",
            "--require-power-relay-on",
            "--require-motor-output-disabled",
            "--require-imu",
        ]
    )
    _validate_args(args)

    assert args.max_motor_command_publishers == 0
    assert _power_state_required(args) is True
    assert args.require_motor_output_disabled is True
    assert args.motor_output_status_topic == "/rinbo/motor_output_enabled"
    assert args.require_imu is True
    assert args.imu_topic == "/imu/data"


def test_handoff_cli_rejects_invalid_limits() -> None:
    args = build_parser().parse_args(
        ["--max-motor-command-publishers", "-1"]
    )
    with pytest.raises(ValueError, match="must be >= 0"):
        _validate_args(args)


@pytest.mark.parametrize(
    ("results", "strict", "expected"),
    [
        ([CheckResult("OK", "check", "ok")], False, 0),
        ([CheckResult("WARN", "check", "warning")], False, 0),
        ([CheckResult("WARN", "check", "warning")], True, 1),
        ([CheckResult("ERROR", "check", "failure")], False, 2),
        ([CheckResult("ERROR", "check", "failure")], True, 2),
        (
            [
                CheckResult("WARN", "warn", "warning"),
                CheckResult("ERROR", "error", "failure"),
            ],
            True,
            2,
        ),
    ],
)
def test_result_exit_code_is_strict_and_error_dominant(
    results: list[CheckResult], strict: bool, expected: int
) -> None:
    assert _result_exit_code(results, strict=strict) == expected


def test_bringup_checker_has_no_command_publisher() -> None:
    source = inspect.getsource(RinboBringupCheck)

    assert "create_publisher" not in source
    assert ".publish(" not in source
    assert "create_client" not in source
    assert "call_async" not in source
