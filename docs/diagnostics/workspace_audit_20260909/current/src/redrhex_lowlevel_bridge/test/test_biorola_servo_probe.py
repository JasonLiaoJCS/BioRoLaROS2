from types import SimpleNamespace

import pytest

from redrhex_lowlevel_bridge import biorola_servo_probe
from redrhex_lowlevel_bridge.biorola_servo_probe import (
    relay_is_exactly_on,
    summarize_power_state,
    validate_feedback_header,
    validate_resolved_motion_topics,
)


def _power_state(**overrides):
    values = {f"v_{index}": 5.0 for index in range(8)}
    values.update({f"i_{index}": 0.0 for index in range(8)})
    values.update({"v_7": 24.0, "i_7": 12.0, "power": True})
    values.update(overrides)
    return SimpleNamespace(**values)


def test_power_summary_uses_bus_ch7_and_only_leg_currents() -> None:
    summary = summarize_power_state(_power_state(i_2=2.5), 7, [1, 2, 3, 4, 5, 6])

    assert summary["bus_voltage_v"] == pytest.approx(24.0)
    assert summary["max_abs_leg_current_a"] == pytest.approx(2.5)
    assert summary["relevant_values_finite"]


def test_power_summary_honors_configured_channels() -> None:
    summary = summarize_power_state(
        _power_state(v_6=26.0, i_0=-1.5, i_7=99.0),
        6,
        [0, 1, 2, 3, 4, 5],
    )

    assert summary["bus_voltage_v"] == pytest.approx(26.0)
    assert summary["max_abs_leg_current_a"] == pytest.approx(1.5)


def test_relay_ack_requires_digital_signal_and_power() -> None:
    base = {
        "received": True,
        "digital": True,
        "signal": True,
        "power": True,
    }
    assert relay_is_exactly_on(base)
    for field in ("digital", "signal", "power"):
        assert not relay_is_exactly_on({**base, field: False})
    assert not relay_is_exactly_on({**base, "received": False})


@pytest.mark.parametrize("mode", ["hold", "test"])
def test_motion_requires_all_hard_guards_before_ros_initialization(
    monkeypatch, mode
) -> None:
    initialized = False

    def unexpected_init():
        nonlocal initialized
        initialized = True
        raise AssertionError("motion guard must run before rclpy.init")

    monkeypatch.setattr(biorola_servo_probe.rclpy, "init", unexpected_init)

    with pytest.raises(SystemExit) as exc_info:
        biorola_servo_probe.main([mode, "sl2", "--confirm-motion"])

    assert "requires --require-power-state and --require-power-on" in str(
        exc_info.value
    )
    assert initialized is False


@pytest.mark.parametrize(
    "escape_flag",
    [
        "--allow-no-subscriber",
        "--allow-existing-command-publishers",
        "--allow-zero-servo-readback",
    ],
)
def test_motion_rejects_safety_escape_flags_before_ros_initialization(
    monkeypatch, escape_flag
) -> None:
    monkeypatch.setattr(
        biorola_servo_probe.rclpy,
        "init",
        lambda: (_ for _ in ()).throw(
            AssertionError("escape guard must run before rclpy.init")
        ),
    )

    with pytest.raises(SystemExit) as exc_info:
        biorola_servo_probe.main(
            [
                "test",
                "sl2",
                "--confirm-motion",
                "--require-power-state",
                "--require-power-on",
                escape_flag,
            ]
        )

    assert "forbids safety escape flags" in str(exc_info.value)


def test_interrupted_servo_probe_exits_nonzero_and_reports_unknown(
    monkeypatch,
) -> None:
    class InterruptedProbe:
        def __init__(self, _args):
            pass

        def run(self):
            raise KeyboardInterrupt

        def destroy_node(self):
            return None

    monkeypatch.setattr(biorola_servo_probe, "BioRoLaServoProbe", InterruptedProbe)
    monkeypatch.setattr(biorola_servo_probe.rclpy, "init", lambda: None)
    monkeypatch.setattr(biorola_servo_probe.rclpy, "ok", lambda: True)
    monkeypatch.setattr(biorola_servo_probe.rclpy, "shutdown", lambda: None)

    with pytest.raises(SystemExit) as exc_info:
        biorola_servo_probe.main(["status"])

    assert exc_info.value.code != 0
    assert "output state is UNKNOWN" in str(exc_info.value)


def _guarded_motion_args(*extra: str) -> list[str]:
    return [
        "test",
        "sl2",
        "--confirm-motion",
        "--require-power-state",
        "--require-power-on",
        *extra,
    ]


@pytest.mark.parametrize(
    ("extra", "expected"),
    [
        (("--max-current-a", "3.0001"), "max-current-a <= 3"),
        (("--min-voltage-v", "17.999"), "min-voltage-v >= 18"),
        (("--max-voltage-v", "30.001"), "max-voltage-v <= 30"),
        (("--max-abs-delta", "201"), "max-abs-delta <= 200"),
        (("--max-abs-delta", "0"), "max-abs-delta <= 200"),
        (
            ("--max-abs-delta", "200", "--delta", "201"),
            "--delta exceeds",
        ),
        (("--servo-control-mode", "0"), "servo-control-mode=2"),
    ],
)
def test_motion_hard_bounds_cannot_be_loosened_before_ros_initialization(
    monkeypatch, extra, expected
) -> None:
    initialized = False

    def unexpected_init():
        nonlocal initialized
        initialized = True
        raise AssertionError("hard bounds must run before rclpy.init")

    monkeypatch.setattr(biorola_servo_probe.rclpy, "init", unexpected_init)

    with pytest.raises(SystemExit) as exc_info:
        biorola_servo_probe.main(_guarded_motion_args(*extra))

    assert expected in str(exc_info.value)
    assert initialized is False


def test_motion_hard_boundaries_are_accepted(monkeypatch) -> None:
    class ReachedRosInit(RuntimeError):
        pass

    monkeypatch.setattr(
        biorola_servo_probe.rclpy,
        "init",
        lambda: (_ for _ in ()).throw(ReachedRosInit()),
    )

    with pytest.raises(ReachedRosInit):
        biorola_servo_probe.main(
            _guarded_motion_args(
                "--max-current-a",
                "3",
                "--min-voltage-v",
                "18",
                "--max-voltage-v",
                "30",
                "--max-abs-delta",
                "200",
                "--delta",
                "200",
                "--servo-control-mode",
                "2",
            )
        )


@pytest.mark.parametrize(
    "extra",
    [
        ("--command-topic", "/fake/command"),
        ("--state-topic", "/fake/state"),
        ("--power-state-topic", "/fake/power"),
        (
            "--power-bus-voltage-channel",
            "6",
            "--leg-current-channels",
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
        ),
        (
            "--leg-current-channels",
            "6",
            "5",
            "4",
            "3",
            "2",
            "1",
        ),
    ],
)
def test_motion_rejects_topic_or_power_mapping_bypass_before_ros(
    monkeypatch, extra
) -> None:
    monkeypatch.setattr(
        biorola_servo_probe.rclpy,
        "init",
        lambda: (_ for _ in ()).throw(
            AssertionError("topic/mapping contract must fail before ROS")
        ),
    )

    with pytest.raises(SystemExit) as exc_info:
        biorola_servo_probe.main(_guarded_motion_args(*extra))

    assert "fixed" in str(exc_info.value)


def test_status_remains_read_only_with_custom_topics_and_mapping(monkeypatch) -> None:
    class ReachedRosInit(RuntimeError):
        pass

    monkeypatch.setattr(
        biorola_servo_probe.rclpy,
        "init",
        lambda: (_ for _ in ()).throw(ReachedRosInit()),
    )

    with pytest.raises(ReachedRosInit):
        biorola_servo_probe.main(
            [
                "status",
                "sl1",
                "--state-topic",
                "/inspection/state",
                "--power-state-topic",
                "/inspection/power",
                "--power-bus-voltage-channel",
                "6",
                "--leg-current-channels",
                "0",
                "1",
                "2",
                "3",
                "4",
                "5",
            ]
        )


def test_motion_rejects_resolved_ros_remap_before_command_publisher() -> None:
    def resolve(topic: str) -> str:
        if topic == biorola_servo_probe.MOTION_POWER_STATE_TOPIC:
            return "/fake/power"
        return topic

    with pytest.raises(RuntimeError) as exc_info:
        validate_resolved_motion_topics(resolve)

    assert "forbids ROS topic remapping" in str(exc_info.value)
    assert "/power/state->/fake/power" in str(exc_info.value)


def test_motion_accepts_unremapped_resolved_topics() -> None:
    validate_resolved_motion_topics(lambda topic: topic)


def test_motion_constructor_rejects_ros_remap_before_publisher(monkeypatch) -> None:
    args = biorola_servo_probe.build_parser().parse_args(_guarded_motion_args())
    publisher_calls = 0
    original_create_publisher = biorola_servo_probe.Node.create_publisher

    def guarded_publisher(self, msg_type, topic, qos_profile, *extra, **kwargs):
        nonlocal publisher_calls
        if topic == biorola_servo_probe.MOTION_COMMAND_TOPIC:
            publisher_calls += 1
            raise AssertionError(
                "remap gate must run before creating the command publisher"
            )
        return original_create_publisher(
            self, msg_type, topic, qos_profile, *extra, **kwargs
        )

    monkeypatch.setattr(
        biorola_servo_probe.BioRoLaServoProbe,
        "create_publisher",
        guarded_publisher,
    )
    if biorola_servo_probe.rclpy.ok():
        biorola_servo_probe.rclpy.shutdown()
    biorola_servo_probe.rclpy.init(
        args=["--ros-args", "-r", "/power/state:=/fake/power"]
    )
    try:
        with pytest.raises(RuntimeError) as exc_info:
            biorola_servo_probe.BioRoLaServoProbe(args)
        assert "forbids ROS topic remapping" in str(exc_info.value)
        assert publisher_calls == 0
    finally:
        if biorola_servo_probe.rclpy.ok():
            biorola_servo_probe.rclpy.shutdown()


def test_feedback_header_requires_fresh_monotonic_sequence_and_stamp() -> None:
    now_ns = 10_000_000_000
    header = SimpleNamespace(
        seq=7,
        stamp=SimpleNamespace(sec=9, nanosec=950_000_000),
    )
    key, error = validate_feedback_header(header, now_ns, None, 0.10)
    assert error is None
    assert key == (7, 9_950_000_000)

    _, error = validate_feedback_header(header, now_ns, key, 0.10)
    assert "duplicate" in error

    stale = SimpleNamespace(
        seq=8,
        stamp=SimpleNamespace(sec=9, nanosec=800_000_000),
    )
    _, error = validate_feedback_header(stale, now_ns, key, 0.10)
    assert "age bound" in error


@pytest.mark.parametrize(
    "publishers",
    [
        [],
        [SimpleNamespace(node_name="fake_bridge", node_namespace="/")],
        [
            SimpleNamespace(
                node_name=biorola_servo_probe.EXPECTED_BRIDGE_NODE_NAME,
                node_namespace="/",
            ),
            SimpleNamespace(node_name="duplicate", node_namespace="/"),
        ],
    ],
)
def test_motion_feedback_requires_unique_expected_bridge_publisher(publishers) -> None:
    probe = object.__new__(biorola_servo_probe.BioRoLaServoProbe)
    probe.get_publishers_info_by_topic = lambda _topic: publishers

    with pytest.raises(RuntimeError):
        probe.assert_expected_feedback_publisher("/motor/state")
