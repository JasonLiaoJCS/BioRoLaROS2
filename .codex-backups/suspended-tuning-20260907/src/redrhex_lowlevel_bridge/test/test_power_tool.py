from contextlib import contextmanager
import fcntl
import json

from types import SimpleNamespace
from types import MethodType

import pytest

from redrhex_lowlevel_bridge import rinbo_power_tool
from redrhex_lowlevel_bridge.rinbo_power_tool import (
    LEG_CURRENT_CHANNELS,
    POWER_BUS_CHANNEL,
    UNKNOWN_GRAPH_ENDPOINT_NAME,
    _classify_graph_endpoint_names,
    _healthy_leg_current_channels,
    _power_feedback_header_key,
    _power_channels,
    _power_state_matches,
    _relay_feedback_violation,
    _validate_resolved_power_topics,
)


@pytest.mark.parametrize(
    ("names", "expected_status"),
    [
        ([], "pending"),
        ([UNKNOWN_GRAPH_ENDPOINT_NAME], "pending"),
        (["/rinbo_ros2_bridge"], "match"),
        (["/wrong_bridge"], "reject"),
        (["/_NODE_NAME_UNKNOWN_"], "reject"),
        (["/rinbo_ros2_bridge", UNKNOWN_GRAPH_ENDPOINT_NAME], "reject"),
    ],
)
def test_graph_convergence_retries_only_empty_or_exact_unknown_placeholder(
    names, expected_status
) -> None:
    assert (
        _classify_graph_endpoint_names(names, "/rinbo_ros2_bridge")
        == expected_status
    )


def _graph_endpoint(node_namespace: str, node_name: str):
    return SimpleNamespace(
        node_namespace=node_namespace,
        node_name=node_name,
    )


_EXPECTED_COMMAND_ENDPOINT = _graph_endpoint(
    "/", "redrhex_rinbo_power_tool"
)
_EXPECTED_STATE_ENDPOINT = _graph_endpoint("/", "rinbo_ros2_bridge")
_UNKNOWN_GRAPH_ENDPOINT = _graph_endpoint(
    "_NODE_NAMESPACE_UNKNOWN_", "_NODE_NAME_UNKNOWN_"
)
_WRONG_GRAPH_ENDPOINT = _graph_endpoint("/", "wrong_node")


class _GraphWaitFake:
    def __init__(self, snapshots) -> None:
        self.args = SimpleNamespace(
            topic="/power/command",
            state_topic="/power/state",
        )
        self.snapshots = snapshots
        self.snapshot_index = 0

    def get_fully_qualified_name(self) -> str:
        return "/redrhex_rinbo_power_tool"

    def get_publishers_info_by_topic(self, topic: str):
        return self.snapshots[self.snapshot_index][topic]

    def _endpoint_names(self, infos) -> list[str]:
        return rinbo_power_tool.RinboPowerTool._endpoint_names(infos)


def test_enable_graph_wait_converges_from_pending_to_exact(monkeypatch) -> None:
    fake = _GraphWaitFake(
        [
            {
                "/power/command": [],
                "/power/state": [_UNKNOWN_GRAPH_ENDPOINT],
            },
            {
                "/power/command": [_UNKNOWN_GRAPH_ENDPOINT],
                "/power/state": [],
            },
            {
                "/power/command": [_EXPECTED_COMMAND_ENDPOINT],
                "/power/state": [_EXPECTED_STATE_ENDPOINT],
            },
        ]
    )
    spin_timeouts = []

    def advance_snapshot(node, *, timeout_sec: float) -> None:
        spin_timeouts.append(timeout_sec)
        node.snapshot_index += 1

    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "spin_once", advance_snapshot)

    rinbo_power_tool.RinboPowerTool.wait_for_expected_enable_graph(fake)

    assert spin_timeouts == [0.05, 0.05]
    assert fake.snapshot_index == 2


@pytest.mark.parametrize(
    ("bad_topic", "bad_infos", "match"),
    [
        ("/power/command", [_WRONG_GRAPH_ENDPOINT], "power enable"),
        (
            "/power/command",
            [_EXPECTED_COMMAND_ENDPOINT, _UNKNOWN_GRAPH_ENDPOINT],
            "power enable",
        ),
        ("/power/state", [_WRONG_GRAPH_ENDPOINT], "power acknowledgement"),
        (
            "/power/state",
            [_EXPECTED_STATE_ENDPOINT, _UNKNOWN_GRAPH_ENDPOINT],
            "power acknowledgement",
        ),
    ],
)
def test_enable_graph_wait_rejects_known_wrong_or_multiple_without_retry(
    monkeypatch, bad_topic, bad_infos, match
) -> None:
    snapshot = {
        "/power/command": [_EXPECTED_COMMAND_ENDPOINT],
        "/power/state": [_EXPECTED_STATE_ENDPOINT],
    }
    snapshot[bad_topic] = bad_infos
    fake = _GraphWaitFake([snapshot])

    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        rinbo_power_tool.rclpy,
        "spin_once",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("known wrong or multiple endpoints must not retry")
        ),
    )

    with pytest.raises(RuntimeError, match=match):
        rinbo_power_tool.RinboPowerTool.wait_for_expected_enable_graph(fake)

    assert fake.snapshot_index == 0


def test_enable_graph_wait_rejects_pending_at_deadline(monkeypatch) -> None:
    fake = _GraphWaitFake(
        [
            {
                "/power/command": [],
                "/power/state": [_UNKNOWN_GRAPH_ENDPOINT],
            }
        ]
    )
    monotonic_values = iter([10.0, 10.25, 11.0])
    spin_calls = []

    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        rinbo_power_tool.time,
        "monotonic",
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(
        rinbo_power_tool.rclpy,
        "spin_once",
        lambda *_args, **_kwargs: spin_calls.append(True),
    )

    with pytest.raises(RuntimeError, match="did not converge within 1.00s"):
        rinbo_power_tool.RinboPowerTool.wait_for_expected_enable_graph(fake)

    assert spin_calls == [True]


def test_enable_graph_wait_rechecks_both_topics_after_pending(monkeypatch) -> None:
    fake = _GraphWaitFake(
        [
            {
                "/power/command": [_EXPECTED_COMMAND_ENDPOINT],
                "/power/state": [_UNKNOWN_GRAPH_ENDPOINT],
            },
            {
                "/power/command": [
                    _EXPECTED_COMMAND_ENDPOINT,
                    _WRONG_GRAPH_ENDPOINT,
                ],
                "/power/state": [_EXPECTED_STATE_ENDPOINT],
            },
        ]
    )
    spin_calls = []

    def advance_snapshot(node, *, timeout_sec: float) -> None:
        spin_calls.append(timeout_sec)
        node.snapshot_index += 1

    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "spin_once", advance_snapshot)

    with pytest.raises(RuntimeError, match="power enable"):
        rinbo_power_tool.RinboPowerTool.wait_for_expected_enable_graph(fake)

    assert spin_calls == [0.05]
    assert fake.snapshot_index == 1


def _state(**overrides):
    values = {
        "digital": True,
        "signal": True,
        "power": False,
    }
    for index in range(8):
        values[f"v_{index}"] = float(index)
        values[f"i_{index}"] = float(index) / 10.0
    values.update(overrides)
    return SimpleNamespace(**values)


def _header_state(sequence=10, stamp_ns=10_000_000_000, **overrides):
    state = _state(**overrides)
    state.header = SimpleNamespace(
        seq=sequence,
        stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        ),
    )
    return state


def test_power_state_ack_requires_exact_relay_tuple() -> None:
    msg = _state()
    assert _power_state_matches(msg, True, True, False)
    assert not _power_state_matches(msg, True, True, True)
    assert not _power_state_matches(msg, True, False, False)


def test_power_channel_contract_separates_bus_from_leg_channels() -> None:
    channels = _power_channels(_state())
    assert POWER_BUS_CHANNEL == 7
    assert LEG_CURRENT_CHANNELS == (1, 2, 3, 4, 5, 6)
    assert channels[POWER_BUS_CHANNEL]["voltage_v"] == 7.0
    assert max(abs(channels[index]["current_a"]) for index in LEG_CURRENT_CHANNELS) == 0.6


def test_relay_power_guard_keeps_healthy_legs_and_excludes_only_declared_leg() -> None:
    state = _state(
        power=True,
        v_7=24.0,
        i_1=50.0,
        i_2=2.9,
        i_3=0.0,
        i_4=0.0,
        i_5=0.0,
        i_6=0.0,
    )

    assert _healthy_leg_current_channels(("L1",)) == (2, 3, 4, 5, 6)
    assert _relay_feedback_violation(state, ("L1",)) is None
    assert "current" in str(_relay_feedback_violation(state, ()))
    assert "current" in str(_relay_feedback_violation(state, ("L2",)))


def test_relay_requires_confirmation_before_ros_initialization(monkeypatch) -> None:
    def unexpected_init(*_args, **_kwargs):
        raise AssertionError("relay confirmation gate must run before rclpy.init")

    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", unexpected_init)

    with pytest.raises(SystemExit, match="without --confirm-relay"):
        rinbo_power_tool.main(["relay"])


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({"v_7": 17.99}, "below"),
        ({"v_7": 30.01}, "above"),
        ({"v_7": float("nan")}, "non-finite"),
        ({"v_7": 24.0, "i_2": 3.01}, "current"),
        ({"v_7": 24.0, "i_2": 3.0}, "current"),
        ({"v_7": 24.0, "i_2": -3.0}, "current"),
    ],
)
def test_relay_power_guard_hard_limits(overrides, expected) -> None:
    violation = _relay_feedback_violation(_state(power=True, **overrides), ("L1",))

    assert expected in str(violation)


@pytest.mark.parametrize(
    "extra",
    [
        ("--topic", "/fake/power"),
        ("--state-topic", "/fake/state"),
        ("--allow-no-subscriber",),
        ("--clean",),
        ("--trigger",),
        ("--repeat", "2"),
        ("--repeat", "9"),
        ("--repeat-delay-s", "0"),
        ("--verify-timeout-s", "4"),
    ],
)
def test_power_write_bypass_options_fail_before_ros_initialization(
    monkeypatch, extra
) -> None:
    monkeypatch.setattr(
        rinbo_power_tool.rclpy,
        "init",
        lambda: (_ for _ in ()).throw(
            AssertionError("power hard gate must run before rclpy.init")
        ),
    )

    with pytest.raises(SystemExit):
        rinbo_power_tool.main(["relay", "--confirm-relay", *extra])


def test_power_write_rejects_resolved_ros_remap() -> None:
    def resolve(topic: str) -> str:
        if topic == "/power/state":
            return "/fake/power"
        return topic

    with pytest.raises(RuntimeError, match="forbid ROS topic remapping"):
        _validate_resolved_power_topics(resolve)


def test_power_write_accepts_unremapped_resolved_topics() -> None:
    _validate_resolved_power_topics(lambda topic: topic)


def test_power_feedback_header_requires_fresh_monotonic_sequence_and_stamp() -> None:
    first = _power_feedback_header_key(
        _header_state(sequence=10, stamp_ns=10_000_000_000),
        now_ns=10_100_000_000,
        previous=None,
    )
    assert first == (10, 10_000_000_000)
    second = _power_feedback_header_key(
        _header_state(sequence=11, stamp_ns=10_050_000_000),
        now_ns=10_100_000_000,
        previous=first,
    )
    assert second == (11, 10_050_000_000)


@pytest.mark.parametrize(
    ("message", "now_ns", "previous", "match"),
    [
        (_header_state(sequence=0), 10_100_000_000, None, "positive"),
        (_header_state(stamp_ns=1), 10_100_000_000, None, "age"),
        (
            _header_state(sequence=10, stamp_ns=10_000_000_000),
            10_100_000_000,
            (10, 10_000_000_000),
            "duplicate/out-of-order",
        ),
    ],
)
def test_power_feedback_header_rejects_replay_or_stale_data(
    message, now_ns, previous, match
) -> None:
    with pytest.raises(RuntimeError, match=match):
        _power_feedback_header_key(message, now_ns=now_ns, previous=previous)


def test_power_ack_reauthenticates_graph_and_header_before_success(
    monkeypatch, capsys
) -> None:
    graph_checks = []
    state = _header_state(
        sequence=10,
        stamp_ns=10_000_000_000,
        digital=True,
        signal=True,
        power=False,
    )
    fake = SimpleNamespace(
        args=SimpleNamespace(
            verify_timeout_s=0.5,
            disabled_legs=("L1",),
            state_topic="/power/state",
        ),
        last_power_state=state,
        last_power_state_received_monotonic=2.0,
        assert_expected_enable_graph=lambda: graph_checks.append(True),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)
        ),
        summarize_power_state=lambda _msg: {"power": False},
    )
    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)

    rinbo_power_tool.RinboPowerTool.wait_for_ack(
        fake,
        digital=True,
        signal=True,
        power=False,
        command_started_monotonic=1.0,
    )

    assert len(graph_checks) == 2
    assert '"power": false' in capsys.readouterr().out


def test_power_cleanup_failure_is_bounded_and_reports_unknown(monkeypatch) -> None:
    ack_requests = []
    published = []
    fake = SimpleNamespace(
        args=SimpleNamespace(
            dry_run=False,
            repeat=3,
            repeat_delay_s=0.05,
        ),
        last_power_state=_state(power=True),
        wait_for_subscriber=lambda: None,
        wait_for_expected_enable_graph=lambda: None,
        assert_expected_enable_graph=lambda: None,
        build_msg=lambda digital, signal, power: SimpleNamespace(
            digital=digital, signal=signal, power=power
        ),
        pub=SimpleNamespace(publish=lambda msg: published.append(msg)),
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
    )

    def fail_ack(**request):
        ack_requests.append(request)
        raise RuntimeError("no trusted acknowledgement")

    fake.wait_for_ack = fail_ack
    fake.publish_state = MethodType(
        rinbo_power_tool.RinboPowerTool.publish_state, fake
    )
    monkeypatch.setattr(rinbo_power_tool.rclpy, "spin_once", lambda *_a, **_k: None)
    monkeypatch.setattr(rinbo_power_tool.time, "sleep", lambda _seconds: None)

    with pytest.raises(RuntimeError, match="Relay state is UNKNOWN"):
        fake.publish_state(True, True, True)

    assert len(ack_requests) == 2
    assert len(published) == 6
    assert all(not msg.power for msg in published[3:])


def test_interrupted_power_command_exits_nonzero_and_keeps_state_unknown(monkeypatch, pinned_configuration) -> None:
    instances = []

    class InterruptedNode:
        def __init__(self, args):
            self.args = args
            self.off_attempts = 0
            instances.append(self)

        def run(self):
            raise KeyboardInterrupt

        def publish_unverified_emergency_off(self):
            self.off_attempts += 1
            return True

        def destroy_node(self):
            return None

    monkeypatch.setattr(rinbo_power_tool, "RinboPowerTool", InterruptedNode)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "shutdown", lambda: None)

    with pytest.raises(SystemExit) as exc_info:
        rinbo_power_tool.main(["relay", "--confirm-relay"])

    assert exc_info.value.code != 0
    assert "relay state is UNKNOWN" in str(exc_info.value)
    assert "unverified best effort" in str(exc_info.value)
    assert instances[0].off_attempts == 1


def test_interrupted_status_does_not_publish_power_command(monkeypatch, pinned_configuration) -> None:
    instances = []

    class InterruptedNode:
        def __init__(self, args):
            self.args = args
            self.off_attempts = 0
            instances.append(self)

        def run(self):
            raise KeyboardInterrupt

        def publish_unverified_emergency_off(self):
            self.off_attempts += 1
            return True

        def destroy_node(self):
            return None

    monkeypatch.setattr(rinbo_power_tool, "RinboPowerTool", InterruptedNode)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "shutdown", lambda: None)

    with pytest.raises(SystemExit) as exc_info:
        rinbo_power_tool.main(["status"])

    assert exc_info.value.code != 0
    assert "relay state is UNKNOWN" in str(exc_info.value)
    assert instances[0].off_attempts == 0


def _configuration_data(disabled=("L1", "L3")):
    return {
        "path": rinbo_power_tool.ORIN_LEGS_CONFIG_PATH,
        "schema_version": 1,
        "revision": 42,
        "hash": "snapshot-hash",
        "disabled_legs": list(disabled),
        "enabled_legs": [name for name in rinbo_power_tool.LEG_CURRENT_CHANNEL_BY_NAME if name not in disabled],
    }


@pytest.fixture
def pinned_configuration(monkeypatch):
    configuration = rinbo_power_tool.LegConfiguration(
        **{**_configuration_data(), "disabled_legs": ("L1", "L3"), "enabled_legs": ("L2", "R1", "R2", "R3")}
    )

    @contextmanager
    def pin():
        yield configuration

    monkeypatch.setattr(rinbo_power_tool, "_pinned_leg_configuration", pin)
    return configuration


@pytest.mark.parametrize("mask", range(64))
def test_every_mask_excludes_only_declared_currents_and_retains_bus_guard(mask):
    names = tuple(rinbo_power_tool.LEG_CURRENT_CHANNEL_BY_NAME)
    disabled = tuple(name for index, name in enumerate(names) if mask & (1 << index))
    enabled_channels = tuple(index + 1 for index in range(6) if not mask & (1 << index))
    assert _healthy_leg_current_channels(disabled) == enabled_channels
    state = _state(power=True, v_7=24.0)
    for index in range(6):
        if mask & (1 << index):
            setattr(state, f"i_{index + 1}", float("nan"))
    assert _relay_feedback_violation(state, disabled) is None
    for channel in enabled_channels:
        setattr(state, f"i_{channel}", 3.01)
        assert "current" in _relay_feedback_violation(state, disabled)
        setattr(state, f"i_{channel}", 0.0)
    state.v_7 = 17.99
    assert "below" in _relay_feedback_violation(state, disabled)
    state.v_7 = float("nan")
    assert "non-finite" in _relay_feedback_violation(state, disabled)


def test_reenabled_leg_immediately_rejoins_current_gate():
    state = _state(power=True, v_7=24.0, i_1=80.0, i_3=80.0)
    assert _relay_feedback_violation(state, ("L1", "L3")) is None
    assert "current" in _relay_feedback_violation(state, ("L1",))


def test_configuration_reader_uses_fixed_ros_free_command(monkeypatch):
    requests = []

    def run(command, **kwargs):
        requests.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=json.dumps(_configuration_data()), stderr="")

    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", run)
    configuration = rinbo_power_tool._read_leg_configuration()
    assert configuration.disabled_legs == ("L1", "L3")
    assert configuration.revision == 42
    assert requests == [
        (["/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs", "status", "--json"],
         {"check": False, "capture_output": True, "text": True, "timeout": 5.0, "cwd": "/"})
    ]


@pytest.mark.parametrize("overrides", [
    {"path": "/tmp/untrusted.yaml"},
    {"schema_version": 2},
    {"schema_version": True},
    {"revision": 0},
    {"revision": True},
    {"hash": ""},
    {"disabled_legs": ["L1", "L1"]},
    {"disabled_legs": ["L1", "X3"]},
    {"disabled_legs": "L1"},
    {"disabled_legs": [1]},
    {"enabled_legs": ["L1", "L2", "R1", "R2", "R3"]},
    {"enabled_legs": ["L2", "R1", "R2"]},
])
def test_configuration_reader_rejects_inconsistent_loader_envelope(monkeypatch, overrides):
    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(
        returncode=0, stdout=json.dumps({**_configuration_data(), **overrides}), stderr=""
    ))
    with pytest.raises(RuntimeError, match="Invalid rinbo_legs status response"):
        rinbo_power_tool._read_leg_configuration()


@pytest.mark.parametrize("failure", [
    FileNotFoundError("rinbo_legs executable missing"),
    PermissionError("rinbo_legs unreadable"),
    rinbo_power_tool.subprocess.TimeoutExpired("rinbo_legs", 5.0),
])
def test_loader_execution_errors_are_fatal_before_ros(monkeypatch, tmp_path, failure):
    monkeypatch.setattr(rinbo_power_tool, "ORIN_LEGS_LOCK_PATH", str(tmp_path / "config.lock"))

    def fail(*_args, **_kwargs):
        raise failure

    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", fail)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: pytest.fail("loader error must not initialize ROS"))
    with pytest.raises(SystemExit, match=r"\[FATAL\] Cannot load Orin leg configuration"):
        rinbo_power_tool.main(["digital"])


@pytest.mark.parametrize("response", ["not JSON", "null", "[]", "{}"])
def test_malformed_loader_response_is_fatal_before_ros(monkeypatch, tmp_path, response):
    monkeypatch.setattr(rinbo_power_tool, "ORIN_LEGS_LOCK_PATH", str(tmp_path / "config.lock"))
    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(
        returncode=0, stdout=response, stderr=""
    ))
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: pytest.fail("loader error must not initialize ROS"))
    with pytest.raises(SystemExit, match=r"\[FATAL\] Invalid rinbo_legs status response"):
        rinbo_power_tool.main(["digital"])


@pytest.mark.parametrize("arguments", [
    ["digital"], ["sensors"], ["relay", "--confirm-relay"],
    ["sequence"], ["status"], ["relay", "--dry-run"],
])
@pytest.mark.parametrize("failure", ["missing", "corrupt", "unreadable", "version"])
def test_invalid_configuration_prevents_all_ros_initialization(monkeypatch, tmp_path, arguments, failure):
    monkeypatch.setattr(rinbo_power_tool, "ORIN_LEGS_LOCK_PATH", str(tmp_path / "config.lock"))
    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(
        returncode=1, stdout="", stderr=f"configuration {failure}"
    ))
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: pytest.fail("invalid config must not initialize ROS"))
    with pytest.raises(SystemExit, match=f"configuration {failure}"):
        rinbo_power_tool.main(arguments)


@pytest.mark.parametrize("arguments", [["off"], ["off", "--dry-run"]])
def test_all_off_does_not_depend_on_configuration(monkeypatch, arguments):
    monkeypatch.setattr(rinbo_power_tool, "_pinned_leg_configuration", lambda: pytest.fail("off must not need config or locks"))
    calls = []
    monkeypatch.setattr(rinbo_power_tool, "_run_ros", lambda args: calls.append(args))
    rinbo_power_tool.main(arguments)
    if "--dry-run" not in arguments:
        assert calls[0].disabled_legs == ()
        assert calls[0].leg_configuration is None


def test_legacy_disabled_leg_cannot_override_orin_mask(monkeypatch):
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: pytest.fail("legacy override must fail before ROS"))
    with pytest.raises(SystemExit, match="--disabled-leg is retired"):
        rinbo_power_tool.main(["relay", "--confirm-relay", "--disabled-leg", "L2"])


def test_snapshot_lock_blocks_manager_until_ros_cleanup(monkeypatch, tmp_path):
    lock_path = tmp_path / "config.lock"
    monkeypatch.setattr(rinbo_power_tool, "ORIN_LEGS_LOCK_PATH", str(lock_path))
    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(
        returncode=0, stdout=json.dumps(_configuration_data()), stderr=""
    ))
    checkpoints = []

    def assert_pinned(stage):
        with lock_path.open("r+") as manager:
            with pytest.raises(BlockingIOError):
                fcntl.flock(manager, fcntl.LOCK_EX | fcntl.LOCK_NB)
        checkpoints.append(stage)

    class FakeNode:
        def __init__(self, args):
            assert_pinned("node")
            assert args.disabled_legs == ("L1", "L3")
            self.args = args

        def run(self):
            assert_pinned("run")

        def destroy_node(self):
            assert_pinned("destroy")

    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: assert_pinned("init"))
    monkeypatch.setattr(rinbo_power_tool, "RinboPowerTool", FakeNode)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(rinbo_power_tool.rclpy, "shutdown", lambda: assert_pinned("shutdown"))
    rinbo_power_tool.main(["digital"])
    assert checkpoints == ["init", "node", "run", "destroy", "shutdown"]
    with lock_path.open("r+") as manager:
        fcntl.flock(manager, fcntl.LOCK_EX | fcntl.LOCK_NB)


def test_manager_write_lock_prevents_read_or_ros_start(monkeypatch, tmp_path):
    lock_path = tmp_path / "config.lock"
    monkeypatch.setattr(rinbo_power_tool, "ORIN_LEGS_LOCK_PATH", str(lock_path))
    monkeypatch.setattr(rinbo_power_tool, "_read_leg_configuration", lambda: pytest.fail("no read until lock acquired"))
    monkeypatch.setattr(rinbo_power_tool.rclpy, "init", lambda **_kwargs: pytest.fail("no ROS until lock acquired"))
    with lock_path.open("w+") as manager:
        fcntl.flock(manager, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(SystemExit, match="Cannot pin Orin leg configuration"):
            rinbo_power_tool.main(["digital"])


def test_loader_failure_releases_shared_lock(monkeypatch, tmp_path):
    lock_path = tmp_path / "config.lock"
    monkeypatch.setattr(rinbo_power_tool, "ORIN_LEGS_LOCK_PATH", str(lock_path))
    monkeypatch.setattr(rinbo_power_tool.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(
        returncode=1, stdout="", stderr="invalid configuration"
    ))
    with pytest.raises(RuntimeError, match="invalid configuration"):
        with rinbo_power_tool._pinned_leg_configuration():
            pytest.fail("invalid configuration must not yield a snapshot")
    with lock_path.open("r+") as manager:
        fcntl.flock(manager, fcntl.LOCK_EX | fcntl.LOCK_NB)
