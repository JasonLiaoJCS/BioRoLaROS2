from __future__ import annotations

import math
import itertools
from types import SimpleNamespace
import time

import pytest
from builtin_interfaces.msg import Time

from redrhex_lowlevel_bridge.rinbo_ros_backend import RinboRosBackend


_telemetry_sequence = itertools.count(1)


class _Stamp:
    sec = 0
    nanosec = 0


class _ClockNow:
    def to_msg(self):
        return Time()


class _Clock:
    def now(self):
        return _ClockNow()


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _Node:
    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()

    def get_publishers_info_by_topic(self, _topic):
        return [SimpleNamespace(node_name="bridge", node_namespace="/")]


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return 1


class _HeaderCapturePublisher(_Publisher):
    def __init__(self):
        super().__init__()
        self.sequences = []

    def publish(self, msg):
        self.sequences.append(int(msg.header.seq))
        super().publish(msg)


class _Header:
    def __init__(self):
        self.seq = 0
        self.stamp = _Stamp()
        self.frame_id = ""


class _LegCmd:
    def __init__(self):
        self.enable = False
        self.direction = False
        self.voltage = 0.0
        self.state = 0
        self.reset_position = False


class _ServoCmd:
    def __init__(self):
        self.position_encoder = 0


class _MotorCmdStamped:
    def __init__(self):
        self.header = _Header()
        self.l1 = _LegCmd()
        self.l2 = _LegCmd()
        self.l3 = _LegCmd()
        self.r1 = _LegCmd()
        self.r2 = _LegCmd()
        self.r3 = _LegCmd()
        self.sl1 = _ServoCmd()
        self.sl2 = _ServoCmd()
        self.sl3 = _ServoCmd()
        self.sr1 = _ServoCmd()
        self.sr2 = _ServoCmd()
        self.sr3 = _ServoCmd()
        self.servo_control_mode = 0


def _backend(**overrides) -> RinboRosBackend:
    kwargs = {
        "node": _Node(),
        "command_topic": "/motor/command",
        "state_topic": "/motor/state",
        "power_state_topic": "/power/state",
        "joint_state_topic": "/joint_states",
        "preview_topic": "/preview",
        "publish_preview": True,
        "allow_enable": False,
        "publish_when_disabled": False,
        "disabled_servo_control_mode": 0,
        "publish_shutdown_disable": False,
        "shutdown_disable_repeats": 0,
        "shutdown_disable_period_s": 0.0,
        "require_state": True,
        "block_if_duplicate_command_publishers": True,
        "state_timeout_s": 0.25,
        "main_position_counts_per_rev": 1000.0,
        "main_pwm_per_rad_s": 10.0,
        "main_max_pwm": 80.0,
        "main_pwm_slew_rate_per_s": 1000.0,
        "main_encoder_zero_counts_rinbo_order": [0.0] * 6,
        "main_encoder_sign_rinbo_order": [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
        "main_velocity_sign_policy_order": [1.0] * 6,
        "main_direction_positive_rinbo_order": [True, True, True, False, False, False],
        "main_velocity_filter_alpha": 0.5,
        "main_velocity_max_dt_s": 0.2,
        "main_velocity_clip_rad_s": 80.0,
        "abad_encoder_zero_rinbo_order": [100, 200, 300, 400, 500, 600],
        "abad_encoder_counts_per_rad": 1000.0,
        "abad_encoder_min": 0,
        "abad_encoder_max": 65535,
        "abad_sign_rinbo_order": [1.0] * 6,
        "servo_control_mode": 2,
        "require_power_state": True,
        "power_state_timeout_s": 0.25,
        "power_bus_voltage_channel": 7,
        "min_bus_voltage": 18.0,
        "max_current_a": 3.0,
        "current_trip_samples": 1,
        "main_joint_names_policy_order": [f"joint_{i}" for i in range(6)],
        "disabled_legs": [],
        "max_disabled_legs": 1,
        "leg_current_channels_rinbo_order": [1, 2, 3, 4, 5, 6],
        "max_bus_voltage": 30.0,
        "max_bus_current_a": 30.0,
        "stop_on_bus_current_limit": False,
        "voltage_trip_samples": 3,
    }
    kwargs.update(overrides)
    backend = RinboRosBackend(**kwargs)
    backend.MotorCmdStamped = _MotorCmdStamped
    return backend


def _command(enable: bool = True) -> SimpleNamespace:
    return SimpleNamespace(
        enable=enable,
        target_velocity_rad_s=[1.0, -2.0, 3.0, -4.0, 5.0, -6.0] + [0.0] * 6,
        target_position_rad=[0.0] * 6 + [0.1, 0.2, 0.3, -0.1, -0.2, -0.3],
    )


def test_shutdown_disable_packets_keep_monotonic_command_sequence() -> None:
    backend = _backend(
        publish_shutdown_disable=True,
        shutdown_disable_repeats=3,
        shutdown_disable_period_s=0.0,
    )
    backend.sequence = 41
    backend.cmd_pub = _HeaderCapturePublisher()

    backend._publish_shutdown_disable()

    assert backend.cmd_pub.sequences == [42, 43, 44]
    assert backend.sequence == 44


def test_shutdown_completes_primary_train_before_disconnect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend = _backend(
        publish_shutdown_disable=True,
        shutdown_disable_repeats=3,
        shutdown_disable_period_s=0.0,
    )
    backend.connected = True
    backend.cmd_pub = _HeaderCapturePublisher()
    monkeypatch.setattr(
        backend,
        "_publish_shutdown_disable_with_fresh_context",
        lambda: pytest.fail("fallback must not run after a complete primary train"),
    )

    backend.shutdown()

    assert backend.cmd_pub.sequences == [1, 2, 3]
    assert backend.shutdown_disable_status == "primary context published 3/3 packets"
    assert not backend.connected


def test_shutdown_uses_fresh_context_after_primary_rcl_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend = _backend(
        publish_shutdown_disable=True,
        shutdown_disable_repeats=3,
        shutdown_disable_period_s=0.0,
    )
    backend.connected = True
    fallback_calls: list[bool] = []

    def _primary_failure() -> int:
        raise RuntimeError("rcl context is invalid")

    def _fallback() -> int:
        fallback_calls.append(True)
        return 3

    monkeypatch.setattr(backend, "_publish_shutdown_disable", _primary_failure)
    monkeypatch.setattr(
        backend, "_publish_shutdown_disable_with_fresh_context", _fallback
    )

    backend.shutdown()

    assert fallback_calls == [True]
    assert backend.shutdown_disable_status == (
        "fresh-context fallback published 3/3 packets"
    )
    assert not backend.connected


def test_startup_disabled_handshake_is_bounded_when_regular_disabled_publish_is_off() -> None:
    backend = _backend(
        publish_when_disabled=False,
        disabled_handshake_repeats=3,
    )
    backend.connected = True
    backend.cmd_pub = _HeaderCapturePublisher()
    backend.preview_pub = _Publisher()

    for _ in range(5):
        backend.send_motor_command(_command(enable=False))

    assert backend.cmd_pub.sequences == [1, 2, 3]
    assert backend._disabled_handshake_packets_remaining == 0
    assert backend.last_actual_publish_state == "preview_only_disabled"


def test_preview_packet_is_non_actionable_even_for_enabled_upstream_command() -> None:
    backend = _backend(allow_enable=False, publish_preview=True)
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()

    backend.send_motor_command(_command(enable=True))

    assert len(backend.preview_pub.messages) == 1
    preview = backend.preview_pub.messages[0]
    assert preview.servo_control_mode == 0
    for name in backend.RINBO_LEG_ORDER:
        leg = getattr(preview, name)
        assert not leg.enable
        assert leg.voltage == 0.0
        assert leg.state == 0
    assert backend.cmd_pub.messages == []


def test_preview_topic_cannot_equal_physical_command_topic() -> None:
    with pytest.raises(ValueError, match="preview_topic"):
        _backend(
            command_topic="/motor/command",
            preview_topic="/motor/command/",
        )


def test_enabled_to_disabled_transition_restarts_bounded_handshake() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        publish_when_disabled=False,
        disabled_handshake_repeats=3,
    )
    backend.connected = True
    backend.cmd_pub = _HeaderCapturePublisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())
    for _ in range(3):
        backend.send_motor_command(_command(enable=False))
    backend.send_motor_command(_command(enable=True))

    for _ in range(4):
        backend.send_motor_command(_command(enable=False))

    assert backend.cmd_pub.sequences == [1, 2, 3, 4, 5, 6, 7]
    assert backend._disabled_handshake_packets_remaining == 0


def test_actual_output_requires_fresh_final_arbiter_ack() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        require_downstream_output_ack=True,
        downstream_output_ack_timeout_s=0.10,
        disabled_handshake_repeats=0,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())

    backend.send_motor_command(_command(enable=True))
    assert not backend.actual_output_enabled()

    backend._on_downstream_output_ack(SimpleNamespace(data=True))
    assert backend.actual_output_enabled()

    backend._on_downstream_output_ack(SimpleNamespace(data=False))
    assert not backend.actual_output_enabled()

    backend._on_downstream_output_ack(SimpleNamespace(data=True))
    backend.last_downstream_output_ack_time = time.monotonic() - 0.11
    assert not backend.actual_output_enabled()


def test_missing_final_arbiter_ack_latches_and_actively_disables() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        require_downstream_output_ack=True,
        downstream_output_ack_timeout_s=0.10,
        disabled_handshake_repeats=0,
        shutdown_disable_repeats=2,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    backend.downstream_enable_epoch_time = time.monotonic() - 0.11

    backend.safety_watchdog()

    assert backend.safety_latched
    assert "arbiter" in backend.safety_latch_reason.lower()
    assert not backend.last_command_was_enabled
    assert all(
        not getattr(backend.cmd_pub.messages[-1], name).enable
        for name in backend.RINBO_LEG_ORDER
    )


def _exact_command(enable: bool = True) -> SimpleNamespace:
    main_names = [f"joint_{index}" for index in range(6)]
    abad_names = [f"abad_{index}" for index in range(6)]
    return SimpleNamespace(
        enable=enable,
        mode=1 if enable else 0,
        joint_names=main_names + abad_names,
        target_velocity_rad_s=[0.5] * 6 + [0.0] * 6,
        target_position_rad=[0.0] * 6 + [0.1] * 6,
        kp=[0.0] * 12,
        kd=[0.0] * 12,
        effort_limit_nm=[0.0] * 12,
    )


def _power_state(
    bus_voltage: float = 24.0,
    current: float = 0.5,
    current_channel: int = 4,
    bus_current: float = 0.0,
    power_relay: bool = True,
    **field_overrides: float,
) -> SimpleNamespace:
    fields = {f"v_{idx}": 24.0 for idx in range(8)}
    fields.update({f"i_{idx}": 0.0 for idx in range(8)})
    fields["v_7"] = bus_voltage
    fields[f"i_{current_channel}"] = current
    fields["i_7"] = bus_current
    fields["power"] = power_relay
    fields["header"] = SimpleNamespace(
        seq=next(_telemetry_sequence), stamp=_Stamp()
    )
    fields.update(field_overrides)
    return SimpleNamespace(**fields)


def _motor_state(
    servo_position_overrides: dict[str, int] | None = None,
    **position_overrides: float,
) -> SimpleNamespace:
    positions = {
        "l1": 10.0,
        "l2": 20.0,
        "l3": 30.0,
        "r1": 40.0,
        "r2": 50.0,
        "r3": 60.0,
    }
    positions.update(position_overrides)
    msg = SimpleNamespace()
    for name, position in positions.items():
        setattr(msg, name, SimpleNamespace(position=position))
    msg.header = SimpleNamespace(seq=next(_telemetry_sequence), stamp=_Stamp())
    servo_positions = {
        name: 100 + index
        for index, name in enumerate(("sl1", "sl2", "sl3", "sr1", "sr2", "sr3"))
    }
    servo_positions.update(servo_position_overrides or {})
    for name, position in servo_positions.items():
        setattr(msg, name, SimpleNamespace(position_encoder=position))
    return msg


def test_rinbo_backend_maps_policy_order_to_rinbo_order() -> None:
    backend = _backend()
    msg = backend._make_motor_cmd_msg(_command(enable=True), enabled=True, preview=False)

    assert msg.r1.enable
    assert msg.r1.voltage == pytest.approx(10.0)
    assert msg.r2.voltage == pytest.approx(20.0)
    assert msg.r3.voltage == pytest.approx(30.0)
    assert msg.l1.voltage == pytest.approx(40.0)
    assert msg.l2.voltage == pytest.approx(50.0)
    assert msg.l3.voltage == pytest.approx(60.0)
    assert msg.servo_control_mode == 2
    assert [msg.sl1.position_encoder, msg.sl2.position_encoder, msg.sl3.position_encoder] == [0, 0, 0]
    assert [msg.sr1.position_encoder, msg.sr2.position_encoder, msg.sr3.position_encoder] == [500, 700, 900]


@pytest.mark.parametrize(
    "per_servo_scales",
    [None, []],
    ids=["parameter-omitted", "empty-array"],
)
def test_rinbo_backend_abad_scale_array_defaults_to_legacy_scalar(
    per_servo_scales: list[float] | None,
) -> None:
    backend = _backend(
        abad_encoder_counts_per_rad=1234.5,
        abad_encoder_counts_per_rad_rinbo_order=per_servo_scales,
    )

    assert backend.abad_encoder_counts_per_rad == pytest.approx(1234.5)
    assert backend.abad_encoder_counts_per_rad_rinbo_order == pytest.approx(
        [1234.5] * 6
    )
    diagnostics = backend.diagnostic_values()
    assert diagnostics["rinbo_abad_encoder_scale_source"] == "legacy-scalar"
    assert (
        diagnostics[
            "rinbo_abad_encoder_counts_per_rad_sl1_sl2_sl3_sr1_sr2_sr3"
        ]
        == "1234.5,1234.5,1234.5,1234.5,1234.5,1234.5"
    )


def test_rinbo_backend_per_servo_abad_scales_round_trip_policy_mapping() -> None:
    scales = [100.0, 200.0, 300.0, 400.0, 500.0, 600.0]
    signs = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
    abad_joint_names = [f"abad_joint_{index}" for index in range(6)]
    backend = _backend(
        abad_encoder_counts_per_rad_rinbo_order=scales,
        abad_sign_rinbo_order=signs,
        publish_abad_joint_feedback=True,
        abad_joint_names_policy_order=abad_joint_names,
    )
    backend.joint_pub = _Publisher()
    command = _command(enable=True)

    outgoing = backend._make_motor_cmd_msg(
        command, enabled=True, preview=False
    )
    outgoing_servo_positions = [
        getattr(outgoing, name).position_encoder
        for name in backend.RINBO_SERVO_ORDER
    ]
    assert outgoing_servo_positions == [90, 240, 210, 360, 600, 420]

    backend._on_rinbo_state(
        _motor_state(
            servo_position_overrides=dict(
                zip(backend.RINBO_SERVO_ORDER, outgoing_servo_positions)
            )
        )
    )

    assert backend.latest_abad_positions_policy == pytest.approx(
        command.target_position_rad[6:12]
    )
    assert list(backend.joint_pub.messages[-1].position[6:]) == pytest.approx(
        command.target_position_rad[6:12]
    )
    diagnostics = backend.diagnostic_values()
    assert diagnostics["rinbo_abad_encoder_scale_source"] == "per-servo"
    assert (
        diagnostics[
            "rinbo_abad_encoder_counts_per_rad_sl1_sl2_sl3_sr1_sr2_sr3"
        ]
        == "100,200,300,400,500,600"
    )


@pytest.mark.parametrize(
    "per_servo_scales",
    [
        [100.0] * 5,
        [100.0] * 7,
        [100.0, 100.0, 100.0, 100.0, 100.0, float("nan")],
        [100.0, 100.0, 100.0, 100.0, 100.0, float("inf")],
        [100.0, 100.0, 100.0, 100.0, 100.0, 0.0],
        [100.0, 100.0, 100.0, 100.0, 100.0, -1.0],
    ],
    ids=["short", "long", "nan", "inf", "zero", "negative"],
)
def test_rinbo_backend_rejects_invalid_per_servo_abad_scales(
    per_servo_scales: list[float],
) -> None:
    with pytest.raises(
        ValueError,
        match="abad_encoder_counts_per_rad_rinbo_order",
    ):
        _backend(
            abad_encoder_counts_per_rad_rinbo_order=per_servo_scales
        )


@pytest.mark.parametrize("disabled_legs", [None, []], ids=["unset", "empty"])
def test_rinbo_backend_without_disabled_legs_keeps_every_leg_enabled(
    disabled_legs: list[str] | None,
) -> None:
    backend = _backend(disabled_legs=disabled_legs)
    msg = backend._make_motor_cmd_msg(_command(enable=True), enabled=True, preview=False)

    assert backend.disabled_legs == []
    assert backend.disabled_leg_fields == set()
    assert all(getattr(msg, name).enable for name in backend.RINBO_LEG_ORDER)
    assert all(getattr(msg, name).voltage > 0.0 for name in backend.RINBO_LEG_ORDER)


def test_rinbo_backend_disabled_command_is_neutral() -> None:
    backend = _backend()
    msg = backend._make_motor_cmd_msg(_command(enable=False), enabled=False, preview=False)

    assert not any(getattr(msg, name).enable for name in backend.RINBO_LEG_ORDER)
    assert msg.servo_control_mode == 0
    assert [getattr(msg, name).position_encoder for name in backend.RINBO_SERVO_ORDER] == [100, 200, 300, 400, 500, 600]


@pytest.mark.parametrize(
    ("physical_leg", "policy_index", "rinbo_index", "main_field", "servo_field"),
    [
        ("L1", 3, 0, "l1", "sl1"),
        ("L2", 4, 1, "l2", "sl2"),
        ("L3", 5, 2, "l3", "sl3"),
        ("R1", 0, 3, "r1", "sr1"),
        ("R2", 1, 4, "r2", "sr2"),
        ("R3", 2, 5, "r3", "sr3"),
    ],
)
def test_rinbo_backend_masks_every_disabled_leg_at_final_main_output(
    physical_leg: str,
    policy_index: int,
    rinbo_index: int,
    main_field: str,
    servo_field: str,
) -> None:
    backend = _backend(disabled_legs=[f" {physical_leg.lower()} "])
    backend.latest_servo_positions_rinbo[rinbo_index] = 777
    backend._slewed_pwm_rinbo_order[rinbo_index] = 50.0

    msg = backend._make_motor_cmd_msg(
        _command(enable=True),
        enabled=True,
        preview=False,
        apply_slew=True,
    )

    mapping = next(
        item for item in backend.POLICY_TO_RINBO_LEGS if item.rinbo_field == main_field
    )
    assert mapping.policy_index == policy_index
    assert backend.disabled_legs == [physical_leg]
    assert backend.disabled_leg_fields == {main_field}
    disabled_main = getattr(msg, main_field)
    assert not disabled_main.enable
    assert disabled_main.state == 0
    assert not disabled_main.direction
    assert disabled_main.voltage == 0.0
    assert backend.last_raw_pwm_rinbo_order[rinbo_index] == 0.0
    assert backend.last_pwm_rinbo_order[rinbo_index] == 0.0
    assert backend._slewed_pwm_rinbo_order[rinbo_index] == 0.0
    # The corresponding physically isolated servo receives its bounded neutral.
    assert (
        getattr(msg, servo_field).position_encoder
        == backend.abad_encoder_zero_rinbo_order[rinbo_index]
    )
    for healthy_field in backend.RINBO_LEG_ORDER:
        if healthy_field == main_field:
            continue
        assert getattr(msg, healthy_field).enable
        assert getattr(msg, healthy_field).voltage > 0.0


def test_rinbo_backend_main_drive_calibration_gate_blocks_all_healthy_outputs() -> None:
    backend = _backend(
        allow_enable=True,
        require_state=False,
        require_power_state=False,
        require_power_relay=False,
        block_if_duplicate_command_publishers=False,
        require_main_drive_calibration=True,
        main_drive_calibrated=False,
        disabled_legs=["L1"],
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()

    backend.send_motor_command(_command(enable=True))

    assert not backend.last_command_was_enabled
    assert backend.last_actual_publish_state == "blocked_main_drive_uncalibrated"
    assert "L2,L3,R1,R2,R3" in backend.last_block_reason
    assert backend.cmd_pub.messages == []

    backend.main_drive_calibrated = True
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled
    assert not backend.cmd_pub.messages[-1].l1.enable
    assert all(
        getattr(backend.cmd_pub.messages[-1], name).enable
        for name in ("l2", "l3", "r1", "r2", "r3")
    )


def test_rinbo_backend_optionally_appends_abad_feedback_in_policy_order(
    monkeypatch,
) -> None:
    state_time = [10.0]
    monkeypatch.setattr(
        "redrhex_lowlevel_bridge.rinbo_ros_backend.time.monotonic",
        lambda: state_time[0],
    )
    abad_joint_names = [f"abad_joint_{index}" for index in range(6)]
    backend = _backend(
        publish_abad_joint_feedback=True,
        abad_joint_names_policy_order=abad_joint_names,
        abad_sign_rinbo_order=[1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
    )
    backend.joint_pub = _Publisher()

    first_servo_positions = {
        "sl1": 1100,
        "sl2": 2200,
        "sl3": 3300,
        "sr1": 4400,
        "sr2": 5500,
        "sr3": 6600,
    }
    backend._on_rinbo_state(
        _motor_state(servo_position_overrides=first_servo_positions)
    )
    first = backend.joint_pub.messages[-1]

    assert list(first.name) == backend.main_joint_names_policy_order + abad_joint_names
    assert list(first.position[6:]) == pytest.approx([-4.0, 5.0, -6.0, 1.0, -2.0, 3.0])
    assert len(first.velocity) == 12
    assert all(math.isfinite(value) for value in first.velocity[6:])

    state_time[0] += 0.1
    second_servo_positions = {
        "sl1": 1110,
        "sl2": 2220,
        "sl3": 3330,
        "sr1": 4440,
        "sr2": 5550,
        "sr3": 6660,
    }
    backend._on_rinbo_state(
        _motor_state(servo_position_overrides=second_servo_positions)
    )
    second = backend.joint_pub.messages[-1]

    assert list(second.name) == backend.main_joint_names_policy_order + abad_joint_names
    assert list(second.position[6:]) == pytest.approx(
        [-4.04, 5.05, -6.06, 1.01, -2.02, 3.03]
    )
    assert len(second.velocity) == 12
    assert all(math.isfinite(value) for value in second.velocity[6:])


def test_rinbo_backend_abad_feedback_off_preserves_six_joint_state_layout() -> None:
    abad_joint_names = [f"abad_joint_{index}" for index in range(6)]
    backend = _backend(
        publish_abad_joint_feedback=False,
        # The real node always passes the contract names; the boolean remains
        # the ABI gate deciding whether they are appended to /joint_states.
        abad_joint_names_policy_order=abad_joint_names,
    )
    backend.joint_pub = _Publisher()

    backend._on_rinbo_state(
        _motor_state(
            servo_position_overrides={
                "sl1": 1100,
                "sl2": 2200,
                "sl3": 3300,
                "sr1": 4400,
                "sr2": 5500,
                "sr3": 6600,
            }
        )
    )
    joint_state = backend.joint_pub.messages[-1]

    assert list(joint_state.name) == backend.main_joint_names_policy_order
    assert len(joint_state.position) == 6
    assert len(joint_state.velocity) == 6


@pytest.mark.parametrize(
    "abad_joint_names",
    [
        [],
        ["abad"] * 6,
        ["joint_0", "abad_1", "abad_2", "abad_3", "abad_4", "abad_5"],
    ],
    ids=["missing", "duplicate", "overlaps-main"],
)
def test_rinbo_backend_abad_feedback_requires_six_unique_nonoverlapping_names(
    abad_joint_names: list[str],
) -> None:
    with pytest.raises(ValueError):
        _backend(
            publish_abad_joint_feedback=True,
            abad_joint_names_policy_order=abad_joint_names,
        )


@pytest.mark.parametrize(
    ("disabled_legs", "message"),
    [
        (["X1"], "Unknown hardware.disabled_legs"),
        (["L1", "l1"], "Duplicate hardware.disabled_legs"),
    ],
)
def test_rinbo_backend_rejects_invalid_or_duplicate_disabled_leg_names(
    disabled_legs: list[str], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        _backend(disabled_legs=disabled_legs, max_disabled_legs=2)


def test_rinbo_backend_rejects_bad_sign_config() -> None:
    with pytest.raises(ValueError):
        _backend(main_velocity_sign_policy_order=[1.0, 1.0, 1.0, 0.0, 1.0, 1.0])


def test_rinbo_backend_power_safety_blocks_enabled_output() -> None:
    backend = _backend(current_trip_samples=2, voltage_trip_samples=3)
    ok, reason = backend._power_ok_for_enable()
    assert not ok
    assert "waiting for" in reason

    for sample in range(3):
        backend._on_power_state(_power_state(bus_voltage=12.0, current=0.5))
        ok, reason = backend._power_ok_for_enable()
        assert not ok
        assert "bus voltage low" in reason
        assert backend._power_fault_requires_latch() is (sample == 2)

    backend._on_power_state(_power_state(bus_voltage=24.0, current=4.5))
    ok, reason = backend._power_ok_for_enable()
    assert not ok
    assert "current high" in reason
    assert not backend._power_fault_requires_latch()

    backend._on_power_state(_power_state(bus_voltage=24.0, current=4.5))
    ok, reason = backend._power_ok_for_enable()
    assert not ok
    assert "current high" in reason
    assert backend._power_fault_requires_latch()

    backend._on_power_state(_power_state(bus_voltage=24.0, current=0.5))
    ok, reason = backend._power_ok_for_enable()
    assert ok, reason


def test_rinbo_backend_power_guard_uses_bus_ch7_and_leg_current_ch1_through_ch6() -> None:
    backend = _backend(
        current_trip_samples=1,
        voltage_trip_samples=1,
        max_current_a=3.0,
        max_bus_current_a=30.0,
        stop_on_bus_current_limit=False,
    )

    # A disconnected L1 voltage channel and a 10 A aggregate bus current must
    # not be confused with the dedicated ch7 bus-voltage or per-leg guards.
    backend._on_power_state(
        _power_state(bus_voltage=24.0, current=2.0, current_channel=1, bus_current=10.0, v_1=0.0)
    )
    ok, reason = backend._power_ok_for_enable()
    assert ok, reason
    assert backend._max_leg_current() == pytest.approx(2.0)

    # A predeclared, physically isolated leg's per-leg channel is excluded;
    # healthy channels and the optional global bus-current guard remain active.
    disabled_l1 = _backend(
        disabled_legs=["L1"],
        current_trip_samples=1,
        voltage_trip_samples=1,
        max_current_a=3.0,
    )
    disabled_l1._on_power_state(
        _power_state(bus_voltage=24.0, current=4.5, current_channel=1, bus_current=4.5, v_1=0.0)
    )
    ok, reason = disabled_l1._power_ok_for_enable()
    assert ok, reason
    assert disabled_l1._max_leg_current() == pytest.approx(0.0)

    disabled_l1._on_power_state(
        _power_state(bus_voltage=24.0, current=4.5, current_channel=2)
    )
    ok, reason = disabled_l1._power_ok_for_enable()
    assert not ok
    assert "current high" in reason


def test_motor_current_feedback_honors_configured_channels_and_disabled_leg() -> None:
    backend = _backend(
        disabled_legs=["L1"],
        leg_current_channels_rinbo_order=[6, 5, 4, 3, 2, 1],
    )
    backend._on_power_state(
        _power_state(i_1=0.1, i_2=0.2, i_3=0.3, i_4=0.4, i_5=0.5, i_6=0.6)
    )

    # Policy order R1,R2,R3,L1,L2,L3; L1 is explicitly neutralized.
    assert backend._motor_currents_policy_order() == pytest.approx(
        [0.3, 0.2, 0.1, 0.0, 0.5, 0.4]
    )


def test_rinbo_backend_bus_current_has_an_independent_debounced_guard() -> None:
    backend = _backend(
        current_trip_samples=2,
        voltage_trip_samples=1,
        max_current_a=3.0,
        max_bus_current_a=8.0,
        stop_on_bus_current_limit=True,
    )

    backend._on_power_state(_power_state(current=2.0, bus_current=10.0))
    ok, reason = backend._power_ok_for_enable()
    assert not ok
    assert "bus current high" in reason
    assert not backend._power_fault_requires_latch()

    backend._on_power_state(_power_state(current=2.0, bus_current=10.0))
    ok, reason = backend._power_ok_for_enable()
    assert not ok
    assert "bus current high" in reason
    assert backend._power_fault_requires_latch()


def test_rinbo_backend_requires_power_relay_and_ignores_unused_nan_channel() -> None:
    backend = _backend(voltage_trip_samples=1, current_trip_samples=1)
    backend._on_power_state(_power_state(power_relay=False))
    ok, reason = backend._power_ok_for_enable()
    assert not ok
    assert "relay" in reason

    backend._on_power_state(_power_state(power_relay=True, v_0=float("nan"), i_0=float("nan")))
    ok, reason = backend._power_ok_for_enable()
    assert ok, reason


def test_rinbo_backend_latches_persistent_power_fault_until_disabled_handshake() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        current_trip_samples=2,
        voltage_trip_samples=2,
        shutdown_disable_repeats=2,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled

    # The first bad sample immediately disables output, while the second
    # distinct sample plus another enabled request makes the fault persistent
    # and latched.
    backend._on_power_state(_power_state(bus_voltage=12.0))
    assert not backend.last_command_was_enabled
    assert not backend.safety_latched
    backend._on_power_state(_power_state(bus_voltage=12.0))
    backend.send_motor_command(_command(enable=True))
    assert backend.safety_latched
    assert not backend.last_command_was_enabled
    assert not backend.is_alive()
    assert all(not getattr(backend.cmd_pub.messages[-1], name).enable for name in backend.RINBO_LEG_ORDER)

    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    assert backend.safety_latched
    assert not backend.last_command_was_enabled

    backend.send_motor_command(_command(enable=False))
    assert backend.safety_latched
    for _ in range(4):
        backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=False))
    assert not backend.safety_latched
    assert backend.is_alive()
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize(
    ("field", "index"),
    [
        ("target_velocity_rad_s", 0),
        ("target_velocity_rad_s", 11),
        ("target_position_rad", 0),
        ("target_position_rad", 11),
    ],
)
def test_rinbo_backend_nonfinite_enabled_command_latches_and_actively_disables(
    field: str, index: int, bad_value: float
) -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        shutdown_disable_repeats=2,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled

    invalid = _command(enable=True)
    getattr(invalid, field)[index] = bad_value
    backend.send_motor_command(invalid)

    assert backend.safety_latched
    assert "non-finite" in backend.safety_latch_reason.lower()
    assert not backend.last_command_was_enabled
    assert backend.last_pwm_rinbo_order == [0.0] * 6
    stop = backend.cmd_pub.messages[-1]
    assert stop.servo_control_mode == backend.disabled_servo_control_mode
    for name in backend.RINBO_LEG_ORDER:
        leg = getattr(stop, name)
        assert not leg.enable
        assert leg.state == 0
        assert leg.voltage == 0.0


@pytest.mark.parametrize(
    ("field", "index"),
    [
        ("target_velocity_rad_s", 0),
        ("target_position_rad", 6),
    ],
)
def test_rinbo_backend_finite_command_with_nonfinite_conversion_latches_and_disables(
    field: str, index: int
) -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        shutdown_disable_repeats=2,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled

    overflow = _command(enable=True)
    getattr(overflow, field)[index] = 1.0e308
    backend.send_motor_command(overflow)

    assert backend.safety_latched
    assert not backend.last_command_was_enabled
    assert backend.last_pwm_rinbo_order == [0.0] * 6
    stop = backend.cmd_pub.messages[-1]
    assert stop.servo_control_mode == backend.disabled_servo_control_mode
    for name in backend.RINBO_LEG_ORDER:
        leg = getattr(stop, name)
        assert not leg.enable
        assert leg.state == 0
        assert leg.voltage == 0.0


def test_rinbo_backend_safety_latch_restarts_post_fault_power_recovery_count() -> None:
    backend = _backend(
        block_if_duplicate_command_publishers=False,
        recovery_healthy_samples=3,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()

    for _ in range(3):
        backend._on_power_state(_power_state())
    assert backend.power_healthy_count == backend.recovery_healthy_samples

    backend._latch_safety("synthetic non-power safety fault")
    assert backend.safety_latched
    assert backend.power_healthy_count == 0

    # A disabled command is the explicit handshake, but it must not reuse
    # healthy samples observed before the fault or clear early.
    backend.send_motor_command(_command(enable=False))
    assert backend.safety_latched
    for expected_count in (1, 2):
        backend._on_power_state(_power_state())
        assert backend.power_healthy_count == expected_count
        backend.send_motor_command(_command(enable=False))
        assert backend.safety_latched

    backend._on_power_state(_power_state())
    assert backend.power_healthy_count == backend.recovery_healthy_samples
    backend.send_motor_command(_command(enable=False))
    assert not backend.safety_latched


def test_rinbo_backend_power_recovery_count_restarts_after_telemetry_gap() -> None:
    backend = _backend(
        block_if_duplicate_command_publishers=False,
        power_state_timeout_s=0.25,
        recovery_healthy_samples=4,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._latch_safety("synthetic safety fault")

    backend._on_power_state(_power_state())
    backend._on_power_state(_power_state())
    assert backend.power_healthy_count == 2

    # Recovery must be consecutive. A telemetry gap makes the two samples
    # above stale, so this first new healthy sample starts a fresh dwell.
    backend.last_power_state_time = (
        time.monotonic() - backend.power_state_timeout_s - 0.1
    )
    backend._on_power_state(_power_state())
    assert backend.power_healthy_count == 1
    backend.send_motor_command(_command(enable=False))
    assert backend.safety_latched

    for expected_count in (2, 3):
        backend._on_power_state(_power_state())
        assert backend.power_healthy_count == expected_count
    backend.send_motor_command(_command(enable=False))
    assert backend.safety_latched

    backend._on_power_state(_power_state())
    assert backend.power_healthy_count == backend.recovery_healthy_samples
    backend.send_motor_command(_command(enable=False))
    assert not backend.safety_latched


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_rinbo_backend_nonfinite_state_faults_healthy_leg_but_sanitizes_disabled_l1(
    bad_value: float,
) -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        shutdown_disable_repeats=2,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.joint_pub = _Publisher()
    backend._on_rinbo_state(_motor_state())
    valid_state_time = backend.last_state_time
    valid_policy_positions = list(backend.latest_positions_policy)
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled

    backend._on_rinbo_state(_motor_state(r1=bad_value))

    assert backend.safety_latched
    assert not backend.last_command_was_enabled
    assert backend.last_state_time == valid_state_time
    assert backend.latest_positions_policy == valid_policy_positions
    assert all(not getattr(backend.cmd_pub.messages[-1], name).enable for name in backend.RINBO_LEG_ORDER)
    assert all(getattr(backend.cmd_pub.messages[-1], name).voltage == 0.0 for name in backend.RINBO_LEG_ORDER)

    degraded = _backend(disabled_legs=["L1"])
    degraded.connected = True
    degraded.joint_pub = _Publisher()
    degraded._on_rinbo_state(_motor_state())
    previous_time = degraded.last_state_time
    previous_l1_policy_position = degraded.latest_positions_policy[3]

    degraded._on_rinbo_state(_motor_state(l1=bad_value))

    assert not degraded.safety_latched
    assert degraded.last_state_time > previous_time
    assert degraded.latest_positions_policy[3] == previous_l1_policy_position
    assert degraded.latest_velocities_policy[3] == 0.0
    assert all(math.isfinite(value) for value in degraded.latest_positions_policy)
    assert all(math.isfinite(value) for value in degraded.latest_velocities_policy)
    assert all(math.isfinite(value) for value in degraded.joint_pub.messages[-1].position)
    assert all(math.isfinite(value) for value in degraded.joint_pub.messages[-1].velocity)


def test_disabled_leg_finite_encoder_garbage_is_replaced_with_bounded_neutral() -> None:
    backend = _backend(disabled_legs=["L1"])
    backend.connected = True
    backend.joint_pub = _Publisher()

    backend._on_rinbo_state(
        _motor_state(l1=123456.0, servo_position_overrides={"sl1": 999999})
    )

    assert not backend.safety_latched
    assert backend.latest_raw_positions_rinbo[0] == pytest.approx(0.0)
    assert backend.latest_positions_policy[3] == pytest.approx(0.0)
    assert backend.latest_velocities_policy[3] == pytest.approx(0.0)
    assert backend.latest_servo_positions_rinbo[0] == 100
    assert backend.latest_abad_positions_policy[3] == pytest.approx(0.0)


def test_out_of_range_servo_readback_on_healthy_leg_latches() -> None:
    backend = _backend(disabled_legs=["L1"])
    backend.connected = True
    backend.joint_pub = _Publisher()

    backend._on_rinbo_state(
        _motor_state(servo_position_overrides={"sr1": 999999})
    )

    assert backend.safety_latched
    assert backend.last_state_time is None
    assert "R1" in backend.safety_latch_reason


def test_rinbo_backend_finite_encoder_inputs_with_nonfinite_derived_position_latch_and_disable(
) -> None:
    encoder_zeros = [0.0] * 6
    encoder_zeros[3] = -1.0e308  # R1 in Rinbo order.
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        shutdown_disable_repeats=2,
        main_encoder_zero_counts_rinbo_order=encoder_zeros,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.joint_pub = _Publisher()
    backend._on_rinbo_state(_motor_state(r1=-1.0e308))
    valid_state_time = backend.last_state_time
    valid_raw_positions = list(backend.latest_raw_positions_rinbo)
    valid_policy_positions = list(backend.latest_positions_policy)
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled

    state = _motor_state(r1=1.0e308)
    assert math.isfinite(state.r1.position)
    assert math.isfinite(encoder_zeros[3])
    backend._on_rinbo_state(state)

    assert backend.safety_latched
    assert not backend.last_command_was_enabled
    assert backend.last_state_time == valid_state_time
    assert backend.latest_raw_positions_rinbo == valid_raw_positions
    assert backend.latest_positions_policy == valid_policy_positions
    stop = backend.cmd_pub.messages[-1]
    for name in backend.RINBO_LEG_ORDER:
        leg = getattr(stop, name)
        assert not leg.enable
        assert leg.state == 0
        assert leg.voltage == 0.0


def test_rinbo_backend_rejects_relay_requirement_without_power_state() -> None:
    with pytest.raises(ValueError):
        _backend(require_power_state=False, require_power_relay=True)


def test_rinbo_backend_exact_command_contract_and_profile_caps_are_final() -> None:
    backend = _backend(
        require_exact_command_contract=True,
        abad_joint_names_policy_order=[f"abad_{index}" for index in range(6)],
        max_main_target_velocity_rad_s=1.0,
        max_abad_target_position_rad=0.18,
    )
    backend._validate_command(_exact_command())

    wrong_order = _exact_command()
    wrong_order.joint_names[0], wrong_order.joint_names[1] = (
        wrong_order.joint_names[1],
        wrong_order.joint_names[0],
    )
    with pytest.raises(ValueError, match="joint_names must exactly match"):
        backend._validate_command(wrong_order)

    fast = _exact_command()
    fast.target_velocity_rad_s[0] = 1.001
    with pytest.raises(ValueError, match="main target velocity"):
        backend._validate_command(fast)

    wide_abad = _exact_command()
    wide_abad.target_position_rad[6] = -0.181
    with pytest.raises(ValueError, match="ABAD target position"):
        backend._validate_command(wide_abad)


def test_rinbo_backend_duplicate_telemetry_cannot_refresh_or_clear_safety() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        shutdown_disable_repeats=2,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.joint_pub = _Publisher()
    state = _motor_state()
    power = _power_state()
    backend._on_rinbo_state(state)
    backend._on_power_state(power)
    accepted_state_time = backend.last_state_time
    accepted_power_time = backend.last_power_state_time
    backend.send_motor_command(_command(enable=True))
    assert backend.last_command_was_enabled

    backend._on_rinbo_state(state)
    assert backend.safety_latched
    assert not backend.last_command_was_enabled
    assert backend.last_state_time == accepted_state_time

    backend._on_power_state(power)
    assert backend.last_power_state_time == accepted_power_time
    # Entering the safety latch discards every pre-fault recovery sample.  A
    # replayed power packet must not rebuild that counter.
    assert backend.power_healthy_count == 0


def test_rinbo_backend_uint32_telemetry_sequence_wrap_is_accepted() -> None:
    backend = _backend()
    backend.connected = True
    backend.joint_pub = _Publisher()
    first = _motor_state()
    first.header.seq = 0xFFFFFFFF
    backend._on_rinbo_state(first)
    wrapped = _motor_state()
    wrapped.header.seq = 0
    backend._on_rinbo_state(wrapped)
    assert backend.last_rinbo_state_sequence == 0


@pytest.mark.parametrize(
    "field",
    ["command_timeout_s", "power_state_timeout_s", "main_pwm_per_rad_s"],
)
@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_rinbo_backend_rejects_nonfinite_safety_float_configuration(
    field: str, bad_value: float
) -> None:
    with pytest.raises(ValueError):
        _backend(**{field: bad_value})


@pytest.mark.parametrize(
    "field",
    [
        "servo_control_mode",
        "disabled_servo_control_mode",
        "abad_encoder_min",
        "abad_encoder_max",
    ],
)
@pytest.mark.parametrize("bad_value", [-1, 1 << 32])
def test_rinbo_backend_rejects_uint32_configuration_out_of_range(
    field: str, bad_value: int
) -> None:
    with pytest.raises(ValueError):
        _backend(**{field: bad_value})


def test_rinbo_backend_rejects_finite_counts_per_rev_with_nonfinite_scale() -> None:
    with pytest.raises(ValueError):
        _backend(main_position_counts_per_rev=1.0e-320)


def test_rinbo_backend_rejects_duplicate_current_channels() -> None:
    with pytest.raises(ValueError, match="must be unique"):
        _backend(leg_current_channels_rinbo_order=[1, 1, 3, 4, 5, 6])


def test_rinbo_backend_rejects_main_pwm_override_above_hardware_cap() -> None:
    with pytest.raises(ValueError, match="main_max_pwm"):
        _backend(main_max_pwm=80.01)


def test_rinbo_backend_watchdog_actively_disables_stale_enabled_stream() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        command_timeout_s=0.05,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())
    backend.send_motor_command(_command(enable=True))
    backend.last_enabled_command_time = time.monotonic() - 0.2

    backend.safety_watchdog()

    assert backend.safety_latched
    assert "command stream stale" in backend.safety_latch_reason
    assert not backend.last_command_was_enabled
    stop = backend.cmd_pub.messages[-1]
    assert all(not getattr(stop, name).enable for name in backend.RINBO_LEG_ORDER)


def test_rinbo_backend_blocks_enable_until_abad_command_calibration_ack() -> None:
    backend = _backend(
        allow_enable=True,
        block_if_duplicate_command_publishers=False,
        require_abad_command_calibration=True,
        abad_command_calibrated=False,
    )
    backend.connected = True
    backend.cmd_pub = _Publisher()
    backend.preview_pub = _Publisher()
    backend.last_state_time = time.monotonic()
    backend._on_power_state(_power_state())

    backend.send_motor_command(_command(enable=True))

    assert not backend.last_command_was_enabled
    assert "ABAD command calibration" in backend.last_block_reason


def test_rinbo_backend_requires_expected_live_telemetry_source_and_stamp() -> None:
    backend = _backend(
        require_single_telemetry_publisher=True,
        expected_telemetry_publisher_node="bridge",
        require_monotonic_telemetry_stamp=True,
    )
    backend.connected = True
    backend.joint_pub = _Publisher()
    backend.safety_watchdog()
    assert backend.telemetry_source_ok[backend.state_topic]

    first = _motor_state()
    first.header.stamp.sec = 1
    backend._on_rinbo_state(first)
    accepted_time = backend.last_state_time
    assert accepted_time is not None

    replay = _motor_state()
    replay.header.stamp.sec = 1
    backend._on_rinbo_state(replay)
    assert backend.last_state_time == accepted_time
    assert "source stamp" in backend.last_block_reason


def test_rinbo_backend_pwm_slew_limits_published_pwm() -> None:
    backend = _backend(main_pwm_slew_rate_per_s=10.0)
    backend._slewed_pwm_rinbo_order = [0.0] * 6
    backend._last_pwm_publish_time = None
    msg = backend._make_motor_cmd_msg(_command(enable=True), enabled=True, preview=False, apply_slew=True)

    assert msg.r1.voltage < 10.0
    assert max(abs(x) for x in backend.last_pwm_rinbo_order) <= 0.200001
