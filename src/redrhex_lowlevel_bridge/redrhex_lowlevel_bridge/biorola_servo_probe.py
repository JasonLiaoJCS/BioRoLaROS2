"""One-at-a-time BioRoLaROS2 servo probe.

This tool is intentionally conservative: it disables all six main legs and,
when motion is confirmed, holds the five non-selected servos at their latest
reported encoder positions while moving only one selected servo by a small
amount.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Iterable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError


SERVO_FIELDS = ("sl1", "sl2", "sl3", "sr1", "sr2", "sr3")
HARD_MAX_CURRENT_A = 3.0
HARD_MIN_BUS_VOLTAGE_V = 18.0
HARD_MAX_BUS_VOLTAGE_V = 30.0
HARD_MAX_SERVO_DELTA_COUNTS = 200
HARD_SERVO_CONTROL_MODE = 2
HARD_MOTOR_ARRIVAL_TIMEOUT_S = 0.25
HARD_POWER_ARRIVAL_TIMEOUT_S = 0.5
HARD_MOTOR_SOURCE_AGE_S = 0.10
HARD_POWER_SOURCE_AGE_S = 0.35
EXPECTED_BRIDGE_NODE_NAME = "rinbo_ros2_bridge"
MOTION_COMMAND_TOPIC = "/motor/command"
MOTION_STATE_TOPIC = "/motor/state"
MOTION_POWER_STATE_TOPIC = "/power/state"
MOTION_BUS_VOLTAGE_CHANNEL = 7
MOTION_LEG_CURRENT_CHANNELS = [1, 2, 3, 4, 5, 6]


def validate_resolved_motion_topics(resolve_topic_name) -> None:
    """Reject ROS remaps that would evade the pre-init argparse topic gate."""

    expected_topics = (
        MOTION_COMMAND_TOPIC,
        MOTION_STATE_TOPIC,
        MOTION_POWER_STATE_TOPIC,
    )
    remapped = []
    for topic in expected_topics:
        resolved = str(resolve_topic_name(topic))
        if resolved != topic:
            remapped.append(f"{topic}->{resolved}")
    if remapped:
        raise RuntimeError(
            "Servo motion/hold forbids ROS topic remapping: " + ", ".join(remapped)
        )


def _bool(value: object) -> bool:
    return bool(value)


def relay_is_exactly_on(summary: dict) -> bool:
    """Require the complete digital/signal/relay acknowledgement tuple."""

    return bool(
        summary.get("received", False)
        and summary.get("digital", False)
        and summary.get("signal", False)
        and summary.get("power", False)
    )


def validate_feedback_header(
    header,
    now_ns: int,
    previous: tuple[int, int] | None,
    max_age_s: float,
) -> tuple[tuple[int, int] | None, str | None]:
    """Validate fresh, positive, monotonically increasing source metadata."""

    nanosec = int(getattr(getattr(header, "stamp", None), "nanosec", -1))
    sec = int(getattr(getattr(header, "stamp", None), "sec", 0))
    if nanosec < 0 or nanosec >= 1_000_000_000:
        return None, "source stamp.nanosec is outside [0,1e9)"
    stamp_ns = sec * 1_000_000_000 + nanosec
    if stamp_ns <= 0:
        return None, "source stamp must be positive"
    if abs(int(now_ns) - stamp_ns) > int(float(max_age_s) * 1.0e9):
        return None, f"source stamp exceeds +/-{max_age_s:g}s age bound"
    sequence = int(getattr(header, "seq", 0)) & 0xFFFFFFFF
    if previous is not None:
        previous_sequence, previous_stamp_ns = previous
        if stamp_ns <= previous_stamp_ns:
            return None, "source stamp is duplicate or non-monotonic"
        delta = (sequence - previous_sequence) & 0xFFFFFFFF
        if delta == 0 or delta >= 0x80000000:
            return None, "source sequence is duplicate or out-of-order"
    return (sequence, stamp_ns), None


def summarize_power_state(
    msg, bus_voltage_channel: int, leg_current_channels: Iterable[int]
) -> dict:
    channels = [
        {
            "index": idx,
            "voltage_v": float(getattr(msg, f"v_{idx}", 0.0)),
            "current_a": float(getattr(msg, f"i_{idx}", 0.0)),
        }
        for idx in range(8)
    ]
    current_channels = [int(channel) for channel in leg_current_channels]
    bus_voltage = channels[int(bus_voltage_channel)]["voltage_v"]
    leg_currents = [channels[channel]["current_a"] for channel in current_channels]
    return {
        "received": True,
        "digital": _bool(getattr(msg, "digital", False)),
        "signal": _bool(getattr(msg, "signal", False)),
        "power": _bool(getattr(msg, "power", False)),
        "clean": _bool(getattr(msg, "clean", False)),
        "bus_voltage_channel": int(bus_voltage_channel),
        "bus_voltage_v": bus_voltage,
        "leg_current_channels": current_channels,
        "max_abs_leg_current_a": max((abs(value) for value in leg_currents), default=0.0),
        "relevant_values_finite": math.isfinite(bus_voltage)
        and all(math.isfinite(value) for value in leg_currents),
        "channels": channels,
    }


class BioRoLaServoProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("biorola_servo_probe")
        self.args = args
        if args.mode in ("hold", "test"):
            # This runs after rclpy.init (so ROS remaps are visible) but before
            # importing messages, creating subscriptions, or creating a command
            # publisher.
            validate_resolved_motion_topics(self.resolve_topic_name)
        try:
            from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped, PowerStateStamped
        except Exception as exc:  # pragma: no cover - requires local ROS overlay
            raise RuntimeError(
                "rinbo_msgs is required. Build/source the workspace first: "
                "source /home/jetson/rinbo_ros_ws/install/setup.bash"
            ) from exc

        self.MotorCmdStamped = MotorCmdStamped
        self.MotorStateStamped = MotorStateStamped
        self.PowerStateStamped = PowerStateStamped
        self.last_motor_state = None
        self.last_motor_state_time = None
        self.last_power_state = None
        self.last_power_state_time = None
        self.motion_mode = args.mode in ("hold", "test")
        self.motor_header_key = None
        self.power_header_key = None
        self.motor_source_error = None
        self.power_source_error = None
        self.sequence = 0

        self.create_subscription(MotorStateStamped, args.state_topic, self._on_motor_state, 10)
        self.create_subscription(PowerStateStamped, args.power_state_topic, self._on_power_state, 10)
        self.pub = None

    def _on_motor_state(self, msg) -> None:
        if self.motion_mode:
            key, error = validate_feedback_header(
                msg.header,
                self.get_clock().now().nanoseconds,
                self.motor_header_key,
                HARD_MOTOR_SOURCE_AGE_S,
            )
            if error is not None:
                self.motor_source_error = error
                return
            self.motor_header_key = key
        self.last_motor_state = msg
        self.last_motor_state_time = time.monotonic()

    def _on_power_state(self, msg) -> None:
        if self.motion_mode:
            key, error = validate_feedback_header(
                msg.header,
                self.get_clock().now().nanoseconds,
                self.power_header_key,
                HARD_POWER_SOURCE_AGE_S,
            )
            if error is not None:
                self.power_source_error = error
                return
            self.power_header_key = key
        self.last_power_state = msg
        self.last_power_state_time = time.monotonic()

    def wait_for_motor_state(self) -> None:
        deadline = time.monotonic() + max(float(self.args.state_timeout_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.last_motor_state is not None:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"No message received on {self.args.state_topic} before timeout.")

    def servo_positions(self) -> list[int]:
        if self.last_motor_state is None:
            return [0] * len(SERVO_FIELDS)
        values = []
        for field in SERVO_FIELDS:
            values.append(int(getattr(getattr(self.last_motor_state, field), "position_encoder", 0)))
        return values

    def validate_servo_readback_for_motion(self, positions: list[int]) -> None:
        if self.args.allow_zero_servo_readback:
            return
        if all(value == 0 for value in positions):
            raise RuntimeError(
                "All servo encoder readbacks are 0. Refusing hold/test because that would command servo targets "
                "relative to a likely invalid readback. Check servo power/signal wiring and /motor/state first."
            )

    def power_summary(self) -> dict:
        if self.last_power_state is None:
            return {"received": False}
        return summarize_power_state(
            self.last_power_state,
            self.args.power_bus_voltage_channel,
            self.args.leg_current_channels,
        )

    def print_status(self) -> None:
        self.wait_for_motor_state()
        rclpy.spin_once(self, timeout_sec=0.05)
        print(
            json.dumps(
                {
                    "motor_state_topic": self.args.state_topic,
                    "servo_order": SERVO_FIELDS,
                    "servo_positions": dict(zip(SERVO_FIELDS, self.servo_positions())),
                    "servo_control_mode": int(getattr(self.last_motor_state, "servo_control_mode", 0)),
                    "power_state": self.power_summary(),
                },
                indent=2,
            )
        )

    def assert_no_existing_motor_command_publishers(self) -> None:
        publishers = self.get_publishers_info_by_topic(self.args.command_topic)
        if publishers and not self.args.allow_existing_command_publishers:
            details = ", ".join(f"{info.node_namespace}/{info.node_name}" for info in publishers)
            raise RuntimeError(
                f"Refusing to publish: existing publishers on {self.args.command_topic}: {details}. "
                "Stop rinbo_cali/rinbo_tripod/RL bridge before running this probe."
            )

    def assert_expected_feedback_publisher(self, topic: str) -> None:
        publishers = self.get_publishers_info_by_topic(topic)
        if len(publishers) != 1:
            raise RuntimeError(
                f"{topic} requires exactly one publisher, got {len(publishers)}."
            )
        publisher = publishers[0]
        if publisher.node_name != EXPECTED_BRIDGE_NODE_NAME:
            raise RuntimeError(
                f"{topic} publisher must be {EXPECTED_BRIDGE_NODE_NAME}, got "
                f"{publisher.node_namespace}/{publisher.node_name}."
            )

    def assert_motion_feedback_sources(self) -> None:
        self.assert_expected_feedback_publisher(self.args.state_topic)
        self.assert_expected_feedback_publisher(self.args.power_state_topic)
        if self.motor_source_error is not None:
            raise RuntimeError(
                f"Untrusted {self.args.state_topic}: {self.motor_source_error}."
            )
        if self.power_source_error is not None:
            raise RuntimeError(
                f"Untrusted {self.args.power_state_topic}: {self.power_source_error}."
            )
        now = time.monotonic()
        if self.last_motor_state is None or self.last_motor_state_time is None:
            raise RuntimeError(f"No trusted message received on {self.args.state_topic}.")
        if self.last_power_state is None or self.last_power_state_time is None:
            raise RuntimeError(f"No trusted message received on {self.args.power_state_topic}.")
        motor_age = now - self.last_motor_state_time
        power_age = now - self.last_power_state_time
        if motor_age > HARD_MOTOR_ARRIVAL_TIMEOUT_S:
            raise RuntimeError(
                f"{self.args.state_topic} stale: {motor_age:.3f}s > "
                f"{HARD_MOTOR_ARRIVAL_TIMEOUT_S:.3f}s."
            )
        if power_age > self.args.power_state_timeout_s:
            raise RuntimeError(
                f"{self.args.power_state_topic} stale: {power_age:.3f}s > "
                f"{self.args.power_state_timeout_s:.3f}s."
            )

    def wait_for_motion_feedback(self) -> None:
        deadline = time.monotonic() + max(float(self.args.state_timeout_s), 0.0)
        last_error = "trusted motor and power feedback not received"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self.assert_motion_feedback_sources()
                return
            except RuntimeError as exc:
                last_error = str(exc)
        raise RuntimeError(f"Motion feedback gate failed: {last_error}")

    def ensure_publisher(self) -> None:
        if self.pub is None:
            self.assert_no_existing_motor_command_publishers()
            self.pub = self.create_publisher(self.MotorCmdStamped, self.args.command_topic, 10)
        deadline = time.monotonic() + max(float(self.args.wait_for_subscriber_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pub.get_subscription_count() == 0 and not self.args.allow_no_subscriber:
            raise RuntimeError(f"No subscriber on {self.args.command_topic}. Start rinbo_ros_bridge first.")

    def power_is_ok(self) -> bool:
        if self.last_power_state is None:
            if self.args.require_power_state:
                self.get_logger().error(f"Waiting for {self.args.power_state_topic}; refusing motion.")
                return False
            return True
        age = time.monotonic() - (self.last_power_state_time or 0.0)
        if age > self.args.power_state_timeout_s:
            self.get_logger().error(f"{self.args.power_state_topic} stale: {age:.3f}s; refusing motion.")
            return False
        summary = self.power_summary()
        if self.args.require_power_on and not relay_is_exactly_on(summary):
            self.get_logger().error(
                "Power acknowledgement is not exactly digital=true, signal=true, "
                "power=true; refusing motion."
            )
            return False
        if not summary.get("relevant_values_finite", False):
            self.get_logger().error("Power guard tripped: non-finite bus voltage or leg current.")
            return False
        max_current = float(summary.get("max_abs_leg_current_a", 0.0))
        if max_current > self.args.max_current_a:
            self.get_logger().error(
                f"Current guard tripped: max_abs_leg_current={max_current:.3f}A > {self.args.max_current_a:.3f}A."
            )
            return False
        bus_voltage = float(summary.get("bus_voltage_v", 0.0))
        if bus_voltage < self.args.min_voltage_v:
            self.get_logger().error(f"Voltage guard tripped: bus_voltage={bus_voltage:.3f}V.")
            return False
        if bus_voltage > self.args.max_voltage_v:
            self.get_logger().error(f"Over-voltage guard tripped: bus_voltage={bus_voltage:.3f}V.")
            return False
        return True

    def build_cmd(
        self, servo_targets: Iterable[int], servo_control_mode: int | None = None
    ):
        self.sequence += 1
        cmd = self.MotorCmdStamped()
        cmd.header.seq = self.sequence
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "biorola_servo_probe"
        for field in ("l1", "l2", "l3", "r1", "r2", "r3"):
            leg = getattr(cmd, field)
            leg.enable = False
            leg.direction = False
            leg.voltage = 0.0
            leg.state = 0
            leg.reset_position = False
        for field, target in zip(SERVO_FIELDS, servo_targets):
            getattr(cmd, field).position_encoder = int(target)
        cmd.servo_control_mode = int(
            self.args.servo_control_mode
            if servo_control_mode is None
            else servo_control_mode
        )
        return cmd

    def publish_disabled_handshake(
        self, targets: list[int], repeats: int = 8
    ) -> None:
        """Rearm/stop the final arbiter with fresh mode-0 packets."""

        assert self.pub is not None
        for _ in range(max(int(repeats), 1)):
            self.pub.publish(self.build_cmd(targets, servo_control_mode=0))
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def publish_targets(self, targets: list[int]) -> None:
        self.assert_motion_feedback_sources()
        if not self.power_is_ok():
            raise RuntimeError("Safety guard refused to publish servo motion command.")
        assert self.pub is not None
        self.pub.publish(self.build_cmd(targets))

    def run_hold(self) -> None:
        self.wait_for_motion_feedback()
        self.ensure_publisher()
        hold = self.servo_positions()
        self.validate_servo_readback_for_motion(hold)
        self.publish_disabled_handshake(hold)
        self.get_logger().warn(f"Holding current servo positions {dict(zip(SERVO_FIELDS, hold))}")
        deadline = time.monotonic() + max(float(self.args.duration_s), 0.0)
        rate_s = 1.0 / max(float(self.args.rate_hz), 1.0)
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_targets(hold)
                rclpy.spin_once(self, timeout_sec=min(0.02, rate_s))
                time.sleep(rate_s)
        finally:
            self.publish_disabled_handshake(hold)

    def run_test(self) -> None:
        if not self.args.confirm_motion:
            raise RuntimeError("Refusing servo motion without --confirm-motion.")
        self.wait_for_motion_feedback()
        self.ensure_publisher()

        servo_index = SERVO_FIELDS.index(self.args.servo)
        start = self.servo_positions()
        self.validate_servo_readback_for_motion(start)
        self.publish_disabled_handshake(start)
        if self.args.target is not None:
            target_value = int(self.args.target)
        else:
            target_value = int(start[servo_index] + self.args.delta)
        delta = target_value - start[servo_index]
        if abs(delta) > self.args.max_abs_delta:
            raise RuntimeError(
                f"Requested move is {delta} counts, larger than --max-abs-delta={self.args.max_abs_delta}."
            )

        self.get_logger().warn(
            f"Testing only {self.args.servo}: {start[servo_index]} -> {target_value} -> {start[servo_index]}. "
            "All main legs are disabled; other servo targets hold their latest readback."
        )
        rate_hz = max(float(self.args.rate_hz), 1.0)
        rate_s = 1.0 / rate_hz
        hold_start_s = max(float(self.args.pre_hold_s), 0.0)
        move_s = max(float(self.args.move_s), rate_s)
        hold_target_s = max(float(self.args.target_hold_s), 0.0)
        return_s = max(float(self.args.return_s), rate_s)

        def run_phase(duration_s: float, begin: int, end: int) -> None:
            steps = max(int(math.ceil(duration_s * rate_hz)), 1)
            for step in range(steps):
                alpha = 1.0 if steps == 1 else step / float(steps - 1)
                value = int(round(begin + (end - begin) * alpha))
                targets = list(start)
                targets[servo_index] = value
                self.publish_targets(targets)
                rclpy.spin_once(self, timeout_sec=min(0.02, rate_s))
                time.sleep(rate_s)

        def hold_phase(duration_s: float, value: int) -> None:
            deadline = time.monotonic() + duration_s
            targets = list(start)
            targets[servo_index] = int(value)
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_targets(targets)
                rclpy.spin_once(self, timeout_sec=min(0.02, rate_s))
                time.sleep(rate_s)

        try:
            hold_phase(hold_start_s, start[servo_index])
            run_phase(move_s, start[servo_index], target_value)
            hold_phase(hold_target_s, target_value)
            run_phase(return_s, target_value, start[servo_index])
            hold_phase(max(float(self.args.post_hold_s), 0.0), start[servo_index])
        finally:
            self.publish_disabled_handshake(start)
        rclpy.spin_once(self, timeout_sec=0.05)
        end = self.servo_positions()
        self.get_logger().warn(f"Final readback {dict(zip(SERVO_FIELDS, end))}")

    def run(self) -> None:
        if self.args.mode == "status":
            self.print_status()
        elif self.args.mode == "hold":
            self.run_hold()
        elif self.args.mode == "test":
            self.run_test()
        else:  # pragma: no cover - argparse constrains choices
            raise RuntimeError(f"Unsupported mode: {self.args.mode}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Safely inspect and move one BioRoLaROS2 servo at a time."
    )
    parser.add_argument("mode", choices=["status", "hold", "test"])
    parser.add_argument("servo", nargs="?", choices=SERVO_FIELDS, default="sl1")
    parser.add_argument("--command-topic", default="/motor/command")
    parser.add_argument("--state-topic", default="/motor/state")
    parser.add_argument("--power-state-topic", default="/power/state")
    parser.add_argument("--state-timeout-s", type=float, default=3.0)
    parser.add_argument("--power-state-timeout-s", type=float, default=0.5)
    parser.add_argument("--wait-for-subscriber-s", type=float, default=2.0)
    parser.add_argument("--allow-no-subscriber", action="store_true")
    parser.add_argument("--allow-existing-command-publishers", action="store_true")
    parser.add_argument("--servo-control-mode", type=int, default=2)
    parser.add_argument("--duration-s", type=float, default=5.0, help="Hold duration for hold mode.")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--delta", type=int, default=20, help="Relative encoder-count move for test mode.")
    parser.add_argument("--target", type=int, default=None, help="Absolute selected-servo encoder target.")
    parser.add_argument("--max-abs-delta", type=int, default=200)
    parser.add_argument("--pre-hold-s", type=float, default=0.5)
    parser.add_argument("--move-s", type=float, default=1.0)
    parser.add_argument("--target-hold-s", type=float, default=0.5)
    parser.add_argument("--return-s", type=float, default=1.0)
    parser.add_argument("--post-hold-s", type=float, default=0.5)
    parser.add_argument("--require-power-state", action="store_true")
    parser.add_argument("--require-power-on", action="store_true")
    parser.add_argument("--min-voltage-v", type=float, default=18.0)
    parser.add_argument("--max-voltage-v", type=float, default=30.0)
    parser.add_argument("--max-current-a", type=float, default=3.0)
    parser.add_argument("--power-bus-voltage-channel", type=int, default=7)
    parser.add_argument(
        "--leg-current-channels",
        type=int,
        nargs=6,
        default=[1, 2, 3, 4, 5, 6],
        metavar=("L1", "L2", "L3", "R1", "R2", "R3"),
    )
    parser.add_argument("--allow-zero-servo-readback", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    return parser


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.power_bus_voltage_channel not in range(8):
        parser.error("--power-bus-voltage-channel must be in [0, 7]")
    if any(channel not in range(8) for channel in args.leg_current_channels):
        parser.error("--leg-current-channels entries must be in [0, 7]")
    if len(set(args.leg_current_channels)) != 6:
        parser.error("--leg-current-channels entries must be unique")
    if args.power_bus_voltage_channel in args.leg_current_channels:
        parser.error("bus voltage channel must not be listed as a leg-current channel")
    if not 0.0 < args.min_voltage_v < args.max_voltage_v:
        parser.error("voltage limits must satisfy 0 < min < max")
    if not math.isfinite(args.max_current_a) or args.max_current_a <= 0.0:
        parser.error("--max-current-a must be positive and finite")
    if args.mode in ("hold", "test"):
        motion_contract = {
            "--command-topic": (args.command_topic, MOTION_COMMAND_TOPIC),
            "--state-topic": (args.state_topic, MOTION_STATE_TOPIC),
            "--power-state-topic": (
                args.power_state_topic,
                MOTION_POWER_STATE_TOPIC,
            ),
        }
        changed_topics = [
            f"{name}={actual!r} (required {expected!r})"
            for name, (actual, expected) in motion_contract.items()
            if actual != expected
        ]
        if changed_topics:
            raise SystemExit(
                "Servo motion/hold uses fixed safety topics: "
                + ", ".join(changed_topics)
            )
        if (
            args.power_bus_voltage_channel != MOTION_BUS_VOLTAGE_CHANNEL
            or list(args.leg_current_channels) != MOTION_LEG_CURRENT_CHANNELS
        ):
            raise SystemExit(
                "Servo motion/hold power mapping is fixed at bus voltage channel 7 "
                "and leg current channels [1,2,3,4,5,6]."
            )
        if not args.confirm_motion:
            raise SystemExit(
                "Refusing servo motion/hold without --confirm-motion."
            )
        if not args.require_power_state or not args.require_power_on:
            raise SystemExit(
                "Servo motion/hold requires --require-power-state and "
                "--require-power-on; fresh exact digital/signal/relay feedback "
                "is mandatory."
            )
        forbidden = []
        if args.allow_no_subscriber:
            forbidden.append("--allow-no-subscriber")
        if args.allow_existing_command_publishers:
            forbidden.append("--allow-existing-command-publishers")
        if args.allow_zero_servo_readback:
            forbidden.append("--allow-zero-servo-readback")
        if forbidden:
            raise SystemExit(
                "Servo motion/hold forbids safety escape flags: "
                + ", ".join(forbidden)
            )
        if args.max_current_a > HARD_MAX_CURRENT_A:
            raise SystemExit(
                f"Servo motion/hold requires --max-current-a <= {HARD_MAX_CURRENT_A:g}."
            )
        if args.min_voltage_v < HARD_MIN_BUS_VOLTAGE_V:
            raise SystemExit(
                "Servo motion/hold requires --min-voltage-v >= "
                f"{HARD_MIN_BUS_VOLTAGE_V:g}."
            )
        if args.max_voltage_v > HARD_MAX_BUS_VOLTAGE_V:
            raise SystemExit(
                "Servo motion/hold requires --max-voltage-v <= "
                f"{HARD_MAX_BUS_VOLTAGE_V:g}."
            )
        if (
            not math.isfinite(args.power_state_timeout_s)
            or args.power_state_timeout_s <= 0.0
            or args.power_state_timeout_s > HARD_POWER_ARRIVAL_TIMEOUT_S
        ):
            raise SystemExit(
                "Servo motion/hold requires 0 < --power-state-timeout-s <= "
                f"{HARD_POWER_ARRIVAL_TIMEOUT_S:g}."
            )
        if not 0 < args.max_abs_delta <= HARD_MAX_SERVO_DELTA_COUNTS:
            raise SystemExit(
                "Servo motion/hold requires 0 < --max-abs-delta <= "
                f"{HARD_MAX_SERVO_DELTA_COUNTS}."
            )
        if abs(args.delta) > min(
            args.max_abs_delta, HARD_MAX_SERVO_DELTA_COUNTS
        ):
            raise SystemExit(
                "Servo test --delta exceeds the configured/hard maximum "
                f"of {min(args.max_abs_delta, HARD_MAX_SERVO_DELTA_COUNTS)} counts."
            )
        if args.servo_control_mode != HARD_SERVO_CONTROL_MODE:
            raise SystemExit(
                "Servo motion/hold requires --servo-control-mode=2 exactly."
            )
    rclpy.init()
    node = None
    try:
        node = BioRoLaServoProbe(args)
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError) as exc:
        raise SystemExit(
            "Servo probe was interrupted before verified completion; servo/motor "
            "output state is UNKNOWN. Use the physical E-stop or power cut, then "
            "verify fresh output-disabled and power-state feedback."
        ) from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
