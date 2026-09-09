"""Power command helper for the BioRoLaROS2/RhexROS2 sbRIO stack."""

from __future__ import annotations

import argparse
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
import fcntl
import json
import math
import os
import signal
import stat
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError


POWER_STATES = {
    "off": (False, False, False),
    "digital": (True, False, False),
    "sensors": (True, True, False),
    "relay": (True, True, True),
}

POWER_BUS_CHANNEL = 7
LEG_CURRENT_CHANNELS = tuple(range(1, 7))
LEG_CURRENT_CHANNEL_BY_NAME = {
    "L1": 1,
    "L2": 2,
    "L3": 3,
    "R1": 4,
    "R2": 5,
    "R3": 6,
}
ORIN_LEGS_EXECUTABLE = "/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs"
ORIN_LEGS_CONFIG_PATH = "/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml"
ORIN_LEGS_LOCK_PATH = ORIN_LEGS_CONFIG_PATH + ".lock"
POWER_COMMAND_TOPIC = "/power/command"
POWER_STATE_TOPIC = "/power/state"
EXPECTED_POWER_STATE_PUBLISHER = "rinbo_ros2_bridge"
HARD_MIN_BUS_VOLTAGE_V = 18.0
HARD_MAX_BUS_VOLTAGE_V = 42.0
HARD_MAX_HEALTHY_LEG_CURRENT_A = 3.0
RELAY_HEALTHY_SAMPLES = 3
RELAY_UNHEALTHY_TRIP_SAMPLES = 3
POWER_FEEDBACK_MAX_AGE_S = 0.35
POWER_GRAPH_CONVERGENCE_TIMEOUT_S = 1.0
UNKNOWN_GRAPH_ENDPOINT_NAME = (
    "_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_"
)


@dataclass(frozen=True)
class LegConfiguration:
    path: str
    schema_version: int
    revision: int
    hash: str
    disabled_legs: tuple[str, ...]
    enabled_legs: tuple[str, ...]

    def summary(self) -> dict:
        return {
            "path": self.path,
            "schema_version": self.schema_version,
            "revision": self.revision,
            "hash": self.hash,
            "disabled_legs": list(self.disabled_legs),
            "enabled_legs": list(self.enabled_legs),
        }


def _read_leg_configuration() -> LegConfiguration:
    """Use the FSM's authoritative loader, never a second YAML parser.

    The caller must hold the shared configuration lock before invoking this
    read-only, ROS-free command. Neither executable nor source is selected by
    shell, working directory, environment variable, or power-tool CLI flags.
    """

    try:
        result = subprocess.run(
            [ORIN_LEGS_EXECUTABLE, "status", "--json"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
            cwd="/",
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(
            f"Cannot load Orin leg configuration {ORIN_LEGS_CONFIG_PATH}: {exc}"
        ) from exc
    if result.returncode != 0:
        raise RuntimeError(
            f"Invalid Orin leg configuration {ORIN_LEGS_CONFIG_PATH}: "
            f"{result.stderr.strip() or result.stdout.strip()}"
        )
    try:
        data = json.loads(result.stdout)
        disabled = data["disabled_legs"]
        enabled = data["enabled_legs"]
        known = set(LEG_CURRENT_CHANNEL_BY_NAME)
        # Validate the inter-process envelope. YAML validation, canonical
        # normalization, revision and hash computation belong to rinbo_legs.
        if (
            data["path"] != ORIN_LEGS_CONFIG_PATH
            or type(data["schema_version"]) is not int
            or data["schema_version"] != 1
            or type(data["revision"]) is not int
            or data["revision"] < 1
            or not isinstance(data["hash"], str)
            or not data["hash"]
            or not isinstance(disabled, list)
            or not isinstance(enabled, list)
            or any(not isinstance(name, str) for name in disabled + enabled)
            or len(set(disabled)) != len(disabled)
            or len(set(enabled)) != len(enabled)
            or set(disabled) & set(enabled)
            or set(disabled) | set(enabled) != known
        ):
            raise ValueError("unexpected schema, source, revision, or leg sets")
        return LegConfiguration(
            path=data["path"],
            schema_version=data["schema_version"],
            revision=data["revision"],
            hash=data["hash"],
            disabled_legs=tuple(disabled),
            enabled_legs=tuple(enabled),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f"Invalid rinbo_legs status response for {ORIN_LEGS_CONFIG_PATH}: {exc}"
        ) from exc


@contextmanager
def _pinned_leg_configuration():
    """Prevent a manager write until the complete power operation has ended."""

    lock_fd = None
    try:
        try:
            lock_fd = os.open(
                ORIN_LEGS_LOCK_PATH,
                os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW,
                0o660,
            )
            if not stat.S_ISREG(os.fstat(lock_fd).st_mode):
                raise OSError("configuration lock is not a regular file")
            fcntl.flock(lock_fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except OSError as exc:
            raise RuntimeError(
                f"Cannot pin Orin leg configuration {ORIN_LEGS_CONFIG_PATH}; "
                f"a configuration change may be running: {exc}"
            ) from exc
        yield _read_leg_configuration()
    finally:
        if lock_fd is not None:
            os.close(lock_fd)


def _classify_graph_endpoint_names(
    names: list[str], expected_name: str
) -> str:
    """Classify a graph snapshot without weakening the single-writer gate.

    Fast DDS can briefly expose one endpoint before its node metadata reaches
    the local graph cache.  Only that exact placeholder (or no endpoint yet)
    is eligible for a bounded retry.  Multiple endpoints, partial metadata,
    and every known-but-wrong FQN remain immediate hard failures.
    """

    if names == [expected_name]:
        return "match"
    if not names or names == [UNKNOWN_GRAPH_ENDPOINT_NAME]:
        return "pending"
    return "reject"


def _summary(topic: str, digital: bool, signal: bool, power: bool, clean: bool, trigger: bool) -> dict:
    return {
        "topic": topic,
        "digital": bool(digital),
        "signal": bool(signal),
        "power": bool(power),
        "clean": bool(clean),
        "trigger": bool(trigger),
    }


def _power_channels(msg) -> list[dict[str, float]]:
    channels = []
    for idx in range(8):
        voltage = float(getattr(msg, f"v_{idx}", 0.0))
        current = float(getattr(msg, f"i_{idx}", 0.0))
        channels.append({"index": idx, "voltage_v": voltage, "current_a": current})
    return channels


def _power_state_matches(msg, digital: bool, signal: bool, power: bool) -> bool:
    """Return whether feedback acknowledges the requested relay state exactly."""

    return (
        bool(msg.digital) == bool(digital)
        and bool(msg.signal) == bool(signal)
        and bool(msg.power) == bool(power)
    )


def _healthy_leg_current_channels(disabled_legs: tuple[str, ...]) -> tuple[int, ...]:
    disabled_channels = {
        LEG_CURRENT_CHANNEL_BY_NAME[name] for name in disabled_legs
    }
    return tuple(
        channel for channel in LEG_CURRENT_CHANNELS if channel not in disabled_channels
    )


def _relay_feedback_violation(msg, disabled_legs: tuple[str, ...] = ()) -> str | None:
    """Return the hard relay-on violation for one fresh power sample."""

    channels = _power_channels(msg)
    bus_voltage = channels[POWER_BUS_CHANNEL]["voltage_v"]
    healthy_current_channels = _healthy_leg_current_channels(disabled_legs)
    healthy_currents = [
        channels[index]["current_a"] for index in healthy_current_channels
    ]
    if not math.isfinite(bus_voltage) or any(
        not math.isfinite(value) for value in healthy_currents
    ):
        return "non-finite bus voltage or healthy-leg current"
    if bus_voltage < HARD_MIN_BUS_VOLTAGE_V:
        return (
            f"bus voltage {bus_voltage:.3f}V is below "
            f"{HARD_MIN_BUS_VOLTAGE_V:.3f}V"
        )
    if bus_voltage > HARD_MAX_BUS_VOLTAGE_V:
        return (
            f"bus voltage {bus_voltage:.3f}V is above "
            f"{HARD_MAX_BUS_VOLTAGE_V:.3f}V"
        )
    max_current = max((abs(value) for value in healthy_currents), default=0.0)
    if max_current >= HARD_MAX_HEALTHY_LEG_CURRENT_A:
        return (
            f"healthy-leg current {max_current:.3f}A is at or above "
            f"{HARD_MAX_HEALTHY_LEG_CURRENT_A:.3f}A"
        )
    return None


def _validate_resolved_power_topics(resolve_topic_name) -> None:
    resolved_command = resolve_topic_name(POWER_COMMAND_TOPIC)
    resolved_state = resolve_topic_name(POWER_STATE_TOPIC)
    if (
        resolved_command != POWER_COMMAND_TOPIC
        or resolved_state != POWER_STATE_TOPIC
    ):
        raise RuntimeError(
            "Power writes forbid ROS topic remapping: "
            f"command resolved to {resolved_command!r}, state resolved to "
            f"{resolved_state!r}."
        )


def _power_feedback_header_key(
    msg, now_ns: int, previous: tuple[int, int] | None
) -> tuple[int, int]:
    try:
        sequence = int(msg.header.seq)
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise RuntimeError("Power feedback has no valid header sequence/stamp.") from exc
    max_age_ns = int(POWER_FEEDBACK_MAX_AGE_S * 1_000_000_000)
    if sequence <= 0 or stamp_ns <= 0:
        raise RuntimeError(
            "Power feedback header sequence/stamp must both be positive."
        )
    age_ns = int(now_ns) - stamp_ns
    if abs(age_ns) > max_age_ns:
        raise RuntimeError(
            f"Power feedback header age {age_ns * 1.0e-9:.3f}s exceeds "
            f"+/-{POWER_FEEDBACK_MAX_AGE_S:.3f}s."
        )
    if previous is not None and (
        sequence <= previous[0] or stamp_ns <= previous[1]
    ):
        raise RuntimeError(
            "Power feedback header is duplicate/out-of-order: "
            f"current=({sequence},{stamp_ns}) previous={previous}."
        )
    return sequence, stamp_ns


def print_dry_run(args: argparse.Namespace) -> None:
    configuration = args.leg_configuration
    config_summary = configuration.summary() if configuration is not None else None
    if args.mode == "status":
        print(json.dumps({"state_topic": args.state_topic, "mode": "status", "dry_run": True, "leg_configuration": config_summary}, indent=2))
        return
    modes = ["digital", "sensors"]
    if args.mode == "sequence":
        if args.include_relay:
            modes.append("relay")
    else:
        modes = [args.mode]
    payload = [
        {
            **_summary(args.topic, *POWER_STATES[mode], clean=args.clean, trigger=args.trigger),
            "leg_configuration": config_summary,
        }
        for mode in modes
    ]
    print(json.dumps(payload[0] if len(payload) == 1 else payload, indent=2))


class RinboPowerTool(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        if args.mode != "off" and not isinstance(
            getattr(args, "leg_configuration", None), LegConfiguration
        ):
            raise RuntimeError("A validated Orin leg configuration is required before creating a power node.")
        super().__init__("redrhex_rinbo_power_tool")
        self.args = args
        _validate_resolved_power_topics(self.resolve_topic_name)
        try:
            from rinbo_msgs.msg import PowerCmdStamped, PowerStateStamped
        except Exception as exc:  # pragma: no cover - requires external RhexROS2 overlay
            raise RuntimeError(
                "rinbo_msgs is required. Build/source ~/rinbo_ros_ws first: source ~/rinbo_ros_ws/install/setup.bash"
            ) from exc
        self.PowerCmdStamped = PowerCmdStamped
        self.PowerStateStamped = PowerStateStamped
        self.last_power_state = None
        self.last_power_state_received_monotonic: float | None = None
        # Status is genuinely read-only: do not even create a graph endpoint
        # on /power/command, because a publisher that never sends can still
        # violate the single-writer safety contract of another process.
        self.pub = (
            None
            if args.mode == "status"
            else self.create_publisher(PowerCmdStamped, args.topic, 10)
        )
        self.create_subscription(PowerStateStamped, args.state_topic, self._on_power_state, 10)
        self.sequence = 0

    def _on_power_state(self, msg) -> None:
        self.last_power_state = msg
        self.last_power_state_received_monotonic = time.monotonic()

    def wait_for_subscriber(self) -> None:
        if self.args.dry_run:
            return
        if self.pub is None:
            raise RuntimeError("Internal error: status mode has no power publisher.")
        deadline = time.monotonic() + max(float(self.args.wait_for_subscriber_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pub.get_subscription_count() == 0 and not self.args.allow_no_subscriber:
            raise RuntimeError(
                f"No subscriber on {self.args.topic}. Start/source rinbo_ros_bridge before sending power commands."
            )

    def build_msg(self, digital: bool, signal: bool, power: bool):
        self.sequence += 1
        msg = self.PowerCmdStamped()
        msg.header.seq = self.sequence
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "redrhex_power"
        msg.digital = bool(digital)
        msg.signal = bool(signal)
        msg.power = bool(power)
        if hasattr(msg, "clean"):
            msg.clean = bool(self.args.clean)
        if hasattr(msg, "trigger"):
            msg.trigger = bool(self.args.trigger)
        return msg

    def summarize(self, msg) -> dict:
        return _summary(
            self.args.topic,
            bool(msg.digital),
            bool(msg.signal),
            bool(msg.power),
            bool(getattr(msg, "clean", False)),
            bool(getattr(msg, "trigger", False)),
        )

    def summarize_power_state(self, msg) -> dict:
        summary = self.summarize(msg)
        summary["topic"] = self.args.state_topic
        channels = _power_channels(msg)
        healthy_current_channels = _healthy_leg_current_channels(
            self.args.disabled_legs
        )
        summary["channels"] = channels
        summary["bus_voltage_v"] = channels[POWER_BUS_CHANNEL]["voltage_v"]
        summary["bus_current_a"] = channels[POWER_BUS_CHANNEL]["current_a"]
        configuration = self.args.leg_configuration
        summary["leg_configuration"] = configuration.summary() if configuration is not None else None
        summary["disabled_legs"] = list(configuration.disabled_legs) if configuration is not None else None
        summary["healthy_leg_current_channels"] = list(healthy_current_channels)
        summary["max_healthy_leg_current_a"] = max(
            (
                abs(channels[index]["current_a"])
                for index in healthy_current_channels
            ),
            default=0.0,
        )
        return summary

    @staticmethod
    def _endpoint_names(infos) -> list[str]:
        return [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in infos
        ]

    def assert_expected_enable_graph(self) -> None:
        """Authenticate the sole command source and sole power-state source."""

        command_infos = self.get_publishers_info_by_topic(self.args.topic)
        command_names = self._endpoint_names(command_infos)
        if command_names != [self.get_fully_qualified_name()]:
            raise RuntimeError(
                "Refusing power enable: /power/command must have only this "
                f"tool as publisher, got {command_names}."
            )
        self.assert_expected_state_graph()

    def wait_for_expected_enable_graph(self) -> None:
        """Wait briefly for endpoint metadata before any energizing publish.

        This is intentionally narrower than retrying an arbitrary identity
        failure: only an empty graph or the exact DDS UNKNOWN placeholder can
        converge.  A second endpoint or a known wrong FQN is rejected on the
        first observation.
        """

        expected_command = self.get_fully_qualified_name()
        expected_state = f"/{EXPECTED_POWER_STATE_PUBLISHER}"
        deadline = time.monotonic() + POWER_GRAPH_CONVERGENCE_TIMEOUT_S
        command_names: list[str] = []
        state_names: list[str] = []

        while rclpy.ok():
            command_names = self._endpoint_names(
                self.get_publishers_info_by_topic(self.args.topic)
            )
            state_names = self._endpoint_names(
                self.get_publishers_info_by_topic(self.args.state_topic)
            )
            command_status = _classify_graph_endpoint_names(
                command_names, expected_command
            )
            state_status = _classify_graph_endpoint_names(
                state_names, expected_state
            )

            if command_status == "reject":
                raise RuntimeError(
                    "Refusing power enable: /power/command must have only this "
                    f"tool as publisher, got {command_names}."
                )
            if state_status == "reject":
                raise RuntimeError(
                    "Refusing power acknowledgement: /power/state must have exactly one "
                    f"publisher named {expected_state}, got {state_names}."
                )
            if command_status == "match" and state_status == "match":
                return
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        raise RuntimeError(
            "Refusing power enable: DDS graph metadata did not converge within "
            f"{POWER_GRAPH_CONVERGENCE_TIMEOUT_S:.2f}s; "
            f"/power/command={command_names}, /power/state={state_names}."
        )

    def assert_expected_state_graph(self) -> None:
        """Authenticate the sole final-bridge power-state publisher."""

        state_infos = self.get_publishers_info_by_topic(self.args.state_topic)
        state_names = self._endpoint_names(state_infos)
        if state_names != [f"/{EXPECTED_POWER_STATE_PUBLISHER}"]:
            raise RuntimeError(
                "Refusing power acknowledgement: /power/state must have exactly one "
                f"publisher named /{EXPECTED_POWER_STATE_PUBLISHER}, got "
                f"{state_names}."
            )

    def wait_for_ack(
        self,
        digital: bool,
        signal: bool,
        power: bool,
        command_started_monotonic: float,
    ) -> None:
        deadline = time.monotonic() + max(float(self.args.verify_timeout_s), 0.0)
        last_processed_receipt: float | None = None
        last_feedback_key: tuple[int, int] | None = None
        healthy_count = 0
        unhealthy_count = 0
        last_violation: str | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            received_after_command = (
                self.last_power_state_received_monotonic is not None
                and self.last_power_state_received_monotonic >= command_started_monotonic
            )
            if received_after_command and self.last_power_state is not None:
                receipt = self.last_power_state_received_monotonic
                if receipt != last_processed_receipt:
                    last_processed_receipt = receipt
                    # Re-check the graph for every fresh acknowledgement.  A
                    # second writer or replacement feedback publisher that
                    # appears after the command was sent must not win a TOCTOU
                    # race and authorize the relay.
                    self.assert_expected_enable_graph()
                    last_feedback_key = _power_feedback_header_key(
                        self.last_power_state,
                        self.get_clock().now().nanoseconds,
                        last_feedback_key,
                    )
                    tuple_matches = _power_state_matches(
                        self.last_power_state,
                        digital=digital,
                        signal=signal,
                        power=power,
                    )

                    # Any observed relay-on sample is safety relevant, even
                    # when its digital/signal tuple does not match the request.
                    if bool(self.last_power_state.power):
                        last_violation = _relay_feedback_violation(
                            self.last_power_state, self.args.disabled_legs
                        )
                        if last_violation is None:
                            unhealthy_count = 0
                            if power and tuple_matches:
                                healthy_count += 1
                            else:
                                healthy_count = 0
                        else:
                            healthy_count = 0
                            unhealthy_count += 1
                            if unhealthy_count >= RELAY_UNHEALTHY_TRIP_SAMPLES:
                                raise RuntimeError(
                                    "Relay feedback failed the hard power guard for "
                                    f"{unhealthy_count} consecutive samples: "
                                    f"{last_violation}."
                                )
                    else:
                        healthy_count = 0
                        unhealthy_count = 0
                        last_violation = None

                    if not power and tuple_matches:
                        self.assert_expected_enable_graph()
                        print(
                            json.dumps(
                                self.summarize_power_state(self.last_power_state),
                                indent=2,
                            )
                        )
                        return
                    if power and tuple_matches and healthy_count >= RELAY_HEALTHY_SAMPLES:
                        self.assert_expected_enable_graph()
                        summary = self.summarize_power_state(self.last_power_state)
                        summary["relay_healthy_samples"] = healthy_count
                        print(json.dumps(summary, indent=2))
                        return
            rclpy.spin_once(self, timeout_sec=0.05)

        last_state = None
        if self.last_power_state is not None:
            last_state = {
                "digital": bool(self.last_power_state.digital),
                "signal": bool(self.last_power_state.signal),
                "power": bool(self.last_power_state.power),
            }
        raise RuntimeError(
            "Power command was not acknowledged on "
            f"{self.args.state_topic} within {self.args.verify_timeout_s:.2f}s; "
            f"requested digital={digital} signal={signal} power={power}, "
            f"last_state={last_state}, last_guard_violation={last_violation}. "
            "Treat relay state as UNKNOWN and verify physically."
        )

    def read_status(self) -> None:
        deadline = time.monotonic() + max(float(self.args.status_timeout_s), 0.0)
        last_processed_receipt: float | None = None
        last_feedback_key: tuple[int, int] | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            receipt = self.last_power_state_received_monotonic
            if (
                self.last_power_state is not None
                and receipt is not None
                and receipt != last_processed_receipt
            ):
                last_processed_receipt = receipt
                self.assert_expected_state_graph()
                last_feedback_key = _power_feedback_header_key(
                    self.last_power_state,
                    self.get_clock().now().nanoseconds,
                    last_feedback_key,
                )
                if bool(self.last_power_state.power):
                    violation = _relay_feedback_violation(
                        self.last_power_state, self.args.disabled_legs
                    )
                    if violation is not None:
                        raise RuntimeError(
                            f"Relay is on but power feedback is unsafe: {violation}."
                        )
                self.assert_expected_state_graph()
                print(
                    json.dumps(
                        self.summarize_power_state(self.last_power_state), indent=2
                    )
                )
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"No message received on {self.args.state_topic} before timeout.")

    def publish_state(
        self,
        digital: bool,
        signal: bool,
        power: bool,
        *,
        _automatic_cleanup: bool = False,
    ) -> None:
        if self.args.dry_run:
            msg = self.build_msg(digital, signal, power)
            print(json.dumps(self.summarize(msg), indent=2))
            return
        if self.pub is None:
            raise RuntimeError("Internal error: status mode cannot publish power.")
        self.wait_for_subscriber()
        if digital or signal or power:
            self.wait_for_expected_enable_graph()
        command_started_monotonic = time.monotonic()
        msg = None
        for _ in range(max(int(self.args.repeat), 1)):
            # The final Rinbo arbiter rejects duplicate/out-of-order source
            # headers.  Every retry therefore needs a fresh sequence as well
            # as a fresh stamp; reusing one message would turn the first valid
            # relay-on request into an immediate fail-closed relay-off.
            msg = self.build_msg(digital, signal, power)
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(max(float(self.args.repeat_delay_s), 0.0))
        assert msg is not None
        self.get_logger().warn(
            f"Published power command digital={msg.digital} signal={msg.signal} power={msg.power}"
        )
        try:
            self.wait_for_ack(
                digital=digital,
                signal=signal,
                power=power,
                command_started_monotonic=command_started_monotonic,
            )
        except RuntimeError as exc:
            relay_observed_on = bool(
                self.last_power_state is not None
                and bool(self.last_power_state.power)
            )
            if _automatic_cleanup or not (power or relay_observed_on):
                raise
            try:
                self.publish_state(
                    False,
                    False,
                    False,
                    _automatic_cleanup=True,
                )
            except RuntimeError as off_exc:
                raise RuntimeError(
                    f"{exc} Automatic all-off acknowledgement also failed: "
                    f"{off_exc} Relay state is UNKNOWN; use the physical E-stop "
                    "or power cut."
                ) from exc
            raise RuntimeError(
                f"{exc} Automatic all-off was acknowledged; relay is off."
            ) from exc

    def publish_unverified_emergency_off(self) -> bool:
        """Best-effort relay-off after an interrupted command.

        This deliberately bypasses acknowledgement waiting because the ROS
        context may already be shutting down.  Returning ``True`` means only
        that fresh off messages were handed to the local publisher; the caller
        must continue to report the physical relay state as UNKNOWN.
        """

        try:
            if not rclpy.ok() or self.pub is None:
                return False
            for _ in range(3):
                self.pub.publish(self.build_msg(False, False, False))
                time.sleep(0.01)
            return True
        except BaseException:
            # Cleanup must not hide the original interruption.  In particular,
            # rclpy can raise again once its context has begun shutting down.
            return False

    def run(self) -> None:
        if self.args.mode == "status":
            self.read_status()
            return
        if self.args.mode == "sequence":
            sequence = ["digital", "sensors"]
            if self.args.include_relay:
                sequence.append("relay")
            for mode in sequence:
                self.publish_state(*POWER_STATES[mode])
                time.sleep(max(float(self.args.step_delay_s), 0.0))
            return
        self.publish_state(*POWER_STATES[self.args.mode])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish BioRoLaROS2/RhexROS2 rinbo_msgs/PowerCmdStamped safely.")
    parser.add_argument("mode", choices=["off", "digital", "sensors", "relay", "sequence", "status"])
    parser.add_argument("--topic", default=POWER_COMMAND_TOPIC)
    parser.add_argument("--state-topic", default=POWER_STATE_TOPIC)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--repeat-delay-s", type=float, default=0.05)
    parser.add_argument("--step-delay-s", type=float, default=0.5)
    parser.add_argument("--wait-for-subscriber-s", type=float, default=2.0)
    parser.add_argument("--status-timeout-s", type=float, default=3.0)
    parser.add_argument(
        "--verify-timeout-s",
        type=float,
        default=3.0,
        help="Require matching /power/state feedback after every command.",
    )
    parser.add_argument(
        "--allow-no-subscriber",
        action="store_true",
        help="Publish even if /power/command has no visible subscriber. Usually not recommended.",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--trigger", action="store_true")
    parser.add_argument(
        "--include-relay",
        action="store_true",
        help="For sequence mode, also publish power=true after digital and signal.",
    )
    parser.add_argument(
        "--confirm-relay",
        action="store_true",
        help="Required for any command that sets power=true.",
    )
    parser.add_argument(
        "--disabled-leg",
        default=None,
        help="Retired: configure the shared Orin mask with ros2 run rinbo_fsm rinbo_legs.",
    )
    return parser


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.disabled_leg is not None:
        raise SystemExit(
            "--disabled-leg is retired and cannot override the Orin configuration. "
            "Use ros2 run rinbo_fsm rinbo_legs on Orin."
        )
    wants_relay = args.mode == "relay" or (args.mode == "sequence" and args.include_relay)
    if wants_relay and not args.confirm_relay and not args.dry_run:
        raise SystemExit("Refusing power=true without --confirm-relay. Keep E-stop ready and rerun intentionally.")
    if args.mode == "status":
        if args.state_topic != POWER_STATE_TOPIC:
            raise SystemExit(
                "Power status requires the exact /power/state topic."
            )
    else:
        if args.topic != POWER_COMMAND_TOPIC or args.state_topic != POWER_STATE_TOPIC:
            raise SystemExit(
                "Power writes require the exact /power/command and /power/state topics."
            )
        if args.allow_no_subscriber:
            raise SystemExit(
                "Power writes forbid --allow-no-subscriber; the final bridge must be visible."
            )
        if args.clean or args.trigger:
            raise SystemExit(
                "Power writes through this tool forbid the legacy --clean/--trigger fields."
            )
        if not 3 <= args.repeat <= 8:
            raise SystemExit("Power writes require --repeat in [3, 8].")
        if not 0.02 <= args.repeat_delay_s <= 0.20:
            raise SystemExit(
                "Power writes require --repeat-delay-s in [0.02, 0.20]."
            )
        if not 0.5 <= args.verify_timeout_s <= 3.0:
            raise SystemExit(
                "Power writes require --verify-timeout-s in [0.5, 3.0]."
            )

    # All-off remains available even if configuration is corrupt or locked.
    # It uses no current exemptions and makes no claim about enabled legs.
    context = nullcontext(None) if args.mode == "off" else _pinned_leg_configuration()
    try:
        with context as configuration:
            args.leg_configuration = configuration
            args.disabled_legs = configuration.disabled_legs if configuration is not None else ()
            if args.dry_run:
                print_dry_run(args)
                return
            if configuration is not None:
                print(json.dumps({"leg_configuration": configuration.summary()}, indent=2))
            _run_ros(args)
    except RuntimeError as exc:
        raise SystemExit(f"[FATAL] {exc}") from exc


def _run_ros(args: argparse.Namespace) -> None:
    # Keep the ROS context alive while handling SIGINT/SIGTERM so an
    # interrupted power operation still gets a chance to hand fresh all-off
    # messages to the publisher.  The result remains UNKNOWN without a fresh
    # acknowledgement, so callers must still use the physical E-stop/cut.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}

    def _request_stop(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, _request_stop)
    try:
        node = RinboPowerTool(args)
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError) as exc:
        off_published = False
        if node is not None and args.mode != "status":
            off_published = node.publish_unverified_emergency_off()
        cleanup = (
            "Fresh power-off messages were published as an unverified best effort; "
            if off_published
            else "No best-effort power-off could be published; "
        )
        raise SystemExit(
            "Power command/status was interrupted before verified completion; "
            f"relay state is UNKNOWN. {cleanup}"
            "use the physical E-stop or power cut, then require a fresh "
            "acknowledged power-off/status check."
        ) from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (RCLError, RuntimeError):
                pass
        if rclpy.ok():
            rclpy.shutdown()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
