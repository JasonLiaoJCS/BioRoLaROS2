"""BioRoLaROS2/RhexROS2 sbRIO bringup checker for RedRhex deployment."""

from __future__ import annotations

import argparse
import math
import os
import re
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:  # pragma: no cover - depends on rclpy version
    RCLError = RuntimeError


@dataclass
class CheckResult:
    level: str
    name: str
    detail: str


def _exact_publisher_result(
    topic: str, publisher_count: int, publisher_names: str
) -> CheckResult:
    """Evaluate a required unique telemetry source without touching ROS."""

    if publisher_count == 1:
        return CheckResult("OK", f"publishers {topic}", f"1: {publisher_names}")
    return CheckResult(
        "ERROR",
        f"publishers {topic}",
        f"expected exactly 1, got {publisher_count}: {publisher_names}",
    )


def _fresh_message_result(
    topic: str,
    *,
    seen: bool,
    receipt_monotonic_s: float | None,
    now_monotonic_s: float,
    timeout_s: float,
) -> CheckResult:
    """Evaluate receipt freshness entirely within the monotonic clock domain."""

    if not seen or receipt_monotonic_s is None:
        return CheckResult("ERROR", topic, "no message before timeout")
    age_s = now_monotonic_s - receipt_monotonic_s
    if not math.isfinite(age_s) or age_s < 0.0 or age_s > timeout_s:
        return CheckResult(
            "ERROR",
            topic,
            f"stale/invalid receipt age={age_s:.4f}s, limit={timeout_s:.4f}s",
        )
    return CheckResult(
        "OK", topic, f"fresh message age={age_s:.4f}s, limit={timeout_s:.4f}s"
    )


def _fresh_boolean_result(
    name: str,
    *,
    seen: bool,
    value: bool | None,
    expected: bool,
    receipt_monotonic_s: float | None,
    now_monotonic_s: float,
    timeout_s: float,
) -> CheckResult:
    """Require both a fresh boolean sample and its exact fail-safe value."""

    freshness = _fresh_message_result(
        name,
        seen=seen,
        receipt_monotonic_s=receipt_monotonic_s,
        now_monotonic_s=now_monotonic_s,
        timeout_s=timeout_s,
    )
    if freshness.level != "OK":
        return freshness
    actual = bool(value)
    if actual != expected:
        return CheckResult(
            "ERROR", name, f"received {str(actual).lower()}, expected {str(expected).lower()}"
        )
    return CheckResult(
        "OK",
        name,
        f"received {str(actual).lower()}; {freshness.detail}",
    )


def _fresh_power_chain_result(
    *,
    seen: bool,
    digital: bool | None,
    signal: bool | None,
    power: bool | None,
    receipt_monotonic_s: float | None,
    now_monotonic_s: float,
    timeout_s: float,
) -> CheckResult:
    """Require one fresh PowerState with every power-chain relay asserted."""

    name = "power chain on (digital/signal/power)"
    freshness = _fresh_message_result(
        name,
        seen=seen,
        receipt_monotonic_s=receipt_monotonic_s,
        now_monotonic_s=now_monotonic_s,
        timeout_s=timeout_s,
    )
    if freshness.level != "OK":
        return freshness

    states = {
        "digital": bool(digital),
        "signal": bool(signal),
        "power": bool(power),
    }
    off_relays = [relay for relay, enabled in states.items() if not enabled]
    state_detail = ", ".join(
        f"{relay}={str(enabled).lower()}" for relay, enabled in states.items()
    )
    if off_relays:
        return CheckResult(
            "ERROR",
            name,
            f"{state_detail}; required true: {', '.join(off_relays)}",
        )
    return CheckResult("OK", name, f"{state_detail}; {freshness.detail}")


def _power_state_required(args: argparse.Namespace) -> bool:
    return bool(args.require_power_state or args.require_power_relay_on)


def _result_exit_code(results: list[CheckResult], *, strict: bool) -> int:
    """Return the documented fail-closed process status for collected checks."""

    if any(result.level == "ERROR" for result in results):
        return 2
    if strict and any(result.level == "WARN" for result in results):
        return 1
    return 0


def _validate_args(args: argparse.Namespace) -> None:
    if args.max_motor_command_publishers < 0:
        raise ValueError("--max-motor-command-publishers must be >= 0")
    positive_timeouts = {
        "--tcp-timeout-s": args.tcp_timeout_s,
        "--message-timeout-s": args.message_timeout_s,
        "--motor-state-timeout-s": args.motor_state_timeout_s,
        "--power-state-timeout-s": args.power_state_timeout_s,
        "--motor-output-status-timeout-s": args.motor_output_status_timeout_s,
        "--imu-timeout-s": args.imu_timeout_s,
    }
    for name, value in positive_timeouts.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
    nonnegative_durations = {
        "--discovery-timeout-s": args.discovery_timeout_s,
        "--rate-sample-s": args.rate_sample_s,
    }
    for name, value in nonnegative_durations.items():
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
    required_topics = {
        "--motor-output-status-topic": (
            args.require_motor_output_disabled,
            args.motor_output_status_topic,
        ),
        "--imu-topic": (args.require_imu, args.imu_topic),
    }
    for name, (required, topic) in required_topics.items():
        if required and not str(topic).strip():
            raise ValueError(f"{name} cannot be empty when its check is required")


def _parse_master_addr(value: str | None) -> tuple[str | None, int | None]:
    if not value:
        return None, None
    if ":" not in value:
        return value, 50051
    host, port_text = value.rsplit(":", 1)
    try:
        return host, int(port_text)
    except ValueError:
        return host, None


def _local_ipv4_addrs() -> list[str]:
    addrs: set[str] = set()
    try:
        output = subprocess.check_output(["hostname", "-I"], text=True, timeout=1.0)
        addrs.update(token for token in output.split() if "." in token)
    except Exception:
        pass
    try:
        _hostname, _aliases, host_addrs = socket.gethostbyname_ex(socket.gethostname())
        addrs.update(host_addrs)
    except OSError:
        pass
    addrs.discard("127.0.0.1")
    return sorted(addrs)


def _stream_stats(arrival_times: list[float]) -> tuple[int, float, float]:
    count = len(arrival_times)
    if count < 2:
        return count, 0.0, float("inf")
    duration = arrival_times[-1] - arrival_times[0]
    rate_hz = (count - 1) / duration if duration > 0.0 else 0.0
    max_gap_s = max(
        later - earlier for earlier, later in zip(arrival_times, arrival_times[1:])
    )
    return count, rate_hz, max_gap_s


class RinboBringupCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("redrhex_rinbo_bringup_check")
        self.args = args
        self.results: list[CheckResult] = []
        self.motor_state_seen = False
        self.power_state_seen = False
        self.last_motor_state = None
        self.last_power_state = None
        self.motor_state_arrival_times: list[float] = []
        self.power_state_arrival_times: list[float] = []
        self.motor_output_status_seen = False
        self.last_motor_output_status: bool | None = None
        self.last_motor_output_status_time: float | None = None
        self.imu_seen = False
        self.last_imu_time: float | None = None
        self.motor_output_status_sub = None
        self.imu_sub = None
        if self.args.require_motor_output_disabled:
            self.motor_output_status_sub = self.create_subscription(
                Bool,
                self.args.motor_output_status_topic,
                self._on_motor_output_status,
                10,
            )
        if self.args.require_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                self.args.imu_topic,
                self._on_imu,
                qos_profile_sensor_data,
            )
        try:
            from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped, PowerCmdStamped, PowerStateStamped
        except Exception as exc:  # pragma: no cover - requires external BioRoLaROS2 overlay
            self.rinbo_import_error = exc
            self.MotorCmdStamped = None
            self.MotorStateStamped = None
            self.PowerCmdStamped = None
            self.PowerStateStamped = None
        else:
            self.rinbo_import_error = None
            self.MotorCmdStamped = MotorCmdStamped
            self.MotorStateStamped = MotorStateStamped
            self.PowerCmdStamped = PowerCmdStamped
            self.PowerStateStamped = PowerStateStamped
            self.create_subscription(MotorStateStamped, self.args.motor_state_topic, self._on_motor_state, 10)
            self.create_subscription(PowerStateStamped, self.args.power_state_topic, self._on_power_state, 10)

    def add(self, level: str, name: str, detail: str) -> None:
        self.results.append(CheckResult(level, name, detail))

    def _on_motor_state(self, msg) -> None:
        self.motor_state_seen = True
        self.last_motor_state = msg
        self.motor_state_arrival_times.append(time.monotonic())

    def _on_power_state(self, msg) -> None:
        self.power_state_seen = True
        self.last_power_state = msg
        self.power_state_arrival_times.append(time.monotonic())

    def _on_motor_output_status(self, msg: Bool) -> None:
        self.motor_output_status_seen = True
        self.last_motor_output_status = bool(msg.data)
        self.last_motor_output_status_time = time.monotonic()

    def _on_imu(self, _msg: Imu) -> None:
        self.imu_seen = True
        self.last_imu_time = time.monotonic()

    def check_env(self) -> None:
        ros_distro = os.environ.get("ROS_DISTRO")
        self.add("OK" if ros_distro else "WARN", "ROS_DISTRO", ros_distro or "not set")

        master = self.args.master_addr or os.environ.get("CORE_MASTER_ADDR")
        local_ip = self.args.local_ip or os.environ.get("CORE_LOCAL_IP")
        core_ip = os.environ.get("CORE_IP")
        host, port = _parse_master_addr(master)
        if host and port:
            self.add("OK", "CORE_MASTER_ADDR", f"{host}:{port}")
        else:
            self.add("ERROR", "CORE_MASTER_ADDR", "not set or invalid; expected <sbRIO_ip>:50051")
        local_addrs = _local_ipv4_addrs()
        if local_ip and (not local_addrs or local_ip in local_addrs):
            detail = local_ip if not local_addrs else f"{local_ip} in host IPs {local_addrs}"
            self.add("OK", "CORE_LOCAL_IP", detail)
        elif local_ip:
            self.add("WARN", "CORE_LOCAL_IP", f"{local_ip} not in host IPs {local_addrs}")
        else:
            self.add("WARN", "CORE_LOCAL_IP", f"not set; host IPs {local_addrs}")
        if core_ip and host and core_ip != host:
            self.add(
                "WARN",
                "CORE_IP",
                f"legacy CORE_IP={core_ip} differs from authoritative CORE_MASTER_ADDR host {host}.",
            )
        elif core_ip:
            self.add("OK", "CORE_IP", f"optional legacy value {core_ip}")
        else:
            self.add(
                "OK",
                "CORE_IP",
                "not set; final rinbo_ros_bridge resolves the sbRIO host from CORE_MASTER_ADDR.",
            )

        if host and port:
            try:
                with socket.create_connection((host, port), timeout=self.args.tcp_timeout_s):
                    pass
            except OSError as exc:
                self.add("WARN", "TCP 50051", f"cannot connect to {host}:{port}: {exc}")
            else:
                self.add("OK", "TCP 50051", f"connected to {host}:{port}")

    def check_bridge_source(self) -> None:
        source_arg = str(self.args.bridge_source or "").strip()
        if not source_arg:
            return
        source_path = Path(source_arg).expanduser()
        if not source_path.exists():
            self.add(
                "WARN",
                "BioRoLaROS2 CORE_IP source check",
                f"{source_path} not found; skip hardcoded CORE_IP check",
            )
            return
        try:
            text = source_path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            self.add("WARN", "BioRoLaROS2 CORE_IP source check", f"cannot read {source_path}: {exc}")
            return
        match = re.search(r'setenv\s*\(\s*"CORE_IP"\s*,\s*"([^"]+)"', text)
        master = self.args.master_addr or os.environ.get("CORE_MASTER_ADDR")
        host, _port = _parse_master_addr(master)
        if not match:
            self.add(
                "OK",
                "BioRoLaROS2 CORE_IP source check",
                f"{source_path} has no obvious hardcoded setenv(\"CORE_IP\", ...)",
            )
            return
        hardcoded_ip = match.group(1)
        if not host:
            self.add(
                "WARN",
                "BioRoLaROS2 hardcoded CORE_IP",
                f"{source_path} hardcodes CORE_IP={hardcoded_ip}; set CORE_MASTER_ADDR=<same_ip>:50051 before bringup.",
            )
        elif hardcoded_ip != host:
            self.add(
                "ERROR",
                "BioRoLaROS2 hardcoded CORE_IP",
                (
                    f"{source_path} hardcodes CORE_IP={hardcoded_ip}, but CORE_MASTER_ADDR host is {host}. "
                    "Edit rinbo_ros_bridge.cpp and rebuild BioRoLaROS2 before hardware bringup."
                ),
            )
        else:
            self.add("OK", "BioRoLaROS2 hardcoded CORE_IP", f"{hardcoded_ip} matches CORE_MASTER_ADDR host")

    def check_rinbo_msgs(self) -> None:
        if self.rinbo_import_error is None:
            self.add("OK", "rinbo_msgs", "MotorStateStamped and PowerStateStamped importable")
            self.check_message_contract()
        else:
            self.add("ERROR", "rinbo_msgs", f"not importable: {self.rinbo_import_error}")

    def _missing_fields(self, obj, fields: list[str]) -> list[str]:
        return [field for field in fields if not hasattr(obj, field)]

    def check_message_contract(self) -> None:
        assert self.MotorCmdStamped is not None
        assert self.MotorStateStamped is not None
        assert self.PowerCmdStamped is not None
        assert self.PowerStateStamped is not None

        errors: list[str] = []
        motor_cmd = self.MotorCmdStamped()
        motor_state = self.MotorStateStamped()
        power_cmd = self.PowerCmdStamped()
        power_state = self.PowerStateStamped()

        leg_names = ["l1", "l2", "l3", "r1", "r2", "r3"]
        servo_names = ["sl1", "sl2", "sl3", "sr1", "sr2", "sr3"]
        missing_motor_cmd = self._missing_fields(motor_cmd, ["header", "servo_control_mode"] + leg_names + servo_names)
        if missing_motor_cmd:
            errors.append(f"MotorCmdStamped missing {missing_motor_cmd}")
        missing_motor_state = self._missing_fields(motor_state, ["header", "servo_control_mode"] + leg_names + servo_names)
        if missing_motor_state:
            errors.append(f"MotorStateStamped missing {missing_motor_state}")

        for leg_name in leg_names:
            if hasattr(motor_cmd, leg_name):
                missing = self._missing_fields(
                    getattr(motor_cmd, leg_name),
                    ["enable", "direction", "voltage", "state", "reset_position"],
                )
                if missing:
                    errors.append(f"MotorCmdStamped.{leg_name} missing {missing}")
            if hasattr(motor_state, leg_name):
                missing = self._missing_fields(getattr(motor_state, leg_name), ["position", "tick_count", "hall_effect"])
                if missing:
                    errors.append(f"MotorStateStamped.{leg_name} missing {missing}")

        for servo_name in servo_names:
            if hasattr(motor_cmd, servo_name):
                missing = self._missing_fields(getattr(motor_cmd, servo_name), ["position_encoder"])
                if missing:
                    errors.append(f"MotorCmdStamped.{servo_name} missing {missing}")
            if hasattr(motor_state, servo_name):
                missing = self._missing_fields(getattr(motor_state, servo_name), ["position_encoder"])
                if missing:
                    errors.append(f"MotorStateStamped.{servo_name} missing {missing}")

        missing_power_cmd = self._missing_fields(power_cmd, ["header", "digital", "signal", "power", "clean", "trigger"])
        if missing_power_cmd:
            errors.append(f"PowerCmdStamped missing {missing_power_cmd}")
        power_state_fields = ["header", "digital", "signal", "power", "clean"]
        for idx in range(8):
            power_state_fields.extend([f"v_{idx}", f"i_{idx}"])
        missing_power_state = self._missing_fields(power_state, power_state_fields)
        if missing_power_state:
            errors.append(f"PowerStateStamped missing {missing_power_state}")

        if errors:
            self.add("ERROR", "BioRoLaROS2 message contract", "; ".join(errors))
        else:
            self.add(
                "OK",
                "BioRoLaROS2 message contract",
                "MotorCmd/State, PowerCmd/State fields match expected rinbo_msgs interface",
            )

    def check_topics(self) -> None:
        deadline = time.monotonic() + max(float(self.args.discovery_timeout_s), 0.0)
        topic_map: dict[str, list[str]] = {}
        required_topics = [
            self.args.motor_state_topic,
            self.args.power_state_topic,
            self.args.motor_command_topic,
        ]
        if self.args.require_motor_output_disabled:
            required_topics.append(self.args.motor_output_status_topic)
        if self.args.require_imu:
            required_topics.append(self.args.imu_topic)
        while rclpy.ok() and time.monotonic() < deadline:
            topic_map = dict(self.get_topic_names_and_types())
            if all(topic in topic_map for topic in required_topics):
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        report_topics = [
            (self.args.motor_command_topic, False),
            (self.args.motor_state_topic, False),
            (self.args.power_command_topic, False),
            (self.args.power_state_topic, _power_state_required(self.args)),
        ]
        if self.args.require_motor_output_disabled:
            report_topics.append((self.args.motor_output_status_topic, True))
        if self.args.require_imu:
            report_topics.append((self.args.imu_topic, True))
        reported: set[str] = set()
        for topic, required in report_topics:
            if topic in reported:
                continue
            reported.add(topic)
            if topic in topic_map:
                self.add("OK", f"topic {topic}", ",".join(topic_map[topic]))
            else:
                self.add(
                    "ERROR" if required else "WARN",
                    f"topic {topic}",
                    "not discovered before timeout",
                )

    def _endpoint_names(self, infos) -> str:
        names = []
        for info in infos:
            node_name = getattr(info, "node_name", "")
            node_namespace = getattr(info, "node_namespace", "")
            full_name = f"{node_namespace.rstrip('/')}/{node_name}".replace("//", "/")
            names.append(full_name or "<unknown>")
        return ", ".join(names) if names else "none"

    def check_graph_endpoints(self) -> None:
        motor_command_publishers = self.get_publishers_info_by_topic(self.args.motor_command_topic)
        motor_command_publisher_count = len(motor_command_publishers)
        if motor_command_publisher_count <= self.args.max_motor_command_publishers:
            self.add(
                "OK",
                f"publishers {self.args.motor_command_topic}",
                f"{motor_command_publisher_count}: {self._endpoint_names(motor_command_publishers)}",
            )
        else:
            self.add(
                "ERROR",
                f"publishers {self.args.motor_command_topic}",
                (
                    f"{motor_command_publisher_count}: {self._endpoint_names(motor_command_publishers)}. "
                    "Stop rinbo_tripod/rinbo_standing or duplicate RL bridge nodes before continuing."
                ),
            )

        power_command_subscribers = self.get_subscriptions_info_by_topic(self.args.power_command_topic)
        if power_command_subscribers:
            self.add(
                "OK",
                f"subscribers {self.args.power_command_topic}",
                f"{len(power_command_subscribers)}: {self._endpoint_names(power_command_subscribers)}",
            )
        else:
            self.add(
                "WARN",
                f"subscribers {self.args.power_command_topic}",
                "none. Start rinbo_ros_bridge before using biorola_power_tool.",
            )

        motor_state_publishers = self.get_publishers_info_by_topic(self.args.motor_state_topic)
        motor_state_count = len(motor_state_publishers)
        if motor_state_count == 1:
            self.add(
                "OK",
                f"publishers {self.args.motor_state_topic}",
                f"1: {self._endpoint_names(motor_state_publishers)}",
            )
        else:
            self.add(
                "ERROR" if self.args.strict else "WARN",
                f"publishers {self.args.motor_state_topic}",
                f"expected exactly 1, got {motor_state_count}: {self._endpoint_names(motor_state_publishers)}",
            )

        power_state_publishers = self.get_publishers_info_by_topic(self.args.power_state_topic)
        power_state_count = len(power_state_publishers)
        if power_state_count == 1:
            self.add(
                "OK",
                f"publishers {self.args.power_state_topic}",
                f"1: {self._endpoint_names(power_state_publishers)}",
            )
        else:
            self.add(
                "ERROR"
                if self.args.strict or _power_state_required(self.args)
                else "WARN",
                f"publishers {self.args.power_state_topic}",
                f"expected exactly 1, got {power_state_count}: {self._endpoint_names(power_state_publishers)}",
            )

        if self.args.require_motor_output_disabled:
            publishers = self.get_publishers_info_by_topic(
                self.args.motor_output_status_topic
            )
            self.results.append(
                _exact_publisher_result(
                    self.args.motor_output_status_topic,
                    len(publishers),
                    self._endpoint_names(publishers),
                )
            )

        if self.args.require_imu:
            publishers = self.get_publishers_info_by_topic(self.args.imu_topic)
            self.results.append(
                _exact_publisher_result(
                    self.args.imu_topic,
                    len(publishers),
                    self._endpoint_names(publishers),
                )
            )

    def wait_for_messages(self) -> None:
        deadline = time.monotonic() + max(float(self.args.message_timeout_s), 0.0)
        ready_since: float | None = None
        power_state_required = _power_state_required(self.args)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            power_ready = self.power_state_seen or not power_state_required
            rinbo_ready = self.rinbo_import_error is not None or (
                self.motor_state_seen and power_ready
            )
            motor_output_ready = not self.args.require_motor_output_disabled or self.motor_output_status_seen
            imu_ready = self.imu_seen or not self.args.require_imu
            ready = rinbo_ready and motor_output_ready and imu_ready
            if ready and ready_since is None:
                ready_since = time.monotonic()
            if ready_since is not None and time.monotonic() - ready_since >= self.args.rate_sample_s:
                break
        now_s = time.monotonic()
        if self.rinbo_import_error is None:
            missing_level = "ERROR" if self.args.strict else "WARN"
            self.add("OK" if self.motor_state_seen else missing_level, self.args.motor_state_topic, "message received" if self.motor_state_seen else "no message before timeout")
            if self.last_motor_state is not None:
                self.add("OK", "motor_state summary", self._summarize_motor_state(self.last_motor_state))
            if power_state_required:
                power_missing_level = (
                    "ERROR"
                    if self.args.strict or self.args.require_power_relay_on
                    else "WARN"
                )
                self.add("OK" if self.power_state_seen else power_missing_level, self.args.power_state_topic, "message received" if self.power_state_seen else "no message before timeout")
            if self.last_power_state is not None:
                self.add("OK", "power_state summary", self._summarize_power_state(self.last_power_state))
            self._report_stream_health(
                self.args.motor_state_topic,
                self.motor_state_arrival_times,
                self.args.motor_state_timeout_s,
            )
            if power_state_required or self.power_state_seen:
                self._report_stream_health(
                    self.args.power_state_topic,
                    self.power_state_arrival_times,
                    self.args.power_state_timeout_s,
                )

        if self.args.require_power_relay_on:
            self.results.append(
                _fresh_power_chain_result(
                    seen=self.power_state_seen,
                    digital=(
                        bool(getattr(self.last_power_state, "digital", False))
                        if self.last_power_state is not None
                        else None
                    ),
                    signal=(
                        bool(getattr(self.last_power_state, "signal", False))
                        if self.last_power_state is not None
                        else None
                    ),
                    power=(
                        bool(getattr(self.last_power_state, "power", False))
                        if self.last_power_state is not None
                        else None
                    ),
                    receipt_monotonic_s=(
                        self.power_state_arrival_times[-1]
                        if self.power_state_arrival_times
                        else None
                    ),
                    now_monotonic_s=now_s,
                    timeout_s=self.args.power_state_timeout_s,
                )
            )

        if self.args.require_motor_output_disabled:
            self.results.append(
                _fresh_boolean_result(
                    f"{self.args.motor_output_status_topic} disabled",
                    seen=self.motor_output_status_seen,
                    value=self.last_motor_output_status,
                    expected=False,
                    receipt_monotonic_s=self.last_motor_output_status_time,
                    now_monotonic_s=now_s,
                    timeout_s=self.args.motor_output_status_timeout_s,
                )
            )

        if self.args.require_imu:
            self.results.append(
                _fresh_message_result(
                    self.args.imu_topic,
                    seen=self.imu_seen,
                    receipt_monotonic_s=self.last_imu_time,
                    now_monotonic_s=now_s,
                    timeout_s=self.args.imu_timeout_s,
                )
            )

    def _report_stream_health(
        self, topic: str, arrival_times: list[float], timeout_s: float
    ) -> None:
        count, rate_hz, max_gap_s = _stream_stats(arrival_times)
        if count < 2:
            self.add(
                "ERROR" if self.args.strict else "WARN",
                f"stream {topic}",
                f"only {count} message(s); cannot validate rate/max gap against {timeout_s:.3f}s timeout",
            )
            return
        within_timeout = max_gap_s <= timeout_s
        self.add(
            "OK" if within_timeout else ("ERROR" if self.args.strict else "WARN"),
            f"stream {topic}",
            f"count={count} rate={rate_hz:.1f}Hz max_gap={max_gap_s:.4f}s timeout={timeout_s:.4f}s",
        )

    def _summarize_power_state(self, msg) -> str:
        channels = []
        for idx in range(8):
            voltage = float(getattr(msg, f"v_{idx}", 0.0))
            current = float(getattr(msg, f"i_{idx}", 0.0))
            if abs(voltage) > 1.0e-6 or abs(current) > 1.0e-6:
                channels.append(f"ch{idx}={voltage:.2f}V/{current:.2f}A")
        rail_summary = "; rails " + ", ".join(channels) if channels else "; rails no nonzero readings"
        return (
            f"digital={bool(getattr(msg, 'digital', False))} "
            f"signal={bool(getattr(msg, 'signal', False))} "
            f"power={bool(getattr(msg, 'power', False))}"
            f"{rail_summary}"
        )

    def _summarize_motor_state(self, msg) -> str:
        positions = []
        for field in ["l1", "l2", "l3", "r1", "r2", "r3"]:
            if hasattr(msg, field):
                leg = getattr(msg, field)
                positions.append(f"{field}={float(getattr(leg, 'position', 0.0)):.1f}")
        servos = []
        for field in ["sl1", "sl2", "sl3", "sr1", "sr2", "sr3"]:
            if hasattr(msg, field):
                servo = getattr(msg, field)
                servos.append(f"{field}={int(getattr(servo, 'position_encoder', 0))}")
        chunks = []
        if positions:
            chunks.append("positions " + ", ".join(positions))
        if servos:
            chunks.append("servos " + ", ".join(servos))
        return "; ".join(chunks) if chunks else "message received"

    def run(self) -> int:
        self.check_env()
        self.check_bridge_source()
        self.check_rinbo_msgs()
        self.check_topics()
        self.wait_for_messages()
        # Sample endpoint cardinality last so the result represents the
        # handoff state after the observation window, not an early graph race.
        self.check_graph_endpoints()
        code = _result_exit_code(self.results, strict=self.args.strict)
        for result in self.results:
            print(f"[{result.level:5}] {result.name}: {result.detail}")
        if code == 2:
            print("\nFix ERROR items before continuing.")
            return 2
        if any(result.level == "WARN" for result in self.results):
            print("\nWARN items may be normal before rinbo_ros_bridge/sbRIO is running, but do not run RL on hardware yet.")
            return code
        print("\nBioRoLaROS2 bridge checks look good.")
        return code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check BioRoLaROS2/RhexROS2 sbRIO environment before RedRhex RL bringup.")
    parser.add_argument("--master-addr", default=None, help="Override CORE_MASTER_ADDR, e.g. 192.168.0.100:50051")
    parser.add_argument("--local-ip", default=None, help="Override CORE_LOCAL_IP for display/check only.")
    parser.add_argument("--tcp-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--bridge-source",
        default="~/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp",
        help="BioRoLaROS2 rinbo_ros_bridge.cpp path used to detect hardcoded CORE_IP.",
    )
    parser.add_argument("--discovery-timeout-s", type=float, default=2.0)
    parser.add_argument("--message-timeout-s", type=float, default=3.0)
    parser.add_argument("--rate-sample-s", type=float, default=1.0)
    parser.add_argument("--motor-state-timeout-s", type=float, default=0.10)
    parser.add_argument("--power-state-timeout-s", type=float, default=0.35)
    parser.add_argument("--motor-output-status-timeout-s", type=float, default=0.20)
    parser.add_argument("--imu-timeout-s", type=float, default=0.10)
    parser.add_argument("--motor-command-topic", default="/motor/command")
    parser.add_argument("--motor-state-topic", default="/motor/state")
    parser.add_argument("--power-command-topic", default="/power/command")
    parser.add_argument("--power-state-topic", default="/power/state")
    parser.add_argument(
        "--motor-output-status-topic", default="/rinbo/motor_output_enabled"
    )
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--max-motor-command-publishers", type=int, default=1)
    parser.add_argument("--require-power-state", action="store_true")
    parser.add_argument(
        "--require-power-relay-on",
        action="store_true",
        help=(
            "Require a fresh /power/state sample with "
            "digital=true, signal=true, and power=true."
        ),
    )
    parser.add_argument(
        "--require-motor-output-disabled",
        action="store_true",
        help=(
            "Require exactly one publisher and a fresh false sample on "
            "--motor-output-status-topic."
        ),
    )
    parser.add_argument(
        "--require-imu",
        action="store_true",
        help="Require exactly one publisher and a fresh message on --imu-topic.",
    )
    parser.add_argument("--strict", action="store_true", help="Return nonzero on WARN as well as ERROR.")
    return parser


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        _validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    rclpy.init()
    node = RinboBringupCheck(args)
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
