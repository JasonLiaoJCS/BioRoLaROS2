"""Read-only BioRoLaROS2 servo/Scon fault diagnostics.

Only snapshot and watch are supported.  The former power-sweep and command-path
write modes are retired because they bypass the normal controller/bridge safety
chain.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

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
RETIRED_WRITE_MODES = frozenset({"power-sweep", "command-path"})
RETIRED_WRITE_MODE_MESSAGE = (
    "biorola_fault_diag power-sweep/command-path are retired: this diagnostic "
    "is read-only and will not create command publishers. Use snapshot/watch; "
    "follow docs/redrhex_sim2real_sbrio.md for approved power and motion tests."
)
LEG_FIELDS = ("l1", "l2", "l3", "r1", "r2", "r3")
SERVO_FIELDS = ("sl1", "sl2", "sl3", "sr1", "sr2", "sr3")
SERVO_BUS_GROUPS = (
    {
        "bus": "bus1",
        "ros_fields": ("sl1", "sr1"),
        "fpga_encoder_registers": ("ID1EncoderBus1", "ID2EncoderBus1"),
        "fpga_command_registers": ("L1PositionBus1", "R1PositionBus1"),
    },
    {
        "bus": "bus2",
        "ros_fields": ("sl2", "sr2"),
        "fpga_encoder_registers": ("ID1EncoderBus2", "ID2EncoderBus2"),
        "fpga_command_registers": ("L2PositionBus2", "R2PositionBus2"),
    },
    {
        "bus": "bus3",
        "ros_fields": ("sl3", "sr3"),
        "fpga_encoder_registers": ("ID1EncoderBus3", "ID2EncoderBus3"),
        "fpga_command_registers": ("L3PositionBus3", "R3PositionBus3"),
    },
)
POWER_CHANNEL_LABELS = {
    0: "#1",
    1: "L1",
    2: "L2",
    3: "L3",
    4: "R1",
    5: "R2",
    6: "R3",
    7: "Bus",
}


def power_tuple_matches(summary: dict, expected: tuple[bool, bool, bool]) -> bool:
    return bool(
        summary.get("received", False)
        and bool(summary.get("digital", False)) == bool(expected[0])
        and bool(summary.get("signal", False)) == bool(expected[1])
        and bool(summary.get("power", False)) == bool(expected[2])
    )


class ReportWriter:
    def __init__(self, output_root: str, mode: str) -> None:
        root = Path(output_root).expanduser()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = root / f"fault_diag_{stamp}_{mode}"
        suffix = 1
        while path.exists():
            path = root / f"fault_diag_{stamp}_{mode}_{suffix}"
            suffix += 1
        path.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.samples_path = path / "samples.jsonl"

    def write_sample(self, sample: dict) -> None:
        with self.samples_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(sample, sort_keys=True) + "\n")

    def finalize(self, summary: dict, verdict_lines: Iterable[str]) -> None:
        summary = dict(summary)
        summary["report_dir"] = str(self.path)
        with (self.path / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        with (self.path / "verdict.txt").open("w", encoding="utf-8") as handle:
            for line in verdict_lines:
                handle.write(str(line) + "\n")


class BioRoLaFaultDiag(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("biorola_fault_diag")
        self.args = args
        try:
            from rinbo_msgs.msg import MotorCmdStamped, MotorStateStamped, PowerCmdStamped, PowerStateStamped
        except Exception as exc:  # pragma: no cover - requires local ROS overlay
            raise RuntimeError(
                "rinbo_msgs is required. Build/source the workspace first: "
                "source /home/jetson/rinbo_ros_ws/install/setup.bash"
            ) from exc

        self.MotorCmdStamped = MotorCmdStamped
        self.MotorStateStamped = MotorStateStamped
        self.PowerCmdStamped = PowerCmdStamped
        self.PowerStateStamped = PowerStateStamped
        self.report = ReportWriter(args.output_root, args.mode)

        self.last_motor_state = None
        self.last_motor_state_time = None
        self.last_power_state = None
        self.last_power_state_time = None
        self.power_pub = None
        self.motor_pub = None
        self.power_seq = 0
        self.motor_seq = 0
        self.samples: list[dict] = []
        self.events: list[str] = []
        self.errors: list[str] = []
        self.command_path_results: list[dict] = []

        self.create_subscription(self.MotorStateStamped, args.motor_state_topic, self._on_motor_state, 10)
        self.create_subscription(self.PowerStateStamped, args.power_state_topic, self._on_power_state, 10)

    def _on_motor_state(self, msg) -> None:
        self.last_motor_state = msg
        self.last_motor_state_time = time.monotonic()

    def _on_power_state(self, msg) -> None:
        self.last_power_state = msg
        self.last_power_state_time = time.monotonic()

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(float(duration_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_initial_messages(self) -> None:
        deadline = time.monotonic() + max(float(self.args.state_timeout_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.last_motor_state is not None and self.last_power_state is not None:
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def endpoint_summary(self, topic: str) -> dict:
        publishers = self.get_publishers_info_by_topic(topic)
        subscribers = self.get_subscriptions_info_by_topic(topic)

        def summarize(infos) -> list[dict]:
            return [
                {
                    "node_name": info.node_name,
                    "node_namespace": info.node_namespace,
                    "topic_type": info.topic_type,
                    "endpoint_type": str(info.endpoint_type),
                }
                for info in infos
            ]

        return {
            "topic": topic,
            "publisher_count": len(publishers),
            "subscription_count": len(subscribers),
            "publishers": summarize(publishers),
            "subscribers": summarize(subscribers),
        }

    def topic_summary(self) -> dict:
        return {
            "motor_command": self.endpoint_summary(self.args.motor_command_topic),
            "motor_state": self.endpoint_summary(self.args.motor_state_topic),
            "power_command": self.endpoint_summary(self.args.power_command_topic),
            "power_state": self.endpoint_summary(self.args.power_state_topic),
        }

    def servo_positions(self) -> dict[str, int]:
        if self.last_motor_state is None:
            return {field: 0 for field in SERVO_FIELDS}
        return {
            field: int(getattr(getattr(self.last_motor_state, field), "position_encoder", 0))
            for field in SERVO_FIELDS
        }

    def servo_bus_summary(self, positions: dict[str, int]) -> list[dict]:
        groups = []
        for group in SERVO_BUS_GROUPS:
            values = {field: int(positions.get(field, 0)) for field in group["ros_fields"]}
            groups.append(
                {
                    "bus": group["bus"],
                    "ros_fields": list(group["ros_fields"]),
                    "fpga_encoder_registers": list(group["fpga_encoder_registers"]),
                    "fpga_command_registers": list(group["fpga_command_registers"]),
                    "positions": values,
                    "all_zero": all(value == 0 for value in values.values()),
                    "any_nonzero": any(value != 0 for value in values.values()),
                }
            )
        return groups

    def main_motor_positions(self) -> dict[str, float]:
        if self.last_motor_state is None:
            return {field: 0.0 for field in LEG_FIELDS}
        return {
            field: float(getattr(getattr(self.last_motor_state, field), "position", 0.0))
            for field in LEG_FIELDS
        }

    def motor_summary(self) -> dict:
        now = time.monotonic()
        positions = self.servo_positions()
        return {
            "received": self.last_motor_state is not None,
            "age_s": None if self.last_motor_state_time is None else now - self.last_motor_state_time,
            "servo_order": list(SERVO_FIELDS),
            "servo_positions": positions,
            "servo_bus_groups": self.servo_bus_summary(positions),
            "servo_all_zero": all(value == 0 for value in positions.values()),
            "servo_any_nonzero": any(value != 0 for value in positions.values()),
            "servo_control_mode": (
                None if self.last_motor_state is None else int(getattr(self.last_motor_state, "servo_control_mode", 0))
            ),
            "main_motor_positions": self.main_motor_positions(),
        }

    def power_summary(self) -> dict:
        if self.last_power_state is None:
            return {"received": False}
        now = time.monotonic()
        channels = []
        for idx in range(8):
            channels.append(
                {
                    "index": idx,
                    "label": POWER_CHANNEL_LABELS.get(idx, "unknown"),
                    "voltage_v": float(getattr(self.last_power_state, f"v_{idx}", 0.0)),
                    "current_a": float(getattr(self.last_power_state, f"i_{idx}", 0.0)),
                }
            )
        nonzero_voltages = [ch["voltage_v"] for ch in channels if abs(ch["voltage_v"]) > 1.0e-6]
        return {
            "received": True,
            "age_s": None if self.last_power_state_time is None else now - self.last_power_state_time,
            "digital": bool(getattr(self.last_power_state, "digital", False)),
            "signal": bool(getattr(self.last_power_state, "signal", False)),
            "power": bool(getattr(self.last_power_state, "power", False)),
            "clean": bool(getattr(self.last_power_state, "clean", False)),
            "min_nonzero_voltage_v": min(nonzero_voltages) if nonzero_voltages else 0.0,
            "max_abs_current_a": max((abs(ch["current_a"]) for ch in channels), default=0.0),
            "channels": channels,
        }

    def collect_sample(self, label: str) -> dict:
        sample = {
            "label": label,
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "monotonic_s": time.monotonic(),
            "motor": self.motor_summary(),
            "power": self.power_summary(),
        }
        self.samples.append(sample)
        self.report.write_sample(sample)
        return sample

    def sample_for(self, label: str, duration_s: float) -> list[dict]:
        duration_s = max(float(duration_s), 0.0)
        rate_hz = max(float(self.args.sample_hz), 0.2)
        interval_s = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        samples = []
        if duration_s == 0:
            self.spin_for(0.05)
            sample = self.collect_sample(label)
            self.check_sample_guards(sample)
            return [sample]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, interval_s))
            sample = self.collect_sample(label)
            self.check_sample_guards(sample)
            samples.append(sample)
            time.sleep(interval_s)
        return samples

    def check_sample_guards(self, sample: dict) -> None:
        power = sample["power"]
        motor = sample["motor"]
        if not power.get("received", False):
            return
        max_current = float(power.get("max_abs_current_a", 0.0))
        if max_current > self.args.max_current_a:
            raise RuntimeError(
                f"Current guard tripped during {sample['label']}: {max_current:.3f}A > {self.args.max_current_a:.3f}A"
            )
        if power.get("power", False) and motor.get("servo_all_zero", False) and max_current > self.args.zero_servo_current_a:
            raise RuntimeError(
                "Servo readback is all zero while relay is on and current is high "
                f"({max_current:.3f}A > {self.args.zero_servo_current_a:.3f}A)"
            )

    def ensure_no_motor_command_publishers(self) -> None:
        publishers = self.get_publishers_info_by_topic(self.args.motor_command_topic)
        if publishers and not self.args.allow_existing_motor_command_publishers:
            details = ", ".join(f"{info.node_namespace}/{info.node_name}" for info in publishers)
            raise RuntimeError(
                f"Refusing test: existing publishers on {self.args.motor_command_topic}: {details}. "
                "Stop rinbo_cali/rinbo_tripod/RL bridge before running this diagnostic."
            )

    def ensure_power_pub(self) -> None:
        if self.power_pub is None:
            self.power_pub = self.create_publisher(self.PowerCmdStamped, self.args.power_command_topic, 10)
        deadline = time.monotonic() + max(float(self.args.wait_for_subscriber_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.power_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.power_pub.get_subscription_count() == 0 and not self.args.allow_no_subscriber:
            raise RuntimeError(f"No subscriber on {self.args.power_command_topic}. Start rinbo_ros_bridge first.")

    def ensure_motor_pub(self) -> None:
        if self.motor_pub is None:
            self.ensure_no_motor_command_publishers()
            self.motor_pub = self.create_publisher(self.MotorCmdStamped, self.args.motor_command_topic, 10)
        deadline = time.monotonic() + max(float(self.args.wait_for_subscriber_s), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.motor_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.motor_pub.get_subscription_count() == 0 and not self.args.allow_no_subscriber:
            raise RuntimeError(f"No subscriber on {self.args.motor_command_topic}. Start rinbo_ros_bridge first.")

    def build_power_msg(self, digital: bool, signal: bool, power: bool):
        self.power_seq += 1
        msg = self.PowerCmdStamped()
        msg.header.seq = self.power_seq
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "biorola_fault_diag"
        msg.digital = bool(digital)
        msg.signal = bool(signal)
        msg.power = bool(power)
        if hasattr(msg, "clean"):
            msg.clean = False
        if hasattr(msg, "trigger"):
            msg.trigger = False
        return msg

    def publish_power_state(self, state_name: str) -> None:
        self.ensure_power_pub()
        repeat = max(int(self.args.repeat), 1)
        for _ in range(repeat):
            # The bridge rejects duplicate/out-of-order power source headers.
            # Build every retry independently so both sequence and timestamp
            # advance; reusing one sequence would make a valid relay-on retry
            # fail closed to power-off.
            msg = self.build_power_msg(*POWER_STATES[state_name])
            self.power_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(max(float(self.args.repeat_delay_s), 0.0))
        self.events.append(f"published power {state_name}")

    def force_power_off(self) -> bool:
        """Publish fresh off packets and require post-command exact feedback."""

        try:
            command_started = time.monotonic()
            self.publish_power_state("off")
            self.spin_for(0.2)
            sample = self.collect_sample("finally_off")
            received_after_command = (
                self.last_power_state_time is not None
                and self.last_power_state_time >= command_started
            )
            if not received_after_command or not power_tuple_matches(
                sample.get("power", {}), POWER_STATES["off"]
            ):
                raise RuntimeError(
                    "final power-off was not acknowledged after the cleanup command"
                )
            return True
        except BaseException as exc:  # best-effort safety path
            self.errors.append(f"failed to publish final off: {exc}")
            self.get_logger().error(
                f"Failed to verify final power off; relay state is UNKNOWN: {exc}"
            )
            return False

    def build_motor_cmd(self, servo_mode: int):
        self.motor_seq += 1
        cmd = self.MotorCmdStamped()
        cmd.header.seq = self.motor_seq
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "biorola_fault_diag"
        for field in LEG_FIELDS:
            leg = getattr(cmd, field)
            leg.enable = False
            leg.direction = False
            leg.voltage = 0.0
            leg.state = 0
            leg.reset_position = False
        for field, target in self.servo_positions().items():
            getattr(cmd, field).position_encoder = int(target)
        cmd.servo_control_mode = int(servo_mode)
        return cmd

    def publish_motor_mode(self, servo_mode: int, repeat: int | None = None) -> None:
        self.ensure_motor_pub()
        count = max(int(self.args.repeat if repeat is None else repeat), 1)
        for _ in range(count):
            cmd = self.build_motor_cmd(servo_mode)
            self.motor_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(max(float(self.args.repeat_delay_s), 0.0))

    def assert_power_is_off_for_command_path(self) -> None:
        self.wait_for_initial_messages()
        power = self.power_summary()
        if not power.get("received", False):
            raise RuntimeError(f"No message received on {self.args.power_state_topic}; refusing command-path.")
        if power.get("power", False):
            raise RuntimeError("Refusing command-path while power=true. Run biorola_power_tool off first.")

    def run_snapshot(self) -> dict:
        self.wait_for_initial_messages()
        sample = self.collect_sample("snapshot")
        summary = self.build_summary("snapshot", extra={"snapshot": sample, "topics": self.topic_summary()})
        print(json.dumps(summary, indent=2, sort_keys=True))
        return summary

    def run_power_sweep(self) -> dict:
        self.ensure_no_motor_command_publishers()
        stages = ["off", "digital", "sensors"]
        if self.args.include_relay:
            if not self.args.confirm_relay:
                raise RuntimeError("Refusing relay stage without --confirm-relay.")
            stages.append("relay")

        off_verified = False
        try:
            for stage in stages:
                self.get_logger().warn(f"Power sweep stage: {stage}")
                self.publish_power_state(stage)
                self.spin_for(max(float(self.args.settle_s), 0.0))
                duration = self.args.relay_sample_s if stage == "relay" else self.args.sample_s
                self.sample_for(f"power_{stage}", duration)
        finally:
            off_verified = self.force_power_off()

        if not off_verified:
            raise RuntimeError(
                "Power-sweep cleanup did not receive an exact post-command off "
                "acknowledgement; relay state is UNKNOWN."
            )

        return self.build_summary("power-sweep", extra={"stages": stages, "topics": self.topic_summary()})

    def run_command_path(self) -> dict:
        self.assert_power_is_off_for_command_path()
        self.ensure_motor_pub()
        self.publish_motor_mode(0, repeat=8)
        modes = parse_modes(self.args.modes)
        self.collect_sample("command_path_before")
        try:
            for mode in modes:
                self.get_logger().warn(f"Command-path servo_control_mode test: {mode}")
                self.publish_motor_mode(mode)
                deadline = time.monotonic() + max(float(self.args.command_ack_timeout_s), 0.0)
                observed = None
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    motor = self.motor_summary()
                    observed = motor.get("servo_control_mode")
                    if observed == mode:
                        break
                sample = self.collect_sample(f"command_mode_{mode}")
                observed = sample["motor"].get("servo_control_mode")
                self.command_path_results.append(
                    {"requested_mode": mode, "observed_mode": observed, "matched": observed == mode}
                )
        finally:
            try:
                self.publish_motor_mode(0, repeat=8)
                self.spin_for(0.1)
                self.collect_sample("command_path_final_mode_0")
            except Exception as exc:
                self.errors.append(f"failed to publish final servo mode 0: {exc}")

        return self.build_summary(
            "command-path",
            extra={"command_path_results": self.command_path_results, "topics": self.topic_summary()},
        )

    def run_watch(self) -> dict:
        self.wait_for_initial_messages()
        deadline = time.monotonic() + max(float(self.args.duration_s), 0.0)
        interval_s = 1.0 / max(float(self.args.sample_hz), 0.2)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, interval_s))
            sample = self.collect_sample("watch")
            power = sample["power"]
            motor = sample["motor"]
            if power.get("age_s") is not None and power["age_s"] > self.args.stale_s:
                self.events.append(f"power state stale: {power['age_s']:.3f}s")
            if motor.get("age_s") is not None and motor["age_s"] > self.args.stale_s:
                self.events.append(f"motor state stale: {motor['age_s']:.3f}s")
            if power.get("max_abs_current_a", 0.0) > self.args.max_current_a:
                self.events.append(f"current high: {power['max_abs_current_a']:.3f}A")
            if motor.get("servo_all_zero", False):
                self.events.append("servo readback all zero")
            time.sleep(interval_s)
        return self.build_summary("watch", extra={"topics": self.topic_summary()})

    def build_summary(self, mode: str, extra: dict | None = None) -> dict:
        last_sample = self.samples[-1] if self.samples else self.collect_sample("summary")
        summary = {
            "mode": mode,
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "report_dir": str(self.report.path),
            "diagnostic_map": self.diagnostic_map(),
            "latest": last_sample,
            "events": dedupe_preserve_order(self.events),
            "errors": self.errors,
            "verdict": self.verdict_lines(),
            "sample_count": len(self.samples),
        }
        if extra:
            summary.update(extra)
        return summary

    def diagnostic_map(self) -> dict:
        return {
            "servo_bus_groups": [
                {
                    "bus": group["bus"],
                    "ros_fields": list(group["ros_fields"]),
                    "fpga_encoder_registers": list(group["fpga_encoder_registers"]),
                    "fpga_command_registers": list(group["fpga_command_registers"]),
                }
                for group in SERVO_BUS_GROUPS
            ],
            "power_channels": [
                {"index": index, "label": label}
                for index, label in sorted(POWER_CHANNEL_LABELS.items())
            ],
        }

    def verdict_lines(self) -> list[str]:
        latest = self.samples[-1] if self.samples else {"motor": {}, "power": {}}
        motor = latest.get("motor", {})
        power = latest.get("power", {})
        topics = self.topic_summary()
        lines: list[str] = []
        max_current = max(
            (float(sample.get("power", {}).get("max_abs_current_a", 0.0)) for sample in self.samples),
            default=0.0,
        )
        relay_all_zero_high_current = any(
            sample.get("power", {}).get("power", False)
            and sample.get("motor", {}).get("servo_all_zero", False)
            and float(sample.get("power", {}).get("max_abs_current_a", 0.0)) > self.args.zero_servo_current_a
            for sample in self.samples
        )
        main_motion = self.main_motor_motion_summary()

        motor_publishers = [
            pub
            for pub in topics["motor_command"]["publishers"]
            if not (pub["node_name"] == self.get_name() and pub["node_namespace"] == self.get_namespace())
        ]
        for error in self.errors:
            lines.append(f"Diagnostic guard/error: {error}")
        if motor_publishers:
            lines.append("/motor/command has existing publishers; stop controllers before hardware tests.")
        if not motor.get("received", False):
            lines.append("No /motor/state received; bridge or CORE motor-state path is not healthy.")
        if not power.get("received", False):
            lines.append("No /power/state received; bridge or CORE power-state path is not healthy.")
        if motor.get("received", False) and motor.get("servo_all_zero", False):
            lines.append("Main state is present but all servo encoder readbacks are 0; suspect servo readback/power/bus path.")
            lines.append(
                "FPGA bus map is Bus1=sl1/sr1, Bus2=sl2/sr2, Bus3=sl3/sr3; "
                "if only one physical bus pair is connected, only that pair should become nonzero."
            )
        if motor.get("received", False):
            bus_groups = motor.get("servo_bus_groups", [])
            zero_buses = [group.get("bus") for group in bus_groups if group.get("all_zero", False)]
            live_buses = [group.get("bus") for group in bus_groups if group.get("any_nonzero", False)]
            if len(zero_buses) == len(SERVO_BUS_GROUPS):
                lines.append(
                    "All three FPGA servo bus groups report zero; if at least one powered servo pair is connected, "
                    "suspect the shared servo controller/RS485 transceiver/bus power path."
                )
            elif live_buses:
                lines.append(f"Servo bus groups with nonzero readback: {', '.join(live_buses)}.")
        if power.get("power", False) and motor.get("servo_all_zero", False):
            lines.append("Relay is on while servo readback is all zero; do not run calibration/tripod.")
        if relay_all_zero_high_current:
            lines.append(
                "Relay was on while servo readback was all zero and current was elevated "
                f"(max observed {max_current:.3f}A); suspect hardware short/bus/rail fault."
            )
        if main_motion["max_delta_counts"] > self.args.main_motion_threshold_counts:
            lines.append(
                "Main motor encoder positions changed during diagnostic without a motor-command publisher "
                f"(max delta {main_motion['max_delta_counts']:.0f} counts on {main_motion['worst_leg']}); "
                "suspect latched low-level command, power-on transient, or hardware backdrive."
            )
        if self.command_path_results:
            if any(result["matched"] for result in self.command_path_results):
                lines.append("Command-path changed observed servo_control_mode at least once; ROS/CORE command path is partly alive.")
            else:
                lines.append("Command-path did not change observed servo_control_mode; suspect motor command handling path.")
        if not lines:
            lines.append("No immediate software-side fault detected in collected samples.")
        return dedupe_preserve_order(lines)

    def main_motor_motion_summary(self) -> dict:
        if not self.samples:
            return {"max_delta_counts": 0.0, "worst_leg": None, "deltas": {}}
        first_positions = self.samples[0].get("motor", {}).get("main_motor_positions", {})
        deltas = {}
        for field in LEG_FIELDS:
            start = float(first_positions.get(field, 0.0))
            max_delta = 0.0
            for sample in self.samples:
                positions = sample.get("motor", {}).get("main_motor_positions", {})
                max_delta = max(max_delta, abs(float(positions.get(field, start)) - start))
            deltas[field] = max_delta
        worst_leg = max(deltas, key=deltas.get) if deltas else None
        return {
            "max_delta_counts": deltas.get(worst_leg, 0.0) if worst_leg else 0.0,
            "worst_leg": worst_leg,
            "deltas": deltas,
        }

    def run(self) -> dict:
        if self.args.mode in RETIRED_WRITE_MODES:
            raise RuntimeError(RETIRED_WRITE_MODE_MESSAGE)
        if self.args.mode == "snapshot":
            summary = self.run_snapshot()
        elif self.args.mode == "watch":
            summary = self.run_watch()
        else:  # pragma: no cover - argparse constrains choices
            raise RuntimeError(f"Unsupported mode: {self.args.mode}")
        self.report.finalize(summary, summary["verdict"])
        return summary


def dedupe_preserve_order(values: Iterable[str]) -> list[str]:
    seen = set()
    result = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result


def parse_modes(value: str) -> list[int]:
    modes = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        modes.append(int(part))
    if not modes:
        raise RuntimeError("No servo_control_mode values provided.")
    return modes


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only BioRoLaROS2 servo/Scon diagnostics. Use snapshot/watch; "
            "power-sweep and command-path are retired."
        )
    )
    parser.add_argument("mode", choices=["snapshot", "power-sweep", "command-path", "watch"])
    parser.add_argument("--motor-command-topic", default="/motor/command")
    parser.add_argument("--motor-state-topic", default="/motor/state")
    parser.add_argument("--power-command-topic", default="/power/command")
    parser.add_argument("--power-state-topic", default="/power/state")
    parser.add_argument("--output-root", default="~/rinbo_logs")
    parser.add_argument("--state-timeout-s", type=float, default=3.0)
    parser.add_argument("--wait-for-subscriber-s", type=float, default=2.0)
    parser.add_argument("--allow-no-subscriber", action="store_true")
    parser.add_argument("--allow-existing-motor-command-publishers", action="store_true")
    parser.add_argument("--sample-hz", type=float, default=5.0)
    parser.add_argument("--sample-s", type=float, default=2.0)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--settle-s", type=float, default=0.25)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--repeat-delay-s", type=float, default=0.05)
    parser.add_argument("--max-current-a", type=float, default=3.0)
    parser.add_argument("--zero-servo-current-a", type=float, default=1.0)
    parser.add_argument("--main-motion-threshold-counts", type=float, default=100.0)
    parser.add_argument("--stale-s", type=float, default=0.5)
    parser.add_argument("--include-relay", action="store_true")
    parser.add_argument("--confirm-relay", action="store_true")
    parser.add_argument("--relay-sample-s", type=float, default=1.0)
    parser.add_argument("--modes", default="0,2")
    parser.add_argument(
        "--require-power-off",
        action="store_true",
        help="Accepted for explicit command-path calls; command-path always refuses power=true.",
    )
    parser.add_argument("--command-ack-timeout-s", type=float, default=1.0)
    return parser


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)

    # This must precede rclpy.init()/node construction so a retired invocation
    # cannot create either the power or motor command publisher.
    if args.mode in RETIRED_WRITE_MODES:
        raise SystemExit(RETIRED_WRITE_MODE_MESSAGE)

    if args.include_relay or args.confirm_relay:
        raise SystemExit(
            "--include-relay/--confirm-relay belonged to retired power-sweep; "
            "snapshot/watch are read-only."
        )

    rclpy.init()
    node = None
    summary = None
    try:
        node = BioRoLaFaultDiag(args)
        try:
            summary = node.run()
        except RuntimeError as exc:
            node.errors.append(str(exc))
            summary = node.build_summary(args.mode, extra={"topics": node.topic_summary()})
            node.report.finalize(summary, summary["verdict"])
            print(json.dumps(summary, indent=2, sort_keys=True))
            raise
        if args.mode != "snapshot":
            print(json.dumps(summary, indent=2, sort_keys=True))
    except (KeyboardInterrupt, ExternalShutdownException, RCLError) as exc:
        if node is not None:
            node.errors.append("interrupted")
            try:
                summary = node.build_summary(args.mode, extra={"topics": node.topic_summary()})
                node.report.finalize(summary, summary["verdict"])
            except BaseException:
                pass
        raise SystemExit("Fault diagnostic was interrupted and is incomplete.") from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
