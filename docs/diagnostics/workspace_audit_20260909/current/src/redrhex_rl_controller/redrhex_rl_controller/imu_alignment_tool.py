"""Read-only suspended-upright IMU alignment sampler."""

from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .observation_builder import (
    _normalize_quat_xyzw,
    _quat_inverse_xyzw,
    _quat_multiply_xyzw,
    _source_stamp_to_float,
)


class _Sampler(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("redrhex_imu_alignment_sampler")
        self.topic = topic
        self.quaternions: list[np.ndarray] = []
        self.gyros: list[np.ndarray] = []
        self.frame_ids: list[str] = []
        self.source_stamps: list[float] = []
        self.invalid_samples = 0
        self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)

    def _on_imu(self, msg: Imu) -> None:
        q = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            dtype=np.float64,
        )
        gyro = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float64,
        )
        stamp = _source_stamp_to_float(msg)
        frame_id = str(msg.header.frame_id).strip()
        orientation_covariance = list(msg.orientation_covariance)
        angular_velocity_covariance = list(msg.angular_velocity_covariance)
        if (
            stamp is None
            or not frame_id
            or (
                orientation_covariance
                and float(orientation_covariance[0]) < 0.0
            )
            or (
                angular_velocity_covariance
                and float(angular_velocity_covariance[0]) < 0.0
            )
            or not np.isfinite(q).all()
            or not np.isfinite(gyro).all()
            or abs(float(np.linalg.norm(q)) - 1.0) > 0.10
        ):
            self.invalid_samples += 1
            return
        self.quaternions.append(_normalize_quat_xyzw(q))
        self.gyros.append(gyro)
        self.frame_ids.append(frame_id)
        self.source_stamps.append(stamp)


def _hemisphere_average(quaternions: list[np.ndarray]) -> np.ndarray:
    reference = quaternions[0]
    aligned = [q if float(np.dot(q, reference)) >= 0.0 else -q for q in quaternions]
    return _normalize_quat_xyzw(np.mean(np.stack(aligned), axis=0))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Sample a stationary suspended-upright IMU; prints JSON and never edits YAML."
    )
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=200)
    parser.add_argument("--max-gyro-rms-rad-s", type=float, default=0.03)
    parser.add_argument("--expected-publisher", default="")
    args = parser.parse_args(argv)
    if args.duration_s <= 0.0 or args.min_samples < 10:
        raise ValueError("duration must be positive and min-samples must be >= 10")

    rclpy.init()
    node = _Sampler(args.topic)
    try:
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        publishers = node.get_publishers_info_by_topic(args.topic)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(publishers) != 1:
        raise RuntimeError(
            f"{args.topic} must have exactly one publisher, got {len(publishers)}"
        )
    publisher = str(publishers[0].node_name).lstrip("/")
    if args.expected_publisher and publisher != args.expected_publisher.lstrip("/"):
        raise RuntimeError(
            f"publisher {publisher!r} != expected {args.expected_publisher!r}"
        )
    if len(node.quaternions) < args.min_samples:
        raise RuntimeError(
            f"only {len(node.quaternions)} valid samples; need {args.min_samples}"
        )
    if len(set(node.frame_ids)) != 1:
        raise RuntimeError(f"IMU frame_id changed during capture: {sorted(set(node.frame_ids))}")
    stamps = np.asarray(node.source_stamps, dtype=np.float64)
    if not np.all(np.diff(stamps) > 0.0):
        raise RuntimeError("IMU source stamps are not strictly monotonic")
    gyro = np.stack(node.gyros)
    gyro_rms = float(math.sqrt(float(np.mean(np.square(gyro)))))
    if gyro_rms > args.max_gyro_rms_rad_s:
        raise RuntimeError(
            f"robot/IMU was not stationary: gyro RMS {gyro_rms:.6f} rad/s"
        )

    q_world_sensor = _hemisphere_average(node.quaternions)
    q_world_policy_upright = np.array(
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)], dtype=np.float64
    )
    q_sensor_policy = _normalize_quat_xyzw(
        _quat_multiply_xyzw(
            _quat_inverse_xyzw(q_world_sensor), q_world_policy_upright
        )
    )
    print(
        json.dumps(
            {
                "read_only": True,
                "sample_count": len(node.quaternions),
                "invalid_sample_count": node.invalid_samples,
                "publisher": publisher,
                "expected_imu_frame_id": node.frame_ids[0],
                "gyro_rms_rad_s": gyro_rms,
                "imu_policy_to_sensor_quaternion_xyzw": q_sensor_policy.tolist(),
                "imu_upright_quaternion_xyzw": q_world_policy_upright.tolist(),
                "imu_alignment_calibrated": False,
                "instruction": "Review/repeat the measurement before manually setting calibrated=true.",
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
