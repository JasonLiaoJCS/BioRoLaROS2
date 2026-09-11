import os
import re
import time
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _sanitize(value):
    value = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip())
    return value or "rinbo_run"


def _unique_run_dir(output_root, run_name, output_dir):
    if output_dir:
        base = os.path.expanduser(output_dir)
        # Share a fresh directory between CSV and bag; never reuse history.
        if os.path.isdir(base) and not os.listdir(base):
            return base
    else:
        output_root = os.path.expanduser(output_root)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(output_root, f"{stamp}_{_sanitize(run_name)}")
    candidate = base
    index = 1
    while True:
        try:
            os.makedirs(candidate, exist_ok=False)
            return candidate
        except FileExistsError:
            candidate = f"{base}_{index}"
            index += 1


def _launch_setup(context):
    output_root = LaunchConfiguration("output_root").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    run_name = _sanitize(LaunchConfiguration("run_name").perform(context))
    profile = LaunchConfiguration("profile").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    start_paused = _as_bool(LaunchConfiguration("start_paused").perform(context))
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    record_csv = _as_bool(LaunchConfiguration("record_csv").perform(context))
    bag_topics = [
        topic.strip()
        for topic in LaunchConfiguration("bag_topics").perform(context).split(",")
        if topic.strip()
    ]

    if "/motor/command" in bag_topics or "/power/command" in bag_topics:
        raise ValueError("diagnostic bag must use monitor topics, not control topics")

    run_dir = _unique_run_dir(output_root, run_name, output_dir)
    raw_bag_dir = os.path.join(run_dir, "raw_bag")

    actions = [
        Node(
            package="rinbo_data_recorder",
            executable="rinbo_data_recorder",
            name="rinbo_data_recorder",
            output="screen",
            parameters=[
                config_file,
                {
                    "output_root": os.path.expanduser(output_root),
                    "output_dir": run_dir,
                    "run_name": run_name,
                    "profile": profile,
                    "auto_start": not start_paused,
                    "command_source": "diagnostic",
                    "record_csv": record_csv,
                    # Keep metadata aligned with the actual rosbag CLI list,
                    # including when the operator overrides bag_topics.
                    "bag_topics": bag_topics,
                },
            ],
        )
    ]

    if record_bag:
        cmd = [
            "ros2", "bag", "record",
            "--max-cache-size", "0",
            "--qos-profile-overrides-path", os.path.join(get_package_share_directory("rinbo_data_recorder"), "config", "diagnostic_bag_qos.yaml"),
            "-o", raw_bag_dir,
        ]
        if start_paused:
            cmd.append("--start-paused")
        cmd.extend(bag_topics)
        actions.append(ExecuteProcess(cmd=cmd, output="screen"))

    print(f"[rinbo_data_recorder] run_dir={run_dir}")
    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("rinbo_data_recorder"),
        "config",
        "logging_tripod_safety.yaml",
    ])

    default_topics = ",".join([
        "/motor/state",
        "/rinbo/monitor/motor_requested",
        "/rinbo/monitor/motor_forwarded",
        "/rinbo/motor_output_enabled",
        "/rinbo/motor_arbiter_ready",
        "/power/state",
        "/pid/data",
        "/rinbo/controller_debug",
        "/rinbo/safety_event",
        "/rinbo/safety_detail",
        "/rosout",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="tripod_safety"),
        DeclareLaunchArgument("run_name", default_value="rinbo_run"),
        DeclareLaunchArgument("output_root", default_value="~/rinbo_logs"),
        DeclareLaunchArgument("output_dir", default_value=""),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("record_csv", default_value="true"),
        DeclareLaunchArgument("start_paused", default_value="false"),
        DeclareLaunchArgument("bag_topics", default_value=default_topics),
        OpaqueFunction(function=_launch_setup),
    ])
