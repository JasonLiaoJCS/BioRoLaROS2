import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_text(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _maybe_add(params: dict, name: str, value: str, value_type):
    text = value.strip()
    if text == "":
        return
    if value_type is bool:
        params[name] = _bool_text(text)
    elif value_type is float:
        params[name] = float(text)
    elif value_type is int:
        params[name] = int(text)
    else:
        params[name] = text


def _csv_list(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _profile_config(profile: str) -> str:
    if profile not in (
        "bench_safe",
        "encoder_only_rig",
        "full_feedback_rig",
        "locomotion_tune",
    ):
        raise RuntimeError(
            "safety_profile must be 'bench_safe', 'encoder_only_rig', "
            "'full_feedback_rig', 'locomotion_tune', or use config:=... with custom."
        )
    return os.path.join(
        get_package_share_directory("redrhex_lowlevel_bridge"),
        "config",
        f"lowlevel_bridge_{profile}.yaml",
    )


def _launch_setup(context, *args, **kwargs):
    safety_profile = LaunchConfiguration("safety_profile").perform(context).strip()
    config = LaunchConfiguration("config").perform(context).strip()
    if safety_profile == "custom":
        if not config:
            raise RuntimeError("safety_profile:=custom requires config:=...")
    elif not config:
        config = _profile_config(safety_profile)
    override_params = {}
    disabled_legs = _csv_list(LaunchConfiguration("disabled_legs").perform(context))
    if disabled_legs:
        override_params["hardware.disabled_legs"] = disabled_legs
    _maybe_add(
        override_params,
        "hardware.max_disabled_legs",
        LaunchConfiguration("max_disabled_legs").perform(context),
        int,
    )

    parameters = [config]
    if override_params:
        parameters.append(override_params)

    return [
        Node(
            package="redrhex_lowlevel_bridge",
            executable="lowlevel_bridge_node",
            name="redrhex_lowlevel_bridge",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "safety_profile",
            default_value="bench_safe",
            description=(
                "bench_safe, encoder_only_rig, full_feedback_rig, "
                "locomotion_tune, or custom."
            ),
        ),
        DeclareLaunchArgument("config", default_value="", description="Optional explicit bridge YAML. Overrides safety_profile."),
        DeclareLaunchArgument("disabled_legs", default_value="", description="Comma-separated physical legs, for example L1. Empty keeps strict mode."),
        DeclareLaunchArgument("max_disabled_legs", default_value="", description="Optional guard; default profile allows one disabled leg."),
        OpaqueFunction(function=_launch_setup),
    ])
