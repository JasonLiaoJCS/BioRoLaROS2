#!/usr/bin/env bash
set -eo pipefail
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
exec /home/jetson/rinbo_ros_ws/install/rinbo_data_recorder/lib/rinbo_data_recorder/rinbo_data_recorder --ros-args -p auto_start:=false -p command_source:=diagnostic
