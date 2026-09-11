#!/usr/bin/env bash
set -eo pipefail
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export PYTHONPATH="$PWD/src/rinbo_control${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m rinbo_control.panel_server "$@"
