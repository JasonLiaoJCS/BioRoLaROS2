#!/bin/bash
ros2 run rinbo_ros_bridge rinbo_ros_bridge &
BRIDGE_PID=$!

trap "kill $BRIDGE_PID; exit;" INT
wait $BRIDGE_PID
