#!/usr/bin/env bash
# Build candidate and simulated tests only. Never run the motion executable.
set -eo pipefail
. /opt/ros/humble/setup.bash
. /home/jetson/rinbo_ros_ws/install/setup.bash
cmake -S /home/jetson/rinbo_ros_ws/src/rinbo_fsm -B /tmp/rinbo-tripod-fix-build -DBUILD_TESTING=ON
cmake --build /tmp/rinbo-tripod-fix-build --target rinbo_tripod rinbo_legs test_tripod_multileg test_robot_config test_cali_multileg test_standing_multileg -j2
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
ctest --test-dir /tmp/rinbo-tripod-fix-build -R 'test_(tripod|cali|standing)_multileg$' --output-on-failure
/tmp/rinbo-tripod-fix-build/test_robot_config --gtest_filter='ConfigTest.TripodTuning*'
