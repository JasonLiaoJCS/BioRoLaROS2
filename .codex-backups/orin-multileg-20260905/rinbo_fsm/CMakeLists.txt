cmake_minimum_required(VERSION 3.8)
project(rinbo_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rinbo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rinbo_cali src/rinbo_cali.cpp)
ament_target_dependencies(rinbo_cali
  rclcpp
  rcl_interfaces
  rinbo_msgs
  std_msgs
)

add_executable(rinbo_standing src/rinbo_standing.cpp)
ament_target_dependencies(rinbo_standing
  rclcpp
  rcl_interfaces
  rinbo_msgs
  std_msgs
)

add_executable(rinbo_tripod src/rinbo_tripod.cpp)
ament_target_dependencies(rinbo_tripod
  rclcpp
  rcl_interfaces
  rinbo_msgs
  std_msgs
)

# Preserve the name only as a fail-before-ROS retirement stub.  Older build and
# install trees contained a writable sine-sweep binary even after its source
# target was removed, so omitting the target is not sufficient to retire it.
add_executable(rinbo_sin_sweep src/rinbo_sin_sweep.cpp)

install(TARGETS
  rinbo_cali
  rinbo_standing
  rinbo_tripod
  rinbo_sin_sweep
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_arbiter_handshake
    test/test_motor_arbiter_handshake.cpp
    ENV ROS_DOMAIN_ID=231
  )
  target_include_directories(test_motor_arbiter_handshake PRIVATE src)
  ament_target_dependencies(test_motor_arbiter_handshake
    rclcpp
    rinbo_msgs
    std_msgs
  )
  ament_add_gtest(test_latest_state_safety
    test/test_latest_state_safety.cpp
    ENV ROS_DOMAIN_ID=231
    TIMEOUT 60
  )
  target_include_directories(test_latest_state_safety PRIVATE src)
  target_compile_definitions(test_latest_state_safety PRIVATE
    RINBO_FSM_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"
  )
  ament_target_dependencies(test_latest_state_safety
    rclcpp
    rcl_interfaces
    rinbo_msgs
  )
  add_test(
    NAME test_rinbo_sin_sweep_retired
    COMMAND ${CMAKE_COMMAND}
      -DPROGRAM=$<TARGET_FILE:rinbo_sin_sweep>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/assert_sin_sweep_retired.cmake
  )
endif()

ament_package()
