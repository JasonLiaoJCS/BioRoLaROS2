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
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(robot_config STATIC src/robot_config.cpp)
target_include_directories(robot_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
ament_target_dependencies(robot_config rclcpp)
target_link_libraries(robot_config yaml-cpp OpenSSL::Crypto)

add_executable(rinbo_legs src/rinbo_legs.cpp)
target_link_libraries(rinbo_legs robot_config)

add_executable(rinbo_manual src/rinbo_manual.cpp)
target_link_libraries(rinbo_manual robot_config)
ament_target_dependencies(rinbo_manual rclcpp rcl_interfaces rinbo_msgs std_msgs)

add_executable(rinbo_cali src/rinbo_cali.cpp)
target_link_libraries(rinbo_cali robot_config)
ament_target_dependencies(rinbo_cali
  rclcpp
  rcl_interfaces
  rinbo_msgs
  std_msgs
)

add_executable(rinbo_standing src/rinbo_standing.cpp)
target_link_libraries(rinbo_standing robot_config)
ament_target_dependencies(rinbo_standing
  rclcpp
  rcl_interfaces
  rinbo_msgs
  std_msgs
)

add_executable(rinbo_tripod src/rinbo_tripod.cpp)
target_link_libraries(rinbo_tripod robot_config)
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
  rinbo_legs
  rinbo_manual
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
    ENV ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
  )
  target_include_directories(test_motor_arbiter_handshake PRIVATE src)
  ament_target_dependencies(test_motor_arbiter_handshake
    rclcpp
    rinbo_msgs
    std_msgs
  )
  ament_add_gtest(test_latest_state_safety
    test/test_latest_state_safety.cpp
    ENV ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
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
  foreach(config_test robot_config cali_multileg standing_multileg tripod_multileg manual_motion manual_controller control_velocity motion_effort)
    ament_add_gtest(test_${config_test}
      test/test_${config_test}.cpp
      ENV ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1
      TIMEOUT 60
    )
    target_include_directories(test_${config_test} PRIVATE src)
    target_compile_definitions(test_${config_test} PRIVATE
      RINBO_FSM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(test_${config_test} robot_config)
    ament_target_dependencies(test_${config_test}
      rclcpp rcl_interfaces rinbo_msgs std_msgs
    )
  endforeach()
endif()

ament_package()
