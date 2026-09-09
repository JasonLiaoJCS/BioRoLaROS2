cmake_minimum_required(VERSION 3.8)
project(rinbo_pid_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rinbo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rinbo_pid_test src/rinbo_pid_test.cpp)
ament_target_dependencies(rinbo_pid_test rclcpp rinbo_msgs std_msgs)

add_executable(rinbo_traj_rslip src/rinbo_traj_rslip.cpp)
ament_target_dependencies(rinbo_traj_rslip rclcpp rinbo_msgs std_msgs)

install(TARGETS 
  rinbo_pid_test
  rinbo_traj_rslip
  DESTINATION lib/${PROJECT_NAME})

ament_package()