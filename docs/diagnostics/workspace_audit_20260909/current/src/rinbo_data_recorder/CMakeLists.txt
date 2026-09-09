cmake_minimum_required(VERSION 3.8)
project(rinbo_data_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rinbo_msgs REQUIRED)

add_executable(rinbo_data_recorder src/rinbo_data_recorder.cpp)

ament_target_dependencies(rinbo_data_recorder
  rclcpp
  std_msgs
  std_srvs
  rinbo_msgs
)

install(TARGETS
  rinbo_data_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rinbo_log_control
  DESTINATION bin
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
