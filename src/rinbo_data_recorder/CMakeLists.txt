cmake_minimum_required(VERSION 3.8)
project(rinbo_data_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rinbo_msgs REQUIRED)

add_executable(rinbo_data_recorder src/rinbo_data_recorder.cpp)

ament_target_dependencies(rinbo_data_recorder
  rclcpp
  std_msgs
  rinbo_msgs
)

install(TARGETS
  rinbo_data_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()