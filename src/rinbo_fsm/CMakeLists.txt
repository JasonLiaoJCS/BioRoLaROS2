cmake_minimum_required(VERSION 3.8)
project(rinbo_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rinbo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rinbo_cali src/rinbo_cali.cpp)
ament_target_dependencies(rinbo_cali
  rclcpp
  rinbo_msgs
)

add_executable(rinbo_standing src/rinbo_standing.cpp)
ament_target_dependencies(rinbo_standing
  rclcpp
  rinbo_msgs
)

add_executable(rinbo_tripod src/rinbo_tripod.cpp)
ament_target_dependencies(rinbo_tripod
  rclcpp
  rinbo_msgs
  std_msgs
)

install(TARGETS
  rinbo_cali
  rinbo_standing
  rinbo_tripod 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()