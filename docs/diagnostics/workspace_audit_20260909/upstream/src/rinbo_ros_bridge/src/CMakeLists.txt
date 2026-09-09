# Find required custom libraries
find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
    message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

# Define the executable and link required libraries
add_executable(rinbo_ros_bridge ./rinbo_ros_bridge.cpp)
target_link_libraries(rinbo_ros_bridge ${CMAKE_DL_LIBS} ${_CORE_LIBRARIES} ${GRPC_PROTO_LIB})

ament_target_dependencies(rinbo_ros_bridge rclcpp rinbo_msgs)
