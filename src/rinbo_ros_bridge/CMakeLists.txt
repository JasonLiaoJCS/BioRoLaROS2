cmake_minimum_required(VERSION 3.8)
project(rinbo_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rinbo_msgs REQUIRED)

set(INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../../install")
get_filename_component(INSTALL_PREFIX "${INSTALL_PREFIX}" ABSOLUTE)

message(STATUS "INSTALL_PREFIX: ${INSTALL_PREFIX}")

if(NOT EXISTS ${INSTALL_PREFIX})
  message(FATAL_ERROR "INSTALL_PREFIX does not exist: ${INSTALL_PREFIX}")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${INSTALL_PREFIX}/include
  ${CMAKE_SOURCE_DIR}/../../third_party/grpc_core/include
)

file(GLOB ABSL_LIBS "${INSTALL_PREFIX}/lib/libabsl_*.a")
file(GLOB UTF8_LIBS "${INSTALL_PREFIX}/lib/libutf8*.a")
file(GLOB UPB_LIBS "${INSTALL_PREFIX}/lib/libupb*.a")

list(LENGTH ABSL_LIBS ABSL_COUNT)
list(LENGTH UTF8_LIBS UTF8_COUNT)
list(LENGTH UPB_LIBS UPB_COUNT)

message(STATUS "Found ${ABSL_COUNT} ABSL libraries")
message(STATUS "Found ${UTF8_COUNT} UTF8 libraries")
message(STATUS "Found ${UPB_COUNT} UPB libraries")

find_library(GRPC_GRPCPP_LIB
  NAMES grpc++
  PATHS "${INSTALL_PREFIX}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(GRPC_LIB
  NAMES grpc
  PATHS "${INSTALL_PREFIX}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(GPR_LIB
  NAMES gpr
  PATHS "${INSTALL_PREFIX}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(PROTOBUF_LIB
  NAMES protobuf libprotobuf
  PATHS "${INSTALL_PREFIX}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(GRPC_REFLECTION_LIB
  NAMES grpc++_reflection
  PATHS "${INSTALL_PREFIX}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(RE2_LIB re2 PATHS ${INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_library(ADDRESS_SORTING_LIB address_sorting PATHS ${INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_library(CARES_LIB cares PATHS ${INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_library(Z_LIB z PATHS ${INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_library(SSL_LIB ssl PATHS ${INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_library(CRYPTO_LIB crypto PATHS ${INSTALL_PREFIX}/lib NO_DEFAULT_PATH)

find_library(GRPC_PROTO_LIB grpc_proto_lib 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

find_library(MOTOR_PROTO_LIB Motor_proto 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

find_library(POWER_PROTO_LIB Power_proto 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

find_library(STD_GRPC_PROTO_LIB std_grpc_proto 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

find_library(REGISTRATION_GRPC_PROTO_LIB registration_grpc_proto 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

find_library(CONNECTION_GRPC_PROTO_LIB connection_grpc_proto 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

find_library(SERVICESERVING_GRPC_PROTO_LIB serviceserving_grpc_proto 
  PATHS ${INSTALL_PREFIX}/lib 
  NO_DEFAULT_PATH 
  REQUIRED
)

message(STATUS "=== gRPC Core Libraries ===")
message(STATUS "grpc++: ${GRPC_GRPCPP_LIB}")
message(STATUS "grpc: ${GRPC_LIB}")
message(STATUS "gpr: ${GPR_LIB}")
message(STATUS "protobuf: ${PROTOBUF_LIB}")
message(STATUS "grpc++_reflection: ${GRPC_REFLECTION_LIB}")

message(STATUS "=== Custom Proto Libraries ===")
message(STATUS "grpc_proto_lib: ${GRPC_PROTO_LIB}")
message(STATUS "Motor_proto: ${MOTOR_PROTO_LIB}")
message(STATUS "Power_proto: ${POWER_PROTO_LIB}")
message(STATUS "std_grpc_proto: ${STD_GRPC_PROTO_LIB}")
message(STATUS "registration_grpc_proto: ${REGISTRATION_GRPC_PROTO_LIB}")
message(STATUS "connection_grpc_proto: ${CONNECTION_GRPC_PROTO_LIB}")
message(STATUS "serviceserving_grpc_proto: ${SERVICESERVING_GRPC_PROTO_LIB}")

add_executable(rinbo_ros_bridge src/rinbo_ros_bridge.cpp)

target_link_libraries(rinbo_ros_bridge 
  ${GRPC_PROTO_LIB}
  ${MOTOR_PROTO_LIB}
  ${POWER_PROTO_LIB}
  ${STD_GRPC_PROTO_LIB}
  ${REGISTRATION_GRPC_PROTO_LIB}
  ${CONNECTION_GRPC_PROTO_LIB}
  ${SERVICESERVING_GRPC_PROTO_LIB}
  
  # gRPC 核心
  ${GRPC_REFLECTION_LIB}
  ${GRPC_GRPCPP_LIB}
  ${GRPC_LIB}
  ${GPR_LIB}
  
  # Protobuf
  ${PROTOBUF_LIB}
  
  ${RE2_LIB}
  ${ADDRESS_SORTING_LIB}
  ${CARES_LIB}
  
  ${UPB_LIBS}
  
  -Wl,--whole-archive
  ${ABSL_LIBS}
  -Wl,--no-whole-archive
  
  ${UTF8_LIBS}
  
  ${Z_LIB}
  ${SSL_LIB}
  ${CRYPTO_LIB}
  ${CMAKE_DL_LIBS}
  pthread
)

ament_target_dependencies(rinbo_ros_bridge 
  rclcpp 
  rinbo_msgs
)

install(TARGETS
  rinbo_ros_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()