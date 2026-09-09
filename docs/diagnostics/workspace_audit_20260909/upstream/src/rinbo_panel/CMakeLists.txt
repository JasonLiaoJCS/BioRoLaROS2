cmake_minimum_required(VERSION 3.8)
project(rinbo_panel)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/rinbo_control_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()