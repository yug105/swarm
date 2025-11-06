cmake_minimum_required(VERSION 3.8)
project(swarm_offboard)

find_package(ament_cmake REQUIRED)

# Install your Python script
install(PROGRAMS
  swarm_offboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
