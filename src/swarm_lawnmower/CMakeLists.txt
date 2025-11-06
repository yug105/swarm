cmake_minimum_required(VERSION 3.8)
project(swarm_lawnmower)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${px4_ros2_cpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build the executable
add_executable(swarm_node
  swarm_drone.cpp
)

ament_target_dependencies(swarm_node
  rclcpp
  px4_ros2_cpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  Eigen3
)

# ✅ Install target (this was missing before)
install(TARGETS swarm_node
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ✅ Install headers (optional but good practice)
install(FILES swarm_drone.hpp
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
