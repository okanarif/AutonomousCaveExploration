cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mission_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable
add_executable(mission_control_node src/mission_control_node.cpp)
ament_target_dependencies(mission_control_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mission_interfaces
  Eigen3
)

# Install
install(TARGETS mission_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
