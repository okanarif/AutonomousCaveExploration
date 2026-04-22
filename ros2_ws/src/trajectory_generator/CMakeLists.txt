cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mission_interfaces REQUIRED)

# Executable
add_executable(trajectory_generation_node src/trajectory_generation_node.cpp)

target_include_directories(trajectory_generation_node PRIVATE
  ${Eigen3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

ament_target_dependencies(trajectory_generation_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  octomap_msgs
  mission_interfaces
)

target_link_libraries(trajectory_generation_node ${OCTOMAP_LIBRARIES})

# Install
install(TARGETS trajectory_generation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
