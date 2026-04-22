cmake_minimum_required(VERSION 3.10)
project(controller)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(eigen3_cmake_module QUIET)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

# ---- Executable --------------------------------------------------------------
add_executable(controller_node src/controller_node.cpp)

ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  geometry_msgs
  mav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# ---- Install -----------------------------------------------------------------
install(TARGETS controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---- Export ------------------------------------------------------------------
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_package()
