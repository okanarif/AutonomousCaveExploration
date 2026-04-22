cmake_minimum_required(VERSION 3.8)
project(lantern_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------
# Find dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# --------------------
# Create executable
# --------------------
add_executable(lantern_detection_node
  src/lantern_detection_node.cpp
)

# --------------------
# Link dependencies
# --------------------
ament_target_dependencies(lantern_detection_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
)


# --------------------
# Install
# --------------------
install(TARGETS lantern_detection_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
# --------------------
# Testing
# --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
