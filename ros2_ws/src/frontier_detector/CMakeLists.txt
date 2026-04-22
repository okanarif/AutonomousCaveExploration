cmake_minimum_required(VERSION 3.5)
project(frontier_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(octomap REQUIRED)

include_directories(include ${OCTOMAP_INCLUDE_DIRS})

add_executable(frontier_detector_node src/frontier_detector_node.cpp)
ament_target_dependencies(frontier_detector_node
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    octomap_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
)
target_link_libraries(frontier_detector_node ${OCTOMAP_LIBRARIES})

install(TARGETS frontier_detector_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
