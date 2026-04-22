cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteTrajectory.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
