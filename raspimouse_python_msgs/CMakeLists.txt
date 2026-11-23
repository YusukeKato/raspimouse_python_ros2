cmake_minimum_required(VERSION 3.8)
project(raspimouse_python_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DriveDistance.action"
  "action/Rotate.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
