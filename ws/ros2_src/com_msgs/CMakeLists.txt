cmake_minimum_required(VERSION 3.8)
project(com_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)  # <-- ADD THIS

set(msg_files
  "msg/CompressedData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs  # <-- ADD THIS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
