cmake_minimum_required(VERSION 3.8)
project(com_cpp)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(com_msgs REQUIRED)

# Compressor executable
add_executable(universal_compressor src/universal_compressor.cpp)
target_link_libraries(universal_compressor
  Boost::iostreams
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(universal_compressor
  com_msgs
  rclcpp
  nav_msgs
  visualization_msgs
  std_msgs
)

# Decompressor executable
add_executable(universal_decompressor src/universal_decompressor.cpp)
target_link_libraries(universal_decompressor
  Boost::iostreams
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(universal_decompressor
  com_msgs
  rclcpp
  nav_msgs
  visualization_msgs
  std_msgs
)

# Hz monitor executable
add_executable(hz_monitor src/hz_monitor.cpp)
target_link_libraries(hz_monitor
  Boost::iostreams
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(hz_monitor
  com_msgs
  rclcpp
  nav_msgs
  visualization_msgs
  std_msgs
)

# Install executables
install(TARGETS universal_compressor universal_decompressor hz_monitor
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()