cmake_minimum_required(VERSION 3.0.2)
project(costmap_compress_decompress)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS iostreams)

catkin_package(
  CATKIN_DEPENDS roscpp visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(costmap_compression_node src/costmap_compression.cpp)
target_link_libraries(costmap_compression_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(costmap_decompression_node src/costmap_decompression.cpp)
target_link_libraries(costmap_decompression_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
