cmake_minimum_required(VERSION 3.0.2)
project(color_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_filters sensor_msgs nav_msgs geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(color_map_lib src/color_map/color.cpp)
target_include_directories(color_map_lib PUBLIC include/color_map)

add_executable(color_map src/main.cpp)
target_link_libraries(color_map color_map_lib)