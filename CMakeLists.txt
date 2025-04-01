cmake_minimum_required(VERSION 3.0.2)
project(husky_waypoint_nav)

## Python package (no need for C++ flags anymore)
## add_compile_options(-std=c++14)  <-- Not needed for pure Python

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  move_base_msgs
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
  rosconsole
  roslib
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs move_base_msgs actionlib tf2 tf2_ros tf2_geometry_msgs gazebo_msgs roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## No add_executable or target_link_libraries needed for Python scripts
