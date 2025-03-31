cmake_minimum_required(VERSION 3.0.2)
project(husky_waypoint_nav)

## Compile as C++14
add_compile_options(-std=c++14)

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

## Waypoint collection node
add_executable(collect_optitrack_waypoints src/collect_optitrack_waypoints.cpp)
target_link_libraries(collect_optitrack_waypoints ${catkin_LIBRARIES})

## Waypoint navigator node
add_executable(optitrack_waypoint_navigator src/optitrack_waypoint_navigator.cpp)
target_link_libraries(optitrack_waypoint_navigator ${catkin_LIBRARIES})

## Gazebo simulation bridge node
add_executable(gazebo_to_pose_bridge src/gazebo_to_pose_bridge.cpp)
target_link_libraries(gazebo_to_pose_bridge ${catkin_LIBRARIES})
