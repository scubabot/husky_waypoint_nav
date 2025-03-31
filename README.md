ROS package for waypoint navigation using OptiTrack pose data and joystick control, designed for the Clearpath Husky robot in both simulation and real-world environments.

---

FEATURES:
- Waypoint Collection via PS4 controller
- Live Navigation through collected or predefined waypoints
- RViz visualization of path
- Simulation mode using Gazebo
- Real-world mode using NatNet and OptiTrack
- Modular Python scripts
- Optional live pose logging and plotting

---

DIRECTORY STRUCTURE:

husky_waypoint_nav/
├── config/
│   └── optitrack_waypoints.txt
├── launch/
│   ├── sim_nav.launch
│   └── real_nav.launch
├── src/
│   ├── collect_optitrack_waypoints.py
│   ├── optitrack_waypoint_navigator.py
│   ├── gazebo_to_pose_bridge.py
│   ├── waypoint_visualizer.py
│   └── generate_dummy_waypoints.py
├── CMakeLists.txt
├── package.xml

---

DEPENDENCIES:
- husky (Clearpath Robotics repo) - NOT tracked in this repo
- natnet_ros - NOT tracked in this repo
- robot_localization
- joy
- teleop_twist_joy
- gazebo_ros
- rviz

Install dependencies manually into your catkin workspace as needed.

---

CONTROLLER MAPPING (PS4):
- Drive: L1 + joystick
- Turbo: R1 + joystick
- Collect Waypoint: L2
- End Collection: R2
- Pause Navigation: Triangle
- Resume Navigation: Circle

---

TO RUN SIMULATION:
roslaunch husky_waypoint_nav sim_nav.launch

TO RUN ON REAL ROBOT:
roslaunch husky_waypoint_nav real_nav.launch

Make sure /natnet_ros/Husky/pose is available.

---

RVIZ:
Waypoint markers are published to /waypoint_markers. Green spheres represent navigation targets.

---

INSTALLATION:
cd ~/catkin_ws/src
git clone git@github.com:scubabot/husky_waypoint_nav.git
cd ..
catkin_make
source devel/setup.bash

Make sure other external packages are cloned separately.

---

MAINTAINER:
Daniel G. (FAU Robotics)

---

LICENSE:
MIT or compatible
