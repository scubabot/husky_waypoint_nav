# husky_nav_ros2

This ROS 2 package provides simple waypoint navigation and path recording functionalities for a Husky A200 robot, utilizing a PD controller and motion capture data.

## Table of Contents

1.  [Package Overview](#package-overview)
2.  [Dependencies](#dependencies)
3.  [Setup and Building](#setup-and-building)
    * [Prerequisites](#prerequisites)
    * [Creating the Workspace](#creating-the-workspace)
    * [Adding Scripts to the Package](#adding-scripts-to-the-package)
    * [Building the Package](#building-the-package)
    * [Sourcing the Workspace](#sourcing-the-workspace)
4.  [Running the Nodes](#running-the-nodes)
    * [Prerequisite Nodes](#prerequisite-nodes)
    * [Launching the Waypoint Recorder](#launching-the-waypoint-recorder)
    * [Launching the Waypoint Navigator](#launching-the-waypoint-navigator)
5.  [Parameters](#parameters)
    * [Modifying Parameters](#modifying-parameters)
    * [WaypointNavigatorPD Parameters (`navigation_simple.py`)](#waypointnavigatorpd-parameters-navigation_simplepy)
    * [WaypointPathRecorderSimplified Parameters (`collection_simple.py`)](#waypointpathrecordersimplified-parameters-collection_simplepy)
6.  [Waypoint File Format (`waypoints.csv`)](#waypoint-file-format-waypointscsv)
7.  [Troubleshooting](#troubleshooting)

## 1. Package Overview

This package contains three primary Python scripts:

* **`robot_math_utils.py`**: A utility script providing common mathematical functions for robotics, such as quaternion to yaw conversion, angle normalization, and 2D distance calculation. This is imported by other scripts and is not a standalone ROS 2 node.
* **`navigation_simple.py` (Node: `simple_navigator_node`)**: Implements a simple PD controller for waypoint navigation. It subscribes to the robot's pose (e.g., from a motion capture system) and publishes velocity commands to drive the robot through a series of waypoints loaded from a CSV file.
* **`collection_simple.py` (Node: `simple_collector_node`)**: Facilitates the recording of a robot's path (trajectory) using joystick inputs. It subscribes to the robot's pose and joystick commands, allowing you to start/stop recording and then processes/saves the recorded path as sparse waypoints in a CSV file.

## 2. Dependencies

This package depends on the following ROS 2 packages and Python libraries:

**ROS 2 Packages:**
* `rclpy` (ROS 2 Python client library)
* `std_msgs` (Standard ROS 2 message types, e.g., String)
* `geometry_msgs` (ROS 2 messages for poses, twists, quaternions, etc.)
* `sensor_msgs` (ROS 2 messages for sensor data, e.g., Joy)
* `builtin_interfaces` (ROS 2 messages for time, duration)
* `nav_msgs` (ROS 2 messages for navigation, e.g., Odometry, Path - general dependency, though not strictly imported in these scripts)
* `tf2_ros`, `tf2_geometry_msgs` (ROS 2 TF2 for transformations - general dependency)
* `nav2_msgs`, `amcl_msgs`, `map_msgs` (Navigation2 specific messages - general dependency)
* `urdf`, `xacro`, `joint_state_publisher`, `robot_state_publisher` (Robot description/state publishing - general dependency)

**Python Libraries:**
* `python3-numpy` (Numerical computing)
* `python3-yaml` (YAML parsing, if using YAML parameter files)
* `setuptools` (Python packaging tool, managed by apt/pip)

## 3. Setup and Building

### Prerequisites

* **ROS 2 Humble Hawksbill** installed on Ubuntu 22.04 (preferably within WSL2 on Windows, or natively on Linux).
* **`colcon`** and `rosdep` tools installed.
* A working `natnet_ros2` package (or another source of `/natnet_ros/base_link/pose` messages).
* A working `joy_node` (for joystick input, e.g., `ros2 run joy joy_node`).

### Creating the Workspace

If you don't already have `~/ros2_ws`, follow these steps to create a clean workspace and your package:

```bash
# Phase 1: Absolute Environment Reset (in a brand new terminal)
# This clears Python user-installed packages/caches that can cause conflicts
cd ~
rm -rf ~/.local/lib/python3.10/site-packages/
rm -rf ~/.cache/pip/
rm -rf ~/.cache/setuptools/
sudo apt update
sudo apt install --reinstall python3-setuptools python3-wheel python3-pip

# Phase 2: Create a new ROS 2 workspace
rm -rf ~/ros2_ws # Ensures no old workspace remains
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the husky_nav_ros2 package
ros2 pkg create --build-type ament_python husky_nav_ros2
Adding Scripts to the Package
Place your Python script files (robot_math_utils.py, navigation_simple.py, collection_simple.py) into the inner husky_nav_ros2 directory (~/ros2_ws/src/husky_nav_ros2/husky_nav_ros2/).

```bash
# Navigate to the inner module directory
cd ~/ros2_ws/src/husky_nav_ros2/husky_nav_ros2/

# Create/Paste your scripts here (using gedit, nano, or cp from a backup)
gedit robot_math_utils.py # Paste content here
gedit navigation_simple.py # Paste content here
gedit collection_simple.py # Paste content here

# IMPORTANT: Modify import statements in navigation_simple.py and collection_simple.py
# Change: `from robot_math_utils import ...`
# To:     `from .robot_math_utils import ...`
# This tells Python to look within the same package.
```
Building the Package
After placing scripts and modifying package.xml/setup.py (see next section), build your workspace.

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Clean old build artifacts and build (do this every time you modify code or config)
rm -rf build install log
colcon build
```
Sourcing the Workspace
After each successful build, you must source your workspace to make its packages and nodes available to ROS 2. Do this in every new terminal you open for ROS 2.

```bash
source /opt/ros/humble/setup.bash # Source your main ROS 2 distro (Humble)
source install/setup.bash          # THEN source your workspace
```
4. Running the Nodes
Prerequisite Nodes
For these husky_nav_ros2 nodes to function, you'll typically need:

Motion Capture Pose Publisher: A node publishing `geometry_msgs/PoseStamped` on `/natnet_ros/base_link/pose` (e.g., your `natnet_ros2` node).
Joystick Driver (for Recorder): A `joy_node` publishing `sensor_msgs/Joy` on `/joy`. You can run it with: `ros2 run joy joy_node` (You might need to install `ros-humble-joy` or `ros-humble-joy-linux` if you don't have it).
Robot Base Controller: Your Husky's base controller should be subscribing to `/cmd_vel` (a `geometry_msgs/Twist` message) to receive movement commands.
Launching the Waypoint Recorder (simple_collector_node)
This node saves waypoints to waypoints.csv.

```bash
ros2 run husky_nav_ros2 simple_collector_node
```
Follow the on-screen instructions regarding joystick buttons (default: Button 0 to START, Button 1 to STOP).
The waypoints.csv file will be created in your current working directory when the node is stopped. Consider moving it to your package's share directory for better management.
Launching the Waypoint Navigator (simple_navigator_node)
This node uses waypoints.csv to drive the robot.

```bash
ros2 run husky_nav_ros2 simple_navigator_node
```
Ensure a waypoints.csv file exists in the directory where you launch the node, or provide its full path if it's elsewhere.
5. Parameters
Parameters allow you to configure the behavior of your nodes without modifying their source code.

Modifying Parameters
Parameters can be changed in a few ways:

Directly in the script (not recommended for deployment): Modify the default values in the __init__ method of your node classes. Requires rebuilding.
Command Line (for quick testing): Append --ros-args -p <param_name>:=<value> to your ros2 run command.
Bash

ros2 run husky_nav_ros2 simple_navigator_node --ros-args -p max_linear_speed:=0.2
YAML File (Recommended for consistency): Create a YAML file (e.g., my_params.yaml) and load it when launching the node.
YAML

# my_params.yaml
waypoint_navigator_pd_node: # Node name as defined in super().__init__
  ros__parameters:
    goal_xy_tolerance: 0.1
    max_linear_speed: 0.25
    # ... other parameters
Then, launch with:
Bash

ros2 run husky_nav_ros2 simple_navigator_node --ros-args --params-file path/to/my_params.yaml
Launch File (Best for system deployment): Define parameters directly within a Python launch file. (See ROS 2 Launch documentation for details).
WaypointNavigatorPD Parameters (navigation_simple.py)
Node name in __init__: waypoint_navigator_pd_node

goal_xy_tolerance (float, default: 0.08): Distance (in meters) to consider the robot "at" the waypoint's X,Y coordinates.
goal_final_theta_tolerance (float, default: 0.08): Angle (in radians) to consider the robot "aligned" with the waypoint's final orientation.
approach_heading_tolerance (float, default: 0.2): Looser angular tolerance while approaching a waypoint's X,Y. If robot's heading deviates more than this, it re-aligns.
max_linear_speed (float, default: 0.15): Maximum linear speed (m/s) the robot will command.
max_angular_speed (float, default: 0.30): Maximum angular speed (rad/s) the robot will command.
linear_Kp (float, default: 0.5): Proportional gain for linear velocity control.
angular_Kp (float, default: 0.9): Proportional gain for angular velocity control.
linear_Kd (float, default: 0.15): Derivative gain for linear velocity control.
angular_Kd (float, default: 0.1): Derivative gain for angular velocity control.
WAYPOINTS_FILENAME (string, default: "waypoints.csv"): The name of the CSV file from which waypoints are loaded. This file is expected to be in the directory where the node is launched, or its full path must be provided.
WaypointPathRecorderSimplified Parameters (collection_simple.py)
Node name in __init__: waypoint_path_recorder_simplified_node

JOYSTICK_BUTTON_START_RECORDING (int, default: 0): The index of the joystick button to start recording the path.
JOYSTICK_BUTTON_STOP_RECORDING (int, default: 1): The index of the joystick button to stop recording and save the path.
WAYPOINTS_FILENAME (string, default: "waypoints.csv"): The name of the CSV file where recorded waypoints will be saved. The file will be created in the directory where the node is launched.
6. Waypoint File Format (waypoints.csv)
The waypoints.csv file should be a plain text file with each line representing a waypoint. Each line should contain three comma-separated floating-point values: x,y,theta.

Example waypoints.csv:

```csv
0.0,0.0,0.0
1.0,0.0,1.57
1.0,1.0,3.14
0.0,1.0,-1.57
```
x: X-coordinate in meters.
y: Y-coordinate in meters.
theta: Orientation (yaw) in radians, ranging from -PI to PI.
7. Troubleshooting
ModuleNotFoundError or No executable found:
Ensure you have sourced your workspace (`source install/setup.bash`) in your current terminal.
Verify your `package.xml` and `setup.py` are correct and you've rebuilt (`colcon build`) after any changes.
Ensure your Python scripts (`.py` files) are in the inner `husky_nav_ros2/husky_nav_ros2/` directory.
Check for correct relative imports (`from .robot_math_utils import ...`).
Node fails to initialize (no INFO messages):
Check the node's initial `get_logger().info()` calls.
Ensure all ROS 2 dependencies are correctly listed in `package.xml` and installed (run `rosdep install --from-paths src --ignore-src -r` from workspace root).
Robot not moving/not recording:
Verify prerequisite nodes are running (`ros2 node list`).
Check topic connections (`ros2 topic info <topic_name>`, `ros2 topic echo <topic_name>`). Ensure messages are being published/subscribed correctly.
Tune PD controller parameters (for navigator).
Check joystick button mappings (for recorder).