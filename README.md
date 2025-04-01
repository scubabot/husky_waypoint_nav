THIS ASSUMES YOU HAVE CLEARPATH PACKAGES INSTALLED AT CATKIN_WS/SRC


# Husky Waypoint Navigation Package

This package enables waypoint collection and autonomous waypoint-based navigation using a Husky robot with pose data from OptiTrack. Designed to work in both simulation (Gazebo) and real-world setups.

---

## 💻 System Requirements
- ROS Noetic
- Ubuntu 20.04
- Docker container (recommended)
- Catkin workspace structure

---

## 📦 Package Contents
- `collect_optitrack_waypoints.py` — collects waypoints using a PS4 controller.
- `optitrack_waypoint_navigator.py` — sends navigation goals to `move_base` or direct `/cmd_vel`.
- `gazebo_to_pose_bridge.py` — simulates OptiTrack pose in Gazebo.
- `generate_dummy_waypoints.py` — testing tool for fake waypoint generation.
- `waypoint_visualizer.py` — displays waypoints as RViz markers.

---

## 🚀 Setup

### Clone into Catkin Workspace:
```bash
=======
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
>>>>>>> b84cbc0daa515b2881a09594754c4f3cc2012d95
cd ~/catkin_ws/src
git clone git@github.com:scubabot/husky_waypoint_nav.git
cd ..
catkin_make
<<<<<<< HEAD
```

### Install Dependencies:
```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
                     ros-noetic-interactive-markers ros-noetic-twist-mux \
                     ros-noetic-move-base ros-noetic-nav-msgs \
                     ros-noetic-actionlib ros-noetic-tf
```

---

## 🐳 Docker Notes

### Common Docker Commands
Start container:
```bash
docker start -ai ros_noetic_dev
```
Open new terminal into container:
```bash
docker exec -it ros_noetic_dev bash
```

Always remember to:
```bash
source ~/catkin_ws/devel/setup.bash
```

---

## 🕹️ Real Robot Launch
Run the system with the real Husky robot and OptiTrack:
```bash
roslaunch husky_waypoint_nav real_nav.launch
```

### PS4 Controls
- **Drive:** L1 + left joystick
- **Turbo:** R1
- **Mark Waypoint:** L2
- **Stop Collection:** R2
- **Pause Navigation:** Triangle
- **Resume Navigation:** Circle

---

## 🧪 Simulation Launch
Run everything in Gazebo with pose simulation:
```bash
roslaunch husky_waypoint_nav sim_nav.launch
```

You can still use L2/R2 to mark waypoints, and view everything in RViz.

---

## 📍 Visualizing Waypoints in RViz
Launch RViz with the correct config:
```bash
rviz -d $(rospack find husky_viz)/rviz/navigation.rviz
```

Ensure the `waypoint_markers` topic is visible.

---

## 🛠️ Helpful Commands

### Debugging Transforms
```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo base_link odom
```

### View Published Topics
```bash
rostopic list
```

### Check if `cmd_vel` is being published:
```bash
rostopic echo /cmd_vel
rostopic info /cmd_vel
```

### Check if pose is received from OptiTrack:
```bash
rostopic echo /natnet_ros/Husky/pose
```

---

## 📁 File Logging
Pose logs are saved to:
```bash
pose_logs/pose_###.txt
```
Visualizations are saved to:
```bash
pose_logs/pose_###_plot.png
```
These are ignored in Git via `.gitignore`.

---

## 🧼 Package Cleanup
Make sure `.gitignore` excludes non-essential folders:
- `pose_logs/`
- `husky/`, `natnet_ros_cpp/`, etc. (installed externally or separately)

---

## 🧪 Testing Checklist
- [ ] Can launch sim and real nav files
- [ ] Can mark waypoints with L2
- [ ] Can stop collection with R2
- [ ] Can visualize waypoints in RViz
- [ ] Navigator follows path or loops correctly
- [ ] No transform errors in TF tree
- [ ] System works with and without `move_base`

---

## 🌐 GitHub Integration

If you're setting this up as a fresh repo:
```bash
git init
git remote add origin git@github.com:your_username/husky_waypoint_nav.git
git add .
git commit -m "Initial commit"
git push -u origin main
```

If the remote repo already has files:
```bash
git pull origin main --allow-unrelated-histories --no-rebase
# Resolve conflicts, if any
# Then:
git push -u origin main
```

---

## 📦 External Dependencies
This package assumes:
- A Clearpath-provided `husky` description and control stack.
- A working `natnet_ros` or `natnet_ros_cpp` package for OptiTrack integration.

Make sure to install or clone them into your workspace.

---
=======
source devel/setup.bash

Make sure other external packages are cloned separately.

---

MAINTAINER:
Daniel G. (FAU Robotics)

---

LICENSE:
MIT or compatible
>>>>>>> b84cbc0daa515b2881a09594754c4f3cc2012d95
