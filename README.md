
# Husky Waypoint Navigation (ROS Noetic)

This package provides custom Python scripts to record, manage, and execute waypoint-based navigation for a Clearpath Husky robot using OptiTrack pose data. Works in both Gazebo simulation and real-world scenarios.

---

## 🚀 Features

- Collect waypoints via PS4 controller (real-time)
- Save & replay waypoints in a loop or one-shot mode
- Visualize waypoints in RViz
- Integrates with OptiTrack motion capture (via `natnet_ros_cpp`)
- Supports both **simulation** and **real robot** modes

---

## 🧱 Dependencies

This package depends on the following (install manually or as submodules if needed):

- ✅ [Clearpath's Husky stack]:
  - `husky_description`
  - `husky_control`
  - `husky_gazebo`
  - `husky_navigation`
- ✅ `natnet_ros_cpp` (for OptiTrack)
- ✅ `joy`, `teleop_twist_joy` (for PS4 joystick control)
- ✅ `robot_localization`
- ✅ `tf`, `rviz`, `actionlib`, `move_base` (if enabled)

---

## 📂 Folder Structure

```bash
husky_waypoint_nav/
├── config/                    # Waypoint files & config
├── launch/                   # sim_nav.launch, real_nav.launch
├── scripts/                  # Python versions of all ROS nodes
├── src/                      # (can be removed if no C++ left)
├── package.xml
├── CMakeLists.txt
🎮 Controller Mapping
L1 + joystick = Drive

R1 = Turbo

L2 = Mark waypoint

R2 = Stop recording

Triangle = Pause playback

Circle = Resume playback

🧪 Simulation & Real Usage
🔧 Launch in Simulation
bash
Copy
Edit
roslaunch husky_waypoint_nav sim_nav.launch
🛰️ Launch on Real Robot
bash
Copy
Edit
roslaunch husky_waypoint_nav real_nav.launch
Waypoints are saved to:

arduino
Copy
Edit
config/optitrack_waypoints.txt
👀 RViz
Visual tools to:

View robot and waypoints

Confirm pose updates from /natnet_ros/Husky/pose

Watch live path execution

Add or review waypoints using our visualization tools

⚙️ Maintainer Notes
Installing Dependencies
bash
Copy
Edit
sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy \
                 ros-noetic-robot-localization ros-noetic-rviz
And install or clone Clearpath’s husky_* packages into your workspace manually
