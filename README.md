
---

```markdown
# husky_waypoint_nav

ROS package for waypoint navigation using OptiTrack pose data and joystick control, designed for the Clearpath Husky robot in both simulation and real-world environments.

---

## 🚀 Features

- **Waypoint Collection** via PS4 controller buttons
- **Live Navigation** through pre-defined or collected waypoints
- **Visualization** in RViz
- **Simulation Mode** using Gazebo with a simulated OptiTrack pose feed
- **Real-World Mode** using NatNet OptiTrack data
- **Modular Python scripts** for easy development and extensibility
- **Pose logging** and optional live plotting
- **Waypoint visualizer** to preview saved paths

---

## 📁 Package Layout

```
husky_waypoint_nav/
├── config/
│   └── optitrack_waypoints.txt   # Default waypoint file
├── launch/
│   ├── sim_nav.launch            # Simulation launch
│   └── real_nav.launch           # Real-world launch
├── src/
│   ├── collect_optitrack_waypoints.py
│   ├── optitrack_waypoint_navigator.py
│   ├── gazebo_to_pose_bridge.py
│   ├── waypoint_visualizer.py
│   └── generate_dummy_waypoints.py
├── CMakeLists.txt
├── package.xml
```

---

## 🧠 Dependencies

| Package                   | Purpose                             |
|--------------------------|-------------------------------------|
| `husky` (Clearpath)      | Robot model and controllers         |
| `natnet_ros`             | OptiTrack pose streaming            |
| `robot_localization`     | EKF-based odometry fusion           |
| `joy` / `teleop_twist_joy` | PS4 controller input               |
| `gazebo_ros`             | Simulation (Gazebo + ROS interface) |
| `rviz`                   | Visualization of robot and poses    |

> Note: `husky` and `natnet_ros` are external packages and **should not be tracked in this repo**. Please clone them separately into your workspace.

---

## 🕹 Controller Layout (PS4)

| Action               | Button |
|----------------------|--------|
| Drive (normal)       | L1 + joystick |
| Drive (turbo)        | R1 + joystick |
| Collect Waypoint     | L2 |
| End Collection       | R2 |
| Pause Navigation     | Triangle |
| Resume Navigation    | Circle |

---

## 🧪 Running in Simulation

```bash
roslaunch husky_waypoint_nav sim_nav.launch
```

## 🤖 Running on Real Robot

```bash
roslaunch husky_waypoint_nav real_nav.launch
```

Make sure the OptiTrack streaming is active and `/natnet_ros/Husky/pose` is being published.

---

## 📍 Visualizing Waypoints

Waypoint markers are published to the topic:

```
/waypoint_markers
```

They appear as green spheres in RViz. You can preview your path before execution.

---

## 🛠 Installation

```bash
cd ~/catkin_ws/src
git clone git@github.com:scubabot/husky_waypoint_nav.git
cd ..
catkin_make
source devel/setup.bash
```

> ⚠ Ensure you’ve installed external dependencies (`husky`, `natnet_ros`, etc.) beforehand.

---

## ✅ TODO & Roadmap

- [x] Collect waypoints with PS4
- [x] Convert all scripts to Python
- [x] Visualize waypoints in RViz
- [ ] Add obstacle-aware local planner
- [ ] Integrate dynamic replanning

---

## 🧑‍💻 Maintainer

**Daniel G.**  
FAU Robotics Research  
Email: [add your contact]

---

## 📜 License

MIT License 
```

---