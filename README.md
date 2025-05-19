# Husky Waypoint Navigation

This package enables the Clearpath Husky UGV to navigate to a series of waypoints using pose data from an OptiTrack Motion Capture (MoCap) system. It includes P and Full PD controllers, with options for logging MoCap and odometry data for analysis.

# Notes on Changes
Latest waypoints and logs are ready for FCRAR paper. The `FullPDNavigator.py` (Full Proportional-Derivative controller) and `p_nav_with_odom_logs.py` (Proportional controller logging odometry) are the primary scripts used. Waypoint files are expected in `X Y` format per line. Need to post-process.

# How to launch

### 1. Launch OptiTrack Node (NatNet)
Ensure your OptiTrack system is running and streaming data. Then, launch the NatNet ROS node:
Make sure to set `world_frame` in the NatNet launch file (e.g., `gui_natnet_ros.launch` or your specific file) to `"map"` or your desired fixed world frame.

    roslaunch natnet_ros_cpp gui_natnet_ros.launch

### 2. RViz Check (Important!)
Always launch RViz to verify your TF tree and pose data:
   - Start RViz: `rviz`
   - Set the "Fixed Frame" to your MoCap world frame (e.g., `"map"`).
   - Add a "TF" display to visualize coordinate frames.
   - Add a "PoseStamped" display subscribing to your MoCap topic (e.g., `/natnet_ros/base_link/pose`) to see the robot's pose.
   - **Expected Robot `base_link` Orientation (relative to its own movement):**
     - **Red arrow (X-axis):** Points directly forward from the robot.
     - **Green arrow (Y-axis):** Points to the robot's left.
     - **Blue arrow (Z-axis):** Points upwards.
If any of these appear incorrect, re-configure the Rigid Body orientation in the OptiTrack Motive software to align with the standard ROS convention (X forward, Y left, Z up).

### 3. (Optional) To Collect New Waypoints
If you need to collect new waypoints (which should be in X Y format, one pair per line):
*(Note: Ensure the `collect_optitrack_waypoints.py` script saves in X Y format or adjust the `load_waypoints` function in the navigator scripts accordingly. The current navigators expect X Y.)*

    rosrun husky_waypoint_nav collect_optitrack_waypoints.py

### 4. Launch Waypoint Navigation Controllers

The controllers use waypoint files (e.g., `optitrack_waypoints_06.txt`) located in the `config/waypoints/` directory of this package. These files should contain X and Y coordinates, one waypoint per line, space-separated.

#### Option A: Full PD Controller (MoCap-based, Logs PD Terms - PREFERRED FOR PERFORMANCE)
This controller uses Proportional-Derivative control for both linear and angular motion based on MoCap feedback. It logs detailed performance data including MoCap pose, target, errors, commands, and the P/D components of the control signals.

* **Script:** `FullPDNavigator.py`
* **Launch file:** `FullPDNavigator.launch` (ensure this name matches your file)

    ```bash
    roslaunch husky_waypoint_nav full_pd_navigator.launch
    ```

* **Key Configurable Parameters (via launch file arguments):**
    * `mocap_topic_name`: (Default: `/natnet_ros/base_link/pose`) Topic for MoCap pose input.
    * `cmd_vel_topic_name`: (Default: `/husky_velocity_controller/cmd_vel`) Topic for robot velocity commands.
    * `waypoint_filename_arg`: (Default: `optitrack_waypoints_06.txt`) Name of the waypoint file.
    * `log_dir_arg`: (Default: `$(find husky_waypoint_nav)/logs`) Directory for CSV logs.
    * `linear_kp_gain`, `linear_kd_gain`: Gains for linear P and D control.
    * `angular_kp_gain`, `angular_kd_gain`: Gains for angular P and D control.
    * `max_lin_vel_arg`, `max_ang_vel_arg`: Maximum linear and angular velocities.
    * `pos_tol_arg`: Position tolerance (meters) for reaching a waypoint.
    * `angle_tol_arg`: Angle tolerance (radians) - used for D-term history reset context.
    * `rate_hz_arg`: Control loop frequency (Hz).

* **Example with Parameter Overrides:**
    ```bash
    roslaunch husky_waypoint_nav full_pd_navigator.launch linear_kp_gain:=0.6 linear_kd_gain:=0.15 max_lin_vel_arg:=0.4 waypoint_filename_arg:="my_other_waypoints.txt"
    ```

#### Option B: P Controller (MoCap-based, Logs MoCap & Odometry - FOR DRIFT ANALYSIS)
This controller uses Proportional control for both linear and angular motion based on MoCap feedback. It is configured to log both the MoCap pose (ground truth) and the robot's internal odometry (`/odom`) simultaneously. This is useful for analyzing odometry drift.

* **Script:** `p_nav_with_odom_logs.py` (or similar name you saved it as)
* **Launch file:** `p_nav_odom_log.launch` (ensure this name matches your file that launches the correct script)

    ```bash
    roslaunch husky_waypoint_nav p_nav_odom_log.launch
    ```

* **Key Configurable Parameters (via launch file arguments for this specific launch file):**
    * `mocap_pose_topic`: MoCap pose topic (for control and logging).
    * `odom_topic`: Odometry topic to log (e.g., `/odom` or `/husky_velocity_controller/odom`).
    * `cmd_vel_topic`: Robot velocity command topic.
    * `waypoint_file`: Name of the waypoint file.
    * `log_file_path` (or `log_dir` depending on your launch file arg name): Directory for CSV logs.
    * `linear_kp`, `angular_kp`: Proportional gains.
    * `max_linear_vel`, `max_angular_vel`: Maximum velocities.
    * `pos_tolerance`, `angle_tolerance`: Tolerances.
    * `rate`: Control loop frequency.

* **Example with Parameter Overrides (ensure arg names match your launch file):**
    ```bash
    # Example assuming launch file args are 'linear_kp_gain', 'odom_topic_name' etc.
    roslaunch husky_waypoint_nav p_nav_odom_log.launch linear_kp_gain:=0.55 odom_topic_name:=/odom
    ```

# Debugging Tips

## Time issues due to offline errors
If you run into time sync errors (e.g., timestamps from 1969/1970 in logs or TF warnings), run the following when connected to the internet:

    sudo apt install ntp
    sudo systemctl enable ntp
    sudo systemctl start ntp
    sudo timedatectl set-ntp true

Then, to sync hardware clock (optional, but good practice):

    sudo hwclock --systohc

Check with `timedatectl status` to confirm NTP sync is active.