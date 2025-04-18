#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseStamped # Using PoseStamped for subscription
from nav_msgs.msg import Odometry
import math
# import numpy as np # Not used
# import copy # Not used
import os, glob
import datetime # Needed for timestamped log files
# import tf # Not using TF in this version
# import tf.transformations # Not using TF in this version

class HuskyOptiTrackNavigator:
    def __init__(self):
        """Initializes the ROS node, parameters, publishers, subscribers, and logging."""
        rospy.init_node('husky_optitrack_navigator', anonymous=True)

        # --- Get Controller Type Parameter ---
        self.controller_type = rospy.get_param("~controller_type", "Align_Drive_Straight")
        rospy.loginfo(f"Initializing navigator with controller type: {self.controller_type}")

        # --- Publishers and Subscribers ---
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_topic = rospy.get_param("~pose_topic", "/natnet_ros/base_link/pose")
        rospy.Subscriber(self.pose_topic, PoseStamped, self.update_pose)
        rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odometry_callback) # Kept for potential future use

        # --- Robot state ---
        self.current_pose_x = 0.0 # From external pose source (Motive)
        self.current_pose_y = 0.0 # From external pose source (Motive)
        self.current_pose_yaw = 0.0 # From external pose source (Motive)
        self.current_odometry_yaw = 0.0 # Updated by odom callback
        self.pose_received_first_time = False # Flag for initial pose

        # --- Controller tuning ---
        # Use parameters from the last successful run or adjust as needed
        self.max_speed = rospy.get_param("~max_speed", 0.2)
        self.max_turning_speed = rospy.get_param("~max_turning_speed", 1.0)
        self.angular_kp = rospy.get_param("~angular_kp", 0.25) # Keep the lowered gain
        self.linear_kp = rospy.get_param("~linear_kp", 0.5)
        self.dist_tolerance = rospy.get_param("~distance_tolerance", 0.1)
        self.deg_tolerance = rospy.get_param("~angle_tolerance_deg", 10.0)
        self.angle_tolerance_rad = math.radians(self.deg_tolerance)
        self.control_rate = rospy.get_param("~control_rate", 60) # Hz
        self.rate = rospy.Rate(self.control_rate)

        # Log the parameters being used at startup
        rospy.loginfo("--- Controller Parameters ---")
        rospy.loginfo(f"  Controller Type: {self.controller_type}")
        rospy.loginfo(f"  Pose Topic: {self.pose_topic}")
        rospy.loginfo(f"  Max Speed: {self.max_speed} m/s")
        rospy.loginfo(f"  Max Turning Speed: {self.max_turning_speed} rad/s")
        rospy.loginfo(f"  Angular Kp: {self.angular_kp}")
        rospy.loginfo(f"  Linear Kp: {self.linear_kp}")
        rospy.loginfo(f"  Distance Tolerance: {self.dist_tolerance} m")
        rospy.loginfo(f"  Angle Tolerance: {self.deg_tolerance} deg ({self.angle_tolerance_rad:.3f} rad)")
        rospy.loginfo(f"  Control Rate: {self.control_rate} Hz")
        rospy.loginfo("---------------------------")

        # --- Logging Setup ---
        self.log_file = None
        self.log_dir = os.path.expanduser(rospy.get_param("~log_dir", "~/catkin_ws/src/husky_waypoint_nav/logs"))
        self.setup_logging()
        rospy.on_shutdown(self.cleanup)

        # Load waypoints
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
             rospy.logerr("No waypoints loaded. Shutting down.")
             rospy.signal_shutdown("No waypoints loaded.")
             return

        # --- Wait for Initial Pose ---
        rospy.loginfo("Waiting for initial pose on %s...", self.pose_topic)
        while not self.pose_received_first_time and not rospy.is_shutdown():
             rospy.sleep(0.1) # Prevent busy-waiting

        # --- Start Navigation ---
        if self.pose_received_first_time and not rospy.is_shutdown():
            rospy.loginfo(f"Initial pose received: X={self.current_pose_x:.2f}, Y={self.current_pose_y:.2f}, Yaw={math.degrees(self.current_pose_yaw):.1f} deg. Starting navigation.")
            self.move_through_waypoints()
        elif not rospy.is_shutdown():
             rospy.logerr("Failed to get initial pose before shutdown request.")
             rospy.signal_shutdown("Pose init failed.")
        else:
            rospy.logwarn("Shutdown requested before navigation could start.")


    def setup_logging(self):
        """Initializes the CSV log file."""
        if not os.path.exists(self.log_dir):
            try:
                os.makedirs(self.log_dir)
                rospy.loginfo(f"Created log directory: {self.log_dir}")
            except OSError as e:
                 rospy.logerr(f"Failed to create log directory {self.log_dir}: {e}")
                 self.log_file = None
                 return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = os.path.join(self.log_dir, f"navigation_log_{timestamp}.csv")

        try:
            self.log_file = open(log_filename, 'w', buffering=1)
            # CSV Header
            self.log_file.write("timestamp,wp_index,goal_x,goal_y,"
                                "current_x,current_y,current_yaw_rad,"
                                "distance_error,heading_error_rad,"
                                "cmd_linear_x,cmd_angular_z\n")
            rospy.loginfo(f"Logging navigation data to: {log_filename}")
        except IOError as e:
            rospy.logerr(f"Failed to open log file {log_filename}: {e}")
            self.log_file = None


    def log_data(self, wp_index, goal_x, goal_y, cmd_linear_x, cmd_angular_z, distance_error, heading_error_rad):
        """Logs the current state and command to the CSV file."""
        if self.log_file and not self.log_file.closed:
            timestamp = rospy.get_time()
            log_entry = (
                f"{timestamp:.4f},{wp_index},"
                f"{goal_x:.4f},{goal_y:.4f},"
                f"{self.current_pose_x:.4f},{self.current_pose_y:.4f},{self.current_pose_yaw:.4f},"
                f"{distance_error:.4f},{heading_error_rad:.4f},"
                f"{cmd_linear_x:.4f},{cmd_angular_z:.4f}\n"
            )
            try:
                self.log_file.write(log_entry)
            except IOError as e:
                rospy.logwarn_throttle(10.0, f"Failed to write to log file: {e}")


    def cleanup(self):
        """Closes the log file and stops the robot on shutdown."""
        rospy.loginfo("Shutting down navigator node.")
        stop_cmd = Twist()
        try:
             if hasattr(self, 'velocity_publisher') and self.velocity_publisher.get_num_connections() > 0:
                 self.velocity_publisher.publish(stop_cmd)
                 rospy.sleep(0.1)
                 self.velocity_publisher.publish(stop_cmd)
                 rospy.loginfo("Stop command sent.")
             else:
                 rospy.logwarn("Velocity publisher not available or not connected, cannot send stop command.")
        except Exception as e:
             rospy.logwarn(f"Could not publish stop command during shutdown: {e}")

        if self.log_file and not self.log_file.closed:
            rospy.loginfo(f"Closing log file: {self.log_file.name}")
            self.log_file.close()
            self.log_file = None


    def update_pose(self, data):
        """Callback function for pose updates from the subscribed PoseStamped topic."""
        # Removed latency calculation logs

        # Update pose state variables
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        orientation_q = data.pose.orientation
        _, _, self.current_pose_yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
        # Set flag on first successful pose update
        if not self.pose_received_first_time:
            self.pose_received_first_time = True


    def odometry_callback(self, data):
         """Callback function for Husky odometry updates."""
         orientation_q = data.pose.pose.orientation
         _, _, self.current_odometry_yaw = self.quaternion_to_euler(
             orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
         )


    def quaternion_to_euler(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw) in radians."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z


    def load_waypoints(self):
        """Loads waypoints (X, Y) from the specified text file."""
        wp_dir_param = rospy.get_param("~waypoint_dir", "~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        wp_prefix = rospy.get_param("~waypoint_prefix", "optitrack_waypoints_")
        wp_dir = os.path.expanduser(wp_dir_param)

        files = sorted(glob.glob(os.path.join(wp_dir, f"{wp_prefix}*.txt")))
        if not files:
            rospy.logerr(f"[ERROR] No waypoint files matching '{wp_prefix}*.txt' found in directory: {wp_dir}")
            return []

        latest_file = files[-1]
        waypoints = []
        rospy.loginfo(f"Attempting to load waypoints from: {latest_file}")
        line_num = 0
        try:
            with open(latest_file, 'r') as f:
                for line in f:
                    line_num += 1
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    try:
                        parts = list(map(float, line.split()))
                        if len(parts) == 2:
                            x, y = parts
                            waypoints.append((x, y))
                        else:
                             rospy.logwarn(f"Skipping malformed line {line_num} in {latest_file}: '{line}' (Expected 2 values: X Y)")
                    except ValueError:
                        rospy.logwarn(f"Skipping non-numeric line {line_num} in {latest_file}: '{line}'")
                        continue
            rospy.loginfo(f"[INFO] Loaded {len(waypoints)} waypoints (X, Y) from {latest_file}")
        except IOError as e:
             rospy.logerr(f"Error reading waypoint file {latest_file}: {e}")
             return []
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred loading waypoints: {e}")
            return []
        return waypoints


    def move_through_waypoints(self):
        """Executes the Align-Then-Drive-Straight navigation logic."""
        if not self.waypoints:
            rospy.logwarn("No waypoints loaded, cannot navigate.")
            return

        for wp_index, (goal_x, goal_y) in enumerate(self.waypoints):
            # Log start of waypoint navigation
            rospy.loginfo(f"--- Moving to Waypoint {wp_index + 1}/{len(self.waypoints)}: Goal=({goal_x:.2f}, {goal_y:.2f}) ---")

            while not rospy.is_shutdown():
                if not self.pose_received_first_time:
                    # This condition should ideally not be hit often after initial wait
                    rospy.logwarn_throttle(5.0,"Waiting for pose update...")
                    self.rate.sleep()
                    continue

                # --- Calculations ---
                current_x = self.current_pose_x
                current_y = self.current_pose_y
                current_yaw = self.current_pose_yaw
                distance_error = math.hypot(goal_x - current_x, goal_y - current_y)

                # Check for goal achievement
                if distance_error < self.dist_tolerance:
                    rospy.loginfo(f"[SUCCESS] Reached Waypoint {wp_index + 1}. Final Pose=({current_x:.3f}, {current_y:.3f}, {math.degrees(current_yaw):.1f} deg). Dist Error: {distance_error:.3f}m")
                    break # Exit this waypoint's loop

                # Calculate heading error
                yaw_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
                yaw_error = self.normalize_angle(yaw_to_goal - current_yaw)

                # --- Align-Then-Drive Logic ---
                cmd = Twist()
                # Determine action based on yaw error and tolerance
                if abs(yaw_error) > self.angle_tolerance_rad:
                    # ALIGN phase
                    cmd.linear.x = 0.0
                    cmd.angular.z = max(min(self.angular_kp * yaw_error, self.max_turning_speed), -self.max_turning_speed)
                else:
                    # DRIVE phase
                    cmd.linear.x = min(self.max_speed, self.linear_kp * distance_error)
                    cmd.angular.z = 0.0 # Force straight

                # --- Detailed console log REMOVED ---

                # Log Data to CSV file (remains)
                self.log_data(
                    wp_index=wp_index + 1, goal_x=goal_x, goal_y=goal_y,
                    cmd_linear_x=cmd.linear.x, cmd_angular_z=cmd.angular.z,
                    distance_error=distance_error, heading_error_rad=yaw_error
                )

                # Publish command and sleep
                self.velocity_publisher.publish(cmd)
                self.rate.sleep()
                # --- Loop End ---

            # --- Post-Waypoint Actions ---
            self.velocity_publisher.publish(Twist()) # Stop command
            if not rospy.is_shutdown():
                rospy.loginfo(f"--- Pausing at Waypoint {wp_index + 1} ---")
                rospy.sleep(0.5) # Pause briefly
            else:
                 # If shutdown happened during the inner while loop
                 rospy.logwarn("Shutdown requested during waypoint approach.")
                 break # Exit the main waypoint FOR loop

        # --- End of All Waypoints ---
        if not rospy.is_shutdown():
            rospy.loginfo("[DONE] Finished all waypoints.")
        else:
            rospy.loginfo("[ABORTED] Waypoint navigation aborted due to shutdown request.")

        # Final stop command
        self.velocity_publisher.publish(Twist())


    def normalize_angle(self, angle):
        """Normalize angle to be within the range [-pi, pi] radians."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

# --- Main Execution Block ---
if __name__ == '__main__':
    try:
        navigator = HuskyOptiTrackNavigator()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down navigator node.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the navigator: {e}")
        import traceback
        traceback.print_exc()