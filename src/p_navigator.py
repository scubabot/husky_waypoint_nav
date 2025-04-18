#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import copy
import os, glob
import datetime # Needed for timestamped log files
# import atexit # rospy.on_shutdown is preferred

class HuskyOptiTrackNavigator:
    def __init__(self):
        rospy.init_node('husky_optitrack_navigator', anonymous=True)

        # --- Get Controller Type Parameter ---
        self.controller_type = rospy.get_param("~controller_type", "P_Controller")
        rospy.loginfo(f"Initializing navigator with controller type: {self.controller_type}")

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.update_pose)
        rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odometry_callback)

        # Robot state
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.current_odometry_yaw = 0.0 # Still useful for relative checks or logging if needed

        # Controller tuning (Now using parameters with Kp naming)
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_turning_speed = rospy.get_param("~max_turning_speed", 1.0)
        # Renamed gain parameters to use Kp convention
        self.angular_kp = rospy.get_param("~angular_kp", 1.5) # Proportional gain for angular velocity
        self.linear_kp = rospy.get_param("~linear_kp", 0.5)   # Proportional gain for linear velocity
        self.dist_tolerance = rospy.get_param("~distance_tolerance", 0.15)
        self.deg_tolerance = rospy.get_param("~angle_tolerance_deg", 3.0)
        self.angle_tolerance_rad = math.radians(self.deg_tolerance)
        self.control_rate = rospy.get_param("~control_rate", 60)
        self.rate = rospy.Rate(self.control_rate)

        # Log the parameters being used
        rospy.loginfo("--- Controller Parameters ---")
        rospy.loginfo(f"  Controller Type: {self.controller_type}")
        rospy.loginfo(f"  Max Speed: {self.max_speed}")
        rospy.loginfo(f"  Max Turning Speed: {self.max_turning_speed}")
        rospy.loginfo(f"  Angular Kp: {self.angular_kp}")
        rospy.loginfo(f"  Linear Kp: {self.linear_kp}")
        rospy.loginfo(f"  Distance Tolerance: {self.dist_tolerance}")
        rospy.loginfo(f"  Angle Tolerance (deg): {self.deg_tolerance}")
        rospy.loginfo(f"  Control Rate: {self.control_rate}")
        rospy.loginfo("---------------------------")


        self.prev_cmd = Twist() # Can be used for smoothing or rate limiting if needed later

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

        rospy.loginfo("Waiting for initial pose...")
        # Wait for the first pose message to initialize current_pose
        while self.current_pose_x == 0.0 and self.current_pose_y == 0.0 and not rospy.is_shutdown():
             try:
                 rospy.wait_for_message('/natnet_ros/base_link/pose', PoseStamped, timeout=1.0)
             except rospy.ROSException:
                 rospy.logwarn_throttle(5.0, "Still waiting for initial pose from OptiTrack...")
             rospy.sleep(0.1) # Prevent busy-waiting

        if not rospy.is_shutdown():
            rospy.loginfo("Initial pose received. Starting navigation.")
            self.move_through_waypoints()
        else:
            rospy.logwarn("Shutdown requested before navigation could start.")


    def setup_logging(self):
        """Initializes the log file."""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            rospy.loginfo(f"Created log directory: {self.log_dir}")

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = os.path.join(self.log_dir, f"navigation_log_{timestamp}.csv")

        try:
            self.log_file = open(log_filename, 'w')
            # Write header row
            self.log_file.write("timestamp,wp_index,goal_x,goal_y,goal_yaw_rad,"
                                "current_x,current_y,current_yaw_rad,"
                                "distance_error,heading_error_rad,"
                                "cmd_linear_x,cmd_angular_z\n")
            rospy.loginfo(f"Logging navigation data to: {log_filename}")
        except IOError as e:
            rospy.logerr(f"Failed to open log file {log_filename}: {e}")
            self.log_file = None

    def log_data(self, wp_index, goal_x, goal_y, goal_yaw, cmd_linear_x, cmd_angular_z, distance_error, heading_error_rad):
        """Logs the current state and command to the CSV file."""
        if self.log_file and not self.log_file.closed:
            timestamp = rospy.get_time() # Use ROS time for consistency
            log_entry = (
                f"{timestamp:.4f},{wp_index},"
                f"{goal_x:.4f},{goal_y:.4f},{goal_yaw:.4f},"
                f"{self.current_pose_x:.4f},{self.current_pose_y:.4f},{self.current_pose_yaw:.4f},"
                f"{distance_error:.4f},{heading_error_rad:.4f},"
                f"{cmd_linear_x:.4f},{cmd_angular_z:.4f}\n"
            )
            try:
                self.log_file.write(log_entry)
            except IOError as e:
                rospy.logwarn_throttle(10.0, f"Failed to write to log file: {e}") # Throttle warnings

    def cleanup(self):
        """Closes the log file and stops the robot on shutdown."""
        rospy.loginfo("Shutting down navigator node.")
        # Ensure robot stops
        stop_cmd = Twist()
        self.velocity_publisher.publish(stop_cmd)
        rospy.sleep(0.1) # Give a moment for the command to be sent
        self.velocity_publisher.publish(stop_cmd) # Send again just in case

        if self.log_file and not self.log_file.closed:
            rospy.loginfo(f"Closing log file: {self.log_file.name}")
            self.log_file.close()
            self.log_file = None

    def update_pose(self, data):
        """Callback function for OptiTrack pose updates."""
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        orientation_q = data.pose.orientation
        _, _, self.current_pose_yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

    def odometry_callback(self, data):
         """Callback function for Husky odometry updates (primarily for yaw)."""
         # While OptiTrack provides absolute yaw, odom yaw can be useful for
         # relative checks or as a fallback if OptiTrack data is noisy/lost.
         orientation_q = data.pose.pose.orientation
         _, _, self.current_odometry_yaw = self.quaternion_to_euler(
             orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
         )
         # You could potentially use odom velocity here too if needed:
         # current_linear_vel = data.twist.twist.linear.x
         # current_angular_vel = data.twist.twist.angular.z

    def quaternion_to_euler(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw)."""
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

        return roll_x, pitch_y, yaw_z # in radians

    def load_waypoints(self):
        # Now using parameters for waypoint location
        wp_dir_param = rospy.get_param("~waypoint_dir", "~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        wp_prefix = rospy.get_param("~waypoint_prefix", "optitrack_waypoints_")
        wp_dir = os.path.expanduser(wp_dir_param)

        files = sorted(glob.glob(os.path.join(wp_dir, f"{wp_prefix}*.txt")))
        if not files:
            rospy.logerr(f"[ERROR] No waypoint files matching '{wp_prefix}*.txt' found in: {wp_dir}")
            return []

        latest_file = files[-1]
        waypoints = []
        try:
            with open(latest_file, 'r') as f:
                for i, line in enumerate(f):
                    # Skip empty lines or lines starting with # (comments)
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    try:
                        parts = list(map(float, line.split()))
                        if len(parts) == 3:
                            x, y, yaw_rad = parts # Read directly as radians
                            waypoints.append((x, y, yaw_rad)) # Append directly
                        elif len(parts) == 2:
                             x, y = parts
                             waypoints.append((x, y, 0.0)) # Default to 0 yaw rad
                             rospy.logwarn_throttle(5.0, f"Waypoint {i+1} in {latest_file} only has X, Y. Assuming 0 yaw goal.")
                        else:
                             rospy.logwarn(f"Skipping malformed line {i+1} in {latest_file}: '{line.strip()}' (Expected 2 or 3 values)")

                    except ValueError:
                        rospy.logwarn(f"Skipping non-numeric line {i+1} in {latest_file}: '{line.strip()}'")
                        continue
            rospy.loginfo(f"[INFO] Loaded {len(waypoints)} waypoints from {latest_file}")
        except IOError as e:
             rospy.logerr(f"Error reading waypoint file {latest_file}: {e}")
             return []

        return waypoints


    def move_through_waypoints(self):
        if not self.waypoints:
            rospy.logwarn("No waypoints to navigate.")
            return

        # --- Main Navigation Loop ---
        for wp_index, (goal_x, goal_y, goal_yaw) in enumerate(self.waypoints): # goal_yaw is in RADIANS
            rospy.loginfo(f"--- Moving to Waypoint {wp_index + 1}: ({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_yaw):.1f} deg) ---")

            # --- Phase 1: Move towards X, Y position ---
            while not rospy.is_shutdown():
                # Calculate errors
                # Renamed 'distance' to 'distance_error' for clarity
                distance_error = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                yaw_to_goal = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x) # Radians
                yaw_error = self.normalize_angle(yaw_to_goal - self.current_pose_yaw) # Radians

                # Check if goal position reached
                if distance_error < self.dist_tolerance:
                    rospy.loginfo(f"[SUCCESS] Reached position for waypoint {wp_index + 1} (Dist Error: {distance_error:.3f} < {self.dist_tolerance})")
                    break

                # Calculate control commands (P-controller logic using Kp gains)
                cmd = Twist()
                # Prioritize turning if angle error is large
                if abs(yaw_error) > self.angle_tolerance_rad: # Compare radians
                    # Only turn
                    cmd.linear.x = 0.0
                    # Use angular_kp
                    cmd.angular.z = max(min(self.angular_kp * yaw_error, self.max_turning_speed), -self.max_turning_speed)
                else:
                    # Move forward and make small angle corrections
                    # Use linear_kp and distance_error
                    cmd.linear.x = min(self.max_speed, self.linear_kp * distance_error)
                    # Use angular_kp
                    cmd.angular.z = max(min(self.angular_kp * yaw_error, self.max_turning_speed), -self.max_turning_speed)

                # --- Log Data ---
                self.log_data(
                    wp_index=wp_index + 1,
                    goal_x=goal_x,
                    goal_y=goal_y,
                    goal_yaw=goal_yaw, # Radians
                    cmd_linear_x=cmd.linear.x,
                    cmd_angular_z=cmd.angular.z,
                    distance_error=distance_error, # Use renamed variable
                    heading_error_rad=yaw_error # Radians
                )

                self.velocity_publisher.publish(cmd)
                self.rate.sleep()

            # Stop movement before final alignment (if not already stopped by shutdown)
            if not rospy.is_shutdown():
                self.velocity_publisher.publish(Twist())
                rospy.sleep(0.1) # Allow robot to settle

            # Check for shutdown signal before starting Phase 2
            if rospy.is_shutdown():
                rospy.logwarn("Shutdown requested during waypoint transition.")
                break

            # --- Phase 2: Final Yaw Alignment ---
            rospy.loginfo(f"Aligning to final yaw ({math.degrees(goal_yaw):.1f} deg) for waypoint {wp_index + 1}")
            yaw_error_final = self.normalize_angle(goal_yaw - self.current_pose_yaw) # Radians
            while abs(yaw_error_final) > self.angle_tolerance_rad and not rospy.is_shutdown(): # Compare radians
                cmd = Twist()
                # Only turn, use angular_kp
                cmd.angular.z = max(min(self.angular_kp * yaw_error_final, self.max_turning_speed), -self.max_turning_speed)

                # --- Log Data (Alignment Phase) ---
                # Calculate current distance error for logging purposes during alignment
                current_distance_error_phase2 = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                self.log_data(
                    wp_index=wp_index + 1,
                    goal_x=goal_x,
                    goal_y=goal_y,
                    goal_yaw=goal_yaw, # Radians
                    cmd_linear_x=cmd.linear.x, # Will be 0
                    cmd_angular_z=cmd.angular.z,
                    distance_error=current_distance_error_phase2, # Log current distance error
                    heading_error_rad=yaw_error_final # Radians
                )

                self.velocity_publisher.publish(cmd)
                # Recalculate error for the loop condition
                yaw_error_final = self.normalize_angle(goal_yaw - self.current_pose_yaw) # Radians
                self.rate.sleep()

            # Check if alignment finished due to shutdown
            if rospy.is_shutdown():
                 rospy.logwarn("Shutdown requested during final alignment.")
                 break

            rospy.loginfo(f"[SUCCESS] Reached final orientation for waypoint {wp_index + 1} (Yaw Error: {math.degrees(yaw_error_final):.2f} deg)")
            self.velocity_publisher.publish(Twist()) # Ensure robot is stopped
            rospy.sleep(0.5) # Pause briefly at the waypoint

        # End of loop
        if not rospy.is_shutdown():
            rospy.loginfo("[DONE] Finished all waypoints")
        else:
            rospy.loginfo("[ABORTED] Waypoint navigation aborted due to shutdown request.")

        # Final stop command regardless of how the loop ended
        self.velocity_publisher.publish(Twist())


    # --- Attribution Comment ---
    # The normalize_angle function below is adapted from
    # code commonly found in robotics navigation examples, including
    # repositories like https://github.com/sciyen/NCKUES-SelfDriving.
    # Please refer to the original source(s) for specific license terms if applicable.
    # --- End Attribution Comment ---
    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

# Main execution block
if __name__ == '__main__':
    try:
        navigator = HuskyOptiTrackNavigator()
        # The node will now run until move_through_waypoints finishes or ROS shuts down.
        # rospy.spin() is not strictly needed here because move_through_waypoints blocks,
        # but adding it won't hurt and keeps callbacks alive if needed after navigation.
        # However, since the script is designed to exit after navigation, we omit spin().
        # The cleanup is handled by rospy.on_shutdown.
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down navigator node.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the navigator: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for debugging
    finally:
        # The rospy.on_shutdown hook (self.cleanup) should handle stopping the robot
        # and closing the log file. Adding an explicit stop here is redundant
        # if the hook works correctly, but can be a safety measure.
        # rospy.loginfo("Navigator node is shutting down.")
        # Note: Avoid creating new publishers in finally if the node is already shutting down.
        # Rely on the cleanup hook.
        pass # Cleanup is handled by rospy.on_shutdown

