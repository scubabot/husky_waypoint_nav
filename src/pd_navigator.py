#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry # Not used for control
import math
# import numpy as np # Not used
# import copy # Not used
import os, glob
import traceback # For error handling
import datetime # Needed for timestamped log files

class HuskyOptiTrackNavigatorPD:
    def __init__(self):
        """Initializes the ROS node, parameters, publishers, subscribers, and logging."""
        rospy.init_node('husky_optitrack_navigator_pd_ads', anonymous=True) # Added _ads for Align-Drive-Straight

        # --- Get Controller Type Parameter ---
        self.controller_type = rospy.get_param("~controller_type", "PD_Align_Drive_Straight")
        rospy.loginfo(f"Initializing navigator with controller type: {self.controller_type}")
        # --- End Parameter ---

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_topic = rospy.get_param("~pose_topic", "/natnet_ros/base_link/pose")
        rospy.Subscriber(self.pose_topic, PoseStamped, self.update_pose)
        # Optional: Subscribe to odometry for logging or comparison if needed
        # rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odometry_callback)


        # Robot state
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0 # Radians
        self.pose_received = False

        # Controller tuning - Using Parameters
        self.max_speed = rospy.get_param("~max_speed", 0.4) # Defaulted back to 0.4
        self.max_turning_speed = rospy.get_param("~max_turning_speed", 2.0) # Defaulted back to 2.0
        self.Kp_angular = rospy.get_param("~Kp_angular", 0.25) # Proportional gain for angular control
        self.Kd_angular = rospy.get_param("~Kd_angular", 0.1) # Derivative gain for angular control
        self.Kp_linear = rospy.get_param("~Kp_linear", 0.5)   # Proportional gain for linear control
        self.dist_tolerance = rospy.get_param("~distance_tolerance", 0.15) # Defaulted back to 0.15
        # Angle Tolerance for switching from ALIGN to DRIVE STRAIGHT (degrees)
        self.align_tolerance_deg = rospy.get_param("~align_tolerance_deg", 5.0) # Defaulted back to 5.0
        self.align_tolerance_rad = math.radians(self.align_tolerance_deg) # Convert to RADIANS

        self.control_rate = rospy.get_param("~control_rate", 60) # Control loop frequency (Hz)
        self.rate = rospy.Rate(self.control_rate)


        # PD Controller State Variables
        self.prev_yaw_error = 0.0 # Radians
        self.last_time = None # Initialize to None, will be set after first pose

        # --- Logging Setup ---
        self.log_file = None
        self.log_dir = os.path.expanduser(rospy.get_param("~log_dir", "~/catkin_ws/src/husky_waypoint_nav/logs"))
        self.setup_logging()
        rospy.on_shutdown(self.cleanup)
        # --- End Logging Setup ---

        # Load waypoints (X, Y only)
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
             rospy.logerr("No waypoints loaded. Shutting down.")
             rospy.signal_shutdown("No waypoints loaded.")
             return

        # --- Wait for Initial Pose ---
        rospy.loginfo("Waiting for the first pose update...")
        while not self.pose_received and not rospy.is_shutdown():
            self.rate.sleep() # Wait for update_pose callback to set self.pose_received

        # --- Initialize Time and Start Navigation ---
        # This block now correctly handles initialization after the first pose is received
        if self.last_time is None and not rospy.is_shutdown():
             self.last_time = rospy.Time.now() # Initialize time HERE
             rospy.loginfo(f"First pose received: X={self.current_pose_x:.2f}, Y={self.current_pose_y:.2f}, Yaw={math.degrees(self.current_pose_yaw):.1f} deg. Starting navigation.")
             self.move_through_waypoints() # Start the main loop
        elif rospy.is_shutdown():
             rospy.logwarn("Shutdown requested before navigation could start.")
        # Removed the problematic 'else' block that caused the error


    def setup_logging(self):
        """Creates the log directory and opens the log file for writing."""
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                rospy.loginfo(f"Created log directory: {self.log_dir}")

            safe_controller_type = "".join(c if c.isalnum() else '_' for c in self.controller_type)
            safe_controller_type = '_'.join(filter(None, safe_controller_type.split('_')))
            if not safe_controller_type: safe_controller_type = "unknown_controller"

            timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            log_filename = f"{safe_controller_type}_{timestamp_str}.csv"
            log_filepath = os.path.join(self.log_dir, log_filename)

            self.log_file = open(log_filepath, 'w', buffering=1)
            header = "timestamp,current_pose_x,current_pose_y,current_pose_yaw," \
                     "target_wp_index,goal_x,goal_y," \
                     "cmd_linear_x,cmd_angular_z," \
                     "distance_error,heading_error_rad," \
                     "p_term_angular,d_term_angular\n"
            self.log_file.write(header)
            rospy.loginfo(f"Logging data to: {log_filepath}")

        except IOError as e:
            rospy.logerr(f"Failed to setup logging: {e}")
            self.log_file = None
            rospy.signal_shutdown("Logging setup failed")
        except OSError as e:
             rospy.logerr(f"Failed to create log directory {self.log_dir}: {e}")
             self.log_file = None
             rospy.signal_shutdown("Logging setup failed")

    def log_data(self, wp_index, goal_x, goal_y, cmd_linear_x, cmd_angular_z, distance_error, heading_error_rad, p_term=0.0, d_term=0.0):
        """Writes a line of data to the log file, including PD terms."""
        if self.log_file and not self.log_file.closed:
            try:
                timestamp = rospy.Time.now().to_sec()
                log_line = f"{timestamp:.4f},{self.current_pose_x:.4f},{self.current_pose_y:.4f},{self.current_pose_yaw:.4f}," \
                           f"{wp_index},{goal_x:.4f},{goal_y:.4f}," \
                           f"{cmd_linear_x:.4f},{cmd_angular_z:.4f}," \
                           f"{distance_error:.4f},{heading_error_rad:.4f}," \
                           f"{p_term:.4f},{d_term:.4f}\n"
                self.log_file.write(log_line)
            except IOError as e:
                rospy.logwarn_throttle(10, f"Failed to write to log file: {e}")

    def cleanup(self):
        """Closes the log file and stops the robot."""
        rospy.loginfo("Cleaning up and closing log file...")
        try:
             if hasattr(self, 'velocity_publisher') and self.velocity_publisher.get_num_connections() > 0:
                 self.velocity_publisher.publish(Twist())
                 rospy.sleep(0.1)
                 self.velocity_publisher.publish(Twist())
                 rospy.loginfo("Stop command sent.")
             else:
                 rospy.logwarn("Velocity publisher not available during cleanup.")
        except Exception as e:
             rospy.logwarn(f"Could not publish stop command during cleanup: {e}")

        if self.log_file and not self.log_file.closed:
            try:
                self.log_file.flush()
                self.log_file.close()
                rospy.loginfo("Log file closed.")
                self.log_file = None
            except IOError as e:
                rospy.logerr(f"Error closing log file: {e}")


    def update_pose(self, data):
        """Callback function for updating the robot's pose. Sets received flag."""
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        self.current_pose_yaw = self.quaternion_to_euler( # Yaw is stored in RADIANS
            data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        )
        # Only set the flag here, don't initialize self.last_time
        if not self.pose_received:
            self.pose_received = True

    # Optional odometry callback (not used for control)
    # def odometry_callback(self, data):
    #      orientation_q = data.pose.pose.orientation
    #      _, _, self.current_odometry_yaw = self.quaternion_to_euler(
    #          orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    #      )

    def quaternion_to_euler(self, x, y, z, w):
        """Converts quaternion to Euler yaw angle (radians)."""
        t3 = +2.0 * (w * z + x * y); t4 = +1.0 - 2.0 * (y * y + z * z); return math.atan2(t3, t4)

    def load_waypoints(self):
        """Loads waypoints (X, Y) from the specified file."""
        wp_dir_param = rospy.get_param("~waypoint_dir", "~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        wp_prefix = rospy.get_param("~waypoint_prefix", "optitrack_waypoints_")
        wp_dir = os.path.expanduser(wp_dir_param)
        waypoints = []
        try:
            files = sorted(glob.glob(os.path.join(wp_dir, f"{wp_prefix}*.txt")))
            if not files:
                rospy.logerr(f"[ERROR] No waypoint files matching '{wp_prefix}*.txt' found in: {wp_dir}")
                return []

            latest_file = files[-1]
            rospy.loginfo(f"Attempting to load waypoints from: {latest_file}")
            with open(latest_file, 'r') as f:
                for i, line in enumerate(f):
                    line = line.strip()
                    if line.startswith('#') or not line: continue
                    try:
                        parts = list(map(float, line.split()))
                        if len(parts) == 2: # Expecting exactly X, Y
                            x, y = parts
                            waypoints.append((x, y))
                        else:
                            rospy.logwarn(f"Skipping malformed line {i+1}: '{line.strip()}' (Expected 2 values: X Y)")
                    except (ValueError, IndexError):
                        rospy.logwarn(f"Skipping invalid line {i+1}: {line.strip()}")

            rospy.loginfo(f"[INFO] Loaded {len(waypoints)} waypoints (X, Y) from {latest_file}")
            return waypoints
        except Exception as e:
            rospy.logerr(f"Failed to load waypoints: {e}")
            rospy.logerr(traceback.format_exc())
            return []

    def normalize_angle(self, angle):
        """Normalize an angle (in radians) to the range [-pi, pi]."""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def reset_pd_state(self):
        """Resets the PD controller's error state."""
        self.prev_yaw_error = 0.0

    def calculate_pd_angular_control(self, current_yaw_rad, target_yaw_rad, dt, is_first_cycle=False):
        """Calculates the PD control output (radians/sec) using radian inputs.
           Returns: angular_vel_clamped (rad/s), yaw_error_rad (rad), p_term, d_term
        """
        if dt <= 1e-6:
             yaw_error_rad = self.normalize_angle(target_yaw_rad - current_yaw_rad)
             p_term = self.Kp_angular * yaw_error_rad
             return 0.0, yaw_error_rad, p_term, 0.0

        Kp = self.Kp_angular
        Kd = self.Kd_angular
        yaw_error_rad = self.normalize_angle(target_yaw_rad - current_yaw_rad)
        p_term = Kp * yaw_error_rad
        d_term = 0.0
        if not is_first_cycle:
            derivative_error = (yaw_error_rad - self.prev_yaw_error) / dt
            d_term = Kd * derivative_error

        angular_vel = p_term + d_term
        angular_vel_clamped = max(min(angular_vel, self.max_turning_speed), -self.max_turning_speed)
        self.prev_yaw_error = yaw_error_rad
        return angular_vel_clamped, yaw_error_rad, p_term, d_term

    def move_through_waypoints(self):
        """Main control loop using Align-Then-Drive-Straight logic with PD turning."""
        if not self.waypoints:
             rospy.logwarn("No waypoints loaded, cannot navigate.")
             return

        for i, (goal_x, goal_y) in enumerate(self.waypoints):
            wp_index = i + 1
            waypoint_label = f"WAYPOINT {wp_index}/{len(self.waypoints)}"
            rospy.loginfo(f"[{waypoint_label}] Navigating to ({goal_x:.2f}, {goal_y:.2f})")

            self.reset_pd_state()
            first_cycle_waypoint = True

            while not rospy.is_shutdown():
                if not self.pose_received or self.last_time is None:
                    self.rate.sleep()
                    continue

                current_time = rospy.Time.now()
                dt = (current_time - self.last_time).to_sec()

                current_x = self.current_pose_x
                current_y = self.current_pose_y
                current_yaw_rad = self.current_pose_yaw
                distance = math.hypot(goal_x - current_x, goal_y - current_y)

                if distance < self.dist_tolerance:
                    rospy.loginfo(f"[{waypoint_label}] Reached position (Dist: {distance:.3f} < {self.dist_tolerance:.3f}).")
                    break

                yaw_to_goal_rad = math.atan2(goal_y - current_y, goal_x - current_x)
                yaw_error_rad = self.normalize_angle(yaw_to_goal_rad - current_yaw_rad)

                angular_vel_pd, _, p_term_val, d_term_val = self.calculate_pd_angular_control(
                    current_yaw_rad, yaw_to_goal_rad, dt, first_cycle_waypoint
                )
                first_cycle_waypoint = False

                if dt <= 1e-6:
                    rospy.logwarn_throttle(1.0, f"[{waypoint_label} Loop] dt ({dt:.5f}) too small, skipping cycle.")
                    self.last_time = current_time
                    self.rate.sleep()
                    continue

                cmd = Twist()
                if abs(yaw_error_rad) > self.align_tolerance_rad:
                    # ALIGN Phase
                    cmd.linear.x = 0.0
                    cmd.angular.z = angular_vel_pd
                else:
                    # DRIVE STRAIGHT Phase
                    cmd.angular.z = 0.0 # Force zero angular velocity
                    linear_vel = self.Kp_linear * distance
                    cmd.linear.x = min(linear_vel, self.max_speed)
                    # Set P/D terms to zero for logging when driving straight
                    p_term_val = 0.0
                    d_term_val = 0.0

                self.log_data(
                    wp_index=wp_index, goal_x=goal_x, goal_y=goal_y,
                    cmd_linear_x=cmd.linear.x, cmd_angular_z=cmd.angular.z,
                    distance_error=distance, heading_error_rad=yaw_error_rad,
                    p_term=p_term_val, d_term=d_term_val
                )

                self.velocity_publisher.publish(cmd)
                self.last_time = current_time
                self.rate.sleep()
            # --- End of Movement Loop ---

            self.velocity_publisher.publish(Twist()) # Stop robot
            if not rospy.is_shutdown():
                rospy.loginfo(f"[{waypoint_label}] Pausing.")
                rospy.sleep(0.5)
            else:
                 rospy.logwarn("Shutdown requested during waypoint approach.")
                 break

            # --- Final Yaw Alignment Loop REMOVED ---

            rospy.loginfo(f"[{waypoint_label}] Completed.")

        # --- End of All Waypoints ---
        if not rospy.is_shutdown():
            rospy.loginfo("[DONE] Finished all waypoints.")
        else:
            rospy.loginfo("[ABORTED] Waypoint navigation aborted due to shutdown request.")

        self.velocity_publisher.publish(Twist())

if __name__ == '__main__':
    navigator = None
    try:
        navigator = HuskyOptiTrackNavigatorPD()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {traceback.format_exc()}")

