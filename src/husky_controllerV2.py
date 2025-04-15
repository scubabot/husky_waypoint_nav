#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import copy
import os, glob

class HuskyOptiTrackNavigator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('husky_optitrack_navigator', anonymous=True)

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.update_pose)
        rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odometry_callback)

        # Robot state variables
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.current_odometry_yaw = 0.0 # Unused

        # Controller tuning parameters
        self.max_speed = 0.4 # Max linear speed (m/s)
        self.max_turning_speed = 1.0 # Max angular speed (rad/s)
        self.angular_gain = 1.0 # Proportional gain for angular velocity
        self.linear_gain = 0.2 # Proportional gain for linear velocity
        self.yaw_error_scaling_divisor = math.pi/2.0 # Divisor for linear speed scaling based on yaw error
        self.deg_tolerance = 3.0  # Angular tolerance in degrees for final alignment
        self.distance_threshold = 0.15 # Distance threshold (m) to consider waypoint reached

        # Control loop rate (Hz)
        self.rate = rospy.Rate(60)

        # Previous command for smoothing
        self.prev_cmd = Twist()

        # Load waypoints
        self.waypoints = self.load_waypoints()

        # Allow time for ROS topics
        rospy.sleep(1.0)
        # Start navigation
        self.move_through_waypoints()

    def update_pose(self, data):
        """Callback function to update the robot's pose from OptiTrack data."""
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        self.current_pose_yaw = self.quaternion_to_euler(
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )

    def odometry_callback(self, data):
        """Callback function to update yaw from odometry data (currently unused)."""
        self.current_odometry_yaw = self.quaternion_to_euler(
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Converts a quaternion into yaw in radians."""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def load_waypoints(self):
        """Loads waypoints (x, y, yaw) from the latest text file."""
        wp_dir = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        files = sorted(glob.glob(os.path.join(wp_dir, "optitrack_waypoints_*.txt")))
        print("DEBUG: Found files:", files)
        if not files:
            rospy.logerr("[ERROR] No waypoint files found in: %s", wp_dir)
            rospy.signal_shutdown("No waypoints found, shutting down.")
            return []

        latest_file = files[-1]
        print("DEBUG: Using file:", latest_file)
        waypoints = []
        try:
            with open(latest_file, 'r') as f:
                print("DEBUG: Successfully opened file.")
                for i, line in enumerate(f):
                    print(f"DEBUG: Reading line {i+1}: '{line.strip()}'")
                    line = line.strip()
                    if not line or line.startswith('#'):
                        print(f"DEBUG: Skipping line {i+1}")
                        continue
                    try:
                        x, y, yaw_rad_original = map(float, line.split())
                        # --- MODIFICATION START: Negate original yaw ---
                        # Assuming the file yaw has opposite sign convention
                        corrected_goal_yaw = -yaw_rad_original
                        # Ensure the corrected yaw is also within [-pi, pi]
                        corrected_goal_yaw = self.normalize_angle(corrected_goal_yaw)
                        # --- MODIFICATION END ---
                        waypoints.append((x, y, corrected_goal_yaw))
                        print(f"DEBUG: Parsed line {i+1} successfully. Original Yaw: {yaw_rad_original:.3f}, Corrected Yaw: {corrected_goal_yaw:.3f}")
                    except ValueError:
                        rospy.logwarn("Skipping malformed line in waypoint file: %s", line)
                        print(f"DEBUG: ValueError on line {i+1}")
                        continue
            rospy.loginfo("[INFO] Loaded %d waypoints from %s (Goal Yaws Negated)", len(waypoints), latest_file) # Log message updated
        except IOError as e:
             rospy.logerr("[ERROR] Could not read waypoint file %s: %s", latest_file, e)
             print(f"DEBUG: IOError opening file: {e}")
             rospy.signal_shutdown("Error reading waypoints.")
             return []
        print("DEBUG: Returning waypoints (with corrected yaws):", waypoints)
        return waypoints

    def move_through_waypoints(self):
        """Main navigation loop to move through the loaded waypoints."""
        if not self.waypoints:
            rospy.logwarn("No waypoints loaded, navigator exiting.")
            return

        # Use the corrected goal_yaw loaded from the file
        for i, (goal_x, goal_y, corrected_goal_yaw) in enumerate(self.waypoints):
            # Log with the corrected yaw value
            rospy.loginfo("Moving to waypoint %d: (%.2f, %.2f, %.2f rad corrected)", i+1, goal_x, goal_y, corrected_goal_yaw)

            while not rospy.is_shutdown():
                distance = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                yaw_to_goal = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)
                # Yaw error uses the robot's current yaw
                yaw_error = self.normalize_angle(yaw_to_goal - self.current_pose_yaw)

                rospy.loginfo_throttle(5.0, "[DEBUG] Waypoint %d: Dist=%.2f m, YawErr=%.2f deg",
                                       i+1, distance, math.degrees(yaw_error))

                if distance < self.distance_threshold:
                    rospy.loginfo("[SUCCESS] Reached waypoint %d position.", i+1)
                    break

                # --- Simultaneous Control Logic ---
                cmd = Twist()
                cmd.angular.z = max(min(self.angular_gain * yaw_error, self.max_turning_speed), -self.max_turning_speed)
                linear_speed_request = self.linear_gain * distance
                scaling_factor = max(0.0, 1.0 - abs(yaw_error) / self.yaw_error_scaling_divisor)
                scaled_linear_speed = min(self.max_speed, linear_speed_request) * scaling_factor
                cmd.linear.x = max(0.0, scaled_linear_speed)
                cmd.linear.x = 0.8 * self.prev_cmd.linear.x + 0.2 * cmd.linear.x
                cmd.angular.z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                self.prev_cmd = cmd
                # --- End Control Logic ---

                self.velocity_publisher.publish(cmd)
                self.rate.sleep()

            # --- Final Yaw Alignment Phase ---
            # Use the corrected goal yaw for alignment
            rospy.loginfo("Aligning to final yaw (%.2f rad corrected) for waypoint %d", corrected_goal_yaw, i+1)
            self.prev_cmd = Twist()
            self.velocity_publisher.publish(self.prev_cmd)
            rospy.sleep(0.1)

            # Yaw error calculation now uses the corrected goal yaw
            yaw_error = self.normalize_angle(corrected_goal_yaw - self.current_pose_yaw)
            while abs(yaw_error) > math.radians(self.deg_tolerance) and not rospy.is_shutdown():
                cmd = Twist()
                cmd.angular.z = max(min(self.angular_gain * yaw_error, self.max_turning_speed), -self.max_turning_speed)
                cmd.angular.z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                self.prev_cmd.angular.z = cmd.angular.z
                self.velocity_publisher.publish(cmd)
                yaw_error = self.normalize_angle(corrected_goal_yaw - self.current_pose_yaw)
                rospy.loginfo_throttle(1.0, "[ALIGN] Aligning yaw for WP %d: Err=%.2f deg", i+1, math.degrees(yaw_error))
                self.rate.sleep()

            rospy.loginfo("Final yaw alignment complete for waypoint %d.", i+1)
            self.prev_cmd = Twist()
            self.velocity_publisher.publish(self.prev_cmd)
            rospy.sleep(0.5)

        # --- End of Waypoint Navigation ---
        rospy.loginfo("[DONE] Finished all waypoints.")
        self.velocity_publisher.publish(Twist())
        rospy.spin()

    def normalize_angle(self, angle):
        """Normalize an angle to be within the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        HuskyOptiTrackNavigator()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
        pass
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", e)
    finally:
        try:
           if hasattr(HuskyOptiTrackNavigator, 'velocity_publisher') and HuskyOptiTrackNavigator.velocity_publisher:
               final_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
               rospy.sleep(0.1)
               final_pub.publish(Twist())
               rospy.loginfo("Sent final stop command.")
        except Exception as final_e:
             rospy.logerr("Could not send final stop command: %s", final_e)
