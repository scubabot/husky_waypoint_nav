#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry # Keep if needed for other purposes, removed from logic below
import math
import numpy as np
import copy # Not explicitly used, can be removed if not needed elsewhere
import os, glob
import time # Used for dt calculation if rospy.Time isn't preferred

class HuskyOptiTrackNavigatorPD: # Renamed class slightly
    def __init__(self):
        rospy.init_node('husky_optitrack_navigator_pd', anonymous=True)

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        # Using OptiTrack for pose feedback
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.update_pose)
        # Removed odometry subscriber as it wasn't used for control
        # rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odometry_callback)

        # Robot state
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.pose_received = False # Flag to wait for the first pose message

        # Controller tuning - Added Kd_angular
        self.max_speed = 0.4
        self.max_turning_speed = 1.0
        self.Kp_angular = 1.5 # Renamed from angular_gain for clarity (Proportional)
        self.Kd_angular = 0.1 # Derivative gain (NEEDS TUNING)
        self.Kp_linear = 0.5  # Renamed from linear_gain for clarity
        self.dist_tolerance = 0.10 # Renamed for clarity
        self.yaw_tolerance_rad = math.radians(3.0) # Use radians internally
        self.rate = rospy.Rate(60) # Control loop frequency

        # PD Controller State Variables
        self.prev_yaw_error = 0.0
        self.last_time = rospy.Time.now()

        # Command smoothing (optional, can sometimes fight PD controller)
        self.enable_smoothing = True
        self.prev_cmd = Twist()

        # Load waypoints
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
             rospy.signal_shutdown("No waypoints loaded.")
             return # Exit if no waypoints

        rospy.loginfo("Waiting for the first pose update...")
        while not self.pose_received and not rospy.is_shutdown():
            self.rate.sleep()
        rospy.loginfo("First pose received. Starting navigation.")

        self.move_through_waypoints()

    def update_pose(self, data):
        """Callback function for updating the robot's pose."""
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        self.current_pose_yaw = self.quaternion_to_euler(
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )
        if not self.pose_received:
            self.last_time = rospy.Time.now() # Initialize time when first pose is received
            self.pose_received = True

    # Removed odometry_callback as it wasn't used

    def quaternion_to_euler(self, x, y, z, w):
        """Converts quaternion to Euler yaw angle."""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def load_waypoints(self):
        """Loads waypoints from the latest file in the specified directory."""
        wp_dir = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints") # Adjust path if needed
        try:
            files = sorted(glob.glob(os.path.join(wp_dir, "optitrack_waypoints_*.txt")))
            if not files:
                rospy.logerr("[ERROR] No waypoint files found matching 'optitrack_waypoints_*.txt' in: %s", wp_dir)
                return []
            latest_file = files[-1]
            waypoints = []
            with open(latest_file, 'r') as f:
                for i, line in enumerate(f):
                    try:
                        parts = list(map(float, line.strip().split()))
                        if len(parts) == 3:
                            x, y, yaw = parts
                            waypoints.append((x, y, math.radians(yaw))) # Store yaw in radians
                        elif len(parts) == 2:
                             x, y = parts
                             waypoints.append((x, y, None)) # Handle waypoints without yaw
                             rospy.logwarn("Waypoint %d has no yaw specified. Will only navigate to position.", i+1)
                        else:
                             rospy.logwarn("Skipping malformed line %d in waypoint file: %s", i+1, line.strip())

                    except ValueError:
                        rospy.logwarn("Skipping invalid line %d in waypoint file: %s", i+1, line.strip())
                        continue
            rospy.loginfo("[INFO] Loaded %d waypoints from %s", len(waypoints), latest_file)
            return waypoints
        except Exception as e:
            rospy.logerr("Failed to load waypoints from %s: %s", wp_dir, e)
            return []

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calculate_pd_angular_control(self, current_yaw, target_yaw):
        """Calculates the PD control output for angular velocity."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time # Store current time for next iteration

        if dt <= 0.0: # Prevent division by zero or negative dt
             # Could use 1.0 / self.rate.sleep_dur.to_sec() as an estimate if needed
             rospy.logwarn_throttle(1.0, "dt <= 0, skipping derivative calculation this cycle.")
             dt = 1e-6 # Use a very small dt to avoid large derivative spikes

        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        # Proportional term
        p_term = self.Kp_angular * yaw_error

        # Derivative term
        derivative_error = (yaw_error - self.prev_yaw_error) / dt
        d_term = self.Kd_angular * derivative_error

        # Update previous error for next iteration
        self.prev_yaw_error = yaw_error

        # Combine terms and clamp
        angular_vel = p_term + d_term
        angular_vel_clamped = max(min(angular_vel, self.max_turning_speed), -self.max_turning_speed)

        # Optional: Log PD components for tuning
        # rospy.loginfo_throttle(1.0, "P: %.2f, D: %.2f, Err: %.2f, Deriv: %.2f, dt: %.4f, Cmd: %.2f",
        #                        p_term, d_term, math.degrees(yaw_error), derivative_error, dt, angular_vel_clamped)

        return angular_vel_clamped, yaw_error


    def move_through_waypoints(self):
        """Main control loop to navigate through the loaded waypoints."""
        for i, (goal_x, goal_y, goal_yaw_rad) in enumerate(self.waypoints):
            rospy.loginfo("[WAYPOINT %d] Navigating to (%.2f, %.2f) %s",
                          i+1, goal_x, goal_y,
                          f"with target yaw {math.degrees(goal_yaw_rad):.1f}°" if goal_yaw_rad is not None else "")

            # Reset PD state for the new waypoint segment
            self.prev_yaw_error = 0.0
            # self.last_time = rospy.Time.now() # Reset time reference at start of segment

            while not rospy.is_shutdown():
                if not self.pose_received:
                    rospy.logwarn_throttle(5.0, "Waiting for pose data...")
                    self.rate.sleep()
                    continue

                distance = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                yaw_to_goal = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)

                # Calculate desired angular velocity using PD controller towards the goal *position*
                angular_vel_pd, yaw_error_to_goal = self.calculate_pd_angular_control(self.current_pose_yaw, yaw_to_goal)

                rospy.loginfo_throttle(2.0, "[DEBUG %d] Dist: %.2f, YawErrToGoal: %.1f°",
                                       i+1, distance, math.degrees(yaw_error_to_goal))

                # Check if goal position is reached
                if distance < self.dist_tolerance:
                    rospy.loginfo("[SUCCESS %d] Reached position for waypoint (%.2f, %.2f)", i+1, goal_x, goal_y)
                    break # Exit position navigation loop

                cmd = Twist()

                # --- Control Logic ---
                # Always calculate angular velocity using PD
                cmd.angular.z = angular_vel_pd

                # Only move forward if oriented reasonably towards the goal
                if abs(yaw_error_to_goal) < self.yaw_tolerance_rad * 2.0: # Allow movement even if not perfectly aligned
                     # Simple P controller for linear speed based on distance
                     linear_vel = self.Kp_linear * distance
                     cmd.linear.x = min(linear_vel, self.max_speed)
                else:
                     # Focus on turning if orientation error is large
                     cmd.linear.x = 0.0
                     # Optional: reduce max turn speed when not aligned? Maybe not necessary with PD.


                # Optional: smooth output
                if self.enable_smoothing:
                    cmd.linear.x = 0.8 * self.prev_cmd.linear.x + 0.2 * cmd.linear.x
                    cmd.angular.z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                    self.prev_cmd = cmd # Store the smoothed command

                self.velocity_publisher.publish(cmd)
                self.rate.sleep()
            # --- End of position navigation loop ---

            # Stop briefly after reaching position before final alignment
            self.velocity_publisher.publish(Twist())
            rospy.sleep(0.2) # Small pause


            # --- Final Yaw Alignment (if goal_yaw is specified) ---
            if goal_yaw_rad is not None:
                rospy.loginfo("[WAYPOINT %d] Aligning to final yaw: %.1f°", i+1, math.degrees(goal_yaw_rad))
                # Reset PD state specifically for yaw alignment
                self.prev_yaw_error = 0.0
                # self.last_time = rospy.Time.now()

                # Calculate initial error for the loop condition
                _, yaw_error_final = self.calculate_pd_angular_control(self.current_pose_yaw, goal_yaw_rad)
                # We recalculate inside the loop, but need an initial value for the while condition

                while abs(yaw_error_final) > self.yaw_tolerance_rad and not rospy.is_shutdown():
                    cmd = Twist()
                    cmd.linear.x = 0.0 # No linear movement during final alignment
                    # Use PD controller for final yaw alignment
                    cmd.angular.z, yaw_error_final = self.calculate_pd_angular_control(self.current_pose_yaw, goal_yaw_rad)

                    # Optional: smooth output (usually less necessary/helpful for pure rotation)
                    if self.enable_smoothing:
                        # Only smooth angular part
                        cmd.angular.z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                        self.prev_cmd.angular.z = cmd.angular.z # Update only angular
                        self.prev_cmd.linear.x = 0.0 # Ensure linear stays zero

                    rospy.loginfo_throttle(1.0, "[ALIGN %d] YawErr: %.1f°, CmdAng: %.2f",
                                            i+1, math.degrees(yaw_error_final), cmd.angular.z)

                    self.velocity_publisher.publish(cmd)
                    self.rate.sleep()

                rospy.loginfo("[SUCCESS %d] Final yaw alignment complete.", i+1)
                self.velocity_publisher.publish(Twist()) # Stop robot after alignment
                rospy.sleep(0.2) # Small pause before next waypoint


        rospy.loginfo("[DONE] Finished all waypoints.")
        self.velocity_publisher.publish(Twist())  # Ensure robot is stopped
        # rospy.spin() # Keep node alive if needed, but script might naturally end here

if __name__ == '__main__':
    try:
        navigator = HuskyOptiTrackNavigatorPD()
        # If the script should exit after completing waypoints, remove rospy.spin()
        # If it should stay alive (e.g., to offer services), keep rospy.spin()
        # Based on the original code, it seems intended to run once and finish.
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", e)
        # Optionally publish a zero Twist message on unexpected exit
        # Might require getting the publisher instance differently if __init__ fails
        # if hasattr(navigator, 'velocity_publisher'):
        #     navigator.velocity_publisher.publish(Twist())
    finally:
        # Ensure the robot stops if the script exits for any reason
        # This requires the publisher to be initialized.
        # A more robust way might involve creating the publisher early or using atexit.
        rospy.loginfo("Publishing zero velocity on exit.")
        # The publisher might not exist if init failed early.
        # A simple approach:
        pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.1) # Give publisher time to connect
        pub.publish(Twist())
        rospy.sleep(0.1) # Give message time to be sent