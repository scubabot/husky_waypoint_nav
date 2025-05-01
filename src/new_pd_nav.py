#!/usr/bin/env python

import rospy
import csv
import math
import time
import os
from geometry_msgs.msg import Twist, PoseStamped
# No Odometry import needed here
from tf.transformations import euler_from_quaternion

class PDNavigator:
    def __init__(self):
        rospy.init_node('pd_navigator', anonymous=True)

        # --- Parameters ---
        self.husky_name = rospy.get_param('~husky_name', 'husky')
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/natnet_ros/' + self.husky_name + '/pose')
        # No odom_topic needed here
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/husky_velocity_controller/cmd_vel')
        self.waypoint_file = rospy.get_param('~waypoint_file', 'optitrack_waypoints_06.txt')
        self.log_file_path_param = rospy.get_param('~log_file_path', '~/catkin_ws/src/husky_waypoint_nav/logs')

        # --- PD Control gains ---
        # Use consistent naming with p_navigator.py
        self.linear_kp = rospy.get_param('~linear_kp', 0.5)   # P gain for linear velocity
        self.angular_kp = rospy.get_param('~angular_kp', 1.5)  # P gain for angular velocity
        self.angular_kd = rospy.get_param('~angular_kd', 0.2)  # D gain for angular velocity

        # Speed limits (Consistent names)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5) # m/s
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0) # rad/s

        # Goal tolerance (Consistent names)
        self.pos_tolerance = rospy.get_param('~pos_tolerance', 0.1) # meters
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.1) # radians

        # Rate for the control loop (Consistent name)
        self.rate_hz = rospy.get_param('~rate', 10) # Hz

        # --- State Variables ---
        self.current_pose = None
        self.current_yaw = None
        self.pose_received = False
        # No odom variables needed here

        # Variables for D term calculation
        self.last_time = None # Initialize later
        self.last_heading_error = 0.0

        self.waypoints = []
        self.current_waypoint_index = 0

        # --- ROS Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.mocap_pose_topic, PoseStamped, self.pose_callback)
        # No odom subscriber needed here

        # --- Load Waypoints ---
        # Consistent path finding logic
        pkg_path = os.path.join(rospy.get_param('~pkg_override_path', os.path.expanduser('~/catkin_ws/src/husky_waypoint_nav')))
        waypoints_full_path = os.path.join(pkg_path, 'config', 'waypoints', self.waypoint_file)
        self.load_waypoints(waypoints_full_path)

        # --- Setup Logging ---
        # Consistent logging setup logic
        log_dir = os.path.expanduser(self.log_file_path_param)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        waypoint_filename_base = os.path.splitext(os.path.basename(self.waypoint_file))[0]
        # Use pd_nav prefix for PD controller logs
        log_filename = os.path.join(log_dir, f"pd_nav_{waypoint_filename_base}_{timestamp}.csv")
        self.log_file = open(log_filename, 'w')
        self.csv_writer = csv.writer(self.log_file)

        # --- Write CSV header - Standardized common columns ---
        self.csv_writer.writerow([
            'timestamp', 'waypoint_index',
            'current_x', 'current_y', 'current_yaw',        # Standardized pose names
            'target_x', 'target_y', 'target_yaw',          # Standardized target names (target_yaw added for clarity though often implicitly defined)
            # No odom columns needed here
            'pos_error', 'angle_error',                    # Standardized error names
            'linear_vel_cmd', 'angular_vel_cmd',           # Standardized command names
            'angular_p_term', 'angular_d_term'             # PD specific terms
        ])
        rospy.loginfo(f"Logging data to: {log_filename}")

    # --- load_waypoints method (Identical to p_navigator.py) ---
    def load_waypoints(self, file_path):
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                next(reader) # Skip header row
                for row in reader:
                    # Expecting format: index, x, y, yaw
                    self.waypoints.append({
                        'x': float(row[1]),
                        'y': float(row[2]),
                        'yaw': float(row[3]) # Store target yaw
                    })
            rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints from {file_path}")
            if not self.waypoints:
                 rospy.logerr("No waypoints loaded.")
                 rospy.signal_shutdown("No waypoints loaded")
        except Exception as e:
            rospy.logerr(f"Error loading waypoints from {file_path}: {e}")
            rospy.signal_shutdown("Waypoint loading failed")

    # --- pose_callback method (Identical to p_navigator.py) ---
    def pose_callback(self, msg):
        self.current_pose = msg.pose
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        self.pose_received = True

    # --- No odom_callback needed here ---

    # --- get_control_command method - Modified for PD ---
    def get_control_command(self, target_pose):
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn_throttle(1.0, "Waiting for current pose from MoCap...")
            # Return Nones for all expected values including PD terms
            return None, None, None, None, None, None

        current_time = rospy.get_time()
        dt = 0.0
        if self.last_time is not None:
            dt = current_time - self.last_time
        # Avoid division by zero or huge derivatives at the start
        if dt <= 0.001: # Handle first loop or time glitch
             dt = 1.0 / self.rate_hz # Estimate dt if necessary
             error_derivative = 0.0 # Assume no change on first real step
        else:
            # Prevent division by zero if dt is extremely small
            error_derivative = self.normalize_angle(self.last_heading_error) / dt

        # Calculate errors (Consistent names)
        dx = target_pose['x'] - self.current_pose.position.x
        dy = target_pose['y'] - self.current_pose.position.y
        pos_error = math.sqrt(dx**2 + dy**2) # Use consistent name: pos_error

        angle_to_goal = math.atan2(dy, dx)
        # Use consistent name: angle_error
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # --- PD Angular Control ---
        # Calculate error derivative based on heading error change
        heading_error_rate = 0.0
        if dt > 0.001: # Avoid division by zero on first iteration or time glitches
             heading_error_rate = self.normalize_angle(angle_error - self.last_heading_error) / dt

        # Calculate P and D terms for angular velocity
        angular_p_term = self.angular_kp * angle_error
        angular_d_term = self.angular_kd * heading_error_rate
        angular_vel = angular_p_term + angular_d_term # PD control law

        # --- P Linear Control ---
        # Consistent variable name: linear_kp
        linear_vel = self.linear_kp * pos_error

        # --- Update states for next D-term calculation ---
        self.last_time = current_time
        self.last_heading_error = angle_error

        # --- Apply saturation and safety limits (Consistent Logic) ---
        linear_vel = max(min(linear_vel, self.max_linear_vel), -self.max_linear_vel)
        # Same logic as p_navigator.py to prioritize turning if angle error is large
        if abs(angle_error) > math.pi / 2 and pos_error > self.pos_tolerance * 0.5 :
             linear_vel = 0
        elif pos_error < self.pos_tolerance:
             pass
        else:
             linear_vel = max(0, linear_vel) # Prevent backward motion unless near goal

        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

        # Return all values needed for logging
        return linear_vel, angular_vel, pos_error, angle_error, angular_p_term, angular_d_term

    # --- normalize_angle method (Identical to p_navigator.py) ---
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # --- navigate method (Structured like p_navigator.py, no odom wait) ---
    def navigate(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Starting PD navigation...")

        # Wait for the first pose message
        rospy.loginfo("Waiting for MoCap pose data...")
        while not self.pose_received and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("MoCap pose received.")
        # No wait for odom needed here

        # Initialize last_time just before the loop starts
        self.last_time = rospy.get_time()
        self.last_heading_error = 0.0 # Reset just in case

        rospy.loginfo("Starting navigation loop.")

        while self.current_waypoint_index < len(self.waypoints) and not rospy.is_shutdown():
            target_pose = self.waypoints[self.current_waypoint_index]

            # Calculate control commands - unpack PD terms as well
            result = self.get_control_command(target_pose)
            if result is None: # Handle case where pose is not yet available
                rate.sleep()
                continue
            # Unpack returned values
            linear_vel, angular_vel, pos_error, angle_error, angular_p_term, angular_d_term = result


            # Create Twist message
            twist_cmd = Twist()
            twist_cmd.linear.x = linear_vel
            twist_cmd.angular.z = angular_vel

            # Publish command
            self.cmd_vel_pub.publish(twist_cmd)

            # Log data - Using standardized column names + PD terms
            log_time = rospy.get_time() # Use consistent time for log row
            log_data = [
                log_time, self.current_waypoint_index,
                self.current_pose.position.x, self.current_y, self.current_yaw,
                target_pose['x'], target_pose['y'], target_pose['yaw'], # Log target yaw
                # No odom data here
                pos_error, angle_error,
                linear_vel, angular_vel,
                angular_p_term, angular_d_term # Add PD specific terms
            ]
            self.csv_writer.writerow(log_data)


            # Check if waypoint is reached (Identical logic)
            if pos_error < self.pos_tolerance:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached (X: {self.current_pose.position.x:.2f}, Y: {self.current_y:.2f})") # Use self.current_y
                self.current_waypoint_index += 1
                # Stop the robot briefly when reaching a waypoint
                self.cmd_vel_pub.publish(Twist()) # Send zero velocity
                if self.current_waypoint_index < len(self.waypoints):
                     rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index}...")
                     # Reset D-term history for the new segment if desired (optional)
                     # self.last_time = rospy.get_time() # Reset time to avoid large dt spike
                     # self.last_heading_error = 0.0 # Reset error prevents using old error for derivative
                else:
                     rospy.loginfo("Final waypoint reached.")
                     break # Exit loop after reaching last waypoint

            rate.sleep()

        # Stop the robot after finishing all waypoints or shutdown (Identical logic)
        rospy.loginfo("Navigation finished or interrupted. Stopping robot.")
        self.cmd_vel_pub.publish(Twist()) # Send zero velocity command
        self.log_file.close() # Close the log file

# --- Main execution block (Structured like p_navigator.py) ---
if __name__ == '__main__':
    try:
        navigator = PDNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")
    except Exception as e:
        # Use f-string for error logging
        rospy.logerr(f"An error occurred in PDNavigator: {e}")
        import traceback
        traceback.print_exc() # Print traceback for debugging
    finally:
        # Consistent robust cleanup
        if 'navigator' in locals() and hasattr(navigator, 'log_file') and navigator.log_file and not navigator.log_file.closed:
             navigator.log_file.close()
             rospy.loginfo("Log file closed.")