#!/usr/bin/env python

import rospy
import csv
import math
import time
import os
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

class FullPDNavigator: # Renamed class for clarity
    def __init__(self):
        # Use a more specific node name
        rospy.init_node('full_pd_navigator', anonymous=True)

        # --- Parameters ---
        self.husky_name = rospy.get_param('~husky_name', 'husky')
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/natnet_ros/' + self.husky_name + '/pose')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/husky_velocity_controller/cmd_vel')
        self.waypoint_file = rospy.get_param('~waypoint_file', 'optitrack_waypoints_06.txt')
        self.log_file_path_param = rospy.get_param('~log_file_path', '~/catkin_ws/src/husky_waypoint_nav/logs')

        # --- PD Control gains ---
        self.linear_kp = rospy.get_param('~linear_kp', 0.5)   # P gain for linear velocity
        self.linear_kd = rospy.get_param('~linear_kd', 0.1)   # D gain for linear velocity (NEW, start small)
        self.angular_kp = rospy.get_param('~angular_kp', 1.5)  # P gain for angular velocity
        self.angular_kd = rospy.get_param('~angular_kd', 0.2)  # D gain for angular velocity

        # Speed limits
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5) # m/s
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0) # rad/s

        # Goal tolerance
        self.pos_tolerance = rospy.get_param('~pos_tolerance', 0.1) # meters
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.1) # radians

        # Rate for the control loop
        self.rate_hz = rospy.get_param('~rate', 10) # Hz

        # --- State Variables ---
        self.current_pose = None
        self.current_yaw = None
        self.pose_received = False

        # Variables for D term calculation
        self.last_time = None
        self.last_heading_error = 0.0
        self.last_pos_error = 0.0 # NEW state for linear D term

        self.waypoints = []
        self.current_waypoint_index = 0

        # --- ROS Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.mocap_pose_topic, PoseStamped, self.pose_callback)

        # --- Load Waypoints ---
        pkg_path = os.path.join(rospy.get_param('~pkg_override_path', os.path.expanduser('~/catkin_ws/src/husky_waypoint_nav')))
        waypoints_full_path = os.path.join(pkg_path, 'config', 'waypoints', self.waypoint_file)
        self.load_waypoints(waypoints_full_path)

        # --- Setup Logging ---
        log_dir = os.path.expanduser(self.log_file_path_param)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        waypoint_filename_base = os.path.splitext(os.path.basename(self.waypoint_file))[0]
        # Use full_pd_nav prefix for log file
        log_filename = os.path.join(log_dir, f"full_pd_nav_{waypoint_filename_base}_{timestamp}.csv")
        self.log_file = open(log_filename, 'w')
        self.csv_writer = csv.writer(self.log_file)

        # --- Write CSV header - Added linear P/D term columns ---
        self.csv_writer.writerow([
            'timestamp', 'waypoint_index',
            'current_x', 'current_y', 'current_yaw',
            'target_x', 'target_y', 'target_yaw',
            'pos_error', 'angle_error',
            'linear_vel_cmd', 'angular_vel_cmd',
            'linear_p_term', 'linear_d_term',      # NEW linear terms logged
            'angular_p_term', 'angular_d_term'
        ])
        rospy.loginfo(f"Logging data to: {log_filename}")

    # --- load_waypoints method (Identical) ---
    def load_waypoints(self, file_path):
        # (Content identical to previous pd_navigator.py - keeping it concise here)
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f); next(reader)
                for row in reader: self.waypoints.append({'x': float(row[1]),'y': float(row[2]),'yaw': float(row[3])})
            rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints from {file_path}")
            if not self.waypoints: rospy.signal_shutdown("No waypoints loaded")
        except Exception as e: rospy.signal_shutdown(f"Waypoint loading failed: {e}")


    # --- pose_callback method (Identical) ---
    def pose_callback(self, msg):
        # (Content identical to previous pd_navigator.py - keeping it concise here)
        self.current_pose = msg.pose
        orientation_q = self.current_pose.orientation; orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list); self.current_yaw = yaw; self.pose_received = True

    # --- get_control_command method - Modified for Full PD ---
    def get_control_command(self, target_pose):
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn_throttle(1.0, "Waiting for current pose from MoCap...")
            # Return Nones for all expected values including NEW linear PD terms
            return None, None, None, None, None, None, None, None

        current_time = rospy.get_time()
        dt = 0.0
        if self.last_time is not None:
            dt = current_time - self.last_time
        # Use estimated dt if first loop or time glitch
        if dt <= 0.001:
             dt = 1.0 / self.rate_hz

        # Calculate errors
        dx = target_pose['x'] - self.current_pose.position.x
        dy = target_pose['y'] - self.current_pose.position.y
        pos_error = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # --- Calculate Derivatives ---
        pos_error_rate = 0.0
        heading_error_rate = 0.0
        if dt > 0.001: # Avoid division by zero on first iteration or if dt is invalid
            pos_error_rate = (pos_error - self.last_pos_error) / dt
            # Use normalized angle difference for heading rate
            heading_error_diff = self.normalize_angle(angle_error - self.last_heading_error)
            heading_error_rate = heading_error_diff / dt

        # --- PD Linear Control ---
        linear_p_term = self.linear_kp * pos_error
        linear_d_term = self.linear_kd * pos_error_rate
        linear_vel = linear_p_term + linear_d_term # PD linear control law

        # --- PD Angular Control ---
        angular_p_term = self.angular_kp * angle_error
        angular_d_term = self.angular_kd * heading_error_rate
        angular_vel = angular_p_term + angular_d_term # PD angular control law

        # --- Update states for next D-term calculation ---
        self.last_time = current_time
        self.last_pos_error = pos_error       # Store current error for next calculation
        self.last_heading_error = angle_error # Store current error for next calculation

        # --- Apply saturation and safety limits (Applied to final combined PD velocity) ---
        linear_vel = max(min(linear_vel, self.max_linear_vel), -self.max_linear_vel)
        # Prioritize turning if angle error is large
        if abs(angle_error) > math.pi / 2 and pos_error > self.pos_tolerance * 0.5 :
             linear_vel = 0 # Stop linear motion to turn
        elif pos_error < self.pos_tolerance:
             pass # Allow backward motion near goal if needed
        else:
             # Generally prevent backward motion unless D-term commands it (e.g., strong damping)
             # This might need adjustment based on tuning. For now, keep max(0,...) might fight D-term damping.
             # Let's allow D-term to command negative velocity for damping, apply limits later.
             # Revisit this if tuning is difficult. Consider only capping positive velocity.
             pass # Allow D-term to potentially create negative velocity for damping

        # Apply limits AFTER combining P and D terms and safety logic
        linear_vel = max(min(linear_vel, self.max_linear_vel), -self.max_linear_vel) # Ensure limits respected
        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

        # Return all values needed for logging
        return linear_vel, angular_vel, pos_error, angle_error, \
               linear_p_term, linear_d_term, angular_p_term, angular_d_term


    # --- normalize_angle method (Identical) ---
    def normalize_angle(self, angle):
        # (Content identical to previous pd_navigator.py - keeping it concise here)
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # --- navigate method (Structured identically, logs new terms) ---
    def navigate(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Starting Full PD navigation...")

        rospy.loginfo("Waiting for MoCap pose data...")
        while not self.pose_received and not rospy.is_shutdown(): rate.sleep()
        rospy.loginfo("MoCap pose received.")

        # Initialize time and errors just before the loop
        self.last_time = rospy.get_time()
        self.last_pos_error = 0.0 # Initialize to zero
        self.last_heading_error = 0.0 # Initialize to zero

        rospy.loginfo("Starting navigation loop.")

        while self.current_waypoint_index < len(self.waypoints) and not rospy.is_shutdown():
            target_pose = self.waypoints[self.current_waypoint_index]

            # Calculate control commands - unpack all terms
            result = self.get_control_command(target_pose)
            if result is None:
                rate.sleep(); continue

            linear_vel, angular_vel, pos_error, angle_error, \
            linear_p_term, linear_d_term, angular_p_term, angular_d_term = result

            # Create Twist message
            twist_cmd = Twist()
            twist_cmd.linear.x = linear_vel
            twist_cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_cmd)

            # Log data - Including new linear P/D terms
            log_time = rospy.get_time()
            log_data = [
                log_time, self.current_waypoint_index,
                self.current_pose.position.x, self.current_pose.position.y, self.current_yaw, # Use self.current_pose.position.y
                target_pose['x'], target_pose['y'], target_pose['yaw'],
                pos_error, angle_error,
                linear_vel, angular_vel,
                linear_p_term, linear_d_term,      # Log linear terms
                angular_p_term, angular_d_term   # Log angular terms
            ]
            self.csv_writer.writerow(log_data)

            # Check if waypoint is reached
            if pos_error < self.pos_tolerance:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached (X: {self.current_pose.position.x:.2f}, Y: {self.current_pose.position.y:.2f})") # Use self.current_pose.position.y
                self.current_waypoint_index += 1
                self.cmd_vel_pub.publish(Twist()) # Stop briefly
                if self.current_waypoint_index < len(self.waypoints):
                     rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index}...")
                     # Reset D-term history for the new segment
                     self.last_time = rospy.get_time()
                     self.last_pos_error = 0.0 # Reset prevents using old error for derivative
                     self.last_heading_error = 0.0
                else:
                     rospy.loginfo("Final waypoint reached.")
                     break

            rate.sleep()

        # Stop the robot after finishing
        rospy.loginfo("Navigation finished or interrupted. Stopping robot.")
        self.cmd_vel_pub.publish(Twist())
        self.log_file.close()

# --- Main execution block (Consistent structure) ---
if __name__ == '__main__':
    try:
        # Instantiate the new class name
        navigator = FullPDNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred in FullPDNavigator: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'navigator' in locals() and hasattr(navigator, 'log_file') and navigator.log_file and not navigator.log_file.closed:
             navigator.log_file.close()
             rospy.loginfo("Log file closed.")