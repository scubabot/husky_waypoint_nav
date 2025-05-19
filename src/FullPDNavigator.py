#!/usr/bin/env python

import rospy
import csv
import math
import time
import os
import traceback # Import for detailed error logging
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

# This script uses MoCap for control and logs PD terms.
# It does not subscribe to or use /odom for control.

class FullPDNavigator:
    def __init__(self):
        rospy.init_node('full_pd_navigator', anonymous=True)

        # --- Parameters ---
        # These names must match the <param name="..."/> in the launch file
        self.husky_name = rospy.get_param('~husky_name', 'husky') # Used only if mocap topic not overridden
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/natnet_ros/' + self.husky_name + '/pose')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/husky_velocity_controller/cmd_vel')
        self.waypoint_file = rospy.get_param('~waypoint_file', 'optitrack_waypoints_06.txt')
        self.log_file_path_param = rospy.get_param('~log_file_path', '~/catkin_ws/src/husky_waypoint_nav/logs')

        # --- PD Control gains ---
        self.linear_kp = rospy.get_param('~linear_kp', 0.5)
        self.linear_kd = rospy.get_param('~linear_kd', 0.1)
        self.angular_kp = rospy.get_param('~angular_kp', 1.5)
        self.angular_kd = rospy.get_param('~angular_kd', 0.2)

        # Speed limits
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)

        # Goal tolerance
        self.pos_tolerance = rospy.get_param('~pos_tolerance', 0.1)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.1) # Radians

        # Rate for the control loop
        self.rate_hz = rospy.get_param('~rate', 10) # Defaulted to 10Hz, launch used 60Hz

        # --- State Variables ---
        self.current_pose = None
        self.current_yaw = None
        self.pose_received = False

        self.last_time = None
        self.last_heading_error = 0.0
        self.last_pos_error = 0.0

        self.waypoints = []
        self.current_waypoint_index = 0

        # --- ROS Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.mocap_pose_topic, PoseStamped, self.pose_callback)

        # --- Load Waypoints ---
        pkg_path_default = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'husky_waypoint_nav')
        pkg_path = rospy.get_param('~pkg_override_path', pkg_path_default)
        waypoints_full_path = os.path.join(pkg_path, 'config', 'waypoints', self.waypoint_file)
        self.load_waypoints(waypoints_full_path)
        if not self.waypoints:
             rospy.logerr("Waypoint loading failed or resulted in empty list in __init__. Shutting down.")
             rospy.signal_shutdown("Waypoints not loaded")
             raise RuntimeError("Waypoints not loaded") # Prevent further execution

        # --- Setup Logging ---
        self.log_file = None
        self.csv_writer = None
        log_dir = os.path.expanduser(self.log_file_path_param)
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError as e:
                 rospy.logerr(f"Failed to create log directory {log_dir}: {e}")
                 self.log_file_path_param = None

        if self.log_file_path_param:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            try:
                waypoint_filename_base = os.path.splitext(os.path.basename(self.waypoint_file))[0]
                log_filename = os.path.join(log_dir, f"full_pd_nav_{waypoint_filename_base}_{timestamp}.csv")
                self.log_file = open(log_filename, 'w')
                self.csv_writer = csv.writer(self.log_file)
                self.csv_writer.writerow([
                    'timestamp', 'waypoint_index',
                    'mocap_x', 'mocap_y', 'mocap_yaw',
                    'target_x', 'target_y', 'target_yaw',
                    'pos_error', 'angle_error',
                    'linear_vel_cmd', 'angular_vel_cmd',
                    'linear_p_term', 'linear_d_term',
                    'angular_p_term', 'angular_d_term'
                ])
                rospy.loginfo(f"Logging data to: {log_filename}")
            except Exception as e:
                 rospy.logerr(f"Failed to setup logging to {log_dir}: {e}")
                 if self.log_file: self.log_file.close()
                 self.log_file = None

        # --- Wait for Initial MoCap Pose (Moved to end of __init__) ---
        rospy.loginfo("Waiting for initial MoCap pose on %s...", self.mocap_pose_topic)
        rate_wait = rospy.Rate(10) # Use a reasonable rate for waiting
        while not self.pose_received and not rospy.is_shutdown():
             rate_wait.sleep()

        if rospy.is_shutdown():
             rospy.logwarn("Shutdown requested during initialization wait.")
             return # Exit __init__ if shutdown

        rospy.loginfo(f"Initial MoCap pose received: X={self.current_pose.position.x:.2f}, Y={self.current_pose.position.y:.2f}")

        # --- Initialize D-Term History (After first pose received) ---
        self.last_time = rospy.get_time()
        if self.waypoints and self.current_pose:
            target_pose_init = self.waypoints[0]
            dx_init = target_pose_init['x'] - self.current_pose.position.x
            dy_init = target_pose_init['y'] - self.current_pose.position.y
            self.last_pos_error = math.sqrt(dx_init**2 + dy_init**2)
            angle_to_goal_init = math.atan2(dy_init, dx_init)
            self.last_heading_error = self.normalize_angle(angle_to_goal_init - self.current_yaw)
            rospy.loginfo("D-term history initialized.")
        else:
             self.last_pos_error = 0.0
             self.last_heading_error = 0.0
             rospy.logwarn("Could not initialize D-term history properly (missing pose or waypoints).")
        
        rospy.loginfo("Initialization complete. Ready for navigation.")


    def load_waypoints(self, file_path):
        """Loads waypoints (X, Y) from the specified text file, assuming X Y format."""
        rospy.loginfo(f"Attempting to load waypoints from: {file_path}")
        waypoints_loaded = []
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f, delimiter=' ', skipinitialspace=True)
                line_num = 0
                for row in reader:
                    line_num += 1
                    if not row or not row[0] or row[0].strip().startswith('#'): continue
                    cleaned_row = [val for val in row if val]
                    if len(cleaned_row) >= 2:
                        try:
                            x = float(cleaned_row[0])
                            y = float(cleaned_row[1])
                            default_yaw = 0.0
                            waypoints_loaded.append({'x': x, 'y': y, 'yaw': default_yaw})
                        except ValueError:
                            rospy.logwarn(f"Skipping non-numeric data in line {line_num} of {file_path}: '{row}'")
                            continue
                    else:
                        rospy.logwarn(f"Skipping malformed line {line_num} in {file_path}: '{row}' (Expected at least 2 values: X Y)")
            self.waypoints = waypoints_loaded
            rospy.loginfo(f"Successfully loaded {len(self.waypoints)} waypoints (X, Y, defaulted Yaw) from {file_path}")
        except IOError as e:
            rospy.logerr(f"Error reading waypoint file {file_path}: {e}")
            self.waypoints = []
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred loading waypoints from {file_path}: {e}")
            traceback.print_exc()
            self.waypoints = []


    def pose_callback(self, msg):
        self.current_pose = msg.pose
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        if not self.pose_received:
            rospy.loginfo_once("First MoCap message processed in callback.") # Log only once
        self.pose_received = True

    def get_control_command(self, target_pose):
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn_throttle(1.0, "get_control_command: Waiting for current pose.")
            return None

        current_time = rospy.get_time()
        dt = 0.0
        if self.last_time is not None: dt = current_time - self.last_time
        if dt <= 0.001: dt = 1.0 / self.rate_hz

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.current_yaw
        target_x = target_pose['x']
        target_y = target_pose['y']

        dx = target_x - current_x
        dy = target_y - current_y
        pos_error = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - current_yaw)

        pos_error_rate = 0.0
        heading_error_rate = 0.0
        if dt > 0.001:
            pos_error_rate = (pos_error - self.last_pos_error) / dt
            heading_error_diff = self.normalize_angle(angle_error - self.last_heading_error)
            heading_error_rate = heading_error_diff / dt

        linear_p_term = self.linear_kp * pos_error
        linear_d_term = self.linear_kd * pos_error_rate
        linear_vel = linear_p_term + linear_d_term

        angular_p_term = self.angular_kp * angle_error
        angular_d_term = self.angular_kd * heading_error_rate
        angular_vel = angular_p_term + angular_d_term

        self.last_time = current_time
        self.last_pos_error = pos_error
        self.last_heading_error = angle_error

        if abs(angle_error) > math.pi / 2 and pos_error > self.pos_tolerance * 0.5 :
             linear_vel = 0.0

        linear_vel = max(min(linear_vel, self.max_linear_vel), -self.max_linear_vel)
        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

        return linear_vel, angular_vel, pos_error, angle_error, \
               linear_p_term, linear_d_term, angular_p_term, angular_d_term


    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def navigate(self):
        if not self.waypoints or not self.pose_received:
             rospy.logerr("Dependencies not met for navigation start (navigate called).")
             return

        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Starting Full PD navigation loop...")

        while self.current_waypoint_index < len(self.waypoints) and not rospy.is_shutdown():
            target_pose = self.waypoints[self.current_waypoint_index]

            result = self.get_control_command(target_pose)
            if result is None:
                rospy.logwarn_throttle(2.0, "Control command calculation skipped, waiting for pose.")
                rate.sleep(); continue

            linear_vel, angular_vel, pos_error, angle_error, \
            linear_p_term, linear_d_term, angular_p_term, angular_d_term = result

            twist_cmd = Twist()
            twist_cmd.linear.x = linear_vel
            twist_cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_cmd)

            if self.csv_writer:
                current_time = rospy.get_time()
                log_mocap_x = self.current_pose.position.x if self.current_pose else None
                log_mocap_y = self.current_pose.position.y if self.current_pose else None
                log_mocap_yaw = self.current_yaw if self.current_yaw is not None else None

                log_data = [
                    f"{current_time:.4f}", self.current_waypoint_index + 1,
                    f"{log_mocap_x:.4f}" if log_mocap_x is not None else "",
                    f"{log_mocap_y:.4f}" if log_mocap_y is not None else "",
                    f"{log_mocap_yaw:.4f}" if log_mocap_yaw is not None else "",
                    f"{target_pose['x']:.4f}", f"{target_pose['y']:.4f}", f"{target_pose['yaw']:.4f}",
                    f"{pos_error:.4f}", f"{angle_error:.4f}",
                    f"{linear_vel:.4f}", f"{angular_vel:.4f}",
                    f"{linear_p_term:.4f}", f"{linear_d_term:.4f}",
                    f"{angular_p_term:.4f}", f"{angular_d_term:.4f}"
                ]
                try:
                    self.csv_writer.writerow(log_data)
                except Exception as e:
                     rospy.logwarn_throttle(10, f"Error writing log data: {e}")

            if pos_error < self.pos_tolerance:
                log_mocap_x = self.current_pose.position.x if self.current_pose else float('nan')
                log_mocap_y = self.current_pose.position.y if self.current_pose else float('nan')
                rospy.loginfo(f"Waypoint {self.current_waypoint_index + 1} reached (MoCap Pose: X={log_mocap_x:.2f}, Y={log_mocap_y:.2f})")
                self.current_waypoint_index += 1
                self.cmd_vel_pub.publish(Twist())

                if self.current_waypoint_index < len(self.waypoints):
                     rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index + 1}...")
                     self.last_time = rospy.get_time()
                     if self.current_pose:
                         next_target_pose = self.waypoints[self.current_waypoint_index]
                         dx_next = next_target_pose['x'] - self.current_pose.position.x
                         dy_next = next_target_pose['y'] - self.current_pose.position.y
                         self.last_pos_error = math.sqrt(dx_next**2 + dy_next**2)
                         angle_to_goal_next = math.atan2(dy_next, dx_next)
                         self.last_heading_error = self.normalize_angle(angle_to_goal_next - self.current_yaw)
                     else:
                         self.last_pos_error = 0.0
                         self.last_heading_error = 0.0
                     rospy.sleep(0.5)
                else:
                     rospy.loginfo("Final waypoint reached.")
                     break
            rate.sleep()

        rospy.loginfo("Navigation loop finished or interrupted.")
        self.cleanup() # Call cleanup when navigation loop ends or is interrupted


    def cleanup(self):
        rospy.loginfo("Executing cleanup...")
        try:
             stop_cmd = Twist()
             if hasattr(self, 'cmd_vel_pub') and self.cmd_vel_pub.get_num_connections() > 0:
                 self.cmd_vel_pub.publish(stop_cmd); rospy.sleep(0.1)
                 self.cmd_vel_pub.publish(stop_cmd); rospy.loginfo("Stop command sent via cleanup.")
        except Exception as e:
             rospy.logwarn(f"Exception sending stop command during cleanup: {e}")
        if hasattr(self, 'log_file') and self.log_file and not self.log_file.closed:
            rospy.loginfo(f"Closing log file via cleanup: {self.log_file.name}")
            self.log_file.close()
            self.log_file = None
            self.csv_writer = None


if __name__ == '__main__':
    navigator = None
    try:
        navigator = FullPDNavigator() # __init__ runs, including waits for pose
        # Check if initialization succeeded (waypoints loaded, pose received) and ROS is ok
        if navigator and navigator.waypoints and navigator.pose_received and not rospy.is_shutdown():
             navigator.navigate() # Call navigate to start the loop
        elif not rospy.is_shutdown(): # Only log error if not already shutting down
             rospy.logerr("Navigator initialization checks failed or shutdown requested before navigation start.")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted by ROS shutdown.")
    except RuntimeError as e: # Catch specific RuntimeError from __init__
        rospy.logerr(f"Runtime error during initialization: {e}")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred during navigation execution: {e}")
        traceback.print_exc()
    finally:
        rospy.loginfo("Executing final cleanup from __main__...")
        if navigator:
             navigator.cleanup()