#!/usr/bin/env python

import rospy
import csv
import math
import time
import os
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry # Import Odometry message
from tf.transformations import euler_from_quaternion

class ProportionalNavigator:
    def __init__(self):
        rospy.init_node('p_navigator', anonymous=True)

        # --- Parameters ---
        self.husky_name = rospy.get_param('~husky_name', 'husky')
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/natnet_ros/' + self.husky_name + '/pose')
        # Use the verified odometry topic name
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/husky_velocity_controller/cmd_vel')
        self.waypoint_file = rospy.get_param('~waypoint_file', 'optitrack_waypoints_06.txt')
        self.log_file_path_param = rospy.get_param('~log_file_path', '~/catkin_ws/src/husky_waypoint_nav/logs')

        # Control gains - USING ORIGINAL NAMES
        self.linear_kp = rospy.get_param('~linear_kp', 0.5) # P gain for linear velocity
        self.angular_kp = rospy.get_param('~angular_kp', 1.5) # P gain for angular velocity

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

        # Odometry state variables
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_yaw = 0.0
        self.odom_received = False

        self.waypoints = []
        self.current_waypoint_index = 0

        # --- ROS Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.mocap_pose_topic, PoseStamped, self.pose_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

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
        log_filename = os.path.join(log_dir, f"p_nav_{waypoint_filename_base}_{timestamp}.csv")
        self.log_file = open(log_filename, 'w')
        self.csv_writer = csv.writer(self.log_file)
        # Write CSV header
        self.csv_writer.writerow([
            'timestamp', 'waypoint_index',
            'current_x', 'current_y', 'current_yaw',
            'target_x', 'target_y', 'target_yaw',
            'odom_x', 'odom_y', 'odom_yaw',
            'pos_error', 'angle_error',
            'linear_vel_cmd', 'angular_vel_cmd'
        ])
        rospy.loginfo(f"Logging data to: {log_filename}")

    def load_waypoints(self, file_path):
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                next(reader) # Skip header row
                for row in reader:
                    self.waypoints.append({
                        'x': float(row[1]),
                        'y': float(row[2]),
                        'yaw': float(row[3])
                    })
            rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints from {file_path}")
            if not self.waypoints:
                 rospy.logerr("No waypoints loaded.")
                 rospy.signal_shutdown("No waypoints loaded")
        except Exception as e:
            rospy.logerr(f"Error loading waypoints from {file_path}: {e}")
            rospy.signal_shutdown("Waypoint loading failed")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        self.pose_received = True

    def odom_callback(self, msg):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_odom_yaw = yaw
        if not self.odom_received:
             rospy.loginfo("First odometry message received.")
        self.odom_received = True

    def get_control_command(self, target_pose):
        if self.current_pose is None or self.current_yaw is None:
            rospy.logwarn_throttle(1.0, "Waiting for current pose from MoCap...")
            return None, None, None, None

        dx = target_pose['x'] - self.current_pose.position.x
        dy = target_pose['y'] - self.current_pose.position.y
        position_error = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Proportional control laws - USING ORIGINAL NAMES
        linear_vel = self.linear_kp * position_error
        angular_vel = self.angular_kp * angle_error

        # Apply saturation limits
        linear_vel = max(min(linear_vel, self.max_linear_vel), -self.max_linear_vel)
        if abs(angle_error) > math.pi / 2 and position_error > self.pos_tolerance * 0.5 :
             linear_vel = 0
        elif position_error < self.pos_tolerance:
             pass
        else:
             linear_vel = max(0, linear_vel)

        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

        return linear_vel, angular_vel, position_error, angle_error


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def navigate(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Starting P navigation...")

        rospy.loginfo("Waiting for MoCap pose data...")
        while not self.pose_received and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("MoCap pose received.")

        rospy.loginfo(f"Waiting for odometry data on topic {self.odom_topic}...")
        while not self.odom_received and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("Odometry data received.")

        rospy.loginfo("Starting navigation loop.")

        while self.current_waypoint_index < len(self.waypoints) and not rospy.is_shutdown():
            target_pose = self.waypoints[self.current_waypoint_index]
            linear_vel, angular_vel, pos_error, angle_error = self.get_control_command(target_pose)

            if linear_vel is None:
                rate.sleep()
                continue

            twist_cmd = Twist()
            twist_cmd.linear.x = linear_vel
            twist_cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_cmd)

            current_time = rospy.get_time()
            log_data = [
                current_time, self.current_waypoint_index,
                self.current_pose.position.x, self.current_pose.position.y, self.current_yaw,
                target_pose['x'], target_pose['y'], target_pose['yaw'],
                self.current_odom_x, self.current_odom_y, self.current_odom_yaw,
                pos_error, angle_error,
                linear_vel, angular_vel
            ]
            self.csv_writer.writerow(log_data)

            if pos_error < self.pos_tolerance:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached (X: {self.current_pose.position.x:.2f}, Y: {self.current_pose.position.y:.2f})")
                self.current_waypoint_index += 1
                self.cmd_vel_pub.publish(Twist())
                if self.current_waypoint_index < len(self.waypoints):
                     rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index}...")
                else:
                     rospy.loginfo("Final waypoint reached.")
                     break

            rate.sleep()

        rospy.loginfo("Navigation finished or interrupted. Stopping robot.")
        self.cmd_vel_pub.publish(Twist())
        self.log_file.close()

if __name__ == '__main__':
    try:
        navigator = ProportionalNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        if 'navigator' in locals() and hasattr(navigator, 'log_file') and navigator.log_file and not navigator.log_file.closed:
             navigator.log_file.close()
             rospy.loginfo("Log file closed due to error or interruption.")