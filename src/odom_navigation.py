#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomPNavigator:
    def __init__(self):
        rospy.init_node('odom_p_navigator', anonymous=True)

        # --- Parameters ---
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/husky_velocity_controller/cmd_vel')
        self.linear_kp = rospy.get_param('~linear_kp', 0.5)
        self.angular_kp = rospy.get_param('~angular_kp', 1.5)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.4)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.8)
        self.pos_tolerance = rospy.get_param('~pos_tolerance', 0.15)
        self.rate_hz = rospy.get_param('~rate', 10)

        # --- Waypoints (Hardcoded, calculated relative to specific MoCap start) ---
        # IMPORTANT: These waypoints assume the robot starts at the MoCap pose:
        # X = -0.7950, Y = 0.6837, Yaw = 1.1860 rad
        # These targets are in the INITIAL /odom frame associated with that start.
        # The robot's actual path will drift due to odometry errors.
        self.waypoints = [
            Point(x=1.1974, y=-2.9258, z=0), Point(x=1.1541, y=-1.6607, z=0),
            Point(x=1.0329, y=-0.6427, z=0), Point(x=1.2210, y=0.9599, z=0), # Corrected typo 1.221 not 1.221,
            Point(x=1.7426, y=0.8527, z=0), Point(x=2.8374, y=-0.6816, z=0),
            Point(x=3.7729, y=-1.4410, z=0), Point(x=3.9232, y=-0.1989, z=0), # Corrected typo 3.7729 not 3.7729,
            Point(x=3.0456, y=0.5297, z=0), Point(x=1.9088, y=0.3682, z=0),
            Point(x=0.9136, y=0.4303, z=0), Point(x=0.1606, y=0.0854, z=0)
        ]
        rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints.")
        rospy.loginfo("Waypoints are relative to MoCap start: X=-0.795, Y=0.684, Yaw=1.186 rad")
        if not self.waypoints:
            rospy.logerr("No waypoints defined.")
            rospy.signal_shutdown("No waypoints")

        # --- State Variables ---
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_yaw = 0.0
        self.odom_received = False
        self.current_waypoint_index = 0

        # --- ROS Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)


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

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Starting Odometry P navigation...")

        rospy.loginfo(f"Waiting for odometry data on topic {self.odom_topic}...")
        while not self.odom_received and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("Odometry data received. Starting navigation loop.")

        while self.current_waypoint_index < len(self.waypoints) and not rospy.is_shutdown():
            target_point = self.waypoints[self.current_waypoint_index]
            target_x = target_point.x
            target_y = target_point.y

            dx = target_x - self.current_odom_x
            dy = target_y - self.current_odom_y
            pos_error = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.current_odom_yaw)

            linear_vel = self.linear_kp * pos_error
            angular_vel = self.angular_kp * angle_error

            linear_vel = max(min(linear_vel, self.max_linear_vel), 0.0)
            angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

            if abs(angle_error) > math.pi / 4:
                linear_vel = 0.0

            twist_cmd = Twist()
            twist_cmd.linear.x = linear_vel
            twist_cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_cmd)

            if pos_error < self.pos_tolerance:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached (Odom Pose: X={self.current_odom_x:.2f}, Y={self.current_odom_y:.2f})")
                self.current_waypoint_index += 1
                self.cmd_vel_pub.publish(Twist()) # Stop
                if self.current_waypoint_index < len(self.waypoints):
                     rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index}...")
                     rospy.sleep(0.5)
                else:
                     rospy.loginfo("Final waypoint reached.")
                     break

            rate.sleep()

        rospy.loginfo("Navigation finished or interrupted. Stopping robot.")
        self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        navigator = OdomPNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred in OdomPNavigator: {e}")
        import traceback
        traceback.print_exc()