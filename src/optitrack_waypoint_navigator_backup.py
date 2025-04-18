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
        rospy.init_node('husky_optitrack_navigator', anonymous=True)

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.update_pose)
        rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odometry_callback)

        # Robot state
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.current_odometry_yaw = 0.0  # Not used yet

        # Controller tuning
        self.max_speed = 0.4
        self.max_turning_speed = 1.0
        self.angular_gain = 1.5
        self.linear_gain = 0.5
        self.deg_tolerance = 3.0  # degrees
        self.rate = rospy.Rate(60)

        self.prev_cmd = Twist()

        # Load waypoints
        self.waypoints = self.load_waypoints()

        rospy.sleep(1.0)  # let TF/pose fill
        self.move_through_waypoints()

    def update_pose(self, data):
        self.current_pose_x = data.pose.position.x
        self.current_pose_y = data.pose.position.y
        self.current_pose_yaw = self.quaternion_to_euler(
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )

    def odometry_callback(self, data):
        self.current_odometry_yaw = self.quaternion_to_euler(
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

    def quaternion_to_euler(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def load_waypoints(self):
        wp_dir = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        files = sorted(glob.glob(os.path.join(wp_dir, "optitrack_waypoints_*.txt")))
        if not files:
            rospy.logerr("[ERROR] No waypoint files found in: %s", wp_dir)
            rospy.signal_shutdown("No waypoints")
        latest_file = files[-1]
        waypoints = []
        with open(latest_file, 'r') as f:
            for line in f:
                try:
                    x, y, yaw = map(float, line.strip().split())
                    waypoints.append((x, y, yaw))
                except:
                    continue
        rospy.loginfo("[INFO] Loaded %d waypoints from %s", len(waypoints), latest_file)
        return waypoints

    def move_through_waypoints(self):
        for goal_x, goal_y, goal_yaw in self.waypoints:
            while not rospy.is_shutdown():
                distance = math.hypot(goal_x - self.current_pose_x, goal_y - self.current_pose_y)
                yaw_to_goal = math.atan2(goal_y - self.current_pose_y, goal_x - self.current_pose_x)
                yaw_error = self.normalize_angle(yaw_to_goal - self.current_pose_yaw)

                rospy.loginfo_throttle(10.0, "[DEBUG] dist=%.2f, yaw_err=%.2f°", distance, math.degrees(yaw_error))

                if distance < 0.15:
                    rospy.loginfo("[SUCCESS] Reached waypoint")
                    break
 
                cmd = Twist()
                if abs(yaw_error) > math.radians(self.deg_tolerance):
                    cmd.angular.z = max(min(self.angular_gain * yaw_error, self.max_turning_speed), -self.max_turning_speed)
                else:
                    cmd.linear.x = min(self.max_speed, self.linear_gain * distance)

                # Optional: smooth output
                cmd.linear.x = 0.8 * self.prev_cmd.linear.x + 0.2 * cmd.linear.x
                cmd.angular.z = 0.8 * self.prev_cmd.angular.z + 0.2 * cmd.angular.z
                self.prev_cmd = cmd

                self.velocity_publisher.publish(cmd)
                self.rate.sleep()

            # Final yaw alignment
            yaw_error = self.normalize_angle(goal_yaw - self.current_pose_yaw)
            while abs(yaw_error) > math.radians(self.deg_tolerance) and not rospy.is_shutdown():
                cmd = Twist()
                cmd.angular.z = max(min(self.angular_gain * yaw_error, self.max_turning_speed), -self.max_turning_speed)
                self.velocity_publisher.publish(cmd)
                yaw_error = self.normalize_angle(goal_yaw - self.current_pose_yaw)
                self.rate.sleep()

        rospy.loginfo("[DONE] Finished all waypoints")
        self.velocity_publisher.publish(Twist())  # Stop robot
        rospy.spin()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        HuskyOptiTrackNavigator()
    except rospy.ROSInterruptException:
        pass
