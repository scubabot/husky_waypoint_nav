#!/usr/bin/env python3

import rospy
import actionlib
import tf
from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
import os
import math

class OptiTrackWaypointNavigator:
    def __init__(self):
        rospy.init_node("optitrack_waypoint_navigator")

        # Params
        self.coordinates_file = rospy.get_param("~outdoor_waypoint_nav/coordinates_file")
        self.pause_button = rospy.get_param("~outdoor_waypoint_nav/pause_button_num", 3)
        self.resume_button = rospy.get_param("~outdoor_waypoint_nav/resume_button_num", 1)
        self.loop_mode = rospy.get_param("~outdoor_waypoint_nav/loop_mode", False)

        # State
        self.paused = False
        self.index = 0
        self.waypoints = self.load_waypoints()
        rospy.loginfo("Navigator: Waypoints list = %s", str(self.waypoints))

        # Subscribers
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Action client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base!")
        print("[NAV DEBUG] Move_base connected, ready to send goals.")

        # Start navigation
        rospy.sleep(2.0)
        rospy.loginfo("Starting navigation loop")
        self.navigate()

    def joy_callback(self, msg):
        rospy.loginfo("Joystick callback triggered (navigator).")
        rospy.loginfo("Buttons pressed: %s", msg.buttons)
        if msg.buttons[self.pause_button] == 1:
            if not self.paused:
                rospy.logwarn("Waypoint navigation PAUSED")
            self.paused = True
        elif msg.buttons[self.resume_button] == 1:
            if self.paused:
                rospy.loginfo("Waypoint navigation RESUMED")
            self.paused = False

    def load_waypoints(self):
        abs_path = self.resolve_path(self.coordinates_file)
        rospy.loginfo("Loading waypoints from: %s", abs_path)
        waypoints = []
        try:
            with open(abs_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 3:
                        x, y, yaw = map(float, parts)
                        waypoints.append((x, y, yaw))
                        rospy.loginfo("Loaded waypoint: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
        except IOError:
            rospy.logerr("Failed to load waypoints file: %s" % abs_path)
            rospy.signal_shutdown("No waypoints found.")
        rospy.loginfo("Loaded %d waypoints." % len(waypoints))
        return waypoints

    def resolve_path(self, path):
        if path.startswith("/"):
            return path
        # You can customize this default location
        pkg_path = rospy.get_param("~outdoor_waypoint_nav/package_path", os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config"))
        return os.path.join(pkg_path, path)

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))

        rospy.loginfo("Sending goal %d: x=%.2f, y=%.2f, yaw=%.2f" % (self.index, x, y, yaw))
        rospy.loginfo("Navigator: Sending MoveBaseGoal now ...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached waypoint %d" % self.index)
        else:
            rospy.logwarn("Failed to reach waypoint %d (status: %d)" % (self.index, state))

    def navigate(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.index < len(self.waypoints):
            if self.paused:
                rate.sleep()
                continue

            x, y, yaw = self.waypoints[self.index]
            self.send_goal(x, y, yaw)

            self.index += 1
            rospy.loginfo("Navigator: Moving to next index = %d", self.index)
            if self.index >= len(self.waypoints) and self.loop_mode:
                rospy.loginfo("Looping back to first waypoint.")
                self.index = 0

if __name__ == '__main__':
    try:
        OptiTrackWaypointNavigator()
    except rospy.ROSInterruptException:
        pass
