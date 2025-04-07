#!/usr/bin/env python3

import rospy
import actionlib
import tf
import os
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy

class OptiTrackWaypointNavigator:
    def __init__(self):
        rospy.init_node("optitrack_waypoint_navigator")

    # Params
        self.coordinates_file = rospy.get_param("~outdoor_waypoint_nav/coordinates_file")
        self.pause_button = rospy.get_param("~outdoor_waypoint_nav/pause_button_num", 3)
        self.resume_button = rospy.get_param("~outdoor_waypoint_nav/resume_button_num", 1)
        self.loop_mode = rospy.get_param("~outdoor_waypoint_nav/loop_mode", False)

        self.paused = False
        self.index = 0
        self.waypoints = []
        self.prev_buttons = [0] * 10

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base!")

        self.navigate()

    def navigate(self):
        rospy.loginfo("Waiting for /outdoor_waypoint_nav/start_nav to be True...")
        while not rospy.get_param("/outdoor_waypoint_nav/start_nav", False) and not rospy.is_shutdown():
            rospy.sleep(0.5)

        rospy.loginfo("Start flag detected. Loading waypoints and starting navigation.")
        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            rospy.logerr("No waypoints loaded. Navigation aborted.")
            return

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.index < len(self.waypoints):
            if self.paused:
                rate.sleep()
                continue
            x, y, yaw = self.waypoints[self.index]
            self.send_goal(x, y, yaw)
            self.index += 1

            if self.index >= len(self.waypoints) and self.loop_mode:
                rospy.loginfo("Looping back to first waypoint.")
                self.index = 0

    def joy_callback(self, msg):
        if self._button_pressed(msg.buttons, self.pause_button):
            if not self.paused:
                rospy.logwarn("Navigation PAUSED")
            self.paused = True

        elif self._button_pressed(msg.buttons, self.resume_button):
            if self.paused:
                rospy.loginfo("Navigation RESUMED")
            self.paused = False

        self.prev_buttons = msg.buttons

    def _button_pressed(self, current, index):
        return current[index] == 1 and self.prev_buttons[index] == 0

    def resolve_path(self, path):
        if path.startswith("/"):
            return path
        base = rospy.get_param("~outdoor_waypoint_nav/package_path", os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config"))
        return os.path.join(base, path)

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
        return waypoints

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"  # or "map" if you use AMCL
        goal.target_pose.header.stamp = rospy.Time.now()
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*quat)

        rospy.loginfo("Sending goal %d: x=%.2f, y=%.2f, yaw=%.2f", self.index, x, y, yaw)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_state()

        if result == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached waypoint %d successfully", self.index)
        else:
            rospy.logwarn("Failed to reach waypoint %d (status %d)", self.index, result)


if __name__ == '__main__':
    try:
        OptiTrackWaypointNavigator()
    except rospy.ROSInterruptException:
        pass
