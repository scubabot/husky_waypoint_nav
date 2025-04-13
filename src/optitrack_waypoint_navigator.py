#!/usr/bin/env python3

import rospy
import actionlib
import tf
import os
import time
import glob
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class OptiTrackWaypointNavigator:
    def __init__(self):
        rospy.init_node("optitrack_waypoint_navigator")

        # Parameters
        self.coordinates_file = rospy.get_param("~husky_waypoint_nav/coordinates_file", None)
        self.pause_button = rospy.get_param("~husky_waypoint_nav/pause_button_num", 3)
        self.resume_button = rospy.get_param("~husky_waypoint_nav/resume_button_num", 1)
        self.loop_mode = rospy.get_param("~husky_waypoint_nav/loop_mode", False)

        self.paused = False
        self.index = 0
        self.waypoints = []
        self.prev_buttons = [0] * 10

        self.cmd_vel_active = False
        self.last_cmd_vel_time = rospy.Time.now()

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base!")

        self.navigate()

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.cmd_vel_active = True
            self.last_cmd_vel_time = rospy.Time.now()

    def navigate(self):
        rospy.loginfo("Loading waypoints and starting navigation...")
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

            rospy.loginfo("\u27a1\ufe0f Sending waypoint %d: x=%.2f, y=%.2f, yaw=%.2f", self.index, x, y, yaw)
            result = self.send_goal(x, y, yaw)

            if result == GoalStatus.SUCCEEDED:
                rospy.loginfo("\u2705 Reached waypoint %d", self.index)
            else:
                rospy.logwarn("\u26a0\ufe0f Failed to reach waypoint %d (status: %d)", self.index, result)

            self.index += 1

            if self.index >= len(self.waypoints) and self.loop_mode:
                rospy.loginfo("\ud83d\udd01 Looping back to first waypoint.")
                try:
                    rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
                    clear_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
                    clear_srv()
                    rospy.loginfo("\ud83e\uddf9 Cleared costmaps before loop restart.")
                except rospy.ROSException:
                    rospy.logwarn("\u26a0\ufe0f Could not call clear_costmaps service.")
                rospy.sleep(2.0)
                self.index = 0

    def joy_callback(self, msg):
        if self._button_pressed(msg.buttons, self.pause_button):
            if not self.paused:
                rospy.logwarn("\u23f8\ufe0f Navigation PAUSED")
            self.paused = True
        elif self._button_pressed(msg.buttons, self.resume_button):
            if self.paused:
                rospy.loginfo("\u25b6\ufe0f Navigation RESUMED")
            self.paused = False
        self.prev_buttons = msg.buttons

    def _button_pressed(self, current, index):
        return current[index] == 1 and self.prev_buttons[index] == 0

    def resolve_path(self, path):
        if path.startswith("/"):
            return path
        base = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        return os.path.join(base, path)

    def load_waypoints(self):
        if self.coordinates_file:
            abs_path = self.resolve_path(self.coordinates_file)
            rospy.loginfo("Loading waypoints from specified file: %s", abs_path)
        else:
            base_path = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
            files = sorted(glob.glob(os.path.join(base_path, "optitrack_waypoints_*.txt")))
            if not files:
                rospy.logerr("\u274c No waypoint files found in: %s", base_path)
                return []
            abs_path = files[-1]
            rospy.logwarn("Using most recent file as fallback: %s", abs_path)

        waypoints = []
        try:
            with open(abs_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 3:
                        x, y, yaw = map(float, parts)
                        waypoints.append((x, y, yaw))
                        rospy.loginfo("  • Loaded: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
        except IOError:
            rospy.logerr("Failed to read waypoint file: %s", abs_path)

        return waypoints

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*quat)

        self.cmd_vel_active = False
        self.client.send_goal(goal)
        rospy.loginfo("\ud83c\udfaf Goal sent. Waiting for result...")

        self.client.wait_for_result(timeout=rospy.Duration(30.0))
        result = self.client.get_state()

        time_since_move = rospy.Time.now() - self.last_cmd_vel_time
        if time_since_move.to_sec() > 3.0:
            rospy.logwarn("\u26a0\ufe0f No recent velocity commands detected from planner!")

        return result

if __name__ == '__main__':
    try:
        OptiTrackWaypointNavigator()
    except rospy.ROSInterruptException:
        pass
