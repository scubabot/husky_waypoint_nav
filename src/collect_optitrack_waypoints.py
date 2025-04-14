#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import tf
import os
import glob

class WaypointCollector:
    def __init__(self):
        # Set and create waypoint directory
        waypoint_dir = os.path.expanduser("~/catkin_ws/src/husky_waypoint_nav/config/waypoints")
        os.makedirs(waypoint_dir, exist_ok=True)

        # Auto-increment filename like optitrack_waypoints_01.txt
        existing = glob.glob(os.path.join(waypoint_dir, "optitrack_waypoints_*.txt"))
        indices = []
        for f in existing:
            try:
                num = int(os.path.basename(f).split('_')[-1].split('.')[0])
                indices.append(num)
            except ValueError:
                continue
        next_index = max(indices + [0]) + 1
        self.file_path = os.path.join(waypoint_dir, f"optitrack_waypoints_{next_index:02d}.txt")

        self.collect_button = rospy.get_param("~husky_waypoint_nav/collect_button_num", 2)  # Square
        self.end_button = rospy.get_param("~husky_waypoint_nav/end_button_num", 0)          # Cross
        self.collect_button_sym = rospy.get_param("~husky_waypoint_nav/collect_button_sym", "square")
        self.end_button_sym = rospy.get_param("~husky_waypoint_nav/end_button_sym", "cross")
        self.num_waypoints = 0
        self.tf_listener = tf.TransformListener()

        try:
            self.file = open(self.file_path, "w")
            rospy.loginfo("[INFO] Saving OptiTrack waypoints to: %s", self.file_path)
        except IOError:
            rospy.logerr("[ERROR] Unable to open waypoint file: %s", self.file_path)
            rospy.signal_shutdown("File error")
            return

        self.last_collect_state = False
        self.last_end_state = False
        rospy.Subscriber("/joy_teleop/joy", Joy, self.joy_callback)

        rospy.loginfo("[INFO] Ready to collect waypoints: [%s] to collect, [%s] to finish.",
                      self.collect_button_sym, self.end_button_sym)

    def joy_callback(self, data):
        collect_now = data.buttons[self.collect_button]
        end_now = data.buttons[self.end_button]

        if collect_now and not self.last_collect_state:
            rospy.loginfo("[LOG] [%s] Collect button pressed", self.collect_button_sym)
            self.save_current_pose()

        if end_now and not self.last_end_state:
            rospy.loginfo("[LOG] [%s] End button pressed", self.end_button_sym)
            rospy.loginfo("[LOG] Saved %d waypoint(s).", self.num_waypoints)
            self.file.close()
            rospy.signal_shutdown("Waypoint collection complete")

        self.last_collect_state = collect_now
        self.last_end_state = end_now

    def save_current_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            x, y = trans[0], trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            self.file.write(f"{x:.3f} {y:.3f} {yaw:.3f}\n")
            self.file.flush()
            self.num_waypoints += 1
            rospy.loginfo("[INFO] Waypoint saved: [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("[ERROR]TF lookup failed: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('collect_optitrack_waypoints')
    WaypointCollector()
    rospy.spin()
