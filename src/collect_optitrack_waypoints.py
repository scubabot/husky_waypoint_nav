#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import tf
import os

class WaypointCollector:
    def __init__(self):
        self.file_path = rospy.get_param("~outdoor_waypoint_nav/coordinates_file", "/tmp/optitrack_waypoints.txt")
        self.collect_button = rospy.get_param("~outdoor_waypoint_nav/collect_button_num", 6)
        self.end_button = rospy.get_param("~outdoor_waypoint_nav/end_button_num", 7)
        self.collect_button_sym = rospy.get_param("~outdoor_waypoint_nav/collect_button_sym", "L2")
        self.end_button_sym = rospy.get_param("~outdoor_waypoint_nav/end_button_sym", "R2")
        self.num_waypoints = 0
        self.tf_listener = tf.TransformListener()

        # Ensure the directory exists
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        try:
            self.file = open(self.file_path, "w")
            rospy.loginfo("Saving OptiTrack waypoints to: %s" % self.file_path)
        except IOError:
            rospy.logerr("Unable to open waypoint file: %s" % self.file_path)
            rospy.signal_shutdown("File error")
            return

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        print("Press %s to collect waypoint.\nPress %s to end collection." % (self.collect_button_sym, self.end_button_sym))

    def joy_callback(self, data):

        rospy.loginfo("Joystick callback triggered.")
        rospy.loginfo("Buttons pressed: %s", data.buttons)

        if data.buttons[self.collect_button]:
            rospy.loginfo("Collect button (%s) pressed", self.collect_button_sym)
            self.save_current_pose()
        elif data.buttons[self.end_button]:
            rospy.loginfo("End button (%s) pressed", self.end_button_sym)
            rospy.loginfo("Saved %d waypoints. Shutting down collector node." % self.num_waypoints)
            self.file.close()
            rospy.signal_shutdown("Waypoint collection complete")

    def save_current_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            x, y = trans[0], trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]

            # Fixed this line (string syntax!)
            self.file.write(f"{x:.3f} {y:.3f} {yaw:.3f}\n")
            self.file.flush()
            self.num_waypoints += 1
            rospy.loginfo("Waypoint saved: [x: %.2f, y: %.2f, yaw: %.2f]" % (x, y, yaw))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('collect_optitrack_waypoints')
    WaypointCollector()
    rospy.spin()
