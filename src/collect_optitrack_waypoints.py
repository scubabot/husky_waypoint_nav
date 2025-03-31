# Converted from collect_optitrack_waypoints.cpp

import rospy
import os
import math
import tf
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rospkg import RosPack

class WaypointCollector:
    def __init__(self):
        rospy.init_node('collect_optitrack_waypoints')

        self.collect_request = False
        self.continue_collection = True
        self.current_pose = PoseStamped()
        self.last_pose = PoseStamped()
        self.min_distance_change = 0.1  # meters
        self.duration_min = rospy.Duration(1.0)  # time between samples

        self.collect_button_num = rospy.get_param("/outdoor_waypoint_nav/collect_button_num", 6)
        self.end_button_num = rospy.get_param("/outdoor_waypoint_nav/end_button_num", 7)
        self.collect_button_sym = rospy.get_param("/outdoor_waypoint_nav/collect_button_sym", "L2")
        self.end_button_sym = rospy.get_param("/outdoor_waypoint_nav/end_button_sym", "R2")

        file_param = rospy.get_param("/outdoor_waypoint_nav/coordinates_file")
        path_abs = os.path.join(RosPack().get_path("husky_waypoint_nav"), file_param)
        self.file = open(path_abs, "a")

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.pose_sub = rospy.Subscriber("/natnet_ros/Husky/pose", PoseStamped, self.pose_callback)
        self.done_pub = rospy.Publisher("/outdoor_waypoint_nav/collection_status", Bool, queue_size=10)

        self.time_last = rospy.Time.now()
        self.num_waypoints = 0

        rospy.loginfo(f"Saving OptiTrack waypoints to: {path_abs}")
        print(f"Press {self.collect_button_sym} to collect waypoint.\nPress {self.end_button_sym} to end collection.\n")

        self.run()

    def joy_callback(self, msg):
        self.collect_request = (msg.buttons[self.collect_button_num] == 1)
        if msg.buttons[self.end_button_num] == 1:
            self.continue_collection = False

    def pose_callback(self, msg):
        self.current_pose = msg

    def distance(self, a, b):
        return math.hypot(a.pose.position.x - b.pose.position.x,
                          a.pose.position.y - b.pose.position.y)

    def get_yaw(self, orientation):
        _, _, yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        return yaw

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.continue_collection:
            rospy.spin()
            if self.collect_request and (rospy.Time.now() - self.time_last > self.duration_min):
                self.collect_request = False
                if self.distance(self.current_pose, self.last_pose) > self.min_distance_change:
                    x = self.current_pose.pose.position.x
                    y = self.current_pose.pose.position.y
                    yaw = self.get_yaw(self.current_pose.pose.orientation)
                    self.file.write(f"{x:.4f} {y:.4f} {yaw:.4f}\n")
                    self.last_pose = self.current_pose
                    self.num_waypoints += 1
                    self.time_last = rospy.Time.now()
                    rospy.loginfo(f"Waypoint saved: [x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}]")
                else:
                    rospy.logwarn("Too close to previous point. Move further to add new waypoint.")
            rate.sleep()

        self.file.close()
        rospy.loginfo(f"Saved {self.num_waypoints} waypoints. Shutting down collector node.")
        self.done_pub.publish(Bool(data=True))

if __name__ == '__main__':
    try:
        WaypointCollector()
    except rospy.ROSInterruptException:
        pass
