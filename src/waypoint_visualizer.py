#!/usr/bin/env python3
import rospy
import visualization_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

import os

def load_waypoints(filepath):
    waypoints = []
    if not os.path.exists(filepath):
        rospy.logwarn("Waypoint file not found: %s", filepath)
        return waypoints

    with open(filepath, 'r') as f:
        for line in f:
            try:
                x, y, yaw = map(float, line.strip().split())
                waypoints.append((x, y, yaw))
            except ValueError:
                continue
    return waypoints

def create_marker(id, x, y, yaw):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "waypoints"
    marker.id = id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1

    q = geometry_msgs.msg.Quaternion()
    import tf
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    marker.scale.x = 0.4  # length of arrow
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Color: green for intermediate, red for last
    if id == -1:
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    marker.color.a = 1.0
    return marker

def publish_waypoints(pub, filepath):
    waypoints = load_waypoints(filepath)
    marker_array = MarkerArray()
    for i, (x, y, yaw) in enumerate(waypoints):
        marker = create_marker(i, x, y, yaw)
        if i == len(waypoints) - 1:
            marker.id = -1  # Overwrite ID for last
        marker_array.markers.append(marker)
    pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('waypoint_visualizer')
    file_param = rospy.get_param("/outdoor_waypoint_nav/coordinates_file", "/home/user/optitrack_waypoints.txt")
    abs_path = os.path.join(rospy.get_param("rospack_path", "/root/catkin_ws/src/husky_waypoint_nav"), file_param)
    pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10)
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        publish_waypoints(pub, abs_path)
        rate.sleep()
