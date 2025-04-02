#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('fake_odom_publisher')
    odom_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    x = 0.0
    y = 0.0
    step = 0.05  # Move 5cm per loop

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # Broadcast transform from odom -> base_link
        br.sendTransform(
            (x, y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            now,
            "base_link",
            "odom"
        )

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        # Simulate forward motion
        x += step

        rate.sleep()

if __name__ == '__main__':
    main()
