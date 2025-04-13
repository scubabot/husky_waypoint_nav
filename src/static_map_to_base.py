#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_tf():
    rospy.init_node('static_map_to_base_link')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf = geometry_msgs.msg.TransformStamped()

    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "map"
    static_tf.child_frame_id = "base_link"

    # Set the fixed transform: here, 1m forward, 0.0 yaw
    static_tf.transform.translation.x = 0.0
    static_tf.transform.translation.y = 0.0
    static_tf.transform.translation.z = 0.0

    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, 0.0)  # yaw = 0 radians

    static_tf.transform.rotation.x = q[0]
    static_tf.transform.rotation.y = q[1]
    static_tf.transform.rotation.z = q[2]
    static_tf.transform.rotation.w = q[3]

    broadcaster.sendTransform(static_tf)
    rospy.loginfo("Static transform published: map → base_link")

    rospy.spin()

if __name__ == '__main__':
    publish_static_tf()
