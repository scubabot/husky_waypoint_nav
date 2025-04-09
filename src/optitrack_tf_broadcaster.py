#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class OptiTrackTFBroadcaster:
    def __init__(self):
        # We'll broadcast a transform called "map" -> "base_link".
        self.br = tf.TransformBroadcaster()

        # Subscribe to the PoseStamped published by OptiTrack:
        rospy.Subscriber("/natnet_ros_cpp/Husky/pose", PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        # Extract translation:
        translation = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        # Extract orientation (quaternion):
        orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        # Broadcast the transform so other nodes see "map -> base_link" in TF
        # with the same timestamp as the Pose:
        self.br.sendTransform(
            translation,
            orientation,
            msg.header.stamp,  # important to keep timing consistent
            "base_link",       # child frame
            "map"              # parent frame
        )

if __name__ == "__main__":
    rospy.init_node("optitrack_tf_broadcaster")
    node = OptiTrackTFBroadcaster()
    rospy.spin()
