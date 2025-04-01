#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def callback(msg):
    cov_msg = PoseWithCovarianceStamped()
    cov_msg.header = msg.header
    cov_msg.pose.pose = msg.pose
    cov_msg.pose.covariance = [0.01] * 36  # small default covariance
    pub.publish(cov_msg)

if __name__ == '__main__':
    rospy.init_node('pose_covariance_converter')
    pub = rospy.Publisher('/natnet_ros/Husky/pose_cov', PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber('/natnet_ros/Husky/pose', PoseStamped, callback)
    rospy.spin()
