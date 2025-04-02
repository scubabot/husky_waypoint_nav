#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class GazeboToPoseBridge:
    def __init__(self):
        rospy.init_node('gazebo_to_pose_bridge', anonymous=True)

        self.model_name = rospy.get_param('~model_name', 'husky')
        self.pub = rospy.Publisher('/natnet_ros/Husky/pose', PoseStamped, queue_size=10)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.loginfo("gazebo_to_pose_bridge started. Waiting for /gazebo/model_states...")
        rospy.spin()

    def callback(self, msg):
        if self.model_name in msg.name:
            index = msg.name.index(self.model_name)
            pose = msg.pose[index]

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.pose = pose

            self.pub.publish(pose_stamped)
            rospy.logdebug("Published pose for %s", self.model_name)
        else:
            rospy.logwarn_throttle(10, "Model '%s' not found in /gazebo/model_states", self.model_name)


if __name__ == '__main__':
    try:
        GazeboToPoseBridge()
    except rospy.ROSInterruptException:
        pass
