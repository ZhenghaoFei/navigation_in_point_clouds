#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry

def cam_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform( (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ),
                      (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                      msg.header.stamp,
                     "zed_left_camera",
                     "map")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/zed/odom', Odometry, cam_pose)
    rospy.spin()