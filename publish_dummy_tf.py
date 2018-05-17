#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry

def cam_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform( (0, 0, 0),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "zed_left_camera",
                     "map")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/zed/odom', Odometry, cam_pose)
    rospy.spin()