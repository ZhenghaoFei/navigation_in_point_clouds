#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import math

yaw = 0.93863332 + math.pi/4
q_init = tf.transformations.quaternion_from_euler(0, 0, yaw)



def static_tf(msg):
    br = tf.TransformBroadcaster()
   
    # br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
    #                  (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
    #                   msg.header.stamp,
    #                  "base_link",
    #                  "odom")


    br.sendTransform( (0, 0, 0, 0 ),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "odom",
                     "map")


    br.sendTransform( (0, 0, 0, 0 ),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "zed_left_camera",
                     "base_link")

    br.sendTransform( (0, 0, 0, 0 ),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "gps",
                     "base_link")


    br.sendTransform( (0, 0, 0, 0 ),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "zed_center",
                     "zed_left_camera")

    br.sendTransform((0, 0, 0, 0 ),
                     (1, 0, 0, 0),
                     msg.header.stamp,
                     "imu",
                     "base_link")



if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/zed/odom', Odometry, static_tf)
    rospy.spin()