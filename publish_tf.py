#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import math

yaw = 0.93863332 + math.pi/4
q_init = tf.transformations.quaternion_from_euler(0, 0, yaw)

def cam_pose(msg):
    br = tf.TransformBroadcaster()
    q_now = tf.transformations.quaternion_multiply([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w], q_init)
   
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ),
                      (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                      msg.header.stamp,
                     "zed_left_camera",
                     "odom")

    br.sendTransform((0, 0, 0, 0 ),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "imu",
                     "zed_left_camera")


    br.sendTransform((0, 0, 0, 0 ),
                      (0, 0, 0, 1),
                      msg.header.stamp,
                     "zen_center",
                     "zed_left_camera")

    # br.sendTransform( (0 , 0,  0, 0),
    #                   (q_init[0], q_init[1], q_init[2], q_init[3]),
    #                   msg.header.stamp,
    #                  "map",
    #                  "earth")

    # br.sendTransform( (0 , 0,  0, 0),
    #                   (q_init[0], q_init[1], q_init[2], q_init[3]),
    #                   msg.header.stamp,
    #                  "map",
    #                  "earth")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/zed/odom', Odometry, cam_pose)
    rospy.spin()