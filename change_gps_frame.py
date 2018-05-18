#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix

import math

pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)


def change_frame(msg):
    msg.header.frame_id = 'earth'
    rospy.loginfo(msg)
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('change_gps_frame')
    rospy.Subscriber('/gps0/fix', NavSatFix, change_frame)
    rospy.spin()