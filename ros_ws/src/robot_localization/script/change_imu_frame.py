#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Imu

import math

pub = rospy.Publisher('/imu/data', Imu, queue_size=1)


def change_frame(msg):

    msg_temp = msg
    msg
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('change_imu_frame')
    rospy.Subscriber('/imu/imu', Imu, change_frame)
    rospy.spin()