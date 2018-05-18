#!/usr/bin/env python  
import roslib
import rospy
import utm
import numpy as np
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix
import math

pub = rospy.Publisher('/gps/pose', Odometry, queue_size=1)
init_pose = [6.05213442e+05, 4.26608607e+06, 1.52632201e+09]

def publish_pose(msg):

    gps_odom = Odometry()   
    gps_odom.header.stamp = msg.header.stamp
    gps_odom.header.frame_id = "map"
    utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
    # odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0.93863332 + math.pi/4)
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0.93863332 + math.pi/4)

    gps_odom.pose.pose = Pose(Point(utm_coord[0] - init_pose[0], utm_coord[1] - init_pose[1], 0.), Quaternion(*odom_quat))
    rospy.loginfo(gps_odom)
    pub.publish(gps_odom)


if __name__ == '__main__':
    rospy.init_node('gps_odom')
    rospy.Subscriber('/gps0/fix', NavSatFix, publish_pose)
    rospy.spin()