#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math
import tf


def poseCallback(odom):
    print "pose callback"
    print ('x = {}'.format(odom.pose.pose.position.x)) #new in python 3
    print ('y = %f' %odom.pose.pose.position.y) #used in python 2
    
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    orientation = math.degrees(euler[2])
    print ('yaw = {}'.format(orientation)) #new in python 3


if __name__ == '__main__':
    try:
        
        rospy.init_node('jackal_pose', anonymous=True)
        
        position_topic = "/odometry/filtered"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback) 
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
