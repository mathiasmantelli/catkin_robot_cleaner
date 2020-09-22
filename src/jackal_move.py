#!/usr/bin/env python

import rospy
import time
from std_srvs.srv import Empty
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
import math
import tf

obstacle_left = False
obstacle_right = False
obstacle_front = False

def reading_sensor(msg):
    #print "Sensor at 134 degree:"
    #print msg.ranges[134] #direita
    #print "Sensor at 360 degree:"
    #print msg.ranges[360] #centro
    #print "Sensor at 595 degree:"
    #print msg.ranges[595] #esquerda
    #print "--------------------"
    global obstacle_left
    global obstacle_right
    global obstacle_front
    if msg.ranges[134] < 0.5:
        print "Obstacle right"
        obstacle_right = True
    else:
        obstacle_right = False

    if msg.ranges[360] < 0.5:
        print "Obstacle front"
        obstacle_front = True
    else:
        obstacle_front = False

    if msg.ranges[595] < 0.5:
        print "Obstacle left"
        obstacle_left = True
    else:
        obstacle_left = False
    #br = tf.TransformBroadcaster()
    #LaserScan laser_tf



def callback(odom):
    #print "pose callback"
    #print ('x = {}'.format(odom.pose.pose.position.x)) #new in python 3
    #print ('y = %f' %odom.pose.pose.position.y) #used in python 2
    
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    orientation = math.degrees(euler[2])
    #print ('yaw = {}'.format(orientation)) #new in python 3

def move_robot():
       
    rate = rospy.Rate(10) # 10hz

    myTwist = Twist()
 
    while not rospy.is_shutdown():
        myTwist.linear.x = np.random.uniform(0.3, 0.5)
        myTwist.angular.z = np.random.uniform(-0.1, 0.1)
        if obstacle_right:
                myTwist.linear.x = 0.0
                myTwist.angular.z = np.random.uniform(0.3,0.8)
        if obstacle_left:
                myTwist.linear.x = 0.0
                myTwist.angular.z = np.random.uniform(-0.8,-0.3) 
        if obstacle_front:
                myTwist.linear.x = 0.0
                myTwist.angular.z = np.random.uniform(0.0,1.0)
        mvrb_publisher.publish(myTwist)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('jackal_move_robot', anonymous=True)

        mvrb_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, callback)
	rospy.Subscriber('/front/scan', LaserScan, reading_sensor) 

        #print 'move: '
        move_robot()
        time.sleep(2)
        #print 'start reset: '
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print 'end reset: '
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
