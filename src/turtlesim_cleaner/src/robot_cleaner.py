#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

x = 0
y = 0
theta = 0

def poseCallBack(pose_message):
    global x, y, theta
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

def move(speed, distance, isForward):
    velocity_message = Twist()

    if (isForward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0

    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    time_t0 = rospy.Time.now().to_sec()
    current_distance = 0
    loop_rate = rospy.Rate(10)
    test = True
    while test:
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        time_t1 = rospy.Time.now().to_sec()
        current_distance = speed * (time_t1 - time_t0)
        

        if not (current_distance < distance):
            test = False
    
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def degrees2radians(angular_speed):
    return angular_speed * 3.1415/180.0

def rotate(angular_speed, relative_angle, isClockwise):
    velocity_message = Twist()

    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0

    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    if (isClockwise):
        velocity_message.angular.z = abs(angular_speed)
    else:
        velocity_message.angular.z = -abs(angular_speed)

    time_t0 = rospy.Time.now().to_sec()
    current_angle = 0
    loop_rate = rospy.Rate(10)    
    test = True
    while test:
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        time_t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (time_t1 - time_t0)
        if not (current_angle < relative_angle):
            test = False
    
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_cleaner', anonymous=True)
        velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10) 
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallBack) 
        time.sleep(2)
        #move(2, 2, 1)
        rotate(degrees2radians(30), degrees2radians(90), 1)
        rospy.spin()

    except rospy.ROSInternalException:
        rospy.loginfo("Node terminated.")
