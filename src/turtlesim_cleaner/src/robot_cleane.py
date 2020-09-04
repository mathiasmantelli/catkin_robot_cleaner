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
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

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

def setDesideredOrientation(desired_angle):
    difference_angle = desired_angle - theta
    isClockwise_h = False
    if (difference_angle < 0):
        isClockwise_h = True
    rotate(abs(difference_angle), abs(difference_angle), isClockwise_h)

def moveGoal(goal_pose, distance_tolerance):
    global x, y, theta
    vel_msg = Twist()
    test = True
    while (test):
        K_linear = 0.5
        distance = abs(math.sqrt(((goal_pose.x-x)**2) + ((goal_pose.y - y)**2)))
        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(goal_pose.y-y, goal_pose.x-x)
        angular_speed = (desired_angle_goal-theta)*K_angular

        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed

        velocity_publisher.publish(vel_msg)

        if (distance < distance_tolerance):
            setDesideredOrientation(degrees2radians(goal_pose.theta))
            test = False

def gridClean():
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    moveGoal(desired_pose, 0.01)

    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 1.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 1.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 9.0, True)
    pass

def spiralClean():
    vel_msg = Twist()

    count = 0

    constant_speed = 4

    vk = 1
    wk = 2
    rk = 0.5
    loop_rate = rospy.Rate(1)

    my_pose = Pose()

    my_pose.x = 5.54
    my_pose.y = 5.54
    my_pose.theta = 0

    moveGoal(my_pose, 0.01)

    test = True
    while(test):
        rk += 0.5
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = constant_speed

        velocity_publisher.publish(vel_msg)

        loop_rate.sleep()

        if((x > 10) or (y > 10)):
            test = False
    
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('robot_cleaner', anonymous=True)
        velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10) 
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallBack) 
        time.sleep(2)
        #move(2, 2, 1)
        
        #rotate(degrees2radians(30), degrees2radians(90), 1)
        
        #setDesideredOrientation(degrees2radians(120))
        
        #goal_pose = Pose()
        #goal_pose.x = 2
        #goal_pose.y = 1
        #goal_pose.theta = degrees2radians(90)
        #moveGoal(goal_pose, 0.01);

        #gridClean()

        spiralClean()
        rospy.spin()

    except rospy.ROSInternalException:
        rospy.loginfo("Node terminated.")
