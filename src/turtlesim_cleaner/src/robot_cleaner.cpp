#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <turtlesim/Pose.h>
#include <math.h>

using namespace std;

ros::Publisher velocity_publisher; 
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose; 

//method to move the robot straight
void move(double speed, double distance, bool isForward){
    geometry_msgs::Twist vel_msg; 
    //distance = speed * time

    (isForward)? vel_msg.linear.x = abs(speed):vel_msg.linear.x = -abs(speed);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    

    //t0: current time 
    //loop
        //publish the velocity
        //estimate the distance = speed * (t1 - t0)
        //current_distance_moved_by_robot <= distance
        //
    double t0 = ros::Time::now().toSec();
    double current_distance = 0; 
    ros::Rate loop_rate(100);
    do{
        velocity_publisher.publish(vel_msg); 
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0); 
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg); 
}

void rotate(double angular_speed, double relative_angle, bool isClockwise){
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    (isClockwise)?vel_msg.angular.z = -abs(angular_speed):vel_msg.angular.z = abs(angular_speed);

    double current_angle = 0;
    double t0 = ros::Time::now().toSec(); 
    double t1;
    ros::Rate loop_rate(100);
    do{
        velocity_publisher.publish(vel_msg);
        current_angle = angular_speed * (t1 - t0);
        t1 = ros::Time::now().toSec(); 
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle < relative_angle);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg); 
}

double degrees2radians(double angle_degree){
    return angle_degree * 3.1415/180.0;
}

double setDesideredOrientation(double desired_angle_radians){
    double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta; 
    bool clockwise = (relative_angle_radians<0)?true:false;

    //cout<<desired_angle_radians << "," << turtlesim_pose.theta << "," << relative_angle_radians << std::endl;
    rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0));
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
    //here I'll use a math theory that moves the robot from (x1, y1) to (x2, y2) in a 2D plane. It's called 'proportional controller'
    geometry_msgs::Twist vel_msg; 
    ros::Rate loop_rate(10);

    do{
        vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta); 

        velocity_publisher.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);
    cout << "end move goal" << std::endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void gridClean(){
    ros::Rate loop(0.5); 
    turtlesim::Pose pose; 
    pose.x = 1; 
    pose.y = 1; 
    pose.theta = 0; 

    moveGoal(pose, 0.01);
    loop.sleep();
    setDesideredOrientation(0);
    loop.sleep();

    move(2, 9, true); 
    loop.sleep();
    rotate(degrees2radians(10), degrees2radians(90), false);
    loop.sleep();
    move(2, 9, true); 
    
    rotate(degrees2radians(10), degrees2radians(90), false); 
    loop.sleep();
    move(2, 1, true);
    rotate(degrees2radians(10), degrees2radians(90), false); 
    loop.sleep();
    move(2, 9, true);

    rotate(degrees2radians(30), degrees2radians(90), true); 
    loop.sleep(); 
    move(2, 1, true);
    rotate(degrees2radians(30), degrees2radians(90), true); 
    loop.sleep(); 
    move(2, 9, true);

    //double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max)

}

void spiralClean(){
    geometry_msgs::Twist vel_msg; 
    double count = 0; 

    double constant_speed = 4; 
    double vk = 1; 
    double wk = 2; 
    double rk = 0.5; 
    ros::Rate loop (1);

    turtlesim::Pose pose; 
    pose.x = 7; 
    pose.y = 7; 
    pose.theta = 0; 

    moveGoal(pose, 0.01);

    do
    {
        rk += 0.5; 
        vel_msg.linear.x = rk; 
        vel_msg.linear.y = 0; 
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = constant_speed;

        //cout << "vel_msg.linear.x = " << vel_msg.linear.x << endl; 
        //cout << "vel_msg.angular.z = " << vel_msg.angular.z << endl; 
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();

        loop.sleep();
        cout << rk << ", " << vk << ", " << wk << endl; 
    } while ((turtlesim_pose.x < 10.5) && (turtlesim_pose.y < 10.5));
    vel_msg.linear.x = 0; 
    velocity_publisher.publish(vel_msg);    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n; 
    double speed, distance, angular_speed, relative_angle; 
    bool isForward, isClockwise; 

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback); 

    /*MOVE THE ROBOT
    cout << "Please type the inputs to move the robot: speed  distance isForward." << std::endl;
    cin >> speed >> distance >> isForward;
    move(speed, distance, isForward);*/

    /*ROTATE THE ROBOT
    cout << "Please type the inputs to turn the robot: angular speed  relative angle clockwise." << std::endl;
    cin >> angular_speed >> relative_angle >> isClockwise;
    rotate(degrees2radians(angular_speed), degrees2radians(relative_angle), isClockwise);*/

    /*SET THE ROBOT HEADING AT A SPECIFIC ORIENTATION
    setDesideredOrientation(degrees2radians(12));*/

    /*MOVE THE ROBOT TO A SPECIFIC POSE
    turtlesim::Pose my_goal_pose;
    my_goal_pose.x = 1; 
    my_goal_pose.y = 9;
    my_goal_pose.theta = 0;
    moveGoal(my_goal_pose, 0.01);
    ros::Rate loop_rate(0.5);
    loop_rate.sleep();*/

    //gridClean();
    spiralClean();


    ros::spin();
    return 0;
}