#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
// #include "odom_model.h"

using namespace std;

float robot_W = 0;
float wheel_d = 0;
float update_rate = 0;
string frame_name = "odom";

float prev_m1_m = 0;
float prev_m2_m = 0;

float target_x_v = 0;
float target_a_z_v = 0;

float cm1 = 0;
float cm2 = 0;

bool cmd_vel_status = true;
bool first_time = 0;

ros::Publisher motor1, motor2, odom_pub;


// Callbacks
void m1tv_clb(std_msgs::Float32 msg){
    cm1 = msg.data;
}

void m2tv_clb(std_msgs::Float32 msg){
    cm2 = msg.data;
}

void cmd_vel_clb(geometry_msgs::Twist msg){
    target_x_v = msg.linear.x;
    target_a_z_v = msg.angular.z;
}

// void set_odom(data)

void control_motors()
{
    if (cmd_vel_status == true)
    {
        float m1_target_v, m2_target_v;
        m1_target_v = target_x_v + target_a_z_v * robot_W * 2;
        m2_target_v = target_x_v - target_a_z_v * robot_W * 2;
        motor1.publish(m1_target_v);
        motor2.publish(m2_target_v);
    }
}

OdometryModel odom_calc();
ros::Time last_time;


void calc_odometry(){
    if (!first_time) {
        ros::Time current_time = ros::Time::now();
        OdometryOut o_out = odom_calc.calc((current_time - last_time).toSec(), cm1-prev_m1_m,  cm2-prev_m2_m);
        prev_m1_m = cm1;
        prev_m2_m = cm2;
    }
    else {
        first_time = false;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonet_odom");
    ros::NodeHandle nh;
    last_time = ros::Time::now();
    odom_calc = OdometryModel(robot_W);
    motor1 = nh.advertise<std_msgs::Float32>("/motor1", 5);
    motor2 = nh.advertise<std_msgs::Float32>("/motor2", 5);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 5, cmd_vel_clb);
    ros::Subscriber enc1_sub = nh.subscribe("/encoder1", 5, m1tv_clb);
    ros::Subscriber enc2_sub = nh.subscribe("/encoder2", 5, m2tv_clb);
    // ros::
    // ros::
    
    return 0;
}
