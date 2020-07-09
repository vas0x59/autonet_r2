#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
// #include <ge

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
// ros::Subs


void control_motors()
{
    if (cmd_vel_status == true)
    {
        float m1_target_v, m2_target_v;
        m1_target_v = target_x_v + target_a_z_v * robot_W * 2;
        m2_target_v = target_x_v - target_a_z_v * robot_W * 2;
        motor1.publish(m1_target_v)
            motor2.publish(m2_target_v)
    }
}

int main()
{
    ros::init(ardc, argv, "autonet_odom");
    ros::NodeHandle nh;

    return 0;
}
