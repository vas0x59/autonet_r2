/*
      ___           ___           ___           ___           ___           ___           ___
     /\  \         /\__\         /\  \         /\  \         /\__\         /\  \         /\  \
    /::\  \       /:/  /         \:\  \       /::\  \       /::|  |       /::\  \        \:\  \
   /:/\:\  \     /:/  /           \:\  \     /:/\:\  \     /:|:|  |      /:/\:\  \        \:\  \
  /::\~\:\  \   /:/  /  ___       /::\  \   /:/  \:\  \   /:/|:|  |__   /::\~\:\  \       /::\  \
 /:/\:\ \:\__\ /:/__/  /\__\     /:/\:\__\ /:/__/ \:\__\ /:/ |:| /\__\ /:/\:\ \:\__\     /:/\:\__\
 \/__\:\/:/  / \:\  \ /:/  /    /:/  \/__/ \:\  \ /:/  / \/__|:|/:/  / \:\~\:\ \/__/    /:/  \/__/
      \::/  /   \:\  /:/  /    /:/  /       \:\  /:/  /      |:/:/  /   \:\ \:\__\     /:/  /
      /:/  /     \:\/:/  /     \/__/         \:\/:/  /       |::/  /     \:\ \/__/     \/__/
     /:/  /       \::/  /                     \::/  /        /:/  /       \:\__\              2020
     \/__/         \/__/                       \/__/         \/__/         \/__/  by Vasily Yuryev

 Robot odometry publisher

 */


#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "odom_model.h"
#include "autonet_odom/SetOdom.h"
//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/json_parser.hpp>
//#include <boost/foreach.hpp>

using namespace std;

double robot_W = 0.32;
double update_rate = 30;
string frame_name = "odom";
string base_frame_name = "base_link";
//string json_config_path = "../config.json";
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
void m1tv_clb(std_msgs::Float32 msg) {
    cm1 = msg.data;
}

void m2tv_clb(std_msgs::Float32 msg) {
    cm2 = msg.data;
}

void cmd_vel_clb(geometry_msgs::Twist msg) {
    target_x_v = msg.linear.x;
    target_a_z_v = msg.angular.z;
}

// void set_odom(data)

void control_motors() {
    if (cmd_vel_status) {
        float m1_target_v = 0, m2_target_v = 0;
        m1_target_v = target_x_v + target_a_z_v * robot_W * 2;
        m2_target_v = target_x_v - target_a_z_v * robot_W * 2;
        std_msgs::Float32 m_msg;
        m_msg.data = m1_target_v;
        motor1.publish(m_msg);
        m_msg.data = m2_target_v;
        motor2.publish(m_msg);
    }
}

OdometryModel odom_calc;
ros::Time last_time;


void calc_odometry() {
    if (!first_time) {
        static tf2_ros::TransformBroadcaster br;

        ros::Time current_time = ros::Time::now();
        OdometryOut o_out = odom_calc.calc((current_time - last_time).toSec(), cm1 - prev_m1_m, cm2 - prev_m2_m);
        prev_m1_m = cm1;
        prev_m2_m = cm2;

        tf2::Quaternion q;
        q.setRPY(0, 0, o_out.o);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = frame_name;
        odom.child_frame_id = base_frame_name;
        odom.pose.pose.position.x = o_out.x;
        odom.pose.pose.position.y = o_out.y;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.angular.z = o_out.vo;
        odom.twist.twist.linear.x = o_out.vx;
        odom.twist.twist.linear.y = o_out.vy;


        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = frame_name;
        transformStamped.child_frame_id = base_frame_name;
        transformStamped.transform.translation.x = o_out.x;
        transformStamped.transform.translation.y = o_out.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        odom_pub.publish(odom);
        br.sendTransform(transformStamped);
        last_time = current_time;
    } else {
        first_time = false;
    }
}

autonet_odom::SetOdomResponse set_odom(autonet_odom::SetOdomRequest data) {
    odom_calc.set(data.x, data.y, data.yaw);
    return autonet_odom::SetOdomResponse();
}

void update() {
    control_motors();
    calc_odometry();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "autonet_odom");
    ros::NodeHandle nh;

    nh.param("/autonet_odom/robot_W", robot_W, 0.32);
    nh.param("/autonet_odom/odom_frame_name", frame_name, string("odom"));
    nh.param("/autonet_odom/base_frame_name", base_frame_name, string("base_link"));
    nh.param("/autonet_odom/update_rate", update_rate, 25.0);
    ROS_INFO("autonet_odom");
    ROS_INFO_STREAM_NAMED("Odom parameters",
                           "robot_W: " << robot_W << " update_rate: " << update_rate << " odom_frame_name: "
                                       << frame_name << " base_frame_name: " << base_frame_name);
    last_time = ros::Time::now();
    odom_calc = OdometryModel(robot_W);
    motor1 = nh.advertise<std_msgs::Float32>("/motor1", 2);
    motor2 = nh.advertise<std_msgs::Float32>("/motor2", 2);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 2, cmd_vel_clb);
    ros::Subscriber enc1_sub = nh.subscribe("/encoder1", 2, m1tv_clb);
    ros::Subscriber enc2_sub = nh.subscribe("/encoder2", 2, m2tv_clb);

    ros::Rate r(update_rate); // 10 hz
    while (ros::ok()) {
        update();
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
