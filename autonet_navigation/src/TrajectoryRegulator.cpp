//
// Created by vasily on 18.07.2020.
//

#include "TrajectoryRegulator.h"
#include <cmath>
#include <algorithm>
using namespace std;

Limit::Limit(double max_x_, double min_x_){
    max_x = max_x_;
    min_x = min_x_;
}
void Limit::clip(double &x){
    x = min(max(x, min_x), max_x);
}

//double Limit::clip(double x){
//    return min(max(x, min_x), max_x);
//}


VelLimits TrajectoryRegulator::getLimits(geometry_msgs::Twist vel_now, double dt) {
    double max_a_vel = vel_now.angular.z + max_a_acl_*dt;
    double min_a_vel = vel_now.angular.z - max_a_acl_*dt;
    max_a_vel = min(max_a_vel, max_a_vel_);
    min_a_vel = max(min_a_vel, -max_a_vel_);

    double max_l_vel = vel_now.linear.z + max_l_acl_*dt;
    double min_l_vel = vel_now.linear.z - max_l_acl_*dt;
    max_l_vel = min(max_l_vel, max_l_vel_);
    min_l_vel = max(min_l_vel, min_l_vel_);

    return {Limit(max_l_vel, min_l_vel), Limit(max_a_vel, min_a_vel)};
}
geometry_msgs::Twist TrajectoryRegulator::getVel(geometry_msgs::Twist vel_now, geometry_msgs::Pose target_pose, double target_l_vel_){
    double dtime = 1;
    VelLimits limits = getLimits(vel_now, dtime);
    double l_vel = target_l_vel_;
    limits.l.clip(l_vel);
    double a_vel = default_a_vel_;
    limits.a.clip(a_vel);

}
