//
// Created by vasily on 18.07.2020.
//

#ifndef SRC_TRAJECTORYREGULATOR_H
#define SRC_TRAJECTORYREGULATOR_H

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

struct Limit {
    double max_x = 0;
    double min_x = 0;
    Limit();
    Limit(double max_x_, double min_x_);
    void clip(double &x);
};

struct VelLimits {
    Limit l;
    Limit a;
};

class TrajectoryRegulator {
public:
    TrajectoryRegulator();
    void setParameters(double max_l_vel, double min_l_vel, double max_a_vel, double max_l_acl, double max_a_acl, double default_a_vel);
    geometry_msgs::Twist getVel(geometry_msgs::Twist vel_now, geometry_msgs::Pose target_pose, double target_l_vel);

private:
    double max_l_vel_, min_l_vel_,  max_a_vel_,  max_l_acl_,  max_a_acl_, default_a_vel_ = 0;
    VelLimits getLimits(geometry_msgs::Twist vel_now, double dt);
};


#endif //SRC_TRAJECTORYREGULATOR_H
