#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <common/timestamp.h>
// #include "image_source.h"

ActionModel::ActionModel(void)
{
    k1 = 1.15;
    k2 = 0.25;
    // N_dist = 1000;
    // sd1 = 0;
    // sd2 = 0;
    // sd3 = 0;
    // e1 = 0;
    // e2 = 0;
    // e3 = 0;
    // del_s = 0;
}

bool ActionModel::updateAction(const pose_xyt_t& odometry)
{

    if (!started_){
        last_pose_ = odometry;
        started_ = true;
        return false;
    }

    ///// TODO: Threshold need to be tuned


    // get initial and final poses
    float x2 = odometry.x;
    float y2 = odometry.y;
    float th2 = odometry.theta;

    float x1 = last_pose_.x;
    float y1 = last_pose_.y;
    float th1 = last_pose_.theta;
    alpha = atan2(y2 - y1, x2 - x1) - th1;
    del_theta = th2 -th1;
    del_s = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

    if (del_s < 0.01 && (fabs(del_theta) < 0.001)){
      return false;
    }
    // creating distribution
    sd1 = k1 * fabs(alpha);
    sd2 = k2 * fabs(del_s);
    sd3 = k1 * fabs(del_theta - alpha);
    last_pose_ = odometry;
    // std::cout<<"odometry x: "<<odometry.x<<" y: "<<odometry.y<<std::endl;

    // std::random_device rd;
    // std::mt19937 gen(rd());
    return true;
}

particle_t ActionModel::applyAction(const particle_t& sample)

{
    ////////////// TODO: Implement your code for sampling new poses from the d  istribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    float x1 = sample.pose.x;
    float y1 = sample.pose.y;
    float th1 = sample.pose.theta;
    particle_t new_sample;


    // {
    // get random number with normal distribution using gen as random source
    std::random_device rd;
    std::mt19937 gen(rd());
    // std::normal_distribution<float> d(mean,stddev);
    std::normal_distribution<float> d1(0,sd1);
    std::normal_distribution<float> d2(0,sd2);
    std::normal_distribution<float> d3(0,sd3);

    // sampled_val = d(gen);
    e1 = d1(gen);
    e2 = d2(gen);
    e3 = d3(gen);
    //std::cout<<sd1<<" "<<sd2<<" "<<sd3<<" "<<e1<<" "<<e2<<" "<<e3<<std::endl;

    new_sample.parent_pose = sample.pose;
    new_sample.pose.x = x1 + (del_s + e2) * cos(th1 + alpha + e1);
    new_sample.pose.y = y1 + (del_s + e2) * sin(th1 + alpha + e1);
    new_sample.pose.theta = wrap_to_pi(th1 + (del_theta + e1 + e3));
    //new_sample.pose.theta = wrap_to_pi(th1 + (del_theta + e1 + e3));
    // new_sample.pose.utime = sample.pose.utime;
    // new_sample.weight = 0.0;
    // std::cout<<"action del s: "<<del_s<<" del theta: "<<del_theta<<std::endl;
    // std::cout<<"new sample x: "<<new_sample.pose.x<<" y: "<<new_sample.pose.y<<std::endl;
    // std::cout<<"sample x: "<<sample.pose.x<<" y: "<<sample.pose.y<<std::endl;

    // }
    new_sample.pose.utime = last_pose_.utime;
    return new_sample;
}
