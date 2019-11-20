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
    k1 = 1;
    k2 = 1;
    // N_dist = 1000;
    sd1 = 0;
    sd2 = 0;
    sd3 = 0;
    del_s = 0;
}

bool ActionModel::updateAction(const pose_xyt_t& odometry)
{

    if (!started_)
    {
        last_pose_ = odometry;
        started_ = true;
        return false;
    }

    // get initial and final poses
    float x2 = odometry.x;
    float y2 = odometry.y;
    float th2 = odometry.theta;

    float x1 = last_pose_.x;
    float y1 = last_pose_.y;
    float th1 = last_pose_.theta;
    float alpha = atan2(y2 - y1, x2 - x1) - th1;

    // std::random_device rd;
    // std::mt19937 gen(rd());


    ///// TODO: Threshold need to be tuned

    if (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) > 0.1 || abs(th1 - th2)>0.1 )
    {

        del_s = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        // creating distribution
        sd1 = k1 * abs(alpha);
        sd2 = k2 * abs(del_s);
        sd3 = k1 * abs(th2 - th1 - alpha);
        last_pose_ = odometry;

        // for (int i_sample = 0; i_sample < 1000; ++i_sample){
        // for (int i_dist = 0; i_dist < N_dist; ++i_dist)
        // }
        return true;
    }
    else
    {
        return false;
    }
}

particle_t ActionModel::applyAction(const particle_t& sample)

{
    ////////////// TODO: Implement your code for sampling new poses from the d  istribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    float x2 = sample.pose.x;
    float y2 = sample.pose.y;
    float th2 = sample.pose.theta;

    float x1 = sample.parent_pose.x;
    float y1 = sample.parent_pose.y;
    float th1 = sample.parent_pose.theta;
    float alpha = atan2(y2 - y1, x2 - x1) - th1;
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
    float e1 = d1(gen);
    float e2 = d2(gen);
    float e3 = d3(gen);


    new_sample.parent_pose = sample.pose;
    new_sample.pose.x = x1 + (del_s + e2) * cos(th1 + alpha + e1);
    new_sample.pose.y = y1 + (del_s + e2) * cos(th1 + alpha + e1);
    new_sample.pose.theta = th1 + ((th2 - th1) + e1 + e3);
    new_sample.pose.utime = utime_now();
    new_sample.weight = 0;

    // }

    return new_sample;
}
