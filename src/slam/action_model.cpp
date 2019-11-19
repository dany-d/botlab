#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include "rand_util.h"
#include <random>
#include "image_source.h"

ActionModel::ActionModel(void)
{
    k1 = 1;
    k2 = 1;
    N_dist = 1000;

    e1[N_dist];
    e2[N_dist];
    e3[N_dist];

    //////////////// TODO: Handle any initialization for your ActionModel ////////////////////////
}

bool ActionModel::updateAction(const particle_t &sample)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    // get initial and final poses
    float x2 = sample.pose.x;
    float y2 = sample.pose.y;
    float th2 = sample.pose.theta;

    float x1 = sample.parent_pose.x;
    float y1 = sample.parent_pose.y;
    float th1 = sample.parent_pose.theta;
    float alpha = atan2(y2 - y1, x2 - x1) - th1;

    // std::random_device rd;
    // std::mt19937 gen(rd());

    if (sqrt((x1 - x2) * (x1 - x2) + (y1 + y2) * (y1 + y2) + (th1 + th2) * (th1 + th2)) > 0.1)
    {

        float del_s = sqrt((x1 - x2) * (x1 - x2) + (y1 + y2) * (y1 + y2));

        // creating distribution
        std::normal_distribution<float> e1(0, k1 * abs(alpha));
        std::normal_distribution<float> e2(0, k2 * abs(del_s));
        std::normal_distribution<float> e3(0, k1 * abs(th2 - th1 - alpha));
        // for (int i_sample = 0; i_sample < 1000; ++i_sample){
        // for (int i_dist = 0; i_dist < N_dist; ++i_dist)
        // }
        return 0;
    }
    else
    {
        return false;
    }
}

particle_t ActionModel::applyAction(const particle_t &sample)

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

    float sampled_val;
    float mean = 0.0;
    float stddev = 1.0;


    // {
    // get random number with normal distribution using gen as random source
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> d(mean,stddev);
    
    sampled_val = d(gen);
    e1[sampled_val];
    e2[sampled_val];
    e3[sampled_val];

    new_sample.parent_pose = sample.pose;
    new_sample.pose.x = x1 + (del_s + e2) * cos(th1 + alpha + e1);
    new_sample.pose.y = y1 + (del_s + e2) * cos(th1 + alpha + e1);
    new_sample.pose.theta = th1 + ((th1 - th2) + e1 + e3);
    new_sample.pose.utime = utime_now();
    new_sample.weight = 0;

    // }

    return new_sample;
}
