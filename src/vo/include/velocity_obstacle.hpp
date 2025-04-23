#pragma once

#include "car.hpp"
#include <vector>

using pose = geometry_msgs::msg::Pose;
using velocity = geometry_msgs::msg::Twist;

constexpr float max_acceleration = 1.5f;
constexpr float time_step = 0.1f;

float r_r = 1.0;
float r_o = 0.5;
float r_total = r_r + r_o;
float time_horizon = 5.0f;

class VelocityObstacle
{
    private:
    // calculate the angle between two points    
    float distance(const pose& s1, const pose& s2);

    // calculation theta: half angle of the cone 
    float getTheta(float d);

    // calculating angle between 2 points and x axis
    float getAngle(const pose& s1, const pose& s2);

    // calculating beta: angle of v_relative
    float getBeta(const velocity& v_relative);

    // calculating v_relative
    velocity getVrelative(const velocity& v_ego, const velocity& v_obstacle);

    //normalize angle difference
    float normalizeAngle(float angle);

    //Builds the 9 possible velocities the ego car could apply
    std::vector<velocity> generateCandidateVelocities(const velocity& ego_vel);

    // Compute the VO-based cost for a velocity sample
    float VelocityObstacle::calculateCollisionCost(const pose& ego_pose, const velocity& ego_velocity, const std::vector<pose>obstacle_poses,const std::vector<velocity>obstacle_vels, const velocity& candidate_velocity, const pose& goal_point);
    
    // Select best velocity sample by evaluating all samples over all obstacles
    velocity selectBestVelocity(const pose& ego_pose, const velocity& ego_vel, const std::vector<pose>& obstacles, const std::vector<velocity>& obstacle_vels, const std::vector<pose>& raceline);

    // find the next point ont the trajectory
    pose findNextGoalPoint(const std::vector<pose>& raceline, const pose& ego_pose);

    public:
    // Constructor
    VelocityObstacle();

    //implementation of the velocity obstacle
    bool checkCollision(const pose& ego_pose, const pose& obstacle_pose, const velocity& v_ego, const velocity& v_obstacle);

    //Function to calculate reachable velocities
    std::vector<velocity> reachableVelocities(const pose& ego_pose, const pose& obstacle_pose, const velocity& v_ego, const velocity& v_obstacle, float r_total);
};