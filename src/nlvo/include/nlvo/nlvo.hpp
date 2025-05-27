#pragma once

#include "common/settings.hpp"

struct VelDisk{
    float cx; // center of the disk in velocity space
    float cy;
    float radius; // r_total / th
};

class NLVO
{
    public:
    // Constructor
    NLVO();

    // Global variables
    const float control_limit_x = 2.5f; // |u_x| <= control_limit_x
    const float control_limit_y = 2.5f; // |u_y| <= control_limit_y
    const float dt = 0.1f; // time step
    const float max_time = 10.0f; // maximum time horizon
    const float max_acceleration = 2.0f; // maximum acceleration
    const float max_speed = 5.0f; // maximum speed

    // Create a set of control limits (bang-bang corners of control set)
    std::vector<std::pair<double, double>> control_set = {
        {control_limit_x, control_limit_y},
        {-control_limit_x, control_limit_y},
        {control_limit_x, -control_limit_y},
        {-control_limit_x, -control_limit_y},
    };

    // Function to select the best velocity
    twist_msg selectBestVelocity(const pose_msg &ego_pose, const twist_msg &ego_vel, const std::vector<Obstacle> &obstacles, const point_msg &goal_point, float r_total);

    // Function to generate candidate velocities
    std::vector<twist_msg> generateACV(const twist_msg& ego_vel);

    // Function to generate candidate velocities (improved version)
    std::vector<twist_msg> generateCandidateVelocities(const twist_msg& ego_vel);

    // Function to calculate cost for a candidate velocity
    float calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const std::vector<Obstacle>& obstacles, const twist_msg& candidate_velocity, const point_msg& goal_point, float r_total, float time_horizon);

    // Function to compute the minimum time horizon
    float computeMinimumTimeHorizon(const pose_msg &ego_pose, const twist_msg &ego_vel, const Obstacle& obstacle, float r_total, std::vector<std::pair<double, double>> control_set);

    // Function to generate NLVO disks
    std::vector<VelDisk> generateNLVODisks(const pose_msg& ego_pose, const Obstacle& obstacle, int trajectory_index, float r_total, float time_horizon);

    // Function to find the next goal point of the obstacle on his trajectory
    int findTrajectoryIndex(const Obstacle& obstacle);

    private:

    // Function to check if a candidate velocity is in the truncated NLVO
    bool isVelocityInNLVO(const twist_msg& candidate_vel, const std::vector<VelDisk>& disks);

    // Function to normalize angle
    float normalizeAngle(float angle);

    // Function to calculate distance between two points
    float distance (const point_msg& s1, const point_msg& s2);

};