#pragma once

#include "common/settings.hpp"
#include "common/car.hpp"


struct AccDisk{
    float cx; // center of the disk in velocity space
    float cy;
    float radius; // r_total / th
};

class NAO
{
    public:
    NAO();

    const float max_acceleration = 5.0f;
    // For now - const time horizon
    const float time_horizon = 1.0f; // seconds
    const float dt = 1.0f; // seconds

    // Function that return the best velocity command given the acceleration
    twist_msg selectBestVelocity(const accel_msg& ego_accel, const twist_msg& ego_vel,const pose_msg& ego_pose, const std::vector<Obstacle>& obstacles_, const point_msg &goal_point, float r_total);

    // Function that find the trajectory index of the obstacle
    int findTrajectoryIndex(const Obstacle& obstacle);


    // Function that generate acceleration disks for single obstacle
    std::vector<AccDisk> generateNAODisks(const Obstacle& obstacle, const twist_msg& ego_vel, const pose_msg& ego_pose, int trajectory_index, float total_r);

    // Function that create accelerations candidates
    std::vector<accel_msg> generateCandidateAccel(const accel_msg& ego_accel);

    //Get the best acceleration
    accel_msg getBestAccel();

    accel_msg global_best_accel;
    private:

    

    // Function that check if a candidate acceleration is in the truncated NAO
    bool isAccelInNao(const accel_msg& candidate_accel, const std::vector<AccDisk>& disks);

    // Function that transform the acceleration to velocity
    twist_msg accelerationToVelocity(const accel_msg& safe_accel, float dt);

    // Function that calculate the cost of a candidate acceleration
    float calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const accel_msg& ego_accel, const twist_msg& candidate_velocity, const point_msg& goal_point, float r_total, float time_horizon);
};