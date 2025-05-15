#pragma once

#include "settings.hpp"
#include "global_variables.hpp"
#include "velocity_obstacle.hpp"

// Struct to hold the center and radius of the disk
struct Disk
{
    float center_x;
    float center_y;
    float radius;
};


class NLVO
{
    public:
    // Constructor
    NLVO();

    // Function to create NLVO disk for every obstacle
    std::vector<Disk> createNLVODisk(const pose_msg& ego_pose, const twist_msg& ego_velocity, const pose_msg& obstacle_pose, const twist_msg& obstacle_velocity);

    // Function that check if the candidate velocity is inside the disk
    bool isVelocityInsideDisk(const twist_msg& candidate, const Disk& disk);

    // Function that check if the candidate velocity is inside the all disks
    bool isVelocityFeasible(const twist_msg& candidate, const std::vector<Disk>& nlvo_disks);

    twist_msg selectBestVelocity(const std::vector<twist_msg> &candidates, const std::vector<Disk> &nlvo_disks, const twist_msg &desired_velocity);

    private:
    // predict obstacle position in time t
    pose_msg predictObstaclePosition(const pose_msg& obstacle_pose, const twist_msg& obstacle_velocity, float t);

    // Compute safe time horizon
    float computeSafeTimeHorizon(const twist_msg &ego_velocity);

    // make sure the value is within the range [min, max]
    float clamp(float value, float min, float max);
};