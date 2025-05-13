#pragma once

#include "settings.hpp"
#include "global_variables.hpp"

// Struct to hold the center and radius of the disk
struct VelocityDisk
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
    std::vector<VelocityDisk> createNLVODisk(const pose_msg& ego_pose, const twist_msg& ego_velocity, const pose_msg& obstacle_pose, const twist_msg& obstacle_velocity);

    private:
    
    // predict obstacle position in time t
    pose_msg predictObstaclePosition(const pose_msg& obstacle_pose, const twist_msg& obstacle_velocity, float t);

    // Compute safe time horizon
    float NLVO::computeSafeTimeHorizon(const twist_msg &ego_velocity);

    // make sure the value is within the range [min, max]
    template <typename T>
    const T& clamp(const T& value, const T& min, const T& max)
    {
        if (v < min) return min;
        if (max < v) return max;
        return v;
    }
};