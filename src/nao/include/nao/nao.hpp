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

    // Function that return the best velocity command given the acceleration
    twist_msg selectBestVelocity(const accel_msg& ego_accel);

    private:
    // Function that create accelerations candidates
    std::vector<accel_msg> generateCandidateAccel(const accel_msg& ego_accel);

    // Function that generate acceleration disks for single obstacle
    std::vector<AccDisk> generateNAODisk(const pose_msg obs_pose, const twist_msg ego_vel, float time_horizon);

    // Function that check if a candidate acceleration is in the truncated NAO
    bool isVelocityInNao(const accel_msg& candidate_accel, const std::vector<AccDisk>& disks);
};