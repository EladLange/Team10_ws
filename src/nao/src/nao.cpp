#include "nao/nao.hpp"

NAO::NAO()
{
    // Empty constructor
}

twist_msg NAO::selectBestVelocity(const accel_msg& ego_accel)
{
    // Step one - generate candidate accelerations:
    std::vector<accel_msg> candidate_velocities = generateCandidateAccel(ego_accel);

    // Step two - create NAO disks
}

std::vector<accel_msg> NAO::generateCandidateAccel(const accel_msg &ego_accel)
{
    int num_of_angles = 30;
    int num_of_accelerations = 3;
    float angle_step = 2 * M_PI / num_of_angles;
    float acceleration_step = max_acceleration / num_of_accelerations;

    float angle;
    accel_msg acceleration;
    std::vector<accel_msg> candidates_accel;

    for (int accel = acceleration_step; accel < max_acceleration; accel++)
    {
        for (angle = 0; angle < 2 * M_PI; angle = angle + angle_step)
        {
            acceleration.linear.x = accel * std::cos(angle);
            acceleration.linear.x = accel * std::sin(angle); 

            candidates_accel.push_back(acceleration);
        }
    }
    return candidates_accel;
}

std::vector<AccDisk> NAO::generateNAODisk(const pose_msg obs_pose, const twist_msg ego_vel, float time_horizon)
{
    float dt = 0.05f;
    float t = 0.0f;
    for (t = dt; t < time_horizon; t++)
    {

    }
}

bool NAO::isVelocityInNao(const accel_msg& candidate_accel, const std::vector<AccDisk>& disks)
{
    for (const auto& disk : disks)
    {
        float dx = candidate_accel.linear.x - disk.cx;
        float dy = candidate_accel.linear.y - disk.cy;

        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < disk.radius)
        {
            return true; // Candidate velocity is in the NAO
        }
    }
    return false;
}