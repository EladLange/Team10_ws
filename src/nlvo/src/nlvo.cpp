#include <algorithm>
#include "nlvo.hpp"
#include "velocity_obstacle.hpp"

VelocityObstacle vo;

// get V relative
twist_msg v_ego;
twist_msg v_obstacle;

twist_msg v_relative = vo.getVrelative(v_ego, v_obstacle);

std::vector<VelocityDisk> NLVO::createNLVODisk(const pose_msg &ego_pose, const twist_msg &ego_velocity, const pose_msg &obstacle_pose, const twist_msg &obstacle_velocity)
{
    float t_h = computeSafeTimeHorizon(ego_velocity);
    std::vector<VelocityDisk> nlvo_disks;
    
    for (float t = delta_t; t <= t_h; t += delta_t)
    {
        pose_msg predicted_obstacle_pose = predictObstaclePosition(obstacle_pose, obstacle_velocity, t);
        
        pose_msg relative_pose;
        relative_pose.position.x = predicted_obstacle_pose.position.x - ego_pose.position.x;
        relative_pose.position.y = predicted_obstacle_pose.position.y - ego_pose.position.y;
        relative_pose.position.z = predicted_obstacle_pose.position.z - ego_pose.position.z;

        float k = 1.0f / t;

        VelocityDisk disk;
        disk.center_x = k * relative_pose.position.x;
        disk.center_y = k * relative_pose.position.y;
        disk.radius = k * r_total;
        
        nlvo_disks.push_back(disk);
    }

    return nlvo_disks;
}

pose_msg NLVO::predictObstaclePosition(const pose_msg &obstacle_pose, const twist_msg &obstacle_velocity, float t)
{
    pose_msg predicted_pose;
    predicted_pose.position.x = obstacle_pose.position.x + obstacle_velocity.linear.x * t;
    predicted_pose.position.y = obstacle_pose.position.y + obstacle_velocity.linear.y * t;
    predicted_pose.position.z = obstacle_pose.position.z;
    
    return predicted_pose;
}

float NLVO::computeSafeTimeHorizon(const twist_msg &ego_velocity)
{
    float t_min_limit = 0.3f;
    float t_max_limit = 2.5f;
    
    float speed = std::sqrt(std::pow(ego_velocity.linear.x, 2) + std::pow(ego_velocity.linear.y, 2));
    
    if (speed < 1e-3) {
        return std::numeric_limits<float>::max(); // No collision if speed is very low
    }
    
    // find stopping time
    float t_s = speed / std::abs(min_acceleration); 

    // if t_s is smaller then t_min_limit, set t_s to t_min_limit
    // if t_s is greater then t_max_limit, set t_s to t_max_limit
    // if t_s is between t_min_limit and t_max_limit, set t_s to t_s
    //float safe_t = std::clamp(t_s, t_min_limit, t_max_limit);
}


