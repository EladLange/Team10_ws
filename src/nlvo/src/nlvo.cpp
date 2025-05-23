#include <iostream>
#include <vector>
#include <cmath>
#include "nlvo/nlvo.hpp"


NLVO::NLVO()
{
    // Empty constructor
}

twist_msg NLVO::selectBestVelocity(const pose_msg &ego_pose, const twist_msg &ego_vel, const std::vector<pose_msg> &obstacles_poses, const std::vector<twist_msg> &obstacle_vels, const point_msg &goal_point, float r_total)
{
    // Create a set of control limits (bang-bang corners of control set)
    std::vector<std::pair<double, double>> control_set = {
        {control_limit_x, control_limit_y},
        {-control_limit_x, control_limit_y},
        {control_limit_x, -control_limit_y},
        {-control_limit_x, -control_limit_y},
    };

    // Find the minimum time horizon
    float min_time_horizon = max_time;

    for (size_t i = 0; i < obstacles_poses.size(); i++)
    {
        float time_horizon = computeMinimumTimeHorizon(ego_pose, ego_vel, obstacles_poses[i], obstacle_vels[i], r_total, control_set);
        std::cout<<"time horizon for obstacle "<<i<<": "<<time_horizon<<std::endl;
        min_time_horizon = std::min(min_time_horizon, time_horizon);
    }

    min_time_horizon += 2.0f;

    // Generate candidate velocities
    std::vector<twist_msg> candidate_velocities = generateCandidateVelocities(ego_vel);
    twist_msg best_velocity = ego_vel; // Default to current velocity

    // Check if candidate velocities are in the truncated NLVO
    std::vector<twist_msg> safe_vels;
    for (const auto &candidate_vel : candidate_velocities)
    {
        bool is_safe = true;
        for (size_t i = 0; i < obstacles_poses.size(); i++)
        {
            if (isVelocityInTruncatedNLVO(candidate_vel, ego_pose, ego_vel, obstacles_poses[i], obstacle_vels[i], r_total, min_time_horizon))
            {
                is_safe = false;
                break;
            }
        }
        if (is_safe)
        {
            safe_vels.push_back(candidate_vel);
        }
    }

    // If no safe velocities found, generate emergency velocities
    if (safe_vels.empty())
    {
        std::cout << "NO SAFE VELOCITIES FOUND" << std::endl;
        best_velocity.linear.x = 0.0f;
        best_velocity.linear.y = 0.0f;
        return best_velocity;
    }

    twist_msg current_ego_vel = ego_vel;

    float best_cost = std::numeric_limits<float>::max();
            
    for (const auto &candidate_vel : safe_vels)
    {
        float cost = calculateCandidateCost(ego_pose, ego_vel, obstacles_poses, obstacle_vels, candidate_vel, goal_point, r_total, min_time_horizon);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_velocity = candidate_vel;
        }
    }

    std::cout << "Selected best velocity: (" << best_velocity.linear.x << ", " << best_velocity.linear.y << ")" << std::endl;
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<std::endl;
    return best_velocity; // Return the best velocity found among the candidates
}

float NLVO::computeMinimumTimeHorizon(const pose_msg &ego_pose, const twist_msg &ego_vel, const pose_msg &obstacle_pose, const twist_msg &obstacle_vel, float r_total, std::vector<std::pair<double, double>> control_set)
{
    float min_collision_time = max_time;
    float r_total_squared = r_total * r_total;
    float first_collision_time_for_control;

    // For every control in the set
    for (const auto &control : control_set)
    {
        first_collision_time_for_control = max_time;
        
        // Calculate the initial relative velocity
        twist_msg relative_velocity;
        relative_velocity.linear.x = ego_vel.linear.x - obstacle_vel.linear.x;
        relative_velocity.linear.y = ego_vel.linear.y - obstacle_vel.linear.y;

        // Calculate the initial relative position
        pose_msg relative_pose;
        relative_pose.position.x = ego_pose.position.x - obstacle_pose.position.x;
        relative_pose.position.y = ego_pose.position.y - obstacle_pose.position.y;

        // Check for collision at different time steps
        for (float t = dt; t < max_time; t += dt)
        {
            // Relative position at time t
            pose_msg future_relative_pose;
            future_relative_pose.position.x = relative_pose.position.x + relative_velocity.linear.x * t + 0.5 * control.first * t * t;
            future_relative_pose.position.y = relative_pose.position.y + relative_velocity.linear.y * t + 0.5 * control.second * t * t;

            double future_relative_pose_length_squared = pow(future_relative_pose.position.x, 2) + pow(future_relative_pose.position.y, 2);

            if (future_relative_pose_length_squared <= r_total_squared) // Collision detected
            {
                first_collision_time_for_control = t;
                break; // Found the first collision time for this control
            }
        }

        // Update the minimum collision time
        min_collision_time = std::min(min_collision_time, first_collision_time_for_control);
    }
    return min_collision_time;
}

std::vector<twist_msg> NLVO::generateACV(const twist_msg &ego_vel)
{
    std::vector<twist_msg> candidate_velocities;

    float delta_x = control_limit_x / 2;
    float delta_y = control_limit_y / 2;

    // Generate candidate velocities based on the ego velocity
    for (float ux = -control_limit_x; ux <= control_limit_x; ux += delta_x)
    {
        for (float uy = -control_limit_y; uy <= control_limit_y; uy += delta_y)
        {
            twist_msg candidate_velocity;
            candidate_velocity.linear.x = ego_vel.linear.x + ux * dt;
            candidate_velocity.linear.y = ego_vel.linear.y + uy * dt;
            candidate_velocities.push_back(candidate_velocity);
        }
    }

    return candidate_velocities;
}

bool NLVO::isVelocityInTruncatedNLVO(const twist_msg &candidate_vel, const pose_msg &ego_pose, const twist_msg &ego_vel, const pose_msg &obstacle_pose, const twist_msg &obstacle_vel, float r_total, float time_horizon)
{
    float r_total_squared = r_total * r_total;

    std::cout << "Checking velocity (" << candidate_vel.linear.x << "," << candidate_vel.linear.y
              << ") with time_horizon=" << time_horizon << std::endl;


    // First, check if we're already too close to the obstacle
    float current_dist_squared = pow(ego_pose.position.x - obstacle_pose.position.x, 2) + pow(ego_pose.position.y - obstacle_pose.position.y, 2);
    if (current_dist_squared <= r_total_squared)
    {
        return true;
    }

    // Check future positions
    for (float t = dt; t < time_horizon; t += dt)
    {
        // Calculate obstacle position at time t
        point_msg obstacle_future_pos;
        obstacle_future_pos.x = obstacle_pose.position.x + obstacle_vel.linear.x * t;
        obstacle_future_pos.y = obstacle_pose.position.y + obstacle_vel.linear.y * t;

        // Calculate ego position at time t with candidate velocity
        pose_msg ego_future_pose;
        ego_future_pose.position.x = ego_pose.position.x + candidate_vel.linear.x * t;
        ego_future_pose.position.y = ego_pose.position.y + candidate_vel.linear.y * t;

        // Calculate relative position
        float rel_x = ego_future_pose.position.x - obstacle_future_pos.x;
        float rel_y = ego_future_pose.position.y - obstacle_future_pos.y;

        // Check if distance is less than r_total
        float dist_squared = rel_x * rel_x + rel_y * rel_y;

        if (dist_squared <= r_total_squared) // Collision detected
        {
            std::cout << "Collision detected at t=" << t
            << " with candidate vel=(" << candidate_vel.linear.x << "," << candidate_vel.linear.y
            << "), dist=" << sqrt(dist_squared) << " vs r_total=" << r_total
            << ", ego future=(" << ego_future_pose.position.x << "," << ego_future_pose.position.y
            << "), obstacle future=(" << obstacle_future_pos.x << "," << obstacle_future_pos.y << ")" << std::endl;
            std::cout << "  COLLISION DETECTED at t=" << t << std::endl;
            return true; // Candidate velocity is in the truncated NLVO
        }
    }
     std::cout << "  NO COLLISION DETECTED within time horizon" << std::endl;
    return false; // Candidate velocity is not in the truncated NLVO
}

std::vector<twist_msg> NLVO::generateCandidateVelocities(const twist_msg& ego_vel)
{

    std::vector<twist_msg> candidate_velocities;
    twist_msg delta_v;
    twist_msg candidate_velocity;

    // Number of angles & accelerations to generate
    // Number of candidate velocities = num_of_angles * num_of_accelerations
    int num_of_angles = 50;
    int num_of_accelerations = 7;
    float angle_step = 2 * M_PI / num_of_angles;
    float acceleration_step = max_acceleration / num_of_accelerations;

    float acceleration, angle;
    for (acceleration = 0.0; acceleration <= max_acceleration; acceleration = acceleration + acceleration_step)
    {
       float delta_v_magnitude = acceleration * dt;
       for (angle = 0; angle < 2 * M_PI; angle = angle + angle_step)
       {
           delta_v.linear.x = delta_v_magnitude * std::cos(angle);
           delta_v.linear.y = delta_v_magnitude * std::sin(angle);

           candidate_velocity.linear.x = ego_vel.linear.x + delta_v.linear.x;
           candidate_velocity.linear.y = ego_vel.linear.y + delta_v.linear.y;

           candidate_velocities.push_back(candidate_velocity);
       }
    }

    return candidate_velocities;
}

float NLVO::calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const std::vector<pose_msg>obstacle_poses,const std::vector<twist_msg>obstacle_vels, const twist_msg& candidate_velocity, const point_msg& goal_point, float r_total, float time_horizon)
{

    float cost = 0.0f;
    // cost function constant
    float obstacle_avoidance_weight = 100.0f;
    float goal_seeling_weight = 150.0f;
    float smoothness_weight = 50.0f;
    float time_step = 1.0f;
    
    // Obstacle avoidance 
    pose_msg ego_future_position;
    ego_future_position.position.x = ego_pose.position.x + candidate_velocity.linear.x * time_step;
    ego_future_position.position.y = ego_pose.position.y + candidate_velocity.linear.y * time_step;
    ego_future_position.position.z = ego_pose.position.z;
    pose_msg obstacle_future_pose; 
    
    float min_distance = std::numeric_limits<float>::max();
    for (size_t i = 0; i < obstacle_poses.size(); i++)
    {
        obstacle_future_pose.position.x = obstacle_poses[i].position.x + obstacle_vels[i].linear.x * time_step;
        obstacle_future_pose.position.y = obstacle_poses[i].position.y + obstacle_vels[i].linear.y * time_step;
        obstacle_future_pose.position.z = obstacle_poses[i].position.z;

        float dist = std::sqrt(std::pow(ego_future_position.position.x - obstacle_future_pose.position.x, 2) + std::pow(ego_future_position.position.y - obstacle_future_pose.position.y, 2));
        if (dist < min_distance)
        {
            min_distance = dist;  
        }  
    }
    //std::cout << "Minimum distance to obstacle: " << min_distance << std::endl;
    float obstacle_avoidance_cost = obstacle_avoidance_weight * (1.0f / min_distance);
    //std::cout << "Obstacle avoidance cost: " << obstacle_avoidance_cost << std::endl;
    
    // goal seeking cost
    float dx = goal_point.x - ego_future_position.position.x;
    float dy = goal_point.y - ego_future_position.position.y;

    float dist_to_goal = std::sqrt(std::pow(dx,2) + std::pow(dy, 2));
    float goal_seeking_cost = goal_seeling_weight * dist_to_goal; 
    //std::cout << "goal seeking cost: " << goal_seeking_cost << std::endl;

    // smoothness cost
    float dx_candidate_vel = candidate_velocity.linear.x - ego_velocity.linear.x;
    float dy_candidate_vel = candidate_velocity.linear.y - ego_velocity.linear.y;
    float dist_candidate_vel = std::sqrt(std::pow(dx_candidate_vel, 2) + std::pow(dy_candidate_vel, 2));
    float smoothness_cost = smoothness_weight * dist_candidate_vel;
    //std::cout << "smoothness cost: " << smoothness_cost << std::endl;
    
    // Combine costs
    cost = obstacle_avoidance_cost + goal_seeking_cost + smoothness_cost;

    return cost;

    std::cout << "Cost for velocity (" << candidate_velocity.linear.x << "," << candidate_velocity.linear.y << "): " << cost << std::endl;
    std::cout << "Obstacle avoidance cost: " << obstacle_avoidance_cost << std::endl;
    std::cout << "Goal seeking cost: " << goal_seeking_cost << std::endl;
    std::cout << "Smoothness cost: " << smoothness_cost << std::endl;

    return cost;
}

