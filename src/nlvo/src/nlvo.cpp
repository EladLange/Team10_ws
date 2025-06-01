#include <iostream>
#include <vector>
#include <cmath>
#include "nlvo/nlvo.hpp"


NLVO::NLVO()
{
    // Empty constructor
}

twist_msg NLVO::selectBestVelocity(const pose_msg &ego_pose, const twist_msg &ego_vel, const std::vector<Obstacle> &obstacles, const point_msg &goal_point, float r_total)
{
    
    // Find the minimum time horizon
    float min_time_horizon = max_time;
    std::cout<<"selectBestVelocity: " << std::endl;
    
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        float time_horizon = computeMinimumTimeHorizon(ego_pose, ego_vel, obstacles[i], r_total, control_set);
        min_time_horizon = std::min(min_time_horizon, time_horizon);
    }
    min_time_horizon += 2.0f;
    std::cout<<"    ego_vel: " << ego_vel.linear.x << ", " << ego_vel.linear.y << std::endl;
    std::cout<<"    min_time_horizon: " << min_time_horizon << std::endl;

    std::vector<VelDisk> all_disks;
    
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        int trajectory_index = findTrajectoryIndex(obstacles[i]);
        std::vector<VelDisk> disks = generateNLVODisks(ego_pose, obstacles[i], trajectory_index, r_total, min_time_horizon);
        all_disks.insert(all_disks.end(), disks.begin(), disks.end());
    }

    // Generate candidate velocities
    std::vector<twist_msg> candidate_velocities = generateCandidateVelocities(ego_vel);
    twist_msg best_velocity = ego_vel; // Default to current velocity

    // Check if candidate velocities are in the truncated NLVO
    std::vector<twist_msg> safe_vels;
    for (const auto &candidate_vel : candidate_velocities)
    {  
        if (!isVelocityInNLVO(candidate_vel, all_disks))
        {
            safe_vels.push_back(candidate_vel);
        }
    }

    // If no safe velocities found, generate emergency velocities
    if (safe_vels.empty())
    {
        std::cout<<"    No safe velocities found, returning emergency velocity."<<std::endl;
        best_velocity.linear.x = 0.0f;
        best_velocity.linear.y = 0.0f;
        return best_velocity;
    }

    float best_cost = std::numeric_limits<float>::max();
            
    for (const auto &candidate_vel : safe_vels)
    {
        float cost = calculateCandidateCost(ego_pose, ego_vel, obstacles, candidate_vel, goal_point, r_total, min_time_horizon);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_velocity = candidate_vel;
        }
    }

    std::cout<<std::endl;
    std::cout<<std::endl;
    return best_velocity; // Return the best velocity found among the candidates
}

float NLVO::computeMinimumTimeHorizon(const pose_msg &ego_pose, const twist_msg &ego_vel, const Obstacle& obstacle, float r_total, std::vector<std::pair<double, double>> control_set)
{
    // Old time horizon - delete if the new one is
    /*
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
    */

    float min_collision_time = max_time;
    
    // Calculate initial relative position
    pose_msg relative_pose;
    relative_pose.position.x = obstacle.pose.position.x - ego_pose.position.x;
    relative_pose.position.y = obstacle.pose.position.y - ego_pose.position.y;
    float d = std::sqrt(pow(relative_pose.position.x, 2) + pow(relative_pose.position.y, 2));

    if (d <= r_total) {
        return 0.0f;
    }

    float alpha = atan2(relative_pose.position.y, relative_pose.position.x);
    float theta = asin(r_total / d);

    // Calculate the time to collision for each control
    for (const auto &control : control_set)
    {
        float first_safe_time = max_time;

        for (float t = 0.0; t < max_time; t += dt)
        {
           // Ego velocity under this control at time t
           twist_msg ego_future_vel;
           ego_future_vel.linear.x = ego_vel.linear.x + control.first * t;
           ego_future_vel.linear.y = ego_vel.linear.y + control.second * t;

            // Relative velocity
            twist_msg v_rel;
            v_rel.linear.x = ego_future_vel.linear.x - obstacle.velocity.linear.x;
            v_rel.linear.y = ego_future_vel.linear.y - obstacle.velocity.linear.y;

            // Check if relative velocity is within the VO cone
            float beta = atan2(v_rel.linear.y, v_rel.linear.x);
            float angle_diff = normalizeAngle(beta - alpha);

            if (std::abs(angle_diff) > theta) 
            {
                first_safe_time = t;
                break; // Exit VO → we're safe
            } 
        }

        // Choose the shortest time among all controls
        min_collision_time = std::min(min_collision_time, first_safe_time);
    }

    return min_collision_time;
}

float NLVO::normalizeAngle(float angle)
{
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    return angle;
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

std::vector<twist_msg> NLVO::generateCandidateVelocities(const twist_msg& ego_vel)
{

    std::vector<twist_msg> candidate_velocities;
    twist_msg delta_v;
    twist_msg candidate_velocity;

    // Number of angles & accelerations to generate
    // Number of candidate velocities = num_of_angles * num_of_accelerations
    int num_of_angles = 10;
    int num_of_accelerations = 3;
    float angle_step = 2 * M_PI / num_of_angles;
    float acceleration_step = max_acceleration / num_of_accelerations;

    candidate_velocities.push_back(ego_vel);
    float acceleration, angle;
    for (acceleration = acceleration_step; acceleration <= max_acceleration; acceleration = acceleration + acceleration_step)
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

float NLVO::calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const std::vector<Obstacle>& obstacles, const twist_msg& candidate_velocity, const point_msg& goal_point, float r_total, float time_horizon)
{
    float cost = 0.0f;
    // cost function constant
    float obstacle_avoidance_weight = 30.0f;
    float goal_seeking_weight = 300.0f;
    float smoothness_weight = 20.0f;
    float time_step = 1.0f;
    
    // Obstacle avoidance 
    pose_msg ego_future_position;
    ego_future_position.position.x = ego_pose.position.x + candidate_velocity.linear.x * time_step;
    ego_future_position.position.y = ego_pose.position.y + candidate_velocity.linear.y * time_step;
    ego_future_position.position.z = ego_pose.position.z;
    pose_msg obstacle_future_pose; 
    
    float min_distance = std::numeric_limits<float>::max();
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        int traj_idx = findTrajectoryIndex(obstacles[i]);
        float speed = std::sqrt(std::pow(obstacles[i].velocity.linear.x, 2) + std::pow(obstacles[i].velocity.linear.y, 2));
        float obstacle_future_s_value = obstacles[i].s_values[traj_idx].x + speed * time_step;

        int obstacle_s_index = nextSIndex(obstacles[i], obstacle_future_s_value);
        obstacle_future_pose.position.x = obstacles[i].raceline[obstacle_s_index].x;
        obstacle_future_pose.position.y = obstacles[i].raceline[obstacle_s_index].y;

        float dist = std::sqrt(std::pow(ego_future_position.position.x - obstacle_future_pose.position.x, 2) + std::pow(ego_future_position.position.y - obstacle_future_pose.position.y, 2));
        if (dist < min_distance)
        {
            min_distance = dist;  
        }  
    }
    float obstacle_avoidance_cost = obstacle_avoidance_weight * (1.0f / (min_distance + 1e-3));
    
    // goal seeking cost
    float dx = goal_point.x - ego_future_position.position.x;
    float dy = goal_point.y - ego_future_position.position.y;

    float dist_to_goal = std::sqrt(std::pow(dx,2) + std::pow(dy, 2));
    float goal_seeking_cost = goal_seeking_weight * dist_to_goal; 

    // smoothness cost
    float dx_candidate_vel = candidate_velocity.linear.x - ego_velocity.linear.x;
    float dy_candidate_vel = candidate_velocity.linear.y - ego_velocity.linear.y;
    float dist_candidate_vel = std::sqrt(std::pow(dx_candidate_vel, 2) + std::pow(dy_candidate_vel, 2));
    float smoothness_cost = smoothness_weight * dist_candidate_vel;
    
    // Combine costs
    cost = obstacle_avoidance_cost + goal_seeking_cost + smoothness_cost;

    return cost;
}

std::vector<VelDisk> NLVO::generateNLVODisks(const pose_msg& ego_pose, const Obstacle& obstacle, int trajectory_index, float r_total, float time_horizon)
{
    std::vector<VelDisk> disks;
    float dt = 0.01f;
    float t = 0.0f;
    int obstacle_s_index = trajectory_index;
    float obstacle_max_s_value = obstacle.s_values[obstacle.s_values.size()-1].x;
    float speed = std::sqrt(std::pow(obstacle.velocity.linear.x, 2) + std::pow(obstacle.velocity.linear.y, 2));
    int disk_count = 0;

    std::cout<<"generateNLVODisks: " << std::endl;
    std::cout<<"    ego pose: " << ego_pose.position.x << ", " << ego_pose.position.y << std::endl;
    for (t = dt; t <= time_horizon; t += dt)
    {
        std::cout<<"    t: " << t << std::endl;
        std::cout<<"        S obs: " << obstacle.s_values[obstacle_s_index].x << std::endl;
        disk_count++;
        float obstacle_future_s_value = obstacle.s_values[obstacle_s_index].x + speed * dt;
        // float obstacle_future_s_value = obstacle.s_values[trajectory_index].x + speed * t;
        
        // find the wanted s value
        if (obstacle_future_s_value > obstacle_max_s_value)
        {
            obstacle_future_s_value = obstacle_future_s_value - obstacle_max_s_value;
        }

        std::cout<< "        obstacle_s_index: " << obstacle_s_index << std::endl;
        std::cout<< "        obstacle_future_s_value: " << obstacle_future_s_value << std::endl;

        pose_msg relative_pose;
        relative_pose.position.x = obstacle.raceline[obstacle_s_index].x - ego_pose.position.x;
        relative_pose.position.y = obstacle.raceline[obstacle_s_index].y - ego_pose.position.y;
        std::cout<<"        obstacle pose: " << obstacle.raceline[obstacle_s_index].x << ", " << obstacle.raceline[obstacle_s_index].y << std::endl;
        std::cout<<"        relative pose: " << relative_pose.position.x << ", " << relative_pose.position.y << std::endl;
        
        // Center of the NLVO disk in velocity space
        VelDisk disk;
        disk.cx = relative_pose.position.x / t;
        disk.cy = relative_pose.position.y / t;
        disk.radius = r_total / t;
        disks.push_back(disk);

        // find the closest s index to the given s value
        obstacle_s_index = nextSIndex(obstacle, obstacle_future_s_value);
    }

    
    return disks; 
}

int NLVO::findTrajectoryIndex(const Obstacle& obstacle)
{
    point_msg point;

    // fallback if raceline is empty
    if (obstacle.raceline.empty())
    {
        //std::cout<<"Raceline is empty"<<std::endl;
        point.x = obstacle.pose.position.x;
        point.y = obstacle.pose.position.y;
        point.z = obstacle.pose.position.z;
        return 0;

    }

    // Find closest point that is in front of ego
    int closest_index = 0;
    double min_dist_squared = std::numeric_limits<double>::max();

    // Iterate through the raceline points
    for (size_t i = 0; i < obstacle.raceline.size(); ++i)
    {
        const auto& raceline_point = obstacle.raceline[i];
        double dx = obstacle.pose.position.x - raceline_point.x;
        double dy = obstacle.pose.position.y - raceline_point.y;

        double squar_dist = dx * dx + dy * dy;

        if (squar_dist < min_dist_squared)
        {
            min_dist_squared = squar_dist;
            closest_index = static_cast<int>(i);
        }
    }
    return closest_index;
}

float NLVO::distance(const point_msg& s1, const point_msg& s2)
{
    float dx = s1.x - s2.x;
    float dy = s1.y - s2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool NLVO::isVelocityInNLVO(const twist_msg &candidate_vel, const std::vector<VelDisk> &disks)
{
    for (const auto& disk : disks)
    {
        float dx = candidate_vel.linear.x - disk.cx;
        float dy = candidate_vel.linear.y - disk.cy;
        // std::cout<<"relative velocity: " << dx << ", " << dy << std::endl;
        
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= disk.radius)
        {
            return true; // Candidate velocity is in the NLVO
        }
    }
    return false;
}

int NLVO::nextSIndex(const Obstacle &obstacle, float s)
{
    float min_dist = std::numeric_limits<float>::max();
    int future_index = 0;
    for (int i = 0; i < obstacle.s_values.size(); i++)
    {
        if (0 < std::abs(s - obstacle.s_values[i].x) < min_dist)
        {
            min_dist = std::abs(s - obstacle.s_values[i].x);
            future_index = i;
        }
    }
    return future_index;
}




