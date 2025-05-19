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
    //  Find the minimum time horizon
    float min_time_horizon = max_time;
    std::cout << "ego pose: (" << ego_pose.position.x << ", " << ego_pose.position.y << ")" << std::endl;
    for (size_t i = 0; i < obstacles_poses.size(); i++)
    {
        float time_horizon = computeMinimumTimeHorizon(ego_pose, ego_vel, obstacles_poses[i], obstacle_vels[i], r_total);
        min_time_horizon = std::min(min_time_horizon, time_horizon);
    }

    std::cout << "Minimum time horizon: " << min_time_horizon << std::endl;
    // Generate candidate velocities
    std::vector<twist_msg> candidate_velocities = generateCandidateVelocities(ego_vel);
    std::cout << "the candidate generated velocities: " << std::endl;
    for (const auto &candidate_vel : candidate_velocities)
    {
        std::cout << "Candidate velocity: (" << candidate_vel.linear.x << ", " << candidate_vel.linear.y << ")" << std::endl;
    }
    // Check if candidate velocities are in the truncated NLVO
    std::vector<twist_msg> safe_vels;
    for (const auto &candidate_vel : candidate_velocities)
    {
        bool is_safe = true;
        for (size_t i = 0; i < obstacles_poses.size(); i++)
        {
            std::cout << "Checking candidate velocity against obstacle " << i << std::endl;
            std::cout << "Obstacle pose: (" << obstacles_poses[i].position.x << ", " << obstacles_poses[i].position.y << ")" << std::endl;
            std::cout << "Obstacle velocity: (" << obstacle_vels[i].linear.x << ", " << obstacle_vels[i].linear.y << ")" << std::endl;
            if (isVelocityInTruncatedNLVO(candidate_vel, ego_pose, ego_vel, obstacles_poses[i], obstacle_vels[i], r_total, min_time_horizon))
            {
                is_safe = false;
                std::cout << "Candidate velocity is in the truncated NLVO: (" << candidate_vel.linear.x << ", " << candidate_vel.linear.y << ")" << std::endl;
                break;
            }
        }
        if (is_safe)
        {
            std::cout << "Candidate velocity is safe: (" << candidate_vel.linear.x << ", " << candidate_vel.linear.y << ")" << std::endl;
            safe_vels.push_back(candidate_vel);
        }
    }
    


    // Select the best velocity from the safe velocities
    point_msg to_goal;
    to_goal.x = goal_point.x - ego_pose.position.x;
    to_goal.y = goal_point.y - ego_pose.position.y;
    to_goal.z = ego_pose.position.z;

    twist_msg current_ego_vel = ego_vel;

    float best_cost = calculateCandidateCost(ego_pose, ego_vel, obstacles_poses, obstacle_vels, current_ego_vel, goal_point); 
    twist_msg best_velocity = ego_vel; // Default to current velocity
    for (const auto &candidate_vel : safe_vels)
    {
        float cost = calculateCandidateCost(ego_pose, ego_vel, obstacles_poses, obstacle_vels, candidate_vel, goal_point);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_velocity = candidate_vel;
        }
    }
    return best_velocity; // Return the best velocity found among the candidates
}

float NLVO::computeMinimumTimeHorizon(const pose_msg &ego_pose, const twist_msg &ego_vel, const pose_msg &obstacle_pose, const twist_msg &obstacle_vel, float r_total)
{
    // 4 accelerations (bang-bang corners of control set)
    std::vector<accel_msg> accelerations;

    // Create a set of control limits
    std::vector<std::pair<double, double>> control_set = {
        {control_limit_x, control_limit_y},
        {-control_limit_x, control_limit_y},
        {control_limit_x, -control_limit_y},
        {-control_limit_x, -control_limit_y},
    };

    float min_collision_time = max_time;
    float r_total_squared = r_total * r_total;
    float first_collision_time_for_control;

    // For every control in the set, create a corresponding acceleration message
    for (const auto &control : control_set)
    {
        first_collision_time_for_control = max_time;
        for (float t = dt; t < max_time; t += dt)
        {
            for (const auto &acceleration : accelerations)
            {
                // Calculate the ego future velocity
                twist_msg ego_future_vel;
                ego_future_vel.linear.x = ego_vel.linear.x + control.first * dt;
                ego_future_vel.linear.y = ego_vel.linear.y + control.second * dt;
    
                twist_msg relative_velocity;
                relative_velocity.linear.x = ego_future_vel.linear.x - obstacle_vel.linear.x;
                relative_velocity.linear.y = ego_future_vel.linear.y - obstacle_vel.linear.y;
    
                // Calculate the relative velocity
                pose_msg relative_pose;
                relative_pose.position.x = ego_pose.position.x - obstacle_pose.position.x;
                relative_pose.position.y = ego_pose.position.y - obstacle_pose.position.y;
    
                // / Relative position at time t
                pose_msg future_relative_pose;
                future_relative_pose.position.x = relative_pose.position.x + relative_velocity.linear.x * t;
                future_relative_pose.position.y = relative_pose.position.y + relative_velocity.linear.y * t;
    
                double future_relative_pose_length_squared = pow(future_relative_pose.position.x, 2) + pow(future_relative_pose.position.y, 2);
                
                if (future_relative_pose_length_squared <= r_total_squared) // Collision detected
                {
                    first_collision_time_for_control = t;
                    break; // Found the first collision time for this control, move to the next control
                }
            }
        }
    }
    min_collision_time = std::min(min_collision_time, first_collision_time_for_control);
    return min_collision_time; // No escape found
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
    for (float t = dt; t < time_horizon; t += dt)
    {
        
        point_msg center;
        center.x = obstacle_pose.position.x + obstacle_vel.linear.x * t;
        center.y = obstacle_pose.position.y + obstacle_vel.linear.y * t;

        // Va * t
        pose_msg ego_future_pose;
        ego_future_pose.position.x = ego_pose.position.x + candidate_vel.linear.x * t;
        ego_future_pose.position.y = ego_pose.position.y + candidate_vel.linear.y * t;
        
        
        // Find scaled relative pose
        pose_msg scaled_relative_pose;
        scaled_relative_pose.position.x = (ego_future_pose.position.x - center.x);
        scaled_relative_pose.position.y = (ego_future_pose.position.y - center.y);

        double relative_pose_length_squared = pow((scaled_relative_pose.position.x), 2) + pow((scaled_relative_pose.position.y), 2);
        
        //std::cout << "t=" << t << " dist=" << std::sqrt(relative_pose_length_squared) << " vs r=" << r_total << std::endl;
        
        if (relative_pose_length_squared <= r_total_squared) // Collision detected
        {
            return true; // Candidate velocity is in the truncated NLVO
        }
    }
    return false; // Candidate velocity is not in the truncated NLVO
}

float NLVO::evaluateCost(const twist_msg &candidate_vel, const point_msg &to_goal, const twist_msg& ego_vel)
{ 
    float speed_wight = 0.001f;
    float alignment_weight = 1.0f;
    float lookahead_time = 5.0f;
    float max_speed = 20.0f;

    float dot_product = candidate_vel.linear.x * to_goal.x + candidate_vel.linear.y * to_goal.y;
    float to_goal_magnitude = sqrt(pow(to_goal.x, 2) + pow(to_goal.y, 2));
    if (dot_product <= 0)
    {
        return std::numeric_limits<float>::max(); // Cost is infinite if the candidate velocity is not in the direction of the goal
    }
    
    /*
    cost = 1 / |candidate_vel| * cos alpha 
    if align with goal - > lower cost
    if velocity is faster - > lower cost
    */
    float alignment_cost = alignment_weight * (to_goal_magnitude / (dot_product+ 1e-6)); // Cost is the time-to-go

    float candidate_speed = sqrt(pow(candidate_vel.linear.x, 2) + pow(candidate_vel.linear.y, 2));
    float desired_speed = to_goal_magnitude / lookahead_time;
    float speed_cost = speed_wight * std::pow((candidate_speed - desired_speed) / max_speed, 2);
    
    return alignment_cost + speed_cost; // Return the total cost
}


std::vector<twist_msg> NLVO::generateCandidateVelocities(const twist_msg& ego_vel) 
{
    
    std::vector<twist_msg> candidate_velocities;
    twist_msg delta_v;
    twist_msg candidate_velocity;    
     
    // Number of angles & accelerations to generate
    // Number of candidate velocities = num_of_angles * num_of_accelerations
    int num_of_angles = 30;
    int num_of_accelerations = 5;
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

        //std::cout << "Candidate velocity: (" << candidate_velocity.linear.x << ", " << candidate_velocity.linear.y << ")" << std::endl;
           candidate_velocities.push_back(candidate_velocity);
       }
    }

    return candidate_velocities;
}


float NLVO::calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const std::vector<pose_msg>obstacle_poses,const std::vector<twist_msg>obstacle_vels, const twist_msg& candidate_velocity, const point_msg& goal_point)
{
    // std::cout << "ego position: " << ego_pose.position.x << ", " << ego_pose.position.y << std::endl;
    // std::cout << "ego velocity: " << ego_velocity.linear.x << ", " << ego_velocity.linear.y << std::endl;
    // std::cout << "candidate velocity: " << candidate_velocity.linear.x << ", " << candidate_velocity.linear.y << std::endl;
    // std::cout << "goal point: " << goal_point.position.x << ", " << goal_point.position.y << std::endl;
    // std::cout << "obstacle poses: " << std::endl;
    // for (size_t i = 0; i < obstacle_poses.size(); i++)
    // {
    //     std::cout << "obstacle " << i << ": " << obstacle_poses[i].position.x << ", " << obstacle_poses[i].position.y << std::endl;
    // }
    // std::cout << "obstacle velocities: " << std::endl;
    // for (size_t i = 0; i < obstacle_vels.size(); i++)
    // {
    //     std::cout << "obstacle " << i << ": " << obstacle_vels[i].linear.x << ", " << obstacle_vels[i].linear.y << std::endl;
    // }

    float cost = 0.0f;
    // cost function constant
    float obstacle_avoidance_weight = 50.0f;
    float goal_seeling_weight = 400.0f;
    float smoothness_weight = 250.0f;
    
    // Obstacle avoidance 
    pose_msg ego_future_position;
    ego_future_position.position.x = ego_pose.position.x + candidate_velocity.linear.x * dt;
    ego_future_position.position.y = ego_pose.position.y + candidate_velocity.linear.y * dt;
    ego_future_position.position.z = ego_pose.position.z;
    pose_msg obstacle_future_pose; 
    
    float min_distance = std::numeric_limits<float>::max();
    for (size_t i = 0; i < obstacle_poses.size(); i++)
    {
        obstacle_future_pose.position.x = obstacle_poses[i].position.x + obstacle_vels[i].linear.x * dt;
        obstacle_future_pose.position.y = obstacle_poses[i].position.y + obstacle_vels[i].linear.y * dt;
        obstacle_future_pose.position.z = obstacle_poses[i].position.z;

        float dist =std::sqrt(std::pow(ego_future_position.position.x - obstacle_future_pose.position.x,2) + std::pow(ego_future_position.position.y - obstacle_future_pose.position.y, 2));
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
}

