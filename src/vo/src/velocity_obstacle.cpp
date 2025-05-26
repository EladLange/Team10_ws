#include "velocity_obstacle.hpp"
#include <cmath>
#include <string>
#include <iostream>
#include <limits>



VelocityObstacle::VelocityObstacle() 
{
    // Empty constructor
}

float VelocityObstacle::distance(const pose_msg& ego_pos, const pose_msg& obstacle_pos)
{
    float d = sqrt(pow(ego_pos.position.x - obstacle_pos.position.x, 2) + pow(ego_pos.position.y - obstacle_pos.position.y, 2));
    return d;
}

float VelocityObstacle::getAngle(const pose_msg& s1, const pose_msg& s2)
{
    float angle = atan2(s2.position.y - s1.position.y, s2.position.x - s1.position.x);
    return angle;
}

float VelocityObstacle::getTheta(float d, float r_total)
{
    float theta = asin(r_total/d);
    return theta;
}

twist_msg VelocityObstacle::getVrelative(const twist_msg& v_ego, const twist_msg& v_obstacle)
{
    /*
    vo = ve - vo/e
    vo/e = ve - vo
    example for x:
    vo/e.x = ve.x - vo.x
    */
   
    twist_msg v_relative;
    v_relative.linear.x = v_ego.linear.x - v_obstacle.linear.x;
    v_relative.linear.y = v_ego.linear.y - v_obstacle.linear.y;
    return v_relative;
}

float VelocityObstacle::getBeta(const twist_msg& v_relative)
{
    float beta = atan2(v_relative.linear.y , v_relative.linear.x);
    return beta;
}

float VelocityObstacle::normalizeAngle(float angle)
{
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle >  M_PI)  angle -= 2 * M_PI;
    return angle;
}

bool VelocityObstacle::checkCollision(const pose_msg& ego_pose, const pose_msg& obstacle_pose, const twist_msg& v_ego, const twist_msg& v_obstacle, float r_total)
{
    // finding distance between ego pose and obstacle pose 
    float d = VelocityObstacle::distance(ego_pose, obstacle_pose);
    //std::cout<<"d: "<<d<<std::endl;

    if (d <= r_total) {
        //std::cout << "Warning: Overlapping radii. Immediate collision." << std::endl;
        return true;
    }

    // finding theta
    float theta = VelocityObstacle::getTheta(d, r_total);
    //std::cout<<"theta: "<<theta<<std::endl;

    // finding alpha angle
    float alpha =  VelocityObstacle::getAngle(ego_pose, obstacle_pose);
    //std::cout<<"alpha: "<<alpha<<std::endl;

    // finding v_relative
    twist_msg v_relative = VelocityObstacle::getVrelative(v_ego, v_obstacle);

    // finding beta angle 
    float beta = VelocityObstacle::getBeta(v_relative);
    //std::cout<<"beta: "<<beta<<std::endl;

    float angle_diff = VelocityObstacle::normalizeAngle(beta - alpha);

    // check if relative velocity is inside the cone
    float condition = std::abs(angle_diff);

    //std::cout<<"condition: "<<condition<<std::endl;

    float relative_speed = sqrt(pow(v_relative.linear.x, 2) + pow(v_relative.linear.y, 2)); // calculate the magnitude of the relative velocity
    float time_to_collision = d / relative_speed; //  calculate the current time to collision

    // Velocities are almost equal, treat as no collision
    if (relative_speed < 1e-3) {
        return false;
    }
    
    // if the relative velocity is inside the cone and the time to collision is less than the time horizon
    else if ((time_to_collision <= time_horizon) && (condition <= theta)) 
    {
        return true;
    }
    else
    {
        return false;
    }
    
}


std::vector<twist_msg> VelocityObstacle::generateCandidateVelocities(const twist_msg& ego_vel) 
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
       float delta_v_magnitude = acceleration * time_step;
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

float VelocityObstacle::calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const std::vector<pose_msg>obstacle_poses,const std::vector<twist_msg>obstacle_vels, const twist_msg& candidate_velocity, const point_msg& goal_point)
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
    float obstacle_avoidance_weight = 100.0f;
    float goal_seeling_weight = 250.0f;
    float smoothness_weight = 50.0f;
    
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

        float dist = distance(ego_future_position, obstacle_future_pose);
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

twist_msg VelocityObstacle::selectBestVelocity(const pose_msg& ego_pose, const twist_msg& ego_vel, const std::vector<pose_msg>& obstacle_poses, const std::vector<twist_msg>& obstacle_vels, const point_msg& goal_point, float r_total) 
{
    std::vector<twist_msg> candidates = generateCandidateVelocities(ego_vel); // Generate candidate velocities
    // std::cout << "Goal point: (" << goal_point.x << ", " << goal_point.y << ")" << std::endl;
    // std::cout << "Ego position: (" << ego_pose.position.x << ", " << ego_pose.position.y << ")" << std::endl;
    // std::cout << "Ego orientation: (" << ego_pose.orientation.x << ", " << ego_pose.orientation.y << ", " << ego_pose.orientation.z << ", " << ego_pose.orientation.w << ")" << std::endl;
    float best_cost = std::numeric_limits<float>::max(); // Initialize with a large number
    //std::cout << "Best cost: " << best_cost << std::endl;
    twist_msg best_velocity; // Default to current velocity
    best_velocity.linear.x = 0.0f;
    best_velocity.linear.y = 0.0f;

    for(const auto& candidate : candidates)
    {
        // std::cout << std::endl;
        bool candidate_collision = false;

        for(size_t i = 0; i < obstacle_poses.size(); i++)
        {
            if (checkCollision(ego_pose, obstacle_poses[i], candidate, obstacle_vels[i], r_total)) 
            {
                candidate_collision = true;
                // std::cout << "Candidate velocity collides with obstacle: " << i << std::endl;
                // std::cout << "Candidate velocity collides with candidate: " << candidate.linear.x << ", " << candidate.linear.y << std::endl;
                break;
            }
        }

        if (!candidate_collision) // the current candidate is not collide
        {
            //std::cout << "Candidate velocity is safe. Continuing.\n";
            float cost = calculateCandidateCost(ego_pose, ego_vel, obstacle_poses, obstacle_vels, candidate, goal_point);
            // std::cout << "Candidate " << candidate.linear.x << ", " << candidate.linear.y << " has " << cost << "" << std::endl;
            // Check if the cost is lower than the best cost
            if (cost < best_cost)
            {
               //std::cout << "Found a better candidate velocity: " << candidate.linear.x << ", " << candidate.linear.y << std::endl;
                best_cost = cost;
                best_velocity = candidate;
            }
        }
    }

    //std::cout << "\nBest velocity found: (" << best_velocity.linear.x << ", " << best_velocity.linear.y << ") with cost: " << best_cost << std::endl;
    return best_velocity; // Return the best velocity found among the candidates
}




