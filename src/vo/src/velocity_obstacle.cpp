#include "velocity_obstacle.hpp"
#include <cmath>
#include <string>
#include <iostream>
#include <limits>


// Global variables
float r_r = 0.5;
float r_o = 0.5;
float r_total = r_r + r_o;
float time_horizon = 3.0;
float max_acceleration = 1.0f;
float time_step = 0.5f;

VelocityObstacle::VelocityObstacle() 
{
    // Empty constructor
}

float VelocityObstacle::distance(const pose& ego_pos, const pose& obstacle_pos)
{
    float d = sqrt(pow(ego_pos.position.x - obstacle_pos.position.x, 2) + pow(ego_pos.position.y - obstacle_pos.position.y, 2));
    return d;
}

float VelocityObstacle::getAngle(const pose& s1, const pose& s2)
{
    float angle = atan2(s2.position.y - s1.position.y, s2.position.x - s1.position.x);
    return angle;
}

float VelocityObstacle::getTheta(float d)
{
    float theta = asin(r_total/d);
    return theta;
}

velocity VelocityObstacle::getVrelative(const velocity& v_ego, const velocity& v_obstacle)
{
    /*
    vo = ve - vo/e
    vo/e = ve - vo
    example for x:
    vo/e.x = ve.x - vo.x
    */
   
    velocity v_relative;
    v_relative.linear.x = v_ego.linear.x - v_obstacle.linear.x;
    v_relative.linear.y = v_ego.linear.y - v_obstacle.linear.y;
    return v_relative;
}

float VelocityObstacle::getBeta(const velocity& v_relative)
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

bool VelocityObstacle::checkCollision(const pose& ego_pose, const pose& obstacle_pose, const velocity& v_ego, const velocity& v_obstacle)
{
    // finding distance between ego pose and obstacle pose 
    float d = VelocityObstacle::distance(ego_pose, obstacle_pose);
    //std::cout<<"d: "<<d<<std::endl;

    if (d <= r_total) {
        //std::cout << "Warning: Overlapping radii. Immediate collision." << std::endl;
        return true;
    }

    // finding theta
    float theta = VelocityObstacle::getTheta(d);
    //std::cout<<"theta: "<<theta<<std::endl;

    // finding alpha angle
    float alpha =  VelocityObstacle::getAngle(ego_pose, obstacle_pose);
    //std::cout<<"alpha: "<<alpha<<std::endl;

    // finding v_relative
    velocity v_relative = VelocityObstacle::getVrelative(v_ego, v_obstacle);

    // finding beta angle 
    float beta = VelocityObstacle::getBeta(v_relative);
    //std::cout<<"beta: "<<beta<<std::endl;

    float angle_diff = VelocityObstacle::normalizeAngle(beta - alpha);

    // check if relative velocity is inside the cone
    float condition = std::abs(angle_diff);

    //std::cout<<"condition: "<<condition<<std::endl;

    float relative_speed = sqrt(pow(v_relative.linear.x, 2) + pow(v_relative.linear.y, 2)); // calculate the magnitude of the relative velocity
    float time_to_collision = d / relative_speed; //  calculate the current time to collision
    
    // if the relative velocity is inside the cone and the time to collision is less than the time horizon
    if ((time_to_collision <= time_horizon) && (condition <= theta)) 
    {
        return true;
    }
    else
    {
        return false;
    }
    
}


std::vector<velocity> VelocityObstacle::generateCandidateVelocities(const velocity& ego_vel) 
{
    
    std::vector<velocity> possible_velocities;
    float delta = max_acceleration * time_step;
    velocity delta_v;
    velocity candidate_velocity;
    float angle_step = M_PI / 4.0f;  // 45 degrees = pi/4 radians 

    // Generate 9 sample velocities around the ego velocity
    // for (int i = 0; i < 8; i++)
    // {
    //     float angle = i * angle_step; // Calculate the angle for the current sample
        
    //     delta_v.linear.x = delta * std::cos(angle);
    //     delta_v.linear.y = delta * std::sin(angle);

    //     candidate_velocity.linear.x = ego_vel.linear.x + delta_v.linear.x;
    //     candidate_velocity.linear.y = ego_vel.linear.y + delta_v.linear.y;

    //     std::cout << "Candidate velocity: (" << candidate_velocity.linear.x << ", " << candidate_velocity.linear.y << ")" << std::endl;

    //     possible_velocities.push_back(candidate_velocity);
    // }


    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) 
        {
            // Calculating dv = a * dt
            // where a is the acceleration and dt is the time step
            delta_v.linear.x = i * max_acceleration * time_step;
            delta_v.linear.y = j * max_acceleration * time_step;

            // Calculate the candidate velocity: candidate_velocity = ego_velocity + delta_v
            candidate_velocity.linear.x = ego_vel.linear.x + delta_v.linear.x;
            candidate_velocity.linear.y = ego_vel.linear.y + delta_v.linear.y;

            std::cout << "Candidate velocity: (" << candidate_velocity.linear.x << ", " << candidate_velocity.linear.y << ")" << std::endl;

            possible_velocities.push_back(candidate_velocity);
        }
    }

    return(possible_velocities);
}

pose  VelocityObstacle::findNextGoalPoint(const std::vector<pose>& raceline, const pose& ego_pose)
{
    int lookahead = 1;
    
    if (raceline.empty()) return ego_pose; // fallback if raceline is empty

    // find the index of the closest point
    int closest_index = 0;
    float min_dist = std::numeric_limits<float>::max(); // large number

    for (int i = 0; i < raceline.size(); i++)
    {
        float dx = raceline[i].position.x - ego_pose.position.x;
        float dy = raceline[i].position.y - ego_pose.position.y;
        float dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_index = i; 
        }
    }

    int goal_index = std::min(closest_index + lookahead, static_cast<int>(raceline.size()-1)); // static_cast<int>(raceline.size()-1) convert size_t to int
    return raceline[goal_index];
}

float VelocityObstacle::calculateCollisionCost(const pose& ego_pose, const velocity& ego_velocity, const std::vector<pose>obstacle_poses,const std::vector<velocity>obstacle_vels, const velocity& candidate_velocity, const pose& goal_point)
{
    float cost = 0.0f;
    // cost function constant
    float obstacle_avoidance_wight = 5.0f;
    float goal_seeling_wight = 0.1f;
    float smoothness_wight = 0.1f;
    float relative_speed = 0.0f;
    float obstacle_avoidance_cost = 0.0f;
    
    // Smoothness cost: minimize velocity changes
    float smoothness_cost = smoothness_wight * std::sqrt(std::pow(candidate_velocity.linear.x - ego_velocity.linear.x, 2) + std::pow(candidate_velocity.linear.y - ego_velocity.linear.y, 2));
    std::cout << "Smoothness cost: " << smoothness_cost << std::endl;

    // Obstacle avoidance 
    for (size_t i = 0; i < obstacle_poses.size(); i++)
    {
        velocity relative_velocity = getVrelative(candidate_velocity, obstacle_vels[i]);
        relative_speed = std::sqrt(std::pow(relative_velocity.linear.x, 2) + std::pow(relative_velocity.linear.y, 2));
        obstacle_avoidance_cost = obstacle_avoidance_wight * relative_speed;
    }
    std::cout << "Obstacle avoidance cost: " << obstacle_avoidance_cost << std::endl;
    
    // goal seeking cost
    float dx = goal_point.position.x - ego_pose.position.x;
    float dy = goal_point.position.y - ego_pose.position.y;

    float dist_to_goal = std::sqrt(std::pow(dx,2) + std::pow(dy, 2));
    float goal_seeking_cost =goal_seeling_wight * dist_to_goal; 
    std::cout << "goal seeking cost cost: " << goal_seeking_cost << std::endl;
    
    cost = obstacle_avoidance_cost + goal_seeking_cost + smoothness_cost;

    return cost;
}

velocity VelocityObstacle::selectBestVelocity(const pose& ego_pose, const velocity& ego_vel, const std::vector<pose>& obstacle_poses, const std::vector<velocity>& obstacle_vels, const std::vector<pose>& raceline) 
{
    std::vector<velocity> candidates = generateCandidateVelocities(ego_vel); // Generate candidate velocities
    pose goal_point = findNextGoalPoint(raceline, ego_pose); // Get goal point from raceline
    std::cout << "Goal point: (" << goal_point.position.x << ", " << goal_point.position.y << ")" << std::endl;
    
    float best_cost = std::numeric_limits<float>::max(); // Initialize with a large number
    std::cout << "Best cost: " << best_cost << std::endl;
    velocity best_velocity = ego_vel; // Default to current velocity

    for (const auto& candidate : candidates)
    {
        bool collision = false;

        // Check collision with each obstacle
        for (size_t i = 0; i < obstacle_poses.size(); ++i)
        {
            if (checkCollision(ego_pose, obstacle_poses[i], ego_vel, obstacle_vels[i]))
            {
                collision = true;
                break;
            }
        }
        
        if (!collision) 
        {
            std::cout << "Current velocity is safe. Continuing.\n";
            return ego_vel;
        }

        // Find the best alternative velocity
        else
        {
            pose obstacle_future_position;
            velocity obstacle_future_velocity;
            for(const auto& candidate : candidates)
            {
                std::cout << std::endl;
                bool candidate_collision = false;

                for(size_t i = 0; i < obstacle_poses.size(); i++)
                {
                    // Check collision with the candidate velocity
                    obstacle_future_velocity.linear.x = obstacle_vels[i].linear.x + max_acceleration * time_step;
                    obstacle_future_velocity.linear.y = obstacle_vels[i].linear.y + max_acceleration * time_step;
                    if (checkCollision(ego_pose, obstacle_poses[i], candidate, obstacle_future_velocity)) 
                    {
                        candidate_collision = true;
                        std::cout << "Candidate velocity collides with obstacle: " << i << std::endl;
                        std::cout << "Candidate velocity collides with candidate: " << candidate.linear.x << ", " << candidate.linear.y << std::endl;
                        break;
                    }
                }

                if (!candidate_collision) // the current candidate is not collide
                {
                    std::cout << "Candidate velocity is safe. Continuing.\n";
                    float cost = calculateCollisionCost(ego_pose, ego_vel, obstacle_poses, obstacle_vels, candidate, goal_point);
                    std::cout << "Candidate velocity cost: " << cost << std::endl;
                    // Check if the cost is lower than the best cost
                    if (cost < best_cost)
                    {
                        std::cout << "Found a better candidate velocity: " << candidate.linear.x << ", " << candidate.linear.y << std::endl;
                        best_cost = cost;
                        best_velocity = candidate;
                    }
                }
            }
        }
    } 
    std::cout << "\nBest velocity found: (" << best_velocity.linear.x << ", " << best_velocity.linear.y << ") with cost: " << best_cost << std::endl;
    return best_velocity; // Return the best velocity found among the candidates
}




