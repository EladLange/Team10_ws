#include "velocity_obstacle.hpp"
#include <cmath>
#include <string>
#include <iostream>
#include <limits>


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
    std::cout<<"d: "<<d<<std::endl;

    if (d <= r_total) {
        std::cout << "Warning: Overlapping radii. Immediate collision." << std::endl;
        return;
    }

    // finding theta
    float theta = VelocityObstacle::getTheta(d);
    std::cout<<"theta: "<<theta<<std::endl;

    // finding alpha angle
    float alpha =  VelocityObstacle::getAngle(ego_pose, obstacle_pose);
    std::cout<<"alpha: "<<alpha<<std::endl;

    // finding v_relative
    velocity v_relative = VelocityObstacle::getVrelative(v_ego, v_obstacle);

    // finding beta angle 
    float beta = VelocityObstacle::getBeta(v_relative);
    std::cout<<"beta: "<<beta<<std::endl;

    float angle_diff = VelocityObstacle::normalizeAngle(beta - alpha);

    // check if relative velocity is inside the cone
    float condition = std::abs(angle_diff);

    std::cout<<"condition: "<<condition<<std::endl;

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

    // Generate 9 sample velocities around the ego velocity
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            velocity delta_v;
            velocity candidate_velocity;

            // Calculating dv = a * dt
            // where a is the acceleration and dt is the time step
            delta_v.linear.x = i * max_acceleration * time_step;
            delta_v.linear.y = j * max_acceleration * time_step;

            // Calculate the candidate velocity: candidate_velocity = ego_velocity + delta_v
            candidate_velocity.linear.x = ego_vel.linear.x + delta_v.linear.x;
            candidate_velocity.linear.y = ego_vel.linear.y + delta_v.linear.y;

            possible_velocities.push_back(candidate_velocity);
        }
    }

}

pose findNextGoalPoint(const std::vector<pose>& raceline, const pose& ego_pose)
{
    int lookahead = 5;
    
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
    float obstacle_avoidance_cost = 5.0f;
    float goal_seeling_cost = 0.1f;
    float relative_speed = 0.0f;
    
    // Smoothness cost: minimize velocity changes
    cost += std::sqrt(std::pow(candidate_velocity.linear.x - ego_velocity.linear.x, 2) + std::pow(candidate_velocity.linear.y - ego_velocity.linear.y, 2));

    // Obstacle avoidance 
    for (size_t i = 0; i < obstacle_poses.size(); i++)
    {
        velocity relative_velocity = getVrelative(candidate_velocity, obstacle_vels[i]);
        relative_speed = std::sqrt(std::pow(relative_velocity.linear.x, 2) + std::pow(relative_velocity.linear.y, 2));
        cost += obstacle_avoidance_cost * relative_speed;
    }
    
    // goal seeking cost
    float dx = goal_point.position.x - ego_pose.position.x;
    float dy = goal_point.position.y - ego_pose.position.y;

    float dist_to_goal = std::sqrt(std::pow(dx,2) + std::pow(dy, 2));
    cost += goal_seeling_cost * dist_to_goal; 
    
    return cost;
}

velocity VelocityObstacle::selectBestVelocity(const pose& ego_pose, const velocity& ego_vel, const std::vector<pose>& obstacle_poses, const std::vector<velocity>& obstacle_vels, const std::vector<pose>& raceline) 
{
    // Generate candidate velocities
    std::vector<velocity> candidates = generateCandidateVelocities(ego_vel);

    // Get goal point from raceline
    pose goal_point = findNextGoalPoint(raceline, ego_pose);

    float best_cost = std::numeric_limits<float>::max(); // Initialize with a large number
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
        
        if (!collision) {
            std::cout << "Current velocity is safe. Continuing.\n";
            return ego_vel;
        }

        // Find the best alternative velocity
        else
        {
            for(const auto& candidate : generateCandidateVelocities(ego_vel))
            {
                bool candidate_collision = false;

                for(size_t i = 0; i < obstacle_poses.size(); i++)
                {
                    if (checkCollision(ego_pose, obstacle_poses[i], candidate, obstacle_vels[i]))
                    candidate_collision = true;
                    break;
                }

                if (!candidate_collision) // the current candidate is not collide
                {
                    float cost = calculateCollisionCost(ego_pose, ego_vel, obstacle_poses, obstacle_vels, candidate, goal_point);

                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        best_velocity = candidate;
                    }
                }
            }
        }
    } 

}




