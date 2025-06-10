#include "nao/nao.hpp"

NAO::NAO()
{
    // Empty constructor
}

twist_msg NAO::selectBestVelocity(const accel_msg& ego_accel, const twist_msg& ego_vel,const pose_msg& ego_pose, const std::vector<Obstacle>& obstacles_, const point_msg &goal_point, float r_total)
{
    // Step one - generate candidate accelerations:
    std::vector<accel_msg> candidate_accels = generateCandidateAccel(ego_accel);
    
    // Step two - create NAO disks
    std::vector<AccDisk> all_disks;
    for (size_t i = 0; i < obstacles_.size(); i++)
    {
        int traj_index = findTrajectoryIndex(obstacles_[i]);
        std::cout<< "trajectory index: " << traj_index << std::endl;
        std::vector<AccDisk> disks = generateNAODisks(obstacles_[i], ego_vel, ego_pose, traj_index, r_total);
        all_disks.insert(all_disks.end(), disks.begin(), disks.end());
    }

    for (const auto &disk : all_disks)
    {
        std::cout << "Disk center: (" << disk.cx << ", " << disk.cy << "), radius: " << disk.radius << std::endl;
    }

    // Step three - check if candidate accelerations are in the NAO
    accel_msg best_acceleration = ego_accel; // Default to current acceleration
    twist_msg best_velocity;
    std::vector<accel_msg> safe_accels;
    for (const auto &candidate_accel : candidate_accels)
    {  
        if (!isAccelInNao(candidate_accel, all_disks))
        {
            safe_accels.push_back(candidate_accel);
        }
    }

    // If no safe accelerations found, return the maximum acceleration im the inverse direction to the ego velocity
    if (safe_accels.empty())
    {
        float theta = std::atan2(ego_vel.linear.y, ego_vel.linear.x);
        best_acceleration.linear.x = -max_acceleration * std::cos(theta);
        best_acceleration.linear.y = -max_acceleration * std::sin(theta);

        return accelerationToVelocity(best_acceleration, dt);
    }

    // Step four - transform the safe accelerations to velocities
    std::vector<twist_msg> safe_velocities;
    for (const auto &safe_accel : safe_accels)
    {
        twist_msg velocity = accelerationToVelocity(safe_accel, dt);
        safe_velocities.push_back(velocity);
    }
    
    
    // Step five - find the best Velocity
    float best_cost = std::numeric_limits<float>::max();
    int best_index = 0;
    for (const auto &candidate_vel : safe_velocities)
    {
        float cost = calculateCandidateCost(ego_pose, ego_vel, ego_accel, candidate_vel, goal_point, r_total, time_horizon);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_velocity.linear.x = candidate_vel.linear.x;
            best_velocity.linear.y = candidate_vel.linear.y;
            
            global_best_accel= safe_accels[best_index];
        } 
        best_index++;
    }
    return best_velocity;
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
            acceleration.linear.y = accel * std::sin(angle); 

            candidates_accel.push_back(acceleration);
        }
    }
    return candidates_accel;
}

std::vector<AccDisk> NAO::generateNAODisks(const Obstacle& obstacle, const twist_msg& ego_vel, const pose_msg& ego_pose, int trajectory_index,  float total_r)
{
    float dt = 0.05f;
    float t = 0.0f;
    std::vector<AccDisk> disks;
    
    float obstacle_max_s_value = obstacle.s_values[obstacle.s_values.size()-1].x;
    int obstacle_s_index = trajectory_index;
    float speed = std::sqrt((obstacle.velocity.linear.x * obstacle.velocity.linear.x) + (obstacle.velocity.linear.y * obstacle.velocity.linear.y));
    float accel_mag= std::sqrt((obstacle.acceleration.linear.x * obstacle.acceleration.linear.x) + (obstacle.acceleration.linear.y * obstacle.acceleration.linear.y));

    for (t = dt; t < time_horizon; t++)
    {
        float obstacle_future_s_value = (accel_mag*dt*dt)/2 + obstacle.s_values[obstacle_s_index].x + speed * dt;
        
        if (obstacle_future_s_value > obstacle_max_s_value)
        {
            obstacle_future_s_value = obstacle_future_s_value - obstacle_max_s_value;
        }

        pose_msg relative_pose;
        relative_pose.position.x = obstacle.raceline[obstacle_s_index].x - ego_pose.position.x;
        relative_pose.position.y = obstacle.raceline[obstacle_s_index].y - ego_pose.position.y;

        // find relative velocity-ask Shiller
        twist_msg relative_velocity;
        relative_velocity.linear.x = obstacle.velocity.linear.x - ego_vel.linear.x;
        relative_velocity.linear.y = obstacle.velocity.linear.y - ego_vel.linear.y;
        // Center of the NAO disk in velocity space
        AccDisk disk;
        disk.cx = (2*relative_pose.position.x)/(t*t) + relative_velocity.linear.x/t;
        disk.cy = (2*relative_pose.position.y)/(t*t) + relative_velocity.linear.y/t;
        disk.radius = (total_r * 2) / t*t;
        disks.push_back(disk);
    }
    return disks;
}

bool NAO::isAccelInNao(const accel_msg& candidate_accel, const std::vector<AccDisk>& disks)
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

twist_msg NAO::accelerationToVelocity(const accel_msg& safe_accel, float dt)
{
    /*
    V= V0 + a * dt
    */

    twist_msg velocity;
    velocity.linear.x = safe_accel.linear.x * dt;
    velocity.linear.y = safe_accel.linear.y * dt;
    std::cout << "Velocity: (" << velocity.linear.x << ", " << velocity.linear.y << ")" << std::endl;
    return velocity;
}

int NAO::findTrajectoryIndex(const Obstacle &obstacle)
{
    point_msg point;

    // fallback if raceline is empty
    if (obstacle.raceline.empty())
    {
        //std::cout<<"Raceline is empty"<<std::endl;
        point.x=obstacle.pose.position.x;
        point.y=obstacle.pose.position.y;
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

float NAO::calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_velocity, const accel_msg& ego_accel, const twist_msg& candidate_velocity, const point_msg& goal_point, float r_total, float time_horizon)
{
    float cost = 0.0f;
    // cost function constant
    float goal_seeking_weight = 120.0f;
    float smoothness_weight = 30.0f;
    
    // Obstacle avoidance 
    pose_msg ego_future_position;
    ego_future_position.position.x = ego_pose.position.x + (ego_accel.linear.x*dt*dt)/2 + candidate_velocity.linear.x * dt; // changed from time_dtep 1.0 to dt 0.017 
    ego_future_position.position.y = ego_pose.position.y + (ego_accel.linear.y*dt*dt)/2 + candidate_velocity.linear.y * dt;
    ego_future_position.position.z = ego_pose.position.z;
    pose_msg obstacle_future_pose; 
    
    // goal seeking cost
    float dx = goal_point.x - ego_future_position.position.x;
    float dy = goal_point.y - ego_future_position.position.y;

    float dist_to_goal = std::sqrt((dx * dx) + (dy * dy));
    float goal_seeking_cost = goal_seeking_weight * dist_to_goal; 

    // smoothness cost
    float dx_candidate_vel = candidate_velocity.linear.x - ego_velocity.linear.x;
    float dy_candidate_vel = candidate_velocity.linear.y - ego_velocity.linear.y;
    float dist_candidate_vel = std::sqrt(dx_candidate_vel * dx_candidate_vel) + (dy_candidate_vel * dy_candidate_vel);
    float smoothness_cost = smoothness_weight * dist_candidate_vel;
    
    // Combine costs
    cost =  goal_seeking_cost + smoothness_cost;

    return cost;
}

accel_msg NAO::getBestAccel()
{
    return global_best_accel;
}   

