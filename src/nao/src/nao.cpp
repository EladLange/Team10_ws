#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include "nao/nao.hpp"
#include <geometry_msgs/msg/accel.hpp>

// ============================================================================
// Implementation of the NAO (Nonlinear Acceleration Obstacle) planning class
// This class computes safe acceleration commands for an autonomous agent
// to avoid collisions with dynamic obstacles while progressing towards a goal.
// ============================================================================

// Constructor: initialize any required state or parameters here
NAO::NAO()
{
    // Currently no initialization logic is needed
}

/**
 * @brief Selects the best acceleration command from a set of candidates
 *        by evaluating collision risk and a cost function.
 *
 * @param ego_pose       Current pose of the ego vehicle (position)
 * @param ego_vel        Current velocity of the ego vehicle
 * @param obstacles_poses Vector of poses for each dynamic obstacle
 * @param obstacle_vels  Vector of velocities for each dynamic obstacle
 * @param goal_point     Target goal point in 2D space
 * @param r_total        Combined radius (ego + obstacle) for collision checking
 * @return geometry_msgs::msg::Accel Best acceleration command
 */
geometry_msgs::msg::Accel NAO::selectBestAcceleration(
    const pose_msg &ego_pose,
    const twist_msg &ego_vel,
    const std::vector<pose_msg> &obstacles_poses,
    const std::vector<twist_msg> &obstacle_vels,
    const point_msg &goal_point,
    float r_total)
{
    // --------------------------------------------------------
    // 1. Compute the minimum time horizon until potential collision
    //    across all obstacles, then extend by one time unit as buffer
    // --------------------------------------------------------
    float min_time_horizon = max_time;
    for (size_t i = 0; i < obstacles_poses.size(); ++i)
    {
        // Compute collision time with obstacle i using NAO
        float th = computeMinimumTimeHorizon(
            ego_pose,
            ego_vel,
            obstacles_poses[i],
            obstacle_vels[i],
            r_total
        );
        std::cout << "[NAO] time horizon for obstacle " << i << ": " << th << std::endl;
        // Keep the smallest time horizon (worst-case)
        min_time_horizon = std::min(min_time_horizon, th);
    }
    // Add a safety margin of 1.0 (seconds)
    min_time_horizon += 1.0f;

    // --------------------------------------------------------
    // 2. Generate a discrete set of candidate accelerations
    // --------------------------------------------------------
    auto candidates = generateCandidateAccelerations(ego_vel);

    // --------------------------------------------------------
    // 3. Filter out any candidate that leads to collision within
    //    the truncated NAO set for the computed horizon.
    // --------------------------------------------------------
    std::vector<geometry_msgs::msg::Accel> safe_accels;
    for (const auto &cand : candidates)
    {
        bool is_safe = true;
        for (size_t i = 0; i < obstacles_poses.size(); ++i)
        {
            // If this acceleration causes a collision, discard it
            if (isAccelerationInTruncatedNAO(
                    cand,
                    ego_pose,
                    ego_vel,
                    obstacles_poses[i],
                    obstacle_vels[i],
                    r_total,
                    min_time_horizon))
            {
                is_safe = false;
                break;
            }
        }
        if (is_safe)
        {
            safe_accels.push_back(cand);
        }
    }

    // --------------------------------------------------------
    // 4. If no safe accelerations remain, issue an emergency brake
    // --------------------------------------------------------
    if (safe_accels.empty())
    {
        geometry_msgs::msg::Accel zero_acc;
        zero_acc.linear.x = 0.0;
        zero_acc.linear.y = 0.0;
        zero_acc.linear.z = 0.0;
        safe_accels.push_back(zero_acc);
    }

    // --------------------------------------------------------
    // 5. Evaluate cost for each safe acceleration and pick the best
    //    cost combines obstacle avoidance, goal seeking, and smoothness
    // --------------------------------------------------------
    auto best_accel = safe_accels.front();
    float best_cost = calculateCandidateCost(
        ego_pose,
        ego_vel,
        obstacles_poses,
        obstacle_vels,
        best_accel,
        goal_point,
        r_total,
        min_time_horizon
    );

    for (const auto &cand : safe_accels)
    {
        float cost = calculateCandidateCost(
            ego_pose,
            ego_vel,
            obstacles_poses,
            obstacle_vels,
            cand,
            goal_point,
            r_total,
            min_time_horizon
        );
        if (cost < best_cost)
        {
            best_cost = cost;
            best_accel = cand;
        }
    }

    std::cout << "[NAO] Selected best acceleration: ("
              << best_accel.linear.x << ", "
              << best_accel.linear.y << ")"
              << " with cost " << best_cost << std::endl;

    return best_accel;
}

/**
 * @brief Computes the minimum time until collision under extreme
 *        accelerations (bang-bang control) within the NAO framework.
 *
 * @param ego_pose      Current pose of the ego vehicle
 * @param ego_vel       Current velocity of the ego vehicle
 * @param obstacle_pose Pose of a single obstacle
 * @param obstacle_vel  Velocity of that obstacle
 * @param r_total       Combined radius threshold for collision
 * @return float Minimum collision time across extreme accel scenarios
 */
float NAO::computeMinimumTimeHorizon(
    const pose_msg &ego_pose,
    const twist_msg &ego_vel,
    const pose_msg &obstacle_pose,
    const twist_msg &obstacle_vel,
    float r_total)
{
    // Prepare a set of corner-case accelerations in x/y directions
    std::vector<std::pair<double,double>> accel_set = {
        {control_limit_ax,  control_limit_ay},
        {-control_limit_ax, control_limit_ay},
        {control_limit_ax, -control_limit_ay},
        {-control_limit_ax,-control_limit_ay},
    };

    float min_collision_time = max_time;
    float r2 = r_total * r_total;  // squared collision radius

    for (auto &a : accel_set)
    {
        float first_col_time = max_time;

        // Estimate future ego velocity after one dt step under accel a
        twist_msg ego_fut_vel;
        ego_fut_vel.linear.x = ego_vel.linear.x + a.first * dt;
        ego_fut_vel.linear.y = ego_vel.linear.y + a.second * dt;

        // Compute relative velocity between ego and obstacle
        twist_msg rel_vel;
        rel_vel.linear.x = ego_fut_vel.linear.x - obstacle_vel.linear.x;
        rel_vel.linear.y = ego_fut_vel.linear.y - obstacle_vel.linear.y;

        // Compute initial relative position
        float rel_x0 = ego_pose.position.x - obstacle_pose.position.x;
        float rel_y0 = ego_pose.position.y - obstacle_pose.position.y;

        // Simulate motion under constant rel_vel and accel a
        for (float t = dt; t < max_time; t += dt)
        {
            // Relative position under p0 + v*t + 0.5*a*t^2
            float x = rel_x0 + rel_vel.linear.x * t + 0.5 * a.first * t * t;
            float y = rel_y0 + rel_vel.linear.y * t + 0.5 * a.second * t * t;
            float dist2 = x*x + y*y;
            if (dist2 <= r2)
            {
                first_col_time = t;
                break;  // stop at first collision
            }
        }
        // Take the minimum over all accel extremes
        min_collision_time = std::min(min_collision_time, first_col_time);
    }

    return min_collision_time;
}

/**
 * @brief Generates a discrete set of acceleration candidates evenly
 *        distributed in angle and magnitude up to max_acceleration.
 *
 * @param ego_vel   Current ego velocity (unused here but could
 *                  be used for velocity-dependent sampling).
 * @return Vector of candidate accel commands
 */
std::vector<geometry_msgs::msg::Accel> NAO::generateCandidateAccelerations(
    const twist_msg & /*ego_vel*/)
{
    std::vector<geometry_msgs::msg::Accel> accels;
    int num_angles = 50;      // angular resolution
    int num_accels = 7;       // radial resolution
    float angle_step = 2 * M_PI / num_angles;
    float accel_step = max_acceleration / num_accels;

    for (float a = 0.0f; a <= max_acceleration; a += accel_step)
    {
        for (float ang = 0.0f; ang < 2 * M_PI; ang += angle_step)
        {
            geometry_msgs::msg::Accel cand;
            // Convert polar accel to rectangular components
            cand.linear.x = a * std::cos(ang);
            cand.linear.y = a * std::sin(ang);
            cand.linear.z = 0.0f;
            accels.push_back(cand);
        }
    }
    return accels;
}

/**
 * @brief Checks if a given acceleration is inside the truncated NAO set,
 *        i.e., leads to collision within the time_horizon.
 *
 * @param candidate_accel  The accel to test
 * @param ego_pose         Current ego pose
 * @param ego_vel          Current ego velocity
 * @param obstacle_pose    Pose of the obstacle
 * @param obstacle_vel     Velocity of the obstacle
 * @param r_total          Combined collision radius
 * @param time_horizon     Time horizon for simulation
 * @return true if collision occurs (unsafe)
 */
bool NAO::isAccelerationInTruncatedNAO(
    const geometry_msgs::msg::Accel &candidate_accel,
    const pose_msg &ego_pose,
    const twist_msg &ego_vel,
    const pose_msg &obstacle_pose,
    const twist_msg &obstacle_vel,
    float r_total,
    float time_horizon)
{
    float r2 = r_total * r_total;
    // Simulate relative motion under constant accel
    for (float t = dt; t < time_horizon; t += dt)
    {
        // Obstacle future position: p_obs + v_obs * t
        float obs_x = obstacle_pose.position.x + obstacle_vel.linear.x * t;
        float obs_y = obstacle_pose.position.y + obstacle_vel.linear.y * t;

        // Ego future position: p_ego + v_ego*t + 0.5*a*t^2
        float ego_x = ego_pose.position.x + ego_vel.linear.x * t + 0.5f * candidate_accel.linear.x * t * t;
        float ego_y = ego_pose.position.y + ego_vel.linear.y * t + 0.5f * candidate_accel.linear.y * t * t;

        float dx = ego_x - obs_x;
        float dy = ego_y - obs_y;
        if (dx*dx + dy*dy <= r2)
        {
            return true;  // collision detected
        }
    }
    return false;  // no collision within horizon
}

/**
 * @brief Computes a weighted cost for a candidate acceleration:
 *        - Obstacle avoidance: inversely proportional to min distance
 *        - Goal seeking: proportional to distance to goal
 *        - Smoothness: penalizes large accel magnitudes
 *
 * @param ego_pose        Current ego pose
 * @param ego_vel         Current ego velocity
 * @param obstacle_poses  Vector of obstacle poses
 * @param obstacle_vels   Vector of obstacle velocities
 * @param candidate_accel Candidate accel
 * @param goal_point      Target goal location
 * @param r_total         (unused here)
 * @param time_horizon    (unused here)
 * @return float Combined cost value
 */
float NAO::calculateCandidateCost(
    const pose_msg &ego_pose,
    const twist_msg &ego_vel,
    const std::vector<pose_msg> &obstacle_poses,
    const std::vector<twist_msg> &obstacle_vels,
    const geometry_msgs::msg::Accel &candidate_accel,
    const point_msg &goal_point,
    float /*r_total*/, float /*time_horizon*/)
{
    float time_step = 1.0f;  // time horizon for cost computation

    // Predict ego future position under candidate accel
    float ego_x = ego_pose.position.x + ego_vel.linear.x * time_step
                  + 0.5f * candidate_accel.linear.x * time_step * time_step;
    float ego_y = ego_pose.position.y + ego_vel.linear.y * time_step
                  + 0.5f * candidate_accel.linear.y * time_step * time_step;

    // 1) Obstacle avoidance cost: inversely to min distance
    float min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < obstacle_poses.size(); ++i)
    {
        float obs_x = obstacle_poses[i].position.x + obstacle_vels[i].linear.x * time_step;
        float obs_y = obstacle_poses[i].position.y + obstacle_vels[i].linear.y * time_step;
        float d = std::hypot(ego_x - obs_x, ego_y - obs_y);
        min_dist = std::min(min_dist, d);
    }
    float avoid_cost = 40.0f * (1.0f / min_dist);

    // 2) Goal seeking cost: proportional to distance to goal
    float goal_dx = goal_point.x - ego_x;
    float goal_dy = goal_point.y - ego_y;
    float goal_cost = 7.0f * std::hypot(goal_dx, goal_dy);

    // 3) Smoothness cost: proportional to accel magnitude
    float accel_mag = std::hypot(candidate_accel.linear.x, candidate_accel.linear.y);
    float smooth_cost = 0.1f * accel_mag;

    // Sum costs for final metric
    return avoid_cost + goal_cost + smooth_cost;
}

/**
 * @brief Auxiliary cost function comparing accel magnitude vs. goal vector.
 *        Not used in primary planner but available for extensions.
 *
 * @param candidate_accel  Candidate accel
 * @param to_goal          Vector from ego to goal
 * @param ego_vel          Current ego velocity (unused)
 * @return float Cost metric
 */
float NAO::evaluateCost(
    const geometry_msgs::msg::Accel &candidate_accel,
    const point_msg &to_goal,
    const twist_msg & /*ego_vel*/)
{
    float a_mag = std::hypot(candidate_accel.linear.x, candidate_accel.linear.y);
    float g_mag = std::hypot(to_goal.x, to_goal.y);
    return std::abs(a_mag - g_mag);
}
