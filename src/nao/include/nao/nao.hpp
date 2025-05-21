// =================================
// include/nao.hpp
// =================================
// This file defines the NAO class, which implements the Nonlinear Acceleration Obstacle (NAO) planning algorithm
// for safe navigation in dynamic environments.

#pragma once

#include "common/settings.hpp"
#include <vector>
#include <geometry_msgs/msg/accel.hpp>

class NAO
{
public:
    // Constructor
    NAO();

    // Global variables (acceleration limits)
    const float control_limit_ax = 2.5f;    // |a_x| <= control_limit_ax
    const float control_limit_ay = 2.5f;    // |a_y| <= control_limit_ay
    const float dt = 0.1f;                  // time step
    const float max_time = 10.0f;           // maximum time horizon
    const float max_acceleration = 5.0f;    // maximum longitudinal acceleration

    /**
     *
     * @brief Select the best acceleration command based on NAO
     *
     * @param ego_pose      Current pose of the ego vehicle
     * @param ego_vel       Current velocity of the ego vehicle
     * @param obstacles_poses  Vector of obstacle poses
     * @param obstacle_vels    Vector of obstacle velocities
     * @param goal_point    Target goal point
     * @param r_total       Combined radius for collision checking
     * @return geometry_msgs::msg::Accel  Best acceleration command
     */
    geometry_msgs::msg::Accel selectBestAcceleration(
        const pose_msg& ego_pose,
        const twist_msg& ego_vel,
        const std::vector<pose_msg>& obstacles_poses,
        const std::vector<twist_msg>& obstacle_vels,
        const point_msg& goal_point,
        float r_total
    );

    /**
     * @brief Generate a set of candidate accelerations
     *
     * @param ego_vel  Current ego vehicle velocity
     * @return std::vector<geometry_msgs::msg::Accel>  List of candidate accelerations
     */
    std::vector<geometry_msgs::msg::Accel> generateCandidateAccelerations(
        const twist_msg& ego_vel
    );

    /**
     * @brief Check if a candidate acceleration lies within the truncated NAO set
     *
     * @param candidate_accel  Candidate acceleration to test
     * @param ego_pose         Ego pose
     * @param ego_vel          Ego velocity
     * @param obstacle_pose    Single obstacle pose
     * @param obstacle_vel     Single obstacle velocity
     * @param r_total          Combined radius for collision checking
     * @param time_horizon     Time horizon to test
     * @return true if inside the NAO region (collision-prone)
     */
    bool isAccelerationInTruncatedNAO(
        const geometry_msgs::msg::Accel& candidate_accel,
        const pose_msg& ego_pose,
        const twist_msg& ego_vel,
        const pose_msg& obstacle_pose,
        const twist_msg& obstacle_vel,
        float r_total,
        float time_horizon
    );

    /**
     * @brief Compute cost for a candidate acceleration
     *
     * @param ego_pose          Ego pose
     * @param ego_vel           Ego velocity
     * @param obstacles_poses   Vector of obstacle poses
     * @param obstacle_vels     Vector of obstacle velocities
     * @param candidate_accel   Candidate acceleration
     * @param goal_point        Target goal point
     * @param r_total           Combined radius
     * @param time_horizon      Time horizon
     * @return float Cost value
     */
    float calculateCandidateCost(
        const pose_msg& ego_pose,
        const twist_msg& ego_vel,
        const std::vector<pose_msg>& obstacles_poses,
        const std::vector<twist_msg>& obstacle_vels,
        const geometry_msgs::msg::Accel& candidate_accel,
        const point_msg& goal_point,
        float r_total,
        float time_horizon
    );

private:
    /**
     * @brief Compute the minimum time horizon until potential collision given NAO
     *
     * @param ego_pose      Ego pose
     * @param ego_vel       Ego velocity
     * @param obstacle_pose Obstacle pose
     * @param obstacle_vel  Obstacle velocity
     * @param r_total       Combined radius
     * @return float Minimum time until collision
     */
    float computeMinimumTimeHorizon(
        const pose_msg& ego_pose,
        const twist_msg& ego_vel,
        const pose_msg& obstacle_pose,
        const twist_msg& obstacle_vel,
        float r_total
    );

    /**
     * @brief Evaluate cost metric for a given acceleration (e.g., distance-to-goal time)
     *
     * @param candidate_accel  Candidate acceleration
     * @param to_goal          Vector from ego to goal
     * @param ego_vel          Ego velocity
     * @return float Cost metric
     */
    float evaluateCost(
        const geometry_msgs::msg::Accel& candidate_accel,
        const point_msg& to_goal,
        const twist_msg& ego_vel
    );
};
