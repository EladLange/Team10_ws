#pragma once

#include "car.hpp"
#include <vector>

using pose = geometry_msgs::msg::Pose;
using velocity = geometry_msgs::msg::Twist;

extern float r_r;
extern float r_o;
extern float r_total;
extern float time_horizon;
extern float max_acceleration;
extern float time_step;

class VelocityObstacle
{
    private:
    // calculate the angle between two points    
    float distance(const pose& s1, const pose& s2);

    // calculation theta: half angle of the cone 
    float getTheta(float d);

    // calculating angle between 2 points and x axis
    float getAngle(const pose& s1, const pose& s2);

    // calculating beta: angle of v_relative
    float getBeta(const velocity& v_relative);

    // calculating v_relative
    velocity getVrelative(const velocity& v_ego, const velocity& v_obstacle);

    //normalize angle difference
    float normalizeAngle(float angle);

    //Builds the 9 possible velocities the ego car could apply
    std::vector<velocity> generateCandidateVelocities(const velocity& ego_vel);

    // Compute the VO-based cost for a velocity sample
    float calculateCollisionCost(const pose& ego_pose, const velocity& ego_velocity, const std::vector<pose>obstacle_poses,const std::vector<velocity>obstacle_vels, const velocity& candidate_velocity, const pose& goal_point);    
    
    // find the next point ont the trajectory
    pose findNextGoalPoint(const std::vector<pose>& raceline, const pose& ego_pose);

    public:
    // Constructor
    VelocityObstacle();

    //implementation of the velocity obstacle
    bool checkCollision(const pose& ego_pose, const pose& obstacle_pose, const velocity& v_ego, const velocity& v_obstacle);

    // Select best velocity sample by evaluating all samples over all obstacles
    velocity selectBestVelocity(const pose& ego_pose, const velocity& ego_vel, const std::vector<pose>& obstacles_poses, const std::vector<velocity>& obstacle_vels, const std::vector<pose>& raceline);

};