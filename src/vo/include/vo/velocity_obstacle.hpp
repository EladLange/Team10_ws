#pragma once

#include "common/car.hpp"
#include <vector>
#include "common/settings.hpp"

extern float r_r;
extern float r_o;
extern float r_total;
extern float time_horizon;
extern float max_acceleration;
extern float time_step;


class VelocityObstacle
{
    private:
    // calculating beta: angle of v_relative
    float getBeta(const twist_msg& v_relative);

    //normalize angle difference
    float normalizeAngle(float angle);

    // Compute the VO-based cost for a velocity sample
    float calculateCandidateCost(const pose_msg& ego_pose, const twist_msg& ego_vel, const std::vector<Car>& obstacles, const twist_msg& candidate_velocity, const point_msg& goal_point);    

    public:
    // Constructor
    VelocityObstacle();

    // calculate the angle between two points    
    float distance(const pose_msg& s1, const pose_msg& s2);

    // calculating angle between 2 points and x axis
    float getAngle(const pose_msg& s1, const pose_msg& s2);

    // calculation theta: half angle of the cone 
    float getTheta(float d, float r_total);

    // calculating v_relative
    twist_msg getVrelative(const twist_msg& v_ego, const twist_msg& v_obstacle);

    //Builds the 9 possible velocities the ego car could apply
    std::vector<twist_msg> generateCandidateVelocities(const twist_msg& ego_vel);

    //implementation of the velocity obstacle
    bool checkCollision(const pose_msg& ego_pose, const twist_msg& ego_vel, const Car& obstacle, float r_total);

    // Select best velocity sample by evaluating all samples over all obstacles
    twist_msg selectBestVelocity(const pose_msg& ego_pose, const twist_msg& ego_vel, const std::vector<Car>& obstacles, const point_msg& foal_point, float r_total);

};