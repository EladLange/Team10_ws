#pragma once

#include "settings.hpp"
#include "car.hpp"

// function to create a visualization marker for the velocity obstacle cone
void setVOConeMarker(vis_marker &cone_marker, const pose_msg& ego_pos, const pose_msg& obstacle_pose,const twist_msg& ego_vel, const twist_msg& obstacle_vel, float r_total);

// function to create a visualization marker to the candidate for debugging
void setCandidateMarker(vis_marker &candidate_marker, const pose_msg& ego_pos, const twist_msg& candidate_velocity, float r_total, float dt);
