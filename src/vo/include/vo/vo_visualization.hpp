#pragma once

#include "common/settings.hpp"
#include "common/car.hpp"

// function to create a visualization marker for the velocity obstacle cone
void setVOConeMarker(vis_marker &cone_marker, const pose_msg& ego_pose, const Car& obstacle, float r_total);

// function to create a visualization marker to the candidate for debugging
void setCandidateMarker(vis_marker &candidate_marker, const twist_msg& candidate_velocity);
