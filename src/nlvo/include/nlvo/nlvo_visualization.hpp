#pragma once

#include "common/settings.hpp"
#include "nlvo.hpp"

void setNLVOMarker(std::vector<vis_marker> &nlvo_marker, const pose_msg& ego_pos, const twist_msg& ego_vel, const pose_msg& obstacle_pos, 
    const twist_msg& obstacle_vel, float r_total, float time_horizon, int &id_counter);