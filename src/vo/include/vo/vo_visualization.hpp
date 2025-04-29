#pragma once

#include "settings.hpp"
#include "car.hpp"

void setVOConeMarker(vis_marker &cone_marker, const pose_msg& ego_pos, const pose_msg& obstacle_pose,const twist_msg& ego_vel, const twist_msg& obstacle_vel, float r_total);