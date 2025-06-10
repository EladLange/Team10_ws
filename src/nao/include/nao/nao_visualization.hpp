#pragma once

#include "common/settings.hpp"
#include "nao.hpp"

void setNAOMarker(vis_marker& disk_marker, const AccDisk& disk, int id);

void setCandidateNAOMarker(vis_marker &candidate_marker, const accel_msg& candidate_accel);

void setVelocityArrowNAOMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now, const std::string& frame_id,int car_index);