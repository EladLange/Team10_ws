#pragma once
 
#include "road.hpp"
#include "common/settings.hpp"

void setRoadMarker(vis_marker& road_marker, const Road& road, rclcpp::Time now);
void setLaneMarker(vis_marker& lane_marker, const Road& road, int lane_index, rclcpp::Time now);