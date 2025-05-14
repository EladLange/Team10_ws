#pragma once

#include "settings.hpp"

//function to create a raceline 
std::vector<point_msg> setRaceline();

//function to visualize the raceline in RVIZ
void visualizeRaceline(const std::vector<point_msg>& raceline, vis_marker_arr& marker_array, const rclcpp::Time& now);