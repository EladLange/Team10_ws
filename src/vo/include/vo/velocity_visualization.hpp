#pragma once

#include "common/settings.hpp"
#include "common/car.hpp"

// function to set a velocity marker for a car
void setVelocityArrowMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now,const std::string& frame_id, int car_index = 0);

// function to create a text marker for a car's velocity
void setVelocityTextMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now);