#pragma once

#include "settings.hpp"
#include "car.hpp"

// function to set a velocity marker for a car
void setVelocityArrowMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now,int car_index = 0);

// function to create a text marker for a car's velocity
void setVelocityTextMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now);