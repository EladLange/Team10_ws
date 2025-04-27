#pragma once

#include "settings.hpp"
#include "car.hpp"

void setVOConeMarker(vis_marker_arr& marker_array, const Car& car, const Car& obstacle, rclcpp::Time now, int car_index = 0, float r_total);