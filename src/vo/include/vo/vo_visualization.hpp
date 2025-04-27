#pragma once

#include "settings.hpp"
#include "car.hpp"

void setVOConeMarker(geometry_msgs::msg::Point& ego_pos, geometry_msgs::msg::Point& obstacle_pos, double r_total);