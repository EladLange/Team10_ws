#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "visualization_msgs/msg/marker_array.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using pose_msg = geometry_msgs::msg::Pose;
using twist_msg = geometry_msgs::msg::Twist;

using vis_marker = visualization_msgs::msg::Marker;
using vis_marker_arr = visualization_msgs::msg::MarkerArray;

using point_msg = geometry_msgs::msg::Point;
