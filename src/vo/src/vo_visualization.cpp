#include "vo_visualization.hpp"

void setVOConeMarker(geometry_msgs::msg::Point& ego_pos, geometry_msgs::msg::Point& obstacle_pos, double r_total)
{
   vis_marker cone_marker;
   cone_marker.header.frame_id = "map";
   cone_marker.header.stamp = rclcpp::Clock().now();
   cone_marker.ns = "vo_cone";
   cone_marker.id = 0;
   cone_marker.type = vis_marker::LINE_LIST;
   cone_marker.action = vis_marker::ADD;
   cone_marker.scale.x = 0.05; // line width

   // yellow cone
   cone_marker.color.r = 0.8;
   cone_marker.color.g = 0.86;
   cone_marker.color.b = 0.22;
   cone_marker.color.a = 1.0;

    // Compute relative position
    double dx = obstacle_pos.x - ego_pos.x;
    double dy = obstacle_pos.y - ego_pos.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= r_total)
    {
        // create 180 degree cone
    }

    else
    {
        double angle_center = std::atan2(dy, dx);
    double angle_offset = std::asin(combined_radius / dist);

    double left_angle = angle_center + angle_offset;
    double right_angle = angle_center - angle_offset;

    // Define how far to extend the lines (visualization only)
    double extension_length = 5.0; // meters per side

    // Left boundary
    geometry_msgs::msg::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;

    geometry_msgs::msg::Point left_boundary;
    left_boundary.x = extension_length * std::cos(left_angle);
    left_boundary.y = extension_length * std::sin(left_angle);
    left_boundary.z = 0.0;

    // Right boundary
    geometry_msgs::msg::Point right_boundary;
    right_boundary.x = extension_length * std::cos(right_angle);
    right_boundary.y = extension_length * std::sin(right_angle);
    right_boundary.z = 0.0;

    // Add lines from origin to boundaries
    cone_marker.points.push_back(origin);
    cone_marker.points.push_back(left_boundary);

    cone_marker.points.push_back(origin);
    cone_marker.points.push_back(right_boundary);
    }   
}