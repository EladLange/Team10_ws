#include "raceline_visualization.hpp"
#include <iostream>

std::vector<pose_msg> setRaceline()
{
    std::vector<pose_msg> raceline;
    // create a raceline with 100 straight points
    for (int i = 0; i < 100; ++i) 
    {
        pose_msg point;
        point.position.x = 10.0 + i * 1.0;
        point.position.y = 0.0;
        point.position.z = 0.2; 
        raceline.push_back(point);
    }
    return raceline;
}

void visualizeRaceline(const std::vector<pose_msg>& raceline, vis_marker_arr& marker_array, const rclcpp::Time& now)
{
    vis_marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "raceline";
    marker.id = 0;
    marker.type = vis_marker::LINE_STRIP;
    marker.action = vis_marker::ADD;
    marker.scale.x = 0.1; // line width
    marker.color.r = 1.0f; 
    marker.color.g = 0.0f; 
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Add points to the marker
    marker.points.clear(); // Clear previous points
    for (const auto& point : raceline) {
        geometry_msgs::msg::Point p;
        p.x = point.position.x;
        p.y = point.position.y;
        p.z = point.position.z;
        marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
}
