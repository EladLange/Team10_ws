#include "velocity_visualization.hpp"
#include <sstream>

void setVelocityArrowMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now, int car_index)
{
    // Extract position and velocity from the Car object
    double car_pose_x = car.getPose().position.x;
    double car_pose_y = car.getPose().position.y;
    double car_vel_x = car.getVelocity().linear.x;
    double car_vel_y = car.getVelocity().linear.y;

    // Set Arrow Marker 
    vis_marker arrow_marker;
    arrow_marker.header.frame_id = "map";
    arrow_marker.header.stamp = now;
    arrow_marker.ns = "velocity_arrow";
    arrow_marker.id = car.isControlled() ? 100 : 200 + car_index; // Different ID for controlled and non-controlled cars
    arrow_marker.type = vis_marker::ARROW;
    arrow_marker.action = vis_marker::ADD;
    arrow_marker.scale.x = 0.1; // Shaft diameter
    arrow_marker.scale.y = 0.2; // Head diameter
    arrow_marker.scale.z = 0.2; // Head length

    // Green arrow for velocity
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.0;

    geometry_msgs::msg::Point start_point, end_point;

    // Set the start point to the car's position
    start_point.x = car_pose_x;
    start_point.y = car_pose_y;
    start_point.z = 0.5; // Slightly above ground

    // Set the end point based on the car's velocity
    end_point.x = car_pose_x + car_vel_x * 1.0;
    end_point.y = car_pose_y + car_vel_y * 1.0;
    end_point.z = 0.5; // Same height as start point

    arrow_marker.points.push_back(start_point);
    arrow_marker.points.push_back(end_point);

    marker_array.markers.push_back(arrow_marker);
}

void setVelocityTextMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now)
{
    // Extract position and velocity from the Car object
    double car_pose_x = car.getPose().position.x;
    double car_pose_y = car.getPose().position.y;
    double car_vel_x = car.getVelocity().linear.x;
    double car_vel_y = car.getVelocity().linear.y;

    vis_marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now;
    text_marker.ns = "velocity_text";
    text_marker.id = car.isControlled() ? 101 : 201; // Different ID for controlled and non-controlled cars
    text_marker.type = vis_marker::TEXT_VIEW_FACING;
    text_marker.action = vis_marker::ADD;
    text_marker.scale.z = 0.5; // Text size

    // Set text color to be white
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    // Set the position of the text
    text_marker.pose.position.x = car_pose_x + car_vel_x;
    text_marker.pose.position.y = car_pose_y + car_vel_y;
    text_marker.pose.position.z = 1.0;

    // Compose text
    std::stringstream ss;
    ss.precision(2);
    ss << std::fixed;
    ss << car.getVelocity().linear.x << "," << car.getVelocity().linear.y; 
    text_marker.text = ss.str();
    
    marker_array.markers.push_back(text_marker);
}