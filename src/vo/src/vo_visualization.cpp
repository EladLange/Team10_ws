#include "vo_visualization.hpp"

void setVOConeMarker(vis_marker_arr& marker_array, const Car& car, const Car& obstacle, rclcpp::Time now, int car_index, float r_total)
{
    // Extract position and velocity from the Car object
    double car_pose_x = car.getPose().position.x;
    double car_pose_y = car.getPose().position.y;
    double obstacle_pose_x = obstacle.getPose().position.x;
    double obstacle_pose_y = obstacle.getPose().position.y;

    // Set Cone Marker 
    vis_marker cone_marker;
    cone_marker.header.frame_id = "map";
    cone_marker.header.stamp = now;
    cone_marker.ns = "vo_cone";
    cone_marker.id = car.isControlled() ? 300 : 400 + car_index; // Different ID for controlled and non-controlled cars
    cone_marker.type = vis_marker::CYLINDER;
    cone_marker.action = vis_marker::ADD;
    cone_marker.scale.x = 0.5; // Base diameter
    cone_marker.scale.y = 0.5; // Base diameter
    cone_marker.scale.z = 1.0; // Height

    // Set the position of the cone marker to the midpoint between the two cars
    geometry_msgs::msg::Point start_point, end_point;
    
    start_point.x = (car_pose_x + obstacle_pose_x) / 2;

}