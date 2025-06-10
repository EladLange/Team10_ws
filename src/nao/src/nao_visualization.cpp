#include "nao/nao_visualization.hpp"

void setNAOMarker(vis_marker &disk_marker, const AccDisk &disk, int id)
{
    disk_marker.header.frame_id = "ego";
    disk_marker.header.stamp = rclcpp::Clock().now();
    disk_marker.ns = "nao_disk";
    disk_marker.id = id;
    disk_marker.type = vis_marker::CYLINDER;
    disk_marker.action = vis_marker::ADD;
    disk_marker.lifetime = rclcpp::Duration::from_seconds(0.1); //0.1

    // Radius to diameter, z thickness
    disk_marker.scale.x = disk.radius * 1.0;
    disk_marker.scale.y = disk.radius * 1.0;
    disk_marker.scale.z = 0.05;

    // blue disk
    disk_marker.color.r = 0.678;
    disk_marker.color.g = 0.847;
    disk_marker.color.b = 0.902;
    disk_marker.color.a = 0.95; // 50% transparent

    // Set the pose of the marker to the disk's center
    disk_marker.pose.position.x = disk.cx;
    disk_marker.pose.position.y = disk.cy;
    disk_marker.pose.position.z = 0.0;
    disk_marker.pose.orientation.w = 1.0;
}

void setCandidateNAOMarker(vis_marker &candidate_marker, const accel_msg& candidate_accel)
{
    candidate_marker.header.frame_id = "ego";//debug-should be "map"
    candidate_marker.header.stamp = rclcpp::Clock().now();
    candidate_marker.ns = "candidate";
    candidate_marker.id = 0;
    candidate_marker.type = vis_marker::LINE_LIST;
    candidate_marker.action = vis_marker::ADD;
    candidate_marker.scale.x = 0.05; // line width

    // red cone
    candidate_marker.color.r = 1.0;
    candidate_marker.color.g = 0.44;
    candidate_marker.color.b = 0.0;
    candidate_marker.color.a = 1.0;

    point_msg car_point;
    car_point.x = 0.0;
    car_point.y = 0.0;
    car_point.z = 0.0;

    point_msg candidate_end_point;
    candidate_end_point.x = candidate_accel.linear.x;
    candidate_end_point.y = candidate_accel.linear.y;
    candidate_end_point.z = car_point.z;

    candidate_marker.points.push_back(car_point);
    candidate_marker.points.push_back(candidate_end_point);
}

void setVelocityArrowNAOMarker(vis_marker_arr& marker_array, const Car& car, rclcpp::Time now, const std::string& frame_id,int car_index)
{
    // Extract position and velocity from the Car object
    double car_pose_x = car.getPose().position.x;
    double car_pose_y = car.getPose().position.y;
    double car_accel_x = car.getAcceleration().linear.x;
    double car_accel_y = car.getAcceleration().linear.y;

    // Set Arrow Marker 
    vis_marker arrow_marker;
    arrow_marker.header.frame_id = frame_id;
    arrow_marker.header.stamp = now;
    arrow_marker.ns = "velocity_arrow";
    arrow_marker.id = car.isControlled() ? 100 : 200 + car_index; // Different ID for controlled and non-controlled cars
    arrow_marker.type = vis_marker::ARROW;
    arrow_marker.action = vis_marker::ADD;
    arrow_marker.scale.x = 0.4; // Shaft diameter
    arrow_marker.scale.y = 0.8; // Head diameter
    arrow_marker.scale.z = 0.8; // Head length

    // White arrow for velocity
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.8;
    arrow_marker.color.b = 0.82;
    arrow_marker.color.a = 1.0;

    geometry_msgs::msg::Point start_point, end_point;
    
    if (frame_id == "ego")
    {
        // Set the start point to the origin
        start_point.x = 0.0;
        start_point.y = 0.0;
        start_point.z = 0.5; // Slightly above ground

        // Set the end point based on the car's velocity
        end_point.x = car_accel_x * 1.0;
        end_point.y = car_accel_y * 1.0;
        end_point.z = 0.5; // Same height as start point

        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);
    }
    
    marker_array.markers.push_back(arrow_marker);
}

