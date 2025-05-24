#include "nlvo/nlvo_visualization.hpp"

// Make nlvo static to avoid multiple definition errors
static NLVO nlvo;

void setNLVOMarker(std::vector<vis_marker> &nlvo_marker, const pose_msg &ego_pos, const twist_msg &ego_vel, const pose_msg &obstacle_pos, const twist_msg &obstacle_vel, float r_total, float time_horizon, int &id_counter)
{
    vis_marker cylinder;
    cylinder.header.frame_id = "map";
    cylinder.header.stamp = rclcpp::Clock().now();
    cylinder.ns = "nlvo_disk";
    cylinder.id = id_counter++;
    cylinder.type = vis_marker::CYLINDER;
    cylinder.action = vis_marker::ADD;

    cylinder.scale.x = r_total * 2.0;
    cylinder.scale.y = r_total * 2.0;
    cylinder.scale.z = 0.01;

    // Yellow disk
    cylinder.color.r = 0.8;
    cylinder.color.g = 0.86;
    cylinder.color.b = 0.22;
    cylinder.color.a = 0.5; // 50% transparent

    // Set the pose of the marker to the car's pose
    for (float t = nlvo.dt; t<= time_horizon; t += nlvo.dt)
    {
        // Obstacle presicted future position
        pose_msg obstacle_future_pos;
        obstacle_future_pos.position.x = obstacle_pos.position.x + obstacle_vel.linear.x * t;
        obstacle_future_pos.position.y = obstacle_pos.position.y + obstacle_vel.linear.x * t;


        // Relative position at time t
        pose_msg relative_pose;
        relative_pose.position.x = obstacle_future_pos.position.x - ego_pos.position.x;
        relative_pose.position.y = obstacle_future_pos.position.y - ego_pos.position.y;
        
        // Center of the NLVO disk in velocity space
        twist_msg center;
        center.linear.x = relative_pose.position.x / t;
        center.linear.y = relative_pose.position.y / t;

        float radius = r_total / t;

        cylinder.pose.position.x = center.linear.x;
        cylinder.pose.position.y = center.linear.y;
        cylinder.pose.position.z = 0.5;
        cylinder.pose.orientation.w = 1.0;

        nlvo_marker.push_back(cylinder);
    }
}