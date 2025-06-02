#include "vo_visualization.hpp"
#include "velocity_obstacle.hpp"
#include "global_variables.hpp"
#include <iostream>

void setVOConeMarker(vis_marker &cone_marker, const pose_msg& ego_pose, const Car& obstacle, float r_total)
{
    VelocityObstacle vo;

    cone_marker.header.frame_id = "ego";
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
    cone_marker.color.a = 0.8;

    float dist= vo.distance(ego_pose, obstacle.getPose());
    point_msg start_point;
    start_point.x = obstacle.getVelocity().linear.x;
    start_point.y = obstacle.getVelocity().linear.y;
    start_point.z = 0.5;

    point_msg end_point;
    float alpha = vo.getAngle(ego_pose, obstacle.getPose());
    float theta = vo.getTheta(dist, r_total);
    int n = 100; // Number of segments for the cone
    
    for (int i = 0; i <=n; ++i)
    {
        float angle = alpha - theta + i * (2 * theta) / n; // Angle from the apex to the boundary
        end_point.x = start_point.x + dist * std::cos(angle);
        end_point.y = start_point.y + dist * std::sin(angle);
        end_point.z = 0.5;

        cone_marker.points.push_back(start_point);
        cone_marker.points.push_back(end_point);
    }
    
}

void setCandidateMarker(vis_marker &candidate_marker, const twist_msg& candidate_velocity)
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
    /*
    point_msg car_point;
    car_point.x = ego_pos.position.x;
    car_point.y = ego_pos.position.y;
    car_point.z = ego_pos.position.z;

    point_msg candidate_end_point;
    candidate_end_point.x = car_point.x + candidate_velocity.linear.x * dt;
    candidate_end_point.y = car_point.y + candidate_velocity.linear.y * dt;
    candidate_end_point.z = car_point.z;
    */

    point_msg car_point;
    car_point.x = 0.0;
    car_point.y = 0.0;
    car_point.z = 0.0;

    point_msg candidate_end_point;
    candidate_end_point.x = candidate_velocity.linear.x;
    candidate_end_point.y = candidate_velocity.linear.y;
    candidate_end_point.z = car_point.z;

    candidate_marker.points.push_back(car_point);
    candidate_marker.points.push_back(candidate_end_point);
}