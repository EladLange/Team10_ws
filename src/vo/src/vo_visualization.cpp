#include "vo_visualization.hpp"
#include "velocity_obstacle.hpp"
#include "global_variables.hpp"
#include <iostream>



// Help function ro create a cone between 2 angles
std::vector<point_msg> createConeLines(const point_msg& apex_point, float start_angle, float end_angle, float extension_length, int num_segments = 10)
{
    std::vector<point_msg> points;
    for (int i = 0; i <= num_segments; ++i)
    {
        float angle = start_angle + i * (end_angle - start_angle) / num_segments;
        point_msg boundary_point;
        boundary_point.x = apex_point.x + extension_length * std::cos(angle);
        boundary_point.y = apex_point.y + extension_length * std::sin(angle);
        boundary_point.z = apex_point.z;

        // Each line is from the car position to the boundary point
        points.push_back(boundary_point);
    }
    return points;
}

void setVOConeMarker(vis_marker &cone_marker, const pose_msg& ego_pos, const pose_msg& obstacle_pos,const twist_msg& ego_vel, const twist_msg& obstacle_vel, float r_total)
{
    VelocityObstacle vo;

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

    float dist= vo.distance(ego_pos, obstacle_pos);
    float extension_length = 15.0; // Length of the cone lines
    point_msg car_point;
    car_point.x = ego_pos.position.x;
    car_point.y = ego_pos.position.y;
    car_point.z = ego_pos.position.z;

    // Number of segments inside the cone (higher = smoother)
    int num_segments = 200;
    
    
    // Compute shifted apex = car_point + v_relative
    twist_msg relative_vector = vo.getVrelative(ego_vel, obstacle_vel);
    point_msg shifted_apex;
    shifted_apex.x = car_point.x + relative_vector.linear.x;
    shifted_apex.y = car_point.y + relative_vector.linear.y;
    shifted_apex.z = car_point.z;

    if (dist <= r_total)
    {
        //std::cout << "there is no way out, just prey" << std::endl;
        
        auto cone_lines = createConeLines(shifted_apex, -M_PI / 2, M_PI / 2, extension_length, num_segments); // // 180 degree cone from -90° to +90° (in radians)
        //std::cout << "Number of points in marker: " << cone_marker.points.size() << std::endl;
        for (const auto& pt : cone_lines) 
        {
            //std::cout << "Point: (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
            cone_marker.points.push_back(shifted_apex);  
            cone_marker.points.push_back(pt);
        }
        //std::cout << "Number of points in marker after push_back: " << cone_marker.points.size() << std::endl;
    }

    else 
    {
        if (vo.checkCollision(ego_pos, obstacle_pos, ego_vel, obstacle_vel, r_total) || vo.distance(ego_pos, obstacle_pos) < 6.0)   
        {
            float alpha = vo.getAngle(ego_pos, obstacle_pos);
            float theta = vo.getTheta(dist, r_total);

            float left_angle = alpha + theta;
            float right_angle = alpha - theta;

            // add relative vector
            auto cone_lines = createConeLines(shifted_apex, left_angle, right_angle, extension_length, num_segments);
            //std::cout << "Collision detected, finding the best velocity" << std::endl;
            //std::cout << "Number of points in marker: " << cone_marker.points.size() << std::endl;
            for (const auto& pt : cone_lines) 
            {
                //std::cout << "Point: (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
                cone_marker.points.push_back(shifted_apex);  
                cone_marker.points.push_back(pt);
            }
            //std::cout << "Number of points in marker after push_back: " << cone_marker.points.size() << std::endl;
        } 
    }
}

void setCandidateMarker(vis_marker &candidate_marker, const pose_msg& ego_pos, const twist_msg& candidate_velocity, float r_total, float dt)
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
    candidate_end_point.x = candidate_velocity.linear.x;// * dt;
    candidate_end_point.y = candidate_velocity.linear.y;// * dt;
    candidate_end_point.z = car_point.z;

    candidate_marker.points.push_back(car_point);
    candidate_marker.points.push_back(candidate_end_point);
}