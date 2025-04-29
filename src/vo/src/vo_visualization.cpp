#include "vo_visualization.hpp"
#include "velocity_obstacle.hpp"
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


    if (dist <= r_total)
    {
        std::cout << "there is no way out, just prey" << std::endl;
        auto cone_lines = createConeLines(ego_pos.position, -M_PI / 2, M_PI / 2, extension_length); // // 180 degree cone from -90° to +90° (in radians)
        //std::cout << "Number of points in marker: " << cone_marker.points.size() << std::endl;
        for (const auto& pt : cone_lines) 
        {
            //std::cout << "Point: (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
            cone_marker.points.push_back(car_point);  
            cone_marker.points.push_back(pt);
        }
        //std::cout << "Number of points in marker after push_back: " << cone_marker.points.size() << std::endl;
    }

    else 
    {
        if (vo.checkCollision(ego_pos, obstacle_pos, ego_vel, obstacle_vel, r_total))
        {
            float alpha = vo.getAngle(ego_pos, obstacle_pos);
            float theta = vo.getTheta(dist, r_total);

            float left_angle = alpha + theta;
            float right_angle = alpha - theta;


            auto cone_lines = createConeLines(car_point, left_angle, right_angle, extension_length, num_segments);
            std::cout << "Collision detected, finding the best velocity" << std::endl;
            //std::cout << "Number of points in marker: " << cone_marker.points.size() << std::endl;
            for (const auto& pt : cone_lines) 
            {
                //std::cout << "Point: (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
                cone_marker.points.push_back(car_point);  
                cone_marker.points.push_back(pt);
            }
            //std::cout << "Number of points in marker after push_back: " << cone_marker.points.size() << std::endl;
        } 
    }
}