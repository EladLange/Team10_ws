#include "raceline_visualization.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

std::vector<point_msg> setRaceline()
{
    std::vector<point_msg> raceline;
    // // create a raceline with 100 straight points
    // for (int i = 0; i < 300; ++i) 
    // {
    //     point_msg point;
    //     point.x = 10.0 + i * 1.0;
    //     point.y = 0.0;
    //     point.z = 0.2; 
    //     raceline.push_back(point);
    // }

    std::string filename = "/home/yonatan/Desktop/Team10_ws/src/race_line/racing_line_new.csv"; // change to raceline path
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening raceline file." << std::endl;
        return raceline;
    }
    std::string line;   // Skip the first line (header)
    std::getline(file , line);
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        point_msg point;
        std::string token;
        // Read x, y, z values from the line
        std::getline(iss, token, ',');
        point.x = std::stod(token);
        std::getline(iss, token, ',');
        point.y = std::stod(token);
        std::getline(iss, token, ',');
        point.z = 0.2;
        raceline.push_back(point);
    }
    file.close();

    return raceline;
}

void visualizeRaceline(const std::vector<point_msg>& raceline, vis_marker_arr& marker_array, const rclcpp::Time& now)
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
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
}
