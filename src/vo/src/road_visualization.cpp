#include "road_visualization.hpp"
#include "geometry_msgs/msg/point.hpp"

// straight line
void setRoadMarker(vis_marker& road_marker, const Road& road, rclcpp::Time now) {
    // Set Road Rectangle
    road_marker.header.frame_id = "map";
    road_marker.header.stamp = now;
    road_marker.ns = "road";
    road_marker.id = 0;
    road_marker.type = vis_marker::CUBE;
    road_marker.action = vis_marker::ADD;
    
    // Put the center of the cube at the center of the road
    road_marker.pose.position.x = road.getLength() / 2.0;
    road_marker.pose.position.y = 0.0;
    road_marker.pose.position.z = -0.05;
    road_marker.pose.orientation.w = 1.0;
    
    // Set cube's sizes
    road_marker.scale.x = road.getLength();
    road_marker.scale.y = road.getNumLanes() * road.getLaneWidth();
    road_marker.scale.z = 0.1;
    
    // Set cube's color
    road_marker.color.r = 0.2;
    road_marker.color.g = 0.2;
    road_marker.color.b = 0.2;
    road_marker.color.a = 1.0;
}


// oval road
// void setRoadMarker(vis_marker& road_marker, const Road& road, rclcpp::Time now) {
    
//     double radius = road.getRadius();
//     double road_length = road.getLength();
    
//     // Set Road Rectangle
//     road_marker.header.frame_id = "map";
//     road_marker.header.stamp = now;
//     road_marker.ns = "road";
//     road_marker.id = 0;
//     road_marker.type = vis_marker::LINE_STRIP;
//     road_marker.action = vis_marker::ADD;

//     // Set line's sizes
//     road_marker.scale.x = 0.1;
    
//     // Set line's color
//     road_marker.color.r = 0.2;
//     road_marker.color.g = 0.2;
//     road_marker.color.b = 0.2;
//     road_marker.color.a = 1.0;

//     double lane_width = road.getLaneWidth();
//     double total_width = road.getNumLanes() * lane_width;
//     double y_bottom = -total_width / 2.0;
//     double y_top = total_width / 2.0;
//     double x_start = radius;
//     double x_end = radius + road_length;


//     point_msg pt;
    
//     // bottom straight
//     pt.z = 0.01;  // slightly above road to avoid Z-fighting
//     pt.x = x_start;
//     pt.y = y_bottom;
//     road_marker.points.push_back(pt);

//     pt.x = x_end;
//     pt.y = y_bottom;
//     road_marker.points.push_back(pt);

//     // top semi-circle (counterclockwise from bottom to top)
//     for (double angle = 0; angle <= M_PI; angle += M_PI / 100) 
//     {
//         pt.x = radius * cos(angle) + x_start;
//         pt.y = radius * sin(angle) + y_top;
//         road_marker.points.push_back(pt);
//     }

//     // top straight
//     pt.x = x_end;
//     pt.y = y_top;
//     road_marker.points.push_back(pt);

//     pt.x = x_start;
//     pt.y = y_top;
//     road_marker.points.push_back(pt);

//     // bottom semi-circle (clockwise from top to bottom)
//     for (double angle = M_PI; angle <= 2 * M_PI; angle += M_PI / 100) 
//     {
//         pt.x = radius * cos(angle) + x_start;
//         pt.y = radius * sin(angle) + y_bottom;
//         road_marker.points.push_back(pt);
//     }
// }

void setLaneMarker(vis_marker& lane_marker, const Road& road, int lane_index, rclcpp::Time now) {
    // Set Lane Line
    lane_marker.header.frame_id = "map";
    lane_marker.header.stamp = now;
    lane_marker.ns = "lane";
    lane_marker.id = lane_index;
    lane_marker.type = vis_marker::LINE_STRIP;
    lane_marker.action = vis_marker::ADD;

    // Set line's sizes
    lane_marker.scale.x = 0.1;
    lane_marker.scale.y = 0.1;

    // Set line's color
    lane_marker.color.r = 0.96;
    lane_marker.color.g = 0.96;
    lane_marker.color.b = 0.96;
    lane_marker.color.a = 1.0;

    // Set line's start and end points
    float y_shift = -1.5 * road.getLaneWidth();
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0;
    p1.y = lane_index * road.getLaneWidth() + y_shift;
    p1.z = 0.01;  // slightly above road to avoid Z-fighting
    p2.x = road.getLength();
    p2.y = lane_index * road.getLaneWidth() + y_shift;
    p2.z = 0.01;
    lane_marker.points.push_back(p1);
    lane_marker.points.push_back(p2);
}
