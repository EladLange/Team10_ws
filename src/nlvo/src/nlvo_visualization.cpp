#include "nlvo/nlvo_visualization.hpp"
#include "nlvo/nlvo_visualization.hpp"

// Make nlvo static to avoid multiple definition errors
static NLVO nlvo;

void setNLVODiskMarker(vis_marker &disk_marker, const VelDisk &disk, int id)
{
    disk_marker.header.frame_id = "ego";
    disk_marker.header.stamp = rclcpp::Clock().now();
    disk_marker.ns = "nlvo_disk";
    disk_marker.id = id;
    disk_marker.type = vis_marker::CYLINDER;
    disk_marker.action = vis_marker::ADD;

    // Radius to diameter, z thickness
    disk_marker.scale.x = disk.radius * 2.0;
    disk_marker.scale.y = disk.radius * 2.0;
    disk_marker.scale.z = 0.05;

    // Yellow disk
    disk_marker.color.r = 0.8;
    disk_marker.color.g = 0.86;
    disk_marker.color.b = 0.22;
    disk_marker.color.a = 0.5; // 50% transparent

    // Set the pose of the marker to the disk's center
    disk_marker.pose.position.x = disk.cx;
    disk_marker.pose.position.y = disk.cy;
    disk_marker.pose.position.z = 0.0;
    disk_marker.pose.orientation.w = 1.0;
}