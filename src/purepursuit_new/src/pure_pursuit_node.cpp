// ============================
// src/pure_pursuit_node.cpp
// ============================
// Main Pure Pursuit ROS2 node with modular vehicle model and full visualization

// ========== INCLUDE SECTION ==========
// Include necessary ROS2 and message headers
#include "rclcpp/rclcpp.hpp"  // Core ROS2 client library
#include "geometry_msgs/msg/pose_stamped.hpp"  // Message type for robot pose
#include "geometry_msgs/msg/pose_array.hpp"  // Message type for robot pose
#include <geometry_msgs/msg/twist_stamped.hpp> // Message type for velocity commands
#include "nav_msgs/msg/path.hpp"  // Message type for paths
#include "visualization_msgs/msg/marker.hpp"  // Message type for RViz visualization markers
#include "visualization_msgs/msg/marker_array.hpp"  // Message type for RViz visualization marker arrays
#include "tf2/LinearMath/Quaternion.h"  // For quaternion math (yaw to quaternion)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Conversion between TF2 and geometry_msgs
#include "tf2_ros/transform_broadcaster.h"  // For publishing dynamic transforms

// Include custom headers for vehicle state and model
#include "types.hpp"  // Defines struct State (x, y, yaw)
#include "vehicle_model_base.hpp"  // Base interface for vehicle models
#include "bicycle_model.hpp"  // Concrete bicycle model implementation
#include "ackermann_model.hpp"  // Concrete ackermann model implementation
#include "drone.hpp"  // Custom drone class for multi-drone control

// Standard library includes
#include <vector>  // For storing path points
#include <memory>  // For smart pointers
#include <cmath>  // For math operations (e.g. atan2, hypot)
#include <fstream>  // For reading path from CSV
#include <sstream>  // For parsing CSV strings
#include <algorithm>  // For clamp function
#include <string>  // For string operations
#include <array>  // For fixed-size arrays

using namespace std::chrono_literals;  // Allow writing 10ms, 1s etc. as time literals

// ========== NODE CLASS DEFINITION ==========
class PurePursuitNode : public rclcpp::Node {
public:
    // Constructor – Initializes publishers, timer, paths and drones
    PurePursuitNode() : Node("pure_pursuit_node") {
        // Create a shared bicycle kinematic model with wheelbase 1.55 meters
        vehicle_model_ = std::make_shared<BicycleModel>(1.55);

        // Define paths to load
        std::array<std::string, 3> path_files = {
            "/home/yonatan/Desktop/Team10_ws/src/purepursuit_new/src/drones_path/clothoid_inner_lane.csv",
            "/home/yonatan/Desktop/Team10_ws/src/purepursuit_new/src/drones_path/clothoid_center_lane.csv",
            "/home/yonatan/Desktop/Team10_ws/src/purepursuit_new/src/drones_path/clothoid_outer_lane.csv"
        };

        // Load all paths
        for (const auto& path_file : path_files) {
            auto path = loadPathFromCSV(path_file);
            if (!path.first.empty() && !path.second.empty()) {
                paths_.push_back(path);
             //   RCLCPP_INFO(this->get_logger(), "Loaded path from %s with %zu points",
               //            path_file.c_str(), path.first.size());
            } else {
               // RCLCPP_WARN(this->get_logger(), "Failed to load path from %s", path_file.c_str());
            }
        }

        // Check if at least one path was successfully loaded
        if (paths_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No paths were loaded! Shutting down node.");
            rclcpp::shutdown();
            return;
        }

        // Create publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        drone_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("drone_pose", 10); // Changed to "drone_pose" to match VO package
        drone_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("drone_paths", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("vel_cmd", 10);
        vehicle_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vehicle_markers", 10);

        // Create a broadcaster to publish transforms for visualization
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create drones (one for each path or up to drones_per_path)
        const int drones_per_path = 3; // number of drones per path
        const int num_drones = std::min(static_cast<int>(paths_.size()), drones_per_path);

        for (int i = 0; i < num_drones; ++i) {
            // Create initial state for the drone
            // Position each drone at the start of its path with some z-offset to avoid collisions
            for (int j=0; j<drones_per_path; ++j){

                State initial_state("drone_" + std::to_string(i),
                               paths_[i].first[0]+10*j,  // x (staggered start positions)
                               paths_[i].second[0], // y (start at the same y position)
                               0.2,       // z (staggered heights)
                               0.0);                // yaw
            // Create the drone with its assigned path
                auto drone = std::make_shared<Drone>(
                    "drone_" + std::to_string(i),
                    vehicle_model_,
                    paths_[i].first,   // x coordinates
                    paths_[i].second,  // y coordinates
                    initial_state
                );
        
                drones_.push_back(drone);
                RCLCPP_INFO(this->get_logger(), "Created drone %d at position (%f, %f, %f)",
                        i, initial_state.x, initial_state.y, initial_state.z);
            }
        }

        // Create a periodic timer that triggers control loop every 10 milliseconds
        timer_ = this->create_wall_timer(10ms, std::bind(&PurePursuitNode::onTimer, this));
    }

private:
    // ========== PRIVATE MEMBER VARIABLES ==========
    std::shared_ptr<VehicleModelBase> vehicle_model_;  // Shared vehicle model for all drones
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;  // Publisher for trajectory visualization
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr drone_poses_pub_;  // Publisher for all drone poses
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr drone_paths_pub_;  // Publisher for all drone paths
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;  // Publisher for velocity commands
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_markers_pub_;  // Publisher for vehicle markers
    rclcpp::TimerBase::SharedPtr timer_;  // Timer object for periodic updates
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // Transform broadcaster for TF visualization

    // Vector of paths, each path is a pair of vectors (x coordinates, y coordinates)
    std::vector<std::pair<std::vector<double>, std::vector<double>>> paths_;

    // Vector of drone objects
    std::vector<std::shared_ptr<Drone>> drones_;


    // ========== LOAD PATH FROM CSV ==========
    // Reads x,y values from a CSV file and returns them as a pair of vectors
    std::pair<std::vector<double>, std::vector<double>> loadPathFromCSV(const std::string &filename) {
        std::vector<double> path_x, path_y;
        std::ifstream file(filename);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", filename.c_str());
            return {path_x, path_y}; // Return empty vectors
        }

        std::string line;
        std::getline(file, line);  // Skip the header line

        while (std::getline(file, line)) {
            std::stringstream ss(line); // Create string stream for parsing line
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');   // Extract x string
            std::getline(ss, y_str, ',');   // Extract y string

            try {
                path_x.push_back(std::stod(x_str));    // Convert x to double
                path_y.push_back(std::stod(y_str));    // Convert y to double
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error parsing line in %s: %s", filename.c_str(), line.c_str());
            }
        }

        return {path_x, path_y};
    }

    // ========== VELOCITY TO TWIST ==========
    // Converts a velocity to a twist message based on the drone's orientation (yaw)
    geometry_msgs::msg::TwistStamped velocityToTwist(double velocity, double yaw,const std_msgs::msg::Header & header)
    {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header = header;          // keep your timestamp/frame
      cmd.twist.linear.x  = velocity * std::cos(yaw);
      cmd.twist.linear.y  = velocity * std::sin(yaw);
      cmd.twist.linear.z  = 0.0;
      cmd.twist.angular.x = 0.0;
      cmd.twist.angular.y = 0.0;
      cmd.twist.angular.z = 0.0;
      return cmd;
    }
    // ========== TIMER CALLBACK ==========
    // Called every 10ms: updates all drone states and publishes visualization
    void onTimer() {
        const double velocity = 5.0; // m/sec
        const double dt = 0.01;      // sec

        // Create pose array for all drones
        geometry_msgs::msg::PoseArray drone_poses;
        drone_poses.header.stamp = now();
        drone_poses.header.frame_id = "map";

        // Create marker array for drone paths
        visualization_msgs::msg::MarkerArray path_markers;

        // Create marker array for drone vehicles
        visualization_msgs::msg::MarkerArray vehicle_markers;

        // Update each drone and collect visualization data
        for (size_t i = 0; i < drones_.size(); ++i) {
            // Update drone state using pure pursuit control
            double new_velocity = 5.0 + i; // velocity for each drone
            drones_[i]->update(dt, new_velocity); // Update drone state

            // Get current drone state
            const State& state = drones_[i]->getState();

            // now publish the twist for this drone:
            auto twist = velocityToTwist(new_velocity, state.yaw, drone_poses.header);
            cmd_vel_pub_->publish(twist); // publish the twist

            // Create pose for this drone
            geometry_msgs::msg::Pose drone_pose;
            drone_pose.position.x = state.x;
            drone_pose.position.y = state.y;
            drone_pose.position.z = state.z;

            // Convert yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, state.yaw);
            drone_pose.orientation = tf2::toMsg(q);

            // Add to pose array
            drone_poses.poses.push_back(drone_pose);

            // Create vehicle marker for this drone
            visualization_msgs::msg::Marker vehicle_marker;
            vehicle_marker.header.frame_id = "map";
            vehicle_marker.header.stamp = now();
            vehicle_marker.ns = "drones";
            vehicle_marker.id = i;
            vehicle_marker.type = visualization_msgs::msg::Marker::CUBE;
            vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
            vehicle_marker.pose = drone_pose;
            vehicle_marker.scale.x = 1.55;  // Length
            vehicle_marker.scale.y = 1.0;  // Width
            vehicle_marker.scale.z = 0.5;  // Height

            // Set color based on drone index (different color for each drone)
            switch (i % 9) {
                case 0:
                    vehicle_marker.color.r = 1.0f;
                    vehicle_marker.color.g = 0.0f;
                    vehicle_marker.color.b = 0.0f;
                    break;
                case 1:
                    vehicle_marker.color.r = 1.0f;
                    vehicle_marker.color.g = 0.0f;
                    vehicle_marker.color.b = 0.0f;
                break;
                case 2:
                    vehicle_marker.color.r = 1.0f;
                    vehicle_marker.color.g = 0.0f;
                    vehicle_marker.color.b = 0.0f;
                break;
                case 3:
                    vehicle_marker.color.r = 0.0f;
                    vehicle_marker.color.g = 1.0f;
                    vehicle_marker.color.b = 0.0f;
                    break;
                case 4:
                    vehicle_marker.color.r = 0.0f;
                    vehicle_marker.color.g = 1.0f;
                    vehicle_marker.color.b = 0.0f;
                    break;
                case 5:
                    vehicle_marker.color.r = 0.0f;
                    vehicle_marker.color.g = 1.0f;
                    vehicle_marker.color.b = 0.0f;
                    break;
                case 6:
                    vehicle_marker.color.r = 0.0f;
                    vehicle_marker.color.g = 0.0f;
                    vehicle_marker.color.b = 1.0f;
                    break;
                case 7:
                    vehicle_marker.color.r = 0.0f;
                    vehicle_marker.color.g = 0.0f;
                    vehicle_marker.color.b = 1.0f;
                    break;
                case 8:
                    vehicle_marker.color.r = 0.0f;
                    vehicle_marker.color.g = 0.0f;
                    vehicle_marker.color.b = 1.0f;
                    break;
            }
            vehicle_marker.color.a = 0.8f;
            vehicle_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

            // Add to vehicle markers
            vehicle_markers.markers.push_back(vehicle_marker);

            // Create path marker for this drone's assigned path
            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = "map";
            path_marker.header.stamp = now();
            path_marker.ns = "paths";
            path_marker.id = i;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;

            // Get the path for this drone
            auto [path_x, path_y] = drones_[i]->getPath();

            // Add all path points
            for (size_t j = 0; j < path_x.size(); ++j) {
                geometry_msgs::msg::Point p;
                p.x = path_x[j];
                p.y = path_y[j];
                p.z = state.z;  // Use drone's z-height for path visualization
                path_marker.points.push_back(p);
            }

            // Set path appearance
            path_marker.scale.x = 0.1;  // Line width

            // Match path color to drone color
            path_marker.color = vehicle_marker.color;
            path_marker.color.a = 0.5f;  // More transparent than the drone

            // Add to path markers
            path_markers.markers.push_back(path_marker);

            // Broadcast transform for this drone
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now();
            tf_msg.header.frame_id = "map";
            tf_msg.child_frame_id = "drone_" + std::to_string(i);
            tf_msg.transform.translation.x = state.x;
            tf_msg.transform.translation.y = state.y;
            tf_msg.transform.translation.z = state.z;
            tf_msg.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(tf_msg);
        }

        // Publish all visualization messages
        drone_poses_pub_->publish(drone_poses);
        vehicle_markers_pub_->publish(vehicle_markers);
        drone_paths_pub_->publish(path_markers);
    }
};

// ========== MAIN FUNCTION ==========
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2 communication
    rclcpp::spin(std::make_shared<PurePursuitNode>());  // Create and run the node
    rclcpp::shutdown();  // Shutdown cleanly
    return 0;
}
