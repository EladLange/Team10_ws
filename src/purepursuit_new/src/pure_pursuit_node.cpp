// ============================
// src/pure_pursuit_node.cpp
// ============================
// Main Pure Pursuit ROS2 node with modular vehicle model and full visualization

#include "rclcpp/rclcpp.hpp"  // ROS2 client library
#include "geometry_msgs/msg/pose_stamped.hpp"  // For publishing robot pose
#include "nav_msgs/msg/path.hpp"  // For publishing path messages
#include "visualization_msgs/msg/marker.hpp"  // For RViz visualization
#include "tf2/LinearMath/Quaternion.h"  // Quaternion support for yaw rotation
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // TF2 geometry conversions
#include "tf2_ros/static_transform_broadcaster.h"  // For broadcasting static transforms

#include "types.hpp"  // Custom type that defines robot state (x, y, yaw)
#include "vehicle_model_base.hpp"  // Abstract base class for vehicle models
#include "bicycle_model.hpp"  // Specific implementation of a bicycle kinematic model

#include <vector>  // Standard vector container
#include <memory>  // For smart pointers
#include <cmath>  // Math functions like atan2, hypot, sqrt
#include <fstream>  // File input
#include <sstream>  // String stream for parsing CSV
#include <algorithm>  // For std::clamp

using namespace std::chrono_literals;  // Enables usage like 10ms

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        // Instantiate vehicle model (bicycle model with 2.5m wheelbase)
        vehicle_model_ = std::make_unique<BicycleModel>(2.5);

        // Load reference path from CSV file
        loadPath("/home/yonatan/Desktop/Team10_ws/src/purepursuit_new/track/yasMarina_path.csv");

        // Shutdown if path is empty
        if (path_x_.empty() || path_y_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path is empty! Shutting down node.");
            rclcpp::shutdown();
            return;
        }

        // Create ROS2 publishers for visualization
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        desired_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("desired_path", 10);
        vehicle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_marker", 10);

        // Broadcast a static transform between map and base_link
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcastStaticTF();

        // Set up timer to call onTimer periodically (every 10ms)
        timer_ = this->create_wall_timer(10ms, std::bind(&PurePursuitNode::onTimer, this));

        // Initialize robot state at the start of the path
        state_ = {path_x_[0], path_y_[0], 0.0};
    }

private:
    // Vehicle model (dynamics)
    std::unique_ptr<VehicleModelBase> vehicle_model_;
    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, desired_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_marker_pub_;
    // Timer for periodic execution
    rclcpp::TimerBase::SharedPtr timer_;
    // Static transform broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    // Path data (x and y coordinates)
    std::vector<double> path_x_, path_y_;
    // Trajectory taken by the robot
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;
    // Current state of the robot
    State state_;

    // Load path from CSV file
    void loadPath(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file");
            return;
        }
        std::string line;
        std::getline(file, line); // Skip header line
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            path_x_.push_back(std::stod(x_str));
            path_y_.push_back(std::stod(y_str));
        }
    }

    // Estimate path curvature at index i using 3-point approximation
    double computeCurvature(size_t i) {
        if (i + 2 >= path_x_.size()) return 0.0;
        double x1 = path_x_[i], y1 = path_y_[i];
        double x2 = path_x_[i + 1], y2 = path_y_[i + 1];
        double x3 = path_x_[i + 2], y3 = path_y_[i + 2];

        double a = std::hypot(x1 - x2, y1 - y2);
        double b = std::hypot(x2 - x3, y2 - y3);
        double c = std::hypot(x3 - x1, y3 - y1);

        double s = (a + b + c) / 2.0;
        double area = std::sqrt(std::max(s * (s - a) * (s - b) * (s - c), 0.0));
        return (4 * area) / (a * b * c + 1e-6);  // Avoid division by zero
    }

    // Pure Pursuit control logic
    std::pair<double, int> purePursuit(const State &state) {
        double L = 2.5;  // Wheelbase
        double base_lookahead = 8.0;
        double lookahead = base_lookahead;

        // Find the closest point to current state
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_x_.size(); ++i) {
            double dist = std::hypot(path_x_[i] - state.x, path_y_[i] - state.y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // From closest point, find a point beyond lookahead distance
        size_t target_idx = closest_idx;
        for (size_t i = closest_idx; i < path_x_.size(); ++i) {
            double dist = std::hypot(path_x_[i] - state.x, path_y_[i] - state.y);
            if (dist >= lookahead) {
                target_idx = i;
                break;
            }
        }

        // Adjust lookahead based on path curvature
        double curvature = computeCurvature(target_idx);
        lookahead = std::clamp(6.0 + 6.0 / (1.0 + std::abs(curvature)), 6.0, 20.0);

        // Compute steering angle
        double alpha = std::atan2(path_y_[target_idx] - state.y, path_x_[target_idx] - state.x) - state.yaw;
        double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead);
        return {delta, static_cast<int>(target_idx)};
    }

    // Main loop called by timer
    void onTimer() {
        auto [delta, _] = purePursuit(state_);
        state_ = vehicle_model_->update(state_, delta, 10.0, 0.01);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = state_.x;
        pose.pose.position.y = state_.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, state_.yaw);
        pose.pose.orientation = tf2::toMsg(q);

        trajectory_.push_back(pose);
        nav_msgs::msg::Path path_msg;
        path_msg.header = pose.header;
        path_msg.poses = trajectory_;
        path_pub_->publish(path_msg);

        nav_msgs::msg::Path desired_path;
        desired_path.header.stamp = now();
        desired_path.header.frame_id = "map";
        for (size_t i = 0; i < path_x_.size(); ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.header = desired_path.header;
            p.pose.position.x = path_x_[i];
            p.pose.position.y = path_y_[i];
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;  // Facing forward
            desired_path.poses.push_back(p);
        }
        desired_path_pub_->publish(desired_path);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "vehicle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose.pose;
        marker.scale.x = 2.5;
        marker.scale.y = 1.0;
        marker.scale.z = 0.6;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        vehicle_marker_pub_->publish(marker);
    }

    // Broadcast static transform between map and base_link
    void broadcastStaticTF() {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        tf_msg.transform.rotation = tf2::toMsg(q);

        static_broadcaster_->sendTransform(tf_msg);
    }
};

// Main entry point
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
