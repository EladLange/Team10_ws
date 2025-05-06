// ============================
// src/pure_pursuit_node.cpp
// ============================
// Main Pure Pursuit ROS2 node with modular vehicle model and full visualization

// ========== INCLUDE SECTION ==========
// Include necessary ROS2 and message headers
#include "rclcpp/rclcpp.hpp"  // Core ROS2 client library
#include "geometry_msgs/msg/pose_stamped.hpp"  // Message type for robot pose
#include "nav_msgs/msg/path.hpp"  // Message type for paths
#include "visualization_msgs/msg/marker.hpp"  // Message type for RViz visualization markers
#include "tf2/LinearMath/Quaternion.h"  // For quaternion math (yaw to quaternion)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Conversion between TF2 and geometry_msgs
#include "tf2_ros/transform_broadcaster.h"  // For publishing dynamic transforms

// Include custom headers for vehicle state and model
#include "types.hpp"  // Defines struct State (x, y, yaw)
#include "vehicle_model_base.hpp"  // Base interface for vehicle models
#include "bicycle_model.hpp"  // Concrete bicycle model implementation

// Standard library includes
#include <vector>  // For storing path points
#include <memory>  // For smart pointers
#include <cmath>  // For math operations (e.g. atan2, hypot)
#include <fstream>  // For reading path from CSV
#include <sstream>  // For parsing CSV strings
#include <algorithm>  // For clamp function

using namespace std::chrono_literals;  // Allow writing 10ms, 1s etc. as time literals

// ========== NODE CLASS DEFINITION ==========
class PurePursuitNode : public rclcpp::Node {
public:
    // Constructor – Initializes publishers, timer, path and initial state
    PurePursuitNode() : Node("pure_pursuit_node") {
        // Create a bicycle kinematic model with wheelbase 2.5 meters
        vehicle_model_ = std::make_unique<BicycleModel>(2.5);

        // Load path from CSV file (list of x,y coordinates)
        loadPath("/home/yonatan/Desktop/Team10_ws/src/purepursuit_new/track/yasMarina_path.csv");

        // Check if path was successfully loaded
        if (path_x_.empty() || path_y_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path is empty! Shutting down node.");
            rclcpp::shutdown();
            return;
        }

        // ROS2 publishers for actual and desired paths, and visualization marker
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);   // Publisher for actual path
        desired_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("desired_path", 10);    // Publisher for planned path
        vehicle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_marker", 10);    // Marker pub

        // Create a broadcaster to publish robot's transform (for TF visualization)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);   // Create TF broadcaster

        // Create a periodic timer that triggers control loop every 10 milliseconds
        timer_ = this->create_wall_timer(10ms, std::bind(&PurePursuitNode::onTimer, this)); // Set loop rate

        // Set initial robot position to first path point with yaw=0
        state_ = {path_x_[0], path_y_[0], 0.0};
    }

private:
    // ========== PRIVATE MEMBER VARIABLES ==========
    std::unique_ptr<VehicleModelBase> vehicle_model_;  // Pointer to current vehicle model implementation
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, desired_path_pub_;  // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_marker_pub_;  // Marker publisher
    rclcpp::TimerBase::SharedPtr timer_;  // Timer object
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // Transform broadcaster for TF
    std::vector<double> path_x_, path_y_;  // Vectors holding x and y coordinates of the path
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;  // List of previously visited robot poses
    State state_;  // Current state of the robot (x, y, yaw)

    // ========== LOAD PATH FROM CSV ==========
    // Reads x,y values from a CSV file and stores them in path_x_ and path_y_
    void loadPath(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file");
            return;
        }
        std::string line;
        std::getline(file, line);  // Skip the header line
        while (std::getline(file, line)) {
            std::stringstream ss(line); // Create string stream for parsing line
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');   // Extract x string
            std::getline(ss, y_str, ',');   // Extract y string
            path_x_.push_back(std::stod(x_str));    // Convert x to double
            path_y_.push_back(std::stod(y_str));    // Convert y to double
        }
    }

    // ========== COMPUTE CURVATURE ==========
    // Calculates approximate curvature using 3 consecutive path points starting from index i
    // Uses triangle area formula (Heron’s) and side lengths to estimate how sharply the path turns
    double computeCurvature(size_t i) {
        if (i + 2 >= path_x_.size()) return 0.0;  // Return 0 if fewer than 3 points left

        // Extract 3 points
        double x1 = path_x_[i], y1 = path_y_[i];    // First point
        double x2 = path_x_[i + 1], y2 = path_y_[i + 1];    // Second point
        double x3 = path_x_[i + 2], y3 = path_y_[i + 2];    // Third point

        // Compute triangle side lengths
        double a = std::hypot(x1 - x2, y1 - y2);    // Distance between p1-p2
        double b = std::hypot(x2 - x3, y2 - y3);    // Distance between p2-p3
        double c = std::hypot(x3 - x1, y3 - y1);    // Distance between p3-p1

        // Semi-perimeter
        double s = (a + b + c) / 2.0;

        // Area using Heron's formula
        double area = std::sqrt(std::max(s * (s - a) * (s - b) * (s - c), 0.0));    // Triangle area - Heron's formula

        // Return curvature = 4*area / (abc), add epsilon to avoid division by zero
        return (4 * area) / (a * b * c + 1e-6); // Final curvaturev
    }

    // ========== PURE PURSUIT CONTROLLER ==========
    // Main control logic to compute required steering angle using lookahead target
    std::pair<double, int> purePursuit(const State &state) {
        double L = 2.5;  // Wheelbase of the vehicle
        double base_lookahead = 8.0;  // Initial lookahead distance
        double lookahead = base_lookahead;  // Initial lookahead

        // Step 1: Find the closest point on the path to the current vehicle position
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_x_.size(); ++i) {
            double dist = std::hypot(path_x_[i] - state.x, path_y_[i] - state.y);   // Distance to path point
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;    // Update closest point index
            }
        }

        // Step 2: Find the first point ahead at the lookahead distance     (Target piont)
        size_t target_idx = closest_idx;    // Start with closest index
        for (size_t i = closest_idx; i < path_x_.size(); ++i) {
            double dist = std::hypot(path_x_[i] - state.x, path_y_[i] - state.y);
            if (dist >= lookahead) {
                target_idx = i;
                break;  // Stop at first point further than lookahead
            }
        }

        // Step 3: Adapt lookahead distance based on curvature (tight curves → smaller lookahead)
        double curvature = computeCurvature(target_idx);    // Estimate curvature
        lookahead = std::clamp(6.0 + 6.0 / (1.0 + std::abs(curvature)), 6.0, 20.0); // Adjust lookahead (range between 6[m] to 20[m])

        // Step 4: Compute steering angle using geometric relation
            // Compute the angle to the lookahead point:
        double alpha = std::atan2(path_y_[target_idx] - state.y, path_x_[target_idx] - state.x) - state.yaw;    // alpha: angle from vehicle heading to the lookahead point
            // Compute the steering angle using the Pure Pursuit formula:
        double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead);    // delta: desired steering angle (radians)
        return {delta, static_cast<int>(target_idx)};  // Return steering angle and index
            /* alpha: The angle between the vehicle’s current heading and the vector pointing from the vehicle to the target lookahead point.
                        positive alpha means the target is to the left of the heading.
                        negative alpha means it's to the right.
                delta: The desired steering angle computed using the Pure Pursuit formula, which assumes a bicycle kinematic model.
                        It depends on alpha, the wheelbase (L), and the lookahead distance.
            */
    }

    // ========== TIMER CALLBACK ==========
    // Called every 10ms: updates robot state, publishes trajectory and visualization
    void onTimer() {
        double velocity = 100.0; // m/sec
        double dt = 0.01;   // sec
        auto [delta, _] = purePursuit(state_);  // Get control command (steering)

        // Simulate vehicle motion based on model
        state_ = vehicle_model_->update(state_, delta, velocity, dt);  // velocity=10, dt=0.01s

        // Create pose message from new state
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();  // Timestamp
        pose.header.frame_id = "map";   // Global frame
        pose.pose.position.x = state_.x;    // Set x position
        pose.pose.position.y = state_.y;    // Set y position
        pose.pose.position.z = 0.0; // Set z position

        // Convert yaw angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, state_.yaw); // Convert yaw to quaternion
        pose.pose.orientation = tf2::toMsg(q);  // Assign orientation

        // Publish actual path (trajectory traveled)
        trajectory_.push_back(pose);    // Append to trajectory
        nav_msgs::msg::Path path_msg;
        path_msg.header = pose.header;
        path_msg.poses = trajectory_;   // Set path
        path_pub_->publish(path_msg);   // Publish trajectory

        // Publish desired path (planned path from file)
        nav_msgs::msg::Path desired_path;
        desired_path.header.stamp = now();
        desired_path.header.frame_id = "map";
        for (size_t i = 0; i < path_x_.size(); ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.header = desired_path.header;
            p.pose.position.x = path_x_[i];
            p.pose.position.y = path_y_[i];
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;  // Default orientation
            desired_path.poses.push_back(p);
        }
        desired_path_pub_->publish(desired_path);   // Publish desired path

        // Publish RViz marker showing vehicle position and size
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "vehicle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose.pose;    // Current pose
        marker.scale.x = 2.5;  // Length of vehicle
        marker.scale.y = 1.0;  // Width
        marker.scale.z = 0.6;  // Height
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // Visible for short time
        vehicle_marker_pub_->publish(marker);   


        visualization_msgs::msg::Marker steering_arrow;
        steering_arrow.header.frame_id = "map";
        steering_arrow.header.stamp = now();
        steering_arrow.ns = "steering";
        steering_arrow.id = 1;
        steering_arrow.type = visualization_msgs::msg::Marker::ARROW;
        steering_arrow.action = visualization_msgs::msg::Marker::ADD;
       // Define the start and end points of the arrow
        geometry_msgs::msg::Point start, end;
        start.x = state_.x;
        start.y = state_.y;
        start.z = 0.5;  // slightly above ground
        // Compute the endpoint based on delta (steering angle) and velocity (arrow length)
        double arrow_length = velocity * 0.2;  // scale length
        end.x = start.x + arrow_length * std::cos(state_.yaw + delta);
        end.y = start.y + arrow_length * std::sin(state_.yaw + delta);
        end.z = 0.5;
        steering_arrow.points.push_back(start);
        steering_arrow.points.push_back(end);

        // Set arrow appearance
        steering_arrow.scale.x = 0.1;  // shaft diameter
        steering_arrow.scale.y = 0.2;  // head diameter
        steering_arrow.scale.z = 0.2;  // head length
        steering_arrow.color.r = 1.0f;
        steering_arrow.color.g = 0.0f;
        steering_arrow.color.b = 0.0f;
        steering_arrow.color.a = 1.0f;
        steering_arrow.lifetime = rclcpp::Duration::from_seconds(0.1);  // persistent

        vehicle_marker_pub_->publish(steering_arrow);
        
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = now();
        text_marker.ns = "info";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = state_.x;
        text_marker.pose.position.y = state_.y;
        text_marker.pose.position.z = 1.5;
        text_marker.scale.z = 0.6;
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;
        text_marker.text = "v: " + std::to_string(velocity) + ", delta: " + std::to_string(delta);
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        vehicle_marker_pub_->publish(text_marker);


        // Broadcast current transform for visualization
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = state_.x;
        tf_msg.transform.translation.y = state_.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg); // Broadcast transform
    }
};

// ========== MAIN FUNCTION ==========
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2 communication
    rclcpp::spin(std::make_shared<PurePursuitNode>());  // Create and run the node
    rclcpp::shutdown();  // Shutdown cleanly
    return 0;
}
