#include "rclcpp/rclcpp.hpp"
#include "vo_visualization.hpp"
#include "velocity_obstacle.hpp"
#include "global_variables.hpp"
#include "car.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

// Type aliases for cleaner code
using vis_marker_arr = visualization_msgs::msg::MarkerArray;
using vis_marker = visualization_msgs::msg::Marker;
using pose_msg = geometry_msgs::msg::Pose;
using twist_msg = geometry_msgs::msg::Twist;
using string_msg = std_msgs::msg::String;
using float32_msg = std_msgs::msg::Float32;

// Global variables
VelocityObstacle vo;
float time_horizon = 3.0f;
float max_acceleration = 0.1f;
float min_acceleration = -3.0f;
float time_step = 1.0f;
float delta_t = 0.1f;

class VONode : public rclcpp::Node {
public:
    VONode() : Node("vo_node") {
        RCLCPP_INFO(this->get_logger(), "Starting VO algorithm node...");

        // Declare parameters
        this->declare_parameter("debug_level", 0);  // 0=info, 1=debug, 2=verbose
        debug_level_ = this->get_parameter("debug_level").as_int();
        RCLCPP_INFO(this->get_logger(), "Debug level set to %d", debug_level_);

        // Publishers
        vo_marker_pub_ = this->create_publisher<vis_marker_arr>("vo_marker_array", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        debug_pub_ = this->create_publisher<string_msg>("vo_debug", 10);

        // Subscribers
        ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "ego_pose", 10, std::bind(&VONode::egoCallback, this, std::placeholders::_1));

        // Subscribe to cmd_vel to get our own velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&VONode::cmdVelCallback, this, std::placeholders::_1));

        obstacles_sub_ = this->create_subscription<vis_marker_arr>(
            "obstacles", 10, std::bind(&VONode::obstaclesCallback, this, std::placeholders::_1));

        // Subscribe to obstacle velocities
        obstacles_vel_sub_ = this->create_subscription<vis_marker_arr>(
            "obstacles_velocity", 10, std::bind(&VONode::obstaclesCallback, this, std::placeholders::_1));

        // Timer for periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VONode::update, this));

        // Initialize raceline
        initializeRaceline();
        RCLCPP_INFO(this->get_logger(), "Initialized raceline with %zu points", raceline_.size());

        RCLCPP_INFO(this->get_logger(), "VO node initialized and ready");
    }

    // Initialize raceline points
    void initializeRaceline() {
        // Create a simple straight line raceline for testing
        for (float x = 0.0f; x < 200.0f; x += 5.0f) {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = 0.0;
            point.z = 0.0;
            raceline_.push_back(point);
        }
    }

private:
    // ROS publishers
    rclcpp::Publisher<vis_marker_arr>::SharedPtr vo_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<string_msg>::SharedPtr debug_pub_;

    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<vis_marker_arr>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<vis_marker_arr>::SharedPtr obstacles_vel_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    pose_msg ego_pose_;
    twist_msg ego_vel_;
    std::vector<pose_msg> obstacle_poses_;
    std::vector<twist_msg> obstacle_velocities_;
    std::vector<geometry_msgs::msg::Point> raceline_;

    // Debug variables
    int debug_level_;
    bool ego_pose_received_ = false;
    bool ego_vel_received_ = false;
    bool obstacles_received_ = false;
    rclcpp::Time last_update_time_;

    void egoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ego_pose_ = msg->pose;
        ego_pose_received_ = true;

        if (debug_level_ > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Received ego pose: (%f, %f, %f)",
                       ego_pose_.position.x, ego_pose_.position.y, ego_pose_.position.z);
        }

        // Publish debug info
        publishDebugInfo("Ego pose updated");
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Store the commanded velocity as our current velocity
        ego_vel_ = *msg;
        ego_vel_received_ = true;

        if (debug_level_ > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Received ego velocity from cmd_vel: (%f, %f, %f)",
                       ego_vel_.linear.x, ego_vel_.linear.y, ego_vel_.linear.z);
        }

        // Publish debug info
        publishDebugInfo("Ego velocity updated from cmd_vel");
    }

    void obstaclesCallback(const vis_marker_arr::SharedPtr msg) {
        // Extract obstacle poses and velocities from marker array
        obstacle_poses_.clear();
        obstacle_velocities_.clear();

        if (debug_level_ > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Received obstacle message with %zu markers", msg->markers.size());
        }

        // First, collect all car markers
        std::vector<vis_marker> car_markers;
        std::vector<vis_marker> velocity_markers;

        for (const auto& marker : msg->markers) {
            if (marker.ns == "cars") {
                car_markers.push_back(marker);
            } else if (marker.ns == "velocity_arrow") {
                velocity_markers.push_back(marker);
            }
        }

        // Process car markers to extract obstacle information
        for (const auto& marker : car_markers) {
            obstacle_poses_.push_back(marker.pose);

            // Initialize with zero velocity
            twist_msg zero_vel;
            zero_vel.linear.x = 0.0;
            zero_vel.linear.y = 0.0;
            zero_vel.linear.z = 0.0;
            zero_vel.angular.x = 0.0;
            zero_vel.angular.y = 0.0;
            zero_vel.angular.z = 0.0;
            obstacle_velocities_.push_back(zero_vel);

            if (debug_level_ > 1) {
                RCLCPP_DEBUG(this->get_logger(), "Added obstacle at position: (%f, %f, %f)",
                           marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            }
        }

        // Try to match velocity markers with car markers
        for (const auto& vel_marker : velocity_markers) {
            // Find the closest car to this velocity marker
            if (vel_marker.type == vis_marker::ARROW && vel_marker.points.size() >= 2) {
                const auto& start = vel_marker.points[0];
                const auto& end = vel_marker.points[1];

                // Find the closest car to the start point
                double min_dist = std::numeric_limits<double>::max();
                size_t closest_car_idx = 0;

                for (size_t i = 0; i < obstacle_poses_.size(); ++i) {
                    const auto& car_pos = obstacle_poses_[i].position;
                    double dist = std::pow(car_pos.x - start.x, 2) +
                                 std::pow(car_pos.y - start.y, 2) +
                                 std::pow(car_pos.z - start.z, 2);

                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_car_idx = i;
                    }
                }

                // If we found a close car, update its velocity
                if (min_dist < 2.0 && closest_car_idx < obstacle_velocities_.size()) {
                    twist_msg& vel = obstacle_velocities_[closest_car_idx];
                    vel.linear.x = end.x - start.x;
                    vel.linear.y = end.y - start.y;
                    vel.linear.z = end.z - start.z;

                    if (debug_level_ > 1) {
                        RCLCPP_DEBUG(this->get_logger(), "Updated obstacle %zu velocity: (%f, %f, %f)",
                                   closest_car_idx, vel.linear.x, vel.linear.y, vel.linear.z);
                    }
                }
            }
        }

        obstacles_received_ = true;

        // Publish debug info
        std::string debug_msg = "Obstacles updated: " + std::to_string(obstacle_poses_.size()) + " obstacles";
        publishDebugInfo(debug_msg);
    }



    void publishDebugInfo(const std::string& message) {
        if (debug_level_ > 0) {
            string_msg msg;
            msg.data = message;
            debug_pub_->publish(msg);
        }
    }

    void update() {
        // Check if we have all the necessary data
        if (!ego_pose_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for ego pose data...");
            return;
        }

        if (obstacle_poses_.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No obstacles detected yet");
            return;
        }

        // Store current time for velocity estimation
        last_update_time_ = this->now();

        // Calculate total radius for collision detection
        float r_total = calculateTotalRadius();

        // Log the current state if in debug mode
        if (debug_level_ > 1) {
            RCLCPP_DEBUG(this->get_logger(), "Ego position: (%f, %f, %f)",
                       ego_pose_.position.x, ego_pose_.position.y, ego_pose_.position.z);
            RCLCPP_DEBUG(this->get_logger(), "Ego velocity: (%f, %f, %f)",
                       ego_vel_.linear.x, ego_vel_.linear.y, ego_vel_.linear.z);
            RCLCPP_DEBUG(this->get_logger(), "Number of obstacles: %zu", obstacle_poses_.size());
            RCLCPP_DEBUG(this->get_logger(), "Number of obstacle velocities: %zu", obstacle_velocities_.size());
        }

        // If we don't have velocity data yet, initialize with a default forward velocity
        if (!ego_vel_received_) {
            ego_vel_.linear.x = 1.0;  // Default forward velocity
            ego_vel_.linear.y = 0.0;
            ego_vel_.linear.z = 0.0;
            ego_vel_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Initializing with default velocity: (%f, %f)",
                      ego_vel_.linear.x, ego_vel_.linear.y);
        }

        // Select the best velocity using the VO algorithm
        twist_msg new_velocity = vo.selectBestVelocity(ego_pose_, ego_vel_, obstacle_poses_, obstacle_velocities_, raceline_, r_total);

        RCLCPP_INFO(this->get_logger(), "New velocity: (%f, %f)",
                   new_velocity.linear.x, new_velocity.linear.y);

        // Publish the new velocity command
        cmd_vel_pub_->publish(new_velocity);

        // Visualize velocity obstacles and debug information
        publishVOMarkers(new_velocity, r_total);

        // Publish debug info
        std::string debug_msg = "Selected velocity: (" +
                               std::to_string(new_velocity.linear.x) + ", " +
                               std::to_string(new_velocity.linear.y) + ")";
        publishDebugInfo(debug_msg);
    }

    float calculateTotalRadius() {
        // Get car dimensions
        geometry_msgs::msg::Vector3 scale;
        scale.x = 1.2; // length
        scale.y = 0.8; // width
        scale.z = 0.5; // height

        float r_ego = 0.5f * std::sqrt(std::pow(scale.x, 2) + std::pow(scale.y, 2));
        float r_obstacle = r_ego;  // Assuming obstacles are the same size

        if (debug_level_ > 1) {
            RCLCPP_DEBUG(this->get_logger(), "Calculated total radius: %f", r_ego + r_obstacle);
        }

        return r_ego + r_obstacle;
    }

    void publishVOMarkers(const twist_msg& selected_velocity, float r_total) {
        vis_marker_arr marker_array;

        // Add VO cones for each obstacle
        for (size_t i = 0; i < obstacle_poses_.size(); ++i) {
            vis_marker cone_marker;
            setVOConeMarker(cone_marker, ego_pose_, obstacle_poses_[i], ego_vel_, obstacle_velocities_[i], r_total);

            // Set a unique ID for each cone
            cone_marker.id = static_cast<int>(i);

            marker_array.markers.push_back(cone_marker);
        }

        // Add marker for the selected velocity
        vis_marker candidate_marker;
        setCandidateMarker(candidate_marker, ego_pose_, selected_velocity, r_total, time_step);
        candidate_marker.id = 1000;  // Use a unique ID
        marker_array.markers.push_back(candidate_marker);

        // Publish all markers
        vo_marker_pub_->publish(marker_array);

        if (debug_level_ > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Published %zu VO visualization markers", marker_array.markers.size());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VONode>());
    rclcpp::shutdown();
    return 0;
}
