// =============================
// purepursuitonline_node.cpp (Online ROS2 Node)
// =============================

#include "rclcpp/rclcpp.hpp"  // Core ROS2 C++ client library
#include "geometry_msgs/msg/pose_stamped.hpp"  // Message for current vehicle pose
#include "nav_msgs/msg/path.hpp"  // Message type for path
#include "std_msgs/msg/float64.hpp"  // Message to publish steering angle
#include "visualization_msgs/msg/marker.hpp"  // Message for RViz visualization
#include "purepursuitonline_pkg/pure_pursuit.hpp"  // Include custom pure pursuit controller
#include <cmath>  // For trigonometric functions like atan2, sin, cos
#include <fstream>


// Main Node class inheriting from rclcpp::Node
class PurePursuitOnlineNode : public rclcpp::Node {
public:
    // Constructor initializes the node and all publishers/subscribers
    PurePursuitOnlineNode() : Node("purepursuitonline_node"), controller_(5.0, 2.5) {  // (lookahead distance, wheelbase)

        // Subscribe to the path topic (published online by planner)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PurePursuitOnlineNode::pathCallback, this, std::placeholders::_1));

        // Subscribe to the current vehicle pose topic (from online localization or simulation)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose", 10, std::bind(&PurePursuitOnlineNode::poseCallback, this, std::placeholders::_1));

        // Publish the calculated steering angle as a numeric value (for logging, control, etc.)
        steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);

        // Publish visualization marker for steering direction (RViz arrow)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/steering_marker", 10);
    }

private:
    // Callback triggered when a new path is received
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::vector<double> x, y;
        for (const auto& pose : msg->poses) {
            x.push_back(pose.pose.position.x);
            y.push_back(pose.pose.position.y);
        }
        controller_.setPath(x, y);  // Store the updated path in the controller
    }

    // Callback triggered when a new vehicle pose is received
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Extract position from Pose message
        State state;
        state.x = msg->pose.position.x;
        state.y = msg->pose.position.y;

        // Convert quaternion to yaw (Euler angle in radians)
        const auto& q = msg->pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        state.yaw = std::atan2(siny_cosp, cosy_cosp);

        // Compute the steering angle using Pure Pursuit controller
        auto [delta, idx] = controller_.computeSteering(state);

        // Log the computed steering angle to console
        RCLCPP_INFO(this->get_logger(), "Computed delta: %.4f (target idx: %d)", delta, idx);

        // Append to CSV log file
        std::ofstream logfile ("/tmp/steering_log.csv", std::ios_base::app);
        logfile << this->now().seconds() << "," << state.x << "," << state.y << "," << state.yaw << "," << delta << "," << idx << "\n";
        logfile.close();

        // Publish the steering angle as a Float64 message
        std_msgs::msg::Float64 delta_msg;
        delta_msg.data = delta;
        steer_pub_->publish(delta_msg);

        // Create and publish a visualization marker (arrow) in RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Use "map" frame for consistency in RViz
        marker.header.stamp = this->now();
        marker.ns = "steering";  // Marker namespace
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;  // Arrow type marker
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position the base of the arrow at the vehicle's current position
        marker.pose.position.x = state.x;
        marker.pose.position.y = state.y;
        marker.pose.position.z = 0.0;

        // Set orientation to represent yaw (rotation about Z-axis)
        marker.pose.orientation.w = std::cos(delta / 2.0);
        marker.pose.orientation.z = std::sin(delta / 2.0);

        // Set arrow dimensions (length, width, height)
        marker.scale.x = 2.0;  // arrow length
        marker.scale.y = 0.3;  // arrow width
        marker.scale.z = 0.3;  // arrow height

        // Color: green for straight, red for sharp turns
        marker.color.a = 1.0;  // alpha (opacity)
        marker.color.r = std::min(1.0, std::abs(delta));
        marker.color.g = 1.0 - marker.color.r;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);  // Send marker to RViz for online visualization
    }

    // Members
    PurePursuitController controller_;  // The actual Pure Pursuit controller object
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;  // Path subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;  // Pose subscriber
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;  // Steering angle publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // RViz marker publisher
};

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    rclcpp::spin(std::make_shared<PurePursuitOnlineNode>());  // Run the node
    rclcpp::shutdown();  // Clean up when done
    return 0;
} // End of node
