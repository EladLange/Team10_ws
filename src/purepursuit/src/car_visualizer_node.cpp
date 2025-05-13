#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class CarVisualizer : public rclcpp::Node {
public:
    CarVisualizer() : Node("car_visualizer") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_marker", 10);
        trajectory_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "vehicle_pose", 10,
            std::bind(&CarVisualizer::poseCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Car visualizer node started.");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_sub_;

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "car";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = msg->pose;

        marker.scale.x = 1.0;
        marker.scale.y = 0.5;
        marker.scale.z = 0.2;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_pub_->publish(marker);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarVisualizer>());
    rclcpp::shutdown();
    return 0;
}
