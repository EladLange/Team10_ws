#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

const double L = 2.5;
const double dt = 0.01;
const double lookahead_distance = 10.0;
const double velocity = 40.0;

struct State {
    double x, y, yaw;
};

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        loadPathFromCSV("/home/yonatan/Desktop/Team10_ws/src/purepursuit/src/Oval_Path_CSV.csv", path_x_, path_y_);
        if (path_x_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path failed to load.");
            rclcpp::shutdown();
            return;
        }

        desired_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("desired_path", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        vehicle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_marker", 10);

        state_ = {path_x_[0], path_y_[0] - 0.0, 0.0};
        // publishDesiredPath();

        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&PurePursuitNode::timerCallback, this));
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcastStaticTF();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_, desired_path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::vector<double> path_x_, path_y_;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;
    State state_;

    // void publishDesiredPath() {
    //     nav_msgs::msg::Path desired_path;
    //     desired_path.header.stamp = this->now();
    //     desired_path.header.frame_id = "map";
    //     for (size_t i = 0; i < path_x_.size(); ++i) {
    //         geometry_msgs::msg::PoseStamped pose;
    //         pose.header = desired_path.header;
    //         pose.pose.position.x = path_x_[i];
    //         pose.pose.position.y = path_y_[i];
    //         pose.pose.position.z = 0.0;
    //         pose.pose.orientation.w = 1.0;
    //         desired_path.poses.push_back(pose);
    //     }
    //     desired_path_publisher_->publish(desired_path);
    // }

    void timerCallback() {
        auto [delta, _] = purePursuitControl(state_, path_x_, path_y_);
        state_ = update(state_, delta);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = state_.x;
        pose.pose.position.y = state_.y;
        pose.pose.position.z = 0.0;

        // Compute orientation from yaw using tf2
        tf2::Quaternion q;
        q.setRPY(0, 0, state_.yaw);
        pose.pose.orientation = tf2::toMsg(q);

        trajectory_.push_back(pose);

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = "map";
        path_msg.poses = trajectory_;
        path_publisher_->publish(path_msg);

        // Marker visualization
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
        
        nav_msgs::msg::Path desired_path;
        desired_path.header.stamp = this->now();
        desired_path.header.frame_id = "map";
        for (size_t i = 0; i < path_x_.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = desired_path.header;
            pose.pose.position.x = path_x_[i];
            pose.pose.position.y = path_y_[i];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            desired_path.poses.push_back(pose);
        }
        desired_path_publisher_->publish(desired_path);
    }

    void broadcastStaticTF() {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void loadPathFromCSV(const std::string &filename, std::vector<double> &x, std::vector<double> &y) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path CSV");
            return;
        }
        std::string line;
        std::getline(file, line); // Skip header
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            x.push_back(std::stod(x_str));
            y.push_back(std::stod(y_str));
        }
    }

    std::pair<double, int> purePursuitControl(const State &state, const std::vector<double> &x, const std::vector<double> &y) {
        double min_diff = std::numeric_limits<double>::max();
        size_t target_idx = 0;
        for (size_t i = 0; i < x.size(); ++i) {
            double dist = std::hypot(x[i] - state.x, y[i] - state.y);
            double diff = std::abs(dist - lookahead_distance);
            if (diff < min_diff) {
                min_diff = diff;
                target_idx = i;
            }
        }
        double alpha = std::atan2(y[target_idx] - state.y, x[target_idx] - state.x) - state.yaw;
        double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead_distance);
        return {delta, static_cast<int>(target_idx)};
    }

    State update(State s, double delta) {
        s.x += velocity * std::cos(s.yaw) * dt;
        s.y += velocity * std::sin(s.yaw) * dt;
        s.yaw += velocity / L * std::tan(delta) * dt;
        return s;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
