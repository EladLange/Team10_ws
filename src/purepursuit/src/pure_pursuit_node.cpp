#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

const double L = 3.0;
const double dt = 0.01;
const double lookahead_distance = 12.0;
const double velocity = 15.0;

struct State {
    double x;
    double y;
    double yaw;
};

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        loadPathFromCSV("/home/yonatan/Desktop/Team10_ws/src/purepursuit/src/Oval_Path_CSV.csv", path_x_, path_y_);
        if (path_x_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path is empty or failed to load.");
            rclcpp::shutdown();
            return;
        }

        desired_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("desired_path", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose", 10);

        nav_msgs::msg::Path desired_path;
        desired_path.header.stamp = this->now();
        desired_path.header.frame_id = "map";
        for (size_t i = 0; i < path_x_.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = desired_path.header;
            pose.pose.position.x = path_x_[i];
            pose.pose.position.y = path_y_[i];
            pose.pose.orientation.w = 1.0;
            desired_path.poses.push_back(pose);
        }
        desired_path_publisher_->publish(desired_path);

        state_ = {path_x_[0], path_y_[0] -1.0, 0.0};

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcastStaticTF();

        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&PurePursuitNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node started.");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::vector<double> path_x_, path_y_;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;
    State state_;

    void timerCallback() {
        auto [delta, _] = purePursuitControl(state_, path_x_, path_y_);
        state_ = update(state_, delta);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = state_.x;
        pose.pose.position.y = state_.y;
        pose.pose.orientation.w = 1.0;

        trajectory_.push_back(pose);

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = "map";
        path_msg.poses = trajectory_;

        path_publisher_->publish(path_msg);
        vehicle_pose_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Trajectory size: %lu | Pose (%.2f, %.2f)", trajectory_.size(), state_.x, state_.y);
    }

    void broadcastStaticTF() {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void loadPathFromCSV(const std::string& filename, std::vector<double>& path_x, std::vector<double>& path_y) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", filename.c_str());
            return;
        }

        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            if (!x_str.empty() && !y_str.empty()) {
                path_x.push_back(std::stod(x_str));
                path_y.push_back(std::stod(y_str));
            }
        }
    }

    std::pair<double, int> purePursuitControl(const State& state, const std::vector<double>& path_x, const std::vector<double>& path_y) {
        double min_diff = std::numeric_limits<double>::max();
        size_t target_idx = 0;
        for (size_t i = 0; i < path_x.size(); ++i) {
            double dx = path_x[i] - state.x;
            double dy = path_y[i] - state.y;
            double dist = std::hypot(dx, dy);
            double diff = std::abs(dist - lookahead_distance);
            if (diff < min_diff) {
                min_diff = diff;
                target_idx = i;
            }
        }
        double alpha = std::atan2(path_y[target_idx] - state.y, path_x[target_idx] - state.x) - state.yaw;
        double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead_distance);
        return {delta, static_cast<int>(target_idx)};
    }

    State update(State state, double delta) {
        state.x += velocity * std::cos(state.yaw) * dt;
        state.y += velocity * std::sin(state.yaw) * dt;
        state.yaw += velocity / L * std::tan(delta) * dt;
        return state;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
