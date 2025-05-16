#include "drone.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

Drone::Drone(const std::string& id, 
             std::shared_ptr<VehicleModelBase> model,
             const std::vector<double>& path_x,
             const std::vector<double>& path_y,
             const State& initial_state)
    : id_(id), 
      state_(initial_state),
      model_(model),
      path_x_(path_x),
      path_y_(path_y)
{
    // Ensure the state has the correct ID
    state_.id = id;
    
    // Adjust lookahead distance based on path complexity
    // This is optional and can be tuned for better performance
    if (!path_x.empty()) {
        // Calculate average distance between path points
        double total_dist = 0.0;
        for (size_t i = 1; i < path_x.size(); ++i) {
            total_dist += std::hypot(path_x[i] - path_x[i-1], path_y[i] - path_y[i-1]);
        }
        double avg_dist = path_x.size() > 1 ? total_dist / (path_x.size() - 1) : 0.0;
        
        // Set lookahead distance to be proportional to average point distance
        // but with minimum and maximum bounds
        lookahead_distance_ = std::clamp(avg_dist * 5.0, 1.0, 5.0);
    }
}

State Drone::update(double dt, double velocity) {
    // Get control command using pure pursuit
    auto [delta, _] = purePursuit();
    
    // Update state using the vehicle model
    state_ = model_->update(state_, delta, velocity, dt);
    
    return state_;
}

const State& Drone::getState() const {
    return state_;
}

std::pair<const std::vector<double>&, const std::vector<double>&> Drone::getPath() const {
    return {path_x_, path_y_};
}

geometry_msgs::msg::PoseStamped Drone::toPoseStamped(const std::string& frame_id) const {
    geometry_msgs::msg::PoseStamped pose;
    
    // Set header
    pose.header.frame_id = frame_id;
    
    // Set position
    pose.pose.position.x = state_.x;
    pose.pose.position.y = state_.y;
    pose.pose.position.z = state_.z;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, state_.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    
    return pose;
}


std::pair<double, int> Drone::purePursuit() const {
    if (path_x_.empty() || path_y_.empty()) {
        return {0.0, -1};
    }

    // === Step 1: Find closest point to current position ===
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_x_.size(); ++i) {
        double dist = std::hypot(path_x_[i] - state_.x, path_y_[i] - state_.y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // === Step 2: Compute dynamic lookahead based on curvature ===
    auto computeCurvature = [&](size_t i) -> double {
        if (i + 2 >= path_x_.size()) return 0.0;
        double x1 = path_x_[i], y1 = path_y_[i];
        double x2 = path_x_[i + 1], y2 = path_y_[i + 1];
        double x3 = path_x_[i + 2], y3 = path_y_[i + 2];

        double a = std::hypot(x1 - x2, y1 - y2);
        double b = std::hypot(x2 - x3, y2 - y3);
        double c = std::hypot(x3 - x1, y3 - y1);

        double s = (a + b + c) / 2.0;
        double area = std::sqrt(std::max(s * (s - a) * (s - b) * (s - c), 0.0));
        return (4.0 * area) / (a * b * c + 1e-6);
    };

    double curvature = computeCurvature(closest_idx);
    double lookahead = std::clamp(2.0 + 2.0 / (1.0 + std::abs(curvature)), 2.0, 7.0);

    // === Step 3: Find target point at lookahead distance ===
    size_t target_idx = closest_idx;
    for (size_t i = closest_idx; i < path_x_.size(); ++i) {
        double dist = std::hypot(path_x_[i] - state_.x, path_y_[i] - state_.y);
        if (dist >= lookahead) {
            target_idx = i;
            break;
        }
    }

    // If target is beyond end of path
    if (target_idx >= path_x_.size()) {
        target_idx = path_x_.size() - 1;
    }

    // === Step 4: Compute steering angle ===
    double dx = path_x_[target_idx] - state_.x;
    double dy = path_y_[target_idx] - state_.y;
    double target_angle = std::atan2(dy, dx);
    double alpha = target_angle - state_.yaw;

    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    double L = 2.5;  // Wheelbase
    double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead);
    delta = std::clamp(delta, -0.5, 0.5);  // Limit to ±30 degrees

    return {delta, static_cast<int>(target_idx)};
}