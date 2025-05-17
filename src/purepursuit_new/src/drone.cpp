// ============================
// src/drone.cpp
// ============================
// Drone class implementation for multi-drone control

#include "drone.hpp" // Custom drone class for multi-drone control
#include <cmath> // For math operations (e.g. atan2, hypot)
#include <limits> // For numeric limits
#include <algorithm> // For std::clamp

Drone::Drone(const std::string& id, // Constructor to initialize the drone
             std::shared_ptr<VehicleModelBase> model, // Pointer to the vehicle model for motion simulation
             const std::vector<double>& path_x, // X coordinates of the path to follow
             const std::vector<double>& path_y, // Y coordinates of the path to follow
             const State& initial_state) // Initial state of the drone
    : id_(id), // Unique identifier for the drone
      state_(initial_state), // Current state of the drone
      model_(model), // Vehicle model for motion simulation
      path_x_(path_x), // X coordinates of the path
      path_y_(path_y) // Y coordinates of the path
{
    // Ensure the state has the correct ID
    state_.id = id;
    
    // Adjust lookahead distance based on path complexity
    // This is optional and can be tuned for better performance
    if (!path_x.empty()) {
        // Calculate average distance between path points
        double total_dist = 0.0;  // Total distance between path points
        for (size_t i = 1; i < path_x.size(); ++i) {
            total_dist += std::hypot(path_x[i] - path_x[i-1], path_y[i] - path_y[i-1]); // Distance between consecutive points
        }
        double avg_dist = path_x.size() > 1 ? total_dist / (path_x.size() - 1) : 0.0; // Average distance between points
        
        // Set lookahead distance to be proportional to average point distance
        // but with minimum and maximum bounds
        lookahead_distance_ = std::clamp(avg_dist * 5.0, 1.0, 5.0); // Minimum 1.0, maximum 5.0
    }
}

State Drone::update(double dt, double velocity) { // Update the drone's state based on time step and velocity
    // Get control command using pure pursuit
    auto [delta, _] = purePursuit();
    
    // Update state using the vehicle model
    state_ = model_->update(state_, delta, velocity, dt);
    
    return state_; // Return updated state
}

const State& Drone::getState() const { // Get the current state of the drone
    return state_;
}

std::pair<const std::vector<double>&, const std::vector<double>&> Drone::getPath() const { // Get the path the drone is following
    return {path_x_, path_y_};
}

geometry_msgs::msg::PoseStamped Drone::toPoseStamped(const std::string& frame_id) const { // Convert the drone's state to a PoseStamped message
    geometry_msgs::msg::PoseStamped pose; // Create a PoseStamped message
    
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
    // ========== PURE PURSUIT CONTROLLER ==========
// Main control logic to compute required steering angle using lookahead target

    if (path_x_.empty() || path_y_.empty()) {
        return {0.0, -1};
    }

    // === Step 1: Find closest point to current position ===
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max(); //
    for (size_t i = 0; i < path_x_.size(); ++i) { // Iterate through path points
        double dist = std::hypot(path_x_[i] - state_.x, path_y_[i] - state_.y); // Calculate distance to each path point
        if (dist < min_dist) { // Check if this point is closer than the previous closest
            min_dist = dist;
            closest_idx = i;
        }
    }

    
    // ========== COMPUTE CURVATURE ==========
    // Calculates approximate curvature using 3 consecutive path points starting from index i
    // Uses triangle area formula (Heron’s) and side lengths to estimate how sharply the path turns
    // === Step 2: Compute dynamic lookahead based on curvature ===
    auto computeCurvature = [&](size_t i) -> double { // Lambda function to compute curvature
        // If not enough points to compute curvature, return 0
        if (i + 2 >= path_x_.size()) return 0.0; // Not enough points to compute curvature
        // Extract 3 points
        double x1 = path_x_[i], y1 = path_y_[i]; // First point
        double x2 = path_x_[i + 1], y2 = path_y_[i + 1]; // Second point
        double x3 = path_x_[i + 2], y3 = path_y_[i + 2]; // Third point
        // Calculate side lengths
        double a = std::hypot(x1 - x2, y1 - y2); // Length between first and second points
        double b = std::hypot(x2 - x3, y2 - y3); // Length between second and third points
        double c = std::hypot(x3 - x1, y3 - y1); // Length between first and third points
        // Calculate area using Heron's formula
        double s = (a + b + c) / 2.0; // Semi-perimeter
        // Return curvature = 4*area / (abc), add epsilon to avoid division by zero
        double area = std::sqrt(std::max(s * (s - a) * (s - b) * (s - c), 0.0));    // Triangle area - Heron's formula
        return (4.0 * area) / (a * b * c + 1e-6); // Curvature formula
    };
    // Compute curvature at closest point
    double curvature = computeCurvature(closest_idx); // Get curvature at closest point
    double lookahead = std::clamp(2.0 + 2.0 / (1.0 + std::abs(curvature)), 2.0, 7.0); // Adjust lookahead based on curvature

    // === Step 3: Find target point at lookahead distance ===
    // Find the target point on the path that is at least lookahead distance away
    // Start searching from the closest point
    // Iterate through path points starting from closest_idx
    // and find the first point that is at least lookahead distance away
    size_t target_idx = closest_idx; // Initialize target index to closest point
    for (size_t i = closest_idx; i < path_x_.size(); ++i) { // Iterate through path points
        double dist = std::hypot(path_x_[i] - state_.x, path_y_[i] - state_.y); // Calculate distance to each path point
        if (dist >= lookahead) { // Check if this point is at least lookahead distance away
            target_idx = i; // Set target index to this point
            break; // Exit loop as we found the target point
        }
    }

    // If target is beyond end of path
    if (target_idx >= path_x_.size()) { // Check if target index is beyond path size
        target_idx = path_x_.size() - 1; // Set target index to last point
    }

    // === Step 4: Compute steering angle ===
    // Calculate the angle to the target point
    // and the angle difference (alpha) between the target angle and current yaw
    double dx = path_x_[target_idx] - state_.x; // Difference in x coordinates
    double dy = path_y_[target_idx] - state_.y; // Difference in y coordinates
    double target_angle = std::atan2(dy, dx); // Angle to target point
    double alpha = target_angle - state_.yaw; // Angle difference between target angle and current yaw

    // Normalize alpha to [-pi, pi]
    // This ensures that the angle difference is within a manageable range
    // to avoid large steering angles
    while (alpha > M_PI) alpha -= 2.0 * M_PI; // Wrap around if greater than pi
    while (alpha < -M_PI) alpha += 2.0 * M_PI; // Wrap around if less than -pi

    double L = 1.55;  // Wheelbase
    double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead); // Steering angle calculation
    // Limit steering angle to ±30 degrees
    delta = std::clamp(delta, -0.5, 0.5);  // Limit to ±30 degrees

    // Return steering angle and target index
    return {delta, static_cast<int>(target_idx)}; // Return steering angle and target index
}