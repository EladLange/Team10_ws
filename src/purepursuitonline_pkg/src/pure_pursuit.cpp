#include "purepursuitonline_pkg/pure_pursuit.hpp"  // Include header for PurePursuitController definition
#include <cmath>     // For mathematical functions like hypot, atan2, sin
#include <limits>    // For numeric_limits<double>::max()

// Constructor: initializes the controller with lookahead distance and wheelbase
PurePursuitController::PurePursuitController(double lookahead_distance, double wheelbase)
    : lookahead_distance_(lookahead_distance), L_(wheelbase) {}

// Sets the reference path as two separate vectors for x and y coordinates
void PurePursuitController::setPath(const std::vector<double>& path_x, const std::vector<double>& path_y) {
    path_x_ = path_x;
    path_y_ = path_y;
}

// Computes the steering angle (delta) given the current state of the vehicle
std::pair<double, int> PurePursuitController::computeSteering(const State& state) {
    double min_diff = std::numeric_limits<double>::max();  // Initialize minimum difference to a large value
    size_t target_idx = 0;  // Index of the point on the path closest to lookahead distance

    // Loop through all path points to find the one closest to the lookahead distance
    for (size_t i = 0; i < path_x_.size(); ++i) {
        double dx = path_x_[i] - state.x;  // Difference in x
        double dy = path_y_[i] - state.y;  // Difference in y
        double dist = std::hypot(dx, dy);  // Euclidean distance to point
        double diff = std::abs(dist - lookahead_distance_);  // How close this distance is to the lookahead
        if (diff < min_diff) {
            min_diff = diff;
            target_idx = i;  // Update target index to the closest point
        }
    }

    // If no path is set, return zero steering and invalid index
    if (path_x_.empty()) return {0.0, -1};

    // Extract coordinates of the selected target point
    double target_x = path_x_[target_idx];
    double target_y = path_y_[target_idx];

    // Compute the angle difference between the vehicle's heading and the direction to the target point
    double alpha = std::atan2(target_y - state.y, target_x - state.x) - state.yaw;

    // Compute steering angle using the Pure Pursuit formula
    double delta = std::atan2(2.0 * L_ * std::sin(alpha), lookahead_distance_);

    // Return both the steering angle and the index of the target point
    return std::make_pair(delta, static_cast<int>(target_idx));
}
