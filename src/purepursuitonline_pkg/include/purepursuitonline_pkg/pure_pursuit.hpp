#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <vector>   // For std::vector to hold path coordinates
#include <utility>  // For std::pair return type

// Structure representing the current state of the vehicle
struct State {
    double x;    // Vehicle's current x position
    double y;    // Vehicle's current y position
    double yaw;  // Vehicle's current heading (orientation) in radians
};

// Class implementing the Pure Pursuit controller
class PurePursuitController {
public:
    /**
     * Constructor: initializes the controller with a specific lookahead distance and wheelbase.
     * param lookahead_distance - distance ahead to target a path point.
     * param wheelbase - length between front and rear axle of the vehicle.
     */
    PurePursuitController(double lookahead_distance, double wheelbase);

    /**
     * Sets the reference path the controller will follow.
     * The path is defined as two separate vectors for x and y coordinates.
     * param path_x - vector of x positions
     * param path_y - vector of y positions
     */
    void setPath(const std::vector<double>& path_x, const std::vector<double>& path_y);

    /**
     * Computes the desired steering angle (delta) based on the current vehicle state.
     * It finds the closest path point to the lookahead distance and calculates the required turn.
     * param state - current position and heading of the vehicle.
     * return pair containing the steering angle (radians) and the index of the target point in the path.
     */
    std::pair<double, int> computeSteering(const State& state);

private:
    std::vector<double> path_x_, path_y_;  // Stored x and y coordinates of the path
    double lookahead_distance_;            // Lookahead distance used to select the goal point
    double L_;                             // Wheelbase of the vehicle
};

#endif  // PURE_PURSUIT_HPP
