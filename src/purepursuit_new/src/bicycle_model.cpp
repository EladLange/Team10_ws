// ============================
// src/bicycle_model.cpp
// ============================
// Implementation of the BicycleModel class, which simulates vehicle motion using a simple kinematic bicycle model

#include "bicycle_model.hpp"  // Include the header file for this class
#include <cmath>  // For trigonometric functions (cos, sin, tan)

// Constructor: initializes the model with a given wheelbase (distance between front and rear axles)
BicycleModel::BicycleModel(double wheelbase) : L_(wheelbase) {}

// Function: update
// Simulates the vehicle's next state given the current state, steering angle (delta), forward velocity, and timestep (dt)
// Arguments:
// - s: current vehicle state (x, y, yaw)
// - delta: steering angle (in radians)
// - velocity: vehicle's linear velocity (assumed constant during dt)
// - dt: time step duration in seconds
// Returns:
// - next: new state after applying the motion model
State BicycleModel::update(const State &s, double delta, double velocity, double dt) const {
    State next = s;  // Start from the current state

    // Update the x-position using the current yaw and velocity
    next.x += velocity * std::cos(s.yaw) * dt;

    // Update the y-position using the current yaw and velocity
    next.y += velocity * std::sin(s.yaw) * dt;

    // Update the yaw angle based on bicycle kinematics:
    // delta is the steering angle, L_ is the wheelbase
    // The formula comes from the arc-based approximation of vehicle turning
    next.yaw += velocity / L_ * std::tan(delta) * dt;

    return next;  // Return the updated state
}
