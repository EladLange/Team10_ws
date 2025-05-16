// ============================
// src/bicycle_model.cpp
// ============================
// Implementation of the BicycleModel class, which simulates vehicle motion using a simple kinematic bicycle model

#include "bicycle_model.hpp"  // Include the header file for this class
#include <cmath>  // For trigonometric functions (cos, sin, tan)
#include <geometry_msgs/msg/pose.hpp>
#include "tf2/LinearMath/Quaternion.h"  // For quaternion math (yaw to quaternion)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Conversion between TF2 and geometry_msgs
#include "tf2_ros/transform_broadcaster.h"  // For publishing dynamic transforms

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

geometry_msgs::msg::Pose BicycleModel::updatePose(const geometry_msgs::msg::Pose &pose, double delta, double velocity, double dt) const {
    geometry_msgs::msg::Pose new_pose = pose;

    // Convert quaternion to yaw
    tf2::Quaternion q_in;
    tf2::fromMsg(pose.orientation, q_in);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_in).getRPY(roll, pitch, yaw);

    // Update position
    new_pose.position.x += velocity * std::cos(yaw) * dt;
    new_pose.position.y += velocity * std::sin(yaw) * dt;

    // Update yaw using bicycle kinematics
    yaw += velocity / L_ * std::tan(delta) * dt;

    // Convert back to quaternion
    tf2::Quaternion q_out;
    q_out.setRPY(0, 0, yaw);
    new_pose.orientation = tf2::toMsg(q_out);

    return new_pose;
}


