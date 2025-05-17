// =================================
// include/drone.hpp
// =================================
// This file defines the Drone class, which implements a pure pursuit controller for a drone
// that follows a specified path. The class includes methods for updating the drone's state,
// computing the pure pursuit control command, and converting the drone's state to a ROS message format.
// It also includes necessary includes for ROS2, geometry messages, and vehicle models.
#pragma once

#include "types.hpp" // Defines the State struct (x, y, yaw)
#include "vehicle_model_base.hpp" // Base interface for vehicle models
#include <memory> // For smart pointers
#include <vector> // For storing path points
#include <string> // For string operations
#include <geometry_msgs/msg/pose_stamped.hpp> // For PoseStamped message
#include <geometry_msgs/msg/pose.hpp> // For Pose message
#include <tf2/LinearMath/Quaternion.h> // For quaternion math (yaw to quaternion)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Conversion between TF2 and geometry_msgs

/**
 * @class Drone
 * @brief Class representing a drone with pure pursuit control
 * 
 * This class encapsulates the state and behavior of a drone that follows
 * a path using the pure pursuit algorithm.
 */
class Drone {
public:
    /**
     * @brief Constructor
     * @param id Unique identifier for the drone
     * @param model Pointer to the vehicle model to use for motion simulation
     * @param path_x X coordinates of the path to follow
     * @param path_y Y coordinates of the path to follow
     * @param initial_state Initial state of the drone
     */
    Drone(const std::string& id, 
          std::shared_ptr<VehicleModelBase> model,
          const std::vector<double>& path_x,
          const std::vector<double>& path_y,
          const State& initial_state);

    /**
     * @brief Update the drone's state using pure pursuit control
     * @param dt Time step in seconds
     * @param velocity Forward velocity in m/s
     * @return Updated state of the drone
     */
    State update(double dt, double velocity);

    /**
     * @brief Get the current state of the drone
     * @return Current state
     */
    const State& getState() const;

    /**
     * @brief Get the path the drone is following
     * @return Pair of vectors containing x and y coordinates
     */
    std::pair<const std::vector<double>&, const std::vector<double>&> getPath() const;

    /**
     * @brief Convert the drone's state to a PoseStamped message
     * @param frame_id Frame ID for the pose message
     * @return PoseStamped message representing the drone's state
     */
    geometry_msgs::msg::PoseStamped toPoseStamped(const std::string& frame_id) const;

private:
    /**
     * @brief Compute the pure pursuit control command
     * @return Pair containing steering angle and target index
     */
    std::pair<double, int> purePursuit() const;

    std::string id_;                       // Drone identifier
    State state_;                          // Current state
    std::shared_ptr<VehicleModelBase> model_; // Vehicle model for motion simulation
    std::vector<double> path_x_;           // X coordinates of path
    std::vector<double> path_y_;           // Y coordinates of path
    double lookahead_distance_ = 2.0;      // Lookahead distance for pure pursuit
};
