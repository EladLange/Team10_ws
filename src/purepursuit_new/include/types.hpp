// ============================
// include/types.hpp
// ============================
// Definition of the State struct used to represent the vehicle's current state in the simulation
// Contains x, y position and yaw (heading/orientation in radians)

#pragma once
#include <string>

// Struct representing the vehicle's state in 2D space
struct State {
    std::string id;  // Identifier for the drone/vehicle
    double x;        // X position in meters
    double y;        // Y position in meters
    double z;        // Z position in meters (for drones)
    double yaw;      // Orientation angle (heading) in radians, where 0 points along the positive X-axis

    // Constructor with default values
    State(const std::string& id_val = "default",
          double x_val = 0.0,
          double y_val = 0.0,
          double z_val = 0.0,
          double yaw_val = 0.0)
        : id(id_val), x(x_val), y(y_val), z(z_val), yaw(yaw_val) {} // Initialize with provided values
};
