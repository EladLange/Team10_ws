// ============================
// include/types.hpp
// ============================
// Definition of the State struct used to represent the vehicle's current state in the simulation
// Contains x, y position and yaw (heading/orientation in radians)

#pragma once

// Struct representing the vehicle's state in 2D space
struct State {
    double x;    // X position in meters
    double y;    // Y position in meters
    double yaw;  // Orientation angle (heading) in radians, where 0 points along the positive X-axis
};
