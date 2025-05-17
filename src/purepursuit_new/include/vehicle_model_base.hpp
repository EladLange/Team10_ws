// ============================
// include/vehicle_model_base.hpp
// ============================
// Abstract base class that defines a standard interface for vehicle motion models
// Any specific model (e.g. bicycle, ackermann) should inherit from this and implement the update() method

#pragma once
#include "types.hpp"  // Defines the State struct (x, y, yaw)

// Abstract base class for vehicle motion models
class VehicleModelBase {
public:
    virtual ~VehicleModelBase() = default;  // Virtual destructor to allow safe polymorphic deletion

    // Pure virtual function to be implemented by all derived vehicle models
    // Simulates motion from state 's' over timestep 'dt' given a steering input 'delta' and velocity
    // Parameters:
    //   - s: current vehicle state (position and orientation)
    //   - delta: steering angle input (in radians)
    //   - velocity: forward velocity (in m/s)
    //   - dt: time step (in seconds)
    // Returns:
    //   - State object representing the new vehicle state after motion
    virtual State update(const State &s, double delta, double velocity, double dt) const = 0; // Pure virtual function to be implemented by derived classes
};
