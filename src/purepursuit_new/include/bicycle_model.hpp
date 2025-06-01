// ============================
// include/bicycle_model.hpp
// ============================
// Declaration of the BicycleModel class, a specific implementation of a kinematic bicycle vehicle model
// This class inherits from VehicleModelBase and overrides the update method using bicycle motion equations

#pragma once
#include "vehicle_model_base.hpp"  // Include base class interface
#include <geometry_msgs/msg/pose.hpp>  // For Pose message type

// Concrete class representing a simple kinematic bicycle model
class BicycleModel : public VehicleModelBase {
public:
    // Constructor: takes the wheelbase of the vehicle (distance between front and rear axles)
    explicit BicycleModel(double wheelbase);

    // Override of the virtual update function from the base class
    // Computes the next vehicle state given current state, steering angle, velocity, and time step
    State update(const State &s, double delta, double velocity, double dt) const override;

    geometry_msgs::msg::Pose updatePose(const geometry_msgs::msg::Pose &pose, double delta, double velocity, double dt) const;


private:
    double L_;  // Wheelbase of the vehicle, affects turning radius
};
