#pragma once
#include "vehicle_model_base.hpp"
#include <cmath>
#include <algorithm>  // Required for std::clamp


class AckermannModel : public VehicleModelBase {
public:
    explicit AckermannModel(double wheelbase) : L_(wheelbase) {}

    State update(const State &state, double steering_angle, double velocity, double dt) const override {
        State next = state;

        // Limit steering to realistic bounds (e.g., ±30°)
        steering_angle = std::clamp(steering_angle, -0.5236, 0.5236); // ±30 deg in rad

        // Kinematic Ackermann model
        next.x += velocity * std::cos(state.yaw) * dt;
        next.y += velocity * std::sin(state.yaw) * dt;
        next.yaw += (velocity / L_) * std::tan(steering_angle) * dt;

        return next;
    }

private:
    double L_;  // Wheelbase
};
