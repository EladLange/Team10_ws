// ============================
// src/bicycle_model.cpp
// ============================
#include "bicycle_model.hpp"
#include <cmath>

BicycleModel::BicycleModel(double wheelbase) : L_(wheelbase) {}

State BicycleModel::update(const State &s, double delta, double velocity, double dt) const {
    State next = s;
    next.x += velocity * std::cos(s.yaw) * dt;
    next.y += velocity * std::sin(s.yaw) * dt;
    next.yaw += velocity / L_ * std::tan(delta) * dt;
    return next;
}