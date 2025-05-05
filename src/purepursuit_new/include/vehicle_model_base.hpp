// ============================
// include/vehicle_model_base.hpp
// ============================
#pragma once
#include "types.hpp"

class VehicleModelBase {
public:
    virtual ~VehicleModelBase() = default;
    virtual State update(const State &s, double delta, double velocity, double dt) const = 0;
};