// ============================
// include/bicycle_model.hpp
// ============================
#pragma once
#include "vehicle_model_base.hpp"

class BicycleModel : public VehicleModelBase {
public:
    explicit BicycleModel(double wheelbase);
    State update(const State &s, double delta, double velocity, double dt) const override;

private:
    double L_;
};