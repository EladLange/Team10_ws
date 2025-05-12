#include "../include/pid_controller.hpp" // or: #include "pid_controller.hpp"
#include <cmath>
#include <algorithm>
 
PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

ControlOutput PIDController::compute(float desired_vel, float current_vel, float dt) {
    float error = desired_vel - current_vel;
    integral_ += error * dt;
    float derivative = (error - prev_error_) / dt;

    float pid_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;

    ControlOutput output;
    if (pid_output >= 0.0f) {
        output.throttle = std::min(pid_output, 100.0f);
        output.brake = 0.0f;
    } else {
        output.brake = std::min(std::abs(pid_output), 100.0f);
        output.throttle = 0.0f;
    }

    return output;
}
