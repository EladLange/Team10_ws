#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>
#include <algorithm>


// Structure to hold the output of the controller
struct ControlOutput {
    float throttle;  // Throttle value: 0-100%
    float brake;     // Brake value: 0-100%
};

// PID controller class
class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    ControlOutput compute(float desired_vel, float current_vel, float dt);

private:
    float kp_, ki_, kd_;     // PID gains
    float integral_;         // Integral term memory
    float prev_error_;       // Previous error for derivative calculation
};

#endif // PID_CONTROLLER_HPP
