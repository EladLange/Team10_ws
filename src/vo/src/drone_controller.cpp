#include "drone_controller.hpp"

DroneController::DroneController(const Road& road)
    : road_(road) {}

void DroneController::control(Car& car, int lane_index) {
    geometry_msgs::msg::Twist vel;
    vel.linear.x = 10.0;  // Constant forward speed
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    // Optional: future improvement to keep car centered in lane
    car.setVelocity(vel);
}