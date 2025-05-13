// #include <iostream>
// #include <fstream>
// #include "pid_controller.hpp" 

// int main() {
//     std::ofstream log_file("velocity_log.csv");
//     log_file << "Time,Speed,Throttle,Brake\n";

//     float time = 0.0f;
//     PIDController pid(2.0f, 0.1f, 0.5f);
//     float desired_velocity = 15.0f;
//     float current_velocity = 0.0f;
//     float dt = 2.0f;

//     for (int i = 0; i < 200; ++i) {
//         ControlOutput ctrl = pid.compute(desired_velocity, current_velocity, dt);

//         if (ctrl.throttle > 0.0f) {
//             current_velocity += (ctrl.throttle / 100.0f) * 5.0f * dt;
//             std::cout << "[Throttle] " << ctrl.throttle << "%\t";
//         } else if (ctrl.brake > 0.0f) {
//             current_velocity -= (ctrl.brake / 100.0f) * 10.0f * dt;
//             current_velocity = std::max(0.0f, current_velocity);
//             std::cout << "[Brakes]   " << ctrl.brake << "%\t";
//         }

//         log_file << time << "," << current_velocity << "," << ctrl.throttle << "," << ctrl.brake << "\n";
//         time += dt;

//         std::cout << "Speed: " << current_velocity << " m/s" << std::endl;
//     }

//     log_file.close();
//     return 0;
// }
