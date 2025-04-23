#include <iostream>   // For input/output streams
#include <fstream>    // For file operations
#include <sstream>    // For string stream parsing
#include <vector>     // For using std::vector
#include <cmath>      // For math functions like sin, cos, hypot, atan2
#include <string>     // For using std::string
#include <cstdlib>    // For exit()

// Vehicle and simulation parameters
const double L = 2.5;             // Wheelbase of the vehicle in meters
const double dt = 0.1;            // Time step for each update in seconds
const double lookahead_distance = 5.0; // Look-ahead distance for Pure Pursuit
const double velocity = 10.0;     // Constant vehicle speed in m/s

// Struct representing the state of the vehicle
struct State {
    double x;     // Position along X axis
    double y;     // Position along Y axis
    double yaw;   // Heading angle in radians
};

// Load a 2D path from a CSV file with format: x,y
void loadPathFromCSV(const std::string& filename, std::vector<double>& path_x, std::vector<double>& path_y) {
    std::ifstream file(filename);  // Open the file
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(1); // Exit program if file not found
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        // If both coordinates are valid, convert to double and store
        if (!x_str.empty() && !y_str.empty()) {
            path_x.push_back(std::stod(x_str));
            path_y.push_back(std::stod(y_str));
        }
    }
}

// Pure Pursuit control algorithm
std::pair<double, int> purePursuitControl(const State& state,
                                          const std::vector<double>& path_x,
                                          const std::vector<double>& path_y) {
    double min_diff = 10;    // Initialize minimum distance difference
    size_t target_idx = 0;    // Index of the target point on the path

    // Find the path point whose distance is closest to the lookahead distance
    for (size_t i = 0; i < path_x.size(); ++i) {
        double dx = path_x[i] - state.x;
        double dy = path_y[i] - state.y;
        double dist = std::hypot(dx, dy); // Euclidean distance
        double diff = std::abs(dist - lookahead_distance);

        if (diff < min_diff) {
            min_diff = diff;
            target_idx = i;
        }
    }

    // Compute the angle between vehicle heading and target point
    double target_x = path_x[target_idx];
    double target_y = path_y[target_idx];
    double alpha = std::atan2(target_y - state.y, target_x - state.x) - state.yaw;

    // Compute steering angle (delta) using Pure Pursuit formula
    double delta = std::atan2(2.0 * L * std::sin(alpha), lookahead_distance);

    return std::make_pair(delta, static_cast<int>(target_idx));
}

// Update the vehicle's state based on current steering angle
State update(State state, double delta) {
    // Update x and y positions using bicycle model
    state.x += velocity * std::cos(state.yaw) * dt;
    state.y += velocity * std::sin(state.yaw) * dt;

    // Update yaw using the kinematic bicycle model
    state.yaw += velocity / L * std::tan(delta) * dt;
    return state;
}

int main() {
    std::vector<double> path_x, path_y;

    // Load path from CSV file
    loadPathFromCSV("path.csv", path_x, path_y);

    if (path_x.empty() || path_y.empty()) {
        std::cerr << "Error: path is empty or could not be loaded." << std::endl;
        return 1;
    }

    // Start the vehicle slightly below the first path point
    State state = { path_x[0], path_y[0] - 3.0, 0.0 };

    std::vector<double> traj_x, traj_y;

    // Simulate Pure Pursuit for 1000 steps or until the vehicle passes the last point
    for (int i = 0; i < 1000; ++i) {
        std::pair<double, int> result = purePursuitControl(state, path_x, path_y);
        double delta = result.first;
        int idx = result.second;

        state = update(state, delta); // Update vehicle position
        traj_x.push_back(state.x);    // Save trajectory point
        traj_y.push_back(state.y);

        // Stop if vehicle moves beyond last x-coordinate
        if (state.x > path_x.back())
            break;
    }

    // Write the resulting trajectory to a CSV file
    std::ofstream out("trajectory.csv");
    out << "x,y,yaw,delta\n";
    State temp_state = { path_x[0], path_y[0] - 3.0, 0.0 };
    for (int i = 0; i < traj_x.size(); ++i) {
        std::pair<double, int> result = purePursuitControl(temp_state, path_x, path_y);
        double delta = result.first;
        out << traj_x[i] << "," << traj_y[i] << "," << temp_state.yaw << "," << delta << "\n";
        temp_state = update(temp_state, delta);
    }
    out.close();

    std::cout << "Trajectory saved to trajectory.csv" << std::endl;
    return 0;
}