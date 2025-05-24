#include "vo/purepursuit.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>    // std::clamp
#include <cmath>        // hypot, sin, cos, atan2, sqrt
#include <limits>       // numeric_limits

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // fromMsg()
#include "tf2/utils.hpp"                            // getYaw()

PurePursuitController::PurePursuitController(double base_lookahead,double min_lookahead,double max_lookahead,double max_angular_z)
  : base_lookahead_(base_lookahead)
  , min_lookahead_(min_lookahead)
  , max_lookahead_(max_lookahead)
  , max_angular_z_(max_angular_z)
{}

bool PurePursuitController::loadPathFromCSV(const std::string & filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "[PurePursuit] Failed to open path file: " << filename << "\n";
    return false;
  }

  path_x_.clear();
  path_y_.clear();
  std::string line;

  // Read first line: skip header if non‐numeric
  if (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string xs, ys;
    if (std::getline(ss, xs, ',') && std::getline(ss, ys, ',')) {
      bool header = xs.find_first_not_of("0123456789.-") != std::string::npos
                 || ys.find_first_not_of("0123456789.-") != std::string::npos;
      if (!header) {
        processLine(line);
      }
    }
  }

  // Read remaining lines
  while (std::getline(file, line)) {
    processLine(line);
  }
  file.close();

  return !path_x_.empty();
}

geometry_msgs::msg::Twist PurePursuitController::computeCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double forward_speed)
{
    geometry_msgs::msg::Twist cmd;

    if (path_x_.empty()) {
        // No path: return zero command
        return cmd;
    }

    // 1) Find the closest path point
    std::size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path_x_.size(); ++i) {
        double dx = path_x_[i] - current_pose.pose.position.x;
        double dy = path_y_[i] - current_pose.pose.position.y;
        double d  = std::hypot(dx, dy);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // 2) Curvature for lookahead
    double curvature = computeCurvature(closest_idx);

    // 3) Dynamic lookahead (shorter on curves)
    const double k_la = 2.0;  // tuning
    double lookahead = std::clamp(
        base_lookahead_ / (1.0 + k_la * std::abs(curvature)),
        min_lookahead_, max_lookahead_);

    // 4) Get vehicle yaw
    tf2::Quaternion q;
    tf2::fromMsg(current_pose.pose.orientation, q);
    double yaw = tf2::getYaw(q);

    // 5) Find target index at ≥ lookahead and in front
    std::size_t target_idx = closest_idx;
    for (std::size_t i = closest_idx; i < path_x_.size(); ++i) {
        double dx = path_x_[i] - current_pose.pose.position.x;
        double dy = path_y_[i] - current_pose.pose.position.y;
        double dx_local = std::cos(yaw) * dx + std::sin(yaw) * dy;
        if (dx_local < 0.0) continue;  // behind car
        if (std::hypot(dx, dy) >= lookahead) {
            target_idx = i;
            break;
        }
    }
    if (target_idx == closest_idx) target_idx = path_x_.size() - 1;

    // 6) Compute heading error alpha in body frame
    double dx = path_x_[target_idx] - current_pose.pose.position.x;
    double dy = path_y_[target_idx] - current_pose.pose.position.y;
    double dx_local =  std::cos(yaw)*dx + std::sin(yaw)*dy;
    double dy_local = -std::sin(yaw)*dx + std::cos(yaw)*dy;
    double alpha    = std::atan2(dy_local, dx_local);

    // 7) Pure Pursuit law: steering
    double omega = 2.0 * forward_speed * std::sin(alpha) / lookahead;
    cmd.angular.z = std::clamp(omega, -max_angular_z_, max_angular_z_);

    // 8) Compute vx, vy in WORLD frame
    cmd.linear.x = forward_speed;
    cmd.linear.y = 0.0;

    // 9) Last goal for visualization
    last_goal_.header = current_pose.header;
    last_goal_.point.x = path_x_[target_idx];
    last_goal_.point.y = path_y_[target_idx];
    last_goal_.point.z = 0.0;

    // ----------- DEBUG PRINT HERE --------------
    // std::cout << "[PurePursuit Debug] target_idx=" << target_idx
    //       << " dx=" << dx
    //       << " dy=" << dy
    //       << " dist=" << std::hypot(dx, dy)
    //       << " vx=" << cmd.linear.x
    //       << " vy=" << cmd.linear.y
    //       << " yaw=" << yaw
    //       << " alpha=" << alpha
    //       << " omega=" << cmd.angular.z
    //       << std::endl;

    return cmd; // Return the computed command
}


void PurePursuitController::processLine(const std::string & line) {
  std::stringstream ss(line);
  std::string xs, ys;
  std::getline(ss, xs, ',');
  std::getline(ss, ys, ',');
  try {
    double x = std::stod(xs);
    double y = std::stod(ys);
    path_x_.push_back(x);
    path_y_.push_back(y);
  } catch (...) {
    // ignore bad lines
  }
}

double PurePursuitController::computeCurvature(std::size_t idx) const {
  // need three consecutive points
  if (idx + 2 >= path_x_.size()) {
    return 0.0;
  }
  double x1 = path_x_[idx],   y1 = path_y_[idx];
  double x2 = path_x_[idx+1], y2 = path_y_[idx+1];
  double x3 = path_x_[idx+2], y3 = path_y_[idx+2];

  double a = std::hypot(x1-x2, y1-y2);
  double b = std::hypot(x2-x3, y2-y3);
  double c = std::hypot(x3-x1, y3-y1);
  double s = 0.5*(a + b + c);
  double area = std::sqrt(std::max(s*(s-a)*(s-b)*(s-c), 0.0));

  return (4.0 * area) / (a * b * c + 1e-6);
}
