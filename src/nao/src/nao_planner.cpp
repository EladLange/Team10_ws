#include "nao/nao_planner.hpp"
#include <cmath>
#include <limits>

NAOPlanner::NAOPlanner(double a_max, double a_min, double j_max,
                       double a_lat_max, double horizon,
                       int num_steps, const std::array<double,4> &weights)
: a_max_(a_max), a_min_(a_min), j_max_(j_max), a_lat_max_(a_lat_max),
  horizon_(horizon), num_steps_(num_steps), w_(weights) {}

void NAOPlanner::setRobotState(const geometry_msgs::msg::Point &pos,
                              const geometry_msgs::msg::Vector3 &vel) {
  robot_pos_ = pos;
  robot_vel_ = vel;
}

void NAOPlanner::setObstacles(
    const std::vector<nao::msg::ObstacleState> &obs) {
  obstacles_ = obs;
}

std::vector<geometry_msgs::msg::Vector3> NAOPlanner::sampleAccels() const {
  std::vector<geometry_msgs::msg::Vector3> samples;
  int grid = 5;  // samples per axis
  for (int i = 0; i <= grid; ++i) {
    double ax = a_min_ + (a_max_ - a_min_) * i / grid;
    for (int j = -grid; j <= grid; ++j) {
      geometry_msgs::msg::Vector3 a;
      a.x = ax;
      a.y = a_lat_max_ * j / grid;
      a.z = 0.0;
      samples.push_back(a);
    }
  }
  return samples;
}

bool NAOPlanner::inNAO(const geometry_msgs::msg::Vector3 &a) const {
  for (const auto &obs : obstacles_) {
    for (int k = 1; k <= num_steps_; ++k) {
      double t = horizon_ * k / num_steps_;
      double ux = obs.position.x;
      double uy = obs.position.y;
      double vxr = robot_vel_.x - obs.velocity.x;
      double vyr = robot_vel_.y - obs.velocity.y;
      double ax_req = 2*ux/(t*t) - 2*vxr/t + obs.acceleration.x;
      double ay_req = 2*uy/(t*t) - 2*vyr/t + obs.acceleration.y;
      double radius = obs.radius;
      double dx = a.x - ax_req;
      double dy = a.y - ay_req;
      double dist = std::sqrt(dx*dx + dy*dy);
      if (dist <= 2*radius/(t*t)) return true;
    }
  }
  return false;
}

double NAOPlanner::costCollision(const geometry_msgs::msg::Vector3 &a) const {
  return inNAO(a) ? std::numeric_limits<double>::infinity() : 0.0;
}

double NAOPlanner::costCurvature(const geometry_msgs::msg::Vector3 &a) const {
  return std::abs(a.y);
}

double NAOPlanner::costDeviation(const geometry_msgs::msg::Vector3 &) const {
  return 0.0;
}

double NAOPlanner::costEffort(const geometry_msgs::msg::Vector3 &a) const {
  return std::sqrt(a.x*a.x + a.y*a.y);
}

geometry_msgs::msg::Vector3 NAOPlanner::computeSafeOptimalAccel() {
  auto candidates = sampleAccels();
  double best_cost = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Vector3 best;
  for (auto &a : candidates) {
    double c = w_[0]*costCollision(a)
             + w_[1]*costCurvature(a)
             + w_[2]*costDeviation(a)
             + w_[3]*costEffort(a);
    if (c < best_cost) {
      best_cost = c;
      best = a;
    }
  }
  return best;
}
