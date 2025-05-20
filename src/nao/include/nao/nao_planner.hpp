#pragma once
#include <vector>
#include <array>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nao/msg/obstacle_state.hpp"

class NAOPlanner {
public:
  NAOPlanner(double a_max, double a_min, double j_max,
             double a_lat_max, double horizon,
             int num_steps, const std::array<double,4> &weights);

  void setRobotState(const geometry_msgs::msg::Point &pos,
                     const geometry_msgs::msg::Vector3 &vel);
  void setObstacles(const std::vector<nao::msg::ObstacleState> &obs);

  geometry_msgs::msg::Vector3 computeSafeOptimalAccel();

private:
  double a_max_, a_min_, j_max_, a_lat_max_;
  double horizon_;
  int num_steps_;
  std::array<double,4> w_;  // {collision, curvature, deviation, effort}

  geometry_msgs::msg::Point robot_pos_;
  geometry_msgs::msg::Vector3 robot_vel_;
  std::vector<nao::msg::ObstacleState> obstacles_;

  std::vector<geometry_msgs::msg::Vector3> sampleAccels() const;
  bool inNAO(const geometry_msgs::msg::Vector3 &a) const;

  double costCollision(const geometry_msgs::msg::Vector3 &a) const;
  double costCurvature(const geometry_msgs::msg::Vector3 &a) const;
  double costDeviation(const geometry_msgs::msg::Vector3 &a) const;
  double costEffort(const geometry_msgs::msg::Vector3 &a) const;
};
