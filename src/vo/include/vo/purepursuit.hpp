// src/vo/include/vo/purepursuit.hpp
#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

/**
 * PurePursuitController implements the Pure Pursuit path-tracking algorithm
 * with dynamic lookahead based on path curvature, and records the last goal point.
 */
class PurePursuitController {
public:
  /**
   * @param base_lookahead   Base lookahead distance (meters)
   * @param min_lookahead    Minimum lookahead distance (meters)
   * @param max_lookahead    Maximum lookahead distance (meters)
   * @param max_angular_z    Maximum angular.z output (rad/s)
   */
  PurePursuitController(double base_lookahead,
                        double min_lookahead,
                        double max_lookahead,
                        double max_angular_z);

  /**
   * Load a 2D path from a CSV file. Each line must contain "x,y".
   * Skips a header row if the first tokens are non-numeric.
   * @return true if file opened and at least one point was parsed
   */
  bool loadPathFromCSV(const std::string & filename);

  /**
   * Compute the commanded Twist for Pure Pursuit.
   * Lookahead distance is adjusted inversely with curvature.
   * @param current_pose   The vehicle’s current pose stamped
   * @param forward_speed  Desired forward (linear.x) speed
   * @return Twist command with linear.x and angular.z set
   */
  geometry_msgs::msg::Twist computeCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double forward_speed);

  /**
   * @return the last goal point selected by computeCommand (in same frame & stamp)
   */
  geometry_msgs::msg::PointStamped lastGoal() const { return last_goal_; }

private:
  void processLine(const std::string & line);       // parse one "x,y" line
  double computeCurvature(std::size_t idx) const;   // approximate curvature

  std::vector<double> path_x_, path_y_;
  double base_lookahead_, min_lookahead_, max_lookahead_, max_angular_z_;
  geometry_msgs::msg::PointStamped last_goal_;
};
