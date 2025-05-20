#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "nao/msg/obstacle_array.hpp"
#include "nao/nao_planner.hpp"

using std::placeholders::_1;

class NAOPlannerNode : public rclcpp::Node {
public:
  NAOPlannerNode()
  : Node("nao_local_planner") {
    // parameters
    this->declare_parameter<double>("a_max", 2.5);
    this->declare_parameter<double>("a_min", -4.0);
    this->declare_parameter<double>("a_lat_max", 8.0);
    this->declare_parameter<double>("horizon", 1.0);
    this->declare_parameter<int>("num_steps", 10);
    this->declare_parameter<std::vector<double>>("weights",
      std::vector<double>{1000.0,1.0,1.0,0.1});

    double a_max, a_min, a_lat_max, horizon;
    int num_steps;
    std::vector<double> weights;
    this->get_parameter("a_max", a_max);
    this->get_parameter("a_min", a_min);
    this->get_parameter("a_lat_max", a_lat_max);
    this->get_parameter("horizon", horizon);
    this->get_parameter("num_steps", num_steps);
    this->get_parameter("weights", weights);

    planner_ = std::make_shared<NAOPlanner>(
      a_max, a_min, 0.0, a_lat_max,
      horizon, num_steps,
      std::array<double,4>{weights[0],weights[1],weights[2],weights[3]});

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&NAOPlannerNode::odomCb, this, _1));
    obs_sub_ = this->create_subscription<nao::msg::ObstacleArray>(
      "/obstacles", 10, std::bind(&NAOPlannerNode::obsCb, this, _1));

    accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
      "/cmd_accel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NAOPlannerNode::controlLoop, this));
  }

private:
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pos_ = msg->pose.pose.position;
    robot_vel_ = msg->twist.twist.linear;
  }
  void obsCb(const nao::msg::ObstacleArray::SharedPtr msg) {
    obstacles_ = msg->obstacles;
  }
  void controlLoop() {
    if (obstacles_.empty()) return;
    planner_->setRobotState(robot_pos_, robot_vel_);
    planner_->setObstacles(obstacles_);
    auto best_accel_vec = planner_->computeSafeOptimalAccel();
    geometry_msgs::msg::AccelStamped cmd;
    cmd.header.stamp = now();
    cmd.accel.linear = best_accel_vec;
    cmd.accel.angular.x = 0.0;
    cmd.accel.angular.y = 0.0;
    cmd.accel.angular.z = 0.0;
    accel_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nao::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Point robot_pos_;
  geometry_msgs::msg::Vector3 robot_vel_;
  std::vector<nao::msg::ObstacleState> obstacles_;
  std::shared_ptr<NAOPlanner> planner_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NAOPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
