#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class EgoController:public rclcpp::Node
{
public:
    EgoController(const std::string &name);


private:
    void msgCallback(const geometry_msgs::msg::Twist & msg);
    geometry_msgs::msg::Twist convertCmdVector(const geometry_msgs::msg::Twist &vel, const geometry_msgs::msg::Twist ego_vel);
    void poseCallback(const geometry_msgs::msg::Pose & msg);
    void velCallback(const geometry_msgs::msg::Twist & msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ego_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ego_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ackermann_pub_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_vel_pub_; 
    geometry_msgs::msg::Twist des_vel;
    geometry_msgs::msg::Twist ego_vel;
    geometry_msgs::msg::Pose ego_pos;
};