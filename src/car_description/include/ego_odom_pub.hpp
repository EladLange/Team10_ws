#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp> 


class OdomPub:public rclcpp::Node
{
public:
    OdomPub(const std::string &name);


private:
    void msgCallback(const geometry_msgs::msg::PoseArray& msg);
    double roundToThreeDecimalPlaces (double value, int decimalPlaces);


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ign_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr odom_pub_; 

    geometry_msgs::msg::Twist des_vel;
};