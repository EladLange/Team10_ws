#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <geometry_msgs/msg/accel.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>


class OdomPub:public rclcpp::Node
{
public:
    OdomPub(const std::string &name);


private:
    void msgCallback(const geometry_msgs::msg::PoseArray& msg);
    double roundToThreeDecimalPlaces (double value, int decimalPlaces);
    void publishStaticTransform();
    // geometry_msgs::msg::Twist convertCmdVector(const geometry_msgs::msg::Twist &vel, const geometry_msgs::msg::Pose ego_pos);


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ign_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ego_vel_pub_; 
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr ego_accel_pub_; 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    geometry_msgs::msg::Twist ego_vel;
    geometry_msgs::msg::Twist last_vel;
    geometry_msgs::msg::Accel ego_accel;
    geometry_msgs::msg::TransformStamped last_pose;
};