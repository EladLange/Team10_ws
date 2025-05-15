#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


class OdomPub:public rclcpp::Node
{
public:
    OdomPub(const std::string &name);


private:
    void msgCallback(const geometry_msgs::msg::PoseArray& msg);
    double roundToThreeDecimalPlaces (double value, int decimalPlaces);


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ign_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr odom_pub_; 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::Twist des_vel;
};