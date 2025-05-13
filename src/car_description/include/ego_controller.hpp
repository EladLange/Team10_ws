#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>


class EgoController:public rclcpp::Node
{
public:
    EgoController(const std::string &name);


private:
    void msgCallback(const geometry_msgs::msg::Twist & msg);


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ackermann_pub_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_vel_pub_; 
    geometry_msgs::msg::Twist des_vel;
    
    

};