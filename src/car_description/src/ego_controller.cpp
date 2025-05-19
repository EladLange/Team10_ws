#include <rclcpp/rclcpp.hpp>
#include "ego_controller.hpp"


using std::placeholders::_1;

float wheels_radius=0.25;//wheels radius in m

EgoController::EgoController(const std::string &name) : Node(name)
{

    vel_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/vel_cmd",10,std::bind(&EgoController::msgCallback,this, _1));   
    ackermann_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ackermann_steering_controller/reference_unstamped",10);
    rear_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands",10);
    
}

void EgoController::msgCallback(const geometry_msgs::msg::Twist & msg)
{
    float temp_vel=msg.linear.x;
    temp_vel=temp_vel/wheels_radius;//linear velocity/wheel radius
    //RCLCPP_INFO_STREAM(get_logger(),"temp vel="<<temp_vel); //debugging
    std_msgs::msg::Float64MultiArray rear_vel;
    rear_vel.data.push_back(temp_vel);
    rear_vel.data.push_back(temp_vel);

    rear_vel_pub_-> publish (rear_vel);
    ackermann_pub_-> publish(msg);
}

int main (int argc, char* argv[])
{
rclcpp::init(argc,argv);
auto node=std::make_shared<EgoController>("ego_controller");
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}