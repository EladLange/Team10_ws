#include <rclcpp/rclcpp.hpp>
#include "ego_controller.hpp"


using std::placeholders::_1;

float wheels_radius=0.25;//wheels radius in m

EgoController::EgoController(const std::string &name) : Node(name)
{
    ego_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>("/ego_pose",10,std::bind(&EgoController::poseCallback,this, _1));
    ego_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/ego_vel",10,std::bind(&EgoController::velCallback,this, _1));
    vel_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/vel_cmd",10,std::bind(&EgoController::msgCallback,this, _1));   
    ackermann_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_steering_controller/reference",10);
    rear_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands",10);
    
}

void EgoController::poseCallback(const geometry_msgs::msg::Pose & msg)
{
   ego_pos=msg;
}

void EgoController::velCallback(const geometry_msgs::msg::Twist & msg)
{
   ego_vel=msg;
}


void EgoController::msgCallback(const geometry_msgs::msg::Twist & msg)
{
    geometry_msgs::msg::Twist des_vel=convertCmdVector(msg,ego_vel);
    float temp_vel=des_vel.linear.x;
    geometry_msgs::msg::TwistStamped ackermann_msg;
    temp_vel=temp_vel/wheels_radius;//linear velocity/wheel radius
    std_msgs::msg::Float64MultiArray rear_vel;
    rear_vel.data.push_back(temp_vel);
    rear_vel.data.push_back(temp_vel);
    ackermann_msg.header.stamp = this->get_clock()->now();
    ackermann_msg.twist=des_vel;

    rear_vel_pub_-> publish (rear_vel);
    ackermann_pub_-> publish(ackermann_msg);
}

geometry_msgs::msg::Twist EgoController::convertCmdVector(const geometry_msgs::msg::Twist &vel, const geometry_msgs::msg::Twist ego_vel){
    geometry_msgs::msg::Twist vel_cmd;

    float k_heading=1.0;
    float theta= atan2(vel.linear.y,vel.linear.x);

    // // Extract yaw from quaternion
    // tf2::Quaternion q(
    //     ego_pos.orientation.x,
    //     ego_pos.orientation.y,
    //     ego_pos.orientation.z,
    //     ego_pos.orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    double yaw=atan2(ego_vel.linear.y,ego_vel.linear.x);

    double abs_yaw=yaw;
    double abs_theta=theta;
    // if (yaw<0){
    //     abs_yaw=yaw+2*M_PI;
    // }
    // if (theta<0){
    //     abs_theta = theta+2*M_PI;
    // }
    double heading_error;
    heading_error=abs_theta-abs_yaw;
    heading_error=std::clamp(heading_error, (-M_PI/8), (M_PI/8));
    RCLCPP_INFO(this->get_logger(), "theta: %f, yaw: %f,heading_error: %f", abs_theta, abs_yaw, heading_error);
    float vel_size= sqrt(pow(vel.linear.x,2)+pow(vel.linear.y,2));
    double vx_local = (sin(heading_error) +cos(heading_error)) * vel_size;
    vel_cmd.linear.x = vx_local;
    // RCLCPP_INFO(this->get_logger(), "vel_cmd_linear_x: %f", vel_cmd.linear.x);  
    vel_cmd.angular.z = k_heading * heading_error;
    return vel_cmd;
}

int main (int argc, char* argv[])
{
rclcpp::init(argc,argv);
auto node=std::make_shared<EgoController>("ego_controller");
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}