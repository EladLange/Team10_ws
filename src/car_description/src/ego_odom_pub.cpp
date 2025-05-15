#include <rclcpp/rclcpp.hpp>
#include "ego_odom_pub.hpp"


using std::placeholders::_1;

OdomPub::OdomPub(const std::string &name) : Node(name)
{

    ign_pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("/world/empty/pose/info",10,std::bind(&OdomPub::msgCallback,this, _1));   
    odom_pub_ = create_publisher<geometry_msgs::msg::Pose>("/ego_pose",10);
    
}

void OdomPub::msgCallback(const geometry_msgs::msg::PoseArray & msg)
{
    geometry_msgs::msg::Pose ego_pose;
    ego_pose= msg.poses[1];
    ego_pose.position.x=roundToThreeDecimalPlaces(ego_pose.position.x,3);
    ego_pose.position.y=roundToThreeDecimalPlaces(ego_pose.position.y,3);
    ego_pose.position.z=roundToThreeDecimalPlaces(ego_pose.position.z,3);
    ego_pose.orientation.z=roundToThreeDecimalPlaces(ego_pose.orientation.z,6);
    ego_pose.orientation.x=roundToThreeDecimalPlaces(ego_pose.orientation.x,6);
    ego_pose.orientation.y=roundToThreeDecimalPlaces(ego_pose.orientation.y,6);
    odom_pub_-> publish(ego_pose);
}



double OdomPub::roundToThreeDecimalPlaces(double value, int decimalPlaces) {
    double factor = std::pow(10.0, decimalPlaces);
    return std::round(value * factor) / factor;
}


int main (int argc, char* argv[])
{
rclcpp::init(argc,argv);
auto node=std::make_shared<OdomPub>("odom_pub");
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}