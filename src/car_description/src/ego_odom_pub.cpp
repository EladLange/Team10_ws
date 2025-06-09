#include <rclcpp/rclcpp.hpp>
#include "ego_odom_pub.hpp"

using std::placeholders::_1;

OdomPub::OdomPub(const std::string &name) : Node(name)
{

    ign_pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("/world/empty/pose/info",10,std::bind(&OdomPub::msgCallback,this, _1));   
    odom_pub_ = create_publisher<geometry_msgs::msg::Pose>("/ego_pose",10);
    ego_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ego_vel",10);
    ego_vel_pub_ = create_publisher<geometry_msgs::msg::Accel>("/ego_accel",10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
  publishStaticTransform();  
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
    
    geometry_msgs::msg::TransformStamped egoTransform;
    egoTransform.header.stamp = this->now();
    egoTransform.header.frame_id = "map";// Global frame
    egoTransform.child_frame_id = "ego";// Ego vehicle frame
    egoTransform.transform.translation.x = ego_pose.position.x;
    egoTransform.transform.translation.y = ego_pose.position.y;
    egoTransform.transform.translation.z = ego_pose.position.z;
    egoTransform.transform.rotation = ego_pose.orientation;

    float dx, dy, vx,vy,ax,ay,dvx,dvy;
    dx=egoTransform.transform.translation.x - last_pose.transform.translation.x;
    dy=egoTransform.transform.translation.y - last_pose.transform.translation.y;
    rclcpp::Time current_time = egoTransform.header.stamp;
    rclcpp::Time last_time = last_pose.header.stamp;
    double dt = (current_time - last_time).seconds();  // returns double in seconds
    vx=dx/dt;
    vy=dy/dt;
    ego_vel.linear.x=vx;
    ego_vel.linear.y=vy;

    dvx=ego_vel.linear.x-last_vel.linear.x;
    dvy=ego_vel.linear.y-last_vel.linear.y;

    ax=dvx/dt;
    ay=dvy/dt;
    ego_accel.linear.x=ax;
    ego_accel.linear.y=ay;
    
    last_vel=ego_vel;
    last_pose=egoTransform;
    ego_vel_pub_->publish(ego_vel);
    ego_accel_pub_->publish(ego_accel);
    tf_broadcaster_->sendTransform(egoTransform);
    odom_pub_-> publish(ego_pose);
}

void OdomPub::publishStaticTransform()
{
    geometry_msgs::msg::TransformStamped static_transform;

    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "ego";
    static_transform.child_frame_id = "base_footprint";

    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;

    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(static_transform);
}

double OdomPub::roundToThreeDecimalPlaces(double value, int decimalPlaces) {
    double factor = std::pow(10.0, decimalPlaces);
    return std::round(value * factor) / factor;
}

// geometry_msgs::msg::Twist OdomPub::convertCmdVector(const geometry_msgs::msg::Twist &vel, const geometry_msgs::msg::Pose ego_pos){
//     geometry_msgs::msg::Twist vel_cmd;
//     float k_heading=0.9;
//     float theta= atan2(vel.linear.y,vel.linear.x);
//     double vx_local = cos(theta) * vel.linear.x + sin(theta) * vel.linear.y;
//     vel_cmd.linear.x = vx_local;
//     double heading_error = theta- ego_pos.orientation.z;
//     vel_cmd.angular.z = k_heading * heading_error;
//     return vel_cmd;
// }

int main (int argc, char* argv[])
{
rclcpp::init(argc,argv);
auto node=std::make_shared<OdomPub>("odom_pub");
node.last_vel.linear.x=0;
node.last_vel.linear.y=0;
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}