#include "rclcpp/rclcpp.hpp"
#include "car.hpp"
#include "Road.hpp"
#include "drone_controller.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


class CarSimulationNode : public rclcpp::Node {
public:
    CarSimulationNode()
    : Node("car_simulation_node"),
      road_(3, 3.0, 100.0), // 3 lanes, 3 meters wide, 100 meters long
      controller_(road_)
    {
        RCLCPP_INFO(this->get_logger(), "Starting car simulation...");

        // Initialize cars
        controlled_car_ = std::make_shared<Car>("ego", true);
        controlled_car_->setPose(makePose(10.0, 0.0));  // Center of first lane

        for (int i = 0; i < 3; ++i) {
            auto drone = std::make_shared<Car>("drone_" + std::to_string(i), false);
            drone->setPose(makePose(0.0 + i * 3.0, 5.0 * (1)));  // One in each lane
            drone->setVelocity(makeVel(5.0,0.0));
            drones_.push_back(drone);
        }

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CarSimulationNode::update, this));
    }

private:
    Road road_;
    DroneController controller_;
    std::shared_ptr<Car> controlled_car_;
    std::vector<std::shared_ptr<Car>> drones_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose makePose(double x, double y) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }
    geometry_msgs::msg::Twist makeVel(double x, double y) {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = x;
        vel.linear.y = y;
        vel.linear.z = 0.0;
        return vel;
    }
    void update() {
        double dt = 0.1;  // 100 ms

        // Update drones
        for (size_t i = 0; i < drones_.size(); ++i) {
           // controller_.control(*drones_[i], static_cast<int>(i));
            drones_[i]->update(0.1);
            publishPose(*drones_[i]);
            publishTF(*drones_[i], "map", drones_[i]->getId());

        }

        // For now, keep ego car static or add logic here later
        controlled_car_->update(0.1);
        publishPose(*controlled_car_);
        publishTF(*controlled_car_, "map", controlled_car_->getId());

        publishMarkers();
    }

    void publishPose(const Car& car) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose = car.getPose();
        pose_pub_->publish(msg);
    }

    void publishTF(const Car& car, const std::string& parent_frame, const std::string& child_frame) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = parent_frame;
        tf_msg.child_frame_id = child_frame;
    
        tf_msg.transform.translation.x = car.getPose().position.x;
        tf_msg.transform.translation.y = car.getPose().position.y;
        tf_msg.transform.translation.z = car.getPose().position.z;
        tf_msg.transform.rotation = car.getPose().orientation;
    
        tf_broadcaster_->sendTransform(tf_msg);
    }
    

    void publishMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;

        int id = 0;
        for (const auto& car : drones_) {
            marker_array.markers.push_back(makeCarMarker(*car, id++));
        }
        marker_array.markers.push_back(makeCarMarker(*controlled_car_, id));

        marker_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker makeCarMarker(const Car& car, int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "cars";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = car.getPose();
        marker.scale.x = 1.2;
        marker.scale.y = 0.8;
        marker.scale.z = 0.5;

        if (car.isControlled()) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        marker.color.a = 1.0;

        return marker;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
