#include "rclcpp/rclcpp.hpp"
#include "car.hpp"
#include "road.hpp"
#include "drone_controller.hpp"
#include "road_visualization.hpp"
#include "velocity_visualization.hpp"

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
        controlled_car_->setVelocity(makeVel(2.0, 0.0));

        for (int i = 0; i < 1; ++i) {
            auto drone = std::make_shared<Car>("drone_" + std::to_string(i), false);
            drone->setPose(makePose(15.0 + i * 3.0, 3));
            drone->setVelocity(makeVel(1.0, -1.0));
            drones_.push_back(drone);
        }

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
        marker_pub_ = this->create_publisher<vis_marker_arr>("visualization_marker_array", 10);
        vo_marker_pub_ = this ->create_publisher<vis_marker_arr>("vo_marker_array", 10);

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

    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr vo_marker_pub_;
    
    geometry_msgs::msg::Pose makePose(double x, double y, double z = 0.2) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
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
            drones_[i]->update(dt);
            publishPose(*drones_[i]);
            publishTF(*drones_[i], "map", drones_[i]->getId());

        }

        // For now, keep ego car static or add logic here later
        controlled_car_->update(dt);
        publishPose(*controlled_car_);
        publishTF(*controlled_car_, "map", controlled_car_->getId());

        publishMarkers();
        publishVOMarkers();
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
        vis_marker_arr marker_array;

        // Drones
        int id = 0;
        for (const auto& car : drones_) {
            marker_array.markers.push_back(makeCarMarker(*car, id++));
            setVelocityArrowMarker(marker_array, *car, this->now(), id);
        }
        
        // Controlled car
        marker_array.markers.push_back(makeCarMarker(*controlled_car_, id));
        setVelocityArrowMarker(marker_array, *controlled_car_, this->now(), 0);
        setVelocityTextMarker(marker_array, *controlled_car_, this->now());

        // Road
        rclcpp::Time now = this->now();
        vis_marker road_marker;
        setRoadMarker(road_marker, road_, now);
        marker_array.markers.push_back(road_marker);
        
        // lane lines
        for (int i = 1; i < road_.getNumLanes(); ++i) {
            vis_marker lane_marker;
            setLaneMarker(lane_marker, road_, i, now);
            marker_array.markers.push_back(lane_marker);
        }

        marker_pub_->publish(marker_array);
    }

    vis_marker makeCarMarker(const Car& car, int id) {
        vis_marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "cars";
        marker.id = id;
        marker.type = vis_marker::CUBE;
        marker.action = vis_marker::ADD;

        marker.pose = car.getPose();
        marker.scale.x = 1.2; // length
        marker.scale.y = 0.8; // width
        marker.scale.z = 0.5; // height
        
        //calculating r_total (to vo calculation) as half the diagonal
        float r_ego = 1/2 * sqrt(pow(marker.scale.x,2) + pow(marker.scale.y,2));
        // In out case r_ego = r_obstacle
        float r_obstacle = r_ego;
        float r_total = r_ego + r_obstacle;

        if (car.isControlled()) {
            marker.color.r = 0.91;
            marker.color.g = 0.12;
            marker.color.b = 0.39;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        marker.color.a = 1.0;

        return marker;
    }

    void publishVOMarkers()
    {

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
