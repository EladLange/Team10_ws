#include "rclcpp/rclcpp.hpp"
#include "car.hpp"
#include "road.hpp"
#include "drone_controller.hpp"
#include "road_visualization.hpp"
#include "velocity_visualization.hpp"
#include "raceline_visualization.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using vis_marker_arr = visualization_msgs::msg::MarkerArray;
using vis_marker = visualization_msgs::msg::Marker;
using pose_msg = geometry_msgs::msg::Pose;
using twist_msg = geometry_msgs::msg::Twist;
using point_msg = geometry_msgs::msg::Point;

class EnvSimNode : public rclcpp::Node {
public:
    EnvSimNode()
    : Node("env_sim_node"),
      road_(3, 3.0, 200.0, 20.0), // 3 lanes, 3 meters wide, 200 meters long, radius 20 meters
      controller_(road_)
    {
        RCLCPP_INFO(this->get_logger(), "Starting environment simulation...");
        
        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
        ego_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ego_pose", 10);
        marker_pub_ = this->create_publisher<vis_marker_arr>("visualization_marker_array", 10);
        obstacles_pub_ = this->create_publisher<vis_marker_arr>("obstacles", 10);
        
        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&EnvSimNode::cmdVelCallback, this, std::placeholders::_1));
        
        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize cars
        controlled_car_ = std::make_shared<Car>("ego", true);
        controlled_car_->setPose(makePose(10.0, 0.0));  // Center of first lane
        controlled_car_->setVelocity(makeVel(1.0, 0.0));

        // Initialize obstacle cars
        initializeObstacleCars();
        
        // Timer for simulation updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&EnvSimNode::update, this));
    }

private:
    Road road_;
    DroneController controller_;
    std::shared_ptr<Car> controlled_car_;
    std::vector<std::shared_ptr<Car>> drones_;
    
    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr marker_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr obstacles_pub_;
    
    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void initializeObstacleCars() {
        // First obstacle 
        auto drone0 = std::make_shared<Car>("drone_0", false);
        drone0->setPose(makePose(30.0, 2.25));
        drone0->setVelocity(makeVel(1.0, 0.0));
        drones_.push_back(drone0);

        // Second obstacle
        auto drone1 = std::make_shared<Car>("drone_1", false);
        drone1->setPose(makePose(40.0, -2.25));
        drone1->setVelocity(makeVel(1.0, 0.0));
        drones_.push_back(drone1);

        // Third obstacle
        auto drone2 = std::make_shared<Car>("drone_2", false);
        drone2->setPose(makePose(40.0, 0.0));
        drone2->setVelocity(makeVel(0.0, 0.0));
        drones_.push_back(drone2);

        // Fourth obstacle
        auto drone3 = std::make_shared<Car>("drone_3", false);
        drone3->setPose(makePose(15.0, 0.0));
        drone3->setVelocity(makeVel(1.0, 0.0));
        drones_.push_back(drone3);

        // Fifth obstacle
        auto drone4 = std::make_shared<Car>("drone_4", false);
        drone4->setPose(makePose(20.0, 0.0));
        drone4->setVelocity(makeVel(0.5, 0.0));
        drones_.push_back(drone4);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Update ego car velocity based on VO algorithm output
        controlled_car_->setVelocity(*msg);
        
        // Update ego car's orientation based on the new velocity
        double yaw = std::atan2(msg->linear.y, msg->linear.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        controlled_car_->setOrientation(q);
    }

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
        for (auto& drone : drones_) {
            drone->update(dt);
            publishPose(*drone);
            publishTF(*drone, "map", drone->getId());
        }
        
        // Update ego car
        controlled_car_->update(dt);
        publishPose(*controlled_car_);
        publishTF(*controlled_car_, "map", controlled_car_->getId());
        
        // Publish ego pose specifically for VO node
        publishEgoPose(*controlled_car_);
        
        // Publish visualization markers
        publishMarkers();
        
        // Publish obstacle information for VO node
        publishObstacles();
    }

    void publishPose(const Car& car) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose = car.getPose();
        pose_pub_->publish(msg);
    }
    
    void publishEgoPose(const Car& car) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose = car.getPose();
        ego_pose_pub_->publish(msg);
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

        // Raceline
        std::vector<point_msg> raceline = setRaceline();
        visualizeRaceline(raceline, marker_array, this->now());

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
        
        // Lane lines
        for (int i = 1; i < road_.getNumLanes(); ++i) {
            vis_marker lane_marker;
            setLaneMarker(lane_marker, road_, i, now);
            marker_array.markers.push_back(lane_marker);
        }

        marker_pub_->publish(marker_array);
    }
    
    void publishObstacles() {
        vis_marker_arr obstacle_array;
        int id = 0;
        
        for (const auto& drone : drones_) {
            vis_marker marker = makeCarMarker(*drone, id++);
            obstacle_array.markers.push_back(marker);
        }
        
        obstacles_pub_->publish(obstacle_array);
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
        marker.scale = car.getCarScale();

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
    
    std::vector<point_msg> getRaceline() {
        // Implementation of raceline generation
        std::vector<point_msg> raceline = setRaceline();
        return raceline;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnvSimNode>());
    rclcpp::shutdown();
    return 0;
}
