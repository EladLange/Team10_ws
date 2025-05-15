#include "rclcpp/rclcpp.hpp"
#include "car.hpp"
#include "road.hpp"
#include "drone_controller.hpp"
#include "road_visualization.hpp"
#include "velocity_visualization.hpp"
#include "vo_visualization.hpp"
#include "velocity_obstacle.hpp"
#include "raceline_visualization.hpp"
#include "global_variables.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

VelocityObstacle vo;

// Global variables
float time_horizon = 10.0f;
float max_acceleration = 1.0f;
float min_acceleration = -3.0f;
float time_step = 1.0f;
float delta_t = 0.1f;

class CarSimulationNode : public rclcpp::Node {
public:
    CarSimulationNode()
    : Node("car_simulation_node"),
      road_(3, 3.0, 200.0, 20.0), // 3 lanes, 3 meters wide, 100 meters long, radius 20 meters
      controller_(road_)
    {
        RCLCPP_INFO(this->get_logger(), "Starting car simulation...");


        intilize_cars();
        // poblishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
        marker_pub_ = this->create_publisher<vis_marker_arr>("visualization_marker_array", 10);
        vo_marker_pub_ = this ->create_publisher<vis_marker_arr>("vo_marker_array", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // subscribers:
        ego_vel_sub_ = this->create_subscription<twist_msg>("ego_velocity", 10, std::bind(&CarSimulationNode::egoVelCallback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CarSimulationNode::update, this));
    }


    void intilize_cars()
    {
                // Initialize cars
                controlled_car_ = std::make_shared<Car>("ego", true);
                controlled_car_->setPose(makePose(10.0, 0.0));  // Center of first lane
                controlled_car_->setVelocity(makeVel(1.0, 0.0));
        
                // // First obstacle 
                auto drone0 = std::make_shared<Car>("drone_0", false);
                drone0->setPose(makePose(30.0, 3.3));
                drone0->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone0);
        
                // // Second obstacle
                auto drone1 = std::make_shared<Car>("done_1", false);
                drone1->setPose(makePose(40.0, -3.3));
                drone1->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone1);
        
                // Third obstacle
                auto drone2 = std::make_shared<Car>("drone_2", false);
                drone2->setPose(makePose(60.0, 0.0));
                drone2->setVelocity(makeVel(0.0, 0.0));
                drones_.push_back(drone2);
        
                // Fourth obstacle
                auto drone3 = std::make_shared<Car>("drone_3", false);
                drone3->setPose(makePose(50.0, 0.0));
                drone3->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone3);
        
                // // Fifth obstacle
                // auto drone4 = std::make_shared<Car>("drone_4", false);
                // drone4->setPose(makePose(20.0, 0.0));
                // drone4->setVelocity(makeVel(0.5, 0.0));
                // drones_.push_back(drone4);
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ego_vel_sub_;
    
    pose_msg makePose(double x, double y, double z = 0.5) {
        pose_msg pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.w = 1.0;
        return pose;
    }
    twist_msg makeVel(double x, double y) {
        twist_msg vel;
        vel.linear.x = x;
        vel.linear.y = y;
        vel.linear.z = 0.0;
        return vel;
    }

    void egoVelCallback(const shared_ptr msg) 
    {
        controlled_car_->setVelocity(*msg);
        RCLCPP_INFO(this->get_logger(), "Ego car velocity set to: (%f, %f)", msg->linear.x, msg->linear.y);
    }
    
    void update() {
        double dt = 0.1;  // 100 ms
        std::vector<pose_msg> obstacle_poses;
        std::vector<twist_msg> obstacle_velocities;

        // Update drones
        for (size_t i = 0; i < drones_.size(); ++i) {
           // controller_.control(*drones_[i], static_cast<int>(i));
            drones_[i]->update(dt);
            publishPose(*drones_[i]);
            publishTF(*drones_[i], "map", drones_[i]->getId());
            obstacle_poses.push_back(drones_[i]->getPose());
            obstacle_velocities.push_back(drones_[i]->getVelocity());
            //RCLCPP_INFO(this->get_logger(), "Drone %zu position: (%f, %f)", i, drones_[i]->getPose().position.x, drones_[i]->getPose().position.y);
            //RCLCPP_INFO(this->get_logger(), "Drone %zu velocity: (%f, %f)", i, drones_[i]->getVelocity().linear.x, drones_[i]->getVelocity().linear.y);
        }
        
        // Get ego car's current pose and velocity
        auto ego_pose = controlled_car_->getPose();
        auto ego_vel = controlled_car_->getVelocity();

        //RCLCPP_INFO(this->get_logger(), "Ego car position: (%f, %f)", ego_pose.position.x, ego_pose.position.y);
        //RCLCPP_INFO(this->get_logger(), "Ego car velocity: (%f, %f)", ego_vel.linear.x, ego_vel.linear.y);

        std::vector<point_msg> raceline = setRaceline();
        float r_total = calculateTotalRadius();
        twist_msg new_ego_velocity = vo.selectBestVelocity(ego_pose, ego_vel, obstacle_poses, obstacle_velocities, raceline, r_total);
        
        // Set the new velocity for the ego car
        controlled_car_->setVelocity(new_ego_velocity);
        // Publish the new velocity
        cmd_vel_pub_->publish(new_ego_velocity);

        // Update ego car's orientation based on the new velocity
        double yaw = std::atan2(new_ego_velocity.linear.y, new_ego_velocity.linear.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        controlled_car_->setOrientation(q);
        RCLCPP_INFO(this->get_logger(), "Ego car orientation: (%f, %f, %f, %f)", q.x(), q.y(), q.z(), q.w());

        // Update ego car's position based on the new velocity
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

        //raceline
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
        
        // lane lines
        for (int i = 1; i < road_.getNumLanes(); ++i) {
            vis_marker lane_marker;
            setLaneMarker(lane_marker, road_, i, now);
            marker_array.markers.push_back(lane_marker);
        }

        marker_pub_->publish(marker_array);
    }

    // Help function to get the scale of the car marker
    geometry_msgs::msg::Vector3 getCarScale()
    {
        geometry_msgs::msg::Vector3 scale;
        scale.x = 3.0; // length
        scale.y = 1.5; // width
        scale.z = 1.0; // height
        return scale;
    }

    float calculateTotalRadius() {
        auto scale = getCarScale();
        float r_ego = 0.5f * std::sqrt(std::pow(scale.x, 2) + std::pow(scale.y, 2));
        float r_obstacle = r_ego;  
        return r_ego + r_obstacle;
    }
   

    vis_marker makeCarMarker(const Car& car, int id) {
        vis_marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "cars";
        marker.id = id;
        marker.type = vis_marker::CUBE;
        marker.action = vis_marker::ADD;
        // Set the pose of the marker to the car's pose
        marker.pose = car.getPose();
        // Set the size of the marker to the car's size
        marker.scale = getCarScale();
        // calculate r_total for the car and obstacle - maybe not needed
        //float r_total = calculateTotalRadius();

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
        vis_marker_arr marker_array;
        vis_marker marker;

        int id = 0;  // Marker ID counter
        auto ego_pose = controlled_car_->getPose();
        auto ego_vel = controlled_car_->getVelocity();
        // check: maybe not needed  
        //auto scale = getCarScale();
        float r_total = calculateTotalRadius();
        //RCLCPP_INFO(this->get_logger(), "Total radius: %f", r_total);
        
        for (const auto& drone : drones_) {
            auto obstacle_pose = drone->getPose();
            auto obstacle_vel = drone->getVelocity();
            // check - maybe not needed
            //float dist = vo.distance(ego_pose, obstacle_pose);
            //RCLCPP_INFO(this->get_logger(), "Distance to drone: %f", dist);
            
            vis_marker cone_marker;
            // Set the properties of the cone marker
            setVOConeMarker(cone_marker, ego_pose, obstacle_pose, ego_vel, obstacle_vel, r_total);
            cone_marker.id = id++;
            marker_array.markers.push_back(cone_marker);
           // RCLCPP_INFO(this->get_logger(), "Number of points in cone marker: %zu", cone_marker.points.size());
        }

        //for debugging: show the candidate velocities
        std::vector<twist_msg> candidate_velocities = vo.generateCandidateVelocities(ego_vel);
        for (const auto& candidate_velocity : candidate_velocities) {
            vis_marker candidate_marker;
            // Set the properties of the candidate marker
            setCandidateMarker(candidate_marker, ego_pose, candidate_velocity, r_total, 5.0);
            candidate_marker.id = id++;
            marker_array.markers.push_back(candidate_marker);
        }

        vo_marker_pub_->publish(marker_array);
        //RCLCPP_INFO(this->get_logger(), "Published %zu markers", marker_array.markers.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
