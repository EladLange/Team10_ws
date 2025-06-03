#include "rclcpp/rclcpp.hpp"
#include "common/car.hpp"
#include "road.hpp"
#include "drone_controller.hpp"
#include "road_visualization.hpp"
#include "velocity_visualization.hpp"
#include "vo_visualization.hpp"
#include "velocity_obstacle.hpp"
#include "raceline_visualization.hpp"
#include "global_variables.hpp"
#include "nlvo/nlvo.hpp"
#include "nlvo/nlvo_visualization.hpp"
#include "pscav/pscav.hpp"
#include "common/settings.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

VelocityObstacle vo;
NLVO nlvo;
PSCAV pscav;
// std::vector<Car> obsVehicles; 
// std::vector<Car> egos;

// Global variables
float time_horizon = 10.0f;
float max_acceleration = 4.0f;
float time_step = 1.0f;

const int num_obstacles = 9*3; // Number of obstacles


bool road_init=true;

class CarSimulationNode : public rclcpp::Node {
public:
    CarSimulationNode()
    : Node("car_simulation_node"),
      road_(3, 5.0, 200.0, 20.0), // 3 lanes, 3 meters wide, 100 meters long, radius 20 meters
      controller_(road_)
    {
        RCLCPP_INFO(this->get_logger(), "Starting car simulation...");


        initialize_cars();

        // publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
        marker_pub_ = this->create_publisher<vis_marker_arr>("visualization_marker_array", 10);
        vo_marker_pub_ = this ->create_publisher<vis_marker_arr>("vo_marker_array", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmd", 10);
        nlvo_marker_pub_ = this->create_publisher<vis_marker_arr>("nlvo_marker_array", 10);
    
        // subscribers
        ego_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/ego_vel",10,std::bind(&CarSimulationNode::egoVelCallback,this, _1));
        ego_pos_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/ego_pose",10,std::bind(&CarSimulationNode::egoPosCallback,this, _1));
        obstacles_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/obstacles_poses",10,std::bind(&CarSimulationNode::obstaclePosCallback,this, _1));
        obstacles_vel_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/obstacles_vels",10,std::bind(&CarSimulationNode::obstacleVelCallback,this, _1));


        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        //     "drone_pose", 10,
        //     [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
        //         this->dronePoseCallback(msg);
        //     }
        // );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(17),
            std::bind(&CarSimulationNode::update, this));
    }

    // void dronePoseCallback(geometry_msgs::msg::PoseArray::SharedPtr msg) {
    //     // Handle the incoming drone pose array message
    //     // RCLCPP_INFO(this->get_logger(), "Received drone pose array with %zu drones", msg->poses.size());

    //     // Update the drones with the received poses
    //     for (size_t i = 0; i < msg->poses.size() && i < drones_.size(); ++i) {
    //         drones_[i]->setPose(msg->poses[i]);
    //         // RCLCPP_INFO(this->get_logger(), "Updated drone %zu pose: (%f, %f, %f)",
    //                 //    i, msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
    //     }
    // }

    void initialize_cars()
    {
        std::vector<point_msg> obs_xyz;
        std::vector<point_msg> obs_s;

        std::string filename ="/home/zvi/Desktop/Team10_ws/src/vo/src/track_points - Copy.csv";
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening raceline file." << std::endl;
        }
        std::string line;   // Skip the first line (header)
        std::getline(file , line);
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            point_msg point;
            point_msg s_point;
            std::string token;
            // Read x, y, z, s values from the line
            std::getline(iss, token, ',');
            point.x = std::stod(token);
            std::getline(iss, token, ',');
            point.y = std::stod(token);
            std::getline(iss, token, ',');
            point.z =0.0; //std::stod(token);
            std::getline(iss, token, ',');
            s_point.x = std::stod(token);
            obs_xyz.push_back(point);
            obs_s.push_back(s_point);
        }
        file.close();

        // create temp raceline for the obstacles
        // float di = 0.05f;
        // for (float i =0; i<300; i+=di)
        // {
        //     point_msg point;
        //     point_msg s_point;
        //     point.x = 10.0f + i;
        //     point.y = 0.0f;
        //     point.z = 0.5f;
        //     s_point.x = point.x;
        //     obs_xyz.push_back(point);
        //     obs_s.push_back(s_point);
        // }

        // print the raceline
        // for (int i = 0; i < obs_xyz.size(); ++i)
        // {
        //     std::cout<<"obs_xyz: "<<obs_xyz[i].x<<", "<<obs_xyz[i].y<<", "<<obs_xyz[i].z<<std::endl;
        //     std::cout<<"obs_s: "<<obs_s[i].x<<std::endl;
        // }

        // Initialize cars
        controlled_car_ = std::make_shared<Car>("ego", true);
        controlled_car_->setPose(makePose(0.0, 0.0));  // Center of first lane
        controlled_car_->setVelocity(makeVel(0.0, 0.0));
        controlled_car_->setRaceline(obs_xyz);
        controlled_car_->setSValues(obs_s);
        
        for (int i=0; i < 3; ++i)
        {   
            // std::vector <point_msg> car_xyz;
            // std::vector <point_msg> car_s;
            for (int j=0;j<(num_obstacles/3);j++){
                auto obs = std::make_shared<Car>("obs_"+std::to_string((i+1)*(j+1)), false);
                obs->setPose(makePose(0.0, 0.0));
                obs->setVelocity(makeVel(0.0, 0.0));
                obs->setRaceline(obs_xyz);
                obs->setSValues(obs_s);
                obstacles_.push_back(obs);  

            }
      
        }

        
        //First obstacle
        // auto obs_0 = std::make_shared<Car>("obs_0", false);
        // obs_0->setPose(makePose(35.0, 0.0));
        // obs_0->setVelocity(makeVel(10.0, 0.0));
        // obs_0->setRaceline(obs_xyz);
        // obs_0->setSValues(obs_s);
        // obstacles_.push_back(obs_0);
        // // Second obstacle
        // auto obs_1 = std::make_shared<Car>("obs_1", false);
        // obs_1->setPose(makePose(20.0, 0.0));
        // obs_1->setVelocity(makeVel(10.0, 0.0));
        // obs_1->setRaceline(obs_xyz);
        // obs_1->setSValues(obs_s);
        // obstacles_.push_back(obs_1);
        // // Third obstacle
        // auto obs_2 = std::make_shared<Car>("obs_2", false);
        // obs_2->setPose(makePose(20.0, 0.0));
        // obs_2->setVelocity(makeVel(10.0, 0.0));
        // obs_2->setRaceline(obs_xyz);
        // obs_2->setSValues(obs_s);
        // obstacles_.push_back(obs_2);
        // // Fourth obstacle
        // auto obs_3 = std::make_shared<Car>("obs_3", false);
        // obs_3->setPose(makePose(30.0, 0.0));
        // obs_3->setVelocity(makeVel(10.0, 0.0));
        // obs_3->setRaceline(obs_xyz);
        // obs_3->setSValues(obs_s);
        // obstacles_.push_back(obs_3);
        // // Fifth obstacle
        // auto obs_4 = std::make_shared<Car>("obs_4", false);
        // obs_4->setPose(makePose(30.0, 0.0));
        // obs_4->setVelocity(makeVel(10.0, 0.0));
        // obs_4->setRaceline(obs_xyz);
        // obs_4->setSValues(obs_s);
        // obstacles_.push_back(obs_4);
        // // Sixth obstacle
        // auto obs_5 = std::make_shared<Car>("obs_5", false);
        // obs_5->setPose(makePose(40.0, 0.0));
        // obs_5->setVelocity(makeVel(10.0, 0.0));
        // obs_5->setRaceline(obs_xyz);
        // obs_5->setSValues(obs_s);
        // obstacles_.push_back(obs_5);
        // // Seventh obstacle
        // auto obs_6 = std::make_shared<Car>("obs_6", false);
        // obs_6->setPose(makePose(40.0, 0.0));
        // obs_6->setVelocity(makeVel(10.0, 0.0));
        // obs_6->setRaceline(obs_xyz);
        // obs_6->setSValues(obs_s);
        // obstacles_.push_back(obs_6);
        // // Eighth obstacle
        // auto obs_7 = std::make_shared<Car>("obs_7", false);
        // obs_7->setPose(makePose(50.0, 0.0));
        // obs_7->setVelocity(makeVel(10.0, 0.0));
        // obs_7->setRaceline(obs_xyz);
        // obs_7->setSValues(obs_s);
        // obstacles_.push_back(obs_7);
        // // Ninth obstacle
        // auto obs_8 = std::make_shared<Car>("obs_8", false);
        // obs_8->setPose(makePose(50.0, 0.0));
        // obs_8->setVelocity(makeVel(10.0, 0.0));
        // obs_8->setRaceline(obs_xyz);
        // obs_8->setSValues(obs_s);
        // obstacles_.push_back(obs_8);
        
        // //Eighth obstacle
        // auto drone8 = std::make_shared<Car>("drone_8", false);
        // drone8->setPose(makePose(80.0,0.0));
        // drone8->setVelocity(makeVel(1.0, 0.0));
        // obstacles_.push_back(drone8);
        // drone8->setRaceline(obs_xyz);
        // drone8->setSValues(obs_s);

    }


    point_msg findNextGoalPoint(const std::vector<point_msg>& raceline, const pose_msg& ego_pose)
    {
        int lookahead_step = 10;
        point_msg point;

        // fallback if raceline is empty
        if (raceline.empty())
        {
            //std::cout<<"Raceline is empty"<<std::endl;
            point.x = ego_pose.position.x;
            point.y = ego_pose.position.y;
            point.z = ego_pose.position.z;
            return point;

        }

        // Find closest point that is in front of ego
        int closest_index = 0;
        double min_dist_squared = std::numeric_limits<double>::max();

        // Iterate through the raceline points
        for (size_t i = 0; i < raceline.size(); ++i)
        {
            const auto& raceline_point = raceline[i];
            double dx = ego_pose.position.x - raceline_point.x;
            double dy = ego_pose.position.y - raceline_point.y;

            double squar_dist = dx * dx + dy * dy;

            if (squar_dist < min_dist_squared)
            {
                min_dist_squared = squar_dist;
                closest_index = static_cast<int>(i);
            }
        }

        // Compute the lookahead distance
        int lookahead_index = closest_index + lookahead_step;

        // Clamp to raceline size
        if (lookahead_index >= static_cast<int>(raceline.size()))
        {
            lookahead_index = static_cast<int>(raceline.size()) - 1;
            // RCLCPP_INFO(get_logger(), "Lookahead index is out of bounds");
        }
        // RCLCPP_INFO(this->get_logger(), "Lookahead index: %d", lookahead_index);
        return raceline[lookahead_index];
    }

private:
    Road road_;
    DroneController controller_;
    std::shared_ptr<Car> controlled_car_;
    std::vector<Obstacle> obsVehicles; 
    // std::vector<std::shared_ptr<Car>> drones_;
    std::vector<std::shared_ptr<Car>> obstacles_;

    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr vo_marker_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr nlvo_marker_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ego_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ego_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_vel_sub_;


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
        // RCLCPP_INFO(this->get_logger(), "Ego car velocity set to: (%f, %f)", msg->linear.x, msg->linear.y);
    }

    void egoPosCallback(const pose_msg msg)
    {
        controlled_car_->setPose(msg);
        //RCLCPP_INFO(this->get_logger(), "Ego car velocity set to: (%f, %f)", msg->linear.x, msg->linear.y);
    }

    void obstaclePosCallback(const geometry_msgs::msg::PoseArray msg)
    {   
        for (size_t i=0; i<obstacles_.size();i++){
        obstacles_[i]->setPose(msg.poses[i]);
        }
    }

    void obstacleVelCallback(const geometry_msgs::msg::PoseArray msg)
    {   
        for (size_t i=0; i<obstacles_.size();i++){
        geometry_msgs::msg::Pose pose = msg.poses[i];
        obstacles_[i]->setVelocity(makeVel(pose.position.x,pose.position.y));
        }
    }

    void update() {
        double dt = 0.017;  // 100 ms
        obsVehicles.clear();

        // Update drones
        for (size_t i = 0; i < obstacles_.size(); ++i) {
        //    controller_.control(*drones_[i], static_cast<int>(i));
            publishPose(*obstacles_[i]);
            publishTF(*obstacles_[i], "map", obstacles_[i]->getId());
            // obstacles_[i]->update(dt);
            // RCLCPP_INFO(this->get_logger(), "Drone %s updated to pose: (%f, %f, %f)",
            //             obstacles_[i].getId().c_str(),
            //             obstacles_[i].getPose().position.x,
            //             obstacles_[i].getPose().position.y,
            //             obstacles_[i].getPose().position.z);

            // Create obstacle struct for this drone
            Obstacle obstacle;
            obstacle.id=obstacles_[i]->getId(); 
            obstacle.pose =obstacles_[i]->getPose();
            obstacle.velocity =obstacles_[i]->getVelocity();
            obstacle.raceline=obstacles_[i]->getRaceline(); 
            obstacle.s_values=obstacles_[i]->getSValues();
            obsVehicles.push_back(obstacle);
        }

        // Get ego car's current pose and velocity
        auto ego_pose = controlled_car_->getPose();
        auto ego_vel = controlled_car_->getVelocity();

        std::vector<point_msg> raceline = buildRaceline();
        point_msg goal_point = findNextGoalPoint(raceline, ego_pose);
        float r_total = calculateTotalRadius();

        twist_msg new_ego_velocity = vo.selectBestVelocity(ego_pose, ego_vel, obstacles_, goal_point, r_total);
        // twist_msg new_ego_velocity = nlvo.selectBestVelocity(ego_pose, ego_vel, obsVehicles, goal_point, r_total);
        // twist_msg new_ego_velocity = pscav.selectBestVelocity(ego_pose, ego_vel, obstacle_poses, obstacle_velocities, goal_point, r_total);
        // twist_msg new_ego_velocity = makeVel(5.0,0.0);
        // Set the new velocity for the ego car
        // controlled_car_->setVelocity(new_ego_velocity);
        // Publish the new velocity
        // RCLCPP_INFO(this->get_logger(), "New ego car velocity set to: (%f, %f)", new_ego_velocity.linear.x, new_ego_velocity.linear.y);
        cmd_vel_pub_->publish(new_ego_velocity);

        // Update ego car's orientation based on the new velocity
        // double yaw = std::atan2(new_ego_velocity.linear.y, new_ego_velocity.linear.x);
        // tf2::Quaternion q;
        // q.setRPY(0, 0, yaw);
        // controlled_car_->setOrientation(q);
        //RCLCPP_INFO(this->get_logger(), "Ego car orientation set to: %f", yaw);

        // Update ego car's position based on the new velocity
        // controlled_car_->update(dt);
        // publishPose(*controlled_car_);
        // publishTF(*controlled_car_, "map", controlled_car_->getId());


        publishMarkers();
        publishVOMarkers();
        // publishNLVOMarkers();
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
        std::vector<point_msg> raceline = buildRaceline();
        visualizeRaceline(raceline, marker_array, this->now());

        // Obstacles
        int id = 0;
        for (const auto& car : obstacles_) {
            marker_array.markers.push_back(makeCarMarker(*car, id++));
            setVelocityArrowMarker(marker_array, *car, this->now(), "map",id);
        }


        // Controlled car
        // marker_array.markers.push_back(makeCarMarker(*controlled_car_, id));
        setVelocityArrowMarker(marker_array, *controlled_car_, this->now(), "map" , 0);
        setVelocityTextMarker(marker_array, *controlled_car_, this->now());

        // Road
        //rclcpp::Time now = this->now();
        vis_marker road_marker;
        road_marker.header.frame_id = "map";
        road_marker.header.stamp = this->now();   
        road_marker.ns="Road";
        road_marker.id=0;
        road_marker.type= vis_marker::MESH_RESOURCE;
        road_marker.mesh_resource = "package://vo/meshes/track.STL";
        road_marker.action = vis_marker::ADD;
        road_marker.scale.x=1.0;
        road_marker.scale.y=1.0;
        road_marker.scale.z=1.0;
        road_marker.color.r=0.0;
        road_marker.color.g=0.0;
        road_marker.color.b=0.0;
        road_marker.color.a=1.0;
        road_marker.pose.position.x=road_marker.pose.position.y=road_marker.pose.position.z=0.0;
        road_marker.pose.orientation.x=road_marker.pose.orientation.y=road_marker.pose.orientation.z=0.0;
        road_marker.pose.orientation.w=1.0;
        // setRoadMarker(road_marker, road_, now);
        marker_array.markers.push_back(road_marker);
        
        // lane lines
        // for (int i = 1; i < road_.getNumLanes(); ++i) {
        //     vis_marker lane_marker;
        //     setLaneMarker(lane_marker, road_, i, now);
        //     marker_array.markers.push_back(lane_marker);
        // }

        marker_pub_->publish(marker_array);
    }

    // Help function to get the scale of the car marker
    geometry_msgs::msg::Vector3 getCarScale()
    {
        geometry_msgs::msg::Vector3 scale;
        scale.x =3.0; // length
        scale.y =1.5; // width
        scale.z =1.0; // height
        return scale;
    }

    float calculateTotalRadius() {
        auto scale = getCarScale();
        float r_ego =0.5f * std::sqrt(std::pow(scale.x, 2) + std::pow(scale.y, 2));
        float r_obstacle = r_ego;
        float r_total = r_ego + r_obstacle;
        return r_total;
    }

    vis_marker makeCarMarker(const Car& car, int id) {
        vis_marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "cars";
        marker.id = id;
        // marker.type = vis_marker::CUBE;
        marker.type= vis_marker::MESH_RESOURCE;
        marker.mesh_resource = "package://vo/meshes/obstacle.STL";
        marker.action = vis_marker::ADD;
        // Set the pose of the marker to the car's pose
        marker.pose = car.getPose();
        // Set the size of the marker to the car's size
        // marker.scale = getCarScale();
        marker.scale.x =1.0;
        marker.scale.y =1.0;
        marker.scale.z =1.0;      
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
        float r_total = calculateTotalRadius();

        for (const auto& obstacle : obstacles_) {

            vis_marker cone_marker;
            // Set the properties of the cone marker
            setVOConeMarker(cone_marker, ego_pose, *obstacle, r_total);
            cone_marker.id = id++;
            marker_array.markers.push_back(cone_marker);
        }

        // // for debugging: show the candidate velocities
        // std::vector<twist_msg> candidate_velocities = vo.generateCandidateVelocities(ego_vel);
        // for (const auto& candidate_velocity : candidate_velocities) {
        //     vis_marker candidate_marker;
        //     // Set the properties of the candidate marker
        //     setCandidateMarker(candidate_marker, candidate_velocity);
        //     candidate_marker.id = id++;
        //     marker_array.markers.push_back(candidate_marker);
        // }

        setVelocityArrowMarker(marker_array, *controlled_car_, this->now(), "ego", 0);
        vo_marker_pub_->publish(marker_array);
    }

    void publishNLVOMarkers()
    {
       vis_marker_arr marker_array;

       int id = 0;  // Marker ID counter
       auto ego_pose = controlled_car_->getPose();
       auto ego_vel = controlled_car_->getVelocity();
       float r_total = calculateTotalRadius();
       for (const auto& obstacle : obsVehicles) {
            float time_horizon = nlvo.computeMinimumTimeHorizon(ego_pose, ego_vel, obstacle, r_total, nlvo.control_set) + 2.0f;

            int trajectory_index = nlvo.findTrajectoryIndex(obstacle);
            std::vector<VelDisk> disks = nlvo.generateNLVODisks(ego_pose, obstacle, trajectory_index, r_total, time_horizon);  

            // Visualize each disk
            for (auto &disk : disks)
            {
                // Shift NLVO disk centers to base_link frame (ego-relative velocity space)
                disk.cx = disk.cx - ego_vel.linear.x;
                disk.cy = disk.cy - ego_vel.linear.y;
                
                // Visualize
                vis_marker disk_marker;
                setNLVODiskMarker(disk_marker, disk, id++);
                marker_array.markers.push_back(disk_marker);
            }

        setVelocityArrowMarker(marker_array, *controlled_car_, this->now(), "ego", 0);
       }

        // for debugging: show the candidate velocities
        std::vector<twist_msg> candidate_velocities = nlvo.generateCandidateVelocities(ego_vel);
        for (const auto& candidate_velocity : candidate_velocities) {
            vis_marker candidate_marker;
            // Set the properties of the candidate marker
            setCandidateMarker(candidate_marker, candidate_velocity);
            candidate_marker.id = id++;
            marker_array.markers.push_back(candidate_marker);
        }

    nlvo_marker_pub_->publish(marker_array);
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
