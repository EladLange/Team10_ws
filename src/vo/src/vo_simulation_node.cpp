#include "rclcpp/rclcpp.hpp" // ROS2 client library
#include "car.hpp" // Car class
#include "road.hpp" // Road class
#include "drone_controller.hpp" // DroneController class
#include "road_visualization.hpp" // RoadVisualization class
#include "velocity_visualization.hpp" // VelocityVisualization class
#include "vo_visualization.hpp" // VOVisualizbstacle class
#include "raceline_visualization.hpp" // Raceliation class
#include "velocity_obstacle.hpp" // VelocityOneVisualization class
#include "global_variables.hpp" // Global variables
#include "nlvo/nlvo.hpp" // NLVO class
#include "nao/nao.hpp" // NAO class
#include "purepursuit.hpp" // PurePursuitController
#include "nlvo/nlvo_visualization.hpp"
#include "pscav/pscav.hpp"


#include <geometry_msgs/msg/pose_stamped.hpp> // PoseStamped message type
#include <geometry_msgs/msg/twist_stamped.hpp> // TwistStamped message type
#include <geometry_msgs/msg/accel.hpp> // Accel message type
#include <visualization_msgs/msg/marker_array.hpp> // MarkerArray message type
#include <tf2_ros/transform_broadcaster.h> // For publishing transforms
#include <geometry_msgs/msg/transform_stamped.hpp> // TransformStamped message type
#include <geometry_msgs/msg/pose_array.hpp> // PoseArray message type
#include <visualization_msgs/msg/marker.hpp> // Marker message type
#include <tf2/utils.hpp> // For quaternion math (yaw to quaternion)
#include <rclcpp/parameter.hpp> // For handling parameters
#include "rcl_interfaces/msg/set_parameters_result.hpp" // For handling parameters


VelocityObstacle vo;
NLVO nlvo;
NAO nao;
PSCAV pscav;

// Global variables
float time_horizon = 7.0f;
float max_acceleration = 1.0f;
float min_acceleration = -3.0f;
float time_step = 1.0f;
float delta_t = 0.1f;

// Pure pursuit parameters
double base_lookahead = 2.0;
double min_lookahead = 2.0;
double max_lookahead = 7.0;
double max_angular_z = 0.5;

double dt = 0.1;  // 100 ms


class CarSimulationNode : public rclcpp::Node {
public:
    CarSimulationNode()
    : Node("car_simulation_node"),
     road_(3, 5.0, 200.0, 20.0), // 3 lanes, 3 meters wide, 100 meters long, radius 20 meters
     controller_(road_),
     purepursuit_controller_(base_lookahead, min_lookahead, max_lookahead, max_angular_z)
    {
        RCLCPP_INFO(this->get_logger(), "Starting car simulation...");


        initialize_cars();

        purepursuitInit(); // Initialize pure pursuit car

        /**
         * @brief change the control mode from the terminal:
         * @ros2 param set /car_simulation_node control_mode VO
         * ros2 param set /car_simulation_node control_mode NLVO
         * ros2 param set /car_simulation_node control_mode NAO
        */
        declare_parameter<std::string>("control_mode", "VO"); // VO / NLVO / NAO (default: NLVO)
        control_mode_ = get_parameter("control_mode").as_string(); // VO / NLVO / NAO
        // Print the control mode - debug
        RCLCPP_INFO(this->get_logger(), "Control mode: %s", control_mode_.c_str());
        


        // publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
        marker_pub_ = this->create_publisher<vis_marker_arr>("visualization_marker_array", 10);
        vo_marker_pub_ = this ->create_publisher<vis_marker_arr>("vo_marker_array", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmd", 10);
        cmd_accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("accel_cmd", 10);
        pp_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("pp_cmd_vel", 10);
        nlvo_marker_pub_ = this->create_publisher<vis_marker_arr>("nlvo_marker_array", 10);
        
        // subscribers
        ego_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/ego_vel",10,std::bind(&egoVelCallback,this, _1));
        ego_pos_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/ego_pose",10,std::bind(&egoPosCallback,this, _1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "drone_pose", 10,
            [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
                this->dronePoseCallback(msg);
            }
        );

        drone_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "drone_vel", 10,
            [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                this->droneVelCallback(msg);
            }
        );
        // Add callback for dynamic parameters 
        param_callback_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
                for (const auto &param : params) {
                    if (param.get_name() == "control_mode") {
                        this->control_mode_ = param.as_string();
                        RCLCPP_INFO(this->get_logger(), "Switched control mode to: %s", control_mode_.c_str());
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "";
                return result;
            }
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CarSimulationNode::update, this));
    }

    void dronePoseCallback(geometry_msgs::msg::PoseArray::SharedPtr msg) {
        // Handle the incoming drone pose array message
        // RCLCPP_INFO(this->get_logger(), "Received drone pose array with %zu drones", msg->poses.size());

        // Update the drones with the received poses
        for (size_t i = 0; i < msg->poses.size() && i < drones_.size(); ++i) {
            drones_[i]->setPose(msg->poses[i]);
            RCLCPP_INFO(this->get_logger(), "Updated drone %zu pose: (%f, %f, %f)",
                       i, msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
        }
    }
    void droneVelCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // Handle the incoming drone velocity message
        RCLCPP_INFO(this->get_logger(), "Received drone velocity: (%f, %f, %f)",
                   msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

        // Update the drones with the received velocities
        for (size_t i = 0; i < drones_.size(); ++i) {
            drones_[i]->setVelocity(msg->twist);
            RCLCPP_INFO(this->get_logger(), "Updated drone %zu velocity: (%f, %f, %f)",
                       i, msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        }
    }

    void initialize_cars()
    {
                //pure pursuit car

                // Initialize cars
                controlled_car_ = std::make_shared<Car>("ego", true);
                controlled_car_->setPose(makePose(10.0, 0.0));  // Center of first lane
                controlled_car_->setVelocity(makeVel(1.0, 0.0));

                // First obstacle
                auto drone0 = std::make_shared<Car>("drone_0", false);
                drone0->setPose(makePose(30.0, 4.5));
                drone0->setVelocity(makeVel(2.0, 0.0));
                drones_.push_back(drone0);

                // Second obstacle
                auto drone1 = std::make_shared<Car>("drone_1", false);
                drone1->setPose(makePose(40.0, -4.5));
                drone1->setVelocity(makeVel(2.0, 0.0));
                drones_.push_back(drone1);

                // Third obstacle
                auto drone2 = std::make_shared<Car>("drone_2", false);
                drone2->setPose(makePose(18.0, 0.0));
                drone2->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone2);

                // Fourth obstacle
                auto drone3 = std::make_shared<Car>("drone_3", false);
                drone3->setPose(makePose(50.0, 0.0));
                drone3->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone3);

                // Fifth obstacle
                auto drone4 = std::make_shared<Car>("drone_4", false);
                drone4->setPose(makePose(20.0, 4.5));
                drone4->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone4);

                // Sixth obstacle
                auto drone5 = std::make_shared<Car>("drone_5", false);
                drone5->setPose(makePose(30.0, -4.5));
                drone5->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone5);

                // Seventh obstacle
                auto drone6 = std::make_shared<Car>("drone_6", false);
                drone6->setPose(makePose(40.0, 0.0));
                drone6->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone6);

                // Eighth obstacle
                auto drone7 = std::make_shared<Car>("drone_7", false);
                drone7->setPose(makePose(50.0, 4.5));
                drone7->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone7);

                // Ninth obstacle  
                auto drone8 = std::make_shared<Car>("drone_8", false);
                drone8->setPose(makePose(60.0, -4.5));
                drone8->setVelocity(makeVel(1.0, 0.0));
                drones_.push_back(drone8);
    }

    void purepursuitInit(){
        //  — now create & init the pure-pursuit car —
        purepursuit_car_ = std::make_shared<Car>("pp_ego", true);
        purepursuit_car_->setPose( makePose( 0.0, 0.0 ) );
        purepursuit_car_->setVelocity( makeVel( 0.0, 0.0 ) );

        // load your waypoint CSV
        if (!purepursuit_controller_.loadPathFromCSV("/home/yonatan/Desktop/Team10_ws/src/vo/src/raceline.csv"))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load PP path CSV");
        }
    }

    point_msg findNextGoalPoint(const std::vector<point_msg>& raceline, const pose_msg& ego_pose) {
        int lookahead_step = 5;
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
            }

        return raceline[lookahead_index];
    }   
private:
    Road road_;
    DroneController controller_;
    PurePursuitController purepursuit_controller_;
    std::shared_ptr<Car> controlled_car_;
    std::shared_ptr<Car> purepursuit_car_;
    std::vector<std::shared_ptr<Car>> drones_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string control_mode_; // "VO" / "NLVO" / "NAO"
    std::shared_ptr<OnSetParametersCallbackHandle> param_callback_handle_;


    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr vo_marker_pub_;
    rclcpp::Publisher<vis_marker_arr>::SharedPtr nlvo_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pp_cmd_vel_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr cmd_accel_pub_;

    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ego_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ego_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr drone_vel_sub_;
    


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

    void purepursuitUpdate(){
        // — Pure pursuit for pp_car_ —
        geometry_msgs::msg::PoseStamped purepursuitPose;
        purepursuitPose.header.stamp    = now();
        purepursuitPose.header.frame_id = "map";
        purepursuitPose.pose            = purepursuit_car_->getPose();
        double forward_speed = 5.0;
        // compute the twist
        auto pp_twist = purepursuit_controller_.computeCommand(purepursuitPose, forward_speed);
        pp_cmd_vel_pub_->publish(pp_twist);

        // extract current yaw of the car
        // tf2::Quaternion q;
        // tf2::fromMsg(purepursuit_car_->getPose().orientation, q);
        // double yaw = tf2::getYaw(q);
        // convert to your Car’s twist_msg
        twist_msg v;
         // body→world conversion:
        v.linear.x = pp_twist.linear.x;
        if (pp_twist.linear.x < 1.0) v.linear.x = pp_twist.linear.y;
        v.linear.y = 0.0;
        v.angular.z = pp_twist.angular.z;
        // set the velocity
        purepursuit_car_->setVelocity(pp_twist);

        // integrate kinematics exactly as you do for the ego car:
        purepursuit_car_->updateAckermann(dt);

        // publish pose & TF for the pure-pursuit car
        publishPose(*purepursuit_car_);
        publishTF  (*purepursuit_car_, "map", purepursuit_car_->getId());
    }

    geometry_msgs::msg::Twist voUpdate(std::vector<pose_msg> obstacle_poses, std::vector<twist_msg> obstacle_velocities,auto ego_pose, auto ego_vel) {
        // — VO for ego car —
        std::vector<point_msg> raceline = setRaceline();
        point_msg goal_point = findNextGoalPoint(raceline, ego_pose);
        float r_total = calculateTotalRadius();
        twist_msg new_ego_velocity = vo.selectBestVelocity(ego_pose, ego_vel, obstacle_poses, obstacle_velocities, goal_point, r_total);
        controlled_car_->setVelocity(new_ego_velocity);
        return new_ego_velocity;
    }

    geometry_msgs::msg::Twist nlvoUpdate(std::vector<pose_msg> obstacle_poses, std::vector<twist_msg> obstacle_velocities,auto ego_pose, auto ego_vel) {
        // — NLVO for ego car —
        std::vector<point_msg> raceline = setRaceline();
        point_msg goal_point = findNextGoalPoint(raceline, ego_pose);
        float r_total = calculateTotalRadius();
        twist_msg new_ego_velocity = nlvo.selectBestVelocity(ego_pose, ego_vel, obstacle_poses, obstacle_velocities, goal_point, r_total);
        controlled_car_->setVelocity(new_ego_velocity);
        return new_ego_velocity;
    }
    
    geometry_msgs::msg::Twist naoUpdate(std::vector<pose_msg> obstacle_poses, std::vector<twist_msg> obstacle_velocities,auto ego_pose, auto ego_vel) {
        // — NAO for ego car —
        std::vector<point_msg> raceline = setRaceline();
        point_msg goal_point = findNextGoalPoint(raceline, ego_pose);
        float r_total = calculateTotalRadius();
        // Set the new acceleration for the ego car
        geometry_msgs::msg::Accel new_ego_accel = nao.selectBestAcceleration(ego_pose, ego_vel, obstacle_poses, obstacle_velocities, goal_point, r_total);
        cmd_accel_pub_->publish(new_ego_accel);
        // Integrate the acceleration to get the new velocity
        twist_msg new_ego_velocity = controlled_car_->getVelocity();
        new_ego_velocity.linear.x += new_ego_accel.linear.x * dt;
        new_ego_velocity.linear.y += new_ego_accel.linear.y * dt;
        controlled_car_->setVelocity(new_ego_velocity);

        return new_ego_velocity;
    }

    void update() {
        //double dt = 0.1;  // 100 ms
        std::vector<pose_msg> obstacle_poses;
        std::vector<twist_msg> obstacle_velocities;

        // Update drones
        for (size_t i = 0; i < drones_.size(); ++i) {
        //    controller_.control(*drones_[i], static_cast<int>(i));
            // publishPose(*drones_[i]);
            // publishTF(*drones_[i], "map", drones_[i]->getId());
            drones_[i]->update(dt);
            obstacle_poses.push_back(drones_[i]->getPose());
            obstacle_velocities.push_back(drones_[i]->getVelocity());
            // check
            std::cout << "Drone " << i << " velocity: " << obstacle_velocities[i].linear.x << ", " << obstacle_velocities[i].linear.y << std::endl;
        }


        // Get ego car's current pose and velocity
        auto ego_pose = controlled_car_->getPose();
        auto ego_vel = controlled_car_->getVelocity();

        std::vector<point_msg> raceline = setRaceline();
        point_msg goal_point = findNextGoalPoint(raceline, ego_pose);
        float r_total = calculateTotalRadius();
        
        twist_msg new_ego_velocity;

        if (control_mode_ == "VO") {
            new_ego_velocity = voUpdate(obstacle_poses, obstacle_velocities, ego_pose, ego_vel);
        }
        else if (control_mode_ == "NLVO") {
            new_ego_velocity = nlvoUpdate(obstacle_poses, obstacle_velocities, ego_pose, ego_vel);
        }
        else if (control_mode_ == "NAO") {
            new_ego_velocity = naoUpdate(obstacle_poses, obstacle_velocities, ego_pose, ego_vel);
        }
        else {
            RCLCPP_WARN(get_logger(), "Unknown control_mode: %s, using default NLVO", control_mode_.c_str());
            new_ego_velocity = nlvoUpdate(obstacle_poses, obstacle_velocities, ego_pose, ego_vel);
        } 

        // Update ego car's orientation based on the new velocity
        double yaw = std::atan2(new_ego_velocity.linear.y, new_ego_velocity.linear.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        controlled_car_->setOrientation(q);
        //RCLCPP_INFO(this->get_logger(), "Ego car orientation set to: %f", yaw);

        // Update ego car's position based on the new velocity
        controlled_car_->update(dt);
        publishPose(*controlled_car_);
        publishTF(*controlled_car_, "map", controlled_car_->getId());

        purepursuitUpdate(); // update pure pursuit car

        publishMarkers();
        publishVOMarkers();
        publishNLVOMarkers();
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
        {
            marker_array.markers.push_back(makeCarMarker(*controlled_car_, id));
            setVelocityArrowMarker(marker_array, *controlled_car_, this->now(), 0);
            setVelocityTextMarker(marker_array, *controlled_car_, this->now());
        }

        // Pure pursuit car
        {
            auto pp = purepursuit_car_.get();
            // make marker
            vis_marker pp_marker = makeCarMarker(*pp, id++);
            // set color to green
            pp_marker.color.r = 0.0;
            pp_marker.color.g = 1.0;
            pp_marker.color.b = 0.0;
            pp_marker.color.a = 1.0;
            // add to marker array
            marker_array.markers.push_back(pp_marker);
            // velocity arrow
            setVelocityArrowMarker(marker_array, *pp, now(), id++);
            // if you want a text marker for the velocity
            setVelocityTextMarker(marker_array, *pp, now());

            // add a line from the car to the last goal
            auto goal = purepursuit_controller_.lastGoal();
            visualization_msgs::msg::Marker m;
            m.header = goal.header;
            m.ns     = "pp_goal";
            m.id     = 999;
            m.type   = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = goal.point;
            m.scale.x = m.scale.y = m.scale.z = 0.3;
            m.color.g = 1.0; m.color.a = 0.8;
            marker_array.markers.push_back(m);
          }
        
        // Road
        {
        rclcpp::Time now = this->now();
        vis_marker road_marker;
        setRoadMarker(road_marker, road_, now);
        marker_array.markers.push_back(road_marker);
        }

        // lane lines
        {
            for (int i = 1; i < road_.getNumLanes(); ++i) {
                vis_marker lane_marker;
                setLaneMarker(lane_marker, road_, i, now());
                marker_array.markers.push_back(lane_marker);
            }
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
        marker.type = vis_marker::CUBE;
        // marker.type= vis_marker::MESH_RESOURCE;
        // marker.mesh_resource = "package://car_description/meshes/obstacle.STL";
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

        // // for debugging: show the candidate velocities
        // std::vector<twist_msg> candidate_velocities = nlvo.generateACV(ego_vel);
        // for (const auto& candidate_velocity : candidate_velocities) {
        //     vis_marker candidate_marker;
        //     // Set the properties of the candidate marker
        //     setCandidateMarker(candidate_marker, ego_pose, candidate_velocity, r_total, 5.0);
        //     candidate_marker.id = id++;
        //     marker_array.markers.push_back(candidate_marker);
        // }

       // for debugging: show the candidate accelerations
       // convert each Accel candidate into a Twist by integrating over delta_t
        std::vector<geometry_msgs::msg::Accel> candidate_accels = nao.generateCandidateAccelerations(ego_vel);
        for (const auto &acc : candidate_accels) {
            // integrate a * dt → Δv, then add to current velocity
            twist_msg cand_vel;
            cand_vel.linear.x  = ego_vel.linear.x + acc.linear.x * delta_t;
            cand_vel.linear.y  = ego_vel.linear.y + acc.linear.y * delta_t;
            cand_vel.linear.z  = 0.0;
            cand_vel.angular.x = 0.0;
            cand_vel.angular.y = 0.0;
            cand_vel.angular.z = 0.0;

            vis_marker cand_marker;
            setCandidateMarker(cand_marker,
                                ego_pose,
                                cand_vel,
                                r_total,
                                time_horizon /* or your min_time_horizon */);
            cand_marker.id = id++;
            marker_array.markers.push_back(cand_marker);
        }

        vo_marker_pub_->publish(marker_array);
        //RCLCPP_INFO(this->get_logger(), "Published %zu markers", marker_array.markers.size());
    }

    void publishNLVOMarkers()
    {
       vis_marker_arr marker_array;

       int id = 0;  // Marker ID counter
       auto ego_pose = controlled_car_->getPose();
       auto ego_vel = controlled_car_->getVelocity();
       float r_total = calculateTotalRadius();

       for (const auto& drone : drones_) {
           auto obstacle_pose = drone->getPose();
           auto obstacle_vel = drone->getVelocity();


        // Compute the NLVO disks
        float time_horizon = nlvo.computeMinimumTimeHorizon(ego_pose, ego_vel, obstacle_pose, obstacle_vel, r_total, nlvo.control_set) + 2.0f;
        std::vector<VelDisk> disks = nlvo.generateNLVODisks(ego_pose, obstacle_pose, obstacle_vel, r_total, time_horizon);  

        
        for (auto &disk : disks) {
        
    }

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
    }

    marker_pub_->publish(marker_array);
    }
    
    geometry_msgs::msg::Twist convertCmdVector(const geometry_msgs::msg::Twist &vel, const geometry_msgs::msg::Pose ego_pos){
    geometry_msgs::msg::Twist vel_cmd;
    float k_heading=0.9;
    float theta= atan2(vel.linear.y,vel.linear.x);
    double vx_local = cos(theta) * vel.linear.x + sin(theta) * vel.linear.y;
    vel_cmd.linear.x = vx_local;
    double heading_error = theta- ego_pos.orientation.z;
    vel_cmd.angular.z = k_heading * heading_error;
    return vel_cmd;
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
