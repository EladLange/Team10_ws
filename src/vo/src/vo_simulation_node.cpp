#include "rclcpp/rclcpp.hpp"
#include "road.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class VOSimulationNode : public rclcpp::Node {
    public:
        VOSimulationNode() : Node("vo_simulation_node"), road_(3, 3.0, 50.0) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
            publishRoadMarkers();
        }
    
    private:
        Road road_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
        void publishRoadMarkers() {
            auto lanes = road_.getLaneCenterlines();
            for (size_t i = 0; i < lanes.size(); ++i) {
                visualization_msgs::msg::Marker lane_marker;
                lane_marker.header.frame_id = "map";
                lane_marker.header.stamp = this->now();
                lane_marker.ns = "road";
                lane_marker.id = i;
                lane_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                lane_marker.action = visualization_msgs::msg::Marker::ADD;
    
                lane_marker.scale.x = 0.1;
                lane_marker.color.r = 1.0;
                lane_marker.color.g = 1.0;
                lane_marker.color.b = 0.0;
                lane_marker.color.a = 1.0;
    
                for (double x = 0; x < road_.getLength(); x += 1.0) {
                    geometry_msgs::msg::Point pt;
                    pt.x = x;
                    pt.y = lanes[i];
                    pt.z = 0.0;
                    lane_marker.points.push_back(pt);
                }
    
                marker_pub_->publish(lane_marker);
            }
        }
    };
    
    int main(int argc, char** argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<VOSimulationNode>());
        rclcpp::shutdown();
        return 0;
    }