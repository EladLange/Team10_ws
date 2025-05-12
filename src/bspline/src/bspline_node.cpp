// Required ROS 2 headers for creating nodes and publishing messages
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Standard library headers
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <array>

using namespace std;

// Vec3 represents a 3D vector (x, y, z)
using Vec3 = array<double, 3>;

// A 4x3 matrix of control points (4 points in 3D)
using Matrix4x3 = array<Vec3, 4>;

// A list of 3D points
using PointList = vector<Vec3>;

// Cubic B-spline basis matrix (4x4)
// Used to compute the spline blending coefficients
const double M[4][4] = {
    {-1.0 / 6, 0.5, -0.5, 1.0 / 6},
    {0.5, -1.0, 0.5, 0.0},
    {-0.5, 0.0, 0.5, 0.0},
    {1.0 / 6, 2.0 / 3, 1.0 / 6, 0.0}
};

// Computes a single point on a cubic B-spline curve for a given segment and parameter u in [0, 1]
Vec3 computeBSplinePoint(double u, const Matrix4x3& cp) {
    double U[4] = {pow(u, 3), pow(u, 2), u, 1.0};
    Vec3 result = {0, 0, 0};
    for (int d = 0; d < 3; ++d) {           // For each dimension x, y, z
        for (int i = 0; i < 4; ++i) {       // Iterate over control points
            double coeff = 0;
            for (int j = 0; j < 4; ++j)     // Compute blending coefficient from basis matrix
                coeff += U[j] * M[j][i];
            result[d] += coeff * cp[i][d];  // Apply to control points
        }
    }
    return result;
}

// Loads control points from a CSV file with format: X,Y[,Z]
// If Z is not present, it is assumed to be 0.0
PointList loadControlPointsFromCSV(const string& filename) {
    PointList points;
    ifstream file(filename);
    string line;

    getline(file, line);  // Skip the header line

    while (getline(file, line)) {
        istringstream ss(line);
        string val;
        Vec3 p = {0, 0, 0};
        getline(ss, val, ','); p[0] = stod(val);  // X
        getline(ss, val, ','); p[1] = stod(val);  // Y
        if (getline(ss, val, ','))                // Z (optional)
            p[2] = val.empty() ? 0.0 : stod(val);
        points.push_back(p);
    }
    return points;
}

// Main ROS 2 node class that computes and publishes the B-spline curve
class BSplineNode : public rclcpp::Node {
public:
    BSplineNode() : Node("bspline_node") {
        // Create a publisher for visualization_msgs::Marker on topic "bspline_marker"
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("bspline_marker", 10);

        // Load control points from a CSV file
        auto points = loadControlPointsFromCSV("src/bspline/src/controlPoints.csv");

        // Add clamping to ensure endpoint continuity (repeat the first and last point 3 times)
        PointList controlPoints;
        controlPoints.push_back(points[1]);
        controlPoints.push_back(points[1]);
        controlPoints.push_back(points[1]);
        controlPoints.insert(controlPoints.end(), points.begin(), points.end());
        controlPoints.push_back(points.back());
        controlPoints.push_back(points.back());
        controlPoints.push_back(points.back());

        // Create a marker for visualizing the B-spline curve in RViz
        marker.header.frame_id = "map";  // RViz coordinate frame
        marker.header.stamp = this->now();
        marker.ns = "bspline";
        marker.id = 0;
        marker.type = marker.LINE_STRIP;
        marker.action = marker.ADD;
        marker.scale.x = 0.05;  // Line width
        marker.color.r = 1.0;   // Red color
        marker.color.a = 1.0;   // Full opacity

        // Generate points along each B-spline segment and append them to the marker
        for (size_t k = 1; k + 3 < controlPoints.size(); ++k) {
            Matrix4x3 seg;
            for (int i = 0; i < 4; ++i)
                seg[i] = controlPoints[k + i];

            for (int r = 0; r <= 100; ++r) {
                double u = double(r) / 100;
                Vec3 pt = computeBSplinePoint(u, seg);

                geometry_msgs::msg::Point ros_pt;
                ros_pt.x = pt[0];
                ros_pt.y = pt[1];
                ros_pt.z = pt[2];
                marker.points.push_back(ros_pt);
            }
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BSplineNode::update, this));
    }

private:
    visualization_msgs::msg::Marker marker;

    // Publisher to publish the marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Publish the marker to RViz
    void update(){
        marker_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published B-spline curve as Marker.");
    }
};

// Entry point of the program: initializes ROS, runs the node, and shuts down
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BSplineNode>());
    rclcpp::shutdown();
    return 0;
}
