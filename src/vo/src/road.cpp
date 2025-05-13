#include "road.hpp"

Road::Road(int num_lanes, double lane_width, double length, double radius)
    : num_lanes_(num_lanes), lane_width_(lane_width), length_(length), radius_(radius) {}

std::vector<double> Road::getLaneCenterlines() const {
    std::vector<double> lanes;
    for (int i = 0; i < num_lanes_; ++i) {
        lanes.push_back((i + 0.5) * lane_width_);
    }
    return lanes;
}

int Road::getNumLanes() const {
    return num_lanes_;
}

double Road::getLaneWidth() const {
    return lane_width_;
}

double Road::getLength() const {
    return length_;
}

double Road::getRadius() const{
    return radius_;
}