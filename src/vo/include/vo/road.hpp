#pragma once

#include <vector>
#include <string>
#include "common/settings.hpp"

class Road {
public:
    //Constructor
    Road(int num_lanes, double lane_width, double length, double radius = 0.0);

    std::vector<double> getLaneCenterlines() const;
    //Get number of lanes
    int getNumLanes() const;
    //Get lane width
    double getLaneWidth() const;
    //Get road length
    double getLength() const;
    // Get road radius
    double getRadius() const;

private:
    //Number of lanes in the road
    int num_lanes_;
    //Width of the lanes in meters
    double lane_width_;
    //Road length in meters
    double length_;
    //Road radius in meters
    double radius_;
};