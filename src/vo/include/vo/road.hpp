#pragma once

#include <vector>
#include <string>
#include "settings.hpp"

class Road {
public:
    //Constructor
    Road(int num_lanes, double lane_width, double length);

    std::vector<double> getLaneCenterlines() const;
    //Get number of lanes
    int getNumLanes() const;
    //Get lane width
    double getLaneWidth() const;
    //Get road length
    double getLength() const;

private:
    //Number of lanes in the road
    int num_lanes_;
    //Width of the lanes in meters
    double lane_width_;
    //Road length in meters
    double length_;
};