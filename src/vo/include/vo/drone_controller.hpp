#pragma once

#include "car.hpp"
#include "road.hpp"

class DroneController {
public:
    //Constructor
    DroneController(const Road& road);
    //Keeps car in the specified lane
    void control(Car& car);

private:
    const Road& road_;
};