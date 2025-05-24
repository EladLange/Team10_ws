#pragma once

#include <string>
#include "common/settings.hpp"


class Car {
public:
    //Constructor
    Car(const std::string& id, bool is_controlled);

    //Set car position manually
    void setPose(const pose_msg& pose);

    //Set car velocity manually
    void setVelocity(const twist_msg& vel);
    
    //Get car position manually
    const pose_msg& getPose() const;

    //Get car velocity manually
    const twist_msg& getVelocity() const;

    //Set the car orientation
    void setOrientation(const tf2::Quaternion& orientation);

    //update car position (integration)
    void update(double dt);

    //Get car ID
    std::string getId() const;

    //Optional- true if car is ego vehicle
    bool isControlled() const;

    //update pure pursuit car position
    void updateAckermann(double dt);

private:
    //Car ID
    std::string id_;
    bool controlled_;
    //Car position & orientation
    pose_msg pose_;
    //Car linear & angular velocity
    twist_msg velocity_;
};