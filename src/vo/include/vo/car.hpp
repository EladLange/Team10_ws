#pragma once

#include <string>
#include "settings.hpp"


class Car {
public:
    //Constructor
    Car(const std::string& id, bool is_controlled);
    //Set car position manually
    void setPose(const geometry_msgs::msg::Pose& pose);
    //Set car velocity manually
    void setVelocity(const geometry_msgs::msg::Twist& vel);
    
    //Get car position manually
    const geometry_msgs::msg::Pose& getPose() const;
    //Get car velocity manually
    const geometry_msgs::msg::Twist& getVelocity() const;
    //update car position (integration)
    void update(double dt);
    //Get car ID
    std::string getId() const;
    //Optional- true if car is ego vehicle
    bool isControlled() const;

private:
    //Car ID
    std::string id_;
    bool controlled_;
    //Car position & orientation
    pose_msg pose_;
    //Car linear & angular velocity
    twist_msg velocity_;
};