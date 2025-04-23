#pragma once

#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
    geometry_msgs::msg::Pose pose_;
    //Car linear & angular velocity
    geometry_msgs::msg::Twist velocity_;
};