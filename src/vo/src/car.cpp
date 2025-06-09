#include "common/car.hpp"
#include <cmath>
#include <iostream>

Car::Car(const std::string& id, bool is_controlled)
    : id_(id), controlled_(is_controlled) {
    pose_.position.x = 0.0;
    pose_.position.y = 0.0;
    pose_.position.z = 0.5;

    pose_.orientation.w = 1.0;
    pose_.orientation.x = 0.0;
    pose_.orientation.y = 0.0;
    pose_.orientation.z = 0.0;

    velocity_.linear.x = 0.0;
    velocity_.angular.z = 0.0;

    acceleration_.linear.x=0.0;
    acceleration_.linear.y=0.0;

}

void Car::setId(const std::string& string) {
    id_ = string;
}

void Car::setPose(const pose_msg& pose) {
    pose_ = pose;
}

void Car::setVelocity(const twist_msg& vel) {
    velocity_ = vel;
}

void Car::setAcceleration(const twist_msg& accel) {
    acceleration_ = accel;
}

void Car::setOrientation(const tf2::Quaternion& orientation) {
    pose_.orientation.x = orientation.x();
    pose_.orientation.y = orientation.y();
    pose_.orientation.z = orientation.z();
    pose_.orientation.w = orientation.w();
}

const pose_msg& Car::getPose() const {
    return pose_;
}

const twist_msg& Car::getVelocity() const {
    return velocity_;
}

const accel_msg& Car::getAcceleration() const {
    return acceleration_;
}

std::string Car::getId() const {
    return id_;
}

bool Car::isControlled() const {
    return controlled_;
}

void Car::update(double dt) {
    pose_.position.x +=(acceleration_.x*dt*dt)/2 velocity_.linear.x * dt;
    pose_.position.y +=(acceleration_.y*dt*dt)/2 velocity_.linear.y * dt;
    pose_.orientation.z+=velocity_.angular.z * dt;
}

void Car::setRaceline(const std::vector<point_msg>& raceline) {
    raceline_ = raceline;
}

void Car::setSValues(const std::vector<point_msg>& s_values) {
    s_values_ = s_values;
}

std::vector<point_msg> Car::getRaceline() const {
    return raceline_;
}

std::vector<point_msg> Car::getSValues() const {
    return s_values_;
}