#include "velocity_obstacle.hpp"

//temp debugger
int main() {
    VelocityObstacle vo;
    pose ego_pose;
    ego_pose.position.x = 0;
    ego_pose.position.y = 0;

    pose obstacle_pose;
    obstacle_pose.position.x = 3;
    obstacle_pose.position.y = 3;

    
    velocity ego_velocity;
    ego_velocity.linear.x = 3;
    ego_velocity.linear.y = 2;
    
    velocity obstacle_velocity;
    obstacle_velocity.linear.x = 0;
    obstacle_velocity.linear.y = 0;

    return 0;
}