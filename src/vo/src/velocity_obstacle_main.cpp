#include "velocity_obstacle.hpp"
#include <iostream>

//temp debugger
int main() {
    VelocityObstacle vo;
    pose_msg ego_pose;
    ego_pose.position.x = 0;
    ego_pose.position.y = 0;

    pose_msg obstacle_pose;
    obstacle_pose.position.x = 0;
    obstacle_pose.position.y = 3;

    twist_msg ego_velocity;
    ego_velocity.linear.x = 1;
    ego_velocity.linear.y = 1;
    
    twist_msg obstacle_velocity;
    obstacle_velocity.linear.x = 1;
    obstacle_velocity.linear.y = -1;

    std::vector<pose_msg> raceline(5);
    // Initialize raceline points raceline = (0,0), (1,1), (2,2), (3,3), (4,4)
    for (int i = 0; i < 5; ++i) {
        raceline[i].position.x = i;
        raceline[i].position.y = i;
    }

    float r_total = 2.0;

    // Test the checkCollision function
    bool collision = vo.checkCollision(ego_pose, obstacle_pose, ego_velocity, obstacle_velocity, r_total);
    if (collision) {
        std::cout << "Collision detected!" << std::endl;
    } else {
        std::cout << "No collision." << std::endl;
    }

    // Test the selectBestVelocity function
    std::vector<pose_msg> obstacle_poses = {obstacle_pose};
    std::vector<twist_msg> obstacle_velocities = {obstacle_velocity};

    // Select the best velocity
    twist_msg best_velocity = vo.selectBestVelocity(ego_pose, ego_velocity, obstacle_poses, obstacle_velocities, raceline, r_total);
    std::cout << "Best velocity: (" << best_velocity.linear.x << ", " << best_velocity.linear.y << ")" << std::endl;

    return 0;
}