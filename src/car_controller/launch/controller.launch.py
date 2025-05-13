from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction



def generate_launch_description():

    robot_controllers=os.path.join(get_package_share_directory("car_controller"),"config","ackermann_param.yaml")

   
    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    output="both",
    )  

    joint_state_broadcaster_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    ackermann_steering_controller= Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    velocity_controller= Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )


    return LaunchDescription([
        control_node,
        velocity_controller,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller
    ])