import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    car_description_dir= get_package_share_directory("car_description")

    rviz_node=  Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",os.path.join(car_description_dir, "RVIZ", "mini_display.rviz")]
    )

    rviz_nlvo=  Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",os.path.join(car_description_dir, "RVIZ", "velocity_space.rviz")]
    )

    vo_node= Node(
        package="vo",
        executable="vo_simulation_node"
    )
    pure_pursuit_node= Node(
        package="purepursuit_new",
        executable="pure_pursuit_node"
    )


    return LaunchDescription([
    rviz_node,
    rviz_nlvo,
    # vo_node#,
    # pure_pursuit_node
    ])