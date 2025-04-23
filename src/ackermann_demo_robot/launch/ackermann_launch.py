from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('ackermann_demo_robot')

    # URDF
    xacro_file = os.path.join(pkg_path, 'urdf', 'ackermann_bot.xacro')
    robot_description_content = os.popen(f'xacro {xacro_file}').read()
    robot_description = {'robot_description': robot_description_content}

    # Controller config
    controller_config = os.path.join(pkg_path, 'config', 'ackermann_controller.yaml')

    return LaunchDescription([

        # 1. Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ])
        ),

        # 2. Load robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        # 3. Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'ackermann_bot', '-topic', 'robot_description'],
            output='screen'
        ),

        # 4. Load controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
            output='screen'
        ),

        # 5. Activate ackermann_steering_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ackermann_steering_controller'],
            output='screen'
        ),
    ])
