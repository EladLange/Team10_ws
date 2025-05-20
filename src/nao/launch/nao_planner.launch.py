from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_local_planner',
            executable='nao_planner_node',
            name='nao_local_planner',
            output='screen',
            parameters=[
                {'a_max': 2.5},
                {'a_min': -4.0},
                {'a_lat_max': 8.0},
                {'horizon': 1.0},
                {'num_steps': 10},
                {'weights': [1000.0,1.0,1.0,0.1]}
            ]
        )
    ])
