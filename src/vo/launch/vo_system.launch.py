from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    debug_level_arg = DeclareLaunchArgument(
        'debug_level',
        default_value='1',
        description='Debug level (0=info, 1=debug, 2=verbose)'
    )

    # Get the launch configuration
    debug_level = LaunchConfiguration('debug_level')

    # Start the environment simulation node
    env_sim_node = Node(
        package='env_sim',
        executable='env_sim_node',
        name='env_sim_node',
        output='screen'
    )

    # Start the VO node with debug parameters
    vo_node = Node(
        package='vo',
        executable='vo_node',
        name='vo_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'debug_level': debug_level}
        ]
    )

    # Return the launch description
    return LaunchDescription([
        debug_level_arg,
        env_sim_node,
        vo_node
    ])