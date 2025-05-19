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
    ros_distro= os.environ ["ROS_DISTRO"]
    is_ignition ="true" if ros_distro =="humble" else "false"

    model_arg= DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(car_description_dir,"urdf","car_urdf.xacro"),
        description="Absolute path to the urdf file of the model"
    )

    robot_description= ParameterValue(Command([
        "xacro ", 
        LaunchConfiguration("model"),
        " is_ignition:=", 
        is_ignition
        ]),
        value_type=str)

    robot_state_publisher=  Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_resource_path =SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
           str(Path(car_description_dir).parent.resolve())]
    )

    gazebo= IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments= [
            ("gz_args", [" -v 4", " -r", " empty.sdf" ])
        ]      
    )

    spawn_pose = DeclareLaunchArgument(
        name="spawn_pose",
        default_value="0.0 0.0 0.1",
        description="Spawn pose as 'x y z' separated by spaces"
    )

    spawn_pose_value = LaunchConfiguration('spawn_pose')

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output= "screen",
        arguments=["-entity","car",
                   "-topic", "robot_description",
                   "-name", "Ego",
                   "-pose", spawn_pose_value]
    )

    ekf_node =Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(car_description_dir,"config","ekf.yaml")]
    )

    ego_controller= Node(
        package="car_description",
        executable="ego_controller"
    )

    joint_state_pub= Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )


    odom_publisher= Node(
        package="car_description",
        executable="ego_pose_pub"
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_node",
        arguments=[
            '/world/empty/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V'
        ]
    )

    robot_controllers=os.path.join(get_package_share_directory("car_controller"),"config","ackermann_param.yaml")

    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    output="both",
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

    rviz_node=  Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",os.path.join(car_description_dir, "RVIZ", "vo_display.rviz")]
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
    model_arg,
    spawn_pose,
    robot_state_publisher,
    gazebo_resource_path,
    gazebo,
    gz_spawn_entity,
    ego_controller,
    ros_gz_bridge,
    odom_publisher,
    control_node,
    joint_state_pub,
    ackermann_steering_controller,
    velocity_controller,
    rviz_node,
    vo_node,
    pure_pursuit_node
    ])