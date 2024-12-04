from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    # Nodes
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
    )

    microRos_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent"
    )

    # Include the teleop-launch.py from teleop_twist_joy package
    teleop_twist_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("teleop_twist_joy"),  # Replace with the correct package name
                "launch",
                "teleop-launch.py"  # Replace with the correct launch file name
            ])
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        teleop_twist_joy_launch,  # Include the teleop_twist_joy launch file
        camera_node,
        microRos_node
    ])
