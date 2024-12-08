from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    # Camera node with specific parameters
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[{
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link_optical'
        }]
    )

    # MicroROS Agent node with serial configuration
    microros_agent_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', '--dev', '/dev/ttyACM0', '-v6'],
        output='screen'
    )

    # Include teleop launch file
    teleop_twist_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("teleop_twist_joy"),
                "launch", 
                "teleop-launch.py"
            ])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='Use sim time if true'
        ),
        teleop_twist_joy_launch,
        camera_node,
        microros_agent_node
    ])