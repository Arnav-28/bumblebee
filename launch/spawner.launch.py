import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the absolute path to the SDF file
    sdf_path = os.path.join(
        get_package_share_directory("bumblebee"), "models", "model.sdf"
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        "x_pose", default_value="0.0", description="X position for the robot"
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        "y_pose", default_value="0.0", description="Y position for the robot"
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        "z_pose", default_value="0.01", description="Z position for the robot"
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "bumblebee",
            "-file",
            sdf_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
        output="screen",
    )

    bridge_params = os.path.join(
        get_package_share_directory("bumblebee"),
        "params",
        "bridge.yaml",
    )

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )
    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bumblebee',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # start_gazebo_ros_image_bridge_cmd = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"],
    #     output="screen",
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    # ld.add_action(start_gazebo_ros_image_bridge_cmd)

    # Add the spawn entity node
    # ld.add_action(spawn_entity)

    return ld
