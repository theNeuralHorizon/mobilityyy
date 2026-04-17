"""Start Gazebo Harmonic with the judge's world and the ROS-Gazebo clock bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grid_world_share = get_package_share_directory('grid_world')
    world_path = os.path.join(grid_world_share, 'worlds', 'grid_world_FINAL.sdf')

    # Tell Gazebo where to find our tile textures (the SDF references absolute
    # paths in the judge's config; override with an env var resource path).
    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=grid_world_share,
    )

    world = LaunchConfiguration('world')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path,
                              description='Path to the SDF world'),
        gz_resource, gz_sim, clock_bridge,
    ])
