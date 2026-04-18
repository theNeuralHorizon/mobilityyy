from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory("grid_world"),
        "worlds",
        "grid_world_FINAL.sdf",
    )
    mini_sim_launch = os.path.join(
        get_package_share_directory("mini_r1_v1_gz"),
        "launch",
        "sim.launch.py",
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="-1.35"),
        DeclareLaunchArgument("spawn_y", default_value="1.80"),
        DeclareLaunchArgument("spawn_z", default_value="0.07"),
        DeclareLaunchArgument("spawn_yaw", default_value="-0.22"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mini_sim_launch),
            launch_arguments={
                "world": world,
                "headless": LaunchConfiguration("headless"),
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            }.items(),
        ),
    ])
