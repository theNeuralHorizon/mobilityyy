from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("rover_bringup")
    r1_gz_share = get_package_share_directory("mini_r1_v1_gz")
    grid_world_share = get_package_share_directory("grid_world")

    world_file = os.path.join(grid_world_share, "worlds", "grid_world_FINAL.sdf")
    autonomy_launch = os.path.join(bringup_share, "launch", "autonomy.launch.py")
    sim_launch = os.path.join(r1_gz_share, "launch", "sim.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("spawn_x", default_value="-1.35"),
        DeclareLaunchArgument("spawn_y", default_value="1.80"),
        DeclareLaunchArgument("spawn_z", default_value="0.10"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        # Launch the R1 V1 robot in Gazebo with the grid world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={"world": world_file}.items(),
        ),
        # Launch autonomy nodes with topic remappings
        IncludeLaunchDescription(PythonLaunchDescriptionSource(autonomy_launch)),
    ])
