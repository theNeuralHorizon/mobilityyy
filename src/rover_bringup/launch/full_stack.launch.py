from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("rover_bringup")
    sim_launch = os.path.join(bringup_share, "launch", "sim_world.launch.py")
    robot_launch = os.path.join(bringup_share, "launch", "robot_spawn.launch.py")
    autonomy_launch = os.path.join(bringup_share, "launch", "autonomy.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="-1.35"),
        DeclareLaunchArgument("spawn_y", default_value="1.80"),
        DeclareLaunchArgument("spawn_z", default_value="0.10"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(autonomy_launch)),
    ])
