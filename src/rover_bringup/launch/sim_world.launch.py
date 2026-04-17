from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    world = os.path.join(
        get_package_share_directory("grid_world"),
        "worlds",
        "grid_world_FINAL.sdf",
    )
    gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(get_package_share_directory("grid_world")),
    )
    gz_args = ["gz", "sim", "-r"]
    server = ExecuteProcess(
        cmd=gz_args + ["-s", world],
        condition=IfCondition(headless),
        output="screen",
    )
    gui = ExecuteProcess(
        cmd=gz_args + [world],
        condition=UnlessCondition(headless),
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        gz_resource,
        server,
        gui,
        clock_bridge,
    ])
