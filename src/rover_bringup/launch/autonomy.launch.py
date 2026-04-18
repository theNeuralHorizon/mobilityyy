from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("rover_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_apriltag_ros", default_value="false"),
        DeclareLaunchArgument("output_dir", default_value=str(os.path.expanduser("~/rover_run_logs"))),
        SetEnvironmentVariable(name="PYTHONNOUSERSITE", value="1"),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_node",
            condition=IfCondition(LaunchConfiguration("use_apriltag_ros")),
            parameters=[os.path.join(bringup_share, "config", "apriltag.yaml")],
            remappings=[
                ("/image_rect", "/r1_mini/camera/image_raw"),
                ("/camera_info", "/r1_mini/camera/camera_info"),
                ("/detections", "/tag_detections_native"),
            ],
            output="screen",
        ),
        Node(
            package="rover_autonomy",
            executable="vision_processor",
            name="vision_processor",
            parameters=[
                os.path.join(bringup_share, "config", "hsv_thresholds.yaml"),
                os.path.join(bringup_share, "config", "tag_map.yaml"),
                {"camera_image_topic": "/r1_mini/camera/image_raw"},
            ],
            output="screen",
        ),
        Node(
            package="rover_autonomy",
            executable="state_machine_controller",
            name="state_machine_controller",
            parameters=[os.path.join(bringup_share, "config", "navigation.yaml")],
            output="screen",
        ),
        Node(
            package="rover_autonomy",
            executable="safety_controller",
            name="safety_controller",
            parameters=[
                os.path.join(bringup_share, "config", "navigation.yaml"),
                {"scan_topic": "/r1_mini/lidar"},
            ],
            output="screen",
        ),
        Node(
            package="rover_logging",
            executable="run_logger",
            name="run_logger",
            parameters=[{
                "camera_image_topic": "/r1_mini/camera/image_raw",
                "output_dir": LaunchConfiguration("output_dir"),
            }],
            output="screen",
        ),
    ])
