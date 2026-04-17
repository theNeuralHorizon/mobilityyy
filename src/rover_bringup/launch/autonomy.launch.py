from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("rover_bringup")
    return LaunchDescription([
        SetEnvironmentVariable(name="PYTHONNOUSERSITE", value="1"),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_node",
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
            ],
            remappings=[
                ("/camera/image_raw", "/r1_mini/camera/image_raw"),
                ("/camera/camera_info", "/r1_mini/camera/camera_info"),
            ],
            output="screen",
        ),
        Node(
            package="rover_autonomy",
            executable="state_machine_controller",
            name="state_machine_controller",
            parameters=[os.path.join(bringup_share, "config", "navigation.yaml")],
            remappings=[
                ("/scan", "/r1_mini/lidar"),
                ("/imu", "/r1_mini/imu"),
            ],
            output="screen",
        ),
        Node(
            package="rover_autonomy",
            executable="safety_controller",
            name="safety_controller",
            parameters=[os.path.join(bringup_share, "config", "navigation.yaml")],
            remappings=[
                ("/scan", "/r1_mini/lidar"),
            ],
            output="screen",
        ),
        Node(
            package="rover_logging",
            executable="run_logger",
            name="run_logger",
            remappings=[
                ("/camera/image_raw", "/r1_mini/camera/image_raw"),
            ],
            output="screen",
        ),
    ])
