from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_xacro = os.path.join(
        get_package_share_directory("rover_robot"),
        "urdf",
        "rover.urdf.xacro",
    )
    robot_description = Command(["xacro ", robot_xacro])
    return LaunchDescription([
        DeclareLaunchArgument("spawn_x", default_value="-1.35"),
        DeclareLaunchArgument("spawn_y", default_value="1.80"),
        DeclareLaunchArgument("spawn_z", default_value="0.10"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "round3_rover",
                "-topic", "robot_description",
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-z", LaunchConfiguration("spawn_z"),
                "-Y", LaunchConfiguration("spawn_yaw"),
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
                "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            ],
            output="screen",
        ),
    ])
