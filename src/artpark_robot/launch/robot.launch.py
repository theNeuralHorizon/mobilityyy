"""Spawn the artpark_bot into an already-running Gazebo sim.

Run after `gz sim -r grid_world_FINAL.sdf` is up. This launch publishes TF
from the xacro URDF, bridges the Gazebo sensor/command topics into ROS, and
spawns the robot at the solid-green START tile with yaw configurable via
argument.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('artpark_robot')
    xacro = PathJoinSubstitution([pkg, 'urdf', 'artpark_bot.urdf.xacro'])

    spawn_x   = LaunchConfiguration('spawn_x')
    spawn_y   = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_description = Command(['xacro ', xacro])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn entity in Gazebo Harmonic via ros_gz_sim
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'artpark_bot',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.05',
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    # Topic bridges: Gazebo ↔ ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/front_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/front_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/floor_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/floor_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x',   default_value='-1.35',
                              description='World x of the START tile (solid green)'),
        DeclareLaunchArgument('spawn_y',   default_value='1.80',
                              description='World y of the START tile (solid green)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Yaw at spawn; adjust after P0 screenshot'),
        rsp, spawn, bridge,
    ])
