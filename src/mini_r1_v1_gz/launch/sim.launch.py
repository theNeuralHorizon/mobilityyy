import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gz_sim_launch(context):
    ros_gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )
    world_path = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gz_args = f"-r -s {world_path}" if headless else f"-r {world_path}"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_gz_launch),
            launch_arguments={
                "gz_args": gz_args,
                "on_exit_shutdown": "true",
            }.items(),
        )
    ]


def generate_launch_description():
    description_package = "mini_r1_v1_description"
    simulation_package = "mini_r1_v1_gz"

    description_package_share = get_package_share_directory(description_package)

    default_world = os.path.join(description_package_share, "worlds", "empty.sdf")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_package_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_control": "false",
        }.items(),
    )
    gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join([
            os.path.dirname(description_package_share),
            os.path.dirname(get_package_share_directory("grid_world")),
        ]),
    )
    gz_launch = OpaqueFunction(function=_gz_sim_launch)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "mini_r1",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    bridge_params = os.path.join(
        get_package_share_directory(simulation_package),
        "config",
        "ros_gz_bridge.yaml",
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )
    launch_args = [
        DeclareLaunchArgument(
            name="world",
            default_value=default_world,
            description="Enter the absolute path to the world in which the robot is to be spawned"
        ),
        DeclareLaunchArgument(name="headless", default_value="true"),
        DeclareLaunchArgument(name="spawn_x", default_value="0.0"),
        DeclareLaunchArgument(name="spawn_y", default_value="0.0"),
        DeclareLaunchArgument(name="spawn_z", default_value="0.07"),
        DeclareLaunchArgument(name="spawn_yaw", default_value="0.0"),
    ]

    nodes = [
        *launch_args,
        gz_resource,
        rsp,
        gz_launch,
        ros_gz_bridge,
        cmd_vel_bridge,
        spawn_entity,
    ]

    # twist_stamper is optional — only add if available
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix("twist_stamper")
        stamper = Node(
                    package="twist_stamper",
                    executable="twist_stamper",
                    remappings=[
                        ('cmd_vel_in', 'cmd_vel'),
                        ('cmd_vel_out', 'cmd_vel_stamped'),
                    ],
        )
        nodes.append(stamper)
    except Exception:
        pass  # twist_stamper not installed, skip it

    return LaunchDescription(nodes)
