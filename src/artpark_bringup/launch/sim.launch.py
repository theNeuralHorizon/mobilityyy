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

    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(grid_world_share),
    )

    # Force Mesa software rendering — required for headless/WSL2 without a GPU.
    # llvmpipe is the Mesa software renderer; LIBGL_ALWAYS_SOFTWARE forces it.
    sw_render1 = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')
    sw_render2 = SetEnvironmentVariable(name='MESA_LOADER_DRIVER_OVERRIDE', value='llvmpipe')
    sw_render3 = SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3')
    # Qt offscreen platform — prevents "could not connect to display" crash
    # when running without X11/Wayland (WSL2 headless, CI, SSH).
    qt_offscreen = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='offscreen')

    world = LaunchConfiguration('world')

    # --headless-rendering: run Gazebo without a display window while still
    # processing sensor rendering (cameras, lidar) via an offscreen context.
    # -s: server-only mode (no GUI process). Combined with QT_QPA_PLATFORM=offscreen
    # this ensures fully headless operation on WSL2/CI.
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v', '3', world],
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
        gz_resource, sw_render1, sw_render2, sw_render3, qt_offscreen,
        gz_sim, clock_bridge,
    ])
