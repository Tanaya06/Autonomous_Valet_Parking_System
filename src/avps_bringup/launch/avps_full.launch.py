import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────
    pkg_gazebo  = get_package_share_directory('avps_gazebo')
    pkg_slam    = get_package_share_directory('avps_slam')
    pkg_nav     = get_package_share_directory('avps_navigation')
    pkg_bringup = get_package_share_directory('avps_bringup')

    # ── Arguments ─────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='slam = build map | navigation = use saved map'
    )

    # ── 1. Gazebo Harmonic + Robot + Bridge ───────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    # ── 2. SLAM (map building mode) ───────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        )
    )

    # ── 3. Navigation (localization + planning mode) ──────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        )
    )

    # ── 4. RViz2 ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'avps.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        mode_arg,
        LogInfo(msg="=== AVPS Lite System Starting (ROS Jazzy / Gz Harmonic) ==="),
        gazebo_launch,
        # Delay other nodes to let Gazebo and the bridge fully initialize
        TimerAction(period=6.0, actions=[
            slam_launch,
            LogInfo(msg="SLAM started."),
        ]),
        TimerAction(period=8.0, actions=[
            rviz_node,
            LogInfo(msg="RViz2 started."),
        ]),
    ])