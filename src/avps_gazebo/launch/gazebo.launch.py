import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo   = get_package_share_directory('avps_gazebo')
    pkg_desc     = get_package_share_directory('avps_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file   = os.path.join(pkg_gazebo, 'worlds', 'parking_lot.world')
    urdf_file    = os.path.join(pkg_desc,   'urdf',   'avps_robot.urdf.xacro')
    robot_desc   = Command(['xacro ', urdf_file])

    return LaunchDescription([

        # ── 1. Launch Gazebo with our parking lot world ───────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'false',
                'pause': 'false',
            }.items()
        ),

        # ── 2. Robot State Publisher ──────────────────────────────
        # Reads URDF and publishes TF for all joints
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }]
        ),

        # ── 3. Spawn the robot in Gazebo ──────────────────────────
        # Delay by 3 seconds to let Gazebo finish loading the world
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_robot',
                    output='screen',
                    arguments=[
                        '-entity', 'avps_robot',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.1',
                        '-Y', '0.0',
                    ]
                )
            ]
        ),
    ])
