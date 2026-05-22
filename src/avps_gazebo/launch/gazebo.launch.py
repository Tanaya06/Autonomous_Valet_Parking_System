import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_gazebo    = get_package_share_directory('avps_gazebo')
    pkg_desc      = get_package_share_directory('avps_description')
    pkg_ros_gz    = get_package_share_directory('ros_gz_sim')

    world_file    = os.path.join(pkg_gazebo, 'worlds', 'parking_lot.world')
    urdf_file     = os.path.join(pkg_desc,   'urdf',   'avps_robot.urdf.xacro')
    robot_desc    = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([

        # ── 1. Launch Gazebo Harmonic with our world ──────────────────
        # Uses gz_sim.launch.py from ros_gz_sim (not gazebo_ros)
        # -r flag starts the simulation running immediately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
            }.items()
        ),

        # ── 2. Robot State Publisher ──────────────────────────────────
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

        # ── 3. Spawn the robot in Gazebo ──────────────────────────────
        # Delay by 3 seconds to let Gz Harmonic finish loading the world
        # Uses ros_gz_sim 'create' executable (not gazebo_ros spawn_entity)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_robot',
                    output='screen',
                    arguments=[
                        '-name',  'avps_robot',
                        '-topic', '/robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.1',
                        '-Y', '0.0',
                    ]
                )
            ]
        ),

        # ── 4. ROS ↔ Gazebo Topic Bridge ─────────────────────────────
        # Bridges Gz internal topics to ROS 2 topics.
        # Format: /ros_topic@ros_type[gz_type  (gz → ros, one-way)
        #         /ros_topic@ros_type]gz_type  (ros → gz, one-way)
        # Delay slightly to let the robot spawn first.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge',
                    output='screen',
                    arguments=[
                        # Simulation clock (gz → ros) — required for use_sim_time
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        # Velocity command (ros → gz)
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        # Odometry (gz → ros)
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        # LiDAR scan (gz → ros)
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        # TF from diff drive plugin (gz → ros)
                        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        # Joint states for robot_state_publisher (gz → ros)
                        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    ]
                )
            ]
        ),
    ])