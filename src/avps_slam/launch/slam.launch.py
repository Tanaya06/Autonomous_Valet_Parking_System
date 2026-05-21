import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam = get_package_share_directory('avps_slam')
    slam_cfg = os.path.join(pkg_slam, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([

        # SLAM Toolbox — Online Async Mode
        # Subscribes to /scan (LaserScan)
        # Subscribes to /tf (odom → base_footprint)
        # Publishes /map (OccupancyGrid)
        # Publishes /tf (map → odom)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_cfg, {'use_sim_time': True}],
        ),
    ])