import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav  = get_package_share_directory('avps_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('avps_slam'),
            'maps', 'parking_lot.yaml'
        ),
        description='Full path to the map yaml file'
    )

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':         LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart':   'true',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        nav2_launch,
    ])