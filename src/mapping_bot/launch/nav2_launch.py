import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('mapping_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_share, 'maps', 'my_room_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Jalankan Nav2 Bringup (output kecepatan otomatis ke /cmd_vel -> mcu_odometry_node)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_bringup
    ])
