import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ramabot_mapping')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox_mapping.yaml')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x', '0.10', '--y', '0.0', '--z', '0.15',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_footprint', '--child-frame-id', 'laser_frame'
            ]
        ),
        Node(
            package='ramabot_mapping',
            executable='wsl_serial_mock',
            name='wsl_serial_mock',
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config, {'use_sim_time': False}],
            output='screen'
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765}],
            output='screen'
        ),
    ])
