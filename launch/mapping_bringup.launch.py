import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ramabot_mapping')
    bot_config = os.path.join(pkg_dir, 'config', 'mapping_bot.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox_mapping.yaml')

    lidar_port = LaunchConfiguration('lidar_port', default='/dev/rplidar')
    esp_port = LaunchConfiguration('esp_port', default='/dev/esp32_map')
    lidar_x = LaunchConfiguration('lidar_x', default='0.10')
    lidar_y = LaunchConfiguration('lidar_y', default='0.00')
    lidar_z = LaunchConfiguration('lidar_z', default='0.15')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('esp_port', default_value='/dev/esp32_map'),
        DeclareLaunchArgument('lidar_x', default_value='0.10'),
        DeclareLaunchArgument('lidar_y', default_value='0.00'),
        DeclareLaunchArgument('lidar_z', default_value='0.15'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                '--yaw', lidar_yaw, '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_footprint', '--child-frame-id', 'laser_frame'
            ]
        ),

        Node(
            package='ramabot_mapping',
            executable='mapping_odom_bridge',
            name='mapping_odom_bridge',
            parameters=[bot_config, {'serial_port': esp_port}],
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
