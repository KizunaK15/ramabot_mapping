import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ramabot = get_package_share_directory('ramabot_mapping')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    bot_config = os.path.join(pkg_ramabot, 'config', 'mapping_bot.yaml')

    lidar_yaw_arg = DeclareLaunchArgument('lidar_yaw', default_value='0.0')
    lidar_roll_arg = DeclareLaunchArgument('lidar_roll', default_value='0.0')
    invert_yaw_arg = DeclareLaunchArgument('invert_yaw', default_value='false')

    odom_bridge_node = Node(
        package='ramabot_mapping',
        executable='mapping_odom_bridge',
        name='mapping_odom_bridge',
        output='screen',
        parameters=[
            bot_config,
            {'invert_yaw': LaunchConfiguration('invert_yaw')}
        ]
    )

    tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint_to_base_link',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.067',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ]
    )

    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_laser',
        arguments=[
            '--x', '0.05', '--y', '0.0', '--z', '0.12',
            '--yaw', LaunchConfiguration('lidar_yaw'),
            '--pitch', '0.0',
            '--roll', LaunchConfiguration('lidar_roll'),
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'
        ]
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a2m8_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '115200',
            'frame_id': 'laser_frame',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Sensitivity'
        }.items()
    )

    return LaunchDescription([
        lidar_yaw_arg,
        lidar_roll_arg,
        invert_yaw_arg,
        odom_bridge_node,
        tf_base_link,
        tf_laser,
        sllidar_launch
    ])