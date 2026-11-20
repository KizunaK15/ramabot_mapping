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
    slam_config = os.path.join(pkg_ramabot, 'config', 'slam_toolbox_mapping.yaml')

    # Argumen kalibrasi cepat tanpa perlu edit kode:
    # lidar_yaw: '0.0' jika kabel/motor RPLidar menghadap BELAKANG robot (sensor menghadap depan)
    #            '3.14159' jika kabel/motor RPLidar menghadap DEPAN robot
    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='Yaw offset (rad) dari base_link ke laser_frame (0.0 atau 3.14159)'
    )
    invert_yaw_arg = DeclareLaunchArgument(
        'invert_yaw',
        default_value='true',
        description='Balik tanda yaw odom dari M5Stack jika berlawanan arah putaran LiDAR'
    )

    # 1. Node Bridge Serial M5Stack Core Grey -> /odom & TF (odom -> base_footprint)
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

    # 2. Static TF: base_footprint -> base_link (tinggi pusat roda = 0.067 m)
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

    # 3. Static TF: base_link -> laser_frame (diselaraskan dengan URDF Ramabot_V3)
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_laser',
        arguments=[
            '--x', '0.05', '--y', '0.0', '--z', '0.12',
            '--yaw', LaunchConfiguration('lidar_yaw'), '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'
        ]
    )

    # 4. Driver RPLidar A2M8 (/dev/rplidar @ 115200 bps)
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

    # 5. Online Async SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config]
    )

    return LaunchDescription([
        lidar_yaw_arg,
        invert_yaw_arg,
        odom_bridge_node,
        tf_base_link,
        tf_laser,
        sllidar_launch,
        slam_toolbox_node
    ])