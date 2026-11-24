import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'mapping_bot'
    pkg_share = get_package_share_directory(pkg_name)
    install_share_dir = os.path.dirname(pkg_share)
    world_file = os.path.join(pkg_share, 'worlds', 'mapping_room.world')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[install_share_dir, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'mapping_bot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mapping_bot', '-z', '0.02'],
        output='screen'
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Node Odometri Sensor Fusion (Porting Firmware M5Stack Core Grey)
    mcu_odom_node = Node(
        package='mapping_bot',
        executable='mcu_odometry_node.py',
        name='mcu_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delay_joint_broad = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner, mcu_odom_node],
        )
    )

    return LaunchDescription([
        gazebo_model_path,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_joint_broad,
        delay_diff_drive
    ])
