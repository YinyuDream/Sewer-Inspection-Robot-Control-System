#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock')

    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_share, 'worlds', 'empty.world'),
        description='Gazebo (Fortress) world file')

    # Robot description via xacro
    robot_description_content = Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'my_bot.urdf.xacro'])])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )

    # Gazebo Fortress via ros_gz_sim
    # This uses gz_sim.launch.py from ros_gz_sim to start the simulator
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # Bridge robot_description -> SDF/Entity spawn using ros_gz_sim_create
    # ros_gz_sim 里推荐：直接在 world 里 include 模型，或用 create 工具；
    # 这里采用 spawn_entity 类似方式：从 /robot_description 生成实体。
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_bot', '-topic', 'robot_description'],
        output='screen'
    )

    # Controller manager + controllers (ros2_control)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg_share, 'config', 'ros2_control.yaml'), {'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delayed_joint_state = TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])
    delayed_diff_drive = TimerAction(period=3.5, actions=[diff_drive_spawner])

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gz_sim_launch,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager,
        delayed_joint_state,
        delayed_diff_drive
    ])
