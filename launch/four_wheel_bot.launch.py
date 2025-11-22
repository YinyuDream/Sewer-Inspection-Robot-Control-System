#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包目录路径
    pkg_share = FindPackageShare(package='four_wheel_bot').find('four_wheel_bot')
    
    # 启动参数定义
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_share, 'worlds', 'four_wheel_bot.world'))
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(pkg_share, 'urdf', 'four_wheel_bot.urdf.xacro'))
    rviz_config = LaunchConfiguration('rviz_config', default=os.path.join(pkg_share, 'config', 'four_wheel_bot.rviz'))
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='Full path to world file to load'
    )
    
    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=urdf_file,
        description='Full path to URDF/XACRO file'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Full path to RViz config file'
    )

    # 将XACRO文件转换为URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_file,
            ' ',
            'use_sim:=', use_sim_time
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}

    # 启动robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # 启动Gazebo服务器
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # 启动Gazebo客户端
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'four_wheel_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # 启动RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 启动关节状态发布器
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 启动差分驱动控制器
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    # 启动关节轨迹控制器
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 延迟启动控制器，确保Gazebo已经加载
    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_controller],
        )
    )

    delayed_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_trajectory_controller],
        )
    )

    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_urdf_file_cmd)
    ld.add_action(declare_rviz_config_cmd)
    
    # 添加节点和进程
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_entity)
    ld.add_action(rviz_node)
    
    # 添加延迟启动的控制器
    ld.add_action(delayed_diff_drive_controller)
    ld.add_action(delayed_joint_trajectory_controller)

    return ld