import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包目录
    pkg_path = get_package_share_directory('my_bot')
    
    # Ignition Gazebo启动
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'ign_args': os.path.join(pkg_path, 'worlds', 'empty.world') + ' -r'
        }.items()
    )

    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('my_bot'),
                    'urdf',
                    'my_bot.urdf.xacro'
                ])
            ])
        }]
    )

    # 在Ignition Gazebo中生成机器人
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1'
        ],
        output='screen'
    )

    # 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz2节点
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'config', 'my_bot.rviz')],
        output='screen'
    )

    # 键盘控制节点
    keyboard_control = Node(
        package='my_bot',
        executable='keyboard_control.py',
        name='keyboard_control',
        output='screen'
    )

    # 差速控制器节点
    diff_drive_controller = Node(
        package='my_bot',
        executable='diff_drive_controller.py',
        name='diff_drive_controller',
        output='screen'
    )

    # 延迟启动控制节点，确保其他节点先启动
    delayed_controls = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[keyboard_control, diff_drive_controller],
        )
    )

    return LaunchDescription([
        ign_gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rviz2,
        delayed_controls,
    ])