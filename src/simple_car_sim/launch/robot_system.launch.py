import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 定义一个可以在启动时修改的变量 (也可以直接硬编码为 True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 查找 ros2_socketcan 包中自带的桥接启动文件
    socketcan_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('ros2_socketcan'), '/launch/socket_can_bridge.launch.xml'
        ]),
        launch_arguments={
            'interface': 'vcan0',
            'use_sim_time': use_sim_time,  # 尝试传递给包含的 launch 文件
            'receiver_interval_sec': '0.01',
            'sender_timeout_ns': '100000000'  
        }.items()
    )

    return LaunchDescription([
        # 声明参数，这样你可以在命令行通过 use_sim_time:=false 关闭它
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # 1. 自动拉起和激活 socket_can_receiver 与 socket_can_sender
        socketcan_bridge_launch,

        # 2. CAN 适配层
        Node(
            package='simple_car_sim',
            executable='can_transceiver.py',
            name='can_transceiver',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}] # <--- 关键修改
        ),

        # 3. 自动控制算法层
        Node(
            package='simple_car_sim',
            executable='autonomous_control.py',
            name='autonomous_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}] # <--- 关键修改
        )
    ])