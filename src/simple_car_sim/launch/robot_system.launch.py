import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 查找 ros2_socketcan 包中自带的桥接启动文件
    socketcan_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('ros2_socketcan'), '/launch/socket_can_bridge.launch.xml'
        ]),
        launch_arguments={'interface': 'can0'}.items()
    )

    return LaunchDescription([
        # 1. 自动拉起和激活 socket_can_receiver 与 socket_can_sender
        socketcan_bridge_launch,

        # 2. CAN 适配层 (翻译 ROS 话题 <-> CAN 帧)
        Node(
            package='simple_car_sim',
            executable='can_transceiver.py',
            name='can_transceiver',
            output='screen'
        ),

        # 3. 自动控制算法层 (业务逻辑)
        Node(
            package='simple_car_sim',
            executable='autonomous_control.py',
            name='autonomous_control',
            output='screen'
        )
    ])
