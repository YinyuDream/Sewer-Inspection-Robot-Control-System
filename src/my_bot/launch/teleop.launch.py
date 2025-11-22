#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='my_bot',
        executable='teleop_keyboard.py',
        name='teleop_keyboard',
        output='screen'
    )

    return LaunchDescription([
        teleop_node
    ])
