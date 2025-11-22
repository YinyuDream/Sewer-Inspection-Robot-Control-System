from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")

    declare_linear = DeclareLaunchArgument(
        "linear_speed",
        default_value="0.6",
        description="Maximum linear speed (m/s) for teleoperation.",
    )

    declare_angular = DeclareLaunchArgument(
        "angular_speed",
        default_value="1.2",
        description="Maximum angular speed (rad/s) for teleoperation.",
    )

    teleop = Node(
        package="four_wheel_robot",
        executable="keyboard_teleop",
        output="screen",
        parameters=[
            {"linear_speed": linear_speed},
            {"angular_speed": angular_speed},
        ],
    )

    return LaunchDescription([declare_linear, declare_angular, teleop])
