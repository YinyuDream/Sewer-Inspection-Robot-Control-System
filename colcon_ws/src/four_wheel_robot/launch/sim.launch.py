from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("four_wheel_robot"))
    xacro_file = package_share / "urdf" / "four_wheel_robot.urdf.xacro"
    rviz_config = package_share / "rviz" / "robot.rviz"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    world_path = LaunchConfiguration("world")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            str(xacro_file),
        ]),
        value_type=str,
    )

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock provided by Gazebo.",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 alongside Gazebo.",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=str(package_share / "worlds" / "flat_world.sdf"),
        description="Ignition Gazebo world file to load.",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world_path],
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "four_wheel_robot", "-topic", "robot_description"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(rviz_config)],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_sim_time,
            declare_use_rviz,
            declare_world,
            robot_state_publisher,
            gz_sim,
            spawn_entity,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=3.0, actions=[diff_drive_spawner]),
            rviz,
        ]
    )
