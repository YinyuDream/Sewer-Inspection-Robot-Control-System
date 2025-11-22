import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_car_sim')
    
    # World file
    world_file = os.path.join(pkg_share, 'worlds', 'pipe_world.sdf')
    
    # URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'simple_car_v2',
                   '-x', '0.0',
                   '-y', '-2.5',
                   '-z', '-0.6',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            # Wheel velocity control
            '/model/simple_car_v2/joint/front_left_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/rear_left_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/front_right_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/rear_right_wheel_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
        ],
        remappings=[
            ('/model/simple_car_v2/joint/front_left_wheel_joint/cmd_vel', '/simple_car/front_left_wheel_vel'),
            ('/model/simple_car_v2/joint/rear_left_wheel_joint/cmd_vel', '/simple_car/rear_left_wheel_vel'),
            ('/model/simple_car_v2/joint/front_right_wheel_joint/cmd_vel', '/simple_car/front_right_wheel_vel'),
            ('/model/simple_car_v2/joint/rear_right_wheel_joint/cmd_vel', '/simple_car/rear_right_wheel_vel'),
        ],
        output='screen'
    )

    # RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        rviz
    ])
