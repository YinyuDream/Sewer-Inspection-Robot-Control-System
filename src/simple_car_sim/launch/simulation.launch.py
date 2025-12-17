import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
                   '-z', '0.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   # Start fully retracted to avoid explosion
                   '-J', 'drive_expansion_joint_top', '0.0',
                   '-J', 'drive_expansion_joint_bottom', '0.0',
                   '-J', 'drive_expansion_joint_left', '0.0',
                   '-J', 'drive_expansion_joint_right', '0.0',
                   '-J', 'expansion_joint_top', '0.0',
                   '-J', 'expansion_joint_bottom', '0.0',
                   '-J', 'expansion_joint_left', '0.0',
                   '-J', 'expansion_joint_right', '0.0'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
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
            '/model/simple_car_v2/joint/drive_wheel_top_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_top_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_bottom_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_bottom_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_left_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_left_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_right_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_right_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # Arm control
            '/model/simple_car_v2/joint/joint_pitch/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/joint_yaw/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # Expansion control (Drive Wheels)
            '/model/simple_car_v2/joint/drive_expansion_joint_top/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_expansion_joint_bottom/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_expansion_joint_left/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_expansion_joint_right/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # Expansion control (Functional Legs)
            '/model/simple_car_v2/joint/expansion_joint_top/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/expansion_joint_bottom/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/expansion_joint_left/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/expansion_joint_right/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # IMU
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            # Odometry (Ground Truth)
            '/model/simple_car_v2/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/simple_car_v2/joint/drive_wheel_top_front_joint/cmd_vel', '/simple_car/top_front_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_top_rear_joint/cmd_vel', '/simple_car/top_rear_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_bottom_front_joint/cmd_vel', '/simple_car/bottom_front_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_bottom_rear_joint/cmd_vel', '/simple_car/bottom_rear_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_left_front_joint/cmd_vel', '/simple_car/left_front_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_left_rear_joint/cmd_vel', '/simple_car/left_rear_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_right_front_joint/cmd_vel', '/simple_car/right_front_wheel_vel'),
            ('/model/simple_car_v2/joint/drive_wheel_right_rear_joint/cmd_vel', '/simple_car/right_rear_wheel_vel'),
            ('/model/simple_car_v2/joint/joint_pitch/cmd_vel', '/simple_car/joint_pitch_vel'),
            ('/model/simple_car_v2/joint/joint_yaw/cmd_vel', '/simple_car/joint_yaw_vel'),
            ('/model/simple_car_v2/joint/drive_expansion_joint_top/cmd_vel', '/simple_car/drive_exp_top_vel'),
            ('/model/simple_car_v2/joint/drive_expansion_joint_bottom/cmd_vel', '/simple_car/drive_exp_bottom_vel'),
            ('/model/simple_car_v2/joint/drive_expansion_joint_left/cmd_vel', '/simple_car/drive_exp_left_vel'),
            ('/model/simple_car_v2/joint/drive_expansion_joint_right/cmd_vel', '/simple_car/drive_exp_right_vel'),
            ('/model/simple_car_v2/joint/expansion_joint_top/cmd_vel', '/simple_car/func_exp_top_vel'),
            ('/model/simple_car_v2/joint/expansion_joint_bottom/cmd_vel', '/simple_car/func_exp_bottom_vel'),
            ('/model/simple_car_v2/joint/expansion_joint_left/cmd_vel', '/simple_car/func_exp_left_vel'),
            ('/model/simple_car_v2/joint/expansion_joint_right/cmd_vel', '/simple_car/func_exp_right_vel'),
            ('/model/simple_car_v2/odometry', '/odom'),
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
