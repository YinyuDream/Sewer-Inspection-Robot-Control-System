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
    world_file = os.path.join(pkg_share, 'worlds', 'pipe_world.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'simple_car',
            '-x', '5.0',
            '-y', '-9.6',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        output='both', 
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str), 'use_sim_time': True}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/simple_car/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/model/simple_car/joint/wheel_rl_joint/cmd_force@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car/joint/wheel_rr_joint/cmd_force@std_msgs/msg/Float64@ignition.msgs.Double',
            '/cmd_pos_fl@std_msgs/msg/Float64@ignition.msgs.Double',
            '/cmd_pos_fr@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car/joint/wheel_rr_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car/joint/wheel_rl_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/model/simple_car/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            
            
            #'/clock@rosgraph_msgs/msg/Clock[ign.msgs.Clock',
            #'/world/pipe_world/control@ros_gz_interfaces/srv/ControlWorld' # <--- 关键桥接
        ],
        remappings=[
            ('/model/simple_car/joint/wheel_rl_joint/cmd_force', '/cmd_force_rl'),
            ('/model/simple_car/joint/wheel_rr_joint/cmd_force', '/cmd_force_rr'),
            ('/model/simple_car/joint/wheel_rl_joint/cmd_vel', '/cmd_vel_rl'),
            ('/model/simple_car/joint/wheel_rr_joint/cmd_vel', '/cmd_vel_rr'),
            ('/model/simple_car/odometry', '/odom'),
            ('/model/simple_car/joint_states', '/joint_states'),
            #('/model/simple_car/joint_states', '/joint_states_raw'),
        ],
        output='screen'
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        arguments=[
            'messages',           # 模式3
            '/joint_states_raw',  # 输入话题 (高频)
            '100.0',               # 目标频率 (Hz)
            '/joint_states'       # 输出话题 (低频)
        ],
        parameters=[{'use_sim_time': True}],  # <--- 必须加上这一句！
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        #rviz,
        #throttle_node
    ])
