import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    ROS 2 Launch 文件
    功能：
    1. 启动 Gazebo 仿真环境，加载指定的世界文件 (pipe_world.sdf)。
    2. 加载机器人 URDF 模型，并将 xacro 转换为 urdf xml。
    3. 在 Gazebo 中生成 (Spawn) 机器人模型。
    4. 启动 Robot State Publisher 节点，发布 TF 坐标变换。
    5. 启动 ROS-Gazebo 桥接器 (Bridge)，连接 ROS 话题和 Gazebo 话题。
    6. 启动 RViz 可视化工具。
    """
    
    # 获取包的共享目录路径
    pkg_share = get_package_share_directory('simple_car_sim')

    # World file
    # 指定 Gazebo 世界文件路径
    world_file = os.path.join(pkg_share, 'worlds', 'pipe_world.sdf')
    
    # URDF file
    # 指定机器人描述文件路径 (Xacro)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    # 使用 xacro 命令处理文件，生成 URDF 字符串
    robot_description = Command(['xacro ', xacro_file])
    
    # Gazebo Sim
    # 包含 ros_gz_sim 包中的 gz_sim.launch.py 来启动 Gazebo
    # -r 参数表示加载指定的世界文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn Robot
    # 启动 ros_gz_sim 的 create 节点，在仿真中生成实体
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', # 从该话题获取机器人描述
                   '-name', 'simple_car_v2',      # 在 Gazebo 中的模型名称
                   '-x', '0.0',                   # 初始位置 X
                   '-y', '-2.5',                  # 初始位置 Y
                   '-z', '0.0',                   # 初始位置 Z
                   '-R', '0.0',                   # 初始姿态 Roll
                   '-P', '0.0',                   # 初始姿态 Pitch
                   '-Y', '0.0',                   # 初始姿态 Yaw
                   # Start fully retracted to avoid explosion
                   # 设置关节初始位置：完全收缩状态，避免生成时即发生剧烈碰撞
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
    # 启动机器人状态发布者，将 URDF 中的关节关系发布为 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}] # 使用仿真时间
    )

    # Bridge
    # 启动参数桥接器，在 ROS 2 和 Gazebo Transport 之间转发消息
    # 格式: /ros_topic@ros_msg_type@gazebo_msg_type
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Joint states (发布关节状态)
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            # Wheel velocity control (驱动轮速度指令)
            # 注意：这里订阅的是 ROS 的 cmd_vel 话题，转发给 Gazebo 的 Joint Controller
            '/model/simple_car_v2/joint/drive_wheel_top_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_top_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_bottom_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_bottom_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_left_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_left_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_right_front_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_wheel_right_rear_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # Arm control (机械臂控制)
            '/model/simple_car_v2/joint/joint_pitch/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/joint_yaw/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # Expansion control (Drive Wheels) (驱动节伸缩控制)
            '/model/simple_car_v2/joint/drive_expansion_joint_top/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_expansion_joint_bottom/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_expansion_joint_left/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/drive_expansion_joint_right/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # Expansion control (Functional Legs) (功能节伸缩控制)
            '/model/simple_car_v2/joint/expansion_joint_top/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/expansion_joint_bottom/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/expansion_joint_left/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/simple_car_v2/joint/expansion_joint_right/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            # IMU
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            # Odometry (Ground Truth) (里程计)
            '/model/simple_car_v2/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        ],
        remappings=[
            # 为了方便使用，将 Gazebo 复杂的长话题名映射为简短的 ROS 话题名
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
