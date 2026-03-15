# 终止所有相关的进程 (清理环境)
pkill -f "ign gazebo"       # 终止 Ignition Gazebo 仿真器
pkill -f "ros2"             # 终止其他 ROS 2 进程
pkill -f "rviz"             # 终止 RViz 可视化工具
pkill -f "parameter_bridge" # 终止 ROS-Gazebo 桥接器

# 使用 colcon 编译工作空间
# --symlink-install: 使用软链接安装脚本和配置文件，修改 Python/Launch/URDF 后无需重新编译即可生效
colcon build --packages-select simple_car_sim
# 刷新环境变量以包含新编译的包
source install/setup.bash

# 启动仿真 (运行 simulation.launch.py)
# 这会启动 Gazebo, 加载机器人, 启动控制器等
ros2 launch simple_car_sim simulation.launch.py