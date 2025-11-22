# my_bot 四轮差速小车 (ROS2 Humble + Gazebo Fortress + RViz2)

## 功能概述
- 四轮差速驱动（左右各两个驱动轮）
- Gazebo Fortress 仿真
- ros2_control + diff_drive_controller 控制轮速
- 键盘遥控 (单终端) 发布 Twist 指令
- RViz2 可视化模型、TF、里程计

## 目录结构
```
final_design/
  ├── src/
  │   └── my_bot/
  │       ├── package.xml
  │       ├── CMakeLists.txt
  │       ├── urdf/my_bot.urdf.xacro
  │       ├── config/ros2_control.yaml
  │       ├── launch/
  │       │    ├── gazebo.launch.py
  │       │    ├── rviz.launch.py
  │       │    └── teleop.launch.py
  │       ├── scripts/teleop_keyboard.py
  │       └── rviz/my_bot.rviz
  └── README.md
```

## 构建步骤
请确保已安装 ROS2 Humble 与 Gazebo Fortress，并已 `source /opt/ros/humble/setup.bash`。

```bash
# 进入工作空间根目录
cd /home/yinyudream/Desktop/final_design
# 如果首次构建，建立构建/安装目录结构
colcon build --symlink-install
# 构建完成后 source 本地工作空间
source install/setup.bash
```

## 运行仿真与控制
在不同终端中执行：

```bash
# 终端1: 启动 Gazebo + 机器人 + 控制器
ros2 launch my_bot gazebo.launch.py

# 等待模型加载与控制器spawner启动(约3-4秒)

# 终端2: 启动键盘控制
ros2 launch my_bot teleop.launch.py
# 使用 w/s/a/d/空格 控制，q 退出

# 终端3: 启动 RViz2 可视化 (可选)
ros2 launch my_bot rviz.launch.py
```

Gazebo 中应看到小车，键盘输入会使其移动；RViz2 中可查看 TF 与里程计 `/diff_drive_controller/odom`。

## 话题与接口
- 指令: `/diff_drive_controller/cmd_vel` (geometry_msgs/Twist)
- 里程计: `/diff_drive_controller/odom`
- JointState: `/joint_states`

## 常见问题
1. 机器人不动: 确认 diff_drive_controller 已成功 spawner (查看终端输出)。
2. 里程计不更新: 确认已发布 Twist 且未超时 (`cmd_vel_timeout`).
3. xacro 找不到: 安装 `sudo apt install ros-humble-xacro` 并重建。
4. Gazebo 插件报错: 确认安装 `gazebo_ros2_control` 包。

## 后续扩展建议
- 添加激光雷达/深度相机传感器。
- 引入导航栈 (nav2) 做路径规划。
- 实现真实硬件接口替换 GazeboSystem。
- 增加速度平滑与安全停障节点。

## 许可证
Apache-2.0
