# 四轮小车仿真项目 (Four Wheel Bot Simulation)

## 项目概述

这是一个基于ROS2和Gazebo的四轮小车仿真项目。项目提供了一个完整的机器人仿真环境，包括机器人模型、物理仿真、传感器集成和键盘控制功能。

### 主要功能

- **机器人模型**: 四轮差速驱动机器人，包含完整的URDF描述
- **仿真环境**: 集成Gazebo物理仿真引擎
- **可视化**: 支持RViz2可视化工具
- **控制接口**: 键盘控制节点，支持WASD控制
- **传感器仿真**: 里程计、TF变换等

## 系统依赖和安装要求

### 必需软件

- **ROS2 Humble** (推荐) 或 ROS2 Foxy
- **Gazebo 11** 或更高版本
- **Python 3.8+**

### ROS2包依赖

项目依赖以下ROS2包：

```bash
# 核心ROS2包
rclpy
std_msgs
geometry_msgs
sensor_msgs
nav_msgs
tf2_ros
visualization_msgs

# Gazebo相关
gazebo_ros_pkgs
gazebo_ros

# 工具包
robot_state_publisher
joint_state_publisher
rviz2
controller_manager
```

## 构建和运行说明

### 1. 创建工作空间

```bash
# 创建工作空间目录
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 复制项目到工作空间
cp -r /path/to/four_wheel_bot .

# 返回工作空间根目录
cd ~/ros2_ws
```

### 2. 安装依赖

```bash
# 更新包列表
sudo apt update

# 安装colcon构建工具
sudo apt install python3-colcon-common-extensions

# 安装项目依赖
rosdep install -i --from-path src --rosdistro humble -y
```

### 3. 构建项目

```bash
# 在工作空间根目录执行
colcon build
```

### 4. 配置环境

```bash
# 加载工作空间环境
source install/setup.bash
```

## 使用方法和控制说明

### 启动完整仿真系统

```bash
# 启动Gazebo仿真、RViz2和控制节点
ros2 launch four_wheel_bot four_wheel_bot.launch.py
```

### 启动键盘控制

```bash
# 在另一个终端中启动键盘控制
ros2 run four_wheel_bot teleop_keyboard
```

### 键盘控制指令

```
四轮小车键盘控制指令:
----------------------------
W: 前进
S: 后退  
A: 左转
D: 右转
空格: 停止
Q: 退出程序
----------------------------
```

### 测试控制功能

```bash
# 运行测试脚本验证控制功能
python3 test_teleop.py
```

## 项目文件结构

```
four_wheel_bot/
├── package.xml              # ROS2包配置文件
├── CMakeLists.txt           # CMake构建配置
├── setup.py                 # Python包安装配置
├── test_teleop.py           # 控制功能测试脚本
├── launch/
│   └── four_wheel_bot.launch.py  # 主启动文件
├── scripts/
│   └── teleop_keyboard.py   # 键盘控制节点
├── urdf/
│   └── four_wheel_bot.urdf.xacro  # 机器人URDF模型
├── config/
│   └── four_wheel_bot.rviz  # RViz2配置文件
└── worlds/
    └── four_wheel_bot.world  # Gazebo世界文件
```

## 故障排除指南

### 常见问题及解决方案

#### 1. Gazebo无法启动

**问题**: Gazebo服务器启动失败
**解决方案**:
```bash
# 检查Gazebo安装
gzserver --version

# 安装缺失的Gazebo模型
sudo apt install gazebo11

# 或者手动下载模型
cd ~/.gazebo
mkdir -p models
```

#### 2. RViz2显示问题

**问题**: RViz2无法显示机器人模型
**解决方案**:
- 检查`robot_state_publisher`节点是否正常运行
- 验证URDF文件是否正确解析
- 在RViz2中重新加载配置

#### 3. 键盘控制无响应

**问题**: 键盘控制节点无法发送命令
**解决方案**:
```bash
# 检查话题发布
ros2 topic list
ros2 topic echo /four_wheel_bot/cmd_vel

# 检查节点状态
ros2 node list
ros2 node info /teleop_keyboard
```

#### 4. 构建错误

**问题**: `colcon build`失败
**解决方案**:
```bash
# 清理构建缓存
rm -rf build install log

# 重新构建
colcon build
```

#### 5. 依赖缺失

**问题**: 缺少ROS2包
**解决方案**:
```bash
# 安装缺失的包
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-controller-manager
```

### 调试技巧

1. **检查节点状态**:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 service list
   ```

2. **查看TF变换**:
   ```bash
   ros2 run tf2_tools view_frames.py
   ```

3. **检查Gazebo模型**:
   ```bash
   gz model --list
   ```

4. **查看日志信息**:
   ```bash
   ros2 topic echo /rosout
   ```

## 技术规格

### 机器人参数

- **车身尺寸**: 0.4m × 0.2m × 0.1m
- **轮子半径**: 0.05m
- **轮子宽度**: 0.02m
- **轮距**: 0.3m (前后)
- **轴距**: 0.15m (左右)

### 控制参数

- **最大线速度**: 0.5 m/s
- **最大角速度**: 1.0 rad/s

## 许可证

本项目采用 Apache-2.0 许可证。

## 联系方式

如有问题或建议，请联系维护者:
- 邮箱: user@example.com