# ROS2 四轮小车仿真项目

这是一个基于ROS2 Humble和Ignition Gazebo的四轮小车仿真项目，包含完整的URDF模型、Gazebo仿真环境、键盘控制和RViz2可视化。

## 项目结构

```
my_bot/
├── package.xml              # ROS2包配置文件
├── CMakeLists.txt           # CMake构建配置
├── urdf/
│   └── my_bot.urdf.xacro    # 四轮小车URDF模型
├── worlds/
│   └── empty.world          # Gazebo仿真世界
├── config/
│   └── my_bot.rviz          # RViz2配置文件
├── launch/
│   ├── my_bot_launch.py     # 主启动文件（传统Gazebo）
│   └── my_bot_ignition.launch.py  # Ignition Gazebo启动文件
├── my_bot/
│   ├── __init__.py
│   ├── keyboard_control.py  # 键盘控制节点
│   └── diff_drive_controller.py  # 差速控制器节点
└── test_system.sh           # 系统测试脚本
```

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Ignition Gazebo Fortress (v6)
- Python 3.10

## 安装依赖

```bash
sudo apt update
sudo apt install -y ros-humble-ros-ign-gazebo ros-humble-robot-state-publisher \
ros-humble-joint-state-publisher ros-humble-xacro ros-humble-rviz2
```

## 构建项目

```bash
cd src/my_bot
colcon build --packages-select my_bot
source install/setup.bash
```

## 运行系统

### 使用Ignition Gazebo（推荐）

```bash
cd src/my_bot
source install/setup.bash
ros2 launch my_bot my_bot_ignition.launch.py
```

### 键盘控制说明

启动系统后，在键盘控制终端中使用以下键控制小车：

- **W**: 前进
- **S**: 后退  
- **A**: 左转
- **D**: 右转
- **空格**: 停止
- **Q**: 退出程序

## 系统组件

### 1. URDF模型
- 四轮差速驱动小车
- 蓝色车身，黑色轮子
- 完整的物理属性和碰撞检测
- Gazebo插件集成

### 2. 控制节点
- **keyboard_control.py**: 键盘输入转换为cmd_vel命令
- **diff_drive_controller.py**: 将cmd_vel转换为轮子速度

### 3. 仿真环境
- 平坦地面世界
- 物理引擎集成
- 可视化界面

### 4. 可视化
- RViz2用于机器人状态可视化
- TF坐标变换显示
- 实时位姿跟踪

## 技术特性

- **差速驱动模型**: 实现精确的运动控制
- **ROS2通信**: 使用标准消息类型（Twist, Float64）
- **Gazebo集成**: 真实的物理仿真
- **模块化设计**: 易于扩展和修改

## 故障排除

### Python环境冲突
如果遇到Python模块导入错误，请确保使用正确的Python环境：

```bash
# 退出conda环境（如果正在使用）
conda deactivate

# 使用系统Python环境
source /opt/ros/humble/setup.bash
```

### Gazebo启动问题
如果Gazebo无法启动，检查Ignition Gazebo安装：

```bash
ign gazebo --versions
```

### 权限问题
确保所有Python脚本有执行权限：

```bash
chmod +x my_bot/*.py
```

## 扩展功能

可以扩展的功能包括：
- 添加传感器（摄像头、激光雷达）
- 实现SLAM和导航
- 添加PID控制器
- 支持多种控制方式（游戏手柄、Web界面）

## 许可证

Apache 2.0