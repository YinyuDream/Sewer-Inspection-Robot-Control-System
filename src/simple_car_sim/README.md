# Simple Car Simulation (ROS 2 Humble + Ignition Gazebo)

这是一个基于 ROS 2 Humble 和 Ignition Gazebo (Fortress) 的四轮小车仿真项目。项目演示了如何构建 URDF 模型、配置 Gazebo 仿真环境、使用 ROS-Gazebo 桥接器以及在 RViz2 中进行可视化。

## 1. 环境要求

*   **操作系统**: Ubuntu 22.04 (Jammy Jellyfish)
*   **ROS 版本**: ROS 2 Humble Hawksbill
*   **Gazebo 版本**: Ignition Gazebo Fortress (Ignition 6)

## 2. 依赖安装

在编译之前，请确保安装了必要的依赖包：

```bash
sudo apt update
sudo apt install ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-teleop-twist-keyboard
```

## 3. 编译项目

进入你的工作空间根目录（例如 `~/Desktop/final_design`）：

```bash
cd ~/Desktop/final_design
colcon build --symlink-install
source install/setup.bash
```

## 4. 运行仿真

### 步骤 1: 启动仿真环境和可视化

运行 Launch 文件，这将启动：
1.  Ignition Gazebo 仿真环境（加载小车和世界）。
2.  `robot_state_publisher` 发布机器人的 TF 变换。
3.  `ros_gz_bridge` 用于 ROS 2 和 Gazebo 之间的通信。
4.  RViz2 用于可视化。

```bash
source install/setup.bash
ros2 launch simple_car_sim simulation.launch.py
```

### 步骤 2: 控制小车

打开一个新的终端窗口，运行键盘控制节点：

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

按照屏幕提示使用键盘（`i`, `j`, `l`, `,` 等）控制小车移动。

## 5. 项目结构说明

*   **urdf/**: 存放机器人的描述文件。
    *   `robot.urdf.xacro`: 定义了小车的物理模型（底盘、轮子）以及 Gazebo 的 `DiffDrive` 差速驱动插件。
*   **worlds/**: 存放 Gazebo 仿真世界文件。
    *   `car_world.sdf`: 定义了光照、地面和小车运行的物理环境。
*   **launch/**: 存放启动脚本。
    *   `simulation.launch.py`: 负责编排所有节点的启动顺序和参数配置。
*   **rviz/**: 存放 RViz 配置文件。
    *   `config.rviz`: 预设了 RobotModel 和 TF 的显示配置。

## 6. 关键技术点

*   **ROS-Gazebo Bridge**: 在 `launch/simulation.launch.py` 中配置，负责将 ROS 2 的 `/cmd_vel` (速度指令) 转发给 Gazebo，并将 Gazebo 的 `/odom` (里程计) 和 `/tf` (坐标变换) 转发回 ROS 2。
*   **DiffDrive Plugin**: 在 `urdf/robot.urdf.xacro` 中配置，使 Gazebo 能够根据速度指令模拟小车的差速运动。
