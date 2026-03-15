# 管道巡检机器人 (Pipe-Inspection Robot)

基于 ROS 2 + Ignition Gazebo + STM32F407 的四轮 Ackermann 管道巡检机器人项目，支持：

- **SIL（Software-in-the-Loop）**：纯仿真闭环控制（ROS 2 算法 + Gazebo 物理环境）
- **HIL（Hardware-in-the-Loop）**：仿真传感器经 CAN 总线驱动真实 STM32 固件控制器

> **说明**：本文件仅保留快速上手概要。完整架构、参数、CAN 映射、测试结果请见 `Explain.md`。

## 项目亮点

- 四轮 Ackermann 转向模型，支持封闭管道路径自主巡航。
- Python（ROS 2）与 C（STM32）控制逻辑参数对齐，便于 SIL/HIL 对照验证。
- 支持 ROS 2 ↔ SocketCAN ↔ STM32 的双向链路。

## 环境依赖

### ROS 2 侧
- Ubuntu 22.04
- ROS 2 Humble（或兼容）
- Ignition Gazebo（Fortress 或兼容）
- `ros_gz_sim`、`ros_gz_bridge`、`rviz2`、`xacro`
- `python-can`、`ros2_socketcan`

### 固件侧
- ARM 交叉编译工具链：`gcc-arm-none-eabi`
- `cmake`（>= 3.22）
- `ninja-build`

## 快速开始

辅助脚本位于 `scripts/` 目录。

### 1) 启动完整仿真（推荐，SIL）

```bash
bash scripts/Gazebo.sh
```

该脚本会构建 `simple_car_sim` 并启动 `simulation.launch.py`（Gazebo + 机器人 + RViz）。

### 2) 单独运行控制节点（SIL）

```bash
bash scripts/SIL.sh
# 或
source install/setup.bash
ros2 run simple_car_sim autonomous_control.py --ros-args -p use_sim_time:=true
```

### 3) 启动 HIL 通信与控制链路

```bash
bash scripts/HIL.sh
```

该脚本启动 `robot_system.launch.py`（`ros2_socketcan` + `can_transceiver.py` + `autonomous_control.py`）。

> HIL 需要实际硬件（STM32 + CAN 设备，如 `can0`）。SocketCAN 配置、虚拟 CAN 测试与刷写流程请见 `Explain.md`。

## 文档说明

- `README.md`：概要与快速上手。
- `Explain.md`：详细系统设计、参数、通信协议、构建与测试说明。

## 许可证

MIT License，详见 `LICENSE`。
