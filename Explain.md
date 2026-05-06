# 管道巡检机器人 (Pipe-Inspection Robot)

一个基于 ROS 2 + Ignition Gazebo 的四轮 Ackermann 转向管道巡检机器人仿真与嵌入式控制系统。项目包含完整的软件在环（SIL）仿真层和运行于 STM32F407xx 的实时嵌入式固件，两者实现完全相同的 Stanley 路径跟踪算法，可通过 CAN 总线进行硬件在环（HIL）联合测试。

---

## 目录

- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [目录结构](#目录结构)
- [技术栈](#技术栈)
- [机器人物理模型](#机器人物理模型)
- [轨道几何](#轨道几何)
- [控制算法](#控制算法)
- [CAN 通信协议](#can-通信协议)
- [ROS 2 仿真层](#ros-2-仿真层)
- [STM32 嵌入式固件](#stm32-嵌入式固件)
- [构建与运行](#构建与运行)
- [配置参考](#配置参考)
- [API 接口](#api-接口)
- [测试与验证](#测试与验证)
- [常用命令](#常用命令)

---

## 项目概述

本项目实现了一个**四轮 Ackermann 转向管道巡检机器人**，机器人在一条封闭的圆角矩形管道轨道内自主运行。系统分为两个紧密耦合的子系统：

- **ROS 2 / Ignition Gazebo 仿真层**（运行于 Linux）：提供软件在环（SIL）测试环境，以及通过物理 CAN 总线将真实传感器数据从 Gazebo 传输至嵌入式固件的硬件在环（HIL）桥接功能。
- **STM32F407xx 嵌入式固件**（运行 FreeRTOS）：生产级机器人实时控制器，通过 CAN 总线接收传感器状态，执行 Stanley 路径跟踪 + 侧倾稳定算法，并将电机电压和转向角指令回传至 CAN 总线。

两个层次分别使用 Python 和 C 实现了**完全相同的控制逻辑**，可直接互换和比较验证。

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                       Linux / ROS 2 主机                      │
│                                                               │
│  ┌──────────────┐   /odom, /imu,     ┌───────────────────┐  │
│  │   Ignition   │   /joint_states    │  autonomous_      │  │
│  │   Gazebo     │ ◄────────────────► │  control.py       │  │
│  │ (物理引擎+   │   /cmd_force_rl,   │  (ROS 2 主控节点) │  │
│  │  传感器)     │   /cmd_force_rr,   └────────┬──────────┘  │
│  └──────────────┘   /cmd_pos_fl/fr            │              │
│                                      *_to_firmware 话题      │
│                                               │              │
│                                     ┌─────────▼───────────┐  │
│                                     │  can_transceiver.py │  │
│                                     │  (ROS 2 ↔ CAN 桥接) │  │
│                                     └─────────┬───────────┘  │
└───────────────────────────────────────────────│──────────────┘
                                                │ SocketCAN (can0)
                                                │ 1 Mbps CAN 总线
                      ┌─────────────────────────▼─────────────────────────┐
                      │                 STM32F407xx 固件                    │
                      │                                                     │
                      │  ┌──────────────────┐   ┌───────────────────────┐  │
                      │  │CanCommunication  │   │  MotionControlTask    │  │
                      │  │     Task         │──►│  (Stanley + PID)      │  │
                      │  │ (CAN帧路由分发)  │   │  motion_control_      │  │
                      │  └──────────────────┘   │  algorithm()          │  │
                      │                         └──────────┬────────────┘  │
                      │  ┌──────────────────┐              │ TIM2 PWM      │
                      │  │ NavigationEkfTask│   ┌──────────▼────────────┐  │
                      │  │   (EKF 状态估计) │   │  电机 / 舵机 驱动器   │  │
                      │  └──────────────────┘   └───────────────────────┘  │
                      │  ┌──────────────────┐                               │
                      │  │  PowerHandleTask │                               │
                      │  └──────────────────┘                               │
                      │  ┌──────────────────┐                               │
                      │  │   MonitorTask    │ (栈水位监控)                  │
                      │  └──────────────────┘                               │
                      └───────────────────────────────────────────────────┘
```

**HIL 模式数据流：**

1. Gazebo 物理引擎以 50–100 Hz 发布 `/odom`、`/imu`、`/joint_states`。
2. `autonomous_control.py` 订阅这些话题，重发布至 `*_to_firmware` 话题。
3. `can_transceiver.py` 订阅 `*_to_firmware` 话题，将传感器数据打包（每两个浮点数一个 CAN 帧）并通过 `0x100`–`0x107`、`0x200`–`0x202` 发送，最后发送 `0x300` 同步帧。
4. CAN 帧经物理总线传至 STM32。
5. STM32 `CanCommunicationTask` 通过中断接收帧 → 入队 → 按 ID 范围分发至对应控制队列。
6. `MotionControlTask` 出队状态数据，运行 `motion_control_algorithm()`，并将四路控制输出（volt_l、volt_r、steer_l、steer_r）打包为单个 CAN 帧（ID `0x180`）回传。
7. `can_transceiver.py` 解码回传帧，发布至 `/cmd_volts_l`、`/cmd_volts_r`、`/cmd_steer_l`、`/cmd_steer_r`。
8. `autonomous_control.py` 接收这些话题并驱动 Gazebo 仿真关节。

---

## 目录结构

```
robot/
├── can_test.py                     # FreeRTOS CAN 通信模拟测试脚本
├── create_pipe_world.py            # 生成管道 world/mesh
├── Explain.md                      # 详细系统设计文档
├── README.md                       # 快速上手指南
├── LICENSE                         # MIT 许可证
├── hil.sh                          # 启动 HIL 通信与控制链路（使用 step_robot_system.launch.py）
├── sil.sh                          # 启动 SIL 控制链路（使用 robot_system.launch.py）
├── sim.sh                          # 构建并启动标准仿真（simulation.launch.py）
├── ssim.sh                         # 构建并启动锁步仿真（step_simulation.launch.py）
├── mil.sh                          # 单独运行 autonomous_control 节点
├── simulation/
│   ├── STM32/                      # HIL 压力测试与数据
│   │   ├── hil_stress_test.py
│   │   ├── plot_results_stm32.py
│   │   ├── hil_stress_results.csv
│   │   ├── hil_stress_kde.png
│   │   └── hill_stress_results_summary.txt
│   └── control/                    # SIL/仿真测试与数据绘制工具
│       ├── plot_unified.py
│       └── simulation_data*.png/csv
├── src/
│   └── simple_car_sim/             # ROS 2 包
│       ├── package.xml             # 包清单和依赖项
│       ├── CMakeLists.txt          # ROS 2 构建配置
│       ├── launch/
│       │   ├── simulation.launch.py         # 主仿真启动文件（标准模式）
│       │   ├── step_simulation.launch.py    # 锁步仿真启动文件
│       │   ├── robot_system.launch.py       # SIL/HIL 系统启动文件（标准模式）
│       │   └── step_robot_system.launch.py  # SIL/HIL 系统启动文件（锁步模式）
│       ├── urdf/
│       │   └── robot.urdf.xacro    # 机器人模型 (Xacro/URDF)
│       ├── worlds/
│       │   ├── pipe_world.sdf      # Ignition Gazebo 世界文件
│       │   └── pipe.obj            # 3D 管道轨道网格
│       ├── rviz/
│       │   ├── config.rviz         # RViz 2 配置文件
│       │   └── auto_display.rviz   # 自动显示配置文件
│       └── scripts/
│           ├── autonomous_control.py  # 主 ROS 2 控制节点 (标准 SIL)
│           ├── step_control.py        # ROS 2 锁步控制节点 (支持 Gazebo 步进控制)
│           ├── can_transceiver.py     # CAN <-> ROS 2 桥接节点 (HIL)
│           ├── battery_sim.py         # 虚拟电池仿真器
│           └── slam_interface.py      # SLAM 接口存根
└── firmware/
    ├── CMakeLists.txt              # 固件顶层 CMake
    ├── CMakePresets.json           # Debug/Release 构建预设
    ├── STM32F407XX_FLASH.ld        # 链接脚本
    ├── startup_stm32f407xx.s       # ARM Cortex-M4 启动汇编
    ├── Controller.ioc              # STM32CubeMX 项目文件
    ├── .clangd                     # Clangd 语言服务器配置
    ├── .settings/                  # 开发环境设置（IDE 配置）
    ├── COLCON_IGNORE               # 标记为 ROS 2 工作空间忽略
    ├── cmake/
    │   ├── gcc-arm-none-eabi.cmake # ARM 交叉编译工具链
    │   ├── starm-clang.cmake       # Clang 交叉编译配置
    │   └── stm32cubemx/            # STM32CubeMX 生成代码（HAL 配置）
    ├── Core/
    │   ├── Inc/                    # C 头文件
    │   │   ├── main.h              # 主程序头文件
    │   │   ├── control.h           # 运动控制算法声明
    │   │   ├── bsp_can.h           # CAN 总线驱动
    │   │   ├── ekf.h               # EKF 状态估计（存根）
    │   │   ├── power_management.h  # 电源管理（存根）
    │   │   └── FreeRTOSConfig.h    # FreeRTOS 配置文件
    │   └── Src/                    # C 源文件
    │       ├── main.c              # 主程序入口
    │       ├── control.c           # 运动控制算法（Stanley + PID）
    │       ├── bsp_can.c           # CAN 总线驱动实现
    │       ├── freertos.c          # FreeRTOS 任务定义
    │       ├── ekf.c               # EKF 状态估计（存根）
    │       └── power_management.c  # 电源管理（存根）
    ├── Drivers/
    │   ├── CMSIS/                  # ARM Cortex-M 软件接口
    │   └── STM32F4xx_HAL_Driver/   # STM32 HAL 库
    ├── Middlewares/
    │   ├── ST/STM32_USB_Device_Library/   # USB 设备库
    │   └── Third_Party/FreeRTOS/   # FreeRTOS v10.3.1
    ├── USB_DEVICE/
    │   ├── App/                    # USB 应用层
    │   └── Target/                 # USB 目标配置
```

---

## 技术栈

| 层次 | 技术 |
|---|---|
| 仿真引擎 | Ignition Gazebo（通过 `ros_gz_sim`）|
| 机器人中间件 | ROS 2（ament_cmake、rclpy）|
| 机器人描述 | Xacro / URDF |
| ROS-Gazebo 桥接 | `ros_gz_bridge` / `parameter_bridge` |
| 可视化 | RViz 2 |
| CAN 总线（Linux） | SocketCAN（`python-can`、`ros2_socketcan`）|
| 嵌入式 RTOS | FreeRTOS v10.3.1（via CMSIS-OS v2）|
| 嵌入式 MCU | STM32F407xx（ARM Cortex-M4F，168 MHz）|
| 嵌入式 HAL | STM32 HAL（STM32CubeMX 生成）|
| 固件构建 | CMake + Ninja + `gcc-arm-none-eabi` |
| 工作空间构建 | colcon |
| 许可证 | MIT |

**ROS 2 包依赖**（来自 `package.xml`）：

- `rclpy`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`
- `can_msgs`、`message_filters`、`cv_bridge`
- `ros_gz_sim`、`ros_gz_bridge`
- `robot_state_publisher`、`rviz2`、`xacro`

---

## 机器人物理模型

机器人模型定义于 `src/simple_car_sim/urdf/robot.urdf.xacro`：

| 组件 | 规格 |
|---|---|
| 车身尺寸 | 0.8 m × 0.6 m × 0.2 m |
| 车身质量 | 10 kg |
| 车轮半径 | 0.16 m |
| 车轮宽度 | 0.04 m |
| 车轮质量 | 1 kg（每个）|
| 轴距（L） | 0.6 m |
| 轮距（W） | 0.68 m |
| 前轴偏移 | 0.3 m（相对车身中心）|

**关节配置：**

| 关节 | 类型 | 范围 / 控制方式 |
|---|---|---|
| `suspension_fl/fr/rl/rr_joint` | 棱柱（垂直） | 行程 ±10 cm；弹簧 2000 N/m，阻尼 50 Nms |
| `steer_fl/fr_joint` | 旋转 | ±0.6 rad；PID 位置控制 Kp=15，Ki=0.01，Kd=0.05 |
| `wheel_fl/fr_joint` | 旋转（镜像转向关节）| 跟随转向关节 |
| `wheel_rl/rr_joint` | 连续旋转 | 力控制（来自 Gazebo 桥接）|

**传感器：**

| 传感器 | 频率 | 状态 |
|---|---|---|
| IMU（`/imu`）| 100 Hz | 启用 |
| 激光雷达（`/scan`）| 10 Hz | 硬件定义但未启用（有传感器定义，但未实际使用） |
| 摄像头（`/camera/image_raw`）| 30 Hz | 硬件定义但未启用（有传感器定义，但未实际使用） |

---

## 轨道几何

机器人在一条**封闭圆角矩形环形轨道**内（管道内部）行驶：

| 参数 | 数值 |
|---|---|
| 直道长度 | ~16 m（弯道中心位于各轴 ±8.0 m 处）|
| 弯道中心坐标 | (±8.0, ±8.0) m |
| 弯道半径 | 1.6 m |
| 管道内径 | 0.8 m |
| 路径外边界 | 弯道中心 + 半径 = 9.6 m |

**区域分类**（由 Stanley 控制器使用）：

| 区域 | 判断条件 | 路径航向 |
|---|---|---|
| 右上弯道 | x > 8.0 且 y > 8.0 | 圆弧，逆时针 |
| 左上弯道 | x < −8.0 且 y > 8.0 | 圆弧，逆时针 |
| 左下弯道 | x < −8.0 且 y < −8.0 | 圆弧，逆时针 |
| 右下弯道 | x > 8.0 且 y < −8.0 | 圆弧，逆时针 |
| 上直道 | y > 8.0（非弯道区）| π（朝左）|
| 下直道 | y < −8.0（非弯道区）| 0（朝右）|
| 左直道 | x < −8.0（非弯道区）| −π/2（朝下）|
| 右直道 | x > 8.0（非弯道区）| +π/2（朝上）|

机器人初始位置：`(5.0, −9.6, 0.0)`，朝向 x 正方向。

---

## 控制算法

`autonomous_control.py`（Python，SIL）与 `control.c`（C，STM32）实现了完全相同的算法，所有参数保持同步。

### Stanley 路径跟踪控制器

```
delta_geo = theta_e + atan2(k * e_geo, v + k_soft)
```

| 参数 | 数值 | 含义 |
|---|---|---|
| `k`（Stanley 增益）| 4.0 | 横向误差增益 |
| `k_soft`（软化系数）| 2.0 | 防止低速时除零 |

- `e_geo`：机器人前轴到路径中心线的横向距离（有符号）。
- `theta_e`：航向误差 = `normalize(psi_path - yaw)`。
- `delta_geo`：几何转向分量。

### 侧倾稳定控制

机器人在弯曲管壁内产生侧倾，附加转向修正以抵消侧倾：

```
delta_roll = -K_roll_p * roll - K_roll_d * roll_rate - K_roll_i * sum_roll
```

| 参数 | 数值 |
|---|---|
| `K_roll_p` | 4.0 |
| `K_roll_d` | 1.5 |
| `K_roll_i` | 0.5 |

综合转向角：`delta = delta_geo + delta_roll`，限幅至 ±45°。

### Ackermann 转向几何

将双轮模型转向角 `delta` 转换为左右前轮独立转向角：

```
delta_left  = atan( 2L * tan(delta) / (2L - W * tan(delta)) )
delta_right = atan( 2L * tan(delta) / (2L + W * tan(delta)) )
```

### 差速计算

```
omega    = v_target * tan(delta) / L
v_left   = v_target - omega * W/2
v_right  = v_target + omega * W/2
w_left   = v_left  / wheel_radius
w_right  = v_right / wheel_radius
```

### 电机电压 PID

```
voltage = Kp*e + Ki*sum(e)*dt + Kd*de/dt + Kf*target_w
```

| 参数 | 数值 |
|---|---|
| `Kp` | 5.0 |
| `Ki` | 1.0 |
| `Kd` | 0.05 |
| `Kf`（前馈）| 0.5 |
| 积分抗饱和限幅 | ±12.0 V |
| 电池电压限制 | ±12.0 V |

### 速度管理

```
expected_speed = desired_max_speed / (1 + 2.0 * |delta|)
expected_speed = max(expected_speed, 0.4)
```

机器人以 0.5 m/s² 加速、1.0 m/s² 减速，限幅至 [0, 1.2] m/s，额定最大速度 **1.0 m/s**。

### 转向平滑滤波

双级滤波，防止突变转向：

1. **低通滤波**：`filtered_delta = 0.3 * filtered_delta + 0.7 * raw_delta`
2. **斜率限制器**：每个时步最大变化量 = `100 rad/s * dt`（固件） / `10 rad/s * dt`（仿真）

### 直流电机动力学模型（仅 SIL）

`DCMotorSim` 对每个驱动电机的电气动力学进行建模：

| 参数 | 数值 |
|---|---|
| 电枢电阻 R | 0.5 Ω |
| 电枢电感 L | 0.01 H |
| 力矩常数 Kt | 0.5 Nm/A |
| 反电动势常数 Ke | 0.5 V·s/rad |
| 粘性摩擦 b | 0.1 Nms |
| 转子转动惯量 J | 0.02 kg·m² |
| 电流限制 | ±12.0 A |

每个时步使用解析解求解电流，避免大 dt 时的数值不稳定：

```
tau = L / R
i_steady = (V_in - V_backemf) / R
i[t+dt]  = i[t] * exp(-dt/tau) + i_steady * (1 - exp(-dt/tau))
torque    = Kt * i - b * omega
```

---

## CAN 通信协议

CAN 总线：`can0`，标准 11 位 ID，1 Mbps。所有浮点值均以 4 字节 IEEE 754 小端格式传输（另有说明除外）。为提高带宽效率，主机到 STM32 的传感器数据帧将两个浮点数值打包到一个 CAN 帧中（共 8 字节），而 STM32 到主机的控制输出帧将四个 uint16 值打包到一个 CAN 帧中。

### 主机 → STM32（传感器数据）

每个 CAN ID 携带两个 IEEE 754 浮点数（小端），顺序如下：

| CAN ID | 内容（第一个浮点数，第二个浮点数） | 格式 |
|---|---|---|
| `0x100` | 里程计位置 X，里程计位置 Y | float32, float32 |
| `0x101` | 里程计位置 Z，四元数 X | float32, float32 |
| `0x102` | 四元数 Y，四元数 Z | float32, float32 |
| `0x103` | 四元数 W，线速度 X | float32, float32 |
| `0x104` | 线速度 Y，线速度 Z | float32, float32 |
| `0x105` | 角速度 X，角速度 Y | float32, float32 |
| `0x106` | 角速度 Z，0.0（保留） | float32, float32 |
| `0x107` | 左后轮速（rad/s），右后轮速（rad/s） | float32, float32 |
| `0x200` | IMU 加速度 X（m/s²），IMU 加速度 Y | float32, float32 |
| `0x201` | IMU 加速度 Z，IMU 角速度 X | float32, float32 |
| `0x202` | IMU 角速度 Y，IMU 角速度 Z | float32, float32 |
| `0x300` | 时间步长 dt（s），0.0（保留）同步帧 | float32, float32 |
| `0x700` | Ping / 回环测试帧 | 原始字节 |

> 注：早期版本中使用的 ID `0x110`、`0x111`、`0x206`、`0x207` 已不再使用，车轮速度数据统一通过 `0x107` 发送。

> 注：`can_transceiver.py` 在每次收到 odom、joint_states 和 IMU 回调后，按上述顺序发布所有传感器帧，最后发送 `0x300` 同步帧以触发固件控制计算。

### STM32 → 主机（控制输出和遥测）

| CAN ID | 内容 | 格式 |
|---|---|---|
| `0x180` | 左电机电压、右电机电压、左转向角、右转向角（4×uint16 LE） | 4×uint16 LE |
| `0x400` | 运动控制执行时间（ms） | float32 |
| `0x500` | MotionControl 任务栈水位（words） | float32 |
| `0x501` | CAN 任务栈水位（words） | float32 |
| `0x502` | EKF 任务栈水位（words） | float32 |
| `0x503` | Power 任务栈水位（words） | float32 |
| `0x701` | Pong / 回环应答 | 原始字节（镜像 0x700） |
| `0x080`–`0x089` | 电源管理输出（存根，未启用） | float32 |
| `0x280`–`0x289` | EKF 状态估计输出（存根，未启用） | float32 |

> 注：早期版本中使用的独立 ID `0x181`、`0x182`、`0x183` 已合并到 `0x180` 帧中。

**控制输出编码（`0x180` 帧）：**

帧数据包含四个连续的 uint16 小端值，顺序为：
1. 左电机电压（mV + 0x8000 偏置）
2. 右电机电压（mV + 0x8000 偏置）
3. 左转向角（mrad + 0x8000 偏置）
4. 右转向角（mrad + 0x8000 偏置）

```
编码：raw = (value_in_mV_or_mrad) + 0x8000
解码：value = (raw - 0x8000) / 1000.0
```

此偏置编码将 ±32.767 V（或 ±32.767 rad）映射至无符号 16 位范围。

### CAN ID 路由（STM32 固件）

`CanCommunicationTask` 按 ID 范围路由传入帧：

| ID 范围 | 目标队列 |
|---|---|
| `0x000`–`0x0FF` | `powerManagementQueue` |
| `0x100`–`0x1FF` | `motionControl` |
| `0x200`–`0x2FF` | `sensorEkfData` |
| `0x700` | 立即回传为 `0x701` |

---

## ROS 2 仿真层

### 包信息：`simple_car_sim`

- **构建类型**：`ament_cmake`
- **版本**：0.0.0
- **维护者**：yinyudream
- **许可证**：MIT

### 启动文件

**`simulation.launch.py`**：标准仿真启动文件

启动完整仿真环境（实时运行）：

| 组件 | 包 | 说明 |
|---|---|---|
| Ignition Gazebo | `ros_gz_sim` | 加载 `pipe_world.sdf`（`-r` 立即运行）|
| 机器人生成器 | `ros_gz_sim/create` | 在 `(5.0, -9.6, 0.0)` 处生成 `simple_car` |
| robot_state_publisher | `robot_state_publisher` | 从 Xacro URDF 发布 TF，`use_sim_time=True` |
| 参数桥接 | `ros_gz_bridge` | 双向桥接时钟、关节、力、转向、IMU、里程计 |
| RViz 2 | `rviz2` | 加载 `rviz/config.rviz` |

**`step_simulation.launch.py`**：锁步仿真启动文件

启动锁步仿真环境（精确步进控制）：

| 组件 | 包 | 说明 |
|---|---|---|
| Ignition Gazebo | `ros_gz_sim` | 加载 `pipe_world.sdf`（不自动运行，等待控制）|
| 机器人生成器 | `ros_gz_sim/create` | 在 `(5.0, -9.6, 0.0)` 处生成 `simple_car` |
| robot_state_publisher | `robot_state_publisher` | 从 Xacro URDF 发布 TF，`use_sim_time=True` |
| 参数桥接 | `ros_gz_bridge` | 双向桥接时钟、关节、力、转向、IMU、里程计，额外桥接控制服务 |
| RViz 2 | `rviz2` | 加载 `rviz/config.rviz` |

**`robot_system.launch.py`**：标准 SIL/HIL 系统启动文件

启动完整的控制系统（标准模式）：
- `ros2_socketcan` 桥接（接口：`vcan0`）
- `can_transceiver.py` CAN 适配层
- `autonomous_control.py` 自动控制算法层
- RViz 2 可视化

**`step_robot_system.launch.py`**：锁步 SIL/HIL 系统启动文件

启动完整的控制系统（锁步模式）：
- `ros2_socketcan` 桥接（接口：`can0`）
- `can_transceiver.py` CAN 适配层
- `step_control.py` 锁步控制算法层

### 节点：`autonomous_control.py`（`CarController`）

主 SIL 控制节点（标准模式），运行频率 **100 Hz**（`sim_dt = 0.01 s`）。在标准仿真模式下使用，与 Gazebo 实时同步运行。

**订阅话题：**

| 话题 | 消息类型 | 用途 |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 真实位姿、方向、速度 |
| `/joint_states` | `sensor_msgs/JointState` | 车轮速度和悬架位置 |
| `/imu` | `sensor_msgs/Imu` | 侧倾率、偏航率、加速度 |
| `/cmd_volts_l` | `std_msgs/Float64` | 来自 STM32 的左电机电压（HIL）|
| `/cmd_volts_r` | `std_msgs/Float64` | 来自 STM32 的右电机电压（HIL）|
| `/cmd_steer_l` | `std_msgs/Float64` | 来自 STM32 的左转向角（HIL）|
| `/cmd_steer_r` | `std_msgs/Float64` | 来自 STM32 的右转向角（HIL）|

**发布话题：**

| 话题 | 消息类型 | 用途 |
|---|---|---|
| `/cmd_force_rl` | `std_msgs/Float64` | 左后轮力矩 → Gazebo |
| `/cmd_force_rr` | `std_msgs/Float64` | 右后轮力矩 → Gazebo |
| `/cmd_vel_rl` | `std_msgs/Float64` | 左后轮电压指令（用于调试） |
| `/cmd_vel_rr` | `std_msgs/Float64` | 右后轮电压指令（用于调试） |
| `/cmd_pos_fl` | `std_msgs/Float64` | 左前转向位置 → Gazebo |
| `/cmd_pos_fr` | `std_msgs/Float64` | 右前转向位置 → Gazebo |
| `/odom_to_firmware` | `nav_msgs/Odometry` | 里程计重发布 → `can_transceiver` |
| `/joint_states_to_firmware` | `sensor_msgs/JointState` | 关节状态重发布 → `can_transceiver` |
| `/imu_to_firmware` | `sensor_msgs/Imu` | IMU 重发布 → `can_transceiver` |

**数据日志：** 每个控制循环写入一行至 `simulation_data.csv`：

```
time, x, y, yaw, velocity, target_velocity, roll, roll_rate,
steering_delta, lateral_error, heading_error, wheel_speed_l, wheel_speed_r
```

**控制源标志：** `use_external_pwm = True`（默认）。为 `True` 时使用来自 STM32 的指令（HIL 模式）；设为 `False` 则使用内部计算结果（纯 SIL 模式，无需 STM32）。

### 节点：`step_control.py`（`CarController`）

锁步 SIL 控制节点，运行频率 **100 Hz**（`sim_dt = 0.01 s`）。在锁步仿真模式下使用，通过 Gazebo 控制服务精确控制仿真步进，确保传感器数据与控制指令严格同步。

**特性：**
- 通过 `/world/pipe_world/control` 服务调用 Gazebo 步进控制
- 每次控制循环前精确步进仿真 `steps_per_loop` 步（默认 10 步，每步 0.001 s）
- 等待传感器数据更新后再执行控制计算
- 支持外部 PWM 指令（HIL 模式）和内部计算（SIL 模式）

**订阅话题：** 与 `autonomous_control.py` 相同

**发布话题：** 与 `autonomous_control.py` 相同，额外发布 `/cmd_vel_rl` 和 `/cmd_vel_rr` 用于速度控制

**锁步机制：**
1. 调用 `ControlWorld` 服务步进仿真
2. 等待 `/clock` 话题更新到目标时间
3. 执行控制算法
4. 发布控制指令到 Gazebo
5. 重复循环

### 节点：`can_transceiver.py`（`CanTransceiver`）

通过 `ros2_socketcan` 桥接 ROS 2 话题与 SocketCAN。为提高带宽效率，传感器数据帧将两个浮点数值打包到一个 CAN 帧中。

**订阅 → CAN 发送：**

传感器数据在收到 odom、joint_states 和 IMU 三个回调后统一发送，打包顺序如下：

| ROS 话题 | CAN ID（每个 ID 携带两个浮点数） |
|---|---|
| `/odom_to_firmware`（位置、四元数、速度） | `0x100`–`0x107`（详见 CAN 通信协议表） |
| `/joint_states_to_firmware`（车轮速度） | `0x107`（与 odom 帧共享） |
| `/imu_to_firmware`（加速度、角速度） | `0x200`–`0x202` |
| 同步帧（触发固件控制计算） | `0x300`（dt, 0.0） |

**CAN 接收 → ROS 发布：**

| CAN ID | ROS 话题 | 解码方式 |
|---|---|---|
| `0x180` | `/cmd_volts_l`，`/cmd_volts_r`，`/cmd_steer_l`，`/cmd_steer_r` | 帧数据包含四个 uint16 小端值，依次解码为：`(uint16_LE - 0x8000) / 1000.0` |
| `0x400` | （未订阅）运动控制执行时间（ms） | float32 |
| `0x500`–`0x503` | （未订阅）任务栈水位 | float32 |

### 节点：`battery_sim.py`

仿真电池，通过 CAN ID `0x000`–`0x00F` 范围发布电压、电流、SOC 和温度。

### 节点：`slam_interface.py`

为未来 SLAM 算法集成预留的接口存根节点。

### 世界生成（`create_pipe_world.py`）

一次性运行以重新生成仿真世界：

```bash
python3 create_pipe_world.py
```

生成：
- `src/simple_car_sim/worlds/pipe.obj` — 圆角矩形管道轨道的 3D 网格。
- `src/simple_car_sim/worlds/pipe_world.sdf` — 嵌入网格的 Ignition Gazebo 世界文件。

---

## STM32 嵌入式固件

### 项目结构

固件目录遵循 STM32CubeMX 生成的典型项目结构，包含以下核心部分：

- **Core/**：用户应用程序代码，包括主循环、控制算法、外设驱动。
  - `Core/Src/control.c`：Stanley 路径跟踪 + 侧倾稳定控制算法（与 Python 仿真完全一致）。
  - `Core/Src/freertos.c`：FreeRTOS 任务定义与 CAN 通信任务。
  - `Core/Src/bsp_can.c`：CAN 总线驱动（滤波、中断、发送/接收）。
  - `Core/Inc/`：对应头文件。
- **Drivers/**：STM32 HAL 库与 CMSIS 设备文件。
- **Middlewares/**：FreeRTOS v10.3.1 和 USB 设备库。
- **USB_DEVICE/**：USB CDC 虚拟串口配置。
- **cmake/**：交叉编译工具链定义与 STM32CubeMX 生成代码的 CMake 集成。
- **其他配置文件**：`STM32F407XX_FLASH.ld`（链接脚本）、`startup_stm32f407xx.s`（启动汇编）、`Controller.ioc`（STM32CubeMX 项目文件）。

完整目录树见上文[目录结构](#目录结构)小节。

### 硬件规格

| 项目 | 规格 |
|---|---|
| MCU | STM32F407xx（Cortex-M4F）|
| 主频 | HSE 8 MHz → PLL → 168 MHz SYSCLK |
| Flash | 1 MB |
| RAM | 192 KB |
| CAN | CAN1 外设，1 Mbps，标准 11 位 ID |
| PWM | TIM2（CH1–CH4），PA0–PA3，1 kHz，1000 计数/周期 |
| 调试接口 | USB CDC（虚拟串口）|

### FreeRTOS 配置（`FreeRTOSConfig.h`）

| 配置项 | 数值 |
|---|---|
| FreeRTOS 版本 | v10.3.1 |
| 时钟节拍率 | 1000 Hz（1 ms）|
| 总堆大小 | 15,360 字节（Heap4 分配器）|
| 最大优先级 | 56 |
| 最小栈大小 | 128 words |
| 任务名最大长度 | 16 字符 |
| 抢占式调度 | 启用 |
| 互斥锁 | 启用 |
| 计数信号量 | 启用 |
| 追踪工具 | 启用 |
| 软件定时器 | 启用（优先级 2，队列深度 10）|
| NVIC 系统调用优先级边界 | 5 |

### FreeRTOS 任务

| 任务函数 | OS 名称 | 优先级 | 栈大小 | 功能 |
|---|---|---|---|---|
| `CanCommunicationTask` | `CanDataCenter` | AboveNormal | 1024 B | ISR → 队列 → 路由 CAN 帧 |
| `MotionControlTask` | `Contorl` | Normal | 1024 B | Stanley 控制 + PWM 输出 |
| `NavigationEkfTask` | `EkfAlgorithm` | Normal | 1024 B | EKF 传感器融合 |
| `PowerHandleTask` | `PowerManagement` | BelowNormal | 1024 B | 电源管理 |
| `MonitorTask` | `MonitorTask` | Low | 1024 B | 每 2 秒报告栈水位 |

### FreeRTOS 队列

| 队列 | 深度 | 元素大小 | 生产者 → 消费者 |
|---|---|---|---|
| `canRxQueue` | 16 | `CAN_RxPacketTypeDef` | CAN ISR → `CanCommunicationTask` |
| `motionControl` | 16 | `Robot_general` | `CanCommunicationTask` → `MotionControlTask` |
| `sensorEkfData` | 16 | `Robot_general` | `CanCommunicationTask` → `NavigationEkfTask` |
| `powerManagementQueue` | 16 | `Robot_general` | `CanCommunicationTask` → `PowerHandleTask` |

### CAN 驱动（`bsp_can.c`）

- **滤波器**：FIFO0 通过所有帧的掩码模式滤波器（滤波器组 0，32 位，掩码 = 0x0000）。
- **ISR 回调**（`HAL_CAN_RxFifo0MsgPendingCallback`）：从 FIFO0 读取帧并以超时 = 0（ISR 上下文非阻塞）方式入队至 `canRxQueue`。
- **`CAN_Send_Data(id, data, len)`**：构建标准 ID 数据帧并调用 `HAL_CAN_AddTxMessage`。

### 运动控制算法（`control.c`）

入口函数：`void motion_control_algorithm(float *status, uint16_t *PWM_Value)`

**输入数组 `status[20]`：**

| 索引 | 内容 |
|---|---|
| `[0]` | X 坐标（m）|
| `[1]` | Y 坐标（m）|
| `[3]` | 四元数 X |
| `[4]` | 四元数 Y |
| `[5]` | 四元数 Z |
| `[6]` | 四元数 W |
| `[7]` | 线速度 X（m/s）|
| `[8]` | 线速度 Y |
| `[9]` | 线速度 Z |
| `[10]` | 角速度 X = 侧倾率（rad/s）|
| `[16]` | 实际左轮速（rad/s）|
| `[17]` | 实际右轮速（rad/s）|

**输出数组 `PWM_Value[4]`**（uint16，偏置编码）：

| 索引 | 内容 |
|---|---|
| `[0]` | 左电机电压（mV + 0x8000）|
| `[1]` | 右电机电压（mV + 0x8000）|
| `[2]` | 左转向角（mrad + 0x8000）|
| `[3]` | 右转向角（mrad + 0x8000）|

每次调用结束时直接写入 TIM2 PWM 寄存器：

```c
TIM_CH1 = (PWM_Value[0] * 1000) / 4095   // 左电机
TIM_CH2 = (PWM_Value[1] * 1000) / 4095   // 右电机
TIM_CH3 = (PWM_Value[2] * 1000) / 4095   // 左转向
TIM_CH4 = (PWM_Value[3] * 1000) / 4095   // 右转向
```

### EKF 和电源管理（存根）

`ekf.c` 和 `power_management.c` 目前为**存根实现**，是为未来扩展预留的架构占位符：

```c
void ekf_algorithm_update(float *sensor_data, float *result);    // EKF 状态估计
void power_management_logic(float *power_status, float *instruction);  // 电源管理
```

---

## 构建与运行

### 前提条件

**ROS 2 侧：**
- Ubuntu 22.04 + ROS 2 Humble（或兼容版本）
- Ignition Gazebo（Fortress 或兼容版本）
- `ros_gz_sim`、`ros_gz_bridge`、`robot_state_publisher`、`rviz2`、`xacro`
- `python3-can`（`pip install python-can`）
- `ros2_socketcan` 包
- SocketCAN 配置：`can0` 接口（虚拟测试可用 `vcan0`）

**固件侧：**
- `gcc-arm-none-eabi` 交叉编译器
- `cmake` >= 3.22
- `ninja-build`

### ROS 2 仿真（SIL）

**标准仿真模式（实时运行）：**

```bash
cd /home/yinyudream/robot

# 完整构建并启动（终止已有进程、构建、加载、启动）：
bash sim.sh

# 或手动执行：
pkill -f "ign gazebo"; pkill -f "ros2"; pkill -f "rviz"; pkill -f "parameter_bridge"
colcon build --packages-select simple_car_sim
source install/setup.bash
ros2 launch simple_car_sim simulation.launch.py
```

**锁步仿真模式（精确控制仿真步进）：**

```bash
cd /home/yinyudream/robot

# 构建并启动锁步仿真：
bash ssim.sh

# 或手动执行：
pkill -f "ign gazebo"; pkill -f "ros2"; pkill -f "rviz"; pkill -f "parameter_bridge"
colcon build --packages-select simple_car_sim
source install/setup.bash
ros2 launch simple_car_sim step_simulation.launch.py
```

> `sim.sh` 和 `ssim.sh` 仅启动仿真环境（Gazebo + 模型 + RViz），不单独包含控制闭环。

> **提示：** 若仅修改 Python / Launch / URDF 文件，可使用 `--symlink-install` 加速构建：
> ```bash
> colcon build --symlink-install --packages-select simple_car_sim
> ```

### 单独运行控制节点

```bash
source install/setup.bash

# 标准控制节点：
ros2 run simple_car_sim autonomous_control.py --ros-args -p use_sim_time:=true

# 锁步控制节点：
ros2 run simple_car_sim step_control.py --ros-args -p use_sim_time:=true

# 或直接使用脚本：
bash sil.sh       # 标准 SIL（使用 robot_system.launch.py）
bash mil.sh       # 单独运行 autonomous_control 节点
```

> SIL 实际运行需要两步：先 `bash sim.sh`（或 `bash ssim.sh`），再 `bash sil.sh`（或运行对应控制节点）。

### HIL 联调运行（仿真 + CAN + 控制）

**HIL 模式（锁步控制，使用物理 CAN 接口）：**

```bash
# 第一步：启动锁步仿真器
bash ssim.sh

# 第二步：启动 HIL 通信与控制链路（使用物理 can0 接口）
bash hil.sh
```

**标准 SIL 模式（虚拟 CAN 接口）：**

```bash
# 第一步：启动标准仿真器
bash sim.sh

# 第二步：启动 SIL 控制链路（使用虚拟 vcan0 接口）
bash sil.sh
```

> HIL 模式使用锁步仿真和控制，通过物理 `can0` 接口与 STM32 通信。`hil.sh` 启动 `step_robot_system.launch.py`（`ros2_socketcan` + `can_transceiver.py` + `step_control.py`）。

> SIL 模式使用标准仿真和虚拟 `vcan0` 接口。`sil.sh` 启动 `robot_system.launch.py`（`ros2_socketcan` + `can_transceiver.py` + `autonomous_control.py` + RViz）。

### 纯 SIL 模式（无需 STM32）

编辑控制节点文件（`autonomous_control.py` 或 `step_control.py`），将：

```python
use_external_pwm = True
```

改为：

```python
use_external_pwm = False
```

重新构建后即可在无 STM32 硬件的环境下运行完整仿真。

**注意：** 两个控制节点都支持此标志切换。标准模式使用 `autonomous_control.py`，锁步模式使用 `step_control.py`。

### 可视化 URDF 模型

```bash
ros2 launch urdf_tutorial display.launch.py \
    model:=/home/yinyudream/robot/src/simple_car_sim/urdf/robot.urdf.xacro
```

### 重新生成仿真世界

```bash
python3 create_pipe_world.py
```

### 固件构建

```bash
cd firmware

# 配置（Debug）
cmake --preset Debug

# 构建
cmake --build --preset Debug

# 输出：firmware/build/Debug/Controller.elf（以及 .bin / .map）
```

Release 构建将 `Debug` 替换为 `Release` 即可。

### CAN 总线配置

```bash
# 以 1 Mbps 启用物理 CAN 接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 监控 CAN 流量
candump can0

# 虚拟 CAN 测试（无硬件）
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### HIL 压力测试

需要通过 `can0` 连接物理 STM32：

```bash
cd simulation/STM32
python3 hil_stress_test.py

# 绘制测试结果
python3 plot_results_stm32.py
```

### 刷写固件

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program firmware/build/Debug/Controller.elf verify reset exit"
```

---

## 配置参考

### 控制参数（Python 与 C 完全同步）

| 参数 | Python | C 宏 | 数值 |
|---|---|---|---|
| 时步 | `sim_dt = 0.01` | `DT 0.01f` | 0.01 s（100 Hz）|
| 轴距 | `L=0.6` | `L 0.6f` | 0.6 m |
| 轮距 | `W=0.68` | `W 0.68f` | 0.68 m |
| 车轮半径 | `wheel_radius=0.16` | `WHEEL_RADIUS 0.16f` | 0.16 m |
| 前轴偏移 | `car_length=0.3` | `CAR_LENGTH 0.3f` | 0.3 m |
| 弯道中心 | `corner_center=8.0` | `CORNER_CENTER 8.0f` | 8.0 m |
| 弯道半径 | `radius=1.6` | `RADIUS 1.6f` | 1.6 m |
| Stanley 增益 k | `k=4.0` | `K_STANLEY 4.0f` | 4.0 |
| 软化系数 | `k_soft=2.0` | `K_SOFT 2.0f` | 2.0 |
| 最大速度 | `desired_max_speed=1.0` | `DESIRED_MAX_SPEED 1.0f` | 1.0 m/s |
| 弯道最小速度 | `max(expected, 0.4)` | 同上 | 0.4 m/s |
| 弯道速度系数 | `2.0` | `SAFE_CORNER_SPEED_FACTOR 2.0f` | 2.0 |
| PID Kp | `Kp=5.0` | `KP 5.0f` | 5.0 |
| PID Ki | `Ki=1.0` | `KI 1.0f` | 1.0 |
| PID Kd | `Kd=0.05` | `KD 0.05f` | 0.05 |
| PID Kf（前馈）| `Kf=0.5` | `KF 0.5f` | 0.5 |
| 侧倾 P 增益 | `K_roll_p=4.0` | `K_ROLL_P 4.0f` | 4.0 |
| 侧倾 D 增益 | `K_roll_d=1.5` | `K_ROLL_D 1.5f` | 1.5 |
| 侧倾 I 增益 | `0.5` | `K_ROLL_I 0.5f` | 0.5 |
| 最大转向角 | ±45° | ±45° | ±0.785 rad |
| 转向滤波 alpha | `0.7` | `FILTER_ALPHA 0.7f` | 0.7 |
| 转向斜率限制 | `delta_slew_rate=10.0` | `DELTA_SLEW_RATE 100.0f` | 100 rad/s（固件）/ 10 rad/s（仿真） |
| 电池电压 | `12.0 V` | `BATTERY_VOLTAGE 12.0f` | ±12 V |

### HIL 测试配置（`hil_stress_test.py`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `CAN_INTERFACE` | `'can0'` | SocketCAN 接口名 |
| `TEST_DURATION` | `15` | 测试时长（秒）|
| 发送间隔 | `0.02 s` | 50 Hz 突发发送 |

### 日志文件路径

在 `autonomous_control.py` 中硬编码：

```
/home/yinyudream/robot/simulation/control/simulation_data.csv
```

---

## API 接口

### 固件 C API

**`bsp_can.h`**

```c
// 配置 CAN1 接收滤波器（通过所有帧）
void CAN_Filter_Config(void);

// 发送标准 CAN 帧
void CAN_Send_Data(uint16_t id, uint8_t *data, uint8_t len);

// 关键数据类型
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_RxPacketTypeDef;

typedef struct {
    union { float f; uint8_t bytes[4]; } FloatBytes;
    uint32_t id;
} Robot_general;

// 对外暴露的队列句柄
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t powerManagementQueueHandle;
extern osMessageQueueId_t sensorEkfDataHandle;
```

**`control.h`**

```c
// 运动控制主入口
// status[20]：机器人状态输入
// PWM_Value[4]：控制输出
void motion_control_algorithm(float *status, uint16_t *PWM_Value);
```

**`ekf.h`**

```c
// EKF 状态估计更新（存根）
// sensor_data[10]：IMU + 编码器输入
// result[10]：EKF 状态输出
void ekf_algorithm_update(float *sensor_data, float *result);
```

**`power_management.h`**

```c
// 电源管理逻辑（存根）
// power_status[10]：电池/电源传感器输入
// instruction[10]：电源控制输出
void power_management_logic(float *power_status, float *instruction);
```

### Python API

**`VehicleDynamics`（`autonomous_control.py` / `step_control.py`）**

两个控制节点共享相同的车辆动力学算法类。

```python
@staticmethod
def geometric_calculation(x, y, yaw, vel, corner_center, radius) -> (e_geo, theta_e)
# 返回用于 Stanley 控制的横向误差（m）和航向误差（rad）

@staticmethod
def calculate_wheel_speed(stanley_delta, vel_target, L=0.6, W=0.68) -> (w_left, w_right)
# 返回左右后轮目标角速度（rad/s）

@staticmethod
def calculate_ackermann_angles(delta, L=0.6, W=0.68) -> (delta_left, delta_right)
# 返回左右前轮独立转向角（rad）

@staticmethod
def stanley_control(x, y, yaw, vel, vel_target, corner_center, radius, roll, sum_roll, roll_rate=0.0)
    -> (w_left, w_right, delta, e_geo, theta_e)
# 完整 Stanley + 侧倾稳定控制，返回轮速目标、转向角及误差分量
```

**`DCMotorSim`（`autonomous_control.py` / `step_control.py`）**

```python
def __init__(self, R=0.5, L=0.01, Kt=1.0, Ke=1.0, b=0.1, J=0.02, dt=0.05)

def step(self, voltage_in: float, angular_vel: float) -> torque: float
# 仿真一个时步的电机动力学，返回输出力矩（Nm）
```

**`CanTransceiver.send_can_float`（`can_transceiver.py`）**

```python
def send_can_float(self, can_id: int, value: float) -> None
# 将 value 打包为 IEEE 754 小端浮点数并发布至 /to_can_bus
```

---

## 测试与验证

### HIL 压力测试结果（实际硬件，2026 年 3 月）

测试条件：15 秒运行，`can0` 上 50 Hz 突发，运动控制 + EKF 同时加载。

| 指标 | 结果 |
|---|---|
| 发送 Ping 总数 | 797 |
| 收到 Pong 总数 | 790 |
| **丢包率** | **0.88%** |
| 运动控制平均执行时间 | **3.48 ms** |
| 运动控制最大执行时间 | **4.00 ms** |

**RTOS 任务栈水位（剩余 words，数值越大越安全）：**

| 任务 | 水位 |
|---|---|
| MotionControl | 128 words |
| CAN DataCenter | 188 words |
| EKF Algorithm | 114 words |
| Power Management | 184 words |

1024 字节（256 words）栈利用率良好。EKF 任务剩余 114 words 最为紧张——若后续扩展 EKF 逻辑需重点关注。

### SIL 仿真数据

以 100 Hz 记录至 `simulation_data.csv`，可使用 `plot_unified.py` 生成以下图表：

- `simulation_data_unified_trajectory.png` — 轨迹与路径跟踪
- `simulation_data_unified_velocity.png` — 速度与目标速度
- `simulation_data_unified_wheel_speeds.png` — 左右轮速
- `simulation_data_unified_steering.png` — 转向角与误差
- `simulation_data_unified_steering_angles.png` — 左右前轮转向角
- `simulation_data_unified_roll_heading_lateral.png` — 侧倾、航向与横向误差
- `hil_stress_kde.png` — HIL 测试运动控制执行时间/延迟分布曲线

绘图命令：

```bash
cd simulation/control

# SIL 仿真结果
python3 plot_unified.py

# HIL 压力测试结果
python3 plot_results_stm32.py
```

### CAN 通信模拟测试

`can_test.py` 脚本提供 FreeRTOS CAN 通信模拟测试功能，使用 `python-can` 库与 `vcan0` 接口通信，模拟多任务 CAN 数据处理：

```bash
# 运行 CAN 测试脚本（需要配置 vcan0 接口）
python3 can_test.py
```

**功能特性：**
- 模拟 FreeRTOS 中的多任务状态数组：`motion_status`、`power_status`、`sensor_data`
- 实现完整的运动控制任务 `motion_control_task()`，包含 Stanley 轨迹跟踪 + 差速速度分配 + 电机电压 PID
- CAN 接收与分发机制，支持 ID 范围路由
- 数据记录功能，可将系统状态记录至 CSV 文件

**使用场景：**
- 在没有 STM32 硬件的情况下测试 CAN 通信协议
- 验证运动控制算法与 CAN 数据流的兼容性
- 调试 CAN 帧格式和路由逻辑

---

## 常用命令

```bash
# ---- ROS 2 ----

# 列出当前话题
ros2 topic list

# 查询某话题的消息类型
ros2 topic type /scan

# 构建包
colcon build --packages-select simple_car_sim

# 符号链接构建（脚本/URDF 修改时更快）
colcon build --symlink-install --packages-select simple_car_sim

# 加载工作空间
source install/setup.bash

# 启动完整仿真（标准模式）
ros2 launch simple_car_sim simulation.launch.py

# 启动锁步仿真（精确步进控制）
ros2 launch simple_car_sim step_simulation.launch.py

# 仅运行控制节点（标准模式）
ros2 run simple_car_sim autonomous_control.py --ros-args -p use_sim_time:=true

# 仅运行控制节点（锁步模式）
ros2 run simple_car_sim step_control.py --ros-args -p use_sim_time:=true

# 可视化 URDF
ros2 launch urdf_tutorial display.launch.py \
    model:=/home/yinyudream/robot/src/simple_car_sim/urdf/robot.urdf.xacro

# 启动 RViz（仓库内配置）
rviz2 -d src/simple_car_sim/rviz/config.rviz
ros2 run rviz2 rviz2 -d src/simple_car_sim/rviz/config.rviz

# 终止所有仿真进程
pkill -f "ign gazebo"; pkill -f "ros2"; pkill -f "rviz"; pkill -f "parameter_bridge"

# ---- CAN 总线 ----

# 1 Mbps 启动 can0
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0

# 虚拟 CAN（无硬件测试）
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

# 监控 CAN 流量
candump can0

# ---- 固件 ----

# 配置 Debug 构建
cmake --preset Debug

# 编译固件
cmake --build --preset Debug

# 刷写固件（OpenOCD + ST-Link）
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program firmware/build/Debug/Controller.elf verify reset exit"

# ---- HIL 测试 ----

cd simulation/STM32
python3 hil_stress_test.py      # 运行压力测试
python3 plot_results_stm32.py   # 绘制测试结果
```

---

## 许可证

本项目采用 [MIT 许可证](LICENSE)。
