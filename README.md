# 基于嵌入式平台的下水道巡检机器人控制系统 (Sewer Inspection Robot Control System)

本项目基于 **STM32F4** 和 **ROS 2** 设计并实现了一个下水道巡检机器人控制系统原型。项目采用了“上位机感知规划 + 下位机实时执行”的分层异构架构，并在 ROS 2 环境中搭建了完整的管道环境建模与硬件在环 (HIL) 仿真验证平台。

---

## 📖 项目简介 (Project Overview)

针对下水道网络封闭、潮湿且具有复杂弯道的环境特征，本系统设计了一个计算紧凑、高实时性、强抗干扰的嵌入式控制核心。通过软件定义系统行为，实现了多传感器融合、动态电源调度与复杂的非完整约束运动控制。

### ✨ 核心特性 
*   **硬实时任务调度**: 采用 STM32 + FreeRTOS，实现关键控制循环 (100Hz) 和多通道数据流的安全无阻塞切换。
*   **现代自控算法应用**: 将 **Stanley 轨迹跟踪算法** 与 **离散 PID** 结合，在资源受限的 MCU 上实现带低通滤波与斜率限制 (Slew Rate Limit) 的平滑姿态控制。
*   **阿克曼运动学闭环**: 原生支持差速转向计算 (Ackermann Steering)，独立计算左右轮转速与舵机偏角，输出协同 PWM。
*   **数字孪生与管道仿真**: 基于 ROS 2 Gazebo 实现自定义下水道管道的地形生成 (`create_pipe_world.py`)，并进行自动化数据采集验证。
*   **车规级总线通信**: 自定义基于 CAN 帧的应用层交互协议，有效规避电磁干扰，实现上下位系统无缝对接。

---

## 🏗️ 架构设计 (Architecture)

系统由上层（决策感知）和下层（嵌入式实时控制）构成：

```mermaid
graph TD;
    subgraph ROS 2 (Host PC / Companion Computer)
        SLAM[slam_interface.py - SLAM & 感知转译]
        Plan[autonomous_control.py - 自主导航与高级指令]
        BattSim[battery_sim.py - 电源模拟器]
        Bridge[can_transceiver.py - 虚拟CAN总线桥接]
        
        SLAM --> Plan
        Plan --> Bridge
        BattSim --> Bridge
    end

    CAN_BUS((CAN Bus))

    subgraph STM32F407 (Embedded Firmware)
        RTOS((FreeRTOS 调度器))
        TaskCAN[CanCommunicationTask]
        TaskControl[MotionControlTask]
        TaskEKF[NavigationEkfTask]
        TaskPower[PowerHandleTask]
        
        Bridge <--> CAN_BUS
        CAN_BUS <--> TaskCAN
        
        TaskCAN -->|MsgQueue| TaskControl
        TaskCAN -->|MsgQueue| TaskEKF
        TaskCAN -->|MsgQueue| TaskPower
        
        TaskControl --> PWM[TIM2 PWM 输出: 左右驱动+转向]
    end
```

---

## 📂 项目模块详述 (Module Details)

### 1. 嵌入式固件模块 (`firmware/`)
使用 C 语言编写，核心运行在 STM32F407 芯片上。
*   **FreeRTOS 任务管理器 (`freertos.c`)**:
    *   `CanCommunicationTask`: 收发枢纽。通过邮箱等待 CAN 接收中断数据，依据 CAN ID 段将数据快速分发至相应的消息队列（`canRxQueue` -> `motionControlHandle` 等），保障数据的线程安全。
    *   `MotionControlTask`: 读取位姿反馈，调用 `control.c` 进行运动学逆解和动力学控制并输出四路 PWM。
    *   `NavigationEkfTask`: 为 IMU 与里程计传感器预留的 EKF 融合任务位 (`ekf.c`)。
    *   `PowerHandleTask`: 监控系统多路电源电压，实现过压欠压保护削载逻辑 (`power_management.c`)。
*   **运动算法库 (`control.c`)**:
    *   通过特定的管道转角逻辑 (`geometric_calculation`) 获取横向误差 $e$ 与路径倾角 $\theta_e$。
    *   核心 Stanley 方程：$$\delta_{raw} = \theta_e + \arctan \left( \frac{K_{stanley} \cdot e}{v + K_{soft}} \right)$$
    *   引入带有阻尼的 `FILTER_ALPHA` 和 `DELTA_SLEW_RATE` 防止因算法突变引起的舵机高频抖动。

### 2. ROS 2 仿真模块 (`src/simple_car_sim/`)
构建于 Ubuntu + ROS 2 平台，包含物理引擎端配置和功能验证脚本。
*   **场景生成**：`create_pipe_world.py` 用于生成具有特定曲率中心 (`CORNER_CENTER`) 及转弯半径的巡检管道世界地图。
*   **协同脚本 (`scripts/`)**:
    *   `can_transceiver.py`: 核心 HIL 桥接器。处理 `python-can`/`ros2_socketcan` 数据。将 ROS 话题 `/odom_to_firmware` 转打包成 CAN 帧发给 MCU；解析 MCU 返回的驱动电压、转向角度 CAN 帧映射到模型话题 (`/cmd_volts_l`, `/cmd_steer_r`等)。
    *   `plot_results.py` / `plot_results_filtered.py`: 大数据可视化套件，对由仿真过程中导出的 `simulation_data*.csv` 轨迹集进行对齐、滤波与误差图表化绘制。

---

## 🔌 CAN 应用层通信协议 (CAN Protocol Stack)

为保证通信稳定，我们在标准 CAN2.0B 的基础上映射了如下自定义协议规则：

| ID 范围 (Hex) | 任务类别 | 说明 | 数据编码 (Encoding logic) |
| :--- | :--- | :--- | :--- |
| `0x000 - 0x0FF` | Pwr & System | 电源管理及保护指令交互 | 4 Byte Float直接强转 |
| `0x100 - 0x1FF` | Rx Controls | 上层给底层的位姿、速度真值 | 4 Byte Float直接强转 |
| `0x200 - 0x2FF` | Rx Sensors | 传感器(IMU/Odom)的融合反馈 | 4 Byte Float直接强转 |
| `0x300` | Tx Motor L | 左侧主轴电机控制电压 | mV转换: `val*1000 + 0x8000` |
| `0x301` | Tx Motor R | 右侧主轴电机控制电压 | mV转换: `val*1000 + 0x8000` |
| `0x302` | Tx Steer L | 左舵机前轮Ackermann偏角指令 | mrad转换: `val*1000 + 0x8000` |
| `0x303` | Tx Steer R | 右舵机前轮Ackermann偏角指令 | mrad转换: `val*1000 + 0x8000` |

---

## 🚀 部署与运行 (Getting Started)

### 依赖配置
建议运行平台：Ubuntu 20.04/22.04 + ROS 2
```bash
sudo apt update
sudo apt install ros-dev-tools ros-humble-ros2-socketcan
pip3 install python-can cantools struct
```

### 1. 编译 ROS 2 工作空间
编译 `simple_car_sim` 子包：
```bash
source /opt/ros/humble/setup.bash
cd ~/robot
colcon build --symlink-install
source install/setup.bash
```

### 2. 构建下水道仿真世界
如果需要重新生成具有特定弯道分布的管道地形：
```bash
python3 create_pipe_world.py
```

### 3. 环境启动
提供了一键启动及批处理测试脚本 `run.sh` / `test.sh`。或者选择手动分步运行：
```bash
# 1. 挂载虚拟 CAN 接口用于单机软软仿真 (HIL调试下请替换为真实的硬件 CAN,如 can0)
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 2. 启动 Gazebo 机器人模型及核心下水道场景
ros2 launch simple_car_sim simulation.launch.py

# 3. 开启 CAN 桥接跨平台通信，打通 Python 仿真算法与底层 C 模型算法回路
ros2 run simple_car_sim can_transceiver.py
```

### 4. 交叉编译固件 (用于部署实体打板)
利用 CMake 工具链生成固件：
```bash
cd firmware
mkdir build && cd build
cmake -G "Ninja" ..     # 推荐安装 arm-none-eabi-gcc 与 Ninja-build
ninja
# 生成后的 .elf 或 .hex/bin 文件可通过 openocd 或 ST-Link 烧录至 STM32F4 芯片。
```

---

## 📈 数据指标验证 (Data & Results)
项目中包含自动化运行过程导出的批量数据 (如 `simulation_data_final_*.csv`) 。
使用 Python 图表套件进行轨迹误差绘制，用于评估 PID 超调量和 Stanley 控制的横向偏移稳态误差：
```bash
python3 src/simple_car_sim/scripts/plot_results_filtered.py
```
*(注：这将在根目录下生成 `simulation_data_filtered.png` 等渲染图片，用于直观评判控制策略在弯道与交叉口的跟随鲁棒性。)*

---

## 📜 许可证 (License)
本项目开源并采用 **[MIT License](LICENSE)** 进行分发，协议适用于本项目中所有的软硬件逻辑以及底层代码模块。
