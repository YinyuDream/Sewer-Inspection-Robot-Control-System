import re

with open('README.md', 'r', encoding='utf-8') as f:
    content = f.read()

replacement = """## 项目亮点

- 四轮 Ackermann 转向模型，支持封闭管道路径自主巡航。
- Python（ROS 2）与 C（STM32）控制逻辑参数对齐，便于 SIL/HIL 对照验证。
- 支持 ROS 2 ↔ SocketCAN ↔ STM32 的双向链路。
- **双模式仿真**：标准实时仿真 + 锁步精确仿真（支持 Gazebo 步进控制）。
- **完整测试工具**：包含 HIL 压力测试、CAN 通信模拟测试（`can_test.py`）和数据可视化工具。

## 关键测试数据与图表

### 1. 速度闭环控制
展示了机器人在巡检过程中纵向速度的闭环控制效果及目标速度的跟踪表现：
![速度控制](simulation/control/simulation_data_unified_velocity.png)

### 2. 位姿与姿态跟踪
展示了机器人在管道运动中的侧倾角(Roll)、航向角(Heading)及横向位置误差的实时控制状态：
![位姿与姿态跟踪](simulation/control/simulation_data_unified_roll_heading_lateral.png)

### 3. HIL 压力测试评估
展示了硬件在环(HIL)模式下固件控制算法的整体响应时间和执行延迟压力：
![HIL 压力分布](simulation/STM32/hil_stress_kde.png)

## 环境依赖"""

content = re.sub(r'## 项目亮点[\s\S]*?## 环境依赖', replacement, content)

with open('README.md', 'w', encoding='utf-8') as f:
    f.write(content)
