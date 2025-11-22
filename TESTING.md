# 四轮小车项目测试验证指南

## 概述

本文档提供了四轮小车仿真项目的完整测试验证步骤，确保所有功能正常运行。测试包括构建验证、仿真系统测试、控制功能测试和集成测试。

## 测试环境要求

- **操作系统**: Ubuntu 22.04 LTS (推荐)
- **ROS2发行版**: Humble Hawksbill
- **Gazebo版本**: 11或更高
- **Python版本**: 3.8+

## 测试步骤

### 1. 构建项目验证

#### 步骤1.1: 检查项目结构
```bash
# 进入项目目录
cd ~/ros2_ws/src/four_wheel_bot

# 验证项目文件结构
ls -la
```
**预期输出**: 应显示以下目录和文件:
- `package.xml`, `CMakeLists.txt`, `setup.py`
- `launch/`, `scripts/`, `urdf/`, `config/`, `worlds/` 目录

#### 步骤1.2: 构建项目
```bash
# 返回工作空间根目录
cd ~/ros2_ws

# 清理之前的构建
rm -rf build install log

# 构建项目
colcon build
```
**预期输出**: 
```
Starting >>> four_wheel_bot
Finished <<< four_wheel_bot [X.XX seconds]

Summary: 1 package finished [X.XX seconds]
```

**验证标准**: 构建过程无错误，所有包成功构建

#### 步骤1.3: 加载环境
```bash
# 加载工作空间环境
source install/setup.bash

# 验证包是否可用
ros2 pkg list | grep four_wheel_bot
```
**预期输出**: `four_wheel_bot`

### 2. 仿真系统启动测试

#### 步骤2.1: 启动完整仿真系统
```bash
# 在终端1中启动完整系统
ros2 launch four_wheel_bot four_wheel_bot.launch.py
```

**预期行为和输出**:

1. **Gazebo启动**:
   - Gazebo窗口应打开
   - 显示地面网格和测试方块
   - 四轮小车模型应出现在(0,0,0.1)位置

2. **RViz2启动**:
   - RViz2窗口应打开
   - 显示机器人模型、TF坐标系和网格
   - 机器人模型应与Gazebo中的模型一致

3. **控制台输出**:
   - 各节点启动成功消息
   - 无错误或警告信息

#### 步骤2.2: 验证系统组件
```bash
# 在另一个终端中检查系统状态
source ~/ros2_ws/install/setup.bash

# 检查节点状态
ros2 node list
```
**预期输出**:
```
/gazebo
/joint_state_publisher
/robot_state_publisher
/rviz2
/spawn_entity
```

**验证话题**:
```bash
ros2 topic list
```
**预期包含的话题**:
```
/four_wheel_bot/cmd_vel
/four_wheel_bot/odom
/joint_states
/robot_description
/tf
/tf_static
```

### 3. 键盘控制功能测试

#### 步骤3.1: 启动键盘控制节点
```bash
# 在终端2中启动键盘控制
ros2 run four_wheel_bot teleop_keyboard
```

**预期输出**:
```
四轮小车键盘控制节点已启动

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

#### 步骤3.2: 测试控制功能

**测试用例1: 前进控制**
- **操作**: 按下 `W` 键
- **预期结果**:
  - 控制台显示: `前进`
  - Gazebo中小车向前移动
  - RViz2中机器人位置更新

**测试用例2: 后退控制**
- **操作**: 按下 `S` 键
- **预期结果**:
  - 控制台显示: `后退`
  - Gazebo中小车向后移动

**测试用例3: 左转控制**
- **操作**: 按下 `A` 键
- **预期结果**:
  - 控制台显示: `左转`
  - Gazebo中小车向左旋转

**测试用例4: 右转控制**
- **操作**: 按下 `D` 键
- **预期结果**:
  - 控制台显示: `右转`
  - Gazebo中小车向右旋转

**测试用例5: 停止控制**
- **操作**: 按下 `空格` 键
- **预期结果**:
  - 控制台显示: `停止`
  - 小车停止运动

**测试用例6: 退出程序**
- **操作**: 按下 `Q` 键
- **预期结果**: 控制节点正常退出

#### 步骤3.3: 验证消息发布
```bash
# 在终端3中监控控制消息
source ~/ros2_ws/install/setup.bash
ros2 topic echo /four_wheel_bot/cmd_vel
```

**预期行为**: 当按下控制键时，应看到相应的Twist消息:
- 前进: `linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}`
- 左转: `linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}`

### 4. Gazebo和RViz2集成测试

#### 步骤4.1: TF变换验证
```bash
# 检查TF变换树
ros2 run tf2_tools view_frames.py

# 查看生成的frames.pdf
evince frames.pdf
```

**预期结果**: TF树应包含以下坐标系:
- `odom` → `chassis`
- `chassis` → 各个轮子关节

#### 步骤4.2: 里程计数据验证
```bash
# 监控里程计数据
ros2 topic echo /four_wheel_bot/odom
```

**预期行为**: 当小车移动时，里程计数据应实时更新位置和方向信息。

#### 步骤4.3: 关节状态验证
```bash
# 监控关节状态
ros2 topic echo /joint_states
```

**预期行为**: 应显示四个轮子关节的状态信息。

### 5. 自动化测试

#### 步骤5.1: 运行控制功能测试
```bash
# 运行自动化测试脚本
python3 test_teleop.py
```

**预期输出**:
```
测试键盘控制节点...
请在另一个终端运行: ros2 run four_wheel_bot teleop_keyboard
然后使用WASD键控制小车，观察这里的输出
✓ 控制节点正常工作，成功接收Twist消息
  消息 1: 线速度=0.5, 角速度=0.0
```

## 故障排除和验证标准

### 常见问题解决方案

#### 问题1: Gazebo无法启动
**症状**: Gazebo窗口不出现或立即崩溃
**解决方案**:
```bash
# 检查Gazebo安装
gzserver --version

# 安装缺失的Gazebo模型
sudo apt install gazebo11

# 重置Gazebo配置
rm -rf ~/.gazebo
```

#### 问题2: RViz2无法显示机器人
**症状**: RViz2中显示"No transform from [chassis] to [odom]"
**解决方案**:
- 检查`robot_state_publisher`节点是否运行
- 验证URDF文件是否正确
- 在RViz2中重新设置Fixed Frame为`odom`

#### 问题3: 键盘控制无响应
**症状**: 按下按键无反应
**解决方案**:
```bash
# 检查节点连接
ros2 node info /teleop_keyboard

# 检查话题发布
ros2 topic info /four_wheel_bot/cmd_vel

# 手动发布测试消息
ros2 topic pub /four_wheel_bot/cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

#### 问题4: 构建错误
**症状**: `colcon build`失败
**解决方案**:
```bash
# 安装缺失的依赖
rosdep install -i --from-path src --rosdistro humble -y

# 清理并重新构建
rm -rf build install log
colcon build
```

### 验证标准总结

| 测试项目 | 成功标准 | 验证方法 |
|---------|----------|----------|
| 构建验证 | 无错误完成构建 | `colcon build`成功 |
| Gazebo启动 | 窗口正常打开，模型加载 | 视觉检查 |
| RViz2启动 | 显示机器人模型和TF | 视觉检查 |
| 键盘控制 | 按键响应，消息发布 | 控制台输出和话题监控 |
| 运动控制 | 小车按指令运动 | Gazebo和RViz2观察 |
| TF变换 | 完整的坐标变换树 | `view_frames.py` |
| 集成测试 | 所有组件协同工作 | 端到端功能测试 |

## 性能基准

### 系统资源使用
- **CPU使用率**: < 80% (在标准硬件上)
- **内存使用**: < 2GB
- **启动时间**: < 30秒

### 控制响应时间
- **键盘输入到Gazebo响应**: < 100ms
- **控制消息发布频率**: 10Hz

## 测试报告模板

完成测试后，请记录以下信息:

```
测试日期: [日期]
测试环境: [ROS2版本, Gazebo版本]
测试结果: [通过/失败]

构建验证: [✓/✗]
仿真系统启动: [✓/✗]
键盘控制功能: [✓/✗]
  - 前进: [✓/✗]
  - 后退: [✓/✗] 
  - 左转: [✓/✗]
  - 右转: [✓/✗]
  - 停止: [✓/✗]
集成测试: [✓/✗]

遇到的问题: [详细描述]
解决方案: [采取的解决措施]
```

通过遵循本测试指南，您可以确保四轮小车项目的所有功能正常运行，并及时发现和解决潜在问题。