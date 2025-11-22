#!/bin/bash

echo "=== ROS2 四轮小车系统测试 ==="

# 加载ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1. 检查ROS2环境..."
ros2 pkg list | grep my_bot

echo "2. 检查键盘控制节点..."
ros2 run my_bot keyboard_control.py --help

echo "3. 检查差速控制器节点..."
ros2 run my_bot diff_drive_controller.py --help

echo "4. 检查URDF模型..."
ros2 run xacro xacro urdf/my_bot.urdf.xacro > /dev/null && echo "URDF模型验证通过"

echo "5. 检查启动文件..."
python3 launch/my_bot_ignition.launch.py --help

echo "=== 测试完成 ==="
echo "要启动完整系统，请运行:"
echo "ros2 launch my_bot my_bot_ignition.launch.py"
echo ""
echo "使用说明:"
echo "- 启动后，在键盘控制终端中使用 WASD 控制小车"
echo "- W: 前进, S: 后退, A: 左转, D: 右转, 空格: 停止, Q: 退出"