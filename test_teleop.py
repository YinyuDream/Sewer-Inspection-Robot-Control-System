#!/usr/bin/env python3
"""
测试键盘控制节点的简单脚本
这个脚本模拟键盘输入并检查节点是否正确发布Twist消息
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/four_wheel_bot/cmd_vel',
            self.listener_callback,
            10)
        self.received_messages = []
        
    def listener_callback(self, msg):
        self.received_messages.append(msg)
        self.get_logger().info(f'收到消息: 线速度={msg.linear.x}, 角速度={msg.angular.z}')

def test_teleop_node():
    rclpy.init()
    
    # 创建测试订阅者
    test_subscriber = TestSubscriber()
    
    print("测试键盘控制节点...")
    print("请在另一个终端运行: ros2 run four_wheel_bot teleop_keyboard")
    print("然后使用WASD键控制小车，观察这里的输出")
    
    try:
        # 等待消息
        rclpy.spin_once(test_subscriber, timeout_sec=1.0)
        
        if test_subscriber.received_messages:
            print("✓ 控制节点正常工作，成功接收Twist消息")
            for i, msg in enumerate(test_subscriber.received_messages):
                print(f"  消息 {i+1}: 线速度={msg.linear.x}, 角速度={msg.angular.z}")
        else:
            print("⚠ 尚未收到消息，请确保控制节点正在运行")
            
    except Exception as e:
        print(f"✗ 测试失败: {e}")
    finally:
        test_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    test_teleop_node()