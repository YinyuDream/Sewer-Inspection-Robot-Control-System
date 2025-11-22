#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # 创建发布者，发布控制命令
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 控制参数
        self.linear_speed = 0.5  # 线速度 m/s
        self.angular_speed = 1.0  # 角速度 rad/s
        
        self.get_logger().info('键盘控制节点已启动')
        self.get_logger().info('使用 WASD 键控制小车运动:')
        self.get_logger().info('W: 前进')
        self.get_logger().info('S: 后退') 
        self.get_logger().info('A: 左转')
        self.get_logger().info('D: 右转')
        self.get_logger().info('空格: 停止')
        self.get_logger().info('Q: 退出')
        
        # 保存原始终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """运行键盘控制循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # 创建速度消息
                twist = Twist()
                
                if key == 'w' or key == 'W':
                    # 前进
                    twist.linear.x = self.linear_speed
                    self.get_logger().info('前进')
                elif key == 's' or key == 'S':
                    # 后退
                    twist.linear.x = -self.linear_speed
                    self.get_logger().info('后退')
                elif key == 'a' or key == 'A':
                    # 左转
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('左转')
                elif key == 'd' or key == 'D':
                    # 右转
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('右转')
                elif key == ' ':
                    # 停止
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info('停止')
                elif key == 'q' or key == 'Q':
                    # 退出
                    self.get_logger().info('退出程序')
                    break
                
                # 发布控制命令
                self.cmd_publisher.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # 发布停止命令
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_publisher.publish(twist)
            
            self.get_logger().info('键盘控制节点已关闭')

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_control = KeyboardControl()
    
    try:
        keyboard_control.run()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()