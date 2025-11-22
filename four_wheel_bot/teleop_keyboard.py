#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # 创建发布者，发布到 /four_wheel_bot/cmd_vel 话题
        self.publisher_ = self.create_publisher(Twist, '/four_wheel_bot/cmd_vel', 10)
        
        # 设置速度参数
        self.linear_speed = 0.5  # 线速度 0.5 m/s
        self.angular_speed = 1.0  # 角速度 1.0 rad/s
        
        # 保存原始终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('四轮小车键盘控制节点已启动')
        self.print_instructions()
        
    def print_instructions(self):
        """打印控制指令说明"""
        instructions = """
        四轮小车键盘控制指令:
        ----------------------------
        W: 前进
        S: 后退  
        A: 左转
        D: 右转
        空格: 停止
        Q: 退出程序
        ----------------------------
        """
        print(instructions)
    
    def get_key(self):
        """非阻塞方式获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """主循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
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
                    else:
                        # 无效按键
                        continue
                    
                    # 发布速度命令
                    self.publisher_.publish(twist)
                
                # 短暂休眠以减少CPU使用
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # 发布停止命令
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info('控制节点已关闭')

def main(args=None):
    rclpy.init(args=args)
    
    teleop_node = TeleopKeyboard()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()