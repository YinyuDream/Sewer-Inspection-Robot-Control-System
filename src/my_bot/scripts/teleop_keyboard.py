#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
键盘控制:
  w/s : 前进 / 后退
  a/d : 左转 / 右转
  空格 : 停止
  q   : 退出

线速度增量: 0.1 m/s  角速度增量: 0.2 rad/s
当前发布到: /diff_drive_controller/cmd_vel
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.get_logger().info('启动键盘控制. 按 q 退出.')

    def run(self):
        print(MSG)
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key(settings)
                if key == 'w':
                    self.linear += 0.1
                elif key == 's':
                    self.linear -= 0.1
                elif key == 'a':
                    self.angular += 0.2
                elif key == 'd':
                    self.angular -= 0.2
                elif key == ' ':
                    self.linear = 0.0
                    self.angular = 0.0
                elif key == 'q':
                    break

                twist = Twist()
                twist.linear.x = self.linear
                twist.angular.z = self.angular
                self.pub.publish(twist)
                print(f"\r线速度: {self.linear:.2f} m/s  角速度: {self.angular:.2f} rad/s    ", end='')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

if __name__ == '__main__':
    import select
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
