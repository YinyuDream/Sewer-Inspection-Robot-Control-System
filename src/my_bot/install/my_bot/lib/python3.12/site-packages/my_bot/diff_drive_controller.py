#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # 机器人参数
        self.wheel_radius = 0.06  # 轮子半径 (m)
        self.wheel_separation = 0.26  # 轮子间距 (m)
        self.wheel_base = 0.3  # 轴距 (m)
        
        # 创建订阅者，订阅cmd_vel话题
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 创建发布者，发布轮子速度命令
        self.front_left_pub = self.create_publisher(Float64, '/front_left_wheel_velocity_controller/command', 10)
        self.front_right_pub = self.create_publisher(Float64, '/front_right_wheel_velocity_controller/command', 10)
        self.rear_left_pub = self.create_publisher(Float64, '/rear_left_wheel_velocity_controller/command', 10)
        self.rear_right_pub = self.create_publisher(Float64, '/rear_right_wheel_velocity_controller/command', 10)
        
        self.get_logger().info('差速控制器节点已启动')
        
    def cmd_vel_callback(self, msg):
        """处理cmd_vel消息，计算轮子速度"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 差速驱动运动学模型
        # v_left = (2 * linear_x - angular_z * self.wheel_separation) / (2 * self.wheel_radius)
        # v_right = (2 * linear_x + angular_z * self.wheel_separation) / (2 * self.wheel_radius)
        
        # 简化模型：四轮差速驱动
        # 对于四轮驱动，前后同侧轮子速度相同
        v_left = (linear_x - angular_z * self.wheel_separation / 2) / self.wheel_radius
        v_right = (linear_x + angular_z * self.wheel_separation / 2) / self.wheel_radius
        
        # 创建轮子速度消息
        front_left_msg = Float64()
        front_left_msg.data = v_left
        
        front_right_msg = Float64()
        front_right_msg.data = v_right
        
        rear_left_msg = Float64()
        rear_left_msg.data = v_left
        
        rear_right_msg = Float64()
        rear_right_msg.data = v_right
        
        # 发布轮子速度命令
        self.front_left_pub.publish(front_left_msg)
        self.front_right_pub.publish(front_right_msg)
        self.rear_left_pub.publish(rear_left_msg)
        self.rear_right_pub.publish(rear_right_msg)
        
        # 调试信息
        self.get_logger().debug(f'线速度: {linear_x:.2f}, 角速度: {angular_z:.2f}')
        self.get_logger().debug(f'左轮速度: {v_left:.2f}, 右轮速度: {v_right:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    diff_drive_controller = DiffDriveController()
    
    try:
        rclpy.spin(diff_drive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        diff_drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()