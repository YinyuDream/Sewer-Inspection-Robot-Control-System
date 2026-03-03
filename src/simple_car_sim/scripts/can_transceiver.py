#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
import struct

class CanTransceiver(Node):
    def __init__(self):
        super().__init__('can_transceiver')
        
        # --- 接收部分 ---
        # 订阅 ros2_socketcan 发布的话题 (默认 /from_can_bus)
        self.can_sub = self.create_subscription(
            Frame,
            'from_can_bus',
            self.can_rx_callback,
            10)
        
        # 创建一个发布者，发布转化后的 ROS 话题
        self.ros_pub = self.create_publisher(String, 'interpreted_can_data', 10)

        # --- 发送部分 ---
        # 订阅 ROS 控制指令 (例如 /cmd_vel)
        self.create_subscription(
            Odometry,
            '/odom_to_firmware',
            self.cmd_odom_callback,
            10)
        self.create_subscription(
            JointState,
            '/joint_states_to_firmware',
            self.cmd_joint_state_callback,
            10)
        self.create_subscription(
            Imu,
            '/imu_to_firmware',
            self.cmd_imu_callback,
            10)

        # 创建一个发布者，将 CAN 帧发送给 ros2_socketcan (默认 /to_can_bus)
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        
        self.get_logger().info('CAN Transceiver Node has been started.')

    def can_rx_callback(self, msg):
        """
        处理接收到的 CAN 帧 (CAN -> ROS)
        """
        can_id = msg.id
        data_len = msg.dlc
        data = bytes(msg.data[:data_len])
        
        # 示例: 假设 ID 0x100 是速度反馈
        if can_id >= 0x280 and can_id <= 0x28A:
            self.decode_speed_msg(data)

    def send_can_float(self, can_id, value):
        """Helper to send a single float value as a CAN frame"""
        try:
            can_data = struct.pack('<f', float(value))
            frame = Frame()
            frame.header.stamp = self.get_clock().now().to_msg()
            frame.id = can_id
            frame.dlc = 4
            frame.data = list(can_data)
            self.can_pub.publish(frame)
        except Exception as e:
            self.get_logger().error(f'CAN send error ID {hex(can_id)}: {e}')

    def cmd_odom_callback(self, msg):
        """
        Odom -> CAN (0x100 - 0x109)
        Pos(3): x, y, z
        Quat(4): x, y, z, w
        Vel(3): linear_x, linear_y, angular_z
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v_lin = msg.twist.twist.linear
        v_ang = msg.twist.twist.angular
        a_lin = msg.twist.twist.linear

        # Position 0x100-0x102
        self.send_can_float(0x100, p.x)
        self.send_can_float(0x101, p.y)
        self.send_can_float(0x102, p.z)
        
        # Orientation 0x103-0x106
        self.send_can_float(0x103, q.x)
        self.send_can_float(0x104, q.y)
        self.send_can_float(0x105, q.z)
        self.send_can_float(0x106, q.w)
        
        # Velocity 0x107-0x109 (Lin X, Lin Y, Lin Z)
        self.send_can_float(0x107, v_lin.x)
        self.send_can_float(0x108, v_lin.y)
        self.send_can_float(0x109, v_lin.z)

        # Angular velocity 0x10A-0x10C (Ang X, Ang Y, Ang Z)
        self.send_can_float(0x10A, v_ang.x)
        self.send_can_float(0x10B, v_ang.y)
        self.send_can_float(0x10C, v_ang.z)

        # Acceleration 0x10D-0x10F (Lin Acc X, Lin Acc Y, Lin Acc Z)
        self.send_can_float(0x10D, a_lin.x)
        self.send_can_float(0x10E, a_lin.y)
        self.send_can_float(0x10F, a_lin.z)

    def cmd_joint_state_callback(self, msg):
        """
        Encoders -> CAN (0x206 - 0x209)
        """
        # 假设 JointState 中包含四个驱动轮的速度
        if len(msg.velocity) >= 4:
            self.send_can_float(0x206, msg.velocity[0])
            self.send_can_float(0x207, msg.velocity[1])
            self.send_can_float(0x208, msg.velocity[2])
            self.send_can_float(0x209, msg.velocity[3])

    def cmd_imu_callback(self, msg):
        """
        IMU -> CAN (0x200 - 0x205)
        Accel(3): x, y, z
        Gyro(3): x, y, z
        """
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        
        # Accel 0x200-0x202
        self.send_can_float(0x200, acc.x)
        self.send_can_float(0x201, acc.y)
        self.send_can_float(0x202, acc.z)
        
        # Gyro 0x203-0x205
        self.send_can_float(0x203, gyro.x)
        self.send_can_float(0x204, gyro.y)
        self.send_can_float(0x205, gyro.z)

    def decode_speed_msg(self, data):
        """
        示例解析逻辑
        """
        try:
            # 假设数据格式: 4字节浮点数 (小端)
            if len(data) >= 4:
                value = struct.unpack('<f', data[0:4])[0]
                
                # 如果要接收 uint16_t 小端数字 (2字节), 使用 '<H'
                # value_uint16 = struct.unpack('<H', data[0:2])[0]
                
                return value
            
        except Exception as e:
            self.get_logger().error(f'Decoding error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CanTransceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
