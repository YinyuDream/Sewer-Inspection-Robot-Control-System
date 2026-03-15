#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
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
            '/from_can_bus',
            self.can_rx_callback,
            10)
        
        # 创建一个发布者，发布转化后的 ROS 话题
        # Publishes PWM values back to autonomous_control
        # Independent topics for each control command as requested
        self.pub_cmd_volts_l = self.create_publisher(Float64, '/cmd_volts_l', 10)
        self.pub_cmd_volts_r = self.create_publisher(Float64, '/cmd_volts_r', 10)
        self.pub_cmd_steer_l = self.create_publisher(Float64, '/cmd_steer_l', 10)
        self.pub_cmd_steer_r = self.create_publisher(Float64, '/cmd_steer_r', 10)

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
        self.can_pub = self.create_publisher(Frame, '/to_can_bus', 10)
        
        self.get_logger().info('CAN Transceiver Node has been started.')
        
    def can_rx_callback(self, msg):
        print(f"Received CAN frame ID: {hex(msg.id)}, Data: {msg.data.hex()}")
        can_id = msg.id
        data = msg.data
        
        # Mapping based on control.c output assumption:
        # ID 0x180: Left Voltage
        # ID 0x181: Right Voltage
        # ID 0x182: Left Steer
        # ID 0x183: Right Steer
        
        if can_id == 0x180: # Left Motor Voltage (mV + offset)
            raw_val = struct.unpack('<H', bytes(data[:2]))[0] 
            # Reverse: Raw = Val*1000 + 0x8000 -> Val = (Raw - 0x8000) / 1000.0
            val = (raw_val - 0x8000) / 1000.0
            self.pub_cmd_volts_l.publish(Float64(data=val))
            
        elif can_id == 0x181: # Right Motor Voltage
            raw_val = struct.unpack('<H', bytes(data[:2]))[0] 
            val = (raw_val - 0x8000) / 1000.0
            self.pub_cmd_volts_r.publish(Float64(data=val))

        elif can_id == 0x182: # Left Steer (mrad + offset) -> rad
            raw_val = struct.unpack('<H', bytes(data[:2]))[0] 
            val = (raw_val - 0x8000) / 1000.0
            self.pub_cmd_steer_l.publish(Float64(data=val))

        elif can_id == 0x183: # Right Steer (mrad + offset) -> rad
            raw_val = struct.unpack('<H', bytes(data[:2]))[0] 
            val = (raw_val - 0x8000) / 1000.0
            self.pub_cmd_steer_r.publish(Float64(data=val))


    def send_can_float(self, can_id, value):
        """Helper to send a single float value as a CAN frame"""
        # print(f"Sending CAN frame ID: {hex(can_id)}, Value: {value}")
        try:
            can_data = struct.pack('<f', float(value))
            frame = Frame()
            # frame.header.stamp = self.get_clock().now().to_msg() # can_msgs/Frame typically has its own header structure or is simplified
            frame.id = can_id
            frame.dlc = 4
            # Ensure data is exactly 8 bytes long as per uint8[8] definition
            full_data = [0] * 8
            for i, b in enumerate(can_data):
                full_data[i] = b
            frame.data = full_data
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
        # 根据 joint name 匹配对应的 CAN ID，发送速度值
        for i, name in enumerate(msg.name):
            if name == 'wheel_rl_joint' and len(msg.velocity) > i:
                self.send_can_float(0x110, msg.velocity[i])
                self.send_can_float(0x206, msg.velocity[i])
            elif name == 'wheel_rr_joint' and len(msg.velocity) > i:
                self.wheel_rr_speed = msg.velocity[i]
                self.send_can_float(0x111, msg.velocity[i])
                self.send_can_float(0x207, msg.velocity[i])

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
