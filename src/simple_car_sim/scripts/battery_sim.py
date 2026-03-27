#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import struct
import random
import time

class BatterySim(Node):
    """
    虚拟电池模拟器
    输出: CAN 0x000 - 0x00A (状态)
    输入: CAN 0x010 - 0x01A (指令)
    """
    def __init__(self):
        super().__init__('battery_sim')
        
        # --- CAN 通信 ---
        self.can_pub = self.create_publisher(Frame, '/to_can_bus', 10)
        self.can_sub = self.create_subscription(Frame, '/from_can_bus', self.can_callback, 100)
        
        # --- 电池参数 (24V 锂电池) ---
        self.nominal_voltage = 24.0
        self.capacity_ah = 10.0
        self.current_ah = 10.0 # 满电
        self.max_voltage = 25.2
        self.min_voltage = 19.0
        
        # --- 内部状态 ---
        self.output_enabled = False
        self.voltage = self.max_voltage
        self.current = 0.0
        self.temperature = 25.0
        self.soc = 100.0 # %
        self.status_flags = 0x00 # 0: Normal, 1: Error
        
        # --- 仿真循环 (10Hz) ---
        self.dt = 0.1
        self.create_timer(self.dt, self.simulation_loop)
        
        self.get_logger().info('Battery Simulation Node Started (24V System)')

    def can_callback(self, msg):
        """处理控制指令 0x010 - 0x01A"""
        can_id = msg.id
        
        if 0x010 <= can_id <= 0x01A:
            # 仅记录接收到的指令，不做逻辑处理
            try:
                # 尝试解析第一个 float 数据用于打印
                val = 0.0
                if len(msg.data) >= 4:
                    val = struct.unpack('<f', bytes(msg.data[:4]))[0]
                self.get_logger().info(f"Battery CMD Recv: ID={hex(can_id)} Val={val:.2f}")
            except:
                self.get_logger().info(f"Battery CMD Recv: ID={hex(can_id)} Len={len(msg.data)}")

    def simulation_loop(self):
        """
        周期发送固定/测试状态数据 (10Hz)
        """
        # 发送固定的测试数据
        self.send_can_float(0x000, 24.0)   # Voltage
        self.send_can_float(0x001, 1.5)    # Current
        self.send_can_float(0x002, 95.5)   # SOC
        self.send_can_float(0x003, 35.0)   # Temp
        self.send_can_float(0x004, 0.0)    # Status
        self.send_can_float(0x005, 10.0)   # Capacity

    def send_can_float(self, can_id, value):
        """发送单精度浮点数 CAN 帧"""
        try:
            can_data = struct.pack('<f', float(value))
            frame = Frame()
            frame.header.stamp = self.get_clock().now().to_msg()
            frame.id = can_id
            frame.dlc = 4
            frame.data = list(can_data)
            self.can_pub.publish(frame)
        except Exception as e:
            self.get_logger().error(f'CAN Send Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BatterySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
