#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_rx_callback,
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
        if can_id == 0x100:
            self.decode_speed_msg(data)

    def cmd_rx_callback(self, msg):
        """
        处理接收到的 ROS 指令 (ROS -> CAN)
        """
        # 示例: 将 Twist 消息中的线速度转换为 CAN 帧
        # 假设 CAN ID 0x200用于发送控制指令
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 打包数据: 两个 float (4 bytes each) -> 8 bytes total
        # '<ff' 表示小端序，两个浮点数
        try:
            can_data = struct.pack('<ff', linear_x, angular_z)
            
            # 构造 CAN 帧
            frame = Frame()
            frame.header.stamp = self.get_clock().now().to_msg()
            frame.id = 0x200
            frame.dlc = 8
            # 将 bytes 转换为 list[int] 以符合消息定义
            frame.data = list(can_data)  
            
            # 发布到 CAN 总线
            self.can_pub.publish(frame)
            # self.get_logger().info(f'Sent CAN ID: 0x200, Vel: {linear_x:.2f}, Ang: {angular_z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Encoding error: {e}')

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
                
                output_msg = String()
                output_msg.data = f"Speed detected: {value:.2f}"
                self.ros_pub.publish(output_msg)
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
