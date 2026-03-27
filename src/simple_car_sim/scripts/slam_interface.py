#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge

# 引入消息过滤器进行时间对其
import message_filters

import struct
import math
import numpy as np

class SlamAlgorithmInterface(Node):
    """
    SLAM 算法接口节点
    订阅: 
      - /camera/image_raw (视觉数据)
      - /scan (激光雷达数据)
      - /from_can_bus (CAN上的 EKF 融合数据)
    发布:
      - /odom_slam (SLAM 计算出的里程计)
    """
    def __init__(self):
        super().__init__('slam_algorithm_interface')
        
        # 1. 使用 message_filters 创建同步订阅
        # 这里的 Subscriber 是 message_filters.Subscriber, 不是 self.create_subscription
        self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, '/scan')

        # 创建近似时间同步器 (ApproximateTimeSynchronizer)
        # 队列长度=10, 允许的最大时间误差=0.1秒
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.camera_sub, self.lidar_sub], 
            queue_size=10, 
            slop=0.1
        )
        # 注册同步回调函数
        self.ts.registerCallback(self.sensors_callback)
            
        # 2. 订阅 CAN 数据 (获取 EKF 变量, 保持独立因为频率不同)
        self.can_sub = self.create_subscription(
            Frame,
            '/from_can_bus',
            self.can_callback,
            100) # 队列稍微大一点，防止丢失高频CAN帧
            
        # 3. 发布 SLAM 结果
        self.odom_pub = self.create_publisher(Odometry, '/odom_slam', 10)
        
        # EKF 融合变量存储 (0x280 - 0x28A)
        # 假设这些 ID 对应以下状态向量 (仅为示例，需根据实际通信协议定义)
        # 0x280: Pos X
        # 0x281: Pos Y
        # 0x282: Pos Z
        # 0x283: Quat X
        # 0x284: Quat Y
        # 0x285: Quat Z
        # 0x286: Quat W
        # 0x287: Vel X
        # 0x288: Vel Y
        # 0x289: Vel Z
        # 0x28A: Status / Confidence / Covariance Scale
        self.ekf_state = {
            'pos_x': 0.0, 'pos_y': 0.0, 'pos_z': 0.0,
            'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0, 'quat_w': 1.0,
            'vel_x': 0.0, 'vel_y': 0.0, 'vel_z': 0.0,
            'confidence': 1.0
        }
        
        self.cv_bridge = CvBridge()
        
        # 定时器已移除，改由 sensors_callback 驱动
        # self.create_timer(0.05, self.run_slam_loop)
        
        self.get_logger().info('SLAM Interface Node Started')

    def sensors_callback(self, image_msg, scan_msg):
        """
        同步传感器回调: 同时处理对齐后的 视觉 + 雷达 数据
        """
        # ====================================================
        # SLAM 核心逻辑触发点
        # ====================================================
        try:
            current_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            current_scan = scan_msg
            
            # 由于 CAN 数据是异步的，这里使用最近的 EKF 状态
            # TODO: 将 current_image, current_scan, self.ekf_state 输入到 SLAM 算法
            
            # --- 模拟输出 ---
            optim_x = self.ekf_state['pos_x']
            optim_y = self.ekf_state['pos_y']
            
            # 构建 Odometry 消息
            odom_msg = Odometry()
            odom_msg.header.stamp = image_msg.header.stamp # 使用图像时间戳对齐
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            odom_msg.pose.pose.position.x = self.ekf_state['pos_x']
            odom_msg.pose.pose.position.y = self.ekf_state['pos_y']
            odom_msg.pose.pose.position.z = self.ekf_state['pos_z']
            
            odom_msg.pose.pose.orientation.x = self.ekf_state['quat_x']
            odom_msg.pose.pose.orientation.y = self.ekf_state['quat_y']
            odom_msg.pose.pose.orientation.z = self.ekf_state['quat_z']
            odom_msg.pose.pose.orientation.w = self.ekf_state['quat_w']
            
            odom_msg.twist.twist.linear.x = self.ekf_state['vel_x']
            odom_msg.twist.twist.linear.y = self.ekf_state['vel_y']
            odom_msg.twist.twist.linear.z = self.ekf_state['vel_z']
            
            self.odom_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f'SLAM Loop Error: {e}')

    def can_callback(self, msg):
        """解析 EKF CAN 帧"""
        can_id = msg.id
        
        # 过滤只处理 0x280 - 0x28A 范围的 ID
        if 0x280 <= can_id <= 0x28A:
            try:
                # 假设数据是 float (4 bytes)
                value = struct.unpack('<f', bytes(msg.data[:4]))[0]
                
                if can_id == 0x280: self.ekf_state['pos_x'] = value
                elif can_id == 0x281: self.ekf_state['pos_y'] = value
                elif can_id == 0x282: self.ekf_state['pos_z'] = value
                
                elif can_id == 0x283: self.ekf_state['quat_x'] = value
                elif can_id == 0x284: self.ekf_state['quat_y'] = value
                elif can_id == 0x285: self.ekf_state['quat_z'] = value
                elif can_id == 0x286: self.ekf_state['quat_w'] = value
                
                elif can_id == 0x287: self.ekf_state['vel_x'] = value
                elif can_id == 0x288: self.ekf_state['vel_y'] = value
                elif can_id == 0x289: self.ekf_state['vel_z'] = value
                
                elif can_id == 0x28A: self.ekf_state['confidence'] = value
                
            except Exception as e:
                self.get_logger().warn(f'CAN Decode Error ID {hex(can_id)}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SlamAlgorithmInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
