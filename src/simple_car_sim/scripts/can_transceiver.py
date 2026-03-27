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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import threading
import time

def micro_sleep(us):
    """高精度忙等，防止硬件 FIFO 溢出"""
    target = time.perf_counter_ns() + us * 1000
    while time.perf_counter_ns() < target:
        pass

class CanTransceiver(Node):
    def __init__(self):
        super().__init__('can_transceiver')
        
        # 回调组：发送端所有回调互斥，防止 publish_sim_data() 被并发重入
        self._tx_cb_group = MutuallyExclusiveCallbackGroup()
        # 接收端独立回调组，不阻塞发送
        self._rx_cb_group = MutuallyExclusiveCallbackGroup()
        # 发送锁：额外保护 publish_sim_data 中的帧序列原子性
        self._tx_lock = threading.Lock()

        # --- 接收部分 ---
        # 订阅 ros2_socketcan 发布的话题 (默认 /from_can_bus)
        self.can_sub = self.create_subscription(
            Frame,
            '/from_can_bus',
            self.can_rx_callback,
            100,
            callback_group=self._rx_cb_group)
        
        # 创建一个发布者，发布转化后的 ROS 话题
        # Publishes PWM values back to autonomous_control
        # Independent topics for each control command as requested
        self.pub_cmd_volts_l = self.create_publisher(Float64, '/cmd_volts_l', 10)
        self.pub_cmd_volts_r = self.create_publisher(Float64, '/cmd_volts_r', 10)
        self.pub_cmd_steer_l = self.create_publisher(Float64, '/cmd_steer_l', 10)
        self.pub_cmd_steer_r = self.create_publisher(Float64, '/cmd_steer_r', 10)

        # --- 发送部分 ---
        # 订阅 ROS 控制指令 — 全部放入同一个互斥回调组
        self.create_subscription(
            Odometry,
            '/odom_to_firmware',
            self.cmd_odom_callback,
            10,
            callback_group=self._tx_cb_group)
        self.create_subscription(
            JointState,
            '/joint_states_to_firmware',
            self.cmd_joint_state_callback,
            10,
            callback_group=self._tx_cb_group)
        self.create_subscription(
            Imu,
            '/imu_to_firmware',
            self.cmd_imu_callback,
            10,
            callback_group=self._tx_cb_group)

        # 创建一个发布者，将 CAN 帧发送给 ros2_socketcan (默认 /to_can_bus)
        self.can_pub = self.create_publisher(Frame, '/to_can_bus', 100)
        # Timer: periodically publish simulation timestamp on CAN ID 0x300
        # 100Hz 发送频率，与 autonomous_control.py 保持一致
        self.timer = self.create_timer(0.01, self.publish_sim_data, callback_group=self._tx_cb_group)
        
        self.get_logger().info('CAN Transceiver Node has been started.')

        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.ori = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        self.vel_lin = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.vel_ang = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.imu_vel_ang = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.imu_acc_lin = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.wheel_speed = {'rl': 0.0, 'rr': 0.0}
        

    def can_rx_callback(self, msg):
        # print(f"Received CAN frame ID: {hex(msg.id)}, Data: {msg.data.tobytes().hex()}")
        can_id = msg.id
        data = msg.data
        
        # Mapping based on control.c output assumption:
        # ID 0x180: Left Voltage, Right Voltage, Left Steering, Right Steering (4 floats)
        if can_id == 0x180:
            values = list(struct.unpack('<HHHH', bytes(data[:8])))
            values = [(v - 0x8000) / 1000.0 for v in values] # Convert back to float with offset
            #self.get_logger().info(f"Decoded CAN 0x180 - Volts L: {values[0]:.3f}, Volts R: {values[1]:.3f}, Steer L: {values[2]:.3f}, Steer R: {values[3]:.3f}")
            self.pub_cmd_volts_l.publish(Float64(data=values[0]))
            self.pub_cmd_volts_r.publish(Float64(data=values[1]))
            self.pub_cmd_steer_l.publish(Float64(data=values[2]))
            self.pub_cmd_steer_r.publish(Float64(data=values[3]))


    def send_can_2_float(self, can_id, value_1, value_2):
        """Helper to send a single float value as a CAN frame"""
        try:
            can_data = struct.pack('<ff', float(value_1), float(value_2))
            frame = Frame()
            # frame.header.stamp = self.get_clock().now().to_msg() # can_msgs/Frame typically has its own header structure or is simplified
            frame.id = can_id
            frame.dlc = 8
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
        postion = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_vel = msg.twist.twist.linear
        angular_vel = msg.twist.twist.angular
        self.pos['x'], self.pos['y'], self.pos['z'] = postion.x, postion.y, postion.z
        self.ori['x'], self.ori['y'], self.ori['z'], self.ori['w'] = orientation.x, orientation.y, orientation.z, orientation.w
        self.vel_lin['x'], self.vel_lin['y'], self.vel_lin['z'] = linear_vel.x, linear_vel.y, linear_vel.z
        self.vel_ang['x'], self.vel_ang['y'], self.vel_ang['z'] = angular_vel.x, angular_vel.y, angular_vel.z
        # 不再在回调中同步发送，由定时器统一发送

    def cmd_joint_state_callback(self, msg):
        """
        Encoders -> CAN (0x206 - 0x209)
        """
        # 根据 joint name 匹配对应的 CAN ID，发送速度值
        for i, name in enumerate(msg.name):
            if name == 'wheel_rl_joint' and len(msg.velocity) > i:
                self.wheel_speed['rl'] = msg.velocity[i]
            elif name == 'wheel_rr_joint' and len(msg.velocity) > i:
                self.wheel_speed['rr'] = msg.velocity[i]

    def cmd_imu_callback(self, msg):
        """
        IMU -> CAN (0x200 - 0x205)
        Accel(3): x, y, z
        Gyro(3): x, y, z
        """
        lin_acc = msg.linear_acceleration
        ang_vel = msg.angular_velocity
        self.imu_acc_lin['x'], self.imu_acc_lin['y'], self.imu_acc_lin['z'] = lin_acc.x, lin_acc.y, lin_acc.z
        self.imu_vel_ang['x'], self.imu_vel_ang['y'], self.imu_vel_ang['z'] = ang_vel.x, ang_vel.y, ang_vel.z


    def publish_sim_data(self):
        """Publish the current simulation time (seconds as float) on CAN ID 0x300."""
        # print("Publishing sim data")
        try:
            # dt 在函数开头、发送任何帧之前计算，准确反映两次 odom 回调的真实间隔
            now = self.get_clock().now()
            current_sim_time = now.nanoseconds / 1e9
            dt = current_sim_time - self.last_sim_time if hasattr(self, 'last_sim_time') else 0.01
            self.last_sim_time = current_sim_time
            dt = max(0.001, min(dt, 0.1))  # 限幅保护

            # 先发送所有数据帧，确保 can_test.py 在收到 0x300 触发时数据已就绪
            self.send_can_2_float(0x100, self.pos['x'], self.pos['y'])
            self.send_can_2_float(0x101, self.pos['z'], self.ori['x'])
            self.send_can_2_float(0x102, self.ori['y'], self.ori['z'])
            micro_sleep(100) # 100us 短暂睡眠，防止 FIFO 溢出
            self.send_can_2_float(0x103, self.ori['w'], self.vel_lin['x'])
            self.send_can_2_float(0x104, self.vel_lin['y'], self.vel_lin['z'])
            self.send_can_2_float(0x105, self.vel_ang['x'], self.vel_ang['y'])
            micro_sleep(100) # 100us 短暂睡眠，防止 FIFO 溢出
            self.send_can_2_float(0x106, self.vel_ang['z'], 0.0)
            self.send_can_2_float(0x107, self.wheel_speed['rl'], self.wheel_speed['rr'])

            self.send_can_2_float(0x200, self.imu_acc_lin['x'], self.imu_acc_lin['y'])
            micro_sleep(100) # 100us 短暂睡眠，防止 FIFO 溢出
            self.send_can_2_float(0x201, self.imu_acc_lin['z'], self.imu_vel_ang['x'])
            self.send_can_2_float(0x202, self.imu_vel_ang['y'], self.imu_vel_ang['z'])

            # 0x300 同步帧最后发送，触发 can_test.py 的控制计算
            self.send_can_2_float(0x300, dt, 0.0)
 
        except Exception as e:
            self.get_logger().error(f'Failed to publish sim time: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CanTransceiver()
    executor = MultiThreadedExecutor() # 使用多线程执行器
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
