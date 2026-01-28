#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math

class AutonomousController(Node):
    """
    自主控制器节点
    功能：
    1. 接收里程计(Odometry)信息，监控机器人的状态。
    2. 控制机器人自动展开并适应管道。
    3. 控制机器人沿管道行进。
    """
    def __init__(self):
        super().__init__('autonomous_controller')
        
        # Publishers for Drive Wheels (Front and Rear for each leg)
        # 初始化驱动轮速度发布者 (每个腿有前后两个驱动轮，共8个)
        # 对应 URDF 中的连续关节 (continuous joint)
        self.pub_top_front = self.create_publisher(Float64, '/simple_car/top_front_wheel_vel', 10)
        self.pub_top_rear = self.create_publisher(Float64, '/simple_car/top_rear_wheel_vel', 10)
        self.pub_bottom_front = self.create_publisher(Float64, '/simple_car/bottom_front_wheel_vel', 10)
        self.pub_bottom_rear = self.create_publisher(Float64, '/simple_car/bottom_rear_wheel_vel', 10)
        self.pub_left_front = self.create_publisher(Float64, '/simple_car/left_front_wheel_vel', 10)
        self.pub_left_rear = self.create_publisher(Float64, '/simple_car/left_rear_wheel_vel', 10)
        self.pub_right_front = self.create_publisher(Float64, '/simple_car/right_front_wheel_vel', 10)
        self.pub_right_rear = self.create_publisher(Float64, '/simple_car/right_rear_wheel_vel', 10)
        
        # Publishers for Expansion (Drive Section)
        # 初始化驱动节(Drive Section)的伸缩关节控制器
        # 对应 URDF 中的移动关节 (prismatic joint)，用于改变驱动腿的长度
        # 恢复为速度控制接口 (Velocity Command)，实际逻辑在 Python 层实现位置闭环
        self.pub_drive_exp_top = self.create_publisher(Float64, '/simple_car/drive_exp_top_vel', 10)
        self.pub_drive_exp_bottom = self.create_publisher(Float64, '/simple_car/drive_exp_bottom_vel', 10)
        self.pub_drive_exp_left = self.create_publisher(Float64, '/simple_car/drive_exp_left_vel', 10)
        self.pub_drive_exp_right = self.create_publisher(Float64, '/simple_car/drive_exp_right_vel', 10)
        
        # Publishers for Expansion (Functional Section)
        # 初始化功能节(Functional Section)的伸缩关节控制器
        self.pub_func_exp_top = self.create_publisher(Float64, '/simple_car/func_exp_top_vel', 10)
        self.pub_func_exp_bottom = self.create_publisher(Float64, '/simple_car/func_exp_bottom_vel', 10)
        self.pub_func_exp_left = self.create_publisher(Float64, '/simple_car/func_exp_left_vel', 10)
        self.pub_func_exp_right = self.create_publisher(Float64, '/simple_car/func_exp_right_vel', 10)
        
        # Subscribers
        # 订阅里程计信息，用于获取位置和速度
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # 订阅关节状态信息 (可选，暂未使用)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State
        self.current_speed = 0.0
        self.target_speed = -0.25 # 目标行进速度 (m/s)
        
        # Pipe Radius is 0.8m.
        # Robot Radius (collapsed) = 0.27 (body) + 0.1 (bogie) + 0.18 (wheel) = 0.55m.
        # Gap to fill = 0.8 - 0.55 = 0.25m.
        # We add some compression for grip.
        # 管道参数與目标伸缩量计算：
        # 管道半径 = 0.8m
        # 机器人收缩状态半径 ≈ 0.55m (车身+转向架+轮子)
        # 需要填充的空隙 = 0.25m
        # 使用位置控制(Python层闭环)，设置一个大于空隙 (0.25m) 的值以产生持续压力
        self.target_expansion_drive = 0.30 # 驱动节目标伸长量 (m)
        self.target_expansion_func = 0.30 # 功能节目标伸长量 (m)
        self.expansion_ramp_time = 3.0 # 展开过程持续时间 (秒)
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        
        # Joint positions for custom feedback control
        self.joint_positions = {}
        for name in ['drive_expansion_joint_top', 'drive_expansion_joint_bottom', 
                     'drive_expansion_joint_left', 'drive_expansion_joint_right',
                     'expansion_joint_top', 'expansion_joint_bottom',
                     'expansion_joint_left', 'expansion_joint_right']:
             self.joint_positions[name] = 0.0

        # Timer for control loop
        # 创建定时器，执行控制循环 (10Hz)
        self.create_timer(0.1, self.control_loop)
        # 创建定时器，打印状态信息 (1Hz)
        self.create_timer(1.0, self.print_status)
        
        self.start_time = time.time()
        self.state = "INIT" # 状态机: INIT(初始化), EXPANDING(展开中), RUNNING(运行中)
        self.expand_start_time = None

    def odom_callback(self, msg):
        """
        处理里程计回调
        更新机器人的位置和速度变量
        """
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        
        # Calculate linear velocity magnitude
        # 计算线速度的大小
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.vel_linear = math.sqrt(vx*vx + vy*vy + vz*vz)
        
        # Calculate angular velocity magnitude
        # 计算角速度的大小
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z
        self.vel_angular = math.sqrt(wx*wx + wy*wy + wz*wz)

    def joint_state_callback(self, msg):
        """
        更新关节位置，用于闭环控制
        """
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i]

    def compute_position_control(self, target, current, kp=2.0, max_vel=0.5):
        """
        简单的 P 控制器实现位置控制
        :param target: 目标位置
        :param current: 当前位置
        :param kp: 比例增益
        :param max_vel: 最大速度限制
        :return: 速度指令
        """
        error = target - current
        vel = error * kp
        # 限制速度范围
        if vel > max_vel:
            vel = max_vel
        elif vel < -max_vel:
            vel = -max_vel
        return vel

    def publish_expansion(self, drive_target, func_target):
        """
        发布伸缩指令 (内部计算速度指令实现位置控制)
        :param drive_target: 驱动节目标位置 (m)
        :param func_target: 功能节目标位置 (m)
        """
        # Drive Section Control
        self.pub_drive_exp_top.publish(Float64(data=self.compute_position_control(
            drive_target, self.joint_positions.get('drive_expansion_joint_top', 0.0))))
        self.pub_drive_exp_bottom.publish(Float64(data=self.compute_position_control(
            drive_target, self.joint_positions.get('drive_expansion_joint_bottom', 0.0))))
        self.pub_drive_exp_left.publish(Float64(data=self.compute_position_control(
            drive_target, self.joint_positions.get('drive_expansion_joint_left', 0.0))))
        self.pub_drive_exp_right.publish(Float64(data=self.compute_position_control(
            drive_target, self.joint_positions.get('drive_expansion_joint_right', 0.0))))
        
        # Functional Section Control
        # return
        self.pub_func_exp_top.publish(Float64(data=self.compute_position_control(
            func_target, self.joint_positions.get('expansion_joint_top', 0.0))))
        self.pub_func_exp_bottom.publish(Float64(data=self.compute_position_control(
            func_target, self.joint_positions.get('expansion_joint_bottom', 0.0))))
        self.pub_func_exp_left.publish(Float64(data=self.compute_position_control(
            func_target, self.joint_positions.get('expansion_joint_left', 0.0))))
        self.pub_func_exp_right.publish(Float64(data=self.compute_position_control(
            func_target, self.joint_positions.get('expansion_joint_right', 0.0))))

    def publish_drive(self, speed):
        """
        发布驱动轮速度指令
        :param speed: 目标线速度
        """
        msg = Float64()
        msg.data = speed
        self.pub_top_front.publish(msg)
        self.pub_top_rear.publish(msg)
        self.pub_bottom_front.publish(msg)
        self.pub_bottom_rear.publish(msg)
        self.pub_left_front.publish(msg)
        self.pub_left_rear.publish(msg)
        self.pub_right_front.publish(msg)
        self.pub_right_rear.publish(msg)

    def control_loop(self):
        """
        主控制循环
        实现简单的状态机逻辑
        """
        elapsed = time.time() - self.start_time
        
        if self.state == "INIT":
            # 初始化阶段：等待系统稳定 (2秒)
            if elapsed > 2.0:
                self.state = "EXPANDING"
                self.get_logger().info("State: EXPANDING")
                self.expand_start_time = time.time()
                
        elif self.state == "EXPANDING":
            # 展开阶段：逐渐增加伸缩指令，平滑接触管壁
            ramp_elapsed = time.time() - self.expand_start_time if self.expand_start_time else 0.0
            ramp_ratio = min(ramp_elapsed / self.expansion_ramp_time, 1.0)
            
            drive_cmd = ramp_ratio * self.target_expansion_drive
            func_cmd = ramp_ratio * self.target_expansion_func
            
            self.publish_expansion(drive_cmd, func_cmd)
            
            if ramp_ratio >= 1.0:
                self.state = "RUNNING"
                self.get_logger().info("State: RUNNING")
                
        elif self.state == "RUNNING":
            # 运行阶段：保持压力，并向前驱动
            self.publish_expansion(self.target_expansion_drive, self.target_expansion_func) # Keep pressure
            self.publish_drive(self.target_speed)
            if abs(self.vel_linear) < 0.005:
                pass
                # 卡住时完全收缩 (位置设为0)
                # self.publish_expansion(0.0, 0.0)
                    

    def print_status(self):
        """
        打印机器人状态到终端
        """
        print(f"\033[2J\033[H") # 这是一个 ANSI 转义序列，用于清屏和光标复位
        print("="*40)
        print(f"Pipeline Robot Autonomous Monitor (管道机器人自主监控)")
        print("="*40)
        print(f"State (状态):           {self.state}")
        print(f"Time (运行时间):            {time.time() - self.start_time:.1f} s")
        print("-" * 20)
        print(f"Position (位置 X,Y,Z): ({self.pos_x:.2f}, {self.pos_y:.2f}, {self.pos_z:.2f})")
        print(f"Linear Velocity (线速度):  {self.vel_linear:.2f} m/s")
        print(f"Angular Velocity (角速度): {self.vel_angular:.2f} rad/s")
        print("-" * 20)
        print(f"Target Exp Drive (目标驱动伸缩): {self.target_expansion_drive:.2f} m")
        print(f"Target Exp Func (目标功能伸缩):  {self.target_expansion_func:.2f} m")
        print(f"Target Speed (目标速度):       {self.target_speed:.2f} m/s")
        print("="*40)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop everything
        # 程序退出时，停止所有运动
        node.publish_drive(0.0)
        node.publish_expansion(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
