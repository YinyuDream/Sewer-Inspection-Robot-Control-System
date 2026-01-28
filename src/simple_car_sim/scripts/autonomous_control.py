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
        # 订阅关节状态信息，用于闭环位置控制
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State
        self.current_speed = 0.0
        self.target_speed = -0.25 # 目标行进速度 (m/s)
        self.joint_positions = {} # 存储关节当前位置
        
        # Pipe Radius is 0.8m.
        # Robot Radius (collapsed) = 0.27 (body) + 0.1 (bogie) + 0.18 (wheel) = 0.55m.
        self.collapsed_radius = 0.55 # 收缩状态下的基础半径
        
        # Gap to fill = 0.8 - 0.55 = 0.25m.
        # We add some compression for grip.
        # 管道参数與目标伸缩量计算：
        # 管道半径 = 0.8m
        # 机器人收缩状态半径 ≈ 0.55m (车身+转向架+轮子)
        # 需要填充的空隙 = 0.25m
        # 目标伸缩量需要略小于空隙，或者通过力控制。
        # 这里使用位置控制，设置一个适中的伸长量，依靠物理引擎的接触弹性产生压力（抓地力）
        self.target_expansion_drive = 0.20 # 驱动节目标伸长量 (m)
        self.target_expansion_func = 0.18 # 功能节目标伸长量 (m)
        self.expansion_ramp_time = 3.0 # 展开过程持续时间 (秒)
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        
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
        处理关节状态回调
        更新关节位置信息，用于 P 控制器
        """
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]

    def get_joint_position(self, keyword):
        """
        获取包含指定关键字的关节位置
        简单处理：返回第一个匹配项的的位置
        """
        for name, pos in self.joint_positions.items():
            if keyword in name:
                return pos
        return 0.0 # 默认值

    def set_target_diameter(self, diameter):
        """
        设置机器人的目标直径
        会自动计算所需的伸缩量
        :param diameter: 目标直径 (米)
        """
        target_radius = diameter / 2.0
        # 伸缩量 = 目标半径 - 收缩状态半径
        expansion = target_radius - self.collapsed_radius
        
        # 限制在物理范围内 (0.0 ~ 0.8)
        expansion = max(0.0, min(expansion, 0.8))
        
        self.target_expansion_drive = expansion
        # 功能节稍微小一点，避免干涉或为了特定功能
        self.target_expansion_func = max(0.0, expansion - 0.02)
        
        self.get_logger().info(f"Set diameter to {diameter:.2f}m (Expansion: {expansion:.2f}m)")

    def calculate_pid_velocity(self, target_pos, keyword, kp=5.0, max_vel=0.5):
        """
        计算 P 控制器的速度指令
        :param target_pos: 目标位置 (米)
        :param keyword: 关节名称关键字
        :param kp: 比例系数
        :param max_vel: 最大速度限制
        """
        current_pos = self.get_joint_position(keyword)
        error = target_pos - current_pos
        vel = error * kp
        
        # 限制最大速度
        return max(min(vel, max_vel), -max_vel)

    def publish_expansion(self, drive_target, func_target):
        """
        发布伸缩指令 (位置控制 -> 速度指令)
        :param drive_target: 驱动节目标位置 (m)
        :param func_target: 功能节目标位置 (m)
        """
        # 使用 P 控制器计算速度
        vel_drive_top = self.calculate_pid_velocity(drive_target, 'drive_expansion_joint_top')
        vel_drive_bottom = self.calculate_pid_velocity(drive_target, 'drive_expansion_joint_bottom')
        vel_drive_left = self.calculate_pid_velocity(drive_target, 'drive_expansion_joint_left')
        vel_drive_right = self.calculate_pid_velocity(drive_target, 'drive_expansion_joint_right')
        
        vel_func_top = self.calculate_pid_velocity(func_target, 'expansion_joint_top')
        vel_func_bottom = self.calculate_pid_velocity(func_target, 'expansion_joint_bottom')
        vel_func_left = self.calculate_pid_velocity(func_target, 'expansion_joint_left')
        vel_func_right = self.calculate_pid_velocity(func_target, 'expansion_joint_right')

        # 发布计算出的速度
        self.pub_drive_exp_top.publish(Float64(data=vel_drive_top))
        self.pub_drive_exp_bottom.publish(Float64(data=vel_drive_bottom))
        self.pub_drive_exp_left.publish(Float64(data=vel_drive_left))
        self.pub_drive_exp_right.publish(Float64(data=vel_drive_right))
        
        self.pub_func_exp_top.publish(Float64(data=vel_func_top))
        self.pub_func_exp_bottom.publish(Float64(data=vel_func_bottom))
        self.pub_func_exp_left.publish(Float64(data=vel_func_left))
        self.pub_func_exp_right.publish(Float64(data=vel_func_right))

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
            
            # 脱困逻辑 / 自动收缩演示
            # 如果检测到速度过低（卡住），尝试减小直径脱困
            if abs(self.vel_linear) < 0.01:
                self.get_logger().warn("Stuck detected! Shrinking...")
                
                # 暂时缩小直径到 1.2m (约等于收缩状态)
                current_drive_target = self.target_expansion_drive
                self.set_target_diameter(1.2) 
                
                # 发布收缩后的指令并等待
                self.publish_expansion(self.target_expansion_drive, self.target_expansion_func)
                time.sleep(2.0)
                
                # 恢复原来的直径
                self.get_logger().info("Restoring diameter...")
                self.target_expansion_drive = current_drive_target
                self.target_expansion_func = max(0.0, current_drive_target - 0.02)

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
