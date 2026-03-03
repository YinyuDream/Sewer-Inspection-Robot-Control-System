#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
import time
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def geometric_calculation(x, y, yaw, vel, corner_center, radius):
    """
    计算横向误差 e 和航向误差 theta_e
    假设小车逆时针 (CCW) 行驶
    """
    e = 0.0
    psi_path = 0.0
    d_path = corner_center + radius  # 路径边界位置 (例如 2.0)
    # --- 区域判定逻辑 ---
    
    # 1. 右上圆角区域
    if x > corner_center and y > corner_center:
        xc, yc = corner_center, corner_center
        dist = math.sqrt((x - xc)**2 + (y - yc)**2)
        e = dist - radius
        # 路径切线方向: 相对于圆心的角度 + 90度 (逆时针)
        angle_to_center = math.atan2(y - yc, x - xc)
        psi_path = angle_to_center + math.pi / 2

    # 2. 左上圆角区域
    elif x < -corner_center and y > corner_center:
        xc, yc = -corner_center, corner_center
        dist = math.sqrt((x - xc)**2 + (y - yc)**2)
        e = dist - radius
        angle_to_center = math.atan2(y - yc, x - xc)
        psi_path = angle_to_center + math.pi / 2

    # 3. 左下圆角区域
    elif x < -corner_center and y < -corner_center:
        xc, yc = -corner_center, -corner_center
        dist = math.sqrt((x - xc)**2 + (y - yc)**2)
        e = dist - radius
        angle_to_center = math.atan2(y - yc, x - xc)
        psi_path = angle_to_center + math.pi / 2

    # 4. 右下圆角区域
    elif x > corner_center and y < -corner_center:
        xc, yc = corner_center, -corner_center
        dist = math.sqrt((x - xc)**2 + (y - yc)**2)
        e = dist - radius
        angle_to_center = math.atan2(y - yc, x - xc)
        psi_path = angle_to_center + math.pi / 2

    # 5. 上边直线 (y=2)
    elif y > corner_center:
        e = y - d_path
        psi_path = math.pi  # 向左开

    # 6. 下边直线 (y=-2)
    elif y < -corner_center:
        e = -d_path - y
        psi_path = 0.0  # 向右开

    # 7. 左边直线 (x=-2)
    elif x < -corner_center:
        e = -d_path - x
        psi_path = -math.pi / 2  # 向下开

    # 8. 右边直线 (x=2)
    elif x > corner_center:
        e = x - d_path
        psi_path = math.pi / 2  # 向上开

    # --- 计算航向误差 ---
    theta_e = normalize_angle(psi_path - yaw)
    return e, theta_e

def calculate_wheel_speed(stanley_delta, vel_target, L = 0.6, W = 0.68):
    # vel_target 应该是在中心线的线速度 m/s
    omega = vel_target * math.tan(stanley_delta) / L
    # print(omega)
    v_left = vel_target - (omega * W / 2)
    v_right = vel_target + (omega * W / 2)
    
    # 将左右轮的线速度转换为角速度
    wheel_radius = 0.16
    w_left = v_left / wheel_radius
    w_right = v_right / wheel_radius
    
    return w_left, w_right



def stanley_control(x, y, yaw, vel, vel_target, corner_center, radius, roll):
    k = 4.0  # Stanley 控制增益 (增大以加强横向误差修正)
    k_soft = 0.3  # 软化速度，避免低速时控制过激
    e, theta_e = geometric_calculation(x, y, yaw, vel, corner_center, radius)
    e = -roll / 3
    # Stanley 控制律
    # 增加前馈控制项或增加 P 增益
    delta = theta_e + math.atan2(k * e, vel + k_soft)
    delta = max(min(delta, math.radians(45)), math.radians(-45))  # 放宽转向角度限制
    
    # Differential drive kinematics
    # 使用较小的 L 值 (0.4) 来增加转向灵敏度，因为差速车可以原地转向
    # W (0.68) 保持物理真实值
    v_left, v_right = calculate_wheel_speed(delta, vel_target, L=0.6, W=0.68)
    # print(e, math.degrees(theta_e), math.degrees(delta))
    return v_left, v_right

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        
        # Publishers for Drive Wheels (Rear)
        self.pub_rl = self.create_publisher(Float64, '/cmd_vel_rl', 10)
        self.pub_rr = self.create_publisher(Float64, '/cmd_vel_rr', 10)
        # self.pub_rl = self.create_publisher(Float64, '/cmd_force_rl', 10)
        # self.pub_rr = self.create_publisher(Float64, '/cmd_force_rr', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # States
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.front_x = 0.0
        self.front_y = 0.0
        self.vel_linear = 0.0
        self.roll_truth = 0.0
        self.pitch_truth = 0.0
        self.yaw_truth = 0.0
        
        self.imu_yaw = 0.0
        self.imu_accel_x = 0.0
        self.imu_angular_vel_z = 0.0
        
        self.wheel_rl_speed = 0.0
        self.wheel_rr_speed = 0.0
        self.suspension_fl_joint = 0.0
        self.suspension_fr_joint = 0.0
        self.suspension_rl_joint = 0.0
        self.suspension_rr_joint = 0.0
        
        # Control targets
        self.target_speed_linear = 0.2 # m/s
        self.wheel_radius = 0.16
        self.car_length = 0.3
        
        # PID state
        self.error_sum = {'rl': 0.0, 'rr': 0.0}
        self.last_error = {'rl': 0.0, 'rr': 0.0}

        self.start_time = time.time()
        
        self.create_timer(0.05, self.control_loop) # 20Hz
        self.create_timer(0.5, self.print_status)  # 2Hz
        
    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        self.front_x = self.pos_x + self.car_length * math.cos(self.yaw_truth)
        self.front_y = self.pos_y + self.car_length * math.sin(self.yaw_truth)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.vel_linear = math.sqrt(vx*vx + vy*vy)
        
        # Calculate yaw from quaternion
        q = msg.pose.pose.orientation
        quat = [q.w, q.x, q.y, q.z]
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.roll_truth = roll
        self.pitch_truth = pitch
        self.yaw_truth = yaw
        
    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.w, q.x, q.y, q.z]
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.imu_yaw = yaw
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_angular_vel_z = msg.angular_velocity.z
        
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'wheel_rl_joint' and len(msg.velocity) > i:
                self.wheel_rl_speed = msg.velocity[i]
            elif name == 'wheel_rr_joint' and len(msg.velocity) > i:
                self.wheel_rr_speed = msg.velocity[i]
            elif name == 'suspension_fl_joint' and len(msg.position) > i:
                self.suspension_fl_joint = msg.position[i]
            elif name == 'suspension_fr_joint' and len(msg.position) > i:
                self.suspension_fr_joint = msg.position[i]
            elif name == 'suspension_rl_joint' and len(msg.position) > i:
                self.suspension_rl_joint = msg.position[i]
            elif name == 'suspension_rr_joint' and len(msg.position) > i:
                self.suspension_rr_joint = msg.position[i]
                
    def calculate_torque(self, target_angular_speed, current_angular_speed, wheel):
        # 调整 PID 参数以获得更平稳的控制
        Kp = 4.0
        Ki = 0.5
        Kd = 0.0
        
        error = target_angular_speed - current_angular_speed
        
        self.error_sum[wheel] += error * 0.05
        # 积分限幅
        self.error_sum[wheel] = max(min(self.error_sum[wheel], 20.0), -20.0)
        
        u_p = Kp * error
        u_i = Ki * self.error_sum[wheel]
        u_d = Kd * (error - self.last_error[wheel]) / 0.05
        
        self.last_error[wheel] = error
        
        torque = u_p + u_i + u_d
        # 力矩限幅，避免过大导致打滑或不稳定
        # URDF limit is 500, but reasonable control torque is lower
        torque = max(min(torque, 50.0), -50.0)
        return torque

    def control_loop(self):
        
        target_w_l, target_w_r = stanley_control(self.front_x, self.front_y, self.yaw_truth, self.vel_linear, self.target_speed_linear, corner_center=2.0, radius=1.6, roll=self.roll_truth)
        target_w = self.target_speed_linear / self.wheel_radius  # 直接使用线速度转换为角速度作为目标
        # torque_rl = self.calculate_torque(target_w, self.wheel_rl_speed, 'rl')
        # torque_rr = self.calculate_torque(target_w, self.wheel_rr_speed, 'rr')
        torque_rl = self.calculate_torque(target_w_l, self.wheel_rl_speed, 'rl')
        torque_rr = self.calculate_torque(target_w_r, self.wheel_rr_speed, 'rr')
        # print(torque_rl, torque_rr)
        # print(torque_rl, torque_rr)
        self.pub_rl.publish(Float64(data=target_w_l))
        self.pub_rr.publish(Float64(data=target_w_r))
        
    def print_status(self):
        
        target_w_l, target_w_r = stanley_control(self.front_x, self.front_y, self.yaw_truth, self.vel_linear, self.target_speed_linear, corner_center=2.0, radius=1.6, roll=self.roll_truth)
        print(target_w_l, target_w_r)
        print(f"--- Time: {time.time() - self.start_time:.1f} s ---")
        print(f"[Ground Truth] Pos: ({self.pos_x:.2f}, {self.pos_y:.2f})  Vel: {self.vel_linear:.2f} m/s  Yaw: {math.degrees(self.yaw_truth):.1f} deg")
        print(f"[Orientation ] Roll: {math.degrees(self.roll_truth):.1f} deg  Pitch: {math.degrees(self.pitch_truth):.1f} deg  Yaw: {math.degrees(self.yaw_truth):.1f} deg")
        print(f"[IMU Sensor  ] Accel X: {self.imu_accel_x:.2f} m/s^2  Yaw: {math.degrees(self.imu_yaw):.1f} deg  Yaw Rate: {math.degrees(self.imu_angular_vel_z):.1f} deg/s")
        print(f"[Encoder     ] RL Speed: {self.wheel_rl_speed:.1f} rad/s  RR Speed: {self.wheel_rr_speed:.1f} rad/s")
        print(f"[Suspension  ] FL: {self.suspension_fl_joint:.3f} deg  FR: {self.suspension_fr_joint:.3f} deg  RL: {self.suspension_rl_joint:.3f} deg  RR: {self.suspension_rr_joint:.3f} deg")

def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_rl.publish(Float64(data=0.0))
        node.pub_rr.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
