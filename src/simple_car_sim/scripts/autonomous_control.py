#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
import time
import math
import csv
import os

# ========================================== #
#            公用数学与辅助函数                #
# ========================================== #

class MathUtils:
    """提供通用的数学计算辅助函数"""
    
    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """将四元数转换为欧拉角 (Roll, Pitch, Yaw)"""
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

    @staticmethod
    def normalize_angle(angle):
        """将角度归一化到 [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))


# ========================================== #
#               核心控制算法类                 #
# ========================================== #

class VehicleDynamics:
    """小车运动学与控制相关算法"""

    @staticmethod
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
        theta_e = MathUtils.normalize_angle(psi_path - yaw)
        return e, theta_e

    @staticmethod
    def calculate_wheel_speed(stanley_delta, vel_target, L=0.6, W=0.68):
        """根据前轮转向角和目标线速度，计算左右轮的目标角速度"""
        omega = vel_target * math.tan(stanley_delta) / L
        v_left = vel_target - (omega * W / 2)
        v_right = vel_target + (omega * W / 2)
        
        # 将左右轮的线速度转换为角速度
        wheel_radius = 0.16
        w_left = v_left / wheel_radius
        w_right = v_right / wheel_radius
        
        return w_left, w_right

    @staticmethod
    def calculate_ackermann_angles(delta, L=0.6, W=0.68):
        """根据自行车模型的转角 delta 计算左右前轮的转角 (Ackermann Steering)"""
        if abs(delta) < 1e-6:
            return 0.0, 0.0
        
        tan_delta = math.tan(delta)
        delta_left = math.atan(2 * L * tan_delta / (2 * L - W * tan_delta))
        delta_right = math.atan(2 * L * tan_delta / (2 * L + W * tan_delta))
        
        return delta_left, delta_right

    @staticmethod
    def stanley_control(x, y, yaw, vel, vel_target, corner_center, radius, roll, sum_roll, roll_rate=0.0):
        """Stanley 轨迹跟踪控制器与 Roll 稳定控制"""
        k = 4.0  # Stanley Control Gain
        k_soft = 2.0  # Softening speed limits
        
        # 计算几何误差
        e_geo, theta_e = VehicleDynamics.geometric_calculation(x, y, yaw, vel, corner_center, radius)
        
        # --- PIPE ROLL STABILIZATION LOGIC ---
        K_roll_p = 4.0
        K_roll_d = 1.5
        
        delta_geo = theta_e + math.atan2(k * e_geo, vel + k_soft) 
        
        # 添加防侧倾控制
        delta_roll = - K_roll_p * roll - K_roll_d * roll_rate - 0.5 * sum_roll
        
        delta = delta_geo + delta_roll
        # 限幅
        delta = max(min(delta, math.radians(45)), math.radians(-45))  
        
        # 计算驱动轮预期速度
        v_left, v_right = VehicleDynamics.calculate_wheel_speed(delta, vel_target, L=0.6, W=0.68)
        
        return v_left, v_right, delta, e_geo, theta_e


# ========================================== #
#               电机仿真模型类                 #
# ========================================== #

class DCMotorSim:
    """
    高保真直流电机仿真模型 (用于 HIL/SIL 测试)
    模拟物理电机的 电感-电阻-反电动势-粘滞摩擦 特性
    """
    def __init__(self, R=0.5, L=0.01, Kt=1.0, Ke=1.0, b=0.1, J=0.02, dt=0.05):
        self.R = 0.5      # 电枢电阻 (Ohm)
        self.L = 0.01     # 电枢电感 (Henry)
        self.Kt = 0.5     # 力矩常数 (Nm/A)
        self.Ke = 0.5     # 反电动势常数
        self.b = 0.1      # 粘滞摩擦系数 (Nms)
        self.J = 0.02     # 转子惯量 (kg*m^2)
        self.dt = dt
        self.current = 0.0 # 当前电流 (Amps)

    def step(self, voltage_in, angular_vel):
        """更新电机状态并输出电磁力矩"""
        # 1. 计算反电动势
        back_emf = self.Ke * angular_vel

        # 2. 计算电流更新 (解析解，解决大步长 simulation 不稳定问题)
        if self.R > 1e-4:
            tau = self.L / self.R
            i_steady = (voltage_in - back_emf) / self.R
            decay = math.exp(-self.dt / tau)
            self.current = self.current * decay + i_steady * (1.0 - decay)
        else:
            self.current = 0.0

        # 4. 驱动器电流保护
        MAX_CURRENT = 12.0 # Amps
        self.current = max(min(self.current, MAX_CURRENT), -MAX_CURRENT)

        # 5. 计算电磁力矩并减去摩擦
        torque_electromagnetic = self.Kt * self.current
        torque_output = torque_electromagnetic - (self.b * angular_vel)

        return torque_output


# ========================================== #
#               ROS 2 控制节点                 #
# ========================================== #

class CarController(Node):
    """
    小车主控制器节点，负责传感器数据订阅、控制算法计算以及指令发布。
    """
    def __init__(self):
        super().__init__('car_controller')
        self._init_publishers_subscribers()
        self._init_status_variables()
        self._init_simulation_config()
        self._init_logging()
        
        # 启动定时器
        self.create_timer(self.sim_dt, self.control_loop) # 控制主循环
        self.create_timer(0.5, self.print_status)         # 状态打印

    def _init_publishers_subscribers(self):
        """初始化 ROS 发布者和订阅者"""
        # 控制指令发布
        self.pub_rl = self.create_publisher(Float64, '/cmd_force_rl', 10)
        self.pub_rr = self.create_publisher(Float64, '/cmd_force_rr', 10)
        self.pub_steer_fl = self.create_publisher(Float64, '/cmd_pos_fl', 10)
        self.pub_steer_fr = self.create_publisher(Float64, '/cmd_pos_fr', 10)

        # 数据订阅
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # 中继数据发布 (供 Firmware 或其他节点使用)
        self.pub_odom = self.create_publisher(Odometry, '/odom_to_firmware', 10)
        self.pub_jointstate = self.create_publisher(JointState, '/joint_states_to_firmware', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu_to_firmware', 10)
        
        # 接收外部控制指令 (Independent Topics)
        self.create_subscription(Float64, '/cmd_volts_l', self.cb_volts_l, 10)
        self.create_subscription(Float64, '/cmd_volts_r', self.cb_volts_r, 10)
        self.create_subscription(Float64, '/cmd_steer_l', self.cb_steer_l, 10)
        self.create_subscription(Float64, '/cmd_steer_r', self.cb_steer_r, 10)
        
        self.use_external_pwm = True 
        self.ext_volts_l = 0.0
        self.ext_volts_r = 0.0
        self.ext_steer_l = 0.0
        self.ext_steer_r = 0.0

    def cb_volts_l(self, msg): self.ext_volts_l = msg.data
    def cb_volts_r(self, msg): self.ext_volts_r = msg.data
    def cb_steer_l(self, msg): self.ext_steer_l = msg.data
    def cb_steer_r(self, msg): self.ext_steer_r = msg.data

    def _init_status_variables(self):
        """初始化存储车辆状态的变量"""
        # 车辆位姿状态
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.front_x = 0.0
        self.front_y = 0.0
        self.vel_linear = 0.0
        
        self.roll_truth = 0.0
        self.pitch_truth = 0.0
        self.yaw_truth = 0.0
        
        # IMU 及关节状态
        self.imu_yaw = 0.0
        self.imu_accel_x = 0.0
        self.imu_angular_vel_z = 0.0
        self.imu_roll_rate = 0.0
        self.imu_yaw_rate = 0.0
        
        self.wheel_rl_speed = 0.0
        self.wheel_rr_speed = 0.0
        self.suspension_fl_joint = 0.0
        self.suspension_fr_joint = 0.0
        self.suspension_rl_joint = 0.0
        self.suspension_rr_joint = 0.0

        # PID 状态
        self.voltage_error_sum = {'rl': 0.0, 'rr': 0.0}
        self.voltage_last_error = {'rl': 0.0, 'rr': 0.0}
        self.error_roll_sum = 0.0

        # 转向平滑辅助
        self.current_delta = 0.0
        self.filtered_delta = 0.0
        self.delta_slew_rate = 10.0  # rad/s, maximum steering speed

    def _init_simulation_config(self):
        """初始化物理与仿真配置"""
        self.target_speed_linear = 0.0 # 当前平滑目标速度 (m/s)
        self.desired_max_speed = 1.0   # 最终期望速度
        self.wheel_radius = 0.16
        self.car_length = 0.3
        
        self.sim_dt = 0.01  # 仿真步长 (50Hz)
        self.motor_l = DCMotorSim(dt=self.sim_dt)
        self.motor_r = DCMotorSim(dt=self.sim_dt)
        
        # Performance monitoring
        self.last_loop_time = None
        self.get_logger().info('Node started. Waiting for simulation clock...')

    def _init_logging(self):
        """初始化数据记录 CSV"""
        # Use ROS Time for logging consistency with simulation
        self.start_time = None 
        self.log_file_path = '/home/yinyudream/robot/simulation/control/simulation_data.csv'
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.log_counter = 0  # Initialize counter for flush logic
        self.csv_writer.writerow([
            'time', 'x', 'y', 'yaw', 'velocity', 'target_velocity', 
            'roll', 'roll_rate', 'steering_delta', 'lateral_error', 
            'heading_error', 'wheel_speed_l', 'wheel_speed_r'
        ])

    # ---------------- 传感器回调函数 ---------------- #
    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        
        # 将位置前推到车头中心 (用于前视控制)
        self.front_x = self.pos_x + self.car_length * math.cos(self.yaw_truth)
        self.front_y = self.pos_y + self.car_length * math.sin(self.yaw_truth)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.vel_linear = math.sqrt(vx*vx + vy*vy)
        
        q = msg.pose.pose.orientation
        roll, pitch, yaw = MathUtils.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.roll_truth = roll
        self.pitch_truth = pitch
        self.yaw_truth = yaw
        
        self.pub_odom.publish(msg)  # 转发给 Firmware 或其他节点
        
    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = MathUtils.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.imu_yaw = yaw
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_angular_vel_z = msg.angular_velocity.z
        self.imu_roll_rate = msg.angular_velocity.x
        self.imu_yaw_rate = msg.angular_velocity.z

        self.pub_imu.publish(msg)  # 转发给 Firmware 或其他节点
        
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'wheel_rl_joint' and len(msg.velocity) > i:
                self.wheel_rl_speed = 0.05 * msg.velocity[i] + 0.95 * self.wheel_rl_speed
            elif name == 'wheel_rr_joint' and len(msg.velocity) > i:
                self.wheel_rr_speed = 0.05 * msg.velocity[i] + 0.95 * self.wheel_rr_speed
            elif name == 'suspension_fl_joint' and len(msg.position) > i:
                self.suspension_fl_joint = msg.position[i]
            elif name == 'suspension_fr_joint' and len(msg.position) > i:
                self.suspension_fr_joint = msg.position[i]
            elif name == 'suspension_rl_joint' and len(msg.position) > i:
                self.suspension_rl_joint = msg.position[i]
            elif name == 'suspension_rr_joint' and len(msg.position) > i:
                self.suspension_rr_joint = msg.position[i]

        self.pub_jointstate.publish(msg)  # 转发给 Firmware 或其他节点

    # ---------------- 底层控制功能 ---------------- #
    def calculate_voltage_pid(self, target_w, current_w, wheel):
        """模拟驱动器内部的速度闭环控制 (Input: Speed -> Output: Voltage)"""
        Kp = 5.0
        Ki = 1.0
        Kd = 0.05
        Kf = 0.5
        
        error = target_w - current_w
        self.voltage_error_sum[wheel] += error * self.sim_dt
        self.voltage_error_sum[wheel] = max(min(self.voltage_error_sum[wheel], 12.0), -12.0)
        
        d_error = (error - self.voltage_last_error[wheel]) / self.sim_dt
        self.voltage_last_error[wheel] = error

        voltage = (Kp * error) + (Ki * self.voltage_error_sum[wheel]) + (Kf * target_w) + (Kd * d_error)
        
        # 物理限制 (24V System, Max 12V per side assumption)
        BATTERY_VOLTAGE = 12.0
        voltage = max(min(voltage, BATTERY_VOLTAGE), -BATTERY_VOLTAGE)
        
        return voltage

    # ---------------- 主控制循环 ---------------- #
    def control_loop(self):
        """核心控制周期"""
        # Performance monitoring
        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.last_loop_time is None:
            self.last_loop_time = now
            return # Skip first loop to initialize last_loop_time

        dt = now - self.last_loop_time
        
        # Avoid huge dt on first loop or after pause for calculation stability
        if dt > 1.0 or dt < 0.0:
             dt = self.sim_dt
        
        self.last_loop_time = now
        
        # If the loop lag is significant (> 50ms), log a warning
        if dt > 0.05:
            self.get_logger().warn(f'Control loop lag: {dt:.3f}s (expected 0.010s)')
        
        # 1. 过弯自动减速机制
        SAFE_CORNER_SPEED_FACTOR = 2.0
        expected_speed = self.desired_max_speed / (1.0 + SAFE_CORNER_SPEED_FACTOR * abs(self.current_delta))
        expected_speed = max(expected_speed, 0.4)
        
        # 速度平滑滤波 (模拟加速度)
        if self.target_speed_linear < expected_speed:
             self.target_speed_linear += 0.5 * self.sim_dt
        else:
             self.target_speed_linear -= 1.0 * self.sim_dt
        self.target_speed_linear = max(min(self.target_speed_linear, 1.2), 0.0)

        # 2. 计算 Stanley 并且获得各轮目标角速度
        self.error_roll_sum += self.roll_truth * self.sim_dt
        target_w_l, target_w_r, raw_target_delta, e_geo, theta_e = VehicleDynamics.stanley_control(
            self.front_x, self.front_y, self.yaw_truth, self.vel_linear, 
            self.target_speed_linear, 
            corner_center=8.0, radius=1.6, roll=self.roll_truth, 
            sum_roll=self.error_roll_sum, roll_rate=self.imu_roll_rate
        )
        
        # 3. 转向平滑处理 (低通滤波与摆率限制)
        filter_alpha = 0.7
        self.filtered_delta = self.filtered_delta * (1.0 - filter_alpha) + raw_target_delta * filter_alpha
        
        max_delta_change = self.delta_slew_rate * self.sim_dt
        delta_change = self.filtered_delta - self.current_delta
        delta_change = max(min(delta_change, max_delta_change), -max_delta_change)
        self.current_delta += delta_change
        delta = self.current_delta

        # 4. 数据记录
        self._log_data(delta, e_geo, theta_e)

        # 5. 指令下发
        
        # Determine control source
        if self.use_external_pwm:
            # Use external commands directly
            delta_l = self.ext_steer_l
            delta_r = self.ext_steer_r
            voltage_l = self.ext_volts_l
            voltage_r = self.ext_volts_r
        else:
            # Internal calculation
            delta_l, delta_r = VehicleDynamics.calculate_ackermann_angles(delta, L=0.6, W=0.68)
            voltage_l = self.calculate_voltage_pid(target_w_l, self.wheel_rl_speed, 'rl')
            voltage_r = self.calculate_voltage_pid(target_w_r, self.wheel_rr_speed, 'rr')

        # Publish Steering
        self.pub_steer_fl.publish(Float64(data=delta_l))
        self.pub_steer_fr.publish(Float64(data=delta_r))

        # Calculate and Publish Torque (Motor Model Step)
        torque_l_cmd = self.motor_l.step(voltage_l, self.wheel_rl_speed)
        torque_r_cmd = self.motor_r.step(voltage_r, self.wheel_rr_speed)

        if not hasattr(self, '_diag_count'):
            self._diag_count = 0
        self._diag_count += 1
        if self._diag_count % 100 == 0:
            import math
            self.get_logger().info(
                f"[AC_DIAG] pos=({self.pos_x:.3f},{self.pos_y:.3f}) "
                f"ext_steer=({math.degrees(self.ext_steer_l):.2f}°,{math.degrees(self.ext_steer_r):.2f}°) "
                f"ext_volt=({self.ext_volts_l:.3f},{self.ext_volts_r:.3f}) "
                f"torque=({torque_l_cmd:.3f},{torque_r_cmd:.3f}) "
                f"wheel_spd=({self.wheel_rl_speed:.3f},{self.wheel_rr_speed:.3f})"
            )

        self.pub_rl.publish(Float64(data=torque_l_cmd))
        self.pub_rr.publish(Float64(data=torque_r_cmd))

    def _log_data(self, delta, e_geo, theta_e):
        """记录系统状态至 CSV 文件"""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
            
        current_time = now - self.start_time
        try:
            self.csv_writer.writerow([
                current_time, self.pos_x, self.pos_y, self.yaw_truth, 
                self.vel_linear, self.target_speed_linear, self.roll_truth, 
                self.imu_roll_rate, delta, e_geo, theta_e, 
                self.wheel_rl_speed, self.wheel_rr_speed
            ])
            self.log_counter += 1
            if self.log_counter % 50 == 0:  # Flash every 0.5s (50 loops) to avoid IO blocking
                self.log_file.flush()
        except ValueError:
            pass

    def print_status(self):
        """打印终端监控界面信息"""
        if self.start_time is None:
             return 

        target_w_l, target_w_r, delta, e_geo, theta_e = VehicleDynamics.stanley_control(
            self.front_x, self.front_y, self.yaw_truth, self.vel_linear, 
            self.target_speed_linear, corner_center=8.0, radius=1.6, 
            roll=self.roll_truth, sum_roll=self.error_roll_sum, roll_rate=self.imu_roll_rate
        )
        # print(f"Target W: L={target_w_l:.2f}, R={target_w_r:.2f} | Delta: {math.degrees(delta):.2f} deg | Lat Err: {e_geo:.3f}m | Hdg Err: {math.degrees(theta_e):.2f} deg")
        # now = self.get_clock().now().nanoseconds / 1e9
        # print(f"--- Time: {now - self.start_time:.1f} s ---")
        # print(f"[Ground Truth] Pos: ({self.pos_x:.2f}, {self.pos_y:.2f})  Vel: {self.vel_linear:.2f} m/s")
        # print(f"[Orientation ] Roll: {math.degrees(self.roll_truth):.1f} deg  Pitch: {math.degrees(self.pitch_truth):.1f} deg  Yaw: {math.degrees(self.yaw_truth):.1f} deg")
        # print(f"[IMU Sensor  ] Accel X: {self.imu_accel_x:.2f} m/s^2  Yaw: {math.degrees(self.imu_yaw):.1f} deg  Yaw Rate: {math.degrees(self.imu_angular_vel_z):.1f} deg/s")
        # print(f"[Encoder     ] RL Speed: {self.wheel_rl_speed:.1f} rad/s  RR Speed: {self.wheel_rr_speed:.1f} rad/s")
        # print(f"[Suspension  ] FL: {self.suspension_fl_joint:.3f} deg  FR: {self.suspension_fr_joint:.3f} deg  RL: {self.suspension_rl_joint:.3f} deg  RR: {self.suspension_rr_joint:.3f} deg")

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
