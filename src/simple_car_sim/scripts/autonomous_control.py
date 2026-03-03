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



def stanley_control(x, y, yaw, vel, vel_target, corner_center, radius, roll, sum_roll):
    k = 10.0  # Stanley 控制增益 (增大以加强横向误差修正)
    k_soft = 0.3  # 软化速度，避免低速时控制过激
    e, theta_e = geometric_calculation(x, y, yaw, vel, corner_center, radius)
    # e += - math.tan(roll - math.pi/4)
    # print(e, theta_e)
    # Stanley 控制律
    # 增加前馈控制项或增加 P 增益
    delta = theta_e + math.atan2(k * e, vel + k_soft) - 8 * math.tan(roll) - 2 * math.tan(sum_roll)  # 积分项
    # print(delta)
    delta = max(min(delta, math.radians(45)), math.radians(-45))  # 放宽转向角度限制
    
    # Differential drive kinematics
    # 使用较小的 L 值 (0.4) 来增加转向灵敏度，因为差速车可以原地转向
    # W (0.68) 保持物理真实值
    v_left, v_right = calculate_wheel_speed(delta, vel_target, L=0.6, W=0.68)
    # print(e, math.degrees(theta_e), math.degrees(delta))
    # print(f"Target Vel: {vel_target:.2f} m/s  Left Wheel: {v_left:.2f} m/s  Right Wheel: {v_right:.2f} m/s")
    return v_left, v_right


class DCMotorSim:
    """
    高保真直流电机仿真模型 (用于 HIL/SIL 测试)
    模拟物理电机的 电感-电阻-反电动势-粘滞摩擦 特性
    """
    def __init__(self, R=0.5, L=0.01, Kt=1.0, Ke=1.0, b=0.1, J=0.02, dt=0.05):
        # 参数已调整以适配 15kg 级机器人 (0.16m 轮半径)
        self.R = 0.5      # 电枢电阻 (Ohm)
        self.L = 0.01     # 电枢电感 (Henry)
        self.Kt = 0.5     # 减小力矩常数，削弱“暴力”加速 (Nm/A)
        self.Ke = 0.5     # 反电动势常数 (匹配Kt)
        self.b = 0.1      # 粘滞摩擦系数 (Nms)
        self.J = 0.02     # 转子惯量 (kg*m^2)
        self.dt = dt
        
        self.current = 0.0 # 当前电流 (Amps)

    def step(self, voltage_in, angular_vel):
        """
        输入: 驱动电压(V), 当前机械转速(rad/s)
        输出: 产生的电磁力矩(Nm)
        """
        # 1. 计算反电动势 (Back-EMF): V_emf = Ke * omega
        back_emf = self.Ke * angular_vel

        # 2. 计算电流更新 (使用解析解，解决大步长 simulation 不稳定问题)
        # 欧拉积分在 dt > L/R 时会发散，产生剧烈震荡 (Current Instability)
        # 精确解: i(t+dt) = i(t) * exp(-dt/tau) + I_steady * (1 - exp(-dt/tau))
        if self.R > 1e-4:
            tau = self.L / self.R
            i_steady = (voltage_in - back_emf) / self.R
            decay = math.exp(-self.dt / tau)
            self.current = self.current * decay + i_steady * (1.0 - decay)
        else:
            self.current = 0.0

        # 3. (已替换为解析解，跳过欧拉积分)
        # self.current += di_dt * self.dt

        # 4. 驱动器电流保护 (Current Limit)
        # 【关键修改】限制最大电流为 12A (对于 Kt=0.5，最大输出力矩为 6Nm)
        # 物理原因: 经计算单轮理论最大静摩擦力矩约为 5.88 Nm (轮载2.5kg, mu=1.5, r=0.16)
        # 超过 6Nm 必定导致突破物理极限的原地打滑烧胎，之后瞬间加速到50rad/s。
        MAX_CURRENT = 12.0 # Amps
        self.current = max(min(self.current, MAX_CURRENT), -MAX_CURRENT)

        # 5. 计算电磁力矩: T_e = Kt * i
        torque_electromagnetic = self.Kt * self.current

        # 6. 减去电机内部摩擦: T_out = T_e - b * omega
        torque_output = torque_electromagnetic - (self.b * angular_vel)

        return torque_output

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        
        # Publishers for Drive Wheels (Force/Torque Control)
        # HIL 模式下必须发送力矩
        self.pub_rl = self.create_publisher(Float64, '/cmd_force_rl', 10)
        self.pub_rr = self.create_publisher(Float64, '/cmd_force_rr', 10)

        # self.pub_rl = self.create_publisher(Float64, '/cmd_force_rl', 10)
        # self.pub_rr = self.create_publisher(Float64, '/cmd_force_rr', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.pub_odom = self.create_publisher(Odometry, '/odom_to_firmware', 10)
        self.pub_jointstate = self.create_publisher(JointState, '/joint_states_to_firmware', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu_to_firmware', 10)
        


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
        # 使用软启动，初始目标设为0，在循环中缓慢增加到1.0
        self.target_speed_linear = 0.0 # m/s
        self.desired_max_speed = 1.0   # 最终期望速度
        self.wheel_radius = 0.16
        self.car_length = 0.3
        
        # HIL Simulation State
        self.sim_dt = 0.01  # 提高控制频率至 100Hz 解决物理震荡和非线性响应
        # 实例化 HIL 电机模型 (24V 体系)
        self.motor_l = DCMotorSim(dt=self.sim_dt)
        self.motor_r = DCMotorSim(dt=self.sim_dt)
        
        # 模拟驱动器内部的 PID (Speed -> Voltage)
        self.voltage_error_sum = {'rl': 0.0, 'rr': 0.0}
        self.voltage_last_error = {'rl': 0.0, 'rr': 0.0}

        # PID state
        self.error_sum = {'rl': 0.0, 'rr': 0.0}
        self.last_error = {'rl': 0.0, 'rr': 0.0}
        self.error_roll_sum = 0.0

        self.start_time = time.time()
        
        self.create_timer(self.sim_dt, self.control_loop) # 100Hz
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
                
    def calculate_voltage_pid(self, target_w, current_w, wheel):
        # 模拟驱动器内部的速度闭环控制 (Input: Speed -> Output: Voltage)
        Kp = 8.0   # 提高比例增益，增强跟随和差速执行能力
        Ki = 1.0   # 积分增益，解决稳态静差
        Kd = 0.05  # 微分增益
        Kf = 0.5   # 前馈增益 (匹配修改后的反电动势常数 Ke = 0.5)
        
        # 为了防止转向时内外轮速度突变导致打滑，限制目标速度的变化率 (这在实体车上也是必要的)
        # 这里采用简单的误差限幅取代复杂的轨迹生成
        error = target_w - current_w
        # error = max(min(error, 10.0), -10.0) # 限制最大瞬间误差反应
        
        self.voltage_error_sum[wheel] += error * self.sim_dt
        
        # 积分限幅 (Anti-windup)
        # 允许积分项贡献大部分电压，但避免完全饱和
        self.voltage_error_sum[wheel] = max(min(self.voltage_error_sum[wheel], 12.0), -12.0)
        
        # 微分项计算
        # 注意: 实际使用中需对 error 进行滤波，这里直接差分
        d_error = (error - self.voltage_last_error[wheel]) / self.sim_dt
        self.voltage_last_error[wheel] = error

        # 计算电压
        voltage = (Kp * error) + (Ki * self.voltage_error_sum[wheel]) + (Kf * target_w)
        
        # 物理电压限制 (24V 电池)
        BATTERY_VOLTAGE = 12.0
        voltage = max(min(voltage, BATTERY_VOLTAGE), -BATTERY_VOLTAGE)
        
        return voltage

    def calculate_torque(self, target_angular_speed, current_angular_speed, wheel):
        # [Legacy] Used for compatibility with old interface, but logic is now voltage-based.
        # This function name is misleading for HIL mode, but we keep it for now.
        pass

    def control_loop(self):
        # 软启动：以 0.5 m/s^2 的加速度逐渐增加目标速度，避免起步"地板油"造成瞬间打滑
        self.target_speed_linear = min(self.desired_max_speed, self.target_speed_linear + 0.5 * self.sim_dt)
        self.target_speed_linear += 0.5 * self.sim_dt  # 加速到目标速度，增加前馈项帮助克服初始静摩擦
        self.error_roll_sum += self.roll_truth * self.sim_dt  # 累积 roll 误差用于积分控制
        target_w_l, target_w_r = stanley_control(self.front_x, self.front_y, self.yaw_truth, self.vel_linear, self.target_speed_linear, corner_center=4.0, radius=1.6, roll=self.roll_truth, sum_roll=self.error_roll_sum)
        # target_w = self.target_speed_linear / self.wheel_radius  # 直接使用线速度转换为角速度作为目标
        # print(target_w_l, target_w_r)
        # 1. 驱动器层：计算所需电压 (PID)
        # [紧急修正] 考虑到左后轮 (RL) 的物理安装可能反了：
        #   Case A: 编码器反了 -> 电机正转读数负 -> PID 正反馈 -> 震荡
        #   Case B: 电机线反了 -> 给正电压却反转 -> PID 认为是负速度 -> 给更大正电压 -> 锁死在负极限
        # 尝试方案：假设左轮是镜像安装，物理上需要反向控制，且读数也反向。
        # 我们在这里统一做符号翻转：
        # LEFT_SIGN = -1.0  (如果左轮原本就需要反转运行)
        
        # 你的反馈是：之前震荡(+50/-50)，改了力矩反向后锁死(-50)。这说明单纯改输出不够。
        # 极有可能是：左轮的正方向定义本身就是反的 (常见于差速车，左右轮对向安装)
        # 所以我们把 目标速度反过来，或者把 反馈速度反过来。
        
        voltage_l = self.calculate_voltage_pid(target_w_l, self.wheel_rl_speed, 'rl')
        voltage_r = self.calculate_voltage_pid(target_w_r, self.wheel_rr_speed, 'rr')
        
        # 2. 物理HIL层：计算电机真实力矩 (Motor Dynamics)
        # [恢复] 去掉之前的 -1.0 尝试，改用更彻底的逻辑
        torque_l_cmd = self.motor_l.step(voltage_l, self.wheel_rl_speed)
        torque_r_cmd = self.motor_r.step(voltage_r, self.wheel_rr_speed)

        # print(target_w_l, target_w_r)
        # print(torque_l_cmd, torque_r_cmd)
        # print(self.wheel_rl_speed, self.wheel_rr_speed)
        self.pub_rl.publish(Float64(data=torque_l_cmd))
        self.pub_rr.publish(Float64(data=torque_r_cmd))
        
    def print_status(self):
        
        target_w_l, target_w_r = stanley_control(self.front_x, self.front_y, self.yaw_truth, self.vel_linear, self.target_speed_linear, corner_center=4.0, radius=1.6, roll=self.roll_truth, sum_roll=self.error_roll_sum)
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
