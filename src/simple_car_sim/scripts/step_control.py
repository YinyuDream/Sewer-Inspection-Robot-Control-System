#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import ControlWorld
import time
import math
import csv
import os

# ========================================== #
#            公用数学与辅助函数                #
# ========================================== #

class MathUtils:
    @staticmethod
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

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


# ========================================== #
#               核心控制算法类                 #
# ========================================== #

class VehicleDynamics:
    @staticmethod
    def geometric_calculation(x, y, yaw, vel, corner_center, radius):
        e = 0.0
        psi_path = 0.0
        d_path = corner_center + radius
        if x > corner_center and y > corner_center:
            xc, yc = corner_center, corner_center
            dist = math.sqrt((x - xc)**2 + (y - yc)**2)
            e = dist - radius
            angle_to_center = math.atan2(y - yc, x - xc)
            psi_path = angle_to_center + math.pi / 2
        elif x < -corner_center and y > corner_center:
            xc, yc = -corner_center, corner_center
            dist = math.sqrt((x - xc)**2 + (y - yc)**2)
            e = dist - radius
            angle_to_center = math.atan2(y - yc, x - xc)
            psi_path = angle_to_center + math.pi / 2
        elif x < -corner_center and y < -corner_center:
            xc, yc = -corner_center, -corner_center
            dist = math.sqrt((x - xc)**2 + (y - yc)**2)
            e = dist - radius
            angle_to_center = math.atan2(y - yc, x - xc)
            psi_path = angle_to_center + math.pi / 2
        elif x > corner_center and y < -corner_center:
            xc, yc = corner_center, -corner_center
            dist = math.sqrt((x - xc)**2 + (y - yc)**2)
            e = dist - radius
            angle_to_center = math.atan2(y - yc, x - xc)
            psi_path = angle_to_center + math.pi / 2
        elif y > corner_center:
            e = y - d_path
            psi_path = math.pi
        elif y < -corner_center:
            e = -d_path - y
            psi_path = 0.0
        elif x < -corner_center:
            e = -d_path - x
            psi_path = -math.pi / 2
        elif x > corner_center:
            e = x - d_path
            psi_path = math.pi / 2
        theta_e = MathUtils.normalize_angle(psi_path - yaw)
        return e, theta_e

    @staticmethod
    def calculate_wheel_speed(stanley_delta, vel_target, L=0.6, W=0.68):
        omega = vel_target * math.tan(stanley_delta) / L
        v_left = vel_target - (omega * W / 2)
        v_right = vel_target + (omega * W / 2)
        wheel_radius = 0.16
        w_left = v_left / wheel_radius
        w_right = v_right / wheel_radius
        return w_left, w_right

    @staticmethod
    def calculate_ackermann_angles(delta, L=0.6, W=0.68):
        if abs(delta) < 1e-6:
            return 0.0, 0.0
        tan_delta = math.tan(delta)
        delta_left = math.atan(2 * L * tan_delta / (2 * L - W * tan_delta))
        delta_right = math.atan(2 * L * tan_delta / (2 * L + W * tan_delta))
        return delta_left, delta_right

    @staticmethod
    def stanley_control(x, y, yaw, vel, vel_target, corner_center, radius, roll, sum_roll, roll_rate=0.0):
        k = 4.0
        k_soft = 2.0
        e_geo, theta_e = VehicleDynamics.geometric_calculation(x, y, yaw, vel, corner_center, radius)
        K_roll_p = 4.0
        K_roll_d = 1.5
        delta_geo = theta_e + math.atan2(k * e_geo, vel + k_soft)
        delta_roll = - K_roll_p * roll - K_roll_d * roll_rate - 0.5 * sum_roll
        delta = delta_geo + delta_roll
        delta = max(min(delta, math.radians(45)), math.radians(-45))
        v_left, v_right = VehicleDynamics.calculate_wheel_speed(delta, vel_target, L=0.6, W=0.68)
        return v_left, v_right, delta, e_geo, theta_e


# ========================================== #
#               电机仿真模型类                 #
# ========================================== #

class DCMotorSim:
    def __init__(self, R=0.5, L=0.01, Kt=1.0, Ke=1.0, b=0.1, J=0.02, dt=0.05):
        self.R = 0.5
        self.L = 0.01
        self.Kt = 0.5
        self.Ke = 0.5
        self.b = 0.1
        self.J = 0.02
        self.dt = dt
        self.current = 0.0

    def step(self, voltage_in, angular_vel):
        back_emf = self.Ke * angular_vel
        if self.R > 1e-4:
            tau = self.L / self.R
            i_steady = (voltage_in - back_emf) / self.R
            decay = math.exp(-self.dt / tau)
            self.current = self.current * decay + i_steady * (1.0 - decay)
        else:
            self.current = 0.0
        MAX_CURRENT = 12.0
        self.current = max(min(self.current, MAX_CURRENT), -MAX_CURRENT)
        torque_electromagnetic = self.Kt * self.current
        torque_output = torque_electromagnetic - (self.b * angular_vel)
        return torque_output


# ========================================== #
#               ROS 2 锁步控制节点             #
# ========================================== #

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        # 使用仿真时间
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # 锁步相关
        self.sim_dt = 0.01
        self.gazebo_step_size = 0.001
        self.steps_per_loop = max(1, int(self.sim_dt / self.gazebo_step_size))
        self.current_sim_time = 0.0

        # 订阅仿真时钟
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # 准备调用 Gazebo 控制服务
        world_name = 'pipe_world'
        self.control_client = self.create_client(ControlWorld, f'/world/{world_name}/control')
        while not self.control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'等待 Gazebo 控制服务 /world/{world_name}/control ...')
        self.get_logger().info('✅ 成功连接至 Gazebo 控制服务，准备开始锁步！')

        # 外部命令标志（用户要求）
        self.new_ext_cmd_received = 0

        # 初始化其他功能
        self._init_publishers_subscribers()
        self._init_status_variables()
        self._init_simulation_config()
        self._init_logging()

    def clock_callback(self, msg):
        # builtin_interfaces/Time fields
        try:
            self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        except Exception:
            try:
                self.current_sim_time = msg.sec + msg.nanosec * 1e-9
            except Exception:
                pass

    def step_and_wait(self):
        target_time = self.current_sim_time + self.sim_dt
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.multi_step = self.steps_per_loop
        future = self.control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None or not getattr(future.result(), 'success', False):
            self.get_logger().error('调用 Gazebo 步进服务失败！')
            return

        timeout = time.time() + 2.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            if self.current_sim_time >= target_time - 1e-5:
                break
            if time.time() > timeout:
                self.get_logger().warn('等待传感器时间同步超时！')
                break

    # ---------------- 初始化与回调 ---------------- #
    def _init_publishers_subscribers(self):
        self.pub_rl = self.create_publisher(Float64, '/cmd_force_rl', 10)
        self.pub_rr = self.create_publisher(Float64, '/cmd_force_rr', 10)
        self.pub_steer_fl = self.create_publisher(Float64, '/cmd_pos_fl', 10)
        self.pub_steer_fr = self.create_publisher(Float64, '/cmd_pos_fr', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom_to_firmware', 10)
        self.pub_jointstate = self.create_publisher(JointState, '/joint_states_to_firmware', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu_to_firmware', 10)

        # 外部控制
        self.create_subscription(Float64, '/cmd_volts_l', self.cb_volts_l, 10)
        self.create_subscription(Float64, '/cmd_volts_r', self.cb_volts_r, 10)
        # 以左前转向作为新指令触发源
        self.create_subscription(Float64, '/cmd_steer_l', self.cb_steer_l, 10)
        self.create_subscription(Float64, '/cmd_steer_r', self.cb_steer_r, 10)

        self.use_external_pwm = True
        self.ext_volts_l = 0.0
        self.ext_volts_r = 0.0
        self.ext_steer_l = 0.0
        self.ext_steer_r = 0.0

    def cb_volts_l(self, msg):
        self.ext_volts_l = msg.data
        self.new_ext_cmd_received += 1

    def cb_volts_r(self, msg):
        self.ext_volts_r = msg.data
        self.new_ext_cmd_received += 1

    def cb_steer_l(self, msg):
        self.ext_steer_l = msg.data
        self.new_ext_cmd_received += 1

    def cb_steer_r(self, msg):
        self.ext_steer_r = msg.data
        self.new_ext_cmd_received += 1

    def _init_status_variables(self):
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
        self.imu_roll_rate = 0.0
        self.imu_yaw_rate = 0.0
        self.wheel_rl_speed = 0.0
        self.wheel_rr_speed = 0.0
        self.suspension_fl_joint = 0.0
        self.suspension_fr_joint = 0.0
        self.suspension_rl_joint = 0.0
        self.suspension_rr_joint = 0.0
        self.voltage_error_sum = {'rl': 0.0, 'rr': 0.0}
        self.voltage_last_error = {'rl': 0.0, 'rr': 0.0}
        self.error_roll_sum = 0.0
        self.current_delta = 0.0
        self.filtered_delta = 0.0
        self.delta_slew_rate = 10.0

    def _init_simulation_config(self):
        self.target_speed_linear = 0.0
        self.desired_max_speed = 1.0
        self.wheel_radius = 0.16
        self.car_length = 0.3
        self.motor_l = DCMotorSim(dt=self.sim_dt)
        self.motor_r = DCMotorSim(dt=self.sim_dt)
        self.last_loop_time = None
        self.get_logger().info('Node started. Waiting for simulation clock...')

    def _init_logging(self):
        self.start_time = None
        self.log_file_path = '/home/yinyudream/robot/simulation/control/simulation_data.csv'
        os.makedirs(os.path.dirname(self.log_file_path), exist_ok=True)
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.log_counter = 0
        self.csv_writer.writerow([
            'time', 'x', 'y', 'yaw', 'velocity', 'target_velocity',
            'roll', 'roll_rate', 'steering_delta', 'lateral_error',
            'heading_error', 'wheel_speed_l', 'wheel_speed_r'
        ])

    # 传感器回调
    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
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
        self.pub_odom.publish(msg)

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = MathUtils.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.imu_yaw = yaw
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_angular_vel_z = msg.angular_velocity.z
        self.imu_roll_rate = msg.angular_velocity.x
        self.imu_yaw_rate = msg.angular_velocity.z
        self.pub_imu.publish(msg)

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
        self.pub_jointstate.publish(msg)

    # 速度 PID
    def calculate_voltage_pid(self, target_w, current_w, wheel):
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
        BATTERY_VOLTAGE = 12.0
        voltage = max(min(voltage, BATTERY_VOLTAGE), -BATTERY_VOLTAGE)
        return voltage

    # 执行单步控制
    def execute_control_step(self):
        dt = self.sim_dt
        SAFE_CORNER_SPEED_FACTOR = 2.0
        expected_speed = self.desired_max_speed / (1.0 + SAFE_CORNER_SPEED_FACTOR * abs(self.current_delta))
        expected_speed = max(expected_speed, 0.4)
        if self.target_speed_linear < expected_speed:
             self.target_speed_linear += 0.5 * dt
        else:
             self.target_speed_linear -= 1.0 * dt
        self.target_speed_linear = max(min(self.target_speed_linear, 1.2), 0.0)

        self.error_roll_sum += self.roll_truth * dt
        target_w_l, target_w_r, raw_target_delta, e_geo, theta_e = VehicleDynamics.stanley_control(
            self.front_x, self.front_y, self.yaw_truth, self.vel_linear,
            self.target_speed_linear, corner_center=8.0, radius=1.6,
            roll=self.roll_truth, sum_roll=self.error_roll_sum, roll_rate=self.imu_roll_rate
        )

        filter_alpha = 0.7
        self.filtered_delta = self.filtered_delta * (1.0 - filter_alpha) + raw_target_delta * filter_alpha
        max_delta_change = self.delta_slew_rate * dt
        delta_change = self.filtered_delta - self.current_delta
        delta_change = max(min(delta_change, max_delta_change), -max_delta_change)
        self.current_delta += delta_change
        delta = self.current_delta

        self._log_data(delta, e_geo, theta_e)

        if self.use_external_pwm:
            delta_l, delta_r = self.ext_steer_l, self.ext_steer_r
            voltage_l, voltage_r = self.ext_volts_l, self.ext_volts_r
        else:
            delta_l, delta_r = VehicleDynamics.calculate_ackermann_angles(delta, L=0.6, W=0.68)
            voltage_l = self.calculate_voltage_pid(target_w_l, self.wheel_rl_speed, 'rl')
            voltage_r = self.calculate_voltage_pid(target_w_r, self.wheel_rr_speed, 'rr')

        self.pub_steer_fl.publish(Float64(data=delta_l))
        self.pub_steer_fr.publish(Float64(data=delta_r))

        torque_l_cmd = self.motor_l.step(voltage_l, self.wheel_rl_speed)
        torque_r_cmd = self.motor_r.step(voltage_r, self.wheel_rr_speed)

        self.pub_rl.publish(Float64(data=torque_l_cmd))
        self.pub_rr.publish(Float64(data=torque_r_cmd))

    def _log_data(self, delta, e_geo, theta_e):
        now = self.current_sim_time
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
            if self.log_counter % 50 == 0:
                self.log_file.flush()
        except ValueError:
            pass

    def print_status(self):
        if self.start_time is None:
             return
        target_w_l, target_w_r, delta, e_geo, theta_e = VehicleDynamics.stanley_control(
            self.front_x, self.front_y, self.yaw_truth, self.vel_linear,
            self.target_speed_linear, corner_center=8.0, radius=1.6,
            roll=self.roll_truth, sum_roll=self.error_roll_sum, roll_rate=self.imu_roll_rate
        )
        self.get_logger().info(f'pos=({self.pos_x:.2f},{self.pos_y:.2f}) vel={self.vel_linear:.2f} delta={math.degrees(delta):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    try:
        node.get_logger().info('正在初始化第一步...')
        node.step_and_wait()
        loop_counter = 0
        while rclpy.ok():
            if node.use_external_pwm:
                while node.new_ext_cmd_received < 4 and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.005)
                node.new_ext_cmd_received = 0
            #time.sleep(0.1)
            node.execute_control_step()
            node.step_and_wait()

    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在退出...')
    finally:
        node.pub_rl.publish(Float64(data=0.0))
        node.pub_rr.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
