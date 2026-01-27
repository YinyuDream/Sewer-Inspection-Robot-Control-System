#!/usr/bin/python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ==========================================================
#                  终端交互提示信息
# ==========================================================
msg = """
Control Your Pipeline Robot!
控制你的管道机器人！
---------------------------
Moving around (移动控制 - 差速驱动):
        w
   a    s    d

Arm Control (机械臂关节控制):
        i
   j    k    l

Expansion Control (伸缩机构同步控制):
   u : Expand All (Stick to wall - 全体撑墙)
   o : Contract All (Release - 全体收缩)

功能键说明:
w/s : 增加/减小线速度目标值 (驱动所有轮子前进/后退)
a/d : 增加/减小角速度目标值 (左侧轮子后退，右侧轮子前进，实现原地转向)
i/k : 机械臂 Pitch (俯仰) 向上/向下
j/l : 机械臂 Yaw (偏航) 向左/向右
u/o : 增加/减小 伸缩腿的目标长度

space: force stop - 空格键急停 (速度置零)
q   : quit - 退出程序

Current settings (当前设定):
Linear Vel:  {linear:.2f}
Angular Vel: {angular:.2f}
Pitch Vel:   {pitch:.2f}
Yaw Vel:     {yaw:.2f}
Expansion Pos: {expansion:.2f}
"""

def getKey(settings):
    """
    读取终端的一个按键输入，不阻塞，不需要回车
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardController(Node):
    """
    键盘控制节点
    功能：
    1. 监听键盘输入。
    2. 维护各轴的目标速度/位置变量。
    3. 将控制指令发布到具体的 ROS 话题上 (映射到 Gazebo 关节控制器)。
    """
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # ==========================================
        # 1. 驱动轮发布器 (Drive Wheels)
        # ==========================================
        # 机器人有8个驱动轮，分为上下左右4组，每组前后2个。
        # 为了配合 launch 文件的 bridge 映射，我们需要分别向每个关节话题发布指令。
        
        # Top (上)
        self.pub_top_f = self.create_publisher(Float64, '/simple_car/top_front_wheel_vel', 10)
        self.pub_top_r = self.create_publisher(Float64, '/simple_car/top_rear_wheel_vel', 10)
        
        # Bottom (下)
        self.pub_bottom_f = self.create_publisher(Float64, '/simple_car/bottom_front_wheel_vel', 10)
        self.pub_bottom_r = self.create_publisher(Float64, '/simple_car/bottom_rear_wheel_vel', 10)
        
        # Left (左)
        self.pub_left_f = self.create_publisher(Float64, '/simple_car/left_front_wheel_vel', 10)
        self.pub_left_r = self.create_publisher(Float64, '/simple_car/left_rear_wheel_vel', 10)
        
        # Right (右)
        self.pub_right_f = self.create_publisher(Float64, '/simple_car/right_front_wheel_vel', 10)
        self.pub_right_r = self.create_publisher(Float64, '/simple_car/right_rear_wheel_vel', 10)
        
        # ==========================================
        # 2. 机械臂关节发布器 (Arm Joints)
        # ==========================================
        self.pub_pitch = self.create_publisher(Float64, '/simple_car/joint_pitch_vel', 10)
        self.pub_yaw = self.create_publisher(Float64, '/simple_car/joint_yaw_vel', 10)
        
        # ==========================================
        # 3. 伸缩机构发布器 (Expansion Joints)
        # ==========================================
        # 驱动节伸缩 (Drive Section)
        self.pub_drive_exp_top = self.create_publisher(Float64, '/simple_car/drive_exp_top_vel', 10)
        self.pub_drive_exp_bottom = self.create_publisher(Float64, '/simple_car/drive_exp_bottom_vel', 10)
        self.pub_drive_exp_left = self.create_publisher(Float64, '/simple_car/drive_exp_left_vel', 10)
        self.pub_drive_exp_right = self.create_publisher(Float64, '/simple_car/drive_exp_right_vel', 10)
        
        # 功能节伸缩 (Functional Section)
        self.pub_func_exp_top = self.create_publisher(Float64, '/simple_car/func_exp_top_vel', 10)
        self.pub_func_exp_bottom = self.create_publisher(Float64, '/simple_car/func_exp_bottom_vel', 10)
        self.pub_func_exp_left = self.create_publisher(Float64, '/simple_car/func_exp_left_vel', 10)
        self.pub_func_exp_right = self.create_publisher(Float64, '/simple_car/func_exp_right_vel', 10)

        # 状态变量初始化
        self.target_linear = 0.0   # 目标线速度
        self.target_angular = 0.0  # 目标角速度
        self.target_pitch = 0.0    # 机械臂 Pitch 速度
        self.target_yaw = 0.0      # 机械臂 Yaw 速度
        self.target_expansion = 0.0 # 伸缩位置 (这里简化为 velocity 控制或者位置步进控制，取决于 Gazebo 插件类型)
        # 注意: 如果 Gazebo 插件是 position 控制，这里需要发布位置；如果是 velocity 控制，这里发布速度。
        # 根据 URDF，伸缩关节使用的是 JointController，p_gain=150，这通常意味着它是位置控制器 (Position PID)。
        # 所以我们需要维护一个当前位置，并增加/减少它。
        self.current_expansion_pos = 0.2 # 初始伸缩长度
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz 循环

    def timer_callback(self):
        # 1. 读取按键
        key = getKey(self.settings)
        
        # 2. 处理按键逻辑
        if key == 'w':
            self.target_linear += 0.5
        elif key == 's':
            self.target_linear -= 0.5
        elif key == 'a':
            self.target_angular += 0.5
        elif key == 'd':
            self.target_angular -= 0.5
        elif key == 'i':
            self.target_pitch += 0.1
        elif key == 'k':
            self.target_pitch -= 0.1
        elif key == 'j':
            self.target_yaw += 0.1
        elif key == 'l':
            self.target_yaw -= 0.1
        elif key == 'u':
            self.current_expansion_pos += 0.05
            if self.current_expansion_pos > 0.8: self.current_expansion_pos = 0.8
        elif key == 'o':
            self.current_expansion_pos -= 0.05
            if self.current_expansion_pos < 0.0: self.current_expansion_pos = 0.0
        elif key == ' ':
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.target_pitch = 0.0
            self.target_yaw = 0.0
        elif key == '\x03' or key == 'q': # Ctrl+C 或 q
            rclpy.shutdown()
            sys.exit()

        # 打印状态
        if key != '':
            print(msg.format(
                linear=self.target_linear,
                angular=self.target_angular,
                pitch=self.target_pitch,
                yaw=self.target_yaw,
                expansion=self.current_expansion_pos
            ))

        # 3. 计算各轮速度 (差速模型)
        # 左侧轮速度 = 线速度 - 角速度
        # 右侧轮速度 = 线速度 + 角速度
        # (这里假设简单的差速转向模型)
        vel_left = self.target_linear - self.target_angular
        vel_right = self.target_linear + self.target_angular
        # 上下轮子如果也参与转向，可以类似处理，或者仅用于前进
        vel_top = self.target_linear
        vel_bottom = self.target_linear

        # 4. 发布命令
        
        # 封装发布函数
        def pub_float(publisher, val):
            m = Float64()
            m.data = float(val)
            publisher.publish(m)

        # 发布轮子速度 (前后轮同速)
        pub_float(self.pub_top_f, vel_top)
        pub_float(self.pub_top_r, vel_top)
        pub_float(self.pub_bottom_f, vel_bottom)
        pub_float(self.pub_bottom_r, vel_bottom)
        
        pub_float(self.pub_left_f, vel_left)
        pub_float(self.pub_left_r, vel_left)
        pub_float(self.pub_right_f, vel_right)
        pub_float(self.pub_right_r, vel_right)

        # 发布机械臂关节速度
        pub_float(self.pub_pitch, self.target_pitch)
        pub_float(self.pub_yaw, self.target_yaw)

        # 发布伸缩控制 (位置控制)
        # 所有 8 条腿同步伸缩
        pub_float(self.pub_drive_exp_top, self.current_expansion_pos)
        pub_float(self.pub_drive_exp_bottom, self.current_expansion_pos)
        pub_float(self.pub_drive_exp_left, self.current_expansion_pos)
        pub_float(self.pub_drive_exp_right, self.current_expansion_pos)
        
        pub_float(self.pub_func_exp_top, self.current_expansion_pos)
        pub_float(self.pub_func_exp_bottom, self.current_expansion_pos)
        pub_float(self.pub_func_exp_left, self.current_expansion_pos)
        pub_float(self.pub_func_exp_right, self.current_expansion_pos)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()
        self.pub_func_exp_right = self.create_publisher(Float64, '/simple_car/func_exp_right_vel', 10)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0
        self.expansion_vel = 0.0
        
        self.step = 1.0       # 速度调节步长
        self.arm_step = 0.5   # 机械臂速度步长
        self.exp_step = 0.5   # 伸缩速度步长 (Increased expansion speed)
        self.settings = termios.tcgetattr(sys.stdin) # 保存终端设置以便恢复

    def getKey(self):
        """
        读取单个按键输入，不回显，用于实时控制
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_speeds(self):
        """
        根据当前状态发布所有控制指令
        实现差速控制逻辑
        """
        # 差速转向逻辑：
        # 左侧轮速度 = 线速度 - 角速度
        # 右侧轮速度 = 线速度 + 角速度
        # 上下轮速度 = 线速度 (不参与转向，或者说作为从动轮/辅助轮)
        
        msg_l = Float64()
        msg_l.data = self.linear_vel - self.angular_vel
        
        msg_r = Float64()
        msg_r.data = self.linear_vel + self.angular_vel
        
        msg_t = Float64()
        msg_t.data = self.linear_vel
        
        msg_b = Float64()
        msg_b.data = self.linear_vel
        
        msg_pitch = Float64()
        msg_pitch.data = self.pitch_vel
        
        msg_yaw = Float64()
        msg_yaw.data = self.yaw_vel
        
        msg_exp = Float64()
        msg_exp.data = self.expansion_vel
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)
        self.pub_top.publish(msg_t)
        self.pub_bottom.publish(msg_b)
        self.pub_pitch.publish(msg_pitch)
        self.pub_yaw.publish(msg_yaw)
        
        # Publish expansion velocity to all 8 expansion joints
        # 同时向所有8个伸缩关节发送指令
        self.pub_drive_exp_top.publish(msg_exp)
        self.pub_drive_exp_bottom.publish(msg_exp)
        self.pub_drive_exp_left.publish(msg_exp)
        self.pub_drive_exp_right.publish(msg_exp)
        
        self.pub_func_exp_top.publish(msg_exp)
        self.pub_func_exp_bottom.publish(msg_exp)
        self.pub_func_exp_left.publish(msg_exp)
        self.pub_func_exp_right.publish(msg_exp)

    def run(self):
        """
        主运行循环
        """
        try:
            print(msg.format(linear=self.linear_vel, angular=self.angular_vel, pitch=self.pitch_vel, yaw=self.yaw_vel, expansion=self.expansion_vel))
            while True:
                key = self.getKey()
                if key == 'w':
                    self.linear_vel += self.step
                elif key == 's':
                    self.linear_vel -= self.step
                elif key == 'a':
                    self.angular_vel += self.step
                elif key == 'd':
                    self.angular_vel -= self.step
                elif key == 'i':
                    self.pitch_vel += self.arm_step
                elif key == 'k':
                    self.pitch_vel -= self.arm_step
                elif key == 'j':
                    self.yaw_vel += self.arm_step
                elif key == 'l':
                    self.yaw_vel -= self.arm_step
                elif key == 'u':
                    self.expansion_vel = self.exp_step # Expand (伸展)
                elif key == 'o':
                    self.expansion_vel = -self.exp_step # Contract (收缩)
                elif key == ' ':
                    # 重置所有速度
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    self.pitch_vel = 0.0
                    self.yaw_vel = 0.0
                    self.expansion_vel = 0.0
                elif key == 'q':
                    break
                elif key == '\x03': # Ctrl-C
                    break
                
                # 如果按下了有效键，刷新显示并发布命令
                if key in ['w', 's', 'a', 'd', 'i', 'k', 'j', 'l', 'u', 'o', ' ']:
                    print(msg.format(linear=self.linear_vel, angular=self.angular_vel, pitch=self.pitch_vel, yaw=self.yaw_vel, expansion=self.expansion_vel))
                    self.publish_speeds()
                    
        except Exception as e:
            print(e) # 打印错误信息

        finally:
            # 退出前发送停止指令
            msg_stop = Float64()
            msg_stop.data = 0.0
            self.pub_left.publish(msg_stop)
            self.pub_right.publish(msg_stop)
            self.pub_top.publish(msg_stop)
            self.pub_bottom.publish(msg_stop)
            self.pub_pitch.publish(msg_stop)
            self.pub_yaw.publish(msg_stop)
            
            # Stop expansion (停止伸缩)
            self.pub_drive_exp_top.publish(msg_stop)
            self.pub_drive_exp_bottom.publish(msg_stop)
            self.pub_drive_exp_left.publish(msg_stop)
            self.pub_drive_exp_right.publish(msg_stop)
            self.pub_func_exp_top.publish(msg_stop)
            self.pub_func_exp_bottom.publish(msg_stop)
            self.pub_func_exp_left.publish(msg_stop)
            self.pub_func_exp_right.publish(msg_stop)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
