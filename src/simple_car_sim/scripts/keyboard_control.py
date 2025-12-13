#!/usr/bin/python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

msg = """
Control Your Pipeline Robot!
---------------------------
Moving around:
        w
   a    s    d

Arm Control:
        i
   j    k    l

Expansion Control:
   u : Expand All (Stick to wall)
   o : Contract All (Release)

w/s : increase/decrease linear velocity (all wheels)
a/d : increase/decrease rotational velocity (left/right differential)
i/k : pitch up/down (arm)
j/l : yaw left/right (arm)
u/o : expand/contract legs

space: force stop
q   : quit

Current speeds:
Linear:  {linear:.2f}
Angular: {angular:.2f}
Pitch:   {pitch:.2f}
Yaw:     {yaw:.2f}
Expansion: {expansion:.2f}
"""

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Publishers for 4 wheels
        self.pub_top = self.create_publisher(Float64, '/simple_car/top_wheel_vel', 10)
        self.pub_bottom = self.create_publisher(Float64, '/simple_car/bottom_wheel_vel', 10)
        self.pub_left = self.create_publisher(Float64, '/simple_car/left_wheel_vel', 10)
        self.pub_right = self.create_publisher(Float64, '/simple_car/right_wheel_vel', 10)
        
        # Publishers for Arm
        self.pub_pitch = self.create_publisher(Float64, '/simple_car/joint_pitch_vel', 10)
        self.pub_yaw = self.create_publisher(Float64, '/simple_car/joint_yaw_vel', 10)
        
        # Publishers for Expansion (Drive Section)
        self.pub_drive_exp_top = self.create_publisher(Float64, '/simple_car/drive_exp_top_vel', 10)
        self.pub_drive_exp_bottom = self.create_publisher(Float64, '/simple_car/drive_exp_bottom_vel', 10)
        self.pub_drive_exp_left = self.create_publisher(Float64, '/simple_car/drive_exp_left_vel', 10)
        self.pub_drive_exp_right = self.create_publisher(Float64, '/simple_car/drive_exp_right_vel', 10)
        
        # Publishers for Expansion (Functional Section)
        self.pub_func_exp_top = self.create_publisher(Float64, '/simple_car/func_exp_top_vel', 10)
        self.pub_func_exp_bottom = self.create_publisher(Float64, '/simple_car/func_exp_bottom_vel', 10)
        self.pub_func_exp_left = self.create_publisher(Float64, '/simple_car/func_exp_left_vel', 10)
        self.pub_func_exp_right = self.create_publisher(Float64, '/simple_car/func_exp_right_vel', 10)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0
        self.expansion_vel = 0.0
        
        self.step = 1.0
        self.arm_step = 0.5
        self.exp_step = 0.5 # Increased expansion speed
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_speeds(self):
        # Left = linear - angular
        # Right = linear + angular
        # Top = linear
        # Bottom = linear
        
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
        self.pub_drive_exp_top.publish(msg_exp)
        self.pub_drive_exp_bottom.publish(msg_exp)
        self.pub_drive_exp_left.publish(msg_exp)
        self.pub_drive_exp_right.publish(msg_exp)
        
        self.pub_func_exp_top.publish(msg_exp)
        self.pub_func_exp_bottom.publish(msg_exp)
        self.pub_func_exp_left.publish(msg_exp)
        self.pub_func_exp_right.publish(msg_exp)

    def run(self):
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
                    self.expansion_vel = self.exp_step # Expand
                elif key == 'o':
                    self.expansion_vel = -self.exp_step # Contract
                elif key == ' ':
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    self.pitch_vel = 0.0
                    self.yaw_vel = 0.0
                    self.expansion_vel = 0.0
                elif key == 'q':
                    break
                elif key == '\x03': # Ctrl-C
                    break
                
                if key in ['w', 's', 'a', 'd', 'i', 'k', 'j', 'l', 'u', 'o', ' ']:
                    print(msg.format(linear=self.linear_vel, angular=self.angular_vel, pitch=self.pitch_vel, yaw=self.yaw_vel, expansion=self.expansion_vel))
                    self.publish_speeds()
                    
        except Exception as e:
            print(e)

        finally:
            msg_stop = Float64()
            msg_stop.data = 0.0
            self.pub_left.publish(msg_stop)
            self.pub_right.publish(msg_stop)
            self.pub_top.publish(msg_stop)
            self.pub_bottom.publish(msg_stop)
            self.pub_pitch.publish(msg_stop)
            self.pub_yaw.publish(msg_stop)
            
            # Stop expansion
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
