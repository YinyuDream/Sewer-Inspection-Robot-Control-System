#!/usr/bin/python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

msg = """
Control Your Car!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease linear velocity (both wheels)
a/d : increase/decrease differential velocity (turn left/right)
space: force stop
q   : quit

Current speeds:
Left:  {left:.2f}
Right: {right:.2f}
"""

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Publishers for 4 wheels
        self.pub_fl = self.create_publisher(Float64, '/simple_car/front_left_wheel_vel', 10)
        self.pub_fr = self.create_publisher(Float64, '/simple_car/front_right_wheel_vel', 10)
        self.pub_rl = self.create_publisher(Float64, '/simple_car/rear_left_wheel_vel', 10)
        self.pub_rr = self.create_publisher(Float64, '/simple_car/rear_right_wheel_vel', 10)
        
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.step = 1.0
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
        msg_l = Float64()
        msg_l.data = self.left_vel
        msg_r = Float64()
        msg_r.data = self.right_vel
        
        # Left wheels
        self.pub_fl.publish(msg_l)
        self.pub_rl.publish(msg_l)
        
        # Right wheels
        self.pub_fr.publish(msg_r)
        self.pub_rr.publish(msg_r)

    def run(self):
        try:
            print(msg.format(left=self.left_vel, right=self.right_vel))
            while True:
                key = self.getKey()
                if key == 'w':
                    self.left_vel += self.step
                    self.right_vel += self.step
                elif key == 's':
                    self.left_vel -= self.step
                    self.right_vel -= self.step
                elif key == 'a':
                    self.left_vel -= self.step
                    self.right_vel += self.step
                elif key == 'd':
                    self.left_vel += self.step
                    self.right_vel -= self.step
                elif key == ' ':
                    self.left_vel = 0.0
                    self.right_vel = 0.0
                elif key == 'q':
                    break
                elif key == '\x03': # Ctrl-C
                    break
                
                if key in ['w', 's', 'a', 'd', ' ']:
                    print(msg.format(left=self.left_vel, right=self.right_vel))
                    self.publish_speeds()
                    
        except Exception as e:
            print(e)

        finally:
            msg_stop = Float64()
            msg_stop.data = 0.0
            self.pub_fl.publish(msg_stop)
            self.pub_fr.publish(msg_stop)
            self.pub_rl.publish(msg_stop)
            self.pub_rr.publish(msg_stop)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
