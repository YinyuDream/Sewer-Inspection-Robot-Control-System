import sys
import termios
import time
import tty
from select import select

import rclpy
from geometry_msgs.msg import Twist

# Mapping arrow/WASD keys to twist commands (v: forward velocity, w: angular velocity)
KEY_BINDINGS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.5),
    "d": (0.0, -1.5),
    "W": (1.0, 0.0),
    "S": (-1.0, 0.0),
    "A": (0.0, 1.5),
    "D": (0.0, -1.5),
    "\u001b[A": (1.0, 0.0),
    "\u001b[B": (-1.0, 0.0),
    "\u001b[C": (0.0, -1.5),
    "\u001b[D": (0.0, 1.5),
}

STOP_BINDINGS = {" ", "x", "X", "q", "Q"}

HELP = """\nControl your four wheel robot!\n-----------------------------\nMove:        w/s or arrow up/down\nTurn:        a/d or arrow left/right\nStop:        space\nExit:        q\n\nPublishing to /cmd_vel\n"""


class KeyboardTeleop:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("keyboard_teleop")
        self.publisher = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.speed = self.node.declare_parameter("linear_speed", 0.6).value
        self.turn = self.node.declare_parameter("angular_speed", 1.2).value
    self.loop_dt = 0.05
        self.settings = termios.tcgetattr(sys.stdin)
        print(HELP)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        ready, _, _ = select([sys.stdin], [], [], 0.1)
        key = ""
        if ready:
            key = sys.stdin.read(1)
            if key == "\u001b":
                key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    twist.linear.x = lin * self.speed
                    twist.angular.z = ang * self.turn
                elif key in STOP_BINDINGS:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if key.lower() == "q":
                        self.publisher.publish(twist)
                        break
                elif key:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher.publish(twist)
                time.sleep(self.loop_dt)
        finally:
            self.restore_terminal()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            rclpy.shutdown()


def main():
    teleop = KeyboardTeleop()
    teleop.run()


if __name__ == "__main__":
    main()
