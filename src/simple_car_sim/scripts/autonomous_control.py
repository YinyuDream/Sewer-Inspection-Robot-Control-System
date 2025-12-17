#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import math

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')
        
        # Publishers for Drive Wheels (Front and Rear for each leg)
        self.pub_top_front = self.create_publisher(Float64, '/simple_car/top_front_wheel_vel', 10)
        self.pub_top_rear = self.create_publisher(Float64, '/simple_car/top_rear_wheel_vel', 10)
        self.pub_bottom_front = self.create_publisher(Float64, '/simple_car/bottom_front_wheel_vel', 10)
        self.pub_bottom_rear = self.create_publisher(Float64, '/simple_car/bottom_rear_wheel_vel', 10)
        self.pub_left_front = self.create_publisher(Float64, '/simple_car/left_front_wheel_vel', 10)
        self.pub_left_rear = self.create_publisher(Float64, '/simple_car/left_rear_wheel_vel', 10)
        self.pub_right_front = self.create_publisher(Float64, '/simple_car/right_front_wheel_vel', 10)
        self.pub_right_rear = self.create_publisher(Float64, '/simple_car/right_rear_wheel_vel', 10)
        
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
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State
        self.current_speed = 0.0
        self.target_speed = 0.25 # m/s
        
        # Pipe Radius is 0.8m.
        # Robot Radius (collapsed) = 0.27 (body) + 0.1 (bogie) + 0.18 (wheel) = 0.55m.
        # Gap to fill = 0.8 - 0.55 = 0.25m.
        # We add some compression for grip.
        self.target_expansion_drive = 0.20 # m (gentle compression)
        self.target_expansion_func = 0.18 # m (gentle compression)
        self.expansion_ramp_time = 3.0
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.print_status)
        
        self.start_time = time.time()
        self.state = "INIT" # INIT, EXPANDING, RUNNING
        self.expand_start_time = None

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        
        # Calculate linear velocity magnitude
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.vel_linear = math.sqrt(vx*vx + vy*vy + vz*vz)
        
        # Calculate angular velocity magnitude
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z
        self.vel_angular = math.sqrt(wx*wx + wy*wy + wz*wz)

    def joint_state_callback(self, msg):
        pass # Can be used to monitor expansion force/position

    def publish_expansion(self, drive_val, func_val):
        msg_drive = Float64()
        msg_drive.data = drive_val
        msg_func = Float64()
        msg_func.data = func_val
        
        self.pub_drive_exp_top.publish(msg_drive)
        self.pub_drive_exp_bottom.publish(msg_drive)
        self.pub_drive_exp_left.publish(msg_drive)
        self.pub_drive_exp_right.publish(msg_drive)
        
        self.pub_func_exp_top.publish(msg_func)
        self.pub_func_exp_bottom.publish(msg_func)
        self.pub_func_exp_left.publish(msg_func)
        self.pub_func_exp_right.publish(msg_func)

    def publish_drive(self, speed):
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
        elapsed = time.time() - self.start_time
        
        if self.state == "INIT":
            if elapsed > 2.0:
                self.state = "EXPANDING"
                self.get_logger().info("State: EXPANDING")
                self.expand_start_time = time.time()
                
        elif self.state == "EXPANDING":
            ramp_elapsed = time.time() - self.expand_start_time if self.expand_start_time else 0.0
            ramp_ratio = min(ramp_elapsed / self.expansion_ramp_time, 1.0)
            drive_cmd = ramp_ratio * self.target_expansion_drive
            func_cmd = ramp_ratio * self.target_expansion_func
            self.publish_expansion(drive_cmd, func_cmd)
            if ramp_ratio >= 1.0:
                self.state = "RUNNING"
                self.get_logger().info("State: RUNNING")
                
        elif self.state == "RUNNING":
            self.publish_expansion(self.target_expansion_drive, self.target_expansion_func) # Keep pressure
            self.publish_drive(self.target_speed)

    def print_status(self):
        print(f"\033[2J\033[H") # Clear screen
        print("="*40)
        print(f"Pipeline Robot Autonomous Monitor")
        print("="*40)
        print(f"State:           {self.state}")
        print(f"Time:            {time.time() - self.start_time:.1f} s")
        print("-" * 20)
        print(f"Position (X,Y,Z): ({self.pos_x:.2f}, {self.pos_y:.2f}, {self.pos_z:.2f})")
        print(f"Linear Velocity:  {self.vel_linear:.2f} m/s")
        print(f"Angular Velocity: {self.vel_angular:.2f} rad/s")
        print("-" * 20)
        print(f"Target Exp (Drive): {self.target_expansion_drive:.2f} m")
        print(f"Target Exp (Func):  {self.target_expansion_func:.2f} m")
        print(f"Target Speed:       {self.target_speed:.2f} m/s")
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
        node.publish_drive(0.0)
        node.publish_expansion(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
