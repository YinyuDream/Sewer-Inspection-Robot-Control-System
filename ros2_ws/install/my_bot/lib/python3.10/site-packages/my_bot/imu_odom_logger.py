import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class ImuOdomLogger(Node):
    def __init__(self):
        super().__init__('imu_odom_logger')
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_position = None
        self.current_orientation = None # From IMU

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_orientation = (roll, pitch, yaw)
        self.print_status()

    def print_status(self):
        if self.current_position and self.current_orientation:
            pos = self.current_position
            roll, pitch, yaw = self.current_orientation
            self.get_logger().info(
                f"\nPosition (Odom): x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}\n"
                f"Orientation (IMU): roll={math.degrees(roll):.2f}, pitch={math.degrees(pitch):.2f}, yaw={math.degrees(yaw):.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
