import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math
import time

from .serial_handler import SerialHandler
from .encoder_parser import parse_encoder_data


class PicoBridge(Node):
    def __init__(self):
        super().__init__('pico_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value

        # Serial handler (auto reconnects if unplugged)
        self.serial = SerialHandler(
            port=self.port,
            baud=self.baud,
            timeout=0.1,
            reconnect_delay=2.0,
            logger=self.get_logger()
        )

        # ROS topics
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer for serial read (20 Hz)
        self.timer = self.create_timer(0.05, self.read_serial)

        # Robot parameters
        self.wheel_radius = 0.03  # meters
        self.ticks_per_rev = 390  # adjust to your encoder spec
        self.wheel_separation = 0.16  # distance between wheels (m)

        # State variables
        self.last_time = self.get_clock().now()
        self.prev_left = 0
        self.prev_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info("✅ Pico Serial Bridge started")

    def cmd_vel_callback(self, msg):
        """Send linear & angular velocity to Pico."""
        linear = msg.linear.x
        angular = msg.angular.z
        command = f"VEL:{linear:.3f}:{angular:.3f}\n"
        self.serial.write(command)
        self.get_logger().debug(f"Sent to Pico: {command.strip()}")

    def read_serial(self):
        """Read encoder feedback and publish Odometry."""
        line = self.serial.read_line()
        if not line.startswith("ENC:"):
            return

        left_ticks, right_ticks = parse_encoder_data(line)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # Δ ticks
        delta_left = left_ticks - self.prev_left
        delta_right = right_ticks - self.prev_right
        self.prev_left = left_ticks
        self.prev_right = right_ticks

        # Convert ticks → meters
        ticks_to_m = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        left_dist = delta_left * ticks_to_m
        right_dist = delta_right * ticks_to_m

        # Linear & angular displacement
        d_center = (left_dist + right_dist) / 2.0
        d_theta = (right_dist - left_dist) / self.wheel_separation

        # Update pose
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Create quaternion from yaw
        q = quaternion_from_euler(0, 0, self.theta)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        )

        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        self.odom_pub.publish(odom)

        self.get_logger().debug(
            f"📬 Encoders: L={left_ticks}, R={right_ticks}, Pose=({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down Pico bridge...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
