#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class PicoSerialBridge(Node):
    def __init__(self):
        super().__init__('pico_serial_bridge')

        # 🔧 Update this to your Pico serial port (check with: ls /dev/ttyACM*)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # Parameters for robot geometry
        self.wheel_base = 0.18  # meters
        self.max_speed = 255    # match MAX_PWM in Pico code

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Pico Serial Bridge started. Listening to /cmd_vel")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Compute differential wheel speeds (ticks or PWM-like values)
        left_speed = linear - (angular * self.wheel_base / 2.0)
        right_speed = linear + (angular * self.wheel_base / 2.0)

        # Map [-1.0, 1.0] m/s to [-MAX_PWM, MAX_PWM]
        left_pwm = int(max(-self.max_speed, min(self.max_speed, left_speed * self.max_speed)))
        right_pwm = int(max(-self.max_speed, min(self.max_speed, right_speed * self.max_speed)))

        # Send to Pico
        command = f"m {left_pwm} {right_pwm}\r"
        self.serial_port.write(command.encode())

        self.get_logger().info(f"Sent: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = PicoSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
