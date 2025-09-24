#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)


    def cmd_callback(self, msg):
        self.get_logger().info(f"Sent: this is TOWARDS the arduino")

        # Convert Twist (linear.x, angular.z) into left/right wheel velocities
        wheel_base = 0.3  # distance between wheels (m)
        wheel_radius = 0.05  # wheel radius (m)

        v = msg.linear.x
        omega = msg.angular.z

        v_left  = (v - omega * wheel_base / 2.0) / wheel_radius
        v_right = (v + omega * wheel_base / 2.0) / wheel_radius

        v0 = v_left
        v1 = v_right
        v2 = v_left
        v3 = v_right
        command = f"[{v0:.6f},{v1:.6f},{v2:.6f},{v3:.6f}]\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
