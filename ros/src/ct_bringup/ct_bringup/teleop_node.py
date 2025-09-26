#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        
        # Subscribe to velocity commands
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Serial connection to Arduino
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Opened serial port {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Robot parameters (adjust to your robot)
        self.length = 0.3       # front-back wheel separation [m]
        self.width = 0.25       # left-right wheel separation [m]
        self.wheel_radius = 0.05  # wheel radius [m]

    def cmd_callback(self, msg: Twist):
        """
        Convert cmd_vel (Twist) to mecanum wheel velocities
        and send them to Arduino as [w1,w2,w3,w4]\n
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        R = self.wheel_radius
        L = self.length / 2.0
        W = self.width / 2.0
        LW = L + W

        # Mecanum inverse kinematics
        w1 = (vx - vy - LW * omega) / R  # front-left
        w2 = (vx + vy + LW * omega) / R  # front-right
        w3 = (vx + vy - LW * omega) / R  # rear-left
        w4 = (vx - vy + LW * omega) / R  # rear-right

        # Format and send to Arduino
        command = f"[{w1:.4f},{w2:.4f},{w3:.4f},{w4:.4f}]\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent wheel velocities: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
