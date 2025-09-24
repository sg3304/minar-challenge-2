#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_velocities', 10)
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  # short timeout for non-blocking read
        self.timer = self.create_timer(0.05, self.read_arduino)  # 20 Hz

    def cmd_callback(self, msg):
        # Convert Twist to wheel velocities
        wheel_base = 0.3
        wheel_radius = 0.05
        v = msg.linear.x
        omega = msg.angular.z

        v_left  = (v - omega * wheel_base / 2.0) / wheel_radius
        v_right = (v + omega * wheel_base / 2.0) / wheel_radius

        command = f"[{v_left:.2f},{v_right:.2f},{v_left:.2f},{v_right:.2f}]\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

    def read_arduino(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # parse line into floats
                    line = line.strip('[]')
                    velocities = [float(x) for x in line.split(',')]
                    if len(velocities) == 4:
                        msg = Float32MultiArray()
                        msg.data = velocities
                        self.pub.publish(msg)
                        self.get_logger().info(f"Received from Arduino: {velocities}")
        except Exception as e:
            self.get_logger().error(f"Error reading from Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
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
