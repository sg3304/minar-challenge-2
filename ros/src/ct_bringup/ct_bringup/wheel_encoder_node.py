#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600  # standard

class WheelVelocityNode(Node):
    def __init__(self):
        super().__init__('wheel_velocity_node')
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_velocities', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
        self.get_logger().info(f"REceived: this is FROM the arduino")


    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                self.get_logger().info(f"Wheel velocity line: {line}")
                # Remove brackets and split
                line = line.strip('[]')
                velocities = [float(x) for x in line.split(',')]

                if len(velocities) == 4:  # sanity check
                    msg = Float32MultiArray()
                    msg.data = velocities
                    self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to read/parse serial data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityNode()
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
