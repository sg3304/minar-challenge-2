#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200 #standard number

class WheelEncoderNode(Node):
    # https://docs.ros.org/en/rolling/p/rclpy/api/node.html#rclpy.node.Node
    def __init__(self):
        super().__init__('wheel_encoder_node')
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_encoders', 10) # The 10 is the QoS history depth: (https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
                                                                                  # 10 is the default value.
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz aka 0.05secs
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
        #self.get_logger().info(f'Reading wheel encoders from {SERIAL_PORT}')

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                self.get_logger().info(f"Received: {line}")

                # Remove brackets if present and split by comma
                line = line.strip("[]")
                values = [float(x) for x in line.split(',')]

                if len(values) != 4:
                    self.get_logger().error(f"Unexpected number of values: {len(values)}")
                    return

                positions = values
                velocities = [0.0] * 4  # placeholder since your TeleopNode doesn't send velocities

                msg = Float32MultiArray()
                msg.data = positions + velocities
                self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to read/parse serial data: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
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