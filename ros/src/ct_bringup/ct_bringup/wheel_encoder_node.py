#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

SERIAL_PORT = '/dev/asd'
BAUD_RATE = 115200  # standard number


class WheelEncoderNode(Node):
    # https://docs.ros.org/en/rolling/p/rclpy/api/node.html#rclpy.node.Node
    def __init__(self):
        super().__init__('wheel_encoder_node')
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_encoders',
                                         10)  # The 10 is the QoS history depth: (https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
        # 10 is the default value.
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz aka 0.05secs
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
        # self.get_logger().info(f'Reading wheel encoders from {SERIAL_PORT}')

    def timer_callback(self):
        positions = []
        velocities = []

        try:
            # Expect a single JSON-like array line: [0.00,0.00,0.00,0.00]
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            # Parse list of floats from the line
            try:
                values = json.loads(line)
            except json.JSONDecodeError:
                # Try to recover from trailing commas/spaces
                cleaned = line.replace(' ', '')
                if cleaned.startswith('[') and cleaned.endswith(']'):
                    inner = cleaned[1:-1]
                    values = [float(x) for x in inner.split(',') if x]
                else:
                    raise

            if not isinstance(values, list) or len(values) != 4:
                raise ValueError("Expected 4 float values like [pos0,pos1,pos2,pos3]")

            positions = [float(v) for v in values]
            # The first index is always vel0 etc. Build [pos0..pos3, vel0..vel3]
            velocities = positions[:]  # replicate positions into velocities per requirement

            # Publish as Float32MultiArray: [pos0,pos1,pos2,pos3,vel0,vel1,vel2,vel3]
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
