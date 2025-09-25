#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import serial
import math
import tf_transformations
from tf2_ros import TransformBroadcaster

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        
        # ROS interfaces
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, 'wheel_velocities', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Serialx
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.timer = self.create_timer(0.05, self.read_arduino)  # 20 Hz
        self.last_time = self.get_clock().now()

        # Robot params
        self.wheel_base = 0.3
        self.wheel_radius = 0.05

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        v_left  = (v - omega * self.wheel_base / 2.0) / self.wheel_radius
        v_right = (v + omega * self.wheel_base / 2.0) / self.wheel_radius

        command = f"[{v_left:.2f},{v_right:.2f},{v_left:.2f},{v_right:.2f}]\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

    def read_arduino(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    line = line.strip('[]')
                    velocities = [float(x) for x in line.split(',')]
                    if len(velocities) == 4:
                        msg = Float32MultiArray()
                        msg.data = velocities
                        self.pub_wheels.publish(msg)
                        
                        # Use left/right average
                        v_left = velocities[0] * self.wheel_radius
                        v_right = velocities[1] * self.wheel_radius
                        v = (v_left + v_right) / 2.0
                        omega = (v_right - v_left) / self.wheel_base
                        
                        self.update_odometry(v, omega)
        except Exception as e:
            self.get_logger().error(f"Error reading from Arduino: {e}")

    def update_odometry(self, v, omega):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Integrate pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Create quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.pub_odom.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

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
